"""Iterative Closest Point: register two laser scans, and say how well you did it.

Two variants, because they fail differently and the course teaches both:

*   `icp_point_to_point()` — correspondences between points, closed form from the centroids and the
    covariance matrix `H = src_centred @ dst_centred.T`, one SVD.  The formula the old tutorial
    sheets worked through with pen and paper; simple, and it slides along a long straight wall
    because every point on the wall is an equally good match for every other.
*   `icp_point_to_line()` — the residual is the distance from a source point to the *line* fitted to
    the neighbourhood of its corresponding point, linearised in the rotation.  A wall then constrains
    the direction across itself and leaves the direction along it free, which is the correct
    behaviour and which shows up as an enormous covariance in exactly that direction — see `cond`.

Both are given the correspondence search as `nearest_neighbour()` with two implementations
(`brute` and `kdtree` when SciPy is installed), because the search is the hot loop: it is the other
half of the "measure the vectorisation, do not guess it" lesson, and the lecture animation of the
same name is the picture of what these two functions do.

The uncertainty is not decoration.  A registration reported without a covariance cannot tell a
corridor — where the along-wall direction is unconstrained — from an open hall, and the reader has to
take the number on trust.  `covariance` here is `sigma² (JᵀJ)⁻¹` of the final linear system in the
3-vector (x, y, theta), so `sx`, `sy` and `sth` come out of the same fit that produced the pose.

The one thing that silently ruins a registration is a scan with `inf` in it.  A beam that hit nothing
is not a point, and feeding `inf` — or, worse, the `range_max` that the ROS bridge substitutes for
it — into a centroid drags the whole result toward the sensor.  `points_from_scan()` drops those
beams and counts them, so a run on the wrong no-echo dialect is visible in the log instead of inside
the numbers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

import numpy as np

TWO_PI = 2.0 * math.pi


# ---------------------------------------------------------------------------------- poses and points
def se2(x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> np.ndarray:
    """Homogeneous 3x3 transform [[R, t], [0, 1]] — the form every ROS message uses."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def inverse(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid 2D transform by transposing the rotation — cheaper and better than inv()."""
    R, t = T[:2, :2], T[:2, 2]
    out = np.eye(3)
    out[:2, :2] = R.T
    out[:2, 2] = -R.T @ t
    return out


def transform(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """T applied to an (n, 2) array of points."""
    return pts @ T[:2, :2].T + T[:2, 2]


def pose_to_T(pose) -> np.ndarray:
    x, y, theta = (float(v) for v in _triple(pose))
    return se2(x, y, theta)


def relative(a, b) -> np.ndarray:
    """The transform that carries points from the frame of pose `a` into the frame of pose `b`.

    A point on the floor measured once from each of two poses is the same point, so this is the
    answer an algorithm has to produce — and, given the two poses from the simulator's `/truth`, it
    is also the ground truth of the registration exercise.  Getting the order backwards is the most
    common one-character bug in an ICP exercise and the only symptom is that the error is large.
    """
    return inverse(pose_to_T(b)) @ pose_to_T(a)


def triple_of(T: np.ndarray) -> tuple:
    """(x, y, theta) of a homogeneous transform."""
    return float(T[0, 2]), float(T[1, 2]), float(math.atan2(T[1, 0], T[0, 0]))


def error_between(estimated: np.ndarray, truth: np.ndarray) -> tuple:
    """(|translation| in m, |rotation| in deg) between two transforms — the one number to report."""
    d = inverse(truth) @ estimated
    return float(np.linalg.norm(d[:2, 2])), float(abs(math.degrees(math.atan2(d[1, 0], d[0, 0]))))


def _triple(p):
    if hasattr(p, "x") and hasattr(p, "theta"):
        return p.x, p.y, p.theta
    seq = tuple(p)
    return float(seq[0]), float(seq[1]), float(seq[2] if len(seq) > 2 else 0.0)


# ---------------------------------------------------------------------------------- scan -> points
def points_from_scan(scan, stride: int = 1, drop_uninformative: bool = True,
                     max_range_eps: float = 0.06, min_range: float = 0.05,
                     angle_min: float | None = None) -> tuple:
    """(n, 2) body-frame points of one scan, plus the count of beams that carried no information.

    Same rule as in `mcl.py` and for the same reason: `inf` and a reading at `range_max` mean
    "nothing within range", which is a statement about what is *not* there and cannot be a point.
    """
    get = (lambda k, d=None: scan.get(k, d)) if isinstance(scan, dict) else \
          (lambda k, d=None: getattr(scan, k, d))                             # noqa: E731
    ranges = np.asarray(get("ranges", []), dtype=float)
    if not drop_uninformative:
        # Same reason as in `mcl.py`: this dialect reports a missing echo as `range_max`, and `inf`
        # through a centroid is not a wrong answer, it is no answer at all.
        ranges = np.where(np.isfinite(ranges), ranges, float(get("range_max", 8.0)))
    step = float(get("angle_increment", TWO_PI / max(len(ranges), 1)))
    a0 = float(get("angle_min", 0.0) if angle_min is None else angle_min)
    idx = np.arange(0, len(ranges), max(int(stride), 1))
    ok = ranges[idx] > max(min_range, float(get("range_min", 0.0) or 0.0))
    if drop_uninformative:
        ok &= np.isfinite(ranges[idx]) & (ranges[idx] < float(get("range_max", 8.0)) - max_range_eps)
    idx = idx[ok]
    a = a0 + idx * step
    r = ranges[idx]
    dropped = int(max(len(ranges) - len(idx) * max(int(stride), 1), 0)) + int(
        np.count_nonzero(~np.isfinite(ranges)))
    return np.column_stack([r * np.cos(a), r * np.sin(a)]), dropped


def nearest_neighbour(src: np.ndarray, dst: np.ndarray, method: str = "brute",
                      chunk: int = 4096) -> np.ndarray:
    """Index in `dst` of the nearest point of `dst` to each point of `src`.

    `brute` is the full distance matrix in row blocks, which at 360 by 360 points is 130 000
    distances — nothing, and it has no dependency.  `kdtree` needs SciPy and wins when the point
    clouds grow to tens of thousands, which is what the lecture's animation compares; ask for it and
    the tool says which one ran, because a measured speed-up of a thing that silently did not run is
    worse than no measurement.
    """
    if len(dst) == 0:
        raise ValueError("nearest_neighbour: the target cloud is empty")
    if method == "kdtree":
        try:
            from scipy.spatial import cKDTree                 # only when it happens to be installed
        except Exception as exc:
            raise RuntimeError(f"method='kdtree' asked for, SciPy is not installed ({exc})") from exc
        return cKDTree(dst).query(src, k=1)[1]
    out = np.empty(len(src), dtype=np.int64)
    for i0 in range(0, len(src), chunk):
        block = src[i0:i0 + chunk]
        d = ((block[:, None, 0] - dst[None, :, 0]) ** 2
             + (block[:, None, 1] - dst[None, :, 1]) ** 2)
        out[i0:i0 + len(block)] = np.argmin(d, axis=1)
    return out


def mean_match(src: np.ndarray, dst: np.ndarray, T: np.ndarray,
               max_corr: float | None = None) -> float:
    """Mean correspondence distance of `src` onto `dst` **at a given transform** — what ICP minimises.

    Two uses, and the second is the reason it is in the library rather than in a test.  First, as a
    fixture yardstick: if the truth does not fit a pair of scans well, the pair is a bad pair and any
    number measured on it is noise.  Second, asked *at the aliased pose*: in a corridor with posts
    every 4 m, a slide of exactly one period fits the scan as well as the truth does — often better,
    since it also cancels part of the noise.  Then ICP is not broken, it did its job, and the fitness
    value it reports is a true statement about a wrong pose.  `max_corr` mirrors the gate the solver
    used, so the number means the same thing as `IcpResult.fitness`.
    """
    moved = transform(T, src)
    d = np.linalg.norm(moved - dst[nearest_neighbour(moved, dst)], axis=1)
    if max_corr is not None:
        d = d[d <= max_corr]
    return float(np.mean(d)) if len(d) else float("nan")


def _line_params(src: np.ndarray, dst: np.ndarray, idx: np.ndarray, k: int) -> np.ndarray:
    """Point and unit normal of the line fitted to the `k` nearest neighbours of each correspondence.

    The eigenvector of the neighbourhood covariance with the *smallest* variance is the normal —
    the direction in which the points do not vary is the direction across the wall.  The returned
    normal is flipped to face the source point, so a residual is a signed distance with the same
    sign for every point of a plane, which is what makes the linear system the textbook one.
    """
    k = max(int(k), 3)
    n = len(src)
    out = np.zeros((n, 4))                                # px, py, nx, ny
    for i in range(n):
        d2 = (dst[:, 0] - dst[idx[i], 0]) ** 2 + (dst[:, 1] - dst[idx[i], 1]) ** 2
        near = np.argpartition(d2, min(k, len(d2) - 1))[:k]
        pts = dst[near]
        c = pts.mean(axis=0)
        cov = (pts - c).T @ (pts - c) / max(len(pts) - 1, 1)
        w, v = np.linalg.eigh(cov)
        normal = v[:, 0]                                  # smallest eigenvalue = across the wall
        if np.dot(normal, src[i] - c) < 0.0:
            normal = -normal
        out[i] = (c[0], c[1], normal[0], normal[1])
    return out


@dataclass
class IcpResult:
    """What a run produced: the pose, how it got there and how much to believe it."""

    T: np.ndarray = field(default_factory=lambda: np.eye(3))
    iterations: int = 0
    converged: bool = False
    fitness: float = float("nan")            # mean correspondence distance at the end, m
    correspondences: int = 0
    rejected: int = 0                        # dropped by max_corr: the outliers that were not used
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))
    cond: float = float("nan")               # condition of JᵀJ: the degeneracy meter
    dof: int = 3                             # 3 constrained, 4 for the linearised non-rigid form
    yaw_step: float = 0.0                    # the rotation the linearisation thought it took, rad
    history: list = field(default_factory=list)   # (iteration, mean distance, |dx|, |dtheta|)
    nn_method: str = "brute"

    @property
    def pose(self) -> tuple:
        return triple_of(self.T)

    @property
    def sigmas(self) -> tuple:
        """(sx, sy, sth) as 1 sigma in m, m and rad — the numbers a grader can check."""
        d = np.maximum(np.diag(self.covariance), 0.0)
        return float(math.sqrt(d[0])), float(math.sqrt(d[1])), float(math.sqrt(d[2]))

    def as_dict(self) -> dict:
        out = asdict(self)
        out.update(T=self.T.tolist(), covariance=self.covariance.tolist(), pose=self.pose,
                   sigmas=self.sigmas, x=self.pose[0], y=self.pose[1], theta=self.pose[2])
        out.pop("history", None)
        return out


# --------------------------------------------------------------------------------------------- ICP
def icp(src: np.ndarray, dst: np.ndarray, T0: np.ndarray | None = None, mode: str = "line",
        max_iterations: int = 30, tolerance: float = 1e-4, max_corr: float = 1.5,
        neighbour_lines: int = 8, sigma_z: float = 0.02, method: str = "brute",
        unit_sincos: bool = True) -> IcpResult:
    """Register `src` onto `dst`; returns the transform, the covariance and the iteration history.

    `mode="point"` is point-to-point, `mode="line"` point-to-line.  `max_corr` is the correspondence
    gate: a "match" farther away than that is an outlier and is dropped, which is what keeps a wrong
    initial guess from being confidently registered onto the opposite wall.  `tolerance` stops the
    loop when a step moves less than that in metres *and* in radians.

    `unit_sincos` is the difference between a registration and a story about one — see `_solve_line`.
    Left off, the linearised solve drifts on a pair taken beside a long wall: 0.54 rad of yaw error
    and a correspondence distance of 4 mm, which is the most convincing wrong answer this algorithm
    can produce.  It is kept switchable because producing that answer is part of the exercise.
    """
    T = np.eye(3) if T0 is None else np.asarray(T0, dtype=float).copy()
    res = IcpResult(nn_method=method)
    if len(src) < 3 or len(dst) < 3:
        res.rejected = -1
        return res
    for it in range(1, max_iterations + 1):
        moved = transform(T, src)
        idx = nearest_neighbour(moved, dst, method=method)
        d = np.linalg.norm(moved - dst[idx], axis=1)
        keep = d < max_corr
        if keep.sum() < 3:
            res.rejected += int((~keep).sum())
            res.history.append((it, float("nan"), 0.0, 0.0))
            continue
        p, q = moved[keep], dst[idx[keep]]
        if mode == "line":
            Tx, cov, cond, dof = _solve_line(p, _line_params(p, dst, idx[keep], neighbour_lines),
                                             sigma_z, unit_sincos)
        else:
            Tx, cov, cond, dof = _solve_point(p, q, sigma_z)
        # One increment of the pose, as a transform and not as a vector: the drifting 4-parameter form
        # below does not produce a (x, y, theta) step at all, and a `step` array would have to lie
        # about what its third number means.
        T = Tx @ T
        res.T = T                                      # the pose, not only the fit that produced it
        dx = float(np.linalg.norm(Tx[:2, 2]))
        dth = float(math.atan2(Tx[1, 0], Tx[0, 0]))
        res.rejected += int((~keep).sum())
        res.correspondences = int(keep.sum())
        res.fitness = float(np.mean(d[keep]))
        res.covariance, res.cond, res.dof, res.yaw_step = cov, cond, dof, dth
        res.history.append((it, res.fitness, dx, abs(dth)))
        res.iterations = it
        if dx < tolerance and abs(dth) < tolerance:
            res.converged = True
            break
    return res


def _solve_point(p: np.ndarray, q: np.ndarray, sigma_z: float) -> tuple:
    """Point-to-point: centroids to centred coordinates, one SVD, and the reflection guard.

    `H = p_centred.T @ q_centred`, `R = V diag(1, det(VUᵀ)) Uᵀ`.  The diagonal correction is not
    pedantry: without it the SVD is free to answer with a mirror image, which in 2D has a
    determinant of −1 and is not a rotation, and it will do it on the odd scan pair.
    """
    pc, qc = p.mean(axis=0), q.mean(axis=0)
    H = (p - pc).T @ (q - qc)
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    d = np.sign(np.linalg.det(V @ U.T))
    R = V @ np.diag([1.0, d]) @ U.T
    t = qc - R @ pc
    J = np.zeros((2 * len(p), 3))                      # d(T p)/d(x, y, theta) at the current estimate
    J[:len(p), 0] = 1.0                                # the x row of every point
    J[:len(p), 2] = -p[:, 1]
    J[len(p):, 1] = 1.0                                # and the y row
    J[len(p):, 2] = p[:, 0]
    JtJ = J.T @ J
    Tx = se2(t[0], t[1], math.atan2(R[1, 0], R[0, 0]))
    return Tx, _cov(JtJ, max(float(np.mean(np.hypot(*(p - q).T))) ** 2, sigma_z ** 2)), \
        _cond(JtJ), 3


def _solve_line(p: np.ndarray, lines: np.ndarray, sigma_z: float,
                unit_sincos: bool = True) -> tuple:
    """Point-to-line as one linearised normal equation: 3 parameters, or 4.

    The residual of one pair is `r = n·(R p + t - c)`.  Linearising at the current estimate,
    `R ≈ I + ξ·[·]×`, and `ξ×p = ξ(-p_y, p_x)`, so the derivative of that residual is

        dr/d(t_x, t_y, ξ) = (n_x,  n_y,  n_y p_x - n_x p_y)

    **and the last term is the whole exercise.**  The lecture writes the moment arm as the cross
    product `n × p = n_x p_y - n_y p_x`, which is the *negative* of the derivative above — copying it
    into the third column gives a Jacobian that is right about translation and mirrored about
    rotation.  Measured here (correspondences frozen, pose perturbed numerically): the correct column
    correlates **+1.000** with the derivative, the cross-product spelling **−0.978**.  With the sign
    wrong, point-to-point ICP still converges (it does not use this column) while point-to-line walks
    the correspondence distance up from 0.12 m to 0.34 m and settles 53° away with a confident
    covariance — so the two modes disagree, which is the only reason the bug was found.

    The arm is measured from the **centroid of the cloud**, `q = p − p̄`, not from the map origin.
    That is worth a factor: in a 25 × 17 m hall `|p|` reaches 10 m, which makes the rotation column
    ten times the translation columns, and the fit was measurably worse for it — see
    `docs/verification.md`.  Centred, the step means "rotate about the middle of what you can see,
    then shift", which is also how the correction should be read.

    `JᵀJ` is the same matrix the covariance comes out of, so the degeneracy of the fit and the size of
    the error bar are literally the same object.  With `unit_sincos=False` the fourth column makes the
    step linear at the price of a transform that is not a rigid motion; both forms are measured in
    `docs/verification.md` and neither is claimed to be wrong here, only differently constrained.
    """
    c, nx_, ny_ = lines[:, :2], lines[:, 2], lines[:, 3]
    p0 = p.mean(axis=0)
    q = p - p0
    r = nx_ * (p[:, 0] - c[:, 0]) + ny_ * (p[:, 1] - c[:, 1])
    J = np.column_stack([nx_, ny_, ny_ * q[:, 0] - nx_ * q[:, 1]])
    JtJ = J.T @ J
    if not unit_sincos:                                   # the linearised, non-rigid 4-parameter form
        # dr/ds is the column above; dr/dc = n·q is the one the angle never has.
        step = _lstsq(np.column_stack([J, nx_ * q[:, 0] + ny_ * q[:, 1]]), r)
        A = np.array([[1.0 + step[3], -step[2]], [step[2], 1.0 + step[3]]])
        t = step[:2] + p0 - A @ p0
        return (np.array([[A[0, 0], A[0, 1], t[0]], [A[1, 0], A[1, 1], t[1]], [0.0, 0.0, 1.0]]),
                _cov(JtJ, _s2(r, len(p), sigma_z)), _cond(JtJ), 4)
    try:
        step = -np.linalg.solve(JtJ + np.eye(3) * 1e-9, J.T @ r)
    except np.linalg.LinAlgError:
        step = np.zeros(3)
    ca, sa = math.cos(step[2]), math.sin(step[2])
    R = np.array([[ca, -sa], [sa, ca]])
    t = step[:2] + p0 - R @ p0
    return (np.array([[ca, -sa, t[0]], [sa, ca, t[1]], [0.0, 0.0, 1.0]]),
            _cov(JtJ, _s2(r, len(p), sigma_z)), _cond(JtJ), 3)


def _lstsq(J: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Solve the normal equations, or zero when the wall has left nothing to solve for."""
    try:
        return -np.linalg.solve(J.T @ J + np.eye(J.shape[1]) * 1e-9, J.T @ r)
    except np.linalg.LinAlgError:
        return np.zeros(J.shape[1])


def _s2(residual: np.ndarray, n: int, sigma_z: float) -> float:
    """Variance one residual actually carried, floored at the sensor's own figure.

    Believing a datasheet over the evidence a fit left behind is how a covariance ends up smaller than
    the error it is supposed to describe.
    """
    return max(float(np.dot(residual, residual) / max(n - 3, 1)), float(sigma_z ** 2))


def _cov(JtJ: np.ndarray, s2: float) -> np.ndarray:
    """`s2 (JᵀJ)⁻¹` — and a huge covariance when the fit does not constrain the pose.

    `s2` is the variance one residual actually carried, taken from the residuals themselves and
    floored at `sigma_z²`: believing a sensor figure over the evidence a fit left behind is how a
    covariance ends up smaller than the error it describes.  A corridor constrains the direction
    across itself and nothing along itself, `JᵀJ` is nearly singular there, and the sigma in metres
    says so.  Handing back a pseudo-inverse with no sign of that would turn an unconstrained
    direction into a confident number, and noticing that difference is the exercise.
    """
    try:
        if _cond(JtJ) > 1e10:
            raise np.linalg.LinAlgError("singular")
        return float(s2) * np.linalg.inv(JtJ)
    except np.linalg.LinAlgError:
        return np.diag([99.0, 99.0, 9.0])


def _cond(JtJ: np.ndarray) -> float:
    w = np.linalg.svd(JtJ, compute_uv=False)
    return float(w[0] / w[-1]) if w[-1] > 0 else float("inf")


def _exp(step: np.ndarray) -> np.ndarray:
    """(dx, dy, dtheta) as a homogeneous transform — kept for the tests that build a step by hand."""
    return se2(float(step[0]), float(step[1]), float(step[2]))


def register_scans(scan_a, scan_b, T_guess: np.ndarray | None = None, **kwargs) -> IcpResult:
    """`icp()` on two raw scans, with the point clouds and the dropped beams handled once.

    `T_guess` is where the previous pose said `b` is relative to `a` — odometry in the ordinary case,
    the identity in the degenerate demo, and a deliberately wrong value in the basin-of-attraction
    sweep.  Whatever it is, the result says what it was, because "it converged from a 0.5 m wrong
    start" and "it converged" are different claims.
    """
    a, dropped_a = points_from_scan(scan_a, stride=kwargs.pop("stride", 1))
    b, dropped_b = points_from_scan(scan_b, stride=kwargs.pop("stride", 1))
    res = icp(a, b, T_guess, **kwargs)
    res.rejected += dropped_a + dropped_b
    return res
