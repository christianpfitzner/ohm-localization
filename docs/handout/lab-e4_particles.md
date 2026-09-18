# E4 — Generating the particles of a Monte Carlo localiser

*M1 · 180 minutes of a 180-minute session · 50 points · file `student/particles_template.py`*

Command that decides it: `python3 tools/lab_check.py e4_particles`

Your function: `sample_uniform, sample_gaussian, move_particles, resample, weighted_estimate`

A particle is not an estimate, a cloud of particles is. Five functions that between them are the entire lifecycle of a Monte Carlo localiser's hypothesis set: draw the prior, move the cloud one odometry step, weight it (that part is the sensor model, and E5 is about it), resample it, and read one pose and one σ off the cloud. Everything here is measurable without a robot, and the criteria are deliberately built so that a plausible implementation fails them.

Two of the seven criteria are about spread, and they are the two this repository earned the hard way. The reference filter was run twice with the same parameters and the same drive and came out at 0.20 m and 1.26 m of RMSE; the only difference was that one version predicted once per LIDAR scan at 20 Hz and the other once per wheel message at 50 Hz. A per-step σ means "per message", so the second version spread its cloud by the square root of 2.5 per second for no reason but the topic rate, and in a hall where the LIDAR says little nothing pulled it back in. The motion noise is therefore stated in two kinds: the three alphas are proportional to the motion inside the step, and the two floors are per square root of second — measured, one second of motion with the alphas off spreads the cloud 0.419 m at 20 Hz and 0.423 m at 10 Hz when the floor is a rate, and 0.202 m against 0.141 m when it is not.

The same lesson in the other direction is why a filter that does not spread dies: a cloud that cannot widen, or a σ you invent instead of measuring, is a filter that walks into a wrong pose confidently a few seconds later. The band on the spread criterion has both edges for that reason.

## Do this, in this order

1. Draw the prior: `sample_uniform()`. The `GridMap` gives `sample_free(n, rng)`, which does it in three lines — the criterion is about what you get, not how you got it, and the two failure modes (samples in a wall, samples in one corner) are both measured. Then `sample_gaussian()`, including the case every implementation forgets: a prior at θ = 3.0 rad with σ = 0.5 has 39 % of its samples past π, and they have to come back around at −π.
2. `move_particles()`. With every noise field of `p` at zero the answer is a rigid transform of the cloud, and the criterion checks it to the micrometre — that is where the three usual mistakes surface: the odometry *pose* used instead of the odometry *step*, the rotation taken about the world origin instead of the robot, and one turn where the model has two.
3. Then the noise: three draws, `s₁` and `s₃` for the two rotations and `s₂` for the distance, each one an alpha part and a floor part, the floor multiplied by sqrt(dt). Measure the spread after 20 steps and write the number down before you look at the criterion — the number is the protocol entry, and the criterion only says whether it is in the band (the reference implementation lands at 0.75 m, which is 12 % of the distance the cloud travelled).
4. `resample()` with the weights the sensor model would give. The criterion hands you three modes — one carrying 80 % of the weight, one carrying 20 %, one carrying nothing — and asks for 960, 240 and 0 copies of them. A resampler that sorts and keeps the best n gives 1200, 0 and 0, which is a filter that has thrown away the second hypothesis along with the evidence for it. Systematic resampling, one uniform draw and n evenly spaced pointers, gets all three numbers and is eight lines.
5. `weighted_estimate()`. Average the heading as a direction, not as a number — a cloud straddling ±π means 180°, and its arithmetic mean says 0° and drives the robot backwards. The σ comes out of the spread of the cloud, because that number is what the grader scores as NEES and what RViz draws the ellipse from.

## Protocol — what goes in the write-up

* The coverage map of `sample_uniform()`: a histogram of your samples against free area, and the count of distinct 0.5 m cells hit. One sentence on what is wrong with a cloud that fails this.
* The measured σ of `sample_gaussian()` against the σ that was asked for, for three priors including the one at θ = 3.0 rad, with the number of samples that came around the wrap.
* The spread of the cloud against the update rate — the same second of motion chopped into 5, 10, 20 and 40 updates — as one plot with two curves: one with the alphas on, one with only the floors on. Say which of the two curves is flat, and why that is the right shape for one and the wrong shape for the other.
* One table of the resampling: the weights before, the copies after, and N_eff before and after, for the three modes of the criterion and for four equal modes.
* `n` against the criteria: the smallest cloud that passes all seven of them, and the smallest that passes them with every random seed the grader uses.

## Hand in

* All five functions passing all seven criteria: `python3 tools/lab_check.py e4_particles` prints PASS.
* The spread-against-update-rate plot with its two curves, and the resampling table for the three modes and for four equal modes.

## What is measured

| criterion | pts | what it measures | the ask | measured here |
|---|---|---|---|---|
| `uniform-on-the-floor` | 8 | 1200 samples over the `production` map: distance to the nearest wall, and coverage of the free 0.5 m cells | no sample nearer than 150 mm to a wall, at least 55 % of the free cells hit, busiest quarter no more than 6× the quietest | closest 192 mm, 81 % of the cells hit, 2.8× |
| `gaussian-prior-with-the-asked-sigma` | 8 | 4000 draws around (3, 2, 3.0) with σ (0.4, 0.25, 0.5): the mean, the measured σ of the sample, and the fraction past the ±π wrap | mean within 3 standard errors, measured σ within 0.85…1.20 of the σ asked for, 28…52 % wrapped, nothing outside (−π, π] | σ (0.400, 0.247, 0.502), 39.5 % wrapped — which is what a prior at θ = 3.0 rad with σ = 0.5 really contains |
| `noiseless-motion-is-rigid` | 4 | The cloud after one step with every noise field at zero, against the rigid three-part transform | worst particle within 0.1 mm and 0.01° of it | 0.000 mm and 0.000° |
| `the-cloud-spreads` | 8 | Position spread of 1200 particles after 20 steps of 50 ms with the filter's own parameters | inside the band 0.60…0.95 m, heading spread under 0.35 rad, mean within 150 mm of the noiseless motion | 0.75 m (0.744…0.800 over twenty seeds), 0.23 rad, 71…93 mm; double the noise and it is 1.46 m, none of it and 0.000 m |
| `noise-floors-are-rates` | 6 | One second of the same motion as 20 × 50 ms, 10 × 100 ms and 40 × 25 ms, with the alphas off and only the floors on | both spread ratios within 0.85…1.18 | 0.419 m, 0.423 m and 0.416 m — ratios 0.99 and 1.01; the same test on a per-step σ gives 0.202 m and 0.141 m, ratio 1.43 |
| `resample-follows-weights` | 8 | Three modes carrying 80 %, 20 % and nothing: copies of each after resampling, and the mean of the result | copies within ±5 % of 0.8n and 0.2n, none of the zero-weight mode, mean within 10 mm of 8.0 m, still n particles | 960 and 240 exactly, 0 dead survivors, mean 8.000 m |
| `estimate-averages-headings` | 8 | A cloud of poses straddling ±π, and a cloud of known spread: the estimate's heading and its σ | heading at least 2.9 rad out of ±π, position within 10 mm, σ within 20 mm of the cloud's own | 3.14 rad, σ (0.198, 0.101) |

*The `measured here` column is the reference solution (`solution/particles_solution.py`) run through the command above; the ask is the threshold in `config/exercises_localization.json`. Both are reproduced by `python3 tools/lab_check.py --check`. `docs/verification.md` §19 (and §14 for the four MCL drives) has the runs.*

**What this is for.** The sampling half of the filter, and the one parameter that decides whether a localiser survives a degenerate hall: how fast the cloud is allowed to widen.
