"""Tests for the generated student sheets (`tools/make_handout.py`).

What is being protected here is a promise: the thresholds on the paper are the thresholds in
`config/tasks_localization.json`. That promise is invisible otherwise — the sheet *looks* generated, the
numbers *look* plausible, and a cohort can chase an edited target for a semester before anyone notices. So
the tests parse the generated LaTeX and read the numbers back out, rather than fingerprinting the file: a
test that greps for one fixed string passes when the generator breaks and the string happens to survive,
which is the failure mode this repository keeps hitting elsewhere (`docs/verification.md` §8 items 10/11).

One test needs `latexmk` and skips without it, because the PDF is the artefact a teacher prints while the
tests still have to run on a machine without TeX.
"""
import json
import os
import re
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import make_handout as mh                                              # noqa: E402

TASKS = os.path.join(ROOT, "config", "tasks_localization.json")
CHECKED_IN = os.path.join(ROOT, "docs", "handout")
GENERATOR = os.path.join(ROOT, "tools", "make_handout.py")


def generate(tmp_path, tasks=TASKS):
    """Run the generator as a subprocess into `tmp_path`; returns (tex, md, CompletedProcess)."""
    run = subprocess.run([sys.executable, GENERATOR, "--tasks", tasks, "--out", str(tmp_path)],
                         capture_output=True, text=True)
    tex = (tmp_path / "exercises.tex").read_text(encoding="utf-8")
    md = (tmp_path / "exercises.md").read_text(encoding="utf-8")
    return tex, md, run


def sections(tex):
    """The .tex split into one (heading, body) chunk per task, in file order.

    The heading is a *capturing* group on purpose: without it `re.split` hands back only the pieces between
    matches, and pairing them up two-and-two silently compares the second sheet against the third.
    """
    parts = re.split(r"^\\section\{([^}\n]*)\}", tex, flags=re.M)
    return [(head, body) for head, body in zip(parts[1::2], parts[2::2])]


def plain(line):
    """Undo just enough LaTeX to read a threshold row.

    Matching the escaped form with a regex would be a second parser to keep in sync with the generator, and
    a change to the *markup* — not to the numbers — would then fail the test as though the mark scheme had
    moved. The comparison happens on the rendered text instead.
    """
    return (line.replace(r"\(\le\)", "<=").replace(r"\(\ge\)", ">=").replace(r"\(\times\)", "x")
            .replace(r"\(\sigma\)", "sigma").replace(" \\\\", "").strip())


def graded_rows(body):
    """The 'What is graded' table of one sheet, as {quantity: target}."""
    table = body.split(r"\begin{tabular}")[1].split(r"\end{tabular}")[0]
    rows = {}
    for line in table.splitlines():
        if " & " not in line or line.lstrip().startswith(r"\textbf"):
            continue
        quantity, target = line.rsplit(" & ", 1)
        rows[plain(quantity)] = plain(target)
    return rows


def test_one_section_per_task_in_grading_order(tmp_path):
    tex, _, run = generate(tmp_path)
    assert run.returncode == 0, run.stderr
    tasks, _ = mh.load(TASKS)
    found = sections(tex)
    assert len(found) == len(tasks), "one section per task: no title-page section, no extras"
    assert [head for head, _ in found] == [mh.to_tex(t["title"]) for t in tasks], \
        "the sheets follow the task file's own 'order'"


def test_at_least_one_table_per_protocol_item(tmp_path):
    """Every protocol item gets its own form, which is the entire reason the generator exists."""
    tex, _, _ = generate(tmp_path)
    tasks, _ = mh.load(TASKS)
    for (head, body), task in zip(sections(tex), tasks):
        tables = body.count(r"\begin{tabular}")
        assert tables >= len(task["checks"]) + 2, \
            f"{task['id']}: {tables} tables for {len(task['checks'])} items (+ mark scheme + checkpoint)"
    # A `|` is legitimate in a LaTeX column spec, so look for the markdown table *syntax* instead.
    body = tex.split(r"\begin{verbatim}")[0]
    assert "|---" not in body and "\n| " not in body, "the LaTeX sheet must not leak markdown tables"


def test_md_sheet_mirrors_the_tex_sheet(tmp_path):
    """Same numbers, same items, same order, in the version students read on a screen."""
    tex, md, _ = generate(tmp_path)
    tasks, _ = mh.load(TASKS)
    assert md.count("\n## ") == len(tasks)
    for task in tasks:
        assert f"## {task['title']}" in md
        assert f"{task['points']} pts" in md and f"`{task['world']}`" in md
        for item in task["checks"]:
            assert mh.to_md(item) in md, f"{task['id']}: protocol item missing from the markdown sheet"
        assert f"{task['rmse_max'] * 1000:.0f} mm" in md, f"{task['id']}: threshold missing from markdown"
        for hint in task["hints"]:
            assert mh.to_md(hint) in md
    assert md.count("|---") >= sum(len(t["checks"]) for t in tasks) + 2 * len(tasks)


@pytest.mark.parametrize("task_no", range(4))
def test_threshold_numbers_come_from_the_json(tmp_path, task_no):
    """Read the numbers back off the sheet and compare them against the task file.

    This is the test the generator is justified by. Whole-dictionary equality, so a row that disappeared, a
    row that was invented, and a row whose number drifted all fail; a per-key search would pass happily on
    a sheet that had quietly lost its NEES line.
    """
    tex, _, _ = generate(tmp_path)
    task = mh.load(TASKS)[0][task_no]
    body = sections(tex)[task_no][1]
    assert graded_rows(body) == {
        "accuracy (RMSE over the graded window)": f"<= {task['rmse_max'] * 1000:.0f} mm",
        "improvement over raw odometry": f">= {task['improvement_min']:g}x",
        "worst single error": f"<= {task['max_error_max'] * 1000:.0f} mm",
        "rate of kf/pose": f">= {task['rate_min']:g} Hz",
        "wall contacts": f"<= {task['contacts_max']:d}",
        "NEES (sigma honest)": f"[{task['nees'][0]:g}--{task['nees'][1]:g}]",
    }, f"{task['id']}: the sheet's mark scheme is not the task file's"


def test_points_and_protocol_labels_are_the_tasks_own(tmp_path):
    tex, _, _ = generate(tmp_path)
    tasks, _ = mh.load(TASKS)
    for number, ((_, body), task) in enumerate(zip(sections(tex), tasks), 1):
        assert "\\pts{" + str(task["points"]) + "}" in body, f"{task['id']}: points not on its sheet"
        assert task["id"].replace("_", "\\_") in body, "the sheet has to name the task it is for"
        for item in range(1, len(task["checks"]) + 1):
            assert f"\\textbf{{C{item}.{number}}}" in body, f"{task['id']}: C{item}.{number} missing"
        assert body.count("\\pts{") == 1, f"{task['id']}: the points line belongs on its sheet once"


def test_scaffold_block_is_verbatim_and_still_matches_the_documentation(tmp_path):
    """The commands are printed in a verbatim block *and* must still be the documented ones.

    The generator holds them as constants — its only input is the task file — so the drift check runs from
    this side: a command that changes in `docs/exercises.md` while the sheet keeps printing the old version
    fails here, in a test run, rather than in a room full of students running a command that no longer
    exists.
    """
    tex, md, _ = generate(tmp_path)
    docs = open(os.path.join(ROOT, "docs", "exercises.md"), encoding="utf-8").read()
    readme = open(os.path.join(ROOT, "README.md"), encoding="utf-8").read()
    block = tex.split(r"\begin{verbatim}")[1].split(r"\end{verbatim}")[0]
    assert len(mh.SCAFFOLD) == 4, "grade / record (two lines) / replay"
    for line in mh.SCAFFOLD:
        assert line in block, "printed verbatim, unescaped"
        assert line in docs, f"{line[:44]}… is not in docs/exercises.md, so the sheet quotes a command " \
                             "the documentation no longer shows"
    assert "run_lab.sh grade --task mcl_production" in readme, \
        "the README's graded command line has moved; the sheet quotes the exercise document and the two " \
        "should still describe the same workflow (the sheet uses the template, the README the solution)"
    assert "```bash" in md and mh.SCAFFOLD[0] in md, "the markdown sheet fences the same commands"


def test_output_is_deterministic(tmp_path):
    """Two runs are byte-identical, which is what lets --check diff instead of eyeball."""
    tex_a, md_a, _ = generate(tmp_path / "a")
    tex_b, md_b, _ = generate(tmp_path / "b")
    assert tex_a == tex_b and md_a == md_b
    assert "\r" not in tex_a, "LF only, so the diff is the same diff on any checkout"


def test_check_passes_on_the_checked_in_sheets():
    """The sheets in the repository are what the current task file generates."""
    run = subprocess.run([sys.executable, GENERATOR, "--check"], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    assert "4 sheets match" in run.stdout, run.stdout


def test_check_fails_when_a_threshold_moves_without_the_sheet(tmp_path):
    """A threshold edited in the task file has to be detectable, which is --check's whole job."""
    cfg = json.load(open(TASKS, encoding="utf-8"))
    cfg["tasks"][0]["rmse_max"] = 0.04
    mutated = tmp_path / "tasks.json"
    mutated.write_text(json.dumps(cfg), encoding="utf-8")
    run = subprocess.run([sys.executable, GENERATOR, "--tasks", str(mutated), "--check"],
                         capture_output=True, text=True)
    assert run.returncode == 1, "drift has to be a non-zero exit, not a printed hint"
    assert "exercises.tex: differs" in run.stdout + run.stderr


@pytest.mark.parametrize("drop", ["rmse_max", "checks", "nees", "points", "world", "hints"])
def test_a_missing_field_fails_loudly_instead_of_printing_an_empty_cell(tmp_path, drop):
    cfg = json.load(open(TASKS, encoding="utf-8"))
    del cfg["tasks"][1][drop]
    mutated = tmp_path / "tasks.json"
    mutated.write_text(json.dumps(cfg), encoding="utf-8")
    run = subprocess.run([sys.executable, GENERATOR, "--tasks", str(mutated),
                          "--out", str(tmp_path / "out")], capture_output=True, text=True)
    assert run.returncode != 0
    assert drop in run.stderr, f"{drop} should be named in the failure, got: {run.stderr[:200]}"


def test_an_unmappable_character_is_refused_not_guessed(tmp_path):
    """σ on the sheet has to be σ; a generator that silently drops a symbol prints a wrong mark scheme."""
    cfg = json.load(open(TASKS, encoding="utf-8"))
    cfg["tasks"][0]["text"] += "  ⟨bracketed⟩"
    mutated = tmp_path / "tasks.json"
    mutated.write_text(json.dumps(cfg), encoding="utf-8")
    run = subprocess.run([sys.executable, GENERATOR, "--tasks", str(mutated),
                          "--out", str(tmp_path / "out")], capture_output=True, text=True)
    assert run.returncode != 0 and "no LaTeX mapping" in run.stderr


def test_column_widths_never_exceed_the_text_block():
    """Every table shape the generator can emit has to fit an A4 sheet with 166 mm of text."""
    for columns in (mh.RUN, mh.SWEEP, mh.SIGMA, mh.CONF, mh.GENERIC,
                    ["checkpoint", "demo shown", "assistant initials", "date"]):
        width = mh._column_widths(columns)
        total = sum(width) + 0.48 * len(columns)
        assert total <= 16.6, f"{columns}: {total:.2f} cm wide, and the page is 16.6 cm"
        assert all(w >= 1.0 for w in width), f"{columns}: a column nobody can write in: {width}"


@pytest.mark.skipif(shutil.which("latexmk") is None, reason="no TeX on this machine")
def test_the_sheets_compile_to_one_sheet_per_task(tmp_path):
    """The page count is measured, not hoped for, and so is the absence of overflow.

    Built in a temporary directory so a test run does not dirty the repository. One page per task is the
    practicum's shape, and `Overfull` in the log means a table hangs into the margin — which is precisely
    where a student writes their numbers.
    """
    generate(tmp_path)
    run = subprocess.run(["latexmk", "-pdf", "-interaction=nonstopmode", "-halt-on-error",
                          "exercises.tex"], cwd=str(tmp_path), capture_output=True, text=True, timeout=900)
    assert run.returncode == 0, run.stdout[-3000:]
    assert (tmp_path / "exercises.pdf").exists()
    info = subprocess.run(["pdfinfo", "exercises.pdf"], cwd=str(tmp_path),
                          capture_output=True, text=True).stdout
    pages = int(re.search(r"Pages:\s+(\d+)", info).group(1))
    tasks, _ = mh.load(TASKS)
    assert pages >= len(tasks), f"{pages} pages for {len(tasks)} tasks: a task did not get its own page"
    assert pages <= 2 * len(tasks) + 2, f"{pages} pages: the sheets spilled and are no longer printable"
    log = (tmp_path / "exercises.log").read_text(errors="replace")
    assert "Overfull" not in log, "a table hangs into the margin; students write their numbers there"
    print(f"generated PDF: {pages} pages for {len(tasks)} tasks")


@pytest.mark.skipif(shutil.which("latexmk") is None or shutil.which("pdftotext") is None,
                    reason="no TeX on this machine")
def test_checked_in_pdf_is_the_current_render():
    """If a PDF is committed it is the one this task file generates, because paper is the artefact in use.

    The failure this prevents is the ordinary one: the task file is re-tuned, the .tex is regenerated, and
    the PDF in the drawer that gets handed out still carries last week's targets.
    """
    pdf = os.path.join(CHECKED_IN, "exercises.pdf")
    assert os.path.exists(pdf), "docs/handout/exercises.pdf is what gets printed; build it"
    text = subprocess.run(["pdftotext", pdf, "-"], capture_output=True, text=True).stdout
    for task in mh.load(TASKS)[0]:
        assert f"{task['rmse_max'] * 1000:.0f} mm" in text, \
            f"{task['id']}: the PDF has no {task['rmse_max'] * 1000:.0f} mm line, so it predates the task file"
        assert task["title"].split(":")[0] in text, f"{task['id']} is not in the PDF at all"
