#!/usr/bin/env python3
"""The student sheets, rendered from `config/tasks_localization.json` rather than retyped.

Two things are generated here: `docs/handout/exercises.tex` (and its PDF, built by hand) and
`docs/handout/exercises.md`, one practicum sheet per task, each with the thresholds as a table and — the
reason this file exists at all — **an empty table under every protocol item**, with column headings and
four blank rows. `docs/exercises.md` is prose, and a protocol filled into prose is a protocol whose
numbers cannot be found: `todo.md` §6 records that the sheets want actual cells, because a group writes
"about 3 cm, pretty good" under a paragraph and writes **0.031 / 0.187 / 6.0 / 0.42** into a row with a
heading that says `RMSE [mm] | raw odom [mm] | improvement [-] | NEES [-]`.

The `Target` column is generated from the JSON's thresholds, never typed. That is what makes `--check`
worth running in CI: a threshold that was edited in the task file but never reached the sheet shows up as
a drift failure rather than as a cohort chasing a target that no longer exists — the failure this
repository has already been taught twice from the other direction, where documentation quoted a number
the code no longer produced (`docs/verification.md` §8 items 10 and 11).

Deterministic by construction: no timestamps, no dictionary-order dependence, output written with `LF`.
Two renders of one JSON are byte-identical, which is what lets `--check` diff instead of compare.

    python3 tools/make_handout.py                  # write .tex and .md into docs/handout/
    python3 tools/make_handout.py --check          # regenerate into a tempdir, diff, exit 1 on drift
    latexmk -pdf -interaction=nonstopmode -halt-on-error docs/handout/exercises.tex

It reads one file and imports nothing from `ohm_localization`: the sheet is paper for a room where the
code may not even run, and a generator that imports the package it documents inherits its import errors.
"""
import argparse
import json
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_TASKS = os.path.join(ROOT, "config", "tasks_localization.json")
OUT_DIR = os.path.join(ROOT, "docs", "handout")

COURSE = "Intelligente Robotik — Praktikum"
SUBTITLE = "Localising a mecanum robot: Monte-Carlo localisation and ICP scan matching"

# What must be present in every task, because every one of these is rendered somewhere below. A missing
# field is a hard error rather than an empty cell: a sheet with a silent hole in the mark scheme is worse
# than a generator that refuses to run.
REQUIRED = ("id", "title", "points", "world", "text", "checks", "hints", "rmse_max", "max_error_max",
            "improvement_min", "rate_min", "contacts_max", "nees")

# The characters the task file actually contains (measured: σ — × → · ° Σ ² − ≈), each with the LaTeX
# that renders it, plus the few that a threshold edit is likely to introduce next. Anything outside this
# table raises: mojibake on a printed sheet is found by a student, not by the author.
TEX_CHARS = {
    "—": "---", "–": "--", "’": "'", "‘": "'", "…": "\\ldots{}",
    "σ": "\\(\\sigma\\)", "Σ": "\\(\\Sigma\\)", "×": "\\(\\times\\)", "→": "\\(\\rightarrow\\)",
    "·": "\\(\\cdot\\)", "≈": "\\(\\approx\\)", "≥": "\\(\\ge\\)", "≤": "\\(\\le\\)",
    "²": "\\textsuperscript{2}", "−": "\\textminus{}", "°": "\\textdegree{}",
}
# Every text command here ends in braces, not in a letter: `\\textdegree and` makes TeX swallow the space
# as part of the control word, which is how the first sheet printed `+170°and − 170°points` — the minus
# gained a space it should not have and the degree lost the one it needed.

TEX_ESCAPES = {"\\": "\\textbackslash{}", "&": "\\&", "%": "\\%", "$": "\\$", "#": "\\#",
               "_": "\\_", "{": "\\{", "}": "\\}", "~": "\\textasciitilde{}", "^": "\\textasciicircum{}"}

# Column headings per protocol item, chosen from what the item asks for. The fallback is the fixed
# protocol template of `intelligent-robotics/praktikum/LAB-CONCEPT.md` §6 — question, method, measured vs.
# target, explanation of the deviation — which is the shape the corrections are read in, so an item with
# no better shape still gets filled in the same way everything else in the course is filled.
SWEEP = ["parameter", "value tried", "RMSE [mm]", "improvement [-]", "NEES [-]", "comment"]
RUN = ["run / settings", "RMSE [mm]", "raw odom [mm]", "improvement [-]", "NEES [-]", "verdict"]
SIGMA = ["configuration", "reported σ [mm]", "actual error [mm]", "NEES [-]", "should be [-]"]
CONF = ["quantity", "value", "how it was measured", "what could make it wrong"]
GENERIC = ["question / item", "method", "measured", "target", "explanation of the deviation"]
ROWS = 4                                            # blank rows per protocol table

# The three commands that produce everything the sheets ask for, copied verbatim from
# `docs/exercises.md` (§"The development loop" and §"How it runs"). They are constants rather than read
# out of the markdown, because this generator's input is the task file and nothing else — but they are
# *asserted* against that file by `test/test_handout.py`, so a command that changes in the documentation
# while the sheet keeps printing the old one fails a test instead of costing a group an hour.
SCAFFOLD = [
    "./tools/run_lab.sh grade --task mcl_production --controller student/mcl_template.py --headless",
    "OHM_RECORD=/tmp/l1.jsonl ./tools/run_lab.sh grade --task mcl_production \\",
    "        --controller tools/record_scans.py --headless            # one drive, everything it saw",
    "./tools/mcl_report.py /tmp/l1.jsonl --particles 250 --stride 1 --sigma-z 0.5 --timing",
]
SCAFFOLD_WHY = ("The graded run is one number and takes about 36 s. One recording of the same drive "
                "turns every question after it into a three-second replay, so the sweep tables below are "
                "a loop over `mcl_report.py`, not four lab sessions.")


def load(path):
    """The task file, in the order the grader runs it, with every rendered field checked on the way."""
    with open(path, encoding="utf-8") as fh:
        cfg = json.load(fh)
    if not isinstance(cfg.get("tasks"), list) or not cfg["tasks"]:
        raise SystemExit(f"{path}: no 'tasks' list to render")
    by_id = {}
    for task in cfg["tasks"]:
        for key in REQUIRED:
            if key not in task:
                raise SystemExit(f"{path}: task {task.get('id', '?')!r} has no {key!r} — "
                                 f"the sheet renders it, so it cannot be guessed")
        if not isinstance(task["nees"], list) or len(task["nees"]) != 2:
            raise SystemExit(f"task {task['id']!r}: 'nees' must be [lo, hi], got {task['nees']!r}")
        if not task["checks"]:
            raise SystemExit(f"task {task['id']!r}: no protocol items — the sheet would be a title page")
        by_id[task["id"]] = task
    order = cfg.get("order") or list(by_id)
    missing = [t for t in by_id if t not in order]
    if missing:
        raise SystemExit(f"{path}: tasks not in 'order': {missing}")
    return [by_id[t] for t in order], {"points": sum(t["points"] for t in by_id.values()),
                                       "pass_from": cfg.get("pass_from", 50)}


def _tex_escape(text, tt=False):
    """Escape one run of plain text, with the unicode check on the way through.

    An unmapped non-ASCII character is an error rather than a substitution: on a printed sheet, `σ` turning
    into `??` or into nothing is found by a student in front of a mark scheme, which is the worst place in
    this course to discover a rendering bug.
    """
    out = []
    for ch in text:
        if ord(ch) > 126 and ch not in TEX_CHARS:
            raise SystemExit(f"make_handout: no LaTeX mapping for {ch!r} (U+{ord(ch):04X}) — add it to "
                             f"TEX_CHARS or fix the task file; a printed sheet must not guess")
        if tt and ch == " ":
            ch = "\\ "                               # \texttt gobbles spaces; \texttt{a\\ b} does not
        out.append(TEX_ESCAPES.get(ch, TEX_CHARS.get(ch, ch)))
    return "".join(out)


def to_tex(text):
    """The task file's markdown-flavoured prose (`**bold**`, `` `code` ``) as LaTeX.

    Tokenised, not piped through a chain of `re.sub`: the obvious version escapes *after* inserting
    `\\texttt{}`, which is how the first version of this function printed
    `\\textbackslash{}texttt\\{/<robot>/kf/pose\\}` on the sheet instead of the topic name — a mangled
    ``kf/pose`` in the middle of the sentence that tells the student what to publish. Each span is
    escaped once, by the rules of the environment it ends up in.
    """
    out, pos = [], 0
    for match in re.finditer(r"\*\*.+?\*\*|`[^`]+`", str(text)):
        out.append(_tex_escape(str(text)[pos:match.start()]))
        span = match.group(0)
        if span.startswith("**"):
            out.append(r"\textbf{" + _tex_escape(span[2:-2]) + "}")
        else:
            out.append(r"\texttt{" + _tex_escape(span[1:-1], tt=True) + "}")
        pos = match.end()
    out.append(_tex_escape(str(text)[pos:]))
    return "".join(out)


def to_md(text):
    """Task prose for the markdown sheet: markdown already, except that pipes would break a table."""
    return str(text).replace("|", "\\|").replace("\n", " ")


def columns_for(item):
    """The headings of the table a protocol item is answered in.

    Keyed on words the item itself uses, deliberately: `SWEEP` for anything asking for a sweep, `SIGMA`
    for the honesty-of-σ items where the two numbers compared are the reported σ and the real error, `RUN`
    for the run itself. A rule list rather than a per-task table of headings, because the headings should
    follow from the *question* — and a new item that mentions none of them still gets the course template.
    """
    low = item.lower()
    if "sweep" in low or "at least four" in low:
        return SWEEP
    if "nees" in low and ("σ" in item or "sigma" in low or "describe" in low):
        return SIGMA
    if "rmse" in low and ("improvement" in low or "odometry" in low):
        return RUN
    if "costs" in low or "what broke first" in low:
        return CONF
    return GENERIC


def thresholds(task):
    """(quantity, target) pairs, all of them read out of the task file — nothing typed in here.

    Units are converted for the sheet (m → mm) because that is how the numbers are read out loud in the
    room, and the conversion is done here rather than in the sheet's prose so a threshold change cannot
    arrive with the wrong unit still attached. `test/test_handout.py` parses these strings back out of
    the generated file and compares them against the JSON, which is the only reason the word "generated"
    here is a claim rather than an intention.
    """
    return [("accuracy (RMSE over the graded window)", f"\\(\\le\\) {task['rmse_max'] * 1000:.0f} mm"),
            ("improvement over raw odometry", f"\\(\\ge\\) {task['improvement_min']:g}\\(\\times\\)"),
            ("worst single error", f"\\(\\le\\) {task['max_error_max'] * 1000:.0f} mm"),
            ("rate of `kf/pose`".replace("`", ""), f"\\(\\ge\\) {task['rate_min']:g} Hz"),
            ("wall contacts", f"\\(\\le\\) {task['contacts_max']:d}"),
            ("NEES (σ honest)", f"[{task['nees'][0]:g}--{task['nees'][1]:g}]")]


def thresholds_md(task):
    """The same targets for the markdown sheet: same numbers, this file's typographic conventions."""
    return [(q, t.replace("\\(\\le\\)", "≤").replace("\\(\\ge\\)", "≥")
             .replace("\\(\\times\\)", "×").replace("--", "–"))
            for q, t in thresholds(task)]


def _column_widths(columns):
    """Column widths that fit the text block, with the longest word in a heading driving its share.

    Equal columns look tidy right up until a heading word is 1.7 pt wider than its share — `improvement`
    in a six-column table did exactly that, and because \raggedright{} switches hyphenation off, the
    heading would not break and the table hung into the margin instead. A share damped-proportional to the longest
    word (`len ** 0.8`, so one long heading word cannot swallow the row) is deterministic, needs no per-table magic numbers, and is already right for the next heading
    somebody adds.
    """
    words = [max([len(w) for w in re.split(r"[\s/,()+]+", str(c)) if w] or [4]) ** 0.8 for c in columns]
    usable = 16.4 - 0.48 * len(columns)          # 164 mm of text width, less padding and rules per column
    width = [max(1.2, usable * w / sum(words)) for w in words]
    overflow = sum(width) - usable
    return [w - overflow / len(width) for w in width] if overflow > 0 else width


def tabular_tex(columns, rows=ROWS, body=None, cells=None):
    """A ruled grid, not a booktabs table: this one is written into with a pen.

    booktabs rules are for reading, and the honest reason there are vertical lines here is that the cells
    are the content — a group has to know which box a 0.031 belongs in. Column widths come out of the
    usable text width (166 mm on this geometry) minus the per-column padding and rules, so a six-column
    table fits the page and a two-column one does not look like a mistake; `cells` overrides them when one
    column has to carry prose.

    `body` rows are rendered instead of the blanks, which is how the threshold table reuses this layout:
    the mark scheme and the answer form are then the same shape, and the target column of a protocol table
    and the target column of the mark scheme cannot drift apart into two different-looking things.
    """
    n = len(columns)
    width = cells or _column_widths(columns)
    spec = "|" + "".join(">{" + "\\raggedright\\hyphenpenalty 500\\arraybackslash" + "}p{"
                          + f"{w:.2f}" + "cm}|" for w in width)
    blank = " & ".join([r"\rule{0pt}{2.4em}"] * n) + " \\\\"
    head = " & ".join("\\textbf{" + to_tex(c) + "}" for c in columns) + " \\\\"
    lines = ["\\begin{tabular}{" + spec + "}", "\\hline", head, "\\hline"]
    lines += list(body) if body else [blank] * rows
    lines += ["\\hline", "\\end{tabular}"]
    return "\n".join(lines)


def table_md(columns, rows=ROWS):
    head = "| " + " | ".join(to_tex_free(c) for c in columns) + " |"
    sep = "|" + "|".join(["---"] * len(columns)) + "|"
    blank = "| " + " | ".join([" "] * len(columns)) + " |"
    return "\n".join([head, sep] + [blank] * rows)


def to_tex_free(text):
    """Strip the inline-LaTeX we generate for the .tex sheet so the .md sheet stays readable."""
    return (str(text).replace("\\(", "$").replace("\\)", "$")
            .replace("\\textbf{", "**").replace("\\texttt{", "`").replace("}", ""))


PREAMBLE = r"""%% Generated by tools/make_handout.py -- edit config/tasks_localization.json and regenerate.
\documentclass[a4paper,10pt]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{lmodern}
\usepackage[textwidth=166mm,top=20mm,bottom=18mm]{geometry}
\usepackage{array}
\usepackage{booktabs}
\usepackage{longtable}
\usepackage{textcomp}
\setlength{\parindent}{0pt}
\setlength{\parskip}{0.6ex}
\newcommand{\pts}[1]{\textsc{#1 pts}}
\pagestyle{plain}
\begin{document}
"""

FOOTER = r"\end{document}" + "\n"


def header_tex(tasks, meta):
    """The block every sheet is headed with: what this is, and who ran it with which seed.

    The right-hand box is per person and per run because the course says so: protocols are one per person
    even when the code is shared, and the per-group seeds exist so measured numbers cannot be handed
    between groups (`intelligent-robotics/praktikum/LAB-CONCEPT.md` §6). A name-less, seed-less sheet is a
    sheet whose numbers cannot be attributed to a run, which is the same fault this repository keeps
    finding in its own documentation.
    """
    rows = [r"\begin{tabular}{|l|p{3.6cm}|}", r"\hline"]
    for label in ("Name", "Date", "Group", "Seed"):
        rows.append(label + " & " + r"\rule{0pt}{1.4em}" + r" \\")
        rows.append(r"\hline")
    rows.append(r"\end{tabular}")
    left = "\n".join([
        r"\textbf{Sheets " + tasks[0]["title"].split(" ")[0] + "---"
        + tasks[-1]["title"].split(" ")[0] + "}          (" + str(len(tasks)) + " tasks)",
        "Total " + str(meta["points"]) + " pts, pass from " + str(meta["pass_from"]) + "\\%. "
        r"One sheet per task; hand in one protocol per person (max.\ 2 pages) --- the code is shared, the "
        r"explanation is not. Every number below has to be reproducible from a command you wrote down.",
    ])
    # The label line and the rule about protocols need a paragraph break between them: inside a minipage a
    # single newline is a space, and they came out as one run-on sentence on the compiled sheet.
    left = left.replace("\n", "\n\n", 1)
    return "\n".join([
        r"\begin{center}\large\bfseries " + to_tex(COURSE) + r"\end{center}",
        r"\vspace{-6pt}\begin{center}" + to_tex(SUBTITLE) + r"\end{center}",
        r"\vspace{2pt}",
        r"\noindent\begin{minipage}[t]{0.60\textwidth}\raggedright" + "\n" + left + "\n"
        + r"\end{minipage}\hfill\begin{minipage}[t]{0.38\textwidth}" + "\n" + "\n".join(rows) + "\n"
        + r"\end{minipage}",
        r"\vspace{4pt}",
    ])


def scaffold_tex():
    """The commands, in a verbatim block: nothing in them needs escaping and nothing may be reflowed."""
    return "\n".join([r"\vspace{2pt}\noindent\textbf{How the numbers are produced}",
                      to_tex(SCAFFOLD_WHY), r"\vspace{2pt}{\small" + "\n" + r"\begin{verbatim}"]
                     + SCAFFOLD + [r"\end{verbatim}" + "\n" + r"}", r"\vspace{6pt}"])


def scaffold_md():
    """The same block for the markdown sheet: the prose is already markdown, the commands need a fence."""
    return ("**How the numbers are produced** — " + SCAFFOLD_WHY
            + "\n\n```bash\n" + "\n".join(SCAFFOLD) + "\n```\n")



def sheet_tex(task, number):
    out = ["\\section{" + to_tex(task["title"]) + "}"]
    # The task id and the hall name go through the escaper too: `\texttt{mcl_production}` is a
    # "Missing $ inserted" at compile time, because \texttt sets the font but does not make `_` safe.
    out.append("\\noindent\\hfill\\pts{" + str(task["points"]) + "}"
               "\\hspace{1em} hall: \\texttt{" + _tex_escape(task["world"], tt=True) + "}"
               "\\hspace{1em}\\texttt{task: " + _tex_escape(task["id"], tt=True) + "}")
    out.append(r"\vspace{2pt}\noindent\textbf{Task}")
    out.append(to_tex(task["text"]))
    out.append(r"\vspace{2pt}\noindent\textbf{What is graded}")
    out.append(tabular_tex(["quantity", "target (from the task file)"],
                           body=[to_tex(q) + " & " + t + r" \\" for q, t in thresholds(task)],
                           cells=[9.2, 6.0]))
    out.append(r"\vspace{2pt}\noindent\textbf{Protocol} --- one table per item. Four blank rows each; add"
               r" rows if you need them, and write the command that produced a number under the table.")
    for index, item in enumerate(task["checks"], 1):
        out.append("\\vspace{4pt}\\noindent\\textbf{C%d.%d} \\emph{%s}" % (index, number, to_tex(item)))
        out.append(tabular_tex(columns_for(item)))
    out.append(r"\vspace{4pt}\noindent\textbf{Pointers}")
    out.append(r"\begin{itemize}\itemsep1pt")
    for hint in task["hints"]:
        out.append(r"\item " + to_tex(hint))
    out.append(r"\end{itemize}")
    out.append(r"\vspace{4pt}\noindent\textbf{Checkpoint}")
    out.append(tabular_tex(["checkpoint", "demo shown", "assistant initials", "date"], rows=1))
    out.append(r"\vspace{2pt}\noindent AI tools used (language, debugging, API questions only, never the"
               r" reference solution)\hrulefill")
    out.append(r"\newpage")
    # Blank lines, not newlines: LaTeX reads one newline as a space, and a protocol table that is
    # still inside the paragraph that asked the question gets set inline after it and runs 13 cm off the
    # page — which is exactly what the first compiled sheet did.
    return "\n\n".join(out)


def sheet_md(task, number, meta):
    out = [f"## {task['title']}", "",
           f"*{task['points']} pts · hall `{task['world']}` · task id `{task['id']}`*", "",
           "**Task**", "", to_md(task["text"]), "", "**What is graded**", "",
           "| quantity | target (from the task file) |", "|---|---|"]
    for q, t in thresholds_md(task):
        out.append(f"| {to_md(q)} | {t} |")
    out += ["", "**Protocol** — one table per item, four blank rows each. Write the command that produced "
            "a number under the table.", ""]
    for index, item in enumerate(task["checks"], 1):
        out += [f"**C{index}.{number}** — *{to_md(item)}*", "", table_md(columns_for(item)), ""]
    out += ["**Pointers**", ""]
    out += [f"- {to_md(h)}" for h in task["hints"]]
    out += ["", "**Checkpoint**", "", table_md(["checkpoint", "demo shown", "assistant initials", "date"],
                                               rows=1), "",
            "AI tools used (language, debugging, API questions only — not the reference solution): ______",
            ""]
    return "\n".join(out)


def render(tasks, meta):
    """Both documents, from the same data and in the same order, so the two cannot disagree."""
    tex = [PREAMBLE, header_tex(tasks, meta), scaffold_tex()]
    for number, task in enumerate(tasks, 1):
        tex.append(sheet_tex(task, number))
    tex.append(FOOTER)
    md = [f"# {COURSE} — {SUBTITLE}", "",
          f"{len(tasks)} tasks, {meta['points']} points, pass from {meta['pass_from']} %. "
          "Generated from `config/tasks_localization.json` by `tools/make_handout.py`; the thresholds "
          "below are that file's, not retyped.", "", scaffold_md()]
    for number, task in enumerate(tasks, 1):
        md.append(sheet_md(task, number, meta))
    return "\n\n".join(tex), "\n\n".join(md)


def write(path, text):
    old = None
    if os.path.exists(path):
        with open(path, encoding="utf-8") as fh:
            old = fh.read()
    if old == text:
        return False                                  # untouched: --check on a clean tree writes nothing
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--tasks", default=DEFAULT_TASKS, help="the task file to render (default: our own)")
    ap.add_argument("--out", default=OUT_DIR, help="where the two sheets go (default: docs/handout)")
    ap.add_argument("--check", action="store_true",
                    help="regenerate into a tempdir and diff against what is on disk; exit 1 on drift")
    args = ap.parse_args(argv)

    tasks, meta = load(args.tasks)
    tex, md = render(tasks, meta)

    if args.check:
        drift = []
        with tempfile.TemporaryDirectory() as tmp:
            for name, text in (("exercises.tex", tex), ("exercises.md", md)):
                written = write(os.path.join(tmp, name), text)
                if written:                           # fresh tempdir: always written, that is the point
                    pass
                target = os.path.join(args.out, name)
                if not os.path.exists(target):
                    drift.append(f"{name}: not on disk — the sheet was never generated")
                    continue
                with open(target, encoding="utf-8") as fh:
                    on_disk = fh.read()
                if on_disk != text:
                    first = next((i for i, (a, b) in enumerate(zip(on_disk.splitlines(),
                                                                   text.splitlines())) if a != b), 0)
                    drift.append(f"{name}: differs from a fresh render (first differing line "
                                 f"{first + 1} of {len(text.splitlines())})")
        if drift:
            print("make_handout --check: the sheets are not what this task file says\n  "
                  + "\n  ".join(drift)
                  + "\nrun:  python3 tools/make_handout.py", file=sys.stderr)
            return 1
        print(f"make_handout --check: {len(tasks)} sheets match the task file "
              f"({meta['points']} pts, sum of the checks: "
              f"{sum(len(t['checks']) for t in tasks)} protocol tables)")
        return 0

    for name, text in (("exercises.tex", tex), ("exercises.md", md)):
        changed = write(os.path.join(args.out, name), text)
        print(f"  {'wrote' if changed else 'unchanged'} {os.path.relpath(os.path.join(args.out, name), ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
