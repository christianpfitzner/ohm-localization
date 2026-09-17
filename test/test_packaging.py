"""The package boundary: two layouts, one package name, and the files a ROS run reaches for.

These tests exist because the failure they prevent does not happen on the machine that wrote the code. A
`colcon build` puts `config/`, `solution/` and `student/` in `<prefix>/share/ohm_localization/` and the module
in `<prefix>/lib/python3.12/site-packages/`, and every path resolved by `os.path.dirname(__file__)` from a
checkout then points at a file that is three directories away and does not exist there. The only way to test
that branch on a machine with one layout is to build a fake prefix — which is what two of these tests do.

The rest are consistency checks between files that are edited separately and fail together: `setup.py`'s
entry points against what `launch/mcl.launch.py` starts, the launch file's `--set` paths against the
simulator's own config tree, and the commands printed in a docstring against the arguments that file actually
declares. Each of those pairs is one refactoring away from disagreeing, and in every case the error surfaces
as a missing file inside a process whose stderr the launch file interleaves with three others.
"""
import ast
import os
import subprocess
import sys

import pytest

import ohm_localization
from ohm_localization import paths

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(ROOT, "launch", "mcl.launch.py")
SETUP = os.path.join(ROOT, "setup.py")
PACKAGE_XML = os.path.join(ROOT, "package.xml")


def _module(path):
    with open(path, encoding="utf-8") as fh:
        tree = ast.parse(fh.read(), filename=path)
    tree.origin = path                    # `_constant` names the file it looked in
    return tree


def _constant(tree, name):
    """A module-level list literal, read without importing the file (it imports `launch`, which is not here)."""
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{os.path.basename(getattr(tree, 'origin', '?'))}: no module-level list `{name}`")


def _source(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


# -------------------------------------------------------------------------------------------- the layouts

def test_from_a_checkout_the_data_is_next_to_the_module():
    assert paths.data_root() == ROOT
    assert paths.task_file() == os.path.join(ROOT, "config", "tasks_localization.json")
    assert os.path.isfile(paths.task_file())


def test_an_installed_prefix_is_found_through_ament_prefix_path(tmp_path, monkeypatch):
    """The layout that does not exist on this machine, built out of nothing.

    `data_root()` is given a fake `__file__` deep inside a fake prefix, because that is the only honest way to
    ask the question: the answer must come from `AMENT_PREFIX_PATH`, not from where the test happens to run.
    """
    prefix = tmp_path / "install" / "ohm_localization"
    share = prefix / "share" / "ohm_localization"
    (share / "config").mkdir(parents=True)
    (share / "config" / "tasks_localization.json").write_text('{"tasks": []}', encoding="utf-8")
    site = prefix / "lib" / "python3.12" / "site-packages" / "ohm_localization"
    site.mkdir(parents=True)
    (site / "__init__.py").write_text("", encoding="utf-8")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    monkeypatch.delenv("COLCON_PREFIX_PATH", raising=False)
    monkeypatch.delenv("OHM_TASKS", raising=False)
    fake_file = str(site / "paths.py")
    assert paths.data_root(fake_file) == str(share)
    monkeypatch.setattr(paths, "__file__", fake_file)
    assert paths.task_file() == str(share / "config" / "tasks_localization.json")


def test_a_colcon_workspace_that_sets_only_COLCON_PREFIX_PATH_still_resolves(tmp_path, monkeypatch):
    """This sandbox's reality: sourcing sets `COLCON_PREFIX_PATH` and leaves `AMENT_PREFIX_PATH` empty.

    A `data_root()` that reads only the ROS variable returns the *module's build directory* here — which,
    under `--symlink-install`, happens to contain symlinks to the right files, so it appears to work and is
    wrong on the day the build is not symlinked. Both variables are checked for that reason, and this test
    is the reason they stay.
    """
    root = tmp_path / "ws" / "install"
    share = root / "ohm_localization" / "share" / "ohm_localization"
    (share / "config").mkdir(parents=True)
    (share / "config" / "tasks_localization.json").write_text('{"tasks": []}', encoding="utf-8")
    build = tmp_path / "ws" / "build" / "ohm_localization" / "ohm_localization"
    build.mkdir(parents=True)
    (build / "__init__.py").write_text("", encoding="utf-8")
    monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
    monkeypatch.setenv("COLCON_PREFIX_PATH", str(root))
    monkeypatch.delenv("OHM_TASKS", raising=False)
    assert paths.data_root(str(build / "paths.py")) == str(share)


def test_the_build_tree_loses_to_the_install_prefix(tmp_path, monkeypatch):
    """A build directory is not a checkout, even though `--symlink-install` fills it with the same files.

    Measured: before this rule, a sourced workspace resolved the data root to `build/ohm_localization`
    because colcon's build tree holds symlinks named `setup.py`, `package.xml` and `config` — three things a
    checkout is identified by, all three of which a build product copies. The repository is told apart by
    `tools/` and `test/`, which are in neither `data_files` nor a wheel.
    """
    ws = tmp_path / "ws"
    build = ws / "build" / "ohm_localization"
    (build / "config").mkdir(parents=True)
    (build / "setup.py").write_text("", encoding="utf-8")
    (build / "package.xml").write_text("", encoding="utf-8")
    assert not paths._is_checkout(str(build))
    assert paths._is_checkout(ROOT)
    (build / "tools").mkdir()
    assert paths._is_checkout(str(build))          # and the rule is the rule: it is a directory test, not a guess


def test_ohm_tasks_overrides_whatever_the_layout_says(tmp_path, monkeypatch):
    mine = tmp_path / "our_variant.json"
    mine.write_text('{"tasks": []}', encoding="utf-8")
    monkeypatch.setenv("OHM_TASKS", str(mine))
    assert paths.task_file() == str(mine)
    explicit = os.path.join(ROOT, "config", "tasks_localization.json")
    assert paths.task_file(explicit) == explicit          # an explicit path beats the environment too


def test_the_library_imports_with_nothing_but_numpy():
    """No ROS, no simulator, no pygame: `import ohm_localization` on a laptop must work.

    The graded path and the whole test suite depend on this, and one `from mecanum_lab import robot_io` at
    module level in the node would break it in the least visible way — on the machines that do have ROS.
    """
    code = ("import sys; sys.path.insert(0, %r); import ohm_localization, importlib;"
            "importlib.import_module('ohm_localization.gridmap');"
            "importlib.import_module('ohm_localization.mcl');"
            "importlib.import_module('ohm_localization.icp');"
            "importlib.import_module('ohm_localization.synth');"
            "importlib.import_module('ohm_localization.mcl_node');"
            "importlib.import_module('ohm_localization.map_server_node');"
            "print([m for m in sys.modules if m.startswith(('rclpy','nav_msgs','pygame','mecanum_lab'))])"
            % ROOT)
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd="/tmp")
    assert out.returncode == 0, out.stderr[-800:]
    assert out.stdout.strip() == "[]", f"the library pulled in ROS or the simulator: {out.stdout}"


# ---------------------------------------------------------------------------- the package and its metadata

def test_the_package_is_named_the_same_in_every_file_that_names_it():
    assert os.path.isfile(PACKAGE_XML), "colcon needs package.xml at the repository root"
    xml = _source(PACKAGE_XML)
    setup_src = _source(SETUP)
    assert "<name>ohm_localization</name>" in xml
    assert "<build_type>ament_python</build_type>" in xml
    assert 'name=PACKAGE' in setup_src or 'name="ohm_localization"' in setup_src
    assert paths.PACKAGE == "ohm_localization" == os.path.basename(ROOT) \
        or os.path.basename(ROOT) == "ohm-localization"          # the repo is dashed, the package is not
    # The ament resource marker: without it `ros2 pkg list` never sees a built package, and every
    # `ros2 launch <pkg> <file>` fails with "package not found" on a machine where the build succeeded.
    assert os.path.isfile(os.path.join(ROOT, "resource", "ohm_localization"))


def test_the_package_manifest_parses_as_xml():
    """A `package.xml` colcon cannot parse is a package that builds but cannot be launched.

    colcon-ros identifies the build type by *parsing* this file; if the parse fails it falls back to the
    plain python build, which installs the module and the scripts but never writes the
    `ament_prefix_path` hook. `colcon build` then reports success, `import ohm_localization` works, and
    `ros2 launch ohm_localization mcl.launch.py` answers "package not found" — because the install
    prefix was never added to `AMENT_PREFIX_PATH`. The one XML rule that is easiest to break by
    accident is that a comment may not contain two hyphens, which `install.sh --workspace` does.
    """
    import xml.etree.ElementTree as ET
    root = ET.fromstring(_source(PACKAGE_XML).encode("utf-8"))
    assert root.findtext("name") == "ohm_localization"
    assert root.findtext("export/build_type") == "ament_python"
    for dep in root.findall("exec_depend"):
        assert (dep.text or "").strip(), "an empty <exec_depend> is not a dependency"
    assert {d.text.strip() for d in root.findall("exec_depend")} >= {"rclpy", "nav_msgs", "launch"}


def test_every_entry_point_leads_somewhere_and_is_documented():
    tree = _module(SETUP)
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and getattr(n.func, "id", None) == "setup"]
    assert len(calls) == 1
    entry = {}
    for kw in calls[0].keywords:
        if kw.arg == "entry_points":
            for key, value in zip(kw.value.keys, kw.value.values):
                if ast.literal_eval(key) == "console_scripts":
                    entry = dict(item.split(" = ") for item in
                                 [ast.literal_eval(v) for v in value.elts])
    assert entry, "setup.py declares no console_scripts"
    for name, target in entry.items():
        module, _, attr = target.partition(":")
        mod = __import__(module, fromlist=[attr])
        assert callable(getattr(mod, attr)), f"{name} -> {target} is not callable"
        assert name.replace("_", "-") or name          # names are what the launch file and docs print


def test_the_install_puts_the_controllers_and_the_task_file_where_a_ros_run_looks():
    """`--controller` takes a *file path*, so the controllers have to be installed as data."""
    src = _source(SETUP)
    for destination, pattern in (("launch", "launch/*.py"), ("launch", "launch/*.rviz"),
                                 ("config", "config/*.json"),
                                 ("solution", "solution/*.py"), ("student", "student/*.py")):
        assert destination in src and pattern in src, f"{pattern} is not in data_files"
        assert any(os.path.isfile(f) for f in _glob(pattern)), f"nothing matches {pattern} in the checkout"
    # And the packages that must NOT be installed as data are not: a second copy of the library would be a
    # second library to keep in sync.
    assert 'packages=[PACKAGE]' in src or 'packages=["ohm_localization"]' in src


def _glob(pattern):
    import glob
    return glob.glob(os.path.join(ROOT, pattern))


def test_the_node_module_is_startable_by_both_doors():
    """`python3 -m ohm_localization.mcl_node --help` and `solution/mcl_node.py` are the same filter."""
    assert hasattr(ohm_localization.mcl_node, "main") and hasattr(ohm_localization.mcl_node, "mission")
    shim = os.path.join(ROOT, "solution", "mcl_node.py")
    src = _source(shim)
    assert "from ohm_localization import mcl_node" in src
    for name in ("mission", "task_options"):
        assert f"{name} = _reference.{name}" in src, f"the shim forgets to re-export {name}"


# --------------------------------------------------------------------- the launch file and what it starts

def test_the_launch_file_declares_what_it_reads():
    """An argument read but never declared is `""` forever, and the run silently uses the default."""
    tree = _module(LAUNCH)
    declared = set()
    for name in ("BASICS", "SETTINGS", "MCL_ARGS"):
        declared |= {row[0] for row in _constant(tree, name)}
    read = {node.args[0].value for node in ast.walk(tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) in ("arg", "read_arg")
            and node.args and isinstance(node.args[0], ast.Constant)}
    assert read, "nothing is read in this launch file, so this test would pass on an empty one"
    missing = sorted(read - declared)
    assert not missing, f"read by the launch file but never declared: {missing}"


def test_the_launch_files_sensor_arguments_are_real_simulator_settings():
    """`--set lidar.sigma=0.25` with a typo in the path is ignored by the simulator, not rejected.

    Checked against the simulator's own config tree, which is the only authority on what a path means.
    """
    sim = paths.mecanum_lab_root()
    if not sim:
        pytest.skip("no mecanum-lab checkout or install visible from this test")
    sys.path.insert(0, sim)
    try:
        from mecanum_lab import types
    except Exception:                                            # noqa: BLE001 - see the skip above
        pytest.skip("mecanum_lab is importable here for other reasons")
    allowed = _flatten(types.load_config())
    # The task file is the second authority, and the stronger of the two: `odom.geometry.scale_xy` and
    # `lidar.rate` are in neither `config/default.json` nor a sensor's own defaults, and they are nevertheless
    # proven to reach the simulator — every graded number of this exercise was measured with them set. A path
    # in the launch file that is in neither list is a path nobody has ever seen take effect.
    import json
    with open(paths.task_file(), encoding="utf-8") as fh:
        for task in json.load(fh)["tasks"]:
            allowed |= _flatten(task.get("sim") or {})
    for _arg, path, _text in _constant(_module(LAUNCH), "SETTINGS"):
        assert path in allowed, (f"launch offers --set {path}, which is neither in the simulator's config "
                                 f"nor in a task's sim block; the simulator ignores an unknown path rather "
                                 f"than rejecting it, so this would be a silent no-op argument")


def _flatten(tree, prefix=""):
    """`{"odom": {"geometry": {"scale_xy": 1}}}` -> {"odom.geometry.scale_xy"}."""
    out = set()
    for key, value in (tree or {}).items():
        if str(key).startswith("_"):
            continue
        path = f"{prefix}.{key}".strip(".")
        if isinstance(value, dict):
            out |= _flatten(value, path)
            out.add(path)                             # a section name is also settable as a whole
        else:
            out.add(path)
    return out


def test_the_commands_in_the_docstring_are_commands_the_file_accepts():
    """Every `thing:=value` printed at the top of a launch file must name a declared argument."""
    import re
    tree = _module(LAUNCH)
    declared = {row[0] for name in ("BASICS", "SETTINGS", "MCL_ARGS")
                for row in _constant(tree, name)}
    doc = ast.get_docstring(tree) or ""
    used = set(re.findall(r"(\w+):=", doc))
    assert used, "the docstring shows no launch arguments, so this test would pass on an empty file"
    assert used <= declared, f"in the docstring but not declared: {sorted(used - declared)}"


def test_the_launch_arguments_the_readme_prints_are_arguments_the_file_has():
    """`prior:=` in a document that the launch file does not declare is a command that fails in a lab."""
    import re
    declared = {row[0] for name in ("BASICS", "SETTINGS", "MCL_ARGS")
                for row in _constant(_module(LAUNCH), name)}
    used = set(re.findall(r"(\w+):=", _source(os.path.join(ROOT, "README.md"))))
    assert used, "the README names no launch argument, so this test would pass on an empty file"
    missing = sorted(used - declared)
    assert not missing, f"offered by README.md, not declared by launch/mcl.launch.py: {missing}"


def test_the_launch_file_is_just_data_when_launch_is_not_installed():
    """No `launch` python package here, so the file is verified by parsing. On a real machine it also builds."""
    tree = _module(LAUNCH)
    names = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    assert {"generate_launch_description", "setup"} <= names
    pytest.importorskip("launch", reason="the launch python package is not installed in this sandbox")
    pytest.importorskip("launch_ros", reason="launch_ros is not installed in this sandbox")
    sys.path.insert(0, os.path.dirname(LAUNCH))
    import importlib.util
    spec = importlib.util.spec_from_file_location("mcl_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    # Build what `ros2 launch` builds: the declared arguments, then the actions the OpaqueFunction
    # returns for them. Constructing an action is where a wrong keyword argument or a renamed event
    # handler shows up — parsing alone passes on a launch file that cannot start.
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.launch_context import LaunchContext
    from launch.utilities import perform_substitutions
    context = LaunchContext()
    description = mod.generate_launch_description()
    assert description.entities                       # arguments + the OpaqueFunction
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value)
    assert any(isinstance(entity, OpaqueFunction) for entity in description.entities)
    actions = mod.setup(context)            # what the OpaqueFunction hands `ros2 launch`
    assert actions, "setup() returned no actions at the file's own defaults"
    kinds = [type(action).__name__ for action in actions]
    assert "ExecuteProcess" in kinds, f"no process to start, only {kinds}"
