"""
Build a client-deliverable copy of this repo with algorithms/ and calib/
compiled to .pyd (via Cython) instead of shipped as readable .py source.

Everything else -- main.py, backend/, config.py, Data/, fake/, requirements.txt
-- is copied as plain files. This only hides the measurement/calibration
algorithms (the actual IP); the server/hardware-orchestration layer stays
plain source since a client reading it gains little and it is more fragile
to compile (asyncio/aiohttp internals, hardware SDK glue).

Must run with the SAME interpreter used on the rig (Python 3.10 x64, the
CloudComPy310 env) -- a compiled .pyd is ABI-locked to one CPython minor
version, exactly like cloudComPy/PyRVC (see CLAUDE.md's "Environment
gotchas"). Building with any other interpreter produces a .pyd the rig's
Python cannot import.

Usage:
    python packaging/build_release.py [output_dir]

    output_dir defaults to a sibling directory: ../backend-inspection-release
    (deliberately outside this git repo -- it's a build artifact, not source).
"""

import shutil
import subprocess
import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parent.parent

# Directories whose .py files get compiled. Kept narrow and explicit rather
# than "everything" -- backend/ and main.py are server/hardware plumbing a
# client gains little from reading, and are more fragile to compile (asyncio,
# aiohttp, hardware SDK glue) for no real IP-protection benefit.
COMPILE_DIRS = ["algorithms", "calib"]

# Left as plain source even inside COMPILE_DIRS:
#   __init__.py     - all empty in this repo; compiling packages' __init__ adds
#                      Cython edge cases for zero protection benefit.
#   test_calibrate_rig.py - a dev-only verification script (`python
#                      calib/test_calibrate_rig.py`), not something a client
#                      derives value from reading, and not meant to run as a
#                      compiled extension's __main__.
EXCLUDE_NAMES = {"__init__.py", "test_calibrate_rig.py"}

# Not copied into the release at all.
COPY_IGNORE = shutil.ignore_patterns(
    ".git", ".gitignore", "__pycache__", "*.pyc", "*.pyo",
    "build", "packaging",  # this tool's own output/self
)


def check_interpreter():
    if sys.version_info[:2] != (3, 10):
        sys.exit(
            f"This must be run with Python 3.10 x64 (the rig's interpreter) -- "
            f"got {sys.version_info.major}.{sys.version_info.minor}. A .pyd "
            f"compiled with a different minor version will fail to import on "
            f"the rig with an opaque error. Re-run with the CloudComPy310 env's "
            f"python.exe."
        )
    try:
        import Cython  # noqa: F401
    except ImportError:
        sys.exit("Cython is not installed in this interpreter. `pip install cython`.")


def discover_targets(output_dir: Path):
    """(dotted_module_name, relative_py_path) for every file to compile."""
    targets = []
    for compile_dir in COMPILE_DIRS:
        root = output_dir / compile_dir
        for py_file in sorted(root.rglob("*.py")):
            if py_file.name in EXCLUDE_NAMES:
                continue
            rel = py_file.relative_to(output_dir)
            dotted = ".".join(rel.with_suffix("").parts)
            targets.append((dotted, rel))
    return targets


def _compile_one(output_dir: Path, dotted: str, rel: Path) -> bool:
    """Compile a single module in its own subprocess. One file's pre-existing
    bug (e.g. a genuine NameError in unreachable debug code that only Cython's
    stricter static analysis catches) must not abort the other N-1 files --
    report it and leave that one file as plain source instead."""
    setup_py = output_dir / "_cython_setup_one.py"
    setup_py.write_text(
        "from setuptools import setup, Extension\n"
        "from Cython.Build import cythonize\n"
        "import sys\n"
        "dotted, rel = sys.argv[1], sys.argv[2]\n"
        "setup(script_args=['build_ext', '--inplace'],\n"
        "      ext_modules=cythonize([Extension(dotted, [rel])],\n"
        "                            language_level=3,\n"
        "                            compiler_directives={'always_allow_keywords': True}))\n",
        encoding="utf-8",
    )
    result = subprocess.run(
        [sys.executable, str(setup_py), dotted, str(rel).replace("\\", "/")],
        cwd=output_dir,
        capture_output=True, text=True,
    )
    setup_py.unlink()
    if result.returncode != 0:
        print(f"  FAILED: {rel} -- left as plain source. Compiler output:")
        print("    " + "\n    ".join(result.stderr.strip().splitlines()[-15:]))
        return False
    return True


def build(output_dir: Path):
    targets = discover_targets(output_dir)
    if not targets:
        sys.exit(f"No .py files found under {COMPILE_DIRS} in {output_dir} -- "
                  f"did the copy step run correctly?")

    print(f"Compiling {len(targets)} modules (one at a time, so a single "
          f"failure doesn't block the rest):")

    compiled, failed = [], []
    for dotted, rel in targets:
        print(f"  {rel} ...")
        if _compile_one(output_dir, dotted, rel):
            compiled.append((dotted, rel))
        else:
            failed.append((dotted, rel))

    # Remove the .py sources that got compiled, and the generated .c
    # intermediates (those are near-fully-readable transliterated source --
    # keeping them around would defeat the entire point of this). Files that
    # failed to compile keep their .py (that's the whole point of isolating
    # failures above) but any stray .c from the failed attempt is cleaned up.
    for dotted, rel in compiled:
        (output_dir / rel).unlink()
    for dotted, rel in targets:
        c_file = (output_dir / rel).with_suffix(".c")
        if c_file.exists():
            c_file.unlink()

    # setuptools' build/ temp tree (.obj files, import libs, a duplicate copy
    # of each .pyd) -- only the in-place .pyd files under algorithms/calib/
    # matter for delivery.
    build_tmp = output_dir / "build"
    if build_tmp.exists():
        shutil.rmtree(build_tmp)

    print(f"\n{len(compiled)}/{len(targets)} modules compiled to .pyd.")
    if failed:
        print(f"{len(failed)} left as plain .py source (compile failed -- "
              f"see FAILED output above for each):")
        for dotted, rel in failed:
            print(f"  {rel}")


def main():
    check_interpreter()

    output_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        REPO_DIR.parent / "backend-inspection-release")

    if output_dir.exists():
        sys.exit(f"{output_dir} already exists -- remove it first (this script "
                  f"does not merge into an existing release directory).")

    print(f"Copying {REPO_DIR} -> {output_dir} ...")
    shutil.copytree(REPO_DIR, output_dir, ignore=COPY_IGNORE)

    build(output_dir)

    print(f"\nRelease copy ready at: {output_dir}")
    print(f"Ship this directory in place of a git checkout. It still needs the "
          f"same per-machine setup as the source repo (ENVIRONMENT.md, "
          f"envActivation.py's CloudComPy path, backend.config.json on the "
          f"Electron side) -- this only changes what's readable, not how it's run.")


if __name__ == "__main__":
    main()
