"""Compile the ACU logic + sim.c into a DLL the Python harness can load.

Uses `zig cc` from the `ziglang` pip package (py -m pip install ziglang) unless a
`gcc`/`clang` is on PATH. Run directly to build, or let acu_sim.py call build().
"""
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = HERE / "build" / "acu_sim.dll"

# The firmware files under test. Everything else is stubbed in sim.c.
SOURCES = [
    ROOT / "Core/Src/APP.c",
    ROOT / "Core/Src/state_machine.c",
    ROOT / "Core/Src/Autonomous_functions.c",
    ROOT / "Core/Src/hardware_abstaction.c",
    ROOT / "Core/Src/ring_buffer.c",
    ROOT / "Core/Src/ble_handler.c",
    ROOT / "Core/DBC/autonomous_t26.c",
    HERE / "sim.c",
]
HEADER_DIRS = [HERE / "stub", ROOT / "Core/Inc", ROOT / "Core/DBC"]

FLAGS = [
    "-std=gnu11", "-O1", "-g", "-shared",
    "-D__ARM_ARCH_7EM__=1",  # makes millis() use HAL_GetTick() like on the target
    "-w",
    # The firmware relies on GCC accepting these; clang makes them errors by default.
    "-Wno-error=implicit-function-declaration",
    "-Wno-error=int-conversion",
    "-Wno-error=incompatible-pointer-types",
]


def compiler():
    for cc in ("gcc", "clang"):
        if shutil.which(cc):
            return [cc]
    try:
        import ziglang  # noqa: F401
    except ImportError:
        sys.exit("No C compiler. Install one with:  py -m pip install ziglang")
    return [sys.executable, "-m", "ziglang", "cc", "-target", "x86_64-windows-gnu"]


def needs_build():
    if not OUT.exists():
        return True
    built = OUT.stat().st_mtime
    inputs = SOURCES + list((HERE / "stub").glob("*.h")) + list((ROOT / "Core/Inc").glob("*.h"))
    return any(p.stat().st_mtime > built for p in inputs)


def build(force=False):
    if not force and not needs_build():
        return OUT
    OUT.parent.mkdir(exist_ok=True)
    cmd = compiler() + FLAGS + [f"-I{d}" for d in HEADER_DIRS] + [str(s) for s in SOURCES] + ["-o", str(OUT)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        sys.exit("Build failed:\n" + result.stderr)
    return OUT


if __name__ == "__main__":
    print(build(force=True))
