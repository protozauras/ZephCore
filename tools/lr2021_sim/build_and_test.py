#!/usr/bin/env python3
"""Windowless MSVC build + test runner for tools/lr2021_sim.

Rules (user directive 2026-09-19):
  - No cmd.exe windows: everything runs under subprocess with
    CREATE_NO_WINDOW.
  - SetErrorMode(0x8007) (SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX
    | SEM_NOOPENFILEERRORBOX) is set BEFORE spawning children so crashed
    test exes never pop a WER dialog; the error mode is inherited.
  - vcvars64 is invoked inside the windowless cmd (build_msvc2.bat).

Usage:  python build_and_test.py [build|test|all]   (default: all)
"""

import ctypes
import os
import subprocess
import sys

SIM_DIR = os.path.dirname(os.path.abspath(__file__))
CREATE_NO_WINDOW = 0x08000000

# Kill WER crash dialogs for this process and all children.
ctypes.windll.kernel32.SetErrorMode(0x8007)


def run(argv, cwd=None, log=None):
    """Run a command windowless; optionally tee output to a log file."""
    lf = open(log, "w", encoding="utf-8", errors="replace") if log else None
    try:
        p = subprocess.run(argv, cwd=cwd or SIM_DIR,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT,
                           creationflags=CREATE_NO_WINDOW)
        out = p.stdout.decode("utf-8", errors="replace")
        if lf:
            lf.write(out)
        return p.returncode, out
    finally:
        if lf:
            lf.close()


def build():
    rc, out = run(["cmd.exe", "/c", "build_msvc2.bat"])
    if rc != 0 or "CL_EXIT=0" not in out or "SIM_EXIT=0" not in out \
            or "VENDOR_EXIT=0" not in out:
        print(out)
        for name in ("build_cl.log", "build_cl_vendor.log",
                     "build_cl_link.log"):
            path = os.path.join(SIM_DIR, name)
            if os.path.exists(path):
                print(f"--- {name} ---")
                print(open(path, encoding="utf-8",
                           errors="replace").read()[-4000:])
        return 1
    print("MSVC build: OK (vendor + sim)")
    return 0


def test():
    rc, out = run([os.path.join(SIM_DIR, "lr2021_sim_tests.exe")])
    print(out)
    print(f"test exit code: {rc}")
    return 0 if rc == 0 else 1


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("build", "all"):
        if build() != 0:
            return 1
    if what in ("test", "all"):
        return test()
    return 0


if __name__ == "__main__":
    sys.exit(main())
