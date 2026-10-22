#!/usr/bin/env python3
"""Run the portable proof orchestration regressions without shell-script stubs."""
from pathlib import Path
import subprocess
import sys


if __name__ == "__main__":
    root = Path(__file__).resolve().parent.parent
    sys.exit(subprocess.run(
        ["node", "--test", "tools/parity/proof-runner.test.mjs"], cwd=root, check=False,
    ).returncode)
