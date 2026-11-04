#!/usr/bin/env python3
"""Prove the real coverage profile excludes competing test processes."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
use std::{env, fs, path::PathBuf, thread, time::{Duration, Instant}};
fn state() -> PathBuf { PathBuf::from(env::var_os("SCHEDULING_STATE").unwrap()) }
#[test]
fn finding_sphere_minus_box_open_mesh() {
    let dir = state();
    fs::write(dir.join("started"), b"started").unwrap();
    thread::sleep(Duration::from_millis(300));
    fs::write(dir.join("finished"), b"finished").unwrap();
}
#[test]
fn competing_work() {
    let dir = state();
    let deadline = Instant::now() + Duration::from_secs(5);
    while !dir.join("started").exists() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(5));
    }
    assert!(dir.join("finished").exists(), "competing test entered the reserved pool");
}
'''


def main():
    with tempfile.TemporaryDirectory(prefix="coverage-scheduling-") as temporary:
        root = Path(temporary)
        (root / "tests").mkdir()
        (root / ".config").mkdir()
        (root / "Cargo.toml").write_text('[package]\nname = "coverage-scheduling-fixture"\nversion = "0.0.0"\nedition = "2021"\n[workspace]\n')
        (root / "tests/op_seq_81.rs").write_text(SOURCE)
        config = root / ".config/nextest.toml"
        original = (ROOT / ".config/nextest.toml").read_text()
        config.write_text(original)
        state = root / "state"
        command = ["cargo", "nextest", "run", "--test-threads", "2"]
        env = dict(os.environ, CARGO_TARGET_DIR=str(root / "target"),
                   SCHEDULING_STATE=str(state), NEXTEST_PROFILE="coverage")
        for exclusive in (True, False):
            state.mkdir()
            if not exclusive:
                altered = original.replace('threads-required = "num-test-threads"', '')
                assert altered != original
                config.write_text(altered)
            result = subprocess.run(command, cwd=root, env=env, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if exclusive:
                assert result.returncode == 0, result.stdout
            else:
                assert result.returncode != 0 and "competing test entered the reserved pool" in result.stdout, result.stdout
            shutil.rmtree(state)
        print("Coverage scheduling passed; removing the reservation fails the concurrency control.")


if __name__ == "__main__":
    main()
