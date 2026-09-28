#!/usr/bin/env python3
"""Exercise the package-notifier release selection with a stub GitHub API."""

import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parent.parent
WORKFLOW = ROOT / ".github/workflows/notify-openzcad-package.yml"


def release_script():
    lines = WORKFLOW.read_text().splitlines()
    start = lines.index("      - name: Confirm paired package publication")
    run = lines.index("        run: |", start) + 1
    end = next(i for i in range(run, len(lines)) if lines[i].startswith("      - uses:"))
    return textwrap.dedent("\n".join(lines[run:end])) + "\n"


class NotifierTests(unittest.TestCase):
    def run_release(self, main_kernel, main_io, event_kernel, event_io, event_sha):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            gh = root / "gh"
            gh.write_text(
                "#!/usr/bin/env python3\n"
                "import os, sys\n"
                "url = sys.argv[2]\n"
                "kind = 'KERNEL' if 'crates%2Fwasm%2Fpkg' in url else 'IO'\n"
                "revision = 'MAIN' if '&sha=main&' in url else 'EVENT'\n"
                "print(os.environ[revision + '_' + kind])\n"
            )
            gh.chmod(0o755)
            output = root / "output"
            summary = root / "summary"
            env = dict(
                os.environ,
                PATH=f"{root}:{os.environ['PATH']}",
                GITHUB_SHA=event_sha,
                GITHUB_OUTPUT=str(output),
                GITHUB_STEP_SUMMARY=str(summary),
                MAIN_KERNEL=main_kernel,
                MAIN_IO=main_io,
                EVENT_KERNEL=event_kernel,
                EVENT_IO=event_io,
            )
            result = subprocess.run(
                ["bash", "-e", "-o", "pipefail", "-c", release_script()],
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            return result, output.read_text() if output.exists() else ""

    def test_batched_push_dispatches_current_release(self):
        release = "a" * 40
        event = "b" * 40
        result, output = self.run_release(release, release, release, release, event)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output, "notify=true\n")

    def test_superseded_event_skips_dispatch(self):
        current = "c" * 40
        prior = "a" * 40
        result, output = self.run_release(current, current, prior, prior, prior)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output, "notify=false\n")

    def test_split_main_release_fails(self):
        result, output = self.run_release("a" * 40, "b" * 40, "a" * 40, "b" * 40, "c" * 40)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(output, "")


if __name__ == "__main__":
    unittest.main()
