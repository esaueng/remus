#!/usr/bin/env python3
"""Exercise publishing against local Git repositories without external writes."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().with_name('publish-wasm-package-pr.py')


class PackageRefreshTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / 'checkout'
        self.repo.mkdir()
        self.env = dict(os.environ, GIT_AUTHOR_NAME='fixture', GIT_COMMITTER_NAME='fixture',
                        GIT_AUTHOR_EMAIL='fixture@example.invalid',
                        GIT_COMMITTER_EMAIL='fixture@example.invalid',
                        GITHUB_REPOSITORY='fixture/kernel', GIT_CONFIG_COUNT='1',
                        GIT_CONFIG_KEY_0='core.hooksPath', GIT_CONFIG_VALUE_0='/dev/null')
        subprocess.run(['git', 'init', '--bare', str(self.root / 'remote.git')],
                       check=True, capture_output=True)
        self.git('init', '-b', 'main')
        for root in ('crates/wasm/pkg', 'crates/wasm-io/pkg'):
            package = self.repo / root
            package.mkdir(parents=True)
            (package / 'package.json').write_text('{"version":"1.2.3"}')
            (package / 'kernel.wasm').write_bytes(b'baseline')
        self.git('add', '.')
        self.git('commit', '-m', 'fixture')
        self.source = self.git('rev-parse', 'HEAD')
        self.env['GITHUB_SHA'] = self.source
        self.git('remote', 'add', 'origin', str(self.root / 'remote.git'))
        self.git('push', 'origin', 'main')
        fakebin = self.root / 'bin'
        fakebin.mkdir()
        gh = fakebin / 'gh'
        gh.write_text('#!/usr/bin/env python3\nimport os,sys,json\n'
                      'with open(os.environ["GH_LOG"],"a") as f: f.write(json.dumps(sys.argv[1:])+"\\n")\n'
                      'print("[]" if sys.argv[1:3]==["pr","list"] else "https://example.invalid/pull/1")\n')
        gh.chmod(0o755)
        self.log = self.root / 'gh.log'
        self.env['GH_LOG'] = str(self.log)
        self.env['PATH'] = str(fakebin) + os.pathsep + self.env['PATH']
        self.stage()

    def git(self, *args):
        return subprocess.check_output(['git', *args], cwd=self.repo,
                                       env=self.env, stderr=subprocess.DEVNULL, text=True).strip()

    def stage(self, content=b'generated'):
        (self.repo / 'crates/wasm/pkg/kernel.wasm').write_bytes(content)
        for root in ('crates/wasm/pkg', 'crates/wasm-io/pkg'):
            (self.repo / root / 'package.json').write_text('{"version":"1.2.4"}')
        self.git('add', 'crates/wasm/pkg', 'crates/wasm-io/pkg')

    def publish(self):
        return subprocess.run(['python3', str(SCRIPT)], cwd=self.repo,
                              env=self.env, capture_output=True, text=True)

    def test_creates_source_pinned_branch_and_ready_pr_without_touching_main(self):
        result = self.publish()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git('ls-remote', 'origin', 'refs/heads/main').split()[0], self.source)
        self.assertEqual(self.git('rev-parse', 'HEAD^'), self.source)
        self.assertNotIn('[skip ci]', self.git('log', '-1', '--format=%s'))
        branch = 'codex/wasm-refresh-' + self.source
        self.assertIn(branch, self.git('ls-remote', '--heads', 'origin'))
        calls = [json.loads(line) for line in self.log.read_text().splitlines()]
        self.assertEqual(calls[-1][:2], ['pr', 'create'])
        self.assertNotIn('--draft', calls[-1])
        self.assertFalse(any('merge' in call for call in calls))

    def test_refuses_mismatched_source_before_pushing(self):
        self.env['GITHUB_SHA'] = '0' * 40
        self.assertNotEqual(self.publish().returncode, 0)
        self.assertNotIn('wasm-refresh', self.git('ls-remote', '--heads', 'origin'))

    def test_refuses_staged_source_code(self):
        (self.repo / 'source.rs').write_text('synthetic fixture')
        self.git('add', 'source.rs')
        self.assertNotEqual(self.publish().returncode, 0)
        self.assertNotIn('wasm-refresh', self.git('ls-remote', '--heads', 'origin'))

    def test_rerun_reuses_output_but_refuses_overwriting_different_output(self):
        self.assertEqual(self.publish().returncode, 0)
        published = self.git('rev-parse', 'HEAD')
        self.git('reset', '--hard', self.source)
        self.stage()
        result = self.publish()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git('ls-remote', 'origin', 'refs/heads/codex/wasm-refresh-' + self.source).split()[0], published)
        self.stage(b'different output')
        result = self.publish()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('refusing to overwrite', result.stderr)
        self.assertEqual(self.git('ls-remote', 'origin', 'refs/heads/codex/wasm-refresh-' + self.source).split()[0], published)


if __name__ == '__main__':
    unittest.main()
