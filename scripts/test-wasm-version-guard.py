#!/usr/bin/env python3
"""Version enforcement against real Git snapshots and the publisher boundary."""
import json
from pathlib import Path
import runpy
import subprocess
import unittest

HERE = Path(__file__).resolve().parent
PackageRefreshTests = runpy.run_path(str(HERE / 'test-package-refresh-pr.py'))['PackageRefreshTests']


class VersionGuardTests(PackageRefreshTests):
    def versions(self, kernel, translator=None, staged=True):
        for root, value in zip(('crates/wasm/pkg', 'crates/wasm-io/pkg'),
                               (kernel, translator or kernel)):
            (self.repo / root / 'package.json').write_text(json.dumps({'version': value}))
        if staged:
            self.git('add', 'crates')

    def guard(self, head=None):
        args = ['python3', str(HERE / 'check-wasm-version.py'), '--base', self.source]
        if head:
            args += ['--head', head]
        return subprocess.run(args, cwd=self.repo, env=self.env, capture_output=True, text=True)

    def test_same_version_cannot_publish_changed_bytes(self):
        self.versions('1.2.3')
        result = self.publish()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('must increase', result.stderr)
        self.assertFalse(self.log.exists())
        self.assertNotIn('wasm-refresh', self.git('ls-remote', '--heads', 'origin'))

    def test_unstaged_bump_cannot_hide_a_stale_staged_version(self):
        self.versions('1.2.3')
        self.versions('1.2.4', staged=False)
        self.assertNotEqual(self.publish().returncode, 0)
        self.assertFalse(self.log.exists())

    def test_translator_only_change_requires_a_shared_bump(self):
        self.git('reset', '--hard', self.source)
        (self.repo / 'crates/wasm-io/pkg/kernel.wasm').write_bytes(b'new translator')
        self.git('add', 'crates')
        self.assertNotEqual(self.guard().returncode, 0)
        self.versions('1.2.4')
        self.assertEqual(self.guard().returncode, 0)

    def test_split_versions_and_rollbacks_are_refused(self):
        self.versions('1.2.4', '1.2.3')
        self.assertIn('must match', self.guard().stderr)
        self.versions('1.2.2')
        self.assertIn('must increase', self.guard().stderr)

    def test_committed_pr_diff_requires_increase(self):
        self.versions('1.2.3')
        self.git('commit', '-m', 'unversioned package')
        self.assertNotEqual(self.guard('HEAD').returncode, 0)
        self.versions('1.2.4')
        self.git('commit', '-m', 'versioned package')
        self.assertEqual(self.guard('HEAD').returncode, 0)

    def test_later_base_detects_reused_version_from_parallel_refresh(self):
        self.git('commit', '-m', 'first refresh')
        self.source = self.git('rev-parse', 'HEAD')
        self.stage(b'second refresh')
        self.assertNotEqual(self.guard().returncode, 0)
        self.versions('1.2.5')
        self.assertEqual(self.guard().returncode, 0)

    def test_unchanged_payload_and_provenance_only_edits_keep_version(self):
        self.git('reset', '--hard', self.source)
        path = self.repo / 'crates/wasm/pkg/package.json'
        path.write_text(json.dumps({'version': '1.2.3', 'homepage': 'https://example.invalid'}))
        self.git('add', 'crates')
        self.assertEqual(self.guard().returncode, 0)

    def test_conditional_export_priority_requires_a_bump(self):
        self.git('reset', '--hard', self.source)
        path = self.repo / 'crates/wasm/pkg/package.json'
        path.write_text(json.dumps({'version': '1.2.3', 'exports': {'.': {'node': './a.js', 'default': './b.js'}}}))
        self.git('add', 'crates'); self.git('commit', '-m', 'fixture exports')
        self.source = self.git('rev-parse', 'HEAD')
        path.write_text(json.dumps({'version': '1.2.3', 'exports': {'.': {'default': './b.js', 'node': './a.js'}}}))
        self.git('add', 'crates')
        self.assertNotEqual(self.guard().returncode, 0)


if __name__ == '__main__':
    unittest.main()
