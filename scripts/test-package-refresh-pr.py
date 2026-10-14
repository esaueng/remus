#!/usr/bin/env python3
"""Exercise publishing against local Git repositories without external writes."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().with_name('publish-wasm-package-pr.py')
GATE = Path(__file__).resolve().with_name('wasm-size-gate.mjs')
REPO = 'fixture/kernel'
PREVIOUS = 'a' * 40
KIB = 1024
# Run the size gate's own decision on a description, with synthetic size reports.
GATE_EVAL = """
import { readFileSync } from 'node:fs';
import { evaluateGate } from %s;
const { body, growth, hardLimit } = JSON.parse(readFileSync(0, 'utf8'));
const report = (rawBytes, failures = []) => ({
  metrics: { rawBytes, gzipBytes: 0, brotliBytes: 0 }, failures, warnings: [] });
const failures = hardLimit ? [{ code: 'raw-hard', metric: 'raw', bytes: 1, budgetBytes: 0, reason: 'fixture' }] : [];
const result = evaluateGate({ base: report(8000000), pr: report(8000000 + growth, failures), body });
console.log(JSON.stringify(result.failures.map((failure) => failure.code)));
"""


def commit(sha, subject):
    return {'sha': sha, 'commit': {'message': subject + '\n\nbody'}}


def pull(number, merge_sha, body, merged=True, base='main'):
    return {'number': number, 'merged_at': '2026-10-13T00:00:00Z' if merged else None,
            'base': {'ref': base}, 'merge_commit_sha': merge_sha, 'body': body}


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
                        GITHUB_REPOSITORY=REPO, GIT_CONFIG_COUNT='1',
                        GIT_CONFIG_KEY_0='core.hooksPath', GIT_CONFIG_VALUE_0='/dev/null')
        subprocess.run(['git', 'init', '--bare', str(self.root / 'remote.git')],
                       check=True, capture_output=True)
        self.git('init', '-b', 'main')
        for root in ('crates/wasm/pkg', 'crates/wasm-io/pkg'):
            package = self.repo / root
            package.mkdir(parents=True)
            (package / 'package.json').write_text('{"version":"2026.1.3"}')
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
        # A fake gh: `api` answers from the GH_FIXTURE map (unknown paths fail),
        # `pr create` records its description, `pr list` finds no open PR.
        gh.write_text('#!/usr/bin/env python3\nimport os,sys,json\nfrom pathlib import Path\n'
                      'args=sys.argv[1:]\n'
                      'if "--body-file" in args: args.append(Path(args[args.index("--body-file")+1]).read_text())\n'
                      'with open(os.environ["GH_LOG"],"a") as f: f.write(json.dumps(args)+"\\n")\n'
                      'if args[0]=="api":\n'
                      '    fixture=json.loads(Path(os.environ["GH_FIXTURE"]).read_text())\n'
                      '    if args[1] not in fixture: sys.exit("gh: Not Found (HTTP 404)")\n'
                      '    print(json.dumps(fixture[args[1]])); sys.exit()\n'
                      'print("[]" if args[:2]==["pr","list"] else "https://example.invalid/pull/1")\n')
        gh.chmod(0o755)
        self.log = self.root / 'gh.log'
        self.env['GH_LOG'] = str(self.log)
        self.fixture = self.root / 'api.json'
        self.fixture.write_text('{}')
        self.env['GH_FIXTURE'] = str(self.fixture)
        self.env['PATH'] = str(fakebin) + os.pathsep + self.env['PATH']
        self.stage()

    def git(self, *args):
        return subprocess.check_output(['git', *args], cwd=self.repo,
                                       env=self.env, stderr=subprocess.DEVNULL, text=True).strip()

    def stage(self, content=b'generated'):
        (self.repo / 'crates/wasm/pkg/kernel.wasm').write_bytes(content)
        for root in ('crates/wasm/pkg', 'crates/wasm-io/pkg'):
            (self.repo / root / 'package.json').write_text('{"version":"2026.1.4"}')
        self.git('add', 'crates/wasm/pkg', 'crates/wasm-io/pkg')

    def publish(self):
        return subprocess.run(['python3', str(SCRIPT)], cwd=self.repo,
                              env=self.env, capture_output=True, text=True)

    def history(self, commits, pulls, commit_pulls=None):
        """Serve a refresh history: the previous refresh, then `commits` up to the source."""
        commits = [*commits, commit(self.source, 'fix(kernel): the source commit')]
        api = {
            f'repos/{REPO}/commits?sha={self.source}&path=crates/wasm/pkg/package.json&per_page=30': [
                commit('b' * 40, 'docs(wasm): edit the package readme'),
                commit('c' * 40, f'chore(wasm): refresh packages to v2026.1.3 from {PREVIOUS[:12]} (#5)'),
            ],
        }
        for page in range(len(commits) // 100 + 1):
            api[f'repos/{REPO}/compare/{PREVIOUS[:12]}...{self.source}?per_page=100&page={page + 1}'] = {
                'commits': commits[page * 100:(page + 1) * 100]}
        for item in pulls:
            api[f'repos/{REPO}/pulls/{item["number"]}'] = item
        for item in commits:
            api.setdefault(f'repos/{REPO}/commits/{item["sha"]}/pulls', [])
        for sha, items in (commit_pulls or {}).items():
            api[f'repos/{REPO}/commits/{sha}/pulls'] = items
        self.fixture.write_text(json.dumps(api))

    def description(self):
        result = self.publish()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()]
        create = [call for call in calls if call[:2] == ['pr', 'create']]
        self.assertEqual(len(create), 1)
        return create[0][-1]

    def gate(self, body, growth, hard_limit=False):
        # wasm-version.yml runs this on ubuntu-24.04, which ships Node; a
        # self-hosted policy runner without it reports the skip.
        if shutil.which('node') is None:
            self.skipTest('node is required to run the size gate on the description')
        script = GATE_EVAL % json.dumps(GATE.as_uri())
        result = subprocess.run(['node', '--input-type=module', '-e', script],
                                input=json.dumps({'body': body, 'growth': growth, 'hardLimit': hard_limit}),
                                capture_output=True, text=True, check=True)
        return json.loads(result.stdout)

    def test_carries_source_declarations_so_declared_growth_passes_the_gate(self):
        squash = 'WASM-Size: +100 KiB raw, curve projection, Tier: core (writes the journal)'
        landed = 'WASM-Size: +40 KiB raw, sketch solver, Tier: lazy:sketch'
        fillers = [commit(f'{n:040x}', f'docs: filler {n}') for n in range(1, 101)]
        self.history(
            [*fillers,
             # The previous refresh's PR already carried a line: never carry it twice.
             commit('d' * 40, 'chore(wasm): refresh packages to v2026.1.3 from ' + 'e' * 12 + ' (#6)'),
             commit('1' * 40, 'feat(ops): curve projection (#11)'),
             commit('2' * 40, 'feat(sketch): solver step'),
             commit('3' * 40, 'Merge pull request #12 from fixture/sketch'),
             commit('4' * 40, 'fix(io): no size change (#13)'),
             # A subject naming a PR merged elsewhere does not count.
             commit('5' * 40, 'docs: follow up on (#14)')],
            [pull(6, 'd' * 40, f'{squash} (carried from #11)'),
             pull(11, '1' * 40, f'Adds projection.\r\n\r\n{squash}\r\n<!-- WASM-Size: hidden, Tier: core -->'),
             pull(12, '3' * 40, f'{landed}\n'),
             pull(13, '4' * 40, 'WASM-Size: none'),
             pull(14, 'f' * 40, 'WASM-Size: +1 MiB raw, unrelated, Tier: core')],
            {'2' * 40: [pull(12, '3' * 40, f'{landed}\n'), pull(15, None, 'WASM-Size: open, Tier: core', merged=False)]})
        body = self.description()
        self.assertIn('## WASM size declarations', body)
        self.assertEqual([line for line in body.splitlines() if line.startswith('WASM-Size:')],
                         [f'{squash} (carried from #11)', f'{landed} (carried from #12)',
                          'WASM-Size: none (carried from #13)'])
        self.assertIn(f'`{PREVIOUS[:12]}`..`{self.source[:12]}`', body)
        self.assertEqual(self.gate(body, 140 * KIB), [])
        # A carried declaration never excuses a hard limit.
        self.assertEqual(self.gate(body, 140 * KIB, hard_limit=True), ['raw-hard'])

    def test_growth_within_the_threshold_needs_no_carried_line(self):
        self.history([commit('1' * 40, 'fix(ops): small change (#11)')],
                     [pull(11, '1' * 40, 'No size change worth declaring.')])
        body = self.description()
        self.assertNotIn('\nWASM-Size:', body)
        self.assertIn('No source PR merged in', body)
        self.assertEqual(self.gate(body, 64 * KIB), [])

    def test_undeclared_source_growth_still_fails_the_gate(self):
        self.history([commit('1' * 40, 'feat(ops): grows quietly (#11)'),
                      commit('2' * 40, 'feat(ops): grows quietly too (#12)')],
                     [pull(11, '1' * 40, '<!-- WASM-Size: +40 KiB, template, Tier: core -->'),
                      pull(12, '2' * 40, None)])
        body = self.description()
        self.assertNotIn('\nWASM-Size:', body)
        self.assertEqual(self.gate(body, 80 * KIB), ['growth-undeclared'])

    def test_lookup_failure_publishes_with_a_note_and_the_gate_fails_closed(self):
        body = self.description()  # the default fixture answers no API call
        self.assertIn('Could not collect', body)
        self.assertNotIn('\nWASM-Size:', body)
        self.assertEqual(self.gate(body, 80 * KIB), ['growth-undeclared'])

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
