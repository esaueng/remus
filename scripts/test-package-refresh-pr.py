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


def pull(number, merge_sha, body, merged=True, base='main', edited=None):
    # `edited` is the description's last edit time, which GraphQL alone serves.
    return {'number': number, 'merged_at': '2026-10-13T00:00:00Z' if merged else None,
            'base': {'ref': base}, 'merge_commit_sha': merge_sha, 'body': body, 'edited': edited}


def description_reply(item):
    return {'data': {'repository': {'pullRequest': {
        'body': item['body'], 'mergedAt': item['merged_at'], 'lastEditedAt': item['edited']}}}}


class PackageRefreshTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = self.root / 'checkout'
        self.repo.mkdir()
        # Retain the shared commit/push hooks. Use the checkout's configured
        # identity locally; unconfigured CI runners retain synthetic fixtures.
        identity = {}
        for key, fallback in (('name', 'fixture'), ('email', 'fixture@example.invalid')):
            configured = subprocess.run(['git', 'config', '--get', 'user.' + key],
                                        cwd=SCRIPT.parent.parent, capture_output=True,
                                        text=True, check=False)
            value = configured.stdout.strip() if configured.returncode == 0 else ''
            identity[key] = value or fallback
        self.env = dict(os.environ, GIT_AUTHOR_NAME=identity['name'], GIT_COMMITTER_NAME=identity['name'],
                        GIT_AUTHOR_EMAIL=identity['email'], GIT_COMMITTER_EMAIL=identity['email'],
                        GITHUB_REPOSITORY=REPO)
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
        # A fake gh: `api` answers from the GH_FIXTURE map (unknown paths fail;
        # a GraphQL query is keyed `graphql:<PR number>`), `pr create` records
        # its description, `pr list --head` finds no open PR, and a plain
        # `pr list` answers GH_OPEN_PRS.
        gh.write_text('#!/usr/bin/env python3\nimport os,sys,json\nfrom pathlib import Path\n'
                      'args=sys.argv[1:]\n'
                      'if "--body-file" in args: args.append(Path(args[args.index("--body-file")+1]).read_text())\n'
                      'with open(os.environ["GH_LOG"],"a") as f: f.write(json.dumps(args)+"\\n")\n'
                      'if args[0]=="api":\n'
                      '    fixture=json.loads(Path(os.environ["GH_FIXTURE"]).read_text())\n'
                      '    key=args[1]\n'
                      '    if key=="graphql": key+=":"+next(a for a in args if a.startswith("number="))[7:]\n'
                      '    if key not in fixture: sys.exit("gh: Not Found (HTTP 404)")\n'
                      '    print(json.dumps(fixture[key])); sys.exit()\n'
                      'if args[:2]==["pr","list"]: print("[]" if "--head" in args else os.environ.get("GH_OPEN_PRS","[]")); sys.exit()\n'
                      'print("https://example.invalid/pull/1")\n')
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
        for item in [*pulls, *(item for items in (commit_pulls or {}).values() for item in items)]:
            api[f'graphql:{item["number"]}'] = description_reply(item)
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
        # local run without it reports the skip.
        if shutil.which('node') is None:
            self.skipTest('node is required to run the size gate on the description')
        script = GATE_EVAL % json.dumps(GATE.as_uri())
        result = subprocess.run(['node', '--input-type=module', '-e', script],
                                input=json.dumps({'body': body, 'growth': growth, 'hardLimit': hard_limit}),
                                capture_output=True, text=True, check=True)
        return json.loads(result.stdout)

    def test_fixture_preserves_inherited_hooks_configuration(self):
        for key in ('GIT_CONFIG_COUNT', 'GIT_CONFIG_KEY_0', 'GIT_CONFIG_VALUE_0'):
            self.assertEqual(self.env.get(key), os.environ.get(key))

    def test_carries_source_declarations_so_declared_growth_passes_the_gate(self):
        squash = 'WASM-Size: +100 KiB raw, curve projection, Tier: core (writes the journal)'
        landed = 'WASM-Size: +40 KiB raw, sketch solver, Tier: lazy:sketch'
        unnumbered = 'WASM-Size: +30 KiB raw, draft angles, Tier: core'
        fillers = [commit(f'{n:040x}', f'docs: filler {n}') for n in range(1, 101)]
        # A hand-made refresh landed under a source-style title still marks its lines.
        sketch = pull(12, '3' * 40, f'{landed}\nWASM-Size: +9 KiB raw, old, Tier: core (carried from #3, by hand)\n')
        self.history(
            [*fillers,
             # An earlier refresh PR: its lines were carried there, never carry them again.
             commit('d' * 40, 'chore(wasm): refresh packages to v2026.1.3 from ' + 'e' * 12 + ' (#6)'),
             commit('1' * 40, 'feat(ops): curve projection (#11)'),
             commit('2' * 40, 'feat(sketch): solver step'),
             commit('3' * 40, 'Merge pull request #12 from fixture/sketch'),
             commit('4' * 40, 'fix(io): no size change (#13)'),
             # A subject naming a PR merged elsewhere does not count.
             commit('5' * 40, 'docs: follow up on (#14)'),
             commit('6' * 40, 'feat(ops): draft angles')],
            [pull(6, 'd' * 40, f'{squash}\n{landed} (carried from #12)'),
             pull(11, '1' * 40, f'Adds projection.\r\n\r\n{squash}\r\n<!--\r\nWASM-Size: hidden template, Tier: core\r\n-->'),
             sketch,
             pull(13, '4' * 40, 'WASM-Size: none'),
             pull(14, 'f' * 40, 'WASM-Size: +1 MiB raw, unrelated, Tier: core')],
            {'2' * 40: [sketch,
                        # An open PR's test merge, or a PR into another branch, is no source.
                        pull(15, '2' * 40, 'WASM-Size: open, Tier: core', merged=False),
                        pull(16, '2' * 40, 'WASM-Size: release, Tier: core', base='release')],
             '6' * 40: [pull(17, '6' * 40, unnumbered)]})
        body = self.description()
        self.assertIn('## WASM size declarations', body)
        self.assertEqual([line for line in body.splitlines() if line.startswith('WASM-Size:')],
                         [f'{squash} (carried from #11)', f'{landed} (carried from #12)',
                          'WASM-Size: none (carried from #13)', f'{unnumbered} (carried from #17)'])
        self.assertIn(f'`{PREVIOUS[:12]}`..`{self.source[:12]}`', body)
        self.assertEqual(self.gate(body, 140 * KIB), [])
        # A carried declaration never excuses a hard limit.
        self.assertEqual(self.gate(body, 140 * KIB, hard_limit=True), ['raw-hard'])

    def test_subject_naming_another_span_pr_still_looks_up_its_own_pr(self):
        own = 'WASM-Size: +70 KiB raw, rebased follow-up, Tier: core'
        # A rebase-landed commit whose subject names #11, which merged earlier in
        # the same span: the commit's own PR (#18) must still be found and carried.
        follow_up = pull(18, '2' * 40, own)
        self.history([commit('1' * 40, 'feat(ops): curve projection (#11)'),
                      commit('2' * 40, 'fix(ops): follow up (#11)')],
                     [pull(11, '1' * 40, 'No size change.'), follow_up],
                     {'2' * 40: [follow_up]})
        body = self.description()
        self.assertIn(f'{own} (carried from #18)', body.splitlines())
        self.assertEqual(self.gate(body, 70 * KIB), [])

    def test_subject_naming_an_unknown_pr_does_not_abort_collection(self):
        declared = 'WASM-Size: +80 KiB raw, curve projection, Tier: core (writes the journal)'
        own = 'WASM-Size: +65 KiB raw, cherry-picked fix, Tier: core'
        # #69 has no PR in this repo (an upstream-era number): its lookup 404s,
        # and the commit's own PR (#19) is still found through its association.
        cherry = pull(19, '1' * 40, own)
        self.history([commit('1' * 40, 'fix(ops): carried from upstream (#69)'),
                      commit('2' * 40, 'feat(ops): curve projection (#20)')],
                     [pull(20, '2' * 40, declared)],
                     {'1' * 40: [cherry]})
        body = self.description()
        self.assertNotIn('Could not collect', body)
        self.assertEqual([line for line in body.splitlines() if line.startswith('WASM-Size:')],
                         [f'{own} (carried from #19)', f'{declared} (carried from #20)'])
        self.assertEqual(self.gate(body, 145 * KIB), [])

    def test_squash_merged_refresh_dates_the_span_from_its_source(self):
        declared = 'WASM-Size: +80 KiB raw, curve projection, Tier: core (writes the journal)'
        self.history([commit('1' * 40, 'feat(ops): curve projection (#20)')],
                     [pull(20, '1' * 40, declared)])
        api = json.loads(self.fixture.read_text())
        # The newest refresh landed squash-merged under its PR title, which
        # names no source; an older one still has a source-style subject.
        api[f'repos/{REPO}/commits?sha={self.source}&path=crates/wasm/pkg/package.json&per_page=30'] = [
            commit('c' * 40, 'chore(wasm): refresh committed packages to v2026.1.3 (#5)'),
            commit('d' * 40, f'chore(wasm): refresh packages to v2026.1.2 from {"e" * 12} (#4)'),
        ]
        api[f'repos/{REPO}/pulls/5'] = {**pull(5, 'c' * 40, 'carried lines'),
                                         'head': {'ref': 'codex/wasm-refresh-' + PREVIOUS}}
        for key in [key for key in api if key.startswith(f'repos/{REPO}/compare/{PREVIOUS[:12]}...')]:
            api[key.replace(PREVIOUS[:12], PREVIOUS, 1)] = api.pop(key)
        self.fixture.write_text(json.dumps(api))
        body = self.description()
        self.assertNotIn('Could not collect', body)
        self.assertIn(f'`{PREVIOUS[:12]}`..`{self.source[:12]}`', body)
        self.assertEqual([line for line in body.splitlines() if line.startswith('WASM-Size:')],
                         [f'{declared} (carried from #20)'])
        calls = self.log.read_text()
        self.assertNotIn('e' * 12, calls)

    def test_declarations_edited_in_after_the_merge_are_not_carried(self):
        reviewed = 'WASM-Size: +70 KiB raw, curve projection, Tier: core'
        planted = 'WASM-Size: +4 MiB raw, unreviewed, Tier: core'
        # #21 was edited before it merged: what merged is what was reviewed.
        # #22's current description gained a line after the merge.
        self.history([commit('1' * 40, 'feat(ops): curve projection (#21)'),
                      commit('2' * 40, 'feat(ops): quiet growth (#22)')],
                     [pull(21, '1' * 40, reviewed, edited='2026-10-12T23:59:59Z'),
                      pull(22, '2' * 40, f'Declares nothing at merge.\n{planted}',
                           edited='2026-10-13T00:00:01Z')])
        body = self.description()
        self.assertEqual([line for line in body.splitlines() if line.startswith('WASM-Size:')],
                         [f'{reviewed} (carried from #21)'])
        self.assertNotIn(planted, body)
        self.assertIn('description changed after the PR merged: #22.', body)
        self.assertEqual(self.gate(body, 70 * KIB), [])

    def test_only_edited_descriptions_leave_nothing_carried(self):
        self.history([commit('1' * 40, 'feat(ops): quiet growth (#22)')],
                     [pull(22, '1' * 40, 'WASM-Size: +4 MiB raw, unreviewed, Tier: core',
                           edited='2026-10-14T00:00:00Z')])
        body = self.description()
        self.assertNotIn('\nWASM-Size:', body)
        self.assertIn('No source PR merged in', body)
        self.assertIn('description changed after the PR merged: #22.', body)
        # The edited-in line excuses nothing, so the growth still fails the gate.
        self.assertEqual(self.gate(body, 80 * KIB), ['growth-undeclared'])

    def test_description_lookup_failure_carries_nothing(self):
        self.history([commit('1' * 40, 'feat(ops): curve projection (#21)')],
                     [pull(21, '1' * 40, 'WASM-Size: +70 KiB raw, curve projection, Tier: core')])
        api = json.loads(self.fixture.read_text())
        del api['graphql:21']
        self.fixture.write_text(json.dumps(api))
        body = self.description()
        self.assertIn('Could not collect', body)
        self.assertNotIn('\nWASM-Size:', body)

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
        create = [call for call in calls if call[:2] == ['pr', 'create']]
        self.assertEqual(len(create), 1)
        self.assertNotIn('--draft', create[0])
        # After the PR exists, only the superseded-refresh lookup follows it.
        self.assertEqual([call[:2] for call in calls[calls.index(create[0]) + 1:]], [['pr', 'list']])
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


    def test_closes_only_refresh_prs_this_build_supersedes(self):
        older, diverged, unknown = 'a' * 40, 'b' * 40, 'c' * 40
        branch = 'codex/wasm-refresh-'
        self.env['GH_OPEN_PRS'] = json.dumps([
            {'number': 1, 'headRefName': branch + older, 'isCrossRepository': False},
            {'number': 2, 'headRefName': branch + diverged, 'isCrossRepository': False},
            # The compare API has no answer for it: never close on a guess.
            {'number': 3, 'headRefName': branch + unknown, 'isCrossRepository': False},
            # A fork can name its branch like a refresh.
            {'number': 4, 'headRefName': branch + older, 'isCrossRepository': True},
            {'number': 5, 'headRefName': 'feat/source-change', 'isCrossRepository': False},
            {'number': 6, 'headRefName': branch + 'not-a-sha', 'isCrossRepository': False},
            {'number': 7, 'headRefName': branch + self.source, 'isCrossRepository': False},
        ])
        self.fixture.write_text(json.dumps({
            f'repos/{REPO}/compare/{older}...{self.source}': {'status': 'ahead'},
            f'repos/{REPO}/compare/{diverged}...{self.source}': {'status': 'diverged'},
        }))
        result = self.publish()
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()]
        closes = [call for call in calls if call[:2] == ['pr', 'close']]
        self.assertEqual(len(closes), 1)
        self.assertEqual(closes[0][2], '1')
        self.assertIn('--delete-branch', closes[0])
        comment = closes[0][closes[0].index('--comment') + 1]
        self.assertIn('Superseded by https://example.invalid/pull/1', comment)
        # The replacement exists before anything is closed.
        self.assertLess(calls.index(next(c for c in calls if c[:2] == ['pr', 'create'])),
                        calls.index(closes[0]))

if __name__ == '__main__':
    unittest.main()
