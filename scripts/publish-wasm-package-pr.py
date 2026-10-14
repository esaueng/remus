#!/usr/bin/env python3
"""Publish staged package output on a source-pinned branch for normal PR review.

A refresh PR carries the combined kernel growth of every source PR merged since
the committed package was built, and the size gate (`scripts/wasm-size-gate.mjs`)
reads `WASM-Size:` lines from the refresh PR's own description. So the
description carries the declarations of those source PRs, each marked with its
PR number. When no source PR declared one, nothing is carried and the gate
still fails on growth above its threshold, so a human reviews it.
"""
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path


def run(*args):
    return subprocess.check_output(args, text=True).strip()


PACKAGE_MANIFEST = 'crates/wasm/pkg/package.json'
# Subjects of refresh commits on main, old and new style.
REFRESH_SUBJECT = re.compile(
    r'^chore\(wasm\): refresh (?:committed )?packages to v\S+ from ([0-9a-f]{7,40})\b')
PR_SUBJECT = re.compile(r'\(#([0-9]+)\)$|^Merge pull request #([0-9]+) ')
CARRIED = re.compile(r'\(carried from #[0-9]+')
DECLARATION_PREFIX = 'WASM-Size:'
PAGE = 100


def api(path):
    return json.loads(run('gh', 'api', path))


def size_declarations(body):
    """The `WASM-Size:` lines of a description, as the size gate reads them."""
    text = re.sub(r'<!--.*?(?:-->|\Z)', '', (body or '').replace('\r\n', '\n').replace('\r', '\n'),
                  flags=re.S)
    return [line.strip() for line in text.split('\n') if line.startswith(DECLARATION_PREFIX)]


def previous_source(repo, source):
    """Source commit of the newest package refresh in the history of `source`."""
    for commit in api(f'repos/{repo}/commits?sha={source}&path={PACKAGE_MANIFEST}&per_page=30'):
        match = REFRESH_SUBJECT.match(commit['commit']['message'].split('\n', 1)[0])
        if match:
            return match.group(1)
    raise RuntimeError(f'no package refresh found in the history of {source[:12]}')


def merged_prs(repo, base, head):
    """The PRs merged into main whose merge commit lies in base..head, by number."""
    commits = []
    page = 1
    while True:
        batch = api(f'repos/{repo}/compare/{base}...{head}?per_page={PAGE}&page={page}')['commits']
        commits += batch
        if len(batch) < PAGE:
            break
        page += 1
    shas = {commit['sha'] for commit in commits}
    pulls = {}

    def accept(number, pull=None):
        if number in pulls:
            return True
        pull = pull or api(f'repos/{repo}/pulls/{number}')
        if pull.get('merged_at') and pull['base']['ref'] == 'main' and pull.get('merge_commit_sha') in shas:
            pulls[number] = pull
            return True
        return False

    for commit in commits:
        subject = commit['commit']['message'].split('\n', 1)[0]
        if REFRESH_SUBJECT.match(subject):
            continue
        match = PR_SUBJECT.search(subject)
        if match:
            number = int(match.group(1) or match.group(2))
            # Only this commit's own PR settles it: a subject may name another
            # PR merged in the same span (a rebased follow-up, for example).
            if accept(number) and pulls[number].get('merge_commit_sha') == commit['sha']:
                continue
        # Commits of a merge-commit landing, or a subject without its PR number.
        for pull in api(f'repos/{repo}/commits/{commit["sha"]}/pulls'):
            accept(pull['number'], pull)
    return [pulls[number] for number in sorted(pulls)]


def size_section(repo, source):
    """The description section that carries the source PRs' size declarations."""
    heading = '## WASM size declarations\n\n'
    try:
        base = previous_source(repo, source)
        carried = [f'{line} (carried from #{pull["number"]})'
                   for pull in merged_prs(repo, base, source)
                   for line in size_declarations(pull.get('body'))
                   if not CARRIED.search(line)]
    except (subprocess.CalledProcessError, RuntimeError, KeyError, TypeError, ValueError) as error:
        return (heading + f'Could not collect the source PRs\' `{DECLARATION_PREFIX}` lines ({error}). '
                'If the size gate reports undeclared growth, copy the source PRs\' declarations here.\n')
    span = f'`{base}`..`{source[:12]}`'
    if not carried:
        return (heading + f'No source PR merged in {span} declared a `{DECLARATION_PREFIX}` line. '
                'If the size gate reports undeclared growth, review those PRs and declare it here.\n')
    return (heading + f'Carried from the source PRs merged in {span}, whose combined growth this '
            'refresh packages:\n\n' + '\n'.join(carried) + '\n')


def publish():
    source = os.environ['GITHUB_SHA']
    if not re.fullmatch(r'[0-9a-f]{40}', source) or run('git', 'rev-parse', 'HEAD') != source:
        raise RuntimeError('Package checkout must match the exact build source commit')
    paths = run('git', 'diff', '--cached', '--name-only', '-z').split('\0')
    paths = [p for p in paths if p]
    if not paths:
        return
    if any(not p.startswith(('crates/wasm/pkg/', 'crates/wasm-io/pkg/')) for p in paths):
        raise RuntimeError('Only generated package files may be published')
    run('python3', str(Path(__file__).with_name('check-wasm-version.py')), '--base', source)
    version = json.loads(run('git', 'show', ':crates/wasm/pkg/package.json'))['version']
    branch = 'codex/wasm-refresh-' + source
    tree = run('git', 'write-tree')
    auth = ['git', '-c', 'credential.helper=!gh auth git-credential']
    existing = run(*auth, 'ls-remote', '--heads', 'origin', 'refs/heads/' + branch)
    if existing:
        run(*auth, 'fetch', 'origin', branch)
        if run('git', 'rev-parse', 'FETCH_HEAD^{tree}') != tree:
            raise RuntimeError('Existing refresh branch has different output; refusing to overwrite it')
    else:
        run('git', '-c', 'user.name=github-actions[bot]',
            '-c', 'user.email=41898282+github-actions[bot]@users.noreply.github.com',
            'commit', '-m', f'chore(wasm): refresh packages to v{version} from {source[:12]}')
        run(*auth, 'push', 'origin', 'HEAD:refs/heads/' + branch)
    prs = json.loads(run('gh', 'pr', 'list', '--repo', os.environ['GITHUB_REPOSITORY'],
                         '--head', branch, '--base', 'main', '--state', 'open', '--json', 'url'))
    if prs:
        print(prs[0]['url'])
        return
    body = (f'Refresh both committed WASM packages built and validated from `{source}`.\n\n'
            'The build runner completed optimized builds, smoke tests, and installed-package '
            'consumer checks before transferring the package archive to this publisher.\n\n'
            'Review flags: generated distribution files only. This PR requires normal checks '
            'and review; it does not merge automatically or bypass branch protection. '
            'If source changes before integration, rebuild instead of reusing stale output.\n\n'
            + size_section(os.environ['GITHUB_REPOSITORY'], source))
    with tempfile.TemporaryDirectory() as directory:
        body_file = Path(directory) / 'body.md'
        body_file.write_text(body)
        print(run('gh', 'pr', 'create', '--repo', os.environ['GITHUB_REPOSITORY'],
                  '--base', 'main', '--head', branch,
                  '--title', f'chore(wasm): refresh committed packages to v{version}',
                  '--body-file', str(body_file)))


if __name__ == '__main__':
    publish()
