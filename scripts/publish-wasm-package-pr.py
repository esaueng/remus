#!/usr/bin/env python3
"""Publish staged package output on a source-pinned branch for normal PR review."""
import json
import os
import re
import subprocess
import tempfile
from pathlib import Path


def run(*args):
    return subprocess.check_output(args, text=True).strip()


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
    version = json.loads(Path('crates/wasm/pkg/package.json').read_text())['version']
    if not re.fullmatch(r'\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?', version):
        raise RuntimeError('Invalid package version')
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
            'If source changes before integration, rebuild instead of reusing stale output.\n')
    with tempfile.TemporaryDirectory() as directory:
        body_file = Path(directory) / 'body.md'
        body_file.write_text(body)
        print(run('gh', 'pr', 'create', '--repo', os.environ['GITHUB_REPOSITORY'],
                  '--base', 'main', '--head', branch,
                  '--title', f'chore(wasm): refresh committed packages to v{version}',
                  '--body-file', str(body_file)))


if __name__ == '__main__':
    publish()
