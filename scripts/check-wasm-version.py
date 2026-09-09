#!/usr/bin/env python3
"""Reject changed WASM distributions that reuse or lower their release version."""
import argparse
import json
import re
import subprocess

ROOTS = ('crates/wasm/pkg', 'crates/wasm-io/pkg')
PROVENANCE = {'version', 'homepage', 'repository', 'remusSourceCommit'}


def git(*args):
    return subprocess.check_output(['git', *args], text=True).strip()


def manifest(revision, root):
    return json.loads(git('show', f'{revision}:{root}/package.json'))


def version(value):
    if not isinstance(value, str) or not re.fullmatch(r'(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)', value):
        raise ValueError(f'Expected a stable package version, got {value!r}')
    return tuple(map(int, value.split('.')))


def contract(package):
    # Nested conditional-export ordering is significant to Node resolution.
    return json.dumps({key: package[key] for key in sorted(package) if key not in PROVENANCE})


def check(base, head=None):
    before = [manifest(base, root) for root in ROOTS]
    after = [manifest(head or '', root) for root in ROOTS]
    old = [version(package['version']) for package in before]
    new = [version(package['version']) for package in after]
    if old[0] != old[1] or new[0] != new[1]:
        raise ValueError('Kernel and translator versions must match')
    args = ['diff', '--name-only', '-z', '--no-renames']
    args += [base, head] if head else ['--cached', base]
    paths = git(*args, '--', *ROOTS).split('\0')
    changed = any(contract(a) != contract(b) for a, b in zip(before, after))
    metadata = {f'{root}/package.json' for root in ROOTS}
    ignored = {f'{root}/.gitignore' for root in ROOTS}
    changed |= any(path and path not in metadata | ignored for path in paths)
    if new[0] < old[0] or (changed and new[0] <= old[0]):
        raise ValueError('Changed kernel packages must increase the shared version: '
                         f'{before[0]["version"]} -> {after[0]["version"]}')
    print(f'WASM version guard passed: {before[0]["version"]} -> {after[0]["version"]}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', required=True)
    parser.add_argument('--head', help='Omit to validate the staged index')
    options = parser.parse_args()
    check(options.base, options.head)
