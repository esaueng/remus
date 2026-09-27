import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { test } from 'node:test';
import { nextVersion, versionPackages } from './version-wasm-packages.mjs';

const roots = ['crates/wasm/pkg', 'crates/wasm-io/pkg'];
function fixture(t) {
  const root = mkdtempSync(join(tmpdir(), 'remus-version-test-'));
  t.after(() => rmSync(root, { recursive: true, force: true }));
  const git = (...args) => execFileSync('git', ['-c', 'core.hooksPath=/dev/null', ...args], {
    cwd: root, stdio: 'pipe', env: { ...process.env,
      GIT_AUTHOR_NAME: 'fixture', GIT_COMMITTER_NAME: 'fixture',
      GIT_AUTHOR_EMAIL: 'fixture@example.invalid', GIT_COMMITTER_EMAIL: 'fixture@example.invalid'
    }
  });
  for (const [i, path] of roots.entries()) {
    mkdirSync(join(root, path), { recursive: true });
    writeFileSync(join(root, path, 'package.json'), JSON.stringify({
      name: i ? 'remus-wasm-io' : 'remus-wasm', version: '2026.1.0', files: ['kernel.wasm', 'index.js']
    }));
    writeFileSync(join(root, path, 'kernel.wasm'), 'baseline');
    writeFileSync(join(root, path, 'index.js'), 'export const kernel = 1;');
  }
  git('init'); git('add', '.'); git('commit', '-m', 'fixture');
  return { root, git };
}
function update(root, index, fields) {
  const path = join(root, roots[index], 'package.json');
  writeFileSync(path, JSON.stringify({ ...JSON.parse(readFileSync(path)), ...fields }));
}
function versions(root) {
  return roots.map(path => JSON.parse(readFileSync(join(root, path, 'package.json'))).version);
}

test('stable versions advance patches and honor explicit release increases', () => {
  assert.equal(nextVersion('2026.1.0', '2026.1.0', true), '2026.1.1');
  assert.equal(nextVersion('2026.1.9', '2026.1.0', true), '2026.1.10');
  assert.equal(nextVersion('2026.1.9', '2026.1.0', false), '2026.1.9');
  assert.equal(nextVersion('2026.1.9', '2026.2.0', false), '2026.2.0');
  assert.throws(() => nextVersion('2026.1.0-beta', '2026.1.0', true));
  assert.throws(() => nextVersion('2026.1.0', 'garbage', true));
  assert.throws(() => nextVersion('2026.1.0', '3.0.0', true), /Remus calendar/);
  assert.equal(nextVersion('2.130.63', '2026.1.0', true), '2026.1.0');
  assert.equal(nextVersion('2026.1.9', '2027.1.0', true), '2027.1.0');
});
for (const index of [0, 1]) {
  test(`changing package ${index} advances both versions exactly once`, t => {
    const { root, git } = fixture(t);
    writeFileSync(join(root, roots[index], 'kernel.wasm'), 'new kernel');
    assert.equal(versionPackages(root), '2026.1.1');
    assert.deepEqual(versions(root), ['2026.1.1', '2026.1.1']);
    assert.equal(versionPackages(root), '2026.1.1');
    git('add', '.'); git('commit', '-m', 'generated package');
    // wasm-pack emits the Cargo version again on the next build.
    for (const i of [0, 1]) update(root, i, { version: '2026.1.0' });
    assert.equal(versionPackages(root), '2026.1.1');
    writeFileSync(join(root, roots[index], 'index.js'), 'export const kernel = 2;');
    assert.equal(versionPackages(root), '2026.1.2');
  });
}
test('unchanged packages, metadata order, provenance and build leftovers retain version', t => {
  const { root } = fixture(t);
  update(root, 0, { homepage: 'https://example.invalid', remusSourceCommit: 'a'.repeat(40) });
  writeFileSync(join(root, roots[0], 'local-build.log'), 'ignored by npm');
  assert.equal(versionPackages(root), '2026.1.0');
});
test('package export contract changes advance version', t => {
  const { root } = fixture(t);
  update(root, 1, { exports: { '.': './index.js' } });
  assert.equal(versionPackages(root), '2026.1.1');
});
test('explicit minor release keeps the declared version', t => {
  const { root } = fixture(t);
  for (const i of [0, 1]) update(root, i, { version: '2026.2.0' });
  assert.equal(versionPackages(root), '2026.2.0');
});
test('split versions are rejected before either manifest is written', t => {
  const { root } = fixture(t);
  update(root, 1, { version: '2026.1.1' });
  assert.throws(() => versionPackages(root), /must match/);
  assert.deepEqual(versions(root), ['2026.1.0', '2026.1.1']);
});

test('conditional export priority is part of package identity', t => {
  const { root, git } = fixture(t);
  update(root, 0, { exports: { '.': { node: './index.js', default: './index.js' } } });
  git('add', '.'); git('commit', '-m', 'fixture exports');
  update(root, 0, { exports: { '.': { default: './index.js', node: './index.js' } } });
  assert.equal(versionPackages(root), '2026.1.1');
});
