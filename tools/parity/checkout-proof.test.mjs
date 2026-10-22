import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve } from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

import { collectCheckoutProof, validateCheckoutProof } from './checkout-proof.mjs';

test('raw CRLF mutations refuse even when Git normalization reports clean package paths', (t) => {
  const root = mkdtempSync(resolve(tmpdir(), 'remus checkout ü '));
  t.after(() => rmSync(root, { recursive: true, force: true }));
  const git = (...args) => execFileSync('git', args, { cwd: root, encoding: 'utf8' }).trim();
  git('init', '--quiet');
  const source = fileURLToPath(new URL('../../', import.meta.url));
  const sourceGit = (...args) => execFileSync('git', args, { cwd: source, encoding: 'utf8' }).trim();
  const objects = resolve(sourceGit('rev-parse', '--path-format=absolute', '--git-common-dir'), 'objects');
  mkdirSync(resolve(root, '.git/objects/info'), { recursive: true });
  writeFileSync(resolve(root, '.git/objects/info/alternates'), `${objects.replaceAll('\\', '/')}\n`);
  // Reuse an existing public source commit as data; no synthetic commit or hook bypass.
  git('update-ref', 'HEAD', sourceGit('rev-parse', 'HEAD'));
  git('read-tree', 'HEAD');
  const roots = ['crates/wasm/pkg', 'crates/wasm-io/pkg'];
  const files = git('ls-files', '-z', '--', ...roots).split('\0').filter(Boolean);
  git('-c', 'core.autocrlf=false', 'checkout-index', '--force', '--', ...files);
  const proof = collectCheckoutProof(root);
  assert.equal(validateCheckoutProof(proof), files.length);
  assert.equal(proof.roots.length, 2);
  git('config', 'core.autocrlf', 'true');
  assert.equal(git('status', '--porcelain=v1', '--', ...roots), '');
  const packageJson = resolve(root, 'crates/wasm/pkg/package.json');
  const raw = execFileSync('git', ['show', 'HEAD:crates/wasm/pkg/package.json'], { cwd: root, encoding: 'utf8' });
  writeFileSync(packageJson, raw.replaceAll('\n', '\r\n'));
  // Refresh only the stat cache through Git's normal conversion path. The
  // committed/index blob must remain unchanged while raw CRLF bytes remain.
  git('add', '--renormalize', '--', 'crates/wasm/pkg/package.json');
  assert.equal(git('rev-parse', ':crates/wasm/pkg/package.json'), proof.files.find((file) => file.path === 'crates/wasm/pkg/package.json').indexBlob);
  assert.equal(git('status', '--porcelain=v1', '--', ...roots), '', 'normalized Git status hides this raw byte mutation');
  assert.throws(() => collectCheckoutProof(root), /raw committed bytes differ/);
  writeFileSync(packageJson, raw);
  writeFileSync(resolve(root, 'crates/wasm/pkg/foreign-extra.js'), 'export {};');
  assert.throws(() => collectCheckoutProof(root), /file set differs/);
  for (const mutate of [
    (changed) => { changed.files[0].rawBlob = '12'.repeat(20); },
    (changed) => { changed.roots.pop(); },
    (changed) => { changed.files.pop(); },
    (changed) => { changed.files.push(changed.files[0]); changed.fileCount += 1; },
  ]) {
    const changed = structuredClone(proof);
    mutate(changed);
    assert.throws(() => validateCheckoutProof(changed));
  }
});
