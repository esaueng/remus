import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readdirSync } from 'node:fs';
import { relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

import { hashFile } from './provenance.mjs';

const roots = ['crates/wasm/pkg', 'crates/wasm-io/pkg'];

/** Raw distribution identity is separate from its still-unverified original build source. */
export function validateCheckoutProof(proof) {
  assert.equal(proof?.schemaVersion, 1, 'missing committed checkout proof schema');
  assert.equal(proof.mode, 'committed-checkout', 'wrong committed checkout evidence mode');
  assert.match(proof.sourceRevision ?? '', /^(?:[0-9a-f]{40}|[0-9a-f]{64})$/, 'checkout revision is invalid');
  assert.equal(proof.sourceRelationship, 'unverified original build source; raw distribution bytes equal Git index');
  assert.ok(Array.isArray(proof.files) && proof.files.length > 0, 'checkout file census is missing');
  assert.equal(proof.fileCount, proof.files.length, 'checkout file denominator differs');
  assert.equal(new Set(proof.files.map((file) => file.path)).size, proof.files.length, 'duplicate checkout file');
  for (const file of proof.files) {
    assert.ok(roots.some((root) => file.path.startsWith(`${root}/`)), 'foreign checkout file');
    assert.match(file.indexBlob ?? '', /^(?:[0-9a-f]{40}|[0-9a-f]{64})$/, 'index blob is invalid');
    assert.equal(file.rawBlob, file.indexBlob, `raw committed bytes differ from index: ${file.path}`);
    assert.match(file.sha256 ?? '', /^[0-9a-f]{64}$/, 'raw checkout SHA-256 is missing');
  }
  assert.deepEqual(proof.roots?.map((entry) => entry.directory), roots, 'both committed package roots are required');
  for (const entry of proof.roots) {
    const count = proof.files.filter((file) => file.path.startsWith(`${entry.directory}/`)).length;
    assert.ok(count > 0 && entry.trackedCount === count && entry.actualCount === count, 'tracked and actual package file sets differ');
  }
  return proof.fileCount;
}

export function collectCheckoutProof(repoRoot) {
  const git = (...args) => execFileSync('git', args, { cwd: repoRoot, encoding: 'utf8', maxBuffer: 4 * 1024 * 1024 });
  const entries = git('ls-files', '--stage', '-z', '--', ...roots).split('\0').filter(Boolean);
  const files = entries.map((entry) => {
    const match = /^(100644|100755) ([0-9a-f]{40}|[0-9a-f]{64}) 0\t(.+)$/.exec(entry);
    assert.ok(match, 'committed package index contains a non-regular or unmerged entry');
    const path = match[3];
    const rawBlob = git('hash-object', '--no-filters', '--', path).trim();
    return { path, indexBlob: match[2], rawBlob, sha256: hashFile(resolve(repoRoot, path)) };
  });
  const rootCensus = roots.map((directory) => {
    const actual = [];
    function visit(path) {
      for (const entry of readdirSync(path, { withFileTypes: true })) {
        const child = resolve(path, entry.name);
        if (entry.isDirectory()) visit(child);
        else {
          assert.ok(entry.isFile(), 'committed package checkout contains a non-regular file');
          actual.push(relative(repoRoot, child).replaceAll('\\', '/'));
        }
      }
    }
    visit(resolve(repoRoot, directory));
    const tracked = files.filter((file) => file.path.startsWith(`${directory}/`)).map((file) => file.path);
    assert.deepEqual(actual.sort(), tracked.sort(), `actual committed package file set differs: ${directory}`);
    return { directory, trackedCount: tracked.length, actualCount: actual.length };
  });
  const proof = {
    schemaVersion: 1, mode: 'committed-checkout', sourceRevision: git('rev-parse', 'HEAD').trim(),
    sourceRelationship: 'unverified original build source; raw distribution bytes equal Git index',
    fileCount: files.length, roots: rootCensus, files,
  };
  validateCheckoutProof(proof);
  return proof;
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    assert.equal(process.argv.length, 2, 'usage: checkout-proof.mjs');
    console.log(JSON.stringify(collectCheckoutProof(fileURLToPath(new URL('../../', import.meta.url))), null, 2));
  } catch (error) { console.error(`O1.5 committed checkout rejected: ${error.message}`); process.exitCode = 1; }
}
