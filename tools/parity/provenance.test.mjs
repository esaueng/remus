import assert from 'node:assert/strict';
import test from 'node:test';

import {
  describeStaleness,
  requireObservations,
  validateInstalledEntry,
  validateProvenance,
} from './provenance.mjs';

function freshProvenance(overrides = {}) {
  return {
    mode: 'fresh-tarball',
    sourceRevision: 'abc123',
    sourceDirty: false,
    toolchain: { rustc: 'rustc 1.0', cargo: 'cargo 1.0', node: 'v22' },
    target: 'wasm32-unknown-unknown',
    buildOptions: { profile: 'release' },
    packageVersion: '2.130.35',
    wasmSha256: 'ab'.repeat(32),
    tarballSha256: 'cd'.repeat(32),
    installedEntry: '/tmp/consumer/node_modules/remus-wasm/remus_wasm.js',
    installedDir: '/tmp/consumer/node_modules/remus-wasm',
    ...overrides,
  };
}

test('wrong installed entry is refused', () => {
  assert.throws(
    () => validateInstalledEntry(
      '/tmp/other/remus_wasm.js',
      '/tmp/consumer/node_modules/remus-wasm',
    ),
    /resolved outside install/,
  );
  assert.throws(() => validateInstalledEntry(null, '/tmp/consumer'), /missing/);
  assert.ok(validateInstalledEntry(
    '/tmp/consumer/node_modules/remus-wasm/remus_wasm.js',
    '/tmp/consumer/node_modules/remus-wasm',
  ));
});

test('missing provenance fields are refused', () => {
  assert.throws(() => validateProvenance(null), /missing/);
  assert.throws(() => validateProvenance({}), /missing required field/);
  const missingTarball = freshProvenance({ tarballSha256: null });
  assert.throws(() => validateProvenance(missingTarball), /tarball hash/);
  const badWasm = freshProvenance({ wasmSha256: 'not-a-hash' });
  assert.throws(() => validateProvenance(badWasm), /SHA-256/);
  assert.ok(validateProvenance(freshProvenance()));
});

test('stale or mismatched committed artifacts are reported, never silently attributed', () => {
  const fresh = freshProvenance();
  const same = {
    ...fresh, mode: 'committed-package', tarballSha256: null, tarballPath: null,
  };
  const matching = describeStaleness(fresh, same);
  assert.equal(matching.stale, false);
  assert.equal(matching.versionMatch, true);
  assert.equal(matching.wasmMatch, true);

  const staleVersion = describeStaleness(fresh, { ...same, packageVersion: '2.130.34' });
  assert.equal(staleVersion.stale, true);
  assert.equal(staleVersion.versionMatch, false);
  assert.match(staleVersion.verdict, /stale\/mismatched/);

  const staleWasm = describeStaleness(fresh, { ...same, wasmSha256: 'ef'.repeat(32) });
  assert.equal(staleWasm.stale, true);
  assert.equal(staleWasm.wasmMatch, false);
});

test('missing case observations are refused', () => {
  assert.throws(() => requireObservations('contract/exact-fuse', null), /missing/);
  assert.throws(
    () => requireObservations('contract/exact-fuse', { native: null }),
    /missing/,
  );
  assert.throws(
    () => requireObservations('contract/exact-fuse', { native: { volume: 1 } }),
    /no outcome/,
  );
  assert.ok(requireObservations('contract/exact-fuse', {
    native: { outcome: 'success' },
    fresh: { outcome: 'success' },
  }));
});
