import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve } from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

import {
  describeStaleness,
  requireObservations,
  validateInstalledEntry,
  validateProvenance,
} from './provenance.mjs';

function freshProvenance(overrides = {}) {
  return {
    mode: 'fresh-tarball',
    sourceRevision: 'ab'.repeat(20),
    sourceDirty: false,
    toolchain: { rustc: 'rustc 1.0', cargo: 'cargo 1.0', node: 'v22', wasmPack: 'wasm-pack 0.15' },
    target: 'nodejs',
    buildOptions: { target: 'nodejs', profile: 'release', features: '--no-default-features' },
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
  assert.throws(() => validateProvenance(freshProvenance({ wasmSha256: ['ab'.repeat(32)] })), /SHA-256/);
  assert.throws(() => validateProvenance(freshProvenance({ tarballSha256: ['ab'.repeat(32)] })), /tarball hash/);
  assert.ok(validateProvenance(freshProvenance()));
});

test('malformed source identity and unavailable toolchains cannot pass as proof', () => {
  for (const sourceRevision of ['abc123', ' ', 42, 'z'.repeat(40)]) {
    assert.throws(() => validateProvenance(freshProvenance({ sourceRevision })), /source revision/);
  }
  for (const sourceDirty of ['false', 0, {}, []]) {
    assert.throws(() => validateProvenance(freshProvenance({ sourceDirty })), /source dirty flag/);
  }
  for (const field of ['rustc', 'cargo', 'node', 'wasmPack']) {
    for (const unavailable of [null, '', ' ', 42]) {
      const toolchain = { ...freshProvenance().toolchain, [field]: unavailable };
      assert.throws(() => validateProvenance(freshProvenance({ toolchain })), /toolchain/);
    }
  }
});

test('fresh artifacts require declared compatible build configuration', () => {
  for (const buildOptions of [{}, [], 'release', { profile: 'release' }]) {
    assert.throws(() => validateProvenance(freshProvenance({ buildOptions })), /build/);
  }
  for (const field of ['target', 'profile', 'features']) {
    for (const missing of [null, '', ' ', 42]) {
      const buildOptions = { ...freshProvenance().buildOptions, [field]: missing };
      assert.throws(() => validateProvenance(freshProvenance({ buildOptions })), /build/);
    }
  }
  assert.throws(
    () => validateProvenance(freshProvenance({ target: 'web' })),
    /build target/,
  );
  assert.throws(
    () => validateProvenance(freshProvenance({ target: 'unknown', buildOptions: {
      ...freshProvenance().buildOptions, target: 'unknown',
    } })),
    /build target/,
  );
  // wasm-pack supports user-defined Cargo profiles through --profile.
  assert.ok(validateProvenance(freshProvenance({ buildOptions: {
    ...freshProvenance().buildOptions, profile: 'ci-test',
  } })));
});

test('native and unverified committed evidence retain their distinct metadata', () => {
  const native = {
    mode: 'native',
    sourceRevision: 'ab'.repeat(20),
    sourceDirty: true,
    toolchain: { rustc: 'rustc 1.0', cargo: 'cargo 1.0', node: 'v22', wasmPack: null },
    target: 'host',
    buildOptions: { profile: 'release', binary: 'remus-parity-native', facade: 'remus::Model' },
    packageVersion: null,
    wasmSha256: null,
    installedEntry: '/tmp/remus-parity-native',
  };
  assert.ok(validateProvenance(native));
  assert.throws(() => validateProvenance({ ...native, buildOptions: {} }), /build/);

  const committed = freshProvenance({
    mode: 'committed-package',
    target: 'wasm32-unknown-unknown',
    toolchain: native.toolchain,
    buildOptions: { origin: 'committed bytes; build options unknown' },
    tarballSha256: null,
    sourceRelationship: 'unverified — committed bytes; source relationship unknown',
  });
  assert.ok(validateProvenance(committed));
  assert.throws(() => validateProvenance({ ...committed, buildOptions: {} }), /build/);
  assert.throws(
    () => validateProvenance({ ...committed, sourceRelationship: 'built from current source' }),
    /unverified/,
  );
});

test('failed git status is refused instead of being recorded as a clean source tree', () => {
  const repoRoot = fileURLToPath(new URL('../../', import.meta.url));
  const invalidIndex = mkdtempSync(resolve(tmpdir(), 'remus-parity-invalid-index-'));
  const env = { ...process.env, GIT_INDEX_FILE: invalidIndex };
  try {
    // An index read failure leaves HEAD readable, isolating the status command
    // from the already-covered missing-revision branch.
    const head = spawnSync('git', ['rev-parse', 'HEAD'], { cwd: repoRoot, env });
    assert.ifError(head.error);
    assert.equal(head.status, 0);
    const status = spawnSync('git', ['status', '--porcelain=v1'], { cwd: repoRoot, env });
    assert.ifError(status.error);
    assert.notEqual(status.status, 0);
    const child = spawnSync(process.execPath, ['--input-type=module', '--eval', `
      import { collectSourceProvenance } from ${JSON.stringify(new URL('./provenance.mjs', import.meta.url).href)};
      try {
        collectSourceProvenance(${JSON.stringify(repoRoot)});
      } catch (error) {
        console.error(error.message);
        process.exitCode = 2;
      }
    `], { cwd: repoRoot, env, encoding: 'utf8' });
    assert.ifError(child.error);
    assert.equal(child.status, 2, `invalid index was accepted: ${child.stdout}\n${child.stderr}`);
    assert.match(child.stderr, /working tree status/);
  } finally {
    rmSync(invalidIndex, { recursive: true, force: true });
  }
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
