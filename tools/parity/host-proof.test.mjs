import assert from 'node:assert/strict';
import test from 'node:test';

import { HOSTS, validateHostProof } from './host-proof.mjs';

function goodHostProof(host = HOSTS[0]) {
  return {
    schema_version: 1,
    sourceRevision: 'ab'.repeat(20), sourceDirty: false, checkoutSha: 'ab'.repeat(20),
    expected: { runner: host.runner, platform: host.platform, arch: host.arch, rustHost: host.rustHost },
    runner: { os: host.runnerOs, arch: host.runnerArch },
    node: { platform: host.platform, arch: host.arch, version: 'v24', v8: '13.6' },
    rustc: { host: host.rustHost, verbose: `rustc 1.96\nhost: ${host.rustHost}\nLLVM version: 22` },
    unameMachine: host.platform === 'darwin' && host.arch === 'arm64' ? 'arm64' : host.nativeArch,
    buildTarget: null,
    native: Object.fromEntries([['kernel', 'remus-parity-native'], ['splitIo', 'remus-split-io-native']].map(([key, binary]) => [key, {
      path: `/tmp/${binary}`, sha256: '12'.repeat(32),
      runtime: { schemaVersion: 1, binary, os: host.nativeOs, arch: host.nativeArch, pointerWidth: 64 },
    }])),
  };
}

test('four native compiler and runtime tuples are recognized with actual identities', () => {
  assert.deepEqual(HOSTS.map(({ runner, platform, arch, rustHost }) => [runner, platform, arch, rustHost]), [
    ['ubuntu-24.04', 'linux', 'x64', 'x86_64-unknown-linux-gnu'],
    ['ubuntu-24.04-arm', 'linux', 'arm64', 'aarch64-unknown-linux-gnu'],
    ['macos-15-intel', 'darwin', 'x64', 'x86_64-apple-darwin'],
    ['macos-15', 'darwin', 'arm64', 'aarch64-apple-darwin'],
  ]);
  for (const host of HOSTS) {
    assert.deepEqual(validateHostProof(goodHostProof(host)), {
      platform: host.platform, arch: host.arch, rustHost: host.rustHost, expectedRunner: host.runner,
    });
  }
});

test('local execution records the actual host without manufacturing GitHub labels', () => {
  const proof = goodHostProof();
  proof.expected = null;
  proof.runner = { os: null, arch: null };
  assert.equal(validateHostProof(proof).expectedRunner, null);
});

test('matrix labels cannot qualify the wrong Node, compiler or native executable architecture', () => {
  for (const mutate of [
    (proof) => { proof.expected.arch = 'arm64'; },
    (proof) => { proof.expected.runner = 'ubuntu-24.04-arm'; },
    (proof) => { proof.node.arch = 'arm64'; },
    (proof) => { proof.rustc.host = 'aarch64-unknown-linux-gnu'; },
    (proof) => { proof.rustc.verbose = proof.rustc.verbose.replace('x86_64', 'aarch64'); },
    (proof) => { proof.runner.arch = 'ARM64'; },
    (proof) => { proof.unameMachine = 'aarch64'; },
    (proof) => { proof.native.kernel.runtime.arch = 'aarch64'; },
    (proof) => { proof.native.splitIo.runtime.os = 'macos'; },
    (proof) => { proof.native.splitIo.runtime.binary = 'remus-parity-native'; },
    (proof) => { proof.native.kernel.runtime.pointerWidth = 32; },
    (proof) => { proof.buildTarget = 'aarch64-unknown-linux-gnu'; },
  ]) {
    const proof = goodHostProof();
    mutate(proof);
    assert.throws(() => validateHostProof(proof));
  }
});

test('missing host evidence, stale checkout identity and malformed native hashes refuse', () => {
  for (const mutate of [
    (proof) => { proof.schema_version = 2; },
    (proof) => { delete proof.node; },
    (proof) => { delete proof.native.splitIo; },
    (proof) => { delete proof.node.v8; },
    (proof) => { proof.native.kernel.sha256 = 'wrong'; },
    (proof) => { proof.sourceDirty = 'false'; },
    (proof) => { proof.checkoutSha = '34'.repeat(20); },
    (proof) => { proof.rustc.verbose = 'rustc 1.96'; },
    (proof) => { proof.rustc.verbose += '\nhost: x86_64-unknown-linux-gnu'; },
    (proof) => { proof.expected = {}; },
  ]) {
    const proof = goodHostProof();
    mutate(proof);
    assert.throws(() => validateHostProof(proof));
  }
  assert.throws(() => validateHostProof(null), /missing/);
});
