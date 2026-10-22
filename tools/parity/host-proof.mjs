import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { machine } from 'node:os';
import { resolve, win32 } from 'node:path';
import { fileURLToPath } from 'node:url';

import { collectSourceProvenance, hashFile } from './provenance.mjs';

// These are native hosts, not cross-compilation targets. Labels are checked
// against the compiler, Node process and both actually executed Rust binaries.
export const HOSTS = Object.freeze([
  { runner: 'ubuntu-24.04', platform: 'linux', arch: 'x64', rustHost: 'x86_64-unknown-linux-gnu', runnerOs: 'Linux', runnerArch: 'X64', nativeOs: 'linux', nativeArch: 'x86_64' },
  { runner: 'ubuntu-24.04-arm', platform: 'linux', arch: 'arm64', rustHost: 'aarch64-unknown-linux-gnu', runnerOs: 'Linux', runnerArch: 'ARM64', nativeOs: 'linux', nativeArch: 'aarch64' },
  { runner: 'macos-15-intel', platform: 'darwin', arch: 'x64', rustHost: 'x86_64-apple-darwin', runnerOs: 'macOS', runnerArch: 'X64', nativeOs: 'macos', nativeArch: 'x86_64' },
  { runner: 'macos-15', platform: 'darwin', arch: 'arm64', rustHost: 'aarch64-apple-darwin', runnerOs: 'macOS', runnerArch: 'ARM64', nativeOs: 'macos', nativeArch: 'aarch64' },
  { runner: 'windows-2025', platform: 'win32', arch: 'x64', rustHost: 'x86_64-pc-windows-msvc', runnerOs: 'Windows', runnerArch: 'X64', nativeOs: 'windows', nativeArch: 'x86_64' },
]);

function nonemptyString(value) {
  return typeof value === 'string' && value.trim().length > 0;
}

function requireWindowsExecutable(path) {
  assert.ok(nonemptyString(path) && win32.isAbsolute(path) && win32.parse(path).root.length > 1
    && /\.exe$/i.test(path), 'Windows executable must have an absolute drive/UNC path and explicit .exe suffix');
}

/** Reject missing/mixed host evidence while leaving package-origin claims separate. */
export function validateHostProof(proof) {
  assert.ok(proof && typeof proof === 'object', 'host proof is missing');
  assert.equal(proof.schema_version, 1, 'unsupported host-proof schema');
  assert.match(proof.sourceRevision ?? '', /^(?:[0-9a-f]{40}|[0-9a-f]{64})$/, 'host source revision is invalid');
  assert.equal(typeof proof.sourceDirty, 'boolean', 'host source dirty flag is invalid');
  assert.equal(proof.checkoutSha, proof.sourceRevision, 'host checkout SHA differs from source');
  const host = HOSTS.find(({ platform, arch }) => platform === proof.node?.platform && arch === proof.node?.arch);
  assert.ok(host, 'unsupported Node host platform/architecture');
  assert.ok(nonemptyString(proof.node.version) && nonemptyString(proof.node.v8), 'Node runtime versions are missing');
  assert.ok(nonemptyString(proof.rustc?.verbose), 'verbose compiler identity is missing');
  const matches = [...proof.rustc.verbose.matchAll(/^host: (\S+)$/gm)];
  assert.equal(matches.length, 1, 'compiler must report exactly one host triple');
  assert.equal(proof.rustc.host, matches[0][1], 'compiler host disagrees with verbose identity');
  assert.equal(proof.rustc.host, host.rustHost, 'Node and Rust compiler architectures differ');
  const expectedMachine = host.platform === 'darwin' && host.arch === 'arm64' ? 'arm64' : host.nativeArch;
  if (host.platform === 'win32') {
    assert.equal(proof.osMachineSource, 'node:os.machine', 'Windows native OS machine probe is missing');
    assert.equal(proof.osMachine, expectedMachine, 'Windows native OS machine differs from compiler/runtime');
    assert.equal(proof.unameMachine, null, 'Windows proof cannot substitute a compatibility-shell uname');
  } else {
    assert.equal(proof.unameMachine, expectedMachine, 'OS machine identity differs from compiler/runtime');
    // Older POSIX reports retain their original uname evidence and remain replayable.
    if (proof.osMachine !== undefined) assert.equal(proof.osMachine, expectedMachine, 'Node OS machine differs from uname');
  }
  assert.ok(proof.buildTarget === null, 'cross-target builds cannot qualify a native host');
  if (proof.expected !== null) {
    assert.deepEqual(proof.expected, {
      runner: host.runner, platform: host.platform, arch: host.arch, rustHost: host.rustHost,
    }, 'declared matrix host differs from actual compiler/runtime');
  }
  if (proof.runner?.os !== null || proof.runner?.arch !== null) {
    assert.equal(proof.runner?.os, host.runnerOs, 'GitHub runner OS differs from actual runtime');
    assert.equal(proof.runner?.arch, host.runnerArch, 'GitHub runner architecture differs from actual runtime');
  }
  for (const [key, binary] of [['kernel', 'remus-parity-native'], ['splitIo', 'remus-split-io-native']]) {
    const runner = proof.native?.[key];
    assert.ok(nonemptyString(runner?.path), `${key}: native executable path is missing`);
    if (host.platform === 'win32') requireWindowsExecutable(runner.path);
    assert.match(runner?.sha256 ?? '', /^[0-9a-f]{64}$/, `${key}: native executable hash is missing`);
    assert.deepEqual(runner.runtime, {
      schemaVersion: 1, binary, os: host.nativeOs, arch: host.nativeArch, pointerWidth: 64,
    }, `${key}: actual native executable identity differs from compiler/runtime`);
  }
  return { platform: host.platform, arch: host.arch, rustHost: host.rustHost, expectedRunner: proof.expected?.runner ?? null };
}

/** Collect facts from the current process/compiler and the exact runner files. */
export function collectHostProof(nativeRunner, splitRunner, env = process.env) {
  const repoRoot = fileURLToPath(new URL('../../', import.meta.url));
  const source = collectSourceProvenance(repoRoot);
  const run = (command, args) => execFileSync(command, args, {
    encoding: 'utf8', timeout: 15_000, maxBuffer: 1024 * 1024,
  }).trim();
  const verbose = run('rustc', ['-vV']);
  const host = [...verbose.matchAll(/^host: (\S+)$/gm)][0]?.[1];
  const expectedValues = [env.O15_EXPECT_RUNNER, env.O15_EXPECT_PLATFORM, env.O15_EXPECT_ARCH, env.O15_EXPECT_RUST_HOST];
  const expected = expectedValues.every((value) => value === undefined) ? null : {
    runner: expectedValues[0], platform: expectedValues[1], arch: expectedValues[2], rustHost: expectedValues[3],
  };
  const native = {};
  for (const [key, runner] of [['kernel', nativeRunner], ['splitIo', splitRunner]]) {
    const path = resolve(runner);
    if (process.platform === 'win32') requireWindowsExecutable(path);
    const sha256 = hashFile(path);
    const runtime = JSON.parse(run(path, ['--runtime-identity']));
    assert.equal(hashFile(path), sha256, `${key}: executable changed while collecting identity`);
    native[key] = { path, sha256, runtime };
  }
  const proof = {
    schema_version: 1,
    sourceRevision: source.sourceRevision,
    sourceDirty: source.sourceDirty,
    checkoutSha: env.GITHUB_SHA ?? source.sourceRevision,
    expected,
    runner: { os: env.RUNNER_OS ?? null, arch: env.RUNNER_ARCH ?? null },
    node: { platform: process.platform, arch: process.arch, version: process.version, v8: process.versions.v8 },
    rustc: { host, verbose },
    unameMachine: process.platform === 'win32' ? null : run('uname', ['-m']),
    osMachine: machine(),
    osMachineSource: 'node:os.machine',
    buildTarget: env.CARGO_BUILD_TARGET ?? null,
    native,
  };
  validateHostProof(proof);
  return proof;
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  if (process.argv.length !== 4) {
    console.error('usage: host-proof.mjs NATIVE_RUNNER SPLIT_IO_RUNNER');
    process.exitCode = 2;
  } else {
    try {
      console.log(JSON.stringify(collectHostProof(...process.argv.slice(2)), null, 2));
    } catch (error) {
      console.error(`O1.5 host proof rejected: ${error.message}`);
      process.exitCode = 1;
    }
  }
}
