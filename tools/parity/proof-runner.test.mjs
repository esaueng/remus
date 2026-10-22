import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { existsSync, mkdirSync, mkdtempSync, readFileSync, readdirSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { resolve } from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';

import { nativeExecutables, runProof } from '../../scripts/test-o15-proof.mjs';

function fixture(t, failure = '') {
  const root = mkdtempSync(resolve(tmpdir(), 'remus proof ü '));
  t.after(() => rmSync(root, { recursive: true, force: true }));
  const calls = [];
  const env = { ...process.env, O15_PROOF_OUT: resolve(root, 'reports with spaces'), RUST_MIN_STACK: '33554432' };
  const execute = (command, args, options) => {
    calls.push({ command, args, options });
    assert.equal(options.shell, false, 'paths and package arguments must never pass through a shell');
    const output = (value) => {
      if (Number.isInteger(options.stdio?.[1])) writeFileSync(options.stdio[1], JSON.stringify(value));
    };
    if ((command === 'cargo' || command.endsWith('cargo.exe')) && args[0] === 'metadata') {
      return { status: 0, stdout: JSON.stringify(failure === 'metadata' ? {} : { target_directory: resolve(root, 'build with spaces') }) };
    }
    if ((command === 'cargo' || command.endsWith('cargo.exe')) && args[0] === 'build' && failure === 'real-child') {
      return spawnSync(process.execPath, ['-e', 'process.exit(7)'], options);
    }
    if ((command === 'cargo' || command.endsWith('cargo.exe')) && args[0] === 'build' && failure === 'launch') {
      return spawnSync(resolve(root, 'missing executable'), [], options);
    }
    if (args[0] === 'tools/parity/host-proof.mjs') {
      output({ schema_version: 1, sourceRevision: 'ab'.repeat(20), host: 'orchestration fixture only' });
      return { status: failure === 'host' ? 5 : 0 };
    }
    if (args[0] === 'tools/parity/checkout-proof.mjs') {
      output(failure === 'malformed-checkout' ? {} : {
        schemaVersion: 1, mode: 'committed-checkout', sourceRevision: (failure === 'mixed-checkout' ? 'ef' : 'ab').repeat(20),
        sourceRelationship: 'unverified original build source; raw distribution bytes equal Git index',
        fileCount: 2,
        roots: ['crates/wasm/pkg', 'crates/wasm-io/pkg'].map((directory) => ({ directory, trackedCount: 1, actualCount: 1 })),
        files: ['crates/wasm/pkg', 'crates/wasm-io/pkg'].map((directory) => ({ path: `${directory}/package.json`, indexBlob: 'cd'.repeat(20), rawBlob: 'cd'.repeat(20), sha256: '12'.repeat(32) })),
      });
      return { status: failure === 'checkout' ? 6 : 0 };
    }
    if (command === 'wasm-pack' || command.endsWith('wasm-pack.exe')) {
      mkdirSync(args[args.indexOf('--out-dir') + 1], { recursive: true });
      return { status: failure === 'build' ? 3 : 0 };
    }
    if (args[0]?.endsWith('-matrix.mjs')) {
      const failed = failure === args[0];
      output({ passed: !failed });
      return { status: failed ? 1 : 0 };
    }
    if (args[0] === 'tools/parity/proof-verdict.mjs') {
      assert.equal(args.length, 4);
      assert.ok(args.slice(1).every(existsSync), 'both reports and actual host receipt must reach the verdict');
      output({ passed: failure !== 'verdict' });
      return { status: failure === 'verdict' ? 4 : 0 };
    }
    return { status: 0 };
  };
  return {
    root, calls, env,
    run: (overrides) => runProof({ repoRoot: root, env, temporaryRoot: root, execute, arch: 'x64', ...overrides }),
    prepareArm: ({ env: before, temporary, receipt }) => {
      if (failure === 'arm-preflight') { writeFileSync(receipt, JSON.stringify({ passed: false, state: 'failed' })); throw new Error('refused ARM preflight'); }
      const child = { ...before, RUSTC: 'C:\\actual native ü\\rustc.exe', RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', RUSTUP_TOOLCHAIN: '1.96.0-aarch64-pc-windows-msvc', CARGO_BUILD_JOBS: '1', CARGO_HOME: resolve(temporary, 'cargo-home'), PATH: 'C:\\actual native ü;C:\\x64 helpers', O15_WINDOWS_ARM_BUILD: receipt };
      delete child.CARGO_BUILD_TARGET;
      const proof = { passed: true, state: 'prepared', cargo: { path: 'C:\\actual native ü\\cargo.exe' }, helpers: { wasmPack: { path: 'C:\\x64 helpers\\wasm-pack.exe' } }, environment: { RUSTC: child.RUSTC, PATH: child.PATH, CARGO_HOME: child.CARGO_HOME }, buildCommands: [] };
      return { env: child, proof };
    },
    finishArm: ({ proof, env: child, receipt }) => {
      assert.equal(child.RUSTC, proof.environment.RUSTC); assert.equal(proof.buildCommands.length, 4);
      proof.state = failure === 'arm-finish' ? 'failed' : 'completed';
      writeFileSync(receipt, JSON.stringify(proof));
      if (failure === 'arm-finish') throw new Error('changed ARM helper after build');
    },
    status: (result) => readFileSync(resolve(result.proofDir, 'status.tsv'), 'utf8'),
    freshDirectories: () => readdirSync(root).filter((name) => name.startsWith('remus-o15-proof.')),
  };
}

test('both slices use the exact qualified build arguments and shared fresh pair', (t) => {
  const f = fixture(t);
  const result = f.run();
  assert.equal(result.status, 0);
  assert.equal(f.status(result), 'contract\t0\nsplit-io\t0\n');
  const builds = f.calls.filter(({ command }) => command === 'wasm-pack');
  assert.equal(builds.length, 2);
  assert.deepEqual(builds.map(({ args }) => args.filter((arg) => !arg.startsWith(f.root))), [
    ['build', 'crates/wasm', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', '--', '--locked', '--no-default-features'],
    ['build', 'crates/wasm-io', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', '--', '--locked'],
  ]);
  const contract = f.calls.find(({ args }) => args[0] === 'tools/parity/contract-matrix.mjs');
  const split = f.calls.find(({ args }) => args[0] === 'tools/parity/split-io-matrix.mjs');
  assert.equal(contract.args[2], builds[0].args[builds[0].args.indexOf('--out-dir') + 1]);
  assert.equal(split.args[2], contract.args[2]);
  assert.equal(split.args[3], builds[1].args[builds[1].args.indexOf('--out-dir') + 1]);
  assert.deepEqual(split.args.slice(-2), ['crates/wasm/pkg', 'crates/wasm-io/pkg']);
  assert.equal(contract.args.at(-1), 'crates/wasm/pkg');
  const config = readFileSync(resolve(result.proofDir, 'build-config.txt'), 'utf8');
  const argv = config.split('\n').filter((line) => line.startsWith('[')).map((line) => JSON.parse(line));
  assert.deepEqual(argv, f.calls.filter(({ command, args }) => command === 'wasm-pack' || command === 'cargo' && ['test', 'build'].includes(args[0]))
    .map(({ command, args }) => [command, ...args]));
  assert.equal(argv.length, 4);
  assert.match(config, /RUST_MIN_STACK="33554432"/);
  assert.deepEqual(f.freshDirectories(), []);
});

test('contract failure retains both independent reports and fails the proof', (t) => {
  const f = fixture(t, 'tools/parity/contract-matrix.mjs');
  const result = f.run();
  assert.equal(result.status, 1);
  assert.equal(f.status(result), 'contract\t1\nsplit-io\t0\n');
  assert.equal(JSON.parse(readFileSync(resolve(result.proofDir, 'o15-contract.json'))).passed, false);
  assert.equal(JSON.parse(readFileSync(resolve(result.proofDir, 'o15-split-io.json'))).passed, true);
  assert.ok(f.calls.some(({ args }) => args[0] === 'tools/parity/proof-verdict.mjs'));
});

test('split failure remains red and does not reuse contract success', (t) => {
  const f = fixture(t, 'tools/parity/split-io-matrix.mjs');
  const result = f.run();
  assert.equal(result.status, 1);
  assert.equal(f.status(result), 'contract\t0\nsplit-io\t1\n');
});

test('successful harness process exits cannot bypass a refused report verdict', (t) => {
  const f = fixture(t, 'verdict');
  const result = f.run();
  assert.equal(result.status, 1);
  assert.equal(f.status(result), 'contract\t0\nsplit-io\t0\n');
});

test('wrong actual host refuses before package builds or geometric observations', (t) => {
  const f = fixture(t, 'host');
  const result = f.run();
  assert.equal(result.status, 5);
  assert.equal(f.status(result), 'contract\tnot_run\nsplit-io\tnot_run\n');
  assert.ok(!f.calls.some(({ command }) => command === 'wasm-pack'));
  assert.ok(!existsSync(resolve(result.proofDir, 'o15-contract.json')));
});

test('build refusal preserves host evidence, marks unexecuted slices and cleans temporary bytes', (t) => {
  const f = fixture(t, 'build');
  const result = f.run();
  assert.equal(result.status, 3);
  assert.equal(f.status(result), 'contract\tnot_run\nsplit-io\tnot_run\n');
  assert.deepEqual(readdirSync(result.proofDir).filter((name) => name.endsWith('.json')).sort(), ['committed-checkout.json', 'host.json']);
  assert.deepEqual(f.freshDirectories(), []);
});

test('refused, malformed or mixed-source checkout bytes stop before fresh packages', (t) => {
  for (const [failure, code] of [['checkout', 6], ['malformed-checkout', 1], ['mixed-checkout', 1]]) {
    const f = fixture(t, failure);
    const result = f.run();
    assert.equal(result.status, code);
    assert.equal(f.status(result), 'contract\tnot_run\nsplit-io\tnot_run\n');
    assert.ok(!f.calls.some(({ command }) => command === 'wasm-pack'));
  }
});

test('repeated invocations cannot reuse previous passing reports', (t) => {
  const f = fixture(t);
  const first = f.run();
  const second = fixture(t, 'build').run({ env: f.env });
  assert.equal(first.status, 0);
  assert.equal(second.status, 3);
  assert.notEqual(first.proofDir, second.proofDir);
  assert.ok(existsSync(resolve(first.proofDir, 'verdict.json')));
  assert.ok(!existsSync(resolve(second.proofDir, 'o15-contract.json')));
});

test('a real failing subprocess and a missing executable both refuse before any geometry', (t) => {
  for (const [failure, code] of [['real-child', 7], ['launch', 1], ['metadata', 1]]) {
    const f = fixture(t, failure);
    const result = f.run();
    assert.equal(result.status, code);
    assert.equal(f.status(result), 'contract\tnot_run\nsplit-io\tnot_run\n');
    assert.ok(!f.calls.some(({ args }) => args[0]?.endsWith('-matrix.mjs')));
  }
});

test('Windows executable names include .exe and remain literal native paths', (t) => {
  const f = fixture(t);
  const target = resolve(f.root, 'build with spaces');
  assert.deepEqual(nativeExecutables(target, 'win32'), [
    resolve(target, 'release/remus-parity-native.exe'), resolve(target, 'release/remus-split-io-native.exe'),
  ]);
  assert.ok(nativeExecutables(target, 'linux').every((path) => !path.endsWith('.exe')));
  assert.equal(f.run({ platform: 'win32' }).status, 0);
  const host = f.calls.find(({ args }) => args[0] === 'tools/parity/host-proof.mjs');
  assert.deepEqual(host.args.slice(1), nativeExecutables(target, 'win32'));
});

test('CLI rejects unexpected arguments before building or creating reusable evidence', (t) => {
  const f = fixture(t);
  const script = fileURLToPath(new URL('../../scripts/test-o15-proof.mjs', import.meta.url));
  const child = spawnSync(process.execPath, [script, 'unexpected'], { encoding: 'utf8', env: f.env });
  assert.ifError(child.error);
  assert.equal(child.status, 2);
  assert.match(child.stderr, /usage/);
  assert.ok(!existsSync(f.env.O15_PROOF_OUT));
});

test('ARM geometry builds and scorers receive the verified native toolchain environment', (t) => {
  const f = fixture(t, 'tools/parity/contract-matrix.mjs');
  const result = f.run({ platform: 'win32', arch: 'arm64', prepareArm: f.prepareArm, finishArm: f.finishArm });
  assert.equal(result.status, 1); assert.equal(f.status(result), 'contract\t1\nsplit-io\t0\n');
  assert.ok(f.calls.every(({ options }) => options.env.RUSTC === 'C:\\actual native ü\\rustc.exe'));
  assert.ok(f.calls.every(({ options }) => options.env.RUSTC_WRAPPER === '' && options.env.RUSTC_WORKSPACE_WRAPPER === '' && !Object.hasOwn(options.env, 'CARGO_BUILD_TARGET')));
  assert.ok(f.calls.filter(({ command, args }) => args[0] === 'test' || args[0] === 'metadata' || args[0] === 'build' && !args.includes('nodejs')).every(({ command }) => command === 'C:\\actual native ü\\cargo.exe'));
  assert.equal(f.calls.filter(({ command }) => command === 'C:\\x64 helpers\\wasm-pack.exe').length, 2);
  const hosts = f.calls.filter(({ args }) => args[0] === 'tools/parity/host-proof.mjs'); assert.equal(hosts.length, 2);
  const builds = f.calls.filter(({ command }) => command.endsWith('wasm-pack.exe'));
  assert.ok(f.calls.indexOf(hosts[0]) < f.calls.indexOf(builds[0]));
  assert.ok(f.calls.indexOf(hosts[1]) > f.calls.indexOf(builds[1]));
  const proof = JSON.parse(readFileSync(resolve(result.proofDir, 'windows-arm-build.json')));
  assert.equal(proof.state, 'completed'); assert.equal(proof.buildCommands.length, 4);
  assert.ok(proof.buildCommands.every((row) => row.status === 0 && row.environment.RUSTC === 'C:\\actual native ü\\rustc.exe'));
  assert.deepEqual(f.freshDirectories(), []);
});

test('ARM preflight and changed-helper refusal retain evidence and never run geometric slices', (t) => {
  for (const failure of ['arm-preflight', 'arm-finish']) {
    const f = fixture(t, failure), result = f.run({ platform: 'win32', arch: 'arm64', prepareArm: f.prepareArm, finishArm: f.finishArm });
    assert.equal(result.status, 1); assert.equal(f.status(result), 'contract\tnot_run\nsplit-io\tnot_run\n');
    assert.ok(!f.calls.some(({ args }) => args[0]?.endsWith('-matrix.mjs')));
    if (failure === 'arm-preflight') assert.equal(f.calls.length, 0);
    assert.equal(JSON.parse(readFileSync(resolve(result.proofDir, 'windows-arm-build.json'))).state, 'failed');
    assert.deepEqual(f.freshDirectories(), []);
  }
});
