import assert from 'node:assert/strict';
import { mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import test from 'node:test';

import { join, win32 } from 'node:path';

import { HOSTS, WINDOWS_ARM_INSTALLATION, WINDOWS_ARM_TOOLCHAIN, peIdentity, prepareWindowsArmBuild, finishWindowsArmBuild, validateHostProof, validateWindowsArmBuild } from './host-proof.mjs';

function goodHostProof(host = HOSTS[0]) {
  const proof = {
    schema_version: 1,
    sourceRevision: 'ab'.repeat(20), sourceDirty: false, checkoutSha: 'ab'.repeat(20),
    expected: { runner: host.runner, platform: host.platform, arch: host.arch, rustHost: host.rustHost },
    runner: { os: host.runnerOs, arch: host.runnerArch },
    node: { platform: host.platform, arch: host.arch, version: 'v24', v8: '13.6' },
    rustc: { host: host.rustHost, verbose: `rustc 1.96\nhost: ${host.rustHost}\nLLVM version: 22` },
    unameMachine: host.platform === 'win32' ? null : host.platform === 'darwin' && host.arch === 'arm64' ? 'arm64' : host.nativeArch,
    ...(host.platform === 'win32' ? { osMachine: 'x86_64', osMachineSource: 'node:os.machine' } : {}),
    buildTarget: null,
    native: Object.fromEntries([['kernel', 'remus-parity-native'], ['splitIo', 'remus-split-io-native']].map(([key, binary]) => [key, {
      path: host.platform === 'win32' ? `C:\\proof with spaces\\${binary}.exe` : `/tmp/${binary}`, sha256: '12'.repeat(32),
      runtime: { schemaVersion: 1, binary, os: host.nativeOs, arch: host.nativeArch, pointerWidth: 64 },
    }])),
  };
  if (host.platform === 'win32' && host.arch === 'arm64') {
    const build = armBuildFixture(); proof.windowsArmBuild = build;
    proof.node = structuredClone(build.node); proof.rustc = structuredClone(build.rustc); proof.osMachine = 'arm64';
    for (const runner of Object.values(proof.native)) Object.assign(runner, { bytes: 512, pe: { format: 'PE32+', machine: 'arm64', machineCode: 0xaa64 } });
  }
  return proof;
}

test('six native compiler and runtime tuples are recognized with actual identities', () => {
  assert.deepEqual(HOSTS.map(({ runner, platform, arch, rustHost }) => [runner, platform, arch, rustHost]), [
    ['ubuntu-24.04', 'linux', 'x64', 'x86_64-unknown-linux-gnu'],
    ['ubuntu-24.04-arm', 'linux', 'arm64', 'aarch64-unknown-linux-gnu'],
    ['macos-15-intel', 'darwin', 'x64', 'x86_64-apple-darwin'],
    ['macos-15', 'darwin', 'arm64', 'aarch64-apple-darwin'],
    ['windows-2025', 'win32', 'x64', 'x86_64-pc-windows-msvc'],
    ['windows-11-arm', 'win32', 'arm64', 'aarch64-pc-windows-msvc'],
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

test('Windows x64 requires actual native OS-machine evidence and both executable identities', () => {
  const host = {
    runner: 'windows-2025', platform: 'win32', arch: 'x64', rustHost: 'x86_64-pc-windows-msvc',
    runnerOs: 'Windows', runnerArch: 'X64', nativeOs: 'windows', nativeArch: 'x86_64',
  };
  const proof = goodHostProof(host);
  proof.unameMachine = null;
  proof.osMachine = 'x86_64';
  proof.osMachineSource = 'node:os.machine';
  for (const runner of Object.values(proof.native)) runner.path = `C:\\proof with spaces\\${runner.runtime.binary}.exe`;
  assert.equal(validateHostProof(proof).expectedRunner, 'windows-2025');
  for (const mutate of [
    (changed) => { delete changed.osMachine; },
    (changed) => { delete changed.osMachineSource; },
    (changed) => { changed.osMachineSource = 'runner-label'; },
    (changed) => { changed.osMachine = 'arm64'; },
    (changed) => { changed.unameMachine = 'x86_64'; },
    (changed) => { changed.rustc.host = 'x86_64-pc-windows-gnu'; },
    (changed) => { changed.native.kernel.runtime.os = 'linux'; },
    (changed) => { changed.native.splitIo.runtime.arch = 'aarch64'; },
    (changed) => { changed.native.splitIo.sha256 = ''; },
  ]) {
    const changed = structuredClone(proof);
    mutate(changed);
    assert.throws(() => validateHostProof(changed));
  }
  for (const path of ['/tmp/runner.exe', 'runner.exe', 'C:\\proof\\runner', 'C:\\proof\\runner.cmd', 'C:\\proof\\runner.bat']) {
    const changed = structuredClone(proof);
    changed.native.kernel.path = path;
    assert.throws(() => validateHostProof(changed), /explicit \.exe/);
  }
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

test('Windows ARM is registered as a separate native geometry host', () => {
  assert.ok(HOSTS.some((host) => host.runner === 'windows-11-arm'
    && host.platform === 'win32' && host.arch === 'arm64'
    && host.rustHost === 'aarch64-pc-windows-msvc'));
});

function armBuildFixture() {
  const pe = (machine = 'arm64') => ({ format: 'PE32+', machine, machineCode: machine === 'arm64' ? 0xaa64 : 0x8664 });
  const exe = (path) => ({ path, bytes: 512, sha256: '12'.repeat(32), pe: pe() });
  const temporary = 'C:\\Temp\\proof ü';
  const node = { ...exe('C:\\native node\\node.exe'), platform: 'win32', arch: 'arm64', version: 'v24.22.0', v8: '13.9' };
  const rustc = { ...exe('C:\\native toolchain\\rustc.exe'), host: 'aarch64-pc-windows-msvc', verbose: 'rustc 1.96.0 (fixture)\nhost: aarch64-pc-windows-msvc' };
  const cargo = { ...exe('C:\\native toolchain\\cargo.exe'), version: 'cargo 1.96.0 (fixture)' };
  const rustup = { ...exe('C:\\tools\\rustup.exe'), version: 'rustup 1.29.1 (fixture)', emulated: false };
  const helpers = {
    wasmPack: { ...exe('C:\\helpers ü\\wasm-pack.exe'), pe: pe('x64'), emulated: true, version: 'wasm-pack 0.15.0', bytes: 5649408, sha256: 'cf62764a253de34c32d5f8c5655e0503f2f7773ee3674189343e339281a9a023' },
    wasmBindgen: { ...exe('C:\\helpers ü\\wasm-bindgen.exe'), pe: pe('x64'), emulated: true, version: 'wasm-bindgen 0.2.126', bytes: WINDOWS_ARM_INSTALLATION.executableBytes, sha256: WINDOWS_ARM_INSTALLATION.executableSha256 },
  };
  const environment = { RUSTFLAGS: '', CARGO_ENCODED_RUSTFLAGS: '', CARGO_BUILD_TARGET: '', RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', RUSTUP_TOOLCHAIN: WINDOWS_ARM_TOOLCHAIN, RUSTC: rustc.path, CARGO_HOME: win32.join(temporary, 'cargo-home'), WASM_PACK_CACHE: win32.join(temporary, 'wasm-pack-cache'), CARGO_BUILD_JOBS: '1', RUST_MIN_STACK: '33554432', toolchainPathPrefix: win32.dirname(cargo.path), helperPathPrefix: win32.dirname(helpers.wasmBindgen.path) };
  environment.PATH = environment.toolchainPathPrefix + ';' + environment.helperPathPrefix + ';C:\\Windows';
  const lookup = { rustc: rustc.path, cargo: cargo.path, wasmPack: helpers.wasmPack.path, wasmBindgen: helpers.wasmBindgen.path };
  const proof = { schemaVersion: 1, passed: true, state: 'completed', sourceRevision: 'ab'.repeat(20), checkoutSha: 'ab'.repeat(20), sourceDirty: false, osMachine: 'arm64', temporary, node, rustc, cargo, rustup, helpers, installation: { ...WINDOWS_ARM_INSTALLATION }, wasmBindgenDependency: '0.2.126', environment, lookupBefore: { ...lookup }, lookupAfter: { ...lookup }, after: structuredClone({ node, rustc, cargo, helpers }), sourceRevisionAfter: 'ab'.repeat(20), sourceDirtyAfter: false, cacheBindgenCandidates: [], commands: [], buildCommands: [] };
  const stamp = (second) => `2026-10-21T00:00:${String(second).padStart(2, '0')}.000Z`;
  for (const [name, record, argv, stdout] of [
    ['source-before', { path: 'git' }, ['rev-parse', 'HEAD'], proof.sourceRevision],
    ['dirty-before', { path: 'git' }, ['status', '--porcelain', '--untracked-files=normal'], ''],
    ['rustup-version', rustup, ['--version'], rustup.version],
    ['rustup-rustc', rustup, ['which', '--toolchain', WINDOWS_ARM_TOOLCHAIN, 'rustc'], rustc.path],
    ['rustup-cargo', rustup, ['which', '--toolchain', WINDOWS_ARM_TOOLCHAIN, 'cargo'], cargo.path],
    ['rustc-version', rustc, ['-vV'], rustc.verbose], ['cargo-version', cargo, ['-V'], cargo.version],
    ['wasm-pack-version', helpers.wasmPack, ['--version'], helpers.wasmPack.version],
    ['wasm-bindgen-version', helpers.wasmBindgen, ['--version'], helpers.wasmBindgen.version],
    ['rustc-after', rustc, ['-vV'], rustc.verbose], ['cargo-after', cargo, ['-V'], cargo.version],
    ['wasm-pack-after', helpers.wasmPack, ['--version'], helpers.wasmPack.version],
    ['wasm-bindgen-after', helpers.wasmBindgen, ['--version'], helpers.wasmBindgen.version],
    ['source-after', { path: 'git' }, ['rev-parse', 'HEAD'], proof.sourceRevisionAfter],
    ['dirty-after', { path: 'git' }, ['status', '--porcelain', '--untracked-files=normal'], ''],
  ]) proof.commands.push({ name, executable: record.path, argv, stdout, status: 0, startedAt: stamp(name.endsWith('-after') ? 8 : 0), completedAt: stamp(name.endsWith('-after') ? 9 : 1) });
  const pack = helpers.wasmPack.path;
  const builds = [
    [cargo.path, 'test', '--locked', '--profile', 'ci-test', '-p', 'remus-parity'],
    [cargo.path, 'build', '--locked', '--release', '-p', 'remus-parity'],
    [pack, 'build', 'crates/wasm', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', win32.join(temporary, 'kernel'), '--', '--locked', '--no-default-features'],
    [pack, 'build', 'crates/wasm-io', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', win32.join(temporary, 'io'), '--', '--locked'],
  ];
  proof.buildCommands = builds.map((argv, index) => ({ argv, status: 0, environment: structuredClone(environment), startedAt: stamp(index + 2), completedAt: stamp(index + 3) }));
  return proof;
}

test('Windows ARM binds arm64 OS evidence, native ARM PE files and emulated helper recipes', () => {
  const host = goodHostProof(HOSTS.at(-1));
  assert.equal(validateHostProof(host).expectedRunner, 'windows-11-arm');
  assert.equal(validateWindowsArmBuild(host.windowsArmBuild, { completed: true }), true);
  for (const mutate of [
    (p) => { p.osMachine = 'aarch64'; }, (p) => { p.node.arch = 'x64'; },
    (p) => { p.rustc.pe.machine = 'x64'; }, (p) => { p.windowsArmBuild.cargo.pe.machineCode = 0x8664; },
    (p) => { p.native.kernel.pe.machine = 'x64'; }, (p) => { p.native.splitIo.pe.machineCode = 0x8664; },
    (p) => { delete p.native.kernel.bytes; }, (p) => { p.native.splitIo.path = 'C:\\runner'; },
    (p) => { delete p.windowsArmBuild; },
  ]) { const changed = structuredClone(host); mutate(changed); assert.throws(() => validateHostProof(changed)); }
  for (const mutate of [
    (p) => { p.state = 'preparing'; }, (p) => { p.sourceDirty = true; }, (p) => { p.checkoutSha = 'cd'.repeat(20); },
    (p) => { p.installation.checksum = false; }, (p) => { p.installation.fallback = 'cargo-install'; },
    (p) => { p.installation.assetId++; }, (p) => { p.installation.archiveSha256 = '00'.repeat(32); },
    (p) => { p.helpers.wasmBindgen.sha256 = '00'.repeat(32); }, (p) => { p.helpers.wasmPack.bytes--; },
    (p) => { p.helpers.wasmBindgen.emulated = false; }, (p) => { p.helpers.wasmBindgen.version = 'wasm-bindgen 0.2.100'; },
    (p) => { p.environment.RUSTC_WRAPPER = 'wrapper.exe'; }, (p) => { p.environment.CARGO_BUILD_TARGET = 'aarch64-pc-windows-msvc'; },
    (p) => { p.environment.PATH = p.environment.helperPathPrefix + ';' + p.environment.PATH; },
    (p) => { p.lookupAfter.cargo = 'C:\\shim\\cargo.exe'; }, (p) => { p.after.rustc.sha256 = '00'.repeat(32); },
    (p) => { p.buildCommands.pop(); }, (p) => { p.buildCommands[2].argv[5] = 'bundler'; },
    (p) => { p.buildCommands[0].environment.RUSTC = 'C:\\shim\\rustc.exe'; },
    (p) => { p.buildCommands[3].status = 1; }, (p) => { p.sourceDirtyAfter = true; },
    (p) => { p.commands.find((c) => c.name === 'wasm-bindgen-after').startedAt = '2026-10-21T00:00:00.000Z'; },
    (p) => { p.cacheBindgenCandidates = ['C:\\cache\\wasm-bindgen.exe']; },
    (p) => { p.wasmBindgenDependency = '0.2.127'; }, (p) => { p.rustup.emulated = true; },
  ]) { const changed = armBuildFixture(); mutate(changed); assert.throws(() => validateWindowsArmBuild(changed, { completed: true })); }
});

test('actual PE parser refuses truncated, foreign and malformed machine evidence', () => {
  const bytes = Buffer.alloc(512); bytes.write('MZ'); bytes.writeUInt32LE(64, 0x3c);
  bytes.write('PE\0\0', 64); bytes.writeUInt16LE(0xaa64, 68); bytes.writeUInt16LE(1, 70);
  bytes.writeUInt16LE(112, 84); bytes.writeUInt16LE(2, 86); bytes.writeUInt16LE(0x20b, 88);
  assert.equal(peIdentity(bytes).machine, 'arm64');
  for (const mutate of [
    (b) => b.subarray(0, 60), (b) => { b.write('NO'); return b; },
    (b) => { b.writeUInt32LE(0xffffffff, 0x3c); return b; },
    (b) => { b.writeUInt16LE(0xa641, 68); return b; },
    (b) => { b.writeUInt16LE(0x10b, 88); return b; },
    (b) => { b.writeUInt16LE(65535, 70); return b; },
    (b) => { b.writeUInt16LE(0, 86); return b; },
  ]) assert.throws(() => peIdentity(mutate(Buffer.from(bytes))));
});

test('real preflight and post-build refusal retain explicit failed envelopes', (t) => {
  const root = mkdtempSync(join(tmpdir(), 'remus-arm-preflight ü '));
  t.after(() => rmSync(root, { recursive: true, force: true }));
  const receipt = join(root, 'prepared.json');
  assert.throws(() => prepareWindowsArmBuild({ env: { ...process.env, RUSTC_WRAPPER: 'intentionally-refused-wrapper' }, repoRoot: root, temporary: root, receipt }));
  const failed = JSON.parse(readFileSync(receipt)); assert.equal(failed.passed, false); assert.equal(failed.state, 'failed'); assert.ok(failed.error);
  assert.ok(failed.commands.every((row) => row.executable === 'git'));
  const proof = armBuildFixture(), completed = join(root, 'completed.json');
  assert.throws(() => finishWindowsArmBuild({ proof, env: {}, repoRoot: root, receipt: completed }));
  assert.equal(JSON.parse(readFileSync(completed)).passed, false);
  assert.equal(JSON.parse(readFileSync(completed)).state, 'failed');
});

for (const name of ['source-after', 'dirty-after']) {
  test(`${name} must observe the source after the final full WASM build`, () => {
    const proof = armBuildFixture();
    const command = proof.commands.find((row) => row.name === name);
    command.startedAt = command.completedAt = '2026-10-21T00:00:05.000Z';
    assert.throws(() => validateWindowsArmBuild(proof, { completed: true }));
  });
}

test('completed ARM source observations retain the actual producer command order', () => {
  const proof = armBuildFixture();
  const dirty = proof.commands.pop();
  proof.commands.splice(proof.commands.length - 1, 0, dirty);
  assert.throws(() => validateWindowsArmBuild(proof, { completed: true }));
});
