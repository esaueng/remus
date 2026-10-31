import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { machine } from 'node:os';
import { existsSync, readFileSync, readdirSync, realpathSync, writeFileSync } from 'node:fs';
import { delimiter, dirname, join, resolve, win32 } from 'node:path';
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
  { runner: 'windows-11-arm', platform: 'win32', arch: 'arm64', rustHost: 'aarch64-pc-windows-msvc', runnerOs: 'Windows', runnerArch: 'ARM64', nativeOs: 'windows', nativeArch: 'aarch64' },
]);

function nonemptyString(value) {
  return typeof value === 'string' && value.trim().length > 0;
}

function requireWindowsExecutable(path) {
  assert.ok(nonemptyString(path) && win32.isAbsolute(path) && win32.parse(path).root.length > 1
    && /\.exe$/i.test(path), 'Windows executable must have an absolute drive/UNC path and explicit .exe suffix');
}

export const WINDOWS_ARM_TOOLCHAIN = '1.96.0-aarch64-pc-windows-msvc';
export const WINDOWS_ARM_INSTALLATION = Object.freeze({
  action: 'taiki-e/install-action@1e29709c0ec63babbe8f266397a465f29a825f91',
  tool: 'wasm-pack@0.15.0,wasm-bindgen@0.2.126', checksum: true, fallback: 'none',
  assetUrl: 'https://github.com/wasm-bindgen/wasm-bindgen/releases/download/0.2.126/wasm-bindgen-0.2.126-x86_64-pc-windows-msvc.tar.gz',
  assetId: 456882691, archiveBytes: 7583858,
  archiveSha256: '5a3773c7e69cfb2d865e235e9210de184c8c3af1787720646ec1a8bbe09c6179',
  member: 'wasm-bindgen-0.2.126-x86_64-pc-windows-msvc/wasm-bindgen.exe',
  executableBytes: 5889536,
  executableSha256: '2d5de73be088f1b53764fb298fe24f9fd44f438fa02e5d208159780b20e858ed',
  archiveRetainedInProof: false,
});

export function peIdentity(bytes) {
  assert.ok(Buffer.isBuffer(bytes) && bytes.length >= 64, 'truncated DOS header');
  assert.equal(bytes.toString('ascii', 0, 2), 'MZ', 'missing DOS signature');
  const offset = bytes.readUInt32LE(0x3c);
  assert.ok(offset >= 64 && offset <= bytes.length - 24, 'PE header outside file');
  assert.equal(bytes.toString('ascii', offset, offset + 4), 'PE\0\0', 'missing PE signature');
  const sections = bytes.readUInt16LE(offset + 6), optional = bytes.readUInt16LE(offset + 20);
  assert.ok(sections > 0 && optional >= 112 && offset + 24 + optional + sections * 40 <= bytes.length, 'truncated PE sections');
  assert.equal(bytes.readUInt16LE(offset + 24), 0x20b, 'expected PE32+');
  assert.ok(bytes.readUInt16LE(offset + 22) & 2, 'PE is not executable');
  const code = bytes.readUInt16LE(offset + 4);
  assert.ok(code === 0xaa64 || code === 0x8664, 'unsupported PE machine');
  return { format: 'PE32+', machine: code === 0xaa64 ? 'arm64' : 'x64', machineCode: code };
}

function validatePeRecord(record, machine) {
  requireWindowsExecutable(record?.path);
  assert.match(record.sha256 ?? '', /^[0-9a-f]{64}$/);
  assert.ok(Number.isSafeInteger(record.bytes) && record.bytes > 0, 'executable byte count is missing');
  assert.deepEqual(record.pe, { format: 'PE32+', machine, machineCode: machine === 'arm64' ? 0xaa64 : 0x8664 });
}

function windowsExecutable(path) {
  requireWindowsExecutable(path);
  const actual = realpathSync(path); requireWindowsExecutable(actual);
  const bytes = readFileSync(actual);
  const sha256 = createHash('sha256').update(bytes).digest('hex');
  assert.equal(hashFile(actual), sha256, 'executable changed while reading PE bytes');
  return { path: actual, bytes: bytes.length, sha256, pe: peIdentity(bytes) };
}

export function findWindowsExecutable(name, env) {
  for (const folder of (env.PATH ?? '').split(delimiter)) {
    const file = join(folder.replace(/^"|"$/g, ''), name + '.exe');
    if (existsSync(file)) return realpathSync(file);
  }
  throw new Error(`missing ${name}.exe on the prepared PATH`);
}

/** A prepared envelope qualifies native host setup; only completed can gate WASM reports. */
export function validateWindowsArmBuild(proof, { completed = false } = {}) {
  assert.equal(proof?.schemaVersion, 1);
  assert.equal(proof.passed, true);
  assert.ok(['prepared', 'completed'].includes(proof.state));
  if (completed) assert.equal(proof.state, 'completed', 'Windows ARM build evidence is incomplete');
  assert.match(proof.sourceRevision ?? '', /^[0-9a-f]{40}$/);
  assert.equal(proof.checkoutSha, proof.sourceRevision);
  assert.equal(proof.sourceDirty, false);
  assert.equal(proof.osMachine, 'arm64');
  for (const record of [proof.node, proof.rustc, proof.cargo]) validatePeRecord(record, 'arm64');
  assert.equal(proof.node.platform, 'win32'); assert.equal(proof.node.arch, 'arm64');
  assert.match(proof.node.version, /^v24\./); assert.ok(nonemptyString(proof.node.v8));
  assert.equal(proof.rustc.host, 'aarch64-pc-windows-msvc');
  assert.match(proof.rustc.verbose, /^rustc 1\.96\.0[ (]/);
  assert.equal([...proof.rustc.verbose.matchAll(/^host: (\S+)$/gm)].length, 1);
  assert.match(proof.rustc.verbose, /^host: aarch64-pc-windows-msvc$/m);
  assert.match(proof.cargo.version, /^cargo 1\.96\.0[ (]/);
  assert.deepEqual(proof.installation, WINDOWS_ARM_INSTALLATION);
  assert.equal(proof.wasmBindgenDependency, '0.2.126');
  validatePeRecord(proof.rustup, proof.rustup?.pe?.machine);
  assert.ok(['arm64', 'x64'].includes(proof.rustup.pe.machine));
  assert.equal(proof.rustup.emulated, proof.rustup.pe.machine === 'x64');
  assert.ok(nonemptyString(proof.rustup.version));
  for (const [key, version] of [['wasmPack', 'wasm-pack 0.15.0'], ['wasmBindgen', 'wasm-bindgen 0.2.126']]) {
    const helper = proof.helpers[key]; validatePeRecord(helper, 'x64');
    assert.equal(helper.emulated, true); assert.equal(helper.version, version);
  }
  assert.equal(proof.helpers.wasmPack.bytes, 5649408);
  assert.equal(proof.helpers.wasmPack.sha256, 'cf62764a253de34c32d5f8c5655e0503f2f7773ee3674189343e339281a9a023');
  assert.equal(proof.helpers.wasmBindgen.bytes, WINDOWS_ARM_INSTALLATION.executableBytes);
  assert.equal(proof.helpers.wasmBindgen.sha256, WINDOWS_ARM_INSTALLATION.executableSha256);
  const env = proof.environment;
  for (const key of ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'CARGO_BUILD_TARGET', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER']) assert.equal(env[key], '', `unsupported Windows ARM ${key}`);
  assert.equal(env.RUSTUP_TOOLCHAIN, WINDOWS_ARM_TOOLCHAIN);
  assert.equal(env.CARGO_BUILD_JOBS, '1'); assert.equal(env.RUST_MIN_STACK, '33554432');
  assert.equal(env.RUSTC, proof.rustc.path);
  assert.equal(env.CARGO_HOME, win32.join(proof.temporary, 'cargo-home'));
  assert.equal(env.WASM_PACK_CACHE, win32.join(proof.temporary, 'wasm-pack-cache'));
  assert.equal(env.toolchainPathPrefix, win32.dirname(proof.cargo.path));
  assert.equal(win32.dirname(proof.rustc.path), env.toolchainPathPrefix);
  assert.equal(env.helperPathPrefix, win32.dirname(proof.helpers.wasmBindgen.path));
  assert.equal(win32.dirname(proof.helpers.wasmPack.path), env.helperPathPrefix);
  assert.deepEqual(env.PATH.split(';').slice(0, 2), [env.toolchainPathPrefix, env.helperPathPrefix]);
  const expectedLookup = { rustc: proof.rustc.path, cargo: proof.cargo.path, wasmPack: proof.helpers.wasmPack.path, wasmBindgen: proof.helpers.wasmBindgen.path };
  assert.deepEqual(proof.lookupBefore, expectedLookup);
  const requireVersion = (name, record, argv, expected) => {
    const rows = proof.commands.filter((row) => row.name === name); assert.equal(rows.length, 1, `missing/duplicate ${name}`);
    assert.equal(rows[0].status, 0); assert.equal(rows[0].executable, record.path);
    assert.deepEqual(rows[0].argv, argv); assert.equal(rows[0].stdout.trim(), expected);
    assert.ok(Number.isFinite(Date.parse(rows[0].startedAt)) && Date.parse(rows[0].completedAt) >= Date.parse(rows[0].startedAt));
    return rows[0];
  };
  for (const [name, record, argv, version] of [
    ['source-before', { path: 'git' }, ['rev-parse', 'HEAD'], proof.sourceRevision],
    ['dirty-before', { path: 'git' }, ['status', '--porcelain', '--untracked-files=normal'], ''],
    ['rustup-version', proof.rustup, ['--version'], proof.rustup.version],
    ['rustup-rustc', proof.rustup, ['which', '--toolchain', WINDOWS_ARM_TOOLCHAIN, 'rustc'], proof.rustc.path],
    ['rustup-cargo', proof.rustup, ['which', '--toolchain', WINDOWS_ARM_TOOLCHAIN, 'cargo'], proof.cargo.path],
    ['rustc-version', proof.rustc, ['-vV'], proof.rustc.verbose], ['cargo-version', proof.cargo, ['-V'], proof.cargo.version],
    ['wasm-pack-version', proof.helpers.wasmPack, ['--version'], proof.helpers.wasmPack.version],
    ['wasm-bindgen-version', proof.helpers.wasmBindgen, ['--version'], proof.helpers.wasmBindgen.version],
  ]) requireVersion(name, record, argv, version);
  const commandNames = ['source-before', 'dirty-before', 'rustup-version', 'rustup-rustc', 'rustup-cargo', 'rustc-version', 'cargo-version', 'wasm-pack-version', 'wasm-bindgen-version'];
  if (proof.state === 'completed') {
    assert.deepEqual(proof.cacheBindgenCandidates, [], 'wasm-pack unexpectedly used a cached fallback helper');
    assert.deepEqual(proof.lookupAfter, expectedLookup);
    assert.deepEqual(proof.after, { node: proof.node, rustc: proof.rustc, cargo: proof.cargo, helpers: proof.helpers });
    assert.equal(proof.sourceRevisionAfter, proof.sourceRevision); assert.equal(proof.sourceDirtyAfter, false);
    const cargo = proof.cargo.path, pack = proof.helpers.wasmPack.path;
    const expectedBuilds = [
      [cargo, 'test', '--locked', '--profile', 'ci-test', '-p', 'remus-parity'],
      [cargo, 'build', '--locked', '--release', '-p', 'remus-parity'],
      [pack, 'build', 'crates/wasm', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', win32.join(proof.temporary, 'kernel'), '--', '--locked', '--no-default-features'],
      [pack, 'build', 'crates/wasm-io', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', win32.join(proof.temporary, 'io'), '--', '--locked'],
    ];
    assert.deepEqual(proof.buildCommands.map((row) => row.argv), expectedBuilds);
    for (const row of proof.buildCommands) { assert.equal(row.status, 0); assert.deepEqual(row.environment, env); assert.ok(Date.parse(row.completedAt) >= Date.parse(row.startedAt)); }
    for (let index = 1; index < proof.buildCommands.length; index++) assert.ok(Date.parse(proof.buildCommands[index - 1].completedAt) <= Date.parse(proof.buildCommands[index].startedAt));
    for (const name of ['rustc', 'cargo', 'wasm-pack', 'wasm-bindgen']) {
      const record = name === 'rustc' ? proof.rustc : name === 'cargo' ? proof.cargo : name === 'wasm-pack' ? proof.helpers.wasmPack : proof.helpers.wasmBindgen;
      const argv = name === 'rustc' ? ['-vV'] : name === 'cargo' ? ['-V'] : ['--version'];
      const version = name === 'rustc' ? proof.rustc.verbose : record.version;
      const after = requireVersion(name + '-after', record, argv, version);
      assert.ok(Date.parse(proof.buildCommands.at(-1).completedAt) <= Date.parse(after.startedAt));
      commandNames.push(name + '-after');
    }
    for (const [name, argv, value] of [
      ['source-after', ['rev-parse', 'HEAD'], proof.sourceRevisionAfter],
      ['dirty-after', ['status', '--porcelain', '--untracked-files=normal'], ''],
    ]) {
      const after = requireVersion(name, { path: 'git' }, argv, value);
      assert.ok(Date.parse(proof.buildCommands.at(-1).completedAt) <= Date.parse(after.startedAt), `${name} precedes the final WASM build`);
    }
    commandNames.push('source-after', 'dirty-after');
    assert.ok(Math.max(...proof.commands.filter((row) => !/-after$/.test(row.name)).map((row) => Date.parse(row.completedAt))) <= Date.parse(proof.buildCommands[0].startedAt));
  }
  assert.deepEqual(proof.commands.map((row) => row.name), commandNames, 'unexpected, duplicated or reordered build-tool observations');
  return true;
}

function armCommand(proof, receipt, env, repoRoot, name, executable, argv) {
  const record = { name, executable, argv, startedAt: new Date().toISOString() };
  proof.commands.push(record);
  try {
    record.stdout = execFileSync(executable, argv, { cwd: repoRoot, env, encoding: 'utf8', timeout: 15000, maxBuffer: 1024 * 1024 });
    record.status = 0; return record.stdout.trim();
  } catch (error) { record.status = error.status ?? 1; record.error = error.message; record.stdout = error.stdout?.toString() ?? ''; record.stderr = error.stderr?.toString() ?? ''; throw error; }
  finally { record.completedAt = new Date().toISOString(); writeFileSync(receipt, JSON.stringify(proof, null, 2) + '\n'); }
}

export function prepareWindowsArmBuild({ env, repoRoot, temporary, receipt }) {
  const proof = { schemaVersion: 1, passed: false, state: 'preparing', temporary, installation: { ...WINDOWS_ARM_INSTALLATION }, commands: [], buildCommands: [] };
  const run = (name, path, args, selected = env) => armCommand(proof, receipt, selected, repoRoot, name, path, args);
  try {
    assert.equal(process.platform, 'win32'); assert.equal(process.arch, 'arm64');
    proof.sourceRevision = run('source-before', 'git', ['rev-parse', 'HEAD']);
    proof.sourceDirty = run('dirty-before', 'git', ['status', '--porcelain', '--untracked-files=normal']) !== '';
    proof.checkoutSha = env.GITHUB_SHA ?? proof.sourceRevision; proof.osMachine = machine();
    for (const key of ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'CARGO_BUILD_TARGET', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER']) assert.equal(env[key] ?? '', '', `unsupported Windows ARM ${key}`);
    const lock = readFileSync(join(repoRoot, 'Cargo.lock'), 'utf8');
    const bindgen = [...lock.matchAll(/\[\[package\]\]\r?\nname = "wasm-bindgen"\r?\nversion = "([^"]+)"/g)];
    assert.equal(bindgen.length, 1); proof.wasmBindgenDependency = bindgen[0][1];
    const rustup = windowsExecutable(findWindowsExecutable('rustup', env));
    proof.rustup = { ...rustup, emulated: rustup.pe.machine === 'x64', version: run('rustup-version', rustup.path, ['--version']) };
    const rustcPath = run('rustup-rustc', rustup.path, ['which', '--toolchain', WINDOWS_ARM_TOOLCHAIN, 'rustc']);
    const cargoPath = run('rustup-cargo', rustup.path, ['which', '--toolchain', WINDOWS_ARM_TOOLCHAIN, 'cargo']);
    proof.node = { platform: process.platform, arch: process.arch, version: process.version, v8: process.versions.v8, ...windowsExecutable(process.execPath) };
    proof.rustc = windowsExecutable(rustcPath); proof.cargo = windowsExecutable(cargoPath);
    proof.helpers = { wasmPack: windowsExecutable(findWindowsExecutable('wasm-pack', env)), wasmBindgen: windowsExecutable(findWindowsExecutable('wasm-bindgen', env)) };
    const child = { ...env, RUSTC: proof.rustc.path, RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', CARGO_HOME: join(temporary, 'cargo-home'), WASM_PACK_CACHE: join(temporary, 'wasm-pack-cache') };
    for (const key of Object.keys(child)) if (key.toLowerCase() === 'path') delete child[key];
    delete child.CARGO_BUILD_TARGET;
    child.PATH = dirname(proof.cargo.path) + delimiter + dirname(proof.helpers.wasmBindgen.path) + delimiter + env.PATH;
    proof.environment = Object.fromEntries(['RUSTC', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'CARGO_BUILD_TARGET', 'RUSTUP_TOOLCHAIN', 'CARGO_HOME', 'WASM_PACK_CACHE', 'CARGO_BUILD_JOBS', 'RUST_MIN_STACK', 'PATH'].map((key) => [key, child[key] ?? '']));
    Object.assign(proof.environment, { toolchainPathPrefix: dirname(proof.cargo.path), helperPathPrefix: dirname(proof.helpers.wasmBindgen.path) });
    proof.rustc.verbose = run('rustc-version', proof.rustc.path, ['-vV'], child); proof.rustc.host = /^host: (\S+)$/m.exec(proof.rustc.verbose)?.[1];
    proof.cargo.version = run('cargo-version', proof.cargo.path, ['-V'], child);
    for (const [name, key] of [['wasm-pack', 'wasmPack'], ['wasm-bindgen', 'wasmBindgen']]) Object.assign(proof.helpers[key], { emulated: true, version: run(name + '-version', proof.helpers[key].path, ['--version'], child) });
    proof.lookupBefore = Object.fromEntries([['rustc', 'rustc'], ['cargo', 'cargo'], ['wasmPack', 'wasm-pack'], ['wasmBindgen', 'wasm-bindgen']].map(([key, name]) => [key, findWindowsExecutable(name, child)]));
    proof.state = 'prepared'; proof.passed = true; validateWindowsArmBuild(proof);
    return { proof, env: { ...child, O15_WINDOWS_ARM_BUILD: receipt } };
  } catch (error) { proof.passed = false; proof.state = 'failed'; proof.error = error.message; throw error; }
  finally { writeFileSync(receipt, JSON.stringify(proof, null, 2) + '\n'); }
}

export function finishWindowsArmBuild({ proof, env, repoRoot, receipt }) {
  const run = (name, path, args) => armCommand(proof, receipt, env, repoRoot, name, path, args);
  try {
    const after = { node: { platform: process.platform, arch: process.arch, version: process.version, v8: process.versions.v8, ...windowsExecutable(process.execPath) }, rustc: windowsExecutable(proof.rustc.path), cargo: windowsExecutable(proof.cargo.path), helpers: {} };
    after.rustc.verbose = run('rustc-after', after.rustc.path, ['-vV']); after.rustc.host = /^host: (\S+)$/m.exec(after.rustc.verbose)?.[1];
    after.cargo.version = run('cargo-after', after.cargo.path, ['-V']);
    for (const [name, key] of [['wasm-pack', 'wasmPack'], ['wasm-bindgen', 'wasmBindgen']]) after.helpers[key] = { ...windowsExecutable(proof.helpers[key].path), emulated: true, version: run(name + '-after', proof.helpers[key].path, ['--version']) };
    proof.after = after;
    const candidates = []; let visited = 0;
    function scan(path, depth = 0) {
      if (!existsSync(path)) return;
      assert.ok(depth <= 16, 'helper cache nesting exceeds probe budget');
      for (const item of readdirSync(path, { withFileTypes: true })) {
        assert.ok(++visited <= 4096, 'helper cache census exceeds probe budget');
        assert.ok(!item.isSymbolicLink(), 'helper cache contains a link');
        const file = join(path, item.name);
        if (item.isDirectory()) scan(file, depth + 1);
        else if (item.name.toLowerCase() === 'wasm-bindgen.exe') candidates.push(file);
      }
    }
    scan(env.WASM_PACK_CACHE); proof.cacheBindgenCandidates = candidates;
    proof.lookupAfter = Object.fromEntries([['rustc', 'rustc'], ['cargo', 'cargo'], ['wasmPack', 'wasm-pack'], ['wasmBindgen', 'wasm-bindgen']].map(([key, name]) => [key, findWindowsExecutable(name, env)]));
    proof.sourceRevisionAfter = run('source-after', 'git', ['rev-parse', 'HEAD']);
    proof.sourceDirtyAfter = run('dirty-after', 'git', ['status', '--porcelain', '--untracked-files=normal']) !== '';
    proof.state = 'completed'; validateWindowsArmBuild(proof, { completed: true });
  } catch (error) { proof.passed = false; proof.state = 'failed'; proof.error = error.message; throw error; }
  finally { writeFileSync(receipt, JSON.stringify(proof, null, 2) + '\n'); }
  return proof;
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
  const expectedMachine = ['darwin', 'win32'].includes(host.platform) && host.arch === 'arm64' ? 'arm64' : host.nativeArch;
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
  if (host.platform === 'win32' && host.arch === 'arm64') {
    validateWindowsArmBuild(proof.windowsArmBuild);
    const build = proof.windowsArmBuild;
    assert.equal(build.sourceRevision, proof.sourceRevision);
    assert.equal(build.sourceDirty, proof.sourceDirty);
    assert.equal(build.checkoutSha, proof.checkoutSha);
    assert.deepEqual(proof.node, build.node);
    assert.deepEqual(proof.rustc, build.rustc);
    for (const runner of Object.values(proof.native)) validatePeRecord(runner, 'arm64');
  }
  return { platform: host.platform, arch: host.arch, rustHost: host.rustHost, expectedRunner: proof.expected?.runner ?? null };
}

/** Collect facts from the current process/compiler and the exact runner files. */
export function collectHostProof(nativeRunner, splitRunner, env = process.env) {
  const repoRoot = fileURLToPath(new URL('../../', import.meta.url));
  const source = collectSourceProvenance(repoRoot);
  const run = (command, args) => execFileSync(command, args, {
    env, encoding: 'utf8', timeout: 15_000, maxBuffer: 1024 * 1024,
  }).trim();
  const arm = process.platform === 'win32' && process.arch === 'arm64';
  const build = arm ? JSON.parse(readFileSync(env.O15_WINDOWS_ARM_BUILD, 'utf8')) : null;
  if (arm) validateWindowsArmBuild(build);
  const verbose = run(arm ? build.rustc.path : 'rustc', ['-vV']);
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
    const pe = arm ? windowsExecutable(path) : {};
    if (arm) assert.equal(pe.sha256, sha256, `${key}: executable changed while binding PE evidence`);
    native[key] = { ...pe, path, sha256, runtime };
  }
  const proof = {
    schema_version: 1,
    sourceRevision: source.sourceRevision,
    sourceDirty: source.sourceDirty,
    checkoutSha: env.GITHUB_SHA ?? source.sourceRevision,
    expected,
    runner: { os: env.RUNNER_OS ?? null, arch: env.RUNNER_ARCH ?? null },
    node: { platform: process.platform, arch: process.arch, version: process.version, v8: process.versions.v8, ...(arm ? windowsExecutable(process.execPath) : {}) },
    rustc: { host, verbose, ...(arm ? windowsExecutable(build.rustc.path) : {}) },
    unameMachine: process.platform === 'win32' ? null : run('uname', ['-m']),
    osMachine: machine(),
    osMachineSource: 'node:os.machine',
    buildTarget: env.CARGO_BUILD_TARGET ?? null,
    native,
    ...(arm ? { windowsArmBuild: build } : {}),
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
