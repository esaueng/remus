import assert from 'node:assert/strict';
import { execFileSync, spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { copyFileSync, existsSync, lstatSync, mkdirSync, mkdtempSync, readFileSync, readdirSync, realpathSync, writeFileSync } from 'node:fs';
import { machine, tmpdir } from 'node:os';
import { delimiter, dirname, isAbsolute, join, relative, resolve, sep, win32 } from 'node:path';
import { fileURLToPath } from 'node:url';
import { gunzipSync } from 'node:zlib';

export const TOOLCHAIN = '1.96.0-aarch64-pc-windows-msvc';
export const PACKAGE = 'remus-windows-arm-tooling-probe';
// Expected installer inputs, not an observation that its archive is retained.
// The executable digest was independently read from the checksum-verified asset.
export const BINDGEN_INSTALLATION = Object.freeze({
  action: 'taiki-e/install-action@1e29709c0ec63babbe8f266397a465f29a825f91',
  manifest: 'manifests/wasm-bindgen.json',
  tool: 'wasm-pack@0.15.0,wasm-bindgen@0.2.126', checksum: true, fallback: 'none',
  assetUrl: 'https://github.com/wasm-bindgen/wasm-bindgen/releases/download/0.2.126/wasm-bindgen-0.2.126-x86_64-pc-windows-msvc.tar.gz',
  assetId: 456882691, archiveBytes: 7583858,
  archiveSha256: '5a3773c7e69cfb2d865e235e9210de184c8c3af1787720646ec1a8bbe09c6179',
  member: 'wasm-bindgen-0.2.126-x86_64-pc-windows-msvc/wasm-bindgen.exe',
  executableBytes: 5889536,
  executableSha256: '2d5de73be088f1b53764fb298fe24f9fd44f438fa02e5d208159780b20e858ed',
  archiveRetainedInProbe: false,
});
const WASM = 'remus_windows_arm_tooling_probe_bg.wasm';
const ENTRY = 'remus_windows_arm_tooling_probe.js';
const INPUTS = ['windows-arm.mjs', 'fixtures/native.rs', 'fixtures/load-installed.cjs', 'fixtures/wasm/Cargo.toml', 'fixtures/wasm/Cargo.lock', 'fixtures/wasm/src/lib.rs'];
const here = realpathSync(dirname(fileURLToPath(import.meta.url)));
const sha256 = (bytes) => createHash('sha256').update(bytes).digest('hex');
const fileRecord = (path) => { const bytes = readFileSync(path); return { bytes: bytes.length, sha256: sha256(bytes) }; };
const json = (path) => JSON.parse(readFileSync(path, 'utf8'));

export function peIdentity(bytes) {
  assert.ok(Buffer.isBuffer(bytes) && bytes.length >= 64, 'truncated DOS header');
  assert.equal(bytes.toString('ascii', 0, 2), 'MZ', 'missing DOS signature');
  const offset = bytes.readUInt32LE(0x3c);
  assert.ok(offset >= 64 && offset <= bytes.length - 24, 'PE header outside file');
  assert.equal(bytes.toString('ascii', offset, offset + 4), 'PE\0\0', 'missing PE signature');
  assert.ok(bytes.readUInt16LE(offset + 6) > 0, 'PE contains no sections');
  const optionalBytes = bytes.readUInt16LE(offset + 20);
  assert.ok(optionalBytes >= 112 && offset + 24 + optionalBytes <= bytes.length, 'truncated PE optional header');
  assert.ok(offset + 24 + optionalBytes + 40 * bytes.readUInt16LE(offset + 6) <= bytes.length, 'truncated PE section table');
  assert.equal(bytes.readUInt16LE(offset + 24), 0x20b, 'expected PE32+');
  assert.ok((bytes.readUInt16LE(offset + 22) & 2) !== 0, 'PE is not executable');
  const code = bytes.readUInt16LE(offset + 4);
  assert.ok(code === 0xaa64 || code === 0x8664, 'unsupported PE machine');
  return { format: 'PE32+', machine: code === 0xaa64 ? 'arm64' : 'x64', machineCode: code };
}

export function requireWindowsExe(path) {
  assert.equal(typeof path, 'string');
  assert.ok(win32.isAbsolute(path) && win32.parse(path).root.length > 1 && /\.exe$/i.test(path), 'expected an absolute Windows .exe path');
  return path;
}

function executable(path) {
  requireWindowsExe(path);
  const actual = realpathSync(path);
  requireWindowsExe(actual);
  assert.ok(lstatSync(actual).isFile(), 'executable is not a regular file');
  const bytes = readFileSync(actual);
  return { path: actual, bytes: bytes.length, sha256: sha256(bytes), pe: peIdentity(bytes) };
}

export function contained(root, path) {
  const base = realpathSync(root), actual = realpathSync(path), tail = relative(base, actual);
  assert.ok(tail && tail !== '..' && !tail.startsWith('..' + sep) && !isAbsolute(tail), 'resolved file escapes its root');
  return actual;
}

export function npmCli(nodePath, environment = process.env) {
  // Invoke real npm JavaScript with the selected native Node; never execute a .cmd file.
  const candidates = [environment.npm_execpath, join(dirname(nodePath), 'node_modules', 'npm', 'bin', 'npm-cli.js')].filter(Boolean);
  for (const candidate of candidates) {
    if (!existsSync(candidate)) continue;
    const actual = realpathSync(candidate), root = dirname(dirname(actual));
    if (relative(root, actual) !== join('bin', 'npm-cli.js')) continue;
    if (!existsSync(join(root, 'package.json')) || json(join(root, 'package.json')).name !== 'npm') continue;
    return contained(root, actual);
  }
  throw new Error('cannot find the actual npm/bin/npm-cli.js for the selected Node');
}

export function findExe(name, environment = process.env) {
  for (const folder of (environment.PATH ?? '').split(delimiter)) {
    const candidate = join(folder.replace(/^"|"$/g, ''), name + '.exe');
    if (existsSync(candidate)) return realpathSync(candidate);
  }
  throw new Error(`missing executable: ${name}.exe`);
}

export function selectBindgen(candidates) {
  const matching = candidates.filter((item) => /^wasm-bindgen 0\.2\.126(?:\s|$)/.test(item.version));
  assert.equal(matching.length, 1, 'require one unambiguous observed wasm-bindgen 0.2.126 helper');
  return matching[0];
}

function insideWindows(root, path) {
  assert.ok(win32.isAbsolute(root) && win32.parse(root).root.length > 1);
  assert.ok(win32.isAbsolute(path) && win32.parse(path).root.length > 1);
  const tail = win32.relative(root, path);
  assert.ok(tail && tail !== '..' && !tail.startsWith('..\\') && !win32.isAbsolute(tail), 'reported installed path escapes its package');
}

export function validateProof(proof) {
  assert.equal(proof.schemaVersion, 1);
  assert.equal(proof.scope, 'Windows ARM tooling only; no Remus geometry qualification');
  assert.equal(proof.passed, true);
  assert.match(proof.source.revision, /^[a-f0-9]{40}$/);
  assert.equal(proof.source.checkoutSha, proof.source.revision);
  assert.equal(proof.source.dirtyBefore, false);
  assert.equal(proof.source.dirtyAfter, false);
  assert.equal(proof.runner.os, 'Windows');
  assert.equal(proof.runner.arch, 'ARM64');
  assert.equal(proof.runner.repository, 'esaueng/remus');
  assert.equal(proof.node.platform, 'win32');
  assert.equal(proof.node.arch, 'arm64');
  assert.match(proof.node.version, /^v24\./);
  assert.equal(proof.osMachine, 'arm64');
  assert.equal(proof.osMachineSource, 'node:os.machine');
  assert.deepEqual(proof.environment, { CARGO_BUILD_TARGET: '', RUSTFLAGS: '', CARGO_ENCODED_RUSTFLAGS: '', RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', RUSTUP_TOOLCHAIN: TOOLCHAIN, CARGO_BUILD_JOBS: '1', RUST_MIN_STACK: '33554432' });
  assert.equal(proof.rustc.host, 'aarch64-pc-windows-msvc');
  assert.match(proof.rustc.version, /^rustc 1\.96\.0[ (]/);
  assert.match(proof.cargoVersion, /^cargo 1\.96\.0[ (]/);
  assert.equal(proof.buildEnvironment.RUSTC, proof.rustc.path);
  assert.equal(proof.buildEnvironment.RUSTC_WRAPPER, '');
  assert.equal(proof.buildEnvironment.RUSTC_WORKSPACE_WRAPPER, '');
  assert.equal(proof.buildEnvironment.CARGO_HOME, win32.join(proof.temporary, 'cargo-home'));
  for (const item of [proof.node, proof.rustc, proof.cargo, proof.native]) {
    requireWindowsExe(item.path);
    assert.match(item.sha256, /^[a-f0-9]{64}$/);
    assert.ok(Number.isSafeInteger(item.bytes) && item.bytes > 0);
    assert.deepEqual(item.pe, { format: 'PE32+', machine: 'arm64', machineCode: 0xaa64 });
  }
  assert.deepEqual(proof.native.result, { schemaVersion: 1, os: 'windows', arch: 'aarch64', pointerWidth: 64, result: 42 });
  for (const helper of [proof.helpers.rustup, proof.helpers.wasmPack, proof.helpers.wasmBindgen]) {
    requireWindowsExe(helper.path);
    assert.match(helper.sha256, /^[a-f0-9]{64}$/);
    assert.ok(['arm64', 'x64'].includes(helper.pe.machine));
    assert.deepEqual(helper.pe, { format: 'PE32+', machine: helper.pe.machine, machineCode: helper.pe.machine === 'x64' ? 0x8664 : 0xaa64 });
    assert.equal(helper.emulated, helper.pe.machine === 'x64');
  }
  assert.equal(proof.helpers.wasmPack.pe.machine, 'x64');
  assert.equal(proof.helpers.wasmPack.version.trim(), 'wasm-pack 0.15.0');
  assert.match(proof.helpers.wasmBindgen.version, /^wasm-bindgen 0\.2\.126(?:\s|$)/);
  assert.deepEqual(proof.bindgenInstallation, BINDGEN_INSTALLATION);
  assert.equal(proof.helpers.wasmBindgen.pe.machine, 'x64');
  assert.equal(proof.helpers.wasmBindgen.bytes, BINDGEN_INSTALLATION.executableBytes);
  assert.equal(proof.helpers.wasmBindgen.sha256, BINDGEN_INSTALLATION.executableSha256);
  assert.equal(proof.helpers.wasmBindgen.observation, 'preinstalled x64 helper and pre-build version probe; not a child-process trace');
  assert.equal(proof.buildEnvironment.toolchainPathPrefix, win32.dirname(proof.cargo.path));
  assert.equal(win32.dirname(proof.rustc.path), proof.buildEnvironment.toolchainPathPrefix);
  assert.equal(proof.buildEnvironment.helperPathPrefix, win32.dirname(proof.helpers.wasmBindgen.path));
  assert.deepEqual(proof.buildEnvironment.PATH.split(';').slice(0, 2), [proof.buildEnvironment.toolchainPathPrefix, proof.buildEnvironment.helperPathPrefix]);
  assert.equal(proof.bindgenPathBeforeBuild, proof.helpers.wasmBindgen.path);
  assert.equal(proof.bindgenPathAfterBuild, proof.helpers.wasmBindgen.path);
  const selected = selectBindgen(proof.helpers.wasmBindgenCandidates);
  assert.equal(selected.path, proof.helpers.wasmBindgen.path);
  assert.equal(selected.sha256, proof.helpers.wasmBindgen.sha256);
  assert.equal(selected.bytes, proof.helpers.wasmBindgen.bytes);
  assert.deepEqual(selected.pe, proof.helpers.wasmBindgen.pe);
  assert.equal(proof.installed.packageName, PACKAGE);
  assert.equal(proof.installed.packageVersion, '0.0.0');
  assert.equal(proof.installed.platform, 'win32');
  assert.equal(proof.installed.arch, 'arm64');
  assert.equal(proof.installed.nodeVersion, proof.node.version);
  assert.equal(proof.installed.nodePath, proof.node.path);
  assert.equal(proof.installed.result, 42);
  assert.match(proof.package.wasmSha256, /^[a-f0-9]{64}$/);
  assert.match(proof.installed.wasmSha256, /^[a-f0-9]{64}$/);
  assert.equal(proof.installed.wasmSha256, proof.package.wasmSha256);
  insideWindows(proof.installed.packageRoot, proof.installed.entry);
  insideWindows(proof.installed.packageRoot, proof.installed.wasmPath);
  assert.equal(win32.basename(proof.installed.entry), ENTRY);
  assert.equal(win32.basename(proof.installed.wasmPath), WASM);
  assert.equal(win32.basename(proof.installed.packageRoot), PACKAGE);
  const installedRoot = win32.join(proof.temporary, 'consumer', 'node_modules', PACKAGE);
  assert.equal(proof.installed.packageRoot, installedRoot);
  assert.equal(proof.installed.entry, win32.join(installedRoot, ENTRY));
  assert.equal(proof.installed.wasmPath, win32.join(installedRoot, WASM));
  assert.match(proof.package.archive.sha256, /^[a-f0-9]{64}$/);
  assert.ok(Number.isSafeInteger(proof.package.archive.bytes) && proof.package.archive.bytes > 0);
  assert.equal(proof.package.archive.filename, PACKAGE + '-0.0.0.tgz');
  assert.match(proof.package.archive.integrity, /^sha512-[A-Za-z0-9+/]{86}==$/);
  assert.ok(proof.package.files.length >= 3);
  assert.equal(new Set(proof.package.files.map((file) => file.path)).size, proof.package.files.length);
  for (const file of proof.package.files) {
    assert.ok(typeof file.path === 'string' && file.path.split('/').every((part) => part && part !== '.' && part !== '..') && !/[\\:]/.test(file.path));
    assert.match(file.sha256, /^[a-f0-9]{64}$/);
    assert.ok(Number.isSafeInteger(file.bytes) && file.bytes > 0);
    assert.equal(file.installedSha256, file.sha256);
  }
  const wasmMembers = proof.package.files.filter((file) => file.path.endsWith('.wasm'));
  assert.equal(wasmMembers.length, 1);
  assert.equal(wasmMembers[0].path, WASM);
  assert.equal(wasmMembers[0].sha256, proof.package.wasmSha256);
  assert.ok(proof.package.files.some((file) => file.path === ENTRY));
  assert.ok(proof.package.files.some((file) => file.path === 'package.json'));
  assert.deepEqual(Object.keys(proof.inputs.files).sort(), [...INPUTS].sort());
  for (const file of Object.values(proof.inputs.files)) { assert.match(file.sha256, /^[a-f0-9]{64}$/); assert.ok(Number.isSafeInteger(file.bytes) && file.bytes > 0); }
  for (const path of [proof.inputs.directory, proof.output, proof.temporary]) assert.ok(win32.isAbsolute(path) && win32.parse(path).root.length > 1);
  assert.equal(proof.inputs.nativePath, win32.join(proof.inputs.directory, 'fixtures', 'native.rs'));
  assert.equal(proof.inputs.loaderPath, win32.join(proof.inputs.directory, 'fixtures', 'load-installed.cjs'));
  assert.equal(proof.native.path, win32.join(proof.temporary, 'native-probe.exe'));
  for (const name of ['rustup-version', 'rustup-rustc', 'rustup-cargo', 'rustc-version', 'cargo-version', 'wasm-pack-version', 'native-build', 'native-run', 'wasm-bindgen-version', 'wasm-build', 'npm-version', 'npm-pack', 'npm-install', 'installed-load']) {
    const records = proof.commands.filter((command) => command.name === name);
    assert.equal(records.length, 1, `missing or duplicated command: ${name}`);
    assert.equal(records[0].status, 0, `failed command: ${name}`);
    assert.ok(Array.isArray(records[0].argv) && records[0].argv.every((arg) => typeof arg === 'string'));
  }
  const call = (name) => proof.commands.find((item) => item.name === name);
  assert.equal(call('wasm-bindgen-version').executable, proof.helpers.wasmBindgen.path);
  assert.deepEqual(call('wasm-bindgen-version').argv, ['--version']);
  assert.ok(proof.commands.indexOf(call('wasm-bindgen-version')) < proof.commands.indexOf(call('wasm-build')), 'preinstalled helper version must precede wasm-pack');
  assert.equal(call('native-build').executable, proof.rustc.path);
  assert.deepEqual(call('native-build').argv, [proof.inputs.nativePath, '--edition=2021', '-O', '-o', proof.native.path]);
  assert.equal(call('native-run').executable, proof.native.path);
  assert.deepEqual(call('native-run').argv, []);
  assert.equal(call('wasm-build').executable, proof.helpers.wasmPack.path);
  assert.deepEqual(call('wasm-build').argv, ['build', win32.join(proof.temporary, 'crate'), '--target', 'nodejs', '--release', '--no-opt', '--out-dir', win32.join(proof.temporary, 'pkg'), '--', '--locked']);
  for (const name of ['npm-version', 'npm-pack', 'npm-install', 'installed-load']) assert.equal(call(name).executable, proof.node.path);
  for (const name of ['npm-version', 'npm-pack', 'npm-install']) assert.equal(call(name).argv[0], proof.npm.path);
  for (const name of ['npm-version', 'npm-pack', 'installed-load']) assert.equal(call(name).cwd, proof.inputs.directory);
  assert.deepEqual(call('npm-version').argv, [proof.npm.path, '--version']);
  assert.deepEqual(call('npm-pack').argv, [proof.npm.path, 'pack', '--json', '--ignore-scripts', '--pack-destination', win32.join(proof.output, 'archives'), win32.join(proof.temporary, 'pkg')]);
  assert.deepEqual(call('npm-install').argv, [proof.npm.path, 'install', '--ignore-scripts', '--no-audit', '--no-fund', win32.join(proof.output, 'archives', proof.package.archive.filename)]);
  assert.equal(call('npm-install').cwd, win32.join(proof.temporary, 'consumer'));
  assert.deepEqual(call('installed-load').argv, [proof.inputs.loaderPath, installedRoot]);
  assert.match(proof.npm.sha256, /^[a-f0-9]{64}$/);
  assert.match(proof.npm.version, /^\d+\.\d+\.\d+$/);
  return true;
}

export function verifyArtifact(root) {
  const proof = json(join(root, 'proof.json')); validateProof(proof);
  const native = readFileSync(contained(root, join(root, 'native-probe.exe')));
  assert.equal(sha256(native), proof.native.sha256);
  assert.equal(native.length, proof.native.bytes);
  assert.deepEqual(peIdentity(native), proof.native.pe);
  const archive = readFileSync(contained(root, join(root, 'archives', proof.package.archive.filename)));
  assert.equal(sha256(archive), proof.package.archive.sha256);
  assert.equal(archive.length, proof.package.archive.bytes);
  assert.equal('sha512-' + createHash('sha512').update(archive).digest('base64'), proof.package.archive.integrity);
  assert.deepEqual(packageArchiveMembers(archive), proof.package.files.map(({ path, bytes, sha256 }) => ({ path, bytes, sha256 })).sort((a, b) => a.path.localeCompare(b.path, 'en')), 'archive members differ from packed/installed file evidence');
  for (const leaf of ['package', 'installed-package']) {
    assert.deepEqual(packageMembers(join(root, leaf)), proof.package.files.map((file) => file.path).sort(), 'retained package file census changed');
    for (const file of proof.package.files) assert.deepEqual(fileRecord(contained(join(root, leaf), join(root, leaf, file.path))), { bytes: file.bytes, sha256: file.sha256 });
  }
  return true;
}

export function packageArchiveMembers(archive) {
  // The fixed fixture uses short ASCII names. Unsupported TAR extensions refuse.
  assert.ok(archive.length <= 16 * 1024 * 1024, 'compressed archive exceeds probe budget');
  const tar = gunzipSync(archive, { maxOutputLength: 16 * 1024 * 1024 });
  assert.equal(tar.length % 512, 0, 'TAR blocks are truncated');
  const files = [], seen = new Map(); let offset = 0, ended = false;
  const text = (b, start, size) => b.subarray(start, start + size).toString('utf8').split('\0')[0];
  const octal = (b, start, size) => { const value = text(b, start, size).trim(); assert.match(value, /^[0-7]+$/, 'unsupported TAR numeric field'); const number = Number.parseInt(value, 8); assert.ok(Number.isSafeInteger(number)); return number; };
  while (offset + 512 <= tar.length) {
    const header = tar.subarray(offset, offset + 512);
    if (header.every((byte) => byte === 0)) {
      assert.ok(offset + 1024 <= tar.length && tar.subarray(offset).every((byte) => byte === 0), 'invalid TAR end padding');
      ended = true; break;
    }
    assert.ok(seen.size < 64, 'archive member count exceeds probe budget');
    const checksum = [...header].reduce((sum, byte, index) => sum + (index >= 148 && index < 156 ? 32 : byte), 0);
    assert.equal(octal(header, 148, 8), checksum, 'TAR header checksum differs');
    assert.equal(text(header, 257, 6), 'ustar', 'unsupported TAR format');
    assert.equal(text(header, 345, 155), '', 'TAR prefix extension is not needed by this probe');
    const name = text(header, 0, 100), kind = header[156], size = octal(header, 124, 12);
    assert.ok(kind === 0 || kind === 48 || kind === 53, 'only regular files and directories are allowed');
    assert.ok(!/[\\:\u0000-\u001f\u007f-\uffff]/.test(name), 'unsupported TAR member path');
    const parts = name.replace(/\/$/, '').split('/');
    assert.equal(parts[0], 'package'); assert.ok(parts.every((part) => part && part !== '.' && part !== '..'));
    const path = parts.slice(1).join('/'), key = path.toLowerCase();
    assert.ok(!seen.has(key), 'duplicate archive member');
    for (const [prior, priorKind] of seen) {
      assert.ok(!(key.startsWith(prior + '/') && priorKind !== 53) && !(prior.startsWith(key + '/') && kind !== 53), 'archive ancestor collision');
    }
    seen.set(key, kind);
    const start = offset + 512, end = start + size;
    assert.ok(end <= tar.length, 'TAR payload is truncated');
    if (kind === 53) assert.equal(size, 0, 'directory has a payload');
    else { assert.ok(path && !name.endsWith('/'), 'invalid regular file path'); files.push({ path, bytes: size, sha256: sha256(tar.subarray(start, end)) }); }
    offset = start + Math.ceil(size / 512) * 512;
  }
  assert.ok(ended && files.length > 0, 'missing archive files or end markers');
  return files.sort((a, b) => a.path.localeCompare(b.path, 'en'));
}

function packageMembers(root) {
  const found = [];
  function visit(path) {
    for (const item of readdirSync(path, { withFileTypes: true })) {
      const child = join(path, item.name); assert.ok(!item.isSymbolicLink());
      if (item.isDirectory()) visit(child);
      else { assert.ok(lstatSync(child).isFile()); found.push(relative(root, child).replaceAll('\\', '/')); }
    }
  }
  visit(root); return found.sort();
}

export function validateWorkflowPolicy(text) {
  assert.match(text, /on:\n  pull_request:\n    paths:\n      - '\.github\/workflows\/windows-arm-tooling\.yml'\n      - 'scripts\/platform-probes\/\*\*'/);
  assert.doesNotMatch(text, /pull_request_target|workflow_dispatch|merge_group|secrets\.|\n  push:|CARGO_BUILD_TARGET:/);
  assert.match(text, /permissions:\n  contents: read\n/);
  assert.doesNotMatch(text, /\w+: write/);
  assert.match(text, /runs-on: windows-11-arm/);
  assert.match(text, /timeout-minutes: 25/);
  assert.match(text, /timeout-minutes: 20/);
  assert.match(text, /node-version: 24\n          architecture: arm64/);
  assert.match(text, /toolchain: '1\.96\.0-aarch64-pc-windows-msvc'\n          targets: wasm32-unknown-unknown/);
  assert.match(text, /RUSTUP_TOOLCHAIN: '1\.96\.0-aarch64-pc-windows-msvc'/);
  assert.match(text, /uses: taiki-e\/install-action@1e29709c0ec63babbe8f266397a465f29a825f91[^\n]*\n        with:\n          tool: wasm-pack@0\.15\.0,wasm-bindgen@0\.2\.126\n          checksum: true\n          fallback: none\n/);
  assert.match(text, /if: always\(\)/);
  assert.match(text, /persist-credentials: false/);
  assert.equal([...text.matchAll(/uses: [^@\n]+@[a-f0-9]{40}(?: |\n)/g)].length, 5);
  return true;
}

export function run(outputPath) {
  let out = resolve(outputPath);
  assert.ok(!existsSync(join(out, 'proof.json')), 'refuse to overwrite an existing proof');
  mkdirSync(out, { recursive: true });
  out = realpathSync(out);
  const proof = { schemaVersion: 1, scope: 'Windows ARM tooling only; no Remus geometry qualification', passed: false, output: out, commands: [] };
  let environment;
  function command(name, path, args, cwd = here) {
    const record = { name, executable: path, argv: args, cwd, startedAt: new Date().toISOString() };
    proof.commands.push(record);
    try {
      const result = spawnSync(path, args, { cwd, env: environment, encoding: 'utf8', timeout: 600000, maxBuffer: 16 * 1024 * 1024, stdio: ['ignore', 'pipe', 'pipe'] });
      record.status = result.status;
      writeFileSync(join(out, name + '.stdout.log'), result.stdout ?? '');
      writeFileSync(join(out, name + '.stderr.log'), result.stderr ?? '');
      if (result.error) throw result.error;
      assert.equal(result.status, 0, `command failed: ${name}`);
      return result.stdout.trim();
    } catch (error) {
      record.error = error.message;
      throw error;
    } finally { record.completedAt = new Date().toISOString(); }
  }
  try {
    const git = (args) => execFileSync('git', args, { cwd: here, encoding: 'utf8' }).trim();
    proof.source = { revision: git(['rev-parse', 'HEAD']), checkoutSha: process.env.GITHUB_SHA, dirtyBefore: git(['status', '--porcelain', '--untracked-files=normal']) !== '' };
    proof.runner = { os: process.env.RUNNER_OS, arch: process.env.RUNNER_ARCH, repository: process.env.GITHUB_REPOSITORY, event: process.env.GITHUB_EVENT_NAME, runId: process.env.GITHUB_RUN_ID, runAttempt: process.env.GITHUB_RUN_ATTEMPT };
    proof.osMachine = machine();
    proof.osMachineSource = 'node:os.machine';
    proof.node = { ...executable(process.execPath), platform: process.platform, arch: process.arch, version: process.version };
    assert.equal(proof.node.platform, 'win32'); assert.equal(proof.node.arch, 'arm64'); assert.equal(proof.node.pe.machine, 'arm64');
    assert.equal(proof.source.dirtyBefore, false); assert.equal(proof.source.revision, proof.source.checkoutSha);
    proof.environment = Object.fromEntries(['CARGO_BUILD_TARGET', 'RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER', 'RUSTUP_TOOLCHAIN', 'CARGO_BUILD_JOBS', 'RUST_MIN_STACK'].map((key) => [key, process.env[key] ?? '']));
    assert.deepEqual(proof.environment, { CARGO_BUILD_TARGET: '', RUSTFLAGS: '', CARGO_ENCODED_RUSTFLAGS: '', RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', RUSTUP_TOOLCHAIN: TOOLCHAIN, CARGO_BUILD_JOBS: '1', RUST_MIN_STACK: '33554432' });
    const temporary = realpathSync(mkdtempSync(join(tmpdir(), 'remus-windows-arm-tooling.')));
    proof.temporary = temporary; // Preserve failed build inputs until the hosted runner is discarded.
    environment = { ...process.env, CARGO_TARGET_DIR: join(temporary, 'target'), WASM_PACK_CACHE: join(temporary, 'wasm-pack-cache') };
    proof.buildEnvironment = { CARGO_TARGET_DIR: environment.CARGO_TARGET_DIR, WASM_PACK_CACHE: environment.WASM_PACK_CACHE };
    proof.helpers = { rustup: executable(findExe('rustup')), wasmPack: executable(findExe('wasm-pack')) };
    for (const helper of Object.values(proof.helpers)) helper.emulated = helper.pe.machine === 'x64';
    proof.helpers.rustup.version = command('rustup-version', proof.helpers.rustup.path, ['--version']);
    const rustc = command('rustup-rustc', proof.helpers.rustup.path, ['which', '--toolchain', TOOLCHAIN, 'rustc']);
    const cargo = command('rustup-cargo', proof.helpers.rustup.path, ['which', '--toolchain', TOOLCHAIN, 'cargo']);
    proof.rustc = executable(rustc); proof.cargo = executable(cargo);
    assert.equal(proof.rustc.pe.machine, 'arm64'); assert.equal(proof.cargo.pe.machine, 'arm64');
    // wasm-pack discovers cargo/rustc by PATH; put the verified actual toolchain first.
    environment.PATH = dirname(proof.cargo.path) + delimiter + process.env.PATH;
    proof.buildEnvironment.toolchainPathPrefix = dirname(proof.cargo.path);
    Object.assign(environment, { RUSTC: proof.rustc.path, RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', CARGO_HOME: join(temporary, 'cargo-home') });
    Object.assign(proof.buildEnvironment, { RUSTC: environment.RUSTC, RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', CARGO_HOME: environment.CARGO_HOME });
    proof.rustc.version = command('rustc-version', proof.rustc.path, ['-vV']);
    proof.rustc.host = /^host: (.+)$/m.exec(proof.rustc.version)?.[1]?.trim();
    proof.cargoVersion = command('cargo-version', proof.cargo.path, ['-V']);
    proof.helpers.wasmPack.version = command('wasm-pack-version', proof.helpers.wasmPack.path, ['--version']);
    proof.inputs = { directory: here, nativePath: realpathSync(join(here, 'fixtures', 'native.rs')), loaderPath: realpathSync(join(here, 'fixtures', 'load-installed.cjs')), files: Object.fromEntries(INPUTS.map((path) => [path, fileRecord(join(here, path))])) };
    const native = join(temporary, 'native-probe.exe');
    command('native-build', proof.rustc.path, [proof.inputs.nativePath, '--edition=2021', '-O', '-o', native]);
    proof.native = executable(native);
    assert.equal(proof.native.pe.machine, 'arm64');
    proof.native.result = JSON.parse(command('native-run', proof.native.path, []));
    copyFileSync(native, join(out, 'native-probe.exe'));
    // Use wasm-pack's standard exact-version PATH lookup. Both helpers are
    // official x64 assets, explicitly emulated; no cache files are rearranged.
    proof.bindgenInstallation = { ...BINDGEN_INSTALLATION };
    proof.helpers.wasmBindgen = executable(findExe('wasm-bindgen'));
    assert.equal(proof.helpers.wasmBindgen.pe.machine, 'x64');
    assert.equal(proof.helpers.wasmBindgen.bytes, BINDGEN_INSTALLATION.executableBytes);
    assert.equal(proof.helpers.wasmBindgen.sha256, BINDGEN_INSTALLATION.executableSha256);
    proof.helpers.wasmBindgen.emulated = true;
    proof.helpers.wasmBindgen.observation = 'preinstalled x64 helper and pre-build version probe; not a child-process trace';
    proof.helpers.wasmBindgen.version = command('wasm-bindgen-version', proof.helpers.wasmBindgen.path, ['--version']);
    assert.match(proof.helpers.wasmBindgen.version, /^wasm-bindgen 0\.2\.126(?:\s|$)/);
    proof.buildEnvironment.helperPathPrefix = dirname(proof.helpers.wasmBindgen.path);
    environment.PATH = proof.buildEnvironment.toolchainPathPrefix + delimiter + proof.buildEnvironment.helperPathPrefix + delimiter + process.env.PATH;
    proof.buildEnvironment.PATH = environment.PATH;
    proof.bindgenPathBeforeBuild = findExe('wasm-bindgen', environment);
    assert.equal(proof.bindgenPathBeforeBuild, proof.helpers.wasmBindgen.path);
    const crate = join(temporary, 'crate'); mkdirSync(join(crate, 'src'), { recursive: true });
    for (const path of ['Cargo.toml', 'Cargo.lock', 'src/lib.rs']) copyFileSync(join(here, 'fixtures', 'wasm', path), join(crate, path));
    const pkg = join(temporary, 'pkg');
    command('wasm-build', proof.helpers.wasmPack.path, ['build', crate, '--target', 'nodejs', '--release', '--no-opt', '--out-dir', pkg, '--', '--locked']);
    const candidates = [];
    function scan(path) { if (!existsSync(path)) return; for (const item of readdirSync(path, { withFileTypes: true })) { const file = join(path, item.name); if (item.isDirectory()) scan(file); else if (item.name === 'wasm-bindgen.exe') candidates.push(file); } }
    scan(environment.WASM_PACK_CACHE);
    proof.bindgenPathAfterBuild = findExe('wasm-bindgen', environment);
    assert.equal(proof.bindgenPathAfterBuild, proof.helpers.wasmBindgen.path);
    candidates.push(proof.bindgenPathAfterBuild);
    proof.helpers.wasmBindgenCandidates = [...new Set(candidates.map((path) => realpathSync(path)))].map((path, index) => ({ ...executable(path), version: command(`wasm-bindgen-candidate-${index}`, path, ['--version']) }));
    const selected = selectBindgen(proof.helpers.wasmBindgenCandidates);
    assert.equal(selected.path, proof.helpers.wasmBindgen.path);
    assert.equal(selected.sha256, proof.helpers.wasmBindgen.sha256);
    assert.deepEqual(selected.pe, proof.helpers.wasmBindgen.pe);
    const npm = npmCli(proof.node.path);
    proof.npm = { path: npm, ...fileRecord(npm), version: command('npm-version', proof.node.path, [npm, '--version']) };
    const archives = join(out, 'archives'); mkdirSync(archives);
    const packed = JSON.parse(command('npm-pack', proof.node.path, [npm, 'pack', '--json', '--ignore-scripts', '--pack-destination', archives, pkg]));
    assert.equal(packed.length, 1); const pack = packed[0];
    assert.equal(pack.name, PACKAGE); assert.equal(pack.version, '0.0.0');
    const archive = contained(archives, join(archives, pack.filename));
    assert.equal('sha512-' + createHash('sha512').update(readFileSync(archive)).digest('base64'), pack.integrity);
    const consumer = join(temporary, 'consumer'); mkdirSync(consumer); writeFileSync(join(consumer, 'package.json'), '{"private":true}\n');
    command('npm-install', proof.node.path, [npm, 'install', '--ignore-scripts', '--no-audit', '--no-fund', archive], consumer);
    const installed = contained(join(consumer, 'node_modules'), join(consumer, 'node_modules', PACKAGE));
    const files = pack.files.map(({ path }) => {
      const source = contained(pkg, join(pkg, path)); const target = contained(installed, join(installed, path));
      const original = fileRecord(source), actual = fileRecord(target);
      assert.deepEqual(actual, original, `installed bytes differ: ${path}`);
      for (const [leaf, from] of [['package', source], ['installed-package', target]]) { const to = join(out, leaf, path); mkdirSync(dirname(to), { recursive: true }); copyFileSync(from, to); }
      return { path, ...original, installedSha256: actual.sha256 };
    });
    assert.deepEqual(packageMembers(installed), files.map((file) => file.path).sort(), 'installed package census differs from the actual npm archive');
    proof.package = { name: pack.name, version: pack.version, archive: { filename: pack.filename, integrity: pack.integrity, ...fileRecord(archive) }, files, wasmSha256: files.find((file) => file.path.endsWith('.wasm'))?.sha256 };
    proof.installed = JSON.parse(command('installed-load', proof.node.path, [proof.inputs.loaderPath, installed]));
    proof.source.dirtyAfter = git(['status', '--porcelain', '--untracked-files=normal']) !== '';
    proof.passed = true;
    validateProof(proof);
  } catch (error) { proof.passed = false; proof.error = error.message; process.exitCode = 1; }
  writeFileSync(join(out, 'proof.json'), JSON.stringify(proof, null, 2) + '\n');
  if (proof.passed) {
    try { verifyArtifact(out); }
    catch (error) { proof.passed = false; proof.error = error.message; process.exitCode = 1; writeFileSync(join(out, 'proof.json'), JSON.stringify(proof, null, 2) + '\n'); }
  }
  console.log(JSON.stringify({ passed: proof.passed, scope: proof.scope, proof: join(out, 'proof.json'), error: proof.error }));
  return proof;
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  assert.equal(process.argv.length, 3, 'usage: node windows-arm.mjs OUTPUT_DIRECTORY');
  run(process.argv[2]);
}
