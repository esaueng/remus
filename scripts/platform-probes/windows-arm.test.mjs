import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { copyFileSync, existsSync, mkdirSync, mkdtempSync, readFileSync, realpathSync, rmSync, symlinkSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { delimiter, dirname, join, win32 } from 'node:path';
import test from 'node:test';
import { fileURLToPath } from 'node:url';
import { gzipSync } from 'node:zlib';

import { BINDGEN_INSTALLATION, PACKAGE, TOOLCHAIN, contained, findExe, npmCli, packageArchiveMembers, peIdentity, requireWindowsExe, selectBindgen, validateProof, validateWorkflowPolicy, verifyArtifact } from './windows-arm.mjs';

const here = dirname(fileURLToPath(import.meta.url));
const arm = { format: 'PE32+', machine: 'arm64', machineCode: 0xaa64 };
function pe(code = 0xaa64) {
  const bytes = Buffer.alloc(512); bytes.write('MZ'); bytes.writeUInt32LE(64, 0x3c);
  bytes.write('PE\0\0', 64); bytes.writeUInt16LE(code, 68); bytes.writeUInt16LE(1, 70);
  bytes.writeUInt16LE(112, 84); bytes.writeUInt16LE(2, 86); bytes.writeUInt16LE(0x20b, 88);
  return bytes;
}
test('PE identity distinguishes actual ARM64 and x64 helpers', () => {
  assert.deepEqual(peIdentity(pe()), arm);
  assert.deepEqual(peIdentity(pe(0x8664)), { format: 'PE32+', machine: 'x64', machineCode: 0x8664 });
  for (const mutate of [
    (b) => b.subarray(0, 50), (b) => { b.write('NO'); return b; },
    (b) => { b.writeUInt32LE(0xfffffff0, 0x3c); return b; },
    (b) => { b.write('XX', 64); return b; }, (b) => { b.writeUInt16LE(0xa641, 68); return b; },
    (b) => { b.writeUInt16LE(0, 70); return b; }, (b) => { b.writeUInt16LE(100, 70); return b; },
    (b) => { b.writeUInt16LE(65535, 84); return b; }, (b) => { b.writeUInt16LE(0, 86); return b; },
    (b) => { b.writeUInt16LE(0x10b, 88); return b; },
  ]) assert.throws(() => peIdentity(mutate(pe())));
});

test('executed Windows paths must be explicit absolute .exe files', () => {
  for (const path of ['C:\\tool path ü\\probe.exe', '\\\\server\\share\\probe.EXE']) requireWindowsExe(path);
  for (const path of ['probe.exe', '/tmp/probe.exe', '\\probe.exe', 'C:probe.exe', 'C:\\probe', 'C:\\probe.cmd', 'C:\\probe.bat']) assert.throws(() => requireWindowsExe(path));
});

function fixture(t) {
  const root = mkdtempSync(join(tmpdir(), 'remus-arm probe ü '));
  t.after(() => rmSync(root, { recursive: true, force: true })); return root;
}
function makeNpm(root) {
  const npm = join(root, 'node_modules', 'npm'); mkdirSync(join(npm, 'bin'), { recursive: true });
  writeFileSync(join(npm, 'package.json'), '{"name":"npm"}'); const cli = join(npm, 'bin', 'npm-cli.js'); writeFileSync(cli, '// fixture'); return cli;
}
test('actual npm CLI accepts real aliases, refuses pnpm/missing/escaped entries, and needs no shell', (t) => {
  const root = fixture(t), cli = makeNpm(root), node = join(root, 'node.exe');
  assert.equal(npmCli(node, {}), realpathSync(cli));
  const alias = root + '-alias'; symlinkSync(root, alias, process.platform === 'win32' ? 'junction' : 'dir');
  t.after(() => rmSync(alias, { recursive: true, force: true }));
  assert.equal(npmCli(join(alias, 'node.exe'), {}), realpathSync(cli));
  const foreign = join(root, 'other'); mkdirSync(foreign); writeFileSync(join(foreign, 'pnpm.cjs'), '// pnpm');
  assert.throws(() => npmCli(join(foreign, 'node.exe'), { npm_execpath: join(foreign, 'pnpm.cjs') }));
  assert.throws(() => contained(foreign, cli));
  const link = join(foreign, 'escape'); symlinkSync(dirname(cli), link, process.platform === 'win32' ? 'junction' : 'dir');
  assert.throws(() => contained(foreign, join(link, 'npm-cli.js')));
});

function goodProof() {
  const exe = (name) => ({ path: `C:\\tools ü\\${name}.exe`, bytes: 512, sha256: '12'.repeat(32), pe: arm });
  const helper = (name) => ({ ...exe(name), pe: { format: 'PE32+', machine: 'x64', machineCode: 0x8664 }, emulated: true });
  const node = { ...exe('node'), platform: 'win32', arch: 'arm64', version: 'v24.20.0' };
  const p = {
    schemaVersion: 1, scope: 'Windows ARM tooling only; no Remus geometry qualification', passed: true,
    source: { revision: 'ab'.repeat(20), checkoutSha: 'ab'.repeat(20), dirtyBefore: false, dirtyAfter: false },
    runner: { os: 'Windows', arch: 'ARM64', repository: 'esaueng/remus' }, node, osMachine: 'arm64', osMachineSource: 'node:os.machine',
    environment: { CARGO_BUILD_TARGET: '', RUSTFLAGS: '', CARGO_ENCODED_RUSTFLAGS: '', RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', RUSTUP_TOOLCHAIN: TOOLCHAIN, CARGO_BUILD_JOBS: '1', RUST_MIN_STACK: '33554432' },
    rustc: { ...exe('rustc'), host: 'aarch64-pc-windows-msvc', version: 'rustc 1.96.0 (fixture)' }, cargo: exe('cargo'), cargoVersion: 'cargo 1.96.0 (fixture)',
    native: { ...exe('native-probe'), result: { schemaVersion: 1, os: 'windows', arch: 'aarch64', pointerWidth: 64, result: 42 } },
    helpers: { rustup: helper('rustup'), wasmPack: { ...helper('wasm-pack'), version: 'wasm-pack 0.15.0' }, wasmBindgen: { ...helper('wasm-bindgen'), version: 'wasm-bindgen 0.2.126', observation: 'preinstalled x64 helper and pre-build version probe; not a child-process trace' } },
    installed: { packageName: PACKAGE, packageVersion: '0.0.0', platform: 'win32', arch: 'arm64', nodeVersion: node.version, nodePath: node.path, result: 42, wasmSha256: '34'.repeat(32) },
    package: { archive: { sha256: '56'.repeat(32), bytes: 42, filename: PACKAGE + '-0.0.0.tgz', integrity: 'sha512-' + 'A'.repeat(86) + '==' }, files: ['package.json', 'remus_windows_arm_tooling_probe.js', 'remus_windows_arm_tooling_probe_bg.wasm'].map((path) => ({ path, bytes: 8, sha256: '34'.repeat(32), installedSha256: '34'.repeat(32) })), wasmSha256: '34'.repeat(32) },
    inputs: { directory: 'C:\\source', nativePath: 'C:\\source\\fixtures\\native.rs', loaderPath: 'C:\\source\\fixtures\\load-installed.cjs', files: Object.fromEntries(['windows-arm.mjs', 'fixtures/native.rs', 'fixtures/load-installed.cjs', 'fixtures/wasm/Cargo.toml', 'fixtures/wasm/Cargo.lock', 'fixtures/wasm/src/lib.rs'].map((path) => [path, { bytes: 1, sha256: '12'.repeat(32) }])) }, temporary: 'C:\\Temp\\remus-arm.aBc 123 ü', output: 'C:\\source\\target\\proof', npm: { path: 'C:\\npm\\bin\\npm-cli.js', version: '11.0.0', sha256: '78'.repeat(32) }, commands: [],
  };
  p.native.path = win32.join(p.temporary, 'native-probe.exe');
  p.bindgenInstallation = { ...BINDGEN_INSTALLATION };
  Object.assign(p.helpers.wasmBindgen, { path: 'C:\\helpers ü\\wasm-bindgen.exe', bytes: BINDGEN_INSTALLATION.executableBytes, sha256: BINDGEN_INSTALLATION.executableSha256 });
  p.bindgenPathBeforeBuild = p.bindgenPathAfterBuild = p.helpers.wasmBindgen.path;
  p.installed.packageRoot = win32.join(p.temporary, 'consumer', 'node_modules', PACKAGE);
  p.installed.entry = win32.join(p.installed.packageRoot, 'remus_windows_arm_tooling_probe.js');
  p.installed.wasmPath = win32.join(p.installed.packageRoot, 'remus_windows_arm_tooling_probe_bg.wasm');
  p.helpers.wasmBindgenCandidates = [p.helpers.wasmBindgen];
  p.buildEnvironment = { RUSTC: p.rustc.path, RUSTC_WRAPPER: '', RUSTC_WORKSPACE_WRAPPER: '', CARGO_HOME: win32.join(p.temporary, 'cargo-home'), toolchainPathPrefix: win32.dirname(p.cargo.path), helperPathPrefix: win32.dirname(p.helpers.wasmBindgen.path) };
  p.buildEnvironment.PATH = p.buildEnvironment.toolchainPathPrefix + ';' + p.buildEnvironment.helperPathPrefix + ';C:\\Windows';
  p.commands = ['rustup-version', 'rustup-rustc', 'rustup-cargo', 'rustc-version', 'cargo-version', 'wasm-pack-version', 'native-build', 'native-run', 'wasm-bindgen-version', 'wasm-build', 'npm-version', 'npm-pack', 'npm-install', 'installed-load'].map((name) => ({ name, executable: node.path, argv: [], status: 0 }));
  const call = (name) => p.commands.find((item) => item.name === name);
  Object.assign(call('wasm-bindgen-version'), { executable: p.helpers.wasmBindgen.path, argv: ['--version'] });
  Object.assign(call('native-build'), { executable: p.rustc.path, argv: [p.inputs.nativePath, '--edition=2021', '-O', '-o', p.native.path] });
  call('native-run').executable = p.native.path;
  Object.assign(call('wasm-build'), { executable: p.helpers.wasmPack.path, argv: ['build', win32.join(p.temporary, 'crate'), '--target', 'nodejs', '--release', '--no-opt', '--out-dir', win32.join(p.temporary, 'pkg'), '--', '--locked'] });
  for (const name of ['npm-version', 'npm-pack', 'installed-load']) call(name).cwd = p.inputs.directory;
  call('npm-version').argv = [p.npm.path, '--version'];
  call('npm-pack').argv = [p.npm.path, 'pack', '--json', '--ignore-scripts', '--pack-destination', win32.join(p.output, 'archives'), win32.join(p.temporary, 'pkg')];
  call('npm-install').argv = [p.npm.path, 'install', '--ignore-scripts', '--no-audit', '--no-fund', win32.join(p.output, 'archives', p.package.archive.filename)];
  call('npm-install').cwd = win32.join(p.temporary, 'consumer');
  call('installed-load').argv = [p.inputs.loaderPath, p.installed.packageRoot];
  return p;
}
test('complete proof binds native Node/compiler/program and labels emulated helpers separately', () => {
  assert.equal(validateProof(goodProof()), true);
  for (const mutate of [
    (p) => { p.passed = false; }, (p) => { p.source.dirtyAfter = true; }, (p) => { p.source.checkoutSha = 'cd'.repeat(20); },
    (p) => { p.node.arch = 'x64'; }, (p) => { p.node.version = 'v22.0.0'; }, (p) => { p.node.pe = p.helpers.rustup.pe; },
    (p) => { p.osMachine = 'x86_64'; }, (p) => { p.rustc.host = 'x86_64-pc-windows-msvc'; }, (p) => { p.rustc.pe = p.helpers.rustup.pe; },
    (p) => { p.native.pe = p.helpers.rustup.pe; }, (p) => { p.native.result.arch = 'x86_64'; }, (p) => { p.native.result.result = 43; },
    (p) => { p.helpers.wasmPack.emulated = false; }, (p) => { p.environment.CARGO_BUILD_TARGET = 'aarch64-pc-windows-msvc'; },
    (p) => { p.buildEnvironment.RUSTC = p.helpers.rustup.path; }, (p) => { p.environment.RUSTC_WRAPPER = 'wrapper.exe'; },
    (p) => { p.buildEnvironment.RUSTC_WORKSPACE_WRAPPER = 'wrapper.exe'; },
    (p) => { p.package.files[0].installedSha256 = '56'.repeat(32); }, (p) => { p.installed.wasmSha256 = '56'.repeat(32); },
    (p) => { p.installed.arch = 'x64'; }, (p) => { p.commands.pop(); }, (p) => { p.commands[0].status = 1; },
    (p) => { delete p.package.wasmSha256; delete p.installed.wasmSha256; },
    (p) => { p.package.files.pop(); }, (p) => { p.package.files.push({ ...p.package.files[2], path: 'extra.wasm' }); },
    (p) => { p.installed.entry = 'C:\\outside\\remus_windows_arm_tooling_probe.js'; },
    (p) => { p.installed.wasmPath = 'C:\\outside\\remus_windows_arm_tooling_probe_bg.wasm'; },
    (p) => { delete p.inputs.files['fixtures/native.rs']; }, (p) => { p.package.archive.bytes = 0; },
    (p) => { p.commands.find((c) => c.name === 'wasm-build').argv = ['build']; },
    (p) => { p.commands.find((c) => c.name === 'npm-install').executable = 'C:\\npm.cmd'; },
    (p) => { p.commands.find((c) => c.name === 'npm-pack').argv = [p.npm.path]; },
    (p) => { p.commands.find((c) => c.name === 'npm-install').argv = [p.npm.path]; },
    (p) => { p.commands.find((c) => c.name === 'installed-load').argv = ['anything.js']; },
    (p) => { p.commands.find((c) => c.name === 'npm-install').cwd = 'C:\\other'; },
    (p) => { p.installed.packageRoot = 'C:\\different\\' + PACKAGE; p.installed.entry = win32.join(p.installed.packageRoot, 'remus_windows_arm_tooling_probe.js'); p.installed.wasmPath = win32.join(p.installed.packageRoot, 'remus_windows_arm_tooling_probe_bg.wasm'); },
  ]) { const p = goodProof(); mutate(p); assert.throws(() => validateProof(p)); }
});

test('an unrelated old global wasm-bindgen does not mask the exact downloaded helper', () => {
  const matching = { path: 'C:\\cache\\wasm-bindgen.exe', version: 'wasm-bindgen 0.2.126' };
  const old = { path: 'C:\\global\\wasm-bindgen.exe', version: 'wasm-bindgen 0.2.100' };
  assert.equal(selectBindgen([old, matching]), matching);
  assert.throws(() => selectBindgen([old]));
  assert.throws(() => selectBindgen([matching, { ...matching, path: 'C:\\other\\wasm-bindgen.exe' }]));
});

test('wasm-bindgen binds the pinned official x64 asset, standard PATH lookup and native toolchain priority', (t) => {
  const p = goodProof();
  validateProof(p);
  const root = fixture(t), toolchain = join(root, 'native-toolchain'), helper = join(root, 'x64-helper'), fallback = join(root, 'fallback-cache');
  for (const dir of [toolchain, helper, fallback]) mkdirSync(dir);
  writeFileSync(join(toolchain, 'cargo.exe'), pe());
  writeFileSync(join(helper, 'wasm-bindgen.exe'), pe(0x8664)); writeFileSync(join(fallback, 'wasm-bindgen.exe'), pe());
  const environment = { PATH: [toolchain, helper, fallback].join(delimiter) };
  assert.equal(findExe('cargo', environment), realpathSync(join(toolchain, 'cargo.exe')));
  assert.equal(findExe('wasm-bindgen', environment), realpathSync(join(helper, 'wasm-bindgen.exe')));
  for (const mutate of [
    (x) => { x.bindgenInstallation.assetUrl = 'https://example.com/wasm-bindgen.tar.gz'; },
    (x) => { x.bindgenInstallation.assetId++; },
    (x) => { x.bindgenInstallation.archiveBytes++; },
    (x) => { x.bindgenInstallation.archiveSha256 = '00'.repeat(32); },
    (x) => { x.bindgenInstallation.checksum = false; },
    (x) => { x.bindgenInstallation.fallback = 'cargo-install'; },
    (x) => { x.bindgenInstallation.action = 'taiki-e/install-action@v2'; },
    (x) => { x.bindgenInstallation.action = 'taiki-e/install-action@ab6895351a88b908e96a4697860d4ffea5e57dbe'; },
    (x) => { x.bindgenInstallation.archiveRetainedInProbe = true; },
    (x) => { x.helpers.wasmBindgen.sha256 = '00'.repeat(32); },
    (x) => { x.helpers.wasmBindgen.bytes++; },
    (x) => { x.helpers.wasmBindgen.pe = arm; x.helpers.wasmBindgen.emulated = false; },
    (x) => { x.helpers.wasmBindgen.emulated = false; },
    (x) => { x.buildEnvironment.helperPathPrefix = win32.join(x.temporary, 'unrelated-bin'); },
    (x) => { x.buildEnvironment.toolchainPathPrefix = x.buildEnvironment.helperPathPrefix; },
    (x) => { x.buildEnvironment.PATH = x.buildEnvironment.helperPathPrefix + ';' + x.buildEnvironment.PATH; },
    (x) => { x.helpers.wasmBindgen.version = 'wasm-bindgen 0.2.100'; },
    (x) => { x.bindgenPathBeforeBuild = win32.join(x.temporary, 'unexpected', 'wasm-bindgen.exe'); },
    (x) => { x.bindgenPathAfterBuild = win32.join(x.temporary, 'unexpected', 'wasm-bindgen.exe'); },
    (x) => { x.helpers.wasmBindgenCandidates = [{ ...x.helpers.wasmBindgen, sha256: '00'.repeat(32) }]; },
    (x) => { x.helpers.wasmBindgenCandidates = [{ ...x.helpers.wasmBindgen, bytes: 1 }]; },
    (x) => { x.helpers.wasmBindgenCandidates = [{ ...x.helpers.wasmBindgen, pe: arm }]; },
    (x) => { const version = x.commands.splice(x.commands.findIndex((c) => c.name === 'wasm-bindgen-version'), 1)[0]; x.commands.push(version); },
  ]) { const changed = structuredClone(p); mutate(changed); assert.throws(() => validateProof(changed)); }
});

test('retained native/archive/package bytes must match the successful receipt', (t) => {
  const root = fixture(t), p = goodProof(), digest = (bytes) => createHash('sha256').update(bytes).digest('hex');
  const native = pe();
  p.native.sha256 = digest(native); p.native.bytes = native.length;
  mkdirSync(join(root, 'archives')); writeFileSync(join(root, 'native-probe.exe'), native);
  for (const file of p.package.files) {
    const bytes = file.path === 'package.json' ? Buffer.from(JSON.stringify({ name: PACKAGE, version: '0.0.0', main: p.package.files[1].path })) : Buffer.from('fixture:' + file.path);
    Object.assign(file, { sha256: digest(bytes), installedSha256: digest(bytes), bytes: bytes.length });
    for (const leaf of ['package', 'installed-package']) { mkdirSync(join(root, leaf), { recursive: true }); writeFileSync(join(root, leaf, file.path), bytes); }
  }
  const candidates = (process.env.PATH ?? '').split(delimiter).map((path) => join(path, 'npm')).filter((path) => existsSync(path));
  const cli = npmCli(realpathSync(process.execPath), { npm_execpath: candidates[0] });
  function pack(source) {
    const result = spawnSync(process.execPath, [cli, 'pack', '--json', '--ignore-scripts', '--pack-destination', join(root, 'archives'), source], { encoding: 'utf8', timeout: 30000 });
    assert.equal(result.status, 0, result.stderr); const info = JSON.parse(result.stdout)[0];
    const bytes = readFileSync(join(root, 'archives', info.filename));
    Object.assign(p.package.archive, { sha256: digest(bytes), bytes: bytes.length, integrity: info.integrity });
  }
  pack(join(root, 'package'));
  p.package.wasmSha256 = p.installed.wasmSha256 = p.package.files[2].sha256;
  writeFileSync(join(root, 'proof.json'), JSON.stringify(p));
  assert.equal(verifyArtifact(root), true);
  writeFileSync(join(root, 'installed-package', p.package.files[0].path), 'tampered');
  assert.throws(() => verifyArtifact(root));
  copyFileSync(join(root, 'package', p.package.files[0].path), join(root, 'installed-package', p.package.files[0].path));
  writeFileSync(join(root, 'package', 'phantom.js'), 'phantom');
  assert.throws(() => verifyArtifact(root));
  rmSync(join(root, 'package', 'phantom.js'));
  writeFileSync(join(root, 'native-probe.exe'), pe(0x8664));
  assert.throws(() => verifyArtifact(root));
  writeFileSync(join(root, 'native-probe.exe'), native);
  const other = join(root, 'different-package'); mkdirSync(other);
  for (const file of p.package.files) copyFileSync(join(root, 'package', file.path), join(other, file.path));
  writeFileSync(join(other, p.package.files[1].path), 'different actual archive payload');
  pack(other); writeFileSync(join(root, 'proof.json'), JSON.stringify(p));
  assert.throws(() => verifyArtifact(root), /archive members/);
});

function tarFixture(entries) {
  const blocks = [];
  for (const [name, data = 'x', kind = '0'] of entries) {
    const header = Buffer.alloc(512), bytes = Buffer.from(data);
    header.write(name); header.write('0000644\0', 100); header.write('0000000\0', 108); header.write('0000000\0', 116);
    header.write(bytes.length.toString(8).padStart(11, '0') + '\0', 124); header.write('00000000000\0', 136);
    header.fill(32, 148, 156); header.write(kind, 156); header.write('ustar\0', 257); header.write('00', 263);
    const checksum = [...header].reduce((a, b) => a + b, 0);
    header.write(checksum.toString(8).padStart(6, '0') + '\0 ', 148);
    blocks.push(header, bytes, Buffer.alloc((512 - bytes.length % 512) % 512));
  }
  return Buffer.concat([...blocks, Buffer.alloc(1024)]);
}
test('bounded archive parser refuses malformed, unsafe and ambiguous members', () => {
  assert.equal(packageArchiveMembers(gzipSync(tarFixture([['package/file', 'payload']])))[0].path, 'file');
  for (const entries of [
    [['package/../escape']], [['/absolute']], [['package/back\\slash']], [['package/colon:path']],
    [['package/file', '', '2']], [['package/file', '', 'x']], [['package/file'], ['package/file']],
    [['package/Case'], ['package/case']], [['package/file'], ['package/file/child']],
    Array.from({ length: 65 }, (_, index) => ['package/file' + index]),
  ]) assert.throws(() => packageArchiveMembers(gzipSync(tarFixture(entries))));
  const corrupt = tarFixture([['package/file']]); corrupt[0] ^= 1;
  assert.throws(() => packageArchiveMembers(gzipSync(corrupt)), /checksum/);
  assert.throws(() => packageArchiveMembers(Buffer.from('not gzip')));
  assert.throws(() => packageArchiveMembers(gzipSync(Buffer.alloc(16 * 1024 * 1024 + 1))));
});

test('preflight failure retains an explicit failed receipt without invoking build tools', (t) => {
  const root = fixture(t), out = join(root, 'failure');
  const result = spawnSync(process.execPath, [join(here, 'windows-arm.mjs'), out], { env: { ...process.env, RUSTC_WRAPPER: 'intentionally-refused-wrapper' }, encoding: 'utf8', timeout: 15000 });
  assert.equal(result.status, 1, result.stderr);
  const proof = JSON.parse(readFileSync(join(out, 'proof.json'), 'utf8'));
  assert.equal(proof.passed, false); assert.ok(proof.error); assert.deepEqual(proof.commands, []);
  assert.throws(() => validateProof(proof));
});

test('workflow stays isolated, native ARM, pinned, bounded and read-only', () => {
  const text = readFileSync(join(here, '../../.github/workflows/windows-arm-tooling.yml'), 'utf8');
  validateWorkflowPolicy(text);
  for (const mutate of [
    (s) => s.replace('windows-11-arm', 'windows-2025'), (s) => s.replace('architecture: arm64', 'architecture: x64'),
    (s) => s.replace('contents: read', 'contents: write'), (s) => s.replace('1.96.0-aarch64-pc-windows-msvc', '1.96.0-x86_64-pc-windows-msvc'),
    (s) => s.replace('timeout-minutes: 25', 'timeout-minutes: 90'), (s) => s.replace('if: always()', 'if: success()'),
    (s) => s.replace('pull_request:', 'pull_request_target:'), (s) => s.replace("'scripts/platform-probes/**'", "'**'"),
    (s) => s.replace('persist-credentials: false', 'persist-credentials: true'),
    (s) => s.replace(',wasm-bindgen@0.2.126', ''), (s) => s.replace('wasm-bindgen@0.2.126', 'wasm-bindgen@0.2.100'),
    (s) => s.replace('checksum: true', 'checksum: false'), (s) => s.replace('fallback: none', 'fallback: cargo-install'),
    (s) => s.replace('1e29709c0ec63babbe8f266397a465f29a825f91', '0'.repeat(40)),
  ]) assert.throws(() => validateWorkflowPolicy(mutate(text)));
});
