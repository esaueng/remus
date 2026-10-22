#!/usr/bin/env node
// One portable driver owns the same bounded proof on every registered host.
import { spawnSync } from 'node:child_process';
import { appendFileSync, closeSync, mkdirSync, mkdtempSync, openSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { isAbsolute, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { prepareWindowsArmBuild, finishWindowsArmBuild } from '../tools/parity/host-proof.mjs';
import { validateCheckoutProof } from '../tools/parity/checkout-proof.mjs';

const repository = fileURLToPath(new URL('../', import.meta.url));
const unitTests = [
  'provenance', 'contract-matrix', 'split-io-matrix', 'host-proof', 'proof-verdict', 'npm-cli', 'proof-runner', 'checkout-proof',
].map((name) => `tools/parity/${name}.test.mjs`);

export function nativeExecutables(targetDirectory, platform = process.platform) {
  const suffix = platform === 'win32' ? '.exe' : '';
  return ['remus-parity-native', 'remus-split-io-native'].map((name) => resolve(targetDirectory, `release/${name}${suffix}`));
}

function status(result) {
  return !result.error && Number.isInteger(result.status) && result.status >= 0 ? result.status : 1;
}

/** Injectable subprocess execution keeps orchestration failure tests portable. */
export function runProof({ repoRoot = repository, env = process.env, temporaryRoot = tmpdir(), platform = process.platform, arch = process.arch, execute = spawnSync, prepareArm = prepareWindowsArmBuild, finishArm = finishWindowsArmBuild } = {}) {
  const proofRoot = resolve(repoRoot, env.O15_PROOF_OUT || 'target/o15-proof');
  mkdirSync(proofRoot, { recursive: true });
  const proofDir = mkdtempSync(resolve(proofRoot, 'run.'));
  const statuses = resolve(proofDir, 'status.tsv');
  writeFileSync(statuses, 'contract\tnot_run\nsplit-io\tnot_run\n');
  console.log(`O1.5 reports: ${proofDir}`);
  const config = resolve(proofDir, 'build-config.txt');
  writeFileSync(config, ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS', 'CARGO_BUILD_TARGET', 'CARGO_TARGET_DIR', 'CARGO_BUILD_JOBS', 'RUST_MIN_STACK']
    .map((key) => `${key}=${JSON.stringify(env[key] ?? '')}\n`).join(''));

  let executionEnv = env, armBuild, armReceipt;
  function invoke(argv, { output, capture = false, build = false } = {}) {
    if (build) appendFileSync(config, `${JSON.stringify(argv)}\n`);
    const record = build && armBuild ? { argv, environment: structuredClone(armBuild.environment), startedAt: new Date().toISOString() } : null;
    if (record) armBuild.buildCommands.push(record);
    const fd = output ? openSync(resolve(proofDir, output), 'w') : null;
    try {
      const result = execute(argv[0], argv.slice(1), {
        cwd: repoRoot, env: executionEnv, encoding: 'utf8', shell: false,
        stdio: fd !== null ? ['ignore', fd, 'inherit'] : capture ? ['ignore', 'pipe', 'inherit'] : 'inherit',
        maxBuffer: 16 * 1024 * 1024,
      });
      if (result.error) console.error(result.error.message);
      if (record) record.status = status(result);
      return { ...result, status: status(result) };
    } finally {
      if (record) { record.status ??= 1; record.completedAt = new Date().toISOString(); writeFileSync(armReceipt, JSON.stringify(armBuild, null, 2) + '\n'); }
      if (fd !== null) closeSync(fd);
    }
  }
  function requireSuccess(argv, options) {
    const result = invoke(argv, options);
    if (result.status !== 0) throw Object.assign(new Error(`proof command failed: ${JSON.stringify(argv)}`), { status: result.status });
    return result;
  }

  let freshRoot;
  try {
    if (platform === 'win32' && arch === 'arm64') {
      freshRoot ??= mkdtempSync(resolve(temporaryRoot, 'remus-o15-proof.'));
      armReceipt = resolve(proofDir, 'windows-arm-build.json');
      const prepared = prepareArm({ env, repoRoot, temporary: freshRoot, receipt: armReceipt });
      armBuild = prepared.proof; executionEnv = prepared.env;
      writeFileSync(armReceipt, JSON.stringify(armBuild, null, 2) + '\n');
      appendFileSync(config, `WINDOWS_ARM_ENV=${JSON.stringify(armBuild.environment)}\n`);
    }
    const cargo = armBuild?.cargo.path ?? 'cargo';
    const wasmPack = armBuild?.helpers.wasmPack.path ?? 'wasm-pack';
    requireSuccess([process.execPath, '--test', ...unitTests]);
    requireSuccess([cargo, 'test', '--locked', '--profile', 'ci-test', '-p', 'remus-parity'], { build: true });
    requireSuccess([cargo, 'build', '--locked', '--release', '-p', 'remus-parity'], { build: true });
    const metadata = requireSuccess([cargo, 'metadata', '--locked', '--no-deps', '--format-version', '1'], { capture: true });
    const target = JSON.parse(metadata.stdout).target_directory;
    if (typeof target !== 'string' || !isAbsolute(target)) throw new Error('cargo metadata omitted its absolute target directory');
    const [native, splitNative] = nativeExecutables(target, platform);
    requireSuccess([process.execPath, 'tools/parity/host-proof.mjs', native, splitNative], { output: 'host.json' });
    requireSuccess([process.execPath, 'tools/parity/checkout-proof.mjs'], { output: 'committed-checkout.json' });
    const checkout = JSON.parse(readFileSync(resolve(proofDir, 'committed-checkout.json'), 'utf8'));
    validateCheckoutProof(checkout);
    const hostProof = JSON.parse(readFileSync(resolve(proofDir, 'host.json'), 'utf8'));
    if (checkout.sourceRevision !== hostProof.sourceRevision) throw new Error('committed checkout revision differs from actual host proof');
    freshRoot ??= mkdtempSync(resolve(temporaryRoot, 'remus-o15-proof.'));
    const kernel = resolve(freshRoot, 'kernel');
    const io = resolve(freshRoot, 'io');
    requireSuccess([wasmPack, 'build', 'crates/wasm', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', kernel, '--', '--locked', '--no-default-features'], { build: true });
    requireSuccess([wasmPack, 'build', 'crates/wasm-io', '--target', 'nodejs', '--release', '--no-opt', '--out-dir', io, '--', '--locked'], { build: true });
    if (armBuild) {
      finishArm({ proof: armBuild, env: executionEnv, repoRoot, receipt: armReceipt });
      requireSuccess([process.execPath, 'tools/parity/host-proof.mjs', native, splitNative], { output: 'host.json' });
    }
    const contract = invoke([process.execPath, 'tools/parity/contract-matrix.mjs', native, kernel, 'crates/wasm/pkg'], { output: 'o15-contract.json' });
    // A failed contract must still leave independent translator evidence.
    const split = invoke([process.execPath, 'tools/parity/split-io-matrix.mjs', splitNative, kernel, io, 'crates/wasm/pkg', 'crates/wasm-io/pkg'], { output: 'o15-split-io.json' });
    const summary = `contract\t${contract.status}\nsplit-io\t${split.status}\n`;
    writeFileSync(statuses, summary);
    console.log(summary.trimEnd());
    const verdict = invoke([process.execPath, 'tools/parity/proof-verdict.mjs', resolve(proofDir, 'o15-contract.json'), resolve(proofDir, 'o15-split-io.json'), resolve(proofDir, 'host.json')], { output: 'verdict.json' });
    return { status: contract.status || split.status || verdict.status ? 1 : 0, proofDir };
  } catch (error) {
    if (armBuild) {
      armBuild.passed = false; armBuild.state = 'failed'; armBuild.error = error.message;
      writeFileSync(armReceipt, JSON.stringify(armBuild, null, 2) + '\n');
    }
    console.error(error.message);
    return { status: Number.isInteger(error.status) ? error.status : 1, proofDir };
  } finally {
    if (freshRoot) rmSync(freshRoot, { recursive: true, force: true, maxRetries: 3 });
  }
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  if (process.argv.length !== 2) {
    console.error('usage: test-o15-proof.mjs (optional O15_PROOF_OUT output directory)');
    process.exitCode = 2;
  } else {
    try { process.exitCode = runProof().status; }
    catch (error) { console.error(error.message); process.exitCode = 1; }
  }
}
