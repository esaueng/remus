import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { existsSync, readFileSync, readdirSync } from 'node:fs';
import { resolve, sep } from 'node:path';

function runCapture(command, args, cwd) {
  try {
    return execFileSync(command, args, { encoding: 'utf8', cwd }).trim();
  } catch {
    return null;
  }
}

function sha256Hex(bytes) {
  return createHash('sha256').update(bytes).digest('hex');
}

export function validateInstalledEntry(entry, installedDir) {
  assert.ok(entry, 'installed package entry is missing');
  assert.ok(installedDir, 'installed package dir is missing');
  assert.ok(
    entry.startsWith(`${installedDir}${sep}`),
    `package resolved outside install: ${entry}`,
  );
  return true;
}

export function validateProvenance(provenance) {
  assert.ok(provenance && typeof provenance === 'object', 'provenance is missing');
  for (const field of [
    'mode',
    'sourceRevision',
    'sourceDirty',
    'toolchain',
    'buildOptions',
    'packageVersion',
    'wasmSha256',
    'installedEntry',
  ]) {
    assert.ok(
      provenance[field] !== undefined && provenance[field] !== null && provenance[field] !== '',
      `provenance is missing required field: ${field}`,
    );
  }
  assert.ok(
    ['native', 'fresh-tarball', 'committed-package'].includes(provenance.mode),
    `unknown provenance mode: ${provenance.mode}`,
  );
  // The tarball hash is required for freshly packed artifacts. Committed
  // package directories were not packed by this harness run, so they carry
  // no tarball hash by construction.
  if (provenance.mode === 'fresh-tarball') {
    assert.ok(
      provenance.tarballSha256 && /^[0-9a-f]{64}$/.test(provenance.tarballSha256),
      'fresh-tarball provenance is missing its tarball hash',
    );
  }
  assert.ok(
    /^[0-9a-f]{64}$/.test(provenance.wasmSha256),
    'provenance wasm hash is not a SHA-256 hex digest',
  );
  return true;
}

export function collectSourceProvenance(repoRoot) {
  const root = resolve(repoRoot);
  const sourceRevision = runCapture('git', ['rev-parse', 'HEAD'], root);
  assert.ok(sourceRevision, 'cannot determine source revision (git rev-parse HEAD)');
  const status = runCapture('git', ['status', '--porcelain=v1'], root) ?? '';
  const sourceDirty = status.trim().length > 0;
  const toolchain = {
    rustc: runCapture('rustc', ['--version'], root),
    cargo: runCapture('cargo', ['--version'], root),
    node: runCapture('node', ['--version'], root),
    wasmPack: runCapture('wasm-pack', ['--version'], root),
  };
  return { sourceRevision, sourceDirty, toolchain };
}

export function readPackageVersion(packageDir) {
  const pkgJson = JSON.parse(readFileSync(resolve(packageDir, 'package.json'), 'utf8'));
  assert.ok(pkgJson.version, `${packageDir}/package.json has no version`);
  return String(pkgJson.version);
}

export function findWasmArtifact(packageDir) {
  const dir = resolve(packageDir);
  const entries = readdirSync(dir);
  const wasmFile = entries.find((name) => name.endsWith('_bg.wasm'));
  assert.ok(wasmFile, `no *_bg.wasm artifact in ${dir}`);
  return resolve(dir, wasmFile);
}

export function hashFile(path) {
  return sha256Hex(readFileSync(path));
}

export function collectFreshProvenance({
  repoRoot,
  packageDir,
  tarballPath,
  installedEntry,
  installedDir,
  buildOptions,
}) {
  validateInstalledEntry(installedEntry, installedDir);
  const source = collectSourceProvenance(repoRoot);
  const packageVersion = readPackageVersion(packageDir);
  const wasmFile = findWasmArtifact(packageDir);
  const wasmSha256 = hashFile(wasmFile);
  assert.ok(existsSync(tarballPath), `packed tarball is missing: ${tarballPath}`);
  const tarballSha256 = hashFile(tarballPath);
  const provenance = {
    mode: 'fresh-tarball',
    sourceRevision: source.sourceRevision,
    sourceDirty: source.sourceDirty,
    toolchain: source.toolchain,
    target: buildOptions?.target ?? 'wasm32-unknown-unknown',
    buildOptions: buildOptions ?? {},
    packageVersion,
    packageDir: resolve(packageDir),
    wasmFile,
    wasmSha256,
    tarballPath: resolve(tarballPath),
    tarballSha256,
    installedEntry,
    installedDir,
    // A freshly packed tarball is built from this source tree, so the
    // source revision applies to it. It is still recorded as a claim with
    // its dirty flag, never as an unconditional identity.
    sourceRelationship: source.sourceDirty
      ? 'built from a dirty source tree; revision is the base commit, not a clean identity'
      : 'built from the recorded source revision',
  };
  validateProvenance(provenance);
  return provenance;
}

export function collectCommittedProvenance({
  repoRoot,
  packageDir,
  installedEntry,
  installedDir,
}) {
  validateInstalledEntry(installedEntry, installedDir);
  const source = collectSourceProvenance(repoRoot);
  const packageVersion = readPackageVersion(packageDir);
  const wasmFile = findWasmArtifact(packageDir);
  const wasmSha256 = hashFile(wasmFile);
  const provenance = {
    mode: 'committed-package',
    // The current source SHA is recorded only as the harness baseline the
    // committed bytes are compared against, never as the revision those
    // bytes were built from. Attribution requires a rebuild.
    sourceRevision: source.sourceRevision,
    sourceDirty: source.sourceDirty,
    toolchain: source.toolchain,
    target: 'wasm32-unknown-unknown',
    buildOptions: { origin: 'committed bytes; build options unknown' },
    packageVersion,
    packageDir: resolve(packageDir),
    wasmFile,
    wasmSha256,
    tarballPath: null,
    tarballSha256: null,
    installedEntry,
    installedDir,
    sourceRelationship:
      'unverified — committed/distributed bytes; never label these results with the current source SHA',
  };
  validateProvenance(provenance);
  return provenance;
}

export function describeStaleness(freshProvenance, committedProvenance) {
  assert.ok(freshProvenance && committedProvenance, 'both provenances are required');
  const versionMatch = freshProvenance.packageVersion === committedProvenance.packageVersion;
  const wasmMatch = freshProvenance.wasmSha256 === committedProvenance.wasmSha256;
  const stale = !versionMatch || !wasmMatch;
  return {
    versionMatch,
    wasmMatch,
    stale,
    verdict: stale
      ? 'stale/mismatched: committed bytes differ from the fresh source build; do not attribute committed results to the current source revision'
      : 'matching: committed bytes are byte-identical to the fresh source build for the compared artifacts',
    fresh: {
      packageVersion: freshProvenance.packageVersion,
      wasmSha256: freshProvenance.wasmSha256,
    },
    committed: {
      packageVersion: committedProvenance.packageVersion,
      wasmSha256: committedProvenance.wasmSha256,
    },
  };
}

export function requireObservations(caseId, observations) {
  assert.ok(observations, `${caseId}: observations are missing`);
  for (const surface of ['native', 'fresh', 'committed']) {
    if (observations[surface] === undefined) continue;
    assert.ok(
      observations[surface] && typeof observations[surface] === 'object',
      `${caseId}: ${surface} observation is missing`,
    );
    assert.ok(
      observations[surface].outcome,
      `${caseId}: ${surface} observation has no outcome`,
    );
  }
  return true;
}
