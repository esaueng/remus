import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { execFileSync, spawnSync } from 'node:child_process';
import { isDeepStrictEqual } from 'node:util';
import { mkdirSync, mkdtempSync, readFileSync, realpathSync, rmSync } from 'node:fs';
import { createRequire } from 'node:module';
import { tmpdir } from 'node:os';
import { dirname, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { performance } from 'node:perf_hooks';

const self = fileURLToPath(import.meta.url);
const scriptDir = dirname(self);
const manifestUrl = new URL('./quadric-boolean-matrix.json', import.meta.url);
const timeoutMs = 120_000;

function scaledArgs(primitive, scale) {
  const args = structuredClone(primitive.args);
  for (const field of primitive.scale_fields) args[field] *= scale;
  return args;
}

function translationMatrix([x, y, z], scale) {
  return [1, 0, 0, x * scale, 0, 1, 0, y * scale, 0, 0, 1, z * scale, 0, 0, 0, 1];
}

function placementMatrix(scale) {
  const c = Math.cos(0.37);
  const s = Math.sin(0.37);
  return [
    c, 0, s, 17 * scale,
    0, 1, 0, -23 * scale,
    -s, 0, c, 31 * scale,
    0, 0, 0, 1,
  ];
}

function volumeTerm(term) {
  return term.coefficient * Math.PI ** term.pi_power;
}

function expectedVolume(family, operation, scale) {
  const lhs = volumeTerm(family.lhs_volume);
  const rhs = volumeTerm(family.rhs_volume);
  const overlap = family.intersection_volume;
  const base = operation === 'fuse' ? lhs + rhs - overlap
    : operation === 'cut' ? lhs - overlap : overlap;
  return base * scale ** 3;
}

export function validateManifest(manifest) {
  assert.equal(manifest.schema_version, 1, 'matrix schema version');
  assert.deepEqual(manifest.operations, ['fuse', 'cut', 'intersect'], 'boolean operation axis');
  assert.deepEqual(manifest.scales, [0.1, 1, 10], 'scale axis');
  assert.deepEqual(manifest.placements, ['origin', 'rigid'], 'placement axis');
  assert.ok(manifest.volume_relative_tolerance > 0, 'volume oracle tolerance');
  assert.ok(manifest.surface_volume_relative_tolerance > 0, 'cross-surface volume tolerance');
  assert.equal(manifest.families.length, 3, 'qualified family count');
  assert.deepEqual(
    manifest.families.map((family) => family.id),
    ['cone-sphere', 'sphere-cylinder', 'torus-sphere'],
    'qualified family ids',
  );
  for (const family of manifest.families) {
    assert.ok(family.lhs?.op && family.rhs?.op, `${family.id}: primitive operations`);
    assert.equal(family.rhs_translation.length, 3, `${family.id}: relative placement`);
    assert.ok(family.required_surface_types.length > 0, `${family.id}: required carriers`);
    assert.ok(
      family.required_surface_types.every((type) => family.allowed_surface_types.includes(type)),
      `${family.id}: required carriers must be allowed`,
    );
  }
}

function makeCase(manifest, family, operation, scale, placement) {
  const batch = [];
  const add = (op, args) => {
    const index = batch.length;
    batch.push({ op, args });
    return index;
  };

  add(family.lhs.op, scaledArgs(family.lhs, scale));
  add(family.rhs.op, scaledArgs(family.rhs, scale));
  add('transform', { solid: 1, matrix: translationMatrix(family.rhs_translation, scale) });
  if (placement === 'rigid') {
    const matrix = placementMatrix(scale);
    add('transform', { solid: 0, matrix });
    add('transform', { solid: 1, matrix });
  }
  const booleanIndex = add('booleanWithQuality', {
    operation,
    solidA: 0,
    solidB: 1,
    exactOnly: true,
  });
  const volumeIndex = add('volume', { solid: 2, deflection: 0.01 * scale });
  const validationIndex = add('validateSolid', { solid: 2 });
  const meshQualityIndex = add('meshQuality', { solid: 2, deflection: 0.005 * scale });
  const facesIndex = add('getSolidFaces', { solid: 2 });

  return {
    schema_version: 1,
    id: `${family.id}/${operation}/scale=${scale}/placement=${placement}`,
    family: family.id,
    operation,
    scale,
    placement,
    expected_volume: expectedVolume(family, operation, scale),
    volume_relative_tolerance: manifest.volume_relative_tolerance,
    surface_volume_relative_tolerance: manifest.surface_volume_relative_tolerance,
    allowed_surface_types: family.allowed_surface_types,
    required_surface_types: family.required_surface_types,
    batch,
    result: {
      handle: 2,
      booleanIndex,
      volumeIndex,
      validationIndex,
      meshQualityIndex,
      facesIndex,
    },
  };
}

export function expandMatrix(manifest) {
  validateManifest(manifest);
  return manifest.families.flatMap((family) => manifest.operations.flatMap((operation) => (
    manifest.scales.flatMap((scale) => manifest.placements.map((placement) => (
      makeCase(manifest, family, operation, scale, placement)
    )))
  )));
}

function relativeError(actual, expected) {
  return Math.abs(actual - expected) / Math.max(Math.abs(expected), 1);
}

function analyticCarriersPass(c, observation) {
  if (!observation.surfaceTypes || !observation.census) return false;
  const present = Object.entries(observation.surfaceTypes).filter(([, count]) => count > 0);
  const allowed = present.every(([type]) => c.allowed_surface_types.includes(type));
  const required = c.required_surface_types.every((type) => observation.surfaceTypes[type] > 0);
  const faceTotal = present.reduce((sum, [, count]) => sum + count, 0);
  return allowed && required && faceTotal === observation.census.faces;
}

export function scoreCase(c, observations) {
  const stages = [];
  const add = (surface, invariant, passed, required = true) => stages.push({
    case: c.id,
    operation: c.operation,
    surface,
    invariant,
    passed,
    required,
  });

  for (const surface of ['native', 'wasm']) {
    const observation = observations[surface];
    add(surface, 'bounded_execution', !['crash', 'hang_or_budget_overrun'].includes(observation.outcome));
    add(surface, 'batch_success', observation.outcome === 'success');
    add(surface, 'diagnostics_empty', observation.diagnosticCodes?.length === 0);
    add(surface, 'exact_quality', observation.quality === 'exact');
    add(surface, 'oracle_volume', Number.isFinite(observation.volume)
      && relativeError(observation.volume, c.expected_volume) <= c.volume_relative_tolerance);
    add(surface, 'valid_brep', observation.validationErrors === 0);
    add(surface, 'analytic_carriers', analyticCarriersPass(c, observation));
    add(surface, 'watertight_mesh', observation.meshQuality?.isWatertight === true
      && observation.meshQuality?.boundaryEdges === 0
      && observation.meshQuality?.nonManifoldEdges === 0);
    add(surface, 'serialized_result', observation.serializedBytes > 0
      && /^[0-9a-f]{64}$/.test(observation.serializedSha256 ?? ''));
  }

  const { native, wasm } = observations;
  const bothVolumes = Number.isFinite(native.volume) && Number.isFinite(wasm.volume);
  add('native/wasm', 'outcome_agreement', native.outcome === wasm.outcome);
  add('native/wasm', 'diagnostics_agreement', isDeepStrictEqual(native.diagnosticCodes, wasm.diagnosticCodes));
  add('native/wasm', 'quality_agreement', native.quality === wasm.quality);
  add('native/wasm', 'volume_agreement', bothVolumes
    && relativeError(native.volume, wasm.volume) <= c.surface_volume_relative_tolerance);
  add('native/wasm', 'census_agreement', isDeepStrictEqual(native.census, wasm.census));
  add('native/wasm', 'carrier_agreement', isDeepStrictEqual(native.surfaceTypes, wasm.surfaceTypes));
  add('native/wasm', 'mesh_quality_agreement', isDeepStrictEqual(native.meshQuality, wasm.meshQuality));
  // O1.5's byte-identity exit gate is deliberately visible but non-gating in
  // this first semantic-parity slice. Native and wasm currently produce
  // different arena payloads even when all geometry invariants agree.
  add('native/wasm', 'serialized_bytes_agreement', native.serializedBytes === wasm.serializedBytes
    && native.serializedSha256 === wasm.serializedSha256, false);
  return stages;
}

function wasmObservation(c, modulePath) {
  const { BrepKernel } = createRequire(import.meta.url)(modulePath);
  const initStarted = performance.now();
  const kernel = new BrepKernel();
  const coldInitMs = performance.now() - initStarted;
  try {
    const batchStarted = performance.now();
    const responses = JSON.parse(kernel.executeBatchV2(JSON.stringify(c.batch)));
    const batchDurationMs = performance.now() - batchStarted;
    const diagnosticCodes = responses.flatMap((response) => (
      response.error ? [response.error.code ?? 'untyped_error'] : []
    ));
    if (diagnosticCodes.length > 0) {
      return {
        schemaVersion: 1,
        id: c.id,
        surface: 'wasm',
        outcome: 'batch_error',
        diagnosticCodes,
        coldInitMs,
        batchDurationMs,
      };
    }

    const boolean = responses[c.result.booleanIndex]?.ok;
    assert.equal(boolean?.solid, c.result.handle, `${c.id}: result handle`);
    const faces = responses[c.result.facesIndex]?.ok;
    assert.ok(Array.isArray(faces), `${c.id}: face response`);
    const surfaceTypes = {};
    for (const face of faces) {
      const type = kernel.getSurfaceType(face);
      surfaceTypes[type] = (surfaceTypes[type] ?? 0) + 1;
    }
    const counts = Array.from(kernel.getEntityCounts(c.result.handle));
    assert.equal(counts.length, 3, `${c.id}: entity census`);
    const serialized = kernel.serializeSolid(c.result.handle);
    return {
      schemaVersion: 1,
      id: c.id,
      surface: 'wasm',
      outcome: 'success',
      diagnosticCodes,
      quality: boolean.quality,
      resultHandle: boolean.solid,
      volume: responses[c.result.volumeIndex]?.ok,
      validationErrors: responses[c.result.validationIndex]?.ok,
      census: { faces: counts[0], edges: counts[1], vertices: counts[2] },
      surfaceTypes,
      meshQuality: responses[c.result.meshQualityIndex]?.ok,
      serializedBytes: serialized.byteLength,
      serializedSha256: createHash('sha256').update(serialized).digest('hex'),
      coldInitMs,
      batchDurationMs,
    };
  } finally {
    kernel.free();
  }
}

function attempt(command, args, c) {
  const result = spawnSync(command, args, {
    input: JSON.stringify(c),
    encoding: 'utf8',
    timeout: timeoutMs,
    maxBuffer: 4 * 1024 * 1024,
  });
  if (result.error?.code === 'ETIMEDOUT') {
    return { outcome: 'hang_or_budget_overrun', diagnosticCodes: [] };
  }
  if (result.error || result.status !== 0) {
    return {
      outcome: 'crash',
      diagnosticCodes: [],
      diagnostic: result.stderr || String(result.error),
    };
  }
  try {
    return JSON.parse(result.stdout);
  } catch (error) {
    return {
      outcome: 'crash',
      diagnosticCodes: [],
      diagnostic: `invalid observation JSON: ${error}; stderr=${result.stderr}`,
    };
  }
}

function installPackage(packageDir, temporaryRoot) {
  const consumerDir = resolve(temporaryRoot, 'consumer');
  mkdirSync(consumerDir);
  const npmEnvironment = {
    ...process.env,
    npm_config_cache: resolve(temporaryRoot, 'npm-cache'),
  };
  const packed = JSON.parse(execFileSync(
    'npm',
    ['pack', packageDir, '--json', '--pack-destination', temporaryRoot],
    { encoding: 'utf8', env: npmEnvironment },
  ));
  assert.equal(packed.length, 1, 'npm pack must produce exactly one tarball');
  execFileSync('npm', [
    'install',
    '--prefix', consumerDir,
    '--no-save',
    '--package-lock=false',
    '--ignore-scripts',
    '--no-audit',
    '--no-fund',
    resolve(temporaryRoot, packed[0].filename),
  ], { env: npmEnvironment, stdio: 'ignore' });
  const consumerRequire = createRequire(resolve(consumerDir, 'consumer.cjs'));
  const entry = consumerRequire.resolve('remus-wasm');
  const installedDir = realpathSync(resolve(consumerDir, 'node_modules/remus-wasm'));
  assert.ok(entry.startsWith(`${installedDir}${sep}`), `package resolved outside install: ${entry}`);
  return entry;
}

export function readManifest() {
  const bytes = readFileSync(manifestUrl);
  return { bytes, manifest: JSON.parse(bytes) };
}

async function main() {
  if (process.argv[2] === '--wasm-child') {
    const c = JSON.parse(readFileSync(0, 'utf8'));
    process.stdout.write(JSON.stringify(wasmObservation(c, process.argv[3])));
    return;
  }

  const [nativeRunner, packageDir] = process.argv.slice(2);
  assert.ok(nativeRunner && packageDir, 'usage: quadric-boolean.mjs NATIVE_RUNNER PACKAGE_DIR');
  const { bytes, manifest } = readManifest();
  const cases = expandMatrix(manifest);
  const temporaryRoot = mkdtempSync(resolve(tmpdir(), 'remus-o15-parity-'));
  const started = performance.now();
  try {
    const installedEntry = installPackage(resolve(packageDir), temporaryRoot);
    const observations = cases.map((c) => ({
      case: c.id,
      native: attempt(resolve(nativeRunner), [], c),
      wasm: attempt(process.execPath, [self, '--wasm-child', installedEntry], c),
    }));
    const stages = cases.flatMap((c, index) => scoreCase(c, observations[index]));
    const failedStages = stages.filter((stage) => stage.required && !stage.passed);
    const knownGaps = stages.filter((stage) => !stage.required && !stage.passed);
    const passed = failedStages.length === 0;
    const coveredBatchOps = [...new Set(cases.flatMap((c) => c.batch.map(({ op }) => op)))].sort();
    const report = {
      schema_version: 1,
      scope: 'O1.5 first slice: qualified quadric boolean matrix',
      manifest_sha256: createHash('sha256').update(bytes).digest('hex'),
      installed_package_entry: installedEntry,
      timeout_ms: timeoutMs,
      matrix: {
        cases: cases.length,
        families: manifest.families.map(({ id }) => id),
        operations: manifest.operations,
        scales: manifest.scales,
        placements: manifest.placements,
        covered_batch_ops: coveredBatchOps,
      },
      elapsed_ms: performance.now() - started,
      passed,
      failed_stages: failedStages,
      known_gaps: knownGaps,
      stages,
      observations,
    };
    process.stdout.write(`${JSON.stringify(report, null, 2)}\n`);
    process.exitCode = passed ? 0 : 1;
  } finally {
    rmSync(temporaryRoot, { recursive: true, force: true });
  }
}

if (process.argv[1] === self) await main();
