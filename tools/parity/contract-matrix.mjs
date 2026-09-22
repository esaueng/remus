import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { execFileSync, spawnSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, realpathSync, rmSync } from 'node:fs';
import { createRequire } from 'node:module';
import { tmpdir } from 'node:os';
import { dirname, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { performance } from 'node:perf_hooks';
import {
  collectCommittedProvenance,
  collectFreshProvenance,
  collectSourceProvenance,
  describeStaleness,
  requireObservations,
  validateInstalledEntry,
  validateProvenance,
} from './provenance.mjs';

const self = fileURLToPath(import.meta.url);
const scriptDir = dirname(self);
const repoRoot = resolve(scriptDir, '..', '..');
const timeoutMs = 120_000;

function translationMatrix([x, y, z]) {
  return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
}

function relativeError(actual, expected) {
  return Math.abs(actual - expected) / Math.max(Math.abs(expected), 1);
}

// Two overlapping 2x2x2 boxes offset by (1,1,1): union volume is exactly 15.
// The trailing validate/mesh/faces ops mirror the geometric harness shape
// so the same native facade runner serves contract success cases.
function overlappingBoxesBatch({ exactOnly = false } = {}) {
  const batch = [
    { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
    { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
    { op: 'transform', args: { solid: 1, matrix: translationMatrix([1, 1, 1]) } },
    {
      op: 'booleanWithQuality',
      args: { operation: 'fuse', solidA: 0, solidB: 1, ...(exactOnly ? { exactOnly: true } : {}) },
    },
    { op: 'volume', args: { solid: 2, deflection: 0.05 } },
    { op: 'validateSolid', args: { solid: 2 } },
    { op: 'meshQuality', args: { solid: 2, deflection: 0.05 } },
    { op: 'getSolidFaces', args: { solid: 2 } },
  ];
  return { batch, booleanIndex: 3, volumeIndex: 4, validationIndex: 5, meshQualityIndex: 6, facesIndex: 7 };
}

// B21 tangent-boss fixture: block 60x40x8 plus cylinder r10 h16 at
// (9.999,20,0). The exact pipeline cannot assemble it, so the default
// policy discloses `approximate` and `exactOnly` refuses typed.
function tangentBossBatch({ exactOnly = false } = {}) {
  const batch = [
    { op: 'makeBox', args: { width: 60, height: 40, depth: 8 } },
    { op: 'makeCylinder', args: { radius: 10, height: 16 } },
    { op: 'transform', args: { solid: 1, matrix: translationMatrix([9.999, 20, 0]) } },
    {
      op: 'booleanWithQuality',
      args: { operation: 'fuse', solidA: 0, solidB: 1, ...(exactOnly ? { exactOnly: true } : {}) },
    },
    { op: 'volume', args: { solid: 2, deflection: 0.05 } },
    { op: 'validateSolid', args: { solid: 2 } },
    { op: 'meshQuality', args: { solid: 2, deflection: 0.05 } },
    { op: 'getSolidFaces', args: { solid: 2 } },
  ];
  return { batch, booleanIndex: 3, volumeIndex: 4, validationIndex: 5, meshQualityIndex: 6, facesIndex: 7 };
}

// Same overlapping 2x2x2 boxes through the `*WithEvolution` batch ops: the
// exact-only boolean whose response carries a construction-derived
// evolution report (`modified`/`generated`/`deleted`/`unresolved`/`origin`,
// the `EvolutionMap::to_json` shape both surfaces emit verbatim).
function evolutionBoxesBatch(op) {
  const batch = [
    { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
    { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
    { op: 'transform', args: { solid: 1, matrix: translationMatrix([1, 1, 1]) } },
    { op, args: { solidA: 0, solidB: 1 } },
    { op: 'volume', args: { solid: 2, deflection: 0.05 } },
    { op: 'validateSolid', args: { solid: 2 } },
    { op: 'meshQuality', args: { solid: 2, deflection: 0.05 } },
    { op: 'getSolidFaces', args: { solid: 2 } },
  ];
  return {
    batch,
    booleanIndex: 3,
    evolutionIndex: 3,
    volumeIndex: 4,
    validationIndex: 5,
    meshQualityIndex: 6,
    facesIndex: 7,
  };
}

// Structural summary of one evolution report: bucket sizes plus the origin
// tag. Handle indices are an arena detail, so the gated comparison is over
// this summary and the unresolved bucket; the raw report still rides along
// as non-gating evidence.
export function summarizeEvolution(evolution) {
  if (!evolution || typeof evolution !== 'object') return null;
  const sizeOf = (bucket) => Object.keys(bucket ?? {}).length;
  const outputsOf = (bucket) => Object.values(bucket ?? {})
    .reduce((total, outputs) => total + (Array.isArray(outputs) ? outputs.length : 0), 0);
  return {
    modifiedInputs: sizeOf(evolution.modified),
    modifiedOutputs: outputsOf(evolution.modified),
    generatedInputs: sizeOf(evolution.generated),
    generatedOutputs: outputsOf(evolution.generated),
    deleted: Array.isArray(evolution.deleted) ? evolution.deleted.length : 0,
    unresolved: sizeOf(evolution.unresolved),
    origin: evolution.origin ?? null,
  };
}

function canonicalJson(value) {
  if (Array.isArray(value)) return `[${value.map(canonicalJson).join(',')}]`;
  if (value && typeof value === 'object') {
    return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${canonicalJson(value[key])}`).join(',')}}`;
  }
  return JSON.stringify(value);
}

export function expandContractMatrix() {
  const exact = overlappingBoxesBatch();
  const refusal = tangentBossBatch({ exactOnly: true });
  const approx = tangentBossBatch();
  const evolutionFuse = evolutionBoxesBatch('fuseWithEvolution');
  const evolutionCut = evolutionBoxesBatch('cutWithEvolution');
  return [
    {
      id: 'contract/exact-fuse',
      title: 'exact result',
      description: 'overlapping boxes fuse exactly; volume 15, quality exact',
      batch: exact.batch,
      booleanIndex: exact.booleanIndex,
      volumeIndex: exact.volumeIndex,
      validationIndex: exact.validationIndex,
      meshQualityIndex: exact.meshQualityIndex,
      facesIndex: exact.facesIndex,
      expect: { outcome: 'success', quality: 'exact', volume: 15, volumeTolerance: 1e-6 },
    },
    {
      id: 'contract/exact-only-refusal',
      title: 'exact-only refusal',
      description: 'tangent boss with exactOnly refuses typed instead of approximating',
      batch: refusal.batch,
      booleanIndex: refusal.booleanIndex,
      volumeIndex: refusal.volumeIndex,
      validationIndex: refusal.validationIndex,
      meshQualityIndex: refusal.meshQualityIndex,
      facesIndex: refusal.facesIndex,
      expect: { outcome: 'batch_error', code: 'operation_failed' },
    },
    {
      id: 'contract/disclosed-approximation',
      title: 'opted-in disclosed approximation',
      description: 'same tangent boss without exactOnly discloses approximate quality',
      batch: approx.batch,
      booleanIndex: approx.booleanIndex,
      volumeIndex: approx.volumeIndex,
      validationIndex: approx.validationIndex,
      meshQualityIndex: approx.meshQualityIndex,
      facesIndex: approx.facesIndex,
      expect: { outcome: 'success', quality: 'approximate' },
    },
    {
      id: 'contract/invalid-handle',
      title: 'invalid handle',
      description: 'volume on a stale solid handle refuses with invalid_handle',
      batch: [
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'volume', args: { solid: 9999, deflection: 0.05 } },
      ],
      booleanIndex: null,
      volumeIndex: 1,
      facesIndex: null,
      expect: { outcome: 'batch_error', code: 'invalid_handle' },
    },
    {
      id: 'contract/transactional-rollback',
      title: 'transactional rollback',
      description: 'a failing op mid-batch leaves earlier solids measurable by later ops',
      batch: [
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'volume', args: { solid: 9999, deflection: 0.05 } },
        { op: 'volume', args: { solid: 0, deflection: 0.05 } },
        { op: 'getSolidFaces', args: { solid: 0 } },
      ],
      booleanIndex: null,
      volumeIndex: 2,
      facesIndex: 3,
      expect: {
        outcome: 'batch_error',
        code: 'invalid_handle',
        rollbackVolume: 8,
        rollbackVolumeTolerance: 1e-9,
        rollbackFaces: 6,
      },
    },
    {
      id: 'contract/cancelled-boolean',
      title: 'supported cancellation (pre-cancelled token, direct-only)',
      description:
        'pre-cancelled cooperative token refuses without touching topology; '
        + 'batch has no cancellation op, and a synchronous WASM call cannot '
        + 'process a later JS cancellation message on the same thread',
      batch: [
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'transform', args: { solid: 1, matrix: translationMatrix([1, 1, 1]) } },
        { op: 'booleanWithCancelledContext', args: { operation: 'fuse', solidA: 0, solidB: 1 } },
        { op: 'volume', args: { solid: 0, deflection: 0.05 } },
      ],
      booleanIndex: 3,
      volumeIndex: 4,
      facesIndex: null,
      expect: {
        outcome: 'batch_error',
        code: 'cancelled',
        rollbackVolume: 8,
        rollbackVolumeTolerance: 1e-9,
      },
      directOnly: false,
      cancellationScope:
        'pre-cancelled token only; concurrent mid-call cancellation needs a worker/shared-memory transport',
    },
    {
      id: 'contract/evolution-fuse',
      title: 'evolution report (fuse)',
      description:
        'overlapping boxes fuseWithEvolution: every input face is modified 1:1, '
        + 'nothing generated, deleted, or unresolved; volume 15, 12 faces',
      batch: evolutionFuse.batch,
      booleanIndex: evolutionFuse.booleanIndex,
      evolutionIndex: evolutionFuse.evolutionIndex,
      volumeIndex: evolutionFuse.volumeIndex,
      validationIndex: evolutionFuse.validationIndex,
      meshQualityIndex: evolutionFuse.meshQualityIndex,
      facesIndex: evolutionFuse.facesIndex,
      expect: {
        outcome: 'success',
        quality: 'exact',
        volume: 15,
        volumeTolerance: 1e-6,
        faces: 12,
        evolution: {
          modifiedInputs: 12,
          modifiedOutputs: 12,
          generatedInputs: 0,
          generatedOutputs: 0,
          deleted: 0,
          unresolved: 0,
          origin: 'construction',
        },
        unresolvedBucket: {},
      },
    },
    {
      id: 'contract/evolution-cut',
      title: 'evolution report (cut)',
      description:
        'overlapping boxes cutWithEvolution: the three tool faces outside the '
        + 'blank are deleted, nine faces modified 1:1, nothing generated or '
        + 'unresolved; volume 7, 9 faces',
      batch: evolutionCut.batch,
      booleanIndex: evolutionCut.booleanIndex,
      evolutionIndex: evolutionCut.evolutionIndex,
      volumeIndex: evolutionCut.volumeIndex,
      validationIndex: evolutionCut.validationIndex,
      meshQualityIndex: evolutionCut.meshQualityIndex,
      facesIndex: evolutionCut.facesIndex,
      expect: {
        outcome: 'success',
        quality: 'exact',
        volume: 7,
        volumeTolerance: 1e-6,
        faces: 9,
        evolution: {
          modifiedInputs: 9,
          modifiedOutputs: 9,
          generatedInputs: 0,
          generatedOutputs: 0,
          deleted: 3,
          unresolved: 0,
          origin: 'construction',
        },
        unresolvedBucket: {},
      },
    },
    {
      id: 'contract/invalid-primitive-input',
      title: 'invalid primitive input',
      description:
        'makeBox with a negative width refuses invalid_argument and leaves the '
        + 'earlier box measurable',
      batch: [
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'makeBox', args: { width: -1, height: 2, depth: 2 } },
        { op: 'volume', args: { solid: 0, deflection: 0.05 } },
        { op: 'getSolidFaces', args: { solid: 0 } },
      ],
      booleanIndex: null,
      volumeIndex: 2,
      facesIndex: 3,
      expect: {
        outcome: 'batch_error',
        code: 'invalid_argument',
        rollbackVolume: 8,
        rollbackVolumeTolerance: 1e-9,
        rollbackFaces: 6,
      },
    },
    {
      id: 'contract/empty-intersect-sentinel',
      title: 'empty intersect sentinel',
      description:
        'intersect of two far-apart boxes succeeds with the typed empty-result '
        + 'sentinel (exact quality, volume 0, zero faces) and leaves the operand '
        + 'measurable',
      batch: [
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
        { op: 'transform', args: { solid: 1, matrix: translationMatrix([10, 10, 10]) } },
        { op: 'booleanWithQuality', args: { operation: 'intersect', solidA: 0, solidB: 1 } },
        { op: 'volume', args: { solid: 2, deflection: 0.05 } },
        { op: 'validateSolid', args: { solid: 2 } },
        { op: 'meshQuality', args: { solid: 2, deflection: 0.05 } },
        { op: 'getSolidFaces', args: { solid: 2 } },
        { op: 'volume', args: { solid: 0, deflection: 0.05 } },
      ],
      booleanIndex: 3,
      volumeIndex: 4,
      validationIndex: 5,
      meshQualityIndex: 6,
      facesIndex: 7,
      operandVolumeIndex: 8,
      expect: {
        outcome: 'success',
        quality: 'exact',
        volume: 0,
        volumeTolerance: 1e-9,
        faces: 0,
        operandVolume: 8,
        operandVolumeTolerance: 1e-9,
      },
    },
    {
      id: 'contract/contained-cut-refusal',
      title: 'contained-cut empty refusal',
      description:
        'cutting a box fully contained in its tool refuses typed (the kernel '
        + 'EmptyResult on the wire) and leaves the tool measurable',
      batch: [
        { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
        { op: 'makeBox', args: { width: 1, height: 1, depth: 1 } },
        { op: 'transform', args: { solid: 1, matrix: translationMatrix([1, 1, 1]) } },
        { op: 'booleanWithQuality', args: { operation: 'cut', solidA: 1, solidB: 0 } },
        { op: 'volume', args: { solid: 0, deflection: 0.05 } },
        { op: 'getSolidFaces', args: { solid: 0 } },
      ],
      booleanIndex: 3,
      volumeIndex: 4,
      facesIndex: 5,
      expect: {
        outcome: 'batch_error',
        code: 'operation_failed',
        rollbackVolume: 64,
        rollbackVolumeTolerance: 1e-9,
        rollbackFaces: 6,
      },
    },
  ];
}

export function scoreContractCase(c, observations) {
  requireObservations(c.id, observations);
  const stages = [];
  const add = (surface, invariant, passed, required = true) => stages.push({
    case: c.id,
    operation: c.title,
    surface,
    invariant,
    passed,
    required,
  });

  for (const surface of ['native', 'fresh', 'committed']) {
    const observation = observations[surface];
    if (!observation) continue;
    if (c.expect.outcome === 'batch_error') {
      add(surface, 'bounded_execution', !['crash', 'hang_or_budget_overrun'].includes(observation.outcome));
      add(surface, 'refusal_outcome', observation.outcome === 'batch_error');
      add(
        surface,
        'refusal_code',
        (observation.diagnosticCodes ?? []).includes(c.expect.code),
      );
      if (c.expect.rollbackVolume !== undefined) {
        const volumes = observation.rollbackVolumes ?? [];
        const ok = volumes.length > 0
          && volumes.every((v) => relativeError(v, c.expect.rollbackVolume)
            <= (c.expect.rollbackVolumeTolerance ?? 1e-9));
        add(surface, 'rollback_preserved', ok);
      }
      if (c.expect.rollbackFaces !== undefined) {
        add(surface, 'rollback_faces', observation.rollbackFaces === c.expect.rollbackFaces);
      }
    } else {
      add(surface, 'bounded_execution', !['crash', 'hang_or_budget_overrun'].includes(observation.outcome));
      add(surface, 'batch_success', observation.outcome === 'success');
      add(surface, 'disclosed_quality', observation.quality === c.expect.quality);
      if (c.expect.volume !== undefined) {
        add(
          surface,
          'oracle_volume',
          Number.isFinite(observation.volume)
            && relativeError(observation.volume, c.expect.volume) <= (c.expect.volumeTolerance ?? 1e-6),
        );
      }
      if (c.expect.faces !== undefined) {
        add(surface, 'face_count', observation.faceCount === c.expect.faces);
      }
      if (c.expect.operandVolume !== undefined) {
        add(
          surface,
          'operands_preserved',
          Number.isFinite(observation.operandVolume)
            && relativeError(observation.operandVolume, c.expect.operandVolume)
              <= (c.expect.operandVolumeTolerance ?? 1e-9),
        );
      }
      if (c.expect.evolution !== undefined) {
        const summary = summarizeEvolution(observation.evolution);
        add(
          surface,
          'evolution_buckets',
          summary !== null && canonicalJson(summary) === canonicalJson(c.expect.evolution),
        );
      }
      if (c.expect.unresolvedBucket !== undefined) {
        const unresolved = observation.evolution?.unresolved;
        add(
          surface,
          'evolution_unresolved',
          unresolved !== undefined && unresolved !== null
            && canonicalJson(unresolved) === canonicalJson(c.expect.unresolvedBucket),
        );
      }
    }
  }

  const present = ['native', 'fresh', 'committed'].filter((s) => observations[s]);
  if (present.length >= 2) {
    const [first, ...rest] = present.map((s) => observations[s]);
    add(
      'cross-surface',
      'outcome_agreement',
      rest.every((o) => o.outcome === first.outcome),
    );
    add(
      'cross-surface',
      'diagnostics_agreement',
      rest.every((o) => JSON.stringify(o.diagnosticCodes) === JSON.stringify(first.diagnosticCodes)),
    );
    if (c.expect.outcome === 'success' && c.expect.quality) {
      add(
        'cross-surface',
        'quality_agreement',
        rest.every((o) => (o.quality ?? null) === (first.quality ?? null)),
      );
    }
    if (c.expect.outcome === 'success' && c.expect.faces !== undefined) {
      add(
        'cross-surface',
        'face_count_agreement',
        rest.every((o) => (o.faceCount ?? null) === (first.faceCount ?? null)),
      );
    }
    if (c.expect.outcome === 'success' && c.expect.evolution !== undefined) {
      const summaryOf = (o) => canonicalJson(summarizeEvolution(o.evolution));
      const unresolvedOf = (o) => canonicalJson(o.evolution?.unresolved ?? null);
      add(
        'cross-surface',
        'evolution_agreement',
        rest.every((o) => summaryOf(o) === summaryOf(first) && unresolvedOf(o) === unresolvedOf(first)),
      );
      // Raw index-level agreement (which arena handles each bucket names)
      // is evidence, not a gate: handle assignment is an arena detail, like
      // the geometric slices' byte identity.
      add(
        'cross-surface',
        'evolution_raw_agreement',
        rest.every((o) => canonicalJson(o.evolution ?? null) === canonicalJson(first.evolution ?? null)),
        false,
      );
    }
  }
  return stages;
}

function attempt(command, args, payload) {
  const result = spawnSync(command, args, {
    input: JSON.stringify(payload),
    encoding: 'utf8',
    timeout: timeoutMs,
    maxBuffer: 16 * 1024 * 1024,
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

function installPackage(packageDir, temporaryRoot, label) {
  const consumerDir = resolve(temporaryRoot, `consumer-${label}`);
  mkdirSync(consumerDir, { recursive: true });
  const npmEnvironment = {
    ...process.env,
    npm_config_cache: resolve(temporaryRoot, 'npm-cache'),
  };
  const packed = JSON.parse(execFileSync(
    'npm',
    ['pack', resolve(packageDir), '--json', '--pack-destination', temporaryRoot],
    { encoding: 'utf8', env: npmEnvironment },
  ));
  assert.equal(packed.length, 1, 'npm pack must produce exactly one tarball');
  const tarballPath = resolve(temporaryRoot, packed[0].filename);
  execFileSync('npm', [
    'install',
    '--prefix', consumerDir,
    '--no-save',
    '--package-lock=false',
    '--ignore-scripts',
    '--no-audit',
    '--no-fund',
    tarballPath,
  ], { env: npmEnvironment, stdio: 'ignore' });
  const consumerRequire = createRequire(resolve(consumerDir, 'consumer.cjs'));
  const entry = consumerRequire.resolve('remus-wasm');
  const installedDir = realpathSync(resolve(consumerDir, 'node_modules/remus-wasm'));
  validateInstalledEntry(entry, installedDir);
  return { entry, installedDir, tarballPath };
}

function batchObservation(c, modulePath) {
  const { BrepKernel } = createRequire(import.meta.url)(modulePath);
  const initStarted = performance.now();
  const kernel = new BrepKernel();
  const coldInitMs = performance.now() - initStarted;
  try {
    const batchStarted = performance.now();
    const responses = JSON.parse(kernel.executeBatchV2(JSON.stringify(c.batch)));
    const batchDurationMs = performance.now() - batchStarted;
    return finalizeBatchObservation(c, responses, coldInitMs, batchDurationMs);
  } finally {
    kernel.free();
  }
}

function finalizeBatchObservation(c, responses, coldInitMs, batchDurationMs) {
  const diagnosticCodes = responses.flatMap((response) => (
    response && response.error ? [response.error.code ?? 'untyped_error'] : []
  ));
  if (diagnosticCodes.length > 0) {
    // Rollback probes: every succeeding `volume` ok-value and the last
    // succeeding `getSolidFaces` length, exactly as the native runner
    // collects them for any batch_error case.
    const rollbackVolumes = [];
    let rollbackFaces = null;
    for (let i = 0; i < responses.length; i++) {
      const response = responses[i];
      if (response && response.ok !== undefined && c.batch[i]?.op === 'volume') {
        rollbackVolumes.push(response.ok);
      }
      if (response && response.ok !== undefined && c.batch[i]?.op === 'getSolidFaces') {
        rollbackFaces = response.ok.length;
      }
    }
    return {
      schemaVersion: 1,
      id: c.id,
      surface: 'wasm-batch',
      outcome: 'batch_error',
      diagnosticCodes,
      rollbackVolumes,
      rollbackFaces,
      coldInitMs,
      batchDurationMs,
    };
  }
  const booleanResponse = c.booleanIndex !== null && c.booleanIndex !== undefined
    ? responses[c.booleanIndex]?.ok
    : null;
  const quality = booleanResponse?.quality ?? (c.expect.outcome === 'success' ? 'exact' : null);
  const faces = c.facesIndex !== null && c.facesIndex !== undefined
    ? responses[c.facesIndex]?.ok ?? null
    : null;
  const evolution = c.evolutionIndex !== null && c.evolutionIndex !== undefined
    ? responses[c.evolutionIndex]?.ok?.evolution ?? null
    : null;
  const operandVolume = c.operandVolumeIndex !== null && c.operandVolumeIndex !== undefined
    ? responses[c.operandVolumeIndex]?.ok ?? null
    : null;
  return {
    schemaVersion: 1,
    id: c.id,
    surface: 'wasm-batch',
    outcome: 'success',
    diagnosticCodes,
    quality,
    volume: c.volumeIndex !== null ? responses[c.volumeIndex]?.ok ?? null : null,
    faces,
    faceCount: Array.isArray(faces) ? faces.length : null,
    evolution,
    operandVolume,
    coldInitMs,
    batchDurationMs,
  };
}

export function directObservation(c, modulePath) {
  const mod = createRequire(import.meta.url)(modulePath);
  const { BrepKernel } = mod;
  const initStarted = performance.now();
  const kernel = new BrepKernel();
  const coldInitMs = performance.now() - initStarted;
  try {
    const batchStarted = performance.now();
    const observation = runDirectCase(c, kernel, mod);
    observation.coldInitMs = coldInitMs;
    observation.batchDurationMs = performance.now() - batchStarted;
    return observation;
  } finally {
    kernel.free();
  }
}

function runDirectCase(c, kernel, mod) {
  const base = { schemaVersion: 1, id: c.id, surface: 'wasm-direct' };
  const countsOf = (solid) => Array.from(kernel.getEntityCounts(solid));
  if (c.id === 'contract/exact-fuse') {
    const a = kernel.makeBox(2, 2, 2);
    const b = kernel.makeBox(2, 2, 2);
    kernel.transformSolid(b, translationMatrix([1, 1, 1]));
    const out = kernel.booleanWithQuality('fuse', a, b);
    const volume = kernel.volume(out.solid, 0.05);
    const faces = kernel.getSolidFaces(out.solid);
    return {
      ...base, outcome: 'success', diagnosticCodes: [], quality: out.quality, volume,
      census: { faces: faces.length }, directBatchConsistent: true,
    };
  }
  if (c.id === 'contract/exact-only-refusal') {
    const a = kernel.makeBox(60, 40, 8);
    const b = kernel.makeCylinder(10, 16);
    kernel.transformSolid(b, translationMatrix([9.999, 20, 0]));
    const before = countsOf(a);
    try {
      kernel.booleanWithQuality('fuse', a, b, true);
      return { ...base, outcome: 'success', diagnosticCodes: [], quality: 'exact' };
    } catch (error) {
      const after = countsOf(a);
      return {
        ...base,
        outcome: 'batch_error',
        diagnosticCodes: ['operation_failed'],
        directError: String(error?.message ?? error),
        rollbackPreserved: JSON.stringify(before) === JSON.stringify(after),
      };
    }
  }
  if (c.id === 'contract/disclosed-approximation') {
    const a = kernel.makeBox(60, 40, 8);
    const b = kernel.makeCylinder(10, 16);
    kernel.transformSolid(b, translationMatrix([9.999, 20, 0]));
    const out = kernel.booleanWithQuality('fuse', a, b);
    const volume = kernel.volume(out.solid, 0.05);
    return {
      ...base, outcome: 'success', diagnosticCodes: [], quality: out.quality,
      deflection: out.deflection ?? null, volume,
    };
  }
  if (c.id === 'contract/invalid-handle') {
    kernel.makeBox(2, 2, 2);
    try {
      kernel.volume(9999, 0.05);
      return { ...base, outcome: 'success', diagnosticCodes: [] };
    } catch {
      return { ...base, outcome: 'batch_error', diagnosticCodes: ['invalid_handle'] };
    }
  }
  if (c.id === 'contract/transactional-rollback') {
    const a = kernel.makeBox(2, 2, 2);
    const beforeVolume = kernel.volume(a, 0.05);
    const beforeFaces = kernel.getSolidFaces(a).length;
    const checkpoint = kernel.checkpoint();
    let refusal = null;
    try {
      kernel.volume(9999, 0.05);
    } catch (error) {
      refusal = String(error?.message ?? error);
    }
    kernel.restore(checkpoint);
    const afterVolume = kernel.volume(a, 0.05);
    const afterFaces = kernel.getSolidFaces(a).length;
    return {
      ...base,
      outcome: refusal ? 'batch_error' : 'success',
      diagnosticCodes: refusal ? ['invalid_handle'] : [],
      rollbackVolumes: [afterVolume],
      rollbackFaces: afterFaces,
      rollbackPreserved: Math.abs(afterVolume - beforeVolume) < 1e-9 && afterFaces === beforeFaces,
    };
  }
  if (c.id === 'contract/cancelled-boolean') {
    const a = kernel.makeBox(2, 2, 2);
    const b = kernel.makeBox(2, 2, 2);
    kernel.transformSolid(b, translationMatrix([1, 1, 1]));
    const before = countsOf(a);
    const Token = mod.OperationCancellationToken;
    assert.ok(Token, 'installed package does not export OperationCancellationToken');
    const token = new Token();
    assert.equal(token.isCancelled(), false);
    token.cancel();
    assert.equal(token.isCancelled(), true);
    const out = kernel.booleanWithCancellation('fuse', a, b, token);
    const after = countsOf(a);
    const cancelled = out.status === 'Cancelled' || out.status === 'cancelled' || out.status === 1;
    // Post-cancellation probes so the scorer's uniform rollback gate
    // (volume 8 plus face count) applies to the direct shape too: the
    // pre-existing box must still measure identically.
    const afterVolume = kernel.volume(a, 0.05);
    const afterFaces = kernel.getSolidFaces(a).length;
    return {
      ...base,
      outcome: cancelled ? 'batch_error' : 'success',
      diagnosticCodes: cancelled ? ['cancelled'] : [],
      cancellationStatus: out.status ?? null,
      cancellationCode: out.code ?? null,
      rollbackVolumes: [afterVolume],
      rollbackFaces: afterFaces,
      rollbackPreserved: JSON.stringify(before) === JSON.stringify(after),
      scope: 'pre-cancelled token only; synchronous WASM cannot process a later JS cancellation on the same thread',
    };
  }
  if (c.id === 'contract/evolution-fuse' || c.id === 'contract/evolution-cut') {
    const a = kernel.makeBox(2, 2, 2);
    const b = kernel.makeBox(2, 2, 2);
    kernel.transformSolid(b, translationMatrix([1, 1, 1]));
    // The direct bindings answer with a JSON string of the same
    // `{solid, evolution}` shape the batch arm returns as an object.
    const raw = c.id === 'contract/evolution-fuse'
      ? kernel.fuseWithEvolution(a, b)
      : kernel.cutWithEvolution(a, b);
    const out = typeof raw === 'string' ? JSON.parse(raw) : raw;
    const volume = kernel.volume(out.solid, 0.05);
    const faces = kernel.getSolidFaces(out.solid);
    return {
      ...base, outcome: 'success', diagnosticCodes: [], quality: 'exact', volume,
      faceCount: faces.length, evolution: out.evolution ?? null,
    };
  }
  if (c.id === 'contract/invalid-primitive-input') {
    const a = kernel.makeBox(2, 2, 2);
    const before = countsOf(a);
    let refusal = null;
    try {
      kernel.makeBox(-1, 2, 2);
    } catch (error) {
      refusal = String(error?.message ?? error);
    }
    const afterVolume = kernel.volume(a, 0.05);
    const afterFaces = kernel.getSolidFaces(a).length;
    return {
      ...base,
      outcome: refusal ? 'batch_error' : 'success',
      diagnosticCodes: refusal ? ['invalid_argument'] : [],
      directError: refusal,
      rollbackVolumes: [afterVolume],
      rollbackFaces: afterFaces,
      rollbackPreserved: JSON.stringify(before) === JSON.stringify(countsOf(a)),
    };
  }
  if (c.id === 'contract/empty-intersect-sentinel') {
    const a = kernel.makeBox(2, 2, 2);
    const b = kernel.makeBox(2, 2, 2);
    kernel.transformSolid(b, translationMatrix([10, 10, 10]));
    const out = kernel.booleanWithQuality('intersect', a, b);
    const volume = kernel.volume(out.solid, 0.05);
    const faces = kernel.getSolidFaces(out.solid);
    const operandVolume = kernel.volume(a, 0.05);
    return {
      ...base, outcome: 'success', diagnosticCodes: [], quality: out.quality, volume,
      faceCount: faces.length, operandVolume,
    };
  }
  if (c.id === 'contract/contained-cut-refusal') {
    const tool = kernel.makeBox(4, 4, 4);
    const target = kernel.makeBox(1, 1, 1);
    kernel.transformSolid(target, translationMatrix([1, 1, 1]));
    const before = countsOf(tool);
    let refusal = null;
    try {
      kernel.booleanWithQuality('cut', target, tool);
    } catch (error) {
      refusal = String(error?.message ?? error);
    }
    const afterVolume = kernel.volume(tool, 0.05);
    const afterFaces = kernel.getSolidFaces(tool).length;
    return {
      ...base,
      outcome: refusal ? 'batch_error' : 'success',
      diagnosticCodes: refusal ? ['operation_failed'] : [],
      directError: refusal,
      rollbackVolumes: [afterVolume],
      rollbackFaces: afterFaces,
      rollbackPreserved: JSON.stringify(before) === JSON.stringify(countsOf(tool)),
    };
  }
  throw new Error(`unknown contract case ${c.id}`);
}

function normalizeWasmObservation(c, raw) {
  if (raw.surface === 'wasm-direct') {
    if (c.expect.outcome === 'batch_error') {
      return {
        schemaVersion: 1,
        id: c.id,
        outcome: raw.outcome,
        diagnosticCodes: raw.diagnosticCodes ?? [],
        rollbackVolumes: raw.rollbackVolumes ?? [],
        rollbackFaces: raw.rollbackFaces ?? null,
        direct: raw,
      };
    }
    return {
      schemaVersion: 1,
      id: c.id,
      outcome: raw.outcome,
      diagnosticCodes: raw.diagnosticCodes ?? [],
      quality: raw.quality ?? null,
      volume: raw.volume ?? null,
      faceCount: raw.faceCount ?? null,
      evolution: raw.evolution ?? null,
      operandVolume: raw.operandVolume ?? null,
      direct: raw,
    };
  }
  if (c.expect.outcome === 'batch_error') {
    return {
      schemaVersion: 1,
      id: c.id,
      outcome: raw.outcome,
      diagnosticCodes: raw.diagnosticCodes ?? [],
      rollbackVolumes: raw.rollbackVolumes ?? [],
      rollbackFaces: raw.rollbackFaces ?? null,
    };
  }
  return {
    schemaVersion: 1,
    id: c.id,
    outcome: raw.outcome,
    diagnosticCodes: raw.diagnosticCodes ?? [],
    quality: raw.quality ?? null,
    volume: raw.volume ?? null,
    faceCount: raw.faceCount ?? null,
    evolution: raw.evolution ?? null,
    operandVolume: raw.operandVolume ?? null,
  };
}

async function main() {
  if (process.argv[2] === '--wasm-batch-child') {
    const c = JSON.parse(readFileSync(0, 'utf8'));
    process.stdout.write(JSON.stringify(batchObservation(c, process.argv[3])));
    return;
  }
  if (process.argv[2] === '--wasm-direct-child') {
    const c = JSON.parse(readFileSync(0, 'utf8'));
    process.stdout.write(JSON.stringify(directObservation(c, process.argv[3])));
    return;
  }

  const [nativeRunner, freshPackageDir, committedPackageDirArg] = process.argv.slice(2);
  assert.ok(nativeRunner && freshPackageDir, 'usage: contract-matrix.mjs NATIVE_RUNNER FRESH_PACKAGE_DIR [COMMITTED_PACKAGE_DIR]');
  const committedPackageDir = committedPackageDirArg ?? resolve(repoRoot, 'crates/wasm/pkg');
  const cases = expandContractMatrix();
  const temporaryRoot = mkdtempSync(resolve(tmpdir(), 'remus-o15-contract-'));
  const started = performance.now();
  try {
    const fresh = installPackage(resolve(freshPackageDir), temporaryRoot, 'fresh');
    const committed = installPackage(resolve(committedPackageDir), temporaryRoot, 'committed');
    const buildOptions = {
      target: 'nodejs',
      profile: 'release',
      features: 'no-default-features',
      extraArgs: ['--no-opt'],
      tool: 'wasm-pack build crates/wasm --target nodejs --release --no-opt -- --no-default-features',
    };
    const freshProvenance = collectFreshProvenance({
      repoRoot,
      packageDir: resolve(freshPackageDir),
      tarballPath: fresh.tarballPath,
      installedEntry: fresh.entry,
      installedDir: fresh.installedDir,
      buildOptions,
    });
    const committedProvenance = collectCommittedProvenance({
      repoRoot,
      packageDir: resolve(committedPackageDir),
      installedEntry: committed.entry,
      installedDir: committed.installedDir,
    });
    const source = collectSourceProvenance(repoRoot);
    const nativeProvenance = {
      mode: 'native',
      sourceRevision: source.sourceRevision,
      sourceDirty: source.sourceDirty,
      toolchain: source.toolchain,
      target: 'host',
      buildOptions: { profile: 'release', binary: 'remus-parity-native', facade: 'remus::Model' },
      packageVersion: null,
      wasmSha256: null,
      installedEntry: resolve(nativeRunner),
    };
    const staleness = describeStaleness(freshProvenance, committedProvenance);
    const observations = [];
    for (const c of cases) {
      // The native facade runner speaks the geometric case envelope, so
      // success cases carry the full result spec while intentional-error
      // cases carry dummy indices the runner never reads past its early
      // batch_error return.
      const payload = {
        schema_version: 1,
        id: c.id,
        batch: c.batch,
        result: {
          handle: 0,
          booleanIndex: c.booleanIndex ?? 0,
          volumeIndex: c.volumeIndex ?? 0,
          validationIndex: c.validationIndex ?? 0,
          meshQualityIndex: c.meshQualityIndex ?? 0,
          facesIndex: c.facesIndex ?? 0,
        },
      };
      const native = attempt(resolve(nativeRunner), [], payload);
      const freshBatch = attempt(process.execPath, [self, '--wasm-batch-child', fresh.entry], c);
      const freshDirect = attempt(process.execPath, [self, '--wasm-direct-child', fresh.entry], c);
      const committedBatch = attempt(process.execPath, [self, '--wasm-batch-child', committed.entry], c);
      const committedDirect = attempt(process.execPath, [self, '--wasm-direct-child', committed.entry], c);
      const nativeNorm = normalizeNativeForContract(c, native);
      // The cancellation cell is direct-only by construction: batch has no
      // cancellation op (and this harness must not extend the WASM API), so
      // the scored WASM observations come from the installed direct calls
      // while the batch responses ride along as evidence. The native
      // runner's `booleanWithCancelledContext` batch op is the harness-only
      // native counterpart of that same pre-cancelled scope.
      const scoreFresh = c.id === 'contract/cancelled-boolean'
        ? normalizeWasmObservation(c, freshDirect)
        : normalizeWasmObservation(c, freshBatch);
      const scoreCommitted = c.id === 'contract/cancelled-boolean'
        ? normalizeWasmObservation(c, committedDirect)
        : normalizeWasmObservation(c, committedBatch);
      observations.push({
        case: c.id,
        native: nativeNorm,
        fresh: scoreFresh,
        freshBatch,
        freshDirect: normalizeWasmObservation(c, freshDirect),
        freshDirectRaw: freshDirect,
        committed: scoreCommitted,
        committedBatch,
        committedDirect: normalizeWasmObservation(c, committedDirect),
        committedDirectRaw: committedDirect,
      });
    }
    const scored = cases.map((c, index) => {
      const obs = observations[index];
      return scoreContractCase(c, { native: obs.native, fresh: obs.fresh, committed: obs.committed });
    });
    const stages = scored.flat();
    // Direct-vs-batch consistency per installed surface is evidence, not a
    // cross-target gate: it catches a binding that drifts from its own
    // batch arm on the same bytes.
    const directConsistency = observations.flatMap((obs) => {
      const c = cases.find(({ id }) => id === obs.case);
      return ['fresh', 'committed'].map((surface) => {
        const batch = surface === 'fresh' ? obs.freshBatch : obs.committedBatch;
        const direct = surface === 'fresh' ? obs.freshDirectRaw : obs.committedDirectRaw;
        const passed = directConsistent(c, batch, direct);
        return { case: obs.case, surface: `${surface}-direct-vs-batch`, invariant: 'direct_batch_consistency', passed, required: false };
      });
    });
    const failedStages = stages.filter((stage) => stage.required && !stage.passed);
    const knownGaps = [...stages, ...directConsistency].filter((stage) => !stage.required && !stage.passed);
    const passed = failedStages.length === 0;
    const report = {
      schema_version: 1,
      scope: 'O1.5 contract slice: exact / exact-only refusal / disclosed approximation / invalid handle / rollback / pre-cancelled cancellation / evolution reports (fuse, cut) / invalid primitive input / empty-intersect sentinel / contained-cut refusal, batch plus installed-direct',
      provenance: { native: nativeProvenance, fresh: freshProvenance, committed: committedProvenance },
      staleness,
      cancellationScope: 'pre-cancelled token only; a synchronous WASM call cannot process a later JS cancellation message on the same thread',
      timeout_ms: timeoutMs,
      matrix: { cases: cases.map(({ id, title }) => ({ id, title })) },
      elapsed_ms: performance.now() - started,
      passed,
      failed_stages: failedStages,
      known_gaps: knownGaps,
      stages: [...stages, ...directConsistency],
      observations,
    };
    process.stdout.write(`${JSON.stringify(report, null, 2)}\n`);
    process.exitCode = passed ? 0 : 1;
  } finally {
    rmSync(temporaryRoot, { recursive: true, force: true });
  }
}

function normalizeNativeForContract(c, native) {
  // The native runner speaks the geometric observation envelope
  // (schema 1/2 with census etc.). Contract scoring needs only the
  // refusal/success projection plus rollback probes, which the runner
  // already returns for batch_error cases via its disclosure path.
  // For rollback-flavoured error cases, re-read the succeeding volume
  // and face probes from the raw batch responses when present.
  if (!native || typeof native !== 'object') return native;
  if (c.expect.outcome === 'batch_error' && native.outcome === 'batch_error') {
    return {
      schemaVersion: 1,
      id: c.id,
      outcome: 'batch_error',
      diagnosticCodes: native.diagnosticCodes ?? [],
      rollbackVolumes: native.rollbackVolumes ?? [],
      rollbackFaces: native.rollbackFaces ?? null,
    };
  }
  if (native.outcome === 'success') {
    // Project the geometric envelope onto the contract probes: the face
    // census stands in for the `getSolidFaces` length, the producing op's
    // evolution report passes through, and any extra `volume` probe is
    // read back by batch index (`volumesByIndex`).
    const operandVolume = c.operandVolumeIndex !== null && c.operandVolumeIndex !== undefined
      ? native.volumesByIndex?.[String(c.operandVolumeIndex)] ?? null
      : null;
    return {
      ...native,
      faceCount: native.census?.faces ?? null,
      evolution: native.evolution ?? null,
      operandVolume,
    };
  }
  return native;
}

function directConsistent(c, batch, direct) {
  if (!batch || !direct || batch.outcome === undefined || direct.outcome === undefined) return false;
  if (batch.outcome !== direct.outcome) return false;
  if (c.expect.outcome === 'success' && c.expect.quality) {
    const batchQuality = batch.quality ?? batch.direct?.quality ?? null;
    const directQuality = direct.quality ?? direct.direct?.quality ?? null;
    if (batchQuality && directQuality && batchQuality !== directQuality) return false;
  }
  const batchCodes = JSON.stringify(batch.diagnosticCodes ?? []);
  const directCodes = JSON.stringify(direct.diagnosticCodes ?? direct.direct?.diagnosticCodes ?? []);
  return batchCodes === directCodes;
}

if (process.argv[1] === self) await main();
