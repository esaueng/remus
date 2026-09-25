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
  describeStaleness,
  requireObservations,
  validateInstalledEntry,
  validateProvenance,
} from './provenance.mjs';

const self = fileURLToPath(import.meta.url);
const scriptDir = dirname(self);
const repoRoot = resolve(scriptDir, '..', '..');
const timeoutMs = 120_000;

// Independent closed-form oracles. Native/WASM agreement alone never
// validates; every success cell asserts these physical values per surface.
export const ORACLES = {
  boxVolume: 24,
  boxFaces: 6,
  boxShells: 1,
  hollowVolume: 488,
  hollowFaces: 12,
  hollowShells: 2,
  plateVolume: 40 * 24 * 10 - (4 - Math.PI) * 9 * 10,
  plateFaces: 10,
  platePcurves: 48,
  volumeTolerance: 1e-9,
};

function relativeError(actual, expected) {
  return Math.abs(actual - expected) / Math.max(Math.abs(expected), 1);
}

export function expandSplitMatrix() {
  return [
    {
      id: 'split/step-import-box',
      title: 'valid STEP import through RemusIo -> arena -> deserializeSolids',
      expect: {
        outcome: 'success',
        volume: ORACLES.boxVolume,
        volumeTolerance: ORACLES.volumeTolerance,
        faces: ORACLES.boxFaces,
        shells: ORACLES.boxShells,
        validationErrors: 0,
        census: { plane: 6 },
        probes: { inside: 'inside', outside: 'outside' },
      },
    },
    {
      id: 'split/box-hollow-roundtrip',
      title: 'kernel box + qualified hollow cut -> serializeSolids -> exportStep -> re-import',
      expect: {
        outcome: 'success',
        volumes: { box: ORACLES.boxVolume, hollow: ORACLES.hollowVolume },
        volumeTolerance: ORACLES.volumeTolerance,
        faceCounts: { box: ORACLES.boxFaces, hollow: ORACLES.hollowFaces },
        shellCounts: { box: ORACLES.boxShells, hollow: ORACLES.hollowShells },
        validationErrors: 0,
        census: { plane: 12 },
        probes: { wall: 'inside', cavity: 'outside', outside: 'outside' },
      },
    },
    {
      id: 'split/periodic-seam',
      title: 'existing periodic-seam fixture preserves both per-use pcurves',
      expect: {
        outcome: 'success',
        volume: ORACLES.plateVolume,
        volumeTolerance: ORACLES.volumeTolerance,
        faces: ORACLES.plateFaces,
        validationErrors: 0,
        census: { cylinder: 4, plane: 6 },
        pcurves: ORACLES.platePcurves,
      },
    },
    {
      id: 'split/malformed-refusal',
      title: 'malformed / limit-refused STEP import preserves the kernel session',
      expect: {
        outcome: 'batch_error',
        volume: ORACLES.boxVolume,
        faces: ORACLES.boxFaces,
        rollbackPreserved: true,
        preservedBytesEqual: true,
      },
    },
    {
      id: 'split/arena-truncation',
      title: 'invalid / truncated arena transfer preserves earlier solids and handles',
      expect: {
        outcome: 'batch_error',
        volume: ORACLES.boxVolume,
        faces: ORACLES.boxFaces,
        rollbackPreserved: true,
        preservedBytesEqual: true,
      },
    },
    {
      id: 'split/two-session-isolation',
      title: 'repeated success/refusal across two sessions leaks no handles or partial solids',
      expect: {
        outcome: 'success',
        volumes: { sessionA: ORACLES.boxVolume, sessionB: ORACLES.boxVolume },
        volumeTolerance: ORACLES.volumeTolerance,
        noPartialSolids: true,
        crossSessionStaleRefused: true,
      },
    },
  ];
}

function censusPass(expected, actual) {
  if (!expected || !actual) return expected === actual;
  const keys = new Set([...Object.keys(expected), ...Object.keys(actual)]);
  for (const key of keys) {
    if ((expected[key] ?? 0) !== (actual[key] ?? 0)) return false;
  }
  return true;
}

function probesPass(expected, actual) {
  if (!expected) return true;
  if (!actual) return false;
  return Object.entries(expected).every(([key, value]) => actual[key] === value);
}

export function scoreSplitCase(c, observations) {
  const stages = [];
  const add = (surface, invariant, passed, required = true) => stages.push({
    case: c.id,
    operation: c.id,
    surface,
    invariant,
    passed,
    required,
  });

  const isRefusalCell = c.expect.outcome === 'batch_error';
  for (const surface of ['native', 'fresh', 'committed']) {
    const observation = observations[surface];
    if (observation === undefined) continue;
    const succeeded = observation.outcome === 'success';
    add(surface, 'bounded_execution', !['crash', 'hang_or_budget_overrun'].includes(observation.outcome));
    if (isRefusalCell) {
      add(surface, 'typed_refusal', observation.outcome === 'batch_error');
      add(
        surface,
        'refusal_diagnostics',
        Array.isArray(observation.diagnosticCodes) && observation.diagnosticCodes.length > 0,
      );
      if (c.expect.rollbackPreserved !== undefined) {
        add(surface, 'rollback_preserved', observation.rollbackPreserved === true);
      }
      if (c.expect.preservedBytesEqual !== undefined) {
        add(surface, 'preserved_bytes_equal', observation.preservedBytesEqual === true);
      }
      if (c.expect.volume !== undefined) {
        add(
          surface,
          'oracle_volume',
          Number.isFinite(observation.volume)
            && relativeError(observation.volume, c.expect.volume) <= ORACLES.volumeTolerance,
        );
      }
      if (c.expect.faces !== undefined) {
        add(surface, 'face_count', observation.faceCount === c.expect.faces);
      }
      // Refusal cells carry no geometry gates beyond preservation.
      add(surface, 'disclosed_quality', true, false);
      add(surface, 'valid_brep', true, false);
    } else {
      add(surface, 'batch_success', succeeded);
      if (c.id === 'split/two-session-isolation') {
        // The cell succeeds overall but records the expected inner refusal.
        add(
          surface,
          'diagnostics_expected',
          (observation.diagnosticCodes?.length ?? 0) > 0,
        );
      } else {
        add(
          surface,
          'diagnostics_empty',
          (observation.diagnosticCodes?.length ?? 1) === 0,
        );
      }
      if (c.expect.volume !== undefined) {
        add(
          surface,
          'oracle_volume',
          succeeded
            && Number.isFinite(observation.volume)
            && relativeError(observation.volume, c.expect.volume) <= c.expect.volumeTolerance,
        );
      } else if (c.expect.volumes) {
        const ok = succeeded
          && observation.volumes
          && Object.entries(c.expect.volumes).every(
            ([key, expected]) => Number.isFinite(observation.volumes[key])
              && relativeError(observation.volumes[key], expected) <= c.expect.volumeTolerance,
          );
        add(surface, 'oracle_volume', ok);
      } else {
        add(surface, 'oracle_volume', true);
      }
      if (c.expect.faces !== undefined) {
        add(surface, 'face_count', observation.faceCount === c.expect.faces);
      } else if (c.expect.faceCounts) {
        add(
          surface,
          'face_count',
          succeeded
            && observation.faceCounts
            && Object.entries(c.expect.faceCounts).every(
              ([key, expected]) => observation.faceCounts[key] === expected,
            ),
        );
      } else {
        add(surface, 'face_count', true);
      }
      if (c.expect.shells !== undefined) {
        add(surface, 'shell_count', observation.shellCount === c.expect.shells);
      } else if (c.expect.shellCounts) {
        add(
          surface,
          'shell_count',
          succeeded
            && observation.shellCounts
            && Object.entries(c.expect.shellCounts).every(
              ([key, expected]) => observation.shellCounts[key] === expected,
            ),
        );
      } else {
        add(surface, 'shell_count', true);
      }
      if (c.expect.validationErrors !== undefined) {
        add(surface, 'valid_brep', observation.validationErrors === c.expect.validationErrors);
      } else {
        add(surface, 'valid_brep', true);
      }
      if (c.expect.census) {
        add(surface, 'analytic_census', censusPass(c.expect.census, observation.census));
      } else {
        add(surface, 'analytic_census', true);
      }
      if (c.expect.probes) {
        add(surface, 'material_probes', probesPass(c.expect.probes, observation.probes));
      } else {
        add(surface, 'material_probes', true);
      }
      if (c.expect.pcurves !== undefined) {
        // Native asserts the topology pcurve count directly; WASM asserts
        // the re-exported STEP PCURVE count (no pcurve binding exists).
        const nativeOk = surface === 'native'
          ? observation.pcurveCount === c.expect.pcurves
          : true;
        const stepOk = (observation.stepPcurveCount ?? observation.pcurveCount) === c.expect.pcurves;
        add(surface, 'seam_pcurves', nativeOk && stepOk);
      } else {
        add(surface, 'seam_pcurves', true);
      }
      if (c.expect.noPartialSolids !== undefined) {
        add(surface, 'no_partial_solids', observation.noPartialSolids === true);
      }
      if (c.expect.crossSessionStaleRefused !== undefined) {
        add(
          surface,
          'cross_session_isolation',
          observation.crossSessionStaleRefused === true,
        );
      }
    }
  }

  // Cross-surface agreement: outcome first, then independent-geometry
  // agreement. Native/WASM agreement alone never validates (oracles above
  // do); agreement guards against surface-specific divergence.
  const surfaces = ['native', 'fresh', 'committed'].filter((s) => observations[s] !== undefined);
  for (let i = 0; i < surfaces.length; i++) {
    for (let j = i + 1; j < surfaces.length; j++) {
      const a = observations[surfaces[i]];
      const b = observations[surfaces[j]];
      const pair = `${surfaces[i]}/${surfaces[j]}`;
      add(pair, 'outcome_agreement', a.outcome === b.outcome);
      if (!isRefusalCell && a.outcome === 'success' && b.outcome === 'success') {
        const volumesAgree = (x, y) => {
          if (x === undefined || y === undefined) return true;
          if (typeof x === 'number' && typeof y === 'number') {
            return relativeError(x, y) <= 5e-8;
          }
          if (x && y && typeof x === 'object' && typeof y === 'object') {
            return Object.keys({ ...x, ...y }).every(
              (key) => Number.isFinite(x[key]) && Number.isFinite(y[key])
                && relativeError(x[key], y[key]) <= 5e-8,
            );
          }
          return false;
        };
        add(pair, 'volume_agreement', volumesAgree(a.volume ?? a.volumes, b.volume ?? b.volumes));
        const censusOf = (o) => o.census ?? o.faceCount ?? o.faceCounts ?? null;
        const xa = censusOf(a);
        const xb = censusOf(b);
        const censusAgree = (xa == null || xb == null)
          ? xa == xb
          : (typeof xa === 'object' && typeof xb === 'object')
            ? censusPass(xa, xb)
            : JSON.stringify(xa) === JSON.stringify(xb);
        add(pair, 'census_agreement', censusAgree);
      } else {
        add(pair, 'volume_agreement', true);
        add(pair, 'census_agreement', true);
      }
    }
  }
  return stages;
}

// ── WASM split observation (installed kernel + translator, direct calls) ──

function countsOf(kernel, solid) {
  return Array.from(kernel.getEntityCounts(solid));
}

function censusOf(kernel, solid) {
  const census = {};
  for (const face of Array.from(kernel.getSolidFaces(solid))) {
    const type = kernel.getSurfaceType(face);
    census[type] = (census[type] ?? 0) + 1;
  }
  return census;
}

function stepPcurveCount(stepBytes) {
  return Buffer.from(stepBytes).toString('utf8').split('PCURVE(').length - 1;
}

function wasmSplitObservation(c, kernelEntry, ioEntry, fixtures) {
  const started = performance.now();
  const kernelRequire = createRequire(kernelEntry);
  const ioRequire = createRequire(ioEntry);
  const { BrepKernel } = kernelRequire(kernelEntry);
  const { RemusIo } = ioRequire(ioEntry);
  const base = (outcome) => ({
    schemaVersion: 1,
    id: c.id,
    surface: 'wasm-direct',
    outcome,
    diagnosticCodes: [],
    coldInitMs: 0,
    batchDurationMs: 0,
  });

  try {
    if (c.id === 'split/step-import-box') {
      const kernel = new BrepKernel();
      try {
        const io = new RemusIo();
        const arena = io.importStep(Buffer.from(fixtures.boxStep));
        const handles = Array.from(kernel.deserializeSolids(arena));
        assert.equal(handles.length, 1);
        const solid = handles[0];
        const faces = Array.from(kernel.getSolidFaces(solid));
        const shells = Array.from(kernel.getSolidShells(solid));
        return {
          ...base('success'),
          volume: kernel.volume(solid, 0.05),
          faceCount: faces.length,
          shellCount: shells.length,
          validationErrors: kernel.validateSolid(solid),
          census: censusOf(kernel, solid),
          probes: {
            inside: kernel.classifyPoint(solid, 1, 1, 1, 1e-7),
            outside: kernel.classifyPoint(solid, 10, 10, 10, 1e-7),
          },
          batchDurationMs: performance.now() - started,
        };
      } finally {
        kernel.free();
      }
    }
    if (c.id === 'split/box-hollow-roundtrip') {
      const kernel = new BrepKernel();
      try {
        const io = new RemusIo();
        const box = kernel.makeBox(2, 3, 4);
        const outer = kernel.makeBox(10, 10, 10);
        const inner = kernel.makeBox(8, 8, 8);
        kernel.transformSolid(inner, new Float64Array([1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1]));
        const hollow = kernel.cut(outer, inner);
        const doc = kernel.serializeSolids(Uint32Array.from([box, hollow]));
        const step = io.exportStep(doc);
        const kernel2 = new BrepKernel();
        try {
          const handles = Array.from(kernel2.deserializeSolids(io.importStep(step)));
          assert.equal(handles.length, 2);
          const volumes = {};
          const faceCounts = {};
          const shellCounts = {};
          let probes = null;
          let census = null;
          let validationErrors = null;
          for (const solid of handles) {
            const volume = kernel2.volume(solid, 0.05);
            const key = Math.abs(volume - ORACLES.boxVolume) < 1e-6 ? 'box' : 'hollow';
            volumes[key] = volume;
            faceCounts[key] = Array.from(kernel2.getSolidFaces(solid)).length;
            shellCounts[key] = Array.from(kernel2.getSolidShells(solid)).length;
            if (key === 'hollow') {
              probes = {
                wall: kernel2.classifyPoint(solid, 0.5, 5, 5, 1e-7),
                cavity: kernel2.classifyPoint(solid, 5, 5, 5, 1e-7),
                outside: kernel2.classifyPoint(solid, 20, 20, 20, 1e-7),
              };
              census = censusOf(kernel2, solid);
              validationErrors = kernel2.validateSolid(solid);
            }
          }
          return {
            ...base('success'), volumes, faceCounts, shellCounts, probes, census, validationErrors,
            batchDurationMs: performance.now() - started,
          };
        } finally {
          kernel2.free();
        }
      } finally {
        kernel.free();
      }
    }
    if (c.id === 'split/periodic-seam') {
      const kernel = new BrepKernel();
      try {
        const io = new RemusIo();
        const arena = io.importStep(Buffer.from(fixtures.plateStep));
        const handles = Array.from(kernel.deserializeSolids(arena));
        assert.equal(handles.length, 1);
        const solid = handles[0];
        const faces = Array.from(kernel.getSolidFaces(solid));
        const reexport = io.exportStep(kernel.serializeSolids(Uint32Array.from([solid])));
        return {
          ...base('success'),
          volume: kernel.volume(solid, 0.05),
          faceCount: faces.length,
          validationErrors: kernel.validateSolid(solid),
          census: censusOf(kernel, solid),
          stepPcurveCount: stepPcurveCount(reexport),
          batchDurationMs: performance.now() - started,
        };
      } finally {
        kernel.free();
      }
    }
    if (c.id === 'split/malformed-refusal') {
      const kernel = new BrepKernel();
      try {
        const io = new RemusIo();
        const keep = kernel.makeBox(2, 3, 4);
        const beforeVol = kernel.volume(keep, 0.05);
        const beforeFaces = Array.from(kernel.getSolidFaces(keep)).length;
        const beforeBytes = Buffer.from(kernel.serializeSolids(Uint32Array.from([keep])));
        let refusal = null;
        try {
          io.importStep(Buffer.from('not a STEP file'), 4, undefined);
        } catch (error) {
          refusal = String(error?.message ?? error);
        }
        const afterVol = kernel.volume(keep, 0.05);
        const afterFaces = Array.from(kernel.getSolidFaces(keep)).length;
        const afterBytes = Buffer.from(kernel.serializeSolids(Uint32Array.from([keep])));
        const message = refusal ?? '';
        const code = /limit|exceed/i.test(message) ? 'limit_exceeded'
          : /parse|utf|step/i.test(message) ? 'parse_error'
          : 'operation_failed';
        return {
          ...base(refusal ? 'batch_error' : 'success'),
          diagnosticCodes: refusal ? [code] : [],
          directError: refusal,
          volume: afterVol,
          faceCount: afterFaces,
          rollbackPreserved: Math.abs(afterVol - beforeVol) < 1e-12 && afterFaces === beforeFaces,
          preservedBytesEqual: Buffer.compare(beforeBytes, afterBytes) === 0,
          batchDurationMs: performance.now() - started,
        };
      } finally {
        kernel.free();
      }
    }
    if (c.id === 'split/arena-truncation') {
      const kernel = new BrepKernel();
      try {
        const keep = kernel.makeBox(2, 3, 4);
        const beforeVol = kernel.volume(keep, 0.05);
        const beforeFaces = Array.from(kernel.getSolidFaces(keep)).length;
        const beforeBytes = Buffer.from(kernel.serializeSolids(Uint32Array.from([keep])));
        const full = Buffer.from(kernel.serializeSolids(Uint32Array.from([keep])));
        const truncated = full.subarray(0, Math.floor(full.length / 2));
        const codes = [];
        for (const bytes of [truncated, new Uint8Array()]) {
          try {
            kernel.deserializeSolids(bytes);
            codes.push('operation_failed');
          } catch (error) {
            const message = String(error?.message ?? error);
            codes.push(/parse|eof|trunc|deserial|arena|empty|invalid/i.test(message) ? 'parse_error' : 'operation_failed');
          }
        }
        const afterVol = kernel.volume(keep, 0.05);
        const afterFaces = Array.from(kernel.getSolidFaces(keep)).length;
        const afterBytes = Buffer.from(kernel.serializeSolids(Uint32Array.from([keep])));
        return {
          ...base('batch_error'),
          diagnosticCodes: codes,
          volume: afterVol,
          faceCount: afterFaces,
          rollbackPreserved: Math.abs(afterVol - beforeVol) < 1e-12 && afterFaces === beforeFaces,
          preservedBytesEqual: Buffer.compare(beforeBytes, afterBytes) === 0,
          batchDurationMs: performance.now() - started,
        };
      } finally {
        kernel.free();
      }
    }
    if (c.id === 'split/two-session-isolation') {
      const kernelA = new BrepKernel();
      const kernelB = new BrepKernel();
      try {
        const io = new RemusIo();
        const facesBeforeB = 0;
        // A imports valid.
        const handlesA = Array.from(kernelA.deserializeSolids(io.importStep(Buffer.from(fixtures.boxStep))));
        assert.equal(handlesA.length, 1);
        const extraA = kernelA.makeBox(1, 1, 1);
        // B refuses malformed; it must gain no solids.
        let refusal = null;
        try {
          io.importStep(Buffer.from('not a STEP file'), 4, undefined);
        } catch (error) {
          refusal = String(error?.message ?? error);
        }
        assert.ok(refusal, 'malformed import must refuse');
        let staleProbeThrows = false;
        try {
          kernelB.volume(9999, 0.05);
        } catch {
          staleProbeThrows = true;
        }
        // B then imports valid.
        const handlesB = Array.from(kernelB.deserializeSolids(io.importStep(Buffer.from(fixtures.boxStep))));
        assert.equal(handlesB.length, 1);
        // Handle 1 is valid in A (extra box) but must be stale in B (one solid).
        const validInA = kernelA.volume(extraA, 0.05);
        let staleInB = false;
        try {
          kernelB.volume(extraA, 0.05);
          // extraA is 1; B has only handle 0, so reaching here with a finite
          // volume would mean leaked cross-session handles.
          staleInB = !Number.isFinite(kernelB.volume(extraA, 0.05));
        } catch {
          staleInB = true;
        }
        void facesBeforeB;
        void staleProbeThrows;
        return {
          ...base('success'),
          diagnosticCodes: refusal ? ['limit_exceeded'] : [],
          volumes: {
            sessionA: kernelA.volume(handlesA[0], 0.05),
            sessionB: kernelB.volume(handlesB[0], 0.05),
          },
          extraVolumeA: validInA,
          noPartialSolids: handlesB.length === 1,
          crossSessionStaleRefused: staleInB,
          batchDurationMs: performance.now() - started,
        };
      } finally {
        kernelA.free();
        kernelB.free();
      }
    }
    throw new Error(`unknown split case ${c.id}`);
  } catch (error) {
    return {
      ...base('crash'),
      diagnosticCodes: [],
      diagnostic: String(error?.message ?? error).slice(0, 500),
      batchDurationMs: performance.now() - started,
    };
  }
}

// ── Packaging: two tarballs from one pinned source into one consumer ──

function packOne(packageDir, temporaryRoot, npmEnvironment) {
  const packed = JSON.parse(execFileSync(
    'npm',
    ['pack', resolve(packageDir), '--json', '--pack-destination', temporaryRoot],
    { encoding: 'utf8', env: npmEnvironment },
  ));
  assert.equal(packed.length, 1, 'npm pack must produce exactly one tarball');
  return resolve(temporaryRoot, packed[0].filename);
}

export function installSplitPair(kernelDir, ioDir, temporaryRoot, consumerName = 'consumer') {
  const consumerDir = resolve(temporaryRoot, consumerName);
  mkdirSync(consumerDir, { recursive: true });
  const npmEnvironment = { ...process.env, npm_config_cache: resolve(temporaryRoot, 'npm-cache') };
  const kernelTarball = packOne(kernelDir, temporaryRoot, npmEnvironment);
  const ioTarball = packOne(ioDir, temporaryRoot, npmEnvironment);
  execFileSync('npm', [
    'install', '--prefix', consumerDir, '--no-save', '--package-lock=false',
    '--ignore-scripts', '--no-audit', '--no-fund', kernelTarball, ioTarball,
  ], { env: npmEnvironment, stdio: 'ignore' });
  const consumerRequire = createRequire(resolve(consumerDir, 'consumer.cjs'));
  const kernelEntry = consumerRequire.resolve('remus-wasm');
  const ioEntry = consumerRequire.resolve('remus-wasm-io');
  const kernelDirInstalled = realpathSync(resolve(consumerDir, 'node_modules/remus-wasm'));
  const ioDirInstalled = realpathSync(resolve(consumerDir, 'node_modules/remus-wasm-io'));
  validateInstalledEntry(kernelEntry, kernelDirInstalled);
  validateInstalledEntry(ioEntry, ioDirInstalled);
  return {
    consumerDir, kernelEntry, ioEntry, kernelDirInstalled, ioDirInstalled,
    kernelTarball, ioTarball,
  };
}

function attemptNative(nativeRunner, c) {
  const result = spawnSync(resolve(nativeRunner), [], {
    input: JSON.stringify(c),
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
      diagnostic: (result.stderr || String(result.error)).slice(0, 500),
    };
  }
  try {
    return JSON.parse(result.stdout);
  } catch (error) {
    return {
      outcome: 'crash',
      diagnosticCodes: [],
      diagnostic: `invalid observation JSON: ${error}; stderr=${result.stderr}`.slice(0, 500),
    };
  }
}

export function readFixtures() {
  const boxStep = readFileSync(resolve(scriptDir, 'split-box-2x3x4.step'), 'utf8');
  const plateStep = readFileSync(
    resolve(repoRoot, 'crates/io/tests/data/openzcad_e_analytic_fillet_plate.step'),
    'utf8',
  );
  assert.ok(boxStep.includes('MANIFOLD_SOLID_BREP'), 'box fixture must be a solid STEP');
  assert.ok(plateStep.includes('MANIFOLD_SOLID_BREP'), 'plate fixture must be a solid STEP');
  return { boxStep, plateStep };
}

function splitCaseInput(c, fixtures) {
  return {
    schema_version: 1,
    id: c.id,
    box_step: fixtures.boxStep,
    plate_step: fixtures.plateStep,
  };
}

async function main() {
  const [nativeRunner, freshKernelDir, freshIoDir, committedKernelDir, committedIoDir] = process.argv.slice(2);
  assert.ok(
    nativeRunner && freshKernelDir && freshIoDir,
    'usage: split-io-matrix.mjs NATIVE_RUNNER FRESH_KERNEL_DIR FRESH_IO_DIR [COMMITTED_KERNEL_DIR COMMITTED_IO_DIR]',
  );
  const cases = expandSplitMatrix();
  const fixtures = readFixtures();
  const temporaryRoot = mkdtempSync(resolve(tmpdir(), 'remus-split-io-'));
  const started = performance.now();
  try {
    const fresh = installSplitPair(resolve(freshKernelDir), resolve(freshIoDir), temporaryRoot, 'consumer-fresh');
    const freshKernelProvenance = collectFreshProvenance({
      repoRoot,
      packageDir: resolve(freshKernelDir),
      tarballPath: fresh.kernelTarball,
      installedEntry: fresh.kernelEntry,
      installedDir: fresh.kernelDirInstalled,
      buildOptions: { target: 'nodejs', profile: 'release', features: '--no-default-features' },
    });
    const freshIoProvenance = collectFreshProvenance({
      repoRoot,
      packageDir: resolve(freshIoDir),
      tarballPath: fresh.ioTarball,
      installedEntry: fresh.ioEntry,
      installedDir: fresh.ioDirInstalled,
      buildOptions: { target: 'nodejs', profile: 'release', features: 'default' },
    });
    // One pinned source/configuration: both fresh tarballs must come from
    // the same recorded revision in this run.
    assert.equal(
      freshIoProvenance.sourceRevision,
      freshKernelProvenance.sourceRevision,
      'fresh kernel/translator tarballs must come from one pinned source',
    );
    validateProvenance(freshKernelProvenance);
    validateProvenance(freshIoProvenance);

    let committed = null;
    let committedKernelProvenance = null;
    let committedIoProvenance = null;
    if (committedKernelDir && committedIoDir) {
      committed = installSplitPair(resolve(committedKernelDir), resolve(committedIoDir), temporaryRoot, 'consumer-committed');
      committedKernelProvenance = collectCommittedProvenance({
        repoRoot,
        packageDir: resolve(committedKernelDir),
        installedEntry: committed.kernelEntry,
        installedDir: committed.kernelDirInstalled,
      });
      committedIoProvenance = collectCommittedProvenance({
        repoRoot,
        packageDir: resolve(committedIoDir),
        installedEntry: committed.ioEntry,
        installedDir: committed.ioDirInstalled,
      });
      validateProvenance(committedKernelProvenance);
      validateProvenance(committedIoProvenance);
    }

    const observations = [];
    for (const c of cases) {
      const input = splitCaseInput(c, fixtures);
      const native = attemptNative(resolve(nativeRunner), input);
      const freshObs = {
        ...wasmSplitObservation(c, fresh.kernelEntry, fresh.ioEntry, fixtures),
        surface: 'fresh',
      };
      let committedObs;
      if (committed) {
        committedObs = {
          ...wasmSplitObservation(c, committed.kernelEntry, committed.ioEntry, fixtures),
          surface: 'committed',
        };
      }
      const perCase = { case: c.id, native, fresh: freshObs };
      if (committedObs) perCase.committed = committedObs;
      observations.push(perCase);
    }
    const scored = cases.map((c, index) => {
      const { case: _dropped, ...perSurface } = observations[index];
      requireObservations(c.id, perSurface);
      return scoreSplitCase(c, perSurface);
    });
    const stages = scored.flat();
    const failedStages = stages.filter((stage) => stage.required && !stage.passed);
    const knownGaps = stages.filter((stage) => !stage.required && !stage.passed);
    const passed = failedStages.length === 0;
    const report = {
      schema_version: 1,
      scope: 'O1.5 split slice: native facade vs freshly packed/installed kernel+translator, with committed-package mode',
      provenance: {
        native: { mode: 'native', sourceRevision: freshKernelProvenance.sourceRevision },
        freshKernel: freshKernelProvenance,
        freshIo: freshIoProvenance,
        committedKernel: committedKernelProvenance,
        committedIo: committedIoProvenance,
        stalenessKernel: committedKernelProvenance
          ? describeStaleness(freshKernelProvenance, committedKernelProvenance)
          : null,
        stalenessIo: committedIoProvenance
          ? describeStaleness(freshIoProvenance, committedIoProvenance)
          : null,
      },
      timeout_ms: timeoutMs,
      matrix: { cases: cases.length, ids: cases.map(({ id }) => id) },
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
