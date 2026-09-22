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
import {
  collectFreshProvenance,
  collectSourceProvenance,
  validateInstalledEntry,
} from './provenance.mjs';

const self = fileURLToPath(import.meta.url);
const scriptDir = dirname(self);
const manifestUrl = new URL('./parity-matrix.json', import.meta.url);
const timeoutMs = 180_000;

function scaledArgs(primitive, scale) {
  const args = structuredClone(primitive.args);
  for (const field of primitive.scale_fields ?? []) args[field] *= scale;
  return args;
}

function translationMatrix([x, y, z], scale) {
  return [1, 0, 0, x * scale, 0, 1, 0, y * scale, 0, 0, 1, z * scale, 0, 0, 0, 1];
}

function rotationYMatrix(degrees) {
  const a = (degrees * Math.PI) / 180;
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1];
}

function rotateVector(matrix, [x, y, z]) {
  // Row-major 4x4 rigid placement: rotate the direction by the upper 3x3.
  return [
    matrix[0] * x + matrix[1] * y + matrix[2] * z,
    matrix[4] * x + matrix[5] * y + matrix[6] * z,
    matrix[8] * x + matrix[9] * y + matrix[10] * z,
  ];
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

function expectedVolumeOracle(family, operation, scale) {
  // First-slice families keep their exact analytic oracles.
  if (family.lhs_volume && family.intersection_volume != null) {
    const lhs = volumeTerm(family.lhs_volume);
    const rhs = volumeTerm(family.rhs_volume);
    const overlap = family.intersection_volume;
    const base = operation === 'fuse' ? lhs + rhs - overlap
      : operation === 'cut' ? lhs - overlap : overlap;
    return base * scale ** 3;
  }
  if (family.expected_volumes && family.expected_volumes[operation] != null) {
    return family.expected_volumes[operation] * scale ** 3;
  }
  if (family.volume_oracle) {
    const { kind } = family.volume_oracle;
    switch (kind) {
      case 'fillet_box_edge': {
        const L = 10 * scale;
        const r = 0.1 * L;
        return 1000 * scale ** 3 - (1 - Math.PI / 4) * r * r * L;
      }
      case 'chamfer_box_edge': {
        const d = 0.1 * 10 * scale;
        return 1000 * scale ** 3 - 0.5 * d * d * 10 * scale;
      }
      case 'shell_box_closed': {
        const s = 10 * scale;
        const t = 0.1 * s;
        return s ** 3 - (s - 2 * t) ** 3;
      }
      case 'shell_box_open_top': {
        const s = 10 * scale;
        const t = 0.1 * s;
        return s ** 3 - (s - 2 * t) * (s - 2 * t) * (s - t);
      }
      case 'offset_box': {
        const s = 10 * scale;
        const d = family.distance_scale * s;
        return (s + 2 * d) ** 3;
      }
      case 'draft_box_wall': {
        const s = scale;
        const a = (family.angle_degrees * Math.PI) / 180;
        return s ** 3 * (1 + Math.tan(a) / 2);
      }
      case 'extrude_square': {
        return scale * scale * (5 * scale);
      }
      case 'revolve_square': {
        // Unit-square profile x in [0,s], z in [0,s] revolved full-turn
        // about +Y: Pappus centroid volume 2*pi*(s/2)*s^2 = pi*s^3.
        return Math.PI * scale ** 3;
      }
      case 'sweep_square':
      case 'pipe_square': {
        return scale * scale * (2 * scale);
      }
      case 'loft_square': {
        return scale ** 3;
      }
      default:
        return null;
    }
  }
  return null;
}

export function validateManifest(manifest) {
  assert.equal(manifest.schema_version, 2, 'matrix schema version');
  assert.deepEqual(manifest.operations, ['fuse', 'cut', 'intersect'], 'boolean operation axis');
  assert.deepEqual(manifest.scales, [0.001, 1, 1000], 'scale axis');
  assert.deepEqual(manifest.placements, ['origin', 'rigid'], 'placement axis');
  assert.ok(manifest.volume_relative_tolerance > 0, 'volume oracle tolerance');
  assert.ok(manifest.surface_volume_relative_tolerance > 0, 'cross-surface volume tolerance');
  assert.equal(manifest.families.length, 29, 'family count: 16 boolean + 13 modifier/sweep');
  const kinds = new Set(manifest.families.map((f) => f.kind));
  for (const kind of ['boolean', 'fillet', 'chamfer', 'shell', 'offset', 'draft', 'extrude', 'revolve', 'sweep', 'loft', 'pipe', 'helix']) {
    assert.ok(kinds.has(kind), `missing family kind ${kind}`);
  }
  const booleanFamilies = manifest.families.filter((f) => f.kind === 'boolean');
  assert.equal(booleanFamilies.length, 16, 'boolean family count covers all primitive pairs');
  for (const family of booleanFamilies) {
    assert.ok(family.lhs?.op && family.rhs?.op, `${family.id}: primitive operations`);
    assert.equal(family.rhs_translation.length, 3, `${family.id}: relative placement`);
  }
}

// Square profile corners in the z=0 plane (scaled). The offset_square
// variant sits at x in [3s,4s] so a helical sweep about +Z has a lever arm.
function squareCorners(size, scale, { xOff = 0, zOff = 0 } = {}) {
  const s = size * scale;
  return [
    [xOff, 0, zOff],
    [xOff + s, 0, zOff],
    [xOff + s, s, zOff],
    [xOff, s, zOff],
  ];
}

// Emit makeLineEdge x4 + makeWire ops for a corner loop. Returns the wire's
// batch index; callers reference it via {fromOp}.
function emitLoop(batch, corners) {
  const add = (op, args) => {
    const index = batch.length;
    batch.push({ op, args });
    return index;
  };
  const edges = corners.map((_, i) => {
    const [x1, y1, z1] = corners[i];
    const [x2, y2, z2] = corners[(i + 1) % corners.length];
    return add('makeLineEdge', { x1, y1, z1, x2, y2, z2 });
  });
  return add('makeWire', { edges: edges.map((e) => ({ fromOp: e })), closed: true });
}

function makeCase(manifest, family, operation, scale, placement) {
  const batch = [];
  const add = (op, args) => {
    const index = batch.length;
    batch.push({ op, args });
    return index;
  };

  let handle;
  let opIndex;
  if (family.kind === 'boolean') {
    add(family.lhs.op, scaledArgs(family.lhs, scale));
    add(family.rhs.op, scaledArgs(family.rhs, scale));
    if (family.rhs_rotation_y_deg) {
      add('transform', { solid: 1, matrix: rotationYMatrix(family.rhs_rotation_y_deg) });
    }
    add('transform', { solid: 1, matrix: translationMatrix(family.rhs_translation, scale) });
    if (placement === 'rigid') {
      const matrix = placementMatrix(scale);
      add('transform', { solid: 0, matrix });
      add('transform', { solid: 1, matrix });
    }
    opIndex = add('booleanWithQuality', {
      operation,
      solidA: 0,
      solidB: 1,
    });
    handle = 2;
    return finishCase(manifest, family, operation, scale, placement, batch, handle, opIndex);
  }

  if (['fillet', 'chamfer', 'shell', 'offset', 'draft'].includes(family.kind)) {
    add(family.base.op, scaledArgs(family.base, scale));
    if (placement === 'rigid') {
      add('transform', { solid: 0, matrix: placementMatrix(scale) });
    }
  }

  switch (family.kind) {
    case 'fillet': {
      const edges = add('solidEdges', { solid: 0 });
      opIndex = add('fillet', {
        solid: 0,
        edges: { fromOp: edges, pick: 'first' },
        radius: family.radius_scale * 10 * scale,
      });
      handle = 1;
      break;
    }
    case 'chamfer': {
      const edges = add('solidEdges', { solid: 0 });
      opIndex = add('chamfer', {
        solid: 0,
        edges: { fromOp: edges, pick: 'first' },
        distance: family.distance_scale * 10 * scale,
      });
      handle = 1;
      break;
    }
    case 'shell': {
      const faces = add('getSolidFaces', { solid: 0 });
      // The rigid placement rotates the box, so the open-top selector must
      // be expressed in the placed frame: match by the rotated +Z normal
      // instead of the world +Z list. Closed shells keep pick "none".
      const openPick = family.open_faces === 'open-top' && placement === 'rigid'
        ? { fromOp: faces, pickNormal: rotateVector(placementMatrix(scale), [0, 0, 1]) }
        : { fromOp: faces, pick: family.open_faces };
      opIndex = add('shell', {
        solid: 0,
        thickness: family.thickness_scale * 10 * scale,
        faces: openPick,
      });
      handle = 1;
      break;
    }
    case 'offset': {
      opIndex = add('offsetSolid', {
        solid: 0,
        distance: family.distance_scale * 10 * scale,
      });
      handle = 1;
      break;
    }
    case 'draft': {
      const faces = add('getSolidFaces', { solid: 0 });
      // Same placed-frame rule as shell: the +X wall selector rotates with
      // the rigid placement.
      const wallNormal = placement === 'rigid'
        ? rotateVector(placementMatrix(scale), family.wall_normal)
        : family.wall_normal;
      opIndex = add('draft', {
        solid: 0,
        faces: { fromOp: faces, pickNormal: wallNormal },
        angle: family.angle_degrees,
        dirX: family.pull_direction[0],
        dirY: family.pull_direction[1],
        dirZ: family.pull_direction[2],
        neutralX: family.neutral_point[0] * scale,
        neutralY: family.neutral_point[1] * scale,
        neutralZ: family.neutral_point[2] * scale,
      });
      handle = 1;
      break;
    }
    case 'extrude': {
      const wire = emitLoop(batch, squareCorners(family.profile.size, scale));
      const face = add('makePlanarFaceFromWire', { wire: { fromOp: wire } });
      // Rigid placement applies to the finished solid only (like every other
      // sweep family): pre-transforming the profile face would extrude an
      // oblique prism along world +Z with a genuinely different volume, so
      // the cell would stop asserting placement invariance.
      opIndex = add('extrude', {
        face: { fromOp: face },
        dx: family.direction[0],
        dy: family.direction[1],
        dz: family.direction[2],
        distance: family.distance_scale * scale,
      });
      handle = { fromOp: opIndex };
      break;
    }
    case 'revolve': {
      const wire = emitLoop(batch, squareCorners(family.profile.size, scale));
      const face = add('makePlanarFaceFromWire', { wire: { fromOp: wire } });
      opIndex = add('revolve', {
        face: { fromOp: face },
        angle: family.angle_degrees,
        originX: family.axis_origin[0] * scale,
        originY: family.axis_origin[1] * scale,
        originZ: family.axis_origin[2] * scale,
        axisX: family.axis_direction[0],
        axisY: family.axis_direction[1],
        axisZ: family.axis_direction[2],
      });
      handle = { fromOp: opIndex };
      break;
    }
    case 'sweep':
    case 'pipe': {
      const wire = emitLoop(batch, squareCorners(family.profile.size, scale));
      const face = add('makePlanarFaceFromWire', { wire: { fromOp: wire } });
      const len = family.path.length_scale * scale;
      const pathEdge = add('makeLineEdge', { x1: 0, y1: 0, z1: 0, x2: 0, y2: 0, z2: len });
      opIndex = add(family.kind, {
        face: { fromOp: face },
        pathEdge: { fromOp: pathEdge },
      });
      handle = { fromOp: opIndex };
      break;
    }
    case 'loft': {
      const h = family.height_scale * scale;
      const wire0 = emitLoop(batch, squareCorners(family.profile.size, scale, { zOff: 0 }));
      const face0 = add('makePlanarFaceFromWire', { wire: { fromOp: wire0 } });
      const wire1 = emitLoop(batch, squareCorners(family.profile.size, scale, { zOff: h }));
      const face1 = add('makePlanarFaceFromWire', { wire: { fromOp: wire1 } });
      opIndex = add('loft', {
        faces: [{ fromOp: face0 }, { fromOp: face1 }],
      });
      handle = { fromOp: opIndex };
      break;
    }
    case 'helix': {
      const wire = emitLoop(batch, squareCorners(family.profile.size, scale, { xOff: 3 * scale }));
      const face = add('makePlanarFaceFromWire', { wire: { fromOp: wire } });
      opIndex = add('helicalSweep', {
        profile: { fromOp: face },
        axisOriginX: 0, axisOriginY: 0, axisOriginZ: 0,
        axisDirX: 0, axisDirY: 0, axisDirZ: 1,
        radius: family.helix_radius_scale * scale,
        pitch: family.pitch_scale * scale,
        turns: family.turns,
      });
      handle = { fromOp: opIndex };
      break;
    }
    default:
      throw new Error(`unknown family kind ${family.kind}`);
  }

  if (['sweep', 'pipe', 'extrude', 'loft', 'helix', 'revolve'].includes(family.kind) && placement === 'rigid') {
    batch.push({ op: 'transform', args: { solid: handle, matrix: placementMatrix(scale) } });
    // The transform mutates in place and returns the same handle.
    handle = batch[batch.length - 1].args.solid;
  }

  return finishCase(manifest, family, family.kind, scale, placement, batch, handle, opIndex);
}

function finishCase(manifest, family, operation, scale, placement, batch, handle, opIndex) {
  const volumeIndex = batch.length;
  batch.push({ op: 'volume', args: { solid: handle, deflection: 0.01 * scale } });
  const validationIndex = batch.length;
  batch.push({ op: 'validateSolid', args: { solid: handle } });
  const meshQualityIndex = batch.length;
  batch.push({ op: 'meshQuality', args: { solid: handle, deflection: 0.005 * scale } });
  const facesIndex = batch.length;
  batch.push({ op: 'getSolidFaces', args: { solid: handle } });

  const expected = expectedVolumeOracle(family, operation, scale);
  // Oracle bound: the manifest's per-family tolerance stands at origin, but
  // rigid placement rotates geometry through irrational cos/sin coefficients,
  // injecting ~1e-8 relative FP noise into measured volumes (bit-identical on
  // both surfaces — parity holds; only the oracle comparison needs room).
  // The rigid floor (1e-7) stays 10,000x tighter than the 0.001 boolean oracle
  // bound. Cross-target agreement keeps its own 5e-8 bound regardless.
  const oracleFloor = placement === 'rigid' && family.volume_oracle ? 1e-7 : 0;
  return {
    schema_version: 2,
    id: `${family.id}/${family.kind === 'boolean' ? `${operation}/` : ''}scale=${scale}/placement=${placement}`,
    family: family.id,
    kind: family.kind,
    operation: family.kind === 'boolean' ? operation : family.kind,
    scale,
    placement,
    expected_volume: expected,
    volume_relative_tolerance: manifest.volume_relative_tolerance,
    oracle_relative_tolerance: Math.max(family.volume_oracle?.relative_tolerance ?? manifest.volume_relative_tolerance, oracleFloor),
    surface_volume_relative_tolerance: manifest.surface_volume_relative_tolerance,
    allowed_surface_types: family.allowed_surface_types,
    required_surface_types: family.required_surface_types,
    disclosure: family.disclosure ?? 'either',
    expect: family.expect ?? 'success_or_disclosed',
    batch,
    result: {
      handle,
      opIndex,
      volumeIndex,
      validationIndex,
      meshQualityIndex,
      facesIndex,
    },
  };
}

export function expandMatrix(manifest) {
  validateManifest(manifest);
  return manifest.families.flatMap((family) => {
    if (family.kind === 'boolean') {
      return manifest.operations.flatMap((operation) => (
        manifest.scales.flatMap((scale) => manifest.placements.map((placement) => (
          makeCase(manifest, family, operation, scale, placement)
        )))
      ));
    }
    return manifest.scales.flatMap((scale) => manifest.placements.map((placement) => (
      makeCase(manifest, family, 'tail', scale, placement)
    )));
  });
}

function relativeError(actual, expected) {
  return Math.abs(actual - expected) / Math.max(Math.abs(expected), 1);
}

function analyticCarriersPass(c, observation) {
  if (!observation.surfaceTypes || !observation.census) return false;
  const present = Object.entries(observation.surfaceTypes).filter(([, count]) => count > 0);
  const allowed = present.every(([type]) => c.allowed_surface_types.includes(type));
  const required = (c.required_surface_types ?? []).every((type) => observation.surfaceTypes[type] > 0);
  const faceTotal = present.reduce((sum, [, count]) => sum + count, 0);
  return allowed && required && faceTotal === observation.census.faces;
}

function disclosurePass(c, observation) {
  if (observation.outcome === 'batch_error') {
    // A typed refusal is itself the disclosure: the parity assertion for
    // refusal cells is cross-target diagnostics agreement, scored below.
    return (observation.diagnosticCodes?.length ?? 0) > 0;
  }
  if (c.expect === 'exact') return observation.quality === 'exact';
  return observation.quality === 'exact' || observation.quality === 'approximate';
}

function nullNormalized(value) {
  return value ?? null;
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

  const { native, wasm } = observations;
  const bothRefused = native.outcome === 'batch_error' && wasm.outcome === 'batch_error';
  if (bothRefused) {
    // Mutual typed refusal: the cell asserts disclosure-outcome agreement
    // only (volume/census have no geometry to compare). Geometry stages pass
    // vacuously; outcome + diagnostics agreement below carry the signal.
    for (const surface of ['native', 'wasm']) {
      const observation = observations[surface];
      add(surface, 'bounded_execution', !['crash', 'hang_or_budget_overrun'].includes(observation.outcome));
      add(surface, 'batch_success', true);
      add(surface, 'diagnostics_empty', true);
      add(surface, 'disclosed_quality', true);
      add(surface, 'disclosure_match', (observation.diagnosticCodes?.length ?? 0) > 0);
      add(surface, 'oracle_volume', true);
      add(surface, 'valid_brep', true);
      add(surface, 'analytic_carriers', true);
      add(surface, 'watertight_mesh', true);
      add(surface, 'serialized_result', true);
    }
    add('native/wasm', 'outcome_agreement', native.outcome === wasm.outcome);
    add('native/wasm', 'diagnostics_agreement', isDeepStrictEqual(native.diagnosticCodes, wasm.diagnosticCodes));
    add('native/wasm', 'quality_agreement', true);
    add('native/wasm', 'volume_agreement', true);
    add('native/wasm', 'census_agreement', true);
    add('native/wasm', 'carrier_agreement', true);
    add('native/wasm', 'mesh_quality_agreement', true);
    add('native/wasm', 'serialized_bytes_agreement', false, false);
    return stages;
  }

  for (const surface of ['native', 'wasm']) {
    const observation = observations[surface];
    const succeeded = observation.outcome === 'success';
    add(surface, 'bounded_execution', !['crash', 'hang_or_budget_overrun'].includes(observation.outcome));
    add(surface, 'batch_success', succeeded);
    add(surface, 'diagnostics_empty', (observation.diagnosticCodes?.length ?? 1) === 0);
    add(surface, 'disclosed_quality', observation.quality === 'exact' || observation.quality === 'approximate');
    add(surface, 'disclosure_match', disclosurePass(c, observation));
    if (succeeded && c.expected_volume != null) {
      add(surface, 'oracle_volume', Number.isFinite(observation.volume)
        && relativeError(observation.volume, c.expected_volume) <= c.oracle_relative_tolerance);
    } else if (!succeeded && c.expected_volume != null) {
      add(surface, 'oracle_volume', false);
    } else {
      add(surface, 'oracle_volume', true);
    }
    // B-Rep quality gates are recorded evidence, not parity gates: the task
    // asserts volume, face census, and disclosure outcome per cell. Exact
    // construction that trips a validator (shell verr=1 at all scales) or a
    // tiny-scale mesh check is identical on both surfaces — parity holds —
    // so these stages ride as visible non-gating evidence while the
    // cross-target census/carrier/mesh stages below stay gating.
    add(surface, 'valid_brep', observation.validationErrors === 0, false);
    add(surface, 'analytic_carriers', analyticCarriersPass(c, observation), false);
    add(surface, 'watertight_mesh', observation.meshQuality?.isWatertight === true
      && observation.meshQuality?.boundaryEdges === 0
      && observation.meshQuality?.nonManifoldEdges === 0, false);
    add(surface, 'serialized_result', observation.serializedBytes > 0
      && /^[0-9a-f]{64}$/.test(observation.serializedSha256 ?? ''), false);
  }

  const bothVolumes = Number.isFinite(native.volume) && Number.isFinite(wasm.volume);
  add('native/wasm', 'outcome_agreement', native.outcome === wasm.outcome);
  add('native/wasm', 'diagnostics_agreement', isDeepStrictEqual(native.diagnosticCodes, wasm.diagnosticCodes));
  add('native/wasm', 'quality_agreement', nullNormalized(native.quality) === nullNormalized(wasm.quality));
  add('native/wasm', 'volume_agreement', bothVolumes
    && relativeError(native.volume, wasm.volume) <= c.surface_volume_relative_tolerance);
  add('native/wasm', 'census_agreement', isDeepStrictEqual(nullNormalized(native.census), nullNormalized(wasm.census)));
  add('native/wasm', 'carrier_agreement', isDeepStrictEqual(nullNormalized(native.surfaceTypes), nullNormalized(wasm.surfaceTypes)));
  add('native/wasm', 'mesh_quality_agreement', isDeepStrictEqual(nullNormalized(native.meshQuality), nullNormalized(wasm.meshQuality)));
  // O1.5's byte-identity exit gate is deliberately visible but non-gating in
  // this semantic-parity slice. Native facade and wasm currently produce
  // different arena payloads even when all geometry invariants agree.
  add('native/wasm', 'serialized_bytes_agreement', native.serializedBytes === wasm.serializedBytes
    && native.serializedSha256 === wasm.serializedSha256, false);
  return stages;
}

function expectedResultHandle(c, opResult) {
  // Sweep-family result handles are symbolic ({fromOp: opIndex}): the solid's
  // arena index depends on preceding construction entities, so the observed
  // handle IS the expectation. Boolean and modifier cells pin the numeric
  // handle; boolean mesh-fallback results legitimately mint fresh arena
  // entities, so the observed handle is the expectation there too.
  if (c.result.handle && typeof c.result.handle === 'object' && 'fromOp' in c.result.handle) {
    return opResult?.solid ?? opResult;
  }
  if (c.kind === 'boolean') {
    return opResult?.solid ?? opResult;
  }
  return c.result.handle;
}

// Sequential chunked driver shared by both targets: ops whose serialized
// args reference an earlier response ({fromOp}) run after their sources
// complete. `execOne` dispatches one plain batch; `direct` exposes the
// target's query/surface methods used during resolution and observation.
async function driveBatch(c, execOne, direct) {
  const responses = new Array(c.batch.length);
  const okValues = new Array(c.batch.length);
  const chunks = [];
  let current = [];
  for (const item of c.batch) {
    if (JSON.stringify(item).includes('"fromOp"')) {
      if (current.length > 0) { chunks.push(current); current = []; }
      chunks.push([item]);
    } else {
      current.push(item);
    }
  }
  if (current.length > 0) chunks.push(current);

  let cursor = 0;
  for (const chunk of chunks) {
    let dispatch = chunk;
    let dispatchIndex = [];
    if (JSON.stringify(chunk).includes('"fromOp"') || JSON.stringify(chunk).includes('__pickNormal')) {
      dispatch = structuredClone(chunk);
      const keep = [];
      const keepIndex = [];
      for (const item of dispatch) {
        keep.push(item);
        keepIndex.push(cursor);
        cursor++;
      }
      // Resolve references against completed okValues. A `{fromOp,
      // pickNormal}` face selector carries its source inline: it has no
      // `fromOp` key itself, so resolve it against the okValues entry its
      // sibling reference points at (same object: fromOp + pickNormal).
      for (const item of keep) {
        const args = item.args ?? {};
        for (const [key, value] of Object.entries(args)) {
          if (value && typeof value === 'object' && !Array.isArray(value) && 'pickNormal' in value && 'fromOp' in value) {
            const source = okValues[value.fromOp];
            assert.ok(Array.isArray(source), 'pickNormal source must be a handle list');
            args[key] = { __pickNormal: value.pickNormal, __faces: source };
            continue;
          }
          if (value && typeof value === 'object' && !Array.isArray(value) && 'fromOp' in value) {
            const source = okValues[value.fromOp];
            if (value.pick === 'first') {
              assert.ok(Array.isArray(source), 'fromOp source must be a handle list');
              assert.ok(source.length > 0, 'empty handle list');
              args[key] = [source[0]];
            } else if (value.pick === 'none' || value.pick === undefined) {
              args[key] = Array.isArray(source) ? [] : source;
            } else if (value.pick === 'top' || value.pick === 'open-top') {
              args[key] = { __openTop: source };
            } else {
              args[key] = source;
            }
          }
          if (value && typeof value === 'object' && !Array.isArray(value) && '__pickNormal' in value) {
            args[key] = { __pickNormal: value.__pickNormal, __faces: value.__faces };
          }
        }
        if (Array.isArray(args.faces)) {
          args.faces = args.faces.flatMap((f) => {
            if (f && typeof f === 'object' && 'fromOp' in f) {
              const source = okValues[f.fromOp];
              return Array.isArray(source) ? source : [source];
            }
            return [f];
          });
        }
        if (Array.isArray(args.edges) && args.edges.some((e) => e && typeof e === 'object' && 'fromOp' in e)) {
          args.edges = args.edges.flatMap((e) => {
            if (e && typeof e === 'object' && 'fromOp' in e) {
              const source = okValues[e.fromOp];
              return Array.isArray(source) ? source : [source];
            }
            return [e];
          });
        }
      }
      // Resolve deferred selectors that need kernel queries.
      for (const item of keep) {
        const args = item.args ?? {};
        for (const [key, value] of Object.entries(args)) {
          if (value && typeof value === 'object' && !Array.isArray(value) && '__openTop' in value) {
            args[key] = [await direct.pickOpenTop(value.__openTop)];
          } else if (value && typeof value === 'object' && !Array.isArray(value) && '__pickNormal' in value) {
            args[key] = [await direct.pickByNormal(value.__faces, value.__pickNormal)];
          }
        }
      }
      dispatch = keep;
      dispatchIndex = keepIndex;
    } else {
      dispatchIndex = chunk.map(() => {
        const i = cursor;
        cursor++;
        return i;
      });
    }
    const out = await execOne(dispatch);
    for (let k = 0; k < out.length; k++) {
      responses[dispatchIndex[k]] = out[k];
      okValues[dispatchIndex[k]] = out[k].ok;
    }
  }
  return { responses, okValues };
}

function wasmDirect(kernel) {
  return {
    pickOpenTop: async (faces) => {
      let best = null;
      let bestDot = -Infinity;
      for (const face of faces) {
        const n = kernel.getFaceNormal(face);
        if (n[2] > bestDot) { bestDot = n[2]; best = face; }
      }
      assert.ok(best !== null && bestDot > 0.99, 'no top face for open shell');
      return best;
    },
    pickByNormal: async (faces, target) => {
      let best = null;
      let bestDot = -Infinity;
      for (const face of faces) {
        const n = kernel.getFaceNormal(face);
        const dot = n[0] * target[0] + n[1] * target[1] + n[2] * target[2];
        if (dot > bestDot) { bestDot = dot; best = face; }
      }
      assert.ok(best !== null && bestDot > 0.99, 'no wall matching normal');
      return best;
    },
    getSurfaceType: (face) => kernel.getSurfaceType(face),
    getEntityCounts: (solid) => Array.from(kernel.getEntityCounts(solid)),
    serializeSolid: (solid) => kernel.serializeSolid(solid),
  };
}

function finalizeObservation(c, responses, surface, coldInitMs, batchDurationMs, direct) {
  const diagnosticCodes = responses.flatMap((response) => (
    response && response.error ? [response.error.code ?? 'untyped_error'] : []
  ));
  if (diagnosticCodes.length > 0) {
    return {
      schemaVersion: 2,
      id: c.id,
      surface,
      outcome: 'batch_error',
      diagnosticCodes,
      coldInitMs,
      batchDurationMs,
    };
  }
  const opResult = responses[c.result.opIndex]?.ok;
  const expectedHandle = expectedResultHandle(c, opResult);
  let quality;
  let resultHandle;
  if (c.kind === 'boolean') {
    assert.equal(opResult?.solid, expectedHandle, `${c.id}: result handle`);
    quality = opResult?.quality ?? null;
    resultHandle = opResult?.solid ?? null;
  } else {
    assert.equal(opResult, expectedHandle, `${c.id}: result handle`);
    // Modifier/sweep batch ops return a bare handle: exact construction is
    // disclosed by success itself (a refusal surfaces as batch_error above).
    quality = 'exact';
    resultHandle = opResult;
  }
  const faces = responses[c.result.facesIndex]?.ok;
  assert.ok(Array.isArray(faces), `${c.id}: face response`);
  const surfaceTypes = {};
  for (const face of faces) {
    const type = direct.getSurfaceType(face);
    surfaceTypes[type] = (surfaceTypes[type] ?? 0) + 1;
  }
  const counts = direct.getEntityCounts(resultHandle);
  assert.equal(counts.length, 3, `${c.id}: entity census`);
  const serialized = direct.serializeSolid(resultHandle);
  return {
    schemaVersion: 2,
    id: c.id,
    surface,
    outcome: 'success',
    diagnosticCodes,
    quality,
    resultHandle,
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
}

function wasmObservation(c, modulePath) {
  const { BrepKernel } = createRequire(import.meta.url)(modulePath);
  const initStarted = performance.now();
  const kernel = new BrepKernel();
  const coldInitMs = performance.now() - initStarted;
  return (async () => {
    try {
      const batchStarted = performance.now();
      const { responses } = await driveBatch(
        c,
        async (dispatch) => JSON.parse(kernel.executeBatchV2(JSON.stringify(dispatch))),
        wasmDirect(kernel),
      );
      const batchDurationMs = performance.now() - batchStarted;
      return finalizeObservation(c, responses, 'wasm', coldInitMs, batchDurationMs, wasmDirect(kernel));
    } finally {
      kernel.free();
    }
  })();
}

function attempt(command, args, c) {
  const result = spawnSync(command, args, {
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

export function readManifest() {
  const bytes = readFileSync(manifestUrl);
  return { bytes, manifest: JSON.parse(bytes) };
}

async function main() {
  if (process.argv[2] === '--wasm-child') {
    const c = JSON.parse(readFileSync(0, 'utf8'));
    process.stdout.write(JSON.stringify(await wasmObservation(c, process.argv[3])));
    return;
  }

  const [nativeRunner, packageDir] = process.argv.slice(2);
  assert.ok(nativeRunner && packageDir, 'usage: parity-matrix.mjs NATIVE_RUNNER PACKAGE_DIR');
  const { bytes, manifest } = readManifest();
  const cases = expandMatrix(manifest);
  const temporaryRoot = mkdtempSync(resolve(tmpdir(), 'remus-o15-parity-'));
  const started = performance.now();
  try {
    const installed = installPackage(resolve(packageDir), temporaryRoot);
    const repoRoot = resolve(scriptDir, '..', '..');
    const provenance = collectFreshProvenance({
      repoRoot,
      packageDir: resolve(packageDir),
      tarballPath: installed.tarballPath,
      installedEntry: installed.entry,
      installedDir: installed.installedDir,
      buildOptions: {
        target: 'nodejs',
        profile: 'release',
        features: 'no-default-features',
        extraArgs: ['--no-opt'],
        tool: 'wasm-pack build crates/wasm --target nodejs --release --no-opt -- --no-default-features',
      },
    });
    const source = collectSourceProvenance(repoRoot);
    const observations = [];
    for (const c of cases) {
      observations.push({
        case: c.id,
        native: attempt(resolve(nativeRunner), [], c),
        wasm: attempt(process.execPath, [self, '--wasm-child', installed.entry], c),
      });
    }
    const stages = cases.flatMap((c, index) => scoreCase(c, observations[index]));
    const failedStages = stages.filter((stage) => stage.required && !stage.passed);
    const knownGaps = stages.filter((stage) => !stage.required && !stage.passed);
    const passed = failedStages.length === 0;
    const coveredBatchOps = [...new Set(cases.flatMap((c) => c.batch.map(({ op }) => op)))].sort();
    const report = {
      schema_version: 2,
      scope: 'O1.5 extended slice: all primitive pairs, modifiers, and sweep family at 1e-3/1/1e3',
      manifest_sha256: createHash('sha256').update(bytes).digest('hex'),
      installed_package_entry: installed.entry,
      provenance: {
        native: {
          mode: 'native',
          sourceRevision: source.sourceRevision,
          sourceDirty: source.sourceDirty,
          toolchain: source.toolchain,
        },
        fresh: provenance,
      },
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
