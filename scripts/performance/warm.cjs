// Warm-cache workloads (RFC 0009 stage 0) through the kernel package. The
// native twins are in crates/wasm/examples/performance_baseline/warm.rs and run
// the same call sequences; keep the two in step.
//
// Every sample loads a fresh module instance, so its thread-local caches start
// empty, and records the cache counters its timed phases added. The package
// reports every cache through `cacheMemoryStats` from RFC 0009 stage 1 on;
// earlier packages expose only mesh-stage reuse, measurement reuse and the
// volume memo.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const { performance } = require('node:perf_hooks');

const DISPLAY_DEFLECTION = 0.0148;
const MEASURE_DEFLECTION = 0.08;
const MOVED_FACE_AREA = 1045.93;
const MOVE_DISTANCE = -6;
const RETICK_DISTANCE = -5;
const HOLDER_VOLUME = 50245.22;
const PLATE = [100, 100, 5];
const HOLE_RADIUS = 1.5;
const SLOTS = 6;

function freshKernel(modulePath, kernelClass) {
  delete require.cache[require.resolve(modulePath)];
  const RemusKernel = kernelClass(require(modulePath));
  return new RemusKernel();
}

function nearRel(actual, expected, rel) {
  assert(
    Number.isFinite(actual) && Math.abs(actual - expected) <= rel * Math.abs(expected),
    `${actual} != ${expected} (rel ${rel})`,
  );
}

function sameBits(a, b, what) {
  assert(Object.is(a, b), `${what}: ${a} !== ${b}`);
}

// Packages with `cacheMemoryStats` (RFC 0009 stage 1) report every cache; the
// counters are renamed to the native worker's keys. Older packages expose only
// mesh-stage reuse, measurement reuse and the volume memo.
const NOT_COUNTERS = new Set([
  'scope',
  'enabled',
  'entries',
  'maxEntries',
  'faces',
  'retainedBytes',
  'maxBytes',
]);
const snake = (name) => name.replace(/[A-Z]/g, (c) => `_${c.toLowerCase()}`);

function cacheRows(kernel) {
  if (typeof kernel.cacheMemoryStats !== 'function') return null;
  const { caches } = JSON.parse(kernel.cacheMemoryStats());
  // The library path the native worker drives has no classification cache.
  delete caches.classification;
  return caches;
}

// Counters only; entries and bytes are reported separately.
function counters(kernel) {
  const rows = cacheRows(kernel);
  if (rows) {
    return Object.fromEntries(
      Object.entries(rows).map(([name, row]) => [
        snake(name),
        Object.fromEntries(
          Object.entries(row ?? {})
            .filter(([key]) => !NOT_COUNTERS.has(key))
            .map(([key, value]) => [snake(key), value]),
        ),
      ]),
    );
  }
  const mesh = JSON.parse(kernel.meshReuseStats());
  const measure = JSON.parse(kernel.measurementReuseStats());
  const pick = (obj, keys) => Object.fromEntries(keys.map((k) => [k, obj ? obj[k] : 0]));
  return {
    mesh_stage: pick(mesh, [
      'boundaryHits',
      'edgeHits',
      'planarHits',
      'weldHits',
      'cleanupHits',
      'evictions',
    ]),
    measurement_reuse: pick(measure.reuse, [
      'areaIdentityHits',
      'areaContentHits',
      'areaMisses',
      'areaEvictions',
      'volumeIdentityHits',
      'volumeContentHits',
    ]),
    volume: pick(measure.volumeMemo, ['hits', 'contentHits', 'misses']),
  };
}

function entries(kernel) {
  const rows = cacheRows(kernel);
  if (rows) {
    return Object.fromEntries(
      Object.entries(rows).map(([name, row]) => [snake(name), row ? row.entries : 0]),
    );
  }
  const mesh = JSON.parse(kernel.meshReuseStats());
  const measure = JSON.parse(kernel.measurementReuseStats());
  return {
    mesh_stage: mesh ? mesh.entries : 0,
    measurement_reuse: measure.reuse.areaEntries,
    volume: measure.volumeMemo.entries,
  };
}

function delta(before, after) {
  if (typeof after === 'number') return after - before;
  return Object.fromEntries(Object.entries(after).map(([k, v]) => [k, delta(before[k], v)]));
}

class Phases {
  constructor() {
    this.list = [];
  }
  time(name, f) {
    const start = performance.now();
    const out = f();
    this.list.push([name, (performance.now() - start) * 1e6]);
    return out;
  }
  finish(kernel, start, metrics) {
    const phases = {};
    for (const [name, ns] of this.list) phases[name] = (phases[name] ?? 0) + ns / 1e6;
    const total = this.list.reduce((sum, [, ns]) => sum + ns, 0);
    return [
      total,
      {
        ...metrics,
        phases_ms: phases,
        cache_stats: delta(start, counters(kernel)),
        cache_entries: entries(kernel),
      },
    ];
  }
}

function load(kernel, arena) {
  const solids = Array.from(kernel.deserializeSolids(arena));
  assert.equal(solids.length, 1);
  return solids[0];
}

function valid(kernel, solid) {
  assert.equal(kernel.validateSolid(solid), 0, 'strict validation errors');
}

function areas(kernel, faces) {
  return faces.reduce((sum, f) => sum + kernel.faceArea(f, MEASURE_DEFLECTION), 0);
}

function meshTriangles(kernel, solid) {
  const mesh = kernel.tessellateSolidGroupedBinary(solid, DISPLAY_DEFLECTION);
  try {
    assert(mesh.faceOffsets.length > 0);
    return mesh.indices.length / 3;
  } finally {
    mesh.free();
  }
}

// The binding returns the rows as a JSON string; parsing is part of the
// consumer's cost, so it stays inside the phase.
function relationCount(kernel, solid) {
  const rows = JSON.parse(kernel.solidEdgeRelations(solid));
  assert(Array.isArray(rows));
  return rows.length;
}

function movedFace(kernel, solid) {
  let best;
  let gap = Infinity;
  for (const f of kernel.getSolidFaces(solid)) {
    if (kernel.getSurfaceType(f).toLowerCase() !== 'plane' || kernel.getFaceNormal(f)[0] <= 0.99)
      continue;
    const g = Math.abs(kernel.faceArea(f, 0.05) - MOVED_FACE_AREA);
    if (g < gap) [gap, best] = [g, f];
  }
  assert(gap < 1, `+X face area off by ${gap}`);
  return best;
}

function translation(x, y, z) {
  return Float64Array.from([1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1]);
}

function placed(kernel, solid, [x, y, z]) {
  kernel.transformSolid(solid, translation(x, y, z));
  return solid;
}

function measure(kernel, arena, ctx) {
  const solid = load(kernel, arena);
  const faces = Array.from(kernel.getSolidFaces(solid));
  const start = counters(kernel);
  const p = new Phases();
  p.time('validate_cold', () => valid(kernel, solid));
  p.time('validate_warm', () => valid(kernel, solid));
  p.time('validate_copy', () => {
    // A short-lived probe kernel in the same module, as the consumer's import does.
    const probe = new ctx.RemusKernel();
    try {
      valid(probe, load(probe, arena));
    } finally {
      probe.free();
    }
  });
  const cold = p.time('areas_cold', () => areas(kernel, faces));
  const warm = p.time('areas_warm', () => areas(kernel, faces));
  const vCold = p.time('volume_cold', () => kernel.volume(solid, MEASURE_DEFLECTION));
  const vWarm = p.time('volume_warm', () => kernel.volume(solid, MEASURE_DEFLECTION));
  sameBits(cold, warm, 'warm face areas');
  sameBits(vCold, vWarm, 'warm volume');
  nearRel(vCold, HOLDER_VOLUME, 1e-6);
  return p.finish(kernel, start, { faces: faces.length, area_sum: cold, volume: vCold });
}

function edit(kernel, arena) {
  const solid = load(kernel, arena);
  const face = movedFace(kernel, solid);
  const start = counters(kernel);
  const p = new Phases();
  const tris = p.time('mesh_cold', () => meshTriangles(kernel, solid));
  const trisWarm = p.time('mesh_warm', () => meshTriangles(kernel, solid));
  assert.equal(tris, trisWarm);
  const relations = p.time('relations_source', () => relationCount(kernel, solid));
  const sourceFeatures = p.time(
    'recognize_source',
    () => JSON.parse(kernel.recognizeFeatures(solid, MEASURE_DEFLECTION)).length,
  );
  const moved = p.time('move', () => kernel.moveFaces(solid, Uint32Array.of(face), MOVE_DISTANCE));
  // A later drag tick moves the same source face to another distance.
  const retick = p.time('move_retick', () =>
    kernel.moveFaces(solid, Uint32Array.of(face), RETICK_DISTANCE),
  );
  const movedTris = p.time('mesh_moved', () => meshTriangles(kernel, moved));
  const movedRelations = p.time('relations_moved', () => relationCount(kernel, moved));
  const features = p.time(
    'recognize_moved',
    () => JSON.parse(kernel.recognizeFeatures(moved, MEASURE_DEFLECTION)).length,
  );
  p.time('validate_moved', () => valid(kernel, moved));
  const volume = kernel.volume(moved, MEASURE_DEFLECTION);
  const retickVolume = kernel.volume(retick, MEASURE_DEFLECTION);
  assert(
    volume < retickVolume && retickVolume < HOLDER_VOLUME,
    'moving +X inward must remove material',
  );
  return p.finish(kernel, start, {
    triangles: tris,
    moved_triangles: movedTris,
    relations,
    features: sourceFeatures,
    moved_relations: movedRelations,
    moved_features: features,
    moved_volume: volume,
    retick_volume: retickVolume,
  });
}

function cuts(kernel, arena) {
  const solid = load(kernel, arena);
  const start = counters(kernel);
  const p = new Phases();
  const slot = () => placed(kernel, kernel.makeBox(4, 12, 20), [10, 44, 0]);
  const hole = () => placed(kernel, kernel.makeCylinder(2, 30), [20, 50, -5]);
  const a = p.time('cut_slot', () => kernel.cut(solid, slot()));
  const b = p.time('cut_hole', () => kernel.cut(solid, hole()));
  const c = p.time('cut_slot_repeat', () => kernel.cut(solid, slot()));
  const volumes = [a, b, c].map((s) => kernel.volume(s, MEASURE_DEFLECTION));
  sameBits(volumes[0], volumes[2], 'repeat cut volume');
  assert(volumes.every((v) => v < HOLDER_VOLUME));
  return p.finish(kernel, start, { volumes });
}

function plate(kernel, grid) {
  const start = counters(kernel);
  const p = new Phases();
  const pitch = PLATE[0] / (grid + 1);
  let body = p.time('drill', () => {
    let body = kernel.makeBox(...PLATE);
    for (let i = 1; i <= grid; i++) {
      for (let j = 1; j <= grid; j++) {
        const c = placed(kernel, kernel.makeCylinder(HOLE_RADIUS, PLATE[2] + 2), [
          i * pitch,
          j * pitch,
          -1,
        ]);
        body = kernel.cut(body, c);
      }
    }
    return body;
  });
  for (let k = 0; k < SLOTS; k++) {
    body = p.time(k === 0 ? 'slot_first' : 'slots_rest', () => {
      const s = placed(kernel, kernel.makeBox(1, 0.8 * PLATE[1], PLATE[2] + 2), [
        (k + 1.5) * pitch - 0.5,
        0.1 * PLATE[1],
        -1,
      ]);
      return kernel.cut(body, s);
    });
  }
  const volume = kernel.volume(body, MEASURE_DEFLECTION);
  const expected =
    PLATE[0] * PLATE[1] * PLATE[2] -
    grid * grid * Math.PI * HOLE_RADIUS * HOLE_RADIUS * PLATE[2] -
    SLOTS * 0.8 * PLATE[1] * PLATE[2];
  nearRel(volume, expected, 1e-9);
  return p.finish(kernel, start, { holes: grid * grid, slots: SLOTS, volume });
}

function run({ modulePath, kernelClass, scenario, size, samples, warmup }) {
  let arena;
  if (scenario === 'wasm_warm_plate') {
    assert(size >= 2 && size <= 32);
  } else {
    assert.equal(size, 160, 'holder workloads are sized by its face count');
    const arenaPath = process.env.REMUS_PERF_HAMMER_ARENA;
    assert(arenaPath, 'REMUS_PERF_HAMMER_ARENA names the hammer-holder arena document');
    arena = fs.readFileSync(arenaPath);
  }
  for (let sample = 0; sample < samples + warmup; sample++) {
    const kernel = freshKernel(modulePath, kernelClass);
    const ctx = { RemusKernel: kernel.constructor };
    try {
      const [ns, metrics] =
        scenario === 'wasm_warm_measure'
          ? measure(kernel, arena, ctx)
          : scenario === 'wasm_warm_edit'
            ? edit(kernel, arena)
            : scenario === 'wasm_warm_cuts'
              ? cuts(kernel, arena)
              : plate(kernel, size);
      console.log(
        JSON.stringify({
          schema: 'remus-performance-sample-v1',
          scenario,
          size,
          sample,
          warmup: sample < warmup,
          operation_ns: ns,
          validation: 'passed',
          metrics,
        }),
      );
    } finally {
      kernel.free();
    }
  }
}

module.exports = { run };
