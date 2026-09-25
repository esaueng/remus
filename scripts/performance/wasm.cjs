const assert = require('node:assert/strict');
const { performance } = require('node:perf_hooks');
const path = require('node:path');
const [pkg, scenario, sizeArg, samplesArg, warmupArg] = process.argv.slice(2);
const size = Number(sizeArg),
  samples = Number(samplesArg),
  warmup = Number(warmupArg);
assert(
  ['wasm_transform_direct', 'wasm_transform_batch', 'wasm_bool_fuse', 'wasm_bool_refused'].includes(
    scenario,
  ),
);
assert(Number.isInteger(size) && size >= 2 && size <= 8192);
assert(Number.isInteger(samples) && samples >= 1 && samples <= 1000);
assert(Number.isInteger(warmup) && warmup >= 1 && warmup <= 100);
const { BrepKernel } = require(path.join(path.resolve(pkg), 'remus_wasm_node.cjs'));
const calls = 150;
const matrix = [1, 0, 0, 0.001, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
function checked(output, count) {
  const rows = JSON.parse(output);
  assert.equal(rows.length, count);
  assert(
    rows.every((r) => Object.hasOwn(r, 'ok') && !Object.hasOwn(r, 'error')),
    output,
  );
  return rows;
}
function near(a, b) {
  assert(Number.isFinite(a) && Math.abs(a - b) <= 1e-8, `${a} != ${b}`);
}
function nowNs() {
  return performance.now() * 1e6;
}
// Fixed local boolean edit amid `size` unrelated solids. Seeding stays
// outside the timed edit; snapshot (checkpoint), kernel (fuse or refused
// self-cut), validation (volume/bounds/rollback), serialization (arena
// document bytes) and teardown (kernel free) are timed separately.
function booleanDoc(kernel) {
  const setupStart = nowNs();
  const seeds = Array.from({ length: size + 2 }, () => ({
    op: 'makeBox',
    args: { width: 1, height: 1, depth: 1 },
  }));
  const handles = checked(kernel.executeBatch(JSON.stringify(seeds)), size + 2);
  const ids = handles.map((row) => row.ok);
  assert(ids.every((id) => Number.isInteger(id) && id >= 0 && id <= 0xffffffff));
  assert.equal(new Set(ids).size, size + 2, 'seeding must create distinct solids');
  for (let k = 0; k < size; k++) {
    const x = 1000 + k * 10;
    kernel.transformSolid(ids[k], Float64Array.from([1, 0, 0, x, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]));
  }
  const [a, b] = [ids[size], ids[size + 1]];
  kernel.transformSolid(b, Float64Array.from([1, 0, 0, 0.5, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]));
  const untouched = ids[size - 1];
  const ux = 1000 + (size - 1) * 10;
  const setupNs = nowNs() - setupStart;
  return { ids, a, b, untouched, untouchedBox: [ux, 0, 0, ux + 1, 1, 1], setupNs };
}
function checkBoxed(kernel, solid, expected) {
  const got = Array.from(kernel.boundingBox(solid));
  assert.equal(got.length, 6);
  got.forEach((x, i) => near(x, expected[i]));
}
function booleanSample(kernel, refused) {
  const { ids, a, b, untouched, untouchedBox, setupNs } = booleanDoc(kernel);
  let start = nowNs();
  kernel.checkpoint();
  const snapshotNs = nowNs() - start;
  start = nowNs();
  if (!refused) {
    const r = kernel.fuse(a, b);
    ids.push(r);
  } else {
    assert.throws(() => kernel.cut(a, a), /identical/i);
  }
  const kernelNs = nowNs() - start;
  start = nowNs();
  if (!refused) {
    const r = ids[ids.length - 1];
    near(kernel.volume(r, 0.01), 1.5);
    checkBoxed(kernel, r, [0, 0, 0, 1.5, 1, 1]);
  } else {
    checkBoxed(kernel, a, [0, 0, 0, 1, 1, 1]);
    checkBoxed(kernel, b, [0.5, 0, 0, 1.5, 1, 1]);
  }
  checkBoxed(kernel, untouched, untouchedBox);
  near(kernel.volume(untouched, 0.01), 1);
  const validationNs = nowNs() - start;
  start = nowNs();
  const bytes = kernel.serializeSolids(Uint32Array.from(ids));
  const serNs = nowNs() - start;
  return {
    phasesMs: {
      setup: setupNs / 1e6,
      snapshot: snapshotNs / 1e6,
      kernel: kernelNs / 1e6,
      validation: validationNs / 1e6,
      serialization: serNs / 1e6,
    },
    solids: ids.length,
    serializationBytes: bytes.length,
  };
}
if (scenario === 'wasm_bool_fuse' || scenario === 'wasm_bool_refused') {
  const refused = scenario === 'wasm_bool_refused';
  for (let sample = 0; sample < samples + warmup; sample++) {
    const kernel = new BrepKernel();
    let freed = false;
    try {
      const metrics = booleanSample(kernel, refused);
      const freeStart = nowNs();
      kernel.free();
      freed = true;
      const teardownNs = nowNs() - freeStart;
      metrics.phasesMs.teardown = teardownNs / 1e6;
      const ns =
        (metrics.phasesMs.snapshot +
          metrics.phasesMs.kernel +
          metrics.phasesMs.validation +
          metrics.phasesMs.serialization) *
          1e6 +
        teardownNs;
      console.log(
        JSON.stringify({
          schema: 'remus-performance-sample-v1',
          scenario,
          size,
          sample,
          warmup: sample < warmup,
          operation_ns: ns,
          validation: 'passed',
          metrics: {
            phases_ms: metrics.phasesMs,
            ...(refused
              ? { error_code: 'operation_failed', error_category: 'internal', rollback_checked: true }
              : { volume: 1.5 }),
            untouched_solid_checked: true,
            solids: metrics.solids,
            serialization_bytes: metrics.serializationBytes,
          },
        }),
      );
    } finally {
      if (!freed) kernel.free();
    }
  }
  process.exit(0);
}
for (let sample = 0; sample < samples + warmup; sample++) {
  const kernel = new BrepKernel();
  try {
    const seeds = Array.from({ length: size }, () => ({
      op: 'makeBox',
      args: { width: 1, height: 1, depth: 1 },
    }));
    const handles = checked(kernel.executeBatch(JSON.stringify(seeds)), size);
    const ids = handles.map((row) => row.ok);
    assert(ids.every((id) => Number.isInteger(id) && id >= 0 && id <= 0xffffffff));
    assert.equal(new Set(ids).size, size, 'seeding must create distinct solids');
    const base = handles[0].ok,
      untouched = handles.at(-1).ok;
    const input = JSON.stringify(
      Array.from({ length: calls }, () => ({ op: 'transform', args: { solid: base, matrix } })),
    );
    let output;
    const start = performance.now();
    if (scenario === 'wasm_transform_direct') {
      for (let i = 0; i < calls; i++) kernel.transformSolid(base, Float64Array.from(matrix));
    } else output = kernel.executeBatch(input);
    const ns = (performance.now() - start) * 1e6;
    if (output !== undefined) checked(output, calls);
    const moved = kernel.boundingBox(base),
      stationary = kernel.boundingBox(untouched);
    [0.15, 0, 0, 1.15, 1, 1].forEach((x, i) => near(moved[i], x));
    [0, 0, 0, 1, 1, 1].forEach((x, i) => near(stationary[i], x));
    near(kernel.volume(base, 0.5), 1);
    console.log(
      JSON.stringify({
        schema: 'remus-performance-sample-v1',
        scenario,
        size,
        sample,
        warmup: sample < warmup,
        operation_ns: ns,
        validation: 'passed',
        metrics: { calls, volume: 1, translation_x: 0.15, untouched_solid_checked: true },
      }),
    );
  } finally {
    kernel.free();
  }
}
