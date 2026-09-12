const assert = require('node:assert/strict');
const { performance } = require('node:perf_hooks');
const path = require('node:path');
const [pkg, scenario, sizeArg, samplesArg, warmupArg] = process.argv.slice(2);
const size = Number(sizeArg),
  samples = Number(samplesArg),
  warmup = Number(warmupArg);
assert(['wasm_transform_direct', 'wasm_transform_batch'].includes(scenario));
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
