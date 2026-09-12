const { performance } = require('node:perf_hooks');
const fs = require('node:fs');
const crypto = require('node:crypto');
const root = require('node:path').resolve(
  process.argv[2] || require('node:path').resolve(__dirname, '../../../../crates/wasm/pkg'),
);
const t0 = performance.now();
const { BrepKernel } = require(root + '/remus_wasm_node.cjs');
console.log(
  JSON.stringify({
    node: process.version,
    package: JSON.parse(fs.readFileSync(root + '/package.json')).version,
    coldRequireMs: performance.now() - t0,
    wasmSha256: crypto
      .createHash('sha256')
      .update(fs.readFileSync(root + '/remus_wasm_bg.wasm'))
      .digest('hex'),
    wasmBytes: fs.statSync(root + '/remus_wasm_bg.wasm').size,
  }),
);
function exec(k, ops) {
  const out = JSON.parse(k.executeBatch(JSON.stringify(ops)));
  if (out.some((x) => x.error !== undefined)) throw Error(JSON.stringify(out));
  return out;
}
function build(n) {
  return Array.from({ length: n }, (_, i) => ({
    op: 'makeBox',
    args: { width: 1 + (i % 5) * 0.1, height: 1 + (i % 5) * 0.1, depth: 1 + (i % 5) * 0.1 },
  }));
}
const identity = [1, 0, 0, 0.001, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
for (const seed of [50, 200, 400]) {
  for (const name of ['query_batch', 'build_batch', 'assembly_batch', 'transform_direct']) {
    const samples = [];
    for (let rep = 0; rep < 10; rep++) {
      const k = new BrepKernel();
      exec(k, build(seed));
      const query = Array.from({ length: 150 }, (_, i) => ({
        op: i % 2 ? 'volume' : 'boundingBox',
        args: { solid: 0, deflection: 0.5 },
      }));
      const assembly = Array.from({ length: 50 }, (_, i) => [
        { op: 'copySolid', args: { solid: 0 } },
        {
          op: 'transform',
          args: { solid: 0, matrix: [1, 0, 0, i * 0.01, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1] },
        },
        { op: 'boundingBox', args: { solid: 0 } },
      ]).flat();
      const ops = name === 'query_batch' ? query : name === 'build_batch' ? build(150) : assembly;
      const input = JSON.stringify(ops);
      const start = performance.now();
      let out;
      if (name === 'transform_direct') {
        for (let i = 0; i < 150; i++) k.transformSolid(0, Float64Array.from(identity));
      } else out = k.executeBatch(input);
      const ms = performance.now() - start;
      if (out && JSON.parse(out).some((x) => x.error !== undefined)) throw Error(out);
      if (!Number.isFinite(k.volume(0, 0.5))) throw Error('nonfinite volume');
      if (rep > 0) samples.push(ms);
      k.free();
    }
    samples.sort((a, b) => a - b);
    console.log(
      JSON.stringify({
        name,
        seed,
        ops: 150,
        reps: 9,
        minMs: samples[0],
        medianMs: samples[4],
        maxMs: samples[8],
      }),
    );
  }
}
