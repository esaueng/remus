const assert = require('node:assert/strict');
const path = require('node:path');
const { performance } = require('node:perf_hooks');
const [pkg, sizeArg, mode = 'batch', checkpoint = 'false'] = process.argv.slice(2);
const size = Number(sizeArg);
const wasm = require(path.resolve(pkg));
const k = new wasm.BrepKernel();
for (let i = 0; i < size; i++) k.makeBox(1, 1, 1);
const a = k.makeBox(1, 1, 1), b = k.makeBox(1, 1, 1);
const c = Math.SQRT1_2;
const gfa = mode.startsWith('gfa-');
k.transformSolid(b, new Float64Array(gfa ? [c,-c,0,0.5, c,c,0,0.5-c, 0,0,1,0, 0,0,0,1] : [1,0,0,0.5, 0,1,0,0, 0,0,1,0, 0,0,0,1]));
if (checkpoint === 'true') k.checkpoint();
wasm.transactionProbeReset?.();
const start = performance.now();
let result;
if (mode.endsWith('direct')) result = k.fuse(a, b);
else {
  const rows = JSON.parse(k.executeBatch(JSON.stringify([{op:'fuse',args:{solidA:a,solidB:b}}])));
  assert.equal(rows.length, 1);
  assert.equal(rows[0].error, undefined, JSON.stringify(rows));
  result = rows[0].ok;
}
const ms = performance.now() - start;
const metrics = wasm.transactionProbeRead ? Array.from(wasm.transactionProbeRead()) : null;
assert(Math.abs(k.volume(result, 0.01) - (gfa ? 4 - 2*Math.SQRT2 : 1.5)) < 1e-8);
console.log(JSON.stringify({size, mode, checkpoint, ms, metrics, peakRssBytes:process.resourceUsage().maxRSS*1024, linearMemoryBytes:wasm.__transactionMemory?.buffer.byteLength ?? null}));
k.free();
