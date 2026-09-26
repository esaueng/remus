const assert = require('node:assert/strict');
const path = require('node:path');
const wasm = require(path.resolve(process.argv[2]));
let cases = 0;
for (const stage of [1, 2]) for (const batch of [false, true]) for (const gfa of [false, true]) {
  const k = new wasm.BrepKernel();
  const a = k.makeBox(1,1,1), b = k.makeBox(1,1,1);
  const c = Math.SQRT1_2;
  k.transformSolid(b, new Float64Array(gfa ? [c,-c,0,0.5,c,c,0,0.5-c,0,0,1,0,0,0,0,1] : [1,0,0,0.5,0,1,0,0,0,0,1,0,0,0,0,1]));
  const face = k.getSolidFaces(a)[0];
  k.setFaceName(face, 'retained attribute');
  k.journalBarrier('seed', a);
  k.assemblyNew('retained assembly');
  const sketch = k.sketchNew();
  k.sketchAddPoint(sketch, 1, 2, false);
  const gcs = k.gcsNew();
  k.gcsAddPoint(gcs, 3, 4, false);
  const checkpoint = k.checkpoint();
  const before = k.transactionProbeState(), session = k.transactionProbeSession();
  const failed = [];
  for (let i = 0; i < 3; i++) {
    wasm.transactionProbeFault(stage);
    if (batch) {
      const rows = JSON.parse(k.executeBatch(JSON.stringify([{op:'fuse',args:{solidA:a,solidB:b}}])));
      assert.match(rows[0].error, /PERF-T01 injected/);
    } else assert.throws(() => k.fuse(a,b), /PERF-T01 injected/);
    const id = wasm.transactionProbeFailed();
    assert(id >= 0, 'fault must occur after result allocation');
    failed.push(id);
    assert.equal(k.transactionProbeState(), before);
    assert.equal(k.transactionProbeSession(), session);
    assert.throws(() => k.volume(id, 0.01));
  }
  // One-shot injection lets later items commit in the very same batch.
  wasm.transactionProbeFault(stage);
  const rows = JSON.parse(k.executeBatch(JSON.stringify([
    {op:'makeBox',args:{width:2,height:2,depth:2}},
    {op:'fuse',args:{solidA:a,solidB:b}},
    {op:'fuse',args:{solidA:a,solidB:b}},
  ])));
  assert(Number.isInteger(rows[0].ok));
  assert.match(rows[1].error, /PERF-T01 injected/);
  assert(Number.isInteger(rows[2].ok));
  failed.push(wasm.transactionProbeFailed());
  assert(Math.abs(k.volume(rows[0].ok,0.01) - 8) < 1e-8);
  assert(Math.abs(k.volume(rows[2].ok,0.01) - (gfa ? 4-2*Math.SQRT2 : 1.5)) < 1e-8);
  k.restore(checkpoint);
  assert.equal(k.transactionProbeState(), before);
  assert.equal(k.transactionProbeSession(), session);
  for (let i=0; i<4; i++) k.makeBox(1,1,1);
  for (const id of failed) assert.throws(() => k.volume(id,0.01));
  k.restore(checkpoint);
  assert.equal(k.transactionProbeState(), before);
  k.free();
  cases++;
}
console.log(JSON.stringify({faultMatrixPassed:cases, repeatedFailuresPerCase:3, partialBatch:true, checkpointAndStaleHandles:true}));
