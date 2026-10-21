/** F7: honest assembly errors and atomic refusal in fresh/installed WASM. */
import assert from 'node:assert/strict';

const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
function placement(scale) {
  const c = Math.cos(0.7), s = Math.sin(0.7), cx = Math.cos(0.3), sx = Math.sin(0.3);
  return [c, -s * cx, s * sx, 11 * scale, s, c * cx, -c * sx, -7 * scale,
    0, sx, cx, 5 * scale, 0, 0, 0, 1];
}
function state(k, ids, assembly, sketch, gcs, point) {
  const memory = JSON.parse(k.checkpointMemoryStats());
  return {
    document: k.serializeSolids(Uint32Array.from(ids)),
    counts: ids.map(id => Array.from(k.getEntityCounts(id))),
    journal: k.journalSummary(),
    checkpoints: k.checkpointCount(),
    assembly: k.assemblyFlatten(assembly),
    sketch: k.sketchDof(sketch),
    gcs: Array.from(k.gcsPointPosition(gcs, point)),
    // The retained checkpoint initially shares the current topology. A
    // failed mutation may separate the two and retire temporary handles;
    // each topology must still contain exactly the same live slot count.
    liveSlots: (memory.allocatedSlots - memory.retiredSlots) / memory.uniqueTopologies,
  };
}
function refusal(k, surface, operation, solid, distance, label, invalid = false) {
  let error;
  if (surface === 'direct') {
    if (operation === 'offsetDetailed') {
      const outcome = k.offsetDetailed(solid, distance, false);
      assert.equal(outcome.status, 'error', label);
      error = { code: outcome.code, category: outcome.category, message: outcome.details.message };
    } else {
      assert.throws(() => k[operation](solid, distance), caught => {
        error = caught;
        return true;
      }, label);
      // Legacy direct exports throw message-only JsError objects. Keep
      // that public shape; typed categories are provided by Detailed/v2.
      assert.match(error.message, invalid ? /distance/ : /assembly failed: wire loop walk/, label);
      assert.equal(error.message.includes('invalid input:'), invalid, label);
      return;
    }
  } else {
    const rows = JSON.parse(k[surface](JSON.stringify([{ op: operation, args: { solid, distance } }])));
    if (operation === 'offsetDetailed') {
      assert.equal(rows[0].ok?.status, 'error', label);
      error = { ...rows[0].ok, message: rows[0].ok.details.message };
    } else if (surface === 'executeBatch') {
      assert.equal(typeof rows[0].error, 'string', label);
      assert.match(rows[0].error, invalid ? /invalid input/ : /assembly failed: wire loop walk/, label);
      if (!invalid) assert.ok(!rows[0].error.startsWith('invalid input'), label);
      return;
    } else {
      error = rows[0].error;
      assert.ok(error, label);
    }
  }
  assert.equal(error.code, invalid ? 'invalid_argument' : 'operation_failed', label);
  assert.equal(error.category, invalid ? 'invalid_input' : 'internal', label);
  assert.match(error.message, invalid ? /distance/ : /assembly failed: wire loop walk/, label);
}

export function runOpseqOffsetRefusalPackaged({ BrepKernel }) {
  let passed = 0;
  for (const width of [0.9, 1, 1.1]) {
    for (const distance of [-0.2, -0.25, -0.3]) {
      for (const scale of [1, 1000]) {
        for (const placed of [false, true]) {
          const k = new BrepKernel();
          const label = `F7 width=${width} distance=${distance} scale=${scale} placed=${placed}`;
          try {
            // Preserve the original literal's dense cylinder/box/fuse prefix.
            const cylinder = k.makeCylinder(scale, scale), box = k.makeBox(width * scale, scale, scale);
            const union = k.booleanWithQuality('fuse', box, cylinder, true);
            assert.equal(union.quality, 'exact', label);
            const source = union.solid;
            if (placed) k.transformSolid(source, Float64Array.from(placement(scale)));
            const x = Math.min(width, 1), overlap = (x * Math.sqrt(1 - x * x) + Math.asin(x)) / 2;
            const expected = Math.PI + width - overlap;
            assert.ok(Math.abs(k.volume(source, 0.0001 * scale) / scale ** 3 - expected) < 1e-6 * expected, label);
            const control = k.makeBox(3, 3, 3), history = JSON.parse(k.offsetJournaled(control, 0.1)).solid;
            const ids = [cylinder, box, source, control, history];
            const assembly = k.assemblyNew('retained');
            k.assemblyAddRoot(assembly, 'source', source, Float64Array.from(identity));
            const sketch = k.sketchNew();
            k.sketchAddPoint(sketch, 1, 2, false);
            const gcs = k.gcsNew(), point = k.gcsAddPoint(gcs, 3, 4, false);
            const checkpoint = k.checkpoint();
            const before = state(k, ids, assembly, sketch, gcs, point);
            assert.ok(JSON.parse(before.journal).length > 0, label);
            for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
              for (const operation of ['offsetSolid', 'offsetSolidV2', 'offsetDetailed', 'offsetJournaled']) {
                refusal(k, surface, operation, source, distance * scale, `${label}/${surface}/${operation}`);
                assert.deepEqual(state(k, ids, assembly, sketch, gcs, point), before, `${label}: refusal mutated session`);
                // A genuinely invalid caller argument must retain its category.
                refusal(k, surface, operation, source, 0, `${label}/${surface}/${operation}/zero`, true);
                assert.deepEqual(state(k, ids, assembly, sketch, gcs, point), before, `${label}: invalid-input mutation`);
                passed++;
              }
            }
            k.restore(checkpoint);
            assert.deepEqual(state(k, ids, assembly, sketch, gcs, point), before, `${label}: retained checkpoint changed`);
            const next = k.makeBox(2, 2, 2);
            assert.ok(!ids.includes(next), `${label}: new solid reused a live handle`);
            assert.equal(k.volume(next, 0.01), 8, `${label}: refusal poisoned kernel`);
          } finally { k.free(); }
        }
      }
    }
  }
  console.log(`ok - F7 typed offset assembly refusals and retained session across ${passed} neighboring direct/batch cells`);
}
