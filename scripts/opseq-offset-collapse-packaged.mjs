/**
 * P-Class 8.1 box offset collapse: an inward offset reaching a planar box's
 * half-thickness is the typed `offset_collapsed` refusal on every surface of
 * fresh/installed WASM, and changes nothing in the session.
 */
import assert from 'node:assert/strict';

const quarterTurn = [6.123233995736766e-17, 0, 1, 0, 0, 1, 0, -3, -1, 0, 6.123233995736766e-17, -2, 0, 0, 0, 1];
function oblique(scale) {
  const c = Math.cos(0.7), s = Math.sin(0.7), cx = Math.cos(0.3), sx = Math.sin(0.3);
  return [c, -s * cx, s * sx, 11 * scale, s, c * cx, -c * sx, -7 * scale,
    0, sx, cx, 5 * scale, 0, 0, 0, 1];
}
function state(k, ids) {
  const memory = JSON.parse(k.checkpointMemoryStats());
  return {
    document: k.serializeSolids(Uint32Array.from(ids)),
    counts: ids.map(id => Array.from(k.getEntityCounts(id))),
    journal: k.journalSummary(),
    liveSlots: (memory.allocatedSlots - memory.retiredSlots) / memory.uniqueTopologies,
  };
}
function near(actual, expected, label) {
  assert.ok(Math.abs(actual - expected) <= 1e-12 * Math.max(1, Math.abs(expected)), `${label}: ${actual} != ${expected}`);
}
function refusal(k, surface, operation, solid, distance, thickness, label) {
  if (surface === 'direct') {
    if (operation === 'offsetDetailed') {
      const outcome = k.offsetDetailed(solid, distance, false);
      assert.equal(outcome.status, 'error', label);
      assert.equal(outcome.code, 'offset_collapsed', label);
      assert.equal(outcome.category, 'invalid_input', label);
      near(outcome.details.thickness, thickness, label);
      assert.equal(outcome.details.distance, distance, label);
      assert.match(outcome.details.message, /collapsed the solid/, label);
      return outcome.details.message;
    }
    let error;
    assert.throws(() => k[operation](solid, distance), caught => {
      error = caught;
      return true;
    }, label);
    // Legacy direct exports throw message-only JsError objects.
    assert.match(error.message, /collapsed the solid/, label);
    return error.message;
  }
  const rows = JSON.parse(k[surface](JSON.stringify([{ op: operation, args: { solid, distance } }])));
  if (operation === 'offsetDetailed') {
    const outcome = rows[0].ok;
    assert.equal(outcome?.status, 'error', label);
    assert.equal(outcome.code, 'offset_collapsed', label);
    assert.equal(outcome.category, 'invalid_input', label);
    return outcome.details.message;
  }
  if (surface === 'executeBatch') {
    assert.equal(typeof rows[0].error, 'string', label);
    assert.match(rows[0].error, /collapsed the solid/, label);
    return rows[0].error;
  }
  const error = rows[0].error;
  assert.equal(error.code, 'invalid_argument', label);
  assert.equal(error.category, 'invalid_input', label);
  assert.equal(error.details.kernelCode, 'offset_collapsed', label);
  assert.equal(error.details.argument, 'distance', label);
  near(error.details.thickness, thickness, label);
  return error.message;
}

export function runOpseqOffsetCollapsePackaged({ RemusKernel }) {
  let passed = 0;
  const cells = [
    // The campaign literals: zero thickness after the quarter-turn placement.
    { dims: [2, 5.5, 1], scale: 1, place: quarterTurn, label: 'seed 10434068600042686585' },
    { dims: [5.5, 1, 5], scale: 1, place: quarterTurn, label: 'seed 3388039345656333454' },
  ];
  for (const scale of [0.01, 1, 1000]) {
    for (const placed of [false, true]) {
      cells.push({ dims: [1, 5, 1].map(s => s * scale), scale, place: placed ? oblique(scale) : null, label: `scale=${scale} placed=${placed}` });
    }
  }
  for (const cell of cells) {
    const thickness = Math.min(...cell.dims);
    for (const factor of [0.5, 0.75, 1e6]) {
      const k = new RemusKernel();
      const label = `${cell.label} distance=-${factor}t`;
      try {
        const source = k.makeBox(...cell.dims);
        if (cell.place) k.transformSolid(source, Float64Array.from(cell.place));
        const control = k.makeBox(3, 3, 3), history = JSON.parse(k.offsetJournaled(control, 0.1)).solid;
        const ids = [source, control, history];
        const checkpoint = k.checkpoint();
        const before = state(k, ids);
        assert.ok(JSON.parse(before.journal).length > 0, label);
        const distance = -factor * thickness;
        const messages = new Set();
        for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
          for (const operation of ['offsetSolid', 'offsetSolidV2', 'offsetDetailed']) {
            messages.add(refusal(k, surface, operation, source, distance, thickness, `${label}/${surface}/${operation}`));
            assert.deepEqual(state(k, ids), before, `${label}/${surface}/${operation}: refusal mutated session`);
            passed++;
          }
          // Journaled offsets refuse the same way and publish no history.
          assert.throws(() => {
            if (surface === 'direct') k.offsetJournaled(source, distance);
            else {
              const row = JSON.parse(k[surface](JSON.stringify([{ op: 'offsetJournaled', args: { solid: source, distance } }])))[0];
              if (row.error) throw new Error(typeof row.error === 'string' ? row.error : row.error.message);
            }
          }, /collapsed the solid/, `${label}/${surface}/offsetJournaled`);
          assert.deepEqual(state(k, ids), before, `${label}/${surface}/offsetJournaled: refusal mutated session`);
        }
        assert.equal(messages.size, 1, `${label}: surfaces disagree ${[...messages]}`);
        k.restore(checkpoint);
        assert.deepEqual(state(k, ids), before, `${label}: retained checkpoint changed`);
        // Just short of the half-thickness the offset is still a body.
        const d = -0.45 * thickness, kept = k.offsetSolidV2(source, d);
        const want = cell.dims.reduce((v, s) => v * (s + 2 * d), 1);
        const got = k.volume(kept, 1e-3 * thickness);
        assert.ok(Math.abs(got - want) <= 1e-9 * want, `${label}: control ${got} != ${want}`);
      } finally { k.free(); }
    }
  }
  console.log(`ok - box offset collapse refused as offset_collapsed with the session intact across ${passed} direct/batch cells`);
}
