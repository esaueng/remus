/** Public wasm32 regressions for JSON references and variable-fillet laws. */
import assert from 'node:assert/strict';

function topologyState(kernel, solid) {
  const memory = JSON.parse(kernel.checkpointMemoryStats());
  return {
    arena: kernel.serializeSolids(Uint32Array.of(solid)),
    journal: kernel.journalSummary(),
    allocated: memory.allocatedSlots,
    retired: memory.retiredSlots,
  };
}

export function runWasmParserReferencesPackaged({ BrepKernel }) {
  for (const surface of ['direct', 'directDetailed', 'executeBatch', 'executeBatchV2']) {
    const kernel = new BrepKernel();
    const solid = kernel.makeBox(10, 10, 10);
    const edge = kernel.getSolidEdges(solid)[0];
    const specs = [
      ...[2 ** 32, 2 ** 32 + edge + 1, -1, 0.5, null, '0'].map((invalid) => ({ edge: invalid, law: 'constant', radius: 1 })),
      ...['linear_typo', '', null, 7, {}, []].map((law) => ({ edge, law, radius: 1 })),
    ];
    for (const spec of specs) {
      const before = topologyState(kernel, solid);
      const label = `${surface}: ${JSON.stringify(spec)}`;
      if (surface === 'direct') {
        assert.throws(() => kernel.filletVariable(solid, JSON.stringify([spec])), /invalid input/, label);
      } else if (surface === 'directDetailed') {
        const result = kernel.filletVariableDetailed(solid, JSON.stringify([spec]), false);
        assert.equal(result.status, 'error', label);
        assert.equal(result.category, 'invalid_input', label);
      } else {
        for (const op of ['filletVariable', 'filletVariableDetailed']) {
          const row = JSON.parse(kernel[surface](JSON.stringify([{ op, args: { solid, specs: [spec], exactOnly: false } }])))[0];
          if (op.endsWith('Detailed')) {
            assert.equal(row.ok?.status, 'error', label);
            assert.equal(row.ok.category, 'invalid_input', label);
          } else {
            assert.ok(row.error, label);
            if (surface === 'executeBatchV2') assert.equal(row.error.code, 'invalid_argument', label);
          }
        }
      }
      assert.deepEqual(topologyState(kernel, solid), before, label);
    }
    kernel.free();
  }
  // The omitted-law default must still execute the same constant fillet.
  for (const surface of ['filletVariable', 'filletVariableDetailed']) {
    let expected;
    for (const law of ['constant', undefined]) {
      const kernel = new BrepKernel();
      const solid = kernel.makeBox(10, 10, 10);
      const edge = kernel.getSolidEdges(solid)[0];
      const json = JSON.stringify([{ edge, law, radius: 1 }]);
      const result = kernel[surface](solid, json, false);
      if (surface.endsWith('Detailed')) assert.equal(result.status, 'ok', surface);
      const output = surface.endsWith('Detailed') ? result.value : result;
      const bytes = kernel.serializeSolids(Uint32Array.of(output));
      if (expected === undefined) expected = bytes;
      else assert.deepEqual(bytes, expected, surface);
      kernel.free();
    }
  }
  for (const point of [2 ** 32, 2 ** 32 + 1, -1, 0.5, null, '0']) {
    const kernel = new BrepKernel();
    const sketch = kernel.sketchNew();
    kernel.sketchAddPoint(sketch, 1, 2, false);
    kernel.sketchAddPoint(sketch, 3, 4, false);
    const dof = kernel.sketchDof(sketch);
    assert.throws(() => kernel.sketchAddConstraint(sketch, JSON.stringify({ type: 'fixX', point, value: 42 })), /invalid input/, String(point));
    assert.equal(kernel.sketchDof(sketch), dof);
    assert.deepEqual(JSON.parse(kernel.sketchSolve(sketch, 20, 1e-8)).points, [[1, 2], [3, 4]]);
    kernel.free();
  }
  // Deferred arc constraints are index-checked when added (kernel review
  // WASM-8): an unrepresentable reference is refused before storage, and the
  // sketch and session stay unchanged and solvable.
  for (const field of ['line0', 'line1', 'arc', 'point']) {
    const kernel = new BrepKernel();
    const solid = kernel.makeBox(1, 1, 1);
    const sketch = kernel.sketchNew();
    const points = [[0, 0], [1, 0], [0, 1], [1, 1]];
    for (const [x, y] of points) kernel.sketchAddPoint(sketch, x, y, true);
    kernel.sketchAddArc(sketch, 0, 1, 2);
    const spec = { type: 'tangentLineArc', line: [1, 3], arc: 0, point: 1 };
    if (field === 'line0') spec.line[0] += 2 ** 32;
    else if (field === 'line1') spec.line[1] += 2 ** 32;
    else spec[field] += 2 ** 32;
    const before = topologyState(kernel, solid);
    const dof = kernel.sketchDof(sketch);
    assert.throws(() => kernel.sketchAddConstraint(sketch, JSON.stringify(spec)), /invalid input/, field);
    assert.deepEqual(topologyState(kernel, solid), before);
    assert.equal(kernel.sketchDof(sketch), dof);
    assert.deepEqual(JSON.parse(kernel.sketchSolve(sketch, 20, 1e-8)).points, points);
    kernel.free();
  }
  console.log('ok - JSON edge/sketch references and supplied variable-fillet laws refuse before mutation');
}
