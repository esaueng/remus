/** The recording's three-edge selection through every shipped fillet entry. */
import assert from 'node:assert/strict';

const near = (a, b) => Math.abs(a - b) < 1e-7;
function fixture(k) {
  let boss = k.makeCylinder(6, 35);
  const rim = [...k.getSolidEdges(boss)].filter(e => k.getEdgeCurveType(e) === 'CIRCLE' && near(k.getEdgeVertices(e)[2], 35));
  boss = k.fillet(boss, Uint32Array.from(rim), 2);
  const body = k.fuse(k.makeBox(30, 18, 26), boss);
  k.unifyFaces(body);
  const edges = [...k.getSolidEdges(body)];
  const arc = edges.find(e => k.getEdgeCurveType(e) === 'CIRCLE' && near(k.getEdgeVertices(e)[2], 26) && near(k.edgeLength(e), 3 * Math.PI));
  const vertical = edges.find(e => {
    const p = k.getEdgeVertices(e);
    return k.getEdgeCurveType(e) === 'LINE' && near(p[0], 6) && near(p[3], 6) && near(p[1], 0) && near(p[4], 0) && near(k.edgeLength(e), 26);
  });
  const top = edges.find(e => {
    const p = k.getEdgeVertices(e);
    return k.getEdgeCurveType(e) === 'LINE' && near(p[1], 0) && near(p[4], 0) && near(p[2], 26) && near(p[5], 26) && near(k.edgeLength(e), 24);
  });
  assert.ok([arc, vertical, top].every(e => e !== undefined));
  return { body, edges: [arc, vertical, top] };
}

export function runMixedSeamFilletPackaged({ RemusKernel }) {
  let expected;
  const orders = [[0,1,2],[0,2,1],[1,0,2],[1,2,0],[2,0,1],[2,1,0]];
  for (const [mode, order] of [...orders.map(order => ['detailed', order]), ...['direct','evolution','executeBatch','executeBatchV2'].map(mode => [mode, orders[0]])]) {
    const k = new RemusKernel();
    try {
      const { body, edges } = fixture(k);
      const selected = Uint32Array.from(order.map(i => edges[i]));
      const before = Buffer.from(k.serializeSolid(body));
      const blank = k.volume(body, 0.005);
      let result;
      if (mode === 'detailed') {
        const detailed = k.filletDetailed(body, selected, 1);
        assert.equal(detailed.status, 'ok', JSON.stringify(detailed));
        assert.equal(detailed.details.quality, 'approximate', 'the final NURBS band must remain disclosed');
        result = detailed.value;
      } else if (mode === 'direct') result = k.fillet(body, selected, 1);
      else if (mode === 'evolution') result = k.filletWithEvolution(body, selected, 1).result.solid;
      else {
        const response = JSON.parse(k[mode](JSON.stringify([{ op: 'fillet', args: { solid: body, edges: [...selected], radius: 1 } }])))[0];
        assert.equal(response.error, undefined, JSON.stringify(response));
        result = response.ok;
      }
      assert.notEqual(result, body);
      assert.equal(k.validateSolidRelaxed(result), 0);
      const volume = k.volume(result, 0.005);
      assert.ok(volume > blank + 1 && volume < blank + 10, 'must include the seam addition and convex cut, not a subset');
      expected ??= volume;
      assert.ok(Math.abs(volume - expected) < 1e-6, `${mode}: all entry points and orders agree`);
      assert.deepEqual(Buffer.from(k.serializeSolid(body)), before, 'success preserves the original input');
      if (mode === 'detailed') {
        const strict = k.filletDetailed(body, selected, 1, true);
        assert.equal(strict.status, 'error');
        assert.equal(strict.code, 'exact_only_unattainable');
        assert.deepEqual(Buffer.from(k.serializeSolid(body)), before, 'quality refusal rolls back both stages');
      }
      const oversized = k.filletDetailed(body, selected, 40);
      assert.equal(oversized.status, 'error');
      assert.doesNotMatch(oversized.details.message, /would round on their own/);
      assert.deepEqual(Buffer.from(k.serializeSolid(body)), before, 'geometric refusal preserves the input');
    } finally { k.free(); }
  }
  console.log('ok - mixed seam three-edge fillet, all orders/direct/evolution/batch, quality and rollback');
}
