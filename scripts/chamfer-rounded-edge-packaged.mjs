/** One planar ridge through an earlier round, through every shipped entry. */
import assert from 'node:assert/strict';

const near = (a, b) => Math.abs(a - b) < 1e-7;
function fixture(k) {
  const box = k.makeBox(36, 41, 11.5);
  const far = [...k.getSolidEdges(box)].filter(edge => {
    const p = k.getEdgeVertices(edge);
    return near(Math.abs(p[2] - p[5]), 11.5) && [[36, 0], [0, 41]].some(
      ([x, y]) => near(p[0], x) && near(p[3], x) && near(p[1], y) && near(p[4], y)
    );
  });
  assert.equal(far.length, 2);
  const body = k.fillet(box, Uint32Array.from(far), 1);
  const edge = [...k.getSolidEdges(body)].find(edge => {
    const p = k.getEdgeVertices(edge);
    return near(p[1], 0) && near(p[4], 0) && near(p[2], 0) && near(p[5], 0)
      && k.edgeLength(edge) > 30;
  });
  assert.notEqual(edge, undefined);
  return { body, edge };
}

export function runSingleEdgeChamferPackaged({ RemusKernel }) {
  // Independent section integral for a d=2.1 bevel through one R1 end.
  const removed = 35 * 1.6 + 1.1 * Math.PI / 4 + 1 / 3 + 18 * 1.1 ** 2;
  const expected = 36 * 41 * 11.5 - 2 * (1 - Math.PI / 4) * 11.5 - removed;
  for (const mode of ['direct', 'detailed', 'evolution', 'executeBatch', 'executeBatchV2']) {
    const k = new RemusKernel();
    try {
      const { body, edge } = fixture(k);
      const before = Buffer.from(k.serializeSolid(body));
      const selected = Uint32Array.of(edge);
      let result;
      if (mode === 'direct') result = k.chamfer(body, selected, 2.1);
      else if (mode === 'detailed') {
        const detailed = k.chamferDetailed(body, selected, 2.1, true);
        assert.equal(detailed.status, 'ok', JSON.stringify(detailed));
        assert.equal(detailed.details.quality, 'exact');
        result = detailed.value;
      } else if (mode === 'evolution') result = k.chamferWithEvolution(body, selected, 2.1).result.solid;
      else {
        const reply = JSON.parse(k[mode](JSON.stringify([
          { op: 'chamfer', args: { solid: body, edges: [edge], distance: 2.1 } },
        ])))[0];
        assert.equal(reply.error, undefined, JSON.stringify(reply));
        result = reply.ok;
      }
      assert.notEqual(result, body);
      assert.equal(k.validateSolidRelaxed(result), 0);
      assert.ok(Math.abs(k.volume(result, 0.001) - expected) < 0.002, `${mode}: section volume`);
      assert.equal([...k.getSolidFaces(result)].filter(face => k.getSurfaceType(face) === 'cylinder').length, 2);
      assert.deepEqual(Buffer.from(k.serializeSolid(body)), before, `${mode}: source unchanged`);
    } finally { k.free(); }
  }
  console.log('ok - isolated chamfer through an earlier round: direct/detailed/evolution/batch, exact volume and retained cylinders');
}
