/** Imported rotational direct edits through installed direct and batch APIs. */
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const near = (a, b) => Math.abs(a - b) < 1e-6;
const fixture = new URL('../crates/io/tests/data/shapr3d_walking_stick_foot.step', import.meta.url);
const params = (k, face) => JSON.parse(k.getAnalyticSurfaceParams(face));
function geometrySnapshot(k, solid) {
  // A successful journaled edit legitimately appends construction history.
  // Every serialized source geometry and boundary-authority field stays fixed.
  const { journal, ...geometry } = JSON.parse(
    Buffer.from(k.serializeSolid(solid)).toString('utf8'),
  );
  return geometry;
}
function wall(k, solid, radius) {
  const found = [...k.getSolidFaces(solid)].filter((face) => {
    const p = params(k, face);
    return p.type === 'cylinder' && near(p.radius, radius);
  });
  assert.equal(found.length, 1, `one cylinder at r${radius}`);
  return found[0];
}
function plane(k, solid, z, sign) {
  const found = [...k.getSolidFaces(solid)].filter(
    (face) =>
      k.getSurfaceType(face) === 'plane' &&
      k.getFaceNormal(face)[2] * sign > 0.99 &&
      [...k.getFaceVertices(face)].every((vertex) => near(k.getVertexPosition(vertex)[2], z)),
  );
  assert.equal(found.length, 1, `one plane at z${z}`);
  return found[0];
}
function source(k, io) {
  let [solid] = k.deserializeSolids(io.importStep(readFileSync(fixture)));
  for (const [height, from, to] of [
    [30.156854249, 4, 3],
    [5.656854249, 4, 8.6],
    [7.562236639, 8.6, 5],
    [23.343145751, 4, 5],
    [29.742640685, 3, 5],
  ]) {
    const band = [...k.getSolidFaces(solid)].find((face) => {
      const p = params(k, face);
      return p.type === 'torus' && near(p.center[2], height) && near(p.minorRadius, from);
    });
    assert.notEqual(band, undefined);
    solid = k.resizeBlend(solid, band, from, to);
  }
  for (const radius of [16.5, 12.5]) {
    const edge = [...k.getSolidEdges(solid)].find(
      (edge) =>
        k.getEdgeCurveType(edge) === 'CIRCLE' &&
        near(Math.hypot(...k.getEdgeVertices(edge).slice(0, 2)), radius) &&
        near(k.getEdgeVertices(edge)[2], 50),
    );
    assert.notEqual(edge, undefined);
    solid = k.fillet(solid, Uint32Array.of(edge), 1);
  }
  solid = k.resizeCylindricalFace(solid, wall(k, solid, 12.5), 12);
  let z = 9;
  for (const offset of [13, -7.5, -7, 9]) {
    solid = k.moveFaces(solid, Uint32Array.of(plane(k, solid, z, 1)), offset);
    z += offset;
  }
  return k.resizeCylindricalFace(solid, wall(k, solid, 12), 11.5);
}
function qualify(k, solid) {
  assert.equal(k.validateSolid(solid), 0);
  assert.equal(k.getSolidFaces(solid).length, 13);
  assert.equal(k.getSolidEdges(solid).length, 22);
  assert.equal(k.getSolidVertices(solid).length, 12);
  assert.equal(JSON.parse(k.meshQuality(solid, 0.01)).isWatertight, true);
  assert.deepEqual(
    [...k.getSolidFaces(solid)].map((face) => k.getSurfaceType(face)).sort(),
    [
      'plane',
      'plane',
      'plane',
      'cone',
      'cone',
      'cylinder',
      'cylinder',
      'cylinder',
      'torus',
      'torus',
      'torus',
      'torus',
      'torus',
    ].sort(),
  );
}
export function runRotationalFaceEditsPackaged({ RemusKernel, RemusIo }) {
  for (const mode of ['direct', 'executeBatch', 'executeBatchV2', 'journal', 'journal-batch']) {
    const k = new RemusKernel(),
      io = new RemusIo();
    try {
      let solid = source(k, io);
      qualify(k, solid);
      let refs;
      for (const [from, radius] of [
        [11.5, 11],
        [11, 12],
        [20, 19.5],
        [16.5, 16],
      ]) {
        const face = wall(k, solid, from);
        const before = geometrySnapshot(k, solid);
        let result;
        if (mode === 'direct') result = k.resizeCylindricalFace(solid, face, radius);
        else if (mode === 'journal') {
          const operation = JSON.parse(k.resizeCylindricalFaceJournaled(solid, face, radius));
          result = operation.solid;
          refs ??= ['face', 'edge', 'vertex'].flatMap((kind) =>
            [
              ...k[
                { face: 'getSolidFaces', edge: 'getSolidEdges', vertex: 'getSolidVertices' }[kind]
              ](result),
            ].map((_, index) => k.makeOperationOutputRef(operation.op, kind, index)),
          );
        } else {
          const journaled = mode === 'journal-batch';
          const [response] = JSON.parse(
            k[mode === 'executeBatch' ? 'executeBatch' : 'executeBatchV2'](
              JSON.stringify([
                {
                  op: journaled ? 'resizeCylindricalFaceJournaled' : 'resizeCylindricalFace',
                  args: { solid, face, radius },
                },
              ]),
            ),
          );
          assert.equal(response.error, undefined, JSON.stringify(response));
          result = journaled ? response.ok.solid : response.ok;
        }
        qualify(k, result);
        assert.deepEqual(
          geometrySnapshot(k, solid),
          before,
          'success preserves source geometry and boundary authority',
        );
        if (refs)
          for (const ref of refs) {
            const bound = JSON.parse(k.resolveRef(ref));
            assert.equal(bound.status, 'bound', JSON.stringify(bound));
            assert.equal(bound.provenance, 'construction');
          }
        solid = result;
      }
      const before = k.volume(solid, 0.001);
      const moved = k.moveFaces(solid, Uint32Array.of(plane(k, solid, 16.5, 1)), 14);
      qualify(k, moved);
      assert.ok(Math.abs(k.volume(moved, 0.001) - before - Math.PI * 12 ** 2 * 14) < 1e-5);
      const extended = k.moveFaces(moved, Uint32Array.of(plane(k, moved, 0, -1)), 1);
      qualify(k, extended);
      const round = new RemusKernel();
      try {
        const [restored] = round.deserializeSolids(
          io.importStep(io.exportStep(k.serializeSolids(Uint32Array.of(extended)))),
        );
        qualify(round, restored);
        assert.ok(Math.abs(round.volume(restored, 0.001) - k.volume(extended, 0.001)) < 1e-5);
      } finally {
        round.free();
      }
      const snapshot = Buffer.from(k.serializeSolid(solid));
      const journal = k.journalSummary();
      assert.throws(() => k.resizeCylindricalFace(solid, wall(k, solid, 12), 20));
      assert.deepEqual(Buffer.from(k.serializeSolid(solid)), snapshot);
      assert.equal(k.journalSummary(), journal);
    } finally {
      k.free();
      io.free();
    }
  }
  console.log(
    'ok - rotational edits: full profile, precise bore changes, caps, direct/batch/history, STEP and rollback',
  );
}
