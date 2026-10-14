/** Analytic ray-query contracts through the optimized wasm32 package. */
import assert from 'node:assert/strict';

export function runWasmRayNurbsPackaged({ BrepKernel }) {
  let cells = 0;
  for (const kind of ['box', 'cylinder']) for (const scale of [0.1, 1, 10]) {
    for (const placed of [false, true]) {
      const kernel = new BrepKernel();
      try {
        const solid = kind === 'box'
          ? kernel.makeBox(6 * scale, 4 * scale, 2 * scale)
          : kernel.makeCylinder(2 * scale, 4 * scale);
        if (placed) {
          const c = Math.cos(0.37), s = Math.sin(0.37);
          kernel.transformSolid(solid, Float64Array.from([
            c, 0, s, 37 * scale, 0, 1, 0, -91 * scale,
            -s, 0, c, 13 * scale, 0, 0, 0, 1,
          ]));
        }
        assert.ok(kernel.convertToBspline(solid) >= 3);
        const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(solid)));
        const label = `${kind} scale=${scale} placed=${placed}`;
        const expected = JSON.parse(kernel.solidEdgeRelations(solid));
        assert.equal(expected.length, kind === 'box' ? 12 : 3, label);
        const convex = expected.filter(row => row.relation === 'convex');
        assert.equal(convex.length, kind === 'box' ? 12 : 2, label);
        for (const row of convex) assert.ok(Math.abs(row.dihedralAngle - Math.PI / 2) < 1e-8, label);
        for (const api of ['executeBatch', 'executeBatchV2']) {
          const [row] = JSON.parse(kernel[api](JSON.stringify([
            { op: 'solidEdgeRelations', args: { solid } },
          ])));
          assert.deepEqual(row.ok, expected, `${api} ${label}`);
        }
        const features = kernel.recognizeFeatures(solid, 0.08 * scale);
        assert.deepEqual(JSON.parse(features), JSON.parse(kernel.recognizeFeatures(solid, 0.08 * scale)), label);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), before, label);
        cells++;
      } finally {
        kernel.free();
      }
    }
  }
  console.log(`ok - NURBS ray queries: ${cells} scaled/placed box and periodic-cylinder cells, direct/batch parity and immutable recognition`);
}
