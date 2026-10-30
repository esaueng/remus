/** Periodic and trimmed torus ray-query contracts in the shipping wasm32 module. */
import assert from 'node:assert/strict';

export function runWasmRayTorusPackaged({ RemusKernel }) {
  let cells = 0;
  for (const scale of [0.1, 1, 10]) for (const placed of [false, true]) {
    const kernel = new RemusKernel();
    try {
      const torus = kernel.makeTorus(10 * scale, 3 * scale, 32);
      const tool = kernel.makeBox(8 * scale, 8 * scale, 8 * scale);
      kernel.transformSolid(tool, Float64Array.from([
        1, 0, 0, 6 * scale, 0, 1, 0, -4 * scale,
        0, 0, 1, -4 * scale, 0, 0, 0, 1,
      ]));
      if (placed) {
        const c = Math.cos(0.37), s = Math.sin(0.37);
        const matrix = Float64Array.from([
          c, 0, s, 37 * scale, 0, 1, 0, -91 * scale,
          -s, 0, c, 13 * scale, 0, 0, 0, 1,
        ]);
        for (const solid of [torus, tool]) kernel.transformSolid(solid, matrix);
      }
      const notch = kernel.booleanWithQuality('cut', torus, tool, true);
      assert.equal(notch.quality, 'exact');
      for (const solid of [torus, notch.solid]) {
        const label = `torus scale=${scale} placed=${placed} notch=${solid === notch.solid}`;
        const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(solid)));
        const expected = JSON.parse(kernel.solidEdgeRelations(solid));
        assert.ok(expected.length >= 2, label);
        for (const api of ['executeBatch', 'executeBatchV2']) {
          const [row] = JSON.parse(kernel[api](JSON.stringify([
            { op: 'solidEdgeRelations', args: { solid } },
          ])));
          assert.deepEqual(row.ok, expected, `${api} ${label}`);
        }
        const features = kernel.recognizeFeatures(solid, 0.08 * scale);
        assert.deepEqual(JSON.parse(features), JSON.parse(kernel.recognizeFeatures(solid, 0.08 * scale)), label);
        const volume = kernel.volume(solid, 0.01 * scale) / scale ** 3;
        const oracle = 180 * Math.PI ** 2 - (solid === notch.solid ? 233.17975756277542 : 0);
        assert.ok(Math.abs(volume - oracle) / oracle < 0.001, `${label}: volume=${volume}`);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), before, label);
        cells++;
      }
    } finally {
      kernel.free();
    }
  }
  console.log(`ok - torus ray queries: ${cells} periodic/notched scale/placement cells, direct/batch parity, recognition and quantity`);
}
