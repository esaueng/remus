/** A closed spherical cap must be classified from its own retained region. */
import assert from 'node:assert/strict';

const placement = (scale, placed) => {
  const [sine, cosine] = placed ? [Math.sin(0.37), Math.cos(0.37)] : [0, 1];
  return [cosine, -sine, 0, placed ? 10 * scale : 0,
    sine, cosine, 0, placed ? -7 * scale : 0, 0, 0, 1, placed ? 3 * scale : 0, 0, 0, 0, 1];
};
const world = (matrix, point) => [0, 1, 2].map(row => matrix[4 * row] * point[0]
  + matrix[4 * row + 1] * point[1] + matrix[4 * row + 2] * point[2] + matrix[4 * row + 3]);
function run(kernel, api, operation, args, direct) {
  if (api === 'direct') return direct();
  const [row] = JSON.parse(kernel[api](JSON.stringify([{ op: operation, args }])));
  assert.ok(Object.hasOwn(row, 'ok'), `${api} ${operation}: ${JSON.stringify(row)}`);
  return row.ok;
}
function expectedUnion() {
  const r = 4.5, big = 5.5, d = Math.sqrt(17);
  const lens = Math.PI * (r + big - d) ** 2
    * (d * d + 2 * d * (r + big) - 3 * (r - big) ** 2) / (12 * d);
  return 4 * Math.PI * (r ** 3 + big ** 3) / 3 - lens;
}

export function runOpseqSphereCapPackaged({ BrepKernel }) {
  let cells = 0;
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [0.001, 1, 1000]) for (const placed of [false, true]) for (const swapped of [false, true]) {
      const kernel = new BrepKernel();
      const label = `closed sphere cap ${api} scale=${scale} placed=${placed} swapped=${swapped}`;
      try {
        const a = run(kernel, api, 'makeSphere', { radius: 4.5 * scale, segments: 16 }, () => kernel.makeSphere(4.5 * scale, 16));
        const b = run(kernel, api, 'makeSphere', { radius: 5.5 * scale, segments: 16 }, () => kernel.makeSphere(5.5 * scale, 16));
        const move = [1, 0, 0, 0, 0, 0.7071067811865476, -0.7071067811865475, scale,
          0, 0.7071067811865475, 0.7071067811865476, -4 * scale, 0, 0, 0, 1];
        run(kernel, api, 'transform', { solid: a, matrix: move }, () => kernel.transformSolid(a, Float64Array.from(move)));
        const matrix = placement(scale, placed);
        if (placed) for (const solid of [a, b]) {
          run(kernel, api, 'transform', { solid, matrix }, () => kernel.transformSolid(solid, Float64Array.from(matrix)));
        }
        const operands = kernel.serializeSolids(Uint32Array.of(a, b));
        const solidA = swapped ? b : a, solidB = swapped ? a : b;
        const outcome = run(kernel, api, 'booleanWithQuality', { operation: 'fuse', solidA, solidB, exactOnly: true },
          () => kernel.booleanWithQuality('fuse', solidA, solidB, true));
        assert.equal(outcome.quality, 'exact', label);
        const solid = outcome.solid;
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(a, b)), operands, `${label}: unchanged operands`);
        assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, label);
        for (const value of [kernel.volume(solid, 0.01 * scale), JSON.parse(kernel.massProperties(solid)).volume]) {
          assert.ok(Math.abs(value / scale ** 3 - expectedUnion()) < 1e-5 * expectedUnion(), `${label}: volume ${value / scale ** 3}`);
        }
        const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
        const faces = doc.shells[doc.solids[doc.solid_roots[0]].outer_shell].faces;
        const edgeUses = new Map();
        for (const index of faces) {
          const face = doc.faces[index];
          assert.ok(Object.hasOwn(face.surface, 'Sphere'), `${label}: analytic carrier`);
          for (const wire of [face.outer_wire, ...face.inner_wires]) for (const use of doc.wires[wire].edges) {
            const directions = edgeUses.get(use.edge) ?? [];
            directions.push(use.forward !== face.reversed); edgeUses.set(use.edge, directions);
          }
        }
        for (const directions of edgeUses.values()) {
          assert.equal(directions.length, 2, `${label}: B-Rep incidence`);
          assert.notEqual(directions[0], directions[1], `${label}: opposing coedges`);
        }
        for (const [x, y, z] of [[0, 0, 4], [-5, 0, 0], [0, 0, 0], [0, 1, -4], [0, 1, -8], [0, 0, 6], [0, 0, -9]]) {
          const inside = x * x + (y - 1) ** 2 + (z + 4) ** 2 < 4.5 ** 2 || x * x + y * y + z * z < 5.5 ** 2;
          const point = world(matrix, [x * scale, y * scale, z * scale]);
          assert.equal(kernel.classifyPoint(solid, ...point, 1e-7), inside ? 'inside' : 'outside', `${label}: material ${x},${y},${z}`);
        }
        for (const deflection of [0.02, 0.005]) {
          const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection * scale, 0.35));
          const uses = new Map();
          for (let at = 0; at < mesh.indices.length; at += 3) {
            const triangle = mesh.indices.slice(at, at + 3);
            for (let index = 0; index < 3; index += 1) {
              const a = triangle[index], b = triangle[(index + 1) % 3];
              const key = a < b ? `${a},${b}` : `${b},${a}`, directions = uses.get(key) ?? [];
              directions.push(a < b); uses.set(key, directions);
            }
          }
          for (const directions of uses.values()) {
            assert.equal(directions.length, 2, `${label}: raw mesh incidence`);
            assert.notEqual(directions[0], directions[1], `${label}: raw mesh winding`);
          }
        }
        cells += 1;
      } finally { kernel.free(); }
    }
  }
  console.log(`ok - closed spherical cap: ${cells} direct/batch cells`);
}
