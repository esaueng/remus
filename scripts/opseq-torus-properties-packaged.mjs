/** A bounded torus pocket must be measured in its resolved tube-angle window. */
import assert from 'node:assert/strict';

const expectedVolume = 336.2866423605142;
const json = value => typeof value === 'string' ? JSON.parse(value) : value;
function run(kernel, api, op, args, direct) {
  if (api === 'direct') return direct();
  const [row] = JSON.parse(kernel[api](JSON.stringify([{ op, args }])));
  assert.ok(Object.hasOwn(row, 'ok'), `${api} ${op}: ${JSON.stringify(row)}`);
  return row.ok;
}
function placement(scale, placed) {
  if (!placed) return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const [s, c, sx, cx] = [Math.sin(0.37), Math.cos(0.37), Math.sin(0.41), Math.cos(0.41)];
  return [c, -s * cx, s * sx, 10 * scale, s, c * cx, -c * sx, -7 * scale,
    0, sx, cx, 3 * scale, 0, 0, 0, 1];
}
const world = (matrix, point) => [0, 1, 2].map(row => matrix[4 * row] * point[0]
  + matrix[4 * row + 1] * point[1] + matrix[4 * row + 2] * point[2] + matrix[4 * row + 3]);

function checkMesh(kernel, solid, deflection, scale, matrix, label) {
  const mesh = json(kernel.tessellateSolidGrouped(solid, deflection * scale, 0.35));
  const uses = new Map();
  const origin = world(matrix, [3 * scale, 1.5 * scale, -0.5 * scale]);
  let volume = 0;
  for (let at = 0; at < mesh.indices.length; at += 3) {
    const triangle = mesh.indices.slice(at, at + 3);
    const [a, b, c] = triangle.map(id => mesh.positions.slice(3 * id, 3 * id + 3).map((v, i) => v - origin[i]));
    volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
      + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
    for (let i = 0; i < 3; i += 1) {
      const x = triangle[i], y = triangle[(i + 1) % 3], key = x < y ? `${x},${y}` : `${y},${x}`;
      const directions = uses.get(key) ?? [];
      directions.push(x < y); uses.set(key, directions);
    }
  }
  for (const directions of uses.values()) {
    assert.equal(directions.length, 2, `${label}: raw mesh incidence`);
    assert.notEqual(directions[0], directions[1], `${label}: raw mesh orientation`);
  }
  const error = Math.abs(volume / scale ** 3 - expectedVolume) / expectedVolume;
  assert.ok(error < 0.005, `${label}: mesh quantity at deflection ${deflection}`);
  return error;
}

export function runOpseqTorusPropertiesPackaged({ RemusKernel }) {
  let cells = 0;
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [1, 0.001, 1000]) for (const placed of [false, true]) {
      const kernel = new RemusKernel();
      const label = `periodic torus pocket ${api} scale=${scale} placed=${placed}`;
      try {
        const torus = run(kernel, api, 'makeTorus', { majorRadius: 5, minorRadius: 2, segments: 16 },
          () => kernel.makeTorus(5, 2, 16));
        const cylinder = run(kernel, api, 'makeCylinder', { radius: 5.5, height: 7.5 },
          () => kernel.makeCylinder(5.5, 7.5));
        const move = [1, 0, 0, 3, 0, 1, 0, 1.5, 0, 0, 1, -0.5, 0, 0, 0, 1];
        run(kernel, api, 'transform', { solid: torus, matrix: move }, () => kernel.transformSolid(torus, Float64Array.from(move)));
        const matrix = placement(scale, placed);
        const operands = kernel.serializeSolids(Uint32Array.of(torus, cylinder));
        const outcome = json(run(kernel, api, 'booleanWithQuality', { operation: 'cut', solidA: torus, solidB: cylinder, exactOnly: true },
          () => kernel.booleanWithQuality('cut', torus, cylinder, true)));
        assert.equal(outcome.quality, 'exact', label);
        const solid = outcome.solid;
        // Exercise measurement on similarity transforms of the same exact
        // result. Building the boolean directly at 1e-3/1e3 already refuses on
        // the frozen source and is outside this property-integration family.
        if (scale !== 1 || placed) {
          const transformed = matrix.map((v, at) => at % 4 < 3 && at < 12 ? v * scale : v);
          run(kernel, api, 'transform', { solid, matrix: transformed },
            () => kernel.transformSolid(solid, Float64Array.from(transformed)));
        }
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(torus, cylinder)), operands, `${label}: unchanged operands`);
        assert.deepEqual(Array.from(kernel.getSolidFaces(solid), face => kernel.getSurfaceType(face)).sort(),
          ['cylinder', 'cylinder', 'plane', 'torus'], `${label}: native analytic carriers`);
        assert.equal(json(kernel.validateSolidDetailed(solid)).errorCount, 0, label);
        const properties = json(run(kernel, api, 'massProperties', { solid }, () => kernel.massProperties(solid)));
        assert.ok(Math.abs(properties.volume / scale ** 3 - expectedVolume) < expectedVolume * 1e-5,
          `${label}: mass ${properties.volume / scale ** 3} != independent annulus/disc integral ${expectedVolume}`);
        const probes = scale === 1 && !placed
          ? [-3.7, -0.7, 2.3, 5.3, 8.3, 10.3].flatMap(x => [-4.7, -1.7, 1.3, 4.3, 7.3]
            .flatMap(y => [-2.3, -1.1, 0.3, 1.1, 2.3].map(z => [x, y, z])))
          : [[-0.7, 1.3, -1.1], [-0.7, 1.3, 0.3], [5.3, -1.7, 0.3], [8.3, 1.3, 0.3],
            [-3.7, -4.7, 2.3], [2.3, 7.3, -1.1], [10.3, -1.7, -1.1], [2.3, 1.3, 2.3]];
        for (const [x, y, z] of probes) {
          const torusMaterial = (Math.hypot(x - 3, y - 1.5) - 5) ** 2 + (z + 0.5) ** 2 < 4;
          const cutterMaterial = x * x + y * y < 5.5 ** 2 && z > 0 && z < 7.5;
          assert.equal(kernel.classifyPoint(solid, ...world(matrix, [x * scale, y * scale, z * scale]), 1e-7),
            torusMaterial && !cutterMaterial ? 'inside' : 'outside', `${label}: material ${x},${y},${z}`);
        }
        // Mesh deflection has its own convergence contract; it is not the
        // stricter Gauss mass-properties accuracy assertion above.
        const coarse = checkMesh(kernel, solid, 0.02, scale, matrix, label);
        const fine = checkMesh(kernel, solid, 0.005, scale, matrix, label);
        assert.ok(fine < coarse, `${label}: mesh quantity convergence`);
        cells += 1;
      } finally { kernel.free(); }
    }
  }
  console.log(`ok - periodic torus pocket properties: ${cells} direct/batch cells`);
}
