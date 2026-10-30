/** F3 material-retention regression through actual packaged WASM surfaces. */
import assert from 'node:assert/strict';

function invoke(kernel, surface, op, args, direct, label) {
  if (surface === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
  const result = JSON.parse(kernel[surface](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(result, 'ok'), `${label} ${op}: ${JSON.stringify(result)}`);
  return result.ok;
}

// The public transform API and native fixture both use row-major matrices.
const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const translation = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
const rotationX = angle => {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
};
const rotationZ = angle => {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
};
const multiply = (a, b) => Array.from({ length: 16 }, (_, index) => {
  const row = Math.floor(index / 4), column = index % 4;
  return a[row * 4] * b[column] + (a[row * 4 + 1] * b[4 + column]
    + (a[row * 4 + 2] * b[8 + column] + a[row * 4 + 3] * b[12 + column]));
});
const transformPoint = (matrix, point) => [0, 1, 2].map(row => matrix[row * 4] * point[0]
  + (matrix[row * 4 + 1] * point[1] + (matrix[row * 4 + 2] * point[2] + matrix[row * 4 + 3])));

export function runOpseqF3Packaged({ RemusKernel }) {
  const failures = [];
  let passed = 0;
  for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [1, 1000]) {
      for (const placed of [false, true]) {
        const kernel = new RemusKernel();
        const label = `${surface} F3 scale=${scale} placed=${placed}`;
        const run = (op, args, direct) => invoke(kernel, surface, op, args, direct, label);
        try {
          const tool = run('makeCylinder', { radius: 5.5 * scale, height: 1.5 * scale }, [5.5 * scale, 1.5 * scale]);
          const stock = run('makeCylinder', { radius: 2 * scale, height: 3.5 * scale }, [2 * scale, 3.5 * scale]);
          const placement = placed ? multiply(multiply(translation(10 * scale, -7 * scale, 3 * scale),
            rotationX(0.37)), rotationZ(0.23)) : identity();
          const toolMatrix = multiply(multiply(placement, translation(4 * scale, -2 * scale, -0.5 * scale)),
            rotationX(Math.PI / 6));
          run('transform', { solid: tool, matrix: toolMatrix }, [tool, Float64Array.from(toolMatrix)]);
          run('transform', { solid: stock, matrix: placement }, [stock, Float64Array.from(placement)]);
          const point = local => transformPoint(placement, local.map(coordinate => coordinate * scale));
          const ids = Uint32Array.of(stock, tool);
          const operands = kernel.serializeSolids(ids);
          try {
            const result = run('booleanWithQuality',
              { operation: 'cut', solidA: stock, solidB: tool, exactOnly: true },
              ['cut', stock, tool, true]);
            assert.equal(result.quality, 'exact', `${label}: exact cut`);
            const solid = result.solid;
            const tolerance = 1e-7;
            const kept = point([-1.5, 0, 1]);
            assert.equal(kernel.classifyPoint(stock, ...kept, tolerance), 'inside', label);
            assert.equal(kernel.classifyPoint(tool, ...kept, tolerance), 'outside', label);
            assert.equal(kernel.classifyPoint(solid, ...kept, tolerance), 'inside', `${label}: retained crescent`);
            const removed = point([0.5, -0.5, 1.5]);
            assert.equal(kernel.classifyPoint(stock, ...removed, tolerance), 'inside', label);
            assert.equal(kernel.classifyPoint(tool, ...removed, tolerance), 'inside', label);
            assert.equal(kernel.classifyPoint(solid, ...removed, tolerance), 'outside', `${label}: removed overlap`);
            const c = Math.sqrt(3) / 2;
            for (const x of [-1.5, -0.5, 0.5, 1.5]) {
              for (const y of [-1.5, -0.5, 0.5, 1.5]) {
                if (x * x + y * y >= 4) continue;
                for (const z of [0.25, 0.75, 1.25, 1.75, 2.25, 2.75, 3.25]) {
                  // Independent Cartesian inequalities in the unplaced unit
                  // fixture. Every probe is strictly inside the stock; the
                  // tool coordinates come from its inverse 30-degree rotation.
                  assert.ok(x * x + y * y < 4 && z > 0 && z < 3.5, `${label}: stock grid guard`);
                  const toolY = (y + 2) * c + (z + 0.5) / 2;
                  const toolZ = -(y + 2) / 2 + (z + 0.5) * c;
                  const radialSquared = (x - 4) ** 2 + toolY ** 2;
                  if (Math.abs(radialSquared - 30.25) < 0.02 || Math.abs(toolZ) < 0.02 || Math.abs(toolZ - 1.5) < 0.02) continue;
                  const insideTool = radialSquared < 30.25 && toolZ > 0 && toolZ < 1.5;
                  const probe = point([x, y, z]);
                  assert.equal(kernel.classifyPoint(stock, ...probe, tolerance), 'inside',
                    `${label}: analytic stock membership (${x}, ${y}, ${z})`);
                  assert.equal(kernel.classifyPoint(tool, ...probe, tolerance), insideTool ? 'inside' : 'outside',
                    `${label}: analytic tool membership (${x}, ${y}, ${z})`);
                  assert.equal(kernel.classifyPoint(solid, ...probe, tolerance), insideTool ? 'outside' : 'inside',
                    `${label}: independent material grid (${x}, ${y}, ${z})`);
                }
              }
            }
            assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, label);
            // Independent affine cross-section integral; native regression
            // carries its derivation. Geometry scales volume by scale cubed.
            const expected = 30.24010542122668 * scale ** 3;
            for (const volume of [kernel.volume(solid, 0.0001 * scale), JSON.parse(kernel.massProperties(solid)).volume]) {
              assert.ok(Math.abs(volume - expected) < 1e-4 * scale ** 3, `${label}: volume ${volume}, expected ${expected}`);
            }
            for (const deflection of [0.1, 0.01]) {
              const quality = JSON.parse(kernel.meshQuality(solid, deflection * scale, 0.1));
              assert.equal(quality.boundaryEdges, 0, `${label}: open mesh at ${deflection * scale}`);
              assert.equal(quality.nonManifoldEdges, 0, `${label}: non-manifold mesh`);
              assert.equal(quality.isWatertight, true, label);
            }
          } finally {
            assert.deepEqual(kernel.serializeSolids(ids), operands, `${label}: retained operands`);
          }
          passed += 1;
        } catch (error) {
          failures.push(new Error(`${label}: ${String(error)}`, { cause: error }));
        } finally {
          kernel.free();
        }
      }
    }
  }
  if (failures.length) throw new AggregateError(failures, `OpSeq F3 packaged matrix: ${passed}/12 cells passed`);
  assert.equal(passed, 12);
  console.log('ok - OpSeq F3 exact rotated-cylinder cut retains material across 12 packaged scale/placement/API cells');
}
