/** B79: independent box-oracle partitions through existing planar holes. */
import assert from 'node:assert/strict';

const plate = [[-5, 0, -5], [5, 1, 5]];
const bar = [[-4, 1, -0.5], [4, 2, 0.5]];
const cross = [[-0.5, 1, -4], [0.5, 2, 4]];
const pocket = [[-4, 0.5, -0.5], [4, 1.5, 0.5]];
const through = [[-2, -1, -2], [2, 3, 2]];
const volume = box => box[1].reduce((v, coordinate, i) => v * (coordinate - box[0][i]), 1);
const overlap = (...boxes) => [0, 1, 2].reduce((v, i) => v * Math.max(0,
  Math.min(...boxes.map(box => box[1][i])) - Math.max(...boxes.map(box => box[0][i]))), 1);
const translation = (x, y, z) => Float64Array.of(
  1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1,
);
const fixtures = [
  { name: 'bar through', tool: through, inside: [3, 1.5, 0], outside: [0, 1.5, 0] },
  { name: 'cross through', shape: 'cross', tool: through, inside: [0, 1.5, 3], outside: [0, 1.5, 0] },
  { name: 'pocket through', shape: 'pocket', tool: through, inside: [3, 0.25, 0], outside: [3, 0.75, 0], alsoOutside: [0, 0.25, 0] },
  { name: 'blind', tool: [[-2, -1, -2], [2, 1.5, 2]], inside: [0, 1.75, 0], outside: [0, 1.25, 0] },
  { name: 'pocketing', tool: [[-2, 0.5, -2], [2, 3, 2]], inside: [0, 0.25, 0], outside: [0, 0.75, 0] },
  { name: 'reversed cut', tool: through, reverse: true, inside: [0, -0.5, 0], outside: [0, 0.5, 0] },
  { name: 'intersect', tool: through, operation: 'intersect', inside: [0, 1.5, 0], outside: [3, 1.5, 0] },
  { name: 'fuse', tool: through, operation: 'fuse', inside: [0, -0.5, 0], outside: [3, 1.5, 3] },
  { name: 'outer reaching control', tool: [[-2, -1, -2], [6, 3, 2]], inside: [-3, 1.5, 0], outside: [3, 1.5, 0] },
  { name: 'in footprint control', tool: [[-2, -1, -0.25], [2, 3, 0.25]], inside: [0, 1.5, 0.4], outside: [0, 1.5, 0] },
  { name: 'nested footprint control', shape: 'pocket', tool: [[-4.5, -1, -1], [4.5, 3, 1]], inside: [-4.75, 0.75, 0], outside: [3, 0.25, 0] },
];

export function runB79PlanarHolePartitionsPackaged({ BrepKernel }) {
  for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const [scale, offset] of [[1, [0, 0, 0]], [1, [10, -10, 10]], [1000, [0, 0, 0]]]) {
      for (const fixture of fixtures) {
        const kernel = new BrepKernel();
        const label = `${surface} B79 ${fixture.name} scale=${scale} offset=${offset}`;
        try {
          const point = local => local.map((coordinate, i) => coordinate * scale + offset[i]);
          const box = bounds => {
            const solid = kernel.makeBox(...bounds[1].map((coordinate, i) => (coordinate - bounds[0][i]) * scale));
            kernel.transformSolid(solid, translation(...point(bounds[0])));
            return solid;
          };
          const boolean = (operation, a, b) => {
            let outcome;
            if (surface === 'direct') outcome = kernel.booleanWithQuality(operation, a, b, true);
            else {
              const row = JSON.parse(kernel[surface](JSON.stringify([{ op: 'booleanWithQuality',
                args: { operation, solidA: a, solidB: b, exactOnly: true } }])))[0];
              assert.ok(Object.hasOwn(row, 'ok'), `${label}: ${JSON.stringify(row)}`);
              outcome = row.ok;
            }
            assert.equal(outcome.quality, 'exact', label);
            return outcome.solid;
          };
          let operand;
          let operandVolume;
          let common;
          if (fixture.shape === 'pocket') {
            operand = boolean('cut', box(plate), box(pocket));
            operandVolume = volume(plate) - overlap(plate, pocket);
            common = overlap(plate, fixture.tool) - overlap(plate, pocket, fixture.tool);
          } else {
            const rib = fixture.shape === 'cross' ? boolean('fuse', box(bar), box(cross)) : box(bar);
            operand = boolean('fuse', box(plate), rib);
            operandVolume = volume(plate) + volume(bar);
            common = overlap(plate, fixture.tool) + overlap(bar, fixture.tool);
            if (fixture.shape === 'cross') {
              operandVolume += volume(cross) - overlap(bar, cross);
              common += overlap(cross, fixture.tool) - overlap(bar, cross, fixture.tool);
            }
          }
          const tool = box(fixture.tool);
          const ids = Uint32Array.of(operand, tool);
          const before = kernel.serializeSolids(ids);
          const operation = fixture.operation ?? 'cut';
          const result = fixture.reverse ? boolean(operation, tool, operand) : boolean(operation, operand, tool);
          const expectedUnit = operation === 'fuse' ? operandVolume + volume(fixture.tool) - common
            : operation === 'intersect' ? common : (fixture.reverse ? volume(fixture.tool) : operandVolume) - common;
          const expected = expectedUnit * scale ** 3;
          const measured = JSON.parse(kernel.massProperties(result)).volume;
          assert.ok(Math.abs(measured - expected) <= expected * 1e-9, `${label}: ${measured} vs ${expected}`);
          assert.equal(JSON.parse(kernel.validateSolidDetailed(result)).errorCount, 0, label);
          const uses = new Map();
          for (const face of kernel.getSolidFaces(result)) {
            assert.equal(kernel.getSurfaceType(face), 'plane', label);
            for (const wire of kernel.getFaceWires(face)) {
              for (const edge of kernel.getWireEdges(wire)) uses.set(edge, (uses.get(edge) ?? 0) + 1);
            }
          }
          assert.ok(uses.size > 0 && [...uses.values()].every(count => count === 2), `${label}: BRep edge uses`);
          for (const deflection of [0.1, 0.01]) {
            const mesh = JSON.parse(kernel.meshQuality(result, deflection * scale));
            assert.equal(mesh.boundaryEdges, 0, label);
            assert.equal(mesh.nonManifoldEdges, 0, label);
            assert.equal(mesh.isWatertight, true, label);
          }
          for (const [probe, expectedClass] of [[fixture.inside, 'inside'], [fixture.outside, 'outside']]) {
            assert.equal(kernel.classifyPoint(result, ...point(probe), 1e-7 * scale), expectedClass, label);
          }
          if (fixture.alsoOutside) {
            assert.equal(kernel.classifyPoint(result, ...point(fixture.alsoOutside), 1e-7 * scale), 'outside', label);
          }
          assert.deepEqual(kernel.serializeSolids(ids), before, `${label}: retained operands`);
        } finally {
          kernel.free();
        }
      }
    }
  }
  console.log('ok - B79 exact planar-hole partitions and footprint controls, direct and both batches');
}
