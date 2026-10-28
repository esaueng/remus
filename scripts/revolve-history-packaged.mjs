/** B18 full-turn radial-profile construction history on the distributed API. */
import assert from 'node:assert/strict';

const routes = ['direct', 'executeBatch', 'executeBatchV2'];
const kinds = [['face', 'faces', 'getSolidFaces'], ['edge', 'edges', 'getSolidEdges'], ['vertex', 'vertices', 'getSolidVertices']];
const profiles = [
  [[0, 0], [3, 0], [3, 2], [0, 2]],
  [[1, 0], [3, 0], [3, 2], [1, 2]],
  [[0, 0], [3, 0], [2, 2], [0, 2]],
  [[0, 0], [3, 0], [0, 2]],
];

function profile(kernel, points, scale) {
  const edges = points.map(([r, z], index) => {
    const [nextR, nextZ] = points[(index + 1) % points.length];
    return kernel.makeLineEdge(r * scale, 0, z * scale, nextR * scale, 0, nextZ * scale);
  });
  return kernel.makePlanarFaceFromWire(kernel.makeWire(Uint32Array.from(edges), true));
}

function quantities(points) {
  let volume = 0;
  let area = 0;
  for (let index = 0; index < points.length; index += 1) {
    const [r0, z0] = points[index];
    const [r1, z1] = points[(index + 1) % points.length];
    volume += Math.PI * (r0 * r0 + r0 * r1 + r1 * r1) * (z1 - z0) / 3;
    area += Math.PI * (r0 + r1) * Math.hypot(r1 - r0, z1 - z0);
  }
  return { volume: Math.abs(volume), area };
}

function invoke(kernel, route, face, angle) {
  if (route === 'direct') {
    return JSON.parse(kernel.revolveJournaled(face, 0, 0, 0, 0, 0, 1, angle));
  }
  const [row] = JSON.parse(kernel[route](JSON.stringify([{ op: 'revolveJournaled', args: { face, angle } }])));
  assert.ok(row.ok, `${route}: ${JSON.stringify(row)}`);
  return row.ok;
}

function resolved(kernel, references, solid) {
  const bound = new Map(kinds.map(([kind]) => [kind, new Set()]));
  for (const reference of references) {
    const result = JSON.parse(kernel.resolveRef(reference));
    assert.equal(result.status, 'bound');
    assert.equal(result.provenance, 'construction');
    assert.equal(result.entities.length, 1);
    bound.get(result.entities[0].kind).add(result.entities[0].handle);
  }
  for (const [kind, , query] of kinds) {
    assert.deepEqual(bound.get(kind), new Set(kernel[query](solid)));
  }
}

export function runRevolveHistoryPackaged({ BrepKernel }) {
  assert.equal(typeof BrepKernel.prototype.revolveJournaled, 'function');
  let cells = 0;
  for (const route of routes) {
    for (const scale of [0.001, 1, 1000]) {
      for (const points of profiles) {
        const kernel = new BrepKernel();
        try {
          const face = profile(kernel, points, scale);
          const before = kernel.checkpoint();
          const result = invoke(kernel, route, face, 360);
          assert.equal(result.evolution.origin, 'construction');
          assert.equal(result.evolution.completeness.accounted, true);
          assert.equal(result.evolution.completeness.resolved, true);
          const references = [];
          for (const [kind, plural, query] of kinds) {
            const expected = new Set(kernel[query](result.solid));
            const rows = result.evolution[plural];
            assert.deepEqual(new Set(rows.map((row) => row[kind])), expected);
            for (let index = 0; index < rows.length; index += 1) {
              references.push(kernel.makeOperationOutputRef(result.op, kind, index));
            }
          }
          const exact = quantities(points);
          assert.ok(Math.abs(kernel.volume(result.solid, 0.01 * scale) / scale ** 3 - exact.volume) < exact.volume * 1e-7);
          assert.ok(Math.abs(kernel.surfaceArea(result.solid, 0.01 * scale) / scale ** 2 - exact.area) < exact.area * 1e-7);
          assert.equal(kernel.validateSolid(result.solid), 0);
          resolved(kernel, references, result.solid);
          const after = kernel.checkpoint();
          kernel.linearPatternJournaled(result.solid, 1, 0, 0, 10 * scale, 2);
          resolved(kernel, references, result.solid);
          kernel.restore(after);
          resolved(kernel, references, result.solid);
          kernel.linearPatternJournaled(result.solid, 1, 0, 0, 10 * scale, 2);
          resolved(kernel, references, result.solid);
          if (points === profiles[0]) {
            kernel.restore(after);
            const top = [...kernel.getSolidFaces(result.solid)].find((candidate) =>
              kernel.getSurfaceType(candidate) === 'plane' && kernel.getFaceNormal(candidate)[2] > 0.9);
            assert.notEqual(top, undefined);
            for (const replay of [false, true]) {
              if (replay) kernel.restore(after);
              const edited = JSON.parse(kernel.moveFacesJournaled(result.solid, Uint32Array.of(top), 0.5 * scale));
              assert.ok(Math.abs(kernel.volume(edited.solid, 0.01 * scale) / scale ** 3 - 22.5 * Math.PI) < 1e-7);
              resolved(kernel, references, edited.solid);
              kernel.restore(after);
              resolved(kernel, references, result.solid);
            }
          }
          kernel.restore(before);
          assert.equal(JSON.parse(kernel.journalSummary()).length, 0);
          cells += 1;
        } finally {
          kernel.free();
        }
      }
    }
  }
  for (const route of routes) {
    const kernel = new BrepKernel();
    try {
      const face = profile(kernel, profiles[0], 1);
      const journal = kernel.journalSummary();
      const counts = JSON.parse(kernel.checkpointMemoryStats());
      if (route === 'direct') {
        assert.throws(() => kernel.revolveJournaled(face, 0, 0, 0, 0, 0, 1, 180), /unsupported/);
      } else {
        const [row] = JSON.parse(kernel[route](JSON.stringify([{ op: 'revolveJournaled', args: { face, angle: 180 } }])));
        assert.ok(row.error);
        if (route === 'executeBatchV2') {
          assert.equal(row.error.category, 'unsupported');
          assert.equal(row.error.details.kernelCode, 'revolve_history_unsupported');
        }
      }
      assert.equal(kernel.journalSummary(), journal);
      const after = JSON.parse(kernel.checkpointMemoryStats());
      assert.equal(after.allocatedSlots - after.retiredSlots, counts.allocatedSlots - counts.retiredSlots);
    } finally {
      kernel.free();
    }
  }
  console.log(`ok - journaled analytic revolve: ${cells} direct/batch cells and transactional refusal controls`);
}
