/** Public package regression for B61's shared variable-fillet boundaries. */
import assert from 'node:assert/strict';

const routes = [
  'direct', 'directDetailed',
  'executeBatch:filletVariable', 'executeBatchV2:filletVariable',
  'executeBatch:filletVariableDetailed', 'executeBatchV2:filletVariableDetailed',
];

function invoke(kernel, route, solid, specs, exactOnly = false) {
  if (route === 'direct') return kernel.filletVariable(solid, JSON.stringify(specs));
  if (route === 'directDetailed') return kernel.filletVariableDetailed(solid, JSON.stringify(specs), exactOnly);
  const [batch, operation] = route.split(':');
  const row = JSON.parse(kernel[batch](JSON.stringify([{ op: operation, args: { solid, specs, exactOnly } }])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), JSON.stringify(row));
  return row.ok;
}

function close(actual, expected, tolerance, label) {
  assert.ok(Number.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
    `${label}: ${actual} vs ${expected}`);
}

function closedMesh(kernel, solid, deflection, label) {
  const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection, 0.1));
  assert.ok(mesh.positions.length > 0 && mesh.positions.every(Number.isFinite), `${label}: finite positions`);
  const incidences = new Map();
  for (let i = 0; i < mesh.indices.length; i += 3) {
    const triangle = mesh.indices.slice(i, i + 3);
    assert.ok(triangle.every(v => Number.isInteger(v) && v >= 0 && v * 3 < mesh.positions.length), label);
    for (let j = 0; j < 3; j++) {
      const a = triangle[j], b = triangle[(j + 1) % 3];
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      const value = incidences.get(key) ?? [0, 0];
      value[0]++;
      value[1] += a < b ? 1 : -1;
      incidences.set(key, value);
    }
  }
  for (const [edge, [uses, balance]] of incidences) {
    assert.equal(uses, 2, `${label}: boundary/non-manifold mesh edge ${edge}`);
    assert.equal(balance, 0, `${label}: same-sense mesh edge ${edge}`);
  }
}

function unprovedBoundaryRefusal(BrepKernel, route) {
  const kernel = new BrepKernel();
  const label = `B61 ${route}: unproved large-coordinate boundary`;
  try {
    const solid = kernel.makeBox(1e7, 1e7, 1e7);
    const edge = Array.from(kernel.getSolidEdges(solid)).find(id => {
      const p = kernel.getEdgeVertices(id);
      return p[0] === 0 && p[1] === 0 && p[3] === 0 && p[4] === 0;
    });
    assert.notEqual(edge, undefined, label);
    const state = () => {
      const memory = JSON.parse(kernel.checkpointMemoryStats());
      return {
        arena: kernel.serializeSolids(Uint32Array.of(solid)),
        journal: kernel.journalSummary(),
        liveSlots: memory.allocatedSlots - memory.retiredSlots,
      };
    };
    const before = state();
    const specs = [{ edge, law: 'constant', radius: 9e6 }];
    if (route === 'direct') {
      assert.throws(() => kernel.filletVariable(solid, JSON.stringify(specs)),
        /unsupported-variable-boundary/, label);
    } else {
      let output;
      if (route === 'directDetailed') {
        output = kernel.filletVariableDetailed(solid, JSON.stringify(specs), false);
      } else {
        const [batch, operation] = route.split(':');
        output = JSON.parse(kernel[batch](JSON.stringify([{ op: operation, args: { solid, specs } }])))[0];
      }
      if (route.endsWith('Detailed')) {
        const result = route === 'directDetailed' ? output : output.ok;
        assert.equal(result?.status, 'error', label);
        assert.equal(result.code, 'unsupported-variable-boundary', label);
        assert.equal(result.category, 'unsupported', label);
        assert.equal(result.details.kernelCode, 'unsupported-variable-boundary', label);
      } else if (route.startsWith('executeBatchV2')) {
        assert.equal(output.error?.code, 'operation_failed', label);
        assert.equal(output.error.category, 'unsupported', label);
        assert.equal(output.error.details.kernelCode, 'unsupported-variable-boundary', label);
      } else {
        assert.match(output.error, /native stripe boundary lacks a certified pcurve/, label);
      }
    }
    assert.deepEqual(state(), before, `${label}: source, journal and live-entity rollback`);
  } finally { kernel.free(); }
}

export function runVariableFilletBoundariesPackaged({ BrepKernel }) {
  let cells = 0;
  for (const route of routes) for (const scale of [0.001, 1, 1000]) {
    for (const radiusRatio of [1, 9]) for (const placed of [false, true]) {
      const kernel = new BrepKernel();
      const label = `B61 ${route} scale=${scale} radius=${radiusRatio} placed=${placed}`;
      try {
        const solid = kernel.makeBox(10 * scale, 10 * scale, 10 * scale);
        const edge = Array.from(kernel.getSolidEdges(solid)).find(id => {
          const p = kernel.getEdgeVertices(id);
          return p[0] === 0 && p[1] === 0 && p[3] === 0 && p[4] === 0;
        });
        assert.notEqual(edge, undefined, label);
        if (placed) kernel.transformSolid(solid, Float64Array.of(
          0, 0, 1, 37 * scale,
          1, 0, 0, -19 * scale,
          0, 1, 0, 11 * scale,
          0, 0, 0, 1,
        ));
        const before = kernel.serializeSolids(Uint32Array.of(solid));
        const radius = radiusRatio * scale;
        const specs = [{ edge, law: 'constant', radius }];
        const output = invoke(kernel, route, solid, specs);
        const detailed = route.endsWith('Detailed');
        if (detailed) {
          assert.equal(output.status, 'ok', label);
          // Existing disclosure remains in force; repairing the boundary
          // does not silently promote the variable engine to exact-only.
          assert.equal(output.details.quality, 'approximate', label);
        }
        const result = detailed ? output.value : output;
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), before, `${label}: original operand`);
        assert.equal(kernel.validateSolid(result), 0, label);
        const document = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(result))));
        const nativeUses = document.boundary_authority.coedges.filter(use => document.edges[use.edge].curve?.NurbsCurve);
        assert.equal(nativeUses.length, 8, `${label}: four native boundaries shared by both faces`);
        assert.ok(nativeUses.every(use => use.pcurve), `${label}: both-sided exact pcurves`);
        const [restored] = kernel.deserializeSolids(new TextEncoder().encode(JSON.stringify(document)));
        assert.equal(kernel.validateSolid(restored), 0, `${label}: pcurve arena round trip`);
        if (scale === 1 && radiusRatio === 9 && !placed) {
          const poisoned = structuredClone(document);
          const arc = poisoned.edges.find(edge => edge.curve?.NurbsCurve?.degree === 2).curve.NurbsCurve;
          arc.control_points[1][0] += 0.1; // Preserve endpoints and native trim.
          const [invalid] = kernel.deserializeSolids(new TextEncoder().encode(JSON.stringify(poisoned)));
          assert.ok(kernel.validateSolid(invalid) > 0, `${label}: ordinary validator rejects changed native boundary`);
        }
        const expected = (1000 - (1 - Math.PI / 4) * 10 * radiusRatio ** 2) * scale ** 3;
        close(JSON.parse(kernel.massProperties(result)).volume, expected, 1e-6 * expected, `${label}: mass volume`);
        for (const deflection of [0.01 * scale, 0.001 * scale]) {
          close(kernel.volume(result, deflection), expected, 1e-3 * expected, `${label}: volume`);
          closedMesh(kernel, result, deflection, label);
        }
        const faces = Array.from(kernel.getSolidFaces(result));
        const walls = faces.filter(face => kernel.getSurfaceType(face) !== 'plane');
        assert.equal(walls.length, 1, `${label}: one retained blend wall`);
        for (const boundary of kernel.getFaceEdges(walls[0])) {
          assert.equal(faces.filter(face => Array.from(kernel.getFaceEdges(face)).includes(boundary)).length, 2,
            `${label}: shared wall/cap/support boundary`);
          const [start, end] = kernel.getEdgeParamSpan(boundary);
          for (let sample = 0; sample <= 16; sample++) {
            const p = kernel.evaluateEdgeCurve(boundary, start + (end - start) * sample / 16);
            const local = placed ? [p[1] + 19 * scale, p[2] - 11 * scale, p[0] - 37 * scale] : p;
            close(Math.hypot(local[0] - radius, local[1] - radius), radius, 1e-7, `${label}: native wall boundary`);
          }
        }
        if (detailed) {
          const current = kernel.serializeSolids(Uint32Array.of(solid));
          const refused = invoke(kernel, route, solid, specs, true);
          assert.equal(refused.status, 'error', label);
          assert.equal(refused.code, 'exact_only_unattainable', label);
          assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), current, `${label}: exact-only rollback`);
        }
        cells++;
      } finally { kernel.free(); }
    }
  }
  for (const route of routes) unprovedBoundaryRefusal(BrepKernel, route);
  return { cells, unprovedRefusals: routes.length };
}
