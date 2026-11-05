/** One closed off-axis sphere/cylinder section shared across a cylinder seam. */
import assert from 'node:assert/strict';

const translation = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];

function assertClosedMeshes(kernel, solid, label) {
  for (const deflection of [0.1, 0.01]) {
    const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection, 0.35));
    assert.ok(mesh.indices.length > 0, `${label}: nonempty mesh`);
    assert.equal(mesh.indices.length % 3, 0, label);
    // Count raw vertex IDs and opposite triangle uses, without welding.
    const edges = new Map();
    for (let i = 0; i < mesh.indices.length; i += 3) {
      const triangle = mesh.indices.slice(i, i + 3);
      for (let j = 0; j < 3; j++) {
        const a = triangle[j], b = triangle[(j + 1) % 3];
        assert.notEqual(a, b, `${label}: degenerate mesh edge`);
        const key = `${Math.min(a, b)},${Math.max(a, b)}`;
        const use = edges.get(key) ?? [0, 0];
        use[0]++;
        use[1] += a < b ? 1 : -1;
        edges.set(key, use);
      }
    }
    for (const [edge, use] of edges) {
      assert.deepEqual(use, [2, 0], `${label}: mesh edge ${edge} at ${deflection}`);
    }
  }
}

export function runSphereCylinderClosedSectionPackaged({ RemusKernel, RemusIo }) {
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const swapped of [false, true]) {
      const label = `closed sphere/cylinder section ${api} swapped=${swapped}`;
      const kernel = new RemusKernel();
      const io = new RemusIo();
      const run = (op, args, direct) => {
        if (api === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
        const row = JSON.parse(kernel[api](JSON.stringify([{ op, args }])))[0];
        assert.ok(Object.hasOwn(row, 'ok'), `${label}: ${op}: ${JSON.stringify(row)}`);
        return row.ok;
      };
      try {
        const sphere = run('makeSphere', { radius: 73.5, segments: 32 }, [73.5, 32]);
        const cylinder = run('makeCylinder', { radius: 6, height: 64.10000000000001 }, [6, 64.10000000000001]);
        for (const [solid, matrix] of [[sphere, translation(25.5, -1.5, 0)], [cylinder, translation(0, 0, 58)]]) {
          run('transform', { solid, matrix }, [solid, Float64Array.from(matrix)]);
        }
        const inputs = Uint32Array.of(sphere, cylinder);
        const original = kernel.serializeSolids(inputs);
        const [solidA, solidB] = swapped ? [cylinder, sphere] : [sphere, cylinder];
        const result = run('booleanWithQuality', { operation: 'fuse', solidA, solidB, exactOnly: true }, ['fuse', solidA, solidB, true]);
        assert.equal(result.quality, 'exact', label);
        const solid = result.solid;
        assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, label);
        const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
        const root = doc.solids[doc.solid_roots[0]];
        assert.equal(root.inner_shells.length, 0, label);
        const faces = doc.shells[root.outer_shell].faces;
        assert.deepEqual(faces.map(id => Object.keys(doc.faces[id].surface)[0]).sort(), ['Cylinder', 'Plane', 'Sphere', 'Sphere'], label);
        const uses = new Map();
        for (const id of faces) {
          const face = doc.faces[id];
          for (const wi of [face.outer_wire, ...face.inner_wires]) {
            for (const edge of doc.wires[wi].edges) {
              const directions = uses.get(edge.edge) ?? [];
              directions.push(edge.forward !== face.reversed);
              uses.set(edge.edge, directions);
            }
          }
        }
        for (const [edge, directions] of uses) {
          assert.equal(directions.length, 2, `${label}: edge ${edge} incidence`);
          assert.notEqual(directions[0], directions[1], `${label}: edge ${edge} orientation`);
        }
        assertClosedMeshes(kernel, solid, label);
        // Independent sphere-height integration over the cylinder disk.
        const sphereVolume = 4 * Math.PI * 73.5 ** 3 / 3;
        assert.ok(Math.abs(kernel.volume(solid, 0.01) - sphereVolume - 6030.504746075203) < 0.05, `${label}: added material`);
        for (const [x, y, z, expected] of [[0, 0, 60, 'inside'], [0, 0, 100, 'inside'], [25.5, -1.5, 0, 'inside'], [7, 0, 100, 'outside'], [0, 0, 125, 'outside']]) {
          assert.equal(kernel.classifyPoint(solid, x, y, z, 1e-7), expected, label);
        }
        const step = io.exportStep(kernel.serializeSolids(Uint32Array.of(solid)));
        const restored = kernel.deserializeSolids(io.importStep(step));
        assert.equal(restored.length, 1, `${label}: STEP root count`);
        const copy = restored[0];
        assert.equal(JSON.parse(kernel.validateSolidDetailed(copy)).errorCount, 0, `${label}: STEP validity`);
        assert.ok(Math.abs(kernel.volume(copy, 0.01) - sphereVolume - 6030.504746075203) < 0.05, `${label}: STEP added material`);
        const roundTrip = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(copy))));
        const rootAfter = roundTrip.solids[roundTrip.solid_roots[0]];
        assert.deepEqual(roundTrip.shells[rootAfter.outer_shell].faces.map(id => Object.keys(roundTrip.faces[id].surface)[0]).sort(), ['Cylinder', 'Plane', 'Sphere', 'Sphere'], `${label}: STEP analytic carriers`);
        assertClosedMeshes(kernel, copy, `${label}: STEP`);
        assert.deepEqual(kernel.serializeSolids(inputs), original, `${label}: unchanged inputs`);
      } finally { io.free(); kernel.free(); }
      console.log(`ok - ${label}: exact, closed B-Rep/meshes, STEP round trip and material preserved`);
    }
  }
}
