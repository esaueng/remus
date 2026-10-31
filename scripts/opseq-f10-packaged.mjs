/** F10: independent hemisphere/cylinder union contracts on every JS surface. */
import assert from 'node:assert/strict';

const sub = (a, b) => a.map((v, i) => v - b[i]);
const dot = (a, b) => a.reduce((sum, v, i) => sum + v * b[i], 0);
const state = kernel => {
  const usage = JSON.parse(kernel.checkpointMemoryStats());
  return { liveSlots: usage.allocatedSlots - usage.retiredSlots, journal: kernel.journalSummary() };
};
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function matrix(scale, placed) {
  const a = placed ? 0.37 : 0, b = placed ? 0.23 : 0;
  const c = Math.cos(a), s = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b);
  return [cb, -sb, 0, placed ? 10 * scale : 0, c * sb, c * cb, -s, placed ? -7 * scale : 0,
    s * sb, s * cb, c, placed ? 3 * scale : 0, 0, 0, 0, 1];
}
const world = (m, p) => [0, 1, 2].map(r => m[4 * r] * p[0] + m[4 * r + 1] * p[1] + m[4 * r + 2] * p[2] + m[4 * r + 3]);
const local = (m, p, scale) => [0, 1, 2].map(c => (m[c] * (p[0] - m[3])
  + m[c + 4] * (p[1] - m[7]) + m[c + 8] * (p[2] - m[11])) / scale);
function invoke(kernel, api, op, args, direct, label) {
  if (api === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
  const cell = JSON.parse(kernel[api](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(cell, 'ok'), `${label}: ${op}: ${JSON.stringify(cell)}`);
  return cell.ok;
}

function checkFixture(RemusKernel, api, scale, placed, swapped, segments = 16, height = 1, fineMesh = false) {
  const label = `F10 ${api} scale=${scale} placed=${placed} swapped=${swapped} segments=${segments} h=${height} fine=${fineMesh}`;
  const kernel = new RemusKernel();
  const run = (op, args, direct) => invoke(kernel, api, op, args, direct, label);
  try {
    const sphere = run('makeSphere', { radius: scale, segments }, [scale, segments]);
    const cylinder = run('makeCylinder', { radius: scale, height: height * scale }, [scale, height * scale]);
    const m = matrix(scale, placed);
    if (placed) for (const solid of [sphere, cylinder]) run('transform', { solid, matrix: m }, [solid, Float64Array.from(m)]);
    const ids = Uint32Array.of(sphere, cylinder), original = kernel.serializeSolids(ids);
    const solidA = swapped ? sphere : cylinder, solidB = swapped ? cylinder : sphere;
    if (scale === 1000 && placed) {
      const before = state(kernel);
      const message = 'exact-only policy: the exact boolean pipeline could not produce this result and the approximate fallback was declined';
      if (api === 'direct') {
        assert.throws(() => kernel.booleanWithQuality('fuse', solidA, solidB, true),
          error => !(error instanceof WebAssembly.RuntimeError) && error.message === message, label);
      } else {
        const row = JSON.parse(kernel[api](JSON.stringify([{ op: 'booleanWithQuality',
          args: { operation: 'fuse', solidA, solidB, exactOnly: true } }])))[0];
        if (api === 'executeBatch') assert.deepEqual(row, { error: message }, label);
        else {
          assert.equal(row.error.code, 'operation_failed', label);
          assert.equal(row.error.category, 'quality_refused', label);
          assert.equal(row.error.details.kernelCode, 'exact_only_unattainable', label);
          assert.equal(row.error.message, message, label);
        }
      }
      assert.deepEqual(state(kernel), before, `${label}: refusal rollback`);
      assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: unchanged refused operands`);
      console.log(`ok - ${label}: exact refusal and preserved state`);
      return;
    }
    const outcome = run('booleanWithQuality', { operation: 'fuse', solidA, solidB, exactOnly: true }, ['fuse', solidA, solidB, true]);
    assert.equal(outcome.quality, 'exact', `${label}: exact quality`);
    const solid = outcome.solid;
    const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
    const root = doc.solids[doc.solid_roots[0]];
    const faces = [root.outer_shell, ...root.inner_shells].flatMap(shell => doc.shells[shell].faces);
    assert.equal(faces.length, 3, `${label}: analytic face census`);
    const kinds = faces.map(id => Object.keys(doc.faces[id].surface)[0]).sort();
    assert.deepEqual(kinds, ['Cylinder', 'Plane', 'Sphere'], `${label}: analytic carriers`);
    const uses = new Map();
    for (const id of faces) {
      const face = doc.faces[id];
      for (const wi of [face.outer_wire, ...face.inner_wires]) {
        for (const use of doc.wires[wi].edges) {
          const directions = uses.get(use.edge) ?? [];
          directions.push(use.forward !== face.reversed);
          uses.set(use.edge, directions);
        }
      }
    }
    for (const [edge, directions] of uses) {
      assert.equal(directions.length, 2, `${label}: B-Rep incidence ${edge}`);
      assert.notEqual(directions[0], directions[1], `${label}: B-Rep orientation ${edge}`);
    }
    const validation = JSON.parse(kernel.validateSolidDetailed(solid));
    assert.equal(validation.errorCount, 0, `${label}: strict validation ${JSON.stringify(validation)}`);
    for (const deflection of fineMesh ? [0.00002, 0.00001] : [0.1, 0.01]) {
      const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection * scale, 0.35));
      const point = i => local(m, mesh.positions.slice(3 * i, 3 * i + 3), scale);
      const meshUses = fineMesh ? null : new Map();
      // Fine grids have millions of edges. Sorting exact integer encodings
      // keeps the independent raw-ID incidence oracle bounded in JS memory.
      // No position weld, tolerance or geometric clustering enters this key.
      const vertexCount = mesh.positions.length / 3;
      assert.ok(2 * vertexCount ** 2 < Number.MAX_SAFE_INTEGER, `${label}: exact edge keys`);
      const edgeUses = fineMesh ? new Float64Array(mesh.indices.length) : null;
      let nextEdge = 0;
      assert.equal(mesh.faceOffsets.length, faces.length + 1, `${label}: face grouping`);
      for (let fi = 0; fi < faces.length; fi++) {
        assert.ok(mesh.faceOffsets[fi + 1] > mesh.faceOffsets[fi], `${label}: empty face`);
        const sphereFace = Object.hasOwn(doc.faces[faces[fi]].surface, 'Sphere');
        for (let at = mesh.faceOffsets[fi]; at < mesh.faceOffsets[fi + 1]; at += 3) {
          const indices = mesh.indices.slice(at, at + 3), [a, b, c] = indices.map(point);
          const normal = cross(sub(b, a), sub(c, a));
          assert.ok(Math.hypot(...normal) > 1e-15, `${label}: degenerate triangle`);
          if (sphereFace) {
            assert.ok(dot(normal, a.map((v, i) => (v + b[i] + c[i]) / 3)) > 0, `${label}: inward spherical triangle`);
            for (const p of [a, b, c]) {
              assert.ok(p[2] <= 1e-6, `${label}: upper hemisphere on exterior`);
              assert.ok(Math.abs(Math.hypot(...p) - 1) < 1e-5, `${label}: off-sphere vertex`);
            }
          }
          for (const [a, b] of [[indices[0], indices[1]], [indices[1], indices[2]], [indices[2], indices[0]]]) {
            if (edgeUses) edgeUses[nextEdge++] = 2 * (Math.min(a, b) * vertexCount + Math.max(a, b)) + Number(a < b);
            else {
              const key = a < b ? `${a},${b}` : `${b},${a}`, directions = meshUses.get(key) ?? [];
              directions.push(a < b); meshUses.set(key, directions);
            }
          }
        }
      }
      if (edgeUses) {
        assert.equal(nextEdge, edgeUses.length, `${label}: all edges checked`);
        edgeUses.sort();
        assert.equal(edgeUses.length % 2, 0, `${label}: paired edge uses`);
        for (let i = 0; i < edgeUses.length; i += 2) {
          assert.equal(edgeUses[i] % 2, 0, `${label}: first edge direction ${i}`);
          assert.equal(edgeUses[i + 1], edgeUses[i] + 1, `${label}: opposing raw edge pair ${i}`);
          if (i > 0) assert.ok(edgeUses[i] > edgeUses[i - 1], `${label}: exactly two uses ${i}`);
        }
      } else {
        for (const [edge, directions] of meshUses) {
          assert.equal(directions.length, 2, `${label}: raw mesh incidence ${edge}`);
          assert.notEqual(directions[0], directions[1], `${label}: raw mesh winding ${edge}`);
        }
      }
    }
    const coverage = [0, 0, 0, 0];
    for (const x of [-1.2, -0.8, -0.4, 0.2, 0.6, 1.2]) {
      for (const y of [-0.8, -0.3, 0.2, 0.7]) {
        for (const z of [-1.2, -0.8, -0.3, 0.3, 0.7, 1.2, 1.7]) {
          const radial = Math.hypot(x, y), radius = Math.hypot(radial, z);
          if (Math.abs(radius - 1) < 0.06 || Math.abs(radial - 1) < 0.06 || Math.abs(z - height) < 0.06) continue;
          const ball = radius < 1, tube = radial < 1 && z > 0 && z < height;
          coverage[2 * Number(ball) + Number(tube)]++;
          const p = world(m, [x * scale, y * scale, z * scale]);
          for (const [name, id, inside] of [['sphere', sphere, ball], ['cylinder', cylinder, tube], ['union', solid, ball || tube]]) {
            assert.equal(run('classifyPoint', { solid: id, x: p[0], y: p[1], z: p[2], tolerance: 1e-7 }, [id, ...p, 1e-7]),
              inside ? 'inside' : 'outside', `${label}: ${name} at ${x},${y},${z}`);
          }
        }
      }
    }
    assert.ok(coverage.every(n => n > 0), `${label}: all membership cells`);
    const expectedVolume = Math.PI * (height + 2 / 3), expectedArea = Math.PI * (2 * height + 3);
    // The 32-segment fixture's fragmented cylinder rims integrate over their
    // exact band rectangle instead of the 4096-point trim budget.
    const gauss = JSON.parse(kernel.massProperties(solid)).volume / scale ** 3;
    assert.ok(Math.abs(gauss - expectedVolume) < 1e-9 * expectedVolume, `${label}: Gauss ${gauss}`);
    for (const d of [0.0001, 0.00005]) {
      const volume = kernel.volume(solid, d * scale) / scale ** 3, area = kernel.surfaceArea(solid, d * scale) / scale ** 2;
      assert.ok(Math.abs(volume - expectedVolume) < 1e-4 * expectedVolume, `${label}: volume ${volume}`);
      assert.ok(Math.abs(area - expectedArea) < 1e-4 * expectedArea, `${label}: area ${area}`);
    }
    assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: unchanged operands`);
  } finally { kernel.free(); }
  console.log(`ok - ${label}: material, quantities, opposing B-Rep and raw mesh uses`);
}

export function runOpseqF10Packaged({ RemusKernel }) {
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [1, 0.001, 1000]) {
      for (const placed of [false, true]) {
        for (const swapped of [false, true]) checkFixture(RemusKernel, api, scale, placed, swapped);
      }
    }
    for (const segments of [4, 8, 32]) checkFixture(RemusKernel, api, 1, false, false, segments);
    checkFixture(RemusKernel, api, 1, false, false, 16, 1.25);
  }
  runOpseqF10FineGridPackaged({ RemusKernel });
}

export function runOpseqF10FineGridPackaged({ RemusKernel }) {
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    checkFixture(RemusKernel, api, 0.001, true, false, 16, 1, true);
  }
}
