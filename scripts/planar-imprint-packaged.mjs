/** Bounded planar polygon imprint through the shipped live-session API. */
import assert from 'node:assert/strict';

function invoke(k, route, solid, face, points) {
  if (route === 'direct') return JSON.parse(k.imprintPlanarPolygon(solid, face, Float64Array.from(points)));
  const [row] = JSON.parse(k[route](JSON.stringify([{ op: 'imprintPlanarPolygon', args: { solid, face, points } }])));
  assert.ok(row.ok, JSON.stringify(row)); return row.ok;
}
function refuse(k, route, solid, face, points, code) {
  if (route === 'direct') {
    assert.throws(() => k.imprintPlanarPolygon(solid, face, Float64Array.from(points)), error => {
      if (code === 'unsupported_imprint') assert.match(error.message, /unsupported imprint:/);
      return true;
    }); return;
  }
  const [row] = JSON.parse(k[route](JSON.stringify([{ op: 'imprintPlanarPolygon', args: { solid, face, points } }])));
  assert.ok(row.error, JSON.stringify(row));
  if (route === 'executeBatchV2' && code) assert.equal(row.error.details.kernelCode, code);
  if (route === 'executeBatch' && code === 'unsupported_imprint') assert.match(row.error, /unsupported imprint:/);
}
function reference(k, op, kind, handle) {
  for (let i = 0; i < 64; i++) {
    const r = JSON.parse(k.resolveOperationOutput(op, kind, i));
    if (r.status === 'bound' && r.entities[0].handle === handle) return k.makeOperationOutputRef(op, kind, i);
  }
  assert.fail(`missing ${kind} construction output ${handle}`);
}
function close(actual, expected, label) {
  assert.ok(Math.abs(actual - expected) < Math.max(Math.abs(expected), 1) * 1e-8, `${label}: ${actual} != ${expected}`);
}
// A separate welded directed-incidence and body-relative mesh-volume oracle.
function mesh(k, solid, scale, expected) {
  const m = JSON.parse(k.tessellateSolidGrouped(solid, scale * 1e-4, 0.1));
  const vertices = new Map(), remap = [], uses = new Map();
  assert.ok(m.positions.every(Number.isFinite));
  for (let i = 0; i < m.positions.length; i += 3) {
    const key = m.positions.slice(i, i + 3).map(v => Math.round(v / scale * 1e8)).join(',');
    if (!vertices.has(key)) vertices.set(key, vertices.size);
    remap.push(vertices.get(key));
  }
  const anchor = m.positions.slice(0, 3);
  let volume = 0;
  for (let i = 0; i < m.indices.length; i += 3) {
    const t = m.indices.slice(i, i + 3), welded = t.map(index => remap[index]);
    for (let j = 0; j < 3; j++) {
      const a = welded[j], b = welded[(j + 1) % 3]; assert.notEqual(a, b);
      const key = `${Math.min(a, b)},${Math.max(a, b)}`, entry = uses.get(key) ?? [0, 0];
      entry[0]++; entry[1] += a < b ? 1 : -1; uses.set(key, entry);
    }
    const [a, b, c] = t.map(index => m.positions.slice(index * 3, index * 3 + 3).map((v, axis) => (v - anchor[axis]) / scale));
    volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
  }
  for (const value of uses.values()) assert.deepEqual(value, [2, 0]);
  close(volume, expected, 'independent mesh volume');
}
export function runPlanarImprintPackaged({ BrepKernel }) {
  assert.equal(typeof BrepKernel.prototype.imprintPlanarPolygon, 'function');
  let cells = 0;
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [0.001, 1, 1000]) {
      for (const placed of [false, true]) for (const reversed of [false, true]) {
        const k = new BrepKernel();
        try {
          // Journal the initial box through a face move before capturing refs.
          const initial = k.makeBox(10 * scale, 8 * scale, 3 * scale);
          const top = [...k.getSolidFaces(initial)].find(f => k.getFaceNormal(f)[2] > 0.99);
          const angle = placed ? 0.43 : 0, c = Math.cos(angle), s = Math.sin(angle);
          const origin = placed ? [13, -7, 4] : [0, 0, 0];
          const transform = ([x, y, z]) => [origin[0] + c * x - s * y, origin[1] + s * x + c * y, origin[2] + z];
          if (placed) k.transformSolid(initial, [c, -s, 0, origin[0], s, c, 0, origin[1], 0, 0, 1, origin[2], 0, 0, 0, 1]);
          const seeded = JSON.parse(k.moveFacesJournaled(initial, Uint32Array.of(top), scale));
          const face = [...k.getSolidFaces(seeded.solid)].find(f => k.getFaceNormal(f)[2] > 0.99);
          k.setFaceName(face, 'partition support');
          const oldFace = reference(k, seeded.op, 'face', face);
          const edge = k.getSolidEdges(seeded.solid)[0], oldEdge = reference(k, seeded.op, 'edge', edge);
          const before = k.checkpoint(), history = k.journalSummary();
          const points = [[3, 2, 4], [7, 2, 4], [7, 6, 4], [3, 6, 4]].map(p => transform(p.map(v => v * scale)));
          const boundary = [[0, 2, 4], [7, 2, 4], [7, 6, 4], [0, 6, 4]].map(p => transform(p.map(v => v * scale)));
          if (reversed) { points.reverse(); boundary.reverse(); }
          const result = invoke(k, route, seeded.solid, face, points.flat());
          assert.equal(k.getSolidFaces(result.solid).length, 7);
          assert.equal(k.getSolidEdges(result.solid).length, 16);
          assert.equal(k.getSolidVertices(result.solid).length, 12);
          for (const [patch, area, holes] of [[result.innerFace, 16, 0], [result.surroundFace, 64, 1]]) {
            assert.equal(k.getFaceName(patch), 'partition support');
            assert.equal(k.getSurfaceType(patch), 'plane');
            assert.equal(k.getFaceWires(patch).length - 1, holes);
            close(k.faceArea(patch, scale * 1e-4) / scale ** 2, area, 'patch area');
          }
          const resolved = JSON.parse(k.resolveRef(oldFace));
          assert.equal(resolved.status, 'boundMany'); assert.equal(resolved.provenance, 'construction');
          assert.deepEqual(new Set(resolved.entities.map(e => e.handle)), new Set([result.innerFace, result.surroundFace]));
          const preserved = JSON.parse(k.resolveRef(oldEdge)); assert.equal(preserved.provenance, 'construction');
          assert.equal(preserved.entities[0].handle, edge);
          assert.equal(k.validateSolid(result.solid), 0);
          for (const deflection of [1e-4, 1e-5]) close(k.volume(result.solid, scale * deflection) / scale ** 3, 320, 'volume');
          for (const [x, z, expected] of [[2, 3, 'inside'], [5, 3, 'inside'], [7, 3, 'inside'], [5, 5, 'outside']]) {
            assert.equal(k.classifyPoint(result.solid, ...transform([x * scale, 4 * scale, z * scale]), scale * 1e-7), expected);
          }
          mesh(k, result.solid, scale, 320);
          k.checkpoint(); // Share the result owner while every refusal is tested.
          const state = () => {
            // Finish read-only serialization allocations before sampling the
            // global WASM high-water mark. The observer is not an operation.
            const source = [...k.serializeSolids(Uint32Array.of(seeded.solid, result.solid))];
            const history = k.journalSummary(), faces = [...k.getSolidFaces(result.solid)];
            return { source, history, faces, memory: JSON.parse(k.checkpointMemoryStats()) };
          };
          for (const [target, malformed, code] of [[result.surroundFace, points.flat(), 'unsupported_imprint'], [face, [], null], [face, Array(51).fill(0), null], [face, boundary.flat(), 'unsupported_imprint']]) {
            const snapshot = state(); assert.deepEqual(state(), snapshot, 'read-only snapshot control'); refuse(k, route, target === result.surroundFace ? result.solid : seeded.solid, target, malformed, code); assert.deepEqual(state(), snapshot);
          }
          k.restore(before); assert.equal(k.journalSummary(), history);
          assert.equal(JSON.parse(k.resolveRef(oldFace)).status, 'bound');
          assert.throws(() => k.getSolidFaces(result.solid));
          const replay = invoke(k, route, seeded.solid, face, points.flat());
          assert.notEqual(replay.solid, result.solid); assert.notEqual(replay.innerFace, result.innerFace); assert.notEqual(replay.op, result.op);
          cells++;
        } finally { k.free(); }
      }
    }
    // A real boolean cavity: the untouched inner shell keeps its exact handle.
    const k = new BrepKernel();
    try {
      const outer = k.makeBox(10, 8, 4), inner = k.makeBox(2, 2, 2);
      k.transformSolid(inner, [1, 0, 0, 4, 0, 1, 0, 3, 0, 0, 1, 1, 0, 0, 0, 1]);
      const solid = k.cut(outer, inner), shells = [...k.getSolidShells(solid)];
      assert.equal(shells.length, 2);
      const face = [...k.getShellFaces(shells[0])].find(f => k.getFaceNormal(f)[2] > 0.99);
      const result = invoke(k, route, solid, face, [3, 2, 4, 7, 2, 4, 7, 6, 4, 3, 6, 4]);
      assert.equal(k.getSolidShells(result.solid)[1], shells[1]);
      close(k.volume(result.solid, 1e-4), 312, 'cavity volume'); mesh(k, result.solid, 1, 312);
      assert.equal(k.classifyPoint(result.solid, 5, 4, 2, 1e-7), 'outside');
      assert.equal(k.classifyPoint(result.solid, 2, 4, 2, 1e-7), 'inside');
      cells++;
    } finally { k.free(); }
  }
  console.log(`ok - planar polygon imprint: ${cells} direct/batch cells, construction naming, cavity, rollback and undo`);
}
