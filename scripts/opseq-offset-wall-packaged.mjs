/** F9/F14/F15 raw mesh and material contracts through packaged WASM. */
import assert from 'node:assert/strict';

const sub = (a, b) => a.map((v, i) => v - b[i]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const point = (m, p) => [0, 1, 2].map(r => m[r * 4] * p[0]
  + (m[r * 4 + 1] * p[1] + (m[r * 4 + 2] * p[2] + m[r * 4 + 3])));
function placement(scale, placed) {
  if (!placed) return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const c = Math.cos(0.7), s = Math.sin(0.7), cx = Math.cos(0.3), sx = Math.sin(0.3);
  return [c, -s * cx, s * sx, 11 * scale, s, c * cx, -c * sx, -7 * scale,
    0, sx, cx, 5 * scale, 0, 0, 0, 1];
}
const localPoint = (m, p, scale) => [0, 1, 2].map(c => (m[c] * (p[0] - m[3])
  + m[c + 4] * (p[1] - m[7]) + m[c + 8] * (p[2] - m[11])) / scale);

function invoke(kernel, api, op, args, direct, label) {
  if (api === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
  const cell = JSON.parse(kernel[api](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(cell, 'ok'), `${label}: ${op}: ${JSON.stringify(cell)}`);
  return cell.ok;
}

export function runOpseqOffsetWallPackaged({ RemusKernel }) {
  let passed = 0;
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const offsetOp of ['offsetSolid', 'offsetSolidV2']) {
      for (const [r0, r1, h] of [[1, 1, 1], [4, 2, 3], [0, 1, 1], [1, 0, 1]]) {
        for (const d of [-0.25, 0.25]) {
          const slope = (r1 - r0) / h, shift = d * Math.hypot(slope, 1);
          const lo = r0 === 0 ? -shift / slope : -d;
          const hi = r1 === 0 ? h - shift / slope : h + d;
          const ra = Math.max(0, r0 + slope * lo + shift), rb = Math.max(0, r0 + slope * hi + shift);
          const volume = Math.PI * (hi - lo) * (ra * ra + ra * rb + rb * rb) / 3;
          for (const scale of [1, 1000]) {
            for (const placed of [false, true]) {
              const label = `offset wall ${api}/${offsetOp} radii=${r0}/${r1} h=${h} d=${d} scale=${scale} placed=${placed}`;
              const kernel = new RemusKernel();
              const run = (op, args, direct) => invoke(kernel, api, op, args, direct, label);
              try {
                const source = r0 === r1
                  ? run('makeCylinder', { radius: r0 * scale, height: h * scale }, [r0 * scale, h * scale])
                  : run('makeCone', { bottomRadius: r0 * scale, topRadius: r1 * scale, height: h * scale }, [r0 * scale, r1 * scale, h * scale]);
                const matrix = placement(scale, placed);
                if (placed) run('transform', { solid: source, matrix }, [source, Float64Array.from(matrix)]);
                const ids = Uint32Array.of(source), original = kernel.serializeSolids(ids);
                const solid = run(offsetOp, { solid: source, distance: d * scale }, [source, d * scale]);
                assert.deepEqual(kernel.serializeSolids(ids), original, `${label}: source changed`);
                assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, label);
                const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
                assert.equal(doc.faces.length, r0 === 0 || r1 === 0 ? 2 : 3, `${label}: analytic census`);
                const root = doc.solids[doc.solid_roots[0]], faceOrder = doc.shells[root.outer_shell].faces;
                const curvedKind = r0 === r1 ? 'Cylinder' : 'Cone';
                assert.equal(doc.faces.filter(face => Object.hasOwn(face.surface, curvedKind)).length, 1, `${label}: lost analytic wall`);
                for (const face of doc.faces) {
                  assert.equal(face.reversed, false, `${label}: inverted face`);
                  assert.ok(Object.hasOwn(face.surface, 'Plane') || Object.hasOwn(face.surface, curvedKind), `${label}: unexpected carrier`);
                }
                for (const deflection of [0.01, 0.001]) {
                  const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection * scale, 0.35));
                  const points = Array.from({ length: mesh.positions.length / 3 }, (_, i) =>
                    localPoint(matrix, mesh.positions.slice(3 * i, 3 * i + 3), scale));
                  const uses = new Map();
                  let signed = 0;
                  for (let fi = 0; fi < faceOrder.length; fi++) {
                    assert.ok(mesh.faceOffsets[fi + 1] > mesh.faceOffsets[fi], `${label}: empty face`);
                    const planar = Object.hasOwn(doc.faces[faceOrder[fi]].surface, 'Plane');
                    for (let i = mesh.faceOffsets[fi]; i < mesh.faceOffsets[fi + 1]; i += 3) {
                      const indices = mesh.indices.slice(i, i + 3), [a, b, c] = indices.map(j => points[j]);
                      const center = a.map((v, j) => (v + b[j] + c[j]) / 3);
                      const radius = Math.hypot(center[0], center[1]);
                      const outward = planar ? [0, 0, center[2] < (lo + hi) / 2 ? -1 : 1]
                        : [center[0] / radius, center[1] / radius, -slope];
                      assert.ok(dot(cross(sub(b, a), sub(c, a)), outward) > 0, `${label}: inward triangle`);
                      signed += dot(sub(a, points[0]), cross(sub(b, points[0]), sub(c, points[0]))) / 6;
                      for (const [u, v] of [[indices[0], indices[1]], [indices[1], indices[2]], [indices[2], indices[0]]]) {
                        const key = u < v ? `${u},${v}` : `${v},${u}`, directions = uses.get(key) ?? [];
                        directions.push(u < v); uses.set(key, directions);
                      }
                    }
                  }
                  for (const directions of uses.values()) {
                    assert.equal(directions.length, 2, `${label}: raw mesh boundary/non-manifold`);
                    assert.notEqual(directions[0], directions[1], `${label}: same-sense raw mesh`);
                  }
                  assert.ok(Math.abs(signed - volume) < 0.04 * volume, `${label}: signed mesh volume ${signed} != ${volume}`);
                  const measured = kernel.volume(solid, deflection * scale) / scale ** 3;
                  assert.ok(Math.abs(measured - volume) < 1e-6 * volume, `${label}: volume ${measured} != ${volume}`);
                }
                for (const f of [0.2, 0.5, 0.8]) {
                  const z = lo + f * (hi - lo), r = ra + f * (rb - ra);
                  for (const factor of [0.25, 1.3]) {
                    const p = point(matrix, [factor * r * scale, 0.13 * r * scale, z * scale]);
                    assert.equal(run('classifyPoint', { solid, x: p[0], y: p[1], z: p[2], tolerance: 1e-7 * scale },
                      [solid, ...p, 1e-7 * scale]), factor < 1 ? 'inside' : 'outside', `${label}: material probe ${f}/${factor}`);
                  }
                }
                passed++;
              } finally { kernel.free(); }
            }
          }
        }
      }
    }
  }
  assert.equal(passed, 192);
  console.log('ok - F9/F14/F15 offset walls: raw outward meshes, material and volumes across 192 packaged entry/API/scale/placement cells');
}
