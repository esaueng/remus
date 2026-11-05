/**
 * opseq81 seed 1146645986253340484: a translated torus fused exact-only with
 * a sphere leaves two section holes in one hemisphere. The packaged kernel
 * must keep both holes open in its mesh (the native fixture is
 * `crates/operations/tests/regress_opseq_torus_sphere_fuse.rs`).
 *
 * Every cell runs the seed's own construction through one public path
 * (direct methods, `executeBatch`, `executeBatchV2`), then checks it
 * independently of the kernel's own measures:
 * - the B-Rep: exact quality, strict validity, carriers, opposite edge uses,
 *   unchanged operands;
 * - material: Cartesian ball/torus membership through `classifyPoint`;
 * - volume: an inclusion-exclusion lens-slice integral;
 * - meshes at two deflections: raw index closure and orientation, the same
 *   after welding coincident positions, outward sphere facets, no sphere
 *   facet inside the torus, and the mesh's own signed volume.
 */
import assert from 'node:assert/strict';

const [MAJOR, MINOR, BALL, TORUS_X, TORUS_Z] = [4, 2, 8, -7, -3];

const json = value => (typeof value === 'string' ? JSON.parse(value) : value);
const multiply = (a, b) => Array.from({ length: 16 }, (_, k) => {
  const [row, col] = [Math.floor(k / 4), k % 4];
  return [0, 1, 2, 3].reduce((sum, i) => sum + a[4 * row + i] * b[4 * i + col], 0);
});
const translation = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
const rotationX = t => [1, 0, 0, 0, 0, Math.cos(t), -Math.sin(t), 0, 0, Math.sin(t), Math.cos(t), 0, 0, 0, 0, 1];
const rotationZ = t => [Math.cos(t), -Math.sin(t), 0, 0, Math.sin(t), Math.cos(t), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
// The same rigid placement as the native fixture.
const placement = (scale, placed) => (placed
  ? multiply(multiply(translation(10 * scale, -7 * scale, 3 * scale), rotationX(0.37)), rotationZ(0.23))
  : translation(0, 0, 0));
const apply = (m, p) => [0, 1, 2].map(r => m[4 * r] * p[0] + m[4 * r + 1] * p[1] + m[4 * r + 2] * p[2] + m[4 * r + 3]);
// Rigid inverse: transpose the rotation block, rotate back the translation.
const toLocal = (m, p, scale) => {
  const d = [p[0] - m[3], p[1] - m[7], p[2] - m[11]];
  return [0, 1, 2].map(c => (m[c] * d[0] + m[4 + c] * d[1] + m[8 + c] * d[2]) / scale);
};

const torusDistance = ([x, y, z]) => Math.hypot(Math.hypot(x - TORUS_X, y) - MAJOR, z - TORUS_Z) - MINOR;
const ballDistance = p => Math.hypot(...p) - BALL;

function lens(a, b, d) {
  if (d >= a + b) return 0;
  if (d <= Math.abs(a - b)) return Math.PI * Math.min(a, b) ** 2;
  const ca = Math.min(1, Math.max(-1, (d * d + a * a - b * b) / (2 * d * a)));
  const cb = Math.min(1, Math.max(-1, (d * d + b * b - a * a) / (2 * d * b)));
  return a * a * Math.acos(ca) + b * b * Math.acos(cb)
    - 0.5 * Math.sqrt((-d + a + b) * (d + a - b) * (d - a + b) * (d + a + b));
}

/** Union volume: ball + torus - (torus ∩ ball), the overlap by z-slices. */
function expectedVolume() {
  const n = 20000;
  let overlap = 0;
  for (let i = 0; i < n; i += 1) {
    const t = (i + 0.5) * Math.PI / n;
    const z = TORUS_Z - MINOR * Math.cos(t);
    const h = Math.sqrt(Math.max(0, MINOR * MINOR - (z - TORUS_Z) ** 2));
    const s = Math.sqrt(Math.max(0, BALL * BALL - z * z));
    overlap += (lens(MAJOR + h, s, -TORUS_X) - lens(MAJOR - h, s, -TORUS_X)) * MINOR * Math.sin(t) * Math.PI / n;
  }
  return 4 / 3 * Math.PI * BALL ** 3 + 2 * Math.PI ** 2 * MAJOR * MINOR ** 2 - overlap;
}

function assertClosedOriented(indices, ids, label) {
  const uses = new Map();
  for (let at = 0; at < indices.length; at += 3) {
    const t = [indices[at], indices[at + 1], indices[at + 2]].map(i => ids(i));
    assert.ok(t[0] !== t[1] && t[1] !== t[2] && t[0] !== t[2], `${label}: collapsed triangle`);
    for (let i = 0; i < 3; i += 1) {
      const [a, b] = [t[i], t[(i + 1) % 3]];
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      const use = uses.get(key) ?? [0, 0];
      use[0] += 1;
      use[1] += a < b ? 1 : -1;
      uses.set(key, use);
    }
  }
  for (const [edge, use] of uses) {
    assert.deepEqual(use, [2, 0], `${label}: edge ${edge} incidence/orientation`);
  }
}

function checkMesh(kernel, solid, deflection, scale, matrix, expected, label) {
  const mesh = json(kernel.tessellateSolidGrouped(solid, deflection * scale, 0.35));
  assert.ok(mesh.indices.length > 0 && mesh.indices.length % 3 === 0, `${label}: mesh`);
  assert.equal(mesh.faceOffsets.length, 4, `${label}: three face groups`);
  assertClosedOriented(mesh.indices, i => i, `${label} raw`);
  const weld = new Map();
  const welded = new Uint32Array(mesh.positions.length / 3);
  for (let i = 0; i < welded.length; i += 1) {
    const key = [0, 1, 2].map(c => Math.round(mesh.positions[3 * i + c] / (1e-9 * scale))).join(',');
    if (!weld.has(key)) weld.set(key, i);
    welded[i] = weld.get(key);
  }
  assertClosedOriented(mesh.indices, i => welded[i], `${label} welded`);
  const local = mesh.positions.length / 3;
  const points = Array.from({ length: local }, (_, i) => toLocal(matrix, mesh.positions.slice(3 * i, 3 * i + 3), scale));
  let volume = 0;
  let sphereFacets = 0;
  for (let at = 0; at < mesh.indices.length; at += 3) {
    const [a, b, c] = [0, 1, 2].map(k => points[mesh.indices[at + k]]);
    const cross = [
      (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
      (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
      (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]),
    ];
    volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) + a[1] * (b[2] * c[0] - b[0] * c[2])
      + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6;
    if ([a, b, c].every(p => Math.abs(ballDistance(p)) < 1e-6)) {
      sphereFacets += 1;
      const centroid = [0, 1, 2].map(k => (a[k] + b[k] + c[k]) / 3);
      assert.ok(cross[0] * centroid[0] + cross[1] * centroid[1] + cross[2] * centroid[2] > 0,
        `${label}: inward sphere facet`);
      // The skinned-hole defect left sphere facets across the tube.
      assert.ok(torusDistance(centroid) > -2 * deflection, `${label}: sphere facet inside the torus`);
    }
  }
  assert.ok(sphereFacets > 0, `${label}: sphere facets found`);
  assert.ok(Math.abs(volume - expected) < 1e-3 * expected, `${label}: mesh volume ${volume} vs ${expected}`);
  return mesh.indices.length / 3;
}

function surfaceKinds(kernel, solid) {
  const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
  const root = doc.solids[doc.solid_roots[0]];
  assert.equal(root.inner_shells.length, 0);
  const faces = doc.shells[root.outer_shell].faces.map(id => doc.faces[id]);
  const uses = new Map();
  for (const face of faces) {
    for (const wire of [face.outer_wire, ...face.inner_wires]) {
      for (const edge of doc.wires[wire].edges) {
        const directions = uses.get(edge.edge) ?? [];
        directions.push(edge.forward !== face.reversed);
        uses.set(edge.edge, directions);
      }
    }
  }
  return { faces, uses };
}

export function runOpseqTorusSphereFusePackaged({ RemusKernel }) {
  const expected = expectedVolume();
  // Independent integral against the native fixture's converged value.
  assert.ok(Math.abs(expected - 2324.508919894) < 1e-6, `independent volume ${expected}`);
  const cells = [
    { scale: 1, placed: false },
    { scale: 1, placed: true },
    // Scale 10 also needs the exact radial certificate for the NURBS holes.
    { scale: 10, placed: true },
  ];
  let count = 0;
  for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const { scale, placed } of cells) {
      for (const swapped of [false, true]) {
        const label = `opseq81 torus/sphere fuse ${api} scale=${scale} placed=${placed} swapped=${swapped}`;
        const kernel = new RemusKernel();
        const run = (op, args, direct) => {
          if (api === 'direct') return direct();
          const [row] = JSON.parse(kernel[api](JSON.stringify([{ op, args }])));
          assert.ok(Object.hasOwn(row, 'ok'), `${label}: ${op}: ${JSON.stringify(row)}`);
          return row.ok;
        };
        try {
          // The seed's own construction, in its own order.
          const source = run('makeTorus', { majorRadius: MAJOR * scale, minorRadius: MINOR * scale, segments: 16 },
            () => kernel.makeTorus(MAJOR * scale, MINOR * scale, 16));
          const sphere = run('makeSphere', { radius: BALL * scale, segments: 16 },
            () => kernel.makeSphere(BALL * scale, 16));
          const mirrorArgs = { solid: source, px: -4 * scale, py: 0, pz: 0, nx: 1, ny: 0, nz: 0 };
          const torus = run('mirror', mirrorArgs, () => kernel.mirror(source, -4 * scale, 0, 0, 1, 0, 0));
          const move = translation(scale, 0, -3 * scale);
          run('transform', { solid: torus, matrix: move }, () => kernel.transformSolid(torus, Float64Array.from(move)));
          const matrix = placement(scale, placed);
          if (placed) {
            for (const solid of [torus, sphere]) {
              run('transform', { solid, matrix }, () => kernel.transformSolid(solid, Float64Array.from(matrix)));
            }
          }
          const inputs = Uint32Array.of(torus, sphere);
          const operands = kernel.serializeSolids(inputs);
          const [solidA, solidB] = swapped ? [sphere, torus] : [torus, sphere];
          const result = json(run('booleanWithQuality', { operation: 'fuse', solidA, solidB, exactOnly: true },
            () => kernel.booleanWithQuality('fuse', solidA, solidB, true)));
          assert.equal(result.quality, 'exact', label);
          const solid = result.solid;
          assert.deepEqual(kernel.serializeSolids(inputs), operands, `${label}: operands unchanged`);

          // B-Rep, before any mesh.
          assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, `${label}: validity`);
          const { faces, uses } = surfaceKinds(kernel, solid);
          assert.deepEqual(faces.map(f => Object.keys(f.surface)[0]).sort(), ['Sphere', 'Sphere', 'Torus'], label);
          assert.equal(faces.filter(f => Object.keys(f.surface)[0] === 'Sphere' && f.inner_wires.length === 2).length,
            1, `${label}: one hemisphere carries both section holes`);
          for (const [edge, directions] of uses) {
            assert.equal(directions.length, 2, `${label}: edge ${edge} incidence`);
            assert.notEqual(directions[0], directions[1], `${label}: edge ${edge} orientation`);
          }
          let coverage = [0, 0, 0, 0];
          for (const x of [-12, -10.5, -7, -4.5, -1, 3, 7]) for (const y of [-5.5, -1.5, 0, 2.5, 6]) {
            for (const z of [-5.5, -4, -3, -1.5, 0.5, 4, 7.5]) {
              const p = [x, y, z];
              if (Math.abs(ballDistance(p)) < 0.2 || Math.abs(torusDistance(p)) < 0.2) continue;
              const [ball, tube] = [ballDistance(p) < 0, torusDistance(p) < 0];
              coverage[2 * Number(ball) + Number(tube)] += 1;
              const [wx, wy, wz] = apply(matrix, p.map(v => v * scale));
              assert.equal(kernel.classifyPoint(solid, wx, wy, wz, 1e-7), ball || tube ? 'inside' : 'outside',
                `${label}: material at ${p}`);
            }
          }
          assert.ok(coverage.every(n => n > 0), `${label}: membership cells ${coverage}`);
          const measured = kernel.volume(solid, 0.002 * scale) / scale ** 3;
          assert.ok(Math.abs(measured - expected) < 1e-3 * expected, `${label}: volume ${measured} vs ${expected}`);

          const triangles = [0.01, 0.004].map(d => checkMesh(kernel, solid, d, scale, matrix, expected, `${label} d=${d}`));
          assert.deepEqual(kernel.serializeSolids(inputs), operands, `${label}: operands unchanged after meshing`);
          console.log(`ok - ${label}: exact, valid, material and volume, closed meshes (${triangles.join('/')} triangles)`);
          count += 1;
        } finally {
          kernel.free();
        }
      }
    }
  }
  assert.equal(count, 18);
}
