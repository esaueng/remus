/**
 * B38: exact sphere–torus intersects (P-Class 8.1 seeds 5201364664492509300
 * and 1146645986253340484) keep opposing coedges and their material through
 * the packaged kernel.
 *
 * Seed 5201's sphere-in-torus crescent along the sphere's equator was wound
 * against its face (5 same-sense edges), and the ray classifier read the
 * repaired crescent through a self-folding 3D projection; seed 1146's sphere
 * discs, each bounded by one closed edge, kept the splitter's arbitrary
 * traversal (2 same-sense edges). Both persisted chains run through direct
 * calls, executeBatch and executeBatchV2: the intersect in both operand
 * orders and the torus − sphere cut, unplaced and rigidly placed.
 */
import assert from 'node:assert/strict';

const translation = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];

/** Row-major rigid placement: translation(17,-23,31)·Rx(0.37)·Rz(-0.61). */
const PLACEMENT = (() => {
  const [cx, sx] = [Math.cos(0.37), Math.sin(0.37)];
  const [cz, sz] = [Math.cos(-0.61), Math.sin(-0.61)];
  const r = [
    [cz, -sz, 0],
    [cx * sz, cx * cz, -sx],
    [sx * sz, sx * cz, cx],
  ];
  return [...r[0], 17, ...r[1], -23, ...r[2], 31, 0, 0, 0, 1];
})();

const place = (m, [x, y, z]) => [
  m[0] * x + m[1] * y + m[2] * z + m[3],
  m[4] * x + m[5] * y + m[6] * z + m[7],
  m[8] * x + m[9] * y + m[10] * z + m[11],
];

// Independent slice integrals (sphere disc ∩ torus annulus at each height),
// as in the native `regress_opseq_sphere_torus_orientation`.
const SEEDS = [
  {
    name: 'seed5201',
    sphere: [[0, 3, 0], 5],
    torus: [[0, 0, 0], 5.5, 2.75],
    torusVolume: 2 * Math.PI ** 2 * 5.5 * 2.75 ** 2,
    intersection: 241.46710379318287,
    // The persisted chain: sphere 0, torus 1, mirrored sphere 2.
    chain: run => {
      run('makeSphere', { radius: 5, segments: 16 }, [5, 16]);
      run('makeTorus', { majorRadius: 5.5, minorRadius: 2.75, segments: 16 }, [5.5, 2.75, 16]);
      const image = run('mirror', { solid: 0, px: 0, py: 1.5, pz: 0, nx: 0, ny: 1, nz: 0 }, [0, 0, 1.5, 0, 0, 1, 0]);
      return { sphere: image, torus: 1 };
    },
    // [point, inSphere, inTorus] from the inequalities, every probe at least
    // 0.19 from both surfaces. Point (-5.628, 1.705, -0.078) lies inside the
    // torus only: the ray vote read it inside the intersect through the
    // folded crescent projection.
    probes: [
      [[-5.628333333333334, 1.705, -0.07777777777777839], false, true],
      [[0, 7.5, 0], true, true],
      [[4.5, 4.5, 0], true, true],
      [[0, 7.5, -1], true, true],
      [[0, 7.5, -2.6], false, false],
      [[0, 0, 0], true, false],
    ],
  },
  {
    name: 'seed1146',
    sphere: [[0, 0, 0], 8],
    torus: [[-7, 0, -3], 4, 2],
    torusVolume: 2 * Math.PI ** 2 * 4 * 2 ** 2,
    intersection: 135.97900579133085,
    // The persisted chain: torus 0, sphere 1, mirrored torus 2 moved by (1,0,-3).
    chain: run => {
      run('makeTorus', { majorRadius: 4, minorRadius: 2, segments: 16 }, [4, 2, 16]);
      run('makeSphere', { radius: 8, segments: 16 }, [8, 16]);
      const image = run('mirror', { solid: 0, px: -4, py: 0, pz: 0, nx: 1, ny: 0, nz: 0 }, [0, -4, 0, 0, 1, 0, 0]);
      run('transform', { solid: image, matrix: translation(1, 0, -3) }, [image, Float64Array.from(translation(1, 0, -3))]);
      return { sphere: 1, torus: image };
    },
    probes: [
      [[-3, 0, -3], true, true],
      [[-3, 0, -1.5], true, true],
      [[-4, 3, -3], true, true],
      [[-11, 0, -3], false, true],
      [[0, 0, 0], true, false],
      [[-7, 0, -3], true, false],
    ],
  },
];

function assertClosedMeshes(kernel, solid, label) {
  for (const deflection of [0.1, 0.01]) {
    const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, deflection, 0.35));
    assert.ok(mesh.indices.length > 0, `${label}: nonempty mesh`);
    // Raw vertex IDs, no welding: every edge used twice in opposite senses.
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

/** Every arena edge is used by exactly two coedges running opposite ways. */
function assertOpposingCoedges(kernel, solid, label) {
  const doc = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
  const root = doc.solids[doc.solid_roots[0]];
  const faces = [root.outer_shell, ...root.inner_shells].flatMap(shell => doc.shells[shell].faces);
  const carriers = new Set(faces.map(id => Object.keys(doc.faces[id].surface)[0]));
  assert.deepEqual([...carriers].sort(), ['Sphere', 'Torus'], `${label}: exact carriers`);
  const uses = new Map();
  for (const id of faces) {
    const face = doc.faces[id];
    for (const wire of [face.outer_wire, ...face.inner_wires]) {
      for (const edge of doc.wires[wire].edges) {
        const directions = uses.get(edge.edge) ?? [];
        directions.push(edge.forward !== face.reversed);
        uses.set(edge.edge, directions);
      }
    }
  }
  assert.ok(uses.size > 0, `${label}: edges`);
  for (const [edge, directions] of uses) {
    assert.equal(directions.length, 2, `${label}: edge ${edge} incidence`);
    assert.notEqual(directions[0], directions[1], `${label}: edge ${edge} same-sense`);
  }
}

export function runOpseqSphereTorusIntersectPackaged({ RemusKernel }) {
  for (const seed of SEEDS) {
    const sphereVolume = (4 / 3) * Math.PI * seed.sphere[1] ** 3;
    for (const placed of [false, true]) {
      for (const [operation, sphereFirst] of [['intersect', true], ['intersect', false], ['cut', false]]) {
        for (const api of ['direct', 'executeBatch', 'executeBatchV2']) {
          const label = `${seed.name} ${operation} sphereFirst=${sphereFirst} placed=${placed} ${api}`;
          const kernel = new RemusKernel();
          const run = (op, args, direct) => {
            if (api === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
            const row = JSON.parse(kernel[api](JSON.stringify([{ op, args }])))[0];
            assert.ok(Object.hasOwn(row, 'ok'), `${label}: ${op}: ${JSON.stringify(row)}`);
            return row.ok;
          };
          try {
            const { sphere, torus } = seed.chain(run);
            if (placed) {
              for (const solid of [sphere, torus]) {
                run('transform', { solid, matrix: PLACEMENT }, [solid, Float64Array.from(PLACEMENT)]);
              }
            }
            const inputs = Uint32Array.of(sphere, torus);
            const original = kernel.serializeSolids(inputs);
            const [solidA, solidB] = sphereFirst ? [sphere, torus] : [torus, sphere];
            const result = run(
              'booleanWithQuality',
              { operation, solidA, solidB, exactOnly: true },
              [operation, solidA, solidB, true],
            );
            assert.equal(result.quality, 'exact', label);
            const solid = result.solid;
            assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, `${label}: validity`);
            assertOpposingCoedges(kernel, solid, label);
            assertClosedMeshes(kernel, solid, label);
            const target = operation === 'intersect' ? seed.intersection : seed.torusVolume - seed.intersection;
            const volume = kernel.volume(solid, 0.01);
            assert.ok(Math.abs(volume - target) < 2e-4 * target, `${label}: volume ${volume} against ${target}`);
            for (const [point, inSphere, inTorus] of seed.probes) {
              const inside = operation === 'intersect' ? inSphere && inTorus : inTorus && !inSphere;
              const [x, y, z] = placed ? place(PLACEMENT, point) : point;
              assert.equal(
                kernel.classifyPoint(solid, x, y, z, 1e-7),
                inside ? 'inside' : 'outside',
                `${label}: material at ${point}`,
              );
            }
            assert.deepEqual(kernel.serializeSolids(inputs), original, `${label}: unchanged operands`);
          } finally {
            kernel.free();
          }
          console.log(`ok - ${label}: exact, opposing coedges, closed meshes, slice-integral material`);
        }
      }
    }
  }
}
