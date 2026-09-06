import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const DEFLECTION = 0.1;

const assertCompleteEvolution = (payload, label) => {
  assert.equal(payload.schemaVersion, 1, `${label}: schema version`);
  const source = new Set(payload.source.faces);
  const result = new Set(payload.result.faces);
  const accountedSources = new Set([
    ...payload.evolution.modified.map((claim) => claim.source),
    ...payload.evolution.deleted,
    ...payload.evolution.unresolvedSources,
  ]);
  const accountedResults = new Set([
    ...payload.evolution.modified.flatMap((claim) => claim.results),
    ...payload.evolution.generated.flatMap((claim) => claim.results),
    ...payload.evolution.unresolvedResults.map((claim) => claim.result),
  ]);
  assert.deepEqual(accountedSources, source, `${label}: source coverage`);
  assert.deepEqual(accountedResults, result, `${label}: result coverage`);
};

// Fusing two coaxial revolved annuli that share an exact cylindrical wall —
// the OpenZCAD flange demo's "Union flange blank": a rim (r24..45, t10)
// against a hub (r12..24, h26), both walls exactly r24 and overlapping in z.
// The fuse must resolve that coincident cylindrical face pair and stay
// analytic. A mesh fallback can remain watertight, valid, and close on volume,
// so the exact face and surface-kind assertions must not be relaxed.
export const runOpenZcadAnalyticFlangeBooleanRegression = ({ BrepKernel }) => {
  const kernel = new BrepKernel();

  /** Revolve an axial rectangle (x = radius, z = height) a full turn about +Z. */
  const revolveAnnulus = (r0, r1, z0, z1) => {
    const pts = [
      [r0, 0, z0],
      [r1, 0, z0],
      [r1, 0, z1],
      [r0, 0, z1],
    ];
    const edges = pts.map((p, i) => {
      const n = pts[(i + 1) % pts.length];
      return kernel.makeLineEdge(p[0], p[1], p[2], n[0], n[1], n[2]);
    });
    const wire = kernel.makeWire(Uint32Array.from(edges), true);
    const face = kernel.makePlanarFaceFromWire(wire);
    return kernel.revolve(face, 0, 0, 0, 0, 0, 1, 360);
  };

  const rim = revolveAnnulus(24, 45, -10, 0);
  const hub = revolveAnnulus(12, 24, -26, 0);

  for (const [label, solid] of [
    ['rim', rim],
    ['hub', hub],
  ]) {
    const kinds = Array.from(kernel.getSolidFaces(solid)).map((face) =>
      kernel.getSurfaceType(face),
    );
    assert.equal(
      kinds.filter((type) => type === 'cylinder').length,
      2,
      `${label} operand should have 2 cylindrical walls, got ${JSON.stringify(kinds)}`,
    );
  }

  const fused = kernel.fuse(rim, hub);
  const faceKinds = Array.from(kernel.getSolidFaces(fused)).map((face) =>
    kernel.getSurfaceType(face),
  );
  const cylinders = faceKinds.filter((type) => type === 'cylinder').length;

  assert.ok(
    faceKinds.length <= 12,
    `coaxial annulus fuse mesh-fell-back: ${faceKinds.length} faces ` +
      `(expected <= 12; native gives 7)`,
  );
  assert.ok(
    cylinders >= 3,
    `coaxial annulus fuse lost its cylindrical walls: ${cylinders} cylinder ` +
      `faces of ${faceKinds.length} (expected >= 3)`,
  );

  const fusedVolume = kernel.volume(fused, DEFLECTION);
  const expectedVolume = Math.PI * ((45 * 45 - 24 * 24) * 10 + (24 * 24 - 12 * 12) * 26);
  assert.ok(
    Math.abs(fusedVolume - expectedVolume) < 50,
    `coaxial annulus fuse volume=${fusedVolume}, expected ~${expectedVolume}`,
  );
  console.log(
    `ok - coaxial annulus fuse stayed analytic: ${faceKinds.length} faces, ` +
      `${cylinders} cylindrical`,
  );
};

// OpenZCAD stores the r3 mounting-bore face before rounding the bracket's four
// outside corners. Track that face through typed evolution, widen r3 -> r4.8,
// shrink r4.8 -> r3.8, and preserve the exact analytic topology through STEP.
export const runOpenZcadCylindricalFaceResizeRegression = ({
  BrepKernel,
  decodeEvolutionPayload,
  RemusIo,
}) => {
  // File formats live in the translator module; bodies cross as arena bytes.
  const io = new RemusIo();
  const W = 80;
  const D = 40;
  const PLATE_T = 8;
  const MOUNT_X = 16;
  const MOUNT_Y = 20;
  const BRACKET_DEFLECTION = 0.05;
  const FILLETED_VOLUME = 47_360.940_056_943_74;
  const WIDE_VOLUME = 47_008.076_370_092_516;
  const FINAL_VOLUME = WIDE_VOLUME + Math.PI * (4.8 ** 2 - 3.8 ** 2) * PLATE_T;
  const EXPECTED_FINAL_RADII = [3, 3, 3, 3, 3, 3.8, 4, 10];

  const kernel = new BrepKernel();
  const translated = (solid, x, y, z) =>
    kernel.copyAndTransformSolid(solid, [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1]);
  const rotatedX90AndTranslated = (solid, x, y, z) =>
    kernel.copyAndTransformSolid(solid, [1, 0, 0, x, 0, 0, -1, y, 0, 1, 0, z, 0, 0, 0, 1]);
  const fuseUniform = (...solids) => {
    const result = kernel.fuseAll(Uint32Array.from(solids));
    kernel.unifyFaces(result);
    return result;
  };
  const cutUniform = (target, tools) => {
    for (const tool of tools) target = kernel.cut(target, tool);
    kernel.unifyFaces(target);
    return target;
  };
  const analyticParams = (face) => JSON.parse(kernel.getAnalyticSurfaceParams(face));
  const mountingWall = (solid, radius) => {
    const matches = Array.from(kernel.getSolidFaces(solid)).filter((face) => {
      const surface = analyticParams(face);
      return (
        surface.type === 'cylinder' &&
        Math.abs(surface.radius - radius) < 1e-8 &&
        Math.abs(surface.origin[0] - MOUNT_X) < 1e-8 &&
        Math.abs(surface.origin[1] - MOUNT_Y) < 1e-8 &&
        Math.abs(surface.axis[2]) > 1 - 1e-10
      );
    });
    assert.equal(matches.length, 1, `expected one r${radius} mounting-bore wall`);
    return matches[0];
  };
  const assertVolumeAndClosure = (solid, expected, label) => {
    assert.equal(kernel.validateSolid(solid), 0, `${label}: closed, valid shell`);
    const actual = kernel.volume(solid, BRACKET_DEFLECTION);
    const tolerance = Math.max(Math.abs(expected), 1) * 1e-9;
    assert.ok(
      Math.abs(actual - expected) <= tolerance,
      `${label}: volume=${actual}, expected=${expected}, tolerance=${tolerance}`,
    );
  };
  const cylinderRadii = (activeKernel, solid) =>
    Array.from(activeKernel.getSolidFaces(solid))
      .map((face) => JSON.parse(activeKernel.getAnalyticSurfaceParams(face)))
      .filter((surface) => surface.type === 'cylinder')
      .map((surface) => surface.radius)
      .sort((a, b) => a - b);
  const assertExactCylinderRadii = (activeKernel, solid, label) => {
    const actual = cylinderRadii(activeKernel, solid);
    assert.equal(actual.length, EXPECTED_FINAL_RADII.length, `${label}: cylinder count`);
    actual.forEach((radius, index) => {
      assert.ok(
        Math.abs(radius - EXPECTED_FINAL_RADII[index]) < 1e-8,
        `${label}: analytic cylinder radii ${JSON.stringify(actual)}`,
      );
    });
  };

  const base = kernel.makeBox(W, D, PLATE_T);
  const wall = translated(kernel.makeBox(W, PLATE_T, 32), 0, 32, 7.5);
  const blank = fuseUniform(base, wall);

  const boss = rotatedX90AndTranslated(kernel.makeCylinder(10, 12), 40, 34, 24);
  const bossed = fuseUniform(blank, boss);
  const bossBore = rotatedX90AndTranslated(kernel.makeCylinder(4, 48), 40, 48, 24);
  const bored = cutUniform(bossed, [bossBore]);

  const leftMount = translated(kernel.makeCylinder(3, 12), MOUNT_X, MOUNT_Y, -2);
  const rightMount = translated(kernel.makeCylinder(3, 12), W - MOUNT_X, MOUNT_Y, -2);
  const drilled = cutUniform(bored, [leftMount, rightMount]);
  const sourceMountingWall = mountingWall(drilled, 3);

  const cornerEdges = Array.from(kernel.getSolidEdges(drilled)).filter((edge) => {
    const [ax, ay, az, bx, by, bz] = kernel.getEdgeVertices(edge);
    const atCorner = (x, y, z) =>
      (Math.abs(x) < 0.1 || Math.abs(x - W) < 0.1) &&
      (Math.abs(y) < 0.1 || Math.abs(y - D) < 0.1) &&
      z >= -0.1 &&
      z <= 8.1;
    return (
      atCorner(ax, ay, az) &&
      atCorner(bx, by, bz) &&
      Math.abs(ax - bx) <= 1.5 &&
      Math.abs(ay - by) <= 1.5 &&
      Math.abs(az - bz) >= 4
    );
  });
  assert.equal(cornerEdges.length, 4, 'mounting bracket: four outside corner edges');

  const transported = JSON.stringify(
    kernel.filletWithEvolution(drilled, Uint32Array.from(cornerEdges), 3),
  );
  const evolution = decodeEvolutionPayload(transported);
  assertCompleteEvolution(evolution, 'mounting bracket fillet');
  assert.equal(evolution.evolution.provenance, 'construction');
  const descendantClaims = evolution.evolution.modified.filter(
    (claim) => claim.source === sourceMountingWall,
  );
  assert.equal(descendantClaims.length, 1, 'mounting-bore source must have one lineage claim');
  assert.equal(
    descendantClaims[0].results.length,
    1,
    'mounting-bore source must resolve to one descendant face',
  );
  const descendantMountingWall = descendantClaims[0].results[0];
  assert.ok(
    evolution.result.faces.includes(descendantMountingWall),
    'mounting-bore descendant must belong to the filleted solid',
  );
  const descendantSurface = analyticParams(descendantMountingWall);
  assert.equal(descendantSurface.type, 'cylinder', 'mounting-bore descendant stays analytic');
  assert.ok(Math.abs(descendantSurface.radius - 3) < 1e-8, 'mounting-bore descendant stays r3');
  assertVolumeAndClosure(evolution.result.solid, FILLETED_VOLUME, 'filleted bracket');

  const widened = kernel.resizeCylindricalFace(evolution.result.solid, descendantMountingWall, 4.8);
  assertVolumeAndClosure(widened, WIDE_VOLUME, 'r3 -> r4.8 bracket');
  const widenedMountingWall = mountingWall(widened, 4.8);
  const narrowed = kernel.resizeCylindricalFace(widened, widenedMountingWall, 3.8);
  assertVolumeAndClosure(narrowed, FINAL_VOLUME, 'r4.8 -> r3.8 bracket');
  assertExactCylinderRadii(kernel, narrowed, 'resized bracket');

  const step = io.exportStep(kernel.serializeSolids(Uint32Array.of(narrowed)));
  const stepText = new TextDecoder().decode(step);
  assert.equal(
    stepText.match(/CYLINDRICAL_SURFACE/g)?.length,
    EXPECTED_FINAL_RADII.length,
    'STEP must encode every cylinder analytically',
  );
  const importedKernel = new BrepKernel();
  const imported = Array.from(importedKernel.deserializeSolids(io.importStep(step)));
  assert.equal(imported.length, 1, 'STEP round trip must yield one bracket solid');
  assert.equal(importedKernel.validateSolid(imported[0]), 0, 'STEP bracket: closed, valid shell');
  const importedVolume = importedKernel.volume(imported[0], BRACKET_DEFLECTION);
  assert.ok(
    Math.abs(importedVolume - FINAL_VOLUME) <= Math.abs(FINAL_VOLUME) * 1e-9,
    `STEP bracket: volume=${importedVolume}, expected=${FINAL_VOLUME}`,
  );
  assertExactCylinderRadii(importedKernel, imported[0], 'STEP bracket');
  console.log('ok - OpenZCAD mounting bracket: decoded lineage, r3 -> r4.8 -> r3.8, exact STEP');
};

/** `exports` merges the kernel package (`BrepKernel`, `decodeEvolutionPayload`)
 * with the translator package (`RemusIo`). */
export const runOpenZcadConsumerRegressions = (exports) => {
  runOpenZcadAnalyticFlangeBooleanRegression(exports);
  runOpenZcadCylindricalFaceResizeRegression(exports);
  runWideSphereCapRegression(exports);
  runOffsetConeSphereRegression(exports);
  runOffsetSphereCylinderRegression(exports);
};

export const runWideSphereCapRegression = ({ BrepKernel, RemusIo }) => {
  for (const fixture of ['wide_sphere_cap.step', 'wide_sphere_cap_with_seam.step']) {
    const kernel = new BrepKernel();
    const io = new RemusIo();
    const step = readFileSync(new URL(`../crates/io/tests/data/${fixture}`, import.meta.url));
    const solids = Array.from(kernel.deserializeSolids(io.importStep(step)));
    assert.equal(solids.length, 1);
    const solid = solids[0];
    assert.equal(kernel.getSolidFaces(solid).length, 2);
    const expected = (Math.PI * 16.5 ** 2 * (27 - 16.5)) / 3;
    for (const deflection of [0.045, 0.0045]) {
      const mesh = kernel.tessellateSolid(solid, deflection);
      let volume = 0;
      for (let i = 0; i < mesh.indices.length; i += 3) {
        const a = mesh.indices[i] * 3;
        const b = mesh.indices[i + 1] * 3;
        const c = mesh.indices[i + 2] * 3;
        const p = mesh.positions;
        volume +=
          (p[a] * (p[b + 1] * p[c + 2] - p[b + 2] * p[c + 1]) +
            p[a + 1] * (p[b + 2] * p[c] - p[b] * p[c + 2]) +
            p[a + 2] * (p[b] * p[c + 1] - p[b + 1] * p[c])) /
          6;
      }
      assert.ok(
        Math.abs(volume - expected) / expected < 0.01,
        `wide spherical cap volume ${volume} vs ${expected}`,
      );
      const direct = JSON.parse(kernel.meshQuality(solid, deflection));
      assert.equal(direct.isWatertight, true);
      assert.equal(direct.boundaryEdges, 0);
      assert.equal(direct.nonManifoldEdges, 0);
      const [batch] = JSON.parse(
        kernel.executeBatch(JSON.stringify([{ op: 'meshQuality', args: { solid, deflection } }])),
      );
      assert.deepEqual(batch.ok, direct);
    }
    kernel.free();
    io.free();
  }
  console.log('ok - wide spherical cap preserves closed-form volume and direct/batch mesh quality');
};

export const runOffsetConeSphereRegression = ({ BrepKernel }) => {
  // Independent horizontal-disk overlap integral, also pinned by the native matrix.
  const overlap = 197.10640301106753;
  for (const [operation, expected] of [
    ['fuse', (208 + 256 / 3) * Math.PI - overlap],
    ['cut', 208 * Math.PI - overlap],
    ['intersect', overlap],
  ]) {
    const kernel = new BrepKernel();
    const cone = kernel.makeCone(6, 2, 12);
    const sphere = kernel.makeSphere(4, 24);
    kernel.transformSolid(
      sphere,
      new Float64Array([1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, 6, 0, 0, 0, 1]),
    );
    const result = kernel.booleanWithQuality(operation, cone, sphere, true);
    assert.equal(result.quality, 'exact', `${operation}: exact seam result`);
    assert.equal(kernel.validateSolid(result.solid), 0, `${operation}: valid shell`);
    const volume = kernel.volume(result.solid, 0.01);
    assert.ok(
      Math.abs(volume - expected) < expected * 0.001,
      `${operation}: ${volume} vs ${expected}`,
    );
  }
  console.log('ok - offset cone/sphere: exact fuse, cut, intersect against disk-overlap oracle');
};

export const runOffsetSphereCylinderRegression = ({ BrepKernel }) => {
  // Independent horizontal-disk overlap integral, also pinned by the native matrix.
  const overlap = 294.1884259241949;
  for (const [operation, expected] of [
    ['fuse', (288 + 180) * Math.PI - overlap],
    ['cut', 288 * Math.PI - overlap],
    ['intersect', overlap],
  ]) {
    const kernel = new BrepKernel();
    const sphere = kernel.makeSphere(6, 24);
    const cylinder = kernel.makeCylinder(3, 20);
    kernel.transformSolid(
      cylinder,
      new Float64Array([1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 1, -10, 0, 0, 0, 1]),
    );
    const result = kernel.booleanWithQuality(operation, sphere, cylinder, true);
    assert.equal(result.quality, 'exact', `${operation}: exact seam result`);
    assert.equal(kernel.validateSolid(result.solid), 0, `${operation}: valid shell`);
    const volume = kernel.volume(result.solid, 0.01);
    assert.ok(
      Math.abs(volume - expected) < expected * 0.001,
      `${operation}: ${volume} vs ${expected}`,
    );
  }
  console.log('ok - offset sphere/cylinder: exact fuse, cut, intersect against disk-overlap oracle');
};
