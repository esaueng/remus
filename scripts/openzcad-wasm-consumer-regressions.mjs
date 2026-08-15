import assert from 'node:assert/strict';

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
}) => {
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

  const step = kernel.exportStep(narrowed);
  const stepText = new TextDecoder().decode(step);
  assert.equal(
    stepText.match(/CYLINDRICAL_SURFACE/g)?.length,
    EXPECTED_FINAL_RADII.length,
    'STEP must encode every cylinder analytically',
  );
  const importedKernel = new BrepKernel();
  const imported = Array.from(importedKernel.importStep(step));
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

export const runOpenZcadConsumerRegressions = (packageExports) => {
  runOpenZcadAnalyticFlangeBooleanRegression(packageExports);
  runOpenZcadCylindricalFaceResizeRegression(packageExports);
};
