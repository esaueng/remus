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
  runPartialCylinderResizeRegression(exports);
  runDirectEditHistoryRegression(exports);
  runSurfaceReplacementHistoryRegression(exports);
  runCylindricalRadiusHistoryRegression(exports);
  runDraftHistoryRegression(exports);
  runDefeatureHistoryRegression(exports);
  runHealingHistoryRegression(exports);
  runUnifyHistoryRegression(exports);
  runSewingHistoryRegression(exports);
  runInnerWireHistoryRegression(exports);
  runSplitVertexRefusalRegression(exports);
  runWideSphereCapRegression(exports);
  runOffsetConeSphereRegression(exports);
  runOffsetSphereCylinderRegression(exports);
  runOffsetTorusSphereRegression(exports);
  runTorusNotchRegression(exports);
  runTangencyBandRegression(exports);
  runBooleanScaleRegression(exports);
  runAnisotropicBooleanRegression(exports);
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


export const runOffsetTorusSphereRegression = ({ BrepKernel }) => {
  // Independent annulus/disk overlap integral from the native qualification matrix.
  const overlap = 56.2702148298352;
  for (const [operation, expected] of [
    ['fuse', 48 * Math.PI ** 2 + 36 * Math.PI - overlap],
    ['cut', 48 * Math.PI ** 2 - overlap],
    ['intersect', overlap],
  ]) {
    const kernel = new BrepKernel();
    const torus = kernel.makeTorus(6, 2, 32);
    const sphere = kernel.makeSphere(3, 24);
    kernel.transformSolid(
      sphere,
      new Float64Array([1, 0, 0, 5, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1]),
    );
    const c = Math.cos(0.37);
    const s = Math.sin(0.37);
    const placement = new Float64Array([c, 0, s, 17, 0, 1, 0, -23, -s, 0, c, 31, 0, 0, 0, 1]);
    kernel.transformSolid(torus, placement);
    kernel.transformSolid(sphere, placement);
    const result = kernel.booleanWithQuality(operation, torus, sphere, true);
    assert.equal(result.quality, 'exact', `${operation}: exact torus seam result`);
    assert.equal(kernel.validateSolid(result.solid), 0, `${operation}: valid torus seam shell`);
    const volume = kernel.volume(result.solid, 0.01);
    assert.ok(Math.abs(volume - expected) < expected * 0.001, `${operation}: ${volume} vs ${expected}`);
  }
  console.log('ok - offset torus/sphere: exact rotated fuse, cut, intersect against annulus-overlap oracle');
};

export const runBooleanScaleRegression = ({ BrepKernel }) => {
  for (let exponent = -5; exponent <= 6; exponent += 1) {
    const scale = 10 ** exponent;
    for (const placed of [false, true]) {
      for (const [operation, expected] of [['fuse', 1.16], ['cut', 0.84], ['intersect', 0.16]]) {
        const kernel = new BrepKernel();
        const blank = kernel.makeBox(scale, scale, scale);
        const tool = kernel.makeBox(0.4 * scale, 0.4 * scale, 2 * scale);
        kernel.transformSolid(tool, new Float64Array([1, 0, 0, 0.3 * scale, 0, 1, 0, 0.3 * scale, 0, 0, 1, -0.5 * scale, 0, 0, 0, 1]));
        if (placed) {
          const c = Math.cos(0.37);
          const s = Math.sin(0.37);
          const matrix = new Float64Array([c, 0, s, 17 * scale, 0, 1, 0, -23 * scale, -s, 0, c, 31 * scale, 0, 0, 0, 1]);
          kernel.transformSolid(blank, matrix);
          kernel.transformSolid(tool, matrix);
        }
        const result = kernel.booleanWithQuality(operation, blank, tool, true);
        const label = `${operation} scale=${scale} placed=${placed}`;
        assert.equal(result.quality, 'exact', label);
        assert.equal(kernel.validateSolid(result.solid), 0, label);
        const volume = kernel.volume(result.solid, 0.01 * scale) / scale ** 3;
        assert.ok(Math.abs(volume - expected) / expected < 1e-6, `${label}: ${volume} vs ${expected}`);
        kernel.free();
      }
    }
  }
  console.log('ok - 72 exact through-tool boolean cells across scales 1e-5..1e6 and rigid placement');
};


export const runAnisotropicBooleanRegression = ({ BrepKernel }) => {
  for (const batch of [false, true]) {
    for (const length of [1, 1e3, 1e6]) for (const width of [0.1, 0.001]) {
      for (const placed of [false, true]) for (const operation of ['fuse', 'cut', 'intersect']) {
        const kernel = new BrepKernel();
        try {
          const invoke = (op, args) => {
            const [result] = JSON.parse(kernel.executeBatch(JSON.stringify([{ op, args }])));
            assert.ok(Object.hasOwn(result, 'ok'), JSON.stringify(result));
            return result.ok;
          };
          const box = (x, y, z) => batch ? invoke('makeBox', { width: x, height: y, depth: z }) : kernel.makeBox(x, y, z);
          const transform = (solid, matrix) => batch ? invoke('transform', { solid, matrix }) : kernel.transformSolid(solid, new Float64Array(matrix));
          const boolean = (op, a, b) => batch ? invoke('booleanWithQuality', { operation: op, solidA: a, solidB: b, exactOnly: true }) : kernel.booleanWithQuality(op, a, b, true);
          const blank = box(length, 1, 1);
          const tool = box(width, 0.4, 2);
          transform(tool, [1,0,0,width,0,1,0,0.3,0,0,1,-0.5,0,0,0,1]);
          const c = Math.cos(0.37), s = Math.sin(0.37);
          const placement = [c,0,s,17,0,1,0,-23,-s,0,c,31,0,0,0,1];
          if (placed) for (const solid of [blank, tool]) transform(solid, placement);
          const result = boolean(operation, blank, tool);
          const label = `${operation} length=${length} width=${width} placed=${placed} batch=${batch}`;
          assert.equal(result.quality, 'exact', label);
          assert.equal(kernel.validateSolid(result.solid), 0, label);
          const overlap = width * 0.4;
          const expected = operation === 'fuse' ? length + overlap : operation === 'cut' ? length - overlap : overlap;
          const tolerance = placed ? Math.max(expected * 1e-9, overlap * 1e-5) : overlap * 1e-5;
          assert.ok(Math.abs(kernel.volume(result.solid, 0.001) - expected) <= tolerance, `${label}: world volume`);
          for (const z of [-0.25, 0.5, 1.25]) {
            const x = 1.5 * width;
            const point = placed ? [c*x+s*z+17, -22.5, -s*x+c*z+31] : [x,0.5,z];
            const inside = z === 0.5 ? operation !== 'cut' : operation === 'fuse';
            assert.equal(kernel.classifyPoint(result.solid, ...point, 1e-7), inside ? 'inside' : 'outside', `${label}: point z=${z}`);
          }
          const crop = box(3 * width, 1, 2);
          transform(crop, [1,0,0,0,0,1,0,0,0,0,1,-0.5,0,0,0,1]);
          if (placed) transform(crop, placement);
          const local = boolean('intersect', result.solid, crop);
          assert.equal(local.quality, 'exact', label);
          const expectedLocal = operation === 'fuse' ? 3*width+overlap : operation === 'cut' ? 3*width-overlap : overlap;
          assert.ok(Math.abs(kernel.volume(local.solid, 0.001) - expectedLocal) / overlap <= 1e-5, `${label}: local material volume`);
        } finally { kernel.free(); }
      }
    }
  }
  console.log('ok - 36 anisotropic boolean cells in direct and batch APIs retain local material');
};

export const runTorusNotchRegression = ({ BrepKernel }) => {
  // Independent annular-section quadrature, recomputed in the native matrix.
  const overlap = 233.17975756277542;
  const torusVolume = 180 * Math.PI ** 2;
  for (const batch of [false, true]) for (const scale of [0.1, 1, 10]) {
    for (const placed of [false, true]) for (const operation of ['fuse', 'cut', 'intersect']) {
      const kernel = new BrepKernel();
      try {
        const invoke = (op, args) => {
          const [result] = JSON.parse(kernel.executeBatch(JSON.stringify([{ op, args }])));
          assert.ok(Object.hasOwn(result, 'ok'), JSON.stringify(result));
          return result.ok;
        };
        const transform = (solid, matrix) => batch
          ? invoke('transform', { solid, matrix })
          : kernel.transformSolid(solid, new Float64Array(matrix));
        const torus = batch
          ? invoke('makeTorus', { majorRadius: 10 * scale, minorRadius: 3 * scale, segments: 32 })
          : kernel.makeTorus(10 * scale, 3 * scale, 32);
        const box = batch
          ? invoke('makeBox', { width: 8 * scale, height: 8 * scale, depth: 8 * scale })
          : kernel.makeBox(8 * scale, 8 * scale, 8 * scale);
        transform(box, [1,0,0,6*scale,0,1,0,-4*scale,0,0,1,-4*scale,0,0,0,1]);
        if (placed) {
          const c = Math.cos(0.37), s = Math.sin(0.37);
          const matrix = [c,0,s,17*scale,0,1,0,-23*scale,-s,0,c,31*scale,0,0,0,1];
          for (const solid of [torus, box]) transform(solid, matrix);
        }
        const result = batch
          ? invoke('booleanWithQuality', { operation, solidA: torus, solidB: box, exactOnly: true })
          : kernel.booleanWithQuality(operation, torus, box, true);
        const label = `${operation} scale=${scale} placed=${placed} batch=${batch}`;
        assert.equal(result.quality, 'exact', label);
        assert.equal(kernel.validateSolid(result.solid), 0, label);
        const expected = operation === 'fuse' ? torusVolume + 512 - overlap
          : operation === 'cut' ? torusVolume - overlap : overlap;
        const volume = kernel.volume(result.solid, 0.01 * scale) / scale ** 3;
        assert.ok(Math.abs(volume - expected) / expected < 0.001, `${label}: ${volume} vs ${expected}`);
      } finally {
        kernel.free();
      }
    }
  }
  console.log('ok - torus notch: 18 exact scale/placement cells through direct and batch APIs');
};

export const runTangencyBandRegression = ({ BrepKernel }) => {
  const outcomes = new Map();
  const totals = { exact: 0, refused: 0 };
  for (const batch of [false, true]) for (const scale of [0.1, 1, 10]) {
    for (const placed of [false, true]) for (const operation of ['fuse', 'cut', 'intersect']) {
      for (const epsilon of [-1e-3, -1e-5, -1e-7, -1e-9, 0, 1e-9, 1e-7, 1e-5, 1e-3]) {
        const kernel = new BrepKernel();
        const label = `${operation} epsilon=${epsilon} scale=${scale} placed=${placed}`;
        try {
          const half = 4 + epsilon;
          const cylinder = kernel.makeCylinder(4 * scale, 12 * scale);
          const box = kernel.makeBox(2 * half * scale, 2 * half * scale, 8 * scale);
          kernel.transformSolid(box, new Float64Array([1,0,0,-half*scale,0,1,0,-half*scale,0,0,1,6*scale,0,0,0,1]));
          const c = Math.cos(0.37), s = Math.sin(0.37);
          if (placed) for (const solid of [cylinder, box]) {
            kernel.transformSolid(solid, new Float64Array([c,0,s,17*scale,0,1,0,-23*scale,-s,0,c,31*scale,0,0,0,1]));
          }
          const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(cylinder, box)));
          let result, refusal;
          if (batch) {
            const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([{
              op: 'booleanWithQuality', args: { operation, solidA: cylinder, solidB: box, exactOnly: true },
            }])));
            if (response.error) {
              assert.equal(response.error.category, 'quality_refused', label);
              assert.equal(response.error.details.kernelCode, 'exact_only_unattainable', label);
              refusal = true;
            } else result = response.ok;
          } else {
            try { result = kernel.booleanWithQuality(operation, cylinder, box, true); }
            catch (error) {
              assert.match(String(error), /exact-only policy: the exact boolean pipeline could not produce this result/, label);
              refusal = true;
            }
          }
          const outcome = refusal ? 'refused' : 'exact';
          if (batch) assert.equal(outcome, outcomes.get(label), `${label}: direct/batch outcome`);
          else outcomes.set(label, outcome);
          totals[outcome] += 1;
          if (refusal) {
            assert.ok(epsilon !== 0 && Math.abs(epsilon) <= 1e-7, `${label}: mandatory exact cell refused`);
            assert.deepEqual(kernel.serializeSolids(Uint32Array.of(cylinder, box)), before, `${label}: rollback`);
            continue;
          }
          assert.equal(result.quality, 'exact', label);
          assert.equal(kernel.validateSolid(result.solid), 0, label);
          for (const face of kernel.getSolidFaces(result.solid)) {
            assert.ok(['plane', 'cylinder'].includes(kernel.getSurfaceType(face)), `${label}: analytic carrier`);
          }
          const cap = epsilon < 0 ? 16 * Math.acos(half / 4) - half * Math.sqrt(16 - half * half) : 0;
          const overlap = 6 * (16 * Math.PI - 4 * cap);
          const expected = (operation === 'fuse' ? 192 * Math.PI + 32 * half * half - overlap
            : operation === 'cut' ? 192 * Math.PI - overlap : overlap) * scale ** 3;
          const volume = kernel.volume(result.solid, 0.005 * scale);
          const budget = Math.max(expected * 1e-8, 512 * scale ** 2 * 1e-7);
          assert.ok(Math.abs(volume - expected) <= budget, `${label}: volume ${volume} vs ${expected}`);
          for (const [z, inside] of [[3, operation !== 'intersect'], [9, operation !== 'cut'], [13, operation === 'fuse']]) {
            const point = placed ? [s*z*scale+17*scale, -23*scale, c*z*scale+31*scale] : [0,0,z*scale];
            assert.equal(kernel.classifyPoint(result.solid, ...point, 1e-7), inside ? 'inside' : 'outside', `${label}: material z=${z}`);
          }
          if (epsilon * scale < -16e-7) {
            const middle = (4 + half) * 0.5 * scale;
            for (const [x, y] of [[middle,0],[-middle,0],[0,middle],[0,-middle]]) {
              const z = 9 * scale;
              const point = placed ? [c*x+s*z+17*scale, y-23*scale, -s*x+c*z+31*scale] : [x,y,z];
              assert.equal(kernel.classifyPoint(result.solid, ...point, 1e-7), operation === 'intersect' ? 'outside' : 'inside', `${label}: thin cap ${x},${y}`);
            }
          }
          for (const deflection of [0.005, 0.02]) {
            const quality = JSON.parse(kernel.meshQuality(result.solid, deflection * scale));
            assert.equal(quality.isWatertight, true, `${label}: mesh ${JSON.stringify(quality)}`);
          }
        } finally { kernel.free(); }
      }
    }
  }
  console.log(`ok - tangency direct/batch: ${totals.exact} exact, ${totals.refused} typed-policy refusals with rollback`);
};


export const runPartialCylinderResizeRegression = ({ BrepKernel, RemusIo }) => {
  const fixture = readFileSync(new URL('../crates/io/tests/data/jolly_fox_partial_cylinder.step', import.meta.url));
  const expectedVolume = (radius) => 73 * 41 * 8 + 12 * 30.5 * 12 + Math.PI * radius ** 2 * 12 / 4;
  const io = new RemusIo();
  const qualify = (kernel, solid, radius, label) => {
    const validation = JSON.parse(kernel.validateSolidDetailed(solid));
    assert.equal(validation.errorCount, 0, label);
    assert.equal(validation.warningCount, 0, label);
    assert.ok(Math.abs(kernel.volume(solid, 0.01) - expectedVolume(radius)) < 1e-6, `${label}: volume`);
    const cylinders = Array.from(kernel.getSolidFaces(solid)).filter(face => kernel.getSurfaceType(face) === 'cylinder');
    assert.equal(cylinders.length, 1, `${label}: one analytic wall`);
    const wall = cylinders[0];
    const params = JSON.parse(kernel.getAnalyticSurfaceParams(wall));
    assert.ok(Math.abs(params.radius - radius) < 1e-9, `${label}: radius`);
    for (const [key, expected] of [['origin', [61, 41, 0]], ['axis', [0, 0, 1]]]) {
      params[key].forEach((value, index) => assert.ok(Math.abs(value - expected[index]) < 1e-9, `${label}: ${key}`));
    }
    assert.ok(Math.abs(kernel.faceArea(wall, 0.005) - Math.PI * radius * 12 / 2) < 1e-6, `${label}: wall area`);
    const levels = Array.from(kernel.getFaceEdges(wall)).flatMap(edge => {
      const vertices = kernel.getEdgeVertices(edge);
      return [vertices[2], vertices[5]];
    });
    assert.ok(Math.abs(Math.min(...levels) - 8) < 1e-9 && Math.abs(Math.max(...levels) - 20) < 1e-9, `${label}: axial extent`);
    for (const deflection of [0.005, 0.02]) {
      const quality = JSON.parse(kernel.meshQuality(solid, deflection));
      assert.equal(quality.isWatertight, true, `${label}: ${JSON.stringify(quality)}`);
    }
    return wall;
  };
  try {
    for (const batch of [false, true]) for (const radius of [20, 21, 22, 28, 30.5, 32]) {
      const kernel = new BrepKernel();
      const label = `quarter cylinder r=${radius} batch=${batch}`;
      try {
        const solids = Array.from(kernel.deserializeSolids(io.importStep(fixture)));
        assert.equal(solids.length, 1, label);
        const source = solids[0];
        const face = qualify(kernel, source, 20.5, `${label} source`);
        const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(source)));
        const counts = Array.from(kernel.getEntityCounts(source));
        let result;
        if (batch) {
          const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([{
            op: 'resizeCylindricalFace', args: { solid: source, face, radius },
          }])));
          if (radius >= 30.5) {
            assert.ok(response.error, `${label}: must refuse`);
            assert.match(JSON.stringify(response.error), /quarter-wall sweep may contact a nonadjacent source face/, label);
          } else {
            assert.equal(response.error, undefined, `${label}: ${JSON.stringify(response)}`);
            result = response.ok;
          }
        } else if (radius >= 30.5) {
          assert.throws(() => kernel.resizeCylindricalFace(source, face, radius), /quarter-wall sweep may contact a nonadjacent source face/, label);
        } else result = kernel.resizeCylindricalFace(source, face, radius);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(source)), before, `${label}: source preserved`);
        assert.deepEqual(Array.from(kernel.getEntityCounts(source)), counts, `${label}: source topology`);
        if (radius >= 30.5) continue;
        qualify(kernel, result, radius, label);
        for (const [point, inside] of [
          [[10, 10, 4], true], [[67, 20, 14], true], [[10, 10, 14], false],
          [[50, 45, 14], false], [[50, 30, 22], false],
          [[61 - (radius + 20.5) / (2 * Math.SQRT2), 41 - (radius + 20.5) / (2 * Math.SQRT2), 14], radius > 20.5],
        ]) assert.equal(kernel.classifyPoint(result, ...point, 1e-7), inside ? 'inside' : 'outside', label);
        const step = io.exportStep(kernel.serializeSolids(Uint32Array.of(result)));
        const round = new BrepKernel();
        try {
          const roundSolids = Array.from(round.deserializeSolids(io.importStep(step)));
          assert.equal(roundSolids.length, 1, label);
          qualify(round, roundSolids[0], radius, `${label} STEP`);
        } finally { round.free(); }
      } finally { kernel.free(); }
    }
  } finally { io.free(); }
  console.log('ok - quarter-cylinder direct/batch: 8 exact resizes with STEP round trips, 4 collision refusals with rollback');
};


export const runDirectEditHistoryRegression = ({ BrepKernel }) => {
  for (const batch of [false, true]) {
    const kernel = new BrepKernel();
    try {
      const source = kernel.makeBox(3, 5, 7);
      const edit = (solid, distance) => {
        const faces = [kernel.getSolidFaces(solid)[0]];
        if (!batch) return JSON.parse(kernel.moveFacesJournaled(solid, Uint32Array.from(faces), distance));
        const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([
          { op: 'moveFacesJournaled', args: { solid, faces, distance } },
        ])));
        assert.equal(response.error, undefined, JSON.stringify(response));
        return response.ok;
      };
      const first = edit(source, 0.25);
      const second = edit(first.solid, -0.125);
      for (const [kind, query, count] of [
        ['face', 'getSolidFaces', 6], ['edge', 'getSolidEdges', 12], ['vertex', 'getSolidVertices', 8],
      ]) {
        const live = new Set(kernel[query](second.solid));
        const resolved = new Set();
        for (let index = 0; index < count; index++) {
          const reference = JSON.parse(kernel.resolveOperationOutput(first.op, kind, index));
          assert.equal(reference.status, 'bound', JSON.stringify(reference));
          assert.equal(reference.provenance, 'construction');
          assert.equal(reference.entities.length, 1);
          assert.equal(reference.entities[0].kind, kind);
          resolved.add(reference.entities[0].handle);
        }
        assert.deepEqual(resolved, live, `direct edit ${kind} refs, batch=${batch}`);
      }
      const before = kernel.journalSummary();
      const bytes = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(second.solid)));
      const face = kernel.getSolidFaces(second.solid)[0];
      if (batch) {
        const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([
          { op: 'moveFacesJournaled', args: { solid: second.solid, faces: [face], distance: -100 } },
        ])));
        assert.ok(response.error, 'collapsing move must refuse');
      } else {
        assert.throws(() => kernel.moveFacesJournaled(second.solid, Uint32Array.of(face), -100));
      }
      assert.equal(kernel.journalSummary(), before, 'refusal restores history');
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(second.solid)), bytes, 'refusal restores geometry');
    } finally {
      kernel.free();
    }
  }
  for (const batch of [false, true]) {
    const kernel = new BrepKernel();
    try {
      const wire = kernel.makeRegularPolygonWire(10, 500);
      const profile = kernel.makeFaceFromWire(wire);
      const source = kernel.extrude(profile, 0, 0, 1, 1);
      assert.equal(kernel.validateSolid(source), 0);
      const face = Array.from(kernel.getSolidFaces(source)).find(face => kernel.getFaceNormal(face)[2] > 0.9);
      const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(source)));
      for (const [faces, message] of [[Array(10001).fill(face), /faces must be at most/], [[face], /moveFaces topology work must be at most/]]) {
        if (batch) {
          const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([
            { op: 'moveFacesJournaled', args: { solid: source, faces, distance: 0.25 } },
          ])));
          assert.ok(response.error);
          assert.match(response.error.message, message);
        } else {
          assert.throws(() => kernel.moveFacesJournaled(source, Uint32Array.from(faces), 0.25), message);
        }
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(source)), before);
      }
    } finally { kernel.free(); }
  }
  for (const batch of [false, true]) {
    const kernel = new BrepKernel();
    try {
      const translated = (solid, x, y, z) => {
        kernel.transformSolid(solid, new Float64Array([1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1]));
        return solid;
      };
      const plate = kernel.makeBox(40, 40, 5);
      const boss = translated(kernel.makeBox(16, 16, 10), 12, 12, 5);
      const sharp = kernel.fuse(plate, boss);
      const bored = kernel.cut(sharp, translated(kernel.makeCylinder(3, 17), 20, 20, -1));
      const edge = Array.from(kernel.getSolidEdges(bored)).find(edge => {
        const p = kernel.getEdgeVertices(edge);
        return Math.abs(p[1] - 12) < 1e-8 && Math.abs(p[4] - 12) < 1e-8 &&
          Math.abs(p[2] - 15) < 1e-8 && Math.abs(p[5] - 15) < 1e-8;
      });
      assert.notEqual(edge, undefined);
      const source = kernel.filletV2(bored, Uint32Array.of(edge), 1);
      const sourceVolume = kernel.volume(source, 0.001);
      const top = solid => Array.from(kernel.getSolidFaces(solid))
        .filter(face => JSON.parse(kernel.getAnalyticSurfaceParams(face)).type === 'plane' && kernel.getFaceNormal(face)[2] > 0.9)
        .sort((a, b) => {
          const height = face => Math.max(...Array.from(kernel.getFaceEdges(face))
            .flatMap(edge => { const p = kernel.getEdgeVertices(edge); return [p[2], p[5]]; }));
          return height(b) - height(a);
        })[0];
      const edit = (solid, distance) => {
        const face = top(solid);
        if (!batch) return JSON.parse(kernel.moveFacesJournaled(solid, Uint32Array.of(face), distance));
        const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([
          {op: 'moveFacesJournaled', args: {solid, faces: [face], distance}},
        ])));
        assert.equal(response.error, undefined, JSON.stringify(response));
        return response.ok;
      };
      const first = edit(source, 2);
      const second = edit(first.solid, -1);
      for (const [result, distance] of [[first, 2], [second, 1]]) {
        assert.equal(kernel.validateSolid(result.solid), 0);
        const expected = sourceVolume + distance * (256 - Math.PI * 9);
        assert.ok(Math.abs(kernel.volume(result.solid, 0.001) - expected) <= Math.abs(expected) * 3e-4);
        assert.equal(JSON.parse(kernel.meshQuality(result.solid, 0.01)).isWatertight, true);
      }
      for (const [kind, query] of [['face','getSolidFaces'], ['edge','getSolidEdges'], ['vertex','getSolidVertices']]) {
        const actual = new Set();
        for (let index = 0; index < kernel[query](first.solid).length; index++) {
          const resolved = JSON.parse(kernel.resolveOperationOutput(first.op, kind, index));
          assert.equal(resolved.status, 'bound', JSON.stringify(resolved));
          assert.equal(resolved.provenance, 'construction');
          assert.equal(resolved.entities.length, 1);
          actual.add(resolved.entities[0].handle);
        }
        assert.deepEqual(actual, new Set(kernel[query](second.solid)), `blended ${kind}, batch=${batch}`);
      }
      const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(second.solid)));
      assert.throws(() => edit(second.solid, -100));
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(second.solid)), before);
    } finally { kernel.free(); }
  }
  console.log('ok - direct edit history: planar and blended references across successive edits, direct/batch rollback and work limits');
};


export const runSurfaceReplacementHistoryRegression = ({ BrepKernel, RemusIo }) => {
  const fixture = readFileSync(new URL('../crates/io/tests/data/jolly_fox_partial_cylinder.step', import.meta.url));
  const expectedVolume = radius => 73 * 41 * 8 + 12 * 30.5 * 12 + Math.PI * radius ** 2 * 12 / 4;
  const kinds = [['face', 'getSolidFaces'], ['edge', 'getSolidEdges'], ['vertex', 'getSolidVertices']];
  const replace = (kernel, solid, face, replacement, batch) => {
    if (!batch) return JSON.parse(kernel.replaceSurfaceJournaled(solid, face, JSON.stringify(replacement)));
    const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([
      { op: 'replaceSurfaceJournaled', args: { solid, face, replacement } },
    ])));
    assert.equal(response.error, undefined, JSON.stringify(response));
    return response.ok;
  };
  const references = (kernel, result) => kinds.flatMap(([kind, query]) =>
    Array.from(kernel[query](result.solid), (_, index) => ({
      kind, reference: kernel.makeOperationOutputRef(result.op, kind, index),
    })));
  const checkReferences = (kernel, solid, refs) => {
    for (const [kind, query] of kinds) {
      const resolved = new Set();
      for (const item of refs.filter(item => item.kind === kind)) {
        const result = JSON.parse(kernel.resolveRef(item.reference));
        assert.equal(result.status, 'bound', JSON.stringify(result));
        assert.equal(result.provenance, 'construction');
        assert.equal(result.entities.length, 1);
        assert.equal(result.entities[0].kind, kind);
        resolved.add(result.entities[0].handle);
      }
      assert.deepEqual(resolved, new Set(kernel[query](solid)), `${kind} replacement references`);
    }
  };
  const qualify = (kernel, solid, volume) => {
    const validation = JSON.parse(kernel.validateSolidDetailed(solid));
    assert.equal(validation.errorCount, 0);
    assert.equal(validation.warningCount, 0);
    assert.ok(Math.abs(kernel.volume(solid, 0.005) - volume) < 1e-5);
    for (const deflection of [0.005, 0.02]) {
      assert.equal(JSON.parse(kernel.meshQuality(solid, deflection)).isWatertight, true);
    }
  };
  const checkArena = (kernel, result, refs, volume) => {
    const restored = new BrepKernel();
    try {
      restored.makeBox(1, 1, 1);
      const [solid] = restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(result)));
      qualify(restored, solid, volume);
      checkReferences(restored, solid, refs);
    } finally { restored.free(); }
  };
  for (const batch of [false, true]) {
    const kernel = new BrepKernel();
    try {
      const top = solid => Array.from(kernel.getSolidFaces(solid)).find(face => kernel.getFaceNormal(face)[2] > 0.9);
      const source = kernel.makeBox(3, 5, 7);
      const first = replace(kernel, source, top(source), { type: 'plane', normal: [0, 0, 1], d: 8 }, batch);
      const refs = references(kernel, first);
      const second = replace(kernel, first.solid, top(first.solid), { type: 'plane', normal: [-0.1, 0, 1], d: 7.85 }, batch);
      qualify(kernel, second.solid, 120);
      checkReferences(kernel, second.solid, refs);
      checkArena(kernel, second.solid, refs, 120);
    } finally { kernel.free(); }
    const budgetKernel = new BrepKernel();
    try {
      const wire = budgetKernel.makeRegularPolygonWire(10, 500);
      const profile = budgetKernel.makeFaceFromWire(wire);
      const source = budgetKernel.extrude(profile, 0, 0, 1, 1);
      assert.equal(budgetKernel.validateSolid(source), 0);
      const face = Array.from(budgetKernel.getSolidFaces(source)).find(face => budgetKernel.getFaceNormal(face)[2] > 0.9);
      const before = Uint8Array.from(budgetKernel.serializeSolids(Uint32Array.of(source)));
      const replacement = { type: 'plane', normal: [0, 0, 1], d: 1.25 };
      if (batch) {
        const [response] = JSON.parse(budgetKernel.executeBatchV2(JSON.stringify([
          { op: 'replaceSurfaceJournaled', args: { solid: source, face, replacement } },
        ])));
        assert.ok(response.error);
        assert.match(response.error.message, /moveFaces topology work must be at most/);
      } else {
        assert.throws(() => budgetKernel.replaceSurfaceJournaled(source, face, JSON.stringify(replacement)), /moveFaces topology work must be at most/);
      }
      assert.deepEqual(budgetKernel.serializeSolids(Uint32Array.of(source)), before);
    } finally { budgetKernel.free(); }
    for (const radius of [20, 22, 28]) {
      const kernel = new BrepKernel();
      const io = new RemusIo();
      try {
        const [source] = kernel.deserializeSolids(io.importStep(fixture));
        const wall = solid => Array.from(kernel.getSolidFaces(solid)).find(face => kernel.getSurfaceType(face) === 'cylinder');
        const replacement = (solid, radius) => ({ ...JSON.parse(kernel.getAnalyticSurfaceParams(wall(solid))), radius });
        const first = replace(kernel, source, wall(source), replacement(source, 21), batch);
        const refs = references(kernel, first);
        const second = replace(kernel, first.solid, wall(first.solid), replacement(first.solid, radius), batch);
        qualify(kernel, second.solid, expectedVolume(radius));
        assert.ok(Math.abs(JSON.parse(kernel.getAnalyticSurfaceParams(wall(source))).radius - 20.5) < 1e-9);
        checkReferences(kernel, second.solid, refs);
        checkArena(kernel, second.solid, refs, expectedVolume(radius));
        const roundTrip = new BrepKernel();
        try {
          const [solid] = roundTrip.deserializeSolids(io.importStep(io.exportStep(kernel.serializeSolids(Uint32Array.of(second.solid)))));
          qualify(roundTrip, solid, expectedVolume(radius));
        } finally { roundTrip.free(); }
        const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(second.solid)));
        const journal = kernel.journalSummary();
        const invalid = replacement(second.solid, 32);
        if (batch) {
          const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([
            { op: 'replaceSurfaceJournaled', args: { solid: second.solid, face: wall(second.solid), replacement: invalid } },
          ])));
          assert.ok(response.error);
          assert.match(response.error.message, /quarter-wall sweep may contact/);
        } else {
          assert.throws(() => kernel.replaceSurfaceJournaled(second.solid, wall(second.solid), JSON.stringify(invalid)), /quarter-wall sweep may contact/);
        }
        assert.equal(kernel.journalSummary(), journal);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(second.solid)), before);
        checkReferences(kernel, second.solid, refs);
      } finally { kernel.free(); io.free(); }
    }
  }
  console.log('ok - replacement history: plane and quarter-wall edits, all entity refs, arena/STEP round trips, rollback');
};


export const runCylindricalRadiusHistoryRegression = ({ BrepKernel, RemusIo }) => {
  const kinds = [['face', 'getSolidFaces'], ['edge', 'getSolidEdges'], ['vertex', 'getSolidVertices']];
  const wall = (kernel, solid) => Array.from(kernel.getSolidFaces(solid)).find(face => kernel.getSurfaceType(face) === 'cylinder');
  const resize = (kernel, solid, radius, batch) => {
    const face = wall(kernel, solid);
    if (!batch) return JSON.parse(kernel.resizeCylindricalFaceJournaled(solid, face, radius));
    const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([{ op: 'resizeCylindricalFaceJournaled', args: { solid, face, radius } }])));
    assert.equal(response.error, undefined, JSON.stringify(response));
    return response.ok;
  };
  const references = (kernel, result) => kinds.flatMap(([kind, query]) => Array.from(kernel[query](result.solid), (_, index) => ({kind, reference: kernel.makeOperationOutputRef(result.op, kind, index)})));
  const qualifyRefs = (kernel, solid, refs, complete) => {
    for (const [kind, query] of kinds) {
      const live = new Set(kernel[query](solid));
      const resolved = new Set();
      for (const item of refs.filter(item => item.kind === kind)) {
        const result = JSON.parse(kernel.resolveRef(item.reference));
        if (!complete && result.status === 'dangling') continue;
        assert.ok(['bound', 'boundMany'].includes(result.status), JSON.stringify(result));
        assert.equal(result.provenance, 'construction');
        for (const entity of result.entities) {
          assert.equal(entity.kind, kind);
          assert.ok(live.has(entity.handle));
          resolved.add(entity.handle);
        }
      }
      if (complete) assert.deepEqual(resolved, live);
    }
  };
  const qualify = (kernel, solid, expected) => {
    assert.equal(kernel.validateSolid(solid), 0);
    assert.ok(Math.abs(kernel.volume(solid, 0.005) - expected) < expected * 1e-5);
    assert.equal(JSON.parse(kernel.meshQuality(solid, 0.005)).isWatertight, true);
  };
  const fixture = readFileSync(new URL('../crates/io/tests/data/jolly_fox_partial_cylinder.step', import.meta.url));
  for (const batch of [false, true]) for (const [type, radius] of [['boss', 3], ['boss', 8], ['bore', 2], ['bore', 5], ['quarter', 22]]) {
    const kernel = new BrepKernel();
    const io = new RemusIo();
    try {
      let source;
      if (type === 'quarter') [source] = kernel.deserializeSolids(io.importStep(fixture));
      else {
        const plate = kernel.makeBox(40, 40, 10);
        const cylinder = kernel.makeCylinder(type === 'boss' ? 5 : 3, 10);
        const z = type === 'boss' ? 10 : 0;
        kernel.transformSolid(cylinder, new Float64Array([1,0,0,20,0,1,0,20,0,0,1,z,0,0,0,1]));
        source = type === 'boss' ? kernel.fuse(plate, cylinder) : kernel.cut(plate, cylinder);
      }
      const expected = radius => type === 'quarter' ? 73 * 41 * 8 + 12 * 30.5 * 12 + Math.PI * radius ** 2 * 12 / 4 : 16000 + (type === 'boss' ? 1 : -1) * Math.PI * radius ** 2 * 10;
      const sourceVolume = kernel.volume(source, 0.005);
      const first = resize(kernel, source, radius, batch);
      const firstRefs = references(kernel, first);
      qualify(kernel, first.solid, expected(radius));
      qualifyRefs(kernel, first.solid, firstRefs, true);
      const finalRadius = type === 'quarter' ? 20 : type === 'boss' ? 5 : 3;
      const second = resize(kernel, first.solid, finalRadius, batch);
      const refs = references(kernel, second);
      qualify(kernel, second.solid, expected(finalRadius));
      qualifyRefs(kernel, second.solid, firstRefs, false);
      qualifyRefs(kernel, second.solid, refs, true);
      qualify(kernel, source, sourceVolume);
      const restored = new BrepKernel();
      try {
        restored.makeBox(1, 1, 1);
        const [solid] = restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(second.solid)));
        qualify(restored, solid, expected(finalRadius));
        qualifyRefs(restored, solid, refs, true);
        qualifyRefs(restored, solid, firstRefs, false);
      } finally { restored.free(); }
      const roundTrip = new BrepKernel();
      try {
        const [solid] = roundTrip.deserializeSolids(io.importStep(io.exportStep(kernel.serializeSolids(Uint32Array.of(second.solid)))));
        qualify(roundTrip, solid, expected(finalRadius));
      } finally { roundTrip.free(); }
      for (const invalid of type === 'bore' ? [0, 20, 25] : type === 'quarter' ? [0, 32] : [0]) {
        const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(second.solid)));
        const journal = kernel.journalSummary();
        assert.throws(() => resize(kernel, second.solid, invalid, batch));
        assert.equal(kernel.journalSummary(), journal);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(second.solid)), before);
      }
    } finally { kernel.free(); io.free(); }
  }
  console.log('ok - cylindrical radius history: successive bore/boss/quarter edits, all output refs, arena/STEP, rollback');
};


export const runDraftHistoryRegression = ({ BrepKernel, RemusIo }) => {
  const kinds = [['face','getSolidFaces'], ['edge','getSolidEdges'], ['vertex','getSolidVertices']];
  const wall = (kernel, solid) => Array.from(kernel.getSolidFaces(solid)).find(face =>
    kernel.getSurfaceType(face) === 'plane' && kernel.getFaceNormal(face)[0] > 0.9);
  const edit = (kernel, solid, angleDegrees, batch) => {
    const faces = [wall(kernel, solid)];
    if (!batch) return JSON.parse(kernel.draftJournaled(solid, Uint32Array.from(faces), Float64Array.of(0,0,1), Float64Array.of(0,0,0), angleDegrees));
    const [result] = JSON.parse(kernel.executeBatchV2(JSON.stringify([{op:'draftJournaled', args:{solid,faces,pullDirection:[0,0,1],neutralPoint:[0,0,0],angleDegrees}}])));
    assert.equal(result.error, undefined, JSON.stringify(result));
    return result.ok;
  };
  const references = (kernel, result) => kinds.flatMap(([kind, query]) => Array.from(kernel[query](result.solid), (_, index) => ({kind, reference:kernel.makeOperationOutputRef(result.op,kind,index)})));
  const checkRefs = (kernel, solid, refs) => {
    for (const [kind, query] of kinds) {
      const resolved = new Set();
      for (const item of refs.filter(item => item.kind === kind)) {
        const result = JSON.parse(kernel.resolveRef(item.reference));
        assert.equal(result.status,'bound',JSON.stringify(result));
        assert.equal(result.provenance,'construction');
        assert.equal(result.entities.length,1);
        assert.equal(result.entities[0].kind,kind);
        resolved.add(result.entities[0].handle);
      }
      assert.deepEqual(resolved,new Set(kernel[query](solid)));
    }
  };
  const qualify = (kernel, solid, expected, scale) => {
    assert.equal(kernel.validateSolid(solid),0);
    const volume = kernel.volume(solid,0.005*scale);
    assert.ok(Math.abs(volume-expected)<Math.abs(expected)*1e-5);
    assert.equal(JSON.parse(kernel.meshQuality(solid,0.005*scale)).isWatertight,true);
    const mesh = kernel.tessellateSolid(solid,0.005*scale);
    const origin = mesh.positions.slice(0,3);
    const point = index => [0,1,2].map(axis => mesh.positions[index*3+axis]-origin[axis]);
    let independent = 0;
    for(let i=0;i<mesh.indices.length;i+=3) {
      const a=point(mesh.indices[i]), b=point(mesh.indices[i+1]), c=point(mesh.indices[i+2]);
      independent += (a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6;
    }
    assert.ok(Math.abs(independent-volume)<Math.abs(volume)*2e-3);
  };
  for (const batch of [false,true]) for (const bore of [false,true]) for (const scale of [0.001,1,1000]) {
    const kernel = new BrepKernel();
    const io = new RemusIo();
    try {
      let source = kernel.makeBox(10*scale,10*scale,10*scale);
      if(bore) {
        const tool=kernel.makeCylinder(scale,10*scale);
        kernel.transformSolid(tool,new Float64Array([1,0,0,5*scale,0,1,0,5*scale,0,0,1,0,0,0,0,1]));
        source=kernel.cut(source,tool);
      }
      const sourceVolume=(1000-(bore?Math.PI*10:0))*scale**3;
      const first=edit(kernel,source,5,batch);
      const refs=references(kernel,first);
      qualify(kernel,first.solid,sourceVolume+500*scale**3*Math.tan(5*Math.PI/180),scale);
      checkRefs(kernel,first.solid,refs);
      const second=edit(kernel,first.solid,-2,batch);
      const secondVolume=kernel.volume(second.solid,0.005*scale);
      qualify(kernel,second.solid,secondVolume,scale);
      qualify(kernel,source,sourceVolume,scale);
      checkRefs(kernel,second.solid,refs);
      const restored=new BrepKernel();
      try {
        restored.makeBox(1,1,1);
        const [solid]=restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(second.solid)));
        qualify(restored,solid,secondVolume,scale);
        checkRefs(restored,solid,refs);
      } finally { restored.free(); }
      const roundTrip=new BrepKernel();
      try {
        const importSolid = solid => io.importStep(io.exportStep(kernel.serializeSolids(Uint32Array.of(solid))));
        if (bore && scale === 1000) {
          // The unchanged source also exceeds the importer's fixed circle-sampling budget.
          const limit = /FACE_BOUND curve exceeds the adaptive planar-classification sampling limit/;
          assert.throws(() => importSolid(source), limit);
          assert.throws(() => importSolid(second.solid), limit);
        } else {
          const [solid]=roundTrip.deserializeSolids(importSolid(second.solid));
          qualify(roundTrip,solid,secondVolume,scale);
        }
      } finally { roundTrip.free(); }
      const before=Uint8Array.from(kernel.serializeSolids(Uint32Array.of(second.solid)));
      const journal=kernel.journalSummary();
      for(const angle of [0,-80]) {
        assert.throws(()=>edit(kernel,second.solid,angle,batch));
        assert.equal(kernel.journalSummary(),journal);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(second.solid)),before);
        checkRefs(kernel,second.solid,refs);
      }
    } finally { kernel.free(); io.free(); }
  }
  console.log('ok - draft history: degree units, successive plain/bored edits at three scales, all refs, arena, qualified STEP or explicit large-bore sampling refusal, rollback');
};

export const runDefeatureHistoryRegression = ({BrepKernel, RemusIo}) => {
  const kinds = [['face','getSolidFaces'],['edge','getSolidEdges'],['vertex','getSolidVertices']];
  const refs = (kernel, result, selectedKinds=kinds) => selectedKinds.flatMap(([kind,query]) => Array.from(kernel[query](result.solid), (_,index) => ({kind,ref:kernel.makeOperationOutputRef(result.op,kind,index)})));
  const checkRefs = (kernel, solid, references, allowDeleted, selectedKinds=kinds) => {
    for (const [kind,query] of selectedKinds) {
      const bound = new Set();
      for (const item of references.filter(item=>item.kind===kind)) {
        const resolution=JSON.parse(kernel.resolveRef(item.ref));
        if(allowDeleted && resolution.status==='dangling') continue;
        assert.equal(resolution.status,'bound',JSON.stringify(resolution));
        assert.equal(resolution.provenance,'construction');
        for(const entity of resolution.entities) {assert.equal(entity.kind,kind);bound.add(entity.handle);}
      }
      assert.deepEqual(bound,new Set(kernel[query](solid)));
    }
  };
  const qualify = (kernel,solid,expected,scale) => {
    assert.equal(kernel.validateSolid(solid),0);
    assert.ok(Math.abs(kernel.volume(solid,0.005*scale)-expected)<Math.abs(expected)*1e-6);
    assert.equal(JSON.parse(kernel.meshQuality(solid,0.005*scale)).isWatertight,true);
  };
  for(const batch of [false,true]) for(const type of ['bore','chamfer','fillet','rim','cone']) for(const scale of [0.001,1,10]) {
    const kernel=new BrepKernel(), io=new RemusIo();
    try {
      const move=(solid,x,y,z)=>{kernel.transformSolid(solid,new Float64Array([1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1]));return solid;};
      let source, anchor, expected;
      if(type==='bore') {
        source=kernel.cut(kernel.makeBox(10*scale,10*scale,10*scale),move(kernel.makeCylinder(scale,10*scale),5*scale,5*scale,0));
        expected=1000*scale**3;
      } else {
        let sharp,height,radius;
        if(type==='cone') {
          sharp=kernel.fuse(kernel.makeCylinder(3,5),move(kernel.makeCone(3,1,4),0,0,5));
          height=5;radius=0.25;expected=Math.PI*187/3*scale**3;
        } else if(type==='rim') {
          sharp=kernel.makeCylinder(10*scale,20*scale);height=20*scale;radius=2*scale;expected=Math.PI*2000*scale**3;
        } else {
          sharp=kernel.makeBox(10*scale,10*scale,10*scale);height=10*scale;radius=2*scale;expected=1000*scale**3;
        }
        const edge=Array.from(kernel.getSolidEdges(sharp)).find(edge=>{
          const p=kernel.getEdgeVertices(edge);
          return Math.abs(p[2]-height)<1e-7*scale && Math.abs(p[5]-height)<1e-7*scale;
        });
        assert.notEqual(edge,undefined);
        anchor=JSON.parse(type==='chamfer'?kernel.chamferJournaled(sharp,Uint32Array.of(edge),radius,radius):kernel.filletJournaled(sharp,Uint32Array.of(edge),radius));
        assert.equal(anchor.isPartial,false);
        source=anchor.solid;
        if(type==='cone' && scale!==1) {
          // Direct fillet creation at 10x fails on the unchanged parent; scale the exact valid fixture.
          kernel.transformSolid(source,new Float64Array([scale,0,0,0,0,scale,0,0,0,0,scale,0,0,0,0,1]));
          anchor=null;
        }
      }
      const oldRefs=anchor?refs(kernel,anchor,kinds.slice(0,1)):null;
      const faces=Array.from(kernel.getSolidFaces(source)).filter(face=>{
        const carrier=kernel.getSurfaceType(face);
        if(type==='chamfer') return carrier==='plane' && Array.from(kernel.getFaceNormal(face)).filter(value=>Math.abs(value)>1e-6).length>1;
        return carrier===(type==='bore'||type==='fillet'?'cylinder':'torus');
      });
      assert.equal(faces.length,1);
      const sourceVolume=kernel.volume(source,0.005*scale);
      const edit=(solid,faces,active=kernel)=>{
        if(!batch)return JSON.parse(active.defeatureJournaled(solid,Uint32Array.from(faces)));
        const [response]=JSON.parse(active.executeBatchV2(JSON.stringify([{op:'defeatureJournaled',args:{solid,faces}}])));
        assert.equal(response.error,undefined,JSON.stringify(response));return response.ok;
      };
      const result=edit(source,faces);
      qualify(kernel,result.solid,expected,scale);
      assert.ok(Math.abs(kernel.volume(source,0.005*scale)-sourceVolume)<Math.abs(sourceVolume)*1e-10);
      if(oldRefs)checkRefs(kernel,result.solid,oldRefs,true,kinds.slice(0,1));
      const outputRefs=refs(kernel,result);
      checkRefs(kernel,result.solid,outputRefs,false);
      if(['bore','chamfer','fillet'].includes(type)) {
        const draftKernel=new BrepKernel();
        try {
          const [solid]=draftKernel.deserializeSolids(kernel.serializeSolids(Uint32Array.of(result.solid)));
          const wall=Array.from(draftKernel.getSolidFaces(solid)).find(face=>draftKernel.getFaceNormal(face)[0]>0.9);
          const next=JSON.parse(draftKernel.draftJournaled(solid,Uint32Array.of(wall),Float64Array.of(0,0,1),Float64Array.of(0,0,0),5));
          qualify(draftKernel,next.solid,expected+500*scale**3*Math.tan(5*Math.PI/180),scale);
          checkRefs(draftKernel,next.solid,outputRefs,false);
        } finally {draftKernel.free();}
      }
      const restored=new BrepKernel();
      try {
        restored.makeBox(1,1,1);
        const [solid]=restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(result.solid)));
        qualify(restored,solid,expected,scale);
        checkRefs(restored,solid,outputRefs,false);
        if(oldRefs)checkRefs(restored,solid,oldRefs,true,kinds.slice(0,1));
      } finally {restored.free();}
      const roundTrip=new BrepKernel();
      try {
        const [solid]=roundTrip.deserializeSolids(io.importStep(io.exportStep(kernel.serializeSolids(Uint32Array.of(result.solid)))));
        qualify(roundTrip,solid,expected,scale);
      } finally {roundTrip.free();}
      const imported=new BrepKernel();
      try {
        const [solid]=imported.deserializeSolids(io.importStep(io.exportStep(kernel.serializeSolids(Uint32Array.of(source)))));
        const selection=Array.from(imported.getSolidFaces(solid)).filter(face=>{
          const carrier=imported.getSurfaceType(face);
          if(type==='chamfer')return carrier==='plane' && Array.from(imported.getFaceNormal(face)).filter(value=>Math.abs(value)>1e-6).length>1;
          return carrier===(type==='bore'||type==='fillet'?'cylinder':'torus');
        });
        assert.equal(selection.length,1);
        const healed=edit(solid,selection,imported);
        qualify(imported,healed.solid,expected,scale);
        checkRefs(imported,healed.solid,refs(imported,healed),false);
      } finally {imported.free();}
      const before=Uint8Array.from(kernel.serializeSolids(Uint32Array.of(result.solid))), journal=kernel.journalSummary();
      for(const invalid of [[],faces,Array.from(kernel.getSolidFaces(result.solid))]) {
        assert.throws(()=>edit(result.solid,invalid));
        assert.equal(kernel.journalSummary(),journal);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(result.solid)),before);
      }
    } catch(error) {throw new Error(`defeature ${type} batch=${batch} scale=${scale}: ${error.message}`,{cause:error});}
    finally {kernel.free();io.free();}
  }
  console.log('ok - defeature history: 30 direct/batch scale cells, retained/merged/deleted refs, arena/STEP, rollback');
};

export const runHealingHistoryRegression = ({ BrepKernel, RemusIo }) => {
  const kinds = [['face', 'getSolidFaces'], ['edge', 'getSolidEdges'], ['vertex', 'getSolidVertices']];
  const references = (kernel, result) => kinds.flatMap(([kind, query]) =>
    Array.from(kernel[query](result.solid), (_, index) => ({ kind, ref: kernel.makeOperationOutputRef(result.op, kind, index) })));
  const check = (kernel, solid, refs, expected, scale) => {
    assert.equal(kernel.validateSolid(solid), 0);
    assert.ok(Math.abs(kernel.volume(solid, 0.005 * scale) - expected) < expected * 1e-6);
    assert.equal(JSON.parse(kernel.meshQuality(solid, 0.005 * scale)).isWatertight, true);
    for (const [kind, query] of kinds) {
      const bound = new Set();
      for (const item of refs.filter(item => item.kind === kind)) {
        const result = JSON.parse(kernel.resolveRef(item.ref));
        assert.equal(result.status, 'bound', JSON.stringify(result));
        assert.equal(result.provenance, 'construction');
        for (const entity of result.entities) bound.add(entity.handle);
      }
      assert.deepEqual(bound, new Set(kernel[query](solid)));
    }
  };
  for (const scale of [0.001, 1, 10]) for (const batch of [false, true]) for (const pipeline of [false, true]) {
    const kernel = new BrepKernel(), io = new RemusIo(), restored = new BrepKernel(), imported = new BrepKernel();
    try {
      const source = kernel.makeBox(10 * scale, 10 * scale, 10 * scale);
      const document = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(source))));
      const edge = document.edges[0];
      document.vertices.push(structuredClone(document.vertices[edge.start]));
      edge.start = document.vertices.length - 1;
      const [solid] = kernel.deserializeSolids(new TextEncoder().encode(JSON.stringify(document)));
      assert.equal(kernel.getSolidVertices(solid).length, 9);
      const edit = (active, solid) => {
        const steps = ['fix_shape', 'merge_vertices', 'fix_shape'];
        if (!batch) return JSON.parse(pipeline ? active.runHealPipelineJournaled(solid, steps) : active.fixShapeWithConfigJournaled(solid, '{}'));
        const op = pipeline ? 'runHealPipelineJournaled' : 'fixShapeWithConfigJournaled';
        const [response] = JSON.parse(active.executeBatchV2(JSON.stringify([{ op, args: { solid, steps, configJson: '{}' } }])));
        assert.equal(response.error, undefined, JSON.stringify(response));
        return response.ok;
      };
      const first = edit(kernel, solid), refs = references(kernel, first), expected = 1000 * scale ** 3;
      assert.equal(first.verified, true);
      assert.equal(kernel.getSolidVertices(first.solid).length, 8);
      assert.ok(pipeline ? first.steps.some(step => step.actionsTaken > 0) : first.actionsTaken > 0);
      assert.ok(Number.isSafeInteger(first.op));
      const second = edit(kernel, first.solid);
      check(kernel, second.solid, refs, expected, scale);
      restored.makeBox(1, 1, 1);
      const [copy] = restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(second.solid)));
      check(restored, copy, refs, expected, scale);
      const wall = Array.from(restored.getSolidFaces(copy)).find(face => restored.getFaceNormal(face)[0] > 0.9);
      const drafted = JSON.parse(restored.draftJournaled(copy, Uint32Array.of(wall), Float64Array.of(0, 0, 1), Float64Array.of(0, 0, 0), 5));
      check(restored, drafted.solid, refs, expected + 500 * scale ** 3 * Math.tan(5 * Math.PI / 180), scale);
      const [stepSolid] = imported.deserializeSolids(io.importStep(io.exportStep(kernel.serializeSolids(Uint32Array.of(second.solid)))));
      const healedImport = edit(imported, stepSolid);
      check(imported, healedImport.solid, references(imported, healedImport), expected, scale);
      const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(second.solid))), journal = kernel.journalSummary();
      assert.throws(() => kernel.fixShapeWithConfigJournaled(second.solid, '{"tolerance":-1}'));
      assert.throws(() => kernel.runHealPipelineJournaled(second.solid, ['fix_shape', 'missing']));
      assert.equal(kernel.journalSummary(), journal);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(second.solid)), before);
    } catch (error) {
      throw new Error(`healing history scale=${scale} batch=${batch} pipeline=${pipeline}: ${error.message}`, { cause: error });
    } finally { kernel.free(); io.free(); restored.free(); imported.free(); }
  }
  console.log('ok - healing history: 12 direct/batch cells, repeated repairs, arena, later draft, STEP import, rollback');
};

export const runUnifyHistoryRegression = ({ BrepKernel }) => {
  const kinds = [['face', 'getSolidFaces'], ['edge', 'getSolidEdges'], ['vertex', 'getSolidVertices']];
  for (const scale of [0.001, 1, 10]) for (const batch of [false, true]) {
    const kernel = new BrepKernel(), restored = new BrepKernel();
    try {
      const cube = kernel.makeBox(10 * scale, 10 * scale, 10 * scale);
      const document = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(cube))));
      const shell = document.shells[document.solids[document.solid_roots[0]].outer_shell];
      const original = document.faces[shell.faces[0]], boundary = document.wires[original.outer_wire].edges;
      const vertices = boundary.map(use => document.edges[use.edge][use.forward ? 'start' : 'end']);
      const point = document.vertices[vertices[0]].point.map((v, i) => (v + document.vertices[vertices[2]].point[i]) / 2);
      const center = document.vertices.push({ point, tolerance: 1e-7 }) - 1;
      const spokes = vertices.map(vertex => document.edges.push({ start: center, end: vertex, curve: 'Line', tolerance: null }) - 1);
      const triangles = boundary.map((use, i) => {
        const wire = document.wires.push({ edges: [use, { edge: spokes[(i + 1) % 4], forward: false }, { edge: spokes[i], forward: true }], closed: true }) - 1;
        return document.faces.push({ ...structuredClone(original), outer_wire: wire }) - 1;
      });
      shell.faces.splice(0, 1, ...triangles);
      // Exercise the supported legacy reader, which reconstructs coedge authority.
      document.version = 2;
      document.pcurves = [];
      delete document.boundary_authority;
      delete document.journal;
      delete document.attributes;
      const [source] = kernel.deserializeSolids(new TextEncoder().encode(JSON.stringify(document)));
      assert.equal(kernel.getSolidFaces(source).length, 9);
      assert.equal(kernel.getSolidVertices(source).length, 9);
      const anchor = JSON.parse(kernel.fixShapeWithConfigJournaled(source, '{}'));
      assert.equal(kernel.getSolidFaces(anchor.solid).length, 9);
      const refs = kinds.flatMap(([kind, query]) => Array.from(kernel[query](anchor.solid), (_, index) => ({ kind, ref: kernel.makeOperationOutputRef(anchor.op, kind, index) })));
      const edit = (active, solid) => {
        if (!batch) return JSON.parse(active.runHealPipelineJournaled(solid, ['unify_same_domain']));
        const [response] = JSON.parse(active.executeBatchV2(JSON.stringify([{ op: 'runHealPipelineJournaled', args: { solid, steps: ['unify_same_domain'] } }])));
        assert.equal(response.error, undefined, JSON.stringify(response));
        return response.ok;
      };
      const result = edit(kernel, anchor.solid);
      const check = (active, solid, expected) => {
        let deleted = 0;
        for (const [kind, query] of kinds) {
          const bound = new Set();
          for (const item of refs.filter(item => item.kind === kind)) {
            const resolution = JSON.parse(active.resolveRef(item.ref));
            if (resolution.status === 'dangling') { deleted++; continue; }
            assert.equal(resolution.status, 'bound', JSON.stringify(resolution));
            assert.equal(resolution.provenance, 'construction');
            for (const entity of resolution.entities) bound.add(entity.handle);
          }
          assert.deepEqual(bound, new Set(active[query](solid)));
        }
        assert.equal(deleted, 5);
        assert.equal(active.validateSolid(solid), 0);
        assert.ok(Math.abs(active.volume(solid, 0.005 * scale) - expected) < expected * 1e-6);
        assert.equal(JSON.parse(active.meshQuality(solid, 0.005 * scale)).isWatertight, true);
      };
      check(kernel, result.solid, 1000 * scale ** 3);
      restored.makeBox(1, 1, 1);
      const [copy] = restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(result.solid)));
      const repeated = edit(restored, copy);
      check(restored, repeated.solid, 1000 * scale ** 3);
      const wall = Array.from(restored.getSolidFaces(repeated.solid)).find(face => restored.getFaceNormal(face)[0] > 0.9);
      const drafted = JSON.parse(restored.draftJournaled(repeated.solid, Uint32Array.of(wall), Float64Array.of(0, 0, 1), Float64Array.of(0, 0, 0), 5));
      check(restored, drafted.solid, (1000 + 500 * Math.tan(5 * Math.PI / 180)) * scale ** 3);
    } catch (error) {
      throw new Error(`unify history scale=${scale} batch=${batch}: ${error.message}`, { cause: error });
    } finally { kernel.free(); restored.free(); }
  }
  console.log('ok - unify history: six direct/batch scale cells, merged/deleted references, legacy import, arena and later draft');
};

export const runSewingHistoryRegression = ({ BrepKernel }) => {
  const kinds = [['face', 'getSolidFaces'], ['edge', 'getSolidEdges'], ['vertex', 'getSolidVertices']];
  for (const operator of ['sew_shells', 'fix_wireframe']) for (const scale of [0.001, 1, 10]) for (const batch of [false, true]) {
    const kernel = new BrepKernel(), restored = new BrepKernel();
    try {
      const cube = kernel.makeBox(10 * scale, 10 * scale, 10 * scale);
      const document = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(cube))));
      const shell = document.shells[document.solids[document.solid_roots[0]].outer_shell];
      const sourceEdges = [], sourceVertices = [];
      for (const faceId of shell.faces) {
        const face = document.faces[faceId], vertices = new Map();
        const edges = document.wires[face.outer_wire].edges.map(use => {
          const edge = structuredClone(document.edges[use.edge]);
          for (const key of ['start', 'end']) {
            if (!vertices.has(edge[key])) vertices.set(edge[key], document.vertices.push(structuredClone(document.vertices[edge[key]])) - 1);
            edge[key] = vertices.get(edge[key]);
          }
          const id = document.edges.push(edge) - 1;
          sourceEdges.push(id);
          return { edge: id, forward: use.forward };
        });
        sourceVertices.push(...vertices.values());
        face.outer_wire = document.wires.push({ edges, closed: true }) - 1;
      }
      // Exercise the supported legacy reader, which reconstructs coedge authority.
      document.version = 2;
      document.pcurves = [];
      delete document.boundary_authority;
      delete document.attributes;
      // Explicit fixture construction anchors include the unsewn source entities.
      // The repair boundary still has to independently verify the sewn solid.
      const index = [['face', shell.faces], ['edge', sourceEdges], ['vertex', sourceVertices]]
        .flatMap(([kind, locals]) => locals.map(local => ({ kind, local })))
        .map((entry, ordinal) => ({ ...entry, ordinal }));
      document.journal = { next_op: 1, next_ordinal: index.length, index,
        entries: [{ op: 0, kind: 'disjoint_face_fixture', payload: 'Evolution', construction: true,
          scope: index.map(entry => entry.ordinal), events: index.map(entry => [entry.ordinal, { event: 'Generated', sources: [] }]) }] };
      const [source] = kernel.deserializeSolids(new TextEncoder().encode(JSON.stringify(document)));
      assert.equal(kernel.getSolidFaces(source).length, 6);
      assert.equal(kernel.getSolidEdges(source).length, 24);
      assert.equal(kernel.getSolidVertices(source).length, 24);
      const anchor = { solid: source, op: 0 };
      const refs = kinds.flatMap(([kind, query]) => Array.from(kernel[query](anchor.solid), (_, index) => ({ kind, ref: kernel.makeOperationOutputRef(anchor.op, kind, index) })));
      const edit = (active, solid) => {
        if (!batch) return JSON.parse(active.runHealPipelineJournaled(solid, [operator]));
        const [response] = JSON.parse(active.executeBatchV2(JSON.stringify([{ op: 'runHealPipelineJournaled', args: { solid, steps: [operator] } }])));
        assert.equal(response.error, undefined, JSON.stringify(response));
        return response.ok;
      };
      const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(source))), journal = kernel.journalSummary();
      assert.throws(() => kernel.runHealPipelineJournaled(source, [operator, 'missing']));
      assert.equal(kernel.journalSummary(), journal);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(source)), before);
      const result = edit(kernel, anchor.solid);
      assert.equal(result.verified, true);
      assert.equal(result.steps[0].actionsTaken, 12);
      const check = (active, solid, expected) => {
        let deleted = 0;
        for (const [kind, query] of kinds) {
          const bound = new Set();
          for (const item of refs.filter(item => item.kind === kind)) {
            const resolution = JSON.parse(active.resolveRef(item.ref));
            if (resolution.status === 'dangling') { deleted++; continue; }
            assert.equal(resolution.status, 'bound', JSON.stringify(resolution));
            assert.equal(resolution.provenance, 'construction');
            for (const entity of resolution.entities) bound.add(entity.handle);
          }
          assert.deepEqual(bound, new Set(active[query](solid)));
        }
        assert.equal(deleted, 0);
        assert.equal(active.validateSolid(solid), 0);
        assert.ok(Math.abs(active.volume(solid, 0.005 * scale) - expected) < expected * 1e-6);
        assert.equal(JSON.parse(active.meshQuality(solid, 0.005 * scale)).isWatertight, true);
      };
      check(kernel, result.solid, 1000 * scale ** 3);
      restored.makeBox(1, 1, 1);
      const [copy] = restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(result.solid)));
      const repeated = edit(restored, copy);
      check(restored, repeated.solid, 1000 * scale ** 3);
      const wall = Array.from(restored.getSolidFaces(repeated.solid)).find(face => restored.getFaceNormal(face)[0] > 0.9);
      const drafted = JSON.parse(restored.draftJournaled(repeated.solid, Uint32Array.of(wall), Float64Array.of(0, 0, 1), Float64Array.of(0, 0, 0), 5));
      check(restored, drafted.solid, (1000 + 500 * Math.tan(5 * Math.PI / 180)) * scale ** 3);
    } catch (error) {
      throw new Error(`${operator} history scale=${scale} batch=${batch}: ${error.message}`, { cause: error });
    } finally { kernel.free(); restored.free(); }
  }
  console.log('ok - sewing/wireframe history: twelve direct/batch scale cells, merged edge/vertex references, legacy import, arena and later draft');
};

export const runInnerWireHistoryRegression = ({ BrepKernel }) => {
  const kinds = [['face', 'getSolidFaces'], ['edge', 'getSolidEdges'], ['vertex', 'getSolidVertices']];
  for (const scale of [0.001, 1, 10]) for (const batch of [false, true]) {
    const kernel = new BrepKernel(), restored = new BrepKernel();
    try {
      const refusal = new BrepKernel();
      try {
        const box = refusal.makeBox(10 * scale, 10 * scale, 10 * scale), tool = refusal.makeCylinder(scale, 10 * scale);
        refusal.transformSolid(tool, Float64Array.of(1,0,0,5*scale,0,1,0,5*scale,0,0,1,0,0,0,0,1));
        const bore = refusal.cut(box, tool);
        assert.equal(refusal.validateSolid(bore), 0);
        const bytes = Uint8Array.from(refusal.serializeSolids(Uint32Array.of(bore))), journal = refusal.journalSummary();
        if (batch) {
          const [response] = JSON.parse(refusal.executeBatchV2(JSON.stringify([{ op: 'runHealPipelineJournaled', args: { solid: bore, steps: ['remove_internal_wires'] } }])));
          assert.ok(response.error);
          assert.match(JSON.stringify(response.error), /configured healing result refused/);
        } else {
          assert.throws(() => refusal.runHealPipelineJournaled(bore, ['remove_internal_wires']), /configured healing result refused/);
        }
        assert.equal(refusal.journalSummary(), journal);
        assert.deepEqual(refusal.serializeSolids(Uint32Array.of(bore)), bytes);
      } finally { refusal.free(); }
      const cube = kernel.makeBox(10 * scale, 10 * scale, 10 * scale);
      const document = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(cube))));
      const shell = document.shells[document.solids[document.solid_roots[0]].outer_shell];
      const face = document.faces[shell.faces[0]], boundary = document.wires[face.outer_wire].edges;
      const points = boundary.map(use => document.vertices[document.edges[use.edge][use.forward ? 'start' : 'end']].point);
      const vertices = [[.25,.25],[.25,.75],[.75,.75],[.75,.25]].map(([u,v]) => document.vertices.push({
        point: points[0].map((p,i) => p + u * (points[1][i] - p) + v * (points[3][i] - p)), tolerance: 1e-7 }) - 1);
      const edges = vertices.map((start,i) => ({ edge: document.edges.push({ start, end: vertices[(i+1)%4], curve: 'Line', tolerance: null }) - 1, forward: true }));
      face.inner_wires.push(document.wires.push({ edges, closed: true }) - 1);
      const sourceEdges = document.edges.map((_,i) => i), sourceVertices = document.vertices.map((_,i) => i);
      // Exercise the supported legacy reader, which reconstructs coedge authority.
      document.version = 2;
      document.pcurves = [];
      delete document.boundary_authority;
      delete document.attributes;
      // Explicit fixture construction anchors include the open-hole source entities.
      // The repair boundary still has to independently verify the repaired solid.
      const index = [['face', shell.faces], ['edge', sourceEdges], ['vertex', sourceVertices]]
        .flatMap(([kind, locals]) => locals.map(local => ({ kind, local })))
        .map((entry, ordinal) => ({ ...entry, ordinal }));
      document.journal = { next_op: 1, next_ordinal: index.length, index,
        entries: [{ op: 0, kind: 'open_hole_fixture', payload: 'Evolution', construction: true,
          scope: index.map(entry => entry.ordinal), events: index.map(entry => [entry.ordinal, { event: 'Generated', sources: [] }]) }] };
      const [source] = kernel.deserializeSolids(new TextEncoder().encode(JSON.stringify(document)));
      assert.equal(kernel.getSolidFaces(source).length, 6);
      assert.equal(kernel.getSolidEdges(source).length, 16);
      assert.equal(kernel.getSolidVertices(source).length, 12);
      const anchor = { solid: source, op: 0 };
      const refs = kinds.flatMap(([kind, query]) => Array.from(kernel[query](anchor.solid), (_, index) => ({ kind, ref: kernel.makeOperationOutputRef(anchor.op, kind, index) })));
      const edit = (active, solid) => {
        if (!batch) return JSON.parse(active.runHealPipelineJournaled(solid, ['remove_internal_wires']));
        const [response] = JSON.parse(active.executeBatchV2(JSON.stringify([{ op: 'runHealPipelineJournaled', args: { solid, steps: ['remove_internal_wires'] } }])));
        assert.equal(response.error, undefined, JSON.stringify(response));
        return response.ok;
      };
      const before = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(source))), journal = kernel.journalSummary();
      assert.throws(() => kernel.runHealPipelineJournaled(source, ['remove_internal_wires', 'missing']));
      assert.equal(kernel.journalSummary(), journal);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(source)), before);
      const result = edit(kernel, anchor.solid);
      assert.equal(result.verified, true);
      assert.equal(result.steps[0].actionsTaken, 1);
      const check = (active, solid, expected) => {
        let deleted = 0;
        for (const [kind, query] of kinds) {
          const bound = new Set();
          for (const item of refs.filter(item => item.kind === kind)) {
            const resolution = JSON.parse(active.resolveRef(item.ref));
            if (resolution.status === 'dangling') { deleted++; continue; }
            assert.equal(resolution.status, 'bound', JSON.stringify(resolution));
            assert.equal(resolution.provenance, 'construction');
            for (const entity of resolution.entities) bound.add(entity.handle);
          }
          assert.deepEqual(bound, new Set(active[query](solid)));
        }
        assert.equal(deleted, 8);
        assert.equal(active.validateSolid(solid), 0);
        assert.ok(Math.abs(active.volume(solid, 0.005 * scale) - expected) < expected * 1e-6);
        assert.equal(JSON.parse(active.meshQuality(solid, 0.005 * scale)).isWatertight, true);
      };
      check(kernel, result.solid, 1000 * scale ** 3);
      restored.makeBox(1, 1, 1);
      const [copy] = restored.deserializeSolids(kernel.serializeSolids(Uint32Array.of(result.solid)));
      const repeated = edit(restored, copy);
      check(restored, repeated.solid, 1000 * scale ** 3);
      const wall = Array.from(restored.getSolidFaces(repeated.solid)).find(face => restored.getFaceNormal(face)[0] > 0.9);
      const drafted = JSON.parse(restored.draftJournaled(repeated.solid, Uint32Array.of(wall), Float64Array.of(0, 0, 1), Float64Array.of(0, 0, 0), 5));
      check(restored, drafted.solid, (1000 + 500 * Math.tan(5 * Math.PI / 180)) * scale ** 3);
    } catch (error) {
      throw new Error(`inner-wire history scale=${scale} batch=${batch}: ${error.message}`, { cause: error });
    } finally { kernel.free(); restored.free(); }
  }
  console.log('ok - inner-wire history: six direct/batch scale cells, deleted boundaries and preserved references, legacy import, arena and later draft');
};


export const runSplitVertexRefusalRegression = ({ BrepKernel }) => {
  for (const scale of [0.001, 1, 10]) for (const batch of [false, true]) {
    const kernel = new BrepKernel();
    try {
      const cube = kernel.makeBox(scale, scale, scale);
      const document = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(cube))));
      document.vertices = [{ point: [0, 0, 0], tolerance: 1e-7 }];
      document.edges = []; document.wires = []; document.faces = [];
      for (let i = 0; i < 11; i++) {
        const a = document.vertices.push({ point: [(i+1)*scale, 0, 0], tolerance: 1e-7 }) - 1;
        const b = document.vertices.push({ point: [(i+1)*scale, scale, 0], tolerance: 1e-7 }) - 1;
        const edges = [[0,a],[a,b],[b,0]].map(([start,end]) => ({
          edge: document.edges.push({ start, end, curve: 'Line', tolerance: null }) - 1, forward: true,
        }));
        const wire = document.wires.push({ edges, closed: true }) - 1;
        document.faces.push({ outer_wire: wire, inner_wires: [], surface: { Plane: { normal: [0,0,1], d: 0 } }, reversed: false });
      }
      document.shells = [{ faces: document.faces.map((_,i) => i) }];
      document.solids = [{ outer_shell: 0, inner_shells: [] }]; document.solid_roots = [0];
      document.version = 2; document.pcurves = [];
      delete document.boundary_authority; delete document.attributes;
      const index = [['face', document.faces], ['edge', document.edges], ['vertex', document.vertices]]
        .flatMap(([kind, entries]) => entries.map((_,local) => ({ kind, local })))
        .map((entry, ordinal) => ({ ...entry, ordinal }));
      document.journal = { next_op: 1, next_ordinal: index.length, index,
        entries: [{ op: 0, kind: 'open_vertex_fans_fixture', payload: 'Evolution', construction: true,
          scope: index.map(entry => entry.ordinal), events: index.map(entry => [entry.ordinal, { event: 'Generated', sources: [] }]) }] };
      const [solid] = kernel.deserializeSolids(new TextEncoder().encode(JSON.stringify(document)));
      assert.equal(kernel.getSolidVertices(solid).length, 23);
      const bytes = Uint8Array.from(kernel.serializeSolids(Uint32Array.of(solid))), journal = kernel.journalSummary();
      // These open fans must remain refused even though the splitter can separate their vertex uses.
      for (const steps of [['split_common_vertex'], ['split_common_vertex', 'missing']]) {
        const pattern = steps.length === 1 ? /configured healing result refused/ : /unknown operator/;
        if (batch) {
          const [response] = JSON.parse(kernel.executeBatchV2(JSON.stringify([{ op: 'runHealPipelineJournaled', args: { solid, steps } }])));
          assert.ok(response.error);
          assert.match(JSON.stringify(response.error), pattern);
        } else {
          assert.throws(() => kernel.runHealPipelineJournaled(solid, steps), pattern);
        }
        assert.equal(kernel.journalSummary(), journal);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), bytes);
      }
    } finally { kernel.free(); }
  }
};
