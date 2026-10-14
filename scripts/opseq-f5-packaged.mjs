/** Independent F5/F12 conic-cap material and boundary oracles in packaged WASM. */
import assert from 'node:assert/strict';

const RADIUS = 1.5;
const HEIGHT = 6.5;

function invoke(kernel, surface, op, args, direct, label) {
  if (surface === 'direct') return kernel[op === 'transform' ? 'transformSolid' : op](...direct);
  const result = JSON.parse(kernel[surface](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(result, 'ok'), `${label} ${op}: ${JSON.stringify(result)}`);
  return result.ok;
}

// The public transform surface uses row-major matrices, as does the native
// regression. Apply the same placement to both operands and every probe.
const identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const translation = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
const rotationX = angle => {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1];
};
const rotationZ = angle => {
  const c = Math.cos(angle), s = Math.sin(angle);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
};
const multiply = (a, b) => Array.from({ length: 16 }, (_, index) => {
  const row = Math.floor(index / 4), column = index % 4;
  return a[row * 4] * b[column] + (a[row * 4 + 1] * b[4 + column]
    + (a[row * 4 + 2] * b[8 + column] + a[row * 4 + 3] * b[12 + column]));
});
const transformPoint = (matrix, point) => [0, 1, 2].map(row => matrix[row * 4] * point[0]
  + (matrix[row * 4 + 1] * point[1] + (matrix[row * 4 + 2] * point[2] + matrix[row * 4 + 3])));

function quantities(region, b) {
  // Cartesian disk/rectangle integral, independent of kernel integration:
  // A_overlap = integral_0^b sqrt(r^2-y^2)dy
  //           = (b sqrt(r^2-b^2)+r^2 asin(b/r))/2.
  const x = Math.sqrt(RADIUS ** 2 - b ** 2);
  const arc = RADIUS * Math.asin(b / RADIUS);
  const overlap = (b * x + RADIUS * arc) / 2;
  const disc = Math.PI * RADIUS ** 2;
  const rectangle = RADIUS * b;
  const [area, perimeter] = {
    cylinderCut: [disc - overlap, 2 * Math.PI * RADIUS - arc + b + x + RADIUS],
    boxCut: [rectangle - overlap, b + RADIUS - x + arc],
    intersection: [overlap, arc + b + x + RADIUS],
    union: [disc + rectangle - overlap, 2 * Math.PI * RADIUS - arc + b + RADIUS - x],
  }[region];
  return { volume: area * HEIGHT, area: 2 * area + perimeter * HEIGHT };
}

function contains(region, cylinder, rectangle) {
  switch (region) {
    case 'cylinderCut': return cylinder && !rectangle;
    case 'boxCut': return rectangle && !cylinder;
    case 'intersection': return cylinder && rectangle;
    case 'union': return cylinder || rectangle;
    default: throw new Error(`unknown region ${region}`);
  }
}

function assertOpposingCoedges(kernel, solid, label) {
  // Public exact arena documents retain every wire use, including both
  // seam directions. isEdgeForwardInWire exposes only the first seam use.
  const document = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
  const uses = new Map();
  assert.ok(document.faces.length > 0, `${label}: no exported faces`);
  let cylinders = 0;
  for (const face of document.faces) {
    const carriers = Object.keys(face.surface);
    assert.equal(carriers.length, 1, `${label}: surface encoding`);
    assert.ok(carriers[0] === 'Plane' || carriers[0] === 'Cylinder', `${label}: unexpected ${carriers[0]} carrier`);
    if (carriers[0] === 'Cylinder') cylinders += 1;
    for (const wireIndex of [face.outer_wire, ...face.inner_wires]) {
      const wire = document.wires[wireIndex];
      assert.equal(wire.closed, true, `${label}: open wire`);
      for (const coedge of wire.edges) {
        const directions = uses.get(coedge.edge) ?? [];
        directions.push(coedge.forward !== face.reversed);
        uses.set(coedge.edge, directions);
      }
    }
  }
  assert.ok(cylinders > 0, `${label}: cylinder carrier lost`);
  for (const [edge, directions] of uses) {
    assert.equal(directions.length, 2, `${label}: edge ${edge} use count`);
    assert.notEqual(directions[0], directions[1], `${label}: edge ${edge} same-sense coedges`);
  }
  return cylinders;
}

function checkFixture(BrepKernel, surface, { region, b, scale, placed }) {
  const kernel = new BrepKernel();
  const label = `${surface} F5 ${region} b=${b} scale=${scale} placed=${placed}`;
  const run = (op, args, direct) => invoke(kernel, surface, op, args, direct, label);
  try {
    // Preserve the literal reproduction's creation order and solid operands.
    const rectangle = run('makeBox', { width: RADIUS * scale, height: b * scale, depth: HEIGHT * scale },
      [RADIUS * scale, b * scale, HEIGHT * scale]);
    const cylinder = run('makeCylinder', { radius: RADIUS * scale, height: HEIGHT * scale },
      [RADIUS * scale, HEIGHT * scale]);
    const placement = placed ? multiply(multiply(translation(10 * scale, -7 * scale, 3 * scale),
      rotationX(0.37)), rotationZ(0.23)) : identity();
    for (const solid of [rectangle, cylinder]) {
      run('transform', { solid, matrix: placement }, [solid, Float64Array.from(placement)]);
    }
    const ids = Uint32Array.of(rectangle, cylinder);
    const operands = kernel.serializeSolids(ids);
    try {
      const [operation, a, bSolid] = {
        cylinderCut: ['cut', cylinder, rectangle],
        boxCut: ['cut', rectangle, cylinder],
        intersection: ['intersect', cylinder, rectangle],
        union: ['fuse', rectangle, cylinder],
      }[region];
      const result = run('booleanWithQuality', { operation, solidA: a, solidB: bSolid, exactOnly: true },
        [operation, a, bSolid, true]);
      assert.equal(result.quality, 'exact', `${label}: quality`);
      const solid = result.solid;
      assertOpposingCoedges(kernel, solid, label);
      assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, `${label}: strict validation`);
      const membershipClasses = [0, 0, 0, 0];
      for (const x of [-1.65, -1.25, -0.5, 0.25, 0.75, 1.25, 1.4, 1.65]) {
        for (const y of [-1.25, -0.5, 0.25, 0.6, 0.9, 1.1, 1.4]) {
          if (Math.abs(x * x + y * y - RADIUS ** 2) < 0.02 || Math.abs(x) < 0.02
            || Math.abs(x - RADIUS) < 0.02 || Math.abs(y) < 0.02 || Math.abs(y - b) < 0.02) continue;
          for (const z of [-0.5, 0.5, 3.25, 6, 7]) {
            // All inequalities are evaluated in the independent local fixture;
            // every retained probe is strictly separated from its boundaries.
            const axial = z > 0 && z < HEIGHT;
            const insideCylinder = axial && x * x + y * y < RADIUS ** 2;
            const insideRectangle = axial && x > 0 && x < RADIUS && y > 0 && y < b;
            membershipClasses[Number(insideCylinder) * 2 + Number(insideRectangle)] += 1;
            const point = transformPoint(placement, [x * scale, y * scale, z * scale]);
            const at = `${label}: local=(${x},${y},${z})`;
            assert.equal(kernel.classifyPoint(cylinder, ...point, 1e-7), insideCylinder ? 'inside' : 'outside', `${at}: cylinder`);
            assert.equal(kernel.classifyPoint(rectangle, ...point, 1e-7), insideRectangle ? 'inside' : 'outside', `${at}: box`);
            assert.equal(kernel.classifyPoint(solid, ...point, 1e-7),
              contains(region, insideCylinder, insideRectangle) ? 'inside' : 'outside', `${at}: material`);
          }
        }
      }
      assert.ok(membershipClasses.every(count => count > 0), `${label}: independent grid coverage`);
      const expected = quantities(region, b);
      for (const volume of [kernel.volume(solid, 0.0001 * scale), kernel.volume(solid, 0.00001 * scale),
        JSON.parse(kernel.massProperties(solid)).volume]) {
        assert.ok(Number.isFinite(volume) && Math.abs(volume - expected.volume * scale ** 3) < 1e-4 * scale ** 3,
          `${label}: volume ${volume} vs ${expected.volume} * scale^3`);
      }
      const area = kernel.surfaceArea(solid, 0.0001 * scale);
      assert.ok(Number.isFinite(area) && Math.abs(area - expected.area * scale ** 2) < 1e-4 * scale ** 2,
        `${label}: area ${area} vs ${expected.area} * scale^2`);
      for (const deflection of [0.1, 0.01]) {
        const quality = JSON.parse(kernel.meshQuality(solid, deflection * scale, 0.1));
        assert.equal(quality.boundaryEdges, 0, `${label}: boundary at ${deflection * scale}`);
        assert.equal(quality.nonManifoldEdges, 0, `${label}: non-manifold mesh`);
        assert.equal(quality.isWatertight, true, `${label}: mesh watertight`);
      }
    } finally {
      assert.deepEqual(kernel.serializeSolids(ids), operands, `${label}: retained operands`);
    }
  } finally {
    kernel.free();
  }
}

function translatedCylinderQuantities() {
  // Circle-sector geometry of an r6 disc minus an r8 disc at distance sqrt5.
  const gamma = Math.acos(23 / (12 * Math.sqrt(5)));
  const beta = Math.acos(33 / (16 * Math.sqrt(5)));
  const area = 36 * gamma - 64 * beta + Math.sqrt(191) / 2;
  const perimeter = 12 * gamma + 16 * beta;
  return { volume: 2 * area, area: 2 * area + 2 * perimeter };
}

function checkTranslatedCylinders(BrepKernel, surface, { scale, placed }) {
  const kernel = new BrepKernel();
  const label = `${surface} F12 translated-cylinder cut scale=${scale} placed=${placed}`;
  const run = (op, args, direct) => invoke(kernel, surface, op, args, direct, label);
  try {
    const tool = run('makeCylinder', { radius: 8 * scale, height: 5 * scale }, [8 * scale, 5 * scale]);
    const stock = run('makeCylinder', { radius: 6 * scale, height: 2 * scale }, [6 * scale, 2 * scale]);
    const placement = placed ? multiply(multiply(translation(10 * scale, -7 * scale, 3 * scale),
      rotationX(0.37)), rotationZ(0.23)) : identity();
    const toolMatrix = multiply(placement, translation(scale, 2 * scale, 0));
    run('transform', { solid: tool, matrix: toolMatrix }, [tool, Float64Array.from(toolMatrix)]);
    run('transform', { solid: stock, matrix: placement }, [stock, Float64Array.from(placement)]);
    const ids = Uint32Array.of(tool, stock);
    const operands = kernel.serializeSolids(ids);
    try {
      const result = run('booleanWithQuality', { operation: 'cut', solidA: stock, solidB: tool, exactOnly: true },
        ['cut', stock, tool, true]);
      assert.equal(result.quality, 'exact', `${label}: quality`);
      const solid = result.solid;
      assert.ok(assertOpposingCoedges(kernel, solid, label) >= 2, `${label}: both cylinder walls must survive`);
      assert.equal(JSON.parse(kernel.validateSolidDetailed(solid)).errorCount, 0, `${label}: strict validation`);
      const d = Math.sqrt(5);
      const probes = [];
      // The independent u direction points from stock to tool; v is normal
      // to it. Cartesian circle inequalities below remain the material oracle.
      for (const u of [-6.25, -5.95, -5.85, -5.7, -5.4, -5, 0, 5, 7, 11]) {
        for (const v of [-3.5, -2.5, -1.5, -0.5, 0, 0.5, 1.5, 2.5, 3.5]) {
          for (const z of [-0.5, 0.25, 1, 1.75, 2.25, 4.5, 5.5]) {
            probes.push([(u - 2 * v) / d, (2 * u + v) / d, z, false]);
          }
        }
      }
      // Resolve the thin kept interval between the two left circle roots.
      for (const v of [-2.75, -2, -1, 0, 1, 2, 2.75]) {
        const stockLeft = -Math.sqrt(36 - v * v);
        const toolLeft = d - Math.sqrt(64 - v * v);
        assert.ok(toolLeft > stockLeft, `${label}: analytic crescent interval`);
        for (const fraction of [0.25, 0.5, 0.75]) {
          const u = stockLeft + fraction * (toolLeft - stockLeft);
          for (const z of [0.25, 1, 1.75]) probes.push([(u - 2 * v) / d, (2 * u + v) / d, z, true]);
        }
      }
      const classes = [0, 0, 0, 0];
      let crescentProbes = 0;
      for (const [x, y, z, thinCrescent] of probes) {
        const stockRadiusSquared = x * x + y * y;
        const toolRadiusSquared = (x - 1) ** 2 + (y - 2) ** 2;
        if (!thinCrescent && (Math.abs(stockRadiusSquared - 36) < 0.04 || Math.abs(toolRadiusSquared - 64) < 0.04)) continue;
        const insideStock = stockRadiusSquared < 36 && z > 0 && z < 2;
        const insideTool = toolRadiusSquared < 64 && z > 0 && z < 5;
        if (thinCrescent) {
          assert.ok(insideStock && !insideTool, `${label}: independent thin-crescent guard`);
          crescentProbes += 1;
        }
        classes[Number(insideStock) * 2 + Number(insideTool)] += 1;
        const point = transformPoint(placement, [x * scale, y * scale, z * scale]);
        const at = `${label}: local=(${x},${y},${z})`;
        assert.equal(kernel.classifyPoint(stock, ...point, 1e-7), insideStock ? 'inside' : 'outside', `${at}: stock self-membership`);
        assert.equal(kernel.classifyPoint(tool, ...point, 1e-7), insideTool ? 'inside' : 'outside', `${at}: tool self-membership`);
        assert.equal(kernel.classifyPoint(solid, ...point, 1e-7), insideStock && !insideTool ? 'inside' : 'outside', `${at}: material`);
      }
      assert.equal(crescentProbes, 63, `${label}: thin-crescent coverage`);
      assert.ok(classes.every(count => count > 0), `${label}: independent grid coverage`);
      const expected = translatedCylinderQuantities();
      for (const volume of [kernel.volume(solid, 0.0001 * scale), kernel.volume(solid, 0.00001 * scale),
        JSON.parse(kernel.massProperties(solid)).volume]) {
        assert.ok(Number.isFinite(volume) && Math.abs(volume - expected.volume * scale ** 3) < 1e-4 * scale ** 3,
          `${label}: volume ${volume} vs ${expected.volume} * scale^3`);
      }
      const area = kernel.surfaceArea(solid, 0.0001 * scale);
      assert.ok(Number.isFinite(area) && Math.abs(area - expected.area * scale ** 2) < 1e-4 * scale ** 2,
        `${label}: area ${area} vs ${expected.area} * scale^2`);
      for (const deflection of [0.1, 0.01]) {
        const quality = JSON.parse(kernel.meshQuality(solid, deflection * scale, 0.1));
        assert.equal(quality.boundaryEdges, 0, `${label}: boundary at ${deflection * scale}`);
        assert.equal(quality.nonManifoldEdges, 0, `${label}: non-manifold mesh`);
        assert.equal(quality.isWatertight, true, `${label}: mesh watertight`);
      }
    } finally {
      assert.deepEqual(kernel.serializeSolids(ids), operands, `${label}: retained operands`);
    }
  } finally {
    kernel.free();
  }
}

export function runOpseqF5Packaged({ BrepKernel }) {
  const fixtures = [
    ...[1, 1000].flatMap(scale => [false, true].map(placed => ({ region: 'cylinderCut', b: 1, scale, placed }))),
    ...['boxCut', 'intersection', 'union'].map(region => ({ region, b: 1, scale: 1, placed: false })),
    ...[0.75, 1.25].map(b => ({ region: 'cylinderCut', b, scale: 1, placed: false })),
  ];
  const translatedFixtures = [1, 1000].flatMap(scale => [false, true].map(placed => ({ scale, placed })));
  const failures = [];
  let passed = 0;
  for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const fixture of fixtures) {
      try {
        checkFixture(BrepKernel, surface, fixture);
        passed += 1;
      } catch (error) {
        failures.push(error);
      }
    }
    for (const fixture of translatedFixtures) {
      try {
        checkTranslatedCylinders(BrepKernel, surface, fixture);
        passed += 1;
      } catch (error) {
        failures.push(error);
      }
    }
  }
  const total = 3 * (fixtures.length + translatedFixtures.length);
  if (failures.length) throw new AggregateError(failures, `OpSeq F5/F12 packaged matrix: ${passed}/${total} cells passed`);
  assert.equal(passed, 39);
  console.log('ok - OpSeq F5/F12 closed-form quantities, material, opposing coedges and meshes across 39 packaged API cells');
}
