/** Independent packaged witnesses for checkpoint identity, lofts and distances. */
import assert from 'node:assert/strict';

const surfaces = ['direct', 'executeBatch', 'executeBatchV2'];
const translation = (x, y, z) => Float64Array.of(
  1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1,
);

function invoke(kernel, surface, operation, args, directArgs) {
  if (surface === 'direct') return kernel[operation](...directArgs);
  const response = JSON.parse(kernel[surface](JSON.stringify([{ op: operation, args }])))[0];
  if (Object.hasOwn(response, 'error')) {
    throw new Error(typeof response.error === 'string' ? response.error : JSON.stringify(response.error));
  }
  assert.ok(Object.hasOwn(response, 'ok'), JSON.stringify(response));
  return response.ok;
}

function assertStale(kernel, surface, sketch, point, label) {
  assert.throws(() => invoke(kernel, surface, 'gcsPointPosition', { sketch, point }, [sketch, point]),
    /invalid|retired|not found/i, label);
}

function identityWitnesses(BrepKernel) {
  for (const surface of surfaces) {
    const kernel = new BrepKernel();
    const retained = invoke(kernel, surface, 'gcsNew', {}, []);
    const retainedPoint = invoke(kernel, surface, 'gcsAddPoint',
      { sketch: retained, x: 3, y: 4, fixed: true }, [retained, 3, 4, true]);
    const checkpoint = kernel.checkpoint();
    const staleSketch = invoke(kernel, surface, 'gcsNew', {}, []);
    const stalePoint = invoke(kernel, surface, 'gcsAddPoint',
      { sketch: staleSketch, x: 1, y: 2, fixed: false }, [staleSketch, 1, 2, false]);
    const staleInnerPoint = invoke(kernel, surface, 'gcsAddPoint',
      { sketch: retained, x: 5, y: 6, fixed: false }, [retained, 5, 6, false]);
    kernel.restore(checkpoint);
    const freshSketch = invoke(kernel, surface, 'gcsNew', {}, []);
    const freshPoint = invoke(kernel, surface, 'gcsAddPoint',
      { sketch: freshSketch, x: 99, y: 100, fixed: false }, [freshSketch, 99, 100, false]);
    const freshInnerPoint = invoke(kernel, surface, 'gcsAddPoint',
      { sketch: retained, x: 88, y: 77, fixed: false }, [retained, 88, 77, false]);
    assert.ok(freshSketch > staleSketch, `${surface}: sketch handle high-water`);
    assert.ok(freshInnerPoint > staleInnerPoint, `${surface}: point handle high-water`);
    assertStale(kernel, surface, staleSketch, stalePoint, `${surface}: retired sketch`);
    assertStale(kernel, surface, retained, staleInnerPoint, `${surface}: retired point`);
    assert.deepEqual(Array.from(invoke(kernel, surface, 'gcsPointPosition',
      { sketch: retained, point: retainedPoint }, [retained, retainedPoint])), [3, 4]);
    assert.deepEqual(Array.from(invoke(kernel, surface, 'gcsPointPosition',
      { sketch: freshSketch, point: freshPoint }, [freshSketch, freshPoint])), [99, 100]);
    assert.deepEqual(Array.from(invoke(kernel, surface, 'gcsPointPosition',
      { sketch: retained, point: freshInnerPoint }, [retained, freshInnerPoint])), [88, 77]);
    // A second restore also retires these later allocations without recycling IDs.
    kernel.restore(checkpoint);
    assertStale(kernel, surface, freshSketch, freshPoint, `${surface}: repeated sketch restore`);
    assertStale(kernel, surface, retained, freshInnerPoint, `${surface}: repeated point restore`);
    const latestPoint = invoke(kernel, surface, 'gcsAddPoint',
      { sketch: retained, x: 9, y: 10, fixed: false }, [retained, 9, 10, false]);
    assert.ok(latestPoint > freshInnerPoint, `${surface}: repeated restore high-water`);
    kernel.free();
  }

  // Checkpoints and top-level auxiliary stores have opaque public handles.
  const kernel = new BrepKernel();
  const first = kernel.checkpoint();
  const staleCheckpoint = kernel.checkpoint();
  const staleLegacy = kernel.sketchNew();
  const staleAssembly = kernel.assemblyNew('stale');
  kernel.restore(first);
  const freshCheckpoint = kernel.checkpoint();
  const freshLegacy = kernel.sketchNew();
  const freshAssembly = kernel.assemblyNew('fresh');
  assert.ok(freshCheckpoint > staleCheckpoint);
  assert.ok(freshLegacy > staleLegacy);
  assert.ok(freshAssembly > staleAssembly);
  assert.throws(() => kernel.restore(staleCheckpoint), /invalid/i);
  assert.throws(() => kernel.sketchSolve(staleLegacy, 20, 1e-7), /invalid/i);
  assert.throws(() => kernel.assemblyFlatten(staleAssembly), /invalid/i);
  assert.equal(kernel.checkpointCount(), 2);
  assert.deepEqual(JSON.parse(kernel.assemblyFlatten(freshAssembly)), []);
  kernel.discardCheckpoint(freshCheckpoint);
  const afterDiscard = kernel.checkpoint();
  assert.ok(afterDiscard > freshCheckpoint);
  assert.throws(() => kernel.restore(freshCheckpoint), /invalid/i);
  kernel.free();

  const assemblyKernel = new BrepKernel();
  const part = assemblyKernel.makeBox(2, 3, 4);
  const assembly = assemblyKernel.assemblyNew('retained');
  const assemblyCheckpoint = assemblyKernel.checkpoint();
  const identity = translation(0, 0, 0);
  const staleComponent = assemblyKernel.assemblyAddRoot(assembly, 'stale root', part, identity);
  assemblyKernel.restore(assemblyCheckpoint);
  const freshComponent = assemblyKernel.assemblyAddRoot(assembly, 'fresh root', part, identity);
  assert.ok(freshComponent > staleComponent);
  const beforeAssembly = assemblyKernel.assemblyFlatten(assembly);
  assert.throws(() => assemblyKernel.assemblyAddChild(assembly, staleComponent, 'invalid child', part, identity),
    /invalid|not found/i);
  assert.equal(assemblyKernel.assemblyFlatten(assembly), beforeAssembly);
  assemblyKernel.assemblyAddChild(assembly, freshComponent, 'valid child', part, identity);
  assert.equal(JSON.parse(assemblyKernel.assemblyFlatten(assembly)).length, 2);
  assemblyKernel.free();

  for (const surface of surfaces.slice(1)) {
    for (const oversized of [2 ** 32, 2 ** 32 + 1, Number.MAX_SAFE_INTEGER]) {
      for (const operation of ['fuseAll', 'compoundCut']) {
        const kernel = new BrepKernel();
        const solid = kernel.makeBox(2, 3, 4);
        const before = kernel.serializeSolids(Uint32Array.of(solid));
        for (const handles of [[oversized], [solid, oversized]]) {
          const args = operation === 'fuseAll' ? { solids: handles } : { target: solid, tools: handles };
          const response = JSON.parse(kernel[surface](JSON.stringify([{ op: operation, args }])))[0];
          assert.ok(Object.hasOwn(response, 'error'), `${surface} ${operation}: oversized ID ${oversized}`);
          if (surface === 'executeBatchV2') assert.equal(response.error.code, 'invalid_argument');
          assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), before);
          assert.equal(kernel.volume(solid, 0.01), 24);
        }
        kernel.free();
      }
    }
  }
}

function distanceWitnesses(BrepKernel) {
  // Independent full-sphere extrema: closest points face each other along centres.
  for (const surface of surfaces) {
    for (const offset of [0, 10, 100]) {
      const kernel = new BrepKernel();
      const first = kernel.makeSphere(1, 16);
      const second = kernel.makeSphere(1, 16);
      kernel.transformSolid(first, translation(offset, offset, offset));
      kernel.transformSolid(second, translation(offset, offset, offset + 3));
      const before = kernel.serializeSolids(Uint32Array.of(first, second));
      for (const [a, b, za, zb] of [[first, second, offset + 1, offset + 2],
        [second, first, offset + 2, offset + 1]]) {
        const result = Array.from(invoke(kernel, surface, 'solidToSolidDistance',
          { solidA: a, solidB: b }, [a, b]));
        assert.equal(result.length, 7);
        const expected = [1, offset, offset, za, offset, offset, zb];
        for (let i = 0; i < expected.length; i += 1) {
          assert.ok(Math.abs(result[i] - expected[i]) < 1e-7,
            `${surface}: sphere gap/witness ${i}: ${result[i]} vs ${expected[i]}`);
        }
      }
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(first, second)), before);
      kernel.free();
    }
  }

  // pointToSolidDistance currently has a direct binding only; both batch
  // dispatchers reject that operation. Check its finite trims independently.
  for (const offset of [0, 10, 100]) {
    for (const fixture of ['cylinder', 'holed prism']) {
      const kernel = new BrepKernel();
      try {
        let solid;
        let localQuery;
        if (fixture === 'cylinder') {
          solid = kernel.makeCylinder(2, 4);
          localQuery = [3, 0, -1];
        } else {
          const wire = corners => kernel.makePolygonWire(Float64Array.from(
            corners.flatMap(([x, y]) => [x, y, 0]),
          ));
          const outer = wire([[0, 0], [4, 0], [4, 4], [0, 4]]);
          // Clockwise hole winding also makes the extruded cap wires valid.
          const inner = wire([[1, 1], [1, 3], [3, 3], [3, 1]]);
          const face = kernel.makeFaceFromWires(outer, Uint32Array.of(inner));
          solid = kernel.extrude(face, 0, 0, 1, 2);
          localQuery = [2, 2, -0.25];
        }
        kernel.transformSolid(solid, translation(offset, offset, offset));
        const validation = JSON.parse(kernel.validateSolidDetailed(solid));
        assert.equal(validation.errorCount, 0, `${fixture}: ${JSON.stringify(validation)}`);
        const before = kernel.serializeSolids(Uint32Array.of(solid));
        const query = localQuery.map(coordinate => coordinate + offset);
        const result = Array.from(kernel.pointToSolidDistance(...query, solid));
        assert.equal(result.length, 4);
        const [distance, ...closest] = result;
        const [x, y, z] = closest.map(coordinate => coordinate - offset);
        const expected = fixture === 'cylinder' ? Math.SQRT2 : Math.hypot(1, 0.25);
        assert.ok(Math.abs(distance - expected) < 1e-7,
          `${fixture} at ${offset}: minimum ${distance} vs ${expected}`);
        assert.ok(Math.abs(Math.hypot(...query.map((coordinate, i) => coordinate - closest[i]))
          - distance) < 1e-7, `${fixture}: distance agrees with its witness`);

        // Algebraic membership avoids using the distance routine or a sampled
        // classifier as its own oracle. These points lie on the real bottom rims.
        assert.ok(Math.abs(z) < 1e-7, `${fixture}: bottom boundary z=${z}`);
        if (fixture === 'cylinder') {
          assert.ok(Math.abs(Math.hypot(x, y) - 2) < 1e-7,
            `cylinder: witness lies on the closed radius-2 rim: ${closest}`);
          assert.ok(Math.abs(x - 2) < 1e-7 && Math.abs(y) < 1e-7,
            `cylinder: unique closest rim point: ${closest}`);
        } else {
          const inHoleRange = coordinate => coordinate >= 1 - 1e-7 && coordinate <= 3 + 1e-7;
          const onHoleSide = coordinate => Math.abs(coordinate - 1) < 1e-7
            || Math.abs(coordinate - 3) < 1e-7;
          assert.ok(inHoleRange(x) && inHoleRange(y) && (onHoleSide(x) || onHoleSide(y)),
            `holed prism: witness lies on a bottom hole rim, outside the cap interior: ${closest}`);
          // Four side midpoints tie; any of them is a valid closest witness.
          assert.ok((onHoleSide(x) && Math.abs(y - 2) < 1e-7)
            || (onHoleSide(y) && Math.abs(x - 2) < 1e-7),
          `holed prism: one of the four equally close side midpoints: ${closest}`);
        }
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), before);
      } finally {
        kernel.free();
      }
    }
  }
}

function loftWitnesses(BrepKernel) {
  const positions = (kernel, face) => Array.from(kernel.getFaceVertices(face))
    .flatMap(vertex => Array.from(kernel.getVertexPosition(vertex)));
  const makeSquare = (kernel, z, shift = 0, angle = 0) => {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return kernel.makePolygon(Float64Array.from([[0, 0], [2, 0], [2, 2], [0, 2]]
      .flatMap(([x, y]) => [c * x - s * y + shift, s * x + c * y, z])));
  };
  for (const surface of surfaces) {
    for (const smooth of [false, true]) {
      const kernel = new BrepKernel();
      const faces = smooth ? [makeSquare(kernel, 1), makeSquare(kernel, 2, 1), makeSquare(kernel, 3)]
        : [makeSquare(kernel, 1), makeSquare(kernel, 3, 0, 0.5)];
      const before = faces.map(face => positions(kernel, face));
      const operation = smooth ? 'loftSmooth' : 'loft';
      const solid = invoke(kernel, surface, operation, { faces }, [Uint32Array.from(faces)]);
      assert.ok(Number.isInteger(solid), `${surface} ${operation}: exact solid`);
      const validation = JSON.parse(kernel.validateSolidDetailed(solid));
      assert.equal(validation.errorCount, 0, JSON.stringify(validation));
      assert.ok(!validation.issues.some(issue => /vertex.*surface|free.*edge|open.*shell/i.test(issue.description)),
        `${surface} ${operation}: boundary/carrier agreement: ${JSON.stringify(validation)}`);
      const mesh = JSON.parse(kernel.meshQuality(solid, 0.05));
      assert.equal(mesh.isWatertight, true, `${surface} ${operation}: ${JSON.stringify(mesh)}`);
      // Smooth x=4t(1-t), z=1+2t translates a constant-area square: V=8.
      // Ruled rotation interpolates A(t)=(1-t)I+tR, det A=1-2(1-c)t(1-t).
      const expectedVolume = smooth ? 8 : 8 * (2 + Math.cos(0.5)) / 3;
      const volume = kernel.volume(solid, 0.001);
      assert.ok(Math.abs(volume - expectedVolume) < 1e-4,
        `${surface} ${operation}: independent volume ${volume} vs ${expectedVolume}`);
      if (smooth) {
        const rails = Array.from(kernel.getSolidEdges(solid))
          .filter(edge => kernel.getEdgeCurveType(edge) === 'BSPLINE_CURVE');
        assert.equal(rails.length, 4, `${surface}: four shared curved junction rails`);
        const middleCorners = Array.from({ length: 4 }, (_, i) => before[1].slice(3 * i, 3 * i + 3));
        for (const rail of rails) {
          const [start, end] = kernel.getEdgeParamSpan(rail);
          const midpoint = Array.from(kernel.evaluateEdgeCurve(rail, (start + end) / 2));
          const corner = middleCorners.findIndex(point => Math.hypot(
            ...point.map((coordinate, i) => coordinate - midpoint[i]),
          ) < 1e-7);
          assert.ok(corner >= 0,
            `${surface}: interpolated rail passes through supplied middle corner: ${midpoint}`);
          middleCorners.splice(corner, 1);
        }
        assert.equal(middleCorners.length, 0, `${surface}: every middle corner is interpolated`);
      }
      for (let i = 0; i < faces.length; i += 1) {
        assert.deepEqual(positions(kernel, faces[i]), before[i]);
      }
      kernel.free();
    }
  }
}

export function runNextKernelCorrectnessPackaged({ BrepKernel }) {
  identityWitnesses(BrepKernel);
  distanceWitnesses(BrepKernel);
  loftWitnesses(BrepKernel);
  console.log('ok - checkpoint identities, checked batch handles, analytic distances and valid lofts');
}
