import assert from 'node:assert/strict';

// These witnesses execute generated bindings, including object-valued tsify
// arguments and results. Native tests cannot exercise that conversion.
export function runProjectCurvePackaged({ BrepKernel }) {
  const frame = { origin: [0, 0, 4], xAxis: [1, 0, 0], normal: [0, 0, 1] };
  const near = (actual, expected) => {
    assert.equal(actual.length, expected.length);
    actual.forEach((value, index) => assert.ok(Math.abs(value - expected[index]) < 1e-7));
  };
  const invoke = (kernel, route, op, args, direct) => {
    if (route === 'direct') return direct();
    const response = JSON.parse(kernel[route](JSON.stringify([{ op, args }])))[0];
    assert.equal(response.error, undefined, JSON.stringify(response));
    return response.ok;
  };
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    const kernel = new BrepKernel();
    try {
      const solid = kernel.makeBox(10, 8, 4);
      const face = Array.from(kernel.getSolidFaces(solid))
        .find((candidate) => kernel.getFaceNormal(candidate)[2] > 0.9);
      assert.notEqual(face, undefined);
      const edge = kernel.makeLineEdge(1, 2, 7, 8, 5, 9);
      const options = { planeFrame: frame };
      const args = { edge, dirX: 0, dirY: 0, dirZ: -1, face, options };
      const result = invoke(kernel, route, 'projectCurveOntoFace', args,
        () => kernel.projectCurveOntoFace(edge, 0, 0, -1, face, options));
      assert.equal(typeof result, 'object', `${route}: object result`);
      assert.deepEqual(result.quality, { kind: 'exact' });
      assert.equal(result.clipped, false);
      assert.equal(result.edges.length, 1);
      const projected = result.edges[0];
      assert.equal(projected.sourceStart, 0);
      assert.equal(projected.sourceEnd, 1);
      assert.equal(projected.planeCurve.kind, 'line');
      near(projected.planeCurve.start, [1, 2]);
      near(projected.planeCurve.end, [8, 5]);
      const vertices = Array.from(kernel.getEdgeVertices(projected.edge));
      near(vertices.slice(0, 3), [1, 2, 4]);
      near(vertices.slice(3, 6), [8, 5, 4]);

      // Forward rays stop where the source crosses the target plane.
      const crossing = kernel.makeLineEdge(2, 3, 5, 8, 3, 3);
      const crossingResult = invoke(kernel, route, 'projectCurveOntoFace', { ...args, edge: crossing },
        () => kernel.projectCurveOntoFace(crossing, 0, 0, -1, face, options));
      assert.equal(crossingResult.clipped, true);
      assert.equal(crossingResult.edges.length, 1);
      near([crossingResult.edges[0].sourceStart, crossingResult.edges[0].sourceEnd], [0, 0.5]);
      const solidArgs = { edges: [edge], dirX: 0, dirY: 0, dirZ: -1, solid };
      const onSolid = invoke(kernel, route, 'projectCurvesOntoSolid', solidArgs,
        () => kernel.projectCurvesOntoSolid(Uint32Array.of(edge), 0, 0, -1, solid));
      assert.deepEqual(onSolid.quality, { kind: 'exact' });
      assert.equal(onSolid.sources.length, 1);
      assert.equal(onSolid.sources[0].edges.length, 1);
      assert.equal(onSolid.sources[0].clipped, false);
      near([onSolid.sources[0].edges[0].sourceStart, onSolid.sources[0].edges[0].sourceEnd], [0, 1]);

      const before = kernel.journalSummary();
      const planeArgs = { edges: [edge], dirX: 0, dirY: 0, dirZ: -1, frame };
      const curves = invoke(kernel, route, 'projectCurvesOntoSketchPlane', planeArgs,
        () => kernel.projectCurvesOntoSketchPlane(Uint32Array.of(edge), 0, 0, -1, frame));
      assert.ok(Array.isArray(curves), `${route}: array result`);
      assert.equal(curves.length, 1);
      assert.equal(curves[0].kind, 'line');
      near(curves[0].start, [1, 2]);
      near(curves[0].end, [8, 5]);
      assert.equal(kernel.journalSummary(), before);

      const bytes = kernel.serializeSolids(Uint32Array.of(solid));
      if (route === 'direct') {
        assert.throws(() => kernel.projectCurveOntoFace(edge, 0, 0, 0, face), /project-curve:invalid-direction/);
        assert.throws(() => kernel.projectCurveOntoFace(edge, 0, 0, -1, face,
          { allowApproximate: 'false' }), /project-curve:invalid-options/);
        assert.throws(() => kernel.projectCurvesOntoSketchPlane(Uint32Array.of(edge), 0, 0, -1,
          { ...frame, origin: [0, 0] }), /project-curve:invalid-options/);
      } else {
        const failed = JSON.parse(kernel[route](JSON.stringify([
          { op: 'projectCurveOntoFace', args: { ...args, dirZ: 0 } },
        ])))[0].error;
        if (route === 'executeBatchV2') assert.equal(failed.details.kernelCode, 'project-curve:invalid-direction');
        else assert.match(failed, /direction/);
      }
      assert.equal(kernel.journalSummary(), before);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), bytes);

      // A curved cap boundary must not prevent first-hit projection onto
      // the disk. The parallel lateral carrier contributes no earlier hit.
      const cylinder = kernel.makeCylinder(2, 10);
      const topCap = Array.from(kernel.getSolidFaces(cylinder))
        .find((candidate) => kernel.getSurfaceType(candidate) === 'plane'
          && kernel.getFaceNormal(candidate)[2] > 0.9);
      assert.notEqual(topCap, undefined);
      const aboveCap = kernel.makeLineEdge(-0.5, 0, 12, 0.5, 0, 12);
      const capSource = Array.from(kernel.getEdgeVertices(aboveCap));
      const capJournal = kernel.journalSummary();
      const capBytes = kernel.serializeSolids(Uint32Array.of(cylinder));
      const capArgs = { edges: [aboveCap], dirX: 0, dirY: 0, dirZ: -1, solid: cylinder };
      const onCap = invoke(kernel, route, 'projectCurvesOntoSolid', capArgs,
        () => kernel.projectCurvesOntoSolid(Uint32Array.of(aboveCap), 0, 0, -1, cylinder));
      assert.deepEqual(onCap.quality, { kind: 'exact' });
      assert.equal(onCap.sources.length, 1);
      assert.equal(onCap.sources[0].source, aboveCap);
      assert.equal(onCap.sources[0].clipped, false);
      assert.equal(onCap.sources[0].edges.length, 1);
      const capEdge = onCap.sources[0].edges[0];
      assert.equal(capEdge.face, topCap);
      near([capEdge.sourceStart, capEdge.sourceEnd], [0, 1]);
      near(Array.from(kernel.getEdgeVertices(capEdge.edge)), [-0.5, 0, 10, 0.5, 0, 10]);
      assert.ok(!Array.from(kernel.getSolidEdges(cylinder)).includes(capEdge.edge));
      assert.deepEqual(Array.from(kernel.getEdgeVertices(aboveCap)), capSource);
      assert.equal(kernel.journalSummary(), capJournal);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(cylinder)), capBytes);

      // Oblique rays meet the top cap before the lateral wall. The later
      // curved hit must neither refuse nor approximate this exact circle.
      const obliqueCircle = kernel.makeCircleEdgeWithRef(0, 0, 12, 0, 0, 1, 0.5, 1, 0, 0);
      const obliqueSource = Array.from(kernel.getEdgeVertices(obliqueCircle));
      const obliqueSpan = Array.from(kernel.getEdgeParamSpan(obliqueCircle));
      near(obliqueSpan, [0, 2 * Math.PI]);
      const obliqueJournal = kernel.journalSummary();
      const obliqueBytes = kernel.serializeSolids(Uint32Array.of(cylinder));
      for (const options of [undefined, { allowApproximate: true }]) {
        const obliqueArgs = { edges: [obliqueCircle], dirX: 0.5, dirY: 0, dirZ: -1, solid: cylinder, options };
        const oblique = invoke(kernel, route, 'projectCurvesOntoSolid', obliqueArgs,
          () => kernel.projectCurvesOntoSolid(Uint32Array.of(obliqueCircle), 0.5, 0, -1, cylinder, options));
        assert.deepEqual(oblique.quality, { kind: 'exact' });
        assert.equal(oblique.sources.length, 1);
        assert.equal(oblique.sources[0].source, obliqueCircle);
        assert.equal(oblique.sources[0].clipped, false);
        assert.equal(oblique.sources[0].edges.length, 1);
        const image = oblique.sources[0].edges[0];
        assert.equal(image.face, topCap);
        near([image.sourceStart, image.sourceEnd], obliqueSpan);
        assert.equal(kernel.getEdgeCurveType(image.edge), 'CIRCLE');
        near(Array.from(kernel.getEdgeVertices(image.edge)), [1.5, 0, 10, 1.5, 0, 10]);
        const handles = kernel.getEdgeVertexHandles(image.edge);
        assert.equal(handles[0], handles[1], 'exact cap image remains closed');
        const [start, end] = kernel.getEdgeParamSpan(image.edge);
        near([end - start], [2 * Math.PI]);
        for (let index = 0; index <= 32; index += 1) {
          const [x, y, z] = kernel.evaluateEdgeCurve(image.edge, start + (end - start) * index / 32);
          near([z, Math.hypot(x - 1, y)], [10, 0.5]);
        }
        assert.ok(!Array.from(kernel.getSolidEdges(cylinder)).includes(image.edge));
        assert.deepEqual(Array.from(kernel.getEdgeVertices(obliqueCircle)), obliqueSource);
        assert.deepEqual(Array.from(kernel.getEdgeParamSpan(obliqueCircle)), obliqueSpan);
        assert.equal(kernel.journalSummary(), obliqueJournal);
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(cylinder)), obliqueBytes);
      }

      // A1's first hit is the cylinder's front wall. Default options refuse
      // approximation; explicit consent returns a free NURBS with disclosed
      // deviation. Its independent metric combines cylinder distance and
      // the source-circle distance after pulling back to the y=5 plane.
      const circle = kernel.makeCircleEdgeWithRef(0, 5, 5, 0, 1, 0, 1, 1, 0, 0);
      const circleSource = Array.from(kernel.getEdgeVertices(circle));
      const circleSpan = Array.from(kernel.getEdgeParamSpan(circle));
      const approximateJournal = kernel.journalSummary();
      const approximateBytes = kernel.serializeSolids(Uint32Array.of(cylinder));
      const approximateArgs = { edges: [circle], dirX: 0, dirY: -1, dirZ: 0, solid: cylinder };
      if (route === 'direct') {
        assert.throws(() => kernel.projectCurvesOntoSolid(Uint32Array.of(circle), 0, -1, 0, cylinder),
          (error) => /project-curve:source-refused/.test(error.message)
            && /source 0 refused/.test(error.message) && /approximation was not allowed/.test(error.message));
      } else {
        const failed = JSON.parse(kernel[route](JSON.stringify([
          { op: 'projectCurvesOntoSolid', args: approximateArgs },
        ])))[0].error;
        if (route === 'executeBatchV2') {
          assert.equal(failed.details.kernelCode, 'project-curve:source-refused');
          assert.match(failed.message, /source 0 refused.*approximation was not allowed/);
        } else assert.match(failed, /source 0 refused.*approximation was not allowed/);
      }
      assert.deepEqual(Array.from(kernel.getEdgeVertices(circle)), circleSource);
      assert.deepEqual(Array.from(kernel.getEdgeParamSpan(circle)), circleSpan);
      assert.equal(kernel.journalSummary(), approximateJournal);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(cylinder)), approximateBytes);

      const approximateOptions = { allowApproximate: true };
      const approximate = invoke(kernel, route, 'projectCurvesOntoSolid',
        { ...approximateArgs, options: approximateOptions },
        () => kernel.projectCurvesOntoSolid(Uint32Array.of(circle), 0, -1, 0, cylinder, approximateOptions));
      assert.equal(approximate.quality.kind, 'approximate');
      assert.ok(Number.isFinite(approximate.quality.maxDeviation));
      // The unit circle's diameter and the cylinder radius both give scale 2.
      assert.ok(approximate.quality.maxDeviation > 0 && approximate.quality.maxDeviation <= 2e-6);
      assert.equal(approximate.sources.length, 1);
      assert.equal(approximate.sources[0].source, circle);
      assert.equal(approximate.sources[0].clipped, false);
      assert.equal(approximate.sources[0].edges.length, 1);
      const fittedEdge = approximate.sources[0].edges[0];
      near([fittedEdge.sourceStart, fittedEdge.sourceEnd], circleSpan);
      assert.equal(kernel.getEdgeCurveType(fittedEdge.edge), 'BSPLINE_CURVE');
      assert.notEqual(kernel.getEdgeNurbsData(fittedEdge.edge), null);
      assert.ok(!Array.from(kernel.getSolidEdges(cylinder)).includes(fittedEdge.edge));
      const fittedVertices = kernel.getEdgeVertexHandles(fittedEdge.edge);
      assert.equal(fittedVertices[0], fittedVertices[1], 'closed circle keeps a closed image');
      const [start, end] = kernel.getEdgeParamSpan(fittedEdge.edge);
      let measured = 0;
      for (let index = 0; index <= 4096; index += 1) {
        const [x, y, z] = kernel.evaluateEdgeCurve(fittedEdge.edge, start + (end - start) * index / 4096);
        assert.ok([x, y, z].every(Number.isFinite));
        assert.ok(y > 0 && y < 5 && z >= 4 - 1e-7 && z <= 6 + 1e-7, 'front-wall first hit');
        measured = Math.max(measured, Math.abs(Math.hypot(x, y) - 2), Math.abs(Math.hypot(x, z - 5) - 1));
      }
      assert.ok(measured <= approximate.quality.maxDeviation + 1e-10,
        `sampled deviation ${measured} exceeds disclosed ${approximate.quality.maxDeviation}`);
      assert.deepEqual(Array.from(kernel.getEdgeVertices(circle)), circleSource);
      assert.deepEqual(Array.from(kernel.getEdgeParamSpan(circle)), circleSpan);
      assert.equal(kernel.journalSummary(), approximateJournal);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(cylinder)), approximateBytes);

      // This phase puts a tiny exterior arc between every point of the old
      // 2049-point membership grid. Whole-interval trim qualification must
      // refuse it instead of publishing an unclipped approximate image.
      const lateral = Array.from(kernel.getSolidFaces(cylinder))
        .find((candidate) => kernel.getSurfaceType(candidate) === 'cylinder');
      assert.notEqual(lateral, undefined);
      const phase = Math.PI / 2048;
      const sliver = kernel.makeCircleEdgeWithRef(0, 5, 9 + 5e-7,
        0, 1, 0, 1, Math.cos(phase), 0, Math.sin(phase));
      const sliverWire = kernel.makeWire(Uint32Array.of(sliver), true);
      const sliverSpan = Array.from(kernel.getEdgeParamSpan(sliver));
      near(sliverSpan, [0, 2 * Math.PI]);
      for (let index = 0; index <= 2048; index += 1) {
        const z = kernel.evaluateEdgeCurve(sliver, 2 * Math.PI * index / 2048)[2];
        assert.ok(z < 10, `${route}: the old grid misses the exterior arc`);
      }
      const extremum = kernel.evaluateEdgeCurve(sliver, 3 * Math.PI / 2 + phase);
      assert.ok(extremum[2] > 10 + 4e-7, `${route}: off-grid point lies above the rim`);
      const sliverSource = Array.from(kernel.getEdgeVertices(sliver));
      const sliverSourceBytes = kernel.serializeWires(Uint32Array.of(sliverWire));
      const sliverTargetBytes = kernel.serializeSolids(Uint32Array.of(cylinder));
      const sliverTargetCounts = Array.from(kernel.getEntityCounts(cylinder));
      const sliverJournal = kernel.journalSummary();
      const sliverArgs = { edge: sliver, dirX: 0, dirY: -1, dirZ: 0,
        face: lateral, options: { allowApproximate: true } };
      if (route === 'direct') {
        assert.throws(() => kernel.projectCurveOntoFace(sliver, 0, -1, 0,
          lateral, sliverArgs.options), /project-curve:approximate-clip-unsupported/);
      } else {
        const response = JSON.parse(kernel[route](JSON.stringify([
          { op: 'projectCurveOntoFace', args: sliverArgs },
        ])))[0];
        assert.equal(response.ok, undefined, `${route}: exterior arc must be refused`);
        if (route === 'executeBatchV2') {
          assert.equal(response.error.code, 'operation_failed');
          assert.equal(response.error.details.kernelCode, 'project-curve:approximate-clip-unsupported');
          assert.match(response.error.message, /approximate projections require one unclipped face/);
        } else {
          assert.match(response.error, /approximate projections require one unclipped face/);
        }
      }
      assert.deepEqual(Array.from(kernel.getEdgeVertices(sliver)), sliverSource);
      assert.deepEqual(Array.from(kernel.getEdgeParamSpan(sliver)), sliverSpan);
      assert.deepEqual(kernel.serializeWires(Uint32Array.of(sliverWire)), sliverSourceBytes);
      assert.deepEqual(kernel.serializeSolids(Uint32Array.of(cylinder)), sliverTargetBytes);
      assert.deepEqual(Array.from(kernel.getEntityCounts(cylinder)), sliverTargetCounts);
      assert.equal(kernel.journalSummary(), sliverJournal);
    } finally {
      kernel.free();
    }
  }
  console.log('ok - packaged projection objects, forward-ray clipping, exact oblique cylinder caps, approximate solid images and whole-interval trim refusals');
}
