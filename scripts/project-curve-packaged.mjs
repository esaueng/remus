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
    } finally {
      kernel.free();
    }
  }
  console.log('ok - packaged projection objects, forward-ray clipping, sketch arrays and typed refusals');
}
