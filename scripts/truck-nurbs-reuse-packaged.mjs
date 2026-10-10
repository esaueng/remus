import assert from 'node:assert/strict';

// Independent analytic fixtures authored for this project, without imported data.
export function runTruckNurbsReusePackaged({ BrepKernel }) {
  const kernel = new BrepKernel();
  const curve = kernel.interpolatePoints(
    Float64Array.from([0, 0, 0, 1, 0, 0, 2, 0, 0]), 2,
  );
  const refined = kernel.curveKnotInsert(curve, 0.5, 1);
  const coords = [];
  for (let u = 0; u < 5; u++) {
    for (let v = 0; v < 5; v++) coords.push(u, v, 2 * u + v);
  }
  const face = kernel.interpolateSurface(Float64Array.from(coords), 5, 5, 2, 2);
  const sourceCurve = kernel.getNurbsCurveData(refined);
  const sourceSurface = kernel.getNurbsSurfaceData(face);
  const surface = JSON.parse(sourceSurface);
  const knotU = surface.knotsU.find(k => k > surface.domainU[0] && k < surface.domainU[1]);
  const knotV = surface.knotsV.find(k => k > surface.domainV[0] && k < surface.domainV[1]);
  assert.equal(typeof knotU, 'number');
  assert.equal(typeof knotV, 'number');
  const options = { tolerance: 1e-6, maxWork: 10_000_000 };
  const cubic = { positionTolerance: 1e-6, derivativeTolerance: 1e-6 };
  const cases = [
    ['surfaceKnotRemoveU', { face, knot: knotU, options },
      () => kernel.surfaceKnotRemoveU(face, knotU, JSON.stringify(options))],
    ['surfaceKnotRemoveV', { face, knot: knotV, options },
      () => kernel.surfaceKnotRemoveV(face, knotV, JSON.stringify(options))],
    ['simplifyNurbsSurface', { face, options },
      () => kernel.simplifyNurbsSurface(face, JSON.stringify(options))],
    ['simplifyNurbsCurve', { edge: refined, options },
      () => kernel.simplifyNurbsCurve(refined, JSON.stringify(options))],
    ['fitCubicCurve', { edge: refined, options: cubic },
      () => kernel.fitCubicCurve(refined, JSON.stringify(cubic))],
  ];
  for (const [op, args, direct] of cases) {
    const result = JSON.parse(direct());
    assert.equal(result.status, 'ok', `${op}: ${JSON.stringify(result)}`);
    const [batch] = JSON.parse(kernel.executeBatch(JSON.stringify([{ op, args }])));
    assert.deepEqual(result.value, batch.ok, `${op} direct/batch agreement`);
    assert.equal(result.value.geometryOnly, true);
    assert.ok(result.value.workUsed <= (args.options.maxWork ?? 1_000_000));
    if (op === 'fitCubicCurve') {
      assert.equal(result.value.approximate, true);
      assert.ok(result.value.positionBound <= cubic.positionTolerance);
      assert.ok(result.value.derivativeBound <= cubic.derivativeTolerance);
      assert.deepEqual(result.value.geometry.domain, JSON.parse(sourceCurve).domain);
    } else {
      assert.ok(result.value.removedKnots > 0);
      assert.ok(result.value.deviationBound <= options.tolerance);
    }
  }
  const refused = JSON.parse(kernel.simplifyNurbsSurface(face, '{"maxWork":1}'));
  assert.equal(refused.status, 'error');
  assert.equal(refused.code, 'nurbs_reuse_work_limit');
  assert.equal(refused.category, 'resource_limit');
  const [batchRefused] = JSON.parse(kernel.executeBatchV2(JSON.stringify([
    { op: 'simplifyNurbsSurface', args: { face, options: { maxWork: 1 } } },
  ])));
  assert.equal(batchRefused.error.category, refused.category);
  assert.equal(batchRefused.error.details.kernelCode, refused.code);
  const exact = JSON.parse(kernel.fitCubicCurve(refined, JSON.stringify({ ...cubic, exactOnly: true })));
  assert.equal(exact.category, 'unsupported');
  assert.equal(kernel.getNurbsCurveData(refined), sourceCurve);
  assert.equal(kernel.getNurbsSurfaceData(face), sourceSurface);
  kernel.free();
  console.log('ok - certified NURBS carrier APIs: all five direct/batch results and immutable refusals');
}
