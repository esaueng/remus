#!/usr/bin/env node
'use strict';
const path = require('node:path');
const fs = require('node:fs');
const assert = require('node:assert/strict');
const { kernelClass } = require('../../kernel-class.cjs');
const recipe = require('./holder_recipe.cjs');

// Independent quantity oracle: only the floor's rounded rectangular section
// changes with opening width. Its exact area is 20*8 - 2*(3*3 - pi*3*3/4).
const SECTION_AREA = 142 + 4.5 * Math.PI;
const parse = value => typeof value === 'string' ? JSON.parse(value) : value;

function retainedNurbsCarriersMatch(base, result, width, expectedCount = 8) {
  const old = JSON.parse(base).faces.map(f => f.surface.Nurbs).filter(Boolean);
  const remaining = JSON.parse(Buffer.from(result).toString('utf8')).faces.map(f => f.surface.Nurbs).filter(Boolean);
  assert.equal(old.length, expectedCount); assert.equal(remaining.length, old.length);
  let maximumControlPointResidual = 0;
  for (const carrier of old) {
    const xs = carrier.control_points.flat().map(p => p[0]);
    assert.ok(Math.max(...xs) < -4 || Math.min(...xs) > 26, 'Carrier crosses the rebuilt floor');
    const shift = (width - 46) / 2 * (Math.max(...xs) < -4 ? -1 : 1);
    const { control_points: originalPoints, ...originalOther } = carrier;
    const match = remaining.findIndex(candidate => {
      const { control_points: points, ...other } = candidate;
      if (JSON.stringify(originalOther) !== JSON.stringify(other)) return false;
      if (points.length !== originalPoints.length || points.some((r, i) => r.length !== originalPoints[i].length)) return false;
      return points.every((row, i) => row.every((point, j) => point.every((v, axis) =>
        Math.abs(v - originalPoints[i][j][axis] - (axis === 0 ? shift : 0)) < 1e-11)));
    });
    assert.ok(match >= 0, 'Original NURBS carrier/knots/weights changed');
    const [candidate] = remaining.splice(match, 1);
    candidate.control_points.forEach((row, i) => row.forEach((p, j) => p.forEach((v, axis) => {
      maximumControlPointResidual = Math.max(maximumControlPointResidual,
        Math.abs(v - originalPoints[i][j][axis] - (axis === 0 ? shift : 0)));
    })));
  }
  return { carriers: old.length, knotsWeightsDegreesExact: true, maximumControlPointResidual };
}

function verify(packageDirectory, capturedRuns = null) {
  const data = recipe.fixture();
  const RemusKernel = kernelClass(require(path.join(path.resolve(packageDirectory), 'remus_wasm_node.cjs')));
  const kernel = new RemusKernel(), rows = [], previous = new Map();
  try {
    const prefix = recipe.prepare(kernel, data);
    let referenceMass, referenceFeatures, referenceTextVolumes;
    for (const width of [46, 47, 49, 47, 46]) {
      kernel.restore(prefix.checkpoint);
      const result = recipe.rebuild(kernel, data, prefix, width);
      assert.ok(result.roots.every(s => kernel.validateSolid(s) === 0));
      const meshes = result.roots.map(s => recipe.extract(kernel.tessellateSolidGroupedBinary(
        s, recipe.DISPLAY.linear, recipe.DISPLAY.angular)));
      const quality = recipe.qualify(kernel, data, result, meshes);
      assert.ok(quality.meshChecks.every(m => m.signedVolumeFromFloat32 > 0));
      const mass = parse(kernel.massProperties(result.solid, undefined, undefined, 8));
      const features = recipe.featureSummary(result.roots.map(s => JSON.parse(kernel.recognizeFeatures(s, 0.08))));
      referenceFeatures ??= features;
      assert.deepEqual(features, referenceFeatures, 'Recognized feature inventory changed');
      assert.equal(features[0].holeDiameters.filter(d => d === 5).length, 2,
        'The two original 5 mm mounting bores were not recognized');
      const textVolumes = result.text.map(s => kernel.volume(s, 0.08));
      referenceTextVolumes ??= textVolumes;
      assert.ok(textVolumes.every((v, i) => Math.abs(v - referenceTextVolumes[i]) < 1e-8), 'Letter material changed');
      const retainedNurbs = retainedNurbsCarriersMatch(data.documents.base,
        kernel.serializeSolids(Uint32Array.of(result.solid)), width);
      const retainedTextNurbs = retainedNurbsCarriersMatch(data.documents.text,
        kernel.serializeSolids(Uint32Array.from(result.text)), width, 34);
      referenceMass ??= mass.volume;
      const expectedDelta = (width - 46) * SECTION_AREA;
      // 0.002 mm³ is the diagnostic oracle's absolute quadrature/trim bound,
      // not an operation tolerance and not a replacement for exact byte checks.
      const residual = mass.volume - referenceMass - expectedDelta;
      assert.ok(Math.abs(residual) < 0.002, `Closed-form section delta disagrees: ${residual}`);
      if (previous.has(width)) assert.equal(quality.arenaSha256, previous.get(width), 'Reverse/revisit changed exact arena');
      previous.set(width, quality.arenaSha256);
      rows.push({ width, massVolumeOrder8: mass.volume, expectedVolumeDelta: expectedDelta,
        volumeDeltaResidual: residual, arenaSha256: quality.arenaSha256,
        holderFaces: quality.inventories[0].faces, material: quality.material,
        features, textVolumes, retainedNurbs, retainedTextNurbs,
        allEightMeshesClosed: quality.meshChecks.every(m => m.watertight) });
    }
    // A completely new kernel reconstructs from the immutable fixtures; all
    // reachable arena fields must match the previous owner, not just volume.
    const fresh = new RemusKernel();
    try {
      const fixed = recipe.prepare(fresh, data), result = recipe.rebuild(fresh, data, fixed, 47);
      assert.equal(recipe.sha256(fresh.serializeSolids(Uint32Array.from(result.roots))), previous.get(47));
    } finally { fresh.free(); }
    const full = new RemusKernel(), fullReplay = [];
    try {
      const empty = full.checkpoint();
      for (const width of [46, 47, 49, 47, 46]) {
        full.restore(empty);
        const fixed = recipe.prepare(full, data, { checkpoint: false });
        const result = recipe.rebuild(full, data, fixed, width);
        const arenaSha256 = recipe.sha256(full.serializeSolids(Uint32Array.from(result.roots)));
        assert.equal(arenaSha256, previous.get(width), 'Full recipe differs from suffix replay');
        assert.equal(full.checkpointCount(), 1, 'Full replay leaked a temporary checkpoint');
        fullReplay.push({ width, arenaSha256, checkpointCount: full.checkpointCount(),
          memory: JSON.parse(full.checkpointMemoryStats()) });
      }
      assert.ok(fullReplay.at(-1).memory.retiredSlots > 0, 'Same-owner replay must retain tombstones');
    } finally { full.free(); }
    let capturedAudit = null;
    if (capturedRuns) {
      const records = capturedRuns.flatMap(run => run.records);
      const widths = [...new Set(records.map(row => row.width))].sort((a, b) => a - b);
      const auditRows = [];
      for (const run of capturedRuns) {
        assert.equal(run.metadata.fixture.recipeSha256, data.recipeSha256);
        assert.equal(run.metadata.package.wasmSha256, recipe.sha256(fs.readFileSync(
          path.join(path.resolve(packageDirectory), 'remus_wasm_bg.wasm'))));
      }
      for (const width of widths) {
        kernel.restore(prefix.checkpoint);
        const result = recipe.rebuild(kernel, data, prefix, width);
        assert.ok(result.roots.every(s => kernel.validateSolid(s) === 0));
        const meshes = result.roots.map(s => recipe.extract(kernel.tessellateSolidGroupedBinary(
          s, recipe.DISPLAY.linear, recipe.DISPLAY.angular)));
        const quality = recipe.qualify(kernel, data, result, meshes);
        const volumes = result.roots.map(s => kernel.volume(s, 0.08));
        assert.ok(volumes.slice(1).every((v, i) => Math.abs(v - referenceTextVolumes[i]) < 1e-8),
          'Captured-width lettering volume changed');
        const features = recipe.featureSummary(result.roots.map(s => JSON.parse(kernel.recognizeFeatures(s, 0.08))));
        assert.deepEqual(features, referenceFeatures);
        const mass = parse(kernel.massProperties(result.solid, undefined, undefined, 8));
        const expectedDelta = (width - 46) * SECTION_AREA;
        const residual = mass.volume - referenceMass - expectedDelta;
        assert.ok(Math.abs(residual) < 0.002, `Captured width ${width} quantity residual: ${residual}`);
        const retainedNurbs = retainedNurbsCarriersMatch(data.documents.base,
          kernel.serializeSolids(Uint32Array.of(result.solid)), width);
        const retainedTextNurbs = retainedNurbsCarriersMatch(data.documents.text,
          kernel.serializeSolids(Uint32Array.from(result.text)), width, 34);
        const matches = records.filter(row => row.width === width);
        const quantityRereadAbsoluteDifferences = [];
        for (const row of matches) {
          for (const key of ['arenaSha256', 'meshSha256', 'bbox', 'inventories', 'material'])
            assert.deepEqual(quality[key], row[key], `Captured width ${width}: ${key}`);
          if (row.volumes) quantityRereadAbsoluteDifferences.push(
            row.volumes.map((v, i) => Math.abs(v - volumes[i])));
          if (row.featureSummary) assert.deepEqual(features, row.featureSummary);
        }
        auditRows.push({ width, capturedRecordsMatched: matches.length, massVolumeOrder8: mass.volume,
          expectedVolumeDelta: expectedDelta, volumeDeltaResidual: residual, retainedNurbs,
          retainedTextNurbs, arenaSha256: quality.arenaSha256, meshSha256: quality.meshSha256,
          material: quality.material, bbox: quality.bbox, features, volumes });
        auditRows.at(-1).quantityRereadAbsoluteDifferences = quantityRereadAbsoluteDifferences;
      }
      capturedAudit = { qualified: true, distinctWidths: widths.length,
        capturedRecordsMatched: records.length,
        quantityRereadPolicy: 'Residuals recorded, not assumed bit-identical across the independent mass-query history. Independent floor-delta and lettering-volume invariance gates remain unchanged.',
        rows: auditRows };
    }
    return { qualified: true, timingStatus: 'correctness-only-excluded', sectionArea: SECTION_AREA,
      quantityOracleAbsoluteBound: 0.002, exactArenaRevisitAndFreshOwnerMatch: true,
      fullAndSuffixArenasMatch: true, fullReplay, rows, capturedAudit };
  } finally { kernel.free(); }
}
module.exports = { verify, SECTION_AREA, retainedNurbsCarriersMatch };
if (require.main === module) {
  try {
    assert.ok(process.argv.length >= 3 && process.argv.length <= 5,
      'Expected kernel-pkg [new-output.json [captured-runs.json]]');
    const result = verify(process.argv[2], process.argv[4] ? JSON.parse(fs.readFileSync(process.argv[4])) : null);
    const json = JSON.stringify(result, null, 2) + '\n';
    if (process.argv[3]) fs.writeFileSync(process.argv[3], json, { flag: 'wx' });
    else process.stdout.write(json);
  } catch (e) { console.error(e.stack || e); process.exitCode = 1; }
}
