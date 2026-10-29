#!/usr/bin/env node
'use strict';

// Untimed, separate-process replay of saved arenas. It never warms a timed run.
const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
function bits(value) {
  assert.equal(Number.isFinite(value), true, 'Quantity must be finite');
  const bytes = Buffer.alloc(8);
  bytes.writeDoubleLE(value);
  return bytes.toString('hex');
}

function replay(kernelDirectory, runDirectory) {
  const { RemusKernel } = require(path.join(path.resolve(kernelDirectory), 'remus_wasm_node.cjs'));
  const run = JSON.parse(fs.readFileSync(path.join(runDirectory, 'results.json'), 'utf8'));
  const rows = [];
  for (const [index, saved] of [run.source, ...run.records].entries()) {
    assert.equal(path.basename(saved.artifacts.arena.file), saved.artifacts.arena.file);
    const document = fs.readFileSync(path.join(runDirectory, saved.artifacts.arena.file));
    assert.equal(sha256(document), saved.arenaSha256);
    const kernel = new RemusKernel();
    try {
      const roots = kernel.deserializeSolids(document);
      assert.equal(roots.length, 1);
      const solid = roots[0];
      const faces = Array.from(kernel.getSolidFaces(solid));
      assert.equal(faces.length, saved.faces);
      const faceRecords = faces.map(face => ({ face,
        carrier: JSON.parse(kernel.getAnalyticSurfaceParams(face)),
        areaBits: bits(kernel.faceArea(face, run.metadata.policy.measureDeflection)) }));
      const featuresJson = kernel.recognizeFeatures(solid, run.metadata.policy.measureDeflection);
      rows.push({ snapshot: index === 0 ? 'source' : `iteration-${saved.iteration}`,
        inputArenaSha256: saved.arenaSha256, faceRecords,
        areaBits: bits(kernel.surfaceArea(solid, run.metadata.policy.measureDeflection)),
        volumeBits: bits(kernel.volume(solid, run.metadata.policy.measureDeflection)),
        bboxBits: Array.from(kernel.boundingBox(solid), bits),
        features: JSON.parse(featuresJson), featuresJsonSha256: sha256(featuresJson),
        edgeRelations: kernel.solidEdgeRelations(solid),
        outputArenaSha256: sha256(kernel.serializeSolids(Uint32Array.of(solid))) });
      assert.equal(rows.at(-1).outputArenaSha256, saved.arenaSha256,
        'Untimed read queries must preserve the complete input arena bytes');
      assert.equal(kernel.validateSolid(solid), 0);
    } finally { kernel.free(); }
  }
  return { schemaVersion: 1, diagnosticOnly: true, timingClaimsExcluded: true,
    queryPolicy: 'New kernel per saved arena; module-local production memos persist within this separate replay process',
    measureDeflection: run.metadata.policy.measureDeflection, rows };
}

module.exports = { replay };
if (require.main === module) {
  try {
    assert.equal(process.argv.length, 4, 'Usage: replay_quantities.cjs KERNEL_PKG RUN_DIR');
    process.stdout.write(JSON.stringify(replay(process.argv[2], process.argv[3]), null, 2) + '\n');
  } catch (error) {
    process.stderr.write(`${error.stack || error}\n`);
    process.exitCode = 1;
  }
}
