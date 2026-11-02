/** Public primitive regression for the opt-in shoulder/cap topology transition. */
import assert from 'node:assert/strict';

function geometry(kernel, solid) {
  const body = JSON.parse(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
  // A successful journaled edit appends global history even when the original
  // solid stays unchanged. Compare every serialized geometry field separately.
  delete body.journal;
  return body;
}

function topAt(kernel, solid, height) {
  const faces = Array.from(kernel.getSolidFaces(solid)).filter(face =>
    kernel.getSurfaceType(face) === 'plane' && kernel.getFaceNormal(face)[2] > .99 &&
    Array.from(kernel.getFaceVertices(face)).every(v => Math.abs(kernel.getVertexPosition(v)[2] - height) < 1e-7));
  assert.equal(faces.length, 1, `one upward plane at ${height}`);
  return faces[0];
}

function source(kernel) {
  const box = kernel.makeBox(32, 20, 19.1);
  const boss = kernel.makeCylinder(6, 28.5);
  let solid = kernel.fuse(boss, box);
  solid = kernel.fillet(solid, kernel.getFaceEdges(topAt(kernel, solid, 28.5)), 1.8);
  const near = (a, b) => Math.abs(a - b) < 1e-7;
  const edges = Array.from(kernel.getSolidEdges(solid)).filter(edge => {
    const [ax, ay, az, bx, by, bz] = kernel.getEdgeVertices(edge);
    const atTop = near(az, 19.1) && near(bz, 19.1);
    const kind = kernel.getEdgeCurveType(edge);
    return kind === 'CIRCLE' ? atTop : kind === 'LINE' &&
      ((atTop && ((near(ax, 0) && near(bx, 0)) || (near(ay, 0) && near(by, 0)))) ||
        (Math.abs(az - bz) > 19.09 &&
          ((near(ax, 6) && near(bx, 6) && near(ay, 0) && near(by, 0)) ||
            (near(ay, 6) && near(by, 6) && near(ax, 0) && near(bx, 0)))));
  });
  assert.equal(edges.length, 5);
  solid = kernel.fillet(solid, Uint32Array.from(edges), 5);
  assert.equal(kernel.getSolidFaces(solid).length, 16);
  return solid;
}

function ancestry(kernel, source, result, map) {
  assert.equal(map.origin, 'construction');
  assert.deepEqual(map.unresolved, {});
  const inputs = new Set(Array.from(kernel.getSolidFaces(source)));
  const outputs = new Set(Array.from(kernel.getSolidFaces(result)));
  const accounted = new Set(map.deleted);
  for (const [key, images] of Object.entries(map.modified)) {
    assert.ok(inputs.has(Number(key)));
    assert.ok(!accounted.has(Number(key)));
    accounted.add(Number(key));
    for (const image of images) assert.ok(outputs.has(image));
  }
  assert.deepEqual(accounted, inputs);
  const images = new Set();
  for (const table of [map.modified, map.generated]) {
    for (const [key, row] of Object.entries(table)) {
      assert.ok(inputs.has(Number(key)));
      for (const image of row) images.add(image);
    }
  }
  assert.deepEqual(images, outputs);
}

function invoke(kernel, route, solid, face, distance, prepared) {
  const args = { solid, faces: [face], distance, cornerStations: 256 };
  if (route === 'detailed') return kernel.moveFacesWithQualityDetailed(solid, Uint32Array.of(face), distance, 256);
  if (route === 'journaled') return JSON.parse(kernel.moveFacesJournaledWithQuality(solid, Uint32Array.of(face), distance, 256));
  if (route === 'prepared') return JSON.parse(kernel.moveFacesPreparedJournaledWithQuality(prepared, distance, 256));
  const [method, operation] = route.split(':');
  const row = JSON.parse(kernel[method](JSON.stringify([{ op: operation, args }])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), JSON.stringify(row));
  return row.ok;
}

export function runFaceMoveQualityPackaged({ RemusKernel }) {
  let cells = 0;
  for (const route of ['detailed', 'journaled', 'prepared',
    'executeBatch:moveFacesWithQualityDetailed', 'executeBatchV2:moveFacesWithQualityDetailed',
    'executeBatch:moveFacesJournaledWithQuality', 'executeBatchV2:moveFacesJournaledWithQuality']) {
    const kernel = new RemusKernel();
    try {
      const solid = source(kernel);
      const face = topAt(kernel, solid, 19.1);
      const before = geometry(kernel, solid);
      const journal = kernel.journalSummary();
      const exact = kernel.moveFacesWithQualityDetailed(solid, Uint32Array.of(face), 5);
      assert.equal(exact.status, 'error', `${route}: omitted permission stays exact-only`);
      assert.deepEqual(geometry(kernel, solid), before);
      assert.equal(kernel.journalSummary(), journal);
      const prepared = route === 'prepared' ? kernel.prepareMoveFaces(solid, Uint32Array.of(face)) : undefined;
      const result = invoke(kernel, route, solid, face, 5, prepared);
      const detailed = Object.hasOwn(result, 'status');
      if (detailed) assert.equal(result.status, 'ok', JSON.stringify(result));
      const quality = detailed ? result.details : result;
      assert.equal(quality.quality, 'approximate');
      assert.equal(quality.construction, 'sampledCapJoin');
      assert.equal(quality.cornerStations, 256);
      const moved = detailed ? result.value : result.solid;
      assert.equal(kernel.getSolidFaces(moved).length, 19);
      assert.equal(kernel.validateSolid(moved), 0);
      topAt(kernel, moved, 24.1); topAt(kernel, moved, 28.5);
      if (!detailed) ancestry(kernel, solid, moved, result.faceEvolution);
      assert.deepEqual(geometry(kernel, solid), before);
      if (prepared !== undefined) assert.equal(kernel.releasePreparedMove(prepared), true);
      cells++;
    } finally { kernel.free(); }
  }
  const kernel = new RemusKernel();
  try {
    let solid = source(kernel);
    let height = 19.1;
    for (const distance of [2.6, 2.4, .5, -1.5, -3.1, 4.1]) {
      const before = geometry(kernel, solid);
      const result = JSON.parse(kernel.moveFacesJournaledWithQuality(solid, Uint32Array.of(topAt(kernel, solid, height)), distance, 256));
      ancestry(kernel, solid, result.solid, result.faceEvolution);
      assert.deepEqual(geometry(kernel, solid), before);
      solid = result.solid; height += distance;
      assert.equal(kernel.getSolidFaces(solid).length, height > 21.7 + 1e-7 ? 19 : 16);
      assert.equal(kernel.validateSolid(solid), 0);
      topAt(kernel, solid, height); topAt(kernel, solid, 28.5);
      cells++;
    }
  } finally { kernel.free(); }
  console.log(`ok - sampled cap face moves: ${cells} direct/prepared/batch/cycle cells, strict topology and complete construction ancestry`);
  return { cells };
}
