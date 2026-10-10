#!/usr/bin/env node
'use strict';

// One package pair per process: WASM thread-local memos must not cross variants.
// All correctness diagnostics and artifact writes happen after timed API calls.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const crypto = require('node:crypto');
const { performance } = require('node:perf_hooks');

const TARGET_AREA = 1045.930731242697;
// Native/WASM planar sampling differs by ~1.2e-6 mm² on this imported face.
// This is a selection guard only, not an old/new output comparison tolerance.
const SELECTION_AREA_TOLERANCE = 1e-4;
const TARGET_X = 48;
const DISPLAY_DEFLECTION = 0.02;
const DISPLAY_ANGLE = 0.35;
const MEASURE_DEFLECTION = 0.08;

function options(argv) {
  const result = { warmup: 2, samples: 6, label: 'variant', order: 'geometry-first',
    'timing-status': 'measured' };
  const allowed = new Set(['kernel-pkg', 'io-pkg', 'fixture', 'out', 'label', 'warmup', 'samples', 'order',
    'timing-status']);
  for (let i = 0; i < argv.length; i += 2) {
    const name = argv[i].replace(/^--/, '');
    if (!argv[i].startsWith('--') || !allowed.has(name) || argv[i + 1] === undefined) {
      throw new Error(`Unknown or incomplete argument: ${argv[i]}`);
    }
    result[name] = argv[i + 1];
  }
  for (const key of ['kernel-pkg', 'io-pkg', 'fixture', 'out']) {
    if (!result[key]) throw new Error(`Missing --${key}`);
    result[key] = path.resolve(result[key]);
  }
  for (const key of ['warmup', 'samples']) {
    result[key] = Number(result[key]);
    if (!Number.isSafeInteger(result[key]) || result[key] < (key === 'samples' ? 1 : 0)) {
      throw new Error(`Invalid --${key}`);
    }
  }
  if (!['geometry-first', 'volume-first'].includes(result.order)) throw new Error('Invalid --order');
  if (!['measured', 'contended-smoke-excluded'].includes(result['timing-status'])) {
    throw new Error('Invalid --timing-status');
  }
  return result;
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function sha256(bytes) {
  return crypto.createHash('sha256').update(bytes).digest('hex');
}

function bytesOf(array) {
  return Buffer.from(array.buffer, array.byteOffset, array.byteLength);
}

function doubleBits(value) {
  const bytes = Buffer.allocUnsafe(8);
  bytes.writeDoubleLE(value);
  return bytes.toString('hex');
}

function timed(action) {
  const start = performance.now();
  const result = action();
  return { result, ms: performance.now() - start };
}

function packageMetadata(directory, stem) {
  const info = JSON.parse(fs.readFileSync(path.join(directory, 'package.json'), 'utf8'));
  const wasm = fs.readFileSync(path.join(directory, `${stem}_bg.wasm`));
  const wrapper = fs.readFileSync(path.join(directory, `${stem}_node.cjs`));
  return { directory, name: info.name, version: info.version, wasmBytes: wasm.length,
    wasmSha256: sha256(wasm), wrapperSha256: sha256(wrapper) };
}

function sourceFace(kernel, solid) {
  const faces = Array.from(kernel.getSolidFaces(solid));
  const candidates = [];
  for (let index = 0; index < faces.length; index++) {
    const face = faces[index];
    if (JSON.parse(kernel.getAnalyticSurfaceParams(face)).type !== 'plane') continue;
    if (kernel.getFaceNormal(face)[0] <= 0.99) continue;
    const area = kernel.faceArea(face, 0.05);
    const vertices = Array.from(kernel.getFaceVertices(face));
    const positions = vertices.map(vertex => Array.from(kernel.getVertexPosition(vertex)));
    candidates.push({ face, index, area, positions,
      score: Math.abs(area - TARGET_AREA) });
  }
  candidates.sort((a, b) => a.score - b.score || a.index - b.index);
  const selected = candidates[0];
  assert(selected && selected.score < SELECTION_AREA_TOLERANCE, 'Cannot identify the native holder edit face by area: ' +
    JSON.stringify(candidates.slice(0, 4).map(c => ({ index: c.index, area: c.area,
      score: c.score, xValues: c.positions.map(p => p[0]) }))));
  assert(selected.positions.length > 0 && selected.positions.every(p => Math.abs(p[0] - TARGET_X) < 1e-8),
    'The selected source face must lie on x=48');
  assert(faces.length === 160, `Expected 160 source faces; found ${faces.length}`);
  return selected;
}

function extractMesh(grouped) {
  try {
    return { positions: grouped.takePositions(), indices: grouped.takeIndices(),
      normals: grouped.normals, faceOffsets: grouped.takeFaceOffsets() };
  } finally {
    grouped.free();
  }
}

function meshQuality(mesh, faceCount) {
  const { positions, normals, indices, faceOffsets } = mesh;
  assert(positions.length > 0 && positions.length % 3 === 0, 'Invalid mesh positions');
  assert(normals.length === positions.length, 'Missing mesh normals');
  assert(indices.length > 0 && indices.length % 3 === 0, 'Invalid mesh triangles');
  assert(faceOffsets.length === faceCount + 1 && faceOffsets[0] === 0 &&
    faceOffsets[faceCount] === indices.length, 'Invalid face grouping');
  for (let i = 1; i < faceOffsets.length; i++) {
    assert(faceOffsets[i] >= faceOffsets[i - 1] && faceOffsets[i] % 3 === 0, 'Invalid face offset');
  }
  const vertexCount = positions.length / 3;
  assert(vertexCount < 2 ** 26, 'Mesh too large for exact numeric edge keys');
  for (const value of positions) assert(Number.isFinite(value), 'Nonfinite mesh position');
  for (const value of normals) assert(Number.isFinite(value), 'Nonfinite mesh normal');
  const directed = new Set();
  const undirected = new Map();
  let signedVolume = 0;
  for (let t = 0; t < indices.length; t += 3) {
    const a = indices[t], b = indices[t + 1], c = indices[t + 2];
    assert(a < vertexCount && b < vertexCount && c < vertexCount, 'Out-of-range mesh index');
    assert(a !== b && b !== c && c !== a, 'Degenerate mesh index triangle');
    for (const [p, q] of [[a, b], [b, c], [c, a]]) {
      directed.add(p * vertexCount + q);
      const key = Math.min(p, q) * vertexCount + Math.max(p, q);
      undirected.set(key, (undirected.get(key) || 0) + 1);
    }
    const ax = positions[a * 3], ay = positions[a * 3 + 1], az = positions[a * 3 + 2];
    const bx = positions[b * 3], by = positions[b * 3 + 1], bz = positions[b * 3 + 2];
    const cx = positions[c * 3], cy = positions[c * 3 + 1], cz = positions[c * 3 + 2];
    signedVolume += (ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) +
      az * (bx * cy - by * cx)) / 6;
  }
  let boundaryEdges = 0, nonManifoldEdges = 0;
  for (const key of directed) {
    const a = Math.floor(key / vertexCount), b = key % vertexCount;
    if (!directed.has(b * vertexCount + a)) boundaryEdges++;
  }
  for (const count of undirected.values()) if (count > 2) nonManifoldEdges++;
  assert(boundaryEdges === 0 && nonManifoldEdges === 0, 'Display mesh is not watertight by index');
  return { vertices: vertexCount, triangles: indices.length / 3, boundaryEdges,
    nonManifoldEdges, watertight: true, signedVolumeFromFloat32: signedVolume };
}

function qualify(kernel, solid, volume, featuresJson, mesh, expectedX) {
  assert(Number.isFinite(volume) && volume > 0, 'Invalid measured volume');
  const faces = Array.from(kernel.getSolidFaces(solid));
  assert(faces.length === 160, `Edited holder has ${faces.length} faces`);
  const surfaceCounts = {};
  const planesAtExpectedX = [];
  const analyticRadii = [];
  for (const face of faces) {
    const parameters = JSON.parse(kernel.getAnalyticSurfaceParams(face));
    surfaceCounts[parameters.type] = (surfaceCounts[parameters.type] || 0) + 1;
    if ('radius' in parameters) analyticRadii.push([parameters.type, parameters.radius]);
    if ('majorRadius' in parameters) {
      analyticRadii.push([parameters.type, parameters.majorRadius, parameters.minorRadius]);
    }
    if (parameters.type === 'plane' && kernel.getFaceNormal(face)[0] > 0.99) {
      const xs = Array.from(kernel.getFaceVertices(face), vertex => kernel.getVertexPosition(vertex)[0]);
      if (xs.length && xs.every(x => Math.abs(x - expectedX) < 1e-8)) {
        planesAtExpectedX.push({ parameters, xValues: xs, area: kernel.faceArea(face, 0.05) });
      }
    }
  }
  assert(planesAtExpectedX.some(plane => Math.abs(plane.area - TARGET_AREA) < SELECTION_AREA_TOLERANCE),
    `The selected face did not move to x=${expectedX}`);
  const document = kernel.serializeSolids(Uint32Array.of(solid));
  const features = JSON.parse(featuresJson);
  return { document, faces: faces.length, surfaceCounts, analyticRadii, planesAtExpectedX,
    bbox: Array.from(kernel.boundingBox(solid)), volume, volumeBits: doubleBits(volume),
    features, featuresJsonSha256: sha256(featuresJson),
    mesh: meshQuality(mesh, faces.length), arenaSha256: sha256(document) };
}

function saveArtifacts(directory, prefix, qualified, mesh) {
  const files = {};
  for (const [key, extension] of [['positions', 'f32le'], ['normals', 'f32le'],
    ['indices', 'u32le'], ['faceOffsets', 'u32le']]) {
    const name = `${prefix}.${key}.${extension}`;
    const bytes = bytesOf(mesh[key]);
    fs.writeFileSync(path.join(directory, name), bytes);
    files[key] = { file: name, elements: mesh[key].length, sha256: sha256(bytes) };
  }
  const arena = `${prefix}.arena.json`;
  fs.writeFileSync(path.join(directory, arena), qualified.document);
  const { document: omitted, ...summary } = qualified;
  return { ...summary, artifacts: { ...files, arena: { file: arena, sha256: qualified.arenaSha256 } } };
}

function main() {
  const args = options(process.argv.slice(2));
  assert(os.endianness() === 'LE', 'This harness writes little-endian typed-array artifacts');
  fs.mkdirSync(args.out, { recursive: true });
  const meta = {
    kernel: packageMetadata(args['kernel-pkg'], 'remus_wasm'),
    io: packageMetadata(args['io-pkg'], 'remus_wasm_io'),
    fixture: { path: args.fixture, sha256: sha256(fs.readFileSync(args.fixture)) },
    node: process.version, versions: process.versions, platform: process.platform,
    architecture: process.arch, cpu: os.cpus()[0]?.model,
    affinity: fs.existsSync('/proc/self/status') ?
      fs.readFileSync('/proc/self/status', 'utf8').match(/^Cpus_allowed_list:\s*(.*)$/m)?.[1] : null,
    timing: 'performance.now; synchronous public Node WASM API calls; diagnostics and file writes excluded',
    timingStatus: args['timing-status'],
    timingsExcluded: args['timing-status'] !== 'measured',
    policy: { displayDeflection: DISPLAY_DEFLECTION, displayAngle: DISPLAY_ANGLE,
      measureDeflection: MEASURE_DEFLECTION, distance: '5 + iteration * 0.02',
      selectionAreaTolerance: SELECTION_AREA_TOLERANCE,
      sampleKernel: 'new BrepKernel + deserializeSolids(source serializeSolids document)',
      workflow: args.order,
      order: args.order === 'geometry-first' ?
        ['move', 'validate', 'bbox', 'meshKernel', 'meshTransfer', 'volume', 'recognize'] :
        ['move', 'validate', 'bbox', 'volume', 'meshKernel', 'meshTransfer', 'recognize'],
      cacheLimits: 'production defaults; face display mesh cache 32 MiB; no benchmark override' },
  };
  const { BrepKernel } = require(path.join(args['kernel-pkg'], 'remus_wasm_node.cjs'));
  const { RemusIo } = require(path.join(args['io-pkg'], 'remus_wasm_io_node.cjs'));
  const io = new RemusIo();
  let imported;
  const importStart = performance.now();
  try { imported = io.importStep(fs.readFileSync(args.fixture)); } finally { io.free(); }
  meta.importMs = performance.now() - importStart;
  const sourceKernel = new BrepKernel();
  try {
    const roots = sourceKernel.deserializeSolids(imported);
    assert(roots.length === 1, `Expected one imported solid; found ${roots.length}`);
    const sourceSolid = roots[0];
    const selected = sourceFace(sourceKernel, sourceSolid);
    const sourceDocument = sourceKernel.serializeSolids(Uint32Array.of(sourceSolid));
    // Match the native harness's source-query warmup before any edit samples.
    const sourceVolume = timed(() => sourceKernel.volume(sourceSolid, MEASURE_DEFLECTION));
    const sourceValidation = timed(() => sourceKernel.validateSolid(sourceSolid));
    assert(sourceValidation.result === 0, 'Imported source fails validation');
    const sourceGrouped = timed(() => sourceKernel.tessellateSolidGroupedBinary(
      sourceSolid, DISPLAY_DEFLECTION, DISPLAY_ANGLE));
    const sourceMesh = timed(() => extractMesh(sourceGrouped.result));
    const sourceFeatures = timed(() => sourceKernel.recognizeFeatures(sourceSolid, MEASURE_DEFLECTION));
    const sourceQualified = qualify(sourceKernel, sourceSolid, sourceVolume.result,
      sourceFeatures.result, sourceMesh.result, TARGET_X);
    const source = saveArtifacts(args.out, 'source', sourceQualified, sourceMesh.result);
    source.selectedFace = selected;
    source.stagesMs = { volume: sourceVolume.ms, validate: sourceValidation.ms,
      meshKernel: sourceGrouped.ms, meshTransfer: sourceMesh.ms, recognize: sourceFeatures.ms };
    const records = [];
    for (let i = 0; i < args.warmup + args.samples; i++) {
      const constructor = timed(() => new BrepKernel());
      const kernel = constructor.result;
      try {
        const restoration = timed(() => kernel.deserializeSolids(sourceDocument));
        assert(restoration.result.length === 1, 'Restored clone has the wrong root count');
        const solid = restoration.result[0];
        const restoredFaces = kernel.getSolidFaces(solid);
        assert(restoredFaces.length === 160, 'Restored clone has the wrong face count');
        const face = restoredFaces[selected.index];
        assert(kernel.getFaceNormal(face)[0] > 0.99, 'Restored face selection changed');
        const distance = 5 + i * 0.02;
        const movement = timed(() => kernel.moveFacesJournaled(solid, Uint32Array.of(face), distance));
        const edit = JSON.parse(movement.result);
        assert(Number.isInteger(edit.solid) && Number.isInteger(edit.op), 'Invalid journaled result');
        const validation = timed(() => kernel.validateSolid(edit.solid));
        assert(validation.result === 0, `Edited solid validation failed at iteration ${i}`);
        const bbox = timed(() => kernel.boundingBox(edit.solid));
        let grouped, mesh, volume;
        const display = () => {
          grouped = timed(() => kernel.tessellateSolidGroupedBinary(
            edit.solid, DISPLAY_DEFLECTION, DISPLAY_ANGLE));
          mesh = timed(() => extractMesh(grouped.result));
        };
        const quantity = () => { volume = timed(() => kernel.volume(edit.solid, MEASURE_DEFLECTION)); };
        if (args.order === 'geometry-first') { display(); quantity(); } else { quantity(); display(); }
        const features = timed(() => kernel.recognizeFeatures(edit.solid, MEASURE_DEFLECTION));
        const stagesMs = { move: movement.ms, validate: validation.ms, bbox: bbox.ms,
          meshKernel: grouped.ms, meshTransfer: mesh.ms, volume: volume.ms, recognize: features.ms };
        stagesMs.mesh = stagesMs.meshKernel + stagesMs.meshTransfer;
        stagesMs.kernelPipeline = stagesMs.move + stagesMs.validate + stagesMs.bbox +
          stagesMs.meshKernel + stagesMs.volume + stagesMs.recognize;
        stagesMs.pipelineWithTransfer = stagesMs.kernelPipeline + stagesMs.meshTransfer;
        stagesMs.restoredPipeline = stagesMs.pipelineWithTransfer + constructor.ms + restoration.ms;
        const qualified = qualify(kernel, edit.solid, volume.result, features.result, mesh.result,
          TARGET_X + distance);
        assert(volume.result > sourceVolume.result, 'Outward edit did not increase volume');
        const exported = saveArtifacts(args.out, `case-${String(i).padStart(3, '0')}`,
          qualified, mesh.result);
        const record = { iteration: i, warmup: i < args.warmup, distance, valid: true,
          constructorMs: constructor.ms, restoreMs: restoration.ms, stagesMs, ...exported };
        records.push(record);
        process.stderr.write(`${args.label} ${record.warmup ? 'warmup' : 'sample'} ${i}: ` +
          `${stagesMs.pipelineWithTransfer.toFixed(2)} ms; ${qualified.mesh.triangles} triangles\n`);
      } finally { kernel.free(); }
    }
    assert(sha256(sourceKernel.serializeSolids(Uint32Array.of(sourceSolid))) === sha256(sourceDocument),
      'The persistent source body was modified');
    const result = { label: args.label, workflow: args.order, warmup: args.warmup, samples: args.samples,
      metadata: meta, source, records };
    const json = JSON.stringify(result, null, 2) + '\n';
    fs.writeFileSync(path.join(args.out, 'results.json'), json);
    process.stdout.write(json);
  } finally { sourceKernel.free(); }
}

try { main(); } catch (error) {
  process.stderr.write(`${error.stack || error}\n`);
  process.exitCode = 1;
}
