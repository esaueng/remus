'use strict';

// SPDX-License-Identifier: Apache-2.0
// Numeric plan adapted from OpenZCAD growing-holder.ts at 913bb9ddd015bf2e6284c60a231eaa5afe7f2106.
// See NOTICE.md for attribution, immutable source identity, and fixture provenance.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const { meshQuality } = require('../filleted-face-edit/wasm_edit_bench.cjs');

const AXES = ['x', 'y', 'z'];
const IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
const DISPLAY = { linear: 0.02, angular: 0.35, measurement: 0.08 };
const assert = (ok, message) => { if (!ok) throw Error(message); };
const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const moved = (kernel, solid, xyz) => {
  const matrix = [...IDENTITY];
  [matrix[3], matrix[7], matrix[11]] = xyz;
  return kernel.copyAndTransformSolid(solid, Float64Array.from(matrix));
};

function fixture(directory = path.join(__dirname, 'fixtures')) {
  const bytes = fs.readFileSync(path.join(directory, 'recipe.json'));
  const data = JSON.parse(bytes);
  assert(data.schemaVersion === 1 && data.featureCount === 31, 'Unsupported recipe fixture');
  assert(data.featurePlan.length === 31 && data.featurePlan.filter(f => f.fixed).length === 17,
    'Recipe feature plan changed');
  const documents = {};
  for (const part of ['base', 'text']) {
    assert(data.documents[part].file === `${part}.arena.json`, 'Invalid fixture document name');
    documents[part] = fs.readFileSync(path.join(directory, data.documents[part].file));
    assert(documents[part].length === data.documents[part].bytes &&
      sha256(documents[part]) === data.documents[part].sha256, `Changed ${part} fixture bytes`);
  }
  return { ...data, documents, recipeSha256: sha256(bytes) };
}

// This driver deliberately supports the published, fixed 31-feature fixture,
// including its height and lettering controls. Only opening_width is varied.
function plan(recipe, width) {
  assert(recipe.axis === 'x' && recipe.height.axis === 'z', 'Unsupported fixture axes');
  assert(Number.isFinite(width) && width >= recipe.minimumOpening, 'Opening below recipe minimum');
  const delta = width - recipe.sourceOpening;
  const margin = 1 + 0.05 * Math.max(...AXES.map(a => recipe.envelope.max[a] - recipe.envelope.min[a]));
  const grown = side => AXES.map(a => Math.round((recipe.envelope[side][a] +
    (side === 'min' ? -margin : margin)) * 1e9) / 1e9);
  const pieces = [];
  for (const [side, x, shift] of [['negative', 0, -delta / 2], ['positive', 1, delta / 2]]) {
    for (const [part, z] of [['Lower', 0], ['Upper', 1]]) {
      const min = grown('min'), max = grown('max');
      if (x === 0) max[0] = recipe.cuts[0]; else min[0] = recipe.cuts[1];
      if (z === 0) max[2] = recipe.height.cuts[0]; else min[2] = recipe.height.cuts[1];
      pieces.push({ key: `${side}${part}`, min, max, shift: [shift, 0, 0] });
    }
  }
  return { width, delta, pieces, bridges: [
    { key: 'bridge', axis: 'x', offset: recipe.cuts[0] - delta / 2,
      distance: recipe.cuts[1] - recipe.cuts[0] + delta, section: recipe.section, shift: [0, 0, 0] },
    ...['negative', 'positive'].map((side, index) => ({ key: `${side}Arm`, axis: 'z',
      offset: recipe.height.cuts[0], distance: recipe.height.cuts[1] - recipe.height.cuts[0],
      section: recipe.height.sections[side], shift: [(index === 0 ? -1 : 1) * delta / 2, 0, 0] }))
  ], unionOrder: ['negativeLower', 'negativeArm', 'negativeUpper', 'bridge',
    'positiveLower', 'positiveArm', 'positiveUpper'], textShift: [-delta / 2, 0, 0] };
}

function endpoints(segment) {
  if (segment.objectKind === 'line') return [[segment.x1, segment.y1], [segment.x2, segment.y2]];
  assert(segment.objectKind === 'arc' && segment.radius > 0, 'Unsupported section curve');
  return [segment.startAngleDeg, segment.endAngleDeg].map(degrees => {
    const radians = degrees * Math.PI / 180;
    return [segment.centerX + segment.radius * Math.cos(radians),
      segment.centerY + segment.radius * Math.sin(radians)];
  });
}

// The measured profile objects are unordered and may face either way. Order
// their original exact lines/arcs by endpoints; do not replace arcs by chords.
function orderedSection(section) {
  assert(section.length >= 2 && section.length <= 64, 'Invalid profile size');
  const remaining = section.map(segment => ({ segment, points: endpoints(segment), reversed: false }));
  const result = [remaining.shift()];
  const near = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6;
  while (remaining.length) {
    const end = result.at(-1).points[1];
    const candidates = remaining.map((item, i) => ({ i, start: near(end, item.points[0]),
      end: near(end, item.points[1]) })).filter(item => item.start || item.end);
    assert(candidates.length === 1, 'Ambiguous or disconnected profile');
    const match = candidates[0], [next] = remaining.splice(match.i, 1);
    if (!match.start) { next.points.reverse(); next.reversed = true; }
    result.push(next);
  }
  assert(near(result.at(-1).points[1], result[0].points[0]), 'Open profile');
  return result;
}

function profile(kernel, bridge) {
  const world = ([u, v]) => bridge.axis === 'x' ? [bridge.offset, u, v] : [u, v, bridge.offset];
  const edges = orderedSection(bridge.section).map(({ segment, points, reversed }) => {
    const a = world(points[0]), b = world(points[1]);
    if (segment.objectKind === 'line') return kernel.makeLineEdge(...a, ...b);
    const center = world([segment.centerX, segment.centerY]);
    const normal = bridge.axis === 'x' ? [1, 0, 0] : [0, 0, 1];
    if (reversed) for (let i = 0; i < 3; i++) normal[i] *= -1;
    return kernel.makeCircleArc3d(...a, ...b, ...center, ...normal);
  });
  return kernel.makePlanarFaceFromWire(kernel.makeWire(Uint32Array.from(edges), true));
}

function prepare(kernel, data, { checkpoint = true } = {}) {
  const fixed = plan(data.recipe, data.recipe.sourceOpening);
  const pieces = {};
  for (const piece of fixed.pieces) {
    const [source] = kernel.deserializeSolids(data.documents.base);
    assert(source !== undefined && kernel.getSolidFaces(source).length === data.expected.baseFaces,
      'Base fixture did not restore');
    const dimensions = piece.max.map((v, i) => v - piece.min[i]);
    const mask = moved(kernel, kernel.makeBox(...dimensions), piece.min);
    const result = kernel.intersectDetailed(source, mask);
    assert(result.status === 'ok', `Fixed-prefix intersection refused: ${result.code}`);
    kernel.unifyFaces(result.value);
    assert(kernel.validateSolid(result.value) === 0, 'Invalid fixed-prefix piece');
    pieces[piece.key] = result.value;
  }
  const text = Array.from(kernel.deserializeSolids(data.documents.text));
  assert(text.length === data.expected.textSolids, 'Lettering solid count changed');
  assert(text.every((s, i) => kernel.validateSolid(s) === 0 &&
    kernel.getSolidFaces(s).length === data.expected.textFaces[i]), 'Invalid text fixture');
  const roots = Uint32Array.from([...Object.values(pieces), ...text]);
  return { pieces, text, roots, documentSha256: sha256(kernel.serializeSolids(roots)),
    checkpoint: checkpoint ? kernel.checkpoint() : null };
}

function rebuild(kernel, data, prefix, width) {
  const evaluated = plan(data.recipe, width);
  const bodies = { ...prefix.pieces };
  for (const bridge of evaluated.bridges) {
    const face = profile(kernel, bridge);
    const direction = bridge.axis === 'x' ? [1, 0, 0] : [0, 0, 1];
    bodies[bridge.key] = kernel.extrude(face, ...direction, bridge.distance);
  }
  for (const item of [...evaluated.pieces, ...evaluated.bridges]) {
    // The source compiler emits all six moves, even when their evaluated
    // opening/height shifts happen to be zero. The bridge has no move feature.
    if (item.key !== 'bridge') bodies[item.key] = moved(kernel, bodies[item.key], item.shift);
  }
  const raw = kernel.fuseAll(Uint32Array.from(evaluated.unionOrder.map(key => bodies[key])));
  const candidate = moved(kernel, raw, [0, 0, 0]);
  const encodedReport = kernel.unifyFacesChecked(candidate);
  const report = typeof encodedReport === 'string' ? JSON.parse(encodedReport) : encodedReport;
  const solid = !report.reverted && report.facesMerged > 0 && report.resultErrors === 0 ? candidate : raw;
  const text = prefix.text.map(s => moved(kernel, s, evaluated.textShift));
  return { solid, text, roots: [solid, ...text], report, width };
}

function extract(grouped) {
  try { return { positions: grouped.takePositions(), indices: grouped.takeIndices(),
    normals: grouped.normals, faceOffsets: grouped.takeFaceOffsets() }; }
  finally { grouped.free(); }
}

function inventory(kernel, solid) {
  const counts = {}, radii = [];
  for (const face of kernel.getSolidFaces(solid)) {
    const p = JSON.parse(kernel.getAnalyticSurfaceParams(face));
    counts[p.type] = (counts[p.type] || 0) + 1;
    if (p.radius !== undefined) radii.push([p.type, p.radius]);
    if (p.majorRadius !== undefined) radii.push([p.type, p.majorRadius, p.minorRadius]);
  }
  return { faces: Object.values(counts).reduce((a, b) => a + b, 0), counts,
    radii: radii.sort((a, b) => JSON.stringify(a).localeCompare(JSON.stringify(b))) };
}

function qualify(kernel, data, result, meshes) {
  const delta = result.width - data.recipe.sourceOpening;
  const bbox = Array.from(kernel.boundingBox(result.solid));
  const expected = [-26 - delta / 2, 6.5, 4.5, 48 + delta / 2, 59.5, 62.5];
  assert(bbox.every((n, i) => Math.abs(n - expected[i]) < 1e-6), 'Opening envelope changed incorrectly');
  const inventories = result.roots.map(s => inventory(kernel, s));
  assert(inventories[0].faces === 108, 'Holder feature partition changed');
  const meshChecks = meshes.map((mesh, i) => meshQuality(mesh, inventories[i].faces));
  assert(meshChecks.every(mesh => mesh.signedVolumeFromFloat32 > 0), 'Inverted display mesh');
  const material = [
    { point: [11, 49.5, 8], expected: 'inside' },
    { point: [11, 18, 40], expected: 'outside' },
    { point: [-19 - delta / 2, 18, 40], expected: 'inside' },
    { point: [41 + delta / 2, 18, 40], expected: 'inside' },
    { point: [-9 - delta / 2, 49.5, 8], expected: 'outside' },
    { point: [31 + delta / 2, 49.5, 8], expected: 'outside' }
  ].map(probe => ({ ...probe, actual: kernel.classifyPoint(result.solid, ...probe.point, 1e-7) }));
  assert(material.every(p => p.actual === p.expected), 'Opening material probe changed');
  const document = kernel.serializeSolids(Uint32Array.from(result.roots));
  return { bbox, inventories, meshChecks, material, arenaSha256: sha256(document),
    meshSha256: meshes.map(mesh => Object.fromEntries(Object.entries(mesh).map(([k, array]) =>
      [k, sha256(Buffer.from(array.buffer, array.byteOffset, array.byteLength))]))),
    transferredBytes: meshes.reduce((sum, mesh) => sum + Object.values(mesh).reduce((s, a) => s + a.byteLength, 0), 0) };
}

function featureSummary(features) {
  return features.map(entries => ({
    counts: Object.fromEntries([...new Set(entries.map(f => f.type))].sort()
      .map(type => [type, entries.filter(f => f.type === type).length])),
    holeDiameters: entries.filter(f => f.type === 'hole').map(f => f.diameter).sort((a, b) => a - b)
  }));
}

module.exports = { assert, sha256, fixture, plan, orderedSection, prepare, rebuild, extract,
  inventory, qualify, featureSummary, DISPLAY };
