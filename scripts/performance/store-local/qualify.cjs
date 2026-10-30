#!/usr/bin/env node
'use strict';

// Correctness only. No timing samples are collected by this driver.
const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const { kernelClass } = require('../../kernel-class.cjs');
const recipe = require('../../perf/parameter-rebuild/holder_recipe.cjs');
const { meshQuality } = require('../../perf/filleted-face-edit/wasm_edit_bench.cjs');

const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
function options(argv) {
  const args = { recipe: 'on', 'require-route-proof': 'off' };
  const seen = new Set();
  for (let index = 0; index < argv.length; index += 2) {
    const name = argv[index].replace(/^--/, '');
    assert(argv[index].startsWith('--') && ['kernel-pkg', 'out', 'recipe', 'package-source-sha', 'cases', 'require-route-proof'].includes(name)
      && argv[index + 1] !== undefined && !seen.has(name),
    `Unknown, duplicate or incomplete argument ${argv[index]}`);
    seen.add(name); args[name] = argv[index + 1];
  }
  assert(args['kernel-pkg'] && args.out, 'Required: --kernel-pkg <installed directory> --out <new JSON file>');
  assert(['on', 'off'].includes(args.recipe), 'Invalid --recipe');
  assert(['on', 'off'].includes(args['require-route-proof']), 'Invalid --require-route-proof');
  assert(!args['package-source-sha'] || /^[a-f0-9]{40}$/.test(args['package-source-sha']), 'Invalid source SHA');
  args['kernel-pkg'] = path.resolve(args['kernel-pkg']);
  args.out = path.resolve(args.out);
  assert(!fs.existsSync(args.out), 'Output file already exists');
  return args;
}

// Normalize only entity numbering, face/inner-loop order and a loop's cyclic
// start. Preserve every numeric bit, curve, trim, tolerance and per-use pcurve.
function exact(value) {
  if (typeof value === 'number') {
    assert(Number.isFinite(value), 'Nonfinite arena value');
    const bytes = Buffer.allocUnsafe(8); bytes.writeDoubleLE(value);
    return { f64: bytes.toString('hex') };
  }
  if (Array.isArray(value)) return value.map(exact);
  if (value && typeof value === 'object') return Object.fromEntries(Object.keys(value).sort().map(key => [key, exact(value[key])]));
  return value;
}
function cyclicStart(uses) {
  // Compare complete rotations: identical minimum tokens can have different
  // continuations. Skip losing prefixes without rewriting or sorting uses.
  let first = 0, other = 1, offset = 0;
  while (first < uses.length && other < uses.length && offset < uses.length) {
    const left = uses[(first + offset) % uses.length], right = uses[(other + offset) % uses.length];
    if (left === right) { offset++; continue; }
    if (left > right) {
      first += offset + 1;
      if (first === other) first++;
    } else {
      other += offset + 1;
      if (other === first) other++;
    }
    offset = 0;
  }
  return Math.min(first, other);
}
const rotate = (values, start) => values.slice(start).concat(values.slice(0, start));
function canonicalArena(bytes) {
  const doc = JSON.parse(Buffer.from(bytes).toString('utf8'));
  assert([3, 4, 5].includes(doc.version) && doc.boundary_authority, 'Unsupported arena schema');
  const authority = doc.boundary_authority;
  const loop = loopId => {
    const data = authority.loops[loopId];
    const uses = data.coedges.map(id => {
      const use = authority.coedges[id], edge = doc.edges[use.edge];
      const start = doc.vertices[edge.start], end = doc.vertices[edge.end];
      const shape = { curve: edge.curve, trim: edge.trim ?? null, tolerance: edge.tolerance ?? null };
      // A line carrier has no direction parameter. Its traversal endpoints
      // capture a reversed carrier/use pair without changing material winding.
      if (edge.curve === 'Line') Object.assign(shape, { from: use.forward ? start : end, to: use.forward ? end : start });
      else Object.assign(shape, { start, end, forward: use.forward });
      return JSON.stringify(exact({ ...shape, winding: use.periodic_winding, pcurve: use.pcurve ?? null }));
    });
    return JSON.stringify({ closed: data.closed, uses: rotate(uses, cyclicStart(uses)) });
  };
  const face = id => {
    const data = doc.faces[id], loops = authority.faces[id];
    return JSON.stringify({ surface: exact(data.surface), reversed: data.reversed,
      outer: loop(loops.outer_loop), holes: loops.inner_loops.map(loop).sort() });
  };
  const shell = id => JSON.stringify(doc.shells[id].faces.map(face).sort());
  return { counts: Object.fromEntries(['vertices', 'edges', 'wires', 'faces', 'shells', 'solids'].map(key => [key, doc[key].length])),
    roots: doc.solid_roots.map(id => ({ outer: shell(doc.solids[id].outer_shell), cavities: doc.solids[id].inner_shells.map(shell).sort() })),
    incidence: incidenceGraph(doc, { loop, face, shell }) };
}

function incidenceGraph(doc, content) {
  // Preserve explicit aliases and every endpoint/use/loop/face/shell link.
  // Boundary-content order supplies a concrete traversal, whose dense IDs give
  // a bijection when two encodings agree. This is NOT a complete graph-
  // isomorphism solver: indistinguishable ordering ties may reject isomorphic
  // graphs. They cannot authorize different sharing by matching only a multiset.
  const authority = doc.boundary_authority;
  assert(!(doc.sheet_roots?.length || doc.wire_roots?.length || doc.compounds?.length), 'Incidence qualification supports solid-root documents only');
  assert.equal(authority.faces.length, doc.faces.length, 'Missing face boundary authority');
  const tables = { vertices: doc.vertices, edges: doc.edges, wires: doc.wires,
    coedges: authority.coedges, loops: authority.loops, faces: doc.faces, shells: doc.shells, solids: doc.solids };
  const encoded = Object.fromEntries(Object.keys(tables).map(kind => [kind, []]));
  const indices = Object.fromEntries(Object.keys(tables).map(kind => [kind, new Map()]));
  const data = (kind, id) => {
    assert(Number.isInteger(id) && id >= 0 && id < tables[kind].length, `Invalid ${kind} reference`);
    return tables[kind][id];
  };
  const keyCache = new Map();
  const key = (kind, id) => {
    const cacheKey = `${kind}:${id}`;
    if (keyCache.has(cacheKey)) return keyCache.get(cacheKey);
    const value = data(kind, id);
    let result;
    if (kind === 'faces') result = content.face(id);
    else if (kind === 'loops') result = content.loop(id);
    else if (kind === 'shells') result = content.shell(id);
    else if (kind === 'solids') result = JSON.stringify({ outer: key('shells', value.outer_shell), cavities: value.inner_shells.map(id => key('shells', id)).sort() });
    else if (kind === 'wires') {
      const uses = value.edges.map(useKey);
      result = JSON.stringify({ closed: value.closed, bodyClass: value.body_class ?? null, uses: rotate(uses, cyclicStart(uses)) });
    } else if (kind === 'coedges') result = useKey(value);
    else if (kind === 'edges') {
      const ends = endpoints(value);
      result = JSON.stringify(exact({ curve: value.curve, trim: value.trim ?? null, tolerance: value.tolerance ?? null,
        start: data('vertices', ends.start), end: data('vertices', ends.end) }));
    } else result = JSON.stringify(exact(value));
    keyCache.set(cacheKey, result);
    return result;
  };
  const endpoints = edge => {
    const flipped = edge.curve === 'Line' && JSON.stringify(exact(data('vertices', edge.start))) > JSON.stringify(exact(data('vertices', edge.end)));
    return flipped ? { start: edge.end, end: edge.start, flipped } : { start: edge.start, end: edge.end, flipped };
  };
  const useKey = use => {
    const edge = data('edges', use.edge), ends = endpoints(edge);
    return JSON.stringify({ edge: key('edges', use.edge), forward: use.forward !== ends.flipped,
      winding: exact(use.periodic_winding ?? null), pcurve: exact(use.pcurve ?? null) });
  };
  const ordered = (kind, ids) => ids.slice().sort((a, b) => {
    const left = key(kind, a), right = key(kind, b);
    return left < right ? -1 : left > right ? 1 : 0;
  });
  const visitUse = use => ({ edge: visit('edges', use.edge), forward: use.forward !== endpoints(data('edges', use.edge)).flipped });
  const visit = (kind, id) => {
    const value = data(kind, id), map = indices[kind];
    if (map.has(id)) return map.get(id);
    const dense = encoded[kind].length;
    map.set(id, dense); encoded[kind].push(null);
    let result;
    if (kind === 'vertices') result = exact(value);
    else if (kind === 'edges') {
      const ends = endpoints(value);
      result = { curve: exact(value.curve), trim: exact(value.trim ?? null), tolerance: exact(value.tolerance ?? null),
        start: visit('vertices', ends.start), end: visit('vertices', ends.end) };
    } else if (kind === 'wires') result = { closed: value.closed, bodyClass: value.body_class ?? null,
      uses: rotate(value.edges, cyclicStart(value.edges.map(useKey))).map(visitUse) };
    else if (kind === 'coedges') result = { ...visitUse(value), winding: exact(value.periodic_winding), pcurve: exact(value.pcurve ?? null) };
    else if (kind === 'loops') result = { closed: value.closed,
      uses: rotate(value.coedges, cyclicStart(value.coedges.map(id => key('coedges', id)))).map(id => visit('coedges', id)) };
    else if (kind === 'faces') {
      const loops = authority.faces[id];
      assert.equal(value.inner_wires.length, loops.inner_loops.length, 'Wire/loop authority count mismatch');
      const boundaries = value.inner_wires.map((wire, index) => ({ wire, loop: loops.inner_loops[index] }));
      const boundaryKey = boundary => JSON.stringify([key('wires', boundary.wire), key('loops', boundary.loop)]);
      boundaries.sort((a, b) => boundaryKey(a) < boundaryKey(b) ? -1 : boundaryKey(a) > boundaryKey(b) ? 1 : 0);
      const boundary = (wire, loop) => ({ wire: visit('wires', wire), loop: visit('loops', loop) });
      result = { surface: exact(value.surface), reversed: value.reversed,
        outer: boundary(value.outer_wire, loops.outer_loop), holes: boundaries.map(({ wire, loop }) => boundary(wire, loop)) };
    } else if (kind === 'shells') result = { bodyClass: value.body_class ?? null, faces: ordered('faces', value.faces).map(id => visit('faces', id)) };
    else result = { outer: visit('shells', value.outer_shell), cavities: ordered('shells', value.inner_shells).map(id => visit('shells', id)) };
    encoded[kind][dense] = result;
    return dense;
  };
  const roots = doc.solid_roots.map(id => visit('solids', id));
  // Include orphan table entries too; counts alone would conceal their links.
  for (const kind of Object.keys(tables)) for (const id of ordered(kind, tables[kind].map((_, id) => id))) visit(kind, id);
  return { roots, ...encoded };
}
function move(kernel, solid, x, y, z) {
  kernel.transformSolid(solid, new Float64Array([1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1]));
  return solid;
}
function plate(kernel) {
  let solid = kernel.makeBox(64, 64, 10);
  for (let row = 0; row < 3; row++) for (let column = 0; column < 3; column++) {
    const tool = move(kernel, kernel.makeCylinder(2, 20), 8 + column * 12, 8 + row * 12, -5);
    solid = kernel.cut(solid, tool);
  }
  return solid;
}
const baseVolume = 64 * 64 * 10 - 9 * Math.PI * 4 * 10;
const cases = [
  { name: 'cross_drilled_shaft', kind: 'shaft', op: 'cut', probes: [[0, 0, 6, 'outside'], [0, 2, 6, 'inside'], [0, 0, 3, 'inside'], [0, 4, 6, 'outside']] },
  { name: 'nurbs_patch_fallback', kind: 'nurbs', op: 'cut', expected: 960, probes: [[5, 5, 5, 'outside'], [2, 2, 5, 'inside'], [12, 2, 5, 'outside']] },
  { name: 'cavity_fallback', kind: 'cavity', op: 'cut', expected: 982, probes: [[5, 5, 5, 'outside'], [1.5, 1.5, 5, 'outside'], [3, 3, 3, 'inside']] },
  { name: 'slot_cut', op: 'cut', size: [2, 6, 20], at: [13, 11, -5], expected: baseVolume - 120,
    probes: [[14, 14, 5, 'outside'], [30, 12, 5, 'inside'], [13, 14, 5, 'boundary']] },
  { name: 'rib_fuse', op: 'fuse', size: [6, 3, 4], at: [11, 12.5, 8], expected: baseVolume + 36,
    probes: [[14, 14, 11, 'inside'], [14, 14, 13, 'outside'], [17, 14, 11, 'boundary']] },
  { name: 'window_intersect', op: 'intersect', size: [20, 20, 20], at: [4, 4, -5], expected: 4000 - 4 * Math.PI * 4 * 10,
    probes: [[14, 14, 5, 'inside'], [30, 12, 5, 'outside'], [4, 14, 5, 'boundary']] },
];
function quality(kernel, solid, probes, expected) {
  assert.equal(kernel.validateSolid(solid), 0, 'Strict validation failed');
  const volume = kernel.volume(solid, 0.01);
  assert(Number.isFinite(volume) && volume > 0, 'Invalid volume');
  if (expected !== undefined) assert(Math.abs(volume - expected) < Math.max(1, expected) * 1e-6, 'Independent volume mismatch');
  const faces = kernel.getSolidFaces(solid).length;
  const mesh = recipe.extract(kernel.tessellateSolidGroupedBinary(solid, 0.05, 0.35));
  const meshReport = meshQuality(mesh, faces);
  const welded = kernel.meshQuality(solid, 0.05, 0.35);
  const weldedReport = typeof welded === 'string' ? JSON.parse(welded) : welded;
  assert.equal(weldedReport.boundaryEdges, 0, 'Position-welded mesh is open');
  assert.equal(weldedReport.nonManifoldEdges, 0, 'Position-welded mesh is non-manifold');
  const material = probes.map(([x, y, z, truth]) => {
    const actual = kernel.classifyPoint(solid, x, y, z, 1e-7);
    assert.equal(actual, truth, `Material mismatch at ${x},${y},${z}`);
    return { point: [x, y, z], expected: truth, actual };
  });
  const arena = kernel.serializeSolids(Uint32Array.of(solid));
  return { volume, faces, mesh: meshReport, welded: weldedReport, material, canonical: canonicalArena(arena), arenaSha256: sha256(arena) };
}
const routeNames = ['general_gfa', 'store_local_attempt', 'store_local_success', 'local_cut_success', 'nary'];
function observeBoolean(kernel, run) {
  const unavailable = reason => ({ diagnosticOnly: true, timingClaimsExcluded: true, status: 'unavailable', reason, counts: null, markers: [] });
  if (typeof kernel.setPerformanceTracing !== 'function' || typeof kernel.drainPerformanceTrace !== 'function') {
    return { value: run(), routes: unavailable('Package has no performance trace API') };
  }
  // Enabling clears previous events. Setup and subsequent quality checks stay
  // outside this trace; durations are never used as performance measurements.
  kernel.setPerformanceTracing(true);
  try {
    const value = run(), phases = JSON.parse(kernel.drainPerformanceTrace());
    assert(Array.isArray(phases), 'Malformed performance trace');
    if (phases.length >= 4096) return { value, routes: { ...unavailable('Trace reached its event bound; route counts may be incomplete'), traceEventCount: phases.length } };
    const markers = phases.filter(phase => typeof phase.name === 'string' && phase.name.startsWith('boolean.route.')).map(phase => phase.name.slice('boolean.route.'.length));
    if (!markers.length) return { value, routes: { ...unavailable('No boolean route markers; this package or path does not establish routing'), traceEventCount: phases.length } };
    assert(markers.every(name => routeNames.includes(name)), 'Unknown boolean route marker');
    const counts = Object.fromEntries(routeNames.map(name => [name, 0]));
    for (const name of markers) counts[name]++;
    return { value, routes: { diagnosticOnly: true, timingClaimsExcluded: true, status: 'observed', counts, markers, traceEventCount: phases.length } };
  } finally { kernel.setPerformanceTracing(false); }
}
function assertRoute(routes, fixture, enabled, required) {
  if (routes.status !== 'observed') {
    assert(!required, `${fixture.name}: actual boolean route proof unavailable: ${routes.reason}`);
    return;
  }
  const counts = routes.counts;
  assert(counts.store_local_success <= counts.store_local_attempt, 'Local success without an observed attempt');
  if (!enabled) {
    assert.equal(counts.store_local_attempt, 0, `${fixture.name}: local attempt while disabled`);
    assert.equal(counts.store_local_success, 0, `${fixture.name}: local success while disabled`);
    assert(counts.general_gfa > 0, `${fixture.name}: disabled general route not observed`);
  } else if (['slot_cut', 'rib_fuse', 'window_intersect', 'cross_drilled_shaft'].includes(fixture.name)) {
    assert.equal(counts.store_local_attempt, 1, `${fixture.name}: expected one local attempt`);
    assert.equal(counts.store_local_success, 1, `${fixture.name}: local route did not complete`);
    assert.equal(counts.general_gfa, 0, `${fixture.name}: unexpectedly used general GFA`);
  } else if (fixture.kind === 'cavity') {
    assert(counts.store_local_attempt > 0, 'Cavity local eligibility was not tested');
    assert.equal(counts.store_local_success, 0, 'Cavity unexpectedly completed store-locally');
    assert(counts.general_gfa > 0, 'Cavity decline did not reach general GFA');
  } else if (fixture.kind === 'nurbs') {
    // The original NURBS patch must decline. The wrapper may then flatten the
    // certified planar supports and retry successfully, which is not evidence
    // that the original NURBS representation was handled store-locally.
    assert(counts.store_local_attempt > counts.store_local_success, 'Original NURBS patch decline not observed');
    assert(counts.general_gfa > 0 || counts.store_local_success > 0 || counts.local_cut_success > 0, 'NURBS wrapper completion route not observed');
  }
}
function assertRecipeRoutes(stages, enabled, required) {
  for (const [stage, routes] of Object.entries(stages)) {
    if (routes.status !== 'observed') {
      assert(!required, `holder_recipe ${stage}: actual boolean route proof unavailable: ${routes.reason}`);
      continue;
    }
    assert(routes.counts.store_local_success <= routes.counts.store_local_attempt, `holder_recipe ${stage}: local success without an observed attempt`);
    if (!enabled) {
      assert.equal(routes.counts.store_local_attempt, 0, `holder_recipe ${stage}: local attempt while disabled`);
      assert.equal(routes.counts.store_local_success, 0, `holder_recipe ${stage}: local success while disabled`);
    }
  }
}
function runCase(RemusKernel, fixture, enabled, binding, requireRouteProof) {
  const kernel = new RemusKernel();
  try {
    kernel.setStoreLocalCuts(false);
    let blank, tool;
    if (fixture.kind === 'shaft') {
      blank = kernel.makeCylinder(3, 12); tool = kernel.makeCylinder(1, 10);
      const cosine = Math.cos(Math.PI / 2), sine = Math.sin(Math.PI / 2);
      kernel.transformSolid(tool, new Float64Array([cosine, 0, sine, -5, 0, 1, 0, 0, -sine, 0, cosine, 6, 0, 0, 0, 1]));
    } else if (fixture.kind === 'cavity') {
      const outer = kernel.makeBox(10, 10, 10), inner = move(kernel, kernel.makeBox(2, 2, 2), 4, 4, 4);
      blank = kernel.cut(outer, inner); tool = move(kernel, kernel.makeBox(1, 1, 20), 1, 1, -5);
    } else if (fixture.kind === 'nurbs') {
      blank = kernel.makeBox(10, 10, 10); assert(kernel.convertToBspline(blank) >= 6);
      tool = move(kernel, kernel.makeBox(2, 2, 20), 4, 4, -5);
    } else {
      blank = plate(kernel); tool = move(kernel, kernel.makeBox(...fixture.size), ...fixture.at);
    }
    const input = kernel.serializeSolids(Uint32Array.of(blank, tool));
    kernel.setStoreLocalCuts(enabled);
    const { value: solid, routes } = observeBoolean(kernel, () => {
      if (binding === 'direct') return kernel[fixture.op](blank, tool);
      const rows = JSON.parse(kernel.executeBatch(JSON.stringify([{ op: fixture.op, args: { solidA: blank, solidB: tool } }])));
      assert.equal(typeof rows[0].ok, 'number', JSON.stringify(rows));
      return rows[0].ok;
    });
    assertRoute(routes, fixture, enabled, requireRouteProof);
    assert.equal(sha256(kernel.serializeSolids(Uint32Array.of(blank, tool))), sha256(input), 'Operation changed source operands');
    const probes = fixture.kind ? fixture.probes : [...fixture.probes, [8, 8, 5, 'outside']];
    return { ...quality(kernel, solid, probes, fixture.expected), routes };
  } finally {
    if (typeof kernel.setPerformanceTracing === 'function') kernel.setPerformanceTracing(false);
    kernel.setStoreLocalCuts(false); kernel.free();
  }
}
function firstDifference(a, b, at = 'root') {
  if (JSON.stringify(a) === JSON.stringify(b)) return null;
  if (typeof a === 'string' && typeof b === 'string' && /^[{[]/.test(a) && /^[{[]/.test(b)) {
    try { return firstDifference(JSON.parse(a), JSON.parse(b), at); } catch { /* scalar text below */ }
  }
  if (a && b && typeof a === 'object' && typeof b === 'object') {
    for (const key of [...new Set([...Object.keys(a), ...Object.keys(b)])].sort()) {
      const difference = firstDifference(a[key], b[key], `${at}.${key}`);
      if (difference) return difference;
    }
  }
  return { at, baseline: a, candidate: b };
}
function batch(kernel, op, args = {}) {
  const rows = JSON.parse(kernel.executeBatch(JSON.stringify([{ op, args }])));
  assert(Object.hasOwn(rows[0], 'ok'), `${op}: ${JSON.stringify(rows)}`);
  return rows[0].ok;
}
function settingsContract(RemusKernel) {
  const first = new RemusKernel(), second = new RemusKernel();
  try {
    assert.equal(first.storeLocalCutsEnabled(), false, 'Store-local default changed');
    batch(first, 'setStoreLocalCuts', { enabled: true });
    assert.equal(second.storeLocalCutsEnabled(), true, 'Setting was not shared across kernels');
    const third = new RemusKernel();
    try { assert.equal(third.storeLocalCutsEnabled(), true, 'Constructor reset module setting'); } finally { third.free(); }
    const solid = second.makeBox(10, 10, 10), before = sha256(second.serializeSolids(Uint32Array.of(solid)));
    const failed = JSON.parse(second.executeBatchV2(JSON.stringify([
      { op: 'setStoreLocalCuts', args: { enabled: 'invalid' } },
      { op: 'cut', args: { solidA: solid, solidB: 4294967295 } },
      { op: 'storeLocalCutsEnabled', args: {} },
    ])));
    assert(failed[0].error && failed[1].error && failed[2].ok === true, 'Failed batch item changed setting or stopped later items');
    assert.equal(sha256(second.serializeSolids(Uint32Array.of(solid))), before, 'Refusal changed source input');
    second.setStoreLocalCuts(false); assert.equal(first.storeLocalCutsEnabled(), false);
    return { defaultOff: true, moduleWide: true, constructorPreservesSetting: true, itemRefusalPreservesSettingAndInputs: true };
  } finally { first.setStoreLocalCuts(false); second.free(); first.free(); }
}
function lineageContract(RemusKernel, op, enabled) {
  const kernel = new RemusKernel();
  try {
    kernel.setStoreLocalCuts(false);
    const blank = plate(kernel), tool = move(kernel, kernel.makeBox(2, 6, 20), 13, 11, -5);
    const inputs = canonicalArena(kernel.serializeSolids(Uint32Array.of(blank, tool)));
    kernel.setStoreLocalCuts(enabled);
    const output = batch(kernel, op, { solidA: blank, solidB: tool });
    assert.deepEqual(canonicalArena(kernel.serializeSolids(Uint32Array.of(blank, tool))), inputs, 'Lineage operation changed source geometry');
    assert.equal(kernel.classifyPoint(output.solid, 14, 14, 5, 1e-7), 'outside');
    let resolution = null;
    if (op === 'cutJournaled') {
      const reference = batch(kernel, 'makeOperationOutputRef', { op: output.op, kind: 'face', index: 0 });
      resolution = batch(kernel, 'resolveRef', { ref: reference.ref });
      assert.equal(resolution.status, 'bound'); assert.equal(resolution.provenance, 'construction');
    }
    return { output, resolution, history: batch(kernel, 'journalSummary'), arenaSha256: sha256(kernel.serializeSolids(Uint32Array.of(output.solid))) };
  } finally { kernel.setStoreLocalCuts(false); kernel.free(); }
}
function main(argv = process.argv.slice(2)) {
  const args = options(argv), directory = args['kernel-pkg'];
  const wrapper = path.join(directory, 'remus_wasm_node.cjs');
  const RemusKernel = kernelClass(require(wrapper));
  const packageJson = fs.readFileSync(path.join(directory, 'package.json'));
  const result = { schemaVersion: 1, correctnessOnly: true, package: { name: JSON.parse(packageJson).name, version: JSON.parse(packageJson).version,
    wasmSha256: sha256(fs.readFileSync(path.join(directory, 'remus_wasm_bg.wasm'))), wrapperSha256: sha256(fs.readFileSync(wrapper)), packageJsonSha256: sha256(packageJson),
    suppliedSourceSha: args['package-source-sha'] ?? null, suppliedSourceShaVerification: 'Externally supplied; not verified by the driver.' },
  node: process.version, harnessSha256: sha256(fs.readFileSync(__filename)),
    recipeHarnessSha256: sha256(fs.readFileSync(path.join(__dirname, '../../perf/parameter-rebuild/holder_recipe.cjs'))),
    meshQualityHarnessSha256: sha256(fs.readFileSync(path.join(__dirname, '../../perf/filleted-face-edit/wasm_edit_bench.cjs'))), cases: [], recipe: [], failures: [],
  scope: { actualStoreLocalRoute: 'Diagnostic trace markers around each synthetic boolean and separate recipe prepare/rebuild stages only; absent markers mean unavailable, never zero-success proof. Mesh/quality work is excluded, and trace durations are never timing evidence. Local successes are not inferred from the requested setting.',
    requireRouteProof: args['require-route-proof'] === 'on',
    incidence: 'Matching boundary-ordered graph encodings establish a bijection preserving endpoint, use, wire, loop, face, shell and root aliases. Indistinguishable ordering ties may reject isomorphic graphs; this is not a complete graph-isomorphism solver. Attributes and journal semantics remain separate contracts.',
    adoption: 'Qualification of listed inputs only. Default unchanged; general imported-body, cavity, NURBS, identity and lineage adoption require separate evidence.' } };
  result.settings = settingsContract(RemusKernel);
  result.lineage = [];
  for (const op of ['cutWithEntityEvolution', 'cutJournaled']) {
    const baseline = lineageContract(RemusKernel, op, false), candidate = lineageContract(RemusKernel, op, true);
    assert.deepEqual(candidate, baseline, `${op}: lineage contract changed`);
    result.lineage.push({ op, exactAgreement: true, baselineSha256: sha256(JSON.stringify(baseline)) });
  }
  const selected = args.cases ? args.cases.split(',') : cases.map(fixture => fixture.name);
  assert(selected.length && new Set(selected).size === selected.length && selected.every(name => cases.some(fixture => fixture.name === name)), 'Unknown, empty or duplicate --cases');
  for (const fixture of cases.filter(fixture => selected.includes(fixture.name))) {
    const baseline = runCase(RemusKernel, fixture, false, 'direct', args['require-route-proof'] === 'on');
    for (const enabled of [false, true]) for (const binding of ['direct', 'batch']) {
      const checked = runCase(RemusKernel, fixture, enabled, binding, args['require-route-proof'] === 'on');
      const difference = firstDifference(baseline.canonical, checked.canonical);
      if (difference) result.failures.push({ name: fixture.name, requestedStoreLocal: enabled, binding, reason: 'Exact boundary or incidence content differs', firstDifference: difference });
      assert(Math.abs(checked.volume - baseline.volume) <= Math.max(1, baseline.volume) * 1e-9, `${fixture.name}: volume differs`);
      const { canonical, ...summary } = checked;
      result.cases.push({ name: fixture.name, requestedStoreLocal: enabled, binding, exactBoundaryAgreement: difference === null, canonicalSha256: sha256(JSON.stringify(canonical)), ...summary });
    }
  }
  if (args.recipe === 'on') {
    const data = recipe.fixture();
    result.recipeFixture = { identity: data.identity, recipeSha256: data.recipeSha256, documentSha256: Object.fromEntries(Object.entries(data.documents).map(([key, bytes]) => [key, sha256(bytes)])) };
    for (const width of [47, 49]) {
      let baseline;
      for (const enabled of [false, true]) {
        const kernel = new RemusKernel();
        try {
          kernel.setStoreLocalCuts(enabled);
          const prepared = observeBoolean(kernel, () => recipe.prepare(kernel, data, { checkpoint: false }));
          const rebuilt = observeBoolean(kernel, () => recipe.rebuild(kernel, data, prepared.value, width));
          const built = rebuilt.value;
          const routes = { prepare: prepared.routes, rebuild: rebuilt.routes };
          assertRecipeRoutes(routes, enabled, args['require-route-proof'] === 'on');
          const meshes = built.roots.map(solid => recipe.extract(kernel.tessellateSolidGroupedBinary(solid, recipe.DISPLAY.linear, recipe.DISPLAY.angular)));
          const qualified = recipe.qualify(kernel, data, built, meshes);
          const canonical = canonicalArena(kernel.serializeSolids(Uint32Array.from(built.roots)));
          const difference = baseline ? firstDifference(baseline, canonical) : null;
          if (difference) result.failures.push({ name: 'holder_recipe', width, requestedStoreLocal: enabled, reason: 'Exact boundary or incidence content differs', firstDifference: difference });
          if (!baseline) baseline = canonical;
          result.recipe.push({ width, requestedStoreLocal: enabled, exactBoundaryAgreement: difference === null, canonicalSha256: sha256(JSON.stringify(canonical)), routes, ...qualified });
        } finally {
          if (typeof kernel.setPerformanceTracing === 'function') kernel.setPerformanceTracing(false);
          kernel.setStoreLocalCuts(false); kernel.free();
        }
      }
    }
  }
  result.passed = result.failures.length === 0;
  fs.writeFileSync(args.out, `${JSON.stringify(result, null, 2)}\n`, { flag: 'wx' });
  process.stdout.write(`${result.passed ? 'Qualified' : 'Partial qualification:'} ${result.cases.length} direct/batch cases and ${result.recipe.length} recipe runs; ${result.failures.length} exact-content failures.\n`);
  if (!result.passed) process.exitCode = 1;
}
module.exports = { main, canonicalArena, observeBoolean, assertRoute, assertRecipeRoutes };
if (require.main === module) { try { main(); } catch (error) { process.stderr.write(`${error.stack || error}\n`); process.exitCode = 1; } }
