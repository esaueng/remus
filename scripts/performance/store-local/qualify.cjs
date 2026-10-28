#!/usr/bin/env node
'use strict';

// Correctness only. No timing samples are collected by this driver.
const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const recipe = require('../../perf/parameter-rebuild/holder_recipe.cjs');
const { meshQuality } = require('../../perf/filleted-face-edit/wasm_edit_bench.cjs');

const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
function options(argv) {
  const args = { recipe: 'on' };
  const seen = new Set();
  for (let index = 0; index < argv.length; index += 2) {
    const name = argv[index].replace(/^--/, '');
    assert(argv[index].startsWith('--') && ['kernel-pkg', 'out', 'recipe', 'package-source-sha', 'cases'].includes(name)
      && argv[index + 1] !== undefined && !seen.has(name),
    `Unknown, duplicate or incomplete argument ${argv[index]}`);
    seen.add(name); args[name] = argv[index + 1];
  }
  assert(args['kernel-pkg'] && args.out, 'Required: --kernel-pkg <installed directory> --out <new JSON file>');
  assert(['on', 'off'].includes(args.recipe), 'Invalid --recipe');
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
    let first = 0;
    for (let index = 1; index < uses.length; index++) if (uses[index] < uses[first]) first = index;
    return JSON.stringify({ closed: data.closed, uses: uses.slice(first).concat(uses.slice(0, first)) });
  };
  const face = id => {
    const data = doc.faces[id], loops = authority.faces[id];
    return JSON.stringify({ surface: exact(data.surface), reversed: data.reversed,
      outer: loop(loops.outer_loop), holes: loops.inner_loops.map(loop).sort() });
  };
  const shell = id => JSON.stringify(doc.shells[id].faces.map(face).sort());
  return { counts: Object.fromEntries(['vertices', 'edges', 'wires', 'faces', 'shells', 'solids'].map(key => [key, doc[key].length])),
    roots: doc.solid_roots.map(id => ({ outer: shell(doc.solids[id].outer_shell), cavities: doc.solids[id].inner_shells.map(shell).sort() })) };
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
function runCase(BrepKernel, fixture, enabled, binding) {
  const kernel = new BrepKernel();
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
    let solid;
    if (binding === 'direct') solid = kernel[fixture.op](blank, tool);
    else {
      const rows = JSON.parse(kernel.executeBatch(JSON.stringify([{ op: fixture.op, args: { solidA: blank, solidB: tool } }])));
      assert.equal(typeof rows[0].ok, 'number', JSON.stringify(rows)); solid = rows[0].ok;
    }
    assert.equal(sha256(kernel.serializeSolids(Uint32Array.of(blank, tool))), sha256(input), 'Operation changed source operands');
    const probes = fixture.kind ? fixture.probes : [...fixture.probes, [8, 8, 5, 'outside']];
    return quality(kernel, solid, probes, fixture.expected);
  } finally { kernel.setStoreLocalCuts(false); kernel.free(); }
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
function settingsContract(BrepKernel) {
  const first = new BrepKernel(), second = new BrepKernel();
  try {
    assert.equal(first.storeLocalCutsEnabled(), false, 'Store-local default changed');
    batch(first, 'setStoreLocalCuts', { enabled: true });
    assert.equal(second.storeLocalCutsEnabled(), true, 'Setting was not shared across kernels');
    const third = new BrepKernel();
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
function lineageContract(BrepKernel, op, enabled) {
  const kernel = new BrepKernel();
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
  const { BrepKernel } = require(wrapper);
  const packageJson = fs.readFileSync(path.join(directory, 'package.json'));
  const result = { schemaVersion: 1, correctnessOnly: true, package: { name: JSON.parse(packageJson).name, version: JSON.parse(packageJson).version,
    wasmSha256: sha256(fs.readFileSync(path.join(directory, 'remus_wasm_bg.wasm'))), wrapperSha256: sha256(fs.readFileSync(wrapper)), packageJsonSha256: sha256(packageJson),
    suppliedSourceSha: args['package-source-sha'] ?? null, suppliedSourceShaVerification: 'Externally supplied; not verified by the driver.' },
  node: process.version, harnessSha256: sha256(fs.readFileSync(__filename)),
    recipeHarnessSha256: sha256(fs.readFileSync(path.join(__dirname, '../../perf/parameter-rebuild/holder_recipe.cjs'))),
    meshQualityHarnessSha256: sha256(fs.readFileSync(path.join(__dirname, '../../perf/filleted-face-edit/wasm_edit_bench.cjs'))), cases: [], recipe: [], failures: [],
  scope: { actualStoreLocalRoute: 'Not exposed by the shipped JS API; native work-count tests prove routing separately.',
    adoption: 'Qualification of listed inputs only. Default unchanged; general imported-body, cavity, NURBS, identity and lineage adoption require separate evidence.' } };
  result.settings = settingsContract(BrepKernel);
  result.lineage = [];
  for (const op of ['cutWithEntityEvolution', 'cutJournaled']) {
    const baseline = lineageContract(BrepKernel, op, false), candidate = lineageContract(BrepKernel, op, true);
    assert.deepEqual(candidate, baseline, `${op}: lineage contract changed`);
    result.lineage.push({ op, exactAgreement: true, baselineSha256: sha256(JSON.stringify(baseline)) });
  }
  const selected = args.cases ? args.cases.split(',') : cases.map(fixture => fixture.name);
  assert(selected.length && new Set(selected).size === selected.length && selected.every(name => cases.some(fixture => fixture.name === name)), 'Unknown, empty or duplicate --cases');
  for (const fixture of cases.filter(fixture => selected.includes(fixture.name))) {
    const baseline = runCase(BrepKernel, fixture, false, 'direct');
    for (const enabled of [false, true]) for (const binding of ['direct', 'batch']) {
      const checked = runCase(BrepKernel, fixture, enabled, binding);
      const difference = firstDifference(baseline.canonical, checked.canonical);
      if (difference) result.failures.push({ name: fixture.name, requestedStoreLocal: enabled, binding, reason: 'Exact boundary content differs', firstDifference: difference });
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
        const kernel = new BrepKernel();
        try {
          kernel.setStoreLocalCuts(enabled);
          const prefix = recipe.prepare(kernel, data, { checkpoint: false });
          const built = recipe.rebuild(kernel, data, prefix, width);
          const meshes = built.roots.map(solid => recipe.extract(kernel.tessellateSolidGroupedBinary(solid, recipe.DISPLAY.linear, recipe.DISPLAY.angular)));
          const qualified = recipe.qualify(kernel, data, built, meshes);
          const canonical = canonicalArena(kernel.serializeSolids(Uint32Array.from(built.roots)));
          const difference = baseline ? firstDifference(baseline, canonical) : null;
          if (difference) result.failures.push({ name: 'holder_recipe', width, requestedStoreLocal: enabled, reason: 'Exact boundary content differs', firstDifference: difference });
          if (!baseline) baseline = canonical;
          result.recipe.push({ width, requestedStoreLocal: enabled, exactBoundaryAgreement: difference === null, canonicalSha256: sha256(JSON.stringify(canonical)), ...qualified });
        } finally { kernel.setStoreLocalCuts(false); kernel.free(); }
      }
    }
  }
  result.passed = result.failures.length === 0;
  fs.writeFileSync(args.out, `${JSON.stringify(result, null, 2)}\n`, { flag: 'wx' });
  process.stdout.write(`${result.passed ? 'Qualified' : 'Partial qualification:'} ${result.cases.length} direct/batch cases and ${result.recipe.length} recipe runs; ${result.failures.length} exact-content failures.\n`);
  if (!result.passed) process.exitCode = 1;
}
module.exports = { main, canonicalArena };
if (require.main === module) { try { main(); } catch (error) { process.stderr.write(`${error.stack || error}\n`); process.exitCode = 1; } }
