'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const { canonicalArena, observeBoolean, assertRoute, assertRecipeRoutes } = require('./qualify.cjs');

function document() {
  return {
    version: 3,
    vertices: [[0, 0, 0], [1, 0, 0], [0, 1, 0]].map(point => ({ point, tolerance: 1e-7 })),
    edges: [[0, 1], [1, 2], [2, 0]].map(([start, end]) => ({ start, end, curve: 'Line', tolerance: null })),
    wires: [{ edges: [0, 1, 2].map(edge => ({ edge, forward: true })), closed: true }],
    faces: [{ outer_wire: 0, inner_wires: [], surface: { Plane: { normal: [0, 0, 1], d: 0 } }, reversed: false }],
    shells: [{ faces: [0] }], solids: [{ outer_shell: 0, inner_shells: [] }], solid_roots: [0],
    boundary_authority: {
      loops: [{ coedges: [0, 1, 2], closed: true }],
      coedges: [0, 1, 2].map(edge => ({ edge, forward: true, periodic_winding: [0, 0] })),
      faces: [{ outer_loop: 0, inner_loops: [] }],
    },
  };
}
const canonical = doc => canonicalArena(Buffer.from(JSON.stringify(doc)));

function repeatedUseDocument(sequence) {
  const doc = document();
  doc.wires[0].edges = sequence.map(edge => ({ edge, forward: true }));
  doc.boundary_authority.coedges = sequence.map(edge => ({ edge, forward: true, periodic_winding: [0, 0] }));
  doc.boundary_authority.loops[0].coedges = sequence.map((_, index) => index);
  return doc;
}

test('renumbering and loop start do not change exact boundary content', () => {
  const original = document(), changed = document();
  changed.vertices.reverse();
  changed.edges = changed.edges.map(edge => ({ ...edge, start: 2 - edge.start, end: 2 - edge.end })).reverse();
  changed.wires[0].edges = changed.wires[0].edges.map(use => ({ ...use, edge: 2 - use.edge }));
  changed.wires[0].edges = changed.wires[0].edges.slice(1).concat(changed.wires[0].edges.slice(0, 1));
  changed.boundary_authority.coedges = changed.boundary_authority.coedges.map(use => ({ ...use, edge: 2 - use.edge }));
  changed.boundary_authority.loops[0].coedges = [1, 2, 0];
  assert.deepEqual(canonical(changed), canonical(original));
});

test('complete rotations resolve repeated minimum coedge tokens without changing cyclic content', () => {
  for (const sequence of [[0, 1, 0, 2], [0, 2, 0, 1], [0, 1, 0, 1], [0, 0, 0, 0], [2, 0, 1, 0, 0, 1]]) {
    const expected = canonical(repeatedUseDocument(sequence));
    for (let shift = 1; shift < sequence.length; shift++) {
      const rotated = sequence.slice(shift).concat(sequence.slice(0, shift));
      assert.deepEqual(canonical(repeatedUseDocument(rotated)), expected, `sequence ${sequence}, shift ${shift}`);
    }
  }
});

test('repeated minimum normalization still refuses changed order, traversal and periodic winding', () => {
  const original = repeatedUseDocument([0, 1, 0, 2]);
  // The same token multiset in a non-cyclic order must remain different.
  assert.notDeepEqual(canonical(repeatedUseDocument([0, 0, 1, 2])), canonical(original));
  for (const mutate of [
    doc => { doc.boundary_authority.coedges[2].forward = false; },
    doc => { doc.boundary_authority.coedges[2].periodic_winding = [1, 0]; },
    doc => { doc.boundary_authority.coedges[2].periodic_winding = [-1, 0]; },
  ]) {
    const changed = repeatedUseDocument([0, 1, 0, 2]); mutate(changed);
    assert.notDeepEqual(canonical(changed), canonical(original));
  }
});

test('reversing both a line carrier and its directed use preserves traversal', () => {
  const original = document(), changed = document();
  [changed.edges[0].start, changed.edges[0].end] = [changed.edges[0].end, changed.edges[0].start];
  changed.wires[0].edges[0].forward = false;
  changed.boundary_authority.coedges[0].forward = false;
  assert.deepEqual(canonical(changed), canonical(original));
});

test('stored inherited versus explicit tolerance never normalizes away', () => {
  const original = document(), changed = document();
  changed.edges[0].tolerance = 1e-7;
  assert.notDeepEqual(canonical(changed), canonical(original));
});

test('numeric signed zero and finite sub-ulp surface changes remain distinct', () => {
  const original = document(), changed = document();
  changed.faces[0].surface.Plane.d = -0;
  // JSON.stringify erases -0; use an explicit JSON token to exercise decoding.
  const encoded = JSON.stringify(changed).replace('"d":0', '"d":-0.0');
  assert.notDeepEqual(canonicalArena(Buffer.from(encoded)), canonical(original));
  changed.faces[0].surface.Plane.d = Number.MIN_VALUE;
  assert.notDeepEqual(canonical(changed), canonical(original));
});

test('per-use pcurves, winding, trim and vertex tolerance remain authoritative', () => {
  const original = document();
  for (const mutate of [
    doc => { doc.boundary_authority.coedges[0].pcurve = { curve: { Line2D: [[0, 0], [1, 0]] }, range: [0, 1] }; },
    doc => { doc.boundary_authority.coedges[0].periodic_winding = [1, 0]; },
    doc => { doc.edges[0].trim = [0, 1]; },
    doc => { doc.vertices[0].tolerance = 2e-7; },
  ]) {
    const changed = document(); mutate(changed);
    assert.notDeepEqual(canonical(changed), canonical(original));
  }
});

test('unsupported arena schema is refused instead of silently dropping authority', () => {
  const changed = document(); changed.version = 2;
  assert.throws(() => canonical(changed), /Unsupported arena schema/);
});

function coincidentTriangles() {
  const doc = document();
  doc.vertices.push(...structuredClone(doc.vertices));
  doc.edges.push(...doc.edges.map(edge => ({ ...edge, start: edge.start + 3, end: edge.end + 3 })));
  doc.wires.push({ edges: [3, 4, 5].map(edge => ({ edge, forward: true })), closed: true });
  doc.faces.push({ ...structuredClone(doc.faces[0]), outer_wire: 1 });
  doc.shells[0].faces.push(1);
  doc.boundary_authority.coedges.push(...[3, 4, 5].map(edge => ({ edge, forward: true, periodic_winding: [0, 0] })));
  doc.boundary_authority.loops.push({ coedges: [3, 4, 5], closed: true });
  doc.boundary_authority.faces.push({ outer_loop: 1, inner_loops: [] });
  return doc;
}

test('coincident entity content cannot hide changed endpoint or edge sharing', () => {
  const original = coincidentTriangles();
  for (const mutate of [
    // Rewire two triangles into one six-edge cycle. Every point, edge shape,
    // entity count and vertex degree is unchanged; only vertex aliasing differs.
    doc => { doc.edges[2].end = 3; doc.edges[5].end = 0; },
    // Retain both edge tables and all boundary-use shapes, but share the first
    // triangle's edges instead of the second triangle's coincident distinct ones.
    doc => {
      doc.wires[1].edges = [0, 1, 2].map(edge => ({ edge, forward: true }));
      for (let index = 0; index < 3; index++) doc.boundary_authority.coedges[index + 3].edge = index;
    },
    doc => { doc.faces[1].outer_wire = 0; },
    doc => { doc.boundary_authority.faces[1].outer_loop = 0; },
    doc => { doc.boundary_authority.loops[1].coedges = [0, 1, 2]; },
  ]) {
    const changed = structuredClone(original); mutate(changed);
    const before = canonical(original), after = canonical(changed);
    assert.deepEqual(after.counts, before.counts);
    assert.deepEqual(after.roots, before.roots, 'Boundary content should remain identical in this injected negative');
    assert.notDeepEqual(after.incidence, before.incidence, 'Changed sharing must fail incidence comparison');
  }
});

test('incidence comparison preserves renumbered coincident distinct entities', () => {
  const original = coincidentTriangles(), changed = structuredClone(original);
  const tables = ['vertices', 'edges', 'wires', 'faces', 'shells', 'solids'];
  const reverse = (kind, id) => original[kind].length - 1 - id;
  for (const edge of changed.edges) { edge.start = reverse('vertices', edge.start); edge.end = reverse('vertices', edge.end); }
  for (const wire of changed.wires) {
    for (const use of wire.edges) use.edge = reverse('edges', use.edge);
    wire.edges = wire.edges.slice(1).concat(wire.edges.slice(0, 1));
  }
  for (const face of changed.faces) { face.outer_wire = reverse('wires', face.outer_wire); face.inner_wires = face.inner_wires.map(id => reverse('wires', id)); }
  for (const shell of changed.shells) shell.faces = shell.faces.map(id => reverse('faces', id)).reverse();
  for (const solid of changed.solids) { solid.outer_shell = reverse('shells', solid.outer_shell); solid.inner_shells = solid.inner_shells.map(id => reverse('shells', id)); }
  changed.solid_roots = changed.solid_roots.map(id => reverse('solids', id));
  for (const kind of tables) changed[kind].reverse();
  const authority = changed.boundary_authority;
  for (const coedge of authority.coedges) coedge.edge = reverse('edges', coedge.edge);
  for (const loop of authority.loops) {
    loop.coedges = loop.coedges.map(id => authority.coedges.length - 1 - id);
    loop.coedges = loop.coedges.slice(1).concat(loop.coedges.slice(0, 1));
  }
  for (const face of authority.faces) { face.outer_loop = authority.loops.length - 1 - face.outer_loop; face.inner_loops = face.inner_loops.map(id => authority.loops.length - 1 - id); }
  authority.faces.reverse(); authority.loops.reverse(); authority.coedges.reverse();
  assert.deepEqual(canonical(changed), canonical(original));
});

test('incidence qualification rejects unsupported roots and invalid references', () => {
  const otherRoot = document(); otherRoot.wire_roots = [0];
  assert.throws(() => canonical(otherRoot), /solid-root documents only/);
  const missingVertex = document(); missingVertex.edges[0].start = 999;
  assert.throws(() => canonical(missingVertex), /Invalid vertices reference/);
});

test('inner boundary order changes retain their wire-to-authority pairing', () => {
  const original = document();
  for (const winding of [1, 2]) {
    original.wires.push(structuredClone(original.wires[0]));
    const first = original.boundary_authority.coedges.length;
    original.boundary_authority.coedges.push(...[0, 1, 2].map(edge => ({ edge, forward: true, periodic_winding: [winding, 0] })));
    original.boundary_authority.loops.push({ coedges: [first, first + 1, first + 2], closed: true });
  }
  original.faces[0].inner_wires = [1, 2];
  original.boundary_authority.faces[0].inner_loops = [1, 2];
  const reordered = structuredClone(original);
  reordered.faces[0].inner_wires.reverse();
  reordered.boundary_authority.faces[0].inner_loops.reverse();
  assert.deepEqual(canonical(reordered), canonical(original));
  // The wires have identical geometry, but distinct identity. Reassigning
  // just one face boundary to the other wire must not collapse their aliases.
  reordered.faces[0].inner_wires = [1, 1];
  assert.notDeepEqual(canonical(reordered), canonical(original));
});

test('boolean trace observes only the requested call and disables tracing before quality work', () => {
  let events = [{ name: 'boolean.route.general_gfa' }], tracing = false;
  const changes = [];
  const kernel = {
    setPerformanceTracing(value) { changes.push(value); tracing = value; events = []; },
    drainPerformanceTrace: () => JSON.stringify(events),
  };
  const result = observeBoolean(kernel, () => {
    assert(tracing);
    events.push({ name: 'boolean.route.store_local_attempt' }, { name: 'boolean.route.store_local_success' }, { name: 'unrelated.phase' });
    return 73;
  });
  assert.equal(result.value, 73);
  assert.equal(tracing, false);
  assert.deepEqual(changes, [true, false]);
  assert.equal(result.routes.diagnosticOnly, true);
  assert.equal(result.routes.timingClaimsExcluded, true);
  assert.deepEqual(result.routes.counts, { general_gfa: 0, store_local_attempt: 1, store_local_success: 1, local_cut_success: 0, nary: 0 });
  assertRoute(result.routes, { name: 'slot_cut' }, true, true);
  assert.throws(() => assertRoute(result.routes, { name: 'slot_cut' }, false, true), /while disabled/);
});

test('missing or truncated route markers stay unknown rather than proving zero local calls', () => {
  const unsupported = observeBoolean({}, () => 1).routes;
  assert.equal(unsupported.status, 'unavailable'); assert.equal(unsupported.counts, null);
  assertRoute(unsupported, { name: 'slot_cut' }, true, false);
  assert.throws(() => assertRoute(unsupported, { name: 'slot_cut' }, true, true), /proof unavailable/);
  for (const phases of [[], [{ name: 'unrelated.phase' }], Array.from({ length: 4096 }, () => ({ name: 'boolean.route.store_local_success' }))]) {
    const changes = [];
    const routes = observeBoolean({ setPerformanceTracing: value => changes.push(value), drainPerformanceTrace: () => JSON.stringify(phases) }, () => 1).routes;
    assert.equal(routes.status, 'unavailable'); assert.equal(routes.counts, null);
    assert.deepEqual(changes, [true, false]);
    assert.throws(() => assertRoute(routes, { name: 'rib_fuse' }, true, true), /proof unavailable/);
  }
});

test('boolean tracing is disabled when the operation or trace decoding fails', () => {
  for (const operationFails of [true, false]) {
    const changes = [];
    const kernel = { setPerformanceTracing: value => changes.push(value), drainPerformanceTrace: () => 'not JSON' };
    assert.throws(() => observeBoolean(kernel, () => { if (operationFails) throw new Error('operation refused'); return 1; }));
    assert.deepEqual(changes, [true, false]);
  }
});

test('route contracts distinguish local completion from declines and wrapper retries', () => {
  const routes = overrides => ({ status: 'observed', counts: { general_gfa: 0, store_local_attempt: 0, store_local_success: 0, local_cut_success: 0, nary: 0, ...overrides } });
  const declined = routes({ general_gfa: 1, store_local_attempt: 1 });
  assertRoute(declined, { name: 'cavity_fallback', kind: 'cavity' }, true, true);
  assertRoute(declined, { name: 'nurbs_patch_fallback', kind: 'nurbs' }, true, true);
  assert.throws(() => assertRoute(declined, { name: 'window_intersect' }, true, true), /did not complete/);
  assertRoute(routes({ store_local_attempt: 2, store_local_success: 1 }), { name: 'nurbs_patch_fallback', kind: 'nurbs' }, true, true);
  assert.throws(() => assertRoute(routes({ store_local_attempt: 1, store_local_success: 1 }), { name: 'nurbs_patch_fallback', kind: 'nurbs' }, true, true), /decline not observed/);
  assert.throws(() => assertRoute(routes({ store_local_attempt: 1, store_local_success: 1 }), { name: 'cavity_fallback', kind: 'cavity' }, true, true), /unexpectedly completed/);
  assertRoute(routes({ general_gfa: 1 }), { name: 'slot_cut' }, false, true);
});

test('the repaired cross-drilled witness requires actual local completion', () => {
  const observed = counts => ({ status: 'observed', counts: { general_gfa: 0, store_local_attempt: 0, store_local_success: 0, local_cut_success: 0, nary: 0, ...counts } });
  const fixture = { name: 'cross_drilled_shaft' };
  assert.throws(() => assertRoute(observed({ store_local_attempt: 1, general_gfa: 1 }), fixture, true, true), /local route did not complete/);
  assertRoute(observed({ store_local_attempt: 1, store_local_success: 1 }), fixture, true, true);
  assertRoute(observed({ general_gfa: 1 }), fixture, false, true);
});

test('recipe stages drain separately and report observed zero successes without assuming locality', () => {
  let events = [];
  const kernel = { setPerformanceTracing() { events = []; }, drainPerformanceTrace: () => JSON.stringify(events) };
  const prepare = observeBoolean(kernel, () => { events.push({ name: 'boolean.route.general_gfa' }); return 'prefix'; });
  const rebuild = observeBoolean(kernel, () => { events.push({ name: 'boolean.route.nary' }); return 'model'; });
  assert.equal(prepare.value, 'prefix'); assert.equal(rebuild.value, 'model');
  assert.equal(prepare.routes.counts.general_gfa, 1);
  assert.equal(rebuild.routes.counts.general_gfa, 0, 'Prefix trace leaked into rebuild');
  assert.equal(rebuild.routes.counts.nary, 1);
  const stages = { prepare: prepare.routes, rebuild: rebuild.routes };
  assertRecipeRoutes(stages, false, true);
  assertRecipeRoutes(stages, true, true); // A requested setting proves no success.
  const unsupported = observeBoolean({}, () => null).routes;
  assertRecipeRoutes({ prepare: unsupported, rebuild: unsupported }, true, false);
  assert.throws(() => assertRecipeRoutes({ prepare: unsupported }, true, true), /proof unavailable/);
});
