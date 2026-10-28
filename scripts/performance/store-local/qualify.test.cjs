'use strict';

const assert = require('node:assert/strict');
const test = require('node:test');
const { canonicalArena } = require('./qualify.cjs');

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

test('renumbering and loop start do not change exact boundary content', () => {
  const original = document(), changed = document();
  changed.vertices.reverse();
  changed.edges = changed.edges.map(edge => ({ ...edge, start: 2 - edge.start, end: 2 - edge.end })).reverse();
  changed.boundary_authority.coedges = changed.boundary_authority.coedges.map(use => ({ ...use, edge: 2 - use.edge }));
  changed.boundary_authority.loops[0].coedges = [1, 2, 0];
  assert.deepEqual(canonical(changed), canonical(original));
});

test('reversing both a line carrier and its directed use preserves traversal', () => {
  const original = document(), changed = document();
  [changed.edges[0].start, changed.edges[0].end] = [changed.edges[0].end, changed.edges[0].start];
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
