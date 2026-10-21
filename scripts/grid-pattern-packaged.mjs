/** B18 grid-pattern history contracts through the actual installed WASM API. */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const kinds = [
  ['face', 'getSolidFaces'], ['edge', 'getSolidEdges'], ['vertex', 'getSolidVertices'],
];
const routes = ['direct', 'executeBatch', 'executeBatchV2'];
const decode = (json) => JSON.parse(json);
const close = (actual, expected, scale, label) => {
  assert.ok(Number.isFinite(actual), label);
  assert.ok(Math.abs(actual - expected) <= 1e-6 * Math.max(scale, Math.abs(expected)), `${label}: ${actual} != ${expected}`);
};
const normalize = (vector) => vector.map((value) => value / Math.hypot(...vector));
const positions = (kernel, solid) => Array.from(kernel.getSolidVertices(solid), (v) => Array.from(kernel.getVertexPosition(v)));

function call(kernel, route, operation, args, direct) {
  if (route === 'direct') return direct();
  const row = decode(kernel[route](JSON.stringify([{ op: operation, args }])))[0];
  assert.ok('ok' in row, `${route} ${operation}: ${JSON.stringify(row)}`);
  return row.ok;
}
function grid(kernel, route, solid, settings) {
  const { dirX, dirY, spacingX, spacingY, countX, countY } = settings;
  return call(kernel, route, 'gridPatternJournaled', { solid, ...settings }, () => decode(
    kernel.gridPatternJournaled(solid, ...dirX, ...dirY, spacingX, spacingY, countX, countY),
  ));
}
function resolveRef(kernel, route, reference) {
  return call(kernel, route, 'resolveRef', { ref: reference }, () => decode(kernel.resolveRef(reference)));
}
function output(kernel, route, op, kind, index) {
  return call(kernel, route, 'resolveOperationOutput', { op, kind, index }, () => decode(kernel.resolveOperationOutput(op, kind, index)));
}
function bound(resolution, kind, label) {
  assert.equal(resolution.status, 'bound', `${label}: ${JSON.stringify(resolution)}`);
  assert.equal(resolution.provenance, 'construction', label);
  assert.equal(resolution.entities.length, 1, label);
  assert.equal(resolution.entities[0].kind, kind, label);
  return resolution.entities[0].handle;
}
function anchors(kernel, solid) {
  const anchor = decode(kernel.linearPatternJournaled(solid, 1, 0, 0, 25, 1));
  return kinds.flatMap(([kind, query]) => {
    const expected = new Set(kernel[query](solid));
    const refs = Array.from({ length: expected.size }, (_, index) => {
      const ref = kernel.makeOperationOutputRef(anchor.op, kind, index);
      return { ref, kind, handle: bound(decode(kernel.resolveRef(ref)), kind, 'pre-grid anchor') };
    });
    assert.deepEqual(new Set(refs.map((r) => r.handle)), expected);
    return refs;
  });
}
function assertOriginals(kernel, route, refs) {
  for (const { ref, kind, handle } of refs) {
    assert.equal(bound(resolveRef(kernel, route, ref), kind, 'untouched original'), handle);
  }
}
function fixture(kernel, shape, scale, placed) {
  let solid;
  let volume;
  if (shape === 'cylinder') {
    solid = kernel.makeCylinder(2 * scale, 5 * scale);
    volume = 20 * Math.PI * scale ** 3;
  } else {
    solid = kernel.makeBox(10 * scale, 10 * scale, 10 * scale);
    volume = 1000 * scale ** 3;
    if (shape === 'hollow') {
      const tool = kernel.makeBox(6 * scale, 6 * scale, 6 * scale);
      kernel.transformSolid(tool, Float64Array.from([1, 0, 0, 2 * scale, 0, 1, 0, 2 * scale, 0, 0, 1, 2 * scale, 0, 0, 0, 1]));
      solid = kernel.cut(solid, tool);
      volume -= 216 * scale ** 3;
    }
  }
  const angle = placed ? 0.7 : 0;
  const c = Math.cos(angle), s = Math.sin(angle);
  if (placed) kernel.transformSolid(solid, Float64Array.from([c, -s, 0, 100, s, c, 0, -50, 0, 0, 1, 30, 0, 0, 0, 1]));
  return { solid, volume, dirX: [c, s, 0], dirY: [-s, c, 0] };
}

export function runGridPatternPackaged({ BrepKernel }) {
  assert.equal(typeof BrepKernel.prototype.gridPatternJournaled, 'function', 'journaled grid export');
  const cells = [
    { shape: 'box', scale: 1, countX: 1, countY: 1 },
    { shape: 'box', scale: 0.001, countX: 1, countY: 3 },
    { shape: 'box', scale: 1000, countX: 3, countY: 1 },
    { shape: 'box', scale: 1, countX: 2, countY: 3 },
    { shape: 'box', scale: 1, countX: 2, countY: 2, touching: true },
    { shape: 'box', scale: 1, countX: 2, countY: 2, placed: true },
    { shape: 'box', scale: 1, countX: 2, countY: 2, skew: true },
    { shape: 'cylinder', scale: 0.001, countX: 2, countY: 2 },
    { shape: 'hollow', scale: 1, countX: 2, countY: 2 },
  ];
  for (const route of routes) for (const cell of cells) {
    const kernel = new BrepKernel();
    try {
      const { solid, volume, dirX, dirY } = fixture(kernel, cell.shape, cell.scale, cell.placed);
      const refs = anchors(kernel, solid);
      const checkpoint = kernel.checkpoint(); // Before the operation being invalidated.
      const before = kernel.serializeSolids(Uint32Array.of(solid));
      const settings = {
        dirX: dirX.map((n) => n * 2),
        dirY: cell.skew ? [3, 3, 0] : dirY.map((n) => n * 3),
        spacingX: (cell.touching ? 10 : 25) * cell.scale,
        spacingY: (cell.touching ? 10 : 25) * cell.scale,
        countX: cell.countX, countY: cell.countY,
      };
      const result = grid(kernel, route, solid, settings);
      const members = Array.from(kernel.getCompoundSolids(result.compound));
      assert.equal(members.length, cell.countX * cell.countY);
      assert.equal(members[0], solid);
      assert.equal(new Set(members).size, members.length);
      const resultRefs = [];
      for (const [kind, query] of kinds) {
        const live = new Set(members.flatMap((member) => Array.from(kernel[query](member))));
        assert.equal(live.size, kernel[query](solid).length * members.length);
        const resolved = new Set();
        for (let index = 0; index < live.size; index++) {
          const handle = bound(output(kernel, route, result.op, kind, index), kind, 'grid census');
          assert.ok(!resolved.has(handle), 'duplicate output subject');
          resolved.add(handle);
          resultRefs.push({ kind, handle, ref: kernel.makeOperationOutputRef(result.op, kind, index) });
        }
        assert.deepEqual(resolved, live, 'resolution census equals actual result entities');
        assert.equal(output(kernel, route, result.op, kind, live.size).status, 'noMatch', 'no phantom outputs');
      }
      const entry = decode(kernel.journalSummary()).find((e) => e.op === result.op);
      assert.equal(entry.kind, 'grid_pattern');
      assert.equal(entry.type, 'evolution');
      assert.equal(entry.detail.origin, 'construction');
      assert.equal(entry.detail.events, resultRefs.length);
      const sourcePoints = positions(kernel, solid);
      const dx = normalize(settings.dirX), dy = normalize(settings.dirY);
      for (let iy = 0; iy < cell.countY; iy++) for (let ix = 0; ix < cell.countX; ix++) {
        const member = members[iy * cell.countX + ix];
        const actual = positions(kernel, member);
        const offset = dx.map((n, axis) => n * settings.spacingX * ix + dy[axis] * settings.spacingY * iy);
        assert.equal(actual.length, sourcePoints.length);
        for (const source of sourcePoints) {
          const wanted = source.map((n, axis) => n + offset[axis]);
          assert.ok(actual.some((p) => Math.hypot(...p.map((n, axis) => n - wanted[axis])) <= 1e-6 * cell.scale), 'normalized row/column translation');
        }
        close(kernel.volume(member, 0.01 * cell.scale), volume, volume, 'closed-form member volume');
        assert.equal(kernel.validateSolid(member), 0);
      }
      assertOriginals(kernel, route, refs);
      if (cell.shape === 'box' && cell.countX === 2 && cell.countY === 3) {
        const bytes = kernel.serializeSolids(Uint32Array.from(members));
        const imported = new BrepKernel();
        try {
          imported.makeBox(1, 1, 1); // Force fresh imported entity handles.
          const roots = Array.from(imported.deserializeSolids(bytes));
          for (const [kind, query] of kinds) {
            const wanted = new Set(roots.flatMap((member) => Array.from(imported[query](member))));
            const got = new Set(resultRefs.filter((r) => r.kind === kind).map((r) => bound(decode(imported.resolveRef(r.ref)), kind, 'imported grid ref')));
            assert.deepEqual(got, wanted);
          }
        } finally { imported.free(); }
        const edited = members.at(-1);
        const untouched = new Set(kinds.flatMap(([kind, query]) =>
          members.slice(0, -1).flatMap((member) => Array.from(kernel[query](member), (h) => `${kind}:${h}`)),
        ));
        kernel.moveFacesJournaled(edited, Uint32Array.of(kernel.getSolidFaces(edited)[0]), cell.scale);
        assertOriginals(kernel, route, refs);
        for (const r of resultRefs) if (untouched.has(`${r.kind}:${r.handle}`)) {
          assert.equal(bound(resolveRef(kernel, route, r.ref), r.kind, 'untouched grid instance'), r.handle);
        }
      }
      kernel.restore(checkpoint);
      assertOriginals(kernel, route, refs);
      const restored = decode(new TextDecoder().decode(kernel.serializeSolids(Uint32Array.of(solid))));
      const saved = decode(new TextDecoder().decode(before));
      // Restore retires operation/ordinal identifiers permanently. All other
      // document content, including retained history, is byte-value exact.
      for (const counter of ['next_op', 'next_ordinal']) {
        assert.ok(restored.journal[counter] >= saved.journal[counter]);
        delete restored.journal[counter];
        delete saved.journal[counter];
      }
      assert.deepEqual(restored, saved);
      for (const r of resultRefs) assert.equal(resolveRef(kernel, route, r.ref).status, 'unknownOperation');
      assert.throws(() => kernel.getCompoundSolids(result.compound), /invalid/i);
      const next = grid(kernel, route, solid, settings);
      assert.ok(next.op > result.op, 'restored operation ids remain retired');
      for (const r of resultRefs) assert.equal(resolveRef(kernel, route, r.ref).status, 'unknownOperation');
    } finally { kernel.free(); }
  }

  for (const route of routes) {
    const kernel = new BrepKernel();
    try {
      const solid = kernel.makeBox(10, 10, 10);
      const retired = kernel.makeBox(1, 1, 1);
      kernel.deleteSolid(retired);
      const refs = anchors(kernel, solid);
      const normal = { dirX: [1, 0, 0], dirY: [0, 1, 0], spacingX: 25, spacingY: 25, countX: 2, countY: 2 };
      const invalids = [
        ...[0, -1, 1.5, 2 ** 32 + 1, 10001].map((countX) => ({ countX })),
        { countY: 0 }, { countY: 1.5 }, { countX: 101, countY: 100 },
        { spacingX: 0 }, { spacingY: -1 }, { spacingX: 1e-12 },
        { dirX: [0, 0, 0] }, { dirY: [2, 0, 0] }, { dirY: [-1, 0, 0] },
        { solid: retired },
        { spacingX: 5, countX: 2, countY: 1, overlap: true },
      ];
      if (route === 'direct') invalids.push({ spacingX: NaN }, { countX: Infinity });
      else invalids.push(
        { dirX: [] }, { dirX: [1, 0] }, { dirY: [0, 1, 0, 2] },
        { dirX: [1, 'x', 0] }, { dirY: null }, { countX: '2' }, { countY: null },
        { spacingX: null }, { spacingY: '25' },
      );
      for (const invalid of invalids) {
        const { overlap, ...overrides } = invalid;
        const args = { solid, ...normal, ...overrides };
        const stats = decode(kernel.checkpointMemoryStats());
        const before = { arena: kernel.serializeSolids(Uint32Array.of(solid)), journal: kernel.journalSummary(), slots: stats.allocatedSlots, retired: stats.retiredSlots };
        if (route === 'direct') assert.throws(() => kernel.gridPatternJournaled(args.solid, ...args.dirX, ...args.dirY, args.spacingX, args.spacingY, args.countX, args.countY));
        else {
          const response = decode(kernel[route](JSON.stringify([{ op: 'gridPatternJournaled', args }, { op: 'volume', args: { solid, deflection: 0.01 } }])));
          assert.ok(response[0].error, `${route}: ${JSON.stringify(invalid)}`);
          if (route === 'executeBatchV2') {
            if (overlap) {
              assert.equal(response[0].error.code, 'operation_failed');
              assert.equal(response[0].error.category, 'unsupported');
              assert.equal(response[0].error.details.kernelCode, 'pattern_instances_overlap');
            } else assert.equal(response[0].error.code, args.solid === retired ? 'invalid_handle' : 'invalid_argument', JSON.stringify(invalid));
          }
          close(response[1].ok, 1000, 1000, 'batch continues after refusal');
        }
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(solid)), before.arena);
        assert.equal(kernel.journalSummary(), before.journal);
        const after = decode(kernel.checkpointMemoryStats());
        assert.equal(after.allocatedSlots - after.retiredSlots, before.slots - before.retired, 'refusal preserves live entity census');
        if (overlap) {
          assert.ok(after.allocatedSlots > before.slots, 'failed copy allocations remain retired');
          assert.ok(after.retiredSlots > before.retired, 'every failed copy allocation is retired');
        } else {
          assert.equal(after.allocatedSlots, before.slots);
          assert.equal(after.retiredSlots, before.retired);
        }
        assertOriginals(kernel, route, refs);
      }
    } finally { kernel.free(); }
  }
  console.log('ok - journaled grid: full F/E/V census, normalized placement, original-only naming, persistence/restore and typed rollback');
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const consumer = process.env.REMUS_CONSUMER_DIR;
  const root = process.env.REMUS_PACKAGE_ROOT || resolve(dirname(fileURLToPath(import.meta.url)), '..');
  const require = createRequire(resolve(consumer || root, 'grid-history-witness.cjs'));
  runGridPatternPackaged(require(consumer ? 'remus-wasm' : resolve(root, 'crates/wasm/pkg/remus_wasm_node.cjs')));
}
