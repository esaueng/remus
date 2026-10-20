'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { test } = require('node:test');
const { main, options, distanceAt, summarize, memoryDelta } = require('./drag_session_bench.cjs');
const { meshQuality } = require('../filleted-face-edit/wasm_edit_bench.cjs');

const required = ['--kernel-pkg', '.', '--io-pkg', '.', '--fixture', 'README.md'];

test('unique offsets never repeat requests and the original triangular path stays stable', () => {
  const unique = Array.from({ length: 100 }, (_, i) => distanceAt(i, 'unique'));
  assert.equal(new Set(unique).size, 100);
  assert.equal(unique[0], 5);
  assert.equal(unique[99], 6.98);
  assert.deepEqual([0, 1, 10, 11, 19, 20].map(i => distanceAt(i)), [5, 5.2, 7, 6.8, 5.2, 5]);
});

test('invalid sampling, provenance and timing policies fail before loading WASM', () => {
  for (const args of [
    ['--ticks', '0'], ['--warmup', '-1'], ['--ticks', '1.2'],
    ['--pattern', 'sometimes'], ['--timing-status', 'quiet-ish'],
    ['--package-source-sha', 'unknown'],
  ]) assert.throws(() => options([...required, ...args]), /Invalid/);
  const diagnostic = options([...required, '--timing-status', 'contended-smoke-excluded']);
  assert.equal(diagnostic['timing-status'], 'contended-smoke-excluded');
});

test('existing evidence is refused before package loading and remains byte-identical', () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'remus-drag-output-'));
  const output = path.join(directory, 'existing.json');
  try {
    fs.writeFileSync(output, 'preserved evidence\n');
    assert.throws(() => main([...required, '--out', output]), /Output file already exists/);
    assert.equal(fs.readFileSync(output, 'utf8'), 'preserved evidence\n');
  } finally { fs.rmSync(directory, { recursive: true }); }
});

function record(tick, time, { warmup = false, revisited = false, accepted = true } = {}) {
  const stages = ['restore', 'move', 'validate', 'mesh', 'transfer'];
  return { tick, cold: tick === 0, warmup, revisited, accepted,
    stagesMs: Object.fromEntries([...stages, 'tick', 'tickWithTransfer', 'observedWall']
      .map(stage => [stage, time])),
    memoryDeltas: Object.fromEntries(stages.map(stage => [stage, { allocatedSlots: 1 }])),
    memoryAfter: { allocatedSlots: (tick + 1) * 5 } };
}

test('cold, warmups, first-seen, revisited and refused samples keep distinct denominators', () => {
  const rows = [record(0, 1000, { warmup: true }), record(1, 200, { warmup: true }),
    record(2, 800, { accepted: false }), record(3, 30, { revisited: true }), record(4, 40)];
  const result = summarize(rows, 2, { allocatedSlots: 0 });
  assert.equal(result.ticks, 5);
  assert.equal(result.refused, 1);
  assert.equal(result.cold.stagesMs.tick, 1000);
  assert.equal(result.accepted.tick.max, 1000);
  assert.equal(result.warmedSamples, 2);
  assert.equal(result.warmed.tick.max, 40);
  assert.equal(result.firstSeenSamples, 1);
  assert.equal(result.firstSeen.tick.p95, 40);
  assert.equal(result.revisitedSamples, 1);
  assert.equal(result.revisited.tick.p95, 30);
  assert.equal(result.memoryGrowthByStage.move.allocatedSlots.p50, 1);
  assert.equal(summarize([rows[0]], 2, { allocatedSlots: 0 }).slotsPerTick, null);
});

test('stage memory deltas preserve GC decreases without mutating snapshots', () => {
  const before = { allocatedSlots: 10, jsHeapUsedBytes: 1000 };
  const after = { allocatedSlots: 17, jsHeapUsedBytes: 700 };
  assert.deepEqual(memoryDelta(before, after), { allocatedSlots: 7, jsHeapUsedBytes: -300 });
  assert.deepEqual(before, { allocatedSlots: 10, jsHeapUsedBytes: 1000 });
});

test('the shared output gate accepts a closed tetrahedron and rejects an open display mesh', () => {
  const mesh = { positions: Float32Array.of(0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1),
    normals: new Float32Array(12),
    indices: Uint32Array.of(0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3),
    faceOffsets: Uint32Array.of(0, 3, 6, 9, 12) };
  assert.equal(meshQuality(mesh, 4).watertight, true);
  assert.throws(() => meshQuality({ ...mesh, indices: mesh.indices.slice(0, 9),
    faceOffsets: Uint32Array.of(0, 3, 6, 9) }, 3), /not watertight/);
});
