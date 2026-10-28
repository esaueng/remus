'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { test } = require('node:test');
const { main, options, distanceAt, summarize, memoryDelta, workContract } = require('./drag_session_bench.cjs');
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

test('tick and warmup counts leave a post-cold warmed sample', () => {
  for (const [ticks, warmup] of [[1, 0], [1, 1], [2, 2], [2, 3]]) {
    assert.throws(() => options([...required, '--ticks', String(ticks), '--warmup', String(warmup)]),
      /retained sample/);
  }
  for (const [ticks, warmup] of [[2, 0], [2, 1], [3, 2]]) {
    const result = options([...required, '--ticks', String(ticks), '--warmup', String(warmup)]);
    assert.equal(result.ticks, ticks); assert.equal(result.warmup, warmup);
  }
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

function workRecords(backgroundSlots = 0) {
  const source = 3, initial = backgroundSlots + source;
  const records = [];
  let allocated = initial, retired = 0;
  for (let tick = 0; tick < 3; tick++) {
    const before = { allocatedSlots: allocated, retiredSlots: retired };
    if (tick) retired += source;
    const restore = { allocatedSlots: allocated, retiredSlots: retired };
    allocated += source + (tick === 0 ? initial : 0);
    const move = { allocatedSlots: allocated, retiredSlots: retired };
    records.push({ tick, accepted: true,
      memoryStages: { before, restore, move, validate: { ...move }, mesh: { ...move }, transfer: { ...move } },
      memoryAfter: { ...move } });
  }
  return { records, initial: { allocatedSlots: initial, retiredSlots: 0 } };
}

test('the work budget permits one document copy and keeps recurring work body-local', () => {
  for (const background of [0, 200 * 64]) {
    const { records, initial } = workRecords(background);
    const result = workContract(records, 3, initial);
    assert.equal(result.passed, true);
    assert.equal(result.firstMoveSlotBudget, background + 6);
    assert.equal(result.maximumRecurringSlots, 3);
    assert.equal(result.recurringMoveSlotBudget, 3);
  }
});

test('an extra recurring document copy cannot hide behind matching average growth', () => {
  const { records, initial } = workRecords(200 * 64);
  // A second edit copies unrelated topology; a later apparent reclamation
  // cancels it in the final total. Every individual stage must still fail.
  records[1].memoryStages.move.allocatedSlots += initial.allocatedSlots;
  const result = workContract(records, 3, initial);
  assert.equal(result.passed, false);
  assert.ok(result.violations.some(v => v.tick === 1 && v.stage === 'move'));
  assert.ok(result.violations.some(v => v.tick === 1 && v.stage === 'validate'));
});

test('read-only stages, retirement and untimed reads cannot spend topology slots', () => {
  for (const stage of ['restore', 'validate', 'mesh', 'transfer']) {
    const { records, initial } = workRecords();
    records[1].memoryStages[stage].allocatedSlots++;
    assert.equal(workContract(records, 3, initial).passed, false, stage);
  }
  for (const mutate of [
    r => { r[1].memoryStages.before.retiredSlots--; },
    r => { r[1].memoryStages.move.retiredSlots++; },
    r => { r[1].memoryAfter.allocatedSlots++; },
    r => { r[1].memoryStages.restore.allocatedSlots = Number.NaN; },
    r => { r[1].memoryStages.restore.retiredSlots = -1; },
    r => { delete r[1].memoryStages.mesh; },
    r => { r[1].accepted = false; },
    r => { r[1].tick = 2; },
  ]) {
    const { records, initial } = workRecords();
    mutate(records);
    assert.equal(workContract(records, 3, initial).passed, false);
  }
  assert.equal(workContract([], 3, { allocatedSlots: 3 }).passed, false);
  assert.throws(() => workContract([], 0, { allocatedSlots: 3 }), /Invalid/);
});

test('missing retirement cannot leak live result bodies under a fixed append budget', () => {
  const { records, initial } = workRecords();
  for (const row of records) {
    for (const snapshot of Object.values(row.memoryStages)) snapshot.retiredSlots = 0;
    row.memoryAfter.retiredSlots = 0;
  }
  const result = workContract(records, 3, initial);
  assert.equal(result.passed, false);
  assert.ok(result.violations.some(v => v.message.startsWith('Live topology exceeds')));
});
