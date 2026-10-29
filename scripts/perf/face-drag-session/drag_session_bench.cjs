#!/usr/bin/env node
'use strict';

// One kernel per process, driven the way the OpenZCAD drag preview drives it:
// checkpoint the source once, then per tick restore, move, validate and mesh.
// Diagnostics (memory stats, face counts) are read outside the timed calls.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const crypto = require('node:crypto');
const { performance } = require('node:perf_hooks');
const { kernelClass } = require('../../kernel-class.cjs');
const { meshQuality } = require('../filleted-face-edit/wasm_edit_bench.cjs');

const TARGET_AREA = 1045.930731242697;
// Selection guard only (native/WASM planar sampling differs by ~1.2e-6 mm²).
const SELECTION_AREA_TOLERANCE = 1e-4;
const TARGET_X = 48;
const DISPLAY_DEFLECTION = 0.02;
const DISPLAY_ANGLE = 0.35;
const STAGES = ['restore', 'move', 'validate', 'mesh'];
const TIMINGS = [...STAGES, 'transfer', 'tick', 'tickWithTransfer', 'observedWall'];

function options(argv) {
  const result = { ticks: 200, move: 'prepared', background: 0, window: 20, label: 'variant',
    warmup: 2, pattern: 'repeated', 'timing-status': 'measured' };
  const allowed = new Set(['kernel-pkg', 'io-pkg', 'fixture', 'out', 'label', 'ticks', 'move',
    'background', 'window', 'warmup', 'pattern', 'timing-status', 'package-source-sha']);
  for (let i = 0; i < argv.length; i += 2) {
    const name = argv[i].replace(/^--/, '');
    if (!argv[i].startsWith('--') || !allowed.has(name) || argv[i + 1] === undefined) {
      throw new Error(`Unknown or incomplete argument: ${argv[i]}`);
    }
    result[name] = argv[i + 1];
  }
  for (const key of ['kernel-pkg', 'io-pkg', 'fixture']) {
    if (!result[key]) throw new Error(`Missing --${key}`);
    result[key] = path.resolve(result[key]);
  }
  if (result.out) result.out = path.resolve(result.out);
  for (const key of ['ticks', 'background', 'window', 'warmup']) {
    result[key] = Number(result[key]);
    if (!Number.isSafeInteger(result[key]) ||
        result[key] < (['background', 'warmup'].includes(key) ? 0 : 1)) {
      throw new Error(`Invalid --${key}`);
    }
  }
  if (result.ticks <= Math.max(result.warmup, 1)) {
    throw new Error('Ticks must leave a retained sample after cold and warmup exclusions');
  }
  if (!['prepared', 'unprepared'].includes(result.move)) throw new Error('Invalid --move');
  if (!['unique', 'repeated'].includes(result.pattern)) throw new Error('Invalid --pattern');
  if (!['measured', 'contended-smoke-excluded'].includes(result['timing-status'])) {
    throw new Error('Invalid --timing-status');
  }
  if (result['package-source-sha'] && !/^[0-9a-f]{40}$/.test(result['package-source-sha'])) {
    throw new Error('Invalid --package-source-sha');
  }
  return result;
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function sha256(bytes) {
  return crypto.createHash('sha256').update(bytes).digest('hex');
}

function timed(action) {
  const start = performance.now();
  const result = action();
  return { result, ms: performance.now() - start };
}

function packageMetadata(directory, stem) {
  const info = JSON.parse(fs.readFileSync(path.join(directory, 'package.json'), 'utf8'));
  const wasm = fs.readFileSync(path.join(directory, `${stem}_bg.wasm`));
  return { directory, name: info.name, version: info.version, wasmBytes: wasm.length,
    wasmSha256: sha256(wasm),
    packageJsonSha256: sha256(fs.readFileSync(path.join(directory, 'package.json'))),
    wrapperSha256: sha256(fs.readFileSync(path.join(directory, `${stem}_node.cjs`))) };
}

// Index of the holder's outward +X cap at x=48 within the solid's face list.
function sourceFaceIndex(kernel, solid) {
  const faces = Array.from(kernel.getSolidFaces(solid));
  const candidates = [];
  for (let index = 0; index < faces.length; index++) {
    const face = faces[index];
    if (JSON.parse(kernel.getAnalyticSurfaceParams(face)).type !== 'plane') continue;
    if (kernel.getFaceNormal(face)[0] <= 0.99) continue;
    candidates.push({ index, score: Math.abs(kernel.faceArea(face, 0.05) - TARGET_AREA),
      xs: Array.from(kernel.getFaceVertices(face), vertex => kernel.getVertexPosition(vertex)[0]) });
  }
  candidates.sort((a, b) => a.score - b.score || a.index - b.index);
  const selected = candidates[0];
  assert(selected && selected.score < SELECTION_AREA_TOLERANCE, 'Cannot identify the holder edit face');
  assert(selected.xs.every(x => Math.abs(x - TARGET_X) < 1e-8), 'The selected face must lie on x=48');
  return selected.index;
}

// Outward from 5 mm and back every 20 ticks, so long drags revisit distances.
function distanceAt(tick, pattern = 'repeated') {
  if (pattern === 'unique') return 5 + 0.02 * tick;
  const phase = tick % 20;
  return 5 + 0.2 * (phase <= 10 ? phase : 20 - phase);
}

function memory(kernel) {
  const stats = JSON.parse(kernel.checkpointMemoryStats());
  const processMemory = process.memoryUsage();
  return { allocatedSlots: stats.allocatedSlots, retiredSlots: stats.retiredSlots,
    linearMemoryBytes: stats.linearMemoryBytes, processRssBytes: processMemory.rss,
    jsHeapUsedBytes: processMemory.heapUsed, externalBytes: processMemory.external,
    arrayBufferBytes: processMemory.arrayBuffers };
}

function memoryDelta(before, after) {
  return Object.fromEntries(Object.keys(before).map(key => [key, after[key] - before[key]]));
}

function extractMesh(grouped) {
  try {
    return { positions: grouped.takePositions(), indices: grouped.takeIndices(),
      normals: grouped.normals, faceOffsets: grouped.takeFaceOffsets() };
  } finally { grouped.free(); }
}

function percentile(values, percent) {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.max(0, Math.ceil(sorted.length * percent / 100) - 1)];
}

function quantiles(values) {
  return values.length === 0 ? null :
    { n: values.length, p50: percentile(values, 50), p95: percentile(values, 95), max: Math.max(...values) };
}

function summarize(records, window, before) {
  const accepted = records.filter(record => record.accepted);
  const stageQuantiles = rows => Object.fromEntries(
    TIMINGS.map(stage => [stage, quantiles(rows.map(row => row.stagesMs[stage]))]));
  const warm = accepted.filter(record => !record.cold && !record.warmup);
  const first = records[0].memoryAfter;
  const last = records.at(-1).memoryAfter;
  return {
    ticks: records.length,
    refused: records.length - accepted.length,
    accepted: stageQuantiles(accepted),
    cold: records[0],
    warmedSamples: warm.length,
    warmed: stageQuantiles(warm),
    firstSeenSamples: warm.filter(record => !record.revisited).length,
    firstSeen: stageQuantiles(warm.filter(record => !record.revisited)),
    revisitedSamples: warm.filter(record => record.revisited).length,
    revisited: stageQuantiles(warm.filter(record => record.revisited)),
    memoryGrowthByStage: Object.fromEntries([...STAGES, 'transfer'].map(stage => [stage,
      Object.fromEntries(Object.keys(before).map(key => [key,
        quantiles(accepted.map(record => record.memoryDeltas[stage][key]))]))])),
    // Early and late windows of accepted ticks expose drift from arena growth.
    firstWindow: stageQuantiles(accepted.slice(0, window)),
    lastWindow: stageQuantiles(accepted.slice(-window)),
    // The first write after the checkpoint copies the whole topology once, and
    // the stats then count the checkpoint and the live copy separately. That
    // one-time copy is reported apart from the recurring per-tick growth.
    firstTickSlots: first.allocatedSlots - before.allocatedSlots,
    slotsPerTick: records.length > 1 ?
      (last.allocatedSlots - first.allocatedSlots) / (records.length - 1) : null,
    memoryBefore: before,
    memoryLast: last,
  };
}

// A selected-body edit may copy the checkpoint's document once, then append
// at most one source body's topology per tick. Retired slots intentionally
// remain reserved: this gate never requires recycling external handles.
// These are retained arena-slot budgets, not allocation-byte or CPU budgets.
function workContract(records, sourceSlots, before) {
  const violations = [];
  const fail = (tick, stage, message) => violations.push({ tick, stage, message });
  if (!Number.isSafeInteger(sourceSlots) || sourceSlots <= 0 ||
      !Number.isSafeInteger(before.allocatedSlots) || before.allocatedSlots < sourceSlots) {
    throw new Error('Invalid source/document topology inventory');
  }
  if (records.length < 2) fail(-1, 'capture', 'Need multiple edits to qualify recurring work');
  let previous = before;
  let maximumRecurringSlots = 0;
  for (const [index, row] of records.entries()) {
    if (row.tick !== index || !row.accepted) fail(index, 'result', 'Missing or refused edit');
    const stages = row.memoryStages || {};
    const snapshots = ['before', ...STAGES, 'transfer'].map(stage => [stage, stages[stage]]);
    for (const [stage, snapshot] of snapshots) {
      if (!snapshot || !['allocatedSlots', 'retiredSlots'].every(key =>
        Number.isSafeInteger(snapshot[key]) && snapshot[key] >= 0) ||
          snapshot.retiredSlots > snapshot.allocatedSlots) {
        fail(index, stage, 'Invalid topology-slot snapshot');
      }
    }
    if (snapshots.some(([, snapshot]) => !snapshot ||
        !Number.isSafeInteger(snapshot.allocatedSlots) || !Number.isSafeInteger(snapshot.retiredSlots))) continue;
    if (stages.before.allocatedSlots !== previous.allocatedSlots ||
        stages.before.retiredSlots !== previous.retiredSlots) {
      fail(index, 'before', 'Untimed diagnostics changed topology slots');
    }
    let prior = stages.before;
    for (const stage of [...STAGES, 'transfer']) {
      const current = stages[stage];
      const allocated = current.allocatedSlots - prior.allocatedSlots;
      const retired = current.retiredSlots - prior.retiredSlots;
      const budget = stage === 'move' ? sourceSlots + (index === 0 ? before.allocatedSlots : 0) : 0;
      if (allocated < 0 || allocated > budget) {
        fail(index, stage, `Allocated ${allocated} topology slots; budget ${budget}`);
      }
      if (retired < 0 || retired > (stage === 'restore' ? sourceSlots : 0)) {
        fail(index, stage, `Retired ${retired} topology slots outside restore's source-body budget`);
      }
      if (index > 0 && stage === 'move') maximumRecurringSlots = Math.max(maximumRecurringSlots, allocated);
      prior = current;
    }
    if (!row.memoryAfter || row.memoryAfter.allocatedSlots !== prior.allocatedSlots ||
        row.memoryAfter.retiredSlots !== prior.retiredSlots) {
      fail(index, 'quality', 'Untimed quality checks changed topology slots');
    }
    if (prior.allocatedSlots - prior.retiredSlots > 2 * before.allocatedSlots + sourceSlots) {
      fail(index, 'result', 'Live topology exceeds the checkpoint, its document copy and one edited body');
    }
    previous = row.memoryAfter || prior;
  }
  return { schema: 1, passed: violations.length === 0, sourceSlots,
    initialDocumentSlots: before.allocatedSlots,
    firstMoveSlotBudget: before.allocatedSlots + sourceSlots,
    recurringMoveSlotBudget: sourceSlots, maximumRecurringSlots,
    liveSlotBudget: 2 * before.allocatedSlots + sourceSlots,
    readStageSlotBudget: 0, restoreRetirementSlotBudget: sourceSlots, violations };
}

function main(argv = process.argv.slice(2)) {
  const args = options(argv);
  if (args.out && fs.existsSync(args.out)) throw new Error('Output file already exists');
  const meta = {
    kernel: packageMetadata(args['kernel-pkg'], 'remus_wasm'),
    io: packageMetadata(args['io-pkg'], 'remus_wasm_io'),
    fixtureSha256: sha256(fs.readFileSync(args.fixture)),
    harnessSha256: sha256(fs.readFileSync(__filename)),
    meshQualityHarnessSha256: sha256(fs.readFileSync(
      path.join(__dirname, '../filleted-face-edit/wasm_edit_bench.cjs'))),
    node: process.version, versions: process.versions,
    platform: `${os.platform()} ${os.arch()}`, cpu: os.cpus()[0]?.model,
    affinity: fs.existsSync('/proc/self/status') ?
      fs.readFileSync('/proc/self/status', 'utf8').match(/^Cpus_allowed_list:\s*(.*)$/m)?.[1] : null,
    packageSourceSha: args['package-source-sha'] || null,
    packageSourceProvenance: 'externally supplied; not automatically verified by this driver',
    move: args.move, background: args.background, ticks: args.ticks, warmup: args.warmup,
    pattern: args.pattern, timingStatus: args['timing-status'],
    timingsExcluded: args['timing-status'] !== 'measured',
    timingBoundaries: {
      tick: 'sum of restore/move/relaxed validation/display mesh public kernel-call durations',
      tickWithTransfer: 'tick plus positions/normals/indices/grouping bulk getters and grouped-result free',
      observedWall: 'wall from restore through transfer, including between-stage memory diagnostic reads',
      quality: 'mesh closure, finite buffers, face count and selected cap placement checked after timers',
      excluded: 'consumer orchestration, worker queue, rendering, browser presentation and commit quantities/recognition',
      memory: 'observed arena/WASM/process snapshots; diagnostic reads and JS GC can affect process memory',
      revisited: 'previously requested offset, including refused requests; not a proven cache hit',
    },
    display: { deflection: DISPLAY_DEFLECTION, angle: DISPLAY_ANGLE },
  };
  const RemusKernel = kernelClass(require(path.join(args['kernel-pkg'], 'remus_wasm_node.cjs')));
  const { RemusIo } = require(path.join(args['io-pkg'], 'remus_wasm_io_node.cjs'));
  const io = new RemusIo();
  let imported;
  const importStarted = performance.now();
  try { imported = io.importStep(fs.readFileSync(args.fixture)); } finally { io.free(); }
  meta.importMs = performance.now() - importStarted;

  const kernel = new RemusKernel();
  let prepared = null;
  try {
    for (let i = 0; i < args.background; i++) kernel.makeBox(10, 8, 6);
    const beforeImport = memory(kernel);
    const roots = kernel.deserializeSolids(imported);
    assert(roots.length === 1, `Expected one imported solid; found ${roots.length}`);
    const solid = roots[0];
    meta.sourceTopologySlots = memory(kernel).allocatedSlots - beforeImport.allocatedSlots;
    const faceIndex = sourceFaceIndex(kernel, solid);
    assert(kernel.getSolidFaces(solid).length === 160, 'Expected the 160-face holder');
    const checkpoint = kernel.checkpoint();
    const faces = () => Uint32Array.of(kernel.getSolidFaces(solid)[faceIndex]);
    meta.memoryBeforePrepare = memory(kernel);
    if (args.move === 'prepared') {
      prepared = timed(() => kernel.prepareMoveFaces(solid, faces()));
      meta.prepareMs = prepared.ms;
    }
    meta.memoryAfterPrepare = memory(kernel);
    const before = memory(kernel);
    const records = [];
    const seen = new Set();
    for (let tick = 0; tick < args.ticks; tick++) {
      const distance = distanceAt(tick, args.pattern);
      const revisited = seen.has(distance);
      seen.add(distance);
      const memoryStages = { before: memory(kernel) };
      const wallStarted = performance.now();
      // Restore keeps the source's handles, so the prepared move stays valid.
      const stagesMs = { restore: timed(() => kernel.restore(checkpoint)).ms };
      memoryStages.restore = memory(kernel);
      let moved;
      try {
        moved = args.move === 'prepared' ?
          timed(() => kernel.moveFacesPrepared(prepared.result, distance)) :
          timed(() => kernel.moveFaces(solid, faces(), distance));
      } catch (error) {
        records.push({ tick, distance, cold: tick === 0, warmup: tick < args.warmup, revisited,
          accepted: false, error: String(error.message || error), stagesMs,
          memoryStages, memoryAfter: memory(kernel) });
        continue;
      }
      stagesMs.move = moved.ms;
      memoryStages.move = memory(kernel);
      const validation = timed(() => kernel.validateSolidRelaxed(moved.result));
      stagesMs.validate = validation.ms;
      memoryStages.validate = memory(kernel);
      const grouped = timed(() => kernel.tessellateSolidGroupedBinary(
        moved.result, DISPLAY_DEFLECTION, DISPLAY_ANGLE));
      stagesMs.mesh = grouped.ms;
      memoryStages.mesh = memory(kernel);
      const transfer = timed(() => extractMesh(grouped.result));
      stagesMs.transfer = transfer.ms;
      stagesMs.observedWall = performance.now() - wallStarted;
      memoryStages.transfer = memory(kernel);
      stagesMs.tick = STAGES.reduce((sum, stage) => sum + stagesMs[stage], 0);
      stagesMs.tickWithTransfer = stagesMs.tick + stagesMs.transfer;
      const quality = meshQuality(transfer.result, 160);
      const movedFaces = kernel.getSolidFaces(moved.result);
      assert(movedFaces.length === 160, 'Edited holder face count changed');
      const cap = movedFaces[faceIndex];
      assert(JSON.parse(kernel.getAnalyticSurfaceParams(cap)).type === 'plane', 'Edited cap is not planar');
      const capVertices = Array.from(kernel.getFaceVertices(cap),
        vertex => kernel.getVertexPosition(vertex)[0]);
      assert(capVertices.length > 0 && capVertices.every(x => Math.abs(x - TARGET_X - distance) < 1e-8),
        'Edited cap is not at its requested offset');
      const memoryDeltas = {};
      let previous = memoryStages.before;
      for (const stage of [...STAGES, 'transfer']) {
        memoryDeltas[stage] = memoryDelta(previous, memoryStages[stage]);
        previous = memoryStages[stage];
      }
      records.push({ tick, distance, cold: tick === 0, warmup: tick < args.warmup, revisited,
        accepted: validation.result === 0, stagesMs, triangles: quality.triangles,
        quality: { ...quality, faces: movedFaces.length, capAtRequestedOffset: true },
        memoryStages, memoryDeltas, memoryAfter: memory(kernel) });
    }
    const result = { label: args.label, metadata: meta, summary: summarize(records, args.window, before),
      workContract: workContract(records, meta.sourceTopologySlots, before), records };
    if (args.out) fs.writeFileSync(args.out, JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
    const s = result.summary;
    const line = (name, q) => q ?
      `${name.padEnd(9)} p50 ${q.p50.toFixed(2).padStart(8)}  p95 ${q.p95.toFixed(2).padStart(8)}  max ${q.max.toFixed(2).padStart(8)} ms` :
      `${name.padEnd(9)} none`;
    process.stdout.write([
      `${args.label}: ${meta.kernel.name} ${meta.kernel.version}, move ${args.move}, ` +
        `background ${args.background}, ${s.ticks} ticks, ${s.refused} refused`,
      ...TIMINGS.map(stage => line(stage, s.accepted[stage])),
      `warmed ${s.warmedSamples}; first-seen ${s.firstSeenSamples}; revisited ${s.revisitedSamples}; ` +
        `timing status ${meta.timingStatus}`,
      `tick first ${args.window}: p50 ${s.firstWindow.tick?.p50.toFixed(2)} ms; ` +
        `last ${args.window}: p50 ${s.lastWindow.tick?.p50.toFixed(2)} ms`,
      `slots ${s.memoryBefore.allocatedSlots} -> ${s.memoryLast.allocatedSlots} ` +
        `(first tick ${s.firstTickSlots} including the one-time checkpoint copy, then ` +
        `${s.slotsPerTick === null ? 'n/a' : s.slotsPerTick.toFixed(0)}/tick; ` +
        `${s.memoryLast.retiredSlots} retired), linear memory ` +
        `${s.memoryBefore.linearMemoryBytes} -> ${s.memoryLast.linearMemoryBytes} bytes`,
    ].join('\n') + '\n');
    assert(result.workContract.passed, 'Preview topology work contract failed: ' +
      JSON.stringify(result.workContract.violations));
    return result;
  } finally {
    if (prepared) kernel.releasePreparedMove(prepared.result);
    kernel.free();
  }
}

module.exports = { main, options, distanceAt, summarize, memoryDelta, workContract };
if (require.main === module) {
  try { main(); } catch (error) {
    process.stderr.write(`${error.stack || error}\n`);
    process.exitCode = 1;
  }
}
