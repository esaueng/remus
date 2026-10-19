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

const TARGET_AREA = 1045.930731242697;
// Selection guard only (native/WASM planar sampling differs by ~1.2e-6 mm²).
const SELECTION_AREA_TOLERANCE = 1e-4;
const TARGET_X = 48;
const DISPLAY_DEFLECTION = 0.02;
const DISPLAY_ANGLE = 0.35;
const STAGES = ['restore', 'move', 'validate', 'mesh'];

function options(argv) {
  const result = { ticks: 200, move: 'prepared', background: 0, window: 20, label: 'variant' };
  const allowed = new Set(['kernel-pkg', 'io-pkg', 'fixture', 'out', 'label', 'ticks', 'move',
    'background', 'window']);
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
  for (const key of ['ticks', 'background', 'window']) {
    result[key] = Number(result[key]);
    if (!Number.isSafeInteger(result[key]) || result[key] < (key === 'background' ? 0 : 1)) {
      throw new Error(`Invalid --${key}`);
    }
  }
  if (!['prepared', 'unprepared'].includes(result.move)) throw new Error('Invalid --move');
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
    wasmSha256: sha256(wasm) };
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
function distanceAt(tick) {
  const phase = tick % 20;
  return 5 + 0.2 * (phase <= 10 ? phase : 20 - phase);
}

function memory(kernel) {
  const stats = JSON.parse(kernel.checkpointMemoryStats());
  return { allocatedSlots: stats.allocatedSlots, retiredSlots: stats.retiredSlots,
    linearMemoryBytes: stats.linearMemoryBytes };
}

function percentile(values, percent) {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.max(0, Math.ceil(sorted.length * percent / 100) - 1)];
}

function quantiles(values) {
  return values.length === 0 ? null :
    { p50: percentile(values, 50), p95: percentile(values, 95), max: Math.max(...values) };
}

function summarize(records, window) {
  const accepted = records.filter(record => record.accepted);
  const stageQuantiles = rows => Object.fromEntries(
    [...STAGES, 'tick'].map(stage => [stage, quantiles(rows.map(row => row.stagesMs[stage]))]));
  const first = records[0].memoryAfter;
  const last = records.at(-1).memoryAfter;
  return {
    ticks: records.length,
    refused: records.length - accepted.length,
    accepted: stageQuantiles(accepted),
    // Early and late windows of accepted ticks expose drift from arena growth.
    firstWindow: stageQuantiles(accepted.slice(0, window)),
    lastWindow: stageQuantiles(accepted.slice(-window)),
    slotsPerTick: (last.allocatedSlots - first.allocatedSlots) / Math.max(1, records.length - 1),
    memoryFirst: first,
    memoryLast: last,
  };
}

function main(argv = process.argv.slice(2)) {
  const args = options(argv);
  const meta = {
    kernel: packageMetadata(args['kernel-pkg'], 'remus_wasm'),
    io: packageMetadata(args['io-pkg'], 'remus_wasm_io'),
    fixtureSha256: sha256(fs.readFileSync(args.fixture)),
    node: process.version, platform: `${os.platform()} ${os.arch()}`, cpu: os.cpus()[0]?.model,
    move: args.move, background: args.background, ticks: args.ticks,
    display: { deflection: DISPLAY_DEFLECTION, angle: DISPLAY_ANGLE },
  };
  const { BrepKernel } = require(path.join(args['kernel-pkg'], 'remus_wasm_node.cjs'));
  const { RemusIo } = require(path.join(args['io-pkg'], 'remus_wasm_io_node.cjs'));
  const io = new RemusIo();
  let imported;
  try { imported = io.importStep(fs.readFileSync(args.fixture)); } finally { io.free(); }

  const kernel = new BrepKernel();
  let prepared = null;
  try {
    for (let i = 0; i < args.background; i++) kernel.makeBox(10, 8, 6);
    const roots = kernel.deserializeSolids(imported);
    assert(roots.length === 1, `Expected one imported solid; found ${roots.length}`);
    const solid = roots[0];
    const faceIndex = sourceFaceIndex(kernel, solid);
    const checkpoint = kernel.checkpoint();
    const faces = () => Uint32Array.of(kernel.getSolidFaces(solid)[faceIndex]);
    if (args.move === 'prepared') {
      prepared = timed(() => kernel.prepareMoveFaces(solid, faces()));
      meta.prepareMs = prepared.ms;
    }
    const records = [];
    for (let tick = 0; tick < args.ticks; tick++) {
      const distance = distanceAt(tick);
      // Restore keeps the source's handles, so the prepared move stays valid.
      const stagesMs = { restore: timed(() => kernel.restore(checkpoint)).ms };
      let moved;
      try {
        moved = args.move === 'prepared' ?
          timed(() => kernel.moveFacesPrepared(prepared.result, distance)) :
          timed(() => kernel.moveFaces(solid, faces(), distance));
      } catch (error) {
        records.push({ tick, distance, accepted: false, error: String(error.message || error),
          stagesMs: { restore: stagesMs.restore }, memoryAfter: memory(kernel) });
        continue;
      }
      stagesMs.move = moved.ms;
      const validation = timed(() => kernel.validateSolidRelaxed(moved.result));
      stagesMs.validate = validation.ms;
      const grouped = timed(() => kernel.tessellateSolidGroupedBinary(
        moved.result, DISPLAY_DEFLECTION, DISPLAY_ANGLE));
      stagesMs.mesh = grouped.ms;
      stagesMs.tick = STAGES.reduce((sum, stage) => sum + stagesMs[stage], 0);
      const triangles = grouped.result.takeIndices().length / 3;
      grouped.result.free();
      records.push({ tick, distance, accepted: validation.result === 0, stagesMs, triangles,
        memoryAfter: memory(kernel) });
    }
    const result = { label: args.label, metadata: meta, summary: summarize(records, args.window),
      records };
    if (args.out) fs.writeFileSync(args.out, JSON.stringify(result, null, 2) + '\n');
    const s = result.summary;
    const line = (name, q) => q ?
      `${name.padEnd(9)} p50 ${q.p50.toFixed(2).padStart(8)}  p95 ${q.p95.toFixed(2).padStart(8)}  max ${q.max.toFixed(2).padStart(8)} ms` :
      `${name.padEnd(9)} none`;
    process.stdout.write([
      `${args.label}: ${meta.kernel.name} ${meta.kernel.version}, move ${args.move}, ` +
        `background ${args.background}, ${s.ticks} ticks, ${s.refused} refused`,
      ...[...STAGES, 'tick'].map(stage => line(stage, s.accepted[stage])),
      `tick first ${args.window}: p50 ${s.firstWindow.tick?.p50.toFixed(2)} ms; ` +
        `last ${args.window}: p50 ${s.lastWindow.tick?.p50.toFixed(2)} ms`,
      `slots ${s.memoryFirst.allocatedSlots} -> ${s.memoryLast.allocatedSlots} ` +
        `(${s.slotsPerTick.toFixed(0)}/tick, ${s.memoryLast.retiredSlots} retired), linear memory ` +
        `${s.memoryFirst.linearMemoryBytes} -> ${s.memoryLast.linearMemoryBytes} bytes`,
    ].join('\n') + '\n');
  } finally {
    if (prepared) kernel.releasePreparedMove(prepared.result);
    kernel.free();
  }
}

module.exports = { main };
if (require.main === module) {
  try { main(); } catch (error) {
    process.stderr.write(`${error.stack || error}\n`);
    process.exitCode = 1;
  }
}
