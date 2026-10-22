#!/usr/bin/env node
'use strict';

const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const { performance } = require('node:perf_hooks');
const recipe = require('./holder_recipe.cjs');
const { assert, sha256, DISPLAY } = recipe;
const STAGES = ['restore', 'prefix', 'rebuild', 'validate', 'mesh', 'transfer', 'volume', 'recognize'];

function options(argv) {
  const result = { ticks: 10, warmup: 2, mode: 'preview', rebuild: 'full', pattern: 'repeated',
    'timing-status': 'measured' };
  const allowed = new Set(['kernel-pkg', 'out', 'ticks', 'warmup', 'mode', 'rebuild', 'pattern',
    'timing-status', 'package-source-sha']);
  for (let i = 0; i < argv.length; i += 2) {
    const name = argv[i].replace(/^--/, '');
    assert(argv[i].startsWith('--') && allowed.has(name) && argv[i + 1] !== undefined,
      `Unknown or incomplete argument: ${argv[i]}`);
    result[name] = argv[i + 1];
  }
  for (const key of ['kernel-pkg', 'out']) {
    assert(result[key], `Missing --${key}`);
    result[key] = path.resolve(result[key]);
  }
  for (const key of ['ticks', 'warmup']) {
    result[key] = Number(result[key]);
    assert(Number.isSafeInteger(result[key]) && result[key] >= (key === 'ticks' ? 1 : 0) &&
      result[key] <= 1000, `Invalid --${key}`);
  }
  assert(result.warmup < result.ticks, 'Warmup must leave a retained sample');
  assert(['preview', 'completion'].includes(result.mode), 'Invalid --mode');
  assert(['full', 'suffix'].includes(result.rebuild), 'Invalid --rebuild');
  assert(['unique', 'repeated'].includes(result.pattern), 'Invalid --pattern');
  assert(['measured', 'contended-smoke-excluded'].includes(result['timing-status']), 'Invalid --timing-status');
  assert(!result['package-source-sha'] || /^[0-9a-f]{40}$/.test(result['package-source-sha']),
    'Invalid --package-source-sha');
  return result;
}

function widthAt(index, pattern) {
  return pattern === 'unique' ? 47 + 0.02 * index : [47, 48, 49, 48, 47, 49][index % 6];
}

function memory(kernel) {
  const stats = JSON.parse(kernel.checkpointMemoryStats()), processStats = process.memoryUsage();
  return { allocatedSlots: stats.allocatedSlots, retiredSlots: stats.retiredSlots,
    linearMemoryBytes: stats.linearMemoryBytes, processRssBytes: processStats.rss,
    jsHeapUsedBytes: processStats.heapUsed, externalBytes: processStats.external,
    arrayBufferBytes: processStats.arrayBuffers };
}

const delta = (before, after) => Object.fromEntries(Object.keys(before).map(k => [k, after[k] - before[k]]));
function quantiles(values) {
  if (values.length === 0) return null;
  const sorted = [...values].sort((a, b) => a - b);
  return { n: sorted.length, p50: sorted[Math.ceil(sorted.length / 2) - 1],
    p95: sorted[Math.ceil(sorted.length * 0.95) - 1], max: sorted.at(-1) };
}

function summarize(records) {
  const retained = records.filter(r => !r.cold && !r.warmup);
  const pools = { retained, firstSeen: retained.filter(r => !r.revisited),
    revisited: retained.filter(r => r.revisited) };
  return { total: records.length, cold: records[0],
    ...Object.fromEntries(Object.entries(pools).map(([name, pool]) => [name, {
      samples: pool.length, timingsMs: Object.fromEntries(Object.keys(records[0].stagesMs).map(stage =>
        [stage, quantiles(pool.map(r => r.stagesMs[stage]))])),
      memoryDeltas: Object.fromEntries(STAGES.map(stage => [stage,
        Object.fromEntries(Object.keys(records[0].memoryBefore).map(key =>
          [key, quantiles(pool.filter(r => r.memoryDeltas[stage]).map(r => r.memoryDeltas[stage][key]))]))]))
    }])) };
}

function main(argv = process.argv.slice(2)) {
  const args = options(argv);
  assert(!fs.existsSync(args.out), 'Output file already exists');
  const data = recipe.fixture();
  const packageJson = fs.readFileSync(path.join(args['kernel-pkg'], 'package.json'));
  const wasm = fs.readFileSync(path.join(args['kernel-pkg'], 'remus_wasm_bg.wasm'));
  const wrapper = path.join(args['kernel-pkg'], 'remus_wasm_node.cjs');
  const packageInfo = JSON.parse(packageJson);
  const loadStart = performance.now();
  const { BrepKernel } = require(wrapper);
  const moduleLoadMs = performance.now() - loadStart;
  const constructorStart = performance.now(), kernel = new BrepKernel();
  const constructorMs = performance.now() - constructorStart;
  const meta = { mode: args.mode, rebuildMode: args.rebuild, pattern: args.pattern, ticks: args.ticks, warmup: args.warmup,
    timingStatus: args['timing-status'], timingsExcluded: args['timing-status'] !== 'measured',
    package: { name: packageInfo.name, version: packageInfo.version,
      wasmBytes: wasm.length, wasmSha256: sha256(wasm), wrapperSha256: sha256(fs.readFileSync(wrapper)),
      packageJsonSha256: sha256(packageJson), suppliedSourceSha: args['package-source-sha'] ?? null,
      suppliedSourceShaVerification: 'Externally supplied; not verified by the driver.' },
    fixture: { identity: data.identity, recipeSha256: data.recipeSha256, source: data.source,
      documents: Object.fromEntries(['base', 'text'].map(p => [p, sha256(data.documents[p])])),
      consumer: data.consumer, featureCount: data.featureCount },
    harnessSha256: sha256(fs.readFileSync(__filename)),
    recipeHarnessSha256: sha256(fs.readFileSync(path.join(__dirname, 'holder_recipe.cjs'))),
    meshQualityHarnessSha256: sha256(fs.readFileSync(path.join(__dirname, '../filleted-face-edit/wasm_edit_bench.cjs'))),
    node: process.version, versions: process.versions, cpu: os.cpus()[0]?.model,
    platform: process.platform, architecture: process.arch,
    affinity: fs.existsSync('/proc/self/status') ?
      fs.readFileSync('/proc/self/status', 'utf8').match(/^Cpus_allowed_list:\s*(.*)$/m)?.[1] : null,
    timing: { clock: 'performance.now; synchronous kernel replay stage intervals',
      stageSum: 'Sum of replay stage intervals, including in-block JavaScript, prefix validation and prefix serialization; between-stage memory probes excluded.',
      observedWall: 'Restore through transfer (preview) or recognition (completion), including between-stage probes.',
      quality: 'All quality, material, inventory and full arena reads are outside timings; may affect subsequent ticks.',
      revisit: 'Previously requested width, including a refused request; does not establish a cache hit.',
      cold: 'First tick in this module process after separately reported checkpoint setup; full mode includes all 31 geometry features; not cold application launch.' },
    policy: { ...DISPLAY, height: data.recipe.height.sourceHeight, measurementReuse: 'production defaults',
      prefix: args.rebuild === 'full' ? 'Rebuild all 17 fixed features on every request from immutable exact base/text imports.' :
        'Restore a prepared checkpoint representing all 17 fixed features; this is an explicit suffix-only replay.',
      suffix: '14 features: three sketch/extrude pairs, six body placements, union, and text placement',
      inputToFrame: false }, moduleLoadMs, constructorMs };
  const records = [], seen = new Set();
  try {
    meta.memoryBeforePrepare = memory(kernel);
    const prepareStart = performance.now();
    const prepared = args.rebuild === 'suffix' ? recipe.prepare(kernel, data) : null;
    const checkpoint = prepared ? prepared.checkpoint : kernel.checkpoint();
    meta.prepareMs = performance.now() - prepareStart;
    meta.memoryAfterPrepare = memory(kernel);
    meta.prepareMemoryDelta = delta(meta.memoryBeforePrepare, meta.memoryAfterPrepare);
    meta.prefixDocumentSha256 = prepared?.documentSha256 ?? null;
    for (let index = 0; index < args.ticks; index++) {
      const width = widthAt(index, args.pattern);
      const revisited = seen.has(width); seen.add(width);
      const before = memory(kernel), stagesMs = {}, memoryDeltas = {}, memoryAfterStage = {};
      let prior = before;
      const wallStart = performance.now();
      const stage = (name, action) => {
        const start = performance.now(), result = action();
        stagesMs[name] = performance.now() - start;
        // The final probe is deferred until after observedWall ends below.
        if (name !== (args.mode === 'preview' ? 'transfer' : 'recognize')) {
          const after = memory(kernel);
          memoryAfterStage[name] = after; memoryDeltas[name] = delta(prior, after); prior = after;
        }
        return result;
      };
      stage('restore', () => kernel.restore(checkpoint));
      const prefix = prepared ?? stage('prefix', () => recipe.prepare(kernel, data, { checkpoint: false }));
      meta.prefixDocumentSha256 ??= prefix.documentSha256;
      assert(prefix.documentSha256 === meta.prefixDocumentSha256, 'Fixed-prefix replay changed');
      const result = stage('rebuild', () => recipe.rebuild(kernel, data, prefix, width));
      const validation = stage('validate', () => result.roots.map(s => kernel.validateSolid(s)));
      assert(validation.every(v => v === 0), 'Rebuilt recipe failed strict validation');
      const grouped = stage('mesh', () => result.roots.map(s => kernel.tessellateSolidGroupedBinary(
        s, DISPLAY.linear, DISPLAY.angular)));
      const meshes = stage('transfer', () => grouped.map(recipe.extract));
      let volumes = null, features = null;
      if (args.mode === 'completion') {
        volumes = stage('volume', () => result.roots.map(s => kernel.volume(s, DISPLAY.measurement)));
        features = stage('recognize', () => result.roots.map(s => JSON.parse(kernel.recognizeFeatures(s, DISPLAY.measurement))));
      }
      stagesMs.observedWall = performance.now() - wallStart;
      const after = memory(kernel), lastStage = args.mode === 'preview' ? 'transfer' : 'recognize';
      memoryAfterStage[lastStage] = after; memoryDeltas[lastStage] = delta(prior, after);
      stagesMs.stageSum = STAGES.reduce((sum, stage) => sum + (stagesMs[stage] || 0), 0);
      stagesMs.previewStageSum = ['restore', 'prefix', 'rebuild', 'validate', 'mesh', 'transfer']
        .reduce((sum, stage) => sum + (stagesMs[stage] || 0), 0);
      const qualified = recipe.qualify(kernel, data, result, meshes);
      if (volumes) assert(volumes.every(v => Number.isFinite(v) && v > 0), 'Invalid quantity');
      assert(sha256(kernel.serializeSolids(prefix.roots)) === prefix.documentSha256,
        'Fixed-prefix geometry changed');
      const checkpointCount = kernel.checkpointCount();
      assert(checkpointCount === 1, 'Per-request prefix leaked a retained checkpoint');
      const record = { index, width, cold: index === 0, warmup: index < args.warmup,
        revisited, accepted: true, stagesMs, memoryBefore: before, memoryAfter: after,
        memoryDeltas, memoryAfterStage, validation, volumes, features,
        featureSummary: features ? recipe.featureSummary(features) : null, checkpointCount, ...qualified };
      records.push(record);
      process.stderr.write(`${args.rebuild}/${args.mode}/${args.pattern} ${index}: ${stagesMs.stageSum.toFixed(2)} ms\n`);
    }
    const output = { metadata: meta, records, summary: summarize(records) };
    fs.mkdirSync(path.dirname(args.out), { recursive: true });
    fs.writeFileSync(args.out, JSON.stringify(output, null, 2) + '\n', { flag: 'wx' });
    return output;
  } finally { kernel.free(); }
}

module.exports = { options, widthAt, delta, quantiles, summarize, main };
if (require.main === module) {
  try { main(); } catch (error) { process.stderr.write(`${error.stack || error}\n`); process.exitCode = 1; }
}
