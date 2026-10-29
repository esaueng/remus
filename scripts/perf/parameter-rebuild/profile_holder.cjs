#!/usr/bin/env node
'use strict';

// Observation-only companion to parameter_bench.cjs. Its measured path is unchanged.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const inspector = require('node:inspector');
const os = require('node:os');
const sha256 = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const assert = (condition, message) => { if (!condition) throw Error(message); };
const nowUs = () => Number(process.hrtime.bigint() / 1000n);
const MEASURED_PHASES = ['restore', 'prefix', 'rebuild', 'validate', 'mesh', 'transfer', 'volume', 'recognize'];

function options(argv) {
  const result = { ticks: '6', warmup: '2', mode: 'completion', rebuild: 'full', pattern: 'unique',
    'sample-interval-us': '500', 'timing-status': 'contended-smoke-excluded' };
  const allowed = new Set(['kernel-pkg', 'out', 'ticks', 'warmup', 'mode', 'rebuild', 'pattern',
    'sample-interval-us', 'timing-status', 'package-source-sha', 'package-receipt']);
  for (let index = 0; index < argv.length; index += 2) {
    const name = argv[index].replace(/^--/, '');
    assert(argv[index].startsWith('--') && allowed.has(name) && argv[index + 1] !== undefined,
      `Unknown or incomplete argument: ${argv[index]}`);
    result[name] = argv[index + 1];
  }
  assert(result['timing-status'] === 'contended-smoke-excluded', 'Profiling requires diagnostic-only timing status');
  for (const key of ['kernel-pkg', 'out']) {
    assert(result[key], `Missing --${key}`);
    result[key] = path.resolve(result[key]);
  }
  result['sample-interval-us'] = Number(result['sample-interval-us']);
  assert(Number.isSafeInteger(result['sample-interval-us']) && result['sample-interval-us'] >= 100 &&
    result['sample-interval-us'] <= 10000, 'Invalid CPU sample interval');
  assert(!result['package-source-sha'] || /^[0-9a-f]{40}$/.test(result['package-source-sha']), 'Invalid source SHA');
  if (result['package-receipt']) result['package-receipt'] = path.resolve(result['package-receipt']);
  return result;
}

function inventory(directory) {
  const result = {};
  function visit(relative) {
    for (const entry of fs.readdirSync(path.join(directory, relative), { withFileTypes: true }).sort((a, b) => a.name.localeCompare(b.name))) {
      const name = path.join(relative, entry.name), file = path.join(directory, name);
      assert(!entry.isSymbolicLink(), 'Package inventory cannot contain symbolic links');
      if (entry.isDirectory()) visit(name);
      else {
        assert(entry.isFile(), 'Unsupported package inventory entry');
        const bytes = fs.readFileSync(file);
        result[name.split(path.sep).join('/')] = { bytes: bytes.length, sha256: sha256(bytes) };
      }
    }
  }
  visit('');
  for (const required of ['package.json', 'remus_wasm_bg.wasm', 'remus_wasm_node.cjs'])
    assert(result[required], `Missing package file: ${required}`);
  return result;
}

function harnessInventory() {
  const files = ['profile_holder.cjs', 'parameter_bench.cjs', 'holder_recipe.cjs',
    '../filleted-face-edit/wasm_edit_bench.cjs', 'fixtures/recipe.json', 'fixtures/base.arena.json',
    'fixtures/text.arena.json', 'fixtures/pinned-source-manifest.json'];
  return Object.fromEntries(files.map(name => {
    const bytes = fs.readFileSync(path.join(__dirname, name));
    return [name, { bytes: bytes.length, sha256: sha256(bytes) }];
  }));
}

function unchanged(before, after, label) {
  assert(JSON.stringify(before) === JSON.stringify(after), `${label} changed during capture`);
}

function observe(recipe, Kernel, clock = nowUs) {
  let tick = -1, phase = null, depth = 0;
  const calls = [], spans = [], restore = [];
  function wrapRecipe(name, observedPhase) {
    const original = recipe[name];
    assert(typeof original === 'function', `Missing recipe operation: ${name}`);
    recipe[name] = function observedRecipe(...args) {
      const prior = phase, startUs = clock(); phase = observedPhase;
      try { return original.apply(this, args); }
      finally { spans.push({ phase: observedPhase, tick, startUs, endUs: clock() }); phase = prior; }
    };
    restore.push(() => { recipe[name] = original; });
  }
  for (const [name, observedPhase] of [['prepare', 'prefix'], ['rebuild', 'rebuild'],
    ['qualify', 'qualification'], ['extract', 'transfer']]) wrapRecipe(name, observedPhase);
  const direct = { restore: 'restore', validateSolid: 'validate', tessellateSolidGroupedBinary: 'mesh',
    volume: 'volume', recognizeFeatures: 'recognize', checkpointMemoryStats: 'memory-probe', free: 'teardown' };
  for (const name of Object.getOwnPropertyNames(Kernel.prototype)) {
    const descriptor = Object.getOwnPropertyDescriptor(Kernel.prototype, name);
    if (name === 'constructor' || name.startsWith('__') || typeof descriptor.value !== 'function') continue;
    const original = descriptor.value;
    Object.defineProperty(Kernel.prototype, name, { ...descriptor, value: function observedKernelMethod(...args) {
      if (name === 'restore') tick++;
      const observedPhase = phase || direct[name] || (tick < 0 ? 'setup' : 'after-timing');
      const startUs = clock(), nested = depth++ > 0;
      let status = 'ok';
      try { return original.apply(this, args); }
      catch (error) { status = 'threw'; throw error; }
      finally { depth--; calls.push({ tick, phase: observedPhase, method: name, nested, status,
        startUs, endUs: clock(), firstArgument: typeof args[0] === 'number' ? args[0] : null }); }
    } });
    restore.push(() => Object.defineProperty(Kernel.prototype, name, descriptor));
  }
  return { calls, spans, restore: () => { for (const action of restore.reverse()) action(); } };
}

const writeNew = (file, value) => fs.writeFileSync(file, JSON.stringify(value, null, 2) + '\n', { flag: 'wx' });
const post = (session, method, params = {}) => new Promise((resolve, reject) =>
  session.post(method, params, (error, value) => error ? reject(error) : resolve(value)));

async function main(argv = process.argv.slice(2)) {
  const args = options(argv);
  const driver = require('./parameter_bench.cjs');
  const driverArgs = ['--kernel-pkg', args['kernel-pkg'], '--out', path.join(args.out, 'run.json')];
  for (const key of ['ticks', 'warmup', 'mode', 'rebuild', 'pattern', 'timing-status', 'package-source-sha'])
    if (args[key] !== undefined) driverArgs.push(`--${key}`, String(args[key]));
  const normalized = driver.options(driverArgs);
  fs.mkdirSync(args.out); // Exclusive directory creation protects all raw artifacts.
  const capture = { schemaVersion: 1, kind: 'parameter-holder-diagnostic', status: 'initializing',
    diagnosticOnly: true, timingClaimsExcluded: true, startedAt: new Date().toISOString(),
    options: { mode: args.mode, rebuild: args.rebuild, pattern: args.pattern, ticks: normalized.ticks,
      warmup: normalized.warmup, sampleIntervalUs: args['sample-interval-us'] },
    source: { suppliedSha: args['package-source-sha'] ?? null,
      verification: 'Externally supplied; the profiler hashes package bytes and does not verify source/build lineage.' },
    node: process.version, versions: process.versions, cpu: os.cpus()[0]?.model,
    affinity: fs.existsSync('/proc/self/status') ? fs.readFileSync('/proc/self/status', 'utf8').match(/^Cpus_allowed_list:\s*(.*)$/m)?.[1] : null,
    policy: { measuredPhases: MEASURED_PHASES, productionTracing: false,
      moduleLoading: 'The observer loads the package before the driver; moduleLoadMs is not startup evidence.',
      coverage: 'Retained public kernel calls and prepare/rebuild/extract spans; in-stage JavaScript outside these windows is unassigned.',
      geometry: 'The original driver, quality policies and timing boundaries are unchanged.',
      host: 'Affinity is recorded; exclusive CPU ownership and absence of host activity are not established.' } };
  writeNew(path.join(args.out, 'initial-receipt.json'), capture);
  let observer, session, profileStarted = false, failure;
  try {
    capture.package = { files: inventory(args['kernel-pkg']) };
    capture.harness = harnessInventory();
    capture.package.version = JSON.parse(fs.readFileSync(path.join(args['kernel-pkg'], 'package.json'))).version;
    if (args['package-receipt']) {
      const bytes = fs.readFileSync(args['package-receipt']);
      fs.writeFileSync(path.join(args.out, 'supplied-package-receipt.json'), bytes, { flag: 'wx' });
      capture.source.receiptSha256 = sha256(bytes);
      capture.source.receiptVerification = 'External attestation retained verbatim; not interpreted or independently validated by this profiler.';
    }
    const recipe = require('./holder_recipe.cjs');
    const { BrepKernel } = require(path.join(args['kernel-pkg'], 'remus_wasm_node.cjs'));
    observer = observe(recipe, BrepKernel);
    session = new inspector.Session(); session.connect();
    await post(session, 'Profiler.enable');
    await post(session, 'Profiler.setSamplingInterval', { interval: args['sample-interval-us'] });
    await post(session, 'Profiler.start'); profileStarted = true;
    capture.observationStartUs = nowUs();
    driver.main(driverArgs);
    capture.observationEndUs = nowUs();
    unchanged(capture.package.files, inventory(args['kernel-pkg']), 'Package');
    unchanged(capture.harness, harnessInventory(), 'Harness or fixture');
    capture.status = 'passed';
  } catch (error) {
    failure = error; capture.status = 'failed'; capture.error = String(error.stack || error);
    capture.observationEndUs ??= nowUs();
  } finally {
    if (session) {
      try {
        if (profileStarted) {
          const { profile } = await post(session, 'Profiler.stop');
          assert(profile.startTime <= capture.observationStartUs && capture.observationEndUs <= profile.endTime,
            'CPU and observation timestamps are not aligned');
          writeNew(path.join(args.out, 'profile.cpuprofile'), profile);
        }
      } catch (error) { failure ??= error; capture.status = 'failed'; capture.error = String(error.stack || error); }
      session.disconnect();
    }
    if (observer) {
      observer.restore();
      writeNew(path.join(args.out, 'methods.json'), { calls: observer.calls, spans: observer.spans });
    }
    capture.endedAt = new Date().toISOString();
    capture.artifacts = Object.fromEntries(fs.readdirSync(args.out).sort().map(name => {
      const bytes = fs.readFileSync(path.join(args.out, name));
      return [name, { bytes: bytes.length, sha256: sha256(bytes) }];
    }));
    writeNew(path.join(args.out, 'capture.json'), capture);
  }
  if (failure) throw failure;
  return capture;
}

module.exports = { options, inventory, harnessInventory, unchanged, observe, main, MEASURED_PHASES };
if (require.main === module) main().catch(error => { process.stderr.write(`${error.stack || error}\n`); process.exitCode = 1; });
