#!/usr/bin/env node
'use strict';
// Private diagnostic collector. No CPU profiler or event-clock timing exists here.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const { execFileSync } = require('node:child_process');
const assert = (ok, message) => { if (!ok) throw Error(message); };
const hash = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const KEYS = ['total', 'zero_product_a_only', 'zero_product_b_only', 'zero_product_both',
  'finite_nonzero_zeroaddend', 'finite_nonzero_unitfactor', 'remaining_emulated', 'remaining_fallback'];
const zeros = () => KEYS.map(() => 0n);
const counters = values => Object.fromEntries(KEYS.map((name, index) => [name, values[index].toString()]));
const writeNew = (file, value) => fs.writeFileSync(file, JSON.stringify(value, null, 2) + '\n', { flag: 'wx' });
function options(argv) {
  const args = { 'timing-status': 'contended-smoke-excluded' };
  const allowed = new Set(['kernel-pkg', 'mode', 'out', 'source-root', 'package-source-sha',
    'package-receipt', 'timing-status']);
  const seen = new Set();
  for (let i = 0; i < argv.length; i += 2) {
    const name = argv[i].replace(/^--/, '');
    assert(argv[i].startsWith('--') && allowed.has(name) && argv[i + 1] !== undefined && !seen.has(name),
      `Unknown, duplicate or incomplete argument: ${argv[i]}`);
    seen.add(name); args[name] = argv[i + 1];
  }
  for (const key of ['kernel-pkg', 'mode', 'out']) assert(args[key], `Missing --${key}`);
  assert(['preview', 'completion'].includes(args.mode), 'Invalid --mode');
  assert(args['timing-status'] === 'contended-smoke-excluded', 'Only diagnostic excluded timings are allowed');
  assert(!args['package-source-sha'] || /^[0-9a-f]{40}$/.test(args['package-source-sha']), 'Invalid source SHA');
  args['source-root'] ??= JSON.parse(fs.readFileSync(path.join(__dirname, 'paths.json'))).worktree;
  for (const key of ['kernel-pkg', 'out', 'source-root', 'package-receipt'])
    if (args[key]) args[key] = path.resolve(args[key]);
  for (const key of ['source-root', 'kernel-pkg']) {
    const relative = path.relative(args[key], args.out);
    assert(relative.startsWith(`..${path.sep}`) || relative === '..' || path.isAbsolute(relative),
      'Output must be outside the source and package directories');
  }
  return args;
}
function familyBits(bits) {
  const [a, b, c] = bits.map(value => value & 0x7fffffffffffffffn);
  const finite = value => value < 0x7ff0000000000000n;
  const az = a === 0n, bz = b === 0n, cz = c === 0n;
  if (((az && finite(b)) || (bz && finite(a))) && finite(c)) return az ? (bz ? 2 : 0) : 1;
  if (cz && !az && !bz && finite(a) && finite(b)) return 3;
  if ((a === 0x3ff0000000000000n || b === 0x3ff0000000000000n) && !az && !bz &&
      finite(a) && finite(b) && finite(c)) return 4;
  const low = 573n << 52n, high = 1473n << 52n, addend = 1923n << 52n;
  if (low <= a && a <= high && low <= b && b <= high && c <= addend) return 5;
  return 6;
}
function decode(words) {
  assert(ArrayBuffer.isView(words) && Object.prototype.toString.call(words) === '[object BigUint64Array]',
    'Probe must return BigUint64Array');
  assert(words.length >= 15, 'Truncated probe header');
  const count = Array.from(words.slice(0, 8));
  assert(count[0] === count.slice(1).reduce((a, b) => a + b, 0n), 'Probe total does not equal family sum');
  const lengths = Array.from(words.slice(8, 15), (value, bin) => {
    assert(value <= 16n && value <= count[bin + 1], 'Invalid bounded sample length');
    return Number(value); // Only bounded lengths are converted; operand bits and counts remain BigInt.
  });
  assert(words.length === 15 + 3 * lengths.reduce((a, b) => a + b, 0), 'Probe payload length mismatch');
  let offset = 15;
  const samples = lengths.map((length, bin) => Array.from({ length }, () => {
    const bits = Array.from(words.slice(offset, offset += 3));
    assert(familyBits(bits) === bin, 'Sample does not match declared disjoint family');
    return bits.map(value => value.toString(16).padStart(16, '0'));
  }));
  return { count, samples };
}
function ledger(getter, limits = { events: 100000, triples: 250000 }) {
  const events = [], prefix = [zeros()];
  let triples = 0;
  function clock() {
    assert(events.length < limits.events, 'Diagnostic event budget exhausted');
    const value = decode(getter());
    triples += value.samples.reduce((sum, rows) => sum + rows.length, 0);
    assert(triples <= limits.triples, 'Diagnostic sample budget exhausted');
    events.push(value);
    prefix.push(prefix.at(-1).map((prior, index) => prior + value.count[index]));
    return events.length;
  }
  function selection(indices) {
    const count = zeros(), samples = KEYS.slice(1).map(() => []);
    for (const index of indices) {
      const event = events[index];
      event.count.forEach((value, bin) => { count[bin] += value; });
      event.samples.forEach((rows, bin) => samples[bin].push(...rows.slice(0, 16 - samples[bin].length)));
    }
    return { counts: counters(count), samples: Object.fromEntries(KEYS.slice(1).map((name, bin) => [name, samples[bin]])) };
  }
  function window(start, end) {
    assert(Number.isSafeInteger(start) && Number.isSafeInteger(end) && start >= 0 && start <= end && end <= events.length,
      'Invalid event window');
    const selected = selection(Array.from({ length: end - start }, (_, i) => start + i));
    assert(KEYS.every((name, i) => BigInt(selected.counts[name]) === prefix[end][i] - prefix[start][i]),
      'Event window ledger mismatch');
    return selected;
  }
  return { clock, events, prefix, selection, window };
}
function withoutClockUnits(row) {
  const { startUs, endUs, ...rest } = row;
  return { ...rest, startEvent: startUs, endEvent: endUs };
}
function summarize(observer, observed) {
  const calls = observer.calls.map(row => ({ ...withoutClockUnits(row),
    cold: row.tick === 0, warmup: row.tick >= 0 && row.tick < 2,
    ...observed.window(row.startUs, row.endUs) }));
  const spans = observer.spans.map(row => ({ ...withoutClockUnits(row),
    cold: row.tick === 0, warmup: row.tick >= 0 && row.tick < 2,
    ...observed.window(row.startUs, row.endUs) }));
  const outer = calls.filter(row => !row.nested).sort((a, b) => a.startEvent - b.startEvent);
  const assigned = new Uint8Array(observed.events.length);
  const groups = new Map();
  let lastEnd = 0;
  for (const call of outer) {
    assert(call.startEvent >= lastEnd, 'Outer kernel calls overlap'); lastEnd = call.endEvent;
    const key = `${call.tick}:${call.phase}`;
    if (!groups.has(key)) groups.set(key, { tick: call.tick, phase: call.phase, cold: call.cold,
      warmup: call.warmup, calls: 0, indices: [] });
    const group = groups.get(key); group.calls++;
    for (let index = call.startEvent; index < call.endEvent; index++) {
      assert(!assigned[index], 'Outer kernel drain double-counted'); assigned[index] = 1;
      group.indices.push(index);
    }
  }
  const stages = [...groups.values()].map(({ indices, ...group }) => ({ ...group, ...observed.selection(indices) }));
  const unassigned = observed.selection(Array.from(assigned.keys()).filter(index => !assigned[index]));
  const total = observed.window(0, observed.events.length);
  for (const key of KEYS)
    assert(BigInt(total.counts[key]) === BigInt(unassigned.counts[key]) +
      stages.reduce((sum, stage) => sum + BigInt(stage.counts[key]), 0n), 'Stage partition does not reconcile');
  return { total, outerKernelByStage: stages, unassigned, calls, recipeSpans: spans,
    recognizeFeaturesRoots: calls.filter(row => row.method === 'recognizeFeatures'),
    events: observed.events.map((event, index) => ({ event: index + 1, counts: counters(event.count) })) };
}
function sourceInventory(root) {
  const git = args => execFileSync('git', ['-C', root, ...args], {
    encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'], env: { ...process.env, GIT_OPTIONAL_LOCKS: '0' } });
  const names = [...new Set(git(['ls-files', '--cached', '--others', '--exclude-standard', '-z']).split('\0').filter(Boolean))].sort();
  const files = {};
  for (const name of names) {
    const file = path.join(root, name), stat = fs.lstatSync(file);
    assert(stat.isFile() || stat.isSymbolicLink(), `Unsupported source entry: ${name}`);
    const bytes = fs.readFileSync(file);
    files[name] = { bytes: bytes.length, sha256: hash(bytes), mode: stat.mode & 0o777,
      ...(stat.isSymbolicLink() ? { linkTargetSha256: hash(fs.readlinkSync(file)) } : {}) };
  }
  return { head: git(['rev-parse', 'HEAD']).trim(), tree: git(['rev-parse', 'HEAD^{tree}']).trim(),
    status: git(['status', '--porcelain=v1']), files, filesSha256: hash(JSON.stringify(files)),
    policy: 'Tracked and nonignored untracked source content/modes observed; dirty probe overlay is intentional. Source/package build lineage is not independently established by this collector.' };
}
function unchanged(before, after, label) { assert(JSON.stringify(before) === JSON.stringify(after), `${label} changed during replay`); }
function main(argv = process.argv.slice(2)) {
  const args = options(argv);
  fs.mkdirSync(args.out); // Exclusive: preserve all earlier captures and failed attempts.
  const capture = { schemaVersion: 1, kind: 'private-fma-operand-family-observation', status: 'initializing',
    diagnosticOnly: true, timingClaimsExcluded: true, canonicalRuntimeSuitesDeferred: true,
    options: { mode: args.mode, rebuild: 'full', pattern: 'unique', ticks: 6, warmup: 2,
      timingStatus: 'contended-smoke-excluded' },
    familyOrder: KEYS, countEncoding: 'Lossless unsigned decimal strings',
    operandEncoding: 'Unsigned 16-digit hexadecimal strings, no prefix; [aBits,bBits,cBits]',
    policy: { clock: 'Monotonic drain event sequence only; no timestamp or elapsed-time interpretation.',
      frequency: 'Counters/sample bits are exact for this instrumented execution. Observer side effects may alter CSE, elimination or inlining; this is not a guaranteed shipping dynamic-call census or production-frequency claim.',
      samples: 'First at most16 per family per drain, aggregated first at most16 per selected window; biased bounded witnesses, not representative sampling.',
      additivity: 'Only outerKernelByStage plus unassigned partitions total. Method/root/recipe windows are inclusive and overlap; do not sum them together.',
      quality: 'Original maintained driver gates only; collector does not perform independent shipping/baseline equivalence or canonical runtime tests.',
      rawDriver: 'Unchanged driver run contains diagnostic elapsed values; every such value is excluded from timing claims.' },
    suppliedSourceSha: args['package-source-sha'] ?? null,
    suppliedSourceVerification: 'External label only; not source/build attestation.',
    node: process.version, affinity: fs.readFileSync('/proc/self/status', 'utf8').match(/^Cpus_allowed_list:\s*(.*)$/m)?.[1] ?? null };
  writeNew(path.join(args.out, 'initial-receipt.json'), capture);
  let observer, observed, failure, profiler, packageInventory, harnessInventory, driverResult;
  const fail = error => { failure ??= error; capture.status = 'failed'; capture.error = String(failure.stack || failure); };
  try {
    capture.sourceBefore = sourceInventory(args['source-root']);
    const driverRoot = path.join(args['source-root'], 'scripts/perf/parameter-rebuild');
    profiler = require(path.join(driverRoot, 'profile_holder.cjs'));
    packageInventory = () => Object.fromEntries(Object.entries(profiler.inventory(args['kernel-pkg'])).map(([name, row]) =>
      [name, { ...row, mode: fs.statSync(path.join(args['kernel-pkg'], name)).mode & 0o777 }]));
    harnessInventory = () => ({ maintained: profiler.harnessInventory(),
      collector: { bytes: fs.statSync(__filename).size, sha256: hash(fs.readFileSync(__filename)) } });
    capture.packageBefore = packageInventory(); capture.harnessBefore = harnessInventory();
    if (args['package-receipt']) {
      const receipt = fs.readFileSync(args['package-receipt']);
      fs.writeFileSync(path.join(args.out, 'supplied-package-receipt.json'), receipt, { flag: 'wx' });
      capture.suppliedReceiptSha256 = hash(receipt);
      capture.suppliedReceiptVerification = 'External attestation retained verbatim; not independently validated by collector.';
    }
    const recipe = require(path.join(driverRoot, 'holder_recipe.cjs'));
    const driver = require(path.join(driverRoot, 'parameter_bench.cjs'));
    const module = require(path.join(args['kernel-pkg'], 'remus_wasm_node.cjs'));
    assert(typeof module.takeFmaDispatchProbe === 'function', 'Package lacks free takeFmaDispatchProbe export');
    assert(typeof module.BrepKernel === 'function', 'Package lacks BrepKernel');
    observed = ledger(() => module.takeFmaDispatchProbe());
    capture.initialModuleEvent = observed.clock();
    observer = profiler.observe(recipe, module.BrepKernel, observed.clock);
    const driverArgs = ['--kernel-pkg', args['kernel-pkg'], '--out', path.join(args.out, 'run.raw.json'),
      '--mode', args.mode, '--rebuild', 'full', '--pattern', 'unique', '--ticks', '6', '--warmup', '2',
      '--timing-status', 'contended-smoke-excluded'];
    if (args['package-source-sha']) driverArgs.push('--package-source-sha', args['package-source-sha']);
    capture.driverArgs = driverArgs;
    const run = driverResult = driver.main(driverArgs);
    observed.clock();
    assert(run.metadata.fixture.featureCount === 31 && run.records.length === 6, 'Incomplete full31 replay');
    assert(run.metadata.timingsExcluded === true, 'Driver timings were not excluded');
    assert(run.records.every((row, index) => row.index === index && row.width === driver.widthAt(index, 'unique') &&
      row.accepted === true && row.validation.every(value => value === 0)), 'Replay gate failed');
    capture.quality = { maintainedDriver: 'passed', ticks: 6, retainedTicks: 4, featureCount: 31,
      independentBaselineEquivalence: 'not performed by collector' };
    capture.status = 'passed';
  } catch (error) { fail(error); }
  finally {
    try {
      if (observer) observer.restore();
      if (observed) {
        if (failure) observed.clock();
        const result = summarize(observer ?? { calls: [], spans: [] }, observed);
        if (driverResult) {
          for (const record of driverResult.records) {
            const roots = result.recognizeFeaturesRoots.filter(row => row.tick === record.index && row.phase === 'recognize');
            assert(roots.length === (record.features?.length ?? 0) && roots.every(row =>
              row.status === 'ok' && Number.isSafeInteger(row.firstArgument)), 'Recognition root coverage mismatch');
          }
        }
        writeNew(path.join(args.out, 'dispatch.json'), result);
        capture.observation = { drainEvents: observed.events.length, counters: result.total.counts,
          recognizeFeaturesRoots: result.recognizeFeaturesRoots.length, partitionVerified: true };
      }
      if (capture.packageBefore) {
        capture.packageAfter = packageInventory(); unchanged(capture.packageBefore, capture.packageAfter, 'Package');
      }
      if (capture.harnessBefore) {
        capture.harnessAfter = harnessInventory(); unchanged(capture.harnessBefore, capture.harnessAfter, 'Harness');
      }
      if (capture.sourceBefore) {
        capture.sourceAfter = sourceInventory(args['source-root']);
        unchanged(capture.sourceBefore, capture.sourceAfter, 'Source');
      }
    } catch (error) { fail(error); }
    capture.artifacts = Object.fromEntries(fs.readdirSync(args.out).sort().map(name => {
      const bytes = fs.readFileSync(path.join(args.out, name));
      return [name, { bytes: bytes.length, sha256: hash(bytes) }];
    }));
    writeNew(path.join(args.out, 'capture.json'), capture);
  }
  if (failure) throw failure;
  return capture;
}
module.exports = { KEYS, options, familyBits, decode, ledger, summarize, sourceInventory, main };
if (require.main === module) {
  try { main(); } catch (error) { process.stderr.write(`${error.stack || error}\n`); process.exitCode = 1; }
}
