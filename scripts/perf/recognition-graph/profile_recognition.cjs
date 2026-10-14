#!/usr/bin/env node
'use strict';

// This observer shares the maintained edit workflow. It never runs in measured mode.
const fs = require('node:fs');
const path = require('node:path');
const inspector = require('node:inspector');
const { main } = require('../filleted-face-edit/wasm_edit_bench.cjs');

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

function post(session, method, params = {}) {
  let completed = false, failure, response;
  session.post(method, params, (error, result) => {
    completed = true;
    failure = error;
    response = result;
  });
  assert(completed, `Inspector ${method} did not complete synchronously`);
  if (failure) throw failure;
  return response;
}

function observer(mode) {
  assert(['cpu', 'counts'].includes(mode), '--mode must be cpu or counts');
  return {
    name: `recognition-${mode}`,
    recognize({ kernel, args, iteration, distance, run }) {
      assert(iteration >= args.warmup, 'Warmup must remain unobserved');
      const session = mode === 'cpu' ? new inspector.Session() : null;
      let profile, phases, started = false;
      try {
        if (session) {
          session.connect();
          post(session, 'Profiler.enable');
          post(session, 'Profiler.setSamplingInterval', { interval: 100 });
        }
        kernel.setPerformanceTracing(true);
        if (session) {
          post(session, 'Profiler.start');
          started = true;
        }
        return run();
      } finally {
        try {
          if (started) profile = post(session, 'Profiler.stop').profile;
          phases = JSON.parse(kernel.drainPerformanceTrace());
        } finally {
          kernel.setPerformanceTracing(false);
          if (session) session.disconnect();
        }
        const prefix = path.join(args.out, `recognize-${iteration}`);
        const diagnostic = { diagnosticOnly: true, timingClaimsExcluded: true, iteration, distance };
        if (mode === 'cpu') {
          assert(profile && Array.isArray(phases), 'CPU mode requires the production trace API');
          fs.writeFileSync(`${prefix}.cpuprofile`, JSON.stringify(profile));
          fs.writeFileSync(`${prefix}.phases.json`, JSON.stringify({ ...diagnostic, phases }));
        } else {
          assert(phases.diagnosticOnly && phases.timingClaimsExcluded && phases.counts &&
            Array.isArray(phases.notes), 'Count mode requires the documented temporary overlay');
          fs.writeFileSync(`${prefix}.counts.json`, JSON.stringify({ ...phases, ...diagnostic }));
        }
      }
    },
  };
}

function cli(argv) {
  const argumentsWithoutMode = [];
  let mode = 'cpu';
  for (let i = 0; i < argv.length; i += 2) {
    if (argv[i] === '--mode') {
      assert(argv[i + 1] !== undefined, 'Missing --mode value');
      mode = argv[i + 1];
    } else argumentsWithoutMode.push(argv[i], argv[i + 1]);
  }
  main(argumentsWithoutMode, observer(mode));
}

module.exports = { cli, observer };
if (require.main === module) {
  try { cli(process.argv.slice(2)); } catch (error) {
    process.stderr.write(`${error.stack || error}\n`);
    process.exitCode = 1;
  }
}
