'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { test } = require('node:test');
const { cli, observer } = require('./profile_recognition.cjs');

test('diagnostic CLI rejects measured status before loading packages or creating artifacts', () => {
  const out = path.join(os.tmpdir(), `remus-recognition-negative-${process.pid}`);
  assert.equal(fs.existsSync(out), false);
  assert.throws(() => cli(['--kernel-pkg', '.', '--io-pkg', '.', '--fixture', 'README.md',
    '--out', out, '--timing-status', 'measured']), /require excluded timing status/);
  assert.equal(fs.existsSync(out), false);
});

test('counts cannot be mistaken for the production trace array', () => {
  const changes = [];
  const kernel = { setPerformanceTracing: value => changes.push(value), drainPerformanceTrace: () => '[]' };
  assert.throws(() => observer('counts').recognize({ kernel, args: { warmup: 2, out: '.' },
    iteration: 2, distance: 5.04, run: () => ({ result: '[]', ms: 1 }) }),
  /requires the documented temporary overlay/);
  assert.deepEqual(changes, [true, false]);
});

test('diagnostic mode must be explicit and recognized', () => {
  assert.throws(() => observer('timed'), /must be cpu or counts/);
});
