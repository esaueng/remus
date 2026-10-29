'use strict';

// Verify the original driver bytes represented by the portable JSONL rows.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const assert = require('node:assert/strict');
const readRows = name => fs.readFileSync(path.join(__dirname, name), 'utf8')
  .trimEnd().split('\n').map(line => JSON.parse(line));
const processes = readRows('processes.jsonl');
const samples = readRows('samples.jsonl');
assert.equal(processes.length, 16);
assert.equal(samples.length, 96);
const names = new Set(processes.map(row => row.process));
assert.equal(names.size, 16);
assert(samples.every(row => names.has(row.process)));
for (const run of processes) {
  const records = samples.filter(row => row.process === run.process).map(row => row.record);
  assert.equal(records.length, 6);
  const bytes = Buffer.from(JSON.stringify({
    metadata: run.metadata, records, summary: run.summary,
  }, null, 2) + '\n');
  assert.equal(crypto.createHash('sha256').update(bytes).digest('hex'), run.rawSha256);
}
console.log('Reconstructed and verified all 16 original driver JSON hashes.');
