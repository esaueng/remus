'use strict';

const assert = require('node:assert/strict');
const crypto = require('node:crypto');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { test } = require('node:test');

const repository = path.resolve(__dirname, '../..');
const archive = path.join(repository, 'docs/performance/evidence/capture-sources-2026-10-20');
const readJson = file => JSON.parse(fs.readFileSync(file, 'utf8'));
const digest = file => crypto.createHash('sha256').update(fs.readFileSync(file)).digest('hex');
const manifest = () => readJson(path.join(archive, 'manifest.json'));
const sourceFile = entry => entry.snapshot ? path.join(archive, entry.snapshot) :
  path.join(repository, entry.path);

test('archived runtime sources and inputs match their captured bytes', () => {
  const data = manifest();
  assert.equal(data.schemaVersion, 1);
  assert.equal(data.sources.length, 4);
  for (const entry of [...data.sources, ...data.inputs]) {
    assert.match(entry.sha256, /^[0-9a-f]{64}$/);
    assert.equal(digest(sourceFile(entry)), entry.sha256, entry.path);
  }
});

test('capture archives agree with unchanged provenance and all recorded driver hashes', () => {
  const data = manifest();
  const entries = new Map([...data.sources, ...data.inputs].map(entry => [entry.path, entry]));
  assert.equal(data.captures.length, 2);
  for (const capture of data.captures) {
    const provenanceFile = path.join(repository, capture.provenance);
    assert.equal(digest(provenanceFile), capture.provenanceSha256);
    const provenance = readJson(provenanceFile);
    for (const [file, hash] of Object.entries(provenance.captureCodeSha256 ?? provenance.harnesses)) {
      assert.equal(entries.get(file)?.sha256, hash, file);
    }
    for (const [file, hash] of Object.entries(provenance.evidenceSha256 ?? provenance.evidenceFiles)) {
      assert.equal(digest(path.join(path.dirname(provenanceFile), file)), hash, file);
    }
    const runs = readJson(path.join(repository, capture.runs));
    assert.equal(runs.length, capture.name === 'parameter-rebuild' ? 8 : 4);
    for (const run of runs) {
      assert.equal(run.metadata.harnessSha256, entries.get(capture.driverPath).sha256);
      assert.equal(run.metadata.meshQualityHarnessSha256,
        entries.get('scripts/perf/filleted-face-edit/wasm_edit_bench.cjs').sha256);
      if (run.metadata.recipeHarnessSha256) {
        assert.equal(run.metadata.recipeHarnessSha256,
          entries.get('scripts/perf/parameter-rebuild/holder_recipe.cjs').sha256);
      }
    }
  }
});

test('captured sources load at their original paths and accept the recorded options', () => {
  const data = manifest();
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'remus-capture-sources-'));
  try {
    for (const entry of [...data.sources, ...data.inputs]) {
      const destination = path.join(directory, entry.path);
      fs.mkdirSync(path.dirname(destination), { recursive: true });
      fs.copyFileSync(sourceFile(entry), destination, fs.constants.COPYFILE_EXCL);
      assert.equal(digest(destination), entry.sha256, entry.path);
    }
    for (const capture of data.captures) {
      const driver = require(path.join(directory, capture.driverPath));
      const provenance = readJson(path.join(repository, capture.provenance));
      const commands = provenance.launchOrder ?? provenance.preview;
      const runs = readJson(path.join(repository, capture.runs));
      for (let index = 0; index < commands.length; index++) {
        const command = commands[index].command;
        const offset = command.indexOf(capture.driverPath);
        assert.ok(offset >= 0);
        const options = driver.options(command.slice(offset + 1));
        assert.equal(options.ticks, runs[index].metadata.ticks);
        assert.equal(options.warmup, runs[index].metadata.warmup);
      }
    }
    const recipe = require(path.join(directory, 'scripts/perf/parameter-rebuild/holder_recipe.cjs'));
    assert.equal(recipe.fixture().featureCount, 31);
  } finally { fs.rmSync(directory, { recursive: true }); }
});
