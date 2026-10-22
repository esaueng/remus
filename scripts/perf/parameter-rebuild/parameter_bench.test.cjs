'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const { test } = require('node:test');
const { options, widthAt, delta, summarize, main } = require('./parameter_bench.cjs');
const { fixture, plan, orderedSection } = require('./holder_recipe.cjs');
const { compare } = require('./compare_runs.cjs');
const { check } = require('./check_plan_fidelity.cjs');
const required = ['--kernel-pkg', '.', '--out', 'result.json'];

test('sampling keeps unseen widths distinct from requests that return', () => {
  assert.equal(new Set(Array.from({ length: 100 }, (_, i) => widthAt(i, 'unique'))).size, 100);
  assert.deepEqual(Array.from({ length: 7 }, (_, i) => widthAt(i, 'repeated')), [47, 48, 49, 48, 47, 49, 47]);
});

test('invalid policies and counts fail before package loading', () => {
  assert.equal(options(required).rebuild, 'full');
  assert.equal(options([...required, '--rebuild', 'suffix']).rebuild, 'suffix');
  for (const extra of [['--ticks', '0'], ['--ticks', '1001'], ['--warmup', '10'],
    ['--mode', 'frame'], ['--rebuild', 'automatic'], ['--pattern', 'cached'], ['--package-source-sha', 'unverified'],
    ['--timing-status', 'quiet'], ['--unknown', 'x']]) {
    assert.throws(() => options([...required, ...extra]));
  }
});

test('tick and warmup counts leave a post-cold retained sample', () => {
  for (const [ticks, warmup] of [[1, 0], [1, 1], [2, 2], [2, 3]]) {
    assert.throws(() => options([...required, '--ticks', String(ticks), '--warmup', String(warmup)]),
      /retained sample/);
  }
  for (const [ticks, warmup] of [[2, 0], [2, 1], [3, 2]]) {
    const result = options([...required, '--ticks', String(ticks), '--warmup', String(warmup)]);
    assert.equal(result.ticks, ticks); assert.equal(result.warmup, warmup);
  }
});

test('existing evidence is preserved before loading the package', () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'remus-parameter-evidence-'));
  const out = path.join(directory, 'existing.json');
  try {
    fs.writeFileSync(out, 'preserve\n');
    assert.throws(() => main(['--kernel-pkg', 'missing', '--out', out]), /already exists/);
    assert.equal(fs.readFileSync(out, 'utf8'), 'preserve\n');
  } finally { fs.rmSync(directory, { recursive: true }); }
});

test('canonical fixture checks exact bytes and retains every text solid', () => {
  const data = fixture();
  assert.equal(data.featureCount, 31); assert.equal(data.expected.textSolids, 7);
  const temporary = fs.mkdtempSync(path.join(os.tmpdir(), 'remus-parameter-fixture-'));
  try {
    for (const name of ['recipe.json', 'base.arena.json', 'text.arena.json']) {
      fs.copyFileSync(path.join(__dirname, 'fixtures', name), path.join(temporary, name));
    }
    fs.appendFileSync(path.join(temporary, 'text.arena.json'), ' ');
    assert.throws(() => fixture(temporary), /Changed text fixture/);
  } finally { fs.rmSync(temporary, { recursive: true }); }
});

test('the real recipe moves both ends symmetrically and changes only the floor span', () => {
  const { recipe } = fixture(), before = JSON.stringify(recipe);
  const source = plan(recipe, 46), grown = plan(recipe, 50), minimum = plan(recipe, 16.1);
  assert.equal(grown.bridges[0].distance - source.bridges[0].distance, 4);
  assert.equal(grown.bridges[0].offset, source.bridges[0].offset - 2);
  assert.deepEqual(grown.pieces.map(p => p.shift[0]), [-2, -2, 2, 2]);
  assert.ok(Math.abs(minimum.bridges[0].distance - 0.1) < 1e-12);
  assert.deepEqual(grown.bridges.slice(1).map(b => b.distance), source.bridges.slice(1).map(b => b.distance));
  assert.equal(JSON.stringify(recipe), before);
  assert.throws(() => plan(recipe, 16), /minimum/);
});

test('profile ordering retains curved edges and refuses ambiguous, open or excessive sections', () => {
  const data = fixture(), ordered = orderedSection(data.recipe.section);
  assert.equal(ordered.filter(s => s.segment.objectKind === 'arc').length, 2);
  assert.equal(ordered.length, 6);
  assert.throws(() => orderedSection(data.recipe.section.slice(1)), /disconnected|Open/);
  assert.throws(() => orderedSection([...data.recipe.section, data.recipe.section[0]]), /Ambiguous/);
  assert.throws(() => orderedSection(Array(65).fill(data.recipe.section[0])), /size/);
});

test('summary excludes cold and warmups and preserves negative process-memory observations', () => {
  const row = (index, ms, revisited = false) => ({ index, cold: index === 0, warmup: index < 2,
    revisited, stagesMs: { apiSum: ms, observedWall: ms + 1 }, memoryBefore: { jsHeapUsedBytes: 100 },
    memoryDeltas: { rebuild: { jsHeapUsedBytes: -50 } } });
  const result = summarize([row(0, 1000), row(1, 200), row(2, 30), row(3, 20, true)]);
  assert.equal(result.retained.samples, 2); assert.equal(result.firstSeen.samples, 1);
  assert.equal(result.revisited.samples, 1); assert.equal(result.retained.timingsMs.apiSum.p95, 30);
  assert.equal(result.retained.memoryDeltas.rebuild.jsHeapUsedBytes.p50, -50);
  assert.equal(result.retained.memoryDeltas.volume.jsHeapUsedBytes, null);
  assert.deepEqual(delta({ heap: 100 }, { heap: 80 }), { heap: -20 });
});

test('cross-mode qualification fails on changed trim bytes even when counts and volume agree', () => {
  const hash = 'a'.repeat(64);
  const row = { index: 0, width: 47, cold: true, warmup: true, revisited: false,
    accepted: true, checkpointCount: 1, arenaSha256: hash, volumes: null, features: null,
    validation: Array(8).fill(0), bbox: [0, 0, 0, 1, 1, 1], inventories: Array(8).fill({}),
    material: Array(6).fill({ expected: 'inside', actual: 'inside' }), transferredBytes: 100,
    meshSha256: Array(8).fill({ positions: hash, normals: hash, indices: hash, faceOffsets: hash }) };
  const meta = { ticks: 1, rebuildMode: 'full', fixture: { recipeSha256: hash }, package: { wasmSha256: hash,
    wrapperSha256: hash, packageJsonSha256: hash }, harnessSha256: hash,
    recipeHarnessSha256: hash, meshQualityHarnessSha256: hash, prefixDocumentSha256: hash };
  const preview = { metadata: { ...meta, mode: 'preview' }, records: [row] };
  const completion = { metadata: { ...meta, mode: 'completion' }, records: [{ ...row,
    volumes: Array(8).fill(1), features: Array(8).fill([]) }] };
  assert.equal(compare(preview, completion).qualified, true);
  completion.records[0].arenaSha256 = 'b'.repeat(64);
  assert.throws(() => compare(preview, completion), /arenaSha256/);
  completion.records[0].arenaSha256 = hash;
  delete preview.records[0].meshSha256; delete completion.records[0].meshSha256;
  assert.throws(() => compare(preview, completion), /assertion|Array.isArray/i);
});

test('planner source execution refuses changed or excessive input before evaluation', async () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'remus-parameter-planner-'));
  const source = path.join(directory, 'compiler.ts');
  try {
    fs.writeFileSync(source, 'throw new Error("untrusted input must never execute");');
    await assert.rejects(check(source, []), /not the verified pin/);
    fs.writeFileSync(source, ' '.repeat(64 * 1024 + 1));
    await assert.rejects(check(source, []), /exceeds diagnostic bound/);
  } finally { fs.rmSync(directory, { recursive: true }); }
});
