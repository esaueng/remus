#!/usr/bin/env node
// Contracts for splitting the packaged WASM suites across CI jobs.

import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { test } from 'node:test';
import { parsePart, suiteParts, WEIGHTS as SUITE_WEIGHTS } from './packaged-suite-parts.mjs';

const WEIGHTS = [1, 170, 1, 1, 40, 1, 60, 1, 1, 1, 12, 1, 55, 1, 1];

function runPart(value) {
  const ran = [];
  const lines = [];
  const part = suiteParts(value, line => lines.push(line));
  WEIGHTS.forEach((weight, suite) => part(() => ran.push(suite), weight));
  part.summary();
  return { ran, lines };
}

test('an unset part runs every suite in order', () => {
  for (const value of [undefined, '', '1/1']) {
    assert.deepEqual(runPart(value).ran, WEIGHTS.map((_, suite) => suite));
  }
});

test('every suite runs in exactly one part', () => {
  for (let total = 1; total <= 6; total += 1) {
    const all = [];
    for (let index = 1; index <= total; index += 1) all.push(...runPart(`${index}/${total}`).ran);
    assert.deepEqual(all.sort((a, b) => a - b), WEIGHTS.map((_, suite) => suite), `total ${total}`);
  }
});

test('weights balance the parts', () => {
  const load = value => runPart(value).ran.reduce((sum, suite) => sum + WEIGHTS[suite], 0);
  const loads = [load('1/2'), load('2/2')];
  assert.ok(Math.max(...loads) <= 170 + 5, `unbalanced parts: ${loads}`);
});

test('a named suite takes its recorded weight', () => {
  const runWideSphereCapRegression = () => {};
  const ran = [];
  const part = suiteParts('2/2', () => {});
  part(() => runWideSphereCapRegression());
  for (let suite = 0; suite < 5; suite += 1) part(() => ran.push(suite));
  // The heavy suite fills part 1, so every unit-weight suite lands in part 2.
  assert.deepEqual(ran, [0, 1, 2, 3, 4]);
  assert.ok(SUITE_WEIGHTS.runWideSphereCapRegression > 5);
});

test('each part reports what it ran', () => {
  const { ran, lines } = runPart('2/3');
  assert.equal(lines.at(-1), `# part 2/3 ran ${ran.length} of ${WEIGHTS.length} suites`);
  assert.equal(lines.length, ran.length + 1);
});

test('a malformed part or weight fails instead of running nothing', () => {
  for (const value of ['0/2', '3/2', '2', 'a/b', '1/0', '1/2/3', ' 1/2']) {
    assert.throws(() => parsePart(value), /REMUS_PACKAGE_SUITE_PART/, value);
  }
  assert.throws(() => suiteParts('1/2', () => {})(() => {}, 0), /weight/);
  assert.throws(() => suiteParts('1/2', () => {})(() => {}, Number.NaN), /weight/);
});

test('both package scripts route every suite through the splitter', () => {
  for (const script of ['test-wasm-smoke.mjs', 'test-wasm-tarball-consumer.mjs']) {
    const source = readFileSync(new URL(script, import.meta.url), 'utf8').replaceAll('\r\n', '\n');
    assert.match(source, /const part = suiteParts\(\);/, script);
    assert.match(source, /\npart\.summary\(\);|\n {2}part\.summary\(\);/, script);
    // The aggregate consumer suite takes `part` and splits its own members.
    const bare = source.split('\n')
      .filter(line => /^\s*run[A-Z]\w+\(/.test(line) && !line.endsWith(', part);'));
    assert.deepEqual(bare, [], `${script} calls a suite outside part()`);
  }
});

test('CI runs every part of both suites, and qualification requires them all', () => {
  const read = path => readFileSync(new URL(path, import.meta.url), 'utf8').replaceAll('\r\n', '\n');
  const workflow = read('../.github/workflows/ci-checks.yml');
  const parts = /\n        part: \[([0-9, ]+)\]\n/.exec(workflow)[1].split(', ');
  const total = parts.length;
  assert.deepEqual(parts, Array.from({ length: total }, (_, index) => String(index + 1)));
  assert.ok(workflow.includes(`REMUS_PACKAGE_SUITE_PART: \${{ matrix.part }}/${total}`));
  assert.ok(workflow.includes(`name: WASM runtime (\${{ matrix.suite }} \${{ matrix.part }}/${total})`));
  const qualification = read('reuse-wasm-qualification.py');
  assert.ok(qualification.includes(`f"checks / WASM runtime ({suite} {part}/${total})"`));
  assert.ok(qualification.includes('for suite in ("smoke", "tarball-consumer")'));
  assert.ok(qualification.includes(`for part in (${parts.join(', ')}))`));
});
