#!/usr/bin/env node
'use strict';
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const { stripTypeScriptTypes } = require('node:module');
const { fixture, plan, sha256 } = require('./holder_recipe.cjs');

// Optional source-fidelity diagnostic, requiring Node's TypeScript erasure.
// Only the exact pinned public compiler bytes may be executed. This checks
// its pure numeric planner, not the document commands or consumer build loop.
async function check(sourcePath, capturedRuns) {
  const data = fixture(), compilerPath = data.consumer.recipeCompiler;
  assert.ok(fs.statSync(sourcePath).size <= 64 * 1024, 'Compiler source exceeds diagnostic bound');
  const source = fs.readFileSync(sourcePath, 'utf8');
  assert.equal(sha256(source), data.consumerSourceHashes[compilerPath], 'Compiler source is not the verified pin');
  const first = source.indexOf('const PLANE_FOR_AXIS'), last = source.indexOf('const isNumber');
  assert.ok(first >= 0 && last > first, 'Pinned planner extraction markers changed');
  const javascript = stripTypeScriptTypes(source.slice(first, last), { mode: 'transform' });
  const { growingHolderPlan } = await import(`data:text/javascript;base64,${Buffer.from(javascript).toString('base64')}`);
  const widths = [...new Set([46, ...capturedRuns.flatMap(run => run.records.map(r => r.width))])].sort((a, b) => a - b);
  const rows = [];
  for (const width of widths) {
    const original = growingHolderPlan(data.recipe), adapted = plan(data.recipe, width);
    const scope = { opening_width: width, holder_height: 58, require_min(value, minimum) {
      assert.ok(value >= minimum); return value;
    } };
    const value = expression => typeof expression === 'string' ?
      vm.runInNewContext(expression, scope, { timeout: 100 }) : expression;
    const differences = [];
    for (let i = 0; i < original.pieces.length; i++) {
      for (const [axis, index] of [['x', 0], ['y', 1], ['z', 2]]) {
        differences.push(Math.abs(original.pieces[i].mask.min[axis] - adapted.pieces[i].min[index]),
          Math.abs(original.pieces[i].mask.max[axis] - adapted.pieces[i].max[index]),
          Math.abs(value(original.pieces[i].move[axis]) - adapted.pieces[i].shift[index]));
      }
    }
    for (let i = 0; i < original.bridges.length; i++) {
      assert.deepEqual(original.bridges[i].section, adapted.bridges[i].section);
      differences.push(Math.abs(value(original.bridges[i].offset) - adapted.bridges[i].offset),
        Math.abs(value(original.bridges[i].distance) - adapted.bridges[i].distance));
      for (const [axis, index] of [['x', 0], ['y', 1], ['z', 2]])
        differences.push(Math.abs(value(original.bridges[i].move[axis]) - adapted.bridges[i].shift[index]));
    }
    assert.deepEqual(original.unionOrder, adapted.unionOrder);
    const maximumNumericPlanDifference = Math.max(...differences);
    assert.ok(maximumNumericPlanDifference <= 1e-12, 'Numeric planner differs beyond diagnostic roundoff bound');
    rows.push({ width, profilesAndUnionOrderExact: true, maximumNumericPlanDifference });
  }
  return { consumerRevision: data.consumer.revision, compilerPath, compilerSha256: sha256(source),
    execution: 'Unchanged pure growingHolderPlan function and local helpers extracted from verified pinned compiler source; TypeScript erased. Not the document command compiler or consumer build graph.',
    expressionEvaluation: 'Original generated expressions evaluated with width and held height 58; adapted arithmetic may differ in final binary rounding.',
    numericPlanDiagnosticBound: 1e-12, rows };
}
module.exports = { check };
if (require.main === module) {
  (async () => {
    assert.equal(process.argv.length, 5, 'Expected pinned-compiler.ts captured-runs.json new-output.json');
    assert.ok(!fs.existsSync(process.argv[4]), 'Output already exists');
    const result = await check(process.argv[2], JSON.parse(fs.readFileSync(process.argv[3])));
    fs.writeFileSync(path.resolve(process.argv[4]), JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
  })().catch(error => { console.error(error.stack || error); process.exitCode = 1; });
}
