#!/usr/bin/env node
/**
 * Packaged-runtime witnesses for the B75 constrained-ellipse entity.
 *
 * This module runs inside the real packaged WASM runtime (the
 * `remus_wasm_node.cjs` pair built by `cargo xtask wasm-build`), not the
 * native `cargo test -p remus-wasm` contract suite. It exercises the typed
 * GCS ellipse surface (`gcsAddEllipse`, `gcsEllipseParams`, `gcsSetEllipse`,
 * the nine ellipse constraint tags) through all three call paths — direct,
 * `executeBatch`, and `executeBatchV2` — with independent geometric oracles
 * (wrapper agreement alone is never the oracle).
 *
 * Fixtures (unit scale, origin placement — the scale/translation matrix
 * lives natively in `crates/sketch`):
 * - driven ellipse: fixed center, axes 6 x 2.5, angle 0.7 rad.
 * - tangent placement: axis-aligned 4 x 2 ellipse with a line tangent at a
 *   composed contact point (point-on-ellipse + point-on-line + tangency).
 * - contradictory axis drives: failed detailed solve rolls back.
 *
 * Independent oracles:
 * - solved semiaxes/angle read back to 1e-9 of their driven targets.
 * - contact point satisfies the implicit ellipse equation, lies on the
 *   line, and the line direction is perpendicular to the implicit gradient.
 * - failed detailed solves report unsatisfied + rolledBack with pre-solve
 *   params published.
 * - invalid inputs (non-positive semiaxis, unknown constraint type) refuse
 *   with typed errors without poisoning sibling ops.
 */

import assert from 'node:assert/strict';

const TOL = 1e-10;
const PARAM_TOL = 1e-9;

const assertClose = (actual, expected, label, tol = PARAM_TOL) => {
  assert.equal(typeof actual, 'number', `${label}: must be a number`);
  assert.ok(Number.isFinite(actual), `${label}: must be finite, got ${actual}`);
  assert.ok(
    Math.abs(actual - expected) <= tol,
    `${label}: got ${actual}, want ${expected}`,
  );
};

const batchOk = (response, index, label, v2 = false) => {
  const item = response[index];
  assert.ok(item, `${label}: missing item ${index} in ${JSON.stringify(response)}`);
  if (v2) {
    assert.ok('ok' in item, `${label}: item ${index} failed: ${JSON.stringify(item)}`);
    return item.ok;
  }
  assert.ok('ok' in item, `${label}: item ${index} failed: ${JSON.stringify(item)}`);
  return item.ok;
};

const addConstraint = (list, sketch, constraint) => {
  list.push({ op: 'gcsAddConstraint', args: { sketch, constraint } });
};

export function runEllipsePackaged({ BrepKernel }) {
  // 1. Driven ellipse through all three routes.
  for (const route of ['direct', 'batchV1', 'batchV2']) {
    const kernel = new BrepKernel();
    try {
      let params;
      let detailed;
      if (route === 'direct') {
        const s = kernel.gcsNew();
        const c = kernel.gcsAddPoint(s, 0, 0, true);
        const e = kernel.gcsAddEllipse(s, c, 1, 1, 0);
        kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAxisA', ellipse: e, value: 6 }));
        kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAxisB', ellipse: e, value: 2.5 }));
        kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAngle', ellipse: e, value: 0.7 }));
        const solved = JSON.parse(kernel.gcsSolve(s, 200, TOL));
        assert.equal(solved.converged, true, `driven ${route}: must converge`);
        params = Array.from(kernel.gcsEllipseParams(s, e));
        detailed = JSON.parse(kernel.gcsSolveDetailed(s, 200, TOL));
      } else {
        const program = [
          { op: 'gcsNew', args: {} },
          { op: 'gcsAddPoint', args: { sketch: 0, x: 0, y: 0, fixed: true } },
          { op: 'gcsAddEllipse', args: { sketch: 0, center: 0, a: 1, b: 1, angle: 0 } },
        ];
        addConstraint(program, 0, { type: 'ellipseAxisA', ellipse: 0, value: 6 });
        addConstraint(program, 0, { type: 'ellipseAxisB', ellipse: 0, value: 2.5 });
        addConstraint(program, 0, { type: 'ellipseAngle', ellipse: 0, value: 0.7 });
        program.push(
          { op: 'gcsSolve', args: { sketch: 0, maxIterations: 200, tolerance: TOL } },
          { op: 'gcsEllipseParams', args: { sketch: 0, ellipse: 0 } },
          { op: 'gcsSolveDetailed', args: { sketch: 0, maxIterations: 200, tolerance: TOL } },
          { op: 'gcsDof', args: { sketch: 0 } },
        );
        const raw = route === 'batchV1'
          ? kernel.executeBatch(JSON.stringify(program))
          : kernel.executeBatchV2(JSON.stringify(program));
        const response = JSON.parse(raw);
        const solved = batchOk(response, 6, `driven ${route}`, route === 'batchV2');
        assert.equal(solved.converged, true, `driven ${route}: must converge`);
        params = batchOk(response, 7, `driven ${route} params`, route === 'batchV2');
        detailed = batchOk(response, 8, `driven ${route} detailed`, route === 'batchV2');
        const dof = batchOk(response, 9, `driven ${route} dof`, route === 'batchV2');
        assert.equal(dof.dof, 0, `driven ${route}: fully driven`);
      }
      assertClose(params[0], 0, `driven ${route} cx`);
      assertClose(params[1], 0, `driven ${route} cy`);
      assertClose(params[2], 6, `driven ${route} a`);
      assertClose(params[3], 2.5, `driven ${route} b`);
      assertClose(params[4], 0.7, `driven ${route} angle`);
      assert.equal(detailed.converged, true, `driven ${route} detailed`);
      assert.equal(detailed.classification, 'solved', `driven ${route} class`);
      console.log(`ok - ellipse packaged driven solve, ${route}: [0,0,6,2.5,0.7], solved/dof 0`);
    } finally {
      kernel.free();
    }
  }

  // 2. Tangent placement with an independent geometric oracle.
  for (const route of ['direct', 'batchV1', 'batchV2']) {
    const kernel = new BrepKernel();
    try {
      let qpos;
      if (route === 'direct') {
        const s = kernel.gcsNew();
        const c = kernel.gcsAddPoint(s, 0, 0, true);
        const e = kernel.gcsAddEllipse(s, c, 4, 2, 0);
        const q = kernel.gcsAddPoint(s, 3, 1.5, false);
        const a = kernel.gcsAddPoint(s, -6, 6, true);
        const b = kernel.gcsAddPoint(s, 6, 6.5, false);
        const line = kernel.gcsAddLine(s, a, b);
        for (const [ty, val] of [['ellipseAxisA', 4], ['ellipseAxisB', 2], ['ellipseAngle', 0]]) {
          kernel.gcsAddConstraint(s, JSON.stringify({ type: ty, ellipse: e, value: val }));
        }
        kernel.gcsAddConstraint(s, JSON.stringify({ type: 'pointOnEllipse', point: q, ellipse: e }));
        kernel.gcsAddConstraint(
          s,
          JSON.stringify({ type: 'pointLineDistance', point: q, line, value: 0 }),
        );
        kernel.gcsAddConstraint(
          s,
          JSON.stringify({ type: 'tangentLineEllipse', line, ellipse: e, point: q }),
        );
        const solved = JSON.parse(kernel.gcsSolve(s, 500, TOL));
        assert.equal(solved.converged, true, `tangent ${route}: must converge`);
        qpos = Array.from(kernel.gcsPointPosition(s, q));
        var apos = Array.from(kernel.gcsPointPosition(s, a));
        var bpos = Array.from(kernel.gcsPointPosition(s, b));
      } else {
        const program = [
          { op: 'gcsNew', args: {} },
          { op: 'gcsAddPoint', args: { sketch: 0, x: 0, y: 0, fixed: true } },
          { op: 'gcsAddEllipse', args: { sketch: 0, center: 0, a: 4, b: 2, angle: 0 } },
          { op: 'gcsAddPoint', args: { sketch: 0, x: 3, y: 1.5, fixed: false } },
          { op: 'gcsAddPoint', args: { sketch: 0, x: -6, y: 6, fixed: true } },
          { op: 'gcsAddPoint', args: { sketch: 0, x: 6, y: 6.5, fixed: false } },
          { op: 'gcsAddLine', args: { sketch: 0, p1: 2, p2: 3 } },
        ];
        addConstraint(program, 0, { type: 'ellipseAxisA', ellipse: 0, value: 4 });
        addConstraint(program, 0, { type: 'ellipseAxisB', ellipse: 0, value: 2 });
        addConstraint(program, 0, { type: 'ellipseAngle', ellipse: 0, value: 0 });
        addConstraint(program, 0, { type: 'pointOnEllipse', point: 1, ellipse: 0 });
        addConstraint(program, 0, { type: 'pointLineDistance', point: 1, line: 0, value: 0 });
        addConstraint(program, 0, { type: 'tangentLineEllipse', line: 0, ellipse: 0, point: 1 });
        program.push(
          { op: 'gcsSolve', args: { sketch: 0, maxIterations: 500, tolerance: TOL } },
          { op: 'gcsPointPosition', args: { sketch: 0, point: 1 } },
          { op: 'gcsPointPosition', args: { sketch: 0, point: 2 } },
          { op: 'gcsPointPosition', args: { sketch: 0, point: 3 } },
        );
        const raw = route === 'batchV1'
          ? kernel.executeBatch(JSON.stringify(program))
          : kernel.executeBatchV2(JSON.stringify(program));
        const response = JSON.parse(raw);
        const solved = batchOk(response, 13, `tangent ${route}`, route === 'batchV2');
        assert.equal(solved.converged, true, `tangent ${route}: must converge`);
        qpos = batchOk(response, 14, `tangent ${route} contact`, route === 'batchV2');
        apos = batchOk(response, 15, `tangent ${route} a`, route === 'batchV2');
        bpos = batchOk(response, 16, `tangent ${route} b`, route === 'batchV2');
      }
      const [qx, qy] = qpos;
      // On the 4x2 ellipse: (x/4)^2 + (y/2)^2 == 1.
      assert.ok(
        Math.abs((qx / 4) * (qx / 4) + (qy / 2) * (qy / 2) - 1) < 1e-8,
        `tangent ${route}: contact on ellipse, got (${qx},${qy})`,
      );
      // On the line through a->b.
      const [ax, ay] = apos;
      const [bx, by] = bpos;
      const lx = bx - ax;
      const ly = by - ay;
      const len = Math.hypot(lx, ly);
      assert.ok(
        Math.abs(lx * (qy - ay) - ly * (qx - ax)) / len < 1e-8,
        `tangent ${route}: contact on line`,
      );
      // Line direction perpendicular to the implicit gradient (qx/8, qy/2).
      const gx = qx / 8;
      const gy = qy / 2;
      assert.ok(
        Math.abs(lx * gx + ly * gy) / (len * Math.hypot(gx, gy)) < 1e-8,
        `tangent ${route}: direction perpendicular to gradient`,
      );
      console.log(`ok - ellipse packaged tangent placement, ${route}: contact verified`);
    } finally {
      kernel.free();
    }
  }

  // 3. Failed detailed solve rolls back identically on all routes; errors
  //    are typed (V2 codes) without poisoning siblings.
  for (const route of ['direct', 'batchV1', 'batchV2']) {
    const kernel = new BrepKernel();
    try {
      let detailed;
      let params;
      if (route === 'direct') {
        const s = kernel.gcsNew();
        const c = kernel.gcsAddPoint(s, 0, 0, true);
        const e = kernel.gcsAddEllipse(s, c, 3, 2, 0.1);
        kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAxisA', ellipse: e, value: 3 }));
        kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAxisA', ellipse: e, value: 9 }));
        detailed = JSON.parse(kernel.gcsSolveDetailed(s, 200, TOL));
        params = Array.from(kernel.gcsEllipseParams(s, e));
      } else {
        const program = [
          { op: 'gcsNew', args: {} },
          { op: 'gcsAddPoint', args: { sketch: 0, x: 0, y: 0, fixed: true } },
          { op: 'gcsAddEllipse', args: { sketch: 0, center: 0, a: 3, b: 2, angle: 0.1 } },
          { op: 'gcsAddConstraint', args: { sketch: 0, constraint: { type: 'ellipseAxisA', ellipse: 0, value: 3 } } },
          { op: 'gcsAddConstraint', args: { sketch: 0, constraint: { type: 'ellipseAxisA', ellipse: 0, value: 9 } } },
          { op: 'gcsSolveDetailed', args: { sketch: 0, maxIterations: 200, tolerance: TOL } },
          { op: 'gcsEllipseParams', args: { sketch: 0, ellipse: 0 } },
        ];
        const raw = route === 'batchV1'
          ? kernel.executeBatch(JSON.stringify(program))
          : kernel.executeBatchV2(JSON.stringify(program));
        const response = JSON.parse(raw);
        detailed = batchOk(response, 5, `rollback ${route}`, route === 'batchV2');
        params = batchOk(response, 6, `rollback ${route} params`, route === 'batchV2');
      }
      assert.equal(detailed.converged, false, `rollback ${route}: must fail`);
      assert.equal(detailed.classification, 'unsatisfied', `rollback ${route}: class`);
      assert.equal(detailed.rolledBack, true, `rollback ${route}: flag`);
      assertClose(params[2], 3, `rollback ${route} a`, 1e-12);
      assertClose(params[3], 2, `rollback ${route} b`, 1e-12);
      assertClose(params[4], 0.1, `rollback ${route} angle`, 1e-12);
      console.log(`ok - ellipse packaged rollback, ${route}: unsatisfied + pre-solve params`);
    } finally {
      kernel.free();
    }
  }

  // 4. Typed refusal: non-positive semiaxis + unknown constraint type fail
  //    closed (V2: invalid_argument) while siblings still run.
  {
    const kernel = new BrepKernel();
    try {
      const program = [
        { op: 'gcsNew', args: {} },
        { op: 'gcsAddPoint', args: { sketch: 0, x: 0, y: 0, fixed: true } },
        { op: 'gcsAddEllipse', args: { sketch: 0, center: 0, a: 0, b: 1, angle: 0 } },
        { op: 'gcsAddEllipse', args: { sketch: 0, center: 0, a: 2, b: 1, angle: 0 } },
        { op: 'gcsAddConstraint', args: { sketch: 0, constraint: { type: 'warp' } } },
      ];
      const v1 = JSON.parse(kernel.executeBatch(JSON.stringify(program)));
      assert.ok('ok' in v1[0] && 'ok' in v1[1], 'prefix ops succeed');
      assert.ok('error' in v1[2] && v1[2].error.includes('semiaxis'), `a=0 message: ${JSON.stringify(v1[2])}`);
      assert.ok('ok' in v1[3], 'sibling still runs');
      assert.ok('error' in v1[4], 'unknown type fails');
      const kernel2 = new BrepKernel();
      try {
        const v2 = JSON.parse(kernel2.executeBatchV2(JSON.stringify(program)));
        assert.equal(v2[2].error.code, 'invalid_argument', JSON.stringify(v2[2]));
        assert.equal(v2[2].error.category, 'invalid_input', JSON.stringify(v2[2]));
        assert.ok('ok' in v2[3], 'V2 sibling still runs');
      } finally {
        kernel2.free();
      }
      console.log('ok - ellipse packaged typed refusals: invalid_argument/invalid_input, siblings run');
    } finally {
      kernel.free();
    }
  }

  // 5. Drag sequence: set ellipse params between solves (browser pattern).
  {
    const kernel = new BrepKernel();
    try {
      const s = kernel.gcsNew();
      const c = kernel.gcsAddPoint(s, 0, 0, true);
      const e = kernel.gcsAddEllipse(s, c, 1, 1, 0);
      kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAxisA', ellipse: e, value: 4 }));
      kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAxisB', ellipse: e, value: 2 }));
      kernel.gcsAddConstraint(s, JSON.stringify({ type: 'ellipseAngle', ellipse: e, value: 0 }));
      for (const [cx, a] of [[0, 4], [5, 7], [-3, 2]]) {
        kernel.gcsSetEllipse(s, e, cx, 0, a, 2, 0);
        // The edit itself is exact (pre-solve readback).
        const edited = Array.from(kernel.gcsEllipseParams(s, e));
        assertClose(edited[0], cx, 'drag edit cx');
        assertClose(edited[2], a, 'drag edit a');
        // Solving re-applies the axis drives (a back to 4) while the
        // dragged center sticks (fixed point).
        const solved = JSON.parse(kernel.gcsSolve(s, 200, TOL));
        assert.equal(solved.converged, true, `drag to (${cx},${a}) must converge`);
        const params = Array.from(kernel.gcsEllipseParams(s, e));
        assertClose(params[0], cx, 'drag cx');
        assertClose(params[2], 4, 'drag a re-driven');
      }
      console.log('ok - ellipse packaged drag sequence: edit/solve/edit converges');
    } finally {
      kernel.free();
    }
  }
}

// Allow direct execution: `node scripts/ellipse-packaged.mjs`
// loads the freshly built (or committed) kernel from the standard pkg path.
if (process.argv[1] && process.argv[1].endsWith('ellipse-packaged.mjs')) {
  const { createRequire } = await import('node:module');
  const { dirname, resolve } = await import('node:path');
  const { fileURLToPath } = await import('node:url');
  const __dirname = dirname(fileURLToPath(import.meta.url));
  const projectRoot = resolve(__dirname, '..');
  const require = createRequire(import.meta.url);
  const { BrepKernel } = require(resolve(projectRoot, 'crates/wasm/pkg/remus_wasm_node.cjs'));
  runEllipsePackaged({ BrepKernel });
  console.log('\nAll ellipse packaged witnesses passed');
}
