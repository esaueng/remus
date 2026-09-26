const assert = require('node:assert/strict');
const { performance } = require('node:perf_hooks');
const path = require('node:path');

// Representative packaged-WASM GCS runs. Construction stays outside the
// timer; only gcsSolve / gcsSolveDetailed (or the warm drag re-solves) are
// timed. Independent geometric checks run after the timer.
const [pkg, workloadArg, sizeArg, modeArg, samplesArg, warmupArg] = process.argv.slice(2);
const workload = workloadArg;
const size = Number(sizeArg);
const mode = modeArg;
const samples = Number(samplesArg);
const warmup = Number(warmupArg);

assert(['independent_solved', 'coupled_chain', 'drag'].includes(workload), `workload ${workload}`);
assert(Number.isInteger(size) && [10, 100, 1000].includes(size), `size ${size}`);
assert(['solve', 'detailed'].includes(mode), `mode ${mode}`);
assert(Number.isInteger(samples) && samples >= 1 && samples <= 1000);
assert(Number.isInteger(warmup) && warmup >= 0 && warmup <= 100);

const { BrepKernel } = require(path.join(path.resolve(pkg), 'remus_wasm_node.cjs'));

const TOL = 1e-10;
const MAX_ITER = 100;
const DRAG_STEPS = 20;

function buildIndependent(kernel, sk, nParams) {
  const k = nParams / 2;
  for (let i = 0; i < k; i++) {
    const ax = 10 * i;
    const anchor = kernel.gcsAddPoint(sk, ax, 0, true);
    const free = kernel.gcsAddPoint(sk, ax + 1, 1, false);
    kernel.gcsAddConstraint(sk, JSON.stringify({ type: 'distance', a: anchor, b: free, value: 5 }));
    kernel.gcsAddConstraint(sk, JSON.stringify({ type: 'fixY', point: free, value: 4 }));
  }
}

function buildCoupled(kernel, sk, nParams) {
  const nPts = nParams / 2 + 1;
  const pts = [];
  pts.push(kernel.gcsAddPoint(sk, 0, 0, true));
  for (let i = 1; i < nPts; i++) {
    pts.push(kernel.gcsAddPoint(sk, i, 0.5 * (i % 2), false));
  }
  for (let i = 0; i + 1 < pts.length; i++) {
    const line = kernel.gcsAddLine(sk, pts[i], pts[i + 1]);
    kernel.gcsAddConstraint(sk, JSON.stringify({ type: 'distance', a: pts[i], b: pts[i + 1], value: 1 }));
    kernel.gcsAddConstraint(sk, JSON.stringify({ type: 'horizontal', line }));
  }
  return pts;
}

function parseSolve(out) {
  return typeof out === 'string' ? JSON.parse(out) : out;
}

function checkIndependent(kernel, sk, pts, size, workload) {
  let worst = 0;
  for (let i = 0; i < pts.length; i += 2) {
    const a = kernel.gcsPointPosition(sk, pts[i]);
    const b = kernel.gcsPointPosition(sk, pts[i + 1]);
    const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
    worst = Math.max(worst, Math.abs(d - 5));
    assert(Math.abs(d - 5) <= 1e-6, `${workload} ${size}: dist ${d}`);
  }
  return { oracle: 'hypot_distance_5', worst_abs_err: worst };
}

function collectPoints(kernel, sk, count) {
  const pts = [];
  for (let i = 0; i < count; i++) pts.push(i);
  return pts;
}

for (let sample = 0; sample < samples + warmup; sample++) {
  const kernel = new BrepKernel();
  try {
    const sk = kernel.gcsNew();
    let pts = [];
    if (workload === 'independent_solved') {
      buildIndependent(kernel, sk, size);
      const k = size / 2;
      for (let i = 0; i < 2 * k; i++) pts.push(i);
    } else {
      pts = buildCoupled(kernel, sk, size);
    }

    let operationNs;
    let result;
    let dof;
    if (workload === 'drag') {
      const cold = parseSolve(kernel.gcsSolve(sk, MAX_ITER, TOL));
      assert(cold.converged, `drag cold: ${JSON.stringify(cold)}`);
      const target = pts[25];
      const start = performance.now();
      let iters = 0;
      for (let s = 0; s < DRAG_STEPS; s++) {
        const cur = kernel.gcsPointPosition(sk, target);
        kernel.gcsSetPoint(sk, target, cur[0] + 0.5, cur[1] + 0.25);
        if (mode === 'solve') {
          const r = parseSolve(kernel.gcsSolve(sk, MAX_ITER, TOL));
          assert(r.converged);
          iters += r.iterations;
        } else {
          const r = parseSolve(kernel.gcsSolveDetailed(sk, MAX_ITER, TOL));
          assert(r.converged && !r.rolledBack);
          iters += r.iterations;
        }
      }
      operationNs = (performance.now() - start) * 1e6;
      // Chain oracle after the timer.
      for (let i = 0; i < pts.length; i++) {
        const p = kernel.gcsPointPosition(sk, pts[i]);
        assert(Math.abs(p[0] - i) <= 1e-6 && Math.abs(p[1]) <= 1e-8);
      }
      dof = parseSolve(kernel.gcsDof(sk));
      result = { converged: true, iterations: iters, classification: 'solved' };
      console.log(JSON.stringify({
        schema: 'remus-sketch-perf-sample-v1',
        workload,
        size,
        mode,
        sample,
        warmup: sample < warmup,
        validation: 'passed',
        operation_ns: operationNs,
        resource: 'solved',
        converged: true,
        iterations: iters,
        iterations_per_step: iters / DRAG_STEPS,
        drag_steps: DRAG_STEPS,
        num_params: dof.numParams,
        num_equations: dof.numEquations,
        rank: dof.rank,
        dof: dof.dof,
        classification: 'solved',
        jacobian_bytes: dof.numEquations * dof.numParams * 8,
        residual_bytes: dof.numEquations * 8,
        param_bytes: dof.numParams * 8,
        metrics: { oracle: 'chain_grid', drag_steps: DRAG_STEPS },
      }));
      continue;
    }

    const start = performance.now();
    if (mode === 'solve') {
      result = parseSolve(kernel.gcsSolve(sk, MAX_ITER, TOL));
    } else {
      result = parseSolve(kernel.gcsSolveDetailed(sk, MAX_ITER, TOL));
    }
    operationNs = (performance.now() - start) * 1e6;
    assert(result.converged, `${workload}: ${JSON.stringify(result)}`);
    dof = parseSolve(kernel.gcsDof(sk));
    if (workload === 'independent_solved') {
      const oracle = checkIndependent(kernel, sk, collectPoints(kernel, sk, size), size, workload);
      console.log(JSON.stringify({
        schema: 'remus-sketch-perf-sample-v1',
        workload,
        size,
        mode,
        sample,
        warmup: sample < warmup,
        validation: 'passed',
        operation_ns: operationNs,
        resource: 'solved',
        converged: true,
        iterations: result.iterations,
        max_residual: result.maxResidual,
        num_params: dof.numParams,
        num_equations: dof.numEquations,
        rank: dof.rank,
        dof: dof.dof,
        classification: workload === 'independent_solved' ? 'solved' : 'solved',
        jacobian_bytes: dof.numEquations * dof.numParams * 8,
        residual_bytes: dof.numEquations * 8,
        param_bytes: dof.numParams * 8,
        metrics: oracle,
      }));
    } else {
      for (let i = 0; i < pts.length; i++) {
        const p = kernel.gcsPointPosition(sk, pts[i]);
        assert(Math.abs(p[0] - i) <= 1e-6 && Math.abs(p[1]) <= 1e-8);
      }
      console.log(JSON.stringify({
        schema: 'remus-sketch-perf-sample-v1',
        workload,
        size,
        mode,
        sample,
        warmup: sample < warmup,
        validation: 'passed',
        operation_ns: operationNs,
        resource: 'solved',
        converged: true,
        iterations: result.iterations,
        max_residual: result.maxResidual,
        num_params: dof.numParams,
        num_equations: dof.numEquations,
        rank: dof.rank,
        dof: dof.dof,
        classification: 'solved',
        jacobian_bytes: dof.numEquations * dof.numParams * 8,
        residual_bytes: dof.numEquations * 8,
        param_bytes: dof.numParams * 8,
        metrics: { oracle: 'chain_grid' },
      }));
    }
  } finally {
    kernel.free();
  }
}
