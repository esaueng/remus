/**
 * Exact body-edit microbenchmark, callable in Node or a browser worker.
 * Node: node scripts/bench-body-edit.mjs
 * Browser: import { runBodyEditBenchmark } from this module and pass BrepKernel.
 * Setup, correctness checks and cleanup are outside the timed regions.
 * This measures the kernel and tessellation, not OpenZCAD regeneration or painting.
 */
export function runBodyEditBenchmark(BrepKernel, { samples = 30, warmup = 5 } = {}) {
  const results = [];
  for (const backgroundBodies of [0, 1000, 10000]) {
    for (const operation of ['extrude', 'moveFaces', 'pushPullFace']) {
      const edits = [], meshes = [];
      for (let i = 0; i < warmup + samples; i++) {
        const kernel = new BrepKernel();
        try {
          for (let j = 0; j < backgroundBodies; j++) kernel.makeBox(10, 20, 30);
          const body = kernel.makeBox(10, 20, 30);
          const face = Array.from(kernel.getSolidFaces(body)).find(
            face => kernel.getFaceNormal(face)[2] > 0.99,
          );
          if (face === undefined) throw new Error('Missing top face');
          const selection = Uint32Array.of(face);
          const start = performance.now();
          const result = operation === 'extrude'
            ? kernel.extrude(face, 0, 0, 1, 5)
            : operation === 'moveFaces'
              ? kernel.moveFaces(body, selection, 5)
              : kernel.pushPullFace(body, face, 5);
          const edited = performance.now();
          const mesh = kernel.tessellateSolid(result, 0.1, 0.5);
          const meshed = performance.now();
          try {
            const expected = operation === 'extrude' ? 1000 : 7000;
            if (Math.abs(kernel.volume(result, 0.1) - expected) > 1e-6 ||
                Math.abs(kernel.volume(body, 0.1) - 6000) > 1e-6 ||
                kernel.validateSolid(result) !== 0 || mesh.indices.length === 0) {
              throw new Error(`${operation}: geometry postcondition failed`);
            }
          } finally {
            mesh.free();
          }
          if (i >= warmup) {
            edits.push(edited - start);
            meshes.push(meshed - edited);
          }
        } finally {
          kernel.free();
        }
      }
      const quantile = (values, q) => values.sort((a, b) => a - b)[Math.ceil(values.length * q) - 1];
      results.push({ backgroundBodies, operation, samples,
        editP50Ms: quantile(edits, 0.5), editP95Ms: quantile(edits, 0.95),
        meshP50Ms: quantile(meshes, 0.5), meshP95Ms: quantile(meshes, 0.95) });
    }
  }
  return results;
}

if (typeof process !== 'undefined' && process.versions?.node) {
  const { pathToFileURL } = await import('node:url');
  if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
    const { createRequire } = await import('node:module');
    const require = createRequire(import.meta.url);
    const { BrepKernel } = require('../crates/wasm/pkg/remus_wasm_node.cjs');
    const { version } = require('../crates/wasm/pkg/package.json');
    console.log(JSON.stringify({ runtime: process.version, packageVersion: version,
      results: runBodyEditBenchmark(BrepKernel) }, null, 2));
  }
}
