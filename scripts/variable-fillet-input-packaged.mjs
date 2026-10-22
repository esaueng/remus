/** Installed-package contracts for variable-fillet numeric input validation.
 * Run with REMUS_CONSUMER_DIR pointing at an npm-installed consumer, or use
 * REMUS_PACKAGE_ROOT to select a repository containing freshly built packages.
 */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

function state(kernel, solid) {
  const memory = JSON.parse(kernel.checkpointMemoryStats());
  return {
    arena: kernel.serializeSolids(Uint32Array.of(solid)),
    journal: kernel.journalSummary(),
    allocated: memory.allocatedSlots,
    retired: memory.retiredSlots,
  };
}

function invoke(kernel, surface, solid, specs) {
  if (surface === 'direct') return kernel.filletVariable(solid, JSON.stringify(specs));
  if (surface === 'directDetailed') return kernel.filletVariableDetailed(solid, JSON.stringify(specs), false);
  const [batch, operation] = surface.split(':');
  return JSON.parse(kernel[batch](JSON.stringify([
    { op: operation, args: { solid, specs, exactOnly: false } },
    { op: 'volume', args: { solid, deflection: 0.01 } },
  ])));
}

const surfaces = [
  'direct', 'directDetailed',
  'executeBatch:filletVariable', 'executeBatchV2:filletVariable',
  'executeBatch:filletVariableDetailed', 'executeBatchV2:filletVariableDetailed',
];

export function runVariableFilletInputPackaged({ BrepKernel }) {
  for (const surface of surfaces) {
    const kernel = new BrepKernel();
    try {
      const solid = kernel.makeBox(10, 10, 10);
      const edge = kernel.getSolidEdges(solid)[0];
      for (const field of ['radius', 'start', 'end', 'startRadius', 'endRadius', 'startSetback', 'endSetback']) {
        for (const invalid of ['one', true, [], {}]) {
          const spec = { edge, law: 'constant', radius: 1, start: 1, end: 1, [field]: invalid };
          const label = `${surface}: ${field}=${JSON.stringify(invalid)}`;
          const before = state(kernel, solid);
          if (surface === 'direct') {
            assert.throws(() => invoke(kernel, surface, solid, [spec]), new RegExp(`'${field}'.*must be a number`), label);
          } else {
            const output = invoke(kernel, surface, solid, [spec]);
            const detailed = surface === 'directDetailed' ? output : output[0].ok;
            if (surface === 'directDetailed' || surface.endsWith('Detailed')) {
              assert.equal(detailed?.status, 'error', label);
              assert.equal(detailed.code, 'invalid_argument', label);
              assert.equal(detailed.category, 'invalid_input', label);
              assert.ok(detailed.details.message.includes(`'${field}'`), label);
            } else if (surface.startsWith('executeBatchV2')) {
              assert.equal(output[0].error?.code, 'invalid_argument', label);
              assert.equal(output[0].error.category, 'invalid_input', label);
              assert.ok(output[0].error.message.includes(`'${field}'`), label);
            } else {
              assert.ok(output[0].error?.includes(`'${field}'`), label);
            }
            if (Array.isArray(output)) assert.ok(Math.abs(output[1].ok - 1000) < 1e-9, label);
          }
          assert.deepEqual(state(kernel, solid), before, label);
          assert.ok(Math.abs(kernel.volume(solid, 0.01) - 1000) < 1e-9, label);
        }
      }
    } finally { kernel.free(); }
  }

  // Each group must produce exactly the same body with documented defaults,
  // numeric nulls, inferred laws, aliases, and primary-field precedence.
  const equivalentSpecs = [
    [
      { law: 'constant', radius: 1 },
      {},
      { radius: null, start: null, end: null, startRadius: null, endRadius: null,
        startSetback: null, endSetback: null },
    ],
    [
      { law: 'linear', start: 1, end: 2 },
      { startRadius: 1, endRadius: 2 },
      { start: null, end: null, startRadius: 1, endRadius: 2 },
      { start: 1, end: 2, startRadius: 3, endRadius: 4 },
    ],
  ];
  for (const specs of equivalentSpecs) {
    let expected;
    for (const surface of surfaces) {
      for (const spec of specs) {
        const kernel = new BrepKernel();
        try {
          const solid = kernel.makeBox(10, 10, 10);
          const edge = kernel.getSolidEdges(solid)[0];
          const output = invoke(kernel, surface, solid, [{ edge, ...spec }]);
          const result = surface === 'direct' || surface === 'directDetailed' ? output : output[0].ok;
          const handle = typeof result === 'number' ? result : result?.value;
          if (typeof result !== 'number') assert.equal(result?.status, 'ok', `${surface}: ${JSON.stringify(spec)}`);
          assert.equal(typeof handle, 'number');
          const bytes = kernel.serializeSolids(Uint32Array.of(handle));
          if (expected === undefined) expected = bytes;
          else assert.deepEqual(bytes, expected, `${surface}: ${JSON.stringify(spec)}`);
        } finally { kernel.free(); }
      }
    }
  }
  console.log('ok - variable-fillet numeric fields refuse typed and atomically; defaults and aliases agree');
}

if (process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const consumer = process.env.REMUS_CONSUMER_DIR;
  const root = process.env.REMUS_PACKAGE_ROOT || resolve(dirname(fileURLToPath(import.meta.url)), '..');
  const require = createRequire(resolve(consumer || root, 'variable-fillet-input-witness.cjs'));
  runVariableFilletInputPackaged(require(consumer ? 'remus-wasm' : resolve(root, 'crates/wasm/pkg/remus_wasm_node.cjs')));
}
