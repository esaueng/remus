'use strict';

// Run in a fresh process so each host configuration owns its bindings, table
// and constructor probe. The shipped kernel must work with all three hosts.
const assert = require('node:assert/strict');
const { spawnSync } = require('node:child_process');

function runFusedArithmeticPackaged(kernelFile) {
  const modes = process.env.REMUS_TEST_SCALAR_PACKAGE === '1'
    ? ['scalar'] : ['native', 'unsupported', 'unfused', 'scalar-function-only'];
  for (const mode of modes) {
    const run = spawnSync(process.execPath, [__filename, kernelFile, mode], {
      encoding: 'utf8',
    });
    assert.ifError(run.error);
    assert.equal(run.status, 0, run.stderr);
  }
}

function checkHost(kernelFile, mode) {
  const { RemusKernel } = require(kernelFile);
  const NativeModule = WebAssembly.Module;
  const writeTable = WebAssembly.Table.prototype.set;
  let attempts = 0;
  // Independent capability witness: splat each operand, relaxed_madd, then
  // extract lane zero. Invalid candidate bytes must fail on a capable host.
  const modulePrefix = '0061736d0100000001080160037c7c7c017c03020100070801046d6164640000';
  const fusedBody = '0a160114002000fd142001fd142002fd14fd8702fd21000b';
  const unfusedBody = '0a0c010a0020002001a22002a00b';
  const bytes = (body) => Buffer.from(modulePrefix + body, 'hex');
  const delta = 2 ** -27;
  const qualified = (mode === 'native' || mode === 'scalar-function-only') && WebAssembly.validate(bytes(fusedBody)) &&
    new WebAssembly.Instance(new NativeModule(bytes(fusedBody))).exports.madd(
      1 + delta, 1 - delta, -1,
    ) === -(delta ** 2);
  let slot;
  let installed;
  let vectorSlot;
  WebAssembly.Module = new Proxy(NativeModule, {
    construct(target, args) {
      // The main kernel has already been instantiated by require(). Inspect
      // the optional module requested by the first RemusKernel constructor.
      attempts++;
      if (mode === 'unsupported') throw new WebAssembly.CompileError();
      // (func (export "madd") (param f64 f64 f64) (result f64)
      //   (f64.add (f64.mul (local.get 0) (local.get 1)) (local.get 2)))
      if (mode === 'unfused') return Reflect.construct(target, [bytes(unfusedBody)]);
      if (mode === 'scalar-function-only') return Reflect.construct(target, [bytes(fusedBody)]);
      return Reflect.construct(target, args);
    },
  });
  WebAssembly.Table.prototype.set = function (index, fn) {
    if (typeof fn === 'function' && fn.length === 3) {
      assert.equal(installed, undefined, 'one fused slot per module instance');
      installed = fn;
      slot = { table: this, index };
    }
    if (typeof fn === 'function' && fn.length === 6) {
      assert.equal(vectorSlot, undefined, 'one vector slot per module instance');
      vectorSlot = { table: this, index, dot: fn };
    }
    return writeTable.call(this, index, fn);
  };

  for (let i = 0; i < 2; i++) {
    const kernel = new RemusKernel();
    try {
      assert.equal(kernel.volume(kernel.makeBox(2, 3, 4), 0.1), 24);
    } finally {
      kernel.free();
    }
  }
  assert.equal(attempts, mode === 'scalar' ? 0 : 1);
  assert.equal(Boolean(installed), qualified);
  assert.equal(Boolean(vectorSlot), qualified && mode === 'native');
  if (vectorSlot) {
    const { dot, table, index } = vectorSlot;
    assert.equal(dot(1 + delta, 1 - delta, -1, 1, 0, 0), -(delta ** 2));
    assert.equal(dot(0, 0, 1 + delta, 1 - delta, -1, 1), -(delta ** 2));
    assert.equal(dot(0, 0, -1, 1, 1 + delta, 1 - delta), 0);
    assert.equal(dot(-0, 1, -0, 1, -0, 1), -0);
    assert.equal(dot(0, 0, Number.MIN_VALUE, 0.5, Number.MIN_VALUE, 1), 2 * Number.MIN_VALUE);
    // The same six-argument WASM type is required to replace a table slot.
    const forwarder = Buffer.from(
      '0061736d01000000010b0160067c7c7c7c7c7c017c02070101780166000007080104646f74330000', 'hex',
    );
    let calls = 0;
    const { dot3 } = new WebAssembly.Instance(new NativeModule(forwarder), {
      x: { f(...args) { calls++; return dot(...args); } },
    }).exports;
    writeTable.call(table, index, dot3);
    const kernel = new RemusKernel();
    try {
      assert.equal(kernel.volume(kernel.makeBox(3, 5, 7), 0.1), 105);
      assert.ok(calls > 0, 'the Rust kernel dispatches vector dots during measurement');
    } finally {
      writeTable.call(table, index, dot);
      kernel.free();
    }
  }
  if (installed) {
    assert.equal(installed(1 + delta, 1 - delta, -1), -(delta ** 2));
    assert.equal(installed(Number.MAX_VALUE, 2, -Number.MAX_VALUE), Number.MAX_VALUE);
    assert.equal(installed(Number.MIN_VALUE, 0.5, Number.MIN_VALUE), 2 * Number.MIN_VALUE);
    assert.equal(installed(-0, 1, -0), -0);
    // A type-matched Wasm export forwards calls to a counted JS import in
    // this disposable instance. A distinct box forces a fresh volume read.
    const forwarder = Buffer.from(
      '0061736d0100000001080160037c7c7c017c020701017801660000070801046d6164640000', 'hex',
    );
    let calls = 0;
    const { madd } = new WebAssembly.Instance(new NativeModule(forwarder), {
      x: { f(a, b, c) { calls++; return installed(a, b, c); } },
    }).exports;
    writeTable.call(slot.table, slot.index, madd);
    const kernel = new RemusKernel();
    try {
      assert.equal(kernel.volume(kernel.makeBox(5, 7, 11), 0.1), 385);
      assert.ok(calls > 0, 'the Rust kernel dispatches through the fused slot');
    } finally {
      writeTable.call(slot.table, slot.index, installed);
      kernel.free();
    }
  }
}

module.exports = { runFusedArithmeticPackaged };
if (require.main === module) checkHost(process.argv[2], process.argv[3]);
