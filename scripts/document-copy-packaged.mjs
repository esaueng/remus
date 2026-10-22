import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const bytes = (kernel, roots) => Buffer.from(kernel.serializeSolids(roots));
const stats = (kernel) => JSON.parse(kernel.checkpointMemoryStats());

/** Exercise the fresh-owner boundary against either local or installed packages. */
export function runDocumentCopyPackaged({ BrepKernel, RemusIo }, { onObservation = () => {} } = {}) {
  assert.equal(typeof BrepKernel.prototype.copyDocument, 'function');
  for (const scale of [0.001, 1, 1000]) {
    const source = new BrepKernel();
    let destination;
    let copy;
    try {
      source.makeBox(scale, scale, scale); // live, deliberately unselected
      const solid = source.makeBox(2 * scale, 3 * scale, 4 * scale);
      const checkpoint = source.checkpoint();
      let stale;
      for (let i = 0; i < 8; i++) {
        stale = source.makeBox(scale, scale, scale);
        source.restore(checkpoint);
      }
      const before = bytes(source, [solid, solid]);
      const sourceStats = stats(source);
      const journal = source.journalSummary();
      for (const action of [
        () => source.copyDocument([0xffff_ffff], [], []),
        () => source.copyDocument([solid], [], [], 1),
        () => source.copyDocument([solid, solid], [], [], 1),
        () => source.copyDocument([], [], [], 0xffff_ffff),
        // Out-of-range or non-integer budgets must not wrap or truncate.
        ...[2 ** 32 + 3_000_000, Number.NaN, Infinity, -1, 1.5].map(
          (limit) => () => source.copyDocument([], [], [], limit),
        ),
      ]) assert.throws(action);
      assert.deepEqual(stats(source), sourceStats);
      assert.equal(source.journalSummary(), journal);
      assert.deepEqual(bytes(source, [solid, solid]), before);

      copy = source.copyDocument([solid, solid], [], []);
      const roots = Array.from(copy.solids);
      assert.equal(roots.length, 2);
      assert.equal(roots[0], roots[1]);
      assert.equal(copy.sheets.length, 0);
      assert.equal(copy.wires.length, 0);
      destination = copy.takeKernel();
      assert.throws(() => copy.takeKernel(), /already been taken/);
      copy.free(); copy = undefined;
      assert.deepEqual(bytes(destination, roots), before);
      assert.equal(destination.getSolidFaces(roots[0]).length, 6);
      assert.ok(Math.abs(destination.volume(roots[0], 0.02 * scale) / (24 * scale ** 3) - 1) < 1e-10);
      assert.equal(JSON.parse(destination.validateSolidDetailed(roots[0])).errorCount, 0);
      assert.equal(stats(destination).allocatedSlots, 64);
      assert.equal(stats(destination).retiredSlots, 0);
      assert.equal(destination.checkpointCount(), 0);
      assert.throws(() => destination.restore(checkpoint), /invalid checkpoint/);
      destination.makeBox(5 * scale, scale, scale);
      assert.throws(() => source.volume(stale, 0.02 * scale));
      source.restore(checkpoint);
      assert.deepEqual(bytes(source, [solid, solid]), before);
      assert.ok(source.makeBox(scale, scale, scale) > stale);
      assert.equal(destination.checkpointCount(), 0);
    } finally {
      copy?.free(); destination?.free(); source.free();
    }
  }

  const mixed = new BrepKernel();
  let mixedTransfer;
  let mixedNext;
  try {
    const solid = mixed.makeBox(2, 3, 4);
    const sheet = mixed.makeSheetBody([mixed.getSolidFaces(solid)[0]]);
    const wire = mixed.makeRegularPolygonWire(1, 4);
    mixedTransfer = mixed.copyDocument([solid, solid], [sheet, sheet], [wire, wire]);
    const solids = Array.from(mixedTransfer.solids);
    const sheets = Array.from(mixedTransfer.sheets);
    const wires = Array.from(mixedTransfer.wires);
    assert.equal(sheets[0], sheets[1]); assert.equal(wires[0], wires[1]);
    mixedNext = mixedTransfer.takeKernel();
    assert.deepEqual(bytes(mixedNext, solids), bytes(mixed, [solid, solid]));
    assert.deepEqual(Buffer.from(mixedNext.serializeSheets(sheets)), Buffer.from(mixed.serializeSheets([sheet, sheet])));
    assert.deepEqual(Buffer.from(mixedNext.serializeWires(wires)), Buffer.from(mixed.serializeWires([wire, wire])));
    assert.ok(Math.abs(mixedNext.wireLength(wires[0]) - 4 * Math.SQRT2) < 1e-12);
  } finally { mixedTransfer?.free(); mixedNext?.free(); mixed.free(); }

  // Read exact carrier/trim/authority/attribute/history bytes on a real model.
  const io = new RemusIo();
  const holder = new BrepKernel();
  let holderCopy;
  let nextHolder;
  try {
    const file = new URL('../crates/io/tests/data/shapr3d_hammer_holder.step', import.meta.url);
    const roots = Array.from(holder.deserializeSolids(io.importStep(readFileSync(file))));
    assert.equal(roots.length, 1);
    assert.equal(holder.getSolidFaces(roots[0]).length, 160);
    const before = bytes(holder, roots);
    holder.checkpoint();
    holderCopy = holder.copyDocument(roots, [], []);
    const copiedRoots = Array.from(holderCopy.solids);
    nextHolder = holderCopy.takeKernel();
    holderCopy.free(); holderCopy = undefined;
    assert.deepEqual(bytes(nextHolder, copiedRoots), before);
    assert.equal(nextHolder.getSolidFaces(copiedRoots[0]).length, 160);
    assert.equal(nextHolder.checkpointCount(), 0);
    assert.equal(holder.checkpointCount(), 1);
  } finally {
    holderCopy?.free(); nextHolder?.free(); holder.free(); io.free();
  }

  // Creating a new owner must not re-enable module-wide disabled reuse.
  const source = new BrepKernel();
  let transfer;
  let taken;
  try {
    source.setMeshReuseEnabled(false);
    source.setMeasurementReuseEnabled(false);
    const mesh = source.meshReuseStats();
    const measurement = source.measurementReuseStats();
    transfer = source.copyDocument([], [], [], 0);
    taken = transfer.takeKernel();
    assert.equal(taken.meshReuseStats(), mesh);
    assert.equal(taken.measurementReuseStats(), measurement);
    for (const method of ['executeBatch', 'executeBatchV2']) {
      const result = JSON.parse(source[method](JSON.stringify([{ op: 'copyDocument', args: {} }])));
      assert.ok(result[0].error, 'JSON batches cannot return a kernel owner');
    }
    // Dropping an untaken transfer is also valid and never owns the source.
    source.copyDocument([], [], []).free();
    assert.equal(source.checkpointCount(), 0);
  } finally {
    source.setMeshReuseEnabled(true); source.setMeasurementReuseEnabled(true);
    transfer?.free(); taken?.free(); source.free();
  }

  // An actual WASM edit/restore lifecycle uses the returned owner and drops
  // the previous one each cycle. No timing or linear-memory-shrink assertion.
  let kernel = new BrepKernel();
  let solid = kernel.makeBox(2, 3, 4);
  const canonical = bytes(kernel, [solid]);
  const observations = [];
  try {
    for (let cycle = 0; cycle < 20; cycle++) {
      const checkpoint = kernel.checkpoint();
      for (let tick = 0; tick < 25; tick++) {
        kernel.makeBox(1, 1, 1);
        kernel.restore(checkpoint);
      }
      const before = stats(kernel);
      const transfer = kernel.copyDocument([solid], [], []);
      const roots = Array.from(transfer.solids);
      const next = transfer.takeKernel();
      transfer.free();
      kernel.free(); kernel = next; solid = roots[0];
      const after = stats(kernel);
      assert.deepEqual(bytes(kernel, [solid]), canonical);
      assert.equal(after.allocatedSlots, 64);
      assert.equal(after.retiredSlots, 0);
      assert.ok(before.allocatedSlots <= 1728);
      observations.push({ cycle, before, after });
    }
    onObservation({ cycles: 20, ticksPerCycle: 25, observations });
  } finally { kernel.free(); }
  console.log('ok - selected document copy: fresh owner, exact roots, refusals, checkpoints and 500-tick lifecycle');
}
