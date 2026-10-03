import assert from 'node:assert/strict';

export function runSecurityResourceRegressions({ BrepKernel, RemusIo }) {
  const kernel = new BrepKernel();
  const io = new RemusIo();
  try {
    const solid = kernel.makeBox(2, 3, 4);
    const first = kernel.checkpoint();
    for (let i = 1; i < 32; i++) {
      kernel.makeBox(1, 1, 1);
      kernel.checkpoint();
    }
    for (let i = 0; i < 5000; i++) {
      assert.throws(() => kernel.checkpoint(), /at most 32 checkpoints/);
    }
    assert.equal(kernel.checkpointCount(), 32);
    kernel.restore(first);
    assert.ok(Math.abs(kernel.volume(solid, 0.1) - 24) < 0.01);
    const fresh = kernel.checkpoint();
    assert.equal(fresh, 32);
    kernel.discardCheckpoint(first);
    assert.equal(kernel.checkpointCount(), 0);
    assert.throws(() => kernel.restore(fresh), /invalid checkpoint id/);
    for (let i = 0; i < 5000; i++) {
      const cp = kernel.checkpoint();
      kernel.discardCheckpoint(cp);
    }
    assert.throws(() => kernel.restore(first), /invalid checkpoint id/);

    const document = kernel.serializeSolids(new Uint32Array([solid]));
    const step = io.exportStep(document);
    for (const bad of [1e30, Number.MAX_VALUE, Infinity, NaN, 0, -1, 0.5, 1.5]) {
      assert.throws(() => io.importStep(step, bad), /maxInputBytes/);
      assert.throws(() => io.importStep(step, undefined, bad), /maxEntities/);
      assert.throws(() => io.importPly(new Uint8Array(), undefined, bad), /maxEntities/);
    }
    assert.throws(() => io.importStep(step, 256 * 1024 * 1024 + 1), /maxInputBytes/);
    assert.throws(() => io.importStep(step, undefined, 3_000_001), /maxEntities/);
    assert.ok(io.importStep(step).length > 0);
    const emptyPly = new TextEncoder().encode(
      'ply\nformat ascii 1.0\nelement vertex 3000000\nelement face 0\nend_header\n',
    );
    assert.throws(() => io.importPly(emptyPly), /unexpected end of PLY vertex data/);

    const archive = io.export3mf(document, 0.1);
    const central = archive.findIndex(
      (byte, i) =>
        byte === 0x50 && archive[i + 1] === 0x4b && archive[i + 2] === 1 && archive[i + 3] === 2,
    );
    assert.ok(central >= 0);
    const modelName = new TextEncoder().encode('3D/3dmodel.model');
    // A 3MF archive also carries relationship/content entries. Find the model's
    // central record rather than assuming its order in the archive.
    const view = new DataView(archive.buffer, archive.byteOffset, archive.byteLength);
    let modelCentral;
    for (let offset = central; view.getUint32(offset, true) === 0x02014b50;) {
      const nameLength = view.getUint16(offset + 28, true);
      const name = archive.subarray(offset + 46, offset + 46 + nameLength);
      if (name.length === modelName.length && name.every((value, i) => value === modelName[i])) {
        modelCentral = offset;
        break;
      }
      offset +=
        46 + nameLength + view.getUint16(offset + 30, true) + view.getUint16(offset + 32, true);
    }
    assert.notEqual(modelCentral, undefined);
    const local = view.getUint32(modelCentral + 42, true);
    view.setUint32(modelCentral + 24, 256 * 1024 * 1024, true);
    view.setUint32(local + 22, 256 * 1024 * 1024, true);
    assert.ok(io.import3mf(archive).length > 0);
    console.log('ok - packaged checkpoint retention, import budgets, PLY and forged 3MF metadata');
  } finally {
    kernel.free();
    io.free();
  }
}
