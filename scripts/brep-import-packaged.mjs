import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

/** Verify the generated JS result getters and native-reader fallback boundary. */
export function runBrepImportPackaged({ RemusKernel, RemusIo }) {
  const io = new RemusIo();
  const kernel = new RemusKernel();
  try {
    for (const [name, volume] of [
      ['freecad-box-v1', 1000],
      ['box-v2', 6000],
      ['box-v3', 6000],
    ]) {
      const bytes = readFileSync(new URL(`../crates/io/tests/data/brep/${name}.brep`, import.meta.url));
      const result = io.tryImportBrep(bytes);
      try {
        assert.equal(result.supported, true);
        assert.equal(result.unsupportedReason, undefined);
        assert.ok(result.solids instanceof Uint8Array);
        const solids = kernel.deserializeSolids(result.solids);
        assert.equal(solids.length, 1);
        assert.ok(Math.abs(kernel.volume(solids[0], 0.1) - volume) < 1e-8);
        assert.deepEqual(result.solids, io.importBrep(bytes));
      } finally {
        result.free();
      }
    }
    const text = readFileSync(new URL('../crates/io/tests/data/brep/box-v3.brep', import.meta.url), 'utf8');
    const unsupportedSource = text.replace('Surfaces 6\n1 ', 'Surfaces 6\n2 ');
    assert.notEqual(unsupportedSource, text);
    const unsupportedBytes = new TextEncoder().encode(unsupportedSource);
    const result = io.tryImportBrep(unsupportedBytes);
    try {
      assert.equal(result.supported, false);
      assert.equal(result.solids.length, 0);
      assert.equal(typeof result.unsupportedReason, 'string');
      assert.throws(() => io.importBrep(unsupportedBytes));
    } finally {
      result.free();
    }
    const bytes = new TextEncoder().encode(text);
    assert.throws(() => io.tryImportBrep(bytes, 1));
    assert.throws(() => io.tryImportBrep(bytes, undefined, 10));
    assert.throws(() => io.tryImportBrep(bytes, NaN));
    assert.throws(() => io.tryImportBrep(bytes.subarray(0, Math.floor(bytes.length / 2))));
    assert.throws(() => io.tryImportBrep(Uint8Array.of(255)));
    const invalidPcurve = text.replace('Curve2ds 24\n1 0 0', 'Curve2ds 24\n1 2 0');
    assert.notEqual(invalidPcurve, text);
    assert.throws(() => io.tryImportBrep(new TextEncoder().encode(invalidPcurve)));
    const inverted = text.replace(/\+1 0\s*$/, '-1 0');
    assert.notEqual(inverted, text);
    assert.throws(() => io.tryImportBrep(new TextEncoder().encode(inverted)));
  } finally {
    kernel.free();
    io.free();
  }
}
