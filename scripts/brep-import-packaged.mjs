import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

/** Verify the generated JS result getters and native-reader fallback boundary. */
export function runBrepImportPackaged({ RemusKernel, RemusIo }) {
  const io = new RemusIo();
  const kernel = new RemusKernel();
  try {
    for (const [name, volume, bounds] of [
      ['freecad-box-v1', 1000],
      ['box-v2', 6000],
      ['box-v3', 6000],
      ['two-holes-v1', 864],
      ['blind-pocket-v1', 852],
      ['concave-hole-v1', 752],
      ['oblique-two-holes-v1', 864],
      ['freecad-placed-box-v1', 1_232_000, [0, -40, 0, 140, 40, 110]],
      ['freecad-cylinder-v1', 68750 * Math.PI, [15, -25, 0, 65, 25, 110]],
      ['tube-v1', 252 * Math.PI],
      ['tube-v2', 252 * Math.PI],
      ['tube-v3', 252 * Math.PI],
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
        if (bounds) {
          kernel.boundingBox(solids[0]).forEach((value, i) => assert.ok(Math.abs(value - bounds[i]) < 1e-8));
        }
        assert.deepEqual(result.solids, io.importBrep(bytes));
        const exchange = io.exportStep(result.solids);
        const restored = kernel.deserializeSolids(io.importStep(exchange));
        assert.equal(restored.length, 1);
        assert.ok(Math.abs(kernel.volume(restored[0], 0.1) - volume) < 1e-8);
      } finally {
        result.free();
      }
    }
    const text = readFileSync(new URL('../crates/io/tests/data/brep/box-v3.brep', import.meta.url), 'utf8');
    const unsupportedSource = text.replace('Surfaces 6\n1 ', 'Surfaces 6\n3 ');
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
    const holed = readFileSync(new URL('../crates/io/tests/data/brep/two-holes-v1.brep', import.meta.url), 'utf8');
    const face = holed.indexOf('\nFa\n');
    const hole = holed.indexOf('+', face);
    assert.ok(face >= 0 && hole > face);
    const invalidHole = holed.slice(0, hole) + '-' + holed.slice(hole + 1);
    assert.throws(() => io.tryImportBrep(new TextEncoder().encode(invalidHole)));
    const touching = readFileSync(new URL('../crates/io/tests/data/brep/touching-hole-zero-v1.brep', import.meta.url));
    assert.throws(() => io.tryImportBrep(touching));
    const cylinder = readFileSync(new URL('../crates/io/tests/data/brep/freecad-cylinder-v1.brep', import.meta.url), 'utf8');
    const badSeam = cylinder.replace('3  1 2CN', '3  1 1CN');
    assert.notEqual(badSeam, cylinder);
    assert.throws(() => io.tryImportBrep(new TextEncoder().encode(badSeam)));
    const rotated = cylinder.replace(/(Locations 3\n1\n)[^\n]+\n[^\n]+\n[^\n]+/, '$10.6 0 0.8 37\n0 1 0 -19\n-0.8 0 0.6 11');
    assert.notEqual(rotated, cylinder);
    const placed = io.tryImportBrep(new TextEncoder().encode(rotated));
    try {
      assert.equal(placed.supported, true);
      const [solid] = kernel.deserializeSolids(placed.solids);
      const bounds = [22, -44, -9, 140, 6, 97];
      kernel.boundingBox(solid).forEach((value, i) => assert.ok(Math.abs(value - bounds[i]) < 1e-8));
      assert.ok(Math.abs(kernel.volume(solid, 0.1) - 68750 * Math.PI) < 1e-8);
    } finally {
      placed.free();
    }
  } finally {
    kernel.free();
    io.free();
  }
}
