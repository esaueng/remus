/** Public WASM witnesses for certified containment, history and sheet retirement. */
import assert from 'node:assert/strict';

function journaledBox(BrepKernel, width, surface) {
  const kernel = new BrepKernel();
  const outer = kernel.makeBox(width, 3, 4);
  const inner = kernel.makeBox(1, 1, 1);
  let result;
  if (surface === 'direct') {
    result = JSON.parse(kernel.fuseJournaled(outer, inner));
  } else {
    const response = JSON.parse(kernel[surface](JSON.stringify([
      { op: 'fuseJournaled', args: { solidA: outer, solidB: inner } },
    ])));
    assert.ok(response[0].ok, JSON.stringify(response));
    result = response[0].ok;
  }
  const reference = kernel.makeOperationOutputRef(result.op, 'face', 0);
  return { kernel, result, reference };
}

export function runKernelCorrectnessPackaged({ BrepKernel }) {
  // All corners and the box centre lie in the torus, but (0.75,0,0) is
  // material in the box and air in the torus. Sampling cannot prove inclusion.
  // The independent volume oracle integrates the circular segment of its hole:
  // q(z)=4-sqrt(9-z*z), x>=0.5, |z|<=0.1.
  const cutVolume = 0.12306989979680594;
  const expectedVolume = {
    fuse: 72 * Math.PI ** 2 + cutVolume,
    intersect: 2.1 - cutVolume,
    cut: cutVolume,
  };
  const translation = (x, y, z) => Float64Array.of(
    1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1,
  );
  for (const placement of [0, 10, 100]) {
    for (const operation of ['fuse', 'intersect', 'cut', 'fuseJournaled', 'intersectJournaled', 'cutJournaled']) {
      const journaled = operation.endsWith('Journaled');
      const kind = operation.replace(/Journaled$/, '');
      for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
        const kernel = new BrepKernel();
        const torus = kernel.makeTorus(4, 3, 16);
        const box = kernel.makeBox(3.5, 3, 0.2);
        kernel.transformSolid(torus, translation(placement, placement, placement));
        kernel.transformSolid(box, translation(placement + 0.5, placement - 1.5, placement - 0.1));
        const point = [placement + 0.75, placement, placement];
        const label = `${surface} ${operation} at ${placement}`;
        assert.equal(kernel.classifyPoint(box, ...point, 1e-7), 'inside', label);
        assert.equal(kernel.classifyPoint(torus, ...point, 1e-7), 'outside', label);
        const before = kernel.serializeSolids(Uint32Array.of(box, torus));
        let result;
        let error;
        if (surface === 'direct') {
          try {
            result = kernel[operation](box, torus);
            if (journaled) result = JSON.parse(result).solid;
          } catch (caught) {
            error = caught;
          }
        } else {
          const response = JSON.parse(kernel[surface](JSON.stringify([
            { op: operation, args: { solidA: box, solidB: torus } },
          ])))[0];
          result = journaled ? response.ok?.solid : response.ok;
          error = response.error;
        }
        if (error !== undefined) {
          const message = surface === 'executeBatchV2' ? error.message : String(error);
          assert.match(message, /exact-only policy/, label);
          if (surface === 'executeBatchV2') {
            assert.equal(error.code, 'operation_failed', label);
            assert.equal(error.category, 'quality_refused', label);
            assert.equal(error.details.kernelCode, 'exact_only_unattainable', label);
          }
        } else {
          assert.ok(Number.isInteger(result) && result >= 0, label);
          assert.equal(kernel.validateSolid(result), 0, label);
          assert.ok(Math.abs(kernel.volume(result, 0.001) - expectedVolume[kind]) < 1e-4,
            `${label}: independent volume oracle`);
          assert.equal(kernel.classifyPoint(result, ...point, 1e-7),
            kind === 'intersect' ? 'outside' : 'inside', label);
        }
        if (!journaled || error !== undefined) {
          assert.deepEqual(kernel.serializeSolids(Uint32Array.of(box, torus)), before,
            `${label}: inputs preserved on success or refusal`);
        }
        kernel.free();
      }
    }
  }

  // A cropped cylinder can share a classifier with its complete carrier.
  // It must retain the half-cylinder material or refuse, rather than expand.
  for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const operation of ['fuse', 'intersect', 'fuseJournaled', 'intersectJournaled']) {
      const journaled = operation.endsWith('Journaled');
      const kernel = new BrepKernel();
      const cylinder = kernel.makeCylinder(2, 2);
      const box = kernel.makeBox(2, 4, 2);
      kernel.transformSolid(box, translation(0, -2, 0));
      const half = kernel.intersect(cylinder, box);
      const copy = kernel.copySolid(half);
      const label = `${surface} ${operation} half-cylinder and copy`;
      assert.equal(kernel.validateSolid(half), 0, label);
      assert.ok(Math.abs(kernel.volume(half, 0.01) - 4 * Math.PI) < 1e-8, label);
      const before = kernel.serializeSolids(Uint32Array.of(half, copy));
      let result;
      let error;
      if (surface === 'direct') {
        try {
          result = kernel[operation](half, copy);
          if (journaled) result = JSON.parse(result).solid;
        } catch (caught) {
          error = caught;
        }
      } else {
        const response = JSON.parse(kernel[surface](JSON.stringify([
          { op: operation, args: { solidA: half, solidB: copy } },
        ])))[0];
        result = journaled ? response.ok?.solid : response.ok;
        error = response.error;
      }
      if (error !== undefined) {
        assert.match(surface === 'executeBatchV2' ? error.message : String(error), /exact-only policy/, label);
        if (surface === 'executeBatchV2') {
          assert.equal(error.code, 'operation_failed', label);
          assert.equal(error.category, 'quality_refused', label);
          assert.equal(error.details.kernelCode, 'exact_only_unattainable', label);
        }
      } else {
        assert.ok(Number.isInteger(result) && result >= 0, label);
        assert.equal(kernel.validateSolid(result), 0, label);
        assert.ok(Math.abs(kernel.volume(result, 0.01) - 4 * Math.PI) < 1e-8, label);
        assert.equal(kernel.classifyPoint(result, 1, 0, 1, 1e-7), 'inside', label);
        assert.equal(kernel.classifyPoint(result, -1, 0, 1, 1e-7), 'outside', label);
      }
      if (!journaled || error !== undefined) {
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(half, copy)), before, label);
      }
      kernel.free();
    }
  }

  // A rounded axis dot product cannot certify the same material region.
  // At R=1e12, these torus axes have dot=1 but R*|axisA cross axisB|≈0.01005.
  // Independent 70-digit evaluation of the serialized carriers gives:
  // pA=(707106781187.2512,0,-707106781185.8439), tube distances A≈0.99507,
  // B≈1.00512; pB=(707106781185.8369,0,-707106781187.2582), A≈1.00506,
  // B≈0.99502. Returning the complete A for both operations adds pA to the
  // intersection and drops pB from the union. High-scale classifyPoint is
  // unsuitable as an oracle for this fixture, so require a typed refusal.
  const rotationY = (angle, pivotZ = 0) => {
    const sine = Math.sin(angle);
    const cosine = Math.cos(angle);
    // T(0,0,pivotZ) * Ry(angle) * T(0,0,-pivotZ), in row-major order.
    return Float64Array.of(
      cosine, 0, sine, -pivotZ * sine,
      0, 1, 0, 0,
      -sine, 0, cosine, pivotZ * (1 - cosine),
      0, 0, 0, 1,
    );
  };
  const tiltedCarriers = [
    {
      name: 'high-scale torus',
      make: kernel => kernel.makeTorus(1e12, 1, 16),
      firstTransform: rotationY(Math.PI / 4),
      secondTransform: rotationY(Math.PI / 4 + 1e-14),
    },
    {
      // The tilt moves the axis by 0.009 near z=9e5. The B-interior point
      // (1.008,0,899999.99999999) has A radial distance 1.008 and B≈0.999.
      name: 'long cylinder',
      make: kernel => kernel.makeCylinder(1, 1e6),
      secondTransform: rotationY(1e-8),
    },
    {
      // Rotate about the shared virtual apex (0,0,2e6). The B-interior point
      // (-0.9679999998305,0,100000.000000009) lies radially outside A's
      // radius≈0.95 there. These are complete valid frusta, not cropped walls.
      name: 'long frustum',
      make: kernel => kernel.makeCone(1, 0.5, 1e6),
      secondTransform: rotationY(1e-8, 2e6),
    },
  ];
  for (const fixture of tiltedCarriers) {
    for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
      for (const operation of ['fuse', 'intersect', 'fuseJournaled', 'intersectJournaled']) {
        // Retain a real earlier journal entry, so refusal must also preserve
        // history when the journaled variant encounters these later mutations.
        const seeded = journaledBox(BrepKernel, 2, surface);
        const { kernel } = seeded;
        const first = fixture.make(kernel);
        const second = fixture.make(kernel);
        if (fixture.firstTransform) kernel.transformSolid(first, fixture.firstTransform);
        kernel.transformSolid(second, fixture.secondTransform);
        const label = `${surface} ${operation} ${fixture.name} tilted axes`;
        assert.equal(kernel.validateSolid(first), 0, label);
        assert.equal(kernel.validateSolid(second), 0, label);
        const before = kernel.serializeSolids(Uint32Array.of(first, second));
        const priorReference = kernel.resolveRef(seeded.reference);
        assert.ok(JSON.parse(new TextDecoder().decode(before)).journal.entries.length > 0, label);
        if (surface === 'direct') {
          assert.throws(() => kernel[operation](first, second), /exact-only policy/, label);
        } else {
          const response = JSON.parse(kernel[surface](JSON.stringify([
            { op: operation, args: { solidA: first, solidB: second } },
          ])))[0];
          assert.ok(Object.hasOwn(response, 'error'), `${label}: must refuse ${JSON.stringify(response)}`);
          const error = response.error;
          assert.match(surface === 'executeBatchV2' ? error.message : String(error), /exact-only policy/, label);
          if (surface === 'executeBatchV2') {
            assert.equal(error.code, 'operation_failed', label);
            assert.equal(error.category, 'quality_refused', label);
            assert.equal(error.details.kernelCode, 'exact_only_unattainable', label);
          }
        }
        assert.deepEqual(kernel.serializeSolids(Uint32Array.of(first, second)), before,
          `${label}: operand geometry and existing history preserved`);
        assert.equal(kernel.resolveRef(seeded.reference), priorReference, label);
        kernel.free();
      }
    }
  }

  // Pointed cones use a single cap and apex instead of a frustum's two caps.
  // Their complete canonical carrier remains an exact identity control.
  for (const placement of [0, 10, 100]) {
    for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
      for (const operation of ['fuse', 'intersect']) {
        const kernel = new BrepKernel();
        const cone = kernel.makeCone(1, 0, 2);
        kernel.transformSolid(cone, translation(placement, placement, placement));
        const copy = kernel.copySolid(cone);
        const label = `${surface} ${operation} pointed-cone identity at ${placement}`;
        let result;
        if (surface === 'direct') {
          result = kernel[operation](cone, copy);
        } else {
          const response = JSON.parse(kernel[surface](JSON.stringify([
            { op: operation, args: { solidA: cone, solidB: copy } },
          ])))[0];
          assert.equal(response.error, undefined, JSON.stringify(response));
          result = response.ok;
        }
        assert.ok(Number.isInteger(result), label);
        assert.equal(kernel.validateSolid(result), 0, label);
        assert.ok(Math.abs(kernel.volume(result, 0.01) - 2 * Math.PI / 3) < 1e-8, label);
        kernel.free();
      }
    }
  }

  for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
    const source = journaledBox(BrepKernel, 7, surface);
    const destination = journaledBox(BrepKernel, 2, surface);
    assert.equal(source.result.op, destination.result.op, 'fixture has colliding source operation IDs');
    const { kernel, result, reference } = destination;
    const before = JSON.parse(kernel.resolveRef(reference));
    assert.equal(before.status, 'bound');
    const originalFaces = Array.from(kernel.getSolidFaces(result.solid));
    const bytes = source.kernel.serializeSolids(Uint32Array.of(source.result.solid));
    const imported = kernel.deserializeSolids(bytes);
    assert.equal(imported.length, 1);
    assert.deepEqual(JSON.parse(kernel.resolveRef(reference)), before,
      `${surface}: importing history preserves the original selection`);
    assert.deepEqual(Array.from(kernel.getSolidFaces(result.solid)), originalFaces);
    const importedFaces = Array.from(kernel.getSolidFaces(imported[0]));
    assert.ok(originalFaces.every(face => !importedFaces.includes(face)));
    assert.ok(Math.abs(kernel.volume(result.solid, 0.01) - 24) < 1e-8);
    assert.ok(Math.abs(kernel.volume(imported[0], 0.01) - 84) < 1e-8);
    // A fresh destination still replays the source operation IDs unchanged.
    const fresh = new BrepKernel();
    fresh.deserializeSolids(bytes);
    assert.equal(JSON.parse(fresh.resolveRef(source.reference)).status, 'bound');
    source.kernel.free();
    kernel.free();
    fresh.free();
  }

  for (const surface of ['direct', 'executeBatch', 'executeBatchV2']) {
    const kernel = new BrepKernel();
    const solid = kernel.makeBox(2, 3, 4);
    const face = kernel.getSolidFaces(solid)[0];
    let sheet;
    if (surface === 'direct') {
      sheet = kernel.makeSheetBody(Uint32Array.of(face));
    } else {
      const response = JSON.parse(kernel[surface](JSON.stringify([
        { op: 'makeSheetBody', args: { faces: [face] } },
      ])));
      assert.ok(Number.isInteger(response[0].ok), JSON.stringify(response));
      sheet = response[0].ok;
    }
    const before = kernel.serializeSheets(Uint32Array.of(sheet));
    assert.equal(kernel.sheetArea(sheet, 0.01), 6);
    kernel.deleteSolid(solid);
    assert.throws(() => kernel.getSolidFaces(solid), /invalid.*solid/i);
    assert.equal(kernel.sheetArea(sheet, 0.01), 6);
    assert.deepEqual(kernel.serializeSheets(Uint32Array.of(sheet)), before);
    const mesh = kernel.tessellateSheet(sheet, 0.05);
    assert.ok(mesh.triangleCount > 0);
    mesh.free();
    kernel.free();
  }
  console.log('ok - certified containment, imported history and shared sheet retirement: direct and both batch versions');
}
