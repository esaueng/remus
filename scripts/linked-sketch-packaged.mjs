/** Real packaged WASM witnesses for construction-linked planar sketch lines. */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

function invoke(k, route, op, args, direct) {
  if (route === 'direct') return k[op](...direct);
  const item = JSON.parse(k[route](JSON.stringify([{ op, args }])))[0];
  if (Object.hasOwn(item, 'error')) throw new Error(typeof item.error === 'string' ? item.error : JSON.stringify(item.error));
  assert.ok(Object.hasOwn(item, 'ok'), JSON.stringify(item));
  return item.ok;
}
const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-8, `${a} != ${b}`);
const json = value => typeof value === 'string' ? JSON.parse(value) : value;

export function runLinkedSketchPackaged({ BrepKernel }) {
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    const k = new BrepKernel();
    try {
      // Establish references using only public APIs: the first journaled face
      // move creates a 10 x 8 x 4 box with full construction F/E/V history.
      const initial = k.makeBox(10, 8, 3);
      const faceWith = (solid, axis) => Array.from(k.getSolidFaces(solid)).find(f => k.getFaceNormal(f)[axis] > 0.99);
      const seeded = JSON.parse(k.moveFacesJournaled(initial, Uint32Array.of(faceWith(initial, 2)), 1));
      const top = faceWith(seeded.solid, 2);
      const right = faceWith(seeded.solid, 0);
      const edge = Array.from(k.getSolidEdges(seeded.solid)).find(e => {
        const [x0, y0, z0, x1, y1, z1] = k.getEdgeVertices(e);
        return Math.abs(z0 - 4) < 1e-9 && Math.abs(z1 - 4) < 1e-9 && Math.abs(y0) < 1e-9 && Math.abs(y1) < 1e-9 && Math.abs(x1 - x0) > 9;
      });
      assert.notEqual(edge, undefined);
      const reference = (kind, handle) => {
        for (let i = 0; i < 32; i++) {
          const r = JSON.parse(k.resolveOperationOutput(seeded.op, kind, i));
          if (r.status === 'bound' && r.entities[0].handle === handle) return k.makeOperationOutputRef(seeded.op, kind, i);
        }
        assert.fail(`missing construction output for ${kind} ${handle}`);
      };
      const faceRef = reference('face', top), edgeRef = reference('edge', edge);
      const endpointHandles = k.getEdgeVertexHandles(edge);
      const startRef = reference('vertex', endpointHandles[0]), endRef = reference('vertex', endpointHandles[1]);
      const call = (op, args, direct) => invoke(k, route, op, args, direct);
      const sketch = call('gcsNew', {}, []);
      const frame = { origin: [0, 0, 4], xAxis: [1, 0, 0], normal: [0, 0, 1] };
      call('gcsAttachToFace', { sketch, ref: faceRef, frame }, [sketch, faceRef, frame]);
      const ext = call('gcsAddExternalLine', { sketch, ref: edgeRef, startRef, endRef }, [sketch, edgeRef, startRef, endRef]);
      const point = call('gcsAddPoint', { sketch, x: 4, y: 3, fixed: false }, [sketch, 4, 3, false]);
      const constraint = { type: 'midpoint', point, line: ext.line };
      call('gcsAddConstraint', { sketch, constraint }, [sketch, JSON.stringify(constraint)]);
      assert.equal(json(call('gcsSolve', { sketch, maxIterations: 100, tolerance: 1e-10 }, [sketch, 100, 1e-10])).converged, true);
      close(call('gcsPointPosition', { sketch, point }, [sketch, point])[0], 5);
      const checkpoint = k.checkpoint();
      k.moveFacesJournaled(seeded.solid, Uint32Array.of(right), 2);
      assert.equal(json(call('gcsSolveDetailed', { sketch, maxIterations: 100, tolerance: 1e-10 }, [sketch, 100, 1e-10])).converged, true);
      close(call('gcsPointPosition', { sketch, point }, [sketch, point])[0], 6);
      const resolved = call('gcsResolveRefs', { sketch }, [sketch]);
      assert.equal(resolved.externalLines, 1); close(resolved.frame.origin[2], 4);
      assert.throws(() => call('gcsSetPoint', { sketch, point: ext.start, x: 99, y: 99 }, [sketch, ext.start, 99, 99]), /linked-sketch:read_only_external_point/);
      const before = call('gcsPointPosition', { sketch, point: ext.end }, [sketch, ext.end]);
      const inferred = k.captureSignatureRef('edge', edge, 1e-7);
      assert.throws(() => call('gcsAddExternalLine', { sketch, ref: inferred, startRef, endRef }, [sketch, inferred, startRef, endRef]), /linked-sketch:(inferred_reference|reference_unbound)/);
      assert.deepEqual(Array.from(call('gcsPointPosition', { sketch, point: ext.end }, [sketch, ext.end])), Array.from(before));
      k.restore(checkpoint);
      assert.equal(call('gcsResolveRefs', { sketch }, [sketch]).externalLines, 1);
      close(call('gcsPointPosition', { sketch, point }, [sketch, point])[0], 5);
      const invalidFrame = { ...frame, xAxis: [0, 0, 0] };
      const unattached = call('gcsNew', {}, []);
      assert.throws(() => call('gcsAttachToFace', { sketch: unattached, ref: faceRef, frame: invalidFrame }, [unattached, faceRef, invalidFrame]), /linked-sketch:invalid_frame/);
      console.log(`ok - linked planar sketch ${route}: construction identity, driven midpoint, edit, refusal, undo`);
    } finally { k.free(); }
  }
}

// Run the same witness against either a fresh build or an npm-installed tarball:
// node scripts/linked-sketch-packaged.mjs /absolute/path/to/remus-wasm-package
if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  assert.ok(process.argv[2], 'usage: node scripts/linked-sketch-packaged.mjs PACKAGE_DIRECTORY');
  const pkg = createRequire(import.meta.url)(resolve(process.argv[2], 'remus_wasm_node.cjs'));
  runLinkedSketchPackaged(pkg);
}
