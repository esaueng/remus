/** Real packaged WASM witnesses for construction-linked planar sketch lines and circles. */
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
function refusal(k, route, op, args, direct, kernelCode, category) {
  const prefix = `${kernelCode}:`;
  if (route === 'direct') {
    assert.throws(() => k[op](...direct), error => {
      assert.ok(error.message.startsWith(prefix), error.message);
      return true;
    });
    return;
  }
  const item = JSON.parse(k[route](JSON.stringify([{ op, args }])))[0];
  assert.ok(Object.hasOwn(item, 'error'), JSON.stringify(item));
  if (route === 'executeBatch') {
    assert.ok(item.error.startsWith(prefix), item.error);
  } else {
    assert.equal(item.error.details.kernelCode, kernelCode);
    assert.equal(item.error.category, category);
    assert.ok(item.error.message.startsWith(prefix), item.error.message);
  }
  return item.error;
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
      // Create this before any references exist; a later journaled move makes
      // its line signature ambiguous without introducing an untracked edit.
      const duplicate = k.makeBox(10, 8, 3);
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
      const faceRef = reference('face', top), rightRef = reference('face', right), edgeRef = reference('edge', edge);
      const endpointHandles = k.getEdgeVertexHandles(edge);
      const startRef = reference('vertex', endpointHandles[0]), endRef = reference('vertex', endpointHandles[1]);
      const call = (op, args, direct) => invoke(k, route, op, args, direct);
      const refuse = (op, args, direct, code, category) => refusal(k, route, op, args, direct, `linked-sketch:${code}`, category);
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
      refuse('gcsSetPoint', { sketch, point: ext.start, x: 99, y: 99 }, [sketch, ext.start, 99, 99], 'read_only_external_point', 'invalid_input');
      const before = call('gcsPointPosition', { sketch, point: ext.end }, [sketch, ext.end]);
      const inferred = k.captureSignatureRef('edge', edge, 1e-7);
      assert.equal(JSON.parse(k.resolveRef(inferred)).status, 'bound');
      refuse('gcsAddExternalLine', { sketch, ref: inferred, startRef, endRef }, [sketch, inferred, startRef, endRef], 'inferred_reference', 'unsupported');
      assert.deepEqual(Array.from(call('gcsPointPosition', { sketch, point: ext.end }, [sketch, ext.end])), Array.from(before));
      k.restore(checkpoint);
      assert.equal(call('gcsResolveRefs', { sketch }, [sketch]).externalLines, 1);
      close(call('gcsPointPosition', { sketch, point }, [sketch, point])[0], 5);

      // Successful input refresh precedes the detailed solver's free-parameter
      // snapshot. The contradictory constraints have optimum x=5.5, residual
      // 0.5; rollback publishes free x=5 against the current midpoint x=6.
      const fixX = { type: 'fixX', point, value: 5 };
      call('gcsAddConstraint', { sketch, constraint: fixX }, [sketch, JSON.stringify(fixX)]);
      k.moveFacesJournaled(seeded.solid, Uint32Array.of(right), 2);
      const unsatisfied = json(call('gcsSolveDetailed', { sketch, maxIterations: 100, tolerance: 1e-10 }, [sketch, 100, 1e-10]));
      assert.equal(unsatisfied.converged, false); assert.equal(unsatisfied.rolledBack, true);
      assert.equal(unsatisfied.classification, 'unsatisfied');
      close(call('gcsPointPosition', { sketch, point }, [sketch, point])[0], 5);
      const currentStart = call('gcsPointPosition', { sketch, point: ext.start }, [sketch, ext.start]);
      const currentEnd = call('gcsPointPosition', { sketch, point: ext.end }, [sketch, ext.end]);
      close(Math.abs(currentEnd[0] - currentStart[0]), 12);
      close(unsatisfied.maxResidual, 0.5); close(unsatisfied.publishedMaxResidual, 1);
      assert.equal(json(call('gcsSolve', { sketch, maxIterations: 100, tolerance: 1e-10 }, [sketch, 100, 1e-10])).converged, false);
      close(call('gcsPointPosition', { sketch, point }, [sketch, point])[0], 5.5);
      k.restore(checkpoint);

      const invalidFrame = { ...frame, xAxis: [0, 0, 0] };
      const unattached = call('gcsNew', {}, []);
      refuse('gcsAttachToFace', { sketch: unattached, ref: faceRef, frame: invalidFrame }, [unattached, faceRef, invalidFrame], 'invalid_frame', 'invalid_input');
      refuse('gcsAttachToFace', { sketch: unattached, ref: rightRef, frame }, [unattached, rightRef, frame], 'unsupported_attachment', 'unsupported');
      call('gcsAttachToFace', { sketch: unattached, ref: faceRef, frame }, [unattached, faceRef, frame]);
      const missing = k.makeOperationOutputRef(0xffffffff, 'edge', 0);
      const missingError = refuse('gcsAddExternalLine', { sketch, ref: missing, startRef, endRef }, [sketch, missing, startRef, endRef], 'reference_unbound', 'invalid_input');
      if (route === 'executeBatchV2') {
        assert.equal(missingError.details.referenceRole, 'edge');
        assert.equal(missingError.details.resolution.status, 'unknownOperation');
      }
      k.moveFacesJournaled(duplicate, Uint32Array.of(faceWith(duplicate, 2)), 1);
      const ambiguous = k.captureSignatureRef('edge', edge, 1e-7);
      assert.equal(JSON.parse(k.resolveRef(ambiguous)).status, 'ambiguous');
      const ambiguousError = refuse('gcsAddExternalLine', { sketch, ref: ambiguous, startRef, endRef }, [sketch, ambiguous, startRef, endRef], 'reference_unbound', 'invalid_input');
      if (route === 'executeBatchV2') {
        assert.equal(ambiguousError.details.referenceRole, 'edge');
        assert.equal(ambiguousError.details.resolution.status, 'ambiguous');
        assert.ok(ambiguousError.details.resolution.candidates.length >= 2);
      }
      close(call('gcsPointPosition', { sketch, point }, [sketch, point])[0], 5);
      close(call('gcsPointPosition', { sketch, point: ext.end }, [sketch, ext.end])[0], 10);
      console.log(`ok - linked planar sketch ${route}: construction identity, edit/undo, typed refusals, refresh-first rollback`);
    } finally { k.free(); }
  }
  runLinkedSketchCirclesPackaged({ BrepKernel });
}

// Run the same witness against either a fresh build or an npm-installed tarball:
// node scripts/linked-sketch-packaged.mjs /absolute/path/to/remus-wasm-package
if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  assert.ok(process.argv[2], 'usage: node scripts/linked-sketch-packaged.mjs PACKAGE_DIRECTORY');
  const pkg = createRequire(import.meta.url)(resolve(process.argv[2], 'remus_wasm_node.cjs'));
  runLinkedSketchPackaged(pkg);
}

/** Full-circle witnesses over the real direct and both batch API exports. */
export function runLinkedSketchCirclesPackaged({ BrepKernel }) {
  let cells = 0;
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const scale of [0.001, 1, 1000]) for (const placement of [0, 1, 2]) {
      const k = new BrepKernel();
      try {
        const call = (op, args, direct) => invoke(k, route, op, args, direct);
        const radius = circle => call('gcsCircleRadius', { sketch, circle }, [sketch, circle]);
        const near = (a, b) => close(a / scale, b / scale);
        const refuse = (op, args, direct, code, category) => {
          const prefix = `linked-sketch:${code}`;
          if (route === 'direct') {
            assert.throws(() => call(op, args, direct), error => error.message.startsWith(`${prefix}:`));
          } else {
            const row = JSON.parse(k[route](JSON.stringify([{ op, args }])))[0];
            assert.ok(Object.hasOwn(row, 'error'), JSON.stringify(row));
            if (route === 'executeBatchV2') {
              assert.equal(row.error.details.kernelCode, prefix);
              assert.equal(row.error.category, category);
            } else assert.ok(row.error.startsWith(`${prefix}:`), row.error);
          }
        };
        const boxed = k.makeBox(10 * scale, 8 * scale, 4 * scale);
        const tool = k.makeCylinder(2 * scale, 6 * scale);
        k.transformSolid(tool, Float64Array.from([
          1, 0, 0, 5 * scale, 0, 1, 0, 4 * scale, 0, 0, 1, -scale, 0, 0, 0, 1,
        ]));
        const bored = k.cut(boxed, tool);
        let origin = [0, 0, 4 * scale], normal = [0, 0, 1];
        if (placement) {
          const rotated = placement === 2;
          k.transformSolid(bored, Float64Array.from([
            1, 0, 0, 3 * scale, 0, rotated ? 0 : 1, rotated ? -1 : 0, -5 * scale,
            0, rotated ? 1 : 0, rotated ? 0 : 1, 7 * scale, 0, 0, 0, 1,
          ]));
          origin = rotated ? [3 * scale, -9 * scale, 7 * scale] : [3 * scale, -5 * scale, 11 * scale];
          normal = rotated ? [0, -1, 0] : [0, 0, 1];
        }
        const cylinder = solid => Array.from(k.getSolidFaces(solid)).find(f => k.getSurfaceType(f) === 'cylinder');
        const seeded = JSON.parse(k.resizeCylindricalFaceJournaled(bored, cylinder(bored), 2.5 * scale));
        const top = Array.from(k.getSolidFaces(seeded.solid)).find(f => k.getSurfaceType(f) === 'plane'
          && k.getFaceNormal(f).reduce((sum, n, i) => sum + n * normal[i], 0) > 0.99);
        const ring = Array.from(k.getFaceEdges(top)).find(e => k.getEdgeCurveType(e) === 'CIRCLE');
        const line = Array.from(k.getFaceEdges(top)).find(e => k.getEdgeCurveType(e) === 'LINE');
        const reference = (kind, handle) => {
          for (let i = 0; i < 100; i++) {
            const r = JSON.parse(k.resolveOperationOutput(seeded.op, kind, i));
            if (r.status === 'bound' && r.entities[0].handle === handle) return k.makeOperationOutputRef(seeded.op, kind, i);
          }
          assert.fail(`missing circle fixture output ${kind}:${handle}`);
        };
        const source = reference('edge', ring), faceRef = reference('face', top), lineRef = reference('edge', line);
        const sketch = call('gcsNew', {}, []);
        call('gcsAttachToFace', { sketch, ref: faceRef, frame: { origin, xAxis: [1, 0, 0], normal } },
          [sketch, faceRef, { origin, xAxis: [1, 0, 0], normal }]);
        const external = call('gcsAddExternalCircle', { sketch, ref: source }, [sketch, source]);
        assert.equal(json(call('gcsDof', { sketch }, [sketch])).numParams, 0);
        near(radius(external.circle), 2.5 * scale);
        const position = call('gcsPointPosition', { sketch, point: external.center }, [sketch, external.center]);
        near(position[0], 5 * scale); near(position[1], 4 * scale);
        refuse('gcsAddExternalCircle', { sketch, ref: lineRef }, [sketch, lineRef], 'unsupported_curve', 'unsupported');
        refuse('gcsAddExternalCircle', { sketch, ref: faceRef }, [sketch, faceRef], 'invalid_reference_kind', 'invalid_input');
        const inferred = k.captureSignatureRef('edge', ring, 1e-7);
        const inferredResolution = JSON.parse(k.resolveRef(inferred));
        assert.equal(inferredResolution.status, 'bound'); assert.equal(inferredResolution.provenance, 'inferred');
        refuse('gcsAddExternalCircle', { sketch, ref: inferred }, [sketch, inferred], 'inferred_reference', 'unsupported');
        const center = call('gcsAddPoint', { sketch, x: 6 * scale, y: 5 * scale, fixed: false }, [sketch, 6 * scale, 5 * scale, false]);
        const consumer = call('gcsAddCircle', { sketch, center, radius: scale }, [sketch, center, scale]);
        const constraint = value => call('gcsAddConstraint', { sketch, constraint: value }, [sketch, JSON.stringify(value)]);
        const centerConstraint = constraint({ type: 'coincident', a: center, b: external.center });
        const radiusConstraint = constraint({ type: 'equalRadiusCircleCircle', circle1: consumer, circle2: external.circle });
        const solve = op => json(call(op, { sketch, maxIterations: 100, tolerance: 1e-10 * scale }, [sketch, 100, 1e-10 * scale]));
        const solved = solve('gcsSolveDetailed');
        assert.equal(solved.converged, true); assert.equal(solved.numParams, 3); assert.equal(solved.rank, 3); assert.equal(solved.dof, 0);
        near(radius(consumer), 2.5 * scale);
        const cp = k.checkpoint();
        const retired = call('gcsAddExternalCircle', { sketch, ref: source }, [sketch, source]);
        const consumerConflict = constraint({ type: 'circleRadius', circle: consumer, value: 2.5 * scale });
        const grown = JSON.parse(k.resizeCylindricalFaceJournaled(seeded.solid, cylinder(seeded.solid), 3 * scale));
        const unsatisfied = solve('gcsSolveDetailed');
        assert.equal(unsatisfied.converged, false); assert.equal(unsatisfied.rolledBack, true);
        near(radius(external.circle), 3 * scale); near(radius(consumer), 2.5 * scale);
        near(unsatisfied.publishedMaxResidual, 0.5 * scale);
        call('gcsRemoveConstraint', { sketch, constraint: consumerConflict }, [sketch, consumerConflict]);
        assert.equal(solve('gcsSolve').converged, true); near(radius(consumer), 3 * scale);
        const extConflict = constraint({ type: 'circleRadius', circle: external.circle, value: 4 * scale });
        assert.equal(solve('gcsSolve').converged, false); near(radius(external.circle), 3 * scale);
        assert.equal(solve('gcsSolveDetailed').rolledBack, true); near(radius(external.circle), 3 * scale);
        for (const id of [centerConstraint, radiusConstraint, extConflict]) call('gcsRemoveConstraint', { sketch, constraint: id }, [sketch, id]);
        assert.equal(json(call('gcsDof', { sketch }, [sketch])).numParams, 3);
        near(radius(external.circle), 3 * scale);
        const ellipse = call('gcsAddEllipse', { sketch, center: external.center, a: 2 * scale, b: scale, angle: 0 }, [sketch, external.center, 2 * scale, scale, 0]);
        refuse('gcsSetPoint', { sketch, point: external.center, x: 99, y: 99 }, [sketch, external.center, 99, 99], 'read_only_external_point', 'invalid_input');
        refuse('gcsSetEllipse', { sketch, ellipse, x: 99, y: 99, a: 3, b: 2, angle: 0 }, [sketch, ellipse, 99, 99, 3, 2, 0], 'read_only_external_point', 'invalid_input');
        const currentTop = JSON.parse(k.resolveRef(faceRef)).entities[0].handle;
        const moved = JSON.parse(k.moveFacesJournaled(grown.solid, Uint32Array.of(currentTop), scale));
        const resolved = call('gcsResolveRefs', { sketch }, [sketch]);
        assert.equal(resolved.externalCircles, 2);
        for (let i = 0; i < 3; i++) near(resolved.frame.origin[i], origin[i] + normal[i] * scale);
        near(radius(external.circle), 3 * scale);
        near(k.volume(moved.solid, 0.01 * scale) / scale ** 2, (80 - Math.PI * 9) * 5 * scale);
        assert.equal(k.validateSolid(moved.solid), 0);
        k.restore(cp);
        near(radius(external.circle), 2.5 * scale); near(radius(consumer), 2.5 * scale);
        assert.equal(call('gcsResolveRefs', { sketch }, [sketch]).externalCircles, 1);
        assert.throws(() => call('gcsCircleRadius', { sketch, circle: retired.circle }, [sketch, retired.circle]));
        assert.throws(() => call('gcsPointPosition', { sketch, point: retired.center }, [sketch, retired.center]));
        const next = call('gcsAddExternalCircle', { sketch, ref: source }, [sketch, source]);
        assert.ok(next.circle > retired.circle && next.center > retired.center);
        near(radius(next.circle), 2.5 * scale);
        cells++;
      } finally { k.free(); }
    }
  }
  console.log(`ok - linked complete circles: ${cells} direct/batch cells, fixed-radius conflicts, refresh, typed refusals and undo`);
}
