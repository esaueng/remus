/** Independent square-frustum material and volume oracles for P-Class 7.1. */
import assert from 'node:assert/strict';

function invoke(kernel, route, args) {
  if (route === 'direct') return kernel.sweepWithOptions(
    args.profile, args.pathEdge, args.contactMode ?? 'fixed', Float64Array.from(args.scaleValues),
    args.segments, args.cornerMode ?? 'smooth',
  );
  const row = JSON.parse(kernel[route](JSON.stringify([{ op: 'sweepWithOptions', args }])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), JSON.stringify(row));
  return row.ok;
}

function closedMesh(kernel, solid, scale) {
  const mesh = JSON.parse(kernel.tessellateSolidGrouped(solid, 0.01 * scale, 0.1));
  assert.ok(mesh.indices.length > 0);
  assert.ok(mesh.positions.every(Number.isFinite));
  const raw = new Map();
  for (let i = 0; i < mesh.indices.length; i += 3) {
    const t = mesh.indices.slice(i, i + 3);
    for (let j = 0; j < 3; j++) {
      const a = t[j], b = t[(j + 1) % 3];
      assert.notEqual(a, b, 'degenerate raw triangle');
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      const value = raw.get(key) ?? [0, 0];
      value[0]++;
      value[1] += a < b ? 1 : -1;
      raw.set(key, value);
    }
  }
  for (const [edge, [count, balance]] of raw) {
    assert.equal(count, 2, `raw boundary/non-manifold edge ${edge}`);
    assert.equal(balance, 0, `same-sense raw mesh edge ${edge}`);
  }
  const remap = [], vertices = new Map();
  for (let i = 0; i < mesh.positions.length; i += 3) {
    const key = mesh.positions.slice(i, i + 3).map(value => Math.round(value / scale * 1e7)).join(',');
    if (!vertices.has(key)) vertices.set(key, vertices.size);
    remap.push(vertices.get(key));
  }
  const uses = new Map();
  for (let i = 0; i < mesh.indices.length; i += 3) {
    const t = mesh.indices.slice(i, i + 3).map(index => remap[index]);
    for (let j = 0; j < 3; j++) {
      const a = t[j], b = t[(j + 1) % 3];
      assert.notEqual(a, b, 'degenerate welded triangle');
      const key = a < b ? `${a},${b}` : `${b},${a}`;
      const value = uses.get(key) ?? [0, 0];
      value[0]++;
      value[1] += a < b ? 1 : -1;
      uses.set(key, value);
    }
  }
  for (const [edge, [count, balance]] of uses) {
    assert.equal(count, 2, `boundary/non-manifold edge ${edge}`);
    assert.equal(balance, 0, `same-sense mesh edge ${edge}`);
  }
}

export function runTaperedHollowSweepPackaged({ BrepKernel }) {
  let cells = 0;
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const [scale, placed] of [0.001, 1, 1000].flatMap(scale => [false, true].map(placed => [scale, placed]))) {
      for (const scaleValues of [[0, 1, 1, 2], [0, 2, 1, 0.5], [0, 1, 0.5, 2, 1, 1],
        [0, 1 + 5e-8, 1, 1 + 5e-8]]) {
        const kernel = new BrepKernel();
        try {
          const outer = kernel.makePolygonWire(Float64Array.from(
            [-2, -2, 0, 2, -2, 0, 2, 2, 0, -2, 2, 0].map(value => value * scale),
          ));
          const inner = kernel.makePolygonWire(Float64Array.from(
            [-1, -1, 0, -1, 1, 0, 1, 1, 0, 1, -1, 0].map(value => value * scale),
          ));
          const profile = kernel.makeFaceFromWires(outer, Uint32Array.of(inner));
          const origin = placed ? [3 * scale, -5 * scale, 7 * scale] : [0, 0, 0];
          const pathEdge = kernel.makeLineEdge(...origin,
            origin[0], origin[1], origin[2] + 6 * scale);
          const args = { profile, pathEdge, contactMode: 'fixed', scaleValues, segments: 4 };
          const solid = invoke(kernel, route, args);
          let integral = 0;
          for (let i = 0; i + 3 < scaleValues.length; i += 2) {
            const [t0, s0, t1, s1] = scaleValues.slice(i, i + 4);
            integral += (t1 - t0) * (s0 * s0 + s0 * s1 + s1 * s1) / 3;
          }
          const expected = 12 * 6 * integral;
          const volume = kernel.volume(solid, scale * 0.01) / scale ** 3;
          assert.ok(Math.abs(volume - expected) < expected * 1e-8,
            `${route} scale=${scale} placed=${placed}: volume ${volume} vs ${expected}`);
          assert.equal(kernel.validateSolid(solid), 0);
          for (const t of [0.125, 0.375, 0.625, 0.875]) {
            let sectionScale;
            for (let i = 0; i + 3 < scaleValues.length; i += 2) {
              const [t0, s0, t1, s1] = scaleValues.slice(i, i + 4);
              if (t >= t0 && t <= t1) sectionScale = s0 + (s1 - s0) * (t - t0) / (t1 - t0);
            }
            for (const [x, expectedState] of [[0.5, 'outside'], [1.5, 'inside'], [2.5, 'outside']]) {
              assert.equal(kernel.classifyPoint(solid, origin[0] + x * sectionScale * scale,
                origin[1], origin[2] + 6 * scale * t, 1e-7), expectedState);
            }
          }
          closedMesh(kernel, solid, scale);
          const state = () => ({
            source: Array.from(kernel.serializeSolids(Uint32Array.of(solid))),
            history: kernel.journalSummary(),
            memory: JSON.parse(kernel.checkpointMemoryStats()),
          });
          for (const malformed of [[0, 1], [0, 1, 1], [0, 1, 0, 2], [1, 1, 0, 2], [0, 1, 1, 0],
            [-1e308, 1, 1e308, 2]]) {
            const before = state();
            if (route === 'direct') {
              assert.throws(() => invoke(kernel, route, { ...args, scaleValues: malformed }), /scaleValues/);
            } else {
              const row = JSON.parse(kernel[route](JSON.stringify([
                { op: 'sweepWithOptions', args: { ...args, scaleValues: malformed } },
              ])))[0];
              assert.ok(Object.hasOwn(row, 'error'), JSON.stringify(row));
              if (route === 'executeBatchV2') {
                assert.equal(row.error.code, 'invalid_argument');
                assert.equal(row.error.category, 'invalid_input');
              }
            }
            assert.deepEqual(state(), before);
          }
          cells++;
        } finally { kernel.free(); }
      }
    }
  }
  console.log(`ok - tapered hollow sweeps: ${cells} direct/batch positive cells and malformed-law rollback`);
  runNativeDomainSweeps(BrepKernel);
  runNativeDomainGuidesAndRefusals(BrepKernel);
  runNativeDomainCorners(BrepKernel);

}


const nativeDomains = [
  [0, 1, 4], [10, 20, 4], [-20, -10, 4], [-2, 3, 4], [0, 2, 4], [-1, 1, 4],
  [1e16, 1e16 + 16, 16], [1e16, 1e16 + 64, 64], [0, 1e308, 16],
];
const growingLaw = { values: [0, 1, 1, 2], section: q => 1 + q, integral: 7 / 3 };

function nativeFixture(kernel, domain, scale = 1, placed = false, hollow = true) {
  const square = half => Float64Array.from([-half, -half, 0, half, -half, 0,
    half, half, 0, -half, half, 0].map(value => value * scale));
  const outer = kernel.makePolygonWire(square(2));
  const profile = hollow
    ? kernel.makeFaceFromWires(outer, Uint32Array.of(kernel.makePolygonWire(square(1))))
    : kernel.makeFaceFromWire(outer);
  const origin = placed ? [3 * scale, -5 * scale, 7 * scale] : [0, 0, 0];
  const cps = [...origin, origin[0], origin[1], origin[2] + 6 * scale];
  const pathEdge = nativeEdge(kernel, domain, cps);
  return { profile, pathEdge, origin, cps };
}

function nativeEdge(kernel, [a, b], cps) {
  const edge = kernel.makeNurbsEdge(...cps.slice(0, 3), ...cps.slice(-3), 1,
    Float64Array.of(a, a, b, b), Float64Array.from(cps), Float64Array.of(1, 1));
  assert.deepEqual(JSON.parse(kernel.getNurbsCurveData(edge)).domain, [a, b]);
  return edge;
}

function physicalFrustum(kernel, solid, origin, scale, hollow, law) {
  const expected = (hollow ? 12 : 16) * 6 * law.integral;
  const volume = kernel.volume(solid, scale * 0.01) / scale ** 3;
  assert.ok(Math.abs(volume - expected) < expected * 1e-8, `${volume} vs ${expected}`);
  assert.equal(kernel.validateSolid(solid), 0);
  const points = Array.from(kernel.getSolidVertices(solid), vertex =>
    Array.from(kernel.getVertexPosition(vertex), (value, i) => value - origin[i]));
  assert.ok(points.every(p => p[2] >= -scale * 1e-8 && p[2] <= scale * (6 + 1e-8)));
  for (const q of [0, 0.25, 0.5, 0.75, 1]) {
    const section = points.filter(p => Math.abs(p[2] - 6 * scale * q) < scale * 1e-8);
    const radius = scale * law.section(q);
    const corners = half => section.filter(p => Math.abs(Math.abs(p[0]) - half) < scale * 1e-8
      && Math.abs(Math.abs(p[1]) - half) < scale * 1e-8).length;
    assert.equal(corners(2 * radius), 4, `outer section at physical q=${q}`);
    assert.equal(corners(radius), hollow ? 4 : 0, `inner section at physical q=${q}`);
  }
  for (const q of [0.125, 0.375, 0.625, 0.875]) {
    for (const [x, state] of [[0.5, hollow ? 'outside' : 'inside'], [1.5, 'inside'], [2.5, 'outside']]) {
      assert.equal(kernel.classifyPoint(solid, origin[0] + x * law.section(q) * scale,
        origin[1], origin[2] + 6 * scale * q, 1e-7), state);
    }
  }
  closedMesh(kernel, solid, scale);
}

function runNativeDomainSweeps(BrepKernel) {
  let cells = 0, lawCells = 0;
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const domain of nativeDomains) for (const scale of [0.001, 1, 1000]) {
      for (const placed of [false, true]) for (const hollow of [false, true]) {
        for (const contactMode of ['fixed', 'rmf', 'constantNormal:0,1,0']) {
          const kernel = new BrepKernel();
          try {
            const fixture = nativeFixture(kernel, domain, scale, placed, hollow);
            const original = kernel.getNurbsCurveData(fixture.pathEdge);
            const solid = invoke(kernel, route, { ...fixture, contactMode,
              scaleValues: growingLaw.values, segments: domain[2] });
            physicalFrustum(kernel, solid, fixture.origin, scale, hollow, growingLaw);
            assert.equal(kernel.getNurbsCurveData(fixture.pathEdge), original);
            cells++;
          } finally { kernel.free(); }
        }
      }
    }
    for (const domain of nativeDomains) for (const law of [
      { values: [0, 2, 1, 0.5], section: q => 2 - 1.5 * q, integral: 1.75 },
      { values: [0, 1, 0.5, 2, 1, 1], section: q => q <= 0.5 ? 1 + 2 * q : 3 - 2 * q, integral: 7 / 3 },
      // Encoded law knots remain normalized progress. These lie beyond q=1,
      // so the law clamps to unit scale even when the PATH domain is [10,20].
      { values: [10, 1, 20, 2], section: () => 1, integral: 1 },
    ]) {
      const kernel = new BrepKernel();
      try {
        const fixture = nativeFixture(kernel, domain, 1, true, true);
        const solid = invoke(kernel, route, { ...fixture, contactMode: 'fixed',
          scaleValues: law.values, segments: domain[2] });
        physicalFrustum(kernel, solid, fixture.origin, 1, true, law);
        lawCells++;
      } finally { kernel.free(); }
    }
  }
  assert.equal(cells, 972);
  assert.equal(lawCells, 81);
  console.log(`ok - native-domain sweep geometry: ${cells} physical frustum cells, ${lawCells} normalized-law controls`);
}

function guided(kernel, route, fixture, domain, auxEdge, auxDomain, auxCps) {
  if (route === 'direct') return kernel.guidedSweep(fixture.profile, 1,
    Float64Array.of(domain[0], domain[0], domain[1], domain[1]), Float64Array.from(fixture.cps), Float64Array.of(1, 1),
    1, Float64Array.of(auxDomain[0], auxDomain[0], auxDomain[1], auxDomain[1]), Float64Array.from(auxCps), Float64Array.of(1, 1));
  const row = JSON.parse(kernel[route](JSON.stringify([{ op: 'guidedSweep', args: {
    face: fixture.profile, spineEdge: fixture.pathEdge, auxEdge,
  } }])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), JSON.stringify(row));
  return row.ok;
}

function runNativeDomainGuidesAndRefusals(BrepKernel) {
  let guides = 0, refusals = 0;
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const domain of nativeDomains) {
      const kernel = new BrepKernel();
      try {
        const fixture = nativeFixture(kernel, domain);
        const auxCps = [3, 0, 0, 3, 3, 6];
        const auxEdge = nativeEdge(kernel, domain, auxCps);
        const solid = guided(kernel, route, fixture, domain, auxEdge, domain, auxCps);
        const points = Array.from(kernel.getSolidVertices(solid), v => Array.from(kernel.getVertexPosition(v)));
        const top = points.filter(p => Math.abs(p[2] - 6) < 1e-8);
        assert.ok(Math.abs(Math.max(...top.map(p => Math.abs(p[0]))) - Math.sqrt(8)) < 1e-8);
        assert.equal(kernel.validateSolid(solid), 0);
        // Endpoint orientation only: no continuous guided-wall error claim.
        guides++;
      } finally { kernel.free(); }
    }
    const kernel = new BrepKernel();
    try {
      const fixture = nativeFixture(kernel, [10, 20, 4]);
      const solid = invoke(kernel, route, { ...fixture, scaleValues: growingLaw.values, segments: 4 });
      const auxCps = [3, 0, 0, 3, 3, 6];
      const auxEdge = nativeEdge(kernel, [0, 1], auxCps);
      // A general coarse chart cannot resolve 64 distinct requested stations.
      const coarse = kernel.makeNurbsEdge(0, 0, 0, 0, 0, 6, 1,
        Float64Array.of(1e16, 1e16, 1e16 + 8, 1e16 + 16, 1e16 + 16),
        Float64Array.of(0, 0, 0, 0, 0, 3, 0, 0, 6), Float64Array.of(1, 1, 1));
      const checkpoint = kernel.checkpoint();
      const state = () => ({ source: Array.from(kernel.serializeSolids(Uint32Array.of(solid))),
        path: kernel.getNurbsCurveData(fixture.pathEdge), history: kernel.journalSummary(),
        memory: JSON.parse(kernel.checkpointMemoryStats()), checkpoints: kernel.checkpointCount() });
      for (const kind of ['guide-domain', 'unresolved-domain', 'resource', 'invalid-law']) {
        const before = state();
        let expectedMessage;
        if (kind === 'guide-domain') expectedMessage = 'sweep_guided: unsupported configuration: main and auxiliary spines must have matching native parameter domains';
        const args = { ...fixture, scaleValues: kind === 'invalid-law' ? [0, 1, 1, 0] : growingLaw.values,
          segments: kind === 'resource' ? 10001 : kind === 'unresolved-domain' ? 64 : 4,
          pathEdge: kind === 'unresolved-domain' ? coarse : fixture.pathEdge };
        if (route === 'direct') {
          assert.throws(() => kind === 'guide-domain'
            ? guided(kernel, route, fixture, [10, 20], auxEdge, [0, 1], auxCps)
            : invoke(kernel, route, args), error => {
              if (expectedMessage) assert.equal(error.message, expectedMessage);
              else assert.match(error.message, kind === 'unresolved-domain' ? /sample stations/ : kind === 'resource' ? /segments/ : /scaleValues/);
              return true;
            });
        } else {
          const operation = kind === 'guide-domain' ? { op: 'guidedSweep', args: {
            face: fixture.profile, spineEdge: fixture.pathEdge, auxEdge,
          } } : { op: 'sweepWithOptions', args };
          const row = JSON.parse(kernel[route](JSON.stringify([operation])))[0];
          assert.ok(Object.hasOwn(row, 'error'), JSON.stringify(row));
          if (route === 'executeBatchV2') {
            assert.equal(row.error.code, kind === 'guide-domain' || kind === 'unresolved-domain' ? 'operation_failed' : 'invalid_argument');
            assert.equal(row.error.category, kind === 'guide-domain' || kind === 'unresolved-domain' ? 'unsupported' : 'invalid_input');
            if (expectedMessage) assert.equal(row.error.message, expectedMessage);
          } else if (expectedMessage) assert.equal(row.error, expectedMessage);
        }
        assert.deepEqual(state(), before);
        refusals++;
      }
      kernel.restore(checkpoint);
      assert.ok(Math.abs(kernel.volume(solid, 0.01) - 168) < 1e-8);
      kernel.discardCheckpoint(checkpoint);
      assert.equal(kernel.checkpointCount(), 0);
    } finally { kernel.free(); }
  }
  assert.equal(guides, 27);
  assert.equal(refusals, 12);
  console.log(`ok - native-domain guides: ${guides} endpoint cells; ${refusals} typed checkpoint-ownership rollback controls`);
}

function runNativeDomainCorners(BrepKernel) {
  let positives = 0, refusals = 0;
  const domains = [[0, 1], [10, 20], [-2, 3], [1e6, 1e6 + 16],
    [1e16, 1e16 + 16], [0, 1e-12], [0, 1e-16]];
  for (const route of ['direct', 'executeBatch', 'executeBatchV2']) {
    for (const [start, end] of domains) {
      for (const cornerMode of end === 1e-16 ? ['miter'] : ['miter', 'round:1']) {
        const kernel = new BrepKernel();
        try {
          const profile = kernel.makeFaceFromWire(kernel.makePolygonWire(Float64Array.from(
            [-0.5, -0.5, 0, 0.5, -0.5, 0, 0.5, 0.5, 0, -0.5, 0.5, 0],
          )));
          const pathEdge = kernel.makeNurbsEdge(0, 0, 0, 5, 0, 5, 1,
            Float64Array.from([start, start, start + (end - start) / 2, end, end]),
            Float64Array.from([0, 0, 0, 0, 0, 5, 5, 0, 5]), Float64Array.from([1, 1, 1]));
          const args = { profile, pathEdge, contactMode: 'rmf', scaleValues: [], segments: 4, cornerMode };
          const shouldRefuse = (start > 1e15 && cornerMode === 'round:1') || end === 1e-16;
          if (shouldRefuse) {
            const survivor = kernel.makeBox(1, 1, 1);
            const checkpoint = kernel.checkpoint();
            const state = () => ({
              survivor: Array.from(kernel.serializeSolids(Uint32Array.of(survivor))),
              path: kernel.getNurbsCurveData(pathEdge), history: kernel.journalSummary(),
              memory: JSON.parse(kernel.checkpointMemoryStats()), checkpoints: kernel.checkpointCount(),
            });
            const before = state();
            if (route === 'direct') {
              assert.throws(() => invoke(kernel, route, args), /native knot chart cannot resolve/);
            } else {
              const row = JSON.parse(kernel[route](JSON.stringify([{ op: 'sweepWithOptions', args }])))[0];
              assert.ok(Object.hasOwn(row, 'error'));
              if (route === 'executeBatchV2') {
                assert.equal(row.error.category, 'unsupported');
                assert.equal(row.error.code, 'operation_failed');
              }
              assert.match(route === 'executeBatchV2' ? row.error.message : row.error, /native knot chart cannot resolve/);
            }
            assert.deepEqual(state(), before, `${route}: corner refusal changed checkpoint-owned state`);
            kernel.restore(checkpoint);
            assert.equal(kernel.volume(survivor, 0.01), 1);
            kernel.discardCheckpoint(checkpoint);
            refusals++;
          } else {
            const solid = invoke(kernel, route, args);
            const volume = kernel.volume(solid, 0.01);
            if (cornerMode === 'miter') assert.ok(Math.abs(volume - 10) < 1e-8,
              `${route} [${start},${end}] ${cornerMode}: volume ${volume} vs 10`);
            else assert.ok(volume < 8 + Math.PI / 2 && volume > (8 + Math.PI / 2) * 0.999,
              `${route} [${start},${end}] ${cornerMode}: volume ${volume}`);
            assert.equal(kernel.validateSolid(solid), 0);
            assert.equal(kernel.classifyPoint(solid, -0.25, 0, 5.2, 1e-7),
              cornerMode === 'miter' ? 'inside' : 'outside');
            assert.equal(kernel.classifyPoint(solid, 0, 0, 2.5, 1e-7), 'inside');
            assert.equal(kernel.classifyPoint(solid, 2.5, 0, 5, 1e-7), 'inside');
            closedMesh(kernel, solid, 1);
            positives++;
          }
        } finally { kernel.free(); }
      }
    }
  }
  assert.equal(positives, 33);
  assert.equal(refusals, 6);
  console.log(`ok - native-domain corner modes: ${positives} physical positives, ${refusals} typed refusals`);
}
