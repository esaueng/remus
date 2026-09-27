#!/usr/bin/env node
/**
 * Packaged-runtime witnesses for journaled circular patterns (B18).
 *
 * This module runs inside the real packaged WASM runtime (the
 * `remus_wasm_node.cjs` pair built by `cargo xtask wasm-build`), not the
 * native `cargo test -p remus-wasm` contract suite. It exercises the current
 * public journaled circular-pattern API (`circularPatternJournaled`,
 * `executeBatch`/`executeBatchV2`) and the standard query/measurement
 * bindings.
 *
 * Fixtures (unit scale, origin placement — the full scale/curved/cavity
 * matrix lives natively in
 * `crates/operations/tests/regress_circular_pattern_evolution_fev.rs`):
 * - origin 10-box (volume 1000) patterned count 2 about +Z: the 180-degree
 *   copy touches the source along the origin Z edge and is supported.
 * - offset 10-box translated +30 in X (volume 1000) patterned count 3 about
 *   +Z: three disjoint instances 120 degrees apart.
 *
 * Independent oracles (wrapper agreement alone is never the oracle):
 * - per-instance closed-form volume 1000 and `validateSolid` == 0
 * - all faces `plane`, all edges `LINE`, per-solid census 6/12/8
 * - rotated vertex positions: the 180-degree copy maps (x,y,z)->(-x,-y,z);
 *   the 120/240-degree copies map (x,y,z) by the Z rotation matrix
 * - total F/E/V census 12/24/16 (count 2) and 18/36/24 (count 3) with every
 *   `resolveOperationOutput` bound, construction provenance, distinct
 *   handles of the requested kind
 * - original-versus-generated attribution: indices 0..N-1 resolve to the
 *   source solid's live faces/edges/vertices, the remainder to the copies
 * - journal entry kind `circular_pattern`, type `evolution`, origin
 *   `construction`, event count == F+E+V total
 * - input preservation after every refusal (volumes, census, journal,
 *   serialized bytes, resolvable handles)
 *
 * Documented contracts (preserved here, not redefined):
 * - minimum count 2: count 0 and 1 refuse with "at least 2 copies"
 * - origin-based axis-direction: the axis is the line through the origin
 *   along the direction; a zero direction refuses, a non-unit direction
 *   behaves as its normalization
 * - material overlap refuses with a typed overlap error; the session is
 *   preserved
 * - foreign (deleted) handles refuse without publishing history
 *
 * Direct and both batch paths are exercised: `circularPatternJournaled`
 * direct, `executeBatch` (v1 string errors) and `executeBatchV2` (v2
 * structured errors with codes).
 */

import assert from 'node:assert/strict';

const DEFLECTION = 0.1;
const POS_TOL = 1e-7;
const REL = 1e-9;

const assertClose = (actual, expected, label, rel = REL) => {
  assert.equal(typeof actual, 'number', `${label}: must be a number`);
  assert.ok(Number.isFinite(actual), `${label}: must be finite, got ${actual}`);
  const denom = Math.max(Math.abs(expected), 1e-300);
  const relErr = Math.abs(actual - expected) / denom;
  assert.ok(
    relErr <= rel,
    `${label}: expected ${expected}, got ${actual} (rel ${relErr} > ${rel})`,
  );
};

const translationMatrix = (tx, ty, tz) => [
  1, 0, 0, tx,
  0, 1, 0, ty,
  0, 0, 1, tz,
  0, 0, 0, 1,
];

const rotateZ = (x, y, angle) => {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [x * c - y * s, x * s + y * c];
};

const vertexKey = (p) => p.map((v) => v.toFixed(9)).join(',');

const collectVertexPositions = (kernel, solid) =>
  Array.from(kernel.getSolidVertices(solid)).map((v) => Array.from(kernel.getVertexPosition(v)));

const assertRotatedCopy = (kernel, sourceSolid, copySolid, angle, label) => {
  const sourcePositions = collectVertexPositions(kernel, sourceSolid);
  const copyPositions = collectVertexPositions(kernel, copySolid);
  assert.equal(copyPositions.length, sourcePositions.length, `${label}: vertex count`);
  const expected = new Set(
    sourcePositions.map(([x, y, z]) => {
      const [rx, ry] = rotateZ(x, y, angle);
      return vertexKey([rx, ry, z]);
    }),
  );
  for (const pos of copyPositions) {
    const key = vertexKey(pos);
    // Allow floating-point rotation error: compare against the expected set
    // with a positional tolerance rather than exact string equality.
    let matched = expected.has(key);
    if (!matched) {
      for (const exp of expected) {
        const [ex, ey, ez] = exp.split(',').map(Number);
        const dist = Math.hypot(pos[0] - ex, pos[1] - ey, pos[2] - ez);
        if (dist <= POS_TOL) {
          matched = true;
          break;
        }
      }
    }
    assert.ok(matched, `${label}: copy vertex ${JSON.stringify(pos)} is not a Z-rotation of the source`);
  }
};

const parseDirectPattern = (json) => {
  const payload = JSON.parse(json);
  assert.equal(typeof payload.compound, 'number', `pattern payload needs compound, got ${json}`);
  assert.equal(typeof payload.op, 'number', `pattern payload needs op, got ${json}`);
  return payload;
};

const batchOkV1 = (response, index, label) => {
  const entry = response[index];
  assert.ok(
    entry && 'ok' in entry,
    `${label}: expected ok at index ${index}, got ${JSON.stringify(entry)}`,
  );
  return entry.ok;
};

const batchErrV1 = (response, index, label) => {
  const entry = response[index];
  assert.ok(
    entry && 'error' in entry,
    `${label}: expected error at index ${index}, got ${JSON.stringify(entry)}`,
  );
  assert.equal(typeof entry.error, 'string', `${label}: v1 error must be a string`);
  return entry.error;
};

const batchOkV2 = (response, index, label) => {
  const entry = response[index];
  assert.ok(
    entry && 'ok' in entry,
    `${label}: expected ok at index ${index}, got ${JSON.stringify(entry)}`,
  );
  return entry.ok;
};

const batchErrV2 = (response, index, label) => {
  const entry = response[index];
  assert.ok(
    entry && 'error' in entry,
    `${label}: expected error at index ${index}, got ${JSON.stringify(entry)}`,
  );
  assert.equal(typeof entry.error, 'object', `${label}: v2 error must be structured`);
  assert.equal(typeof entry.error.code, 'string', `${label}: v2 error needs code`);
  assert.equal(typeof entry.error.message, 'string', `${label}: v2 error needs message`);
  return entry.error;
};

// One route = one creation path (direct, batch v1, batch v2). Geometry is
// always verified through the direct query getters on the same session, so
// the queries are an independent oracle rather than wrapper agreement.
const ROUTES = ['direct', 'batchV1', 'batchV2'];

const createPattern = (kernel, route, solid, axis, count) => {
  if (route === 'direct') {
    return parseDirectPattern(kernel.circularPatternJournaled(solid, axis[0], axis[1], axis[2], count));
  }
  const program = JSON.stringify([
    { op: 'circularPatternJournaled', args: { solid, axis, count } },
  ]);
  if (route === 'batchV1') {
    const response = JSON.parse(kernel.executeBatch(program));
    return batchOkV1(response, 0, `circular pattern ${route}`);
  }
  const response = JSON.parse(kernel.executeBatchV2(program));
  return batchOkV2(response, 0, `circular pattern ${route}`);
};

const resolveOutput = (kernel, route, op, kind, index) => {
  if (route === 'direct') {
    return JSON.parse(kernel.resolveOperationOutput(op, kind, index));
  }
  const program = JSON.stringify([
    { op: 'resolveOperationOutput', args: { op, kind, index } },
  ]);
  if (route === 'batchV1') {
    return batchOkV1(JSON.parse(kernel.executeBatch(program)), 0, `resolve ${kind}[${index}] ${route}`);
  }
  return batchOkV2(JSON.parse(kernel.executeBatchV2(program)), 0, `resolve ${kind}[${index}] ${route}`);
};

const journalEntries = (kernel, route) => {
  if (route === 'direct') {
    return JSON.parse(kernel.journalSummary());
  }
  const program = JSON.stringify([{ op: 'journalSummary', args: {} }]);
  if (route === 'batchV1') {
    return batchOkV1(JSON.parse(kernel.executeBatch(program)), 0, `journal ${route}`);
  }
  return batchOkV2(JSON.parse(kernel.executeBatchV2(program)), 0, `journal ${route}`);
};

const assertTotalHistory = (kernel, route, op, sourceSets, compound, count, label) => {
  const members = Array.from(kernel.getCompoundSolids(compound));
  assert.equal(members.length, count, `${label} ${route}: compound has ${count} solids`);
  const expectations = [
    ['face', 6 * count, 'getSolidFaces'],
    ['edge', 12 * count, 'getSolidEdges'],
    ['vertex', 8 * count, 'getSolidVertices'],
  ];
  for (const [kind, expected, query] of expectations) {
    const seen = new Set();
    // Every result entity resolves bound with construction provenance.
    for (let index = 0; index < expected; index += 1) {
      const resolution = resolveOutput(kernel, route, op, kind, index);
      assert.equal(resolution.status, 'bound', `${label} ${route}: ${kind}[${index}] bound, got ${JSON.stringify(resolution)}`);
      assert.equal(resolution.provenance, 'construction', `${label} ${route}: ${kind}[${index}] construction`);
      assert.equal(resolution.entities.length, 1, `${label} ${route}: ${kind}[${index}] one entity`);
      assert.equal(resolution.entities[0].kind, kind, `${label} ${route}: ${kind}[${index}] kind`);
      const handle = resolution.entities[0].handle;
      assert.ok(!seen.has(handle), `${label} ${route}: ${kind} handles distinct`);
      seen.add(handle);
    }
    // The resolved handles cover exactly the live compound entities.
    const live = new Set();
    for (const solid of members) {
      for (const handle of Array.from(kernel[query](solid))) {
        live.add(handle);
      }
    }
    assert.deepEqual(seen, live, `${label} ${route}: ${kind} resolution covers the compound`);
  }
  // Original-versus-generated attribution: the first block of each kind
  // resolves to exactly the source solid's pre-pattern handle set (as a set;
  // output indices follow explorer order, not sorted handles), and the
  // remainder belong to the copies and are absent from the source sets.
  const perKindSource = { face: sourceSets.faces, edge: sourceSets.edges, vertex: sourceSets.vertices };
  for (const kind of ['face', 'edge', 'vertex']) {
    const sourceCount = perKindSource[kind].size;
    const firstBlock = new Set();
    for (let i = 0; i < sourceCount; i += 1) {
      const resolution = resolveOutput(kernel, route, op, kind, i);
      firstBlock.add(resolution.entities[0].handle);
    }
    assert.deepEqual(firstBlock, perKindSource[kind], `${label} ${route}: ${kind} outputs 0..N-1 are the original instance`);
    for (let i = sourceCount; i < sourceCount * count; i += 1) {
      const resolution = resolveOutput(kernel, route, op, kind, i);
      assert.ok(
        !perKindSource[kind].has(resolution.entities[0].handle),
        `${label} ${route}: ${kind}[${i}] is generated, not the original`,
      );
    }
  }
};

const assertJournalEntry = (kernel, route, op, totalEvents, label) => {
  const entries = journalEntries(kernel, route);
  const entry = entries.find((e) => e.op === op);
  assert.ok(entry, `${label} ${route}: journal has entry for op ${op}`);
  assert.equal(entry.kind, 'circular_pattern', `${label} ${route}: kind`);
  assert.equal(entry.type, 'evolution', `${label} ${route}: type`);
  assert.equal(entry.detail.origin, 'construction', `${label} ${route}: origin`);
  assert.equal(entry.detail.events, totalEvents, `${label} ${route}: events == F+E+V`);
};

const assertSolidIsUnitBox = (kernel, solid, label) => {
  assertClose(kernel.volume(solid, DEFLECTION), 1000, `${label}: volume`);
  assert.equal(kernel.validateSolid(solid), 0, `${label}: validates`);
  assert.equal(Array.from(kernel.getSolidFaces(solid)).length, 6, `${label}: 6 faces`);
  assert.equal(Array.from(kernel.getSolidEdges(solid)).length, 12, `${label}: 12 edges`);
  assert.equal(Array.from(kernel.getSolidVertices(solid)).length, 8, `${label}: 8 verts`);
  for (const face of Array.from(kernel.getSolidFaces(solid))) {
    assert.equal(kernel.getSurfaceType(face), 'plane', `${label}: face ${face} plane`);
  }
  for (const edge of Array.from(kernel.getSolidEdges(solid))) {
    assert.equal(kernel.getEdgeCurveType(edge), 'LINE', `${label}: edge ${edge} LINE`);
  }
};

export function runCircularPatternPackaged({ BrepKernel }) {
  // 1. Origin 10-box count 2 about +Z through all three creation paths.
  //    Independent geometry: 180-degree rotation, per-instance 1000 volume,
  //    planar census, total 12/24/16 history with original attribution.
  for (const route of ROUTES) {
    const kernel = new BrepKernel();
    try {
      let source;
      if (route === 'direct') {
        source = kernel.makeBox(10, 10, 10);
      } else {
        const program = JSON.stringify([{ op: 'makeBox', args: { width: 10, height: 10, depth: 10 } }]);
        const response = route === 'batchV1'
          ? JSON.parse(kernel.executeBatch(program))
          : JSON.parse(kernel.executeBatchV2(program));
        source = route === 'batchV1'
          ? batchOkV1(response, 0, 'makeBox origin')
          : batchOkV2(response, 0, 'makeBox origin');
      }
      const sourceSets = {
        faces: new Set(Array.from(kernel.getSolidFaces(source))),
        edges: new Set(Array.from(kernel.getSolidEdges(source))),
        vertices: new Set(Array.from(kernel.getSolidVertices(source))),
      };
      const { compound, op } = createPattern(kernel, route, source, [0, 0, 1], 2);
      const members = Array.from(kernel.getCompoundSolids(compound));
      assert.equal(members.length, 2, `origin count2 ${route}: 2 solids`);
      assert.equal(members[0], source, `origin count2 ${route}: source is the first instance`);
      for (const [i, solid] of members.entries()) {
        assertSolidIsUnitBox(kernel, solid, `origin count2 ${route} solid[${i}]`);
      }
      assertTotalHistory(kernel, route, op, sourceSets, compound, 2, 'origin count2');
      assertJournalEntry(kernel, route, op, 12 + 24 + 16, 'origin count2');
      // 180-degree rotation about Z through the origin.
      assertRotatedCopy(kernel, members[0], members[1], Math.PI, `origin count2 ${route}`);
      // Non-unit axis along the same line behaves as its normalization.
      const normKernel = new BrepKernel();
      try {
        const normSource = normKernel.makeBox(10, 10, 10);
        const normed = parseDirectPattern(normKernel.circularPatternJournaled(normSource, 0, 0, 5, 2));
        const normMembers = Array.from(normKernel.getCompoundSolids(normed.compound));
        assertRotatedCopy(normKernel, normMembers[0], normMembers[1], Math.PI, 'non-unit axis');
        assertClose(normKernel.volume(normMembers[1], DEFLECTION), 1000, 'non-unit axis volume');
      } finally {
        normKernel.free();
      }
      console.log(`ok - circular packaged origin count2, ${route}: 180-degree copy, 12/24/16 construction history`);
    } finally {
      kernel.free();
    }
  }

  // 2. Offset 10-box (+30 X) count 3 about +Z through all three paths.
  //    Three disjoint instances 120 degrees apart; rotation verified per copy.
  for (const route of ROUTES) {
    const kernel = new BrepKernel();
    try {
      let source;
      if (route === 'direct') {
        source = kernel.makeBox(10, 10, 10);
        kernel.transformSolid(source, new Float64Array(translationMatrix(30, 0, 0)));
      } else {
        const program = JSON.stringify([
          { op: 'makeBox', args: { width: 10, height: 10, depth: 10 } },
          { op: 'transform', args: { solid: 0, matrix: translationMatrix(30, 0, 0) } },
        ]);
        const response = route === 'batchV1'
          ? JSON.parse(kernel.executeBatch(program))
          : JSON.parse(kernel.executeBatchV2(program));
        if (route === 'batchV1') {
          batchOkV1(response, 0, 'makeBox offset');
          batchOkV1(response, 1, 'translate offset');
        } else {
          batchOkV2(response, 0, 'makeBox offset');
          batchOkV2(response, 1, 'translate offset');
        }
        source = 0;
      }
      assertClose(kernel.volume(source, DEFLECTION), 1000, `offset source ${route}: volume`);
      const sourceSets = {
        faces: new Set(Array.from(kernel.getSolidFaces(source))),
        edges: new Set(Array.from(kernel.getSolidEdges(source))),
        vertices: new Set(Array.from(kernel.getSolidVertices(source))),
      };
      const { compound, op } = createPattern(kernel, route, source, [0, 0, 1], 3);
      const members = Array.from(kernel.getCompoundSolids(compound));
      assert.equal(members.length, 3, `offset count3 ${route}: 3 solids`);
      assert.equal(members[0], source, `offset count3 ${route}: source first`);
      for (const [i, solid] of members.entries()) {
        assertSolidIsUnitBox(kernel, solid, `offset count3 ${route} solid[${i}]`);
      }
      assertTotalHistory(kernel, route, op, sourceSets, compound, 3, 'offset count3');
      assertJournalEntry(kernel, route, op, 18 + 36 + 24, 'offset count3');
      const step = (2 * Math.PI) / 3;
      assertRotatedCopy(kernel, members[0], members[1], step, `offset count3 ${route} copy1`);
      assertRotatedCopy(kernel, members[0], members[2], 2 * step, `offset count3 ${route} copy2`);
      // Disjoint copies share no vertex positions (contrast the origin case,
      // which shares its Z edge with the source).
      const buckets = members.map((solid) =>
        new Set(collectVertexPositions(kernel, solid).map(vertexKey)),
      );
      for (const [a, b] of [[0, 1], [0, 2], [1, 2]]) {
        const overlap = Array.from(buckets[a]).filter((key) => buckets[b].has(key));
        assert.equal(overlap.length, 0, `offset count3 ${route}: solids ${a}/${b} disjoint`);
      }
      console.log(`ok - circular packaged offset count3, ${route}: 120-degree copies, 18/36/24 construction history`);
    } finally {
      kernel.free();
    }
  }

  // 3. Typed refusals preserve the session on every path: minimum count,
  //    origin-based axis, material overlap, and foreign handles.
  for (const route of ROUTES) {
    const kernel = new BrepKernel();
    try {
      let source;
      if (route === 'direct') {
        source = kernel.makeBox(10, 10, 10);
      } else {
        const program = JSON.stringify([{ op: 'makeBox', args: { width: 10, height: 10, depth: 10 } }]);
        const response = route === 'batchV1'
          ? JSON.parse(kernel.executeBatch(program))
          : JSON.parse(kernel.executeBatchV2(program));
        source = route === 'batchV1'
          ? batchOkV1(response, 0, 'refusal source')
          : batchOkV2(response, 0, 'refusal source');
      }
      const snapshotJournal = JSON.stringify(journalEntries(kernel, route));
      const snapshotBytes = Buffer.from(kernel.serializeSolids(Uint32Array.of(source)));
      const snapshotCounts = Array.from(kernel.getEntityCounts(source));
      const snapshotVolume = kernel.volume(source, DEFLECTION);

      const expectRefusal = (axis, count, v1Pattern, v2Code, v2Pattern, label) => {
        if (route === 'direct') {
          assert.throws(
            () => kernel.circularPatternJournaled(source, axis[0], axis[1], axis[2], count),
            v1Pattern,
            `${label} direct must refuse`,
          );
        } else if (route === 'batchV1') {
          const response = JSON.parse(
            kernel.executeBatch(JSON.stringify([{ op: 'circularPatternJournaled', args: { solid: source, axis, count } }])),
          );
          const message = batchErrV1(response, 0, `${label} batchV1`);
          assert.match(message, v1Pattern, `${label} batchV1 message`);
        } else {
          const response = JSON.parse(
            kernel.executeBatchV2(JSON.stringify([{ op: 'circularPatternJournaled', args: { solid: source, axis, count } }])),
          );
          const error = batchErrV2(response, 0, `${label} batchV2`);
          assert.equal(error.code, v2Code, `${label} batchV2 code`);
          assert.match(error.message, v2Pattern, `${label} batchV2 message`);
        }
        // Every refusal preserves prior session state.
        assert.equal(JSON.stringify(journalEntries(kernel, route)), snapshotJournal, `${label} ${route}: journal preserved`);
        assert.deepEqual(Buffer.from(kernel.serializeSolids(Uint32Array.of(source))), snapshotBytes, `${label} ${route}: geometry preserved`);
        assert.deepEqual(Array.from(kernel.getEntityCounts(source)), snapshotCounts, `${label} ${route}: census preserved`);
        assertClose(kernel.volume(source, DEFLECTION), snapshotVolume, `${label} ${route}: volume preserved`);
        assert.ok(Array.from(kernel.getSolidFaces(source)).length > 0, `${label} ${route}: handles resolve`);
      };

      // Minimum-count contract: the implementation needs at least 2 copies.
      expectRefusal([1, 0, 0], 0, /at least 2 copies/, 'invalid_argument', /at least 2 copies/, 'count 0');
      expectRefusal([1, 0, 0], 1, /at least 2 copies/, 'invalid_argument', /at least 2 copies/, 'count 1');
      // Origin-based-axis contract: a zero direction cannot define the line.
      expectRefusal([0, 0, 0], 2, /zero vector|zero-length|normalize/, 'invalid_argument', /zero vector/, 'zero axis');
      console.log(`ok - circular packaged refusals, ${route}: count/axis contracts preserve the session`);
    } finally {
      kernel.free();
    }
  }

  // 4. Material overlap and foreign handles refuse with typed errors and no
  //    published history, on every path.
  for (const route of ROUTES) {
    const kernel = new BrepKernel();
    try {
      let cylinder;
      if (route === 'direct') {
        cylinder = kernel.makeCylinder(2, 5);
      } else {
        const program = JSON.stringify([{ op: 'makeCylinder', args: { radius: 2, height: 5 } }]);
        const response = route === 'batchV1'
          ? JSON.parse(kernel.executeBatch(program))
          : JSON.parse(kernel.executeBatchV2(program));
        cylinder = route === 'batchV1'
          ? batchOkV1(response, 0, 'overlap cylinder')
          : batchOkV2(response, 0, 'overlap cylinder');
      }
      // A Z-coaxial cylinder rotated about Z sits on itself: instances 0
      // and 1 overlap by the full volume, so the pattern must refuse rather
      // than silently represent intersecting material as a compound.
      const beforeJournal = JSON.stringify(journalEntries(kernel, route));
      const beforeBytes = Buffer.from(kernel.serializeSolids(Uint32Array.of(cylinder)));
      if (route === 'direct') {
        assert.throws(
          () => kernel.circularPatternJournaled(cylinder, 0, 0, 1, 2),
          /overlap/,
          'overlap direct must refuse',
        );
      } else if (route === 'batchV1') {
        const response = JSON.parse(
          kernel.executeBatch(JSON.stringify([{ op: 'circularPatternJournaled', args: { solid: cylinder, axis: [0, 0, 1], count: 2 } }])),
        );
        assert.match(batchErrV1(response, 0, 'overlap batchV1'), /overlap/);
      } else {
        const response = JSON.parse(
          kernel.executeBatchV2(JSON.stringify([{ op: 'circularPatternJournaled', args: { solid: cylinder, axis: [0, 0, 1], count: 2 } }])),
        );
        const error = batchErrV2(response, 0, 'overlap batchV2');
        assert.equal(error.code, 'operation_failed', 'overlap batchV2 code');
        assert.match(error.message, /overlap/);
        assert.equal(error.details.kernelCode, 'pattern_instances_overlap', 'overlap kernel code');
      }
      assert.equal(JSON.stringify(journalEntries(kernel, route)), beforeJournal, `overlap ${route}: no history published`);
      assert.deepEqual(Buffer.from(kernel.serializeSolids(Uint32Array.of(cylinder))), beforeBytes, `overlap ${route}: geometry preserved`);

      // Foreign handle: a deleted solid refuses without publishing history.
      let retired;
      if (route === 'direct') {
        retired = kernel.makeBox(1, 1, 1);
      } else {
        const program = JSON.stringify([{ op: 'makeBox', args: { width: 1, height: 1, depth: 1 } }]);
        const response = route === 'batchV1'
          ? JSON.parse(kernel.executeBatch(program))
          : JSON.parse(kernel.executeBatchV2(program));
        retired = route === 'batchV1'
          ? batchOkV1(response, 0, 'retired box')
          : batchOkV2(response, 0, 'retired box');
      }
      // Resolve the solid handle to an arena id for deletion through the
      // direct delete path (the batch session shares the same arena).
      kernel.deleteSolid(retired);
      const entriesBefore = journalEntries(kernel, route).length;
      if (route === 'direct') {
        assert.throws(
          () => kernel.circularPatternJournaled(retired, 0, 0, 1, 2),
          /invalid solid handle/,
          'foreign direct must refuse',
        );
      } else if (route === 'batchV1') {
        const response = JSON.parse(
          kernel.executeBatch(JSON.stringify([{ op: 'circularPatternJournaled', args: { solid: retired, axis: [0, 0, 1], count: 2 } }])),
        );
        assert.match(batchErrV1(response, 0, 'foreign batchV1'), /invalid solid handle/);
      } else {
        const response = JSON.parse(
          kernel.executeBatchV2(JSON.stringify([{ op: 'circularPatternJournaled', args: { solid: retired, axis: [0, 0, 1], count: 2 } }])),
        );
        const error = batchErrV2(response, 0, 'foreign batchV2');
        assert.equal(error.code, 'invalid_handle', 'foreign batchV2 code');
      }
      assert.equal(journalEntries(kernel, route).length, entriesBefore, `foreign ${route}: no history published`);
      console.log(`ok - circular packaged overlap/foreign refusals, ${route}: typed errors without history`);
    } finally {
      kernel.free();
    }
  }
}

// Allow direct execution: `node scripts/circular-pattern-packaged.mjs`
// loads the freshly built (or committed) kernel from the standard pkg path.
if (process.argv[1] && process.argv[1].endsWith('circular-pattern-packaged.mjs')) {
  const { createRequire } = await import('node:module');
  const { dirname, resolve } = await import('node:path');
  const { fileURLToPath } = await import('node:url');
  const __dirname = dirname(fileURLToPath(import.meta.url));
  const projectRoot = resolve(__dirname, '..');
  const require = createRequire(import.meta.url);
  const { BrepKernel } = require(resolve(projectRoot, 'crates/wasm/pkg/remus_wasm_node.cjs'));
  runCircularPatternPackaged({ BrepKernel });
  console.log('\nAll circular-pattern packaged witnesses passed');
}
