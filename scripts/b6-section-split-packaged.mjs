#!/usr/bin/env node
/**
 * Packaged-runtime witnesses for the B6 plane-section/split family.
 *
 * This module runs inside the real packaged WASM runtime (the `remus_wasm_node.cjs`
 * pair built by `cargo xtask wasm-build`), not the native `cargo test -p remus-wasm`
 * contract suite. It reuses the current public section/split APIs (`section`,
 * `split`, `executeBatch`/`executeBatchV2`) and the standard measurement/query
 * bindings.
 *
 * Fixtures (unit scale, origin placement — the full scale/placement matrix lives
 * natively in `crates/operations/tests/qualify_section_split.rs`):
 * - plain 4-box (volume 64)
 * - hollow box: outer 4-box minus inner 2-box at (1,1,1) (volume 56, 2 shells)
 * - holed plate: 10x10x2 minus 2x2x4 tool at (4,4,-1) (volume 192, 1 shell)
 *
 * Independent oracles (wrapper agreement alone is never the oracle):
 * - hand rectangle-minus-hole areas (16, 12, 96, 20) and half volumes
 *   (32+32, 172+20, 96+96 where used)
 * - hole counts via `getFaceWires` (outer + inners); a lost inner boundary
 *   would read as 16 instead of 12, or 100 instead of 96
 * - plane membership of every section vertex via `getFaceVertices` /
 *   `getVertexPosition`
 * - material side via `classifyPoint` (inside/outside on the correct side)
 * - `validateSolid` == 0, `meshQuality` watertight, all-`plane` census
 * - volume-sum identity for splits, input preservation after every refusal
 *
 * Documented contracts (section vs split differ by design):
 * - `section` miss / vertex-touch => empty face-list success
 * - `section` coincident => one-face outline success
 * - `section` edge-touch => `InvalidInput` ("no closed cross-section")
 * - `split` miss => `InvalidInput` ("entirely on one side")
 * - `split` cavity / inner-wire / edge-contained => `Unsupported` naming the
 *   configuration; every refusal rolls back and preserves handles
 *
 * Direct and batch paths are exercised where supported (`section`/`split` both
 * have batch dispatch; `convexHull`-style direct-only gaps do not apply here).
 */

import assert from 'node:assert/strict';

const DEFLECTION = 0.01;
const TOL = 1e-7;
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

const parseV2 = (json) => JSON.parse(json);

const batchOk = (response, index, label) => {
  const entry = response[index];
  assert.ok(
    entry && 'ok' in entry,
    `${label}: expected ok at index ${index}, got ${JSON.stringify(entry)}`,
  );
  return entry.ok;
};

const batchErr = (response, index, label) => {
  const entry = response[index];
  assert.ok(
    entry && 'error' in entry,
    `${label}: expected error at index ${index}, got ${JSON.stringify(entry)}`,
  );
  return entry.error;
};

// Hollow box via direct bindings: outer 4, inner 2 at (1,1,1).
const makeHollowDirect = (kernel) => {
  const outer = kernel.makeBox(4, 4, 4);
  const inner = kernel.makeBox(2, 2, 2);
  kernel.transformSolid(inner, new Float64Array(translationMatrix(1, 1, 1)));
  const hollow = kernel.cut(outer, inner);
  return { outer, inner, hollow };
};

// Holed plate via direct bindings: 10x10x2 minus 2x2x4 at (4,4,-1).
const makeHoledPlateDirect = (kernel) => {
  const plate = kernel.makeBox(10, 10, 2);
  const tool = kernel.makeBox(2, 2, 4);
  kernel.transformSolid(tool, new Float64Array(translationMatrix(4, 4, -1)));
  const holed = kernel.cut(plate, tool);
  return { plate, tool, holed };
};

const faceHoleCount = (kernel, face) => Array.from(kernel.getFaceWires(face)).length - 1;

const assertSectionOnPlane = (kernel, face, px, py, pz, nx, ny, nz, label) => {
  const vertices = Array.from(kernel.getFaceVertices(face));
  assert.ok(vertices.length >= 3, `${label}: section face must have vertices`);
  const norm = Math.hypot(nx, ny, nz);
  assert.ok(norm > 0, `${label}: plane normal must be non-zero`);
  const d = (nx * px + ny * py + nz * pz) / norm;
  for (const v of vertices) {
    const pos = Array.from(kernel.getVertexPosition(v));
    const dist = Math.abs((nx * pos[0] + ny * pos[1] + nz * pos[2]) / norm - d);
    assert.ok(dist < 1e-6, `${label}: vertex ${v} off plane by ${dist}`);
  }
};

export function runB6SectionSplitPackaged({ BrepKernel }) {
  // 1. Ordinary positive section, direct + batch: 4-box at z=2 => 1 face, area 16.
  {
    const direct = new BrepKernel();
    const box = direct.makeBox(4, 4, 4);
    assertClose(direct.volume(box, DEFLECTION), 64, 'box input volume');
    assert.equal(direct.validateSolid(box), 0, 'box input valid');
    const faces = Array.from(direct.section(box, 0, 0, 2, 0, 0, 1));
    assert.equal(faces.length, 1, 'direct box section must be one face');
    assert.equal(direct.getSurfaceType(faces[0]), 'plane', 'section carrier is plane');
    assert.equal(faceHoleCount(direct, faces[0]), 0, 'box section has no hole');
    assertClose(direct.faceArea(faces[0], DEFLECTION), 16, 'direct box section area');
    assertSectionOnPlane(direct, faces[0], 0, 0, 2, 0, 0, 1, 'direct box section plane');
    assert.equal(direct.validateSolid(box), 0, 'input stays valid after section');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'section', args: { solid: 0, px: 0, py: 0, pz: 2, nx: 0, ny: 0, nz: 1 } },
      { op: 'volume', args: { solid: 0, deflection: DEFLECTION } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    const batchFaces = batchOk(response, 1, 'batch box section');
    assert.equal(batchFaces.length, 1, 'batch box section must be one face');
    // The batch face handle resolves in the batch kernel's session.
    assert.equal(batch.getSurfaceType(batchFaces[0]), 'plane', 'batch section carrier');
    assert.equal(faceHoleCount(batch, batchFaces[0]), 0, 'batch box section has no hole');
    assertClose(batch.faceArea(batchFaces[0], DEFLECTION), 16, 'batch box section area');
    assertClose(batchOk(response, 2, 'box volume after section'), 64, 'batch input preserved');
    console.log('ok - B6 packaged section: ordinary 4-box transverse, direct+batch area 16');
  }

  // 2. Cavity section with hole, direct + batch: hollow through cavity z=2 => area 12, 1 hole.
  // A lost inner boundary would read as 16 with 0 holes.
  {
    const direct = new BrepKernel();
    const { hollow } = makeHollowDirect(direct);
    assertClose(direct.volume(hollow, DEFLECTION), 56, 'hollow input volume');
    assert.equal(direct.validateSolid(hollow), 0, 'hollow input valid');
    const faces = Array.from(direct.section(hollow, 0, 0, 2, 0, 0, 1));
    assert.equal(faces.length, 1, 'direct cavity section must be one face');
    assert.equal(faceHoleCount(direct, faces[0]), 1, 'direct cavity section must carry its hole');
    assertClose(direct.faceArea(faces[0], DEFLECTION), 12, 'direct cavity section area');
    assertSectionOnPlane(direct, faces[0], 0, 0, 2, 0, 0, 1, 'direct cavity section plane');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
      { op: 'transform', args: { solid: 1, matrix: translationMatrix(1, 1, 1) } },
      { op: 'cut', args: { solidA: 0, solidB: 1 } },
      { op: 'volume', args: { solid: 2, deflection: DEFLECTION } },
      { op: 'section', args: { solid: 2, px: 0, py: 0, pz: 2, nx: 0, ny: 0, nz: 1 } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    assertClose(batchOk(response, 4, 'hollow input volume'), 56, 'batch hollow volume');
    const batchFaces = batchOk(response, 5, 'batch cavity section');
    assert.equal(batchFaces.length, 1, 'batch cavity section must be one face');
    assert.equal(faceHoleCount(batch, batchFaces[0]), 1, 'batch cavity section must carry its hole');
    assertClose(batch.faceArea(batchFaces[0], DEFLECTION), 12, 'batch cavity section area');
    // Direct/batch agreement is additional, never the oracle: both already matched hand 12.
    assertClose(
      batch.faceArea(batchFaces[0], DEFLECTION),
      direct.faceArea(faces[0], DEFLECTION),
      'cavity section direct/batch parity',
    );
    console.log('ok - B6 packaged section: hollow-box cavity with hole, direct+batch area 12');
  }

  // 3. Through-hole sections: plate through hole z=1 => 96 with 1 hole; clear x=1 => 20, 0 holes.
  {
    const direct = new BrepKernel();
    const { holed } = makeHoledPlateDirect(direct);
    assertClose(direct.volume(holed, DEFLECTION), 192, 'holed-plate input volume');
    assert.equal(direct.validateSolid(holed), 0, 'holed-plate input valid');

    const through = Array.from(direct.section(holed, 0, 0, 1, 0, 0, 1));
    assert.equal(through.length, 1, 'plate through-hole section must be one face');
    assert.equal(faceHoleCount(direct, through[0]), 1, 'through-hole section must carry its hole');
    assertClose(direct.faceArea(through[0], DEFLECTION), 96, 'plate through-hole section area');

    const clear = Array.from(direct.section(holed, 1, 0, 0, 1, 0, 0));
    assert.equal(clear.length, 1, 'plate clear section must be one face');
    assert.equal(faceHoleCount(direct, clear[0]), 0, 'clear section has no hole');
    assertClose(direct.faceArea(clear[0], DEFLECTION), 20, 'plate clear section area');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 10, height: 10, depth: 2 } },
      { op: 'makeBox', args: { width: 2, height: 2, depth: 4 } },
      { op: 'transform', args: { solid: 1, matrix: translationMatrix(4, 4, -1) } },
      { op: 'cut', args: { solidA: 0, solidB: 1 } },
      { op: 'section', args: { solid: 2, px: 0, py: 0, pz: 1, nx: 0, ny: 0, nz: 1 } },
      { op: 'section', args: { solid: 2, px: 1, py: 0, pz: 0, nx: 1, ny: 0, nz: 0 } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    const batchThrough = batchOk(response, 4, 'batch through-hole section');
    const batchClear = batchOk(response, 5, 'batch clear section');
    assertClose(batch.faceArea(batchThrough[0], DEFLECTION), 96, 'batch through-hole area');
    assert.equal(faceHoleCount(batch, batchThrough[0]), 1, 'batch through-hole has hole');
    assertClose(batch.faceArea(batchClear[0], DEFLECTION), 20, 'batch clear area');
    assert.equal(faceHoleCount(batch, batchClear[0]), 0, 'batch clear has no hole');
    console.log('ok - B6 packaged section: through-hole 96 with hole, clear 20 without');
  }

  // 4. Supported empty section: miss at z=50 => empty face-list success (direct + batch).
  // This is the section side of the section/split contract split: split misses refuse.
  {
    const direct = new BrepKernel();
    const box = direct.makeBox(4, 4, 4);
    const faces = Array.from(direct.section(box, 0, 0, 50, 0, 0, 1));
    assert.equal(faces.length, 0, 'direct miss must be an empty success');
    assertClose(direct.volume(box, DEFLECTION), 64, 'input preserved after empty section');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'section', args: { solid: 0, px: 0, py: 0, pz: 50, nx: 0, ny: 0, nz: 1 } },
      { op: 'volume', args: { solid: 0, deflection: DEFLECTION } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    assert.deepEqual(batchOk(response, 1, 'batch miss'), [], 'batch miss must be empty');
    assertClose(batchOk(response, 2, 'volume after miss'), 64, 'batch input preserved');
    console.log('ok - B6 packaged section: empty miss is an empty success, direct+batch');
  }

  // 5. Boundary cases with documented contracts: coincident outline + vertex-touch empty.
  {
    const direct = new BrepKernel();
    const box = direct.makeBox(4, 4, 4);
    const coincident = Array.from(direct.section(box, 0, 0, 0, 0, 0, 1));
    assert.equal(coincident.length, 1, 'coincident face must return its outline');
    assertClose(direct.faceArea(coincident[0], DEFLECTION), 16, 'coincident outline area');
    assert.equal(faceHoleCount(direct, coincident[0]), 0, 'coincident outline has no hole');

    const vertex = Array.from(direct.section(box, 0, 0, 0, 1, 1, 1));
    assert.equal(vertex.length, 0, 'vertex touch must be an empty success');
    assertClose(direct.volume(box, DEFLECTION), 64, 'input preserved after boundary sections');
    assert.equal(direct.validateSolid(box), 0, 'input still validates');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'section', args: { solid: 0, px: 0, py: 0, pz: 0, nx: 0, ny: 0, nz: 1 } },
      { op: 'section', args: { solid: 0, px: 0, py: 0, pz: 0, nx: 1, ny: 1, nz: 1 } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    assert.equal(batchOk(response, 1, 'batch coincident').length, 1, 'batch coincident outline');
    assert.deepEqual(batchOk(response, 2, 'batch vertex touch'), [], 'batch vertex empty');
    console.log('ok - B6 packaged section: coincident outline 16, vertex-touch empty');
  }

  // 6. Degenerate typed refusal: edge-touch section has no closed wire (InvalidInput).
  // Direct throws; batch carries the typed error; the session survives with handles intact.
  {
    const direct = new BrepKernel();
    const box = direct.makeBox(4, 4, 4);
    const beforeCounts = Array.from(direct.getEntityCounts(box));
    const beforeFaces = Array.from(direct.getSolidFaces(box));
    assert.throws(
      () => direct.section(box, 0, 0, 2, 1, 1, 0),
      /no closed cross-section/,
      'direct edge-touch must refuse with the assembly failure',
    );
    assertClose(direct.volume(box, DEFLECTION), 64, 'direct input volume after refusal');
    assert.equal(direct.validateSolid(box), 0, 'direct input still validates');
    assert.deepEqual(Array.from(direct.getEntityCounts(box)), beforeCounts, 'direct census preserved');
    assert.deepEqual(Array.from(direct.getSolidFaces(box)), beforeFaces, 'direct handles preserved');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'section', args: { solid: 0, px: 0, py: 0, pz: 2, nx: 1, ny: 1, nz: 0 } },
      { op: 'volume', args: { solid: 0, deflection: DEFLECTION } },
      { op: 'validateSolid', args: { solid: 0 } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    const err = batchErr(response, 1, 'batch edge-touch');
    assert.match(err.message, /no closed cross-section/, 'batch refusal names the assembly failure');
    assert.equal(typeof err.code, 'string', 'batch refusal carries a wire code');
    assert.equal(typeof err.category, 'string', 'batch refusal carries a category');
    assertClose(batchOk(response, 2, 'volume after refused section'), 64, 'batch session preserved');
    assert.equal(batchOk(response, 3, 'validate after refusal'), 0, 'batch input still validates');
    console.log('ok - B6 packaged section: edge-touch InvalidInput, session + handles preserved');
  }

  // 7. Ordinary positive split, direct + batch: 4-box mid z=2 => 32 + 32.
  // Independent oracles: hand halves, volume sum, all-plane census, watertight mesh,
  // material side, input preservation; direct/batch agreement is additional.
  {
    const direct = new BrepKernel();
    const box = direct.makeBox(4, 4, 4);
    const inputCounts = Array.from(direct.getEntityCounts(box));
    const halves = Array.from(direct.split(box, 0, 0, 2, 0, 0, 1));
    assert.equal(halves.length, 2, 'direct split must return two halves');
    const [pos, neg] = halves;
    assertClose(direct.volume(pos, DEFLECTION), 32, 'direct split positive half');
    assertClose(direct.volume(neg, DEFLECTION), 32, 'direct split negative half');
    assertClose(
      direct.volume(pos, DEFLECTION) + direct.volume(neg, DEFLECTION),
      64,
      'direct halves sum to input',
    );
    for (const [half, label] of [[pos, 'positive'], [neg, 'negative']]) {
      assert.equal(direct.validateSolid(half), 0, `direct ${label} validates`);
      const quality = JSON.parse(direct.meshQuality(half, DEFLECTION));
      assert.equal(quality.isWatertight, true, `direct ${label} watertight`);
      assert.equal(quality.boundaryEdges, 0, `direct ${label} closed mesh`);
      assert.equal(quality.nonManifoldEdges, 0, `direct ${label} manifold mesh`);
      assert.ok(
        Array.from(direct.getSolidFaces(half)).every((f) => direct.getSurfaceType(f) === 'plane'),
        `direct ${label} stays all-planar`,
      );
    }
    assert.equal(direct.classifyPoint(pos, 2, 2, 3, TOL), 'inside', 'above-plane in positive');
    assert.equal(direct.classifyPoint(neg, 2, 2, 3, TOL), 'outside', 'above-plane outside negative');
    assert.equal(direct.classifyPoint(neg, 2, 2, 1, TOL), 'inside', 'below-plane in negative');
    assert.equal(direct.classifyPoint(pos, 2, 2, 1, TOL), 'outside', 'below-plane outside positive');
    assertClose(direct.volume(box, DEFLECTION), 64, 'direct input preserved');
    assert.deepEqual(Array.from(direct.getEntityCounts(box)), inputCounts, 'direct input census');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'split', args: { solid: 0, px: 0, py: 0, pz: 2, nx: 0, ny: 0, nz: 1 } },
      { op: 'volume', args: { solid: 1, deflection: DEFLECTION } },
      { op: 'volume', args: { solid: 2, deflection: DEFLECTION } },
      { op: 'validateSolid', args: { solid: 1 } },
      { op: 'validateSolid', args: { solid: 2 } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    const splitOut = batchOk(response, 1, 'batch split');
    assert.ok(
      typeof splitOut.positive === 'number' && typeof splitOut.negative === 'number',
      `batch split returns handles, got ${JSON.stringify(splitOut)}`,
    );
    assertClose(batchOk(response, 2, 'batch positive half'), 32, 'batch split positive');
    assertClose(batchOk(response, 3, 'batch negative half'), 32, 'batch split negative');
    // Pin the advertised sides through the returned handles (volumes are
    // symmetric here, so material probes carry the side semantics).
    assertClose(batch.volume(splitOut.positive, DEFLECTION), 32, 'batch positive handle volume');
    assertClose(batch.volume(splitOut.negative, DEFLECTION), 32, 'batch negative handle volume');
    assert.equal(batch.classifyPoint(splitOut.positive, 2, 2, 3, TOL), 'inside', 'batch positive holds above-plane material');
    assert.equal(batch.classifyPoint(splitOut.negative, 2, 2, 3, TOL), 'outside', 'batch negative excludes above-plane material');
    assert.equal(batch.classifyPoint(splitOut.negative, 2, 2, 1, TOL), 'inside', 'batch negative holds below-plane material');
    assert.equal(batch.classifyPoint(splitOut.positive, 2, 2, 1, TOL), 'outside', 'batch positive excludes below-plane material');
    assert.equal(batchOk(response, 4, 'batch positive validates'), 0, 'batch pos validates');
    assert.equal(batchOk(response, 5, 'batch negative validates'), 0, 'batch neg validates');
    assertClose(
      batchOk(response, 2, 'pos') + batchOk(response, 3, 'neg'),
      64,
      'batch halves sum to input',
    );
    // Parity is additional: both already matched hand 32.
    assertClose(batchOk(response, 2, 'pos'), direct.volume(pos, DEFLECTION), 'split pos parity');
    console.log('ok - B6 packaged split: ordinary 4-box halves 32+32, direct+batch');
  }

  // 8. Through-hole split clear of the hole: plate x=1 => 172 + 20.
  // If the hole were filled, the positive half would read 180 instead of 172.
  {
    const direct = new BrepKernel();
    const { holed } = makeHoledPlateDirect(direct);
    const halves = Array.from(direct.split(holed, 1, 0, 0, 1, 0, 0));
    assert.equal(halves.length, 2, 'plate split must return two halves');
    const [pos, neg] = halves;
    assertClose(direct.volume(pos, DEFLECTION), 172, 'plate split positive (with hole)');
    assertClose(direct.volume(neg, DEFLECTION), 20, 'plate split negative (thin slab)');
    assertClose(
      direct.volume(pos, DEFLECTION) + direct.volume(neg, DEFLECTION),
      192,
      'plate halves sum to input',
    );
    assert.equal(direct.validateSolid(pos), 0, 'plate positive validates');
    assert.equal(direct.validateSolid(neg), 0, 'plate negative validates');
    assert.equal(direct.classifyPoint(pos, 2, 2, 1, TOL), 'inside', 'hole-side probe in positive');
    assert.equal(direct.classifyPoint(neg, 2, 2, 1, TOL), 'outside', 'hole-side probe outside negative');
    assert.equal(direct.classifyPoint(neg, 0.5, 5, 1, TOL), 'inside', 'thin-slab probe in negative');
    assert.equal(direct.classifyPoint(pos, 0.5, 5, 1, TOL), 'outside', 'thin-slab probe outside positive');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 10, height: 10, depth: 2 } },
      { op: 'makeBox', args: { width: 2, height: 2, depth: 4 } },
      { op: 'transform', args: { solid: 1, matrix: translationMatrix(4, 4, -1) } },
      { op: 'cut', args: { solidA: 0, solidB: 1 } },
      { op: 'split', args: { solid: 2, px: 1, py: 0, pz: 0, nx: 1, ny: 0, nz: 0 } },
      { op: 'volume', args: { solid: 3, deflection: DEFLECTION } },
      { op: 'volume', args: { solid: 4, deflection: DEFLECTION } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    const splitOut = batchOk(response, 4, 'batch plate split');
    assert.ok(
      typeof splitOut.positive === 'number' && typeof splitOut.negative === 'number',
      `batch plate split returns handles, got ${JSON.stringify(splitOut)}`,
    );
    assertClose(batchOk(response, 5, 'batch plate positive'), 172, 'batch plate positive');
    assertClose(batchOk(response, 6, 'batch plate negative'), 20, 'batch plate negative');
    // Pin the advertised sides through the returned handles: the asymmetric
    // volumes plus material probes fail if the fields ever swap.
    assertClose(batch.volume(splitOut.positive, DEFLECTION), 172, 'batch positive handle holds the holed side');
    assertClose(batch.volume(splitOut.negative, DEFLECTION), 20, 'batch negative handle holds the thin slab');
    assert.equal(batch.classifyPoint(splitOut.positive, 2, 2, 1, TOL), 'inside', 'batch positive holds hole-side material');
    assert.equal(batch.classifyPoint(splitOut.negative, 2, 2, 1, TOL), 'outside', 'batch negative excludes hole-side material');
    assert.equal(batch.classifyPoint(splitOut.negative, 0.5, 5, 1, TOL), 'inside', 'batch negative holds thin-slab material');
    assert.equal(batch.classifyPoint(splitOut.positive, 0.5, 5, 1, TOL), 'outside', 'batch positive excludes thin-slab material');
    console.log('ok - B6 packaged split: holed-plate clear split 172+20, hole-sensitive volumes');
  }

  // 9a. Typed split refusal: cavity shells are outside the qualified subset (Unsupported).
  {
    const direct = new BrepKernel();
    const { hollow } = makeHollowDirect(direct);
    const beforeCounts = Array.from(direct.getEntityCounts(hollow));
    assert.throws(
      () => direct.split(hollow, 0, 0, 0.5, 0, 0, 1),
      /cavity/,
      'direct cavity split must name the cavity',
    );
    assertClose(direct.volume(hollow, DEFLECTION), 56, 'direct hollow preserved');
    assert.equal(direct.validateSolid(hollow), 0, 'direct hollow still validates');
    assert.deepEqual(Array.from(direct.getEntityCounts(hollow)), beforeCounts, 'direct census');
    assert.ok(Array.from(direct.getSolidFaces(hollow)).length > 0, 'direct handles resolve');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'makeBox', args: { width: 2, height: 2, depth: 2 } },
      { op: 'transform', args: { solid: 1, matrix: translationMatrix(1, 1, 1) } },
      { op: 'cut', args: { solidA: 0, solidB: 1 } },
      { op: 'split', args: { solid: 2, px: 0, py: 0, pz: 0.5, nx: 0, ny: 0, nz: 1 } },
      { op: 'volume', args: { solid: 2, deflection: DEFLECTION } },
      { op: 'validateSolid', args: { solid: 2 } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    const err = batchErr(response, 4, 'batch cavity split');
    assert.match(err.message, /split/, 'cavity refusal names the operation');
    assert.match(err.message, /cavity/, 'cavity refusal names the configuration');
    assertClose(batchOk(response, 5, 'hollow volume after refusal'), 56, 'batch session preserved');
    assert.equal(batchOk(response, 6, 'hollow validates after refusal'), 0, 'batch input validates');
    console.log('ok - B6 packaged split: cavity Unsupported, session + handles preserved');
  }

  // 9b. Typed split refusal through the hole mouth: inner-wire crossing (Unsupported).
  {
    const direct = new BrepKernel();
    const { holed } = makeHoledPlateDirect(direct);
    assert.throws(
      () => direct.split(holed, 5, 0, 0, 1, 0, 0),
      /inner wire/,
      'direct inner-wire split must name the inner wire',
    );
    assertClose(direct.volume(holed, DEFLECTION), 192, 'plate preserved after inner-wire refusal');
    assert.equal(direct.validateSolid(holed), 0, 'plate still validates');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 10, height: 10, depth: 2 } },
      { op: 'makeBox', args: { width: 2, height: 2, depth: 4 } },
      { op: 'transform', args: { solid: 1, matrix: translationMatrix(4, 4, -1) } },
      { op: 'cut', args: { solidA: 0, solidB: 1 } },
      { op: 'split', args: { solid: 2, px: 5, py: 0, pz: 0, nx: 1, ny: 0, nz: 0 } },
      { op: 'volume', args: { solid: 2, deflection: DEFLECTION } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    const err = batchErr(response, 4, 'batch inner-wire split');
    assert.match(err.message, /inner wire/, 'inner-wire refusal names the configuration');
    assertClose(batchOk(response, 5, 'plate volume after refusal'), 192, 'batch session preserved');
    console.log('ok - B6 packaged split: inner-wire Unsupported, through-hole session preserved');
  }

  // 9c. Split miss refuses while section miss succeeds: the two contracts differ by design.
  {
    const direct = new BrepKernel();
    const box = direct.makeBox(4, 4, 4);
    const empty = Array.from(direct.section(box, 0, 0, 50, 0, 0, 1));
    assert.equal(empty.length, 0, 'section miss is an empty success');
    assert.throws(
      () => direct.split(box, 0, 0, 50, 0, 0, 1),
      /entirely on one side/,
      'split miss must refuse as InvalidInput',
    );
    assertClose(direct.volume(box, DEFLECTION), 64, 'input preserved across both miss contracts');

    const batch = new BrepKernel();
    const program = JSON.stringify([
      { op: 'makeBox', args: { width: 4, height: 4, depth: 4 } },
      { op: 'section', args: { solid: 0, px: 0, py: 0, pz: 50, nx: 0, ny: 0, nz: 1 } },
      { op: 'split', args: { solid: 0, px: 0, py: 0, pz: 50, nx: 0, ny: 0, nz: 1 } },
      { op: 'volume', args: { solid: 0, deflection: DEFLECTION } },
    ]);
    const response = parseV2(batch.executeBatchV2(program));
    assert.deepEqual(batchOk(response, 1, 'batch section miss'), [], 'batch section miss empty');
    const err = batchErr(response, 2, 'batch split miss');
    assert.match(err.message, /entirely on one side/, 'split miss names one-sided plane');
    assertClose(batchOk(response, 3, 'volume after split miss'), 64, 'batch session preserved');
    console.log('ok - B6 packaged contracts: section miss empty, split miss InvalidInput');
  }
}

// Allow direct execution: `node scripts/b6-section-split-packaged.mjs`
// loads the freshly built (or committed) pair from the standard pkg paths.
if (process.argv[1] && process.argv[1].endsWith('b6-section-split-packaged.mjs')) {
  const { createRequire } = await import('node:module');
  const { dirname, resolve } = await import('node:path');
  const { fileURLToPath } = await import('node:url');
  const __dirname = dirname(fileURLToPath(import.meta.url));
  const projectRoot = resolve(__dirname, '..');
  const require = createRequire(import.meta.url);
  const { BrepKernel } = require(resolve(projectRoot, 'crates/wasm/pkg/remus_wasm_node.cjs'));
  runB6SectionSplitPackaged({ BrepKernel });
  console.log('\nAll B6 section/split packaged witnesses passed');
}
