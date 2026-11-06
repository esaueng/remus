/**
 * Live blend-group drags through the packaged WASM kernel, timed the way
 * OpenZCAD runs them: prepare at drag start, then per frame `restore` the
 * drag's checkpoint and `previewPreparedGroupEdit`, and keep the last value
 * with `commitPreparedGroupEditJournaled`. The unprepared journaled call each
 * frame replaces (`resizeBlendJournaled`, `resizeCylindricalFaceJournaled`)
 * is timed beside it, first call and warm.
 *
 *   cargo xtask wasm-build
 *   # optional, the cumulative body (A2 R1.9, A3 R1, A4 0.7, A6 0.8):
 *   REMUS_CUMULATIVE_GECKO_DOC=/tmp/cumulative-gecko.bin \
 *     cargo test -p remus-wasm --lib write_cumulative_gecko_document -- --ignored
 *   node scripts/perf/prepared-group-edit-bench.mjs [/tmp/cumulative-gecko.bin]
 *
 * Every frame is checked against the unprepared call at the same value in a
 * second kernel (face count, validity, volume to 1e-9) outside the timed
 * regions. Prints one JSON row per measurement.
 */
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
const root = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const { RemusKernel } = require(resolve(root, 'crates/wasm/pkg/remus_wasm_node.cjs'));
const { RemusIo } = require(resolve(root, 'crates/wasm-io/pkg/remus_wasm_io_node.cjs'));

const gecko = readFileSync(resolve(root, 'crates/operations/tests/data/gentle_gecko_filleted.step'));
const cumulativePath = process.argv[2];

const timed = (fn) => {
  const start = performance.now();
  const value = fn();
  return [value, performance.now() - start];
};
const median = (values) => {
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.floor(sorted.length / 2)];
};
const params = (k, face) => JSON.parse(k.getAnalyticSurfaceParams(face));
const boss = (k, solid) => Array.from(k.getSolidFaces(solid)).find((face) => {
  const p = params(k, face);
  return p.type === 'cylinder' && Math.abs(p.axis[2]) > 0.99 &&
    Math.abs(p.origin[0]) < 1e-7 && Math.abs(p.origin[1]) < 1e-7;
});
/** The convex top band along the x = 0 face (axis along Y, on x = −r). */
const x0Band = (k, solid) => Array.from(k.getSolidFaces(solid)).find((face) => {
  const p = params(k, face);
  return p.type === 'cylinder' && Math.abs(p.axis[1]) > 0.99 && p.origin[0] > -1e-7 && p.origin[2] > 18;
});

/** A fresh kernel holding the body: the Gecko STEP, or an arena document. */
const load = (body) => {
  const k = new RemusKernel();
  let solid;
  if (body === 'gecko') {
    const io = new RemusIo();
    try { [solid] = k.deserializeSolids(io.importStep(gecko)); } finally { io.free(); }
  } else {
    [solid] = k.deserializeSolids(readFileSync(cumulativePath));
  }
  return [k, solid];
};

const rows = [];
const report = (row) => { rows.push(row); console.log(JSON.stringify(row)); };

const drag = (body, edit, values) => {
  const [k, solid] = load(body);
  const [plain, plainSolid] = load(body);
  try {
    assert.equal(solid, plainSolid);
    const face = edit === 'A2' ? x0Band(k, solid) : boss(k, solid);
    assert.notEqual(face, undefined, `${body} ${edit}: face`);
    const current = params(k, face).radius;
    const radius = (value) => (edit === 'A2' ? value : value / 2);
    const unprepared = (kernel, value) => (edit === 'A2'
      ? JSON.parse(kernel.resizeBlendJournaled(solid, face, current, radius(value)))
      : JSON.parse(kernel.resizeCylindricalFaceJournaled(solid, face, radius(value))));

    // The unprepared journaled edit, as each frame paid it before.
    const plainCheckpoint = plain.checkpoint();
    const unpreparedTimes = values.map((value) => {
      plain.restore(plainCheckpoint);
      return timed(() => unprepared(plain, value))[1];
    });
    report({ body, edit, phase: 'unprepared', firstMs: unpreparedTimes[0], warmMedianMs: median(unpreparedTimes.slice(1)) });

    const checkpoint = k.checkpoint();
    const [prepared, prepareMs] = timed(() => (edit === 'A2'
      ? k.prepareResizeBlend(solid, face, current)
      : k.prepareResizeCylindricalFace(solid, face)));
    const frameTimes = [];
    for (const value of values) {
      k.restore(checkpoint);
      const [frame, ms] = timed(() => k.previewPreparedGroupEdit(prepared, radius(value)));
      frameTimes.push(ms);
      // Outside the timed region: the frame equals the unprepared edit.
      plain.restore(plainCheckpoint);
      const reference = unprepared(plain, value).solid;
      assert.equal(k.validateSolid(frame), 0, `${body} ${edit} ${value}: valid frame`);
      assert.equal(k.getSolidFaces(frame).length, plain.getSolidFaces(reference).length);
      const [a, b] = [k.volume(frame, 0.001), plain.volume(reference, 0.001)];
      assert.ok(Math.abs(a - b) <= 1e-9 * b, `${body} ${edit} ${value}: volume ${a} vs ${b}`);
    }
    k.restore(checkpoint);
    const [, commitMs] = timed(() => JSON.parse(k.commitPreparedGroupEditJournaled(prepared, radius(values.at(-1)))));
    assert.equal(k.releasePreparedGroupEdit(prepared), true);
    report({
      body, edit, phase: 'prepared', prepareMs,
      firstFrameMs: frameTimes[0], warmFrameMedianMs: median(frameTimes.slice(1)),
      warmFrameMaxMs: Math.max(...frameTimes.slice(1)), commitMs,
    });
  } finally {
    k.free();
    plain.free();
  }
};

const bodies = cumulativePath ? ['gecko', 'cumulative'] : ['gecko'];
for (const body of bodies) {
  drag(body, 'A2', body === 'gecko' ? [1.5, 1.9, 1.5, 1.7, 1.6] : [1.5, 1.7, 1.5, 1.6, 1.8]);
  drag(body, 'A7', [14.6, 17, 21.1, 14.7, 15.4]);
}
console.log(`ok - prepared group edit benchmark (${rows.length} rows)`);
