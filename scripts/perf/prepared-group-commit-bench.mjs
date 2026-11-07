/**
 * W-Fillet + R1 cap rim, A2 R4 -> 1.5 and A7 diameter 12 -> 14.6.
 * Build with wasm-pack --target nodejs, then pass its remus_wasm.js path.
 * The translator defaults to the committed lazy I/O package. Each sample
 * prepares anew, previews, restores the checkpoint and commits journaled.
 * Compare the same command against baseline and candidate packages.
 * No timing assertions; geometry/history parity is covered by native tests.
 */
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const require = createRequire(import.meta.url);
const { RemusKernel } = require(resolve(process.argv[2] ?? 'crates/wasm/pkg/remus_wasm_node.cjs'));
const { RemusIo } = require(resolve(root, 'crates/wasm-io/pkg/remus_wasm_io_node.cjs'));
const io = new RemusIo();
const document = io.importStep(readFileSync(resolve(root,
  'crates/operations/tests/data/gentle_gecko_filleted.step')));
io.free();
const timed = (run) => {
  const start = performance.now();
  const result = run();
  return [result, performance.now() - start];
};
for (const edit of ['A2', 'A7']) {
  const k = new RemusKernel();
  try {
    const [imported] = k.deserializeSolids(document);
    const params = (face) => JSON.parse(k.getAnalyticSurfaceParams(face));
    const cap = Array.from(k.getSolidFaces(imported))
      .map((face) => [face, params(face)])
      .filter(([, p]) => p.type === 'plane' && Math.abs(p.normal[2]) > 0.999)
      .sort((a, b) => b[1].d / b[1].normal[2] - a[1].d / a[1].normal[2])[0][0];
    const solid = k.fillet(imported, k.getFaceEdges(cap), 1);
    const face = Array.from(k.getSolidFaces(solid)).find((face) => {
      const p = params(face);
      return p.type === 'cylinder' && (edit === 'A2'
        ? Math.abs(p.axis[1]) > 0.99 && Math.abs(p.radius - 4) < 1e-7 && p.origin[2] > 18
        : Math.abs(p.axis[2]) > 0.99 && Math.abs(p.origin[0]) < 1e-7 && Math.abs(p.origin[1]) < 1e-7);
    });
    assert.notEqual(face, undefined);
    const value = edit === 'A2' ? 1.5 : 7.3;
    const checkpoint = k.checkpoint();
    for (let sample = 0; sample < 3; sample++) {
      k.restore(checkpoint);
      const prepared = edit === 'A2' ? k.prepareResizeBlend(solid, face, 4)
        : k.prepareResizeCylindricalFace(solid, face);
      const [preview, previewMs] = timed(() => k.previewPreparedGroupEdit(prepared, value));
      const frame = k.serializeSolids(new Uint32Array([preview]));
      k.restore(checkpoint);
      k.setPerformanceTracing(true);
      const [result, commitMs] = timed(() => JSON.parse(k.commitPreparedGroupEditJournaled(prepared, value)));
      const phases = JSON.parse(k.drainPerformanceTrace());
      k.setPerformanceTracing(false);
      const adopted = phases.some((phase) => phase.name === 'blend_group.adopt_preview');
      if (process.env.REMUS_EXPECT_PREVIEW_ADOPTION === '1') assert.ok(adopted, `${edit}: adopted preview`);
      assert.equal(k.validateSolid(result.solid), 0);
      const committedVolume = k.volume(result.solid, 0.001);
      const reference = new RemusKernel();
      try {
        const [frameSolid] = reference.deserializeSolids(frame);
        const frameVolume = reference.volume(frameSolid, 0.001);
        assert.ok(Math.abs(frameVolume - committedVolume) <= 1e-9 * committedVolume);
      } finally {
        reference.free();
      }
      assert.equal(k.releasePreparedGroupEdit(prepared), true);
      const documentSha256 = createHash('sha256')
        .update(k.serializeSolids(new Uint32Array([result.solid]))).digest('hex');
      console.log(JSON.stringify({ edit, sample, previewMs, commitMs, adopted, documentSha256 }));
    }
  } finally {
    k.free();
  }
}
