/** Native-equivalent public history fixture, timed through the Node binding.
 * Usage: node scripts/axial-replay-profile.mjs /path/to/node-package-entry
 * Build each revision with the same wasm-pack flags before comparing.
 */
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';
import { resolve } from 'node:path';
import assert from 'node:assert/strict';

const { RemusKernel } = createRequire(import.meta.url)(resolve(process.argv[2] ?? 'crates/wasm/pkg/remus_wasm_node.cjs'));
const samples = [];
for (let run = 0; run < 3; run++) {
  const k = new RemusKernel();
  try {
    const box = k.makeBox(32.75, 18, 26), boss = k.makeCylinder(6, 35);
    let solid = k.fuse(boss, box);
    const near = (a, b) => Math.abs(a - b) < 1e-7;
    const edges = Array.from(k.getSolidEdges(solid)).filter(edge => {
      const [ax, ay, az, bx, by, bz] = k.getEdgeVertices(edge);
      const atTop = near(az, 26) && near(bz, 26);
      const kind = k.getEdgeCurveType(edge);
      return kind === 'CIRCLE' ? atTop : kind === 'LINE' &&
        ((atTop && ((near(ax, 0) && near(bx, 0)) || (near(ay, 0) && near(by, 0)))) ||
          (Math.abs(az - bz) > 25.99 &&
            ((near(ax, 6) && near(bx, 6) && near(ay, 0) && near(by, 0)) ||
              (near(ay, 6) && near(by, 6) && near(ax, 0) && near(bx, 0)))));
    });
    assert.equal(edges.length, 5);
    solid = k.fillet(solid, Uint32Array.from(edges), 5);
    const stages = [];
    k.setPerformanceTracing(true);
    for (let index = 0; index < 21; index++) {
      const face = Array.from(k.getSolidFaces(solid)).find(face => k.getSurfaceType(face) === 'plane' && k.getFaceNormal(face)[2] > .99 &&
        Array.from(k.getFaceVertices(face)).every(v => k.getVertexPosition(v)[0] > 4.9));
      assert.notEqual(face, undefined);
      const distance = index === 0 ? -6.28 : index % 2 ? .4 : -.3;
      const start = performance.now();
      const result = JSON.parse(k.moveFacesJournaledWithQuality(solid, Uint32Array.of(face), distance, 256));
      stages.push(performance.now() - start);
      solid = result.solid;
      assert.equal(result.quality, 'exact');
    }
    const phases = {};
    for (const p of JSON.parse(k.drainPerformanceTrace())) phases[p.name] = (phases[p.name] ?? 0) + p.duration_ms;
    k.setPerformanceTracing(false);
    const geometry = JSON.parse(new TextDecoder().decode(k.serializeSolids(Uint32Array.of(solid))));
    delete geometry.journal;
    const geometrySha256 = createHash('sha256').update(JSON.stringify(geometry)).digest('hex');
    samples.push(stages.reduce((a, b) => a + b, 0));
    console.log(JSON.stringify({ run, ms: samples.at(-1), stages, phases, faces: k.getSolidFaces(solid).length, volume: k.volume(solid, .01), geometrySha256 }));
  } finally { k.free(); }
}
console.log(JSON.stringify({ samples }));
