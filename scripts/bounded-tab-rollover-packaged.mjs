/** Bounded exact sequential tab runout through direct and installed consumers. */
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const near = (a, b) => Math.abs(a - b) < 1e-7;
const bytes = (k, s) => Buffer.from(k.serializeSolid(s));
function edges(k, s, pick) {
  return Array.from(k.getSolidEdges(s)).filter((e) => {
    const p = Array.from(k.getEdgeVertices(e));
    return pick(p.slice(0, 3), p.slice(3));
  });
}
function freshBracket(k, prefix = false) {
  const profile = Float64Array.from([[0,0],[22,0],[22,8],[52,8],[52,0],[74,0],[74,60],[0,60]].flatMap(([x,y]) => [x,y,0]));
  const base = k.extrude(k.makePolygon(profile), 0, 0, 1, 4);
  const left = k.makeBox(18, 10, 40), right = k.makeBox(18, 10, 40);
  k.transformSolid(right, Float64Array.of(1,0,0,56, 0,1,0,0, 0,0,1,0, 0,0,0,1));
  let body = k.fuse(k.fuse(base, left), right);
  if (prefix) {
    for (const [radius, y, xs] of [[6,60,[0,74]],[2,8,[22,52]]]) {
      const selected = edges(k, body, (a,b) => near(a[0],b[0]) && near(a[1],y) && near(b[1],y) && xs.some((x) => near(a[0],x)));
      assert.equal(selected.length, 2);
      body = k.fillet(body, Uint32Array.from(selected), radius);
    }
  }
  const selected = edges(k, body, (a,b) => near(a[0],b[0]) && near(a[1],b[1])
    && [18,56].some((x) => near(a[0],x)) && [0,10].some((y) => near(a[1],y)) && Math.min(a[2],b[2]) > 4 - 1e-7);
  assert.equal(selected.length, 4);
  return k.fillet(body, Uint32Array.from(selected), 2);
}
function selection(k, body) {
  const selected = edges(k, body, (a,b) => [a,b].every((p) => near(p[1],10) && near(p[2],4)));
  assert.equal(selected.length, 2);
  return Uint32Array.from(selected);
}
function invoke(k, surface, solid, selected, radius) {
  if (surface === 'direct') return k.fillet(solid, selected, radius);
  if (surface === 'directDetailed') return k.filletDetailed(solid, selected, radius, true);
  if (surface === 'evolution') return k.filletWithEvolution(solid, selected, radius);
  const [method, op] = surface.split(':');
  const response = JSON.parse(k[method](JSON.stringify([{ op, args: { solid, edges: Array.from(selected), radius, exactOnly: true } }])))[0];
  assert.equal(response.error, undefined, JSON.stringify(response));
  return response.ok;
}
function material(k, body) {
  let count = 0;
  const probe = (point, expected) => { assert.equal(k.classifyPoint(body, ...point, 1e-7), expected, point.join(',')); count++; };
  for (const mirror of [false,true]) {
    const w = ([x,y,z]) => [mirror ? 74-x : x, y, z];
    for (const z of [0.3,1,2]) {
      const q = 3 - Math.sqrt(6*z - z*z);
      for (const [point, expected] of [
        [[8,10+q/2,4+z],'inside'],[[8,10+q+0.1,4+z],'outside'],
        [[18+q/2,5,4+z],'inside'],[[18+q+0.1,5,4+z],'outside'],
        [[16+(2+q/2)*Math.SQRT1_2,8+(2+q/2)*Math.SQRT1_2,4+z],'inside'],
        [[16+(2+q+0.1)*Math.SQRT1_2,8+(2+q+0.1)*Math.SQRT1_2,4+z],'outside'],
        [[16+(2+q/2)*Math.cos(0.4),2-(2+q/2)*Math.sin(0.4),4+z],'inside'],
        [[16+(2+q+0.1)*Math.cos(0.4),2-(2+q+0.1)*Math.sin(0.4),4+z],'outside'],
        [[16+(Math.sqrt(4-1.85**2)+Math.sqrt((2+q)**2-1.85**2))/2,0.15,4+z],'inside'],
        [[16+Math.sqrt((2+q)**2-1.85**2)+0.1,0.15,4+z],'outside'],
        [[16.5,-0.1,4+z],'outside'],
      ]) probe(w(point), expected);
    }
    probe(w([8,5,30]), 'inside'); probe(w([17.9,0.1,12]), 'outside');
  }
  probe([40,30,2], 'inside'); probe([30,4,2], 'outside');
  assert.equal(count, 72);
}
function meshCensus(mesh, quantum) {
  const p = mesh.positions, ix = mesh.indices;
  const map = new Map(), ids = [];
  for (let i=0;i<p.length;i+=3) {
    const key = quantum ? [p[i],p[i+1],p[i+2]].map((v) => Math.round(v/quantum)).join(',') : i/3;
    if (!map.has(key)) map.set(key,map.size); ids.push(map.get(key));
  }
  const uses = new Map(); let volume=0;
  const ref = [p[0],p[1],p[2]];
  for (let i=0;i<ix.length;i+=3) {
    const native = [ix[i],ix[i+1],ix[i+2]], tri = native.map((j) => ids[j]);
    assert.equal(new Set(tri).size, 3, 'nondegenerate triangle');
    for (const [a,b] of [[tri[0],tri[1]],[tri[1],tri[2]],[tri[2],tri[0]]]) {
      const key = `${Math.min(a,b)},${Math.max(a,b)}`;
      if (!uses.has(key)) uses.set(key,[]); uses.get(key).push(a<b ? 1 : -1);
    }
    const v = native.map((j) => [p[3*j]-ref[0],p[3*j+1]-ref[1],p[3*j+2]-ref[2]]);
    volume += (v[0][0]*(v[1][1]*v[2][2]-v[1][2]*v[2][1])+v[0][1]*(v[1][2]*v[2][0]-v[1][0]*v[2][2])+v[0][2]*(v[1][0]*v[2][1]-v[1][1]*v[2][0]))/6;
  }
  for (const directions of uses.values()) assert.deepEqual(directions.slice().sort(), [-1,1]);
  return volume;
}
export function runBoundedTabRolloverPackaged({ BrepKernel, decodeEvolutionPayload }) {
  const source = readFileSync(new URL('../crates/operations/tests/data/rollover_tabbed_source.arena.json', import.meta.url));
  let expected;
  for (const surface of ['direct','directDetailed','evolution','executeBatch:fillet','executeBatchV2:fillet','executeBatch:filletDetailed','executeBatchV2:filletDetailed']) {
    const k = new BrepKernel();
    try {
      const body = k.deserializeSolid(source), other = k.makeBox(1,2,3), selected = selection(k,body);
      const before = bytes(k,body), otherBefore = bytes(k,other), checkpoint = k.checkpoint();
      const output = invoke(k,surface,body,selected,3);
      let result;
      if (surface === 'evolution') {
        const payload = decodeEvolutionPayload(output);
        result = payload.result.solid;
        assert.equal(payload.evolution.provenance, 'construction');
        assert.equal(payload.evolution.deleted.length, 2);
        assert.equal(payload.evolution.unresolvedSources.length, 0);
        assert.equal(payload.evolution.unresolvedResults.length, 0);
      } else if (typeof output === 'object') {
        assert.equal(output.status, 'ok'); assert.equal(output.details.quality, 'exact');
        assert.equal(output.details.engine, 'rollingBall'); result = output.value;
      } else result = output;
      assert.equal(k.getSolidFaces(result).length,28);
      assert.equal(JSON.parse(k.validateSolidDetailed(result)).errorCount,0);
      const serialized = bytes(k,result), doc = JSON.parse(serialized);
      assert.equal(doc.boundary_authority.coedges.length,156);
      assert.ok(doc.boundary_authority.coedges.every((use) => use.pcurve != null));
      assert.equal(doc.edges.filter((edge) => edge.curve?.NurbsCurve).length,2);
      assert.equal(doc.faces.filter((face) => face.surface?.Nurbs).length,0);
      if (expected === undefined) expected = serialized; else assert.deepEqual(serialized, expected, surface);
      material(k,result);
      assert.deepEqual(bytes(k,body),before); assert.deepEqual(bytes(k,other),otherBefore);
      k.restore(checkpoint); assert.throws(() => k.getSolidFaces(result));
      assert.deepEqual(bytes(k,body),before); assert.deepEqual(bytes(k,other),otherBefore);
      assert.ok(k.makeBox(1,1,1) > result, 'retired handles remain stale');
    } finally { k.free(); }
  }
  // Actual user recipe, including preserved F1/F2 geometry. These public
  // meshes are checked separately from the native raw pre-weld regression.
  for (const prefix of [false,true]) {
    const k = new BrepKernel();
    try {
      const input = freshBracket(k,prefix), selected = selection(k,input), before = bytes(k,input);
      const result = k.filletDetailed(input,selected,3,true);
      assert.equal(result.status,'ok'); assert.equal(result.details.quality,'exact');
      material(k,result.value);
      let coarse;
      for (const [deflection,angle] of [[0.002,0.01],[0.0005,0.005]]) {
        const mesh=k.tessellateSolid(result.value,deflection,angle);
        try {
          const volume=meshCensus(mesh,0); meshCensus(mesh,1e-8); meshCensus(mesh,1e-6);
          const expectedVolume=29746.712312911633-(prefix ? 2*4*(1-Math.PI/4)*6**2-2*4*(1-Math.PI/4)*2**2 : 0);
          assert.ok(Math.abs(volume-expectedVolume)<0.02, `${volume} vs independent ${expectedVolume}`);
          if (coarse!==undefined) assert.ok(Math.abs(volume-coarse)<0.02); coarse=volume;
        } finally { mesh.free(); }
      }
      assert.deepEqual(bytes(k,input),before);
      const refuse=k.filletDetailed(input,selected,4.1,true);
      assert.equal(refuse.status,'error'); assert.equal(refuse.code,'cliff-encountered');
      assert.deepEqual(bytes(k,input),before);
    } finally { k.free(); }
  }
  console.log('ok - bounded exact tab rollover: complete shared PCurves, history, material, quantities, direct/batch, rollback and preserved prefix');
}
