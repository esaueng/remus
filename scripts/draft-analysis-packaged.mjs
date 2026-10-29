/** Signed draft through the real direct and both batch WASM routes. */
import assert from 'node:assert/strict';
import { createRequire } from 'node:module';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const routes = ['direct', 'executeBatch', 'executeBatchV2'];
const near = (a, b) => assert.ok(Math.abs(a - b) < 2e-13, `${a} != ${b}`);
function call(k, route, op, args) {
  if (route === 'direct') {
    if (op === 'getPlanarSolidDraft') return JSON.parse(k[op](args.solid,args.pullX,args.pullY,args.pullZ));
    return JSON.parse(k[op](args.face,args.u,args.v,args.pullX,args.pullY,args.pullZ));
  }
  const row = JSON.parse(k[route](JSON.stringify([{op,args}])))[0];
  assert.ok(Object.hasOwn(row, 'ok'), JSON.stringify(row));
  return row.ok;
}
function refuse(k, route, op, args, code) {
  if (route === 'direct') {
    assert.throws(() => call(k,route,op,args), error => {
      assert.ok(!(error instanceof WebAssembly.RuntimeError));
      assert.ok(error.message.startsWith(code + ':'), error.message);
      return true;
    });
  } else {
    const row=JSON.parse(k[route](JSON.stringify([{op,args}])))[0];
    assert.ok(Object.hasOwn(row,'error'));
    if (route==='executeBatch') assert.ok(code === 'invalid_handle' ? /^invalid (face|solid) handle: index /.test(row.error) : row.error.startsWith(code + ':'), row.error);
    else if (code === 'invalid_handle') assert.equal(row.error.code,code);
    else assert.equal(row.error.details.kernelCode,code);
  }
}
const state=(k,solids)=>({geometry:Array.from(k.serializeSolids(Uint32Array.from(solids))),
  journal:k.journalSummary(),memory:k.checkpointMemoryStats(),checkpoints:k.checkpointCount()});
export function runDraftAnalysisPackaged({RemusKernel}) {
  let fixtures=0;
  for (const route of routes) for (const scale of [1e-3,1,1e3]) for (const angle of [0,0.37]) {
    const k=new RemusKernel();
    try {
      const solid=k.makeBox(2*scale,3*scale,4*scale);
      const c=Math.cos(angle),s=Math.sin(angle);
      k.transformSolid(solid,[c,-s,0,17*scale,s,c,0,-11*scale,0,0,1,9*scale,0,0,0,1]);
      const checkpoint=k.checkpoint();
      const before=state(k,[solid]);
      const args={solid,pullX:1,pullY:2,pullZ:3};
      const rows=call(k,route,'getPlanarSolidDraft',args);
      assert.deepEqual(rows.map(r=>r.face),Array.from(k.getSolidFaces(solid)));
      const expected=[c+2*s,-s+2*c,3].flatMap(x=>[Math.asin(x/Math.sqrt(14)),-Math.asin(x/Math.sqrt(14))]).sort((a,b)=>a-b);
      rows.map(r=>r.angleRadians).sort((a,b)=>a-b).forEach((value,i)=>near(value,expected[i]));
      for (const row of rows) {
        assert.deepEqual(Object.keys(row).sort(),['angleRadians','face','outwardNormal']);
        const n=row.outwardNormal;
        near(Math.hypot(...n),1);
        const vertices=JSON.parse(k.executeBatch(JSON.stringify([{op:'getFaceVertexPositions',args:{face:row.face}}])))[0].ok;
        const center=[0,1,2].map(axis=>vertices.filter((_,i)=>i%3===axis).reduce((a,b)=>a+b,0)/(vertices.length/3));
        const local=[c*(center[0]-17*scale)+s*(center[1]+11*scale),-s*(center[0]-17*scale)+c*(center[1]+11*scale),center[2]-9*scale];
        const sizes=[2*scale,3*scale,4*scale];
        const axis=local.findIndex((x,i)=>Math.min(Math.abs(x),Math.abs(x-sizes[i]))<1e-9*scale);
        assert.ok(axis>=0);
        const sign=local[axis]<sizes[axis]/2?-1:1;
        const normal=[[c,s,0],[-s,c,0],[0,0,1]][axis].map(x=>sign*x);
        n.forEach((x,i)=>near(x,normal[i]));
        near(row.angleRadians,Math.asin((n[0]+2*n[1]+3*n[2])/Math.sqrt(14)));
        const sample=call(k,route,'getFaceDraftAngle',{...args,face:row.face,u:19,v:-21});
        assert.deepEqual(sample,{angleRadians:row.angleRadians,outwardNormal:n});
        const opposite=call(k,route,'getFaceDraftAngle',{face:row.face,u:0,v:0,pullX:-1,pullY:-2,pullZ:-3});
        near(opposite.angleRadians,-row.angleRadians);
      }
      refuse(k,route,'getPlanarSolidDraft',{...args,pullX:0,pullY:0,pullZ:0},'draft-analysis:invalid-pull');
      assert.deepEqual(state(k,[solid]),before);
      k.restore(checkpoint);
      assert.deepEqual(state(k,[solid]),before);
      k.discardCheckpoint(checkpoint);
      fixtures++;
    } finally {k.free();}
  }
  for (const route of routes) {
    const k=new RemusKernel();
    try {
      const solid=k.makeCylinder(1,2,16),before=state(k,[solid]);
      refuse(k,route,'getPlanarSolidDraft',{solid,pullX:0,pullY:0,pullZ:1},'draft-analysis:non-planar-face');
      assert.deepEqual(state(k,[solid]),before);
      fixtures++;
    } finally {k.free();}
  }
  for (const route of routes) {
    const k=new RemusKernel();
    try {
      const sphere=k.makeSphere(2,16), faces=Array.from(k.getSolidFaces(sphere));
      const before=state(k,[sphere]);
      for (const face of faces) for (const v of [-0.7,0,0.7,Math.PI/2]) {
        const sample=call(k,route,'getFaceDraftAngle',{face,u:0.31,v,pullX:0,pullY:0,pullZ:1});
        near(sample.angleRadians,v);
        near(sample.outwardNormal[2],Math.sin(v));
        near(Math.hypot(...sample.outwardNormal),1);
      }
      refuse(k,route,'getFaceDraftAngle',{face:faces[0],u:0,v:2,pullX:0,pullY:0,pullZ:1},'draft-analysis:invalid-parameters');
      refuse(k,route,'getFaceDraftAngle',{face:0xffffffff,u:0,v:0,pullX:0,pullY:0,pullZ:1},'invalid_handle');
      refuse(k,route,'getPlanarSolidDraft',{solid:0xffffffff,pullX:0,pullY:0,pullZ:1},'invalid_handle');
      assert.deepEqual(state(k,[sphere]),before);
      fixtures++;
    } finally {k.free();}
  }
  for (const route of routes) {
    const k=new RemusKernel();
    try {
      const outer=k.makeBox(4,4,4),inner=k.makeBox(2,2,2);
      k.transformSolid(inner,[1,0,0,1,0,1,0,1,0,0,1,1,0,0,0,1]);
      const hollow=k.cut(outer,inner),before=state(k,[hollow]);
      const rows=call(k,route,'getPlanarSolidDraft',{solid:hollow,pullX:0,pullY:0,pullZ:1});
      assert.equal(rows.length,12);
      let cavities=0;
      for (const row of rows) {
        const vertices=JSON.parse(k.executeBatch(JSON.stringify([{op:'getFaceVertexPositions',args:{face:row.face}}])))[0].ok;
        const center=[0,1,2].map(axis=>vertices.filter((_,i)=>i%3===axis).reduce((a,b)=>a+b,0)/(vertices.length/3));
        const isOuter=center.some(x=>Math.abs(x)<1e-12||Math.abs(x-4)<1e-12);
        const radialDot=center.reduce((sum,x,i)=>sum+(x-2)*row.outwardNormal[i],0);
        assert.ok(isOuter?radialDot>0:radialDot<0);
        if (!isOuter) cavities++;
        near(row.angleRadians,Math.asin(row.outwardNormal[2]));
      }
      assert.equal(cavities,6);
      assert.deepEqual(state(k,[hollow]),before);
      fixtures++;
    } finally {k.free();}
  }
  console.log(`Draft-analysis packaged witnesses passed (${fixtures} fixtures; direct/legacy/V2)`);
  return fixtures;
}
if (process.argv[1] && resolve(process.argv[1])===fileURLToPath(import.meta.url)) {
  assert.ok(process.argv[2],'pass actual built or installed remus_wasm.js path');
  runDraftAnalysisPackaged(createRequire(import.meta.url)(resolve(process.argv[2])));
}
