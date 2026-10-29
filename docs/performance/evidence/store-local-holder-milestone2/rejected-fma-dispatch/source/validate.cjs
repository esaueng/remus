'use strict';
const fs=require('node:fs');const path=require('node:path');const assert=require('node:assert/strict');const crypto=require('node:crypto');
const root=__dirname;const variants=['baseline','zero-first','bit-range','both'];
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const corpus=fs.readFileSync(path.join(root,'oracle-corpus.bin'));assert.equal(corpus.length%32,0);
const modules=variants.map(name=>{const bytes=fs.readFileSync(path.join(root,name,'optimized.wasm'));const module=new WebAssembly.Module(bytes);assert.deepEqual(WebAssembly.Module.imports(module),[]);const e=new WebAssembly.Instance(module,{}).exports;return {name,e,moduleHash:hash(bytes),checksums:[],finite:0,nanEquivalent:0};});
const cap=modules[0].e.capacity();assert(modules.every(x=>x.e.capacity()===cap));
const isNaNBits=x=>(x&0x7ff0000000000000n)===0x7ff0000000000000n&&(x&0xfffffffffffffn)!==0n;
const count=corpus.length/32;
for(let start=0;start<count;start+=cap){
 const n=Math.min(cap,count-start);const first=modules[0];const input=Buffer.from(first.e.memory.buffer,first.e.input_ptr(),n*24);
 for(let i=0;i<n;i++)corpus.copy(input,i*24,(start+i)*32,(start+i)*32+24);
 let referenceBits;
 for(const m of modules){
  if(m!==first)input.copy(Buffer.from(m.e.memory.buffer,m.e.input_ptr(),n*24));
  const sum=m.e.check_pass(n);
  const result=Buffer.from(m.e.memory.buffer,m.e.output_ptr(),n*8);
  if(m===first)referenceBits=Buffer.from(result);
  for(let i=0;i<n;i++){
   const expected=corpus.readBigUInt64LE((start+i)*32+24);const actual=result.readBigUInt64LE(i*8);
   if(isNaNBits(expected)){assert(isNaNBits(actual),'native NaN-equivalence '+m.name);m.nanEquivalent++;}
   else{assert.equal(actual,expected,'native mul_add output bits '+m.name+' record '+(start+i));assert.equal(actual,referenceBits.readBigUInt64LE(i*8),'variant non-NaN output bits '+m.name);m.finite++;}
  }
  m.checksums.push(BigInt.asUintN(64,sum).toString(16).padStart(16,'0'));
 }
}
// NaN payload propagation may vary between V8 compilation tiers and loop
// contexts. A raw checksum therefore is meaningful only for non-NaN outputs.
// Keep every original record above, then verify volatile-loop checksums for
// the complete non-NaN-output subset using exactly the same native oracle.
const nonNaNRecords=[];
for(let i=0;i<count;i++)if(!isNaNBits(corpus.readBigUInt64LE(i*32+24)))nonNaNRecords.push(i);
for(let start=0;start<nonNaNRecords.length;start+=cap){
 const n=Math.min(cap,nonNaNRecords.length-start);let referenceSum;
 for(const m of modules){
  const input=Buffer.from(m.e.memory.buffer,m.e.input_ptr(),n*24);
  for(let i=0;i<n;i++)corpus.copy(input,i*24,nonNaNRecords[start+i]*32,nonNaNRecords[start+i]*32+24);
  const sum=m.e.check_pass(n);assert.equal(m.e.run_pass(n,1),sum,'non-NaN loop/check checksum '+m.name);
  if(referenceSum===undefined)referenceSum=sum;else assert.equal(sum,referenceSum,'non-NaN variant checksum '+m.name);
 }
}
const receipt={schema:'remus-standalone-wasm-fma-validation-v2',status:'passed',timed:false,records:count,nonNaNLoopChecksumRecords:nonNaNRecords.length,corpusSha256:hash(corpus),driverSha256:hash(fs.readFileSync(__filename)),node:process.version,nodeExecutableSha256:hash(fs.readFileSync(process.execPath)),variants:modules.map(m=>({name:m.name,moduleSha256:m.moduleHash,exactNonNaNOutputs:m.finite,nativeNaNEquivalentOutputs:m.nanEquivalent,chunkChecksums:m.checksums})),policy:'Exact bits versus native mul_add for every non-NaN result, including signed zero and infinities; native NaNs use unchanged NaN-equivalence policy. All four WASM variants additionally match baseline non-NaN raw output bits. Volatile-loop/check checksums match for the complete non-NaN-output subset. NaN-inclusive raw checksums are diagnostic only because NaN payload propagation can vary between compilation contexts.',scope:'Untimed correctness validation only; no kernel or dispatcher performance claim.'};
fs.writeFileSync(path.join(root,'wasm-validation.json'),JSON.stringify(receipt,null,2)+'\n');console.log('VALIDATED',count,'records for all four modules; no timing');
