'use strict';
const fs=require('node:fs'),crypto=require('node:crypto'),assert=require('node:assert/strict');
const [preview,completion,out]=process.argv.slice(2);assert(preview&&completion&&out);const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const inputs=[['preview',preview],['completion',completion]].map(([phase,file])=>{const raw=fs.readFileSync(file);return {phase,sha256:hash(raw),data:JSON.parse(raw)};});
const names=Object.keys(inputs[0].data.total.counts),families=names.filter(x=>x!=='total');
const acceptedPhases=new Set(['prefix','rebuild','validate','mesh','volume','recognize']);
const retained=r=>r.tick>=2&&!r.warmup&&acceptedPhases.has(r.phase);
const merged={schema:'remus-combined-diagnostic-dispatch-captures-v1',producerSha256:hash(fs.readFileSync(__filename)),inputs:inputs.map(x=>({mode:x.phase,sha256:x.sha256})),selection:{ticks:'>=2',warmup:false,phases:[...acceptedPhases]},total:{counts:Object.fromEntries(names.map(x=>[x,'0'])),samples:Object.fromEntries(families.map(x=>[x,[]]))},outerKernelByStage:[],recognizeFeaturesRoots:[]};
for(const {phase,data}of inputs){assert.deepEqual(Object.keys(data.total.counts),names);const stages=data.outerKernelByStage.filter(retained);for(const row of stages){for(const k of names)merged.total.counts[k]=String(BigInt(merged.total.counts[k])+BigInt(row.counts[k]));for(const f of families)merged.total.samples[f].push(...row.samples[f]);}merged.outerKernelByStage.push(...stages.map(x=>({...x,recipeMode:phase})));merged.recognizeFeaturesRoots.push(...data.recognizeFeaturesRoots.filter(retained).map(x=>({...x,recipeMode:phase})));}
fs.writeFileSync(out,JSON.stringify(merged,null,2)+'\n');console.log('combined untimed diagnostic captures; total calls',merged.total.counts.total);
