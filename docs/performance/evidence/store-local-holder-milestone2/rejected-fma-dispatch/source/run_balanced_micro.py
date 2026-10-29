from pathlib import Path
import subprocess,json,hashlib,statistics,time,sys,os

root=Path(__file__).resolve().parent
workloads=Path(sys.argv[1]); capture=Path(sys.argv[2]);capture.mkdir(exist_ok=False)
hash_file=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def environment():
    return {'at':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'loadavg':Path('/proc/loadavg').read_text().strip(),
            'cpuPressure':Path('/proc/pressure/cpu').read_text().strip(),'meminfo':{x.split(':')[0]:x.split(':',1)[1].strip() for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith(('MemAvailable:','SwapFree:','SwapTotal:'))}}
plan=[['baseline','zero-first','bit-range','both','both','bit-range','zero-first','baseline'],
      ['zero-first','baseline','both','bit-range','bit-range','both','baseline','zero-first']]
receipt={'schema':'remus-standalone-fma-dispatch-balanced-v1','scope':'Micro only; sampled diagnostic operands, not whole kernel or holder latency.',
         'plan':plan,'before':environment(),'driverSha256':hash_file(__file__),'workloadsManifestSha256':hash_file(workloads/'manifest.json'),
         'cpuLease':'CPU31; SMT sibling15 reserved by coordinator; no exclusive-host claim','processes':[]}
rounds=capture/'rounds.json'
p=subprocess.run(['taskset','-c','31','node',str(root/'calibrate.cjs'),str(workloads),str(rounds)],capture_output=True,text=True)
(capture/'calibration.log').write_text(p.stdout+p.stderr);assert p.returncode==0,p.stdout+p.stderr
for block,sequence in enumerate(plan):
    for position,variant in enumerate(sequence):
        name=f'block-{block}-position-{position}-{variant}.json'
        p=subprocess.run(['taskset','-c','31','node',str(root/'micro_process.cjs'),str(workloads),variant,str(rounds),str(capture/name)],capture_output=True,text=True)
        (capture/(name+'.log')).write_text(p.stdout+p.stderr);assert p.returncode==0,p.stdout+p.stderr
        r=json.loads((capture/name).read_text());assert r['affinity'].split(':',1)[1].strip()=='31'
        receipt['processes'].append({'block':block,'position':position,'variant':variant,'file':name,'sha256':hash_file(capture/name)})
    print('finished block',block,flush=True)
receipt['after']=environment();receipt['roundsSha256']=hash_file(rounds)
(capture/'processes.json').write_text(json.dumps(receipt,indent=2)+'\n')
manifest=json.loads((workloads/'manifest.json').read_text());summary={'schema':'remus-standalone-fma-dispatch-summary-v1','processesSha256':hash_file(capture/'processes.json'),'workloads':{},'limits':manifest['limits']+['Standalone loop, common volatile loads/checksum, separate module layout and compiler context; micro effects do not establish package or workflow gains.']}
for w in manifest['workloads']:
    name=w['name'];result={}
    for variant in plan[0][:4]:
        values=[];blocks={0:[],1:[]};checksums=set()
        for p in receipt['processes']:
            if p['variant']!=variant:continue
            samples=json.loads((capture/p['file']).read_text())['samples'];rows=[r for r in samples if r['workload']==name and not r['warmup']]
            assert len(rows)==6
            checksums.update(r['checksum'] for r in rows)
            median=statistics.median(int(r['durationNs'])/r['operations'] for r in rows)
            values.append(median);blocks[p['block']].append(median)
        assert len(values)==4
        result[variant]={'processMedianNsPerOperation':values,'medianNsPerOperation':statistics.median(values),
                         'blockMedians':{str(b):statistics.median(v) for b,v in blocks.items()},'retainedSamples':24,'checksums':sorted(checksums)}
    reference=result['baseline']['medianNsPerOperation']
    for variant in result:
        result[variant]['changeFraction']=result[variant]['medianNsPerOperation']/reference-1
        result[variant]['blockChangeFractions']={str(b):result[variant]['blockMedians'][str(b)]/result['baseline']['blockMedians'][str(b)]-1 for b in (0,1)}
    # Actual holder operands should be finite. If NaNs were sampled, raw NaN
    # payload checksums could vary, so preserve and report instead of hiding it.
    result['crossVariantChecksumsEqual']=all(result[v]['checksums']==result['baseline']['checksums'] for v in plan[0][:4])
    summary['workloads'][name]=result
(capture/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:{v:round(x['changeFraction']*100,3) for v,x in val.items() if isinstance(x,dict) and 'changeFraction'in x} for k,val in summary['workloads'].items()},indent=2))
