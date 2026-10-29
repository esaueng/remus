"""Check the portable evidence and recompute process medians without builds."""
from pathlib import Path
import gzip,hashlib,json,math,statistics

root=Path(__file__).resolve().parent
sha=lambda b:hashlib.sha256(b).hexdigest()
def original_bytes(path):
    if path.exists():return path.read_bytes()
    encoded=json.loads(path.with_name(path.name+'.json').read_text())
    raw=encoded['payload'].encode('utf8')
    assert encoded['originalFile']==path.name
    assert len(raw)==encoded['originalBytes']
    assert sha(raw)==encoded['originalSha256']
    return raw
manifest=json.loads((root/'manifest.json').read_text())
for name,receipt in manifest['files'].items():
    raw=(root/name).read_bytes();assert len(raw)==receipt['bytes'];assert sha(raw)==receipt['sha256'],name
raw=gzip.decompress((root/'inputs/retained-observed-inputs.json.gz').read_bytes())
assert sha(raw)==json.loads((root/'inputs/input-receipt.json').read_text())['uncompressedSha256']
inputs=json.loads(raw);assert sum(int(v)for k,v in inputs['total']['counts'].items()if k!='total')==int(inputs['total']['counts']['total'])
modules=json.loads((root/'proofs/modules-receipt.json').read_text())
c0=json.loads((root/'proofs/range-c0-receipt.json').read_text())
for variant,receipt in modules['variants'].items():
    assert sha((root/'source'/variant/'fma_body.rs').read_bytes())==receipt['fma_source_sha256']
    assert sha((root/'source/wrapper.rs').read_bytes())==receipt['wrapper_sha256']
    assert sha(original_bytes(root/'proofs'/(variant+'-native-tests.log')))==receipt['existing_native_oracle_log_sha256']
selector=json.loads((root/'proofs/proof-receipt.json').read_text())
assert sha(original_bytes(root/'proofs'/selector['native_log']))==selector['native_log_sha256']
for label,digest in c0['testLogs'].items():
    assert sha(original_bytes(root/'proofs'/('range-c0-tests-'+label+'.log')))==digest
assert sha((root/'source/range-c0/fma_body.rs').read_bytes())==c0['sourceSha256']
encoded=json.loads((root/'source/candidate-range-c0.patch.json').read_text())
patch=encoded['payload'].encode('utf8')
assert len(patch)==encoded['originalBytes']
assert sha(patch)==encoded['originalSha256']==c0['candidatePatchSha256']
historical=json.loads((root/'proofs/reproduction-check.json').read_text())
assert sha((root/'proofs/reproduce-before-patch-encoding.py').read_bytes())==historical['reproducerSha256']
for variant in ['wasm-validation.json','wasm-validation-range-c0.json']:
    receipt=json.loads((root/'proofs'/variant).read_text())
    script='validate-range-c0.cjs'if variant=='wasm-validation-range-c0.json'else'validate.cjs'
    assert sha((root/'source'/script).read_bytes())==receipt['driverSha256']
for experiment in ['four-variants','range-c0']:
    d=root/experiment;processes=json.loads((d/'processes.json').read_text());summary=json.loads((d/'summary.json').read_text())
    assert sha((d/'processes.json').read_bytes())==summary['processesSha256']
    producer='run_balanced_micro.py'if experiment=='four-variants'else'run_paired_micro.py'
    assert sha((root/'source'/producer).read_bytes())==processes['driverSha256']
    for p in processes['processes']:
        assert sha((d/p['file']).read_bytes())==p['sha256']
        data=json.loads((d/p['file']).read_text());assert data['affinity'].split(':',1)[1].strip()=='31'
        assert data['driverSha256']==sha((root/'source/micro_process.cjs').read_bytes())
        assert data['roundsSha256']==sha((d/'rounds.json').read_bytes())
        assert data['manifestSha256']==sha((root/'inputs/workloads-manifest.json').read_bytes())
        expected=c0['optimizedWasmSha256']if p['variant']=='range-c0'else modules['variants'][p['variant']]['optimized_wasm_sha256']
        assert data['moduleSha256']==expected
    for name,results in summary['workloads'].items():
        for variant,result in results.items():
            if not isinstance(result,dict):continue
            medians=[];blocks={0:[],1:[]}
            for p in processes['processes']:
                if p['variant']!=variant:continue
                data=json.loads((d/p['file']).read_text())
                rows=[r for r in data['samples']if r['workload']==name and not r['warmup']]
                assert len(rows)==6
                median=statistics.median(int(r['durationNs'])/r['operations']for r in rows)
                medians.append(median);blocks[p['block']].append(median)
            assert medians==result['processMedianNsPerOperation']
            assert statistics.median(medians)==result['medianNsPerOperation']
            for block,values in blocks.items():assert statistics.median(values)==result['blockMedians'][str(block)]
            baseline=results['baseline']['medianNsPerOperation']
            assert math.isclose(result['changeFraction'],statistics.median(medians)/baseline-1,rel_tol=0,abs_tol=1e-15)
        assert results['crossVariantChecksumsEqual']
print('Verified evidence hashes, lossless bounded input, 24 process receipts, and every retained process median.')
