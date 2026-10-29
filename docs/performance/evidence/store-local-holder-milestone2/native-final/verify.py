"""Verify portable native correctness evidence without executing the kernel."""
from pathlib import Path
import argparse,hashlib,json,math,re

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--repo',type=Path);args=parser.parse_args()
sha=lambda raw:hashlib.sha256(raw).hexdigest()
read=lambda name:json.loads((root/name).read_text())
manifest=read('manifest.json')
for name,receipt in manifest['files'].items():
    raw=(root/name).read_bytes();assert len(raw)==receipt['bytes'];assert sha(raw)==receipt['sha256'],name
summary=read('qualification.json');source=summary['source'];assert source['head']=='7364ab7be5e598b7c868bd63239dc831cbceb933'and source['status']==''
files=summary['source_files_sha256'];assert files['crates/math/src/fma.rs']=='d0f8602fc3eadf802223a421844d4fc558445a38f6fd6625994662c18fcb8684'
assert files['crates/algo/src/classifier/torus_patch.rs']=='6439e3d7b658cac1f32c6339f4c1a96347d7030d251e4f3fc33ccad5842974c7'
for key in ['complex_smoke','approximation_census','workspace_tests']:
    receipt=summary[key];assert sha((root/receipt['receipt']).read_bytes())==receipt['receipt_sha256']
workspace=read('workspace-tests.json');assert workspace['source']==source and workspace['git_tree']==summary['git_tree']and workspace['files_sha256']==files
for run in workspace['runs']:
    assert run['exit_code']==0 and run['source_before']==source and run['source_after']==source
assert workspace['runs'][0]['tests_passed']==workspace['runs'][0]['tests_run']==9237
assert workspace['runs'][0]['tests_skipped']==34
assert workspace['runs'][1]['passed']==9 and workspace['runs'][1]['failed']==0
assert workspace['runs'][2]['passed']==33 and workspace['runs'][2]['failed']==workspace['runs'][2]['ignored']==0
census=read('census-qualification.json');assert census['source']==source and census['status']=='passed'and census['execution_exit_code']==0
assert census['rows']==len(census['all_normalized_rows'])==52 and census['matched_committed_snapshot']
assert sha((root/'census.normalized.tsv').read_bytes())==summary['approximation_census']['normalized_rows_sha256']
assert (root/'census.normalized.tsv').read_text().splitlines()==census['all_normalized_rows']
complex=read('complex-qualification.json');assert complex['source']==source and complex['status']=='passed'
assert complex['cases']==14 and len(complex['sample_metrics'])==28
assert complex['settings']=={'store_local_requested':False,'content_memos':True,'phase_clock':False,'perf_counters':False}
assert sum(row['warmup']for row in complex['sample_metrics'])==14
assert len({row['case_id']for row in complex['sample_metrics']})==14
for row in complex['sample_metrics']:
    assert row['validation']=='passed'and row['settings']==complex['settings']
    m=row['metrics'];assert m['input_operands_preserved']and m['material_probes']>=2
    assert math.isfinite(m['volume'])and abs(m['volume']-m['expected_volume'])<=m['volume_bound']
    assert math.isclose(m['volume_bound'],1e-8*max(abs(m['expected_volume']),1),rel_tol=2e-15)
for name,digest in complex['harness_sha256'].items():assert files[name]==digest
for name,digest in complex['counter_source_sha256'].items():assert files[name]==digest
retry=read('compiler-retry.json');comparison=read('compiler-retry-comparison.json')
assert retry['source']==comparison['source']==source and retry['retry_status']=='passed'and retry['retry_exit_code']==0
assert retry['source_profiles_guards_budgets_changed']is False and comparison['status']=='passed'
links=read('receipt-links.json')['recovery']
assert sha((root/links['portableFile']).read_bytes())==links['portableSha256']
assert links['rawLocalReceiptSha256']==complex['compiler_retry']['recovery_receipt_sha256']
assert links['pathRedactedOriginalVerifiedBeforeCopy']
assert complex['compiler_retry']['source_and_profile_changed']is False
assert comparison['retry_status']=='passed14cases28records'and comparison['source_after']==source['head']
assert summary['unqualified_witness']['existing_component_pair_budget']==4096
for filename in ['README.md','report.md']:
    text=(root/filename).read_text()
    for target in re.findall(r'\]\(([^)]+)\)',text):assert(root/target).is_file(),target
private_roots=['/'+name+'/'for name in ['home','tmp']]
for p in root.iterdir():
    if p.is_file():assert not any(prefix in p.read_text()for prefix in private_roots),p.name
if args.repo:
    for name,digest in files.items():assert sha((args.repo/name).read_bytes())==digest,name
print('Verified native source/receipt hashes,9237 workspace passes,9 docs,33 store tests,52 census rows,14 cases/28 semantic records and disclosed compiler retry; no kernel execution.')
