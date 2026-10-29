"""Rebuild rejected standalone experiments in a new temporary output folder."""
from pathlib import Path
import argparse,gzip,hashlib,json,os,shutil,subprocess

root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--wasm-opt',type=Path,required=True,help='Binaryen117 executable matching the retained hash')
args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
receipts=json.loads((root/'proofs/modules-receipt.json').read_text())
assert sha(args.wasm_opt)==receipts['tools']['wasm_opt_sha256'],'wasm-opt byte identity changed'
assert subprocess.check_output(['rustc','-Vv'],text=True).strip()==receipts['tools']['rustc_version']
sysroot=subprocess.check_output(['rustc','--print','sysroot'],text=True).strip()
assert sha(Path(sysroot)/'bin/rustc')==receipts['tools']['actual_rustc_executable_sha256']
for name,digest in receipts['tools']['compiler_driver_libraries'].items():
    assert sha(Path(sysroot)/'lib'/name)==digest
for source in (root/'source').iterdir():
    if source.is_dir():shutil.copytree(source,out/source.name)
    else:shutil.copyfile(source,out/source.name)
cargo=Path(os.environ.get('CARGO_HOME',str(Path.home()/'.cargo')))
cpu=os.environ.get('REMUS_BUILD_CPUS')
def run(command,**kwargs):
    if cpu:command=['taskset','-c',cpu]+command
    subprocess.run(command,check=True,**kwargs)
for variant in ['baseline','zero-first','bit-range','both','range-c0']:
    d=out/variant;shutil.copyfile(out/'wrapper.rs',d/'wrapper.rs')
    run(['rustc','--edition=2024','--test','-Dwarnings','-C','opt-level=2',str(d/'fma_body.rs'),'-o',str(d/'native-tests')])
    with(d/'native-tests.log').open('wb')as log:run([str(d/'native-tests')],stdout=log,stderr=subprocess.STDOUT)
    command=['rustc','--edition=2024','--target','wasm32-unknown-unknown','--crate-name','dispatch_micro','--crate-type','cdylib','-Dwarnings','-C','opt-level=3','-C','lto=fat','-C','codegen-units=1','-C','overflow-checks=true','-C','panic=abort','-C','target-feature=+simd128',f'--remap-path-prefix={d}=/dispatch',f'--remap-path-prefix={cargo}=/cargo',f'--remap-path-prefix={sysroot}=/rust',str(d/'wrapper.rs'),'-o',str(d/'raw.wasm')]
    run(command)
    run([str(args.wasm_opt.resolve()),str(d/'raw.wasm'),'-Oz','--enable-simd','-o',str(d/'optimized.wasm')],env={**os.environ,'BINARYEN_CORES':'2'})
    expected=(json.loads((root/'proofs/range-c0-receipt.json').read_text())['optimizedWasmSha256']if variant=='range-c0'else receipts['variants'][variant]['optimized_wasm_sha256'])
    assert sha(d/'optimized.wasm')==expected,'standalone module changed: '+variant
for source,binary in [('oracle_corpus.rs','oracle-corpus'),('oracle_inputs.rs','oracle-inputs')]:
    run(['rustc','--edition=2024','-Dwarnings','-C','opt-level=3',str(out/source),'-o',str(out/binary)])
with(out/'oracle-corpus.bin').open('wb')as corpus:run([str(out/'oracle-corpus')],stdout=corpus)
run(['node',str(out/'validate.cjs')]);run(['node',str(out/'validate-range-c0.cjs')])
raw=gzip.decompress((root/'inputs/retained-observed-inputs.json.gz').read_bytes())
assert hashlib.sha256(raw).hexdigest()==json.loads((root/'inputs/input-receipt.json').read_text())['uncompressedSha256']
(out/'actual-combined.json').write_bytes(raw)
run(['node',str(out/'prepare_workloads.cjs'),str(out/'actual-combined.json'),str(out/'actual-workloads')])
expected=json.loads((root/'inputs/workloads-manifest.json').read_text())
actual=json.loads((out/'actual-workloads/manifest.json').read_text())
for old,new in zip(expected['workloads'],actual['workloads'],strict=True):
    for field in ['name','records','counts','allocated','inputSha256','expectedSha256']:assert old[field]==new[field],(old['name'],field)
print('Rebuilt all five modules, native-oracle correctness, and exact sampled input/result bytes. No timing was run.')
