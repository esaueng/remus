"""Build a split variant package dir: variants/<name>/ from a base variant + primary/secondary."""
import json, pathlib, re, shutil, subprocess, sys
S = pathlib.Path('/tmp/claude-0/split')
name, primary, secondary = sys.argv[1], pathlib.Path(sys.argv[2]), pathlib.Path(sys.argv[3])
out = S / 'variants' / name
if out.exists(): shutil.rmtree(out)
shutil.copytree(S / 'variants/base', out)
shutil.copy(primary, out / 'remus_wasm_bg.wasm')
shutil.copy(secondary, out / 'remus_wasm_bg.secondary.wasm')
wat = subprocess.run(['wasm-tools', 'print', str(secondary)], capture_output=True, text=True, check=True).stdout
# The secondary's first active segment fills the placeholder table.
tables = dict(re.findall(r'\(import "primary" "([^"]+)" \(table \(;(\d+);\)', wat))
seg = re.search(r'\(elem \(;\d+;\) (?:\(table (\d+)\) )?\(i32\.const 0\) func', wat)
idx = seg.group(1) or '0'
table = {v: k for k, v in tables.items()}[idx]
(out / 'remus_wasm_split.json').write_text(json.dumps({'secondary': 'remus_wasm_bg.secondary.wasm', 'placeholderTable': table}))
glue = (out / 'remus_wasm_node.cjs').read_text()
old = 'let wasmInstance = new WebAssembly.Instance(wasmModule, __wbg_get_imports());\n'
assert glue.count(old) == 1
loader = (S / 'scripts/split-loader.js').read_text()
new = loader + 'let wasmInstance = __split.instantiate(wasmModule, __wbg_get_imports());\n'
glue = glue.replace(old, new)
start = 'wasm.__wbindgen_start();\n'
assert glue.count(start) == 1
glue = glue.replace(start, start + "if (__split.mode === 'eager') __split.loadSync();\nexports.__remusSplit = __split;\n")
(out / 'remus_wasm_node.cjs').write_text(glue)
print(out, table)
