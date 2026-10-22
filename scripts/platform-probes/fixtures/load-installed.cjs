const assert = require('node:assert/strict');
const { createHash } = require('node:crypto');
const { readFileSync, realpathSync } = require('node:fs');
const { dirname, isAbsolute, join, relative, sep } = require('node:path');

const root = realpathSync(process.argv[2]);
function member(path) {
  const actual = realpathSync(path);
  const tail = relative(root, actual);
  assert.ok(tail && !tail.startsWith('..' + sep) && tail !== '..' && !isAbsolute(tail));
  return actual;
}
const manifest = JSON.parse(readFileSync(join(root, 'package.json'), 'utf8'));
const entry = member(join(root, manifest.main));
const wasm = member(join(dirname(entry), 'remus_windows_arm_tooling_probe_bg.wasm'));
const bytes = readFileSync(wasm);
assert.deepEqual([...bytes.subarray(0, 8)], [0, 97, 115, 109, 1, 0, 0, 0]);
assert.equal(WebAssembly.validate(bytes), true);
const kernel = require(entry);
const result = kernel.probe_sum(17, 25);
assert.equal(result, 42);
console.log(JSON.stringify({ schemaVersion: 1, platform: process.platform, arch: process.arch,
  nodeVersion: process.version, nodePath: realpathSync(process.execPath), packageName: manifest.name,
  packageVersion: manifest.version, packageRoot: root, entry, wasmPath: wasm, wasmSha256: createHash('sha256').update(bytes).digest('hex'), result }));
