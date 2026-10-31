const pkg = process.argv[2];
const m = require(`${pkg}/remus_wasm_node.cjs`);
const k = new m.RemusKernel();
const s = m.__remusSplit;
const b = k.makeBox(10, 10, 10);
console.log('after box: loaded', s?.state.loaded, 'placeholderCalls', s?.state.placeholderCalls);
try {
  const c = k.makeCylinder(2, 20);
  const r = k.cut(b, c);
  const f = k.volume(r, 0.001);
  console.log('cut volume', f, 'expected', 1000 - Math.PI * 4 * 10);
  const mesh = k.tessellateSolid ? 'n/a' : '';
} catch (e) {
  console.log('ERR', String(e).slice(0, 200));
}
console.log(
  'end: loaded',
  s?.state.loaded,
  'placeholderCalls',
  s?.state.placeholderCalls,
  'loadMs',
  s?.state.loadMs?.toFixed(1),
);
// usable after an error?
try {
  console.log('box after', k.volume(k.makeBox(1, 1, 1), 0.01));
} catch (e) {
  console.log('ERR2', String(e).slice(0, 200));
}
