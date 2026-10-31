const m = require(`${process.argv[2]}/remus_wasm_node.cjs`);
const s = m.__remusSplit;
const k = new m.RemusKernel();
const b = k.makeBox(10, 10, 10);
let c;
let err;
try {
  c = k.makeCylinder(2, 20);
} catch (e) {
  err = String(e).slice(0, 120);
}
console.log('first cut:', err);

s.preload().then(() => {
  console.log('panic msg:', m.lastPanicMessage());
  try {
    c = k.makeCylinder(2, 20);
    const r = k.cut(b, c);
    console.log('retry cut ok, volume', k.volume(r, 0.001), 'solids', k.solidCount?.());
  } catch (e) {
    console.log('retry ERR', String(e).slice(0, 300));
  }
  try {
    const k2 = new m.RemusKernel();
    console.log('new kernel ok', k2.volume(k2.makeBox(1, 1, 1), 0.01));
  } catch (e) {
    console.log('k2 ERR', String(e).slice(0, 200));
  }
});
