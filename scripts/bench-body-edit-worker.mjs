import * as bindings from '../crates/wasm/pkg/remus_wasm_bg.js';
import { runBodyEditBenchmark } from './bench-body-edit.mjs';
try {
  const { instance } = await WebAssembly.instantiateStreaming(
    fetch('../crates/wasm/pkg/remus_wasm_bg.wasm'),
    { './remus_wasm_bg.js': bindings },
  );
  bindings.__wbg_set_wasm(instance.exports);
  instance.exports.__wbindgen_start();
  const { version } = await (await fetch('../crates/wasm/pkg/package.json')).json();
  postMessage({runtime: navigator.userAgent, packageVersion: version,
    results: runBodyEditBenchmark(bindings.BrepKernel)});
} catch(error) {postMessage({error:String(error)});}
