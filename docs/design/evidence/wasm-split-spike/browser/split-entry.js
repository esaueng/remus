// Browser entry for a wasm-split kernel (prototype), replacing wasm-pack's
// bundler entry `import * as wasm from "./remus_wasm_bg.wasm"`: a bundler's
// ESM-integration would try to resolve the primary's `placeholder` imports as
// a JS module. Instantiate by hand instead.
import * as bg from './remus_wasm_bg.js';

export async function init({ primaryUrl, secondaryUrl, placeholderTable, onPlaceholder }) {
  let exports;
  let loaded = false;
  let loading = null;
  const state = { placeholderCalls: 0 };
  const primaryModule = await WebAssembly.compileStreaming(fetch(primaryUrl));
  const placeholder = {};
  for (const { module, name } of WebAssembly.Module.imports(primaryModule)) {
    if (module !== 'placeholder') continue;
    const slot = Number(name);
    placeholder[name] = (...args) => {
      state.placeholderCalls++;
      onPlaceholder(slot); // must load the secondary synchronously or throw
      return exports[placeholderTable].get(slot)(...args);
    };
  }
  const instance = await WebAssembly.instantiate(primaryModule, {
    './remus_wasm_bg.js': bg,
    placeholder,
  });
  exports = instance.exports;
  bg.__wbg_set_wasm(exports);
  exports.__wbindgen_start();
  function finish(module) {
    if (!loaded) {
      new WebAssembly.Instance(module, { primary: exports });
      loaded = true;
    }
  }
  return {
    bg,
    state,
    get loaded() {
      return loaded;
    },
    preload() {
      loading ??= WebAssembly.compileStreaming(fetch(secondaryUrl)).then(finish);
      return loading;
    },
    loadSync(bytes) {
      finish(new WebAssembly.Module(bytes));
    },
  };
}
