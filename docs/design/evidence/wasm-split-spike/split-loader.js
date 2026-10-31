// wasm-split loader for the wasm-bindgen Node glue (prototype). Replaces the
// glue's single `new WebAssembly.Instance(...)`. The primary module imports one
// `placeholder.<slot>` function per deferred function; primary code reaches
// deferred functions through `call_indirect` on the placeholder table, so a
// placeholder runs only while the secondary is not instantiated. Instantiating
// the secondary overwrites those slots with the real functions.
const __split = (() => {
  const meta = JSON.parse(require('fs').readFileSync(`${__dirname}/remus_wasm_split.json`, 'utf8'));
  const mode = process.env.REMUS_SPLIT_MODE || 'lazy-sync';
  const state = { loaded: false, loading: null, placeholderCalls: 0, loadMs: null };
  let primaryExports;
  const secondaryPath = `${__dirname}/${meta.secondary}`;
  function finish(module) {
    if (state.loaded) return;
    const t0 = performance.now();
    new WebAssembly.Instance(module, { primary: primaryExports });
    state.loaded = true;
    state.loadMs = (state.loadMs ?? 0) + performance.now() - t0;
  }
  function loadSync() {
    if (state.loaded) return;
    const t0 = performance.now();
    const module = new WebAssembly.Module(require('fs').readFileSync(secondaryPath));
    state.loadMs = performance.now() - t0;
    finish(module);
  }
  function preload() {
    if (state.loaded) return Promise.resolve();
    if (!state.loading) {
      state.loading = require('fs')
        .promises.readFile(secondaryPath)
        .then((bytes) => WebAssembly.compile(bytes))
        .then(finish);
    }
    return state.loading;
  }
  const placeholders = new Proxy(
    {},
    {
      get(_, key) {
        const slot = Number(key);
        return (...args) => {
          state.placeholderCalls++;
          if (mode === 'throw') {
            throw new Error(
              `remus: deferred kernel code (slot ${slot}) called before the secondary module loaded`,
            );
          }
          loadSync();
          return primaryExports[meta.placeholderTable].get(slot)(...args);
        };
      },
    },
  );
  function instantiate(module, imports) {
    const names = WebAssembly.Module.imports(module)
      .filter((i) => i.module === 'placeholder')
      .map((i) => i.name);
    imports.placeholder = Object.fromEntries(names.map((n) => [n, placeholders[n]]));
    const instance = new WebAssembly.Instance(module, imports);
    primaryExports = instance.exports;
    return instance;
  }
  return { instantiate, loadSync, preload, state, mode };
})();
