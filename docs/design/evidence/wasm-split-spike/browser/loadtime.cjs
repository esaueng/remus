// Fresh browser context per run: compileStreaming + instantiate (stub imports), CPU throttle optional.
const http = require('http'),
  fs = require('fs');
const { chromium } = require('playwright');
const files = {
  base: 'variants/base/remus_wasm_bg.wasm',
  'all-primary': 'split/all.primary.wasm',
  'perf-primary': 'split/perf.primary.wasm',
  'all-secondary': 'split/all.secondary.wasm',
  'perf-secondary': 'split/perf.secondary.wasm',
};
const server = http
  .createServer((req, res) => {
    const f = files[req.url.slice(1)];
    if (f) {
      res.writeHead(200, { 'content-type': 'application/wasm', 'cache-control': 'no-store' });
      return fs.createReadStream(`/tmp/claude-0/split/${f}`).pipe(res);
    }
    res.writeHead(200, { 'content-type': 'text/html' });
    res.end('<!doctype html>');
  })
  .listen(0, async () => {
    const b = await chromium.launch();
    const runs = Number(process.argv[2] || 10);
    for (const throttle of [1, 4]) {
      for (const name of Object.keys(files)) {
        const comp = [],
          inst = [];
        for (let i = 0; i < runs; i++) {
          const ctx = await b.newContext();
          const p = await ctx.newPage();
          await p.goto(`http://127.0.0.1:${server.address().port}/`);
          const cdp = await ctx.newCDPSession(p);
          await cdp.send('Emulation.setCPUThrottlingRate', { rate: throttle });
          const r = await p.evaluate(async (name) => {
            const resp = await fetch(name);
            const bytes = await resp.arrayBuffer(); // download excluded
            const t0 = performance.now();
            const m = await WebAssembly.compile(bytes);
            const t1 = performance.now();
            const imports = {};
            for (const imp of WebAssembly.Module.imports(m)) {
              if (imp.kind !== 'function') continue; // secondary's non-function imports: skip instantiate
              (imports[imp.module] ??= {})[imp.name] = () => {};
            }
            let t2 = null;
            if (!WebAssembly.Module.imports(m).some((i) => i.kind !== 'function')) {
              await WebAssembly.instantiate(m, imports);
              t2 = performance.now();
            }
            return [t1 - t0, t2 && t2 - t1];
          }, name);
          comp.push(r[0]);
          if (r[1] != null) inst.push(r[1]);
          await ctx.close();
        }
        const med = (a) => (a.length ? a.sort((x, y) => x - y)[a.length >> 1].toFixed(1) : '-');
        const rng = (a) => (a.length ? `${a[0].toFixed(1)}–${a[a.length - 1].toFixed(1)}` : '-');
        console.log(
          `cpu x${throttle} ${name}: compile median ${med(comp)} ms (${rng(comp)}), instantiate median ${med(inst)} ms (${rng(inst)})`,
        );
      }
    }
    await b.close();
    server.close();
  });
