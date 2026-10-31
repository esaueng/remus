const http = require('http'),
  fs = require('fs');
const { chromium } = require('playwright');
const files = {
  '/big': 'variants/base/remus_wasm_bg.wasm',
  '/sec': 'variants/split-perf/remus_wasm_bg.secondary.wasm',
};
const server = http
  .createServer((req, res) => {
    if (files[req.url]) {
      res.writeHead(200, { 'content-type': 'application/wasm' });
      return fs.createReadStream(`/tmp/claude-0/split/${files[req.url]}`).pipe(res);
    }
    res.writeHead(200, { 'content-type': 'text/html' });
    res.end('<!doctype html>');
  })
  .listen(0, async () => {
    const b = await chromium.launch();
    const p = await b.newPage();
    await p.goto(`http://127.0.0.1:${server.address().port}/`);
    console.log('chromium', b.version());
    for (const u of ['/sec', '/big']) {
      console.log(
        u,
        await p.evaluate(async (u) => {
          const bytes = new Uint8Array(await (await fetch(u)).arrayBuffer());
          try {
            const t = performance.now();
            new WebAssembly.Module(bytes);
            return `main-thread sync compile OK, ${bytes.length} B, ${(performance.now() - t).toFixed(0)} ms`;
          } catch (e) {
            return `${bytes.length} B: ${e}`;
          }
        }, u),
      );
    }
    await b.close();
    server.close();
  });
