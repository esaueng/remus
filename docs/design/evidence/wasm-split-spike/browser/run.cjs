// Serve /tmp/claude-0/split and drive Chromium through browser/test.html.
const http = require('http'),
  fs = require('fs'),
  path = require('path');
const { chromium } = require('playwright');
const root = '/tmp/claude-0/split';
const types = {
  '.html': 'text/html',
  '.js': 'text/javascript',
  '.wasm': 'application/wasm',
  '.json': 'application/json',
};
const server = http
  .createServer((req, res) => {
    const p = path.join(root, decodeURIComponent(new URL(req.url, 'http://x').pathname));
    if (!p.startsWith(root) || !fs.existsSync(p)) {
      res.writeHead(404);
      return res.end();
    }
    res.writeHead(200, { 'content-type': types[path.extname(p)] || 'application/octet-stream' });
    fs.createReadStream(p).pipe(res);
  })
  .listen(0, async () => {
    const port = server.address().port;
    const browser = await chromium.launch();
    for (const [v, mode] of process.argv.slice(2).map((a) => a.split(':'))) {
      const page = await browser.newPage();
      const errs = [];
      page.on('pageerror', (e) => errs.push(String(e).slice(0, 300)));
      await page.goto(`http://127.0.0.1:${port}/browser/test.html?v=variants/${v}&mode=${mode}`);
      try {
        await page.waitForFunction(() => document.title === 'done', null, { timeout: 60000 });
      } catch {}
      console.log(
        v,
        mode,
        JSON.stringify(await page.evaluate(() => window.result)),
        errs.join(' | '),
      );
      await page.close();
    }
    await browser.close();
    server.close();
  });
