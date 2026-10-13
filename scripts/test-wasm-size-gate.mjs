import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { once } from 'node:events';
import { mkdirSync, mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { createServer } from 'node:http';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { test } from 'node:test';
import { fileURLToPath } from 'node:url';
import {
  GROWTH_REVIEW_BYTES,
  MODULES,
  evaluateGate,
  formatMarkdown,
  githubPrBody,
  isValidDeclaration,
  sizeDeclarations,
  stripHtmlComments,
} from './wasm-size-gate.mjs';

const SCRIPT = fileURLToPath(new URL('./wasm-size-gate.mjs', import.meta.url));
const POLICY_SCRIPT = fileURLToPath(new URL('./wasm-size-policy.mjs', import.meta.url));
const KIB = 1024;
const LINE = 'WASM-Size: +120 KiB raw, curve projection, Tier: core (writes the journal)';
const codes = (result) => result.failures.map((failure) => failure.code);

// Reports shaped like `wasm-size-policy.mjs --json`; the gate reads only these fields.
function report(rawBytes, { failures = [], warnings = [] } = {}) {
  return { metrics: { rawBytes, gzipBytes: Math.round(rawBytes * 0.35), brotliBytes: Math.round(rawBytes * 0.25) }, failures, warnings };
}
const BASE = report(8_000_000);
const grown = (by, options) => report(BASE.metrics.rawBytes + by, options);
const gate = (pr, body = '', base = BASE) => evaluateGate({ base, pr, body });

test('growth at the threshold passes without a declaration, one byte more fails', () => {
  assert.equal(GROWTH_REVIEW_BYTES, 64 * KIB);
  assert.deepEqual(codes(gate(grown(GROWTH_REVIEW_BYTES))), []);
  const over = gate(grown(GROWTH_REVIEW_BYTES + 1));
  assert.equal(over.pass, false);
  assert.deepEqual(codes(over), ['growth-undeclared']);
  assert.match(over.failures[0].message, /starts with `WASM-Size:`/);
});

test('growth with a WASM-Size line naming a tier passes', () => {
  const result = gate(grown(120 * KIB), `## Change\n\nAdds curve projection.\n\n${LINE}\n`);
  assert.equal(result.pass, true);
  assert.deepEqual(result.validDeclarations, [LINE]);
  for (const tier of ['core', 'native', 'lazy:io', 'lazy:remus-wasm-sketch', 'Core']) {
    assert.equal(gate(grown(120 * KIB), `WASM-Size: +100 KiB raw, thing, Tier: ${tier}`).pass, true, tier);
  }
});

test('a WASM-Size line inside an HTML comment does not count', () => {
  const body = `Intro\n<!--\n${LINE}\n-->\nOutro\n`;
  assert.deepEqual(sizeDeclarations(body), []);
  assert.deepEqual(codes(gate(grown(120 * KIB), body)), ['growth-undeclared']);
  // The PR template's guidance mentions the prefix inside a comment.
  const template =
    '## WASM module tier\n\n<!-- A new capability declares its tier: core, lazy:<module> or native.\n' +
    'add a line that starts with "WASM-Size:" and names the capability and\n' +
    'its tier, for example "WASM-Size: +120 KiB raw, curve projection, Tier: core\n' +
    '(writes the journal)". Write "None" when there is no new capability. -->\n';
  assert.deepEqual(codes(gate(grown(120 * KIB), template)), ['growth-undeclared']);
  // An unterminated comment hides the rest of the description, as on GitHub.
  assert.deepEqual(sizeDeclarations(`<!-- open\n${LINE}\n`), []);
  // A line after a closed comment still counts, also on the comment's last line.
  assert.deepEqual(sizeDeclarations(`<!-- note -->\n${LINE}`), [LINE]);
  assert.deepEqual(sizeDeclarations(`<!-- note -->${LINE}`), [LINE]);
  assert.equal(stripHtmlComments('a<!-- x -->b<!-- y\nz -->c'), 'abc');
});

test('only lines that start with the prefix count', () => {
  for (const body of [
    `  ${LINE}`,
    `- ${LINE}`,
    `> ${LINE}`,
    `See ${LINE}`,
    LINE.replace('WASM-Size:', 'wasm-size:'),
    LINE.replace('WASM-Size:', 'WASM-Size :'),
  ]) {
    assert.deepEqual(sizeDeclarations(body), [], body);
    assert.deepEqual(codes(gate(grown(120 * KIB), body)), ['growth-undeclared'], body);
  }
  assert.deepEqual(sizeDeclarations(`first\r\n${LINE}\r\nlast`), [LINE], 'CRLF descriptions');
  assert.deepEqual(sizeDeclarations(null), []);
});

test('a WASM-Size line without a module tier does not satisfy the rule', () => {
  for (const line of [
    'WASM-Size:',
    'WASM-Size:   ',
    'WASM-Size: +120 KiB raw, curve projection',
    'WASM-Size: Tier: kernel',
    // The tier token must end: end of line, whitespace, `(`, `,` or `;`.
    'WASM-Size: +120 KiB raw, curve projection, Tier: corejunk',
    'WASM-Size: +120 KiB raw, curve projection, Tier: native2',
    'WASM-Size: +120 KiB raw, curve projection, Tier: lazy:',
    'WASM-Size: +120 KiB raw, curve projection, Tier: lazy:io/x',
  ]) {
    assert.equal(isValidDeclaration(line), false, line);
    assert.deepEqual(codes(gate(grown(120 * KIB), line)), ['growth-declaration-incomplete'], line);
  }
  assert.equal(isValidDeclaration(LINE), true);
});

test('a WASM-Size line must name a capability besides the tier and the size', () => {
  for (const line of [
    'WASM-Size: Tier: core',
    'WASM-Size: Tier: core (writes the journal)',
    'WASM-Size: +120 KiB raw, Tier: core',
    'WASM-Size: +120 KiB, Tier: lazy:io (arena documents)',
    'WASM-Size: 70000 B raw; Tier: native',
    'WASM-Size: ok, Tier: core',
  ]) {
    assert.equal(isValidDeclaration(line), false, line);
    assert.deepEqual(codes(gate(grown(120 * KIB), line)), ['growth-declaration-incomplete'], line);
  }
  for (const line of [
    // The AGENTS.md / skill example and the PR template example.
    'WASM-Size: +120 KiB raw, curve projection (P-Class 7.4), Tier: core (writes the journal)',
    'WASM-Size: +120 KiB raw, curve projection, Tier: core (writes the journal)',
    'WASM-Size: Tier: core (writes the journal), curve projection',
    'WASM-Size: +80 KiB raw, STEP import repair, Tier: lazy:remus-wasm-repair',
    'WASM-Size: +80 KiB raw, sketch solver, Tier: lazy:remus_wasm_sketch;',
    'WASM-Size: +80 KiB raw; mesh export; Tier: native',
    'WASM-Size: blend reconstruction Tier: CORE',
  ]) {
    assert.equal(isValidDeclaration(line), true, line);
    assert.equal(gate(grown(120 * KIB), line).pass, true, line);
  }
});

test('a shrinking or unchanged kernel passes without a declaration', () => {
  for (const by of [-200 * KIB, -1, 0, 1]) {
    const result = gate(grown(by));
    assert.equal(result.pass, true, String(by));
    assert.equal(result.delta.rawBytes, by);
  }
  assert.match(formatMarkdown(gate(grown(-2 * KIB))), /Kernel shrank by 2\.0 KiB raw/);
});

test('a PR kernel over a hard limit fails, even when it shrinks or declares its growth', () => {
  const hard = { code: 'kernel-gzip-hard', metric: 'gzip bytes', bytes: 3_700_000, budgetBytes: 3_670_016, reason: 'Exact geometry kernel exceeds its gzip hard limit' };
  for (const [by, body] of [[-10 * KIB, ''], [0, ''], [200 * KIB, LINE]]) {
    const result = gate(grown(by, { failures: [hard] }), body);
    assert.equal(result.pass, false);
    assert.deepEqual(codes(result), ['kernel-gzip-hard']);
    assert.match(result.failures[0].message, /3,700,000 B exceed the hard limit 3,670,016 B/);
  }
  const both = gate(grown(200 * KIB, { failures: [hard] }));
  assert.deepEqual(codes(both), ['kernel-gzip-hard', 'growth-undeclared']);
});

test('review warnings are reported but never fail the gate', () => {
  const review = { code: 'kernel-brotli-review', metric: 'Brotli q11 bytes', bytes: 2_700_000, budgetBytes: 2_621_440, reason: 'Exact geometry kernel requires a compressed-size review' };
  const result = gate(grown(0, { warnings: [review] }));
  assert.equal(result.pass, true);
  assert.deepEqual(result.warnings.map((warning) => warning.code), ['kernel-brotli-review']);
});

test('without a base package the whole kernel counts as growth', () => {
  assert.deepEqual(codes(gate(report(100 * KIB), '', null)), ['growth-undeclared']);
  assert.equal(gate(report(100 * KIB), LINE, null).pass, true);
});

test('the report keeps the comment heading and quotes the declaration safely', () => {
  const body = 'WASM-Size: +1 MiB raw, `odd` thing, Tier: core (edits the session)';
  const markdown = formatMarkdown(gate(grown(1024 * KIB), body), {
    io: { base: 1000, pr: 1500 },
  });
  assert.match(markdown, /^## WASM Binary Size\n/);
  assert.match(markdown, /\| raw \| 8,000,000 B \| 9,048,576 B \| \+1024\.0 KiB \(\+13\.11%\) \|/);
  assert.match(markdown, /Declared: `` WASM-Size: \+1 MiB raw, `odd` thing, Tier: core \(edits the session\) ``/);
  assert.match(markdown, /raw 1,000 B → 1,500 B, \+0\.5 KiB \(\+50\.00%\)/);
  assert.match(formatMarkdown(gate(grown(70 * KIB))), /❌ \*\*growth-undeclared\*\*/);
});

// End to end: the command line runs the real policy script on real files.
function moduleBytes(size) {
  const contents = new Uint8Array(size);
  contents.set([0x00, 0x61, 0x73, 0x6d, 0x01, 0x00, 0x00, 0x00]);
  return contents;
}

function fixture(t, baseSize, prSize) {
  const root = mkdtempSync(join(tmpdir(), 'remus-size-gate-'));
  t.after(() => rmSync(root, { recursive: true, force: true }));
  const write = (file, contents) => {
    mkdirSync(dirname(file), { recursive: true });
    writeFileSync(file, contents);
  };
  const baseDir = join(root, 'base');
  const prDir = join(root, 'pr');
  if (baseSize !== null) write(join(baseDir, MODULES.kernel.base), moduleBytes(baseSize));
  if (prSize !== null) write(join(prDir, MODULES.kernel.pr), moduleBytes(prSize));
  write(join(baseDir, MODULES.io.base), moduleBytes(4096));
  write(join(prDir, MODULES.io.pr), moduleBytes(5120));
  return { root, baseDir, prDir, markdown: join(root, 'report.md') };
}

function run(f, body, ...extra) {
  const bodyFile = join(f.root, 'body.md');
  writeFileSync(bodyFile, body);
  return spawnSync(
    process.execPath,
    [SCRIPT, '--base-dir', f.baseDir, '--pr-dir', f.prDir, '--body-file', bodyFile, '--markdown', f.markdown, ...extra],
    { encoding: 'utf8', env: { ...process.env, GITHUB_ACTIONS: '' } },
  );
}

test('command line: undeclared growth exits 1, a declaration exits 0', (t) => {
  const f = fixture(t, 4096, 4096 + 70 * KIB);
  const missing = run(f, 'No size line here.');
  assert.equal(missing.status, 1, missing.stderr);
  assert.match(missing.stdout, /FAIL growth-undeclared/);
  assert.match(readFileSync(f.markdown, 'utf8'), /growth-undeclared/);
  const declared = run(f, `Body\n${LINE}\n`);
  assert.equal(declared.status, 0, declared.stderr);
  const markdown = readFileSync(f.markdown, 'utf8');
  assert.match(markdown, /Kernel growth of 70\.0 KiB raw is declared/);
  assert.match(markdown, /Translator `remus_wasm_io_bg\.wasm`/);
  // The description never reaches stdout, where it could inject workflow commands.
  assert.doesNotMatch(declared.stdout, /curve projection/);
});

test('command line: shrink passes and a new kernel package counts as growth', (t) => {
  const shrink = run(fixture(t, 4096 + 100 * KIB, 4096), '');
  assert.equal(shrink.status, 0, shrink.stderr);
  const fresh = run(fixture(t, null, 4096 + 70 * KIB), '');
  assert.equal(fresh.status, 1, fresh.stderr);
});

test('command line: a PR kernel over the raw hard limit exits 1', (t) => {
  const policy = JSON.parse(spawnSync(process.execPath, [POLICY_SCRIPT, '--policy'], { encoding: 'utf8' }).stdout);
  const f = fixture(t, policy.rawHardBytes, policy.rawHardBytes + 1);
  const result = run(f, '');
  assert.equal(result.status, 1, result.stderr);
  assert.match(result.stdout, /FAIL kernel-raw-hard/);
  assert.doesNotMatch(result.stdout, /growth-undeclared/);
});

test('command line: a kernel-only artifact (failed build, translator never built) still reports', (t) => {
  const policy = JSON.parse(spawnSync(process.execPath, [POLICY_SCRIPT, '--policy'], { encoding: 'utf8' }).stdout);
  const f = fixture(t, policy.rawHardBytes, policy.rawHardBytes + 1);
  rmSync(join(f.prDir, MODULES.io.pr));
  const result = run(f, '');
  assert.equal(result.status, 1, result.stderr);
  const markdown = readFileSync(f.markdown, 'utf8');
  assert.match(markdown, /kernel-raw-hard/);
  assert.doesNotMatch(markdown, /Translator/);
});

test('command line: missing input is an error, never a verdict', (t) => {
  const f = fixture(t, 4096, null);
  assert.equal(run(f, '').status, 2);
  const notWasm = fixture(t, 4096, 4096);
  writeFileSync(join(notWasm.prDir, MODULES.kernel.pr), 'not wasm');
  const bad = run(notWasm, '');
  assert.equal(bad.status, 2);
  assert.match(bad.stderr, /size policy failed/);
  assert.equal(spawnSync(process.execPath, [SCRIPT], { encoding: 'utf8' }).status, 2);
  assert.equal(run(fixture(t, 4096, 4096), '', '--unknown').status, 2);
});

test('command line: GitHub annotations escape their messages', (t) => {
  const f = fixture(t, 4096, 4096 + 70 * KIB);
  const bodyFile = join(f.root, 'body.md');
  writeFileSync(bodyFile, '');
  const result = spawnSync(process.execPath, [SCRIPT, '--base-dir', f.baseDir, '--pr-dir', f.prDir, '--body-file', bodyFile], {
    encoding: 'utf8',
    env: { ...process.env, GITHUB_ACTIONS: 'true' },
  });
  assert.equal(result.status, 1, result.stderr);
  assert.match(result.stdout, /^::error title=WASM size \(growth-undeclared\)::The kernel grew by 70\.0 KiB raw/m);
});

test('the live PR description wins over the event copy, which stays the fallback', async (t) => {
  const seen = [];
  const server = createServer((request, response) => {
    seen.push({ url: request.url, auth: request.headers.authorization });
    if (request.url === '/repos/o/r/pulls/7') {
      response.writeHead(200, { 'content-type': 'application/json' });
      response.end(JSON.stringify({ body: `edited\n${LINE}` }));
    } else {
      response.writeHead(500);
      response.end();
    }
  });
  server.listen(0, '127.0.0.1');
  await once(server, 'listening');
  t.after(() => server.close());
  const api = `http://127.0.0.1:${server.address().port}`;
  const env = { GITHUB_API_URL: api, GITHUB_REPOSITORY: 'o/r', PR_NUMBER: '7', GH_TOKEN: 'token-value', PR_BODY: 'event copy' };
  assert.equal(await githubPrBody(env), `edited\n${LINE}`);
  assert.deepEqual(seen, [{ url: '/repos/o/r/pulls/7', auth: 'Bearer token-value' }]);
  assert.equal(await githubPrBody({ ...env, PR_NUMBER: '8' }), 'event copy', 'API error');
  assert.equal(await githubPrBody({ ...env, PR_NUMBER: '7; rm' }), 'event copy', 'malformed number');
  assert.equal(await githubPrBody({ ...env, GITHUB_REPOSITORY: '../x' }), 'event copy', 'malformed repository');
  assert.equal(await githubPrBody({ PR_BODY: 'event copy' }), 'event copy', 'no API context');
  assert.equal(await githubPrBody({}), '');
});
