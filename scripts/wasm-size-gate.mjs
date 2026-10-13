#!/usr/bin/env node
/**
 * Per-PR WASM size gate (W1 of docs/design/kernel-wasm-size-architecture.md).
 *
 * Compares the PR's kernel build with the kernel package committed on the
 * base branch and decides two things:
 *
 * 1. The PR kernel stays within the hard limits of the size policy. The
 *    policy is `scripts/wasm-size-policy.mjs`, run here through its `--json`
 *    command line (the same interface `cargo xtask wasm-build` uses), so the
 *    limits live in that one file.
 * 2. A kernel growth above GROWTH_REVIEW_BYTES raw is declared in the PR
 *    description by a line that starts with `WASM-Size:` and names the
 *    capability and its module tier, for example
 *    `WASM-Size: +120 KiB raw, curve projection, Tier: core (writes the journal)`.
 *    HTML comments are removed first, so the PR template's guidance never
 *    counts as a declaration.
 *
 * CI runs this script from a checkout of the BASE commit (trusted code) with
 * the PR's built modules as plain input files; it never runs code from the PR.
 *
 * Usage:
 *   node scripts/wasm-size-gate.mjs --base-dir <checkout> --pr-dir <artifacts>
 *     [--body-file <file> | --github-pr-body] [--markdown <out.md>]
 *
 * `--base-dir` holds the committed packages (crates/wasm/pkg, crates/wasm-io/pkg);
 * `--pr-dir` holds the CI artifacts `pr-remus_wasm.wasm` and `pr-remus_wasm_io.wasm`.
 * `--github-pr-body` reads the current PR description from the GitHub API
 * (GITHUB_API_URL, GITHUB_REPOSITORY, PR_NUMBER, GH_TOKEN), so a re-run sees
 * an edited description, and falls back to the PR_BODY environment variable
 * (the triggering event's copy) when the API is unavailable.
 *
 * Exit status: 0 the gate passes (review warnings allowed), 1 the gate fails,
 * 2 on a usage, input or measurement error.
 */
import { spawnSync } from 'node:child_process';
import { existsSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

/** Kernel growth (raw bytes) above which the PR must carry a `WASM-Size:` line. */
export const GROWTH_REVIEW_BYTES = 64 * 1024;

/** Where each module lives in a base checkout and in the PR artifact directory. */
export const MODULES = Object.freeze({
  kernel: Object.freeze({ base: 'crates/wasm/pkg/remus_wasm_bg.wasm', pr: 'pr-remus_wasm.wasm' }),
  io: Object.freeze({ base: 'crates/wasm-io/pkg/remus_wasm_io_bg.wasm', pr: 'pr-remus_wasm_io.wasm' }),
});

const POLICY_SCRIPT = fileURLToPath(new URL('./wasm-size-policy.mjs', import.meta.url));
const DECLARATION_PREFIX = 'WASM-Size:';
const TIER = /\bTier:\s*(?:core|native|lazy:[A-Za-z0-9_.-]+)/i;
const EXAMPLE = 'WASM-Size: +120 KiB raw, curve projection, Tier: core (writes the journal)';

/**
 * Remove HTML comments the way GitHub renders them: an unterminated `<!--`
 * hides the rest of the description, so it removes the rest here too.
 */
export function stripHtmlComments(text) {
  return text.replace(/<!--[\s\S]*?(?:-->|$)/g, '');
}

/** Every line of the description that starts with `WASM-Size:`, comments removed. */
export function sizeDeclarations(body) {
  return stripHtmlComments(String(body ?? '').replace(/\r\n?/g, '\n'))
    .split('\n')
    .filter((line) => line.startsWith(DECLARATION_PREFIX))
    .map((line) => line.trim());
}

/** A declaration names something after the prefix and a module tier. */
export function isValidDeclaration(line) {
  const rest = line.slice(DECLARATION_PREFIX.length);
  return /\S/.test(rest) && TIER.test(rest);
}

/**
 * Decide the gate from two `wasm-size-policy.mjs --json` reports.
 *
 * `base` is the committed kernel's report, or null when the base has no
 * kernel package (the whole PR kernel then counts as growth). `pr` is the PR
 * kernel's report; its policy `failures` are the hard-limit failures.
 */
export function evaluateGate({ base, pr, body, growthReviewBytes = GROWTH_REVIEW_BYTES }) {
  const failures = pr.failures.map((finding) => ({
    code: finding.code,
    message:
      `${finding.metric} ${bytes(finding.bytes)} exceed the hard limit ${bytes(finding.budgetBytes)}. ` +
      `${finding.reason}.`,
  }));
  const warnings = pr.warnings.map((finding) => ({
    code: finding.code,
    message: `${finding.metric} ${bytes(finding.bytes)} reach the review limit ${bytes(finding.budgetBytes)}. ${finding.reason}.`,
  }));
  const zero = { rawBytes: 0, gzipBytes: 0, brotliBytes: 0 };
  const baseMetrics = base ? base.metrics : zero;
  const delta = {
    rawBytes: pr.metrics.rawBytes - baseMetrics.rawBytes,
    gzipBytes: pr.metrics.gzipBytes - baseMetrics.gzipBytes,
    brotliBytes: pr.metrics.brotliBytes - baseMetrics.brotliBytes,
  };
  const declarations = sizeDeclarations(body);
  const valid = declarations.filter(isValidDeclaration);
  const growthNeedsDeclaration = delta.rawBytes > growthReviewBytes;
  if (growthNeedsDeclaration && valid.length === 0) {
    const growth = `The kernel grew by ${kib(delta.rawBytes)} raw, above the ${kib(growthReviewBytes)} review threshold`;
    const fix =
      `Add a line to the PR description that starts with \`${DECLARATION_PREFIX}\` and names the ` +
      `capability and its module tier (core, lazy:<module> or native), for example ` +
      `\`${EXAMPLE}\`, then re-run the WASM size jobs. The base is the package committed on the ` +
      `base branch, so growth from merged changes whose package refresh has not landed counts too.`;
    failures.push(
      declarations.length === 0
        ? { code: 'growth-undeclared', message: `${growth}, and the PR description has no \`${DECLARATION_PREFIX}\` line. ${fix}` }
        : {
            code: 'growth-declaration-incomplete',
            message: `${growth}, and no \`${DECLARATION_PREFIX}\` line names a module tier (\`Tier: core\`, \`Tier: lazy:<module>\` or \`Tier: native\`). ${fix}`,
          },
    );
  }
  return {
    pass: failures.length === 0,
    base: base ? base.metrics : null,
    pr: pr.metrics,
    delta,
    growthReviewBytes,
    growthNeedsDeclaration,
    declarations,
    validDeclarations: valid,
    failures,
    warnings,
  };
}

function bytes(n) {
  return `${n.toLocaleString('en-US')} B`;
}

function kib(n) {
  return `${(n / 1024).toFixed(1)} KiB`;
}

function mib(n) {
  return `${(n / (1024 * 1024)).toFixed(2)} MiB`;
}

function signed(n, format) {
  return `${n > 0 ? '+' : n < 0 ? '-' : '±'}${format(Math.abs(n))}`;
}

function percent(delta, base) {
  if (!base) return '';
  const value = (delta * 100) / base;
  return ` (${value > 0 ? '+' : ''}${value.toFixed(2)}%)`;
}

/** An inline code span that survives backticks inside the text. */
function codeSpan(text) {
  const longest = Math.max(0, ...(text.match(/`+/g) ?? []).map((run) => run.length));
  const fence = '`'.repeat(longest + 1);
  return `${fence} ${text} ${fence}`;
}

/** The PR comment and job summary. The heading is what the comment job searches for. */
export function formatMarkdown(result, { policy, toolchain, io } = {}) {
  const lines = ['## WASM Binary Size', ''];
  const cell = (value) => (value === null || value === undefined ? '(new)' : bytes(value));
  const row = (label, key, limit) => {
    const baseValue = result.base ? result.base[key] : null;
    const delta = result.delta[key];
    const change = `${signed(delta, kib)}${percent(delta, baseValue)}`;
    lines.push(`| ${label} | ${cell(baseValue)} | ${bytes(result.pr[key])} | ${change} | ${limit} |`);
  };
  lines.push('| Kernel `remus_wasm_bg.wasm` | Base | PR | Delta | Policy |');
  lines.push('|---|---:|---:|---:|---|');
  row('raw', 'rawBytes', policy ? `review ${mib(policy.rawReviewBytes)}, hard ${mib(policy.rawHardBytes)}` : '');
  row('gzip', 'gzipBytes', policy ? `hard ${mib(policy.gzipHardBytes)}` : '');
  row(`Brotli q${policy?.brotliQuality ?? 11}`, 'brotliBytes', policy ? `review ${mib(policy.brotliReviewBytes)}` : '');
  if (io) {
    const ioDelta = io.base === null ? 'new module' : `${signed(io.pr - io.base, kib)}${percent(io.pr - io.base, io.base)}`;
    lines.push('');
    lines.push(
      `Translator \`remus_wasm_io_bg.wasm\` (loaded lazily, outside the kernel policy): ` +
        `raw ${cell(io.base)} → ${bytes(io.pr)}, ${ioDelta}.`,
    );
  }
  lines.push('');
  for (const failure of result.failures) lines.push(`- ❌ **${failure.code}**: ${failure.message}`);
  for (const warning of result.warnings) lines.push(`- ⚠️ ${warning.code}: ${warning.message}`);
  if (result.pass) {
    const growth = result.delta.rawBytes;
    let verdict;
    if (result.growthNeedsDeclaration) verdict = `Kernel growth of ${kib(growth)} raw is declared.`;
    else if (growth > 0) verdict = `Kernel grew by ${kib(growth)} raw, within the ${kib(result.growthReviewBytes)} review threshold.`;
    else if (growth === 0) verdict = 'Kernel size unchanged.';
    else verdict = `Kernel shrank by ${kib(-growth)} raw.`;
    lines.push(`- ✅ Within the hard limits. ${verdict}`);
  }
  for (const declaration of result.validDeclarations) lines.push(`- Declared: ${codeSpan(declaration)}`);
  lines.push('');
  const node = toolchain ? `Node ${toolchain.node}, zlib ${toolchain.zlib}, Brotli ${toolchain.brotli}` : 'Node';
  lines.push(
    `<sub>Measured by the base branch's \`scripts/wasm-size-policy.mjs\` (${node}) against the package ` +
      `committed on the base branch. A kernel growth above ${kib(result.growthReviewBytes)} raw needs a ` +
      `\`${DECLARATION_PREFIX}\` line in the PR description.</sub>`,
  );
  return `${lines.join('\n')}\n`;
}

/** Run the policy's `--json` command line on one module and return its report. */
export function runPolicy(file) {
  const result = spawnSync(process.execPath, [POLICY_SCRIPT, '--json', file], { encoding: 'utf8' });
  // 0 = within the hard limits, 1 = a hard limit exceeded (still a report);
  // anything else (2, a signal, a missing node) is an error, never a verdict.
  if (result.status !== 0 && result.status !== 1) {
    const reason = result.error ? result.error.message : `exit ${result.status ?? result.signal}`;
    throw new Error(`size policy failed on ${file} (${reason}): ${result.stderr.trim()}`);
  }
  const report = JSON.parse(result.stdout);
  if ((result.status === 1) !== report.failures.length > 0) {
    throw new Error(`size policy exit status ${result.status} disagrees with its report for ${file}`);
  }
  return report;
}

/**
 * The current PR description from the GitHub API, so a re-run after an edit
 * sees the edit (a re-run keeps the triggering event's copy). Falls back to
 * PR_BODY, the event's copy, when the API cannot be reached.
 */
export async function githubPrBody(env) {
  const { GITHUB_API_URL: api, GITHUB_REPOSITORY: repo, PR_NUMBER: number, GH_TOKEN: token } = env;
  if (api && /^[\w.-]+\/[\w.-]+$/.test(repo ?? '') && /^[0-9]+$/.test(number ?? '')) {
    try {
      const headers = { Accept: 'application/vnd.github+json' };
      if (token) headers.Authorization = `Bearer ${token}`;
      const response = await fetch(`${api}/repos/${repo}/pulls/${number}`, {
        headers,
        signal: AbortSignal.timeout(30_000),
      });
      if (response.ok) {
        const pull = await response.json();
        return typeof pull.body === 'string' ? pull.body : '';
      }
      console.error(`wasm-size-gate: GitHub API returned ${response.status}; using the event's PR description`);
    } catch (error) {
      console.error(`wasm-size-gate: GitHub API unavailable (${error.message}); using the event's PR description`);
    }
  } else {
    console.error("wasm-size-gate: GitHub API context missing; using the event's PR description");
  }
  return env.PR_BODY ?? '';
}

function parseArgs(argv) {
  const options = { githubPrBody: false };
  const valued = { '--base-dir': 'baseDir', '--pr-dir': 'prDir', '--body-file': 'bodyFile', '--markdown': 'markdown' };
  for (let i = 0; i < argv.length; i++) {
    const arg = argv[i];
    if (arg === '--github-pr-body') options.githubPrBody = true;
    else if (arg in valued && i + 1 < argv.length) options[valued[arg]] = argv[++i];
    else throw new Error(`unexpected argument ${arg}`);
  }
  if (!options.baseDir || !options.prDir) throw new Error('--base-dir and --pr-dir are required');
  if (options.githubPrBody && options.bodyFile) throw new Error('use one of --body-file and --github-pr-body');
  return options;
}

async function main(argv) {
  let options;
  let result;
  let markdown;
  try {
    options = parseArgs(argv);
    const baseKernel = join(options.baseDir, MODULES.kernel.base);
    const prKernel = join(options.prDir, MODULES.kernel.pr);
    const pr = runPolicy(prKernel);
    const base = existsSync(baseKernel) ? runPolicy(baseKernel) : null;
    const baseIo = join(options.baseDir, MODULES.io.base);
    const prIo = join(options.prDir, MODULES.io.pr);
    const io = existsSync(prIo) ? { base: existsSync(baseIo) ? statSync(baseIo).size : null, pr: statSync(prIo).size } : null;
    let body = '';
    if (options.bodyFile) body = readFileSync(options.bodyFile, 'utf8');
    else if (options.githubPrBody) body = await githubPrBody(process.env);
    result = evaluateGate({ base, pr, body });
    markdown = formatMarkdown(result, { policy: pr.policy, toolchain: pr.toolchain, io });
    if (options.markdown) writeFileSync(options.markdown, markdown);
  } catch (error) {
    console.error(`wasm-size-gate: ${error.message}`);
    console.error(
      'usage: wasm-size-gate.mjs --base-dir <checkout> --pr-dir <artifacts> [--body-file <file> | --github-pr-body] [--markdown <out.md>]',
    );
    return 2;
  }
  // Only generated text reaches stdout: the PR description never does, so it
  // cannot inject workflow commands. The report with the declaration goes to
  // the --markdown file.
  const annotate = process.env.GITHUB_ACTIONS === 'true';
  const command = (text) => text.replace(/%/g, '%25').replace(/\r/g, '%0D').replace(/\n/g, '%0A');
  for (const failure of result.failures) {
    console.log(annotate ? `::error title=WASM size (${failure.code})::${command(failure.message)}` : `FAIL ${failure.code}: ${failure.message}`);
  }
  for (const warning of result.warnings) {
    console.log(annotate ? `::warning title=WASM size (${warning.code})::${command(warning.message)}` : `WARN ${warning.code}: ${warning.message}`);
  }
  console.log(
    `kernel raw ${bytes(result.pr.rawBytes)} (${signed(result.delta.rawBytes, kib)}), ` +
      `gzip ${bytes(result.pr.gzipBytes)} (${signed(result.delta.gzipBytes, kib)}), ` +
      `Brotli ${bytes(result.pr.brotliBytes)} (${signed(result.delta.brotliBytes, kib)}): ${result.pass ? 'pass' : 'FAIL'}`,
  );
  return result.pass ? 0 : 1;
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  process.exitCode = await main(process.argv.slice(2));
}
