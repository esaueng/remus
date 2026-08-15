#!/usr/bin/env node
/**
 * Pack and install brepkit-wasm into a disposable consumer, then run the
 * OpenZCAD-shaped exact-geometry regressions through the installed package.
 *
 * Usage: node scripts/test-wasm-tarball-consumer.mjs
 */

import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { mkdtempSync, mkdirSync, realpathSync, rmSync } from 'node:fs';
import { createRequire } from 'node:module';
import { dirname, resolve, sep } from 'node:path';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import { runOpenZcadConsumerRegressions } from './openzcad-wasm-consumer-regressions.mjs';

const scriptDir = dirname(fileURLToPath(import.meta.url));
const projectRoot = resolve(scriptDir, '..');
const packageDir = resolve(projectRoot, 'crates/wasm/pkg');
const temporaryRoot = mkdtempSync(resolve(tmpdir(), 'brepkit-tarball-consumer-'));
const consumerDir = resolve(temporaryRoot, 'consumer');
const npmEnvironment = {
  ...process.env,
  npm_config_cache: resolve(temporaryRoot, 'npm-cache'),
};

try {
  mkdirSync(consumerDir);
  const packOutput = execFileSync(
    'npm',
    ['pack', packageDir, '--json', '--pack-destination', temporaryRoot],
    { encoding: 'utf8', env: npmEnvironment },
  );
  const packed = JSON.parse(packOutput);
  assert.equal(packed.length, 1, 'npm pack must produce exactly one tarball');
  const tarball = resolve(temporaryRoot, packed[0].filename);

  execFileSync(
    'npm',
    [
      'install',
      '--prefix',
      consumerDir,
      '--no-save',
      '--package-lock=false',
      '--ignore-scripts',
      '--no-audit',
      '--no-fund',
      tarball,
    ],
    { env: npmEnvironment, stdio: 'inherit' },
  );

  const consumerRequire = createRequire(resolve(consumerDir, 'consumer.cjs'));
  const resolvedEntry = consumerRequire.resolve('brepkit-wasm');
  const installedPackageDir = realpathSync(resolve(consumerDir, 'node_modules/brepkit-wasm'));
  assert.ok(
    resolvedEntry.startsWith(`${installedPackageDir}${sep}`),
    `brepkit-wasm resolved outside the installed consumer: ${resolvedEntry}`,
  );

  const packageExports = consumerRequire('brepkit-wasm');
  assert.equal(typeof packageExports.BrepKernel, 'function', 'installed BrepKernel export');
  assert.equal(
    typeof packageExports.decodeEvolutionPayload,
    'function',
    'installed decodeEvolutionPayload export',
  );
  console.log(`ok - installed tarball entry resolved from ${resolvedEntry}`);

  runOpenZcadConsumerRegressions(packageExports);
  console.log('\nInstalled-tarball consumer regressions passed');
} finally {
  rmSync(temporaryRoot, { recursive: true, force: true });
}
