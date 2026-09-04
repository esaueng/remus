#!/usr/bin/env node
/**
 * Pack and install remus-wasm and remus-wasm-io into a disposable consumer,
 * then run the OpenZCAD-shaped exact-geometry regressions through the
 * installed packages.
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
const packageDirs = {
  'remus-wasm': resolve(projectRoot, 'crates/wasm/pkg'),
  'remus-wasm-io': resolve(projectRoot, 'crates/wasm-io/pkg'),
};
const temporaryRoot = mkdtempSync(resolve(tmpdir(), 'remus-tarball-consumer-'));
const consumerDir = resolve(temporaryRoot, 'consumer');
const npmEnvironment = {
  ...process.env,
  npm_config_cache: resolve(temporaryRoot, 'npm-cache'),
};

try {
  mkdirSync(consumerDir);
  const tarballs = Object.values(packageDirs).map((packageDir) => {
    const packOutput = execFileSync(
      'npm',
      ['pack', packageDir, '--json', '--pack-destination', temporaryRoot],
      { encoding: 'utf8', env: npmEnvironment },
    );
    const packed = JSON.parse(packOutput);
    assert.equal(packed.length, 1, 'npm pack must produce exactly one tarball');
    return resolve(temporaryRoot, packed[0].filename);
  });

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
      ...tarballs,
    ],
    { env: npmEnvironment, stdio: 'inherit' },
  );

  const consumerRequire = createRequire(resolve(consumerDir, 'consumer.cjs'));
  for (const name of Object.keys(packageDirs)) {
    const resolvedEntry = consumerRequire.resolve(name);
    const installedPackageDir = realpathSync(resolve(consumerDir, `node_modules/${name}`));
    assert.ok(
      resolvedEntry.startsWith(`${installedPackageDir}${sep}`),
      `${name} resolved outside the installed consumer: ${resolvedEntry}`,
    );
    console.log(`ok - installed tarball entry resolved from ${resolvedEntry}`);
  }

  const packageExports = consumerRequire('remus-wasm');
  assert.equal(typeof packageExports.BrepKernel, 'function', 'installed BrepKernel export');
  assert.equal(
    typeof packageExports.decodeEvolutionPayload,
    'function',
    'installed decodeEvolutionPayload export',
  );
  const ioExports = consumerRequire('remus-wasm-io');
  assert.equal(typeof ioExports.RemusIo, 'function', 'installed RemusIo export');

  runOpenZcadConsumerRegressions({ ...packageExports, ...ioExports });
  console.log('\nInstalled-tarball consumer regressions passed');
} finally {
  rmSync(temporaryRoot, { recursive: true, force: true });
}
