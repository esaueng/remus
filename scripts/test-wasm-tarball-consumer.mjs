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
import { runProjectCurvePackaged } from './project-curve-packaged.mjs';
import { runKernelCorrectnessPackaged } from './kernel-correctness-packaged.mjs';
import { runNextKernelCorrectnessPackaged } from './kernel-correctness-next-packaged.mjs';
import { runMaximumDistancePackaged } from './maximum-distance-packaged.mjs';
import { runDraftAnalysisPackaged } from './draft-analysis-packaged.mjs';
import { runWasmParserReferencesPackaged } from './wasm-parser-refs-packaged.mjs';
import { runVariableFilletInputPackaged } from './variable-fillet-input-packaged.mjs';
import { runVariableFilletBoundariesPackaged } from './variable-fillet-boundaries-packaged.mjs';
import { runTaperedHollowSweepPackaged } from './tapered-hollow-sweep-packaged.mjs';
import { runWasmRayNurbsPackaged } from './wasm-ray-nurbs-packaged.mjs';
import { runOpseqF5Packaged } from './opseq-f5-packaged.mjs';
import { runOpseqF6Packaged } from './opseq-f6-packaged.mjs';
import { runOpseqOffsetWallPackaged } from './opseq-offset-wall-packaged.mjs';
import { runOpseqOffsetRefusalPackaged } from './opseq-offset-refusal-packaged.mjs';
import { runOpseqF10Packaged } from './opseq-f10-packaged.mjs';
import { runOpseqF13Packaged } from './opseq-f13-packaged.mjs';
import { runWasmRayTorusPackaged } from './wasm-ray-torus-packaged.mjs';
import { runSecurityResourceRegressions } from './security-resource-packaged.mjs';
import { runPackageQueueWitness } from './queue-contracts-packaged.mjs';
import { runPointedConeCutPackaged } from './pointed-cone-cut-packaged.mjs';
import { runOpseqF3Packaged } from './opseq-f3-packaged.mjs';
import { runOpseqF17Packaged } from './opseq-f17-packaged.mjs';
import { runOpseqSphereCapPackaged } from './opseq-sphere-cap-packaged.mjs';
import { runOpseqTorusPropertiesPackaged } from './opseq-torus-properties-packaged.mjs';
import { runDocumentCopyPackaged } from './document-copy-packaged.mjs';
import { runBoundedTabRolloverPackaged } from './bounded-tab-rollover-packaged.mjs';
import { runPackagedPreviewWorkContract } from './perf/face-drag-session/packaged_contract.cjs';
import { runLinkedSketchPackaged } from './linked-sketch-packaged.mjs';
import { runGridPatternPackaged } from './grid-pattern-packaged.mjs';
import { runRevolveHistoryPackaged } from './revolve-history-packaged.mjs';
import { runPlanarImprintPackaged } from './planar-imprint-packaged.mjs';
import { runMixedSeamFilletPackaged } from './mixed-seam-fillet-packaged.mjs';

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
  assert.equal(typeof packageExports.RemusKernel, 'function', 'installed RemusKernel export');
  assert.equal(
    packageExports.BrepKernel,
    packageExports.RemusKernel,
    'installed deprecated BrepKernel alias is the RemusKernel class',
  );
  assert.equal(
    typeof packageExports.decodeEvolutionPayload,
    'function',
    'installed decodeEvolutionPayload export',
  );
  const ioExports = consumerRequire('remus-wasm-io');
  assert.equal(typeof ioExports.RemusIo, 'function', 'installed RemusIo export');

  runSecurityResourceRegressions({ ...packageExports, ...ioExports });
  runPackageQueueWitness({ ...packageExports, ...ioExports });
  runPointedConeCutPackaged(packageExports);
  runOpseqF3Packaged(packageExports);
  runOpseqF17Packaged(packageExports);
  runOpseqSphereCapPackaged(packageExports);
  runOpseqTorusPropertiesPackaged(packageExports);
  runDocumentCopyPackaged({ ...packageExports, ...ioExports });
  runBoundedTabRolloverPackaged(packageExports);
  runMixedSeamFilletPackaged(packageExports);
  runPackagedPreviewWorkContract({
    kernelPackage: realpathSync(resolve(consumerDir, 'node_modules/remus-wasm')),
    ioPackage: realpathSync(resolve(consumerDir, 'node_modules/remus-wasm-io')),
  });
  runOpenZcadConsumerRegressions({ ...packageExports, ...ioExports });
  runGridPatternPackaged(packageExports);
  runRevolveHistoryPackaged(packageExports);
  runPlanarImprintPackaged(packageExports);
  runProjectCurvePackaged(packageExports);
  runKernelCorrectnessPackaged(packageExports);
  runNextKernelCorrectnessPackaged(packageExports);
  runMaximumDistancePackaged(packageExports);
  runDraftAnalysisPackaged(packageExports);
  runWasmParserReferencesPackaged(packageExports);
  runVariableFilletInputPackaged(packageExports);
  runVariableFilletBoundariesPackaged(packageExports);
  runTaperedHollowSweepPackaged(packageExports);
  runWasmRayNurbsPackaged(packageExports);
  runOpseqF5Packaged(packageExports);
  runOpseqF6Packaged(packageExports);
  runOpseqOffsetWallPackaged(packageExports);
  runOpseqOffsetRefusalPackaged(packageExports);
  runOpseqF10Packaged(packageExports);
  runOpseqF13Packaged(packageExports);
  runWasmRayTorusPackaged(packageExports);
  runLinkedSketchPackaged(packageExports);

  console.log('\nInstalled-tarball consumer regressions passed');
} finally {
  rmSync(temporaryRoot, { recursive: true, force: true });
}
