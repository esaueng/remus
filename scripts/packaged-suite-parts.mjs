/**
 * Split a packaged test script's suites across parallel CI jobs.
 *
 * `REMUS_PACKAGE_SUITE_PART=i/n` selects part `i` of `n`; unset runs every
 * suite, which is what `cargo xtask wasm-build` and a local run do.
 *
 * Every part walks the same suites in the same order and assigns each one
 * to the part with the least weight so far, so each suite runs in exactly
 * one part whatever the weights are. Weights only balance the parts.
 */

// Measured seconds of the suites that take more than about ten (2026-11, one
// local run of both scripts). Every other suite counts as one second. Each
// part prints its suites' times, so a stale entry shows up in the job log.
export const WEIGHTS = {
  runWideSphereCapRegression: 215,
  runOpenZcadGeckoEditRegression: 140,
  runOpseqF13Packaged: 122,
  runOpseqF10Packaged: 82,
  runOpseqF6Packaged: 70,
  runOpseqSphereTorusIntersectPackaged: 60,
  runOpseqTorusSphereFusePackaged: 48,
  runBoundedTabRolloverPackaged: 48,
  runSphereCylinderClosedSectionPackaged: 40,
  runOpseqF17Packaged: 30,
  runOpseqTorusPropertiesPackaged: 15,
  runOpseqSphereCapPackaged: 13,
  runTaperedHollowSweepPackaged: 13,
};

export function parsePart(value) {
  if (value === undefined || value === '') return { index: 0, total: 1 };
  const match = /^([1-9][0-9]?)\/([1-9][0-9]?)$/.exec(value);
  if (!match || Number(match[1]) > Number(match[2])) {
    throw new Error(`REMUS_PACKAGE_SUITE_PART must be "i/n" with 1 <= i <= n, got ${JSON.stringify(value)}`);
  }
  return { index: Number(match[1]) - 1, total: Number(match[2]) };
}

export function suiteParts(value = process.env.REMUS_PACKAGE_SUITE_PART, log = console.log) {
  const { index, total } = parsePart(value);
  const loads = new Array(total).fill(0);
  let seen = 0;
  let ran = 0;
  const part = (run, explicitWeight) => {
    const name = /\b(run\w+)/.exec(String(run))?.[1];
    const weight = explicitWeight ?? WEIGHTS[name] ?? 1;
    if (!(weight > 0)) throw new Error('A suite weight must be positive');
    seen += 1;
    const assigned = loads.indexOf(Math.min(...loads));
    loads[assigned] += weight;
    if (assigned !== index) return;
    const started = performance.now();
    run();
    ran += 1;
    log(`# suite ${name ?? seen}: ${((performance.now() - started) / 1000).toFixed(1)}s`);
  };
  part.summary = () => log(`# part ${index + 1}/${total} ran ${ran} of ${seen} suites`);
  return part;
}
