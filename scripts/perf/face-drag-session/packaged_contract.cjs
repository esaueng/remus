'use strict';

const path = require('node:path');
const { main } = require('./drag_session_bench.cjs');

// Exercise the same public calls against the caller's actual package paths,
// including an npm-installed pair. No timing observations qualify here.
function runPackagedPreviewWorkContract({ kernelPackage, ioPackage }) {
  const fixture = path.resolve(__dirname, '../../../crates/io/tests/data/shapr3d_hammer_holder.step');
  const results = [];
  for (const background of [0, 200]) {
    results.push(main(['--kernel-pkg', kernelPackage, '--io-pkg', ioPackage,
      '--fixture', fixture, '--ticks', '3', '--warmup', '1', '--pattern', 'unique',
      '--background', String(background), '--label', 'packaged-preview-work-contract',
      '--timing-status', 'contended-smoke-excluded']));
  }
  if (results[0].workContract.sourceSlots !== results[1].workContract.sourceSlots) {
    throw new Error('Background bodies changed the source topology inventory');
  }
  console.log('ok - holder preview retained-topology work budget: 0/200 background bodies');
  return results.map(result => result.workContract);
}

module.exports = { runPackagedPreviewWorkContract };
if (require.main === module) {
  if (process.argv.length !== 4) throw new Error('Expected kernel and translator package directories');
  runPackagedPreviewWorkContract({ kernelPackage: path.resolve(process.argv[2]),
    ioPackage: path.resolve(process.argv[3]) });
}
