'use strict';

// The kernel class a remus-wasm package exports. Packages built before the
// RemusKernel rename export it only under the predecessor name, and benchmark
// and parity drivers load such packages as baselines and committed references.
exports.kernelClass = function kernelClass(mod) {
  const Kernel = mod.RemusKernel ?? mod.BrepKernel;
  if (typeof Kernel !== 'function') {
    throw new TypeError('remus-wasm package exports no kernel class');
  }
  return Kernel;
};
