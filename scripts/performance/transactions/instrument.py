#!/usr/bin/env python3
"""Apply diagnostic-only allocation/clone instrumentation to a disposable checkout.

Run only on a clean, isolated checkout. Never ship the resulting WASM package.
The allocator delegates unchanged to System. Counters are process-wide: run one
single-threaded workload per process. Clone bytes mean requested heap bytes during
Topology::clone, not serialization size or CPU memcpy traffic.
"""
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1]).resolve()
here = pathlib.Path(__file__).resolve().parent
p = root / 'crates/topology/src/topology.rs'
s = p.read_text()
s = s.replace('#[derive(Debug, Default, Clone)]\npub struct Topology', '#[derive(Debug, Default)]\npub struct Topology')
body = s.split('pub struct Topology {', 1)[1].split('\n}', 1)[0]
fields = re.findall(r'^    (?:pub\(crate\) )?(\w+):', body, re.M)
impl = '\nimpl Clone for Topology {\n    fn clone(&self) -> Self {\n        let before = crate::transaction_probe::allocated();\n        let result = Self {\n'
impl += ''.join(f'            {f}: self.{f}.clone(),\n' for f in fields)
impl += '        };\n        crate::transaction_probe::record_clone(before);\n        result\n    }\n}\n'
p.write_text(s + impl)
(root / 'crates/topology/src/transaction_probe.rs').write_text((here / 'allocator.rs').read_text())
p = root / 'crates/topology/src/lib.rs'
p.write_text(p.read_text() + '\n#[doc(hidden)]\npub mod transaction_probe;\n')
p = root / 'crates/wasm/src/lib.rs'
p.write_text(p.read_text() + '''
#[wasm_bindgen(js_name = "transactionProbeReset")]
pub fn transaction_probe_reset() { remus_topology::transaction_probe::reset(); }
#[wasm_bindgen(js_name = "transactionProbeRead")]
pub fn transaction_probe_read() -> Vec<f64> {
    remus_topology::transaction_probe::read().iter().map(|v| *v as f64).collect()
}
''')
(root / 'crates/wasm/examples/perf-t01.rs').write_text((here / 'native.rs').read_text())
# Faults run after geometry has allocated a result and after winding mutation,
# inside the existing public operation's transaction on both revisions.
p = root / 'crates/operations/src/boolean/mod.rs'
s = p.read_text()
needle = 'let result = boolean_with_context_impl(topo, op, a, b, context, opts, used_fallback)?;'
assert s.count(needle) == 1
s = s.replace(needle, needle + '\n    crate::transaction_fault::check(1, result)?;')
needle = 'normalize_hole_windings(topo, result)?;'
assert s.count(needle) == 1
s = s.replace(needle, needle + '\n    crate::transaction_fault::check(2, result)?;')
p.write_text(s)
(root / 'crates/operations/src/transaction_fault.rs').write_text((here / 'fault.rs').read_text())
p = root / 'crates/operations/src/lib.rs'
p.write_text(p.read_text() + '\n#[doc(hidden)]\npub mod transaction_fault;\n')
# Reuse the unit test's live-state oracle: no arena-count-only comparison.
s = (here.parents[2] / 'crates/topology/src/transaction/savepoint_tests.rs').read_text()
state = s[s.index('fn state('):s.index('\nfn attributes(')]
state = state.replace('fn state(t: &Topology)', 'pub fn logical_state(t: &crate::Topology)')
p = root / 'crates/topology/src/transaction_probe.rs'
p.write_text(p.read_text() + '\n' + state)
p = root / 'crates/wasm/src/lib.rs'
p.write_text(p.read_text() + '''
#[wasm_bindgen(js_name = "transactionProbeFault")]
pub fn transaction_probe_fault(stage: u8) { remus_operations::transaction_fault::set(stage); }
#[wasm_bindgen(js_name = "transactionProbeFailed")]
pub fn transaction_probe_failed() -> i32 {
    remus_operations::transaction_fault::failed().map_or(-1, |id| id.index() as i32)
}
use crate::kernel::BrepKernel;
#[wasm_bindgen]
impl BrepKernel {
    #[wasm_bindgen(js_name = "transactionProbeState")]
    pub fn transaction_probe_state(&self) -> String {
        remus_topology::transaction_probe::logical_state(self.topo())
    }
    #[wasm_bindgen(js_name = "transactionProbeSession")]
    pub fn transaction_probe_session(&self) -> String {
        format!("{:?}|{:?}|{:?}|{:?}|{}", self.assemblies, self.sketches, self.gcs_sketches, self.checkpoints, self.poisoned)
    }
}
''')
(root / 'crates/wasm/examples/perf-t01-fault.rs').write_text((here / 'native_fault.rs').read_text())
