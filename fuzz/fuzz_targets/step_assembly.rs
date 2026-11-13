//! STEP assembly import, `step::read_step_assembly_with_limits`.
//!
//! `step_reader` and `step_bodies` never build the product graph. This target
//! drives the assembly path: `Graph::read` over `NEXT_ASSEMBLY_USAGE_OCCURRENCE`,
//! product shape and context-dependent representation links, then
//! `Graph::expand` with its cycle, depth, placement and expanded-occurrence
//! checks.
//!
//! Oracles: the import is atomic, so a refusal must leave the fresh topology
//! with no live entities. An accepted import must return resolvable solid
//! definitions; every flattened occurrence must place one of them with a
//! finite transform; the expanded component count must respect the entity
//! limit; and the length scale must be finite and positive.

#![no_main]

use libfuzzer_sys::fuzz_target;
use remus_topology::Topology;

mod common;

fuzz_target!(|data: &[u8]| {
    let Ok(input) = std::str::from_utf8(data) else {
        return;
    };
    let mut topo = Topology::new();
    let limits = common::limits();
    match remus_io::step::read_step_assembly_with_limits(input, &mut topo, limits) {
        Ok(result) => {
            for &solid in &result.solids {
                assert!(topo.solid(solid).is_ok(), "returned solid does not resolve");
            }
            assert!(
                result.assembly.component_count() <= limits.max_model_entities,
                "expanded occurrences exceed the entity limit"
            );
            for (solid, transform) in result.assembly.flatten() {
                assert!(
                    result.solids.contains(&solid),
                    "occurrence places a solid outside the returned definitions"
                );
                assert!(
                    transform.0.iter().flatten().all(|value| value.is_finite()),
                    "occurrence has a non-finite placement"
                );
            }
            assert!(
                result.length_scale_to_mm.is_finite() && result.length_scale_to_mm > 0.0,
                "length scale {} is not finite and positive",
                result.length_scale_to_mm
            );
        }
        Err(_) => common::assert_untouched(&topo, "assembly"),
    }
});
