//! ASCII BREP (FreeCAD saved shape) import, `brep::read_brep_with_limits`.
//!
//! Oracles: the import is transacted, so a refusal must leave the fresh
//! topology with no live entities; an accepted import must return solids
//! that resolve and own at least one face.

#![no_main]

use libfuzzer_sys::fuzz_target;
use remus_topology::Topology;

mod common;

fuzz_target!(|data: &[u8]| {
    let Ok(input) = std::str::from_utf8(data) else {
        return;
    };
    let mut topo = Topology::new();
    match remus_io::brep::read_brep_with_limits(input, &mut topo, common::limits()) {
        Ok(solids) => {
            for solid in solids {
                let faces = remus_topology::explorer::solid_faces(&topo, solid)
                    .expect("returned solid has resolvable faces");
                assert!(!faces.is_empty(), "returned solid has no faces");
            }
        }
        Err(_) => common::assert_untouched(&topo, "BREP"),
    }
});
