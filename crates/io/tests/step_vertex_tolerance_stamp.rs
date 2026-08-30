//! CHARACTERIZATION (RFC 0004 Stage 1, flips at Stage 4): the STEP reader
//! stamps a fixed `1e-7` mm vertex tolerance on every imported vertex,
//! regardless of the file's measured geometry (the reader builds each
//! `Vertex` with the fixed `1e-7` stamp). The tolerant-modeling program
//! replaces this with measured-gap assignment in Stage 4; this pin is the
//! baseline that flip changes.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::{reader::read_step, writer::write_step};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::Topology;

const STAMPED_TOL: f64 = 1e-7;

#[test]
fn step_import_stamps_the_fixed_default_vertex_tolerance() {
    // A box (exact planar geometry) and a cylinder: whatever the file's
    // measured geometry is, the reader stamps the same fixed 1e-7 ball on
    // every vertex.
    let documents = [
        ("box", {
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 10.0, 20.0, 30.0).unwrap();
            write_step(&topo, &[solid]).unwrap()
        }),
        ("cylinder", {
            let mut topo = Topology::new();
            let solid = make_cylinder(&mut topo, 7.5, 12.5).unwrap();
            write_step(&topo, &[solid]).unwrap()
        }),
    ];

    for (name, text) in documents {
        let mut imported = Topology::new();
        let solids = read_step(&text, &mut imported).unwrap();
        assert_eq!(solids.len(), 1, "{name}: one solid re-imported");
        assert!(imported.num_vertices() > 0, "{name}: has vertices");

        let mut checked = 0;
        for (_vid, vertex) in imported.vertices().iter() {
            assert_eq!(
                vertex.tolerance().to_bits(),
                STAMPED_TOL.to_bits(),
                "{name}: imported vertex carries the fixed 1e-7 stamp"
            );
            checked += 1;
        }
        assert!(checked > 0, "{name}: every imported vertex inspected");
    }
}
