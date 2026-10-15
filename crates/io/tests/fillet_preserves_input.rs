//! A successful fillet must leave its input solid untouched.
//!
//! The walking engine reuses the input's untouched face ids in its result and
//! trims shared faces in place, so a fillet that succeeded used to notch the
//! input's cap faces while leaving its side faces untrimmed: the input's edge
//! count grew, its volume dropped and it stopped validating, and a second
//! fillet on the original then hit a phantom cliff.
//!
//! The pocketed plate is round-tripped through an exact arena document first
//! because that is the arena the WASM `fillet` binding sees and the one in
//! which the cascade picks the walking engine for the corner selection.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::mat::Mat4;
use remus_operations::blend_ops::{BlendEngine, fillet_cascade};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::query::filter_filletable_edges;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_vertices};
use remus_topology::solid::SolidId;

const PLATE: (f64, f64, f64) = (160.0, 100.0, 16.0);
const POCKET: (f64, f64, f64) = (18.0, 14.0, 8.0);

/// A `PLATE` box with a `cols` x `rows` grid of `POCKET` pockets cut into its
/// top face.
fn pocketed_plate(topo: &mut Topology, cols: usize, rows: usize) -> SolidId {
    let mut plate = make_box(topo, PLATE.0, PLATE.1, PLATE.2).unwrap();
    let pitch_x = PLATE.0 / cols as f64;
    let pitch_y = PLATE.1 / rows as f64;
    for i in 0..cols {
        for j in 0..rows {
            // The pocket overshoots the top face so the cut has no coplanar cap.
            let pocket = make_box(topo, POCKET.0, POCKET.1, POCKET.2 + 1.0).unwrap();
            let x = pitch_x * (i as f64 + 0.5) - POCKET.0 / 2.0;
            let y = pitch_y * (j as f64 + 0.5) - POCKET.1 / 2.0;
            let z = PLATE.2 - POCKET.2;
            transform_solid(topo, pocket, &Mat4::translation(x, y, z)).unwrap();
            plate = boolean(topo, BooleanOp::Cut, plate, pocket).unwrap();
        }
    }
    plate
}

/// The plate's four outer vertical corner edges.
fn outer_corner_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let tol = 1e-6;
    let corners: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&eid| {
            let edge = topo.edge(eid).unwrap();
            if !matches!(edge.curve(), EdgeCurve::Line) {
                return false;
            }
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let vertical = (a.x() - b.x()).abs() < tol
                && (a.y() - b.y()).abs() < tol
                && ((a.z() - b.z()).abs() - PLATE.2).abs() < tol;
            let on_x = a.x().abs() < tol || (a.x() - PLATE.0).abs() < tol;
            let on_y = a.y().abs() < tol || (a.y() - PLATE.1).abs() < tol;
            vertical && on_x && on_y
        })
        .collect();
    assert_eq!(corners.len(), 4, "plate must have four corner edges");
    corners
}

struct Snapshot {
    counts: (usize, usize, usize),
    volume: f64,
}

fn snapshot(topo: &Topology, solid: SolidId) -> Snapshot {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "solid must validate: {report:?}");
    Snapshot {
        counts: solid_entity_counts(topo, solid).unwrap(),
        volume: solid_volume(topo, solid, 0.01).unwrap(),
    }
}

fn assert_fillet_leaves_input_untouched(cols: usize, rows: usize) {
    let mut history = Topology::new();
    let built = pocketed_plate(&mut history, cols, rows);
    let bytes = remus_io::arena_io::serialize_solid(&history, built).unwrap();

    let mut topo = Topology::new();
    let input = remus_io::arena_io::deserialize_solid(&bytes, &mut topo).unwrap();
    let before = snapshot(&topo, input);
    let input_bytes = remus_io::arena_io::serialize_solid(&topo, input).unwrap();

    let corners = outer_corner_edges(&topo, input);
    let edges = filter_filletable_edges(&topo, input, &corners).unwrap();
    assert_eq!(edges.len(), 4);
    let result = fillet_cascade(&mut topo, input, &edges, 6.0).unwrap();
    assert_ne!(result.solid, input, "fillet must return a new solid");
    assert_eq!(result.engine, BlendEngine::Walking);

    let filleted = snapshot(&topo, result.solid);
    // Four r=6 corners each remove (1 - pi/4) r^2 over the plate height.
    let removed = 4.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 36.0 * PLATE.2;
    assert!(
        (before.volume - filleted.volume - removed).abs() < 0.5,
        "filleted volume {:.4}, expected {:.4}",
        filleted.volume,
        before.volume - removed
    );

    let after = snapshot(&topo, input);
    assert_eq!(
        after.counts, before.counts,
        "fillet changed its input's topology"
    );
    assert!(
        (after.volume - before.volume).abs() < 1e-9 * before.volume,
        "fillet changed its input's volume: {:.4} -> {:.4}",
        before.volume,
        after.volume
    );
    assert!(
        remus_io::arena_io::serialize_solid(&topo, input).unwrap() == input_bytes,
        "fillet changed its input's serialized document"
    );

    // The untouched input fillets again at a different radius: no phantom cliff
    // left behind by the first blend.
    let again = filter_filletable_edges(&topo, input, &corners).unwrap();
    let second = fillet_cascade(&mut topo, input, &again, 6.01).unwrap();
    snapshot(&topo, second.solid);
}

#[test]
fn fillet_leaves_pocketed_plate_input_untouched() {
    assert_fillet_leaves_input_untouched(4, 3);
}

/// Two blends of one input share only the input's own vertices: a contact
/// point never snaps to the coincident vertex the other blend minted, which
/// would cross-link the two results.
#[test]
fn repeated_fillet_does_not_share_vertices_with_an_earlier_result() {
    let mut history = Topology::new();
    let built = pocketed_plate(&mut history, 4, 3);
    let bytes = remus_io::arena_io::serialize_solid(&history, built).unwrap();
    let mut topo = Topology::new();
    let input = remus_io::arena_io::deserialize_solid(&bytes, &mut topo).unwrap();

    let corners = outer_corner_edges(&topo, input);
    let edges = filter_filletable_edges(&topo, input, &corners).unwrap();
    let first = fillet_cascade(&mut topo, input, &edges, 6.0).unwrap();
    let second = fillet_cascade(&mut topo, input, &edges, 6.0).unwrap();
    assert_eq!(first.engine, BlendEngine::Walking);
    assert_eq!(second.engine, BlendEngine::Walking);
    snapshot(&topo, first.solid);
    snapshot(&topo, second.solid);

    let input_vertices = solid_vertices(&topo, input).unwrap();
    let first_vertices = solid_vertices(&topo, first.solid).unwrap();
    let shared: Vec<_> = solid_vertices(&topo, second.solid)
        .unwrap()
        .into_iter()
        .filter(|v| first_vertices.contains(v) && !input_vertices.contains(v))
        .collect();
    assert!(
        shared.is_empty(),
        "the second fillet reuses {} vertices the first one minted",
        shared.len()
    );
}

/// Exact result documents (including loop order, curves, pcurves and winding)
/// must not depend on other bodies or stale wires in the session arena.
#[test]
fn walking_fillet_output_is_identical_with_unrelated_arena_contents() {
    let mut history = Topology::new();
    let built = pocketed_plate(&mut history, 4, 3);
    let bytes = remus_io::arena_io::serialize_solid(&history, built).unwrap();
    let run = |padding: usize| {
        let mut topo = Topology::new();
        let mut foreign = Vec::new();
        for _ in 0..padding {
            foreign.push(make_box(&mut topo, PLATE.0, PLATE.1, PLATE.2).unwrap());
        }
        let input = remus_io::arena_io::deserialize_solid(&bytes, &mut topo).unwrap();
        // A prior blend supplies coincident contact vertices and obsolete
        // face/wire history as well as an overlapping, independently owned body.
        let edges = outer_corner_edges(&topo, input);
        let prior = fillet_cascade(&mut topo, input, &edges, 6.0).unwrap();
        foreign.push(prior.solid);
        for _ in 0..padding {
            foreign.push(make_box(&mut topo, PLATE.0, PLATE.1, PLATE.2).unwrap());
        }
        let untouched: Vec<_> = std::iter::once(input)
            .chain(foreign)
            .map(|solid| {
                (
                    solid,
                    remus_io::arena_io::serialize_solid(&topo, solid).unwrap(),
                )
            })
            .collect();
        let result = fillet_cascade(&mut topo, input, &edges, 6.0).unwrap();
        assert_eq!(result.engine, BlendEngine::Walking);
        assert_eq!(result.succeeded, edges);
        assert!(result.failed.is_empty());
        assert!(!result.is_partial);
        let before = snapshot(&topo, input);
        let after = snapshot(&topo, result.solid);
        let removed = 4.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 36.0 * PLATE.2;
        assert!((before.volume - after.volume - removed).abs() < 0.5);
        for (solid, document) in untouched {
            assert_eq!(
                remus_io::arena_io::serialize_solid(&topo, solid).unwrap(),
                document
            );
        }
        remus_io::arena_io::serialize_solid(&topo, result.solid).unwrap()
    };
    let baseline = run(0);
    for padding in [1, 64, 1024] {
        assert_eq!(run(padding), baseline, "padding bodies per side: {padding}");
    }
}
