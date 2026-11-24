//! Rim blends on a cylinder wall that carries straight edges away from the rim.
//!
//! A plane/cylinder rim fillet is removed (or resized through zero) by
//! extending the wall back up to the plane: the one straight edge that ends
//! on the band's contact vertex, the wall's seam, is lengthened to the new
//! sharp vertex. The rebuild used to treat every straight edge in the wall's
//! boundary wire as a seam candidate and refuse unless there was exactly one.
//! A wall notched from below by a fused box also carries the box's two
//! vertical rulings in that wire, far from the rim, so removing the top rim
//! fillet refused with "closed cylinder support has 3 seam edges". Only the
//! line on the contact vertex moves; the rulings are kept as they are.
//!
//! Reported from a web-app session (2026-11-22): a long direct-edit history
//! left a Ø80 boss whose lower wall kept such rulings, and setting its top
//! rim fillet to R0 refused.
//!
//! Oracle: removing the blend restores the volume of the same fused solid
//! built without the fillet, and the result validates and meshes watertight.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::validate::{ValidateOptions, validate_solid};
use remus_math::compact_sort::CompactSort;
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::{merge_split_rim_arcs, unify_faces};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::resize_blend::resize_blend;
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const RADIUS: f64 = 10.0;
const HEIGHT: f64 = 20.0;
const FILLET: f64 = 2.0;
const DEFLECTION: f64 = 1e-3;

/// A Ø20 × 20 cylinder with a box fused through its lower wall on the +y
/// side, away from the seam: the wall's boundary wire picks up the box's two
/// vertical rulings, which stop well below the top rim.
fn notched_boss(topo: &mut Topology) -> SolidId {
    let cylinder = make_cylinder(topo, RADIUS, HEIGHT).unwrap();
    let block = make_box(topo, 4.0, 10.0, 12.0).unwrap();
    transform_solid(topo, block, &Mat4::translation(-2.0, 5.0, -4.0)).unwrap();
    let fused = boolean(topo, BooleanOp::Fuse, cylinder, block).unwrap();
    // One wall face, as the web kernel's default fuse leaves it, and the top
    // rim one closed circle again, as a face move rebuilds it.
    unify_faces(topo, fused).unwrap();
    merge_split_rim_arcs(topo, fused, Tolerance::new()).unwrap();
    fused
}

fn top_rim(topo: &Topology, solid: SolidId) -> EdgeId {
    let rims: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle) => {
                Tolerance::new().approx_eq(circle.center().z(), HEIGHT)
                    && Tolerance::new().approx_eq(circle.radius(), RADIUS)
            }
            _ => false,
        })
        .collect();
    assert_eq!(rims.len(), 1, "one closed top rim");
    rims[0]
}

fn rim_band(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    let bands: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Torus(torus)
                    if Tolerance::new().approx_eq(torus.minor_radius(), radius)
            )
        })
        .collect();
    assert_eq!(bands.len(), 1, "one rim band");
    bands[0]
}

/// The straight edges of the wall the band rests on, so the fixture provably
/// carries rulings besides its seam.
fn wall_line_count(topo: &Topology, solid: SolidId) -> usize {
    let walls: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), RADIUS)
            )
        })
        .collect();
    assert_eq!(walls.len(), 1, "one boss wall");
    let face = topo.face(walls[0]).unwrap();
    let mut lines: Vec<EdgeId> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .flat_map(|wire| topo.wire(wire).unwrap().edges().to_vec())
        .map(|oriented| oriented.edge())
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line))
        .collect();
    lines.compact_sort_unstable_by_key(|edge| edge.index());
    lines.dedup();
    lines.len()
}

fn assert_sound(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).unwrap();
    assert!(is_watertight(&mesh), "result must mesh watertight");
}

#[test]
fn removing_top_rim_blend_keeps_lower_wall_rulings() {
    let mut topo = Topology::new();
    let sharp = notched_boss(&mut topo);
    let sharp_volume = solid_volume(&topo, sharp, DEFLECTION).unwrap();
    assert!(
        wall_line_count(&topo, sharp) >= 3,
        "fixture must put rulings beside the seam on the wall"
    );

    let rim = top_rim(&topo, sharp);
    let filleted = fillet_v2(&mut topo, sharp, &[rim], FILLET).unwrap().solid;
    let band = rim_band(&topo, filleted, FILLET);

    let removed = resize_blend(&mut topo, filleted, band, FILLET, 0.0)
        .expect("removing the rim blend rebuilds the sharp rim")
        .solid;
    assert_sound(&topo, removed);
    let removed_volume = solid_volume(&topo, removed, DEFLECTION).unwrap();
    assert!(
        (removed_volume - sharp_volume).abs() <= sharp_volume * 1e-6,
        "removed {removed_volume} vs sharp {sharp_volume}"
    );
    top_rim(&topo, removed);
    assert!(
        wall_line_count(&topo, removed) >= 3,
        "the wall keeps its rulings"
    );
}

#[test]
fn resizing_top_rim_blend_keeps_lower_wall_rulings() {
    for new_radius in [3.0, 1.0] {
        let mut topo = Topology::new();
        let sharp = notched_boss(&mut topo);
        let rim = top_rim(&topo, sharp);
        let filleted = fillet_v2(&mut topo, sharp, &[rim], FILLET).unwrap().solid;
        let band = rim_band(&topo, filleted, FILLET);
        let resized = resize_blend(&mut topo, filleted, band, FILLET, new_radius)
            .unwrap_or_else(|error| panic!("resize to {new_radius}: {error}"))
            .solid;
        assert_sound(&topo, resized);
        rim_band(&topo, resized, new_radius);
    }
}
