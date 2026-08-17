//! Regression: a fillet must follow a G1 ridgeline that topology has split
//! into several edges.
//!
//! An L-blank — a plate with a wall seated slightly into it, the OpenZCAD
//! demo bracket's Rev A — leaves each rear corner column cut into pieces at
//! the seat, even though the column is one straight, tangent-continuous
//! ridge. Filleting only the piece the caller named would have to run out in
//! the middle of a smooth edge, where there is no cap face to close against:
//! the blend either failed or produced an open shell.
//!
//! The builder now expands each seed to its whole chain, so the stripe spans
//! the full column and terminates on the real end caps.
//!
//! These drive [`FilletBuilder`] directly, because `blend_ops::fillet_v2`
//! tries the v1 rolling-ball rebuild first for planar line blends — so going
//! through the public wrapper would measure v1 here, not the walking engine.
//! v1 succeeds on this shape but builds the wall as NURBS where the walking
//! engine emits an exact cylinder, and the two tessellate differently: 36.68
//! against 34.07, for an exact 33.91. (On the real bracket v1 fails outright
//! and the wrapper falls through to this builder, which is why that case comes
//! out right either way.)
//!
//! An earlier version of this note claimed v1 over-removed and gave mirrored
//! columns different answers, 49.03 against 36.68. That was a defect in
//! `solid_volume`, not in v1 — see `regress_volume_nurbs_faces`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_blend::fillet_builder::FilletBuilder;
use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const R: f64 = 2.0;

/// Plate 80x40x8 with an 80x8x32 wall seated 0.5 mm into its top at y = 32.
/// The rear corner columns (y = 40) run z = 0..39.5 but are split where the
/// wall's underside and the plate's top meet them.
const TOP: f64 = 39.5;

fn l_blank(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 80.0, 40.0, 8.0).expect("plate");
    let wall = make_box(topo, 80.0, 8.0, 32.0).expect("wall");
    transform_solid(topo, wall, &Mat4::translation(0.0, 32.0, 7.5)).expect("seat wall");
    let fused = boolean(topo, BooleanOp::Fuse, plate, wall).expect("fuse");
    // Merge the coplanar boolean fragments, as a CAD front end does before
    // handing the body back to the user. This is what makes the corner column
    // a single ridgeline: without it the plate and wall keep separate side
    // faces, so the column's pieces have different face pairs and are not one
    // G1 chain at all.
    remus_operations::heal::unify_faces(topo, fused).expect("unify");
    fused
}

/// The lowest piece of the rear corner column at `(cx, 40)`.
fn rear_column_base(topo: &Topology, solid: SolidId, cx: f64) -> Option<EdgeId> {
    remus_topology::explorer::solid_edges(topo, solid)
        .expect("edges")
        .into_iter()
        .find(|&eid| {
            let e = topo.edge(eid).expect("edge");
            let a = topo.vertex(e.start()).expect("v").point();
            let b = topo.vertex(e.end()).expect("v").point();
            let at = |p: &remus_math::vec::Point3| {
                (p.x() - cx).abs() < 1e-6 && (p.y() - 40.0).abs() < 1e-6
            };
            at(&a) && at(&b) && a.z().min(b.z()) < 1e-9 && a.z().max(b.z()) < 8.0 - 1e-9
        })
}

#[test]
fn fillet_follows_a_split_corner_column() {
    let mut topo = Topology::new();
    let solid = l_blank(&mut topo);

    // The premise: the fuse really did split the column. Without a split
    // there is nothing for chain propagation to do and the test is vacuous.
    let seed = rear_column_base(&topo, solid, 0.0)
        .expect("the seated wall should leave the rear corner column split");

    let before = solid_volume(&topo, solid, 0.01).expect("volume before");
    let result = {
        let mut builder = FilletBuilder::new(&mut topo, solid);
        builder.add_edges(&[seed], R);
        builder.build().expect("fillet")
    };

    let shell = topo.solid(result.solid).expect("solid").outer_shell();
    validate_shell_closed(topo.shell(shell).expect("shell"), &topo)
        .expect("filleted solid must be watertight");

    // The blend must span the WHOLE 39.5 mm column, not just the 7.5 mm the
    // caller named: (1 - pi/4) * r^2 * L.
    let after = solid_volume(&topo, result.solid, 0.01).expect("volume after");
    let removed = before - after;
    let expected = (1.0 - std::f64::consts::FRAC_PI_4) * R * R * TOP;
    assert!(
        (removed - expected).abs() < 0.5,
        "expected the full-height {expected:.2} mm³ to be removed, got {removed:.2} — \
         a short stripe means the chain did not propagate"
    );

    // Exactly one cylindrical wall, and it must reach both end caps.
    let walls: Vec<_> = remus_topology::explorer::solid_faces(&topo, result.solid)
        .expect("faces")
        .into_iter()
        .filter(|&f| {
            matches!(topo.face(f).expect("face").surface(),
                FaceSurface::Cylinder(c) if (c.radius() - R).abs() < 1e-9)
        })
        .collect();
    assert_eq!(walls.len(), 1, "one ridgeline should yield one blend wall");

    let wire = topo
        .wire(topo.face(walls[0]).expect("face").outer_wire())
        .expect("wire");
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for oe in wire.edges() {
        let e = topo.edge(oe.edge()).expect("edge");
        for vid in [e.start(), e.end()] {
            let z = topo.vertex(vid).expect("v").point().z();
            lo = lo.min(z);
            hi = hi.max(z);
        }
    }
    assert!(
        lo.abs() < 1e-6 && (hi - TOP).abs() < 1e-6,
        "blend wall should span z 0..{TOP}, got {lo:.3}..{hi:.3}"
    );
}

/// The spine must run end to end regardless of how the chain's edges happen
/// to be oriented in the topology. Edge orientation is a property of the
/// arena, not of the ridgeline: a chain containing a "backwards" edge used to
/// sample it from the wrong end, which put the blend cylinder's origin in the
/// middle of the chain and flipped its axis.
#[test]
fn split_column_fillet_is_orientation_independent() {
    // Both rear columns are the same shape mirrored, so any dependence on
    // incidental edge orientation shows up as a difference between them.
    let mut removals = Vec::new();
    for cx in [0.0, 80.0] {
        let mut topo = Topology::new();
        let solid = l_blank(&mut topo);
        let seed = rear_column_base(&topo, solid, cx).expect("rear corner column");

        let before = solid_volume(&topo, solid, 0.01).expect("before");
        let result = {
            let mut builder = FilletBuilder::new(&mut topo, solid);
            builder.add_edges(&[seed], R);
            builder.build().expect("fillet")
        };
        let shell = topo.solid(result.solid).expect("solid").outer_shell();
        validate_shell_closed(topo.shell(shell).expect("shell"), &topo)
            .unwrap_or_else(|e| panic!("column at x={cx} not watertight: {e:?}"));
        let after = solid_volume(&topo, result.solid, 0.01).expect("after");
        removals.push(before - after);
    }

    let first = removals[0];
    for (i, removed) in removals.iter().enumerate() {
        assert!(
            (removed - first).abs() < 0.01,
            "column {i} removed {removed:.3} but the first removed {first:.3} — \
             the result depends on incidental edge orientation"
        );
    }
}
