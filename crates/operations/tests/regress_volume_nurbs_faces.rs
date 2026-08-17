//! Regression: `solid_volume` must not depend on which face happens to be
//! reversed when the solid carries a curved face.
//!
//! `solid_volume` sends a solid to per-face summation when any face is
//! "reversed and non-planar", and to the whole-solid mesh otherwise. That test
//! is about topology, not accuracy, and per-face summation is only exact when
//! each face's own mesh tiles the same closed surface the solid does — which a
//! trimmed patch need not do.
//!
//! Filleting mirror-image corner columns of an L-blank produces mirror-image
//! solids whose blend walls land on opposite sides of that test, so the two
//! measured 12 mm³ apart: 49.03 and 36.68 where the closed mesh gives both the
//! same 34.7. The geometry was never at fault — only the measurement.
//!
//! The blend wall itself is a right circular cylinder (a constant radius along
//! a straight edge between two planes), so the premise below asks for a curved
//! blend face rather than a b-spline one — the split in `solid_volume` this
//! guards is "reversed and non-planar", which both satisfy.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const R: f64 = 2.0;
/// Corner columns run z = 0..39.5 on this blank.
const COLUMN: f64 = 39.5;

fn l_blank(topo: &mut Topology) -> SolidId {
    let plate = make_box(topo, 80.0, 40.0, 8.0).expect("plate");
    let wall = make_box(topo, 80.0, 8.0, 32.0).expect("wall");
    transform_solid(topo, wall, &Mat4::translation(0.0, 32.0, 7.5)).expect("seat wall");
    let fused = boolean(topo, BooleanOp::Fuse, plate, wall).expect("fuse");
    remus_operations::heal::unify_faces(topo, fused).expect("unify");
    fused
}

fn rear_column_base(topo: &Topology, solid: SolidId, cx: f64) -> EdgeId {
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
        .expect("rear corner column")
}

#[test]
fn blend_volume_is_orientation_independent() {
    let mut measured = Vec::new();
    let mut saw_curved_blend = false;

    for cx in [0.0, 80.0] {
        let mut topo = Topology::new();
        let solid = l_blank(&mut topo);
        let seed = rear_column_base(&topo, solid, cx);
        let before = solid_volume(&topo, solid, 0.01).expect("before");

        // The v1 rolling-ball engine is what emits the blend wall here; that
        // is the shape this regression is about, so call it directly.
        #[allow(deprecated)]
        let out = remus_operations::fillet::fillet_rolling_ball(&mut topo, solid, &[seed], R)
            .expect("fillet");

        // The premise: without a curved face the solid takes a different
        // volume path entirely and the test proves nothing.
        saw_curved_blend |= remus_topology::explorer::solid_faces(&topo, out)
            .expect("faces")
            .into_iter()
            .any(|f| {
                matches!(
                    topo.face(f).expect("face").surface(),
                    FaceSurface::Nurbs(_) | FaceSurface::Cylinder(_)
                )
            });

        let after = solid_volume(&topo, out, 0.01).expect("after");
        measured.push(before - after);
    }

    assert!(
        saw_curved_blend,
        "expected a curved blend wall on this shape"
    );
    let (a, b) = (measured[0], measured[1]);
    assert!(
        (a - b).abs() < 0.01,
        "mirror-image solids measured {a:.3} and {b:.3} — solid_volume still \
         depends on which face is flagged reversed"
    );

    // And the shared answer must be the real one: a quarter-round along the
    // full column removes (1 - pi/4)·r²·L = 33.9, which the inscribed mesh
    // over-reports slightly. Anything near 49 is the open per-face integral.
    let exact = (1.0 - std::f64::consts::FRAC_PI_4) * R * R * COLUMN;
    assert!(
        (a - exact).abs() < 3.0,
        "expected ≈{exact:.2} mm³ removed, got {a:.3}"
    );
}
