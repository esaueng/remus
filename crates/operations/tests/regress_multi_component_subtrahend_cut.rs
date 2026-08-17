//! Regression: cutting a flange blank with a bolt circle supplied as ONE
//! multi-component subtrahend must stay analytic.
//!
//! The blank (rim r24..45 z0..10 fused with hub r12..24 z0..26, unified to 6
//! analytic faces) has two planar faces the bolt circle passes through: the
//! z=0 bottom disc and the z=10 rim cap. Each bolt contributes a CLOSED
//! circular section curve to both.
//!
//! `split_face_2d` routed a plane face to the direct internal-loops path only
//! when it carried exactly ONE closed section. With two or more, the face fell
//! through to the generic wire builder, which returned a single sub-face —
//! the holes were never carved. The bore walls then had nothing to attach to
//! and came out as their own one-face shells, which the assembler dropped as
//! open slivers. The GFA result was rejected for free boundary edges, the mesh
//! fallback was rejected too (8 boundary edges), and the whole Cut failed with
//! `NonManifoldResult`.
//!
//! Cutting the same holes one at a time always worked (one closed section per
//! face per boolean), which is what masked this until the coaxial fuse stopped
//! mesh-falling-back — see `regress_coaxial_annulus_fuse_same_domain`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::revolve::revolve;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const TOL: f64 = 1e-7;
const BOLT_R: f64 = 3.0;
const BOLT_CIRCLE_R: f64 = 34.0;

fn free_and_nonmanifold(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    (
        usage.values().filter(|&&c| c == 1).count(),
        usage.values().filter(|&&c| c >= 3).count(),
    )
}

fn surface_census(topo: &Topology, solid: SolidId) -> HashMap<&'static str, usize> {
    let mut census = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

/// Revolve a rectangular profile in the XZ plane about the Z axis.
fn revolved_annulus(
    topo: &mut Topology,
    r_inner: f64,
    r_outer: f64,
    z_lo: f64,
    z_hi: f64,
) -> SolidId {
    let pts = [
        Point3::new(r_inner, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_lo),
        Point3::new(r_outer, 0.0, z_hi),
        Point3::new(r_inner, 0.0, z_hi),
    ];
    let wire = make_polygon_wire(topo, &pts, TOL).unwrap();
    let face = make_planar_face_from_wire(topo, wire).unwrap();
    revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        std::f64::consts::TAU,
    )
    .unwrap()
}

/// The unified flange blank: 3 cylinders (r45 rim, r24 hub, r12 bore) and
/// 3 planes (z=0 disc, z=10 rim cap, z=26 hub cap).
fn flange_blank(topo: &mut Topology) -> SolidId {
    let rim = revolved_annulus(topo, 24.0, 45.0, 0.0, 10.0);
    let hub = revolved_annulus(topo, 12.0, 24.0, 0.0, 26.0);
    let blank = boolean(topo, BooleanOp::Fuse, rim, hub).expect("blank fuse must succeed");
    unify_faces(topo, blank).unwrap();
    blank
}

/// One bolt cylinder of an `n`-hole circle, long enough to pass clear through
/// the rim.
fn bolt(topo: &mut Topology, i: usize, n: usize) -> SolidId {
    #[allow(clippy::cast_precision_loss)]
    let angle = std::f64::consts::TAU * (i as f64) / (n as f64);
    let c = primitives::make_cylinder(topo, BOLT_R, 16.0).unwrap();
    transform_solid(
        topo,
        c,
        &Mat4::translation(
            BOLT_CIRCLE_R * angle.cos(),
            BOLT_CIRCLE_R * angle.sin(),
            -3.0,
        ),
    )
    .unwrap();
    c
}

/// Fuse `n` disjoint bolt cylinders into a single subtrahend body.
fn bolt_circle(topo: &mut Topology, n: usize) -> SolidId {
    let mut pattern = bolt(topo, 0, n);
    for i in 1..n {
        let next = bolt(topo, i, n);
        pattern = boolean(topo, BooleanOp::Fuse, pattern, next)
            .unwrap_or_else(|e| panic!("bolt-circle fuse {i} failed: {e:?}"));
    }
    pattern
}

/// N = 1 always worked; 2 through 6 all failed identically before the fix.
#[test]
fn multi_component_subtrahend_cut_stays_analytic() {
    for n in 1..=6usize {
        let mut topo = Topology::new();
        let blank = flange_blank(&mut topo);
        let pattern = bolt_circle(&mut topo, n);

        let drilled = boolean(&mut topo, BooleanOp::Cut, blank, pattern)
            .unwrap_or_else(|e| panic!("N={n}: multi-component subtrahend cut failed: {e:?}"));

        // 3 body cylinders + one wall per bolt, and the 3 original planes.
        // A mesh fallback lands in the hundreds and is all-planar.
        let census = surface_census(&topo, drilled);
        assert_eq!(
            census.get("cylinder").copied().unwrap_or(0),
            3 + n,
            "N={n}: 3 body cylinders + {n} bolt walls, got {census:?}"
        );
        assert_eq!(
            census.values().sum::<usize>(),
            6 + n,
            "N={n}: expected an analytic result, got {census:?}"
        );

        let (free, nonmanifold) = free_and_nonmanifold(&topo, drilled);
        assert_eq!(
            (free, nonmanifold),
            (0, 0),
            "N={n}: drilled flange must be watertight and manifold"
        );

        // Volume against the closed form: each bolt removes a r3 plug through
        // the 10mm rim.
        let pi = std::f64::consts::PI;
        #[allow(clippy::cast_precision_loss)]
        let expected = pi
            * ((45.0f64 * 45.0 - 24.0 * 24.0) * 10.0 + (24.0f64 * 24.0 - 12.0 * 12.0) * 26.0
                - (n as f64) * BOLT_R * BOLT_R * 10.0);
        let vol = measure::solid_volume(&topo, drilled, 0.05).unwrap();
        assert!(
            (vol - expected).abs() / expected < 0.005,
            "N={n}: volume {vol} vs analytic {expected}"
        );

        // Ray-cast ground truth: volume alone cannot tell a carved hole from
        // an uncarved one at this scale.
        let opts = ClassifyOptions::default();
        let probe = |p: Point3| classify_point(&topo, drilled, p, &opts).unwrap();
        for i in 0..n {
            #[allow(clippy::cast_precision_loss)]
            let a = std::f64::consts::TAU * (i as f64) / (n as f64);
            assert_eq!(
                probe(Point3::new(
                    BOLT_CIRCLE_R * a.cos(),
                    BOLT_CIRCLE_R * a.sin(),
                    5.0
                )),
                PointClassification::Outside,
                "N={n}: bolt {i} must be drilled through"
            );
            // Midway to the next hole (all the way round for n == 1) the rim
            // must still be solid.
            #[allow(clippy::cast_precision_loss)]
            let mid = a + std::f64::consts::TAU / (2.0 * n as f64);
            assert_eq!(
                probe(Point3::new(
                    BOLT_CIRCLE_R * mid.cos(),
                    BOLT_CIRCLE_R * mid.sin(),
                    5.0
                )),
                PointClassification::Inside,
                "N={n}: material must remain between bolts {i} and {}",
                (i + 1) % n
            );
        }
        assert_eq!(
            probe(Point3::new(18.0, 0.0, 20.0)),
            PointClassification::Inside,
            "N={n}: hub wall above the rim"
        );
        assert_eq!(
            probe(Point3::new(0.0, 0.0, 20.0)),
            PointClassification::Outside,
            "N={n}: the r12 bore stays open"
        );
    }
}

/// The multi-component subtrahend must agree with cutting the same holes one
/// at a time — the path that always worked.
#[test]
fn multi_component_subtrahend_matches_sequential_cuts() {
    const N: usize = 6;

    let mut topo = Topology::new();
    let blank = flange_blank(&mut topo);
    let pattern = bolt_circle(&mut topo, N);
    let together = boolean(&mut topo, BooleanOp::Cut, blank, pattern).expect("single-shot cut");

    let mut seq_topo = Topology::new();
    let mut current = flange_blank(&mut seq_topo);
    for i in 0..N {
        let b = bolt(&mut seq_topo, i, N);
        current = boolean(&mut seq_topo, BooleanOp::Cut, current, b)
            .unwrap_or_else(|e| panic!("sequential bolt {i} cut failed: {e:?}"));
    }

    assert_eq!(
        surface_census(&topo, together),
        surface_census(&seq_topo, current),
        "one multi-component cut must match six sequential cuts"
    );
    let v_together = measure::solid_volume(&topo, together, 0.05).unwrap();
    let v_seq = measure::solid_volume(&seq_topo, current, 0.05).unwrap();
    assert!(
        (v_together - v_seq).abs() / v_seq < 1e-3,
        "volumes must agree: {v_together} vs {v_seq}"
    );
}
