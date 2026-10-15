//! Differential oracles for the ray classifier's torus-trim memo and torus
//! line cull on the imported hammer holder, whose NURBS-bordered blends take
//! the sampled torus trims.
//!
//! * Booleans on the holder (general and store-local, cut, fuse and
//!   intersect) give byte-identical result arenas, or identical refusals, with
//!   the memo off, cold and warm.
//! * Every cardinal and generic ray triple from a lattice over the holder and
//!   from points on and next to every torus face has the same parity and
//!   suspicion with the cull as through the vote loop before it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_algo::classifier::{RayCastGeoms, ray_cast_rays_cached};
use remus_algo::ray_geom_memo::{
    enable_thread_ray_geom_memo, set_thread_ray_geom_memo_capacity, thread_ray_geom_memo_stats,
};
use remus_io::step::reader::read_step;
use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, boolean, set_thread_store_local_cuts};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn hammer() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid =
        read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo).expect("import")[0];
    (topo, solid)
}

type Tool = fn(&mut Topology) -> SolidId;

fn placed(topo: &mut Topology, solid: SolidId, at: (f64, f64, f64)) -> SolidId {
    transform_solid(topo, solid, &Mat4::translation(at.0, at.1, at.2)).unwrap();
    solid
}

/// Tools in the base plate, through an arm, against the blended rims, and the
/// half-holder mask of the opening experiment.
fn cases() -> Vec<(&'static str, BooleanOp, Tool)> {
    vec![
        ("slot", BooleanOp::Cut, |t| {
            let b = make_box(t, 4.0, 12.0, 20.0).unwrap();
            placed(t, b, (10.0, 44.0, 0.0))
        }),
        ("hole", BooleanOp::Cut, |t| {
            let c = make_cylinder(t, 2.0, 30.0).unwrap();
            placed(t, c, (20.0, 50.0, -5.0))
        }),
        ("rib", BooleanOp::Fuse, |t| {
            let b = make_box(t, 3.0, 10.0, 10.0).unwrap();
            placed(t, b, (47.0, 15.0, 20.0))
        }),
        ("blend nick", BooleanOp::Cut, |t| {
            let b = make_box(t, 3.0, 3.0, 3.0).unwrap();
            placed(t, b, (30.0, 38.5, 11.0))
        }),
        ("arm slot", BooleanOp::Cut, |t| {
            let b = make_box(t, 20.0, 4.0, 6.0).unwrap();
            placed(t, b, (30.0, 15.0, 30.0))
        }),
        ("mask common", BooleanOp::Intersect, |t| {
            let b = make_box(t, 29.0, 53.0, 70.0).unwrap();
            placed(t, b, (-18.0, -10.0, 0.0))
        }),
    ]
}

fn outcomes(source: &Topology, solid: SolidId) -> Vec<String> {
    let mut out = Vec::new();
    for store_local in [false, true] {
        set_thread_store_local_cuts(store_local);
        for (name, op, tool) in cases() {
            let mut topo = source.clone();
            let b = tool(&mut topo);
            let result = boolean(&mut topo, op, solid, b).map(|r| {
                (
                    r.index(),
                    remus_io::arena_io::serialize_solid(&topo, r).unwrap(),
                )
            });
            out.push(format!("{name} store_local={store_local}: {result:?}"));
        }
    }
    set_thread_store_local_cuts(false);
    out
}

#[test]
fn torus_trim_memo_leaves_hammer_booleans_bit_identical() {
    let (source, solid) = hammer();
    set_thread_ray_geom_memo_capacity(0);
    let reference = outcomes(&source, solid);
    enable_thread_ray_geom_memo();
    let cold = outcomes(&source, solid);
    let stored = thread_ray_geom_memo_stats();
    let warm = outcomes(&source, solid);
    let recalled = thread_ray_geom_memo_stats();
    set_thread_ray_geom_memo_capacity(0);

    for (k, reference) in reference.iter().enumerate() {
        assert_eq!(&cold[k], reference, "cold memo changed a result");
        assert_eq!(&warm[k], reference, "warm memo changed a result");
    }
    // The holder's sampled trims are stored once and recalled on every later
    // boolean against it.
    assert!(stored.misses >= 4 && stored.hits > 0, "{stored:?}");
    assert_eq!(recalled.misses, stored.misses, "{stored:?} -> {recalled:?}");
    assert!(recalled.hits > stored.hits, "{stored:?} -> {recalled:?}");
}

#[test]
fn torus_cull_matches_the_unculled_votes_on_the_hammer() {
    let (topo, solid) = hammer();
    let geoms = RayCastGeoms::new(&topo, solid).unwrap();
    let tol = Tolerance::default();
    let mut points = Vec::new();
    // A lattice over the holder's box and a margin around it.
    let (lo, hi) = (Point3::new(-30.0, 2.0, 0.0), Point3::new(52.0, 64.0, 67.0));
    let n = 22;
    for i in 0..=n {
        for j in 0..=n {
            for k in 0..=n {
                let f = |a: f64, b: f64, s: i32| a + (b - a) * (f64::from(s) + 0.31) / f64::from(n);
                points.push(Point3::new(
                    f(lo.x(), hi.x(), i),
                    f(lo.y(), hi.y(), j),
                    f(lo.z(), hi.z(), k),
                ));
            }
        }
    }
    // On and next to every torus face, where rays graze the carriers.
    let mut tori = 0;
    for face in solid_faces(&topo, solid).unwrap() {
        let FaceSurface::Torus(torus) = topo.face(face).unwrap().surface() else {
            continue;
        };
        tori += 1;
        for a in 0..24 {
            for b in 0..12 {
                let (u, v) = (
                    std::f64::consts::TAU * f64::from(a) / 24.0,
                    std::f64::consts::TAU * (f64::from(b) + 0.5) / 12.0,
                );
                let p = torus.evaluate(u, v);
                let normal = torus.normal(u, v);
                for offset in [0.0, 1e-6, -1e-6, 1e-3, -1e-3, 0.5] {
                    points.push(p + normal * offset);
                }
            }
        }
    }
    assert_eq!(tori, 14, "the holder's torus faces");
    for point in points {
        assert_eq!(
            ray_cast_rays_cached(&geoms, point, tol, true),
            ray_cast_rays_cached(&geoms, point, tol, false),
            "rays from {point:?}"
        );
    }
}
