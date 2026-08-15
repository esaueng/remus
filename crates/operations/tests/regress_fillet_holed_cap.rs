//! Regression: rim FILLET on a cap that carries holes.
//!
//! `fillet_builder`'s annular rim rebuild (`closed_rim_info` /
//! `assemble_closed_rim`) was gated on the cap being a bare disc, because the
//! rebuild handed the new cap an empty inner-wire list. That restriction is the
//! twin of the one fixed for chamfer in #28: the drilled flange's rim cap is an
//! annulus with a central opening and six bolt holes, so filleting any of its
//! rims fell through to the per-face trimmer and reported "trimming failure".
//!
//! Nobody hit it because the demos fillet boxes, not annular caps — the
//! chamfer twin surfaced first only because the flange demo chamfers its rim.
//!
//! Dropping the cap's holes would be worse than failing: every opening would
//! be filled in and each bore wall would lose the face it pairs with, opening
//! the shell while still looking plausible.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::unify_faces;
use remus_operations::measure;
use remus_operations::primitives;
use remus_operations::revolve::revolve;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::builder::{make_planar_face_from_wire, make_polygon_wire};
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

fn surface_census(topo: &Topology, s: SolidId) -> HashMap<&'static str, usize> {
    let mut m = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        *m.entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    m
}

fn brep_edge_health(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
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

fn mesh_edge_health(topo: &Topology, s: SolidId) -> (usize, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, s, 0.01, 0.1).unwrap();
    let q = 1e6;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for &(a, b) in &[(v[0], v[1]), (v[1], v[2]), (v[2], v[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c >= 3).count(),
    )
}

fn solid_edge_list(topo: &Topology, s: SolidId) -> Vec<EdgeId> {
    let mut seen = Vec::new();
    for fid in solid_faces(topo, s).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                if !seen.contains(&oe.edge()) {
                    seen.push(oe.edge());
                }
            }
        }
    }
    seen
}

/// The drilled flange: rim r24..45 z0..10 fused with hub r12..24 z0..26, then
/// six r3 bolt holes on a 34mm circle.
fn drilled_flange(t: &mut Topology) -> SolidId {
    let revolved = |t: &mut Topology, ri: f64, ro: f64, z0: f64, z1: f64| {
        let pts = [
            Point3::new(ri, 0.0, z0),
            Point3::new(ro, 0.0, z0),
            Point3::new(ro, 0.0, z1),
            Point3::new(ri, 0.0, z1),
        ];
        let w = make_polygon_wire(t, &pts, 1e-7).unwrap();
        let f = make_planar_face_from_wire(t, w).unwrap();
        revolve(
            t,
            f,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::TAU,
        )
        .unwrap()
    };
    let rim = revolved(t, 24.0, 45.0, 0.0, 10.0);
    let hub = revolved(t, 12.0, 24.0, 0.0, 26.0);
    let blank = boolean(t, BooleanOp::Fuse, rim, hub).unwrap();
    unify_faces(t, blank).unwrap();

    let mut pattern = None;
    for i in 0..6 {
        let a = std::f64::consts::TAU * f64::from(i) / 6.0;
        let c = primitives::make_cylinder(t, 3.0, 16.0).unwrap();
        transform_solid(
            t,
            c,
            &Mat4::translation(34.0 * a.cos(), 34.0 * a.sin(), -3.0),
        )
        .unwrap();
        pattern = Some(match pattern {
            None => c,
            Some(p) => boolean(t, BooleanOp::Fuse, p, c).unwrap(),
        });
    }
    boolean(t, BooleanOp::Cut, blank, pattern.unwrap()).unwrap()
}

/// Material a convex rim fillet of radius `r` removes from a rim of radius
/// `big`, by Pappus.
///
/// The removed section is the square corner `r x r` minus the quarter-disc the
/// rolling ball leaves, area `r^2(1 - pi/4)`. Its centroid sits at
/// `[(R - r/2) - (pi/4)(R - r) - r/3] / (1 - pi/4)` from the axis — the square's
/// first moment minus the quarter-disc's, over the remaining area.
fn rim_fillet_volume(big: f64, r: f64) -> f64 {
    use std::f64::consts::PI;
    let area = r * r * (1.0 - PI / 4.0);
    let centroid = ((big - r / 2.0) - (PI / 4.0) * (big - r) - r / 3.0) / (1.0 - PI / 4.0);
    area * std::f64::consts::TAU * centroid
}

/// The two r45 rims and the r24 hub lip.
fn flange_rims(topo: &Topology, body: SolidId) -> Vec<EdgeId> {
    solid_edge_list(topo, body)
        .into_iter()
        .filter(|&e| {
            let ed = topo.edge(e).unwrap();
            if ed.start() != ed.end() {
                return false;
            }
            let p = topo.vertex(ed.start()).unwrap().point();
            let r = p.x().hypot(p.y());
            (r - 45.0).abs() < 1e-6 || ((r - 24.0).abs() < 1e-6 && p.z() >= 25.5)
        })
        .collect()
}

#[test]
fn rim_fillet_preserves_cap_holes() {
    let mut topo = Topology::new();
    let body = drilled_flange(&mut topo);
    let rims = flange_rims(&topo, body);
    assert_eq!(rims.len(), 3, "two r45 rims and the r24 hub lip");

    // Guard the premise: these caps must really be holed annuli, or this test
    // silently degrades into re-testing the bare-disc case.
    let holed_caps = solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let face = topo.face(f).unwrap();
            face.surface().type_tag() == "plane" && !face.inner_wires().is_empty()
        })
        .count();
    assert!(holed_caps >= 2, "the flange caps must be holed annuli");

    let before = measure::solid_volume(&topo, body, 0.05).unwrap();
    let r = 1.5;
    let result =
        blend_ops::fillet_v2(&mut topo, body, &rims, r).expect("all three flange rims must fillet");
    assert!(result.failed.is_empty(), "{:?}", result.failed);

    // One toroidal band per rim; the nine cylinders (3 body + 6 bores) survive.
    let census = surface_census(&topo, result.solid);
    assert_eq!(
        census.get("torus").copied().unwrap_or(0),
        3,
        "one exact torus band per rim: {census:?}"
    );
    assert_eq!(
        census.get("cylinder").copied().unwrap_or(0),
        9,
        "3 body walls + 6 bores: {census:?}"
    );

    assert_eq!(
        brep_edge_health(&topo, result.solid),
        (0, 0),
        "a dropped cap hole would strand its bore wall and open the shell"
    );
    assert_eq!(
        mesh_edge_health(&topo, result.solid),
        (0, 0),
        "and the tessellation must be watertight"
    );

    // Volume against the exact closed form. A convex rim fillet removes the
    // square corner minus the quarter-disc the ball leaves behind; by Pappus
    // that is `area * 2pi * centroid_radius`. Note this is LESS than a chamfer
    // of the same setback would remove — the rounded corner is fuller than the
    // triangular one — which is the direction that catches a fillet applied on
    // the wrong side.
    let after = measure::solid_volume(&topo, result.solid, 0.05).unwrap();
    assert!(after < before, "a convex rim fillet must remove material");
    let removed = before - after;
    let want: f64 = [45.0_f64, 45.0, 24.0]
        .iter()
        .map(|&big| rim_fillet_volume(big, r))
        .sum();
    assert!(
        (removed - want).abs() / want < 5e-3,
        "removed {removed} vs closed form {want}"
    );

    // The bolt holes must still be holes.
    let opts = ClassifyOptions::default();
    assert_eq!(
        classify_point(&topo, result.solid, Point3::new(34.0, 0.0, 5.0), &opts).unwrap(),
        PointClassification::Outside,
        "a bolt hole must survive the fillet"
    );
    assert_eq!(
        classify_point(&topo, result.solid, Point3::new(0.0, 0.0, 20.0), &opts).unwrap(),
        PointClassification::Outside,
        "the r12 bore stays open"
    );
    assert_eq!(
        classify_point(&topo, result.solid, Point3::new(18.0, 0.0, 20.0), &opts).unwrap(),
        PointClassification::Inside,
        "the hub wall is still solid"
    );
}

/// The rim must round INWARD, not flare outward.
///
/// This is the sharp end of the bug. `plane_is_bounded_disc` bailed on any cap
/// with inner wires, so an annular cap never registered as a cylinder's own rim
/// and was treated as a plate the cylinder stands on — the fillet flared
/// outward with plate contact at `r_c + r`. On a washer that put the cap's
/// outer boundary at r=25.5 while its wall stayed at r=24, so the cap passed
/// straight through the wall.
///
/// The result was self-intersecting and still reported zero free and zero
/// non-manifold edges, which is why this asserts on measured geometry rather
/// than on topology counts.
#[test]
fn washer_rim_fillet_rounds_inward() {
    let mut topo = Topology::new();
    let washer = {
        let pts = [
            Point3::new(12.0, 0.0, 0.0),
            Point3::new(24.0, 0.0, 0.0),
            Point3::new(24.0, 0.0, 26.0),
            Point3::new(12.0, 0.0, 26.0),
        ];
        let w = make_polygon_wire(&mut topo, &pts, 1e-7).unwrap();
        let f = make_planar_face_from_wire(&mut topo, w).unwrap();
        revolve(
            &mut topo,
            f,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::TAU,
        )
        .unwrap()
    };

    let rim = solid_edge_list(&topo, washer)
        .into_iter()
        .find(|&e| {
            let ed = topo.edge(e).unwrap();
            if ed.start() != ed.end() {
                return false;
            }
            let p = topo.vertex(ed.start()).unwrap().point();
            (p.x().hypot(p.y()) - 24.0).abs() < 1e-6 && (p.z() - 26.0).abs() < 1e-6
        })
        .expect("washer top rim");

    let r = 1.5;
    let before = measure::solid_volume(&topo, washer, 0.002).unwrap();
    let result = blend_ops::fillet_v2(&mut topo, washer, &[rim], r).expect("washer rim fillet");

    // The band's major radius must be r_c - r (rounding in), never r_c + r.
    let major = solid_faces(&topo, result.solid)
        .unwrap()
        .into_iter()
        .find_map(|f| match topo.face(f).unwrap().surface() {
            remus_topology::face::FaceSurface::Torus(t) => Some(t.major_radius()),
            _ => None,
        })
        .expect("a torus band");
    assert!(
        (major - 22.5).abs() < 1e-9,
        "band major radius {major} must be 24 - 1.5; 25.5 means it flared outward"
    );

    // No face may reach beyond the wall it is bounded by.
    for fid in solid_faces(&topo, result.solid).unwrap() {
        let f = topo.face(fid).unwrap();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let p = topo
                    .vertex(topo.edge(oe.edge()).unwrap().start())
                    .unwrap()
                    .point();
                assert!(
                    p.x().hypot(p.y()) <= 24.0 + 1e-6,
                    "nothing may extend past the r24 wall, found {p:?}"
                );
                assert!(
                    p.z() <= 26.0 + 1e-6,
                    "nothing may extend past the z26 cap, found {p:?}"
                );
            }
        }
    }

    // And the exact closed form, which a wrong-side fillet cannot satisfy.
    let after = measure::solid_volume(&topo, result.solid, 0.002).unwrap();
    let want = rim_fillet_volume(24.0, r);
    assert!(
        ((before - after) - want).abs() / want < 1e-6,
        "removed {} vs closed form {want}",
        before - after
    );
}

/// A fillet radius large enough to reach a bolt hole must be refused, not
/// silently emit a cap whose outer wire crosses its own hole.
#[test]
fn rim_fillet_reaching_a_hole_is_refused() {
    let mut topo = Topology::new();
    let body = drilled_flange(&mut topo);
    // The nearest bolt-hole edge sits at radius 37; a radius of 10 pulls the
    // r45 cap contact in to 35, past it.
    let rim = flange_rims(&topo, body)
        .into_iter()
        .find(|&e| {
            let p = topo.vertex(topo.edge(e).unwrap().start()).unwrap().point();
            (p.x().hypot(p.y()) - 45.0).abs() < 1e-6 && (p.z() - 10.0).abs() < 1e-6
        })
        .expect("top r45 rim");

    let before = measure::solid_volume(&topo, body, 0.05).unwrap();
    let outcome = blend_ops::fillet_v2(&mut topo, body, &[rim], 10.0);
    assert!(
        outcome.is_err(),
        "a radius that reaches the bolt circle must be refused"
    );
    let after = measure::solid_volume(&topo, body, 0.05).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "the refused fillet must leave the input untouched"
    );
}

/// A circular cap hole can hide its farthest point between the old nine fixed
/// samples just as a straight plate edge can hide its nearest point. Put an
/// r=3 hole 34 mm off the r=45 cap axis at 22.5 degrees: its true radial reach
/// is 37 mm, while samples every 45 degrees see at most about 36.79 mm. An
/// outer-rim fillet of 8.1 mm shrinks the cap contact to r=36.9 and therefore
/// crosses the hole.
#[test]
fn off_axis_cap_hole_crossing_is_refused_without_losing_supported_fillet() {
    const BODY_R: f64 = 45.0;
    const BODY_H: f64 = 10.0;
    const HOLE_R: f64 = 3.0;
    const OFFSET: f64 = 34.0;
    const CLEARANCE: f64 = BODY_R - OFFSET - HOLE_R;

    let angle = std::f64::consts::PI / 8.0;
    let hole_x = OFFSET * angle.cos();
    let hole_y = OFFSET * angle.sin();

    let mut topo = Topology::new();
    let blank = {
        let points = [
            Point3::new(12.0, 0.0, 0.0),
            Point3::new(BODY_R, 0.0, 0.0),
            Point3::new(BODY_R, 0.0, BODY_H),
            Point3::new(12.0, 0.0, BODY_H),
        ];
        let wire = make_polygon_wire(&mut topo, &points, 1e-7).unwrap();
        let face = make_planar_face_from_wire(&mut topo, wire).unwrap();
        revolve(
            &mut topo,
            face,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::TAU,
        )
        .unwrap()
    };
    let drill = primitives::make_cylinder(&mut topo, HOLE_R, BODY_H + 4.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(hole_x, hole_y, -2.0)).unwrap();
    let body = boolean(&mut topo, BooleanOp::Cut, blank, drill).unwrap();

    let rim = solid_edge_list(&topo, body)
        .into_iter()
        .find(|&e| {
            let edge = topo.edge(e).unwrap();
            let p = topo.vertex(edge.start()).unwrap().point();
            edge.start() == edge.end()
                && matches!(edge.curve(), EdgeCurve::Circle(c) if (c.radius() - BODY_R).abs() < 1e-9)
                && (p.z() - BODY_H).abs() < 1e-9
        })
        .expect("the holed cylinder has a top outer rim");

    // Guard the adversarial premise: the obsolete nine-point scan really
    // misses this crossing on the actual post-boolean hole circle.
    let cap = solid_faces(&topo, body)
        .unwrap()
        .into_iter()
        .find(|&face_id| {
            let face = topo.face(face_id).unwrap();
            face.surface().type_tag() == "plane"
                && topo
                    .wire(face.outer_wire())
                    .unwrap()
                    .edges()
                    .iter()
                    .any(|edge| edge.edge() == rim)
        })
        .expect("the top cap uses the selected outer rim");
    let hole_edge = topo
        .face(cap)
        .unwrap()
        .inner_wires()
        .iter()
        .flat_map(|&wire_id| topo.wire(wire_id).unwrap().edges())
        .map(|edge| topo.edge(edge.edge()).unwrap())
        .find(|edge| {
            matches!(edge.curve(), EdgeCurve::Circle(circle)
                if (circle.radius() - HOLE_R).abs() < 1e-9)
        })
        .expect("the cap keeps the off-axis circular hole");
    let start = topo.vertex(hole_edge.start()).unwrap().point();
    let end = topo.vertex(hole_edge.end()).unwrap().point();
    let (t0, t1) = hole_edge.curve().domain_with_endpoints(start, end);
    let mut legacy_max: f64 = 0.0;
    for k in 0..=8 {
        let t = t0 + (t1 - t0) * f64::from(k) / 8.0;
        let point = hole_edge.curve().evaluate_with_endpoints(t, start, end);
        legacy_max = legacy_max.max(point.x().hypot(point.y()));
    }
    let rejected_contact = BODY_R - (CLEARANCE + 0.1);
    assert!(
        legacy_max < rejected_contact,
        "the fixture must evade the obsolete samples: max={legacy_max}, contact={rejected_contact}"
    );

    // Preserve the same analytic rebuild just below the exact clearance.
    {
        let mut supported = topo.clone();
        let ok = blend_ops::fillet_v2(&mut supported, body, &[rim], CLEARANCE - 0.1)
            .expect("an outer-rim fillet below the exact hole clearance must remain supported");
        assert!(!ok.is_partial);
        assert_eq!(brep_edge_health(&supported, ok.solid), (0, 0));
        assert_eq!(mesh_edge_health(&supported, ok.solid), (0, 0));
        assert_eq!(
            surface_census(&supported, ok.solid)
                .get("torus")
                .copied()
                .unwrap_or(0),
            1,
            "the supported path must still use one exact torus band"
        );
    }

    let before = measure::solid_volume(&topo, body, 0.01).unwrap();
    let outcome = blend_ops::fillet_v2(&mut topo, body, &[rim], CLEARANCE + 0.1);
    assert!(
        outcome.is_err(),
        "the r=36.9 cap contact crosses a hole reaching r=37 and must be refused"
    );
    let after = measure::solid_volume(&topo, body, 0.01).unwrap();
    assert!(
        (after - before).abs() < 1e-9,
        "the refused fillet must leave the input untouched"
    );
}
