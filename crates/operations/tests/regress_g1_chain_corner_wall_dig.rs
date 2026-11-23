//! A convex fillet chained around vertical corner fillets stays in its band
//! (B267).
//!
//! A 74x60x4 plate gets r = 6 fillets on its two back vertical corners, then
//! an r = 1 fillet on its back top edge. The selection expands along the G1
//! chain: back edge, the two quarter arcs on top of the r = 6 corners, and the
//! two side edges down to the front. The rolling ball used to build the arc
//! stretches as lofted NURBS walls against the corner cylinders, and those
//! walls dipped through material: points inside the plate, well below the
//! fillet band (z < 3, under the cylinder's contact circle at z = 3), came
//! out outside while the result stayed valid and watertight.
//!
//! The arc stretches are now exact tori. The band only removes points within
//! one radius of the filleted top edges, so no point below z = 3, and no
//! point at z = 3.5 more than one radius in from the rounded outline, may
//! change sides; the removed volume is the spandrel swept along the chain
//! (Pappus over the two arcs).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use remus_math::vec::Point3;
use remus_operations::blend_ops::fillet_cascade;
use remus_operations::classify::classify_points;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const CORNER_R: f64 = 6.0;
const BAND_R: f64 = 1.0;

fn edge_where(topo: &Topology, solid: SolidId, test: impl Fn(Point3, Point3) -> bool) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            test(a, b)
        })
        .expect("edge")
}

fn count_faces(topo: &Topology, solid: SolidId, pick: impl Fn(&FaceSurface) -> bool) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| pick(topo.face(f).unwrap().surface()))
        .count()
}

/// Horizontal distance in from the filleted part of the rounded outline:
/// the back edge, the two side edges and the two r = 6 corner arcs.
fn depth_in_from_filleted_outline(x: f64, y: f64) -> f64 {
    let back = 60.0 - CORNER_R;
    if y > back && x < CORNER_R {
        CORNER_R - (x - CORNER_R).hypot(y - back)
    } else if y > back && x > 74.0 - CORNER_R {
        CORNER_R - (x - (74.0 - CORNER_R)).hypot(y - back)
    } else {
        x.min(74.0 - x).min(60.0 - y)
    }
}

#[test]
fn g1_chain_around_corner_fillets_stays_in_its_band() {
    let near = |a: f64, b: f64| (a - b).abs() < 1e-6;
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 74.0, 60.0, 4.0).unwrap();
    let corners: Vec<EdgeId> = [0.0, 74.0]
        .into_iter()
        .map(|x| {
            edge_where(&topo, plate, |a, b| {
                near(a.x(), x) && near(b.x(), x) && near(a.y(), 60.0) && near(b.y(), 60.0)
            })
        })
        .collect();
    let rounded = fillet_cascade(&mut topo, plate, &corners, CORNER_R)
        .unwrap()
        .solid;
    let back = edge_where(&topo, rounded, |a, b| {
        near(a.y(), 60.0) && near(b.y(), 60.0) && near(a.z(), 4.0) && near(b.z(), 4.0)
    });
    let banded = fillet_cascade(&mut topo, rounded, &[back], BAND_R)
        .unwrap()
        .solid;

    // Exact: the arc stretches are tori, and nothing is lofted.
    let nurbs = count_faces(&topo, banded, |s| matches!(s, FaceSurface::Nurbs(_)));
    assert_eq!(nurbs, 0, "{nurbs} NURBS faces, expected exact analytic");
    let tori_before = count_faces(&topo, rounded, |s| matches!(s, FaceSurface::Torus(_)));
    let tori_after = count_faces(&topo, banded, |s| matches!(s, FaceSurface::Torus(_)));
    assert_eq!(
        tori_after - tori_before,
        2,
        "one exact torus band per corner arc"
    );
    // Each torus rolls the r = 1 ball round its corner: centred on the
    // corner cylinder's axis one radius below the top, major radius R − r.
    let mut centres = Vec::new();
    for face in solid_faces(&topo, banded).unwrap() {
        if let FaceSurface::Torus(t) = topo.face(face).unwrap().surface() {
            assert!((t.major_radius() - (CORNER_R - BAND_R)).abs() < 1e-9);
            assert!((t.minor_radius() - BAND_R).abs() < 1e-9);
            assert!(t.z_axis().z().abs() > 1.0 - 1e-12, "axis {:?}", t.z_axis());
            centres.push(t.center());
        }
    }
    for x in [CORNER_R, 74.0 - CORNER_R] {
        let expected = Point3::new(x, 60.0 - CORNER_R, 4.0 - BAND_R);
        assert!(
            centres.iter().any(|c| (*c - expected).length() < 1e-9),
            "no torus centred at {expected:?}: {centres:?}"
        );
    }

    let report = validate_solid(&topo, banded).unwrap();
    assert!(report.is_valid(), "invalid: {:?}", report.issues);
    let quality = welded_mesh_quality(&tessellate_solid(&topo, banded, 0.1).unwrap());
    assert!(quality.is_watertight(), "mesh {quality:?}");

    // Closed form: the spandrel r²(1 − π/4) along 62 mm of back edge, two
    // 54 mm side edges, and (Pappus) two quarter turns about the corner axes
    // at the spandrel centroid's radius, (10 − 3π)/(3(4 − π))·r in from the
    // outline.
    let spandrel = BAND_R * BAND_R * (1.0 - FRAC_PI_4);
    let centroid = (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * BAND_R;
    let straight = (74.0 - 2.0 * CORNER_R) + 2.0 * (60.0 - CORNER_R);
    let removed = spandrel * (straight + 2.0 * FRAC_PI_2 * (CORNER_R - centroid));
    let before = solid_volume(&topo, rounded, 0.01).unwrap();
    let after = solid_volume(&topo, banded, 0.01).unwrap();
    assert!(
        ((before - after) - removed).abs() < 1e-6,
        "volume change {} mm³, closed form {removed} mm³",
        before - after
    );

    // Material side: every point below the band, and every point in the band's
    // layer more than one radius (plus a margin) in from the filleted outline,
    // keeps its side.
    let mut points = Vec::new();
    for i in 0..=296 {
        for j in 0..=240 {
            let (x, y) = (f64::from(i) * 0.25 + 0.01, f64::from(j) * 0.25 + 0.013);
            for k in 0..12 {
                points.push(Point3::new(x, y, f64::from(k) * 0.25 + 0.11));
            }
            if depth_in_from_filleted_outline(x, y) > BAND_R + 0.05 {
                points.push(Point3::new(x, y, 3.5));
            }
        }
    }
    let before = classify_points(&topo, rounded, &points, 0.01, 1e-7).unwrap();
    let after = classify_points(&topo, banded, &points, 0.01, 1e-7).unwrap();
    let changed: Vec<Point3> = points
        .iter()
        .zip(before.iter().zip(&after))
        .filter(|(_, (b, a))| b != a)
        .map(|(p, _)| *p)
        .collect();
    assert!(
        changed.is_empty(),
        "{} points outside the band changed side, e.g. {:?}",
        changed.len(),
        changed.first()
    );
}
