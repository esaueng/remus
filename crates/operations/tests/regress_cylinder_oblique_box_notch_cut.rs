//! Cylinder minus an obliquely placed box that notches the cylinder's top
//! cap rim (B26 proptest seed `c36b6629`).
//!
//! Stock: cylinder r = 2, h = 2 on the z axis. Tool: box 4 × 1.5 × 1.5,
//! rotated π/4 about z, then translated to (2.25, −0.25, 1.75). Everything is
//! scaled ×1000 in the generated case. The box footprint clips a circular
//! segment-shaped region off the cap; its floor at z = 1.75 sits inside the
//! cylinder, so the cut leaves a step notch a quarter-unit deep.
//!
//! One box wall passes exactly through (2, 0), the cylinder's seam line, so
//! the notch's floor arc ends ON the seam and the seam's upper piece is the
//! notch's fourth side. Both seam uses of the lateral store the same `u`
//! copy; the greedy wire walker turned from the seam-up use onto the floor
//! arc, closed the notch early and discarded the bottom rim. The lateral came
//! back as one inside-out band with no bottom rim, the bottom cap was dropped
//! as an open sliver, and the result carried seven same-sense shared edges
//! (`ShellOrientationConsistent`) at every scale — and about two thirds of
//! the true volume. Fixed by adopting the face splitter's DCEL trace when the
//! greedy trace orphans a boundary edge.
//!
//! The oracles below are topological first (both validators, every edge used
//! twice in opposite senses, a closed manifold mesh), then
//! ground-truth classification probes, then volume against an exact closed
//! form computed here from the disc ∩ footprint area.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::FRAC_PI_4;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{Severity, ValidateOptions};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

const R: f64 = 2.0;
const H: f64 = 2.0;
const BOX: (f64, f64, f64) = (4.0, 1.5, 1.5);
const OFFSET: (f64, f64, f64) = (2.25, -0.25, 1.75);

/// The tool box's footprint corners in the xy plane (unit scale), in
/// counter-clockwise order: the box's own xy rectangle rotated π/4 about z
/// and then translated.
fn footprint() -> [(f64, f64); 4] {
    let (c, s) = (FRAC_PI_4.cos(), FRAC_PI_4.sin());
    let place = |x: f64, y: f64| (c * x - s * y + OFFSET.0, s * x + c * y + OFFSET.1);
    [
        place(0.0, 0.0),
        place(BOX.0, 0.0),
        place(BOX.0, BOX.1),
        place(0.0, BOX.1),
    ]
}

/// Signed area of the disc of radius `r` (centred at the origin) intersected
/// with the triangle (origin, `a`, `b`). Summed over a convex polygon's
/// edges this is the exact disc ∩ polygon area: straight pieces contribute
/// triangle areas, pieces outside the disc contribute circular sectors.
fn disc_triangle_area(r: f64, a: (f64, f64), b: (f64, f64)) -> f64 {
    let cross = |p: (f64, f64), q: (f64, f64)| p.0 * q.1 - p.1 * q.0;
    let sector =
        |p: (f64, f64), q: (f64, f64)| 0.5 * r * r * cross(p, q).atan2(p.0 * q.0 + p.1 * q.1);
    // Parameters t in (0, 1) where |a + t (b − a)| = r.
    let d = (b.0 - a.0, b.1 - a.1);
    let qa = d.0 * d.0 + d.1 * d.1;
    let qb = 2.0 * (a.0 * d.0 + a.1 * d.1);
    let qc = a.0 * a.0 + a.1 * a.1 - r * r;
    let at = |t: f64| (a.0 + t * d.0, a.1 + t * d.1);
    let mut cuts = vec![0.0];
    let disc = qb * qb - 4.0 * qa * qc;
    if disc > 0.0 {
        let sq = disc.sqrt();
        for t in [(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)] {
            if t > 0.0 && t < 1.0 {
                cuts.push(t);
            }
        }
    }
    cuts.push(1.0);
    cuts.windows(2)
        .map(|w| {
            let (p, q) = (at(w[0]), at(w[1]));
            let m = at(0.5 * (w[0] + w[1]));
            if m.0 * m.0 + m.1 * m.1 <= r * r {
                0.5 * cross(p, q)
            } else {
                sector(p, q)
            }
        })
        .sum()
}

/// Exact volume of the cut at unit scale: the cylinder minus the notch, a
/// prism of height `H − OFFSET.2` over the disc ∩ footprint region.
fn closed_form_volume() -> f64 {
    let f = footprint();
    let area: f64 = (0..4)
        .map(|i| disc_triangle_area(R, f[i], f[(i + 1) % 4]))
        .sum();
    assert!(area > 0.0, "footprint must clip the disc, area {area}");
    std::f64::consts::PI * R * R * H - area * (H - OFFSET.2)
}

fn build_cut(scale: f64) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let stock = make_cylinder(&mut topo, R * scale, H * scale).unwrap();
    let tool = make_box(&mut topo, BOX.0 * scale, BOX.1 * scale, BOX.2 * scale).unwrap();
    let placement = Mat4::translation(OFFSET.0 * scale, OFFSET.1 * scale, OFFSET.2 * scale)
        * Mat4::rotation_z(FRAC_PI_4);
    transform_solid(&mut topo, tool, &placement).unwrap();
    let outcome = boolean_with_context(
        &mut topo,
        BooleanOp::Cut,
        stock,
        tool,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap();
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "ExactOnly disclosed {:?}",
        outcome.quality
    );
    (topo, outcome.solid)
}

fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let ops = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        ops.is_valid(),
        "{what}: operations strict validator: {:?}",
        ops.issues
            .iter()
            .map(|i| &i.description)
            .collect::<Vec<_>>()
    );
    let report =
        remus_check::validate::validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == Severity::Error)
        .map(|i| format!("{:?}: {}", i.check, i.description))
        .collect();
    assert!(errors.is_empty(), "{what}: check-crate errors {errors:?}");
}

/// Every edge is used exactly twice, in opposite effective senses — the
/// direct statement of a consistently oriented closed shell.
fn assert_edge_uses(topo: &Topology, solid: SolidId, what: &str) {
    let mut uses: BTreeMap<usize, Vec<bool>> = BTreeMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        let wires = std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied());
        for wid in wires {
            for oe in topo.wire(wid).unwrap().edges() {
                uses.entry(oe.edge().index())
                    .or_default()
                    .push(oe.is_forward() != face.is_reversed());
            }
        }
    }
    for (edge, senses) in &uses {
        assert!(
            senses.len() == 2 && senses[0] != senses[1],
            "{what}: edge {edge} has uses {senses:?}"
        );
    }
}

fn assert_mesh_closed(topo: &Topology, solid: SolidId, what: &str) {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    let deflection = ((aabb.max - aabb.min).length() * 1e-4).max(1e-7);
    let mesh = tessellate_solid(topo, solid, deflection).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{what}: open mesh");
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{what}: non-manifold mesh"
    );
}

fn assert_classified(topo: &Topology, solid: SolidId, scale: f64, what: &str) {
    // A point of the notch region: inside the disc and the footprint
    // (the footprint's inner corner is at ≈ (1.19, 0.81), radius ≈ 1.44).
    let (nx, ny) = (1.6, 0.6);
    let probes = [
        // Removed: inside the notch, above its floor.
        (Point3::new(nx, ny, 1.9), false),
        // Kept: directly under the notch floor.
        (Point3::new(nx, ny, 1.6), true),
        // Kept: full height away from the notch.
        (Point3::new(-1.0, 0.0, 1.9), true),
        (Point3::new(0.0, -1.0, 1.9), true),
        // Outside the stock altogether.
        (Point3::new(0.0, 0.0, 2.1), false),
    ];
    for (p, inside) in probes {
        let p = Point3::new(p.x() * scale, p.y() * scale, p.z() * scale);
        let got = classify_point(topo, solid, p, &ClassifyOptions::default()).unwrap();
        let want = if inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(got, want, "{what}: probe {p:?}");
    }
}

fn assert_volume(topo: &Topology, solid: SolidId, scale: f64, what: &str) {
    let expected = closed_form_volume() * scale.powi(3);
    let aabb = solid_bounding_box(topo, solid).unwrap();
    let fine = ((aabb.max - aabb.min).length() * 1e-5).max(1e-7);
    for d in [fine, 0.5 * fine] {
        let v = solid_volume(topo, solid, d).unwrap();
        let rel = ((v - expected) / expected).abs();
        assert!(
            rel < 1e-6,
            "{what}: solid_volume({d:e}) = {v} vs closed form {expected} (rel {rel:e})"
        );
    }
}

fn check_scale(scale: f64) {
    let what = format!("cylinder − oblique box notch at scale {scale}");
    let (topo, solid) = build_cut(scale);
    assert_valid(&topo, solid, &what);
    assert_edge_uses(&topo, solid, &what);
    assert_mesh_closed(&topo, solid, &what);
    assert_classified(&topo, solid, scale, &what);
    assert_volume(&topo, solid, scale, &what);
}

/// The generated failing case: scale 1000.
#[test]
fn cylinder_oblique_box_notch_cut_scale_1000() {
    check_scale(1000.0);
}

/// The same placement at unit scale, so a scale-dependent fix cannot pass
/// one band while breaking the other.
#[test]
fn cylinder_oblique_box_notch_cut_unit_scale() {
    check_scale(1.0);
}

/// And at the generator's small band.
#[test]
fn cylinder_oblique_box_notch_cut_scale_1e_minus_3() {
    check_scale(1e-3);
}

/// The closed-form oracle itself, pinned against an independent midpoint-rule
/// estimate of the disc ∩ footprint area (a 2000 × 2000 grid over the disc's
/// bounding square; the grid's error is far below the 1e-3 slack).
#[test]
fn closed_form_notch_area_matches_grid_estimate() {
    let f = footprint();
    let inside_footprint = |x: f64, y: f64| {
        (0..4).all(|i| {
            let (a, b) = (f[i], f[(i + 1) % 4]);
            (b.0 - a.0) * (y - a.1) - (b.1 - a.1) * (x - a.0) >= 0.0
        })
    };
    let n = 2000;
    let step = 2.0 * R / f64::from(n);
    let mut hits = 0u64;
    for i in 0..n {
        for j in 0..n {
            let x = -R + (f64::from(i) + 0.5) * step;
            let y = -R + (f64::from(j) + 0.5) * step;
            if x * x + y * y <= R * R && inside_footprint(x, y) {
                hits += 1;
            }
        }
    }
    let grid_area = hits as f64 * step * step;
    let exact_area = (std::f64::consts::PI * R * R * H - closed_form_volume()) / (H - OFFSET.2);
    assert!(
        ((grid_area - exact_area) / exact_area).abs() < 1e-3,
        "grid {grid_area} vs exact {exact_area}"
    );
}
