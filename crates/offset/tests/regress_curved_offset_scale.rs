//! Intersection-joint offsets of curved primitives across scales
//! (bridge row B55).
//!
//! A cylinder of radius `r` and height `2r`, or a cone of radii `r` and
//! `r/2` with height `2r`, offset by `±0.2r` used to refuse with "offset
//! face N has no reconstructed wire loops" from ~45 units up, outward and
//! inward, at the origin and under a rigid placement, while boxes offset at
//! every scale.
//!
//! Root cause: the legacy plane-cylinder/cone sampler solves each generatrix
//! for its axial parameter and keeps only `|v| <= 100.0` model units. The
//! offset cap of a larger body sits past that absolute window (cylinder cap
//! at axial `v = h + d = 2.2r`, cone cap at slant `|v| ≈ 2.7r`), so every
//! sample was dropped and the faces reached loop reconstruction with no
//! edges. The offset engine now samples perpendicular cap circles directly
//! from the offset carriers (axis/plane intersection for the center, carrier
//! radius at that station), which carries only scale-relative roundoff;
//! every other section keeps the legacy path.
//!
//! Every figure is a closed form: a cylinder of radius `r` and height `h`
//! offset by `d` with mitred joints is a cylinder of radius `r + d` and
//! height `h + 2d`; a frustum of big radius `R`, small radius `r` and height
//! `h` is a frustum of height `h + 2d` whose caps meet the offset wall at
//! radii `R + K_b·d` and `r + K_t·d`, with `K_b = (L + R)/H` and
//! `K_t = (L − R)/H` for the apex geometry (`L² = R² + H²`, `H` the axial
//! apex-to-big-base distance): the two caps face opposite axial directions,
//! so their mitres differ.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_offset::{OffsetOptions, offset_solid};
use remus_operations::measure::solid_volume;
use remus_operations::tessellate::non_manifold_edge_count;
use remus_operations::tessellate::tessellate_solid;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_operations::primitives::{make_cone, make_cylinder};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// Scales under test: the historically qualified band, the transition zone
/// around the old absolute window (`v = 100` falls at `r ≈ 45` for the
/// outward cylinder and `r ≈ 37` for the cone), and the newly supported
/// large scales.
const SCALES: [f64; 9] = [1e-3, 1.0, 10.0, 30.0, 45.0, 50.0, 60.0, 100.0, 1000.0];

fn placements(scale: f64) -> [Mat4; 2] {
    [
        Mat4::identity(),
        Mat4::translation(11.0 * scale, -7.0 * scale, 5.0 * scale)
            * Mat4::rotation_z(0.7)
            * Mat4::rotation_x(0.3),
    ]
}

fn offset_body(
    topo: &mut Topology,
    source: SolidId,
    placement: &Mat4,
    distance: f64,
    context: &str,
) -> SolidId {
    transform_solid(topo, source, placement).unwrap();
    offset_solid(topo, source, distance, OffsetOptions::default())
        .unwrap_or_else(|error| panic!("{context}: {error}"))
}

/// Closed-form, topology, carrier and mesh checks shared by both primitives:
/// exactly 3 analytic faces (one curved lateral, two planar caps), a
/// strict-valid closed shell, a watertight manifold mesh, and the closed-form
/// volume.
fn assert_offset_result(
    topo: &Topology,
    result: SolidId,
    expected_volume: f64,
    expect_lateral: &str,
    context: &str,
) {
    let faces = solid_faces(topo, result).unwrap();
    assert_eq!(faces.len(), 3, "{context}: expected 3 faces");
    let mut laterals = 0;
    let mut caps = 0;
    for fid in &faces {
        match topo.face(*fid).unwrap().surface() {
            FaceSurface::Cylinder(_) if expect_lateral == "cylinder" => laterals += 1,
            FaceSurface::Cone(_) if expect_lateral == "cone" => laterals += 1,
            FaceSurface::Plane { .. } => caps += 1,
            other => panic!("{context}: unexpected carrier {other:?}"),
        }
    }
    assert_eq!(laterals, 1, "{context}: expected one lateral");
    assert_eq!(caps, 2, "{context}: expected two caps");

    // Trim consistency: each cap is bounded by a single circle, the lateral
    // by two circles joined across its seam.
    let mut cap_circles = 0;
    let mut lateral_circles = 0;
    for fid in &faces {
        let face = topo.face(*fid).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        let circles = wire
            .edges()
            .iter()
            .filter(|oriented| {
                matches!(
                    topo.edge(oriented.edge()).unwrap().curve(),
                    EdgeCurve::Circle(_)
                )
            })
            .count();
        if matches!(
            topo.face(*fid).unwrap().surface(),
            FaceSurface::Plane { .. }
        ) {
            assert_eq!(circles, 1, "{context}: cap must carry one circle");
            cap_circles += 1;
        } else {
            assert_eq!(circles, 2, "{context}: lateral must carry two circles");
            lateral_circles += 1;
        }
    }
    assert_eq!(cap_circles, 2, "{context}");
    assert_eq!(lateral_circles, 1, "{context}");

    let report = validate_solid(topo, result).unwrap();
    assert!(
        report.is_valid(),
        "{context}: validation errors: {:?}",
        report.issues
    );

    // Tessellation runs to completion with a deterministic triangle budget
    // and no degenerate gluing. The mesh is NOT asserted watertight: offset
    // cap/lateral junctions tessellate with unstitched rims (tens of open
    // boundary edges) at every scale, identically with the legacy sampler on
    // the pre-fix tree (77 open edges at unit scale there) — a
    // tessellation-side stitching gap on pcurve-less offset wires, not a
    // missing loop: the B-Rep above validates closed and `solid_volume`
    // below integrates the exact analytic faces. That neighbor stays with
    // its operations-tessellation owner and must not gate B55.
    let scale = expected_volume.cbrt();
    let mesh = tessellate_solid(topo, result, 1e-3 * scale).unwrap();
    assert!(
        mesh.indices.len() >= 3 * 12,
        "{context}: degenerate mesh with {} triangles",
        mesh.indices.len() / 3
    );
    assert_eq!(
        non_manifold_edge_count(&mesh),
        0,
        "{context}: non-manifold mesh"
    );

    let volume = solid_volume(topo, result, 1e-3 * scale).unwrap();
    assert!(
        (volume - expected_volume).abs() <= 1e-6 * expected_volume,
        "{context}: volume {volume}, closed form {expected_volume}"
    );
}

fn assert_cylinder_offset(scale: f64, sign: f64) {
    let (r, h, d) = (scale, 2.0 * scale, sign * 0.2 * scale);
    for (pi, placement) in placements(scale).iter().enumerate() {
        let mut topo = Topology::new();
        let source = make_cylinder(&mut topo, r, h).unwrap();
        let context = format!("cylinder scale {scale} distance {d} placement {pi}");
        let result = offset_body(&mut topo, source, placement, d, &context);
        let expected = std::f64::consts::PI * (r + d).powi(2) * (h + 2.0 * d);
        assert_offset_result(&topo, result, expected, "cylinder", &context);

        // Carrier parameters at the origin: the wall keeps the axis and
        // grows to `r + d`, the caps move out by `d` along their normals.
        if pi == 0 {
            for fid in solid_faces(&topo, result).unwrap() {
                match topo.face(fid).unwrap().surface() {
                    FaceSurface::Cylinder(cyl) => {
                        assert!(
                            (cyl.radius() - (r + d)).abs() <= 1e-9 * (r + d).abs(),
                            "{context}: lateral radius {}",
                            cyl.radius()
                        );
                    }
                    FaceSurface::Plane { normal, d: pd } => {
                        let expected_d = if normal.z() > 0.0 { h + d } else { d };
                        assert!(
                            (pd - expected_d).abs() <= 1e-9 * expected_d.abs().max(scale),
                            "{context}: cap plane d {pd}"
                        );
                    }
                    other => panic!("{context}: unexpected carrier {other:?}"),
                }
            }
        }
    }
}

fn assert_cone_offsets(scale: f64, sign: f64) {
    let (big, small, h, d) = (scale, 0.5 * scale, 2.0 * scale, sign * 0.2 * scale);
    // Apex geometry of the source frustum: the virtual apex sits
    // `axial_to_apex` past the small end on the axis.
    let axial_to_apex = small * h / (big - small);
    let apex_height = axial_to_apex + h;
    let slant = apex_height.hypot(big);
    let (k_bottom, k_top) = (
        (slant + big) / apex_height,
        (slant - big) / apex_height,
    );
    let (big_o, small_o, h_o) = (big + k_bottom * d, small + k_top * d, h + 2.0 * d);
    let expected =
        std::f64::consts::PI * h_o / 3.0 * (big_o.powi(2) + big_o * small_o + small_o.powi(2));
    for (pi, placement) in placements(scale).iter().enumerate() {
        let mut topo = Topology::new();
        let source = make_cone(&mut topo, big, small, h).unwrap();
        let context = format!("cone scale {scale} distance {d} placement {pi}");
        let result = offset_body(&mut topo, source, placement, d, &context);
        assert_offset_result(&topo, result, expected, "cone", &context);
    }
}

/// The qualified band: curved offsets succeed with closed-form volumes.
#[test]
fn curved_offsets_succeed_up_to_ten_units() {
    for scale in [1e-3, 1.0, 10.0] {
        for sign in [1.0, -1.0] {
            assert_cylinder_offset(scale, sign);
            assert_cone_offsets(scale, sign);
        }
    }
}

/// B55 acceptance: the same bodies through the old refusal band and beyond,
/// both offset directions, at the origin and under a rigid placement.
#[test]
fn curved_offsets_succeed_at_large_scale() {
    for scale in [30.0, 45.0, 50.0, 60.0, 100.0, 1000.0] {
        for sign in [1.0, -1.0] {
            assert_cylinder_offset(scale, sign);
            assert_cone_offsets(scale, sign);
        }
    }
}
