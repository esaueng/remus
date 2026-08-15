//! Research probe for the direct blend-editing design
//! (`docs/design/direct-blend-editing-research.md`). Validates or refutes
//! the design's load-bearing assumptions against the live kernel:
//!
//! E1: plane×plane fillet band is an exact cylinder; spring edges are
//!     lines, cross edges are arcs.
//! E2: trihedral fillet corner patch surface type (sphere vs NURBS).
//! E3: plane×cylinder (boss base) fillet band is an exact torus with
//!     exact major/minor radii; closed ring (no cross edges).
//! E4: STEP round-trip preserves those analytic surfaces and radii.
//! E5: the band's measured radius + support geometry re-predicts the band
//!     surface (the recognition "inverse analytic" check), via the actual
//!     `try_analytic_fillet` creation path.
//! E6: exact `defeature` removal restores the sharp edge and original volume.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stdout
)]

use brepkit_blend::query::{GeometricSpine, try_analytic_fillet_surface};
use brepkit_math::mat::Mat4;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_operations::blend_ops::fillet_v2;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::defeature::defeature;
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::edge::{EdgeCurve, EdgeId};
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;

fn edge_curve_name(curve: &EdgeCurve) -> &'static str {
    match curve {
        EdgeCurve::Line => "Line",
        EdgeCurve::Circle(_) => "Circle",
        EdgeCurve::Ellipse(_) => "Ellipse",
        EdgeCurve::Hyperbola(_) => "Hyperbola",
        EdgeCurve::Parabola(_) => "Parabola",
        EdgeCurve::NurbsCurve(_) => "Nurbs",
    }
}

fn surface_name(s: &FaceSurface) -> &'static str {
    match s {
        FaceSurface::Plane { .. } => "Plane",
        FaceSurface::Cylinder(_) => "Cylinder",
        FaceSurface::Cone(_) => "Cone",
        FaceSurface::Sphere(_) => "Sphere",
        FaceSurface::Torus(_) => "Torus",
        FaceSurface::Nurbs(_) => "Nurbs",
    }
}

/// Every edge of the solid, deduplicated.
fn all_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let mut seen = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                if seen.insert(oe.edge().index()) {
                    out.push(oe.edge());
                }
            }
        }
    }
    out
}

fn vertex_point(topo: &Topology, e: EdgeId, which: bool) -> Point3 {
    let edge = topo.edge(e).unwrap();
    topo.vertex(if which { edge.start() } else { edge.end() })
        .unwrap()
        .point()
}

/// The vertical edge of a box at (x, y) corner.
fn find_vertical_edge(topo: &Topology, solid: SolidId, x: f64, y: f64) -> EdgeId {
    all_edges(topo, solid)
        .into_iter()
        .find(|&e| {
            let a = vertex_point(topo, e, true);
            let b = vertex_point(topo, e, false);
            (a.x() - x).abs() < 1e-9
                && (a.y() - y).abs() < 1e-9
                && (b.x() - x).abs() < 1e-9
                && (b.y() - y).abs() < 1e-9
                && (a.z() - b.z()).abs() > 1.0
        })
        .expect("vertical edge")
}

fn dump_face(topo: &Topology, fid: FaceId, label: &str) {
    let face = topo.face(fid).unwrap();
    let surface = face.surface();
    let detail = match surface {
        FaceSurface::Cylinder(c) => format!("r={:.12}", c.radius()),
        FaceSurface::Torus(t) => format!("R={:.12} r={:.12}", t.major_radius(), t.minor_radius()),
        FaceSurface::Sphere(s) => format!("r={:.12}", s.radius()),
        _ => String::new(),
    };
    println!(
        "  {label}: face {} {} {detail} reversed={}",
        fid.index(),
        surface_name(surface),
        face.is_reversed()
    );
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oe in topo.wire(wid).unwrap().edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            println!(
                "    edge {} {} len~{:.6}",
                oe.edge().index(),
                edge_curve_name(edge.curve()),
                (vertex_point(topo, oe.edge(), false) - vertex_point(topo, oe.edge(), true))
                    .length()
            );
        }
    }
}

fn census_print(topo: &Topology, solid: SolidId, label: &str) {
    let faces = solid_faces(topo, solid).unwrap();
    println!("{label}: {} faces", faces.len());
    for fid in faces {
        dump_face(topo, fid, "face");
    }
}

// E1 + E6: single-edge fillet on a box.
#[test]
fn e1_plane_plane_fillet_structure() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let edge = find_vertical_edge(&topo, solid, 40.0, 40.0);
    let result = fillet_v2(&mut topo, solid, &[edge], 3.0).unwrap();
    census_print(&topo, result.solid, "E1 filleted box");

    // Find the band: the cylindrical face.
    let band = solid_faces(&topo, result.solid)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
        .expect("cylinder band");
    let FaceSurface::Cylinder(cyl) = topo.face(band).unwrap().surface() else {
        unreachable!();
    };
    assert!((cyl.radius() - 3.0).abs() < 1e-12, "exact fillet radius");

    // E6: exact tier-2 removal. The two circular wound arcs collapse to the
    // recovered sharp corners instead of being silently chorded.
    let healed = defeature(&mut topo, result.solid, &[band]).unwrap();
    let healed_volume = brepkit_operations::measure::solid_volume(&topo, healed, 0.02).unwrap();
    println!("E6 unfillet volume={healed_volume:.9}");
    assert!((healed_volume - 16_000.0).abs() < 1e-6);
    assert_eq!(solid_faces(&topo, healed).unwrap().len(), 6);
}

// E2: three-edge corner.
#[test]
fn e2_trihedral_corner_structure() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    // Three edges meeting at (40,40,10): vertical edge, and the two top edges.
    let vertical = find_vertical_edge(&topo, solid, 40.0, 40.0);
    let top_edges: Vec<EdgeId> = all_edges(&topo, solid)
        .into_iter()
        .filter(|&e| {
            let a = vertex_point(&topo, e, true);
            let b = vertex_point(&topo, e, false);
            let at_corner = |p: Point3| {
                ((p.x() - 40.0).abs() < 1e-9 && (p.y() - 40.0).abs() < 1e-9)
                    && (p.z() - 10.0).abs() < 1e-9
            };
            (at_corner(a) || at_corner(b)) && (a.z() - b.z()).abs() < 1e-9
        })
        .collect();
    assert_eq!(top_edges.len(), 2);
    let edges: Vec<EdgeId> = std::iter::once(vertical).chain(top_edges).collect();
    let result = fillet_v2(&mut topo, solid, &edges, 3.0).unwrap();
    census_print(&topo, result.solid, "E2 trihedral fillet");
    let faces = solid_faces(&topo, result.solid).unwrap();
    let bands = faces
        .iter()
        .filter(|&&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder) if (cylinder.radius() - 3.0).abs() < 1e-12
            )
        })
        .count();
    let corners: Vec<FaceId> = faces
        .into_iter()
        .filter(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Sphere(sphere) if (sphere.radius() - 3.0).abs() < 1e-12
            )
        })
        .collect();
    assert_eq!(bands, 3, "one exact cylindrical band per selected edge");
    assert_eq!(corners.len(), 1, "one exact spherical corner patch");
    let corner = topo.face(corners[0]).unwrap();
    let corner_edges = topo.wire(corner.outer_wire()).unwrap().edges();
    assert_eq!(corner_edges.len(), 3);
    assert!(corner_edges.iter().all(|edge| matches!(
        topo.edge(edge.edge()).unwrap().curve(),
        EdgeCurve::Circle(_)
    )));
}

// E3: hole-rim fillet (plane×cylinder, convex). The post-base concave
// counterpart is E9 and now passes the same creation gate.
fn drilled_plate(topo: &mut Topology) -> (SolidId, EdgeId) {
    let plate = make_box(topo, 80.0, 40.0, 8.0).unwrap();
    let drill = make_cylinder(topo, 10.0, 16.0).unwrap();
    transform_solid(topo, drill, &Mat4::translation(40.0, 20.0, -4.0)).unwrap();
    let cut = boolean(topo, BooleanOp::Cut, plate, drill).unwrap();
    // The top rim of the bore: circle edge at z = 8.
    let rim = all_edges(topo, cut)
        .into_iter()
        .find(|&e| {
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_))
                && (vertex_point(topo, e, true).z() - 8.0).abs() < 1e-9
        })
        .expect("bore top rim");
    (cut, rim)
}

#[test]
fn e3_boss_base_fillet_structure() {
    let mut topo = Topology::new();
    let (fused, ridgeline) = drilled_plate(&mut topo);
    let vol = brepkit_operations::measure::solid_volume(&topo, fused, 0.05).unwrap();
    println!("E3 drilled volume = {vol:.3} (expect 25600-pi*100*8=23086.7)");
    let result = fillet_v2(&mut topo, fused, &[ridgeline], 3.0).unwrap();
    census_print(&topo, result.solid, "E3 boss base fillet");

    let band = solid_faces(&topo, result.solid)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .expect("torus band");
    let FaceSurface::Torus(torus) = topo.face(band).unwrap().surface() else {
        unreachable!();
    };
    println!(
        "E3 torus: center=({:.6},{:.6},{:.6}) R={:.12} r={:.12}",
        torus.center().x(),
        torus.center().y(),
        torus.center().z(),
        torus.major_radius(),
        torus.minor_radius()
    );
    assert!(
        (torus.minor_radius() - 3.0).abs() < 1e-9,
        "minor = fillet radius"
    );
}

// E4: STEP round-trip of E1 and E3 bodies.
#[test]
fn e4_step_roundtrip_preserves_analytics() {
    let mut topo = Topology::new();
    let (fused, ridgeline) = drilled_plate(&mut topo);
    let filleted = fillet_v2(&mut topo, fused, &[ridgeline], 3.0)
        .unwrap()
        .solid;

    let step = brepkit_io::step::writer::write_step(&topo, &[filleted]).unwrap();
    let mut topo2 = Topology::new();
    let reread = brepkit_io::step::reader::read_step(&step, &mut topo2).unwrap();
    assert_eq!(reread.len(), 1);
    census_print(&topo2, reread[0], "E4 re-read body");

    let torus = solid_faces(&topo2, reread[0])
        .unwrap()
        .into_iter()
        .find_map(|f| match topo2.face(f).unwrap().surface() {
            FaceSurface::Torus(t) => Some(t.clone()),
            _ => None,
        })
        .expect("torus survives STEP round-trip");
    println!(
        "E4 re-read torus: R={:.12} r={:.12}",
        torus.major_radius(),
        torus.minor_radius()
    );
    assert!((torus.minor_radius() - 3.0).abs() < 1e-9);
    assert!((torus.major_radius() - 13.0).abs() < 1e-9);
}

// E5: inverse-analytic recognition check on the E3 band.
#[test]
fn e5_inverse_analytic_check() {
    let mut topo = Topology::new();
    let (fused, ridgeline) = drilled_plate(&mut topo);
    let result = fillet_v2(&mut topo, fused, &[ridgeline], 3.0).unwrap();
    let solid = result.solid;

    // Band = torus face; supports = its two tangent neighbors.
    let band = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();

    // Map edges -> faces to find the two faces adjacent across the band's
    // boundary (the spring edges).
    let band_face = topo.face(band).unwrap();
    let mut supports: Vec<FaceId> = Vec::new();
    for oe in topo.wire(band_face.outer_wire()).unwrap().edges() {
        for fid in solid_faces(&topo, solid).unwrap() {
            if fid == band {
                continue;
            }
            let face = topo.face(fid).unwrap();
            let uses = std::iter::once(face.outer_wire())
                .chain(face.inner_wires().iter().copied())
                .any(|wid| {
                    topo.wire(wid)
                        .unwrap()
                        .edges()
                        .iter()
                        .any(|o| o.edge() == oe.edge())
                });
            if uses && !supports.contains(&fid) {
                supports.push(fid);
            }
        }
    }
    assert_eq!(supports.len(), 2, "band has exactly two supports");
    let s0 = topo.face(supports[0]).unwrap().surface().clone();
    let s1 = topo.face(supports[1]).unwrap().surface().clone();
    println!(
        "E5 supports: {} and {}",
        surface_name(&s0),
        surface_name(&s1)
    );

    // The design's check: does the creation path, given the supports and the
    // measured radius, re-derive this band? The spine of the underlying
    // sharp edge is the ridgeline circle — which no longer exists as an
    // edge. Rebuild it geometrically: circle at the plate top (z=8),
    // radius 10 around the boss axis. For this probe, recover the spine
    // from the torus: center circle at torus.center(), radius = major.
    let torus = match topo.face(band).unwrap().surface() {
        FaceSurface::Torus(t) => t.clone(),
        _ => unreachable!(),
    };

    // The ridgeline circle is exactly recoverable from the supports' geometry
    // (boss cylinder intersected with the plate top plane) and matches the
    // torus center circle up to the radius shift.
    let boss = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find_map(|f| match topo.face(f).unwrap().surface() {
            FaceSurface::Cylinder(c) if topo.face(f).unwrap().is_reversed() => Some(c.clone()),
            _ => None,
        })
        .expect("bore wall cylinder");
    println!(
        "E5 boss axis=({:.3},{:.3},{:.3}) origin=({:.3},{:.3},{:.3}) r={:.6}",
        boss.axis().x(),
        boss.axis().y(),
        boss.axis().z(),
        boss.origin().x(),
        boss.origin().y(),
        boss.origin().z(),
        boss.radius()
    );
    // Concave fillet at the top of a bore: the rolling-ball center circle
    // lies OUTSIDE the wall — major = bore radius + fillet radius = 13, at
    // height plate_top − r = 5 (verified against E3's measured torus).
    assert!((torus.major_radius() - (boss.radius() + 3.0)).abs() < 1e-9);
    assert!((torus.center().z() - (8.0 - 3.0)).abs() < 1e-9);
    // Torus center lies on the bore axis.
    let radial_offset = {
        let d = torus.center() - boss.origin();
        d - boss.axis() * d.dot(boss.axis())
    };
    assert!(radial_offset.length() < 1e-9, "torus center on bore axis");

    // And the full creation-path reuse: query with the recovered ridgeline.
    // (Ridgeline: boss circle at z = plate top = 8.)
    let plate_top_z = 8.0_f64;
    let circle = brepkit_math::curves::Circle3D::new(
        Point3::new(boss.origin().x(), boss.origin().y(), plate_top_z),
        Vec3::new(0.0, 0.0, 1.0),
        boss.radius(),
    )
    .unwrap();
    let spine = GeometricSpine::Circle(circle);
    let predicted =
        try_analytic_fillet_surface(&topo, supports[1], supports[0], &spine, 3.0).unwrap();
    let Some(FaceSurface::Torus(predicted)) = predicted else {
        panic!("creation path did not reconstruct a torus")
    };
    println!(
        "E5 predicted torus: center=({:.6},{:.6},{:.6}) R={:.12} r={:.12}",
        predicted.center().x(),
        predicted.center().y(),
        predicted.center().z(),
        predicted.major_radius(),
        predicted.minor_radius()
    );
    assert!((predicted.minor_radius() - torus.minor_radius()).abs() < 1e-9);
    assert!((predicted.major_radius() - torus.major_radius()).abs() < 1e-9);
    let dc = predicted.center() - torus.center();
    assert!(dc.length() < 1e-9, "predicted torus frame matches");
    assert!(
        predicted.z_axis().dot(torus.z_axis()).abs() > 1.0 - 1e-12,
        "predicted torus symmetry axis matches"
    );
    assert!(
        predicted.x_axis().dot(torus.x_axis()) > 1.0 - 1e-12,
        "predicted torus reference direction matches"
    );
}

// E7: prototype tier-1 in-place resize surgery on the closed-ring torus
// band (r=3 -> r=5): replace the band surface, retarget the two spring
// circles, move the seam-vertex heights; no wire changes needed for a
// closed ring. Validates with validate_solid + volume.
#[test]
fn e7_in_place_resize_ring_band() {
    let mut topo = Topology::new();
    let (cut, rim) = drilled_plate(&mut topo);
    let solid = fillet_v2(&mut topo, cut, &[rim], 3.0).unwrap().solid;

    let band = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    let torus = match topo.face(band).unwrap().surface() {
        FaceSurface::Torus(t) => t.clone(),
        _ => unreachable!(),
    };

    // Collect the band's spring edges (the two circles bounding the band).
    let band_wires: Vec<_> = std::iter::once(topo.face(band).unwrap().outer_wire())
        .chain(topo.face(band).unwrap().inner_wires().iter().copied())
        .collect();
    let mut springs: Vec<EdgeId> = Vec::new();
    for wid in band_wires {
        for oe in topo.wire(wid).unwrap().edges() {
            if !springs.contains(&oe.edge()) {
                springs.push(oe.edge());
            }
        }
    }
    println!("E7 band has {} spring edges", springs.len());
    for &e in &springs {
        let EdgeCurve::Circle(c) = topo.edge(e).unwrap().curve() else {
            panic!("spring edge not a circle");
        };
        println!(
            "  spring edge {} circle center=({:.3},{:.3},{:.3}) r={:.6}",
            e.index(),
            c.center().x(),
            c.center().y(),
            c.center().z(),
            c.radius()
        );
    }

    // Classify the band's boundary edges: the two SPRING circles are
    // coaxial with the torus axis; anything else is a seam CROSS edge.
    let axis = torus.z_axis();
    let mut spring_ids = Vec::new();
    let mut cross_ids = Vec::new();
    for &e in &springs {
        let EdgeCurve::Circle(c) = topo.edge(e).unwrap().curve() else {
            unreachable!();
        };
        let d = c.center() - torus.center();
        let radial = d - axis * d.dot(axis);
        if radial.length() < 1e-9 {
            spring_ids.push(e);
        } else {
            cross_ids.push(e);
        }
    }
    println!("E7 springs={} cross={}", spring_ids.len(), cross_ids.len());
    assert_eq!(spring_ids.len(), 2);

    let r_new = 5.0_f64;
    let r_c = 10.0_f64; // bore radius
    let z_top = 8.0_f64;
    let center_xy = torus.center();

    // New torus: center circle radius r_c + r_new at height z_top - r_new.
    let new_torus = brepkit_math::surfaces::ToroidalSurface::with_axis_and_ref_dir(
        Point3::new(center_xy.x(), center_xy.y(), z_top - r_new),
        r_c + r_new,
        r_new,
        axis,
        torus.x_axis(),
    )
    .unwrap();
    topo.face_mut(band)
        .unwrap()
        .set_surface(FaceSurface::Torus(new_torus));

    // Retarget spring circles: the one on the plate top (z = z_top,
    // radius r_c + r_new) and the one on the bore wall (radius r_c at
    // height z_top - r_new).
    for &e in &spring_ids {
        let EdgeCurve::Circle(c) = topo.edge(e).unwrap().curve() else {
            unreachable!();
        };
        let on_plate = (c.center().z() - z_top).abs() < 1e-9;
        let (center, radius) = if on_plate {
            (
                Point3::new(c.center().x(), c.center().y(), z_top),
                r_c + r_new,
            )
        } else {
            (
                Point3::new(c.center().x(), c.center().y(), z_top - r_new),
                r_c,
            )
        };
        let new_circle =
            brepkit_math::curves::Circle3D::new_with_ref(center, c.normal(), radius, c.u_axis())
                .unwrap();
        topo.edge_mut(e)
            .unwrap()
            .set_curve(EdgeCurve::Circle(new_circle));
    }

    // Rebuild the seam cross edges: circles of radius r_new in the seam
    // plane, centered on the new center circle at the seam angle.
    for &e in &cross_ids {
        let EdgeCurve::Circle(c) = topo.edge(e).unwrap().curve() else {
            unreachable!();
        };
        let old_center = c.center();
        let d = old_center - torus.center();
        let radial_dir = (d - axis * d.dot(axis)).normalize().unwrap();
        let new_center =
            Point3::new(center_xy.x(), center_xy.y(), z_top - r_new) + radial_dir * (r_c + r_new);
        let new_circle =
            brepkit_math::curves::Circle3D::new_with_ref(new_center, c.normal(), r_new, c.u_axis())
                .unwrap();
        topo.edge_mut(e)
            .unwrap()
            .set_curve(EdgeCurve::Circle(new_circle));
    }

    // Move every vertex that sat on the old cylinder-side spring
    // (z = z_top - 3) to the new height (z = z_top - r_new).
    let solid_data_faces = solid_faces(&topo, solid).unwrap();
    let mut targets = Vec::new();
    for fid in &solid_data_faces {
        let (outer, inners): (
            brepkit_topology::wire::WireId,
            Vec<brepkit_topology::wire::WireId>,
        ) = {
            let face = topo.face(*fid).unwrap();
            (face.outer_wire(), face.inner_wires().to_vec())
        };
        for wid in std::iter::once(outer).chain(inners) {
            for oe in topo.wire(wid).unwrap().edges() {
                let (start, end) = {
                    let edge = topo.edge(oe.edge()).unwrap();
                    (edge.start(), edge.end())
                };
                for vid in [start, end] {
                    targets.push(vid);
                }
            }
        }
    }
    let mut moved = 0;
    for vid in targets {
        let p = topo.vertex(vid).unwrap().point();
        if (p.z() - (z_top - 3.0)).abs() < 1e-9 {
            let q = Point3::new(p.x(), p.y(), z_top - r_new);
            topo.vertex_mut(vid).unwrap().set_point(q);
            moved += 1;
        } else if (p.z() - z_top).abs() < 1e-9 {
            // Seam vertex on the plate-side spring: radius 13 -> 15.
            let d = p - Point3::new(center_xy.x(), center_xy.y(), z_top);
            let radial_dir = (d - axis * d.dot(axis)).normalize().unwrap();
            if (radial_dir.length() * 0.0 + (d.length() - 13.0)).abs() < 1e-9 {
                let q =
                    Point3::new(center_xy.x(), center_xy.y(), z_top) + radial_dir * (r_c + r_new);
                topo.vertex_mut(vid).unwrap().set_point(q);
                moved += 1;
            }
        }
    }
    println!("E7 moved {moved} vertex records");

    let report = brepkit_operations::validate::validate_solid(&topo, solid).unwrap();
    println!("E7 validate_solid valid = {}", report.is_valid());
    for issue in &report.issues {
        println!("  issue: {:?} {}", issue.severity, issue.description);
    }
    let vol = brepkit_operations::measure::solid_volume(&topo, solid, 0.05).unwrap();
    // r=3 band: 23086.726 - removed ring; r=5 removes more. Print both.
    println!("E7 volume after resize to r=5: {vol:.3}");
    // Closed-form expectation via Pappus: removed ring cross-section is
    // square [r_c, r_c+r] x [z_top-r, z_top] minus the quarter tube disk.
    let removed = |r: f64| {
        let square = r * r * (r_c + r / 2.0);
        let disk =
            std::f64::consts::PI * r * r / 4.0 * (r_c + r - 4.0 * r / (3.0 * std::f64::consts::PI));
        2.0 * std::f64::consts::PI * (square - disk)
    };
    let expected = 23086.726 - removed(5.0);
    println!("E7 expected volume = {expected:.3}");
    assert!(
        (vol - expected).abs() < 1.0,
        "volume {vol} should match Pappus expectation {expected}"
    );
    assert!(report.is_valid(), "surgery result must validate");
}

// E8: strongest oracle — the E7 resized body must be surface-identical to
// a FRESH r=5 fillet built from the same drilled plate.
#[test]
fn e8_resized_matches_fresh_fillet() {
    let mut topo = Topology::new();
    let (cut, rim) = drilled_plate(&mut topo);
    let resized = fillet_v2(&mut topo, cut, &[rim], 3.0).unwrap().solid;
    let fresh = fillet_v2(&mut topo, cut, &[rim], 5.0).unwrap().solid;

    // Resize in place: 3 -> 5 (same surgery as E7, condensed).
    let band = solid_faces(&topo, resized)
        .unwrap()
        .into_iter()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    let torus = match topo.face(band).unwrap().surface() {
        FaceSurface::Torus(t) => t.clone(),
        _ => unreachable!(),
    };
    let axis = torus.z_axis();
    let (r_c, r_new, z_top) = (10.0_f64, 5.0_f64, 8.0_f64);
    let cxy = torus.center();

    let band_edges: Vec<EdgeId> = topo
        .wire(topo.face(band).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(brepkit_topology::wire::OrientedEdge::edge)
        .collect();
    let mut springs = Vec::new();
    let mut cross = Vec::new();
    for e in band_edges {
        let EdgeCurve::Circle(c) = topo.edge(e).unwrap().curve() else {
            panic!("non-circle band edge")
        };
        let d = c.center() - torus.center();
        let radial = d - axis * d.dot(axis);
        if radial.length() < 1e-9 {
            springs.push((e, c.clone()));
        } else {
            cross.push((e, c.clone()));
        }
    }
    topo.face_mut(band).unwrap().set_surface(FaceSurface::Torus(
        brepkit_math::surfaces::ToroidalSurface::with_axis_and_ref_dir(
            Point3::new(cxy.x(), cxy.y(), z_top - r_new),
            r_c + r_new,
            r_new,
            axis,
            torus.x_axis(),
        )
        .unwrap(),
    ));
    for (e, c) in springs {
        let on_plate = (c.center().z() - z_top).abs() < 1e-9;
        let (center, radius) = if on_plate {
            (
                Point3::new(c.center().x(), c.center().y(), z_top),
                r_c + r_new,
            )
        } else {
            (
                Point3::new(c.center().x(), c.center().y(), z_top - r_new),
                r_c,
            )
        };
        topo.edge_mut(e).unwrap().set_curve(EdgeCurve::Circle(
            brepkit_math::curves::Circle3D::new_with_ref(center, c.normal(), radius, c.u_axis())
                .unwrap(),
        ));
    }
    for (e, c) in cross {
        let d = c.center() - torus.center();
        let radial_dir = (d - axis * d.dot(axis)).normalize().unwrap();
        let new_center = Point3::new(cxy.x(), cxy.y(), z_top - r_new) + radial_dir * (r_c + r_new);
        topo.edge_mut(e).unwrap().set_curve(EdgeCurve::Circle(
            brepkit_math::curves::Circle3D::new_with_ref(new_center, c.normal(), r_new, c.u_axis())
                .unwrap(),
        ));
    }
    // Move vertices: seam vertex on the wall spring (z=5->3) and on the
    // plate spring (r=13->15).
    let mut vids = Vec::new();
    for fid in solid_faces(&topo, resized).unwrap() {
        let (outer, inners) = {
            let face = topo.face(fid).unwrap();
            (face.outer_wire(), face.inner_wires().to_vec())
        };
        for wid in std::iter::once(outer).chain(inners) {
            for oe in topo.wire(wid).unwrap().edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                vids.push(edge.start());
                vids.push(edge.end());
            }
        }
    }
    for vid in vids {
        let p = topo.vertex(vid).unwrap().point();
        if (p.z() - (z_top - 3.0)).abs() < 1e-9 {
            topo.vertex_mut(vid)
                .unwrap()
                .set_point(Point3::new(p.x(), p.y(), z_top - r_new));
        } else if (p.z() - z_top).abs() < 1e-9 {
            let d = p - Point3::new(cxy.x(), cxy.y(), z_top);
            if (d.length() - 13.0).abs() < 1e-9 {
                let dir = (d - axis * d.dot(axis)).normalize().unwrap();
                topo.vertex_mut(vid)
                    .unwrap()
                    .set_point(Point3::new(cxy.x(), cxy.y(), z_top) + dir * (r_c + r_new));
            }
        }
    }

    // Compare surface-by-surface with the fresh r=5 fillet.
    let surfaces_of = |topo: &Topology, s: SolidId| {
        let mut v: Vec<String> = solid_faces(topo, s)
            .unwrap()
            .iter()
            .map(|&f| {
                let surf = topo.face(f).unwrap().surface();
                match surf {
                    FaceSurface::Plane { normal, d } => format!(
                        "Plane({:.6},{:.6},{:.6},{:.6})",
                        normal.x(),
                        normal.y(),
                        normal.z(),
                        d
                    ),
                    FaceSurface::Cylinder(c) => format!("Cyl({:.6})", c.radius()),
                    FaceSurface::Torus(t) => format!(
                        "Torus(c=({:.6},{:.6},{:.6}),R={:.6},r={:.6})",
                        t.center().x(),
                        t.center().y(),
                        t.center().z(),
                        t.major_radius(),
                        t.minor_radius()
                    ),
                    other => surface_name(other).to_string(),
                }
            })
            .collect();
        v.sort();
        v
    };
    let a = surfaces_of(&topo, resized);
    let b = surfaces_of(&topo, fresh);
    println!("E8 resized: {a:#?}");
    println!("E8 fresh:   {b:#?}");
    assert_eq!(
        a, b,
        "resized body must be surface-identical to fresh fillet"
    );

    // Volume agreement.
    let va = brepkit_operations::measure::solid_volume(&topo, resized, 0.05).unwrap();
    let vb = brepkit_operations::measure::solid_volume(&topo, fresh, 0.05).unwrap();
    println!("E8 volumes: resized={va:.4} fresh={vb:.4}");
    assert!((va - vb).abs() < 0.5);
}
// E9: convex post-on-plate base fillet creation.
#[test]
fn e9_boss_base_characterization() {
    for r in [1.0, 2.0, 5.0] {
        let mut topo = Topology::new();
        let plate = make_box(&mut topo, 80.0, 40.0, 8.0).unwrap();
        let boss = make_cylinder(&mut topo, 10.0, 32.0).unwrap();
        transform_solid(&mut topo, boss, &Mat4::translation(40.0, 20.0, 8.0)).unwrap();
        let fused = boolean(&mut topo, BooleanOp::Fuse, plate, boss).unwrap();
        let rim = all_edges(&topo, fused)
            .into_iter()
            .find(|&e| {
                matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(_))
                    && (vertex_point(&topo, e, true).z() - 8.0).abs() < 1e-9
            })
            .unwrap();
        let result = fillet_v2(&mut topo, fused, &[rim], r).unwrap();
        assert!(result.failed.is_empty(), "{:?}", result.failed);
        let report = brepkit_operations::validate::validate_solid(&topo, result.solid).unwrap();
        assert!(report.is_valid(), "{report:?}");

        let bands: Vec<FaceId> = solid_faces(&topo, result.solid)
            .unwrap()
            .into_iter()
            .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Torus(_)))
            .collect();
        assert_eq!(bands.len(), 1);
        let band_face = topo.face(bands[0]).unwrap();
        let FaceSurface::Torus(band) = band_face.surface() else {
            unreachable!()
        };
        assert!(
            band_face.is_reversed(),
            "concave band must not be inside-out"
        );
        assert!((band.minor_radius() - r).abs() < 1e-9);
        assert!((band.major_radius() - (10.0 + r)).abs() < 1e-9);
        assert!((band.center().z() - (8.0 + r)).abs() < 1e-9);

        let area = r * r * (1.0 - std::f64::consts::PI / 4.0);
        let centroid = (r * r * (10.0 + r / 2.0)
            - std::f64::consts::PI * r * r / 4.0
                * (10.0 + r - 4.0 * r / (3.0 * std::f64::consts::PI)))
            / area;
        let expected_add = area * 2.0 * std::f64::consts::PI * centroid;
        let before = brepkit_operations::measure::solid_volume(&topo, fused, 0.05).unwrap();
        let after = brepkit_operations::measure::solid_volume(&topo, result.solid, 0.05).unwrap();
        println!(
            "E9 r={r}: added={:.6} expected={expected_add:.6}",
            after - before
        );
        assert!((after - before - expected_add).abs() < 0.5);
    }
}

// E10: prototype tier-1 in-place resize for an OPEN plane-plane band.
// The topology stays fixed: retarget the cylinder and its two circular end
// arcs, then slide the four shared spring endpoints along their support edges.
// The result must match a fresh r=5 fillet geometrically and volumetrically.
#[test]
fn e10_in_place_resize_open_plane_plane_band() {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, 40.0, 40.0, 10.0).unwrap();
    let sharp_edge = find_vertical_edge(&topo, sharp, 40.0, 40.0);
    let resized = fillet_v2(&mut topo, sharp, &[sharp_edge], 3.0)
        .unwrap()
        .solid;

    // A successful planar fillet can rewrite entities shared by its input, so
    // the fresh oracle must live in an independent topology.
    let mut fresh_topo = Topology::new();
    let fresh_sharp = make_box(&mut fresh_topo, 40.0, 40.0, 10.0).unwrap();
    let fresh_edge = find_vertical_edge(&fresh_topo, fresh_sharp, 40.0, 40.0);
    let fresh = fillet_v2(&mut fresh_topo, fresh_sharp, &[fresh_edge], 5.0)
        .unwrap()
        .solid;

    let band = solid_faces(&topo, resized)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder) if (cylinder.radius() - 3.0).abs() < 1e-9
            )
        })
        .unwrap();
    let cylinder = match topo.face(band).unwrap().surface() {
        FaceSurface::Cylinder(cylinder) => cylinder.clone(),
        _ => unreachable!(),
    };

    // The r=3 cylinder axis is (37,37); at r=5 it is (35,35).
    let axis_shift = Vec3::new(-2.0, -2.0, 0.0);
    let new_cylinder = brepkit_math::surfaces::CylindricalSurface::with_ref_dir(
        cylinder.origin() + axis_shift,
        cylinder.axis(),
        5.0,
        cylinder.x_axis(),
    )
    .unwrap();
    topo.face_mut(band)
        .unwrap()
        .set_surface(FaceSurface::Cylinder(new_cylinder));

    let band_edges: Vec<EdgeId> = topo
        .wire(topo.face(band).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(brepkit_topology::wire::OrientedEdge::edge)
        .collect();
    let cross_edges: Vec<(EdgeId, brepkit_math::curves::Circle3D)> = band_edges
        .iter()
        .filter_map(|&edge| match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle) => Some((edge, circle.clone())),
            _ => None,
        })
        .collect();
    assert_eq!(cross_edges.len(), 2);

    for (edge, circle) in cross_edges {
        let new_circle = brepkit_math::curves::Circle3D::new_with_ref(
            circle.center() + axis_shift,
            circle.normal(),
            5.0,
            circle.u_axis(),
        )
        .unwrap();
        topo.edge_mut(edge)
            .unwrap()
            .set_curve(EdgeCurve::Circle(new_circle));
    }

    // Every affected vertex is shared by one spring and one cross arc. Move
    // it once; the adjacent support-face line edges inherit the new endpoint.
    let mut affected = std::collections::BTreeMap::new();
    for edge_id in band_edges {
        let edge = topo.edge(edge_id).unwrap();
        for vertex in [edge.start(), edge.end()] {
            affected.insert(vertex.index(), vertex);
        }
    }
    assert_eq!(affected.len(), 4);
    for vertex in affected.values().copied() {
        let point = topo.vertex(vertex).unwrap().point();
        let moved = if (point.x() - 40.0).abs() < 1e-9 {
            Point3::new(point.x(), point.y() - 2.0, point.z())
        } else if (point.y() - 40.0).abs() < 1e-9 {
            Point3::new(point.x() - 2.0, point.y(), point.z())
        } else {
            panic!("unexpected spring endpoint {point:?}");
        };
        topo.vertex_mut(vertex).unwrap().set_point(moved);
    }

    let report = brepkit_operations::validate::validate_solid(&topo, resized).unwrap();
    for issue in &report.issues {
        println!("E10 issue: {:?} {}", issue.severity, issue.description);
    }
    assert!(report.is_valid());

    // The exact removed prism is L*r^2*(1-pi/4) for a right-angle round.
    let expected = 40.0 * 40.0 * 10.0 - 10.0 * 5.0_f64.powi(2) * (1.0 - std::f64::consts::PI / 4.0);
    let resized_volume = brepkit_operations::measure::solid_volume(&topo, resized, 0.02).unwrap();
    let fresh_volume = brepkit_operations::measure::solid_volume(&fresh_topo, fresh, 0.02).unwrap();
    println!(
        "E10 volumes resized={resized_volume:.9} fresh={fresh_volume:.9} expected={expected:.9}"
    );
    assert!((resized_volume - expected).abs() < 0.1);
    assert!((resized_volume - fresh_volume).abs() < 1e-8);

    // Face-surface multisets must be identical to a fresh r=5 construction.
    let surfaces = |topology: &Topology, solid: SolidId| {
        let mut values: Vec<String> = solid_faces(topology, solid)
            .unwrap()
            .into_iter()
            .map(|face| match topology.face(face).unwrap().surface() {
                FaceSurface::Plane { normal, d } => format!(
                    "P({:.9},{:.9},{:.9},{:.9})",
                    normal.x(),
                    normal.y(),
                    normal.z(),
                    d
                ),
                FaceSurface::Cylinder(cylinder) => format!(
                    "C(o={:.9},{:.9},{:.9};a={:.9},{:.9},{:.9};r={:.9})",
                    cylinder.origin().x(),
                    cylinder.origin().y(),
                    cylinder.origin().z(),
                    cylinder.axis().x(),
                    cylinder.axis().y(),
                    cylinder.axis().z(),
                    cylinder.radius()
                ),
                other => surface_name(other).to_string(),
            })
            .collect();
        values.sort();
        values
    };
    assert_eq!(surfaces(&topo, resized), surfaces(&fresh_topo, fresh));
}

// E11: resize the complete trihedral blend network in place: three
// plane-plane cylinder bands plus the exact spherical vertex patch.
#[test]
fn e11_in_place_resize_trihedral_corner() {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, 40.0, 40.0, 40.0).unwrap();
    let vertical = find_vertical_edge(&topo, sharp, 40.0, 40.0);
    let top_edges: Vec<EdgeId> = all_edges(&topo, sharp)
        .into_iter()
        .filter(|&edge| {
            let start = vertex_point(&topo, edge, true);
            let end = vertex_point(&topo, edge, false);
            let at_corner = |point: Point3| {
                (point.x() - 40.0).abs() < 1e-9
                    && (point.y() - 40.0).abs() < 1e-9
                    && (point.z() - 40.0).abs() < 1e-9
            };
            (at_corner(start) || at_corner(end)) && (start.z() - end.z()).abs() < 1e-9
        })
        .collect();
    assert_eq!(top_edges.len(), 2);
    let selected: Vec<EdgeId> = std::iter::once(vertical).chain(top_edges).collect();
    let resized = fillet_v2(&mut topo, sharp, &selected, 3.0).unwrap().solid;

    // The planar builder may rewrite topology shared by its input solid on a
    // successful operation, so construct the fresh oracle independently.
    let mut fresh_topo = Topology::new();
    let fresh_sharp = make_box(&mut fresh_topo, 40.0, 40.0, 40.0).unwrap();
    let fresh_vertical = find_vertical_edge(&fresh_topo, fresh_sharp, 40.0, 40.0);
    let fresh_top_edges: Vec<EdgeId> = all_edges(&fresh_topo, fresh_sharp)
        .into_iter()
        .filter(|&edge| {
            let start = vertex_point(&fresh_topo, edge, true);
            let end = vertex_point(&fresh_topo, edge, false);
            let at_corner = |point: Point3| {
                (point.x() - 40.0).abs() < 1e-9
                    && (point.y() - 40.0).abs() < 1e-9
                    && (point.z() - 40.0).abs() < 1e-9
            };
            (at_corner(start) || at_corner(end)) && (start.z() - end.z()).abs() < 1e-9
        })
        .collect();
    let fresh_selected: Vec<EdgeId> = std::iter::once(fresh_vertical)
        .chain(fresh_top_edges)
        .collect();
    let fresh = fillet_v2(&mut fresh_topo, fresh_sharp, &fresh_selected, 5.0)
        .unwrap()
        .solid;

    let map_coordinate = |value: f64, old: f64, new: f64| {
        if (value - old).abs() < 1e-9 {
            new
        } else {
            value
        }
    };
    let map_point = |point: Point3| {
        Point3::new(
            map_coordinate(point.x(), 37.0, 35.0),
            map_coordinate(point.y(), 37.0, 35.0),
            map_coordinate(point.z(), 37.0, 35.0),
        )
    };

    let faces = solid_faces(&topo, resized).unwrap();
    let mut cylinders = 0;
    let mut spheres = 0;
    for face in &faces {
        let replacement = match topo.face(*face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder) if (cylinder.radius() - 3.0).abs() < 1e-9 => {
                cylinders += 1;
                Some(FaceSurface::Cylinder(
                    brepkit_math::surfaces::CylindricalSurface::with_ref_dir(
                        map_point(cylinder.origin()),
                        cylinder.axis(),
                        5.0,
                        cylinder.x_axis(),
                    )
                    .unwrap(),
                ))
            }
            FaceSurface::Sphere(sphere) if (sphere.radius() - 3.0).abs() < 1e-9 => {
                spheres += 1;
                Some(FaceSurface::Sphere(
                    brepkit_math::surfaces::SphericalSurface::new(map_point(sphere.center()), 5.0)
                        .unwrap(),
                ))
            }
            _ => None,
        };
        if let Some(surface) = replacement {
            topo.face_mut(*face).unwrap().set_surface(surface);
        }
    }
    assert_eq!(cylinders, 3);
    assert_eq!(spheres, 1);

    let mut circle_count = 0;
    for edge in all_edges(&topo, resized) {
        let replacement = match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle) if (circle.radius() - 3.0).abs() < 1e-9 => {
                circle_count += 1;
                Some(EdgeCurve::Circle(
                    brepkit_math::curves::Circle3D::new_with_ref(
                        map_point(circle.center()),
                        circle.normal(),
                        5.0,
                        circle.u_axis(),
                    )
                    .unwrap(),
                ))
            }
            _ => None,
        };
        if let Some(curve) = replacement {
            topo.edge_mut(edge).unwrap().set_curve(curve);
        }
    }
    assert_eq!(circle_count, 6);

    let mut vertices = std::collections::BTreeMap::new();
    for edge in all_edges(&topo, resized) {
        let edge_data = topo.edge(edge).unwrap();
        for vertex in [edge_data.start(), edge_data.end()] {
            vertices.insert(vertex.index(), vertex);
        }
    }
    let mut moved = 0;
    for vertex in vertices.values().copied() {
        let point = topo.vertex(vertex).unwrap().point();
        let replacement = map_point(point);
        if (replacement - point).length() > 1e-12 {
            topo.vertex_mut(vertex).unwrap().set_point(replacement);
            moved += 1;
        }
    }
    println!(
        "E11 retargeted {cylinders} cylinders, {spheres} sphere, {circle_count} circles, {moved} vertices"
    );

    let report = brepkit_operations::validate::validate_solid(&topo, resized).unwrap();
    for issue in &report.issues {
        println!("E11 issue: {:?} {}", issue.severity, issue.description);
    }
    assert!(report.is_valid());

    let resized_volume = brepkit_operations::measure::solid_volume(&topo, resized, 0.02).unwrap();
    let fresh_volume = brepkit_operations::measure::solid_volume(&fresh_topo, fresh, 0.02).unwrap();
    println!("E11 volumes resized={resized_volume:.9} fresh={fresh_volume:.9}");
    assert!((resized_volume - fresh_volume).abs() < 1e-8);

    let surfaces = |topology: &Topology, solid: SolidId| {
        let mut values: Vec<String> = solid_faces(topology, solid)
            .unwrap()
            .into_iter()
            .map(|face| match topology.face(face).unwrap().surface() {
                FaceSurface::Plane { normal, d } => format!(
                    "P({:.9},{:.9},{:.9},{:.9})",
                    normal.x(),
                    normal.y(),
                    normal.z(),
                    d
                ),
                FaceSurface::Cylinder(cylinder) => format!(
                    "C(o={:.9},{:.9},{:.9};a={:.9},{:.9},{:.9};r={:.9})",
                    cylinder.origin().x(),
                    cylinder.origin().y(),
                    cylinder.origin().z(),
                    cylinder.axis().x(),
                    cylinder.axis().y(),
                    cylinder.axis().z(),
                    cylinder.radius()
                ),
                FaceSurface::Sphere(sphere) => format!(
                    "S(c={:.9},{:.9},{:.9};r={:.9})",
                    sphere.center().x(),
                    sphere.center().y(),
                    sphere.center().z(),
                    sphere.radius()
                ),
                other => surface_name(other).to_string(),
            })
            .collect();
        values.sort();
        values
    };
    assert_eq!(surfaces(&topo, resized), surfaces(&fresh_topo, fresh));
}

// E12: exact rational NURBS surfaces are normalized before recognition.
// This pins the editable path: a NURBS cylinder produced by the kernel's
// exact conversion must refit to the same elementary cylinder and radius.
#[test]
fn e12_convert_to_elementary_recovers_exact_nurbs_cylinder() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 7.0, 12.0).unwrap();
    let wall = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let cylinder = match topo.face(wall).unwrap().surface() {
        FaceSurface::Cylinder(cylinder) => cylinder.clone(),
        _ => unreachable!(),
    };
    let nurbs = cylinder.to_nurbs(0.0, 12.0).unwrap();
    topo.face_mut(wall)
        .unwrap()
        .set_surface(FaceSurface::Nurbs(nurbs));
    assert!(matches!(
        topo.face(wall).unwrap().surface(),
        FaceSurface::Nurbs(_)
    ));

    let converted =
        brepkit_operations::heal::convert_to_elementary(&mut topo, solid, 1e-7).unwrap();
    println!("E12 converted {converted} entities");
    assert!(converted >= 1);
    let recovered = match topo.face(wall).unwrap().surface() {
        FaceSurface::Cylinder(cylinder) => cylinder,
        other => panic!("expected recovered cylinder, got {}", surface_name(other)),
    };
    assert!((recovered.radius() - 7.0).abs() < 1e-7);
    assert!(recovered.axis().dot(cylinder.axis()).abs() > 1.0 - 1e-12);
    assert!(
        (recovered.origin() - cylinder.origin())
            .cross(cylinder.axis())
            .length()
            < 1e-7
    );
}

// E13: document the current copy gap. Direct editing needs an entity-map copy
// that also remaps pcurves; copy_solid_with_face_map currently drops them.
#[test]
fn e13_copy_solid_face_map_does_not_copy_pcurves_yet() {
    use brepkit_math::curves2d::{Curve2D, Line2D};
    use brepkit_math::vec::{Point2, Vec2};
    use brepkit_topology::pcurve::PCurve;

    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
    let source_face = solid_faces(&topo, solid).unwrap()[0];
    let source_edge = topo
        .wire(topo.face(source_face).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();
    topo.pcurves_mut().set(
        source_edge,
        source_face,
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
            0.0,
            1.0,
        ),
    );
    let (copy, face_map) =
        brepkit_operations::copy::copy_solid_with_face_map(&mut topo, solid).unwrap();
    let copied_face_index = face_map[&source_face.index()];
    let copied_face = topo.face_id_from_index(copied_face_index).unwrap();
    let copied_edge = topo
        .wire(topo.face(copied_face).unwrap().outer_wire())
        .unwrap()
        .edges()[0]
        .edge();
    assert!(topo.pcurves().contains(source_edge, source_face));
    assert!(
        !topo.pcurves().contains(copied_edge, copied_face),
        "update this probe when copy_solid_with_entity_maps remaps pcurves"
    );
    assert_ne!(copy, solid);
}

// E14: establish the representation baseline. STEP import and the two
// production analytic fillet paths currently use exact 3D edge/surface
// geometry without populating the central pcurve registry.
#[test]
fn e14_current_analytic_blends_and_step_import_have_no_registry_pcurves() {
    let mut open_topo = Topology::new();
    let open_sharp = make_box(&mut open_topo, 40.0, 40.0, 10.0).unwrap();
    let open_edge = find_vertical_edge(&open_topo, open_sharp, 40.0, 40.0);
    let _open = fillet_v2(&mut open_topo, open_sharp, &[open_edge], 3.0)
        .unwrap()
        .solid;
    println!("E14 open fillet pcurves={}", open_topo.pcurves().len());
    assert_eq!(open_topo.pcurves().len(), 0);

    let mut ring_topo = Topology::new();
    let (plate, rim) = drilled_plate(&mut ring_topo);
    let _ring = fillet_v2(&mut ring_topo, plate, &[rim], 3.0).unwrap().solid;
    println!("E14 ring fillet pcurves={}", ring_topo.pcurves().len());
    assert_eq!(ring_topo.pcurves().len(), 0);

    let step = include_str!("../../io/tests/data/openzcad_e_analytic_fillet_plate.step");
    let mut step_topo = Topology::new();
    let solids = brepkit_io::step::reader::read_step(step, &mut step_topo).unwrap();
    assert_eq!(solids.len(), 1);
    println!("E14 STEP import pcurves={}", step_topo.pcurves().len());
    assert_eq!(step_topo.pcurves().len(), 0);
}

fn effective_normal(topo: &Topology, face: FaceId, point: Point3) -> Option<Vec3> {
    let face_data = topo.face(face).unwrap();
    if let Some(normal) = face_data.effective_plane_normal() {
        return Some(normal);
    }
    let (u, v) = face_data.surface().project_point(point)?;
    let normal = face_data.surface().normal(u, v);
    if face_data.is_reversed() {
        Some(-normal)
    } else {
        Some(normal)
    }
}

fn edge_is_g1_between(topo: &Topology, edge: EdgeId, a: FaceId, b: FaceId) -> bool {
    let edge_data = topo.edge(edge).unwrap();
    let start = topo.vertex(edge_data.start()).unwrap().point();
    let end = topo.vertex(edge_data.end()).unwrap().point();
    let (t0, mut t1) = edge_data.curve().domain_with_endpoints(start, end);
    if edge_data.is_closed() {
        t1 = t0 + std::f64::consts::TAU;
    }
    let samples: Vec<bool> = [0.2, 0.4, 0.6, 0.8]
        .into_iter()
        .filter_map(|fraction| {
            let t = (t1 - t0).mul_add(fraction, t0);
            let point = edge_data.curve().evaluate_with_endpoints(t, start, end);
            Some(
                effective_normal(topo, a, point)?.dot(effective_normal(topo, b, point)?)
                    > 1.0 - 1e-10,
            )
        })
        .collect();
    samples.len() >= 3 && samples.into_iter().all(|aligned| aligned)
}

// E15: pin the corrected G1 convention on imported STEP. Smooth blend
// contacts have aligned effective outward normals (angle near zero); the
// ordinary bore has no such contacts.
#[test]
fn e15_step_fillet_candidates_use_aligned_normals_not_pi_dihedral() {
    let import = |text: &str| {
        let mut topo = Topology::new();
        let solid = brepkit_io::step::reader::read_step(text, &mut topo).unwrap()[0];
        (topo, solid)
    };
    let fillet_step = include_str!("../../io/tests/data/openzcad_e_analytic_fillet_plate.step");
    let (fillet_topo, fillet_solid) = import(fillet_step);
    let fillet_adjacency = fillet_topo.build_adjacency(fillet_solid).unwrap();
    let mut candidates = Vec::new();
    for face in solid_faces(&fillet_topo, fillet_solid).unwrap() {
        let FaceSurface::Cylinder(cylinder) = fillet_topo.face(face).unwrap().surface() else {
            continue;
        };
        let mut g1_edges = 0;
        let face_data = fillet_topo.face(face).unwrap();
        for wire in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            for oriented in fillet_topo.wire(wire).unwrap().edges() {
                let adjacent = fillet_adjacency.faces_for_edge(oriented.edge());
                if adjacent.len() != 2 || adjacent[0] == adjacent[1] {
                    continue;
                }
                let other = if adjacent[0] == face {
                    adjacent[1]
                } else {
                    adjacent[0]
                };
                if edge_is_g1_between(&fillet_topo, oriented.edge(), face, other) {
                    g1_edges += 1;
                }
            }
        }
        if g1_edges == 2 {
            candidates.push(cylinder.radius());
        }
    }
    candidates.sort_by(f64::total_cmp);
    println!("E15 imported fillet candidates={candidates:?}");
    assert_eq!(candidates, vec![3.0, 3.0, 3.0, 3.0]);

    let bore_step = include_str!("../../io/tests/data/openzcad_a_export_bored_plate.step");
    let (bore_topo, bore_solid) = import(bore_step);
    let bore_adjacency = bore_topo.build_adjacency(bore_solid).unwrap();
    let bore_face = solid_faces(&bore_topo, bore_solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(
                bore_topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .unwrap();
    let bore_data = bore_topo.face(bore_face).unwrap();
    let bore_g1 = std::iter::once(bore_data.outer_wire())
        .chain(bore_data.inner_wires().iter().copied())
        .flat_map(|wire| bore_topo.wire(wire).unwrap().edges().iter())
        .filter(|oriented| {
            let adjacent = bore_adjacency.faces_for_edge(oriented.edge());
            if adjacent.len() != 2 || adjacent[0] == adjacent[1] {
                return false;
            }
            let other = if adjacent[0] == bore_face {
                adjacent[1]
            } else {
                adjacent[0]
            };
            edge_is_g1_between(&bore_topo, oriented.edge(), bore_face, other)
        })
        .count();
    println!("E15 bore G1 edges={bore_g1}");
    assert_eq!(bore_g1, 0);
}

fn counterbore(topo: &mut Topology, large_radius: f64, depth_from_top: f64) -> SolidId {
    let body = make_box(topo, 20.0, 20.0, 10.0).unwrap();
    let small = make_cylinder(topo, 1.5, 12.0).unwrap();
    transform_solid(topo, small, &Mat4::translation(10.0, 10.0, -1.0)).unwrap();
    let through = boolean(topo, BooleanOp::Cut, body, small).unwrap();
    let large = make_cylinder(topo, large_radius, depth_from_top + 1.0).unwrap();
    transform_solid(
        topo,
        large,
        &Mat4::translation(10.0, 10.0, 10.0 - depth_from_top),
    )
    .unwrap();
    boolean(topo, BooleanOp::Cut, through, large).unwrap()
}

fn cylinder_face_of_radius(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder)
                    if (cylinder.radius() - radius).abs() < 1e-9
                        && topo.face(face).unwrap().is_reversed()
            )
        })
        .unwrap()
}

// E16: staged counterbore radius edit must be direct surgery. The generic
// cylindrical-resize boolean cannot preserve an analytic counterbore stage.
#[test]
fn e16_counterbore_large_stage_radius_edit_matches_fresh_feature() {
    let mut topo = Topology::new();
    let edited = counterbore(&mut topo, 3.0, 3.0);
    let large_face = cylinder_face_of_radius(&topo, edited, 3.0);
    let old_cylinder = match topo.face(large_face).unwrap().surface() {
        FaceSurface::Cylinder(cylinder) => cylinder.clone(),
        _ => unreachable!(),
    };
    topo.face_mut(large_face)
        .unwrap()
        .set_surface(FaceSurface::Cylinder(
            brepkit_math::surfaces::CylindricalSurface::with_ref_dir(
                old_cylinder.origin(),
                old_cylinder.axis(),
                4.0,
                old_cylinder.x_axis(),
            )
            .unwrap(),
        ));

    let shoulder = solid_faces(&topo, edited)
        .unwrap()
        .into_iter()
        .find(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { normal, d } => {
                normal.dot(Vec3::new(0.0, 0.0, 1.0)).abs() > 1.0 - 1e-12
                    && (d.abs() - 7.0).abs() < 1e-9
                    && !topo.face(face).unwrap().inner_wires().is_empty()
            }
            _ => false,
        })
        .unwrap();
    let top = solid_faces(&topo, edited)
        .unwrap()
        .into_iter()
        .find(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { normal, d } => {
                normal.dot(Vec3::new(0.0, 0.0, 1.0)).abs() > 1.0 - 1e-12
                    && (d.abs() - 10.0).abs() < 1e-9
                    && !topo.face(face).unwrap().inner_wires().is_empty()
            }
            _ => false,
        })
        .unwrap();
    let affected_faces = [large_face, shoulder, top];
    let mut affected_edges = std::collections::BTreeMap::new();
    let mut affected_vertices = std::collections::BTreeMap::new();
    for face in affected_faces {
        let face_data = topo.face(face).unwrap();
        for wire in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire).unwrap().edges() {
                affected_edges.insert(oriented.edge().index(), oriented.edge());
                let edge = topo.edge(oriented.edge()).unwrap();
                for vertex in [edge.start(), edge.end()] {
                    affected_vertices.insert(vertex.index(), vertex);
                }
            }
        }
    }
    let mut changed_circles = 0;
    for edge in affected_edges.values().copied() {
        let replacement = match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle)
                if (circle.radius() - 3.0).abs() < 1e-9
                    && ((circle.center().z() - 7.0).abs() < 1e-9
                        || (circle.center().z() - 10.0).abs() < 1e-9) =>
            {
                changed_circles += 1;
                Some(EdgeCurve::Circle(
                    brepkit_math::curves::Circle3D::new_with_ref(
                        circle.center(),
                        circle.normal(),
                        4.0,
                        circle.u_axis(),
                    )
                    .unwrap(),
                ))
            }
            _ => None,
        };
        if let Some(curve) = replacement {
            topo.edge_mut(edge).unwrap().set_curve(curve);
        }
    }
    assert_eq!(changed_circles, 2);
    let axis_origin = Point3::new(10.0, 10.0, 0.0);
    let mut moved_vertices = 0;
    for vertex in affected_vertices.values().copied() {
        let point = topo.vertex(vertex).unwrap().point();
        let radial = Vec3::new(
            point.x() - axis_origin.x(),
            point.y() - axis_origin.y(),
            0.0,
        );
        if (radial.length() - 3.0).abs() < 1e-9
            && ((point.z() - 7.0).abs() < 1e-9 || (point.z() - 10.0).abs() < 1e-9)
        {
            let direction = radial.normalize().unwrap();
            topo.vertex_mut(vertex).unwrap().set_point(Point3::new(
                axis_origin.x() + direction.x() * 4.0,
                axis_origin.y() + direction.y() * 4.0,
                point.z(),
            ));
            moved_vertices += 1;
        }
    }
    assert_eq!(moved_vertices, 2);

    let mut fresh_topo = Topology::new();
    let fresh = counterbore(&mut fresh_topo, 4.0, 3.0);
    let edited_volume = brepkit_operations::measure::solid_volume(&topo, edited, 0.02).unwrap();
    let fresh_volume = brepkit_operations::measure::solid_volume(&fresh_topo, fresh, 0.02).unwrap();
    let expected = 4000.0
        - std::f64::consts::PI
            * (1.5_f64.powi(2) * 10.0 + (4.0_f64.powi(2) - 1.5_f64.powi(2)) * 3.0);
    println!(
        "E16 volumes edited={edited_volume:.9} fresh={fresh_volume:.9} expected={expected:.9}"
    );
    assert!((edited_volume - fresh_volume).abs() < 1e-8);
    assert!((edited_volume - expected).abs() < 0.1);
    assert!(
        brepkit_operations::validate::validate_solid(&topo, edited)
            .unwrap()
            .is_valid()
    );

    let radii = |topology: &Topology, solid: SolidId| {
        let mut values: Vec<f64> = solid_faces(topology, solid)
            .unwrap()
            .into_iter()
            .filter_map(|face| match topology.face(face).unwrap().surface() {
                FaceSurface::Cylinder(cylinder) if topology.face(face).unwrap().is_reversed() => {
                    Some(cylinder.radius())
                }
                _ => None,
            })
            .collect();
        values.sort_by(f64::total_cmp);
        values
    };
    assert_eq!(radii(&topo, edited), vec![1.5, 4.0]);
    assert_eq!(radii(&fresh_topo, fresh), vec![1.5, 4.0]);
}

// E17: exact counterbore depth edit by moving the stage interface. The two
// cylinder carriers stay fixed; the annular shoulder, its two rings, and seam
// vertices move together from z=7 to z=5.
#[test]
fn e17_counterbore_depth_edit_matches_fresh_feature() {
    let mut topo = Topology::new();
    let edited = counterbore(&mut topo, 3.0, 3.0);

    let shoulder = solid_faces(&topo, edited)
        .unwrap()
        .into_iter()
        .find(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { normal, d } => {
                normal.dot(Vec3::new(0.0, 0.0, 1.0)).abs() > 1.0 - 1e-12
                    && (d.abs() - 7.0).abs() < 1e-9
                    && !topo.face(face).unwrap().inner_wires().is_empty()
            }
            _ => false,
        })
        .unwrap();
    let FaceSurface::Plane { normal, .. } = topo.face(shoulder).unwrap().surface() else {
        unreachable!()
    };
    let normal = *normal;
    topo.face_mut(shoulder)
        .unwrap()
        .set_surface(FaceSurface::Plane {
            normal,
            d: normal.z() * 5.0,
        });

    let feature_faces: Vec<FaceId> = solid_faces(&topo, edited)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            face == shoulder
                || matches!(
                    topo.face(face).unwrap().surface(),
                    FaceSurface::Cylinder(cylinder)
                        if (cylinder.radius() - 1.5).abs() < 1e-9
                            || (cylinder.radius() - 3.0).abs() < 1e-9
                )
        })
        .collect();
    let mut feature_edges = std::collections::BTreeMap::new();
    let mut feature_vertices = std::collections::BTreeMap::new();
    for face in feature_faces {
        let face_data = topo.face(face).unwrap();
        for wire in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire).unwrap().edges() {
                feature_edges.insert(oriented.edge().index(), oriented.edge());
                let edge = topo.edge(oriented.edge()).unwrap();
                for vertex in [edge.start(), edge.end()] {
                    feature_vertices.insert(vertex.index(), vertex);
                }
            }
        }
    }
    let mut changed_circles = 0;
    for edge in feature_edges.values().copied() {
        let replacement = match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle) if (circle.center().z() - 7.0).abs() < 1e-9 => {
                changed_circles += 1;
                Some(EdgeCurve::Circle(
                    brepkit_math::curves::Circle3D::new_with_ref(
                        Point3::new(circle.center().x(), circle.center().y(), 5.0),
                        circle.normal(),
                        circle.radius(),
                        circle.u_axis(),
                    )
                    .unwrap(),
                ))
            }
            _ => None,
        };
        if let Some(curve) = replacement {
            topo.edge_mut(edge).unwrap().set_curve(curve);
        }
    }
    assert_eq!(changed_circles, 2);
    let mut moved_vertices = 0;
    for vertex in feature_vertices.values().copied() {
        let point = topo.vertex(vertex).unwrap().point();
        if (point.z() - 7.0).abs() < 1e-9 {
            topo.vertex_mut(vertex)
                .unwrap()
                .set_point(Point3::new(point.x(), point.y(), 5.0));
            moved_vertices += 1;
        }
    }
    assert_eq!(moved_vertices, 2);

    let mut fresh_topo = Topology::new();
    let fresh = counterbore(&mut fresh_topo, 3.0, 5.0);
    let edited_volume = brepkit_operations::measure::solid_volume(&topo, edited, 0.02).unwrap();
    let fresh_volume = brepkit_operations::measure::solid_volume(&fresh_topo, fresh, 0.02).unwrap();
    let expected = 4000.0
        - std::f64::consts::PI
            * (1.5_f64.powi(2) * 10.0 + (3.0_f64.powi(2) - 1.5_f64.powi(2)) * 5.0);
    println!(
        "E17 volumes edited={edited_volume:.9} fresh={fresh_volume:.9} expected={expected:.9}"
    );
    assert!((edited_volume - fresh_volume).abs() < 1e-8);
    assert!((edited_volume - expected).abs() < 0.1);
    let report = brepkit_operations::validate::validate_solid(&topo, edited).unwrap();
    for issue in &report.issues {
        println!("E17 issue: {:?} {}", issue.severity, issue.description);
    }
    assert!(report.is_valid());
}

// E18: prove the canonical axial certificate can be recovered from topology
// without feature history: two ordered coaxial stages, one annular shoulder,
// and two openings for a through counterbore.
#[test]
fn e18_counterbore_certificate_recovers_ordered_stages_and_interfaces() {
    let mut topo = Topology::new();
    let solid = counterbore(&mut topo, 3.0, 3.0);
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let mut stages = Vec::new();
    for face in solid_faces(&topo, solid).unwrap() {
        let face_data = topo.face(face).unwrap();
        let FaceSurface::Cylinder(cylinder) = face_data.surface() else {
            continue;
        };
        if !face_data.is_reversed() || cylinder.axis().dot(axis).abs() < 1.0 - 1e-12 {
            continue;
        }
        let mut z_min = f64::INFINITY;
        let mut z_max = f64::NEG_INFINITY;
        let mut rings = std::collections::BTreeSet::new();
        for wire in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire).unwrap().edges() {
                let edge = topo.edge(oriented.edge()).unwrap();
                for vertex in [edge.start(), edge.end()] {
                    let point = topo.vertex(vertex).unwrap().point();
                    z_min = z_min.min(point.z());
                    z_max = z_max.max(point.z());
                }
                if let EdgeCurve::Circle(circle) = edge.curve() {
                    rings.insert((circle.center().z() * 1e9).round() as i64);
                }
            }
        }
        stages.push((z_min, z_max, cylinder.radius(), rings.len(), face));
    }
    stages.sort_by(|a, b| a.0.total_cmp(&b.0));
    println!("E18 stages={stages:?}");
    assert_eq!(stages.len(), 2);
    assert!((stages[0].0 - 0.0).abs() < 1e-9);
    assert!((stages[0].1 - 7.0).abs() < 1e-9);
    assert!((stages[0].2 - 1.5).abs() < 1e-9);
    assert_eq!(stages[0].3, 2);
    assert!((stages[1].0 - 7.0).abs() < 1e-9);
    assert!((stages[1].1 - 10.0).abs() < 1e-9);
    assert!((stages[1].2 - 3.0).abs() < 1e-9);
    assert_eq!(stages[1].3, 2);

    let mut shoulder_count = 0;
    let mut opening_z = Vec::new();
    for face in solid_faces(&topo, solid).unwrap() {
        let face_data = topo.face(face).unwrap();
        let FaceSurface::Plane { normal, d } = face_data.surface() else {
            continue;
        };
        if normal.dot(axis).abs() < 1.0 - 1e-12 {
            continue;
        }
        let mut circle_radii = Vec::new();
        for wire in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire).unwrap().edges() {
                if let EdgeCurve::Circle(circle) = topo.edge(oriented.edge()).unwrap().curve() {
                    circle_radii.push(circle.radius());
                }
            }
        }
        circle_radii.sort_by(f64::total_cmp);
        if circle_radii == vec![1.5, 3.0] && (d.abs() - 7.0).abs() < 1e-9 {
            shoulder_count += 1;
        } else if circle_radii
            .iter()
            .any(|radius| (*radius - 1.5).abs() < 1e-9 || (*radius - 3.0).abs() < 1e-9)
        {
            opening_z.push(d.abs());
        }
    }
    opening_z.sort_by(f64::total_cmp);
    println!("E18 shoulder_count={shoulder_count} openings={opening_z:?}");
    assert_eq!(shoulder_count, 1);
    assert_eq!(opening_z, vec![0.0, 10.0]);
}

// E19: exact tier-2 unfillet for a closed plane-cylinder ring. Collapse the
// torus band and its two spring circles back to one sharp circle shared by the
// support plane and cylinder; retarget the cylinder seam and drop the band.
#[test]
fn e19_unfillet_plane_cylinder_ring_restores_sharp_bore() {
    let mut topo = Topology::new();
    let (sharp, rim) = drilled_plate(&mut topo);
    let sharp_volume = brepkit_operations::measure::solid_volume(&topo, sharp, 0.02).unwrap();
    let filleted = fillet_v2(&mut topo, sharp, &[rim], 3.0).unwrap().solid;

    let band = solid_faces(&topo, filleted)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    let adjacency = topo.build_adjacency(filleted).unwrap();
    let band_edges: Vec<EdgeId> = topo
        .wire(topo.face(band).unwrap().outer_wire())
        .unwrap()
        .edges()
        .iter()
        .map(brepkit_topology::wire::OrientedEdge::edge)
        .collect();
    let mut spring_data = Vec::new();
    for edge in band_edges {
        let adjacent = adjacency.faces_for_edge(edge);
        if adjacent.len() != 2 || adjacent[0] == adjacent[1] {
            continue;
        }
        let other = if adjacent[0] == band {
            adjacent[1]
        } else {
            adjacent[0]
        };
        spring_data.push((edge, other));
    }
    assert_eq!(spring_data.len(), 2);
    let (plate_edge, plate_face) = spring_data
        .iter()
        .copied()
        .find(|(_, face)| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Plane { .. }
            )
        })
        .unwrap();
    let (wall_edge, wall_face) = spring_data
        .iter()
        .copied()
        .find(|(_, face)| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(_)
            )
        })
        .unwrap();
    let wall_circle = match topo.edge(wall_edge).unwrap().curve() {
        EdgeCurve::Circle(circle) => circle.clone(),
        _ => unreachable!(),
    };
    let wall = match topo.face(wall_face).unwrap().surface() {
        FaceSurface::Cylinder(cylinder) => cylinder.clone(),
        _ => unreachable!(),
    };
    let (plane_normal, plane_d) = match topo.face(plate_face).unwrap().surface() {
        FaceSurface::Plane { normal, d } => (*normal, *d),
        _ => unreachable!(),
    };
    let step = plane_d
        - plane_normal.dot(Vec3::new(
            wall.origin().x(),
            wall.origin().y(),
            wall.origin().z(),
        ));
    let sharp_center = wall.origin() + plane_normal * step;
    let sharp_circle = brepkit_math::curves::Circle3D::new_with_ref(
        sharp_center,
        wall_circle.normal(),
        wall.radius(),
        wall_circle.u_axis(),
    )
    .unwrap();

    // Preserve the wall spring edge as the recovered sharp rim. Its seam
    // vertex is shared by the wall's seam generator, so moving it closes the
    // extended wall at the sharp plane.
    let sharp_vertex = topo.edge(wall_edge).unwrap().start();
    topo.vertex_mut(sharp_vertex)
        .unwrap()
        .set_point(sharp_circle.evaluate(0.0));
    topo.edge_mut(wall_edge)
        .unwrap()
        .set_curve(EdgeCurve::Circle(sharp_circle.clone()));

    // Replace the plate spring occurrence by the recovered wall edge while
    // preserving the plate's existing traversal direction relative to its
    // old spring circle.
    let plate_face_data = topo.face(plate_face).unwrap();
    let plate_wires: Vec<_> = std::iter::once(plate_face_data.outer_wire())
        .chain(plate_face_data.inner_wires().iter().copied())
        .collect();
    let mut replaced = 0;
    for wire in plate_wires {
        let old_occurrences = topo.wire(wire).unwrap().edges().to_vec();
        for (slot, occurrence) in old_occurrences.iter().enumerate() {
            if occurrence.edge() != plate_edge {
                continue;
            }
            let old_circle = match topo.edge(plate_edge).unwrap().curve() {
                EdgeCurve::Circle(circle) => circle,
                _ => unreachable!(),
            };
            let same_parameter_direction =
                old_circle.tangent(0.0).dot(sharp_circle.tangent(0.0)) > 0.0;
            let forward = if same_parameter_direction {
                occurrence.is_forward()
            } else {
                !occurrence.is_forward()
            };
            topo.wire_mut(wire).unwrap().edges_mut()[slot] =
                brepkit_topology::wire::OrientedEdge::new(wall_edge, forward);
            replaced += 1;
        }
    }
    assert_eq!(replaced, 1);

    let source_shell = topo.solid(filleted).unwrap().outer_shell();
    let kept_faces: Vec<FaceId> = topo
        .shell(source_shell)
        .unwrap()
        .faces()
        .iter()
        .copied()
        .filter(|face| *face != band)
        .collect();
    let new_shell = topo.add_shell(brepkit_topology::shell::Shell::new(kept_faces).unwrap());
    let healed = topo.add_solid(brepkit_topology::solid::Solid::new(new_shell, Vec::new()));

    let report = brepkit_operations::validate::validate_solid(&topo, healed).unwrap();
    for issue in &report.issues {
        println!("E19 issue: {:?} {}", issue.severity, issue.description);
    }
    assert!(report.is_valid());
    let healed_volume = brepkit_operations::measure::solid_volume(&topo, healed, 0.02).unwrap();
    println!("E19 volumes healed={healed_volume:.9} sharp={sharp_volume:.9}");
    assert!((healed_volume - sharp_volume).abs() < 1e-8);
    assert_eq!(solid_faces(&topo, healed).unwrap().len(), 7);
    assert_eq!(
        solid_faces(&topo, healed)
            .unwrap()
            .into_iter()
            .filter(|face| matches!(topo.face(*face).unwrap().surface(), FaceSurface::Torus(_)))
            .count(),
        0
    );
}
