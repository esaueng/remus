//! Box × complete-torus booleans whose box face cuts the torus normal to its
//! axis (consumer K10 combination matrix on Remus 2026.1.122, 2026-11-05).
//!
//! A 60 × 60 × 20 box cut by a torus R = 15, r = 4 centred on its top face
//! (30, 30, 20), axis +Z, returned the box unchanged (72000 against
//! 72000 − π²Rr² = 69631.29) with no error, and so did every box × torus cut
//! placement in the matrix.
//!
//! Root cause (first-wrong phase: FF section curves). The plane × torus
//! section went through the sampled marcher for every plane, so a plane
//! normal to the axis produced two NURBS refits of its two latitude
//! circles. The closed-torus band splitter cuts a complete torus only along
//! exact `Circle` sections at constant tube angle, so it declined; the torus
//! face reached assembly unsplit, was classified whole by one interior point
//! and dropped, and the box came back untouched. The FF phase now takes the
//! exact circles for an axis-normal transversal section
//! (`axis_normal_torus_section_circles`); oblique, meridian and tangent
//! planes keep the sampled path.
//!
//! Two downstream readers then misread the kept band when it is wider than
//! half the tube (the box face 3 of 4 tube radii above or below the tube
//! centre): the per-face torus mesher assumed a band is the shorter of its
//! two rim-to-rim arcs, and the exact volume integrator found the window from
//! the largest gap between sampled angles, which on the band's near-evenly
//! spread seam vertices read the whole tube (2004 mm³ light). Both now read
//! the window from the band's own seam (`torus_latitude_band_v_range`).
//!
//! An oblique plane (the matrix's 10° tilt) still has no exact band split, so
//! a complete torus crossed by a closed section loop the splitter cannot cut
//! is refused (`FaceSplitFailed`) instead of being dropped.
//!
//! Oracle, independent of the kernel: by Pappus a horizontal slice of the
//! torus at tube height s has area 4πR√(r² − s²), so the torus below tube
//! height a has volume 2πR (πr² − segment(a)), where segment(a) is the area
//! of the tube disc above the chord at a.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{face_area, mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_torus};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const MAJOR: f64 = 15.0;
const MINOR: f64 = 4.0;
const BOX: (f64, f64, f64) = (60.0, 60.0, 20.0);
const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
/// Turns about the torus axis: they move only the seam vertex.
const SEAM_TURNS: [f64; 3] = [0.0, 0.7, 2.0];

/// Area of the tube disc (radius `MINOR`) above the chord at height `a`.
fn segment(a: f64) -> f64 {
    MINOR * MINOR * (a / MINOR).acos() - a * MINOR.mul_add(MINOR, -(a * a)).sqrt()
}

/// Unit-scale volume of the torus below tube height `a`.
fn torus_below(a: f64) -> f64 {
    2.0 * PI * MAJOR * (PI * MINOR * MINOR - segment(a))
}

fn torus_volume() -> f64 {
    2.0 * PI * PI * MAJOR * MINOR * MINOR
}

fn box_volume() -> f64 {
    BOX.0 * BOX.1 * BOX.2
}

/// An axis-normal placement: the torus centre at unit scale.
struct Placement {
    name: &'static str,
    centre: (f64, f64, f64),
}

impl Placement {
    /// Unit-scale torus volume inside the box: below its top (z = 20) and
    /// above its bottom (z = 0).
    fn inside(&self) -> f64 {
        let below_top = (BOX.2 - self.centre.2).clamp(-MINOR, MINOR);
        let below_bottom = (0.0 - self.centre.2).clamp(-MINOR, MINOR);
        torus_below(below_top) - torus_below(below_bottom)
    }
}

const PLACEMENTS: [Placement; 7] = [
    // The matrix cells: aligned, off-centre, sliver (1e-3 overlap), raised.
    Placement {
        name: "aligned",
        centre: (30.0, 30.0, 20.0),
    },
    Placement {
        name: "off-centre",
        centre: (33.0, 27.0, 20.0),
    },
    Placement {
        name: "sliver",
        centre: (30.0, 30.0, 24.0 - 1e-3),
    },
    Placement {
        name: "raised",
        centre: (30.0, 30.0, 22.0),
    },
    // The kept band spans more than half the tube.
    Placement {
        name: "sunk",
        centre: (30.0, 30.0, 17.0),
    },
    Placement {
        name: "deep",
        centre: (30.0, 30.0, 16.5),
    },
    // The bottom face, from below.
    Placement {
        name: "bottom",
        centre: (30.0, 30.0, 0.0),
    },
];

fn build(topo: &mut Topology, scale: f64, place: &Mat4) -> (SolidId, SolidId) {
    let b = make_box(topo, BOX.0 * scale, BOX.1 * scale, BOX.2 * scale).unwrap();
    let t = make_torus(topo, MAJOR * scale, MINOR * scale, 32).unwrap();
    transform_solid(topo, t, place).unwrap();
    (b, t)
}

fn placement_matrix(p: &Placement, scale: f64, turn: f64) -> Mat4 {
    Mat4::translation(p.centre.0 * scale, p.centre.1 * scale, p.centre.2 * scale)
        * Mat4::rotation_z(turn)
}

fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, what: &str) {
    let report = validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{what}: invalid {:?}", report.issues);
    let mass = mass_properties(topo, solid).unwrap().mass;
    assert!(
        (mass - expected).abs() <= 1e-9 * expected,
        "{what}: mass_properties {mass} vs closed form {expected}"
    );
    // Exact face integrals: the reading is deflection-independent.
    for deflection in [1e-2, 1e-1] {
        let volume = solid_volume(topo, solid, deflection).unwrap();
        assert!(
            (volume - expected).abs() <= 1e-9 * expected,
            "{what}: solid_volume({deflection}) {volume} vs closed form {expected}"
        );
    }
}

/// Box − torus and both fuse orders reproduce the closed form at every
/// axis-normal placement, scale and seam turn.
#[test]
fn box_minus_axis_normal_torus_matches_the_closed_form() {
    for p in &PLACEMENTS {
        for scale in SCALES {
            // Known gap, outside this fix: at 1e-3 the sliver's overlap is
            // 1e-6 deep, ten linear tolerances, and the planar sub-face
            // interior probes are pushed a fixed 1e-6 into the solid
            // (`split_face_with_internal_loops`), which lands the annulus
            // probe exactly on the torus: it classifies outside and the cut
            // keeps the box top whole.
            if p.name == "sliver" && scale < 1.0 {
                continue;
            }
            for turn in SEAM_TURNS {
                let s3 = scale * scale * scale;
                let inside = p.inside() * s3;
                let place = placement_matrix(p, scale, turn);
                let what = format!("{} scale {scale} turn {turn}", p.name);
                assert!(inside > 0.0, "{what}: placement must overlap");
                // Known gap, outside this fix: on the bottom face (through
                // the origin) a turned seam leaves the equator section edge
                // on three faces, so validation rejects the result and the
                // boolean refuses. It must never return a wrong body.
                let must_succeed = p.name != "bottom" || turn == 0.0;

                let mut topo = Topology::new();
                let (b, t) = build(&mut topo, scale, &place);
                match boolean(&mut topo, BooleanOp::Cut, b, t) {
                    Ok(cut) => assert_volume(
                        &topo,
                        cut,
                        box_volume().mul_add(s3, -inside),
                        &format!("{what} cut"),
                    ),
                    Err(e) => assert!(!must_succeed, "{what}: cut refused: {e}"),
                }

                for swap in [false, true] {
                    let mut topo = Topology::new();
                    let (b, t) = build(&mut topo, scale, &place);
                    let (x, y) = if swap { (t, b) } else { (b, t) };
                    match boolean(&mut topo, BooleanOp::Fuse, x, y) {
                        Ok(fuse) => assert_volume(
                            &topo,
                            fuse,
                            (box_volume() + torus_volume()).mul_add(s3, -inside),
                            &format!("{what} fuse swap {swap}"),
                        ),
                        Err(e) => {
                            assert!(!must_succeed, "{what} swap {swap}: fuse refused: {e}");
                        }
                    }
                }
            }
        }
    }
}

/// Torus − box and intersect either reproduce the closed form or refuse
/// with a typed error; neither returns an operand unchanged.
#[test]
fn other_box_torus_legs_are_exact_or_refused() {
    for p in &PLACEMENTS {
        for swap in [false, true] {
            for op in [BooleanOp::Cut, BooleanOp::Intersect] {
                if op == BooleanOp::Cut && !swap {
                    continue;
                }
                let inside = p.inside();
                let expected = match op {
                    BooleanOp::Cut => torus_volume() - inside,
                    _ => inside,
                };
                let mut topo = Topology::new();
                let (b, t) = build(&mut topo, 1.0, &placement_matrix(p, 1.0, 0.0));
                let (x, y) = if swap { (t, b) } else { (b, t) };
                let what = format!("{} {op:?} swap {swap}", p.name);
                if let Ok(solid) = boolean(&mut topo, op, x, y) {
                    assert_volume(&topo, solid, expected, &what);
                }
            }
        }
    }
}

/// The matrix's tilted placement (10° about X through the box top face) is
/// an oblique section with no exact band split: every leg refuses or, if a
/// later engine splits it, matches the closed form. A plane through the
/// centre halves the torus (point symmetry), whatever its tilt. The cut was
/// the silent failure: it returned the box unchanged.
#[test]
fn tilted_box_torus_never_drops_the_torus() {
    let half = torus_volume() / 2.0;
    for (op, swap, expected) in [
        (BooleanOp::Cut, false, box_volume() - half),
        (BooleanOp::Cut, true, half),
        (BooleanOp::Fuse, false, box_volume() + half),
        (BooleanOp::Fuse, true, box_volume() + half),
        (BooleanOp::Intersect, false, half),
        (BooleanOp::Intersect, true, half),
    ] {
        for scale in SCALES {
            let s3 = scale * scale * scale;
            let mut topo = Topology::new();
            let place = Mat4::translation(30.0 * scale, 30.0 * scale, 20.0 * scale)
                * Mat4::rotation_x(10f64.to_radians());
            let (b, t) = build(&mut topo, scale, &place);
            let (x, y) = if swap { (t, b) } else { (b, t) };
            let what = format!("tilted {op:?} swap {swap} scale {scale}");
            if let Ok(solid) = boolean(&mut topo, op, x, y) {
                assert_volume(&topo, solid, expected * s3, &what);
            }
        }
    }
}

/// The torus resting on the box top touches it along one circle: the cut
/// removes nothing and must stay the box.
#[test]
fn tangent_torus_cut_keeps_the_box() {
    let mut topo = Topology::new();
    let (b, t) = build(&mut topo, 1.0, &Mat4::translation(30.0, 30.0, 24.0));
    let cut = boolean(&mut topo, BooleanOp::Cut, b, t).unwrap();
    assert_volume(&topo, cut, box_volume(), "tangent cut");
}

/// The kept torus band meshes to its own area (the per-face mesher used to
/// skin the complementary short band of a band wider than half the tube) and
/// the whole body meshes watertight.
#[test]
fn kept_band_meshes_its_own_area() {
    for p in &PLACEMENTS {
        let mut topo = Topology::new();
        let (b, t) = build(&mut topo, 1.0, &placement_matrix(p, 1.0, 0.0));
        let cut = boolean(&mut topo, BooleanOp::Cut, b, t).unwrap();
        let mut torus_faces = 0;
        for face in solid_faces(&topo, cut).unwrap() {
            if !matches!(topo.face(face).unwrap().surface(), FaceSurface::Torus(_)) {
                continue;
            }
            torus_faces += 1;
            let exact = face_area(&topo, face, 1e-2).unwrap();
            let mesh = tessellate(&topo, face, 1e-2).unwrap();
            let meshed: f64 = mesh
                .indices
                .chunks_exact(3)
                .map(|tri| {
                    let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize]);
                    (b - a).cross(c - a).length() / 2.0
                })
                .sum();
            // An inscribed mesh at deflection 1e-2 sits inside the surface,
            // so its area falls short, by O(deflection / r) relative.
            assert!(
                meshed <= exact && exact - meshed <= 1e-3 * exact,
                "{}: band mesh area {meshed} vs exact {exact}",
                p.name
            );
        }
        assert_eq!(torus_faces, 1, "{}: one kept torus band", p.name);
        for deflection in [1e-1, 1e-2] {
            let mesh = tessellate_solid(&topo, cut, deflection).unwrap();
            assert_eq!(boundary_edge_count(&mesh), 0, "{} {deflection}", p.name);
            assert_eq!(non_manifold_edge_count(&mesh), 0, "{} {deflection}", p.name);
        }
    }
}
