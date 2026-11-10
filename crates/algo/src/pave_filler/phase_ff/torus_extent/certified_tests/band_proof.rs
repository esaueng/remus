//! F2: a torus band's provenance, not its width, decides the whole-tube
//! paths.
//!
//! `(0, 2π)` is the marcher band of three different faces: a proved whole
//! tube, a certified disc whose lifted band straddles the projection's
//! `0 = 2π` wrap, and a face nothing could be certified about. Only the
//! first may skip the extent's `v` test and depth margin, or land marched
//! exits exactly on a partner wall's rim. Each test runs through the band
//! phase FF builds (`face_v_band`), the extent it builds from that band
//! (`FaceExtent::from_band`), and, for the marcher, the raw-curve dispatch
//! the pair loop calls (`compute_raw_curves_banded`).

use super::*;
use crate::pave_filler::phase_ff as ff;
use ff::{BandProof, FaceBand, FaceExtent, Strict};
use remus_math::analytic_intersection::BandCoverage;
use remus_math::surfaces::ConicalSurface;

fn band_of(topo: &Topology, face: FaceId) -> FaceBand {
    let surface = topo.face(face).unwrap().surface().clone();
    ff::face_v_band(topo, face, &surface, Tolerance::default()).unwrap()
}

fn extent_of(topo: &Topology, face: FaceId, band: &FaceBand) -> FaceExtent {
    let surface = topo.face(face).unwrap().surface().clone();
    FaceExtent::from_band(topo, face, &surface, band, Tolerance::default())
        .unwrap()
        .unwrap()
}

/// Declare `tolerance` on every edge of the face's outer wire, as an
/// imported boundary whose rim sits that far off its stored curve would.
fn declare_edge_tolerance(topo: &mut Topology, face: FaceId, tolerance: f64) {
    let wire = topo.face(face).unwrap().outer_wire();
    let edges: Vec<_> = topo
        .wire(wire)
        .unwrap()
        .edges()
        .iter()
        .map(OrientedEdge::edge)
        .collect();
    for edge in edges {
        topo.edge_mut(edge)
            .unwrap()
            .set_tolerance(Some(tolerance))
            .unwrap();
    }
}

#[test]
fn wrap_ending_fillet_with_its_rim_past_the_wrap_certifies_no_off_patch_point() {
    // The fillet patch u in [pi/2, pi], v in [-pi/2, dv]: its rim lies dv
    // past the equator, i.e. r*dv off it (2e-7 .. 1e-5 here). With the
    // band's width read as a proof, every such rim made the patch a whole
    // tube: the off-patch parallel v = 3.48 and the inner equator v = pi
    // were then certified inside, strictly and at depth (the reviewed
    // probe). A rim offset within the declared boundary tolerance snaps the
    // marcher band onto the wrap; one beyond it keeps a straddling band.
    // Either way the extent keeps the lifted hull and the band proves no
    // coverage.
    let f = Frame::tilted();
    let torus = f.torus();
    let (u_in, u_out) = (0.75 * PI, 1.5 * PI);
    for dv in [5e-8, 1e-7, 1e-6, 5e-6] {
        let offset = RM * dv;
        for declared in [None, Some((1.5 * offset).max(1e-7))] {
            let (mut topo, face) = rectangle_face(&f, (PI / 2.0, PI), (-PI / 2.0, dv), false);
            if let Some(t) = declared {
                declare_edge_tolerance(&mut topo, face, t);
            }
            let case = format!("dv {dv:e}, declared {declared:?}");
            let band = band_of(&topo, face);
            assert_eq!(band.proof, BandProof::Bounded, "{case}: {band:?}");
            assert_eq!(band.march_hint().coverage, BandCoverage::Bounded, "{case}");
            let (m0, m1) = band.range.unwrap();
            if declared.is_some() {
                // Within the declared tolerance of the wrap: clamped onto it.
                assert!(
                    (m0 - 1.5 * PI).abs() < 1e-9 && m1 == TAU,
                    "{case}: {m0} {m1}"
                );
            } else if offset > 2e-7 {
                // A genuine straddle past the default tolerance: the whole
                // period as a bound, not the clamped (and too narrow) band.
                assert_eq!((m0, m1), (0.0, TAU), "{case}");
            }
            let extent = extent_of(&topo, face, &band);
            let off_patch = torus.evaluate(u_in, 3.48);
            let inner_equator = torus.evaluate(u_in, PI);
            assert!(!extent.contains(off_patch), "{case}: v = 3.48 admitted");
            assert!(!extent.contains_strict(off_patch, 0.0), "{case}");
            assert!(!extent.contains(inner_equator), "{case}: v = pi admitted");
            assert!(!extent.contains_strict(inner_equator, 1e-5), "{case}");
            assert_eq!(
                extent.strict_membership(inner_equator, 1e-5),
                Strict::Outside
            );
            // The patch itself, on both sides of the wrap and on its rim.
            for v in [5.94, TAU - 0.3, -0.3, 0.5 * dv, dv] {
                assert!(extent.contains(torus.evaluate(u_in, v)), "{case}: v = {v}");
            }
            assert!(
                extent.contains_strict(torus.evaluate(u_in, 5.94), 1e-5),
                "{case}"
            );
            assert!(
                !extent.contains_strict(torus.evaluate(u_out, 5.94), 0.0),
                "{case}"
            );
            let (lo, hi) = band.extent.unwrap();
            assert!(
                (hi - lo - (PI / 2.0 + dv)).abs() < 1e-9,
                "{case}: extent {lo} {hi} is not the lifted hull"
            );
        }
    }
}

#[test]
fn straddling_rectangle_rejects_the_inner_equator_and_keeps_both_sides_of_the_wrap() {
    let f = Frame::tilted();
    let torus = f.torus();
    let (u, v) = ((0.2, 0.6), (-0.3, 0.4));
    let (topo, face) = rectangle_face(&f, u, v, false);
    let band = band_of(&topo, face);
    // The marcher still sweeps the whole period (no single [v0, v1] in
    // [0, 2pi) holds the face), but as a bound only.
    assert_eq!(band.range, Some((0.0, TAU)));
    assert_eq!(band.proof, BandProof::Bounded);
    assert_eq!(band.march_hint().coverage, BandCoverage::Bounded);
    let extent = extent_of(&topo, face, &band);
    let um = 0.4;
    for depth in [0.0, 1e-5] {
        assert!(
            !extent.contains_strict(torus.evaluate(um, PI), depth),
            "inner equator certified at depth {depth}"
        );
        assert!(!extent.contains_strict(torus.evaluate(um, v.1 + 0.5), depth));
    }
    assert!(!extent.contains(torus.evaluate(um, PI)));
    assert!(!extent.contains(torus.evaluate(um, v.0 - 0.5)));
    // Real points on both sides of the wrap (projected near 2pi and near 0)
    // and on it.
    for w in [-0.2, -1e-9, 0.0, 1e-9, 0.3] {
        let p = torus.evaluate(um, w);
        assert!(extent.contains(p), "v = {w}");
        assert!(extent.contains_strict(p, 1e-5), "v = {w}");
    }
    // Within the depth of the true rim is not strictly inside.
    assert!(!extent.contains_strict(torus.evaluate(um, v.1 - 1e-6), 1e-5));
    assert!(!extent.contains_strict(torus.evaluate(um, v.0 + 1e-6), 1e-5));
    let (lo, hi) = band.extent.unwrap();
    assert!((hi - lo - 0.7).abs() < 1e-9, "{lo} {hi}");
}

#[test]
fn uncertain_faces_never_arm_the_whole_tube_paths() {
    let f = Frame::axis_z();
    let torus = f.torus();
    // A loop wound clockwise (a complement or an undecided orientation).
    let (cw_topo, cw_face) = clockwise_rectangle_face(&f, (0.2, 0.6), (0.3, 0.9));
    // A whole parallel: a winding in u only.
    let (par_topo, par_face) = build_face(
        &torus,
        vec![Spec::New(Side {
            curve: EdgeCurve::Circle(parallel(&f, 0.8)),
            from: 0.0,
            to: TAU,
        })],
        false,
    );
    // An unsupported (ellipse) side.
    let ellipse = Ellipse3D::new(
        Point3::new(0.0, 0.0, RM * 1.3_f64.sin()),
        Vec3::new(0.0, 0.0, 1.0),
        RM.mul_add(1.3_f64.cos(), R),
        RM.mul_add(1.3_f64.cos(), R),
    )
    .unwrap();
    let (ell_topo, ell_face) = build_face(
        &torus,
        vec![
            Spec::New(Side {
                curve: EdgeCurve::Circle(parallel(&f, 0.3)),
                from: 0.2,
                to: 0.6,
            }),
            Spec::New(Side {
                curve: EdgeCurve::Circle(meridian(&f, 0.6)),
                from: 0.3,
                to: 1.3,
            }),
            Spec::New(Side {
                curve: EdgeCurve::Ellipse(ellipse),
                from: 0.6,
                to: 0.2,
            }),
            Spec::New(Side {
                curve: EdgeCurve::Circle(meridian(&f, 0.2)),
                from: 1.3,
                to: 0.3,
            }),
        ],
        false,
    );
    for (name, topo, face) in [
        ("clockwise", &cw_topo, cw_face),
        ("u winding", &par_topo, par_face),
        ("ellipse side", &ell_topo, ell_face),
    ] {
        let band = band_of(topo, face);
        assert_eq!(band.range, Some((0.0, TAU)), "{name}");
        assert_eq!(band.proof, BandProof::Uncertain, "{name}");
        assert_eq!(band.march_hint().coverage, BandCoverage::Bounded, "{name}");
        let extent = extent_of(topo, face, &band);
        for v in [0.0, 0.5, PI, 5.0] {
            let p = torus.evaluate(0.4, v);
            // Never an inside certificate, at any depth.
            assert!(!extent.contains_strict(p, 0.0), "{name}: v = {v}");
            assert!(!extent.contains_strict(p, 1e-5), "{name}: v = {v}");
            assert_eq!(extent.strict_membership(p, 0.0), Strict::Unknown, "{name}");
            // Nor a rejection: section windows and the corner rescue defer
            // to the lenient test, which keeps every point of the carrier.
            assert!(extent.contains(p), "{name}: v = {v}");
            assert!(extent.strict_or_deferred(p, 1e-5), "{name}: v = {v}");
        }
    }
}

#[test]
fn complete_minor_circle_boundaries_still_prove_the_whole_tube() {
    let f = Frame::tilted();
    let torus = f.torus();
    // A whole meridian winds once in v.
    let (topo, face) = build_face(
        &torus,
        vec![Spec::New(Side {
            curve: EdgeCurve::Circle(meridian(&f, 0.8)),
            from: 0.0,
            to: TAU,
        })],
        false,
    );
    // A swept tube segment: two minor circles joined by a doubled seam.
    let (u0, u1) = (0.3, 1.4);
    let (seg_topo, seg_face) = build_face(
        &torus,
        vec![
            Spec::New(Side {
                curve: EdgeCurve::Circle(parallel(&f, 0.0)),
                from: u0,
                to: u1,
            }),
            Spec::New(Side {
                curve: EdgeCurve::Circle(meridian(&f, u1)),
                from: 0.0,
                to: TAU,
            }),
            Spec::Reuse(0, false),
            Spec::New(Side {
                curve: EdgeCurve::Circle(meridian(&f, u0)),
                from: TAU,
                to: 0.0,
            }),
        ],
        false,
    );
    for (name, topo, face) in [
        ("meridian", &topo, face),
        ("tube segment", &seg_topo, seg_face),
    ] {
        let band = band_of(topo, face);
        assert_eq!(band.range, Some((0.0, TAU)), "{name}");
        assert_eq!(band.proof, BandProof::WholeTube, "{name}");
        assert_eq!(
            band.march_hint().coverage,
            BandCoverage::WholeTube,
            "{name}"
        );
        let extent = extent_of(topo, face, &band);
        // No depth margin at the [0, 2pi] parallel: it is not a boundary.
        let u = if name == "meridian" { 0.8 } else { 0.9 };
        for v in [0.0, 1e-9, TAU - 1e-9, PI] {
            assert!(
                extent.contains_strict(torus.evaluate(u, v), 1e-5),
                "{name}: v = {v}"
            );
        }
    }
}

/// The B39 unit cell (R = 3, r = 0.5 torus about z; a frustum wall between
/// its rims), with torus faces whose marcher bands are all `(0, 2π)`.
mod marcher {
    use super::*;
    use remus_math::context::OperationContext;

    const BR: f64 = 3.0;
    const BRM: f64 = 0.5;

    fn b39_torus() -> ToroidalSurface {
        ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), BR, BRM).unwrap()
    }

    fn b39_cone() -> (ConicalSurface, (f64, f64)) {
        let cone = ConicalSurface::new(
            Point3::new(-1.5, 2.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            2.0_f64.atan2(1.0),
        )
        .unwrap();
        (cone, (5.0_f64.sqrt(), 1.5 * 5.0_f64.sqrt()))
    }

    fn b39_meridian(u: f64) -> Circle3D {
        let radial = Vec3::new(u.cos(), u.sin(), 0.0);
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0) + radial * BR,
            radial.cross(Vec3::new(0.0, 0.0, 1.0)),
            BRM,
            radial,
        )
        .unwrap()
    }

    fn b39_parallel(v: f64) -> Circle3D {
        Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, BRM * v.sin()),
            Vec3::new(0.0, 0.0, 1.0),
            BRM.mul_add(v.cos(), BR),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap()
    }

    fn b39_rectangle(
        (u0, u1): (f64, f64),
        (v0, v1): (f64, f64),
        clockwise: bool,
    ) -> (Topology, FaceId) {
        let side = |curve: Circle3D, from, to| {
            Spec::New(Side {
                curve: EdgeCurve::Circle(curve),
                from,
                to,
            })
        };
        let specs = if clockwise {
            vec![
                side(b39_meridian(u0), v0, v1),
                side(b39_parallel(v1), u0, u1),
                side(b39_meridian(u1), v1, v0),
                side(b39_parallel(v0), u1, u0),
            ]
        } else {
            vec![
                side(b39_parallel(v0), u0, u1),
                side(b39_meridian(u1), v0, v1),
                side(b39_parallel(v1), u1, u0),
                side(b39_meridian(u0), v1, v0),
            ]
        };
        build_face(&b39_torus(), specs, false)
    }

    /// The largest distance of a raw curve end from the cone band's rims,
    /// over every curve, and the smallest.
    fn rim_offsets(band: &FaceBand) -> (f64, f64) {
        let (cone, wall) = b39_cone();
        let torus = FaceSurface::Torus(b39_torus());
        let cone_surface = FaceSurface::Cone(cone.clone());
        let bbox = Aabb3 {
            min: Point3::new(-10.0, -10.0, -10.0),
            max: Point3::new(10.0, 10.0, 10.0),
        };
        let curves = ff::compute_raw_curves_banded(
            &torus,
            &cone_surface,
            &bbox,
            &bbox,
            band,
            &FaceBand::bounded(Some(wall)),
            &OperationContext::default(),
        )
        .unwrap();
        assert!(!curves.is_empty());
        let (mut lo, mut hi) = (f64::INFINITY, 0.0_f64);
        for raw in &curves {
            for p in [raw.p_start, raw.p_end] {
                let (_, v) = cone.project_point(p);
                let off = (v - wall.0).abs().min((v - wall.1).abs());
                lo = lo.min(off);
                hi = hi.max(off);
            }
        }
        (lo, hi)
    }

    #[test]
    fn only_a_proved_whole_tube_lands_exits_on_the_wall_rim() {
        // Genuine whole tube: one full meridian (a v winding).
        let (topo, face) = build_face(
            &b39_torus(),
            vec![Spec::New(Side {
                curve: EdgeCurve::Circle(b39_meridian(0.4)),
                from: 0.0,
                to: TAU,
            })],
            false,
        );
        let whole = band_of(&topo, face);
        assert_eq!(whole.proof, BandProof::WholeTube);
        let (_, worst) = rim_offsets(&whole);
        assert!(worst < 1e-9, "whole-tube exits {worst:e} off the rim");

        // Same (0, 2pi) marcher band, no proof: a patch straddling the wrap,
        // a patch whose rim lies 1e-6 past it, and an uncertain (clockwise)
        // loop. Each keeps the step-short ends every partial patch keeps.
        let partial = [
            ("straddle", b39_rectangle((0.2, 0.6), (-0.3, 0.4), false)),
            (
                "rim past wrap",
                b39_rectangle((0.2, 0.6), (-PI / 2.0, 1e-6), false),
            ),
            ("uncertain", b39_rectangle((0.2, 0.6), (0.3, 0.9), true)),
        ];
        for (name, (topo, face)) in partial {
            let band = band_of(&topo, face);
            assert_eq!(band.range, Some((0.0, TAU)), "{name}");
            assert_ne!(band.proof, BandProof::WholeTube, "{name}");
            let (nearest, _) = rim_offsets(&band);
            assert!(
                nearest > 1e-6,
                "{name}: a partial patch's exit was moved onto the rim ({nearest:e})"
            );
        }
    }
}
