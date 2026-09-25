//! `solid_volume` must never return a leaky-route volume as `Ok` when the
//! whole-solid mesh a body was routed to is open.
//!
//! Several body classes are measured on their closed whole-solid mesh because
//! the per-face routes after that point mis-measure them: a torus notch band,
//! a quadric wall trimmed by a NURBS intersection curve, a non-latitude sphere
//! patch, a scalloped sphere collar. When that mesh came out open,
//! `solid_volume` fell through to `volume_from_direct_face_tessellation`, which
//! integrates a trimmed torus over its analytic bounding rectangle, and
//! returned the result as `Ok`.
//!
//! **Witness.** Roadmap row B51's torus–cone slow-lattice draws #5 and #19
//! (the deterministic B26 sweep in the B39 work): exact, fully valid fuses and
//! cuts whose whole-solid mesh has 6 open edges at every deflection from the
//! clamp down. Traced on `main` at `2a735434` (with this change's
//! `BK_VOL_TRACE` output patched in; main's own trace never emits): the torus
//! notch band declines its band integral on the cone wall, its mesh is open,
//! the direct gate (NURBS-trimmed quadric wall) finds the same open mesh, and
//! the direct face route returned, as `Ok`:
//!
//! | body          | oracle | returned | error |
//! |---------------|--------|----------|-------|
//! | draw #5 fuse  | 55.005 | 34.032   | −38 % |
//! | draw #5 cut   | 27.247 | 34.032   | +25 % |
//! | draw #19 fuse | 69.707 | 30.623   | −56 % |
//! | draw #19 cut  | 25.294 | 30.623   | +21 % |
//!
//! The same route read the pre-B39 torus–cone oblique fuse as 5.43 against
//! 18.775 at deflections 2e-4 and 1e-4, where its mesh opened, while its
//! 5.3e-4 clamp mesh was closed.
//!
//! **Contract asserted.** At every deflection, `solid_volume` returns a volume
//! within [`REL_TOL`] of an independent oracle, or a typed
//! `OperationsError::Unsupported` from `solid_volume`. These meshes stay open
//! at the clamp retry too, and every face is one the boundary-trimmed Gauss
//! fallback measures, so today the answer is that integral: within 1e-4 of
//! the oracle (measured 8e-6 – 7e-5). Should B51's mesh close, the closed
//! mesh measures them and the contract still holds. The open-mesh order
//! itself (exact integral, then the clamp retry, then a typed refusal) is
//! pinned geometry-independently by the unit tests in `measure/volume.rs`.
//!
//! **Oracle.** torus + frustum − I (fuse) or frustum − I (cut), where I is the
//! torus ∩ frustum volume by the midpoint rule over the frustum's local
//! `(z, θ)` with EXACT radial intervals from the line–torus quartic (the B39
//! regression's integrator, generalised to any frustum placement). As a
//! premise, the B-Rep's own boundary-trimmed Gauss integral
//! (`mass_properties`) agrees with it, so the body is right and only the
//! measurement route is at fault.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_4, PI, TAU};

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_cone, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// An `Ok` reading must be this close to the oracle. A gross-error detector:
/// the fall-through readings above are off by 21–56 %.
const REL_TOL: f64 = 2e-3;

/// The Gauss premise: the B-Rep measures its oracle to this (measured
/// 8e-6 – 7e-5 on these four bodies).
const GAUSS_TOL: f64 = 2e-4;

/// One B51 sweep draw: frustum `a` at the origin, torus `b` placed by
/// `T(offset) · Rz(angle)`, as `check_bool_pair` builds it at scale 1.
#[derive(Clone, Copy, Debug)]
struct Draw {
    name: &'static str,
    /// Frustum base radius, top radius, height.
    cone: (f64, f64, f64),
    /// Torus major and minor radius.
    torus: (f64, f64),
    angle: f64,
    offset: (f64, f64, f64),
}

const DRAW_5: Draw = Draw {
    name: "B51 draw #5",
    cone: (3.0, 1.0, 2.5),
    torus: (2.5, 0.75),
    angle: FRAC_PI_4,
    offset: (-1.0, 1.5, 1.5),
};

const DRAW_19: Draw = Draw {
    name: "B51 draw #19",
    cone: (1.0, 2.5, 3.0),
    torus: (4.0, 0.75),
    angle: 0.0,
    offset: (3.0, 4.0, 2.0),
};

impl Draw {
    fn placement(self) -> Mat4 {
        Mat4::translation(self.offset.0, self.offset.1, self.offset.2)
            * Mat4::rotation_z(self.angle)
    }

    fn torus_volume(self) -> f64 {
        let (major, minor) = self.torus;
        2.0 * PI * PI * major * minor * minor
    }

    fn frustum_volume(self) -> f64 {
        let (rb, rt, h) = self.cone;
        PI * h * rt.mul_add(rt, rb.mul_add(rb, rb * rt)) / 3.0
    }

    /// Torus ∩ frustum, with the torus moved back to the origin (so the
    /// frustum sits at the inverse placement): midpoint rule over the
    /// frustum's local `(z, θ)`, exact line–torus radial intervals along each
    /// ray from the axis.
    fn intersection_volume(self) -> f64 {
        let (major, minor) = self.torus;
        let (rb, rt, h) = self.cone;
        let frame = self.placement().inverse().unwrap();
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), major, minor).unwrap();
        let inside = |p: Point3| {
            let radial = p.x().hypot(p.y()) - major;
            radial.mul_add(radial, p.z() * p.z()) < minor * minor
        };
        let (nz, nt) = (400_u32, 800_u32);
        let dz = h / f64::from(nz);
        let dt = TAU / f64::from(nt);
        let mut volume = 0.0;
        for iz in 0..nz {
            let z = (f64::from(iz) + 0.5) * dz;
            let r = (rt - rb).mul_add(z / h, rb);
            let origin_local = Point3::new(0.0, 0.0, z);
            let origin = frame.mul_point(origin_local);
            for it in 0..nt {
                let theta = (f64::from(it) + 0.5) * dt;
                let dir = frame.mul_point(origin_local + Vec3::new(theta.cos(), theta.sin(), 0.0))
                    - origin;
                let mut ts: Vec<f64> =
                    remus_math::analytic_intersection::intersect_line_torus(&torus, origin, dir)
                        .into_iter()
                        .filter(|&t| t > 0.0 && t < r)
                        .collect();
                ts.extend([0.0, r]);
                ts.sort_by(f64::total_cmp);
                for w in ts.windows(2) {
                    if inside(origin + dir * f64::midpoint(w[0], w[1])) {
                        volume += 0.5 * w[1].mul_add(w[1], -w[0] * w[0]) * dt * dz;
                    }
                }
            }
        }
        volume
    }

    fn expected(self, op: BooleanOp, intersection: f64) -> f64 {
        match op {
            BooleanOp::Fuse => self.frustum_volume() + self.torus_volume() - intersection,
            BooleanOp::Cut => self.frustum_volume() - intersection,
            BooleanOp::Intersect => intersection,
        }
    }

    fn exact(self, op: BooleanOp) -> (Topology, SolidId) {
        let (rb, rt, h) = self.cone;
        let (major, minor) = self.torus;
        let mut topo = Topology::new();
        let cone = make_cone(&mut topo, rb, rt, h).unwrap();
        let torus = make_torus(&mut topo, major, minor, 16).unwrap();
        transform_solid(&mut topo, torus, &self.placement()).unwrap();
        let outcome = boolean_with_context(
            &mut topo,
            op,
            cone,
            torus,
            &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
        )
        .unwrap_or_else(|e| panic!("{} {op:?}: exact boolean refused: {e:?}", self.name));
        assert!(
            matches!(outcome.quality, BooleanQuality::Exact),
            "{} {op:?}: ExactOnly returned {:?}",
            self.name,
            outcome.quality
        );
        (topo, outcome.solid)
    }
}

fn rel_err(a: f64, b: f64) -> f64 {
    (a - b).abs() / a.abs().max(b.abs())
}

/// The contract on one draw and op, at a clamped coarse request and at two
/// requests finer than the clamp (the second is the B26 harness's
/// `diag · 1e-5`).
fn assert_never_a_leaky_ok(draw: Draw, op: BooleanOp) {
    let (topo, solid) = draw.exact(op);
    let expected = draw.expected(op, draw.intersection_volume());
    let what = format!("{} {op:?}", draw.name);

    let gauss = mass_properties(&topo, solid).unwrap().mass.abs();
    assert!(
        rel_err(gauss, expected) <= GAUSS_TOL,
        "{what}: premise: the B-Rep's Gauss volume {gauss} must match the oracle {expected}"
    );

    let aabb = solid_bounding_box(&topo, solid).unwrap();
    let diag = (aabb.max - aabb.min).length();
    for deflection in [0.1, diag * 2e-5, diag * 1e-5] {
        match solid_volume(&topo, solid, deflection) {
            Ok(volume) => assert!(
                rel_err(volume, expected) <= REL_TOL,
                "{what} at {deflection:e}: solid_volume returned Ok({volume}) against the \
                 oracle {expected} (Gauss {gauss}): a leaky-route reading"
            ),
            Err(OperationsError::Unsupported { operation, reason }) => assert_eq!(
                operation, "solid_volume",
                "{what} at {deflection:e}: refusal from the wrong operation: {reason}"
            ),
            Err(other) => panic!("{what} at {deflection:e}: untyped failure {other:?}"),
        }
    }
}

#[test]
fn b51_draw5_fuse_volume_is_never_a_leaky_ok() {
    assert_never_a_leaky_ok(DRAW_5, BooleanOp::Fuse);
}

#[test]
fn b51_draw5_cut_volume_is_never_a_leaky_ok() {
    assert_never_a_leaky_ok(DRAW_5, BooleanOp::Cut);
}

#[test]
fn b51_draw19_fuse_volume_is_never_a_leaky_ok() {
    assert_never_a_leaky_ok(DRAW_19, BooleanOp::Fuse);
}

#[test]
fn b51_draw19_cut_volume_is_never_a_leaky_ok() {
    assert_never_a_leaky_ok(DRAW_19, BooleanOp::Cut);
}
