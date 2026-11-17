//! Wrap a planar profile onto a cylinder, preserving arc length, as a solid
//! shell of material between two radii (B73; OpenZCAD M12 emboss/engrave).
//!
//! The profile lies in a plane parallel to the cylinder's axis. A profile
//! point `p` has an arc-length coordinate `s = (p − O)·b` and an axial one
//! `z = (p − O)·a`, with `a` the unit axis, `b = a × n` the plane's own
//! direction across the axis (`n` its normal), `e` the unit reference
//! direction across the axis (angle zero) and `c = a × e`. Wrapped at
//! radius `r`:
//!
//! ```text
//! W_r(s, z) = O + z·a + r·(cos(s/R)·e + sin(s/R)·c)
//! ```
//!
//! `R` is the wrap radius, so arc length along the cylinder of radius `R`
//! equals the profile's length along `b`. The solid is every `W_r(s, z)` with
//! `(s, z)` in the profile and `r` between the inner and outer radii:
//!
//! - its inner and outer faces are exact cylinders;
//! - a profile edge along the axis wraps to a straight edge and walls a
//!   radial plane; one across the axis wraps to an exact circular arc and
//!   walls a plane square to the axis;
//! - every other edge (an oblique line wraps to a helix, an arc or spline to
//!   a curve with no rational form) wraps to a **certified** cubic B-spline:
//!   the fit's distance from the exact wrapped curve is proven from a bound
//!   on the curve's fourth derivative ([`fit_cubic_hermite`]), held within
//!   the wrap tolerance, and its wall is the exact ruled surface between the
//!   inner and outer fits, every ruling radial.
//!
//! [`wrap_profile_on_cone`] wraps onto a cone the same way, through the
//! cone's development (see its module).
//!
//! Lines, circles, ellipses and non-rational B-splines are wrapped; other
//! rational curves, a profile plane that is not parallel to the axis, a
//! profile that spans a full turn, and a degenerate radius range are refused
//! by name. Every refusal leaves the topology unchanged.

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::hermite_fit::fit_cubic_hermite;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::CylindricalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;
use crate::helix::in_place::{Loop, Piece, read_profile_loop};

/// The cylinder a profile is wrapped onto.
#[derive(Debug, Clone, Copy)]
pub struct WrapCylinder {
    /// A point on the axis.
    pub origin: Point3,
    /// The axis direction.
    pub axis: Vec3,
    /// The direction across the axis at which arc length zero lies.
    pub reference: Vec3,
    /// The radius arc length is measured at.
    pub radius: f64,
}

/// Target for a fitted edge's certified distance from its exact wrapped
/// curve, relative to the wrap's scale (never below the linear tolerance).
const RELATIVE_FIT_TOLERANCE: f64 = 1e-7;

/// Most cubic Hermite spans one wrapped edge may take.
const MAX_FIT_SPANS: usize = 4096;

/// Samples per profile piece for the angular-extent check.
const EXTENT_SAMPLES: usize = 64;

/// Directions closer than this (as `1 − |cos|`) count as parallel.
const PARALLEL_TOLERANCE: f64 = 1e-9;

fn invalid(reason: impl Into<String>) -> OperationsError {
    OperationsError::InvalidInput {
        reason: reason.into(),
    }
}

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: "cylinder wrap",
        reason: reason.into(),
    }
}

/// The wrap's frame and its map from the profile plane to the cylinder.
#[derive(Debug, Clone, Copy)]
struct Frame {
    origin: Point3,
    a: Vec3,
    e: Vec3,
    c: Vec3,
    /// The profile plane's own arc-length direction, `axis × plane normal`:
    /// `s` is measured along it, so turning `reference` moves the wrap round
    /// the axis without changing its lengths.
    across: Vec3,
    radius: f64,
}

/// A map from a profile's plane to its unrolled `(s, z)` coordinates: `s`
/// across, along the arc-length direction, `z` along the surface.
trait Unroll {
    /// `(s, z)` of a profile point.
    fn unroll(&self, p: Point3) -> Planar;
    /// `(s, z)` of a profile direction.
    fn unroll_vec(&self, v: Vec3) -> Planar;
}

impl Unroll for Frame {
    fn unroll(&self, p: Point3) -> Planar {
        let d = p - self.origin;
        (d.dot(self.across), d.dot(self.a))
    }

    fn unroll_vec(&self, v: Vec3) -> Planar {
        (v.dot(self.across), v.dot(self.a))
    }
}

impl Frame {
    fn radial(&self, theta: f64) -> Vec3 {
        let (sin, cos) = theta.sin_cos();
        self.e * cos + self.c * sin
    }

    /// `W_r(s, z)`.
    fn wrap(&self, (s, z): (f64, f64), r: f64) -> Point3 {
        self.origin + self.a * z + self.radial(s / self.radius) * r
    }

    /// `W_r` and its derivative along a curve with `(s, z)` and `(s', z')`.
    fn wrap_with_derivative(&self, q: (f64, f64), dq: (f64, f64), r: f64) -> (Point3, Vec3) {
        let theta = q.0 / self.radius;
        let (sin, cos) = theta.sin_cos();
        let across = self.c * cos - self.e * sin;
        (
            self.wrap(q, r),
            self.a * dq.1 + across * (r * dq.0 / self.radius),
        )
    }

    /// `p` with its part across the axis scaled by `k` (a linear map, so it
    /// carries a B-spline exactly).
    fn scale_radially(&self, p: Point3, k: f64) -> Point3 {
        let d = p - self.origin;
        let along = self.a * d.dot(self.a);
        self.origin + along + (d - along) * k
    }
}

/// One profile piece as a curve in `(s, z)` over `x ∈ [0, 1]`, in its
/// natural direction, with bounds on its derivatives of order 1 to 4.
/// A point `(s, z)` of the unrolled plane, or a derivative of one.
type Planar = (f64, f64);

/// The wrapped curve of one ring, its parameter range and its circle angle.
type RingEdge = (EdgeCurve, Option<(f64, f64)>, Option<f64>);

struct Unrolled<'a> {
    piece: &'a Piece,
    /// `(q(x), q'(x))`.
    sample: Box<dyn Fn(f64) -> (Planar, Planar) + 'a>,
    /// `bounds[k-1] ≥ sup |s^(k)|, sup |z^(k)|` over `[0, 1]`.
    bounds: [f64; 4],
}

/// Bounds on the derivatives of a non-rational B-spline's `(s, z)` image,
/// from its hodographs (each derivative curve lies in the convex hull of its
/// control points), reparameterized onto `[0, 1]`.
fn bspline_bounds(curve: &NurbsCurve, frame: &impl Unroll) -> [f64; 4] {
    let degree = curve.degree();
    let knots = curve.knots();
    let (u0, u1) = curve.domain();
    let width = u1 - u0;
    let mut points: Vec<(f64, f64)> = curve
        .control_points()
        .iter()
        .map(|&p| frame.unroll(p))
        .collect();
    let mut bounds = [0.0; 4];
    for (k, bound) in bounds.iter_mut().enumerate() {
        let p = degree.saturating_sub(k);
        if p == 0 {
            break;
        }
        #[allow(clippy::cast_precision_loss)]
        let next: Vec<(f64, f64)> = (0..points.len().saturating_sub(1))
            .map(|i| {
                let span = knots[i + k + p + 1] - knots[i + k + 1];
                if span <= 0.0 {
                    (0.0, 0.0)
                } else {
                    let f = p as f64 / span;
                    (
                        f * (points[i + 1].0 - points[i].0),
                        f * (points[i + 1].1 - points[i].1),
                    )
                }
            })
            .collect();
        let largest = next
            .iter()
            .map(|&(s, z)| s.abs().max(z.abs()))
            .fold(0.0, f64::max);
        #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
        {
            *bound = largest * width.powi(k as i32 + 1);
        }
        points = next;
    }
    bounds
}

fn unroll_piece<'a, U: Unroll>(
    piece: &'a Piece,
    frame: &'a U,
) -> Result<Unrolled<'a>, OperationsError> {
    match &piece.cap {
        EdgeCurve::Line => {
            let (u0, u1) = piece.nurbs.domain();
            let q0 = frame.unroll(piece.nurbs.evaluate(u0));
            let q1 = frame.unroll(piece.nurbs.evaluate(u1));
            let d = (q1.0 - q0.0, q1.1 - q0.1);
            Ok(Unrolled {
                piece,
                sample: Box::new(move |x| ((q0.0 + d.0 * x, q0.1 + d.1 * x), d)),
                bounds: [d.0.abs().max(d.1.abs()), 0.0, 0.0, 0.0],
            })
        }
        EdgeCurve::Circle(circle) => {
            let (a, b) = piece.trim.unwrap_or((0.0, std::f64::consts::TAU));
            let w = b - a;
            let rho = circle.radius();
            Ok(Unrolled {
                piece,
                sample: Box::new(move |x| {
                    let t = a + w * x;
                    let p = circle.evaluate(t);
                    let dp = circle.tangent(t) * (rho * w);
                    (frame.unroll(p), frame.unroll_vec(dp))
                }),
                bounds: [1, 2, 3, 4].map(|k| rho * w.abs().powi(k)),
            })
        }
        EdgeCurve::Ellipse(ellipse) => {
            let (a, b) = piece.trim.unwrap_or((0.0, std::f64::consts::TAU));
            let w = b - a;
            let (major, minor) = (ellipse.semi_major(), ellipse.semi_minor());
            let (u, v, center) = (ellipse.u_axis(), ellipse.v_axis(), ellipse.center());
            Ok(Unrolled {
                piece,
                sample: Box::new(move |x| {
                    let (sin, cos) = (a + w * x).sin_cos();
                    let p = center + u * (major * cos) + v * (minor * sin);
                    let dp = (v * (minor * cos) - u * (major * sin)) * w;
                    (frame.unroll(p), frame.unroll_vec(dp))
                }),
                bounds: [1, 2, 3, 4].map(|k| major.max(minor) * w.abs().powi(k)),
            })
        }
        EdgeCurve::NurbsCurve(_) if !piece.nurbs.is_rational() => {
            let curve = &piece.nurbs;
            let (u0, u1) = curve.domain();
            let w = u1 - u0;
            Ok(Unrolled {
                piece,
                sample: Box::new(move |x| {
                    let d = curve.derivatives(u0 + w * x, 1);
                    (
                        frame.unroll(Point3::new(d[0].x(), d[0].y(), d[0].z())),
                        frame.unroll_vec(d[1] * w),
                    )
                }),
                bounds: bspline_bounds(curve, frame),
            })
        }
        other @ (EdgeCurve::NurbsCurve(_) | EdgeCurve::Parabola(_) | EdgeCurve::Hyperbola(_)) => {
            Err(unsupported(format!(
                "a wrapped profile is made of lines, circles, ellipses and non-rational splines; \
             this one holds a {}",
                other.type_tag()
            )))
        }
    }
}

/// How one wrapped profile edge is represented.
enum WrappedEdge {
    /// Along the axis: straight edges and a radial plane.
    Axial,
    /// Across the axis at one height: circular arcs and a plane.
    Circumferential,
    /// Fitted: the unit-radius fit, scaled to each radius.
    Fitted { unit: NurbsCurve, bound: f64 },
}

/// Classify and, where needed, fit one piece.
fn wrap_piece(
    unrolled: &Unrolled<'_>,
    frame: &Frame,
    scale_tolerance: f64,
) -> Result<WrappedEdge, OperationsError> {
    let ((s0, z0), _) = (unrolled.sample)(0.0);
    let ((s1, z1), _) = (unrolled.sample)(1.0);
    if matches!(unrolled.piece.cap, EdgeCurve::Line) {
        let tiny = Tolerance::new().linear;
        if (s1 - s0).abs() <= tiny {
            return Ok(WrappedEdge::Axial);
        }
        if (z1 - z0).abs() <= tiny {
            return Ok(WrappedEdge::Circumferential);
        }
    }
    // `‖W_1''''‖ ≤ |z''''| + B4(g)` with `g = s / R` and the Faà di Bruno
    // bound for the unit point `(cos g, sin g)`.
    let [d1, d2, d3, d4] = unrolled.bounds;
    let r = frame.radius;
    let (g1, g2, g3, g4) = (d1 / r, d2 / r, d3 / r, d4 / r);
    let b4 = g4 + 4.0 * g1 * g3 + 3.0 * g2 * g2 + 6.0 * g1 * g1 * g2 + g1.powi(4);
    let bound = d4 + b4;
    let fit = fit_cubic_hermite(
        |x| {
            let (q, dq) = (unrolled.sample)(x);
            frame.wrap_with_derivative(q, dq, 1.0)
        },
        0.0,
        1.0,
        bound,
        scale_tolerance,
        MAX_FIT_SPANS,
    )
    .map_err(|e| unsupported(format!("a wrapped edge could not be fitted: {e}")))?;
    Ok(WrappedEdge::Fitted {
        unit: fit.curve,
        bound: fit.remainder_bound,
    })
}

/// The angles a loop's boundary reaches, for the extent check and the
/// cylinders' seam.
#[allow(clippy::cast_precision_loss)]
fn loop_angles(pieces: &[Unrolled<'_>], frame: &Frame) -> Vec<f64> {
    pieces
        .iter()
        .flat_map(|u| {
            (0..=EXTENT_SAMPLES).map(move |k| {
                let ((s, _), _) = (u.sample)(k as f64 / EXTENT_SAMPLES as f64);
                s / frame.radius
            })
        })
        .collect()
}

/// Wrap the planar face `profile` onto `cylinder` between `inner` and
/// `outer` radii.
///
/// See the [module docs](self). The profile's plane must be
/// parallel to the axis. Its arc-length coordinate runs along
/// `axis × plane normal` from `origin`'s foot in the plane, and
/// `reference` is where that coordinate's zero lands round the axis.
///
/// # Errors
///
/// [`OperationsError::InvalidInput`] for a non-finite or degenerate frame or
/// radius range, a profile plane not parallel to the axis, or a profile that
/// spans a full turn; [`OperationsError::Unsupported`] for a profile curve
/// that is not a line, circle, ellipse or non-rational spline, or one whose
/// wrap cannot be fitted within the tolerance. Every error leaves the
/// topology unchanged.
pub fn wrap_profile_solid(
    topo: &mut Topology,
    profile: FaceId,
    cylinder: &WrapCylinder,
    inner: f64,
    outer: f64,
) -> Result<SolidId, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        wrap_impl(topo, profile, cylinder, inner, outer)
    })
}

#[allow(clippy::too_many_lines)]
fn wrap_impl(
    topo: &mut Topology,
    profile: FaceId,
    cylinder: &WrapCylinder,
    inner: f64,
    outer: f64,
) -> Result<SolidId, OperationsError> {
    let linear = Tolerance::new().linear;
    if !(cylinder.radius.is_finite() && cylinder.radius > linear) {
        return Err(invalid("the wrap radius must be a positive length"));
    }
    if !(inner.is_finite() && outer.is_finite() && inner > linear && outer > inner + linear) {
        return Err(invalid(format!(
            "a wrap needs 0 < inner < outer radius, got {inner} and {outer}"
        )));
    }
    let a = cylinder.axis.normalize()?;
    let e = (cylinder.reference - a * cylinder.reference.dot(a)).normalize()?;
    let face = topo.face(profile)?;
    let FaceSurface::Plane { normal, d } = *face.surface() else {
        return Err(invalid("a wrapped profile must be planar"));
    };
    // The plane's normal turned to face away from the axis, so a region
    // lands under where it is drawn whichever way its sketch faces.
    let length = normal.length();
    let normal = normal.normalize()?;
    let normal = if d / length
        - normal.dot(Vec3::new(
            cylinder.origin.x(),
            cylinder.origin.y(),
            cylinder.origin.z(),
        ))
        < 0.0
    {
        -normal
    } else {
        normal
    };
    if normal.dot(a).abs() > PARALLEL_TOLERANCE {
        return Err(invalid(
            "a wrapped profile must lie in a plane parallel to the cylinder's axis",
        ));
    }
    let frame = Frame {
        origin: cylinder.origin,
        a,
        e,
        c: a.cross(e),
        across: a.cross(normal).normalize()?,
        radius: cylinder.radius,
    };
    let outer_wire = face.outer_wire();
    let inner_wires = face.inner_wires().to_vec();
    let mut loops: Vec<Loop> = vec![read_profile_loop(topo, outer_wire, true, "cylinder wrap")?];
    for wire in inner_wires {
        loops.push(read_profile_loop(topo, wire, false, "cylinder wrap")?);
    }

    let unrolled: Vec<Vec<Unrolled<'_>>> = loops
        .iter()
        .map(|lp| {
            lp.pieces
                .iter()
                .map(|piece| unroll_piece(piece, &frame))
                .collect::<Result<_, _>>()
        })
        .collect::<Result<_, _>>()?;

    // The profile must stay within one turn; the cylinders' seam goes in the
    // gap it leaves.
    let angles: Vec<f64> = unrolled
        .iter()
        .flat_map(|u| loop_angles(u, &frame))
        .collect();
    let (low, high) = angles
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &t| {
            (lo.min(t), hi.max(t))
        });
    let extent = high - low;
    if extent.is_nan() || extent >= std::f64::consts::TAU - 1e-6 {
        return Err(invalid(format!(
            "the wrapped profile spans {:.3}° around the cylinder; it must stay within one turn",
            extent.to_degrees()
        )));
    }
    let seam = low - 0.5 * (std::f64::consts::TAU - extent);
    let seam_dir = frame.radial(seam);

    let scale = outer.max(cylinder.radius).max(1.0);
    let fit_tolerance = (RELATIVE_FIT_TOLERANCE * scale).max(linear) / outer.max(1.0);
    let wrapped: Vec<Vec<WrappedEdge>> = unrolled
        .iter()
        .map(|pieces| {
            pieces
                .iter()
                .map(|u| wrap_piece(u, &frame, fit_tolerance))
                .collect::<Result<_, _>>()
        })
        .collect::<Result<_, _>>()?;

    let mut walls = Vec::new();
    let mut outer_wires: Vec<WireId> = Vec::new();
    let mut inner_loop_wires: Vec<WireId> = Vec::new();
    for (l, lp) in loops.iter().enumerate() {
        let points: Vec<(f64, f64)> = lp.points.iter().map(|&(p, _)| frame.unroll(p)).collect();
        // Winding in (s, z), which is right-handed about the outward radial.
        let samples: Vec<(f64, f64)> = unrolled[l]
            .iter()
            .flat_map(|u| {
                #[allow(clippy::cast_precision_loss)]
                (0..EXTENT_SAMPLES).map(move |k| {
                    let x = k as f64 / EXTENT_SAMPLES as f64;
                    (u.sample)(if u.piece.forward { x } else { 1.0 - x }).0
                })
            })
            .collect();
        let area: f64 = samples
            .iter()
            .zip(samples.iter().cycle().skip(1))
            .map(|(p, q)| p.0 * q.1 - q.0 * p.1)
            .sum();
        let ccw = area > 0.0;
        let side = if ccw == lp.outer { 1.0 } else { -1.0 };

        let bound_at = |i: usize| -> f64 {
            let mut worst: f64 = 0.0;
            for (j, piece) in wrapped[l].iter().enumerate() {
                let (ns, ne) = lp.natural_ends(j);
                if (ns == i || ne == i)
                    && let WrappedEdge::Fitted { bound, .. } = piece
                {
                    worst = worst.max(*bound * outer.max(1.0));
                }
            }
            worst
        };
        let vertex = |topo: &mut Topology, i: usize, r: f64| {
            let tol = lp.points[i].1.max(linear).max(2.0 * bound_at(i));
            topo.add_vertex(Vertex::new(frame.wrap(points[i], r), tol))
        };
        let inner_v: Vec<VertexId> = (0..points.len()).map(|i| vertex(topo, i, inner)).collect();
        let outer_v: Vec<VertexId> = (0..points.len()).map(|i| vertex(topo, i, outer)).collect();
        let radial: Vec<EdgeId> = (0..points.len())
            .map(|i| topo.add_edge(Edge::new(inner_v[i], outer_v[i], EdgeCurve::Line)))
            .collect();

        let mut inner_edges = Vec::with_capacity(lp.pieces.len());
        let mut outer_edges = Vec::with_capacity(lp.pieces.len());
        for (j, (piece, shape)) in lp.pieces.iter().zip(&wrapped[l]).enumerate() {
            let (ns, ne) = lp.natural_ends(j);
            let u = &unrolled[l][j];
            let (q0, _) = (u.sample)(0.0);
            let (q1, _) = (u.sample)(1.0);
            let ring = |r: f64| -> Result<RingEdge, OperationsError> {
                Ok(match shape {
                    WrappedEdge::Axial => (EdgeCurve::Line, None, None),
                    WrappedEdge::Circumferential => {
                        let (t0, t1) = (q0.0 / frame.radius, q1.0 / frame.radius);
                        let normal = if t1 > t0 { a } else { -a };
                        let u_axis = frame.radial(t0);
                        let circle = Circle3D::with_axes(
                            frame.origin + a * q0.1,
                            normal,
                            r,
                            u_axis,
                            normal.cross(u_axis),
                        )?;
                        (
                            EdgeCurve::Circle(circle),
                            Some((0.0, (t1 - t0).abs())),
                            None,
                        )
                    }
                    WrappedEdge::Fitted { unit, bound } => {
                        let k = r;
                        let scaled = NurbsCurve::new(
                            unit.degree(),
                            unit.knots().to_vec(),
                            unit.control_points()
                                .iter()
                                .map(|&p| frame.scale_radially(p, k))
                                .collect(),
                            unit.weights().to_vec(),
                        )?;
                        let domain = scaled.domain();
                        (
                            EdgeCurve::NurbsCurve(scaled),
                            Some(domain),
                            Some((bound * k.max(1.0)).max(linear)),
                        )
                    }
                })
            };
            for (r, store, vs) in [
                (inner, &mut inner_edges, &inner_v),
                (outer, &mut outer_edges, &outer_v),
            ] {
                let (curve, trim, tol) = ring(r)?;
                let mut edge = Edge::new(vs[ns], vs[ne], curve);
                edge.set_trim(trim);
                if let Some(tol) = tol {
                    edge.set_tolerance(Some(tol))?;
                }
                store.push(topo.add_edge(edge));
            }

            // The wall, oriented away from the material.
            let (qm, dqm) = (u.sample)(0.5);
            let dir = if piece.forward { 1.0 } else { -1.0 };
            let (ts, tz) = (dqm.0 * dir, dqm.1 * dir);
            let (ns2, nz2) = (tz * side, -ts * side);
            let theta = qm.0 / frame.radius;
            let tangential = frame.c * theta.cos() - frame.e * theta.sin();
            let outward = tangential * ns2 + a * nz2;
            let mid_r = 0.5 * (inner + outer);
            let probe = frame.wrap(qm, mid_r);
            let curve_tangent = frame.wrap_with_derivative(qm, dqm, mid_r).1;
            let natural = curve_tangent.cross(frame.radial(theta));
            let (surface, flip, reversed) = match shape {
                WrappedEdge::Axial | WrappedEdge::Circumferential => {
                    let n = if matches!(shape, WrappedEdge::Axial) {
                        tangential
                    } else {
                        a
                    };
                    let n = if n.dot(outward) < 0.0 { -n } else { n };
                    (
                        FaceSurface::Plane {
                            normal: n,
                            d: crate::dot_normal_point(n, probe),
                        },
                        n.dot(natural) < 0.0,
                        false,
                    )
                }
                WrappedEdge::Fitted { unit, .. } => {
                    let rows = unit
                        .control_points()
                        .iter()
                        .map(|&p| {
                            vec![
                                frame.scale_radially(p, inner),
                                frame.scale_radially(p, outer),
                            ]
                        })
                        .collect();
                    let weights = unit.weights().iter().map(|&w| vec![w, w]).collect();
                    let surface = NurbsSurface::new(
                        unit.degree(),
                        1,
                        unit.knots().to_vec(),
                        vec![0.0, 0.0, 1.0, 1.0],
                        rows,
                        weights,
                    )?;
                    let reversed = surface.normal(0.5, 0.5)?.dot(outward) < 0.0;
                    (FaceSurface::Nurbs(surface), false, reversed)
                }
            };
            let (bottom, top) = (inner_edges[j], outer_edges[j]);
            let edges = if flip {
                vec![
                    OrientedEdge::new(radial[ns], true),
                    OrientedEdge::new(top, true),
                    OrientedEdge::new(radial[ne], false),
                    OrientedEdge::new(bottom, false),
                ]
            } else {
                vec![
                    OrientedEdge::new(bottom, true),
                    OrientedEdge::new(radial[ne], true),
                    OrientedEdge::new(top, false),
                    OrientedEdge::new(radial[ns], false),
                ]
            };
            let wire = topo.add_wire(Wire::new(edges, true)?);
            let mut face = Face::new(wire, vec![], surface);
            if reversed {
                face.set_reversed(true);
            }
            walls.push(topo.add_face(face));
        }

        // Inner and outer faces' loops: counter-clockwise about the outward
        // radial for the outer loop, clockwise for a hole.
        let cap_wire = |topo: &mut Topology, edges: &[EdgeId]| -> Result<WireId, OperationsError> {
            let oriented: Vec<OrientedEdge> = if ccw == lp.outer {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .map(|(&e, piece)| OrientedEdge::new(e, piece.forward))
                    .collect()
            } else {
                edges
                    .iter()
                    .zip(&lp.pieces)
                    .rev()
                    .map(|(&e, piece)| OrientedEdge::new(e, !piece.forward))
                    .collect()
            };
            Ok(topo.add_wire(Wire::new(oriented, true)?))
        };
        outer_wires.push(cap_wire(topo, &outer_edges)?);
        inner_loop_wires.push(cap_wire(topo, &inner_edges)?);
    }

    let cylinder_face = |topo: &mut Topology,
                         r: f64,
                         wires: &[WireId],
                         reversed: bool|
     -> Result<FaceId, OperationsError> {
        let surface = CylindricalSurface::with_ref_dir(frame.origin, a, r, seam_dir)?;
        let mut face = Face::new(
            wires[0],
            wires[1..].to_vec(),
            FaceSurface::Cylinder(surface),
        );
        if reversed {
            face.set_reversed(true);
        }
        Ok(topo.add_face(face))
    };
    let mut faces = walls;
    faces.push(cylinder_face(topo, outer, &outer_wires, false)?);
    faces.push(cylinder_face(topo, inner, &inner_loop_wires, true)?);
    let shell = topo.add_shell(Shell::new(faces)?);
    Ok(topo.add_solid(Solid::new(shell, vec![])))
}

mod cone;
pub use cone::{WrapCone, wrap_profile_on_cone};

#[cfg(test)]
mod tests;
