//! Interior extent of a trimmed torus face for the phase-FF broad phase.
//!
//! A torus face can bulge past every one of its boundary edges. A pipe swept
//! along a circular arc is the common case: its side is one torus face bounded
//! by the two minor circles at its ends and a seam along one equator, so the
//! boundary never touches the opposite equator. A boundary-sampled box then
//! misses the whole outer half of the tube, and every face meeting only that
//! half is rejected before it is ever intersected (issue #1242: an annular
//! cutter's top plane crossing the crown of a swept tube was never sectioned,
//! and the cut came back as the unchanged tube).
//!
//! On a ring torus (`R > r`) each world coordinate is smooth on the carrier
//! and has exactly four critical points, found in closed form below (a whole
//! latitude circle when the coordinate axis is the torus axis). A face's
//! extent along an axis is reached either on its boundary or at one of these
//! interior critical points.
//!
//! A face takes a bound tighter than the whole carrier only once its outer
//! loop is certified (see [`lift`]): every boundary piece's `(u, v)` range is
//! proved from its control hull, the pieces chain into one continuous lift
//! with winding `(0, 0)`, and the sign of the lifted loop's area, with its
//! error bound, matches the kernel's wire contract (the stored outer wire
//! runs counter-clockwise about the stored surface normal; see
//! [`disc_sign`]). The face is then the disc that loop bounds, inside the
//! lifted loop's `(u, v)` hull; a loop wound the other way may mean the
//! disc's complement. An exact rectangular `(u, v)` patch bounded by analytic
//! circles then takes the tight rectangle bound: its boundary arcs carry
//! their own exact extrema (the caller adds them for circles), so a latitude
//! ridge the rectangle crosses peaks on its meridian sides. Any other
//! certified face takes the exact box of its hull rectangle patch (corners,
//! interior critical points, and the closed-form extremes of its meridian
//! and parallel sides). A face whose loop does not certify (unsupported
//! edge carriers, nonpositive weights, a winding, a complement or an
//! undecided orientation), and spindle or horn tori (`R <= r`), keep the
//! whole-carrier box: never an under-bound.
//!
//! The same lift gives the minor-angle band [`torus_minor_band`] the
//! marcher bounds torus traces by, and the `u` window [`torus_u_gap`] the
//! face's extent excludes; the extent reads the band periodically (see
//! `super::torus_v_in_band`). The band carries its provenance
//! ([`TorusBandKind`]): a proved whole tube, a certified disc's lifted hull
//! (which may straddle the projection's wrap), or nothing certified. All
//! three can march over `(0, 2π)`, so only the provenance, never the band's
//! width, may admit the whole-tube shortcuts: the extent's `v`-free strict
//! test and the marcher's exact rim exits.

use remus_math::aabb::Aabb3;
use remus_math::surfaces::ToroidalSurface;
use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::face::FaceId;

use crate::error::AlgoError;

mod lift;

use lift::{Carrier, LoopLift};

const TAU: f64 = std::f64::consts::TAU;

/// The box of the interior points of `face_id` on `torus` where a world
/// coordinate is extreme, or `None` when the face holds none. The caller
/// unions it with the boundary-sampled box.
pub(super) fn torus_face_interior_extent(
    topo: &Topology,
    face_id: FaceId,
    torus: &ToroidalSurface,
    tol: Tolerance,
) -> Option<Aabb3> {
    let Ok(Some(outer)) = certified_disc(topo, face_id, torus) else {
        return Some(torus.aabb());
    };
    // The recognizer lifts its rectangle from boundary samples; take its
    // tight bound only where that rectangle holds the certified hull.
    let slack = 10.0 * tol.angular.max(tol.linear / torus.minor_radius());
    if let Some((u, v)) = crate::classifier::rectangular_torus_domain(topo, face_id, tol)
        && window_holds(u, outer.u, slack)
        && window_holds(v, outer.v, slack)
    {
        return rectangle_extent(torus, u, v, tol);
    }
    Some(patch_box(torus, outer.u, outer.v).unwrap_or_else(|| torus.aabb()))
}

/// Whether the periodic window `[start, start + span]`, widened by `slack`,
/// holds the lifted interval `[lo, hi]`.
fn window_holds((start, span): (f64, f64), (lo, hi): (f64, f64), slack: f64) -> bool {
    if span + 2.0 * slack >= TAU {
        return true;
    }
    let shift = TAU * ((lo - start + slack) / TAU).floor();
    lo - shift >= start - slack && hi - shift <= start + span + slack
}

/// The certified lift of a face's outer loop when it proves the face is the
/// disc that loop bounds (see the module docs); `Ok(None)` otherwise, and
/// always for a spindle or horn torus.
fn certified_disc(
    topo: &Topology,
    face_id: FaceId,
    torus: &ToroidalSurface,
) -> Result<Option<LoopLift>, AlgoError> {
    let Some(carrier) = Carrier::new(torus) else {
        return Ok(None);
    };
    let face = topo.face(face_id)?;
    let Some(outer) = lift::lift_loop(topo, face.outer_wire(), &carrier)? else {
        return Ok(None);
    };
    Ok(outer.area_has_sign(disc_sign(&carrier)).then_some(outer))
}

/// The sign the outer loop's lifted `(u, v)` area takes when the face is
/// the disc it bounds.
///
/// The kernel's wire contract (`remus_check::validate::face::
/// check_face_orientation`) is that the STORED outer-wire winding runs
/// counter-clockwise about the STORED surface normal; the face's `reversed`
/// flag mirrors the effective normal and the effective edge traversal
/// together, so it plays no part (the hammer holder's reversed torus fillets
/// are wound exactly so). The ray classifier likewise reads a single
/// contractible loop as the region it encloses. A loop wound the other way
/// breaks that contract, and what it bounds (the disc, or the torus minus
/// it) is then ambiguous: it takes the whole carrier.
fn disc_sign(carrier: &Carrier) -> f64 {
    carrier.parameter_sense()
}

/// How far short of a whole turn a boundary's attained `v` span may stop and
/// still count as covering the tube: rounding of the attained end-point
/// angles only (the uncovered sliver is under `1e-12·r`, far below any
/// linear tolerance).
const COVERAGE_SLACK: f64 = 1e-12;

/// How a torus face's minor-angle band was established. The band's width
/// alone cannot tell these apart: all three can span a whole turn.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum TorusBandKind {
    /// Proved: some boundary loop sweeps every `v` (a `v` winding, or an
    /// attained lifted `v` span of a whole turn), so the face meets every
    /// minor angle. The only kind that may skip `v` tests outright or land
    /// marched exits on a partner wall's rim.
    WholeTube,
    /// The face is the certified disc its outer loop bounds, inside the
    /// loop's lifted `v` hull. That hull may straddle the projection's wrap.
    Certified,
    /// Nothing about the face's `v` extent is certified (an unsupported
    /// loop, a winding in `u` only, a complement, an undecided orientation,
    /// a spindle torus).
    Uncertain,
}

/// The minor-angle band of a torus face and its provenance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct TorusBand {
    /// The `[v0, v1] ⊂ [0, 2π]` interval of `project_point`'s `v` the
    /// marcher bounds traces by; `(0, 2π)` is the whole tube.
    pub marcher: (f64, f64),
    /// The membership band, read periodically (`super::torus_v_in_band`):
    /// a [`TorusBandKind::Certified`] face's lifted `v` hull exactly as
    /// lifted (it may straddle `0 = 2π`, and it is never snapped inward),
    /// otherwise `(0, 2π)`.
    pub extent: (f64, f64),
    pub kind: TorusBandKind,
}

/// The minor-angle band of a torus face for the phase-FF marcher and the
/// face's extent, with its provenance (see [`TorusBand`]).
///
/// - A face whose boundary is entirely degenerate (an untrimmed torus) has
///   no band (`None`), as before; it IS the whole tube.
/// - [`TorusBandKind::WholeTube`] when some boundary loop is proved to
///   sweep every `v`: a loop with a `v` winding, or one whose attained
///   lifted `v` span (points of the boundary, never a hull) reaches a whole
///   turn. The face contains its boundary, so its projection is then the
///   whole minor circle.
/// - [`TorusBandKind::Certified`] for a face certified as the disc its
///   outer loop bounds. Its extent band is the loop's lifted `v` hull; its
///   marcher band is that hull carried into `[0, 2π]`, with an end within
///   the boundary's declared tolerance of `0` or `2π` clamped to it (the
///   snap is the largest edge or vertex tolerance on the outer loop, never
///   less than `tol.linear`, over the minor radius). A hull that still
///   straddles the wrap cannot be written as one such interval: it marches
///   over the whole tube, as a bound only, and its extent keeps the lifted
///   hull, so its off-face points stay outside.
/// - [`TorusBandKind::Uncertain`] otherwise: the whole tube as a bound,
///   wider than the face, never narrower, and never a proof of coverage.
pub(super) fn torus_minor_band(
    topo: &Topology,
    face_id: FaceId,
    torus: &ToroidalSurface,
    tol: Tolerance,
) -> Result<Option<TorusBand>, AlgoError> {
    let full = |kind| {
        Some(TorusBand {
            marcher: (0.0, TAU),
            extent: (0.0, TAU),
            kind,
        })
    };
    if super::face_boundary_all_degenerate(topo, face_id, tol)? {
        return Ok(None);
    }
    let Some(carrier) = Carrier::new(torus) else {
        return Ok(full(TorusBandKind::Uncertain));
    };
    let face = topo.face(face_id)?;
    let covers =
        |l: &LoopLift| l.winding.1 != 0 || l.attained_v.1 - l.attained_v.0 >= TAU - COVERAGE_SLACK;
    let outer = lift::lift_loop(topo, face.outer_wire(), &carrier)?;
    if outer.as_ref().is_some_and(covers) {
        return Ok(full(TorusBandKind::WholeTube));
    }
    for &wire in face.inner_wires() {
        if lift::lift_loop(topo, wire, &carrier)?
            .as_ref()
            .is_some_and(covers)
        {
            return Ok(full(TorusBandKind::WholeTube));
        }
    }
    let Some(outer) = outer else {
        return Ok(full(TorusBandKind::Uncertain));
    };
    if !outer.area_has_sign(disc_sign(&carrier)) {
        return Ok(full(TorusBandKind::Uncertain));
    }
    let (lo, hi) = outer.v;
    if !(lo.is_finite() && hi.is_finite()) {
        return Ok(full(TorusBandKind::Uncertain));
    }
    let snap = boundary_tolerance(topo, face.outer_wire(), tol)? / torus.minor_radius();
    Ok(Some(TorusBand {
        marcher: band_in_projection((lo, hi), snap).unwrap_or((0.0, TAU)),
        extent: (lo, hi),
        kind: TorusBandKind::Certified,
    }))
}

/// The tolerance a wire's boundary declares: the largest edge or vertex
/// tolerance on it, and never less than `tol.linear`. A boundary point may
/// sit that far off the curve it is stored on, so a lifted band end within
/// it of the projection's wrap is not evidence of a straddle.
fn boundary_tolerance(
    topo: &Topology,
    wire: remus_topology::wire::WireId,
    tol: Tolerance,
) -> Result<f64, AlgoError> {
    let mut declared = tol.linear;
    for oe in topo.wire(wire)?.edges() {
        let edge = topo.edge(oe.edge())?;
        declared = declared
            .max(edge.tolerance().unwrap_or(0.0))
            .max(topo.vertex(edge.start())?.tolerance())
            .max(topo.vertex(edge.end())?.tolerance());
    }
    // A declared value is finite and non-negative by construction (the
    // setters refuse anything else); `max` keeps the floor on a NaN anyway.
    Ok(declared)
}

/// Smallest uncovered `u` arc a torus face's extent records as a gap; the
/// same threshold [`crate::classifier::largest_u_gap`] gives a partial
/// cylinder.
const MIN_U_GAP: f64 = 0.2;

/// The `u` arc a certified torus disc does not reach, for
/// [`super::FaceExtent`]'s membership tests: the complement of its outer
/// loop's lifted `u` hull (a superset of the face's `u` range, so no point
/// of the face falls in it). `None` (no `u` restriction) for any face that
/// does not certify, or whose hull leaves less than [`MIN_U_GAP`] uncovered.
pub(super) fn torus_u_gap(
    topo: &Topology,
    face_id: FaceId,
    torus: &ToroidalSurface,
) -> Result<Option<(f64, f64)>, AlgoError> {
    Ok(certified_disc(topo, face_id, torus)?.and_then(|outer| {
        let (lo, hi) = outer.u;
        (hi - lo <= TAU - MIN_U_GAP).then_some((hi, lo + TAU))
    }))
}

/// A lifted band `(lo, hi)` written in `project_point`'s `[0, 2π)` range,
/// with ends within `snap` of `0` or `2π` clamped to them; `None` when it
/// spans a whole turn or straddles the wrap by more than `snap`. `snap` is
/// the boundary's declared tolerance in radians (see `boundary_tolerance`):
/// the clamped-off sliver is within it, and a straddle beyond it is kept
/// whole by the caller rather than certified as the whole tube.
fn band_in_projection((lo, hi): (f64, f64), snap: f64) -> Option<(f64, f64)> {
    if !(lo.is_finite() && hi.is_finite()) || hi - lo >= TAU {
        return None;
    }
    let shift = TAU * ((lo + snap) / TAU).floor();
    let (lo, hi) = (lo - shift, hi - shift);
    (hi <= TAU + snap).then_some((lo.max(0.0), hi.min(TAU)))
}

/// Exact box of the carrier patch `[u0, u1] x [v0, v1]`. A span of a whole
/// turn or more holds every critical point and side extreme of that
/// parameter, which is the whole band.
fn patch_box(torus: &ToroidalSurface, (u0, u1): (f64, f64), (v0, v1): (f64, f64)) -> Option<Aabb3> {
    let mut points = vec![
        torus.evaluate(u0, v0),
        torus.evaluate(u1, v0),
        torus.evaluate(u0, v1),
        torus.evaluate(u1, v1),
    ];
    let inside = |x: f64, lo: f64, hi: f64| {
        let k = ((x - lo) / std::f64::consts::TAU).floor();
        let x = x - k * std::f64::consts::TAU;
        x >= lo && x <= hi
    };
    for (u, v) in interior_critical_parameters(torus) {
        if inside(u, u0, u1) && inside(v, v0, v1) {
            points.push(torus.evaluate(u, v));
        }
    }
    // Side arcs: p(t) = c + a cos t + b sin t; a coordinate along `e` is
    // extreme at t = atan2(e·b, e·a) and that plus π.
    let (x, y, z) = (torus.x_axis(), torus.y_axis(), torus.z_axis());
    let axes = [
        remus_math::vec::Vec3::new(1.0, 0.0, 0.0),
        remus_math::vec::Vec3::new(0.0, 1.0, 0.0),
        remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
    ];
    for u in [u0, u1] {
        // Meridian at u: c + R·ρ(u) + r(cos v·ρ(u) + sin v·Z).
        let radial = x * u.cos() + y * u.sin();
        for e in axes {
            let t = (e.dot(z)).atan2(e.dot(radial));
            for v in [t, t + std::f64::consts::PI] {
                if inside(v, v0, v1) {
                    points.push(torus.evaluate(u, v));
                }
            }
        }
    }
    for v in [v0, v1] {
        // Parallel at v: c + r sin v·Z + (R + r cos v)(cos u·X + sin u·Y).
        for e in axes {
            let t = (e.dot(y)).atan2(e.dot(x));
            for u in [t, t + std::f64::consts::PI] {
                if inside(u, u0, u1) {
                    points.push(torus.evaluate(u, v));
                }
            }
        }
    }
    Aabb3::try_from_points(points)
}

/// The box of the carrier's interior critical points inside the exact
/// rectangle `(u0, u_span), (v0, v_span)`, on a ring torus.
fn rectangle_extent(
    torus: &ToroidalSurface,
    u_range: (f64, f64),
    v_range: (f64, f64),
    tol: Tolerance,
) -> Option<Aabb3> {
    let metric = torus
        .minor_radius()
        .min(torus.major_radius() - torus.minor_radius());
    // The slack only ever admits a point; a rectangle's own corners are exact
    // boundary samples already, so a few ulps either way cannot lose extent
    // the boundary does not also carry.
    let slack = tol.angular.max(tol.linear / metric);
    Aabb3::try_from_points(
        interior_critical_parameters(torus)
            .into_iter()
            .filter(|&(u, v)| {
                in_periodic_interval(u, u_range, slack) && in_periodic_interval(v, v_range, slack)
            })
            .map(|(u, v)| torus.evaluate(u, v)),
    )
}

/// The `(u, v)` parameters, each in `(-π, π]`, where `x`, `y` or `z` is
/// stationary on the carrier.
///
/// With `p(u, v) = c + (R + r cos v)(cos u·X + sin u·Y) + r sin v·Z`, the
/// component along a unit `e` has `∂/∂u = (R + r cos v)(e·ρ'(u))`. The first
/// factor is positive on a ring torus, so `e·ρ'(u) = 0`, i.e. `u = atan2(e·Y,
/// e·X)` or that plus `π`. Then `∂/∂v = r(−sin v·(e·ρ(u)) + cos v·(e·Z)) = 0`
/// gives `v = atan2(e·Z, e·ρ(u))` or that plus `π`. When `e` is parallel to
/// the axis the first equation holds for every `u`, and the extremes are the
/// whole top and bottom circles; one point of each still carries the extreme
/// value, which is all a box needs.
fn interior_critical_parameters(torus: &ToroidalSurface) -> Vec<(f64, f64)> {
    let pi = std::f64::consts::PI;
    let (x, y, z) = (torus.x_axis(), torus.y_axis(), torus.z_axis());
    let mut out = Vec::with_capacity(12);
    for (ex, ey, ez) in [
        (x.x(), y.x(), z.x()),
        (x.y(), y.y(), z.y()),
        (x.z(), y.z(), z.z()),
    ] {
        let u_star = ey.atan2(ex);
        for u in [u_star, u_star + pi] {
            let radial = ex.mul_add(u.cos(), ey * u.sin());
            let v_star = ez.atan2(radial);
            for v in [v_star, v_star + pi] {
                out.push((wrap(u), wrap(v)));
            }
        }
    }
    out
}

/// `angle` reduced to `(-π, π]`.
fn wrap(angle: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let reduced = angle - tau * (angle / tau).round();
    if reduced <= -std::f64::consts::PI {
        reduced + tau
    } else {
        reduced
    }
}

/// Whether the periodic angle `angle` falls in `[start, start + span]`,
/// widened by `slack` on both sides.
fn in_periodic_interval(angle: f64, (start, span): (f64, f64), slack: f64) -> bool {
    let tau = std::f64::consts::TAU;
    if span + 2.0 * slack >= tau {
        return true;
    }
    let offset = (angle - start + slack).rem_euclid(tau);
    offset <= span + 2.0 * slack
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod certified_tests;
