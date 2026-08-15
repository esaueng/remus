//! Validation and construction for faces carrying inner (hole) wires.
//!
//! `addHolesToFace` and `makeFaceFromWires` are the only two entry points in
//! the kernel that attach an inner wire to a face, and whatever they build
//! is fed straight into `extrude`. An inner wire that is open, that does not
//! lie on the face's surface, or that escapes the outer boundary produces a
//! face that no downstream code can interpret — extrude walks it anyway and
//! emits a solid that looks plausible and is not watertight. The checks here
//! are what turn that class of failure into a typed error at the boundary.
//!
//! # What is and is not checked
//!
//! Always, for every surface type:
//! - each hole wire is topologically closed
//!   ([`validate_wire_closed`](brepkit_topology::validation::validate_wire_closed));
//! - each hole wire is distinct from the outer wire and from every other
//!   inner wire on the face;
//! - every sampled point of each hole wire lies on the face's surface within
//!   tolerance (the generalization of "coplanar" to non-planar surfaces).
//!
//! Only for planar faces:
//! - neither the outer wire nor any hole wire crosses itself;
//! - each hole lies inside the outer wire — every sampled point is inside it
//!   *and* no hole edge crosses an outer edge, which is what a concave outer
//!   contour (an 'A', 'E', 'k' or 'W' outline) needs: a bar spanning the
//!   notch of a 'U' has every corner inside the arms and its middle outside
//!   the face, so point containment alone accepts it;
//! - holes do not overlap or nest inside each other.
//!
//! Containment on a curved surface would need a UV-space point-in-polygon
//! test with periodic-seam unwrapping, which `brepkit-check` keeps private.
//! Rather than approximate it — a wrong containment answer on a cylinder
//! would reject valid input — the three positional checks are skipped there
//! and this limitation is stated rather than hidden.
//!
//! # Residual approximation
//!
//! Every positional test runs on a *chorded outline*: curved edges are
//! sampled at `CLOSED_CURVE_SAMPLES` / `OPEN_CURVE_SAMPLES` parameters
//! and the polygon through those points is what is tested. A curved edge
//! whose excursion outside the outer wire falls entirely between two
//! consecutive samples is therefore not detected. One bezier per edge, which
//! is what a font glyph produces, is comfortably resolved at this density; a
//! multi-span imported NURBS edge is not guaranteed to be. These checks are a
//! boundary guard against the failure classes seen in practice, not a proof
//! of containment.
//!
//! Hole winding is deliberately NOT constrained: `extrude` detects inner-wire
//! winding per wire (`brepkit_operations::winding::inner_wire_is_cw`) and
//! builds correct side faces for either, so requiring CW here would reject
//! input the kernel already handles.

use brepkit_check::util::{point_in_polygon_3d, wire_polygon_curve_sampled};
use brepkit_geometry::extrema::{
    point_to_cone, point_to_cylinder, point_to_sphere, point_to_surface, point_to_torus,
};
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point2, Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::face::{Face, FaceId, FaceSurface};
use brepkit_topology::wire::WireId;

use crate::error::WasmError;
use crate::helpers::{polygons_overlap_2d, segments_intersect_2d};

/// Samples contributed by a closed curved edge when outlining a wire.
const CLOSED_CURVE_SAMPLES: usize = 32;

/// Samples contributed by an open curved edge (an arc, a bezier segment)
/// when outlining a wire. A glyph counter is a handful of short beziers, so
/// one chord per segment would both miss the bow of the curve and leave the
/// containment test with a polygon too coarse to trust.
const OPEN_CURVE_SAMPLES: usize = 8;

/// Relative on-surface tolerance for closed-form surfaces (plane, cylinder,
/// cone, sphere, torus). Their point-to-surface distance is exact, so this
/// only has to absorb the caller's own coordinate round-off.
const EXACT_SURFACE_REL_TOL: f64 = 1e-7;

/// Relative on-surface tolerance for NURBS surfaces. Their closest-point
/// query is iterative rather than closed-form, so a residual well above the
/// linear tolerance is expected for a point that genuinely lies on the
/// surface; holding NURBS to `EXACT_SURFACE_REL_TOL` would reject valid input.
const NURBS_SURFACE_REL_TOL: f64 = 1e-5;

/// Distance from `p` to the (untrimmed) surface `surface`.
///
/// Plane and the four analytic surfaces have closed-form answers. NURBS goes
/// through the grid-seeded Newton projection in `brepkit-geometry` rather
/// than [`FaceSurface::project_point`]: the latter falls back to the domain
/// midpoint when its Newton iteration fails, which would read as an enormous
/// deviation and reject a hole that is in fact on the surface.
fn surface_deviation(surface: &FaceSurface, p: Point3) -> f64 {
    match surface {
        FaceSurface::Plane { normal, d } => {
            let n = *normal;
            n.x()
                .mul_add(p.x(), n.y().mul_add(p.y(), n.z() * p.z()) - *d)
                .abs()
        }
        FaceSurface::Cylinder(c) => point_to_cylinder(p, c).distance,
        FaceSurface::Cone(c) => point_to_cone(p, c).distance,
        FaceSurface::Sphere(s) => point_to_sphere(p, s).distance,
        FaceSurface::Torus(t) => point_to_torus(p, t).distance,
        FaceSurface::Nurbs(n) => point_to_surface(p, n, n.domain_u(), n.domain_v()).distance,
    }
}

/// Relative-to-absolute on-surface tolerance for `surface` at scale `scale`.
fn on_surface_tolerance(surface: &FaceSurface, scale: f64) -> f64 {
    let rel = if matches!(surface, FaceSurface::Nurbs(_)) {
        NURBS_SURFACE_REL_TOL
    } else {
        EXACT_SURFACE_REL_TOL
    };
    // Never tighter than the workspace linear tolerance, so unit-scale
    // geometry is not held to a tolerance smaller than the kernel's own.
    (rel * scale).max(Tolerance::new().linear)
}

/// Largest absolute coordinate over `points`, floored at 1.0.
///
/// Used to turn the relative tolerances above into absolute ones. Flooring
/// at 1.0 keeps sub-millimetre geometry from being held to an absurdly tight
/// bound.
fn coordinate_scale(points: &[Point3]) -> f64 {
    points.iter().fold(1.0, |acc, p| {
        acc.max(p.x().abs()).max(p.y().abs()).max(p.z().abs())
    })
}

/// Outline a wire as a 3D polygon, with curved edges chorded finely enough
/// for a containment test.
fn wire_outline(topo: &Topology, wire: WireId) -> Result<Vec<Point3>, WasmError> {
    let pts = wire_polygon_curve_sampled(topo, wire, CLOSED_CURVE_SAMPLES, OPEN_CURVE_SAMPLES)?;
    if pts.len() < 3 {
        return Err(WasmError::InvalidInput {
            reason: format!(
                "wire {} outlines only {} point(s); a face boundary needs at least 3",
                wire.index(),
                pts.len()
            ),
        });
    }
    Ok(pts)
}

/// The normal used for planar containment tests, or `None` when the face is
/// not planar (containment is not checked there — see the module docs).
fn planar_normal(surface: &FaceSurface) -> Option<Vec3> {
    match surface {
        FaceSurface::Plane { normal, .. } => Some(*normal),
        FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => None,
    }
}

/// True when every point of `inner` lies inside the polygon `outer`.
///
/// Returns the index of the first escaping point instead of a bare bool so
/// the error can name it.
fn first_point_outside(inner: &[Point3], outer: &[Point3], normal: Vec3) -> Option<usize> {
    inner
        .iter()
        .position(|p| !point_in_polygon_3d(p, outer, &normal))
}

/// Drop a planar 3D polygon into 2D along the axis the plane normal is
/// most aligned with — the same projection [`point_in_polygon_3d`] uses,
/// so the two agree about which polygon a point is in.
fn project_to_2d(points: &[Point3], normal: Vec3) -> Vec<Point2> {
    let (ax, ay, az) = (normal.x().abs(), normal.y().abs(), normal.z().abs());
    if az >= ax && az >= ay {
        points.iter().map(|p| Point2::new(p.x(), p.y())).collect()
    } else if ay >= ax {
        points.iter().map(|p| Point2::new(p.x(), p.z())).collect()
    } else {
        points.iter().map(|p| Point2::new(p.y(), p.z())).collect()
    }
}

/// True when two planar loops share any area.
///
/// Vertex containment alone is not enough: two rectangles crossing in a
/// plus sign have no vertex of either inside the other, so the edge-crossing
/// half of [`polygons_overlap_2d`] is what catches them.
fn loops_overlap(a: &[Point3], b: &[Point3], normal: Vec3) -> bool {
    polygons_overlap_2d(&project_to_2d(a, normal), &project_to_2d(b, normal))
}

/// Iterate the closed polygon `poly` as `(start, end)` segment pairs.
fn segments(poly: &[Point2]) -> impl Iterator<Item = (Point2, Point2)> + '_ {
    (0..poly.len()).map(move |i| (poly[i], poly[(i + 1) % poly.len()]))
}

fn point_segment_distance_2d(p: Point2, a: Point2, b: Point2) -> f64 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq <= f64::EPSILON {
        return (p - a).length();
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

fn loops_touch(a: &[Point3], b: &[Point3], normal: Vec3, tolerance: f64) -> bool {
    let (a2, b2) = (project_to_2d(a, normal), project_to_2d(b, normal));
    segments(&a2).any(|(a0, a1)| {
        segments(&b2).any(|(b0, b1)| {
            point_segment_distance_2d(a0, b0, b1) <= tolerance
                || point_segment_distance_2d(a1, b0, b1) <= tolerance
                || point_segment_distance_2d(b0, a0, a1) <= tolerance
                || point_segment_distance_2d(b1, a0, a1) <= tolerance
        })
    })
}

/// True when any segment of `a` properly crosses any segment of `b`.
///
/// The companion to [`first_point_outside`]. Containment cannot be decided by
/// sampling points alone on a *concave* boundary: a bar laid across the notch
/// of a 'U' has every corner inside the two arms while its middle is outside
/// the face. A loop wholly inside another cannot cross it, so "some point
/// inside and no crossing" is what containment actually means here.
fn loops_cross(a: &[Point3], b: &[Point3], normal: Vec3) -> bool {
    let (a2, b2) = (project_to_2d(a, normal), project_to_2d(b, normal));
    segments(&a2)
        .any(|(a1, a2)| segments(&b2).any(|(b1, b2)| segments_intersect_2d(a1, a2, b1, b2)))
}

/// Index of the first segment of `poly` that properly crosses a later,
/// non-adjacent segment of the same loop.
///
/// A figure-8 wire is topologically closed, coplanar and contained, so every
/// other check passes it — and the face it describes has no consistent
/// interior, which `extrude` turns into a plausible-looking, non-watertight
/// solid. [`segments_intersect_2d`] tests for a *proper* crossing, so
/// segments that merely share an endpoint (every adjacent pair) do not
/// register; adjacent pairs are skipped anyway to make that explicit.
fn first_self_crossing(poly: &[Point3], normal: Vec3) -> Option<(usize, usize)> {
    let p = project_to_2d(poly, normal);
    let n = p.len();
    for i in 0..n {
        let (a1, a2) = (p[i], p[(i + 1) % n]);
        // Skip j == i (itself), j == i+1 and the wrap-around pair (i == 0,
        // j == n-1): those share an endpoint by construction.
        for j in (i + 2)..n {
            if i == 0 && j == n - 1 {
                continue;
            }
            let (b1, b2) = (p[j], p[(j + 1) % n]);
            if segments_intersect_2d(a1, a2, b1, b2) {
                return Some((i, j));
            }
        }
    }
    None
}

/// Validate a set of hole wires against a face's outer wire and surface.
///
/// `existing_inner` are the inner wires the face already carries (empty when
/// building a face from scratch); `new_holes` are the wires being added.
/// Both are checked for mutual overlap, so adding a second copy of an
/// existing hole is rejected.
///
/// # Errors
///
/// Returns [`WasmError::InvalidInput`] naming the offending wire when a hole
/// duplicates another wire, is not closed, leaves the face's surface, escapes
/// the outer wire, or overlaps another hole. Returns
/// [`WasmError::Check`] / [`WasmError::Topology`] if the topology cannot be
/// walked at all.
pub fn validate_hole_wires(
    topo: &Topology,
    surface: &FaceSurface,
    outer_wire: WireId,
    existing_inner: &[WireId],
    new_holes: &[WireId],
) -> Result<(), WasmError> {
    // ── Identity: a hole may not be the outer wire or an existing hole ──
    for (i, &hole) in new_holes.iter().enumerate() {
        if hole == outer_wire {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "hole wire {i} (wire {}) is the face's own outer wire",
                    hole.index()
                ),
            });
        }
        if existing_inner.contains(&hole) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "hole wire {i} (wire {}) is already an inner wire of this face",
                    hole.index()
                ),
            });
        }
        if new_holes[..i].contains(&hole) {
            return Err(WasmError::InvalidInput {
                reason: format!("hole wire {} (wire {}) is listed twice", i, hole.index()),
            });
        }
    }

    // ── Closedness ────────────────────────────────────────────────
    for (i, &hole) in new_holes.iter().enumerate() {
        let wire = topo.wire(hole)?;
        brepkit_topology::validation::validate_wire_closed(wire, topo).map_err(|e| {
            WasmError::InvalidInput {
                reason: format!(
                    "hole wire {i} (wire {}) is not a closed loop: {e}",
                    hole.index()
                ),
            }
        })?;
    }

    // ── Outlines ──────────────────────────────────────────────────
    let outer_poly = wire_outline(topo, outer_wire)?;
    let new_polys = new_holes
        .iter()
        .map(|&h| wire_outline(topo, h))
        .collect::<Result<Vec<_>, _>>()?;

    let mut scale = coordinate_scale(&outer_poly);
    for poly in &new_polys {
        scale = scale.max(coordinate_scale(poly));
    }
    let surf_tol = on_surface_tolerance(surface, scale);

    // ── On the face's surface ─────────────────────────────────────
    for (i, poly) in new_polys.iter().enumerate() {
        for p in poly {
            let dev = surface_deviation(surface, *p);
            // `>` rather than `!(<= )` keeps clippy happy, but NaN must not
            // slip through as "on the surface" — reject it explicitly.
            if dev.is_nan() || dev > surf_tol {
                return Err(WasmError::InvalidInput {
                    reason: format!(
                        "hole wire {} (wire {}) does not lie on the face's surface: \
                         point ({:.6}, {:.6}, {:.6}) is {dev:.3e} away, tolerance {surf_tol:.3e}",
                        i,
                        new_holes[i].index(),
                        p.x(),
                        p.y(),
                        p.z(),
                    ),
                });
            }
        }
    }

    // ── Containment and hole-vs-hole overlap (planar faces only) ──
    let Some(normal) = planar_normal(surface) else {
        return Ok(());
    };

    // ── Self-intersection ─────────────────────────────────────────
    if let Some((i, j)) = first_self_crossing(&outer_poly, normal) {
        return Err(WasmError::InvalidInput {
            reason: format!(
                "the outer wire (wire {}) crosses itself: outline segments {i} and {j} \
                 intersect, so the face has no consistent interior",
                outer_wire.index()
            ),
        });
    }
    for (i, poly) in new_polys.iter().enumerate() {
        if let Some((a, b)) = first_self_crossing(poly, normal) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "hole wire {} (wire {}) crosses itself: outline segments {a} and {b} \
                     intersect, so the hole has no consistent interior",
                    i,
                    new_holes[i].index()
                ),
            });
        }
    }

    // ── Containment in the outer wire ─────────────────────────────
    for (i, poly) in new_polys.iter().enumerate() {
        if loops_touch(poly, &outer_poly, normal, surf_tol) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "hole wire {} (wire {}) touches the face's outer boundary",
                    i,
                    new_holes[i].index()
                ),
            });
        }
        if let Some(k) = first_point_outside(poly, &outer_poly, normal) {
            let p = poly[k];
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "hole wire {} (wire {}) is not contained in the face's outer wire: \
                     point ({:.6}, {:.6}, {:.6}) lies outside it",
                    i,
                    new_holes[i].index(),
                    p.x(),
                    p.y(),
                    p.z(),
                ),
            });
        }
        // Every sampled point being inside is not containment on a concave
        // outer contour — the hole may leave and re-enter between samples.
        if loops_cross(poly, &outer_poly, normal) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "hole wire {} (wire {}) is not contained in the face's outer wire: \
                     it crosses the outer boundary",
                    i,
                    new_holes[i].index(),
                ),
            });
        }
    }

    // Two holes of one face must be disjoint. A hole nested in another, or
    // crossing it, describes a region that is already void — the face it
    // produces has no consistent interior. Both cases show up as a point of
    // one loop landing inside the other, so one symmetric test covers them.
    let existing_polys = existing_inner
        .iter()
        .map(|&w| wire_outline(topo, w))
        .collect::<Result<Vec<_>, _>>()?;

    for (i, poly) in new_polys.iter().enumerate() {
        let others = existing_polys
            .iter()
            .zip(existing_inner.iter())
            .map(|(p, w)| (p, *w, true))
            .chain(
                new_polys
                    .iter()
                    .zip(new_holes.iter())
                    .enumerate()
                    .filter(|&(j, _)| j != i)
                    .map(|(_, (p, w))| (p, *w, false)),
            );
        for (other_poly, other_wire, other_is_existing) in others {
            if loops_overlap(poly, other_poly, normal)
                || loops_touch(poly, other_poly, normal, surf_tol)
            {
                let which = if other_is_existing {
                    "an existing inner wire"
                } else {
                    "another new hole wire"
                };
                return Err(WasmError::InvalidInput {
                    reason: format!(
                        "hole wire {} (wire {}) overlaps {which} (wire {}); \
                         holes of one face must be disjoint",
                        i,
                        new_holes[i].index(),
                        other_wire.index()
                    ),
                });
            }
        }
    }

    Ok(())
}

/// Build a new face from `outer_wire` plus validated `hole_wires`.
///
/// The surface is taken from `surface`; the caller supplies it because
/// `addHolesToFace` reuses the source face's surface while
/// `makeFaceFromWires` derives one from the outer wire.
///
/// # Errors
///
/// Propagates every error from [`validate_hole_wires`].
pub fn build_holed_face(
    topo: &mut Topology,
    surface: FaceSurface,
    outer_wire: WireId,
    existing_inner: &[WireId],
    new_holes: &[WireId],
) -> Result<FaceId, WasmError> {
    validate_hole_wires(topo, &surface, outer_wire, existing_inner, new_holes)?;

    let mut inner = existing_inner.to_vec();
    inner.extend_from_slice(new_holes);
    Ok(topo.add_face(Face::new(outer_wire, inner, surface)))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::{EXACT_SURFACE_REL_TOL, NURBS_SURFACE_REL_TOL, validate_hole_wires};
    use brepkit_math::surfaces::{
        ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface,
    };
    use brepkit_math::vec::{Point3, Vec3};
    use brepkit_topology::Topology;
    use brepkit_topology::builder::make_polygon_wire;
    use brepkit_topology::face::FaceSurface;
    use brepkit_topology::wire::WireId;

    const TOL: f64 = 1e-7;

    /// Build a closed line-edge wire through `pts`.
    ///
    /// Line edges contribute only their start vertex to a wire outline, so
    /// the sampled outline is exactly `pts` — which is what lets a "hole" be
    /// placed exactly on a curved surface without solving for a curve that
    /// lies on it.
    fn wire(topo: &mut Topology, pts: &[Point3]) -> WireId {
        make_polygon_wire(topo, pts, TOL).unwrap()
    }

    /// An outer wire far enough from the hole that containment never fires;
    /// on the non-planar surfaces below containment is skipped anyway.
    fn outer_ring(topo: &mut Topology, pts: &[Point3]) -> WireId {
        wire(topo, pts)
    }

    /// Four points on the cylinder of radius `r` about the z axis, at z = 0.
    fn on_cylinder(r: f64) -> Vec<Point3> {
        (0..4)
            .map(|i| {
                let a = f64::from(i) * std::f64::consts::FRAC_PI_2;
                Point3::new(r * a.cos(), r * a.sin(), 0.0)
            })
            .collect()
    }

    // ── non-planar surfaces: the `surface_deviation` arms ──────────
    //
    // Each case places a hole wire exactly on the surface (accepted) and the
    // same wire pushed visibly off it (rejected). Without these, every arm
    // but `Plane` could return 0.0 — accept anything, anywhere — and the
    // suite would stay green.

    fn assert_on_surface(surface: &FaceSurface, on: &[Point3], off: &[Point3]) {
        let mut topo = Topology::new();
        // The outer wire must also be on the surface: it is not deviation
        // checked, but it sets the coordinate scale.
        let outer = outer_ring(&mut topo, on);
        let hole_on = wire(&mut topo, on);
        let hole_off = wire(&mut topo, off);

        validate_hole_wires(&topo, surface, outer, &[], &[hole_on])
            .expect("a hole lying on the surface must be accepted");

        let err = validate_hole_wires(&topo, surface, outer, &[], &[hole_off])
            .expect_err("a hole off the surface must be rejected");
        assert!(
            err.to_string()
                .contains("does not lie on the face's surface"),
            "message was: {err}"
        );
    }

    #[test]
    fn planar_hole_touching_outer_boundary_is_rejected() {
        let mut topo = Topology::new();
        let outer = wire(
            &mut topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0, 0.0, 0.0),
                Point3::new(10.0, 10.0, 0.0),
                Point3::new(0.0, 10.0, 0.0),
            ],
        );
        let touching = wire(
            &mut topo,
            &[
                Point3::new(0.0, 4.0, 0.0),
                Point3::new(2.0, 4.0, 0.0),
                Point3::new(2.0, 6.0, 0.0),
                Point3::new(0.0, 6.0, 0.0),
            ],
        );
        let result = validate_hole_wires(
            &topo,
            &FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
            outer,
            &[],
            &[touching],
        );
        assert!(result.is_err());
    }

    #[test]
    fn cylindrical_face_rejects_a_hole_wire_off_the_cylinder() {
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0)
                .unwrap();
        assert_on_surface(
            &FaceSurface::Cylinder(cyl),
            &on_cylinder(5.0),
            &on_cylinder(5.5),
        );
    }

    #[test]
    fn conical_face_rejects_a_hole_wire_off_the_cone() {
        // Half-angle 45°, apex at the origin, axis +z: radius == height.
        let cone = ConicalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            std::f64::consts::FRAC_PI_4,
        )
        .unwrap();
        let at = |r: f64, z: f64| -> Vec<Point3> {
            (0..4)
                .map(|i| {
                    let a = f64::from(i) * std::f64::consts::FRAC_PI_2;
                    Point3::new(r * a.cos(), r * a.sin(), z)
                })
                .collect()
        };
        assert_on_surface(&FaceSurface::Cone(cone), &at(4.0, 4.0), &at(4.0, 5.0));
    }

    #[test]
    fn spherical_face_rejects_a_hole_wire_off_the_sphere() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 5.0).unwrap();
        assert_on_surface(
            &FaceSurface::Sphere(sphere),
            &on_cylinder(5.0),
            &on_cylinder(5.4),
        );
    }

    #[test]
    fn toroidal_face_rejects_a_hole_wire_off_the_torus() {
        // Major 10, minor 2, axis +z: the outer equator sits at radius 12.
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 2.0).unwrap();
        assert_on_surface(
            &FaceSurface::Torus(torus),
            &on_cylinder(12.0),
            &on_cylinder(12.7),
        );
    }

    #[test]
    fn nurbs_face_rejects_a_hole_wire_off_the_surface() {
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0)
                .unwrap();
        let nurbs =
            brepkit_geometry::convert::surface_to_nurbs::cylinder_to_nurbs(&cyl, (-5.0, 5.0))
                .expect("cylinder → NURBS");
        assert_on_surface(
            &FaceSurface::Nurbs(nurbs),
            &on_cylinder(5.0),
            &on_cylinder(5.5),
        );
    }

    // ── the on-surface tolerance constants ─────────────────────────
    //
    // These pin `EXACT_SURFACE_REL_TOL` / `NURBS_SURFACE_REL_TOL` and the
    // scale-relative machinery around them from BOTH sides: an accepted case
    // just inside the bound and a rejected case just outside it, at two
    // coordinate scales three orders of magnitude apart.

    /// `z = 0` plane with an outer square of half-size `half`, and a hole
    /// square of half-size `half / 2` lifted to `z = lift`. Returns whether
    /// the hole was accepted.
    fn plane_hole_accepted_at_lift(half: f64, lift: f64) -> bool {
        let mut topo = Topology::new();
        let sq = |h: f64, z: f64| -> Vec<Point3> {
            vec![
                Point3::new(-h, -h, z),
                Point3::new(h, -h, z),
                Point3::new(h, h, z),
                Point3::new(-h, h, z),
            ]
        };
        let outer = wire(&mut topo, &sq(half, 0.0));
        let hole = wire(&mut topo, &sq(half / 2.0, lift));
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        };
        validate_hole_wires(&topo, &surface, outer, &[], &[hole]).is_ok()
    }

    #[test]
    fn on_surface_tolerance_is_pinned_from_both_sides() {
        // scale = 10 → tolerance = 1e-7 × 10 = 1e-6.
        assert!(
            plane_hole_accepted_at_lift(10.0, 5e-7),
            "a hole 5e-7 off a scale-10 plane is within tolerance and must be accepted \
             — EXACT_SURFACE_REL_TOL has been tightened"
        );
        assert!(
            !plane_hole_accepted_at_lift(10.0, 5e-6),
            "a hole 5e-6 off a scale-10 plane is outside tolerance and must be rejected \
             — EXACT_SURFACE_REL_TOL has been loosened"
        );
        // Sanity: the bound this brackets is the one the constant computes.
        assert!(
            (EXACT_SURFACE_REL_TOL * 10.0 - 1e-6).abs() < 1e-15,
            "the bracket above assumes a 1e-6 bound at scale 10"
        );
    }

    #[test]
    fn on_surface_tolerance_scales_with_the_geometry() {
        // scale = 1e4 → tolerance = 1e-3. A FIXED absolute tolerance of 1e-6
        // would reject the first of these, so this is what proves the bound
        // is relative rather than constant.
        assert!(
            plane_hole_accepted_at_lift(1e4, 5e-4),
            "at scale 1e4 the bound is 1e-3; 5e-4 must be accepted"
        );
        assert!(
            !plane_hole_accepted_at_lift(1e4, 5e-3),
            "at scale 1e4 the bound is 1e-3; 5e-3 must be rejected"
        );
    }

    #[test]
    fn nurbs_tolerance_is_looser_than_the_closed_form_one_and_floors_at_unit_scale() {
        // A deviation between the two constants: accepted on NURBS, rejected
        // on the closed-form cylinder. If NURBS_SURFACE_REL_TOL were merged
        // into EXACT_SURFACE_REL_TOL, this would fail.
        const { assert!(NURBS_SURFACE_REL_TOL > EXACT_SURFACE_REL_TOL) };

        // Sub-unit geometry: `coordinate_scale` floors at 1.0, so the NURBS
        // bound here is 1e-5 rather than 1e-5 × 0.5. Without that floor the
        // deviation below would be rejected.
        let cyl =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 0.5)
                .unwrap();
        let nurbs =
            brepkit_geometry::convert::surface_to_nurbs::cylinder_to_nurbs(&cyl, (-0.5, 0.5))
                .expect("cylinder → NURBS");

        let mut topo = Topology::new();
        let outer = wire(&mut topo, &on_cylinder(0.5));
        // 5e-6 off a radius-0.5 cylinder: inside the 1e-5 NURBS bound…
        let near = wire(&mut topo, &on_cylinder(0.500_005));
        assert!(
            validate_hole_wires(&topo, &FaceSurface::Nurbs(nurbs), outer, &[], &[near]).is_ok(),
            "the unit-scale floor in coordinate_scale keeps the NURBS bound at 1e-5"
        );
        // …and outside the 1e-7 closed-form bound for the same cylinder.
        assert!(
            validate_hole_wires(&topo, &FaceSurface::Cylinder(cyl), outer, &[], &[near]).is_err(),
            "the closed-form bound is far tighter than the NURBS one"
        );
    }
}
