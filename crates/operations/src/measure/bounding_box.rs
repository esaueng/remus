//! Bounding box computation for B-rep solids.

use std::collections::HashSet;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use brepkit_math::aabb::Aabb3;
use brepkit_math::surfaces::{SphericalSurface, ToroidalSurface};
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;

use super::helpers::{collect_solid_vertex_points, compute_angular_range};

/// Compute the axis-aligned bounding box of a solid.
///
/// Uses vertex positions as the base AABB, then expands for non-planar
/// surfaces by sampling edge midpoints on the surface. This captures
/// curvature without over-expanding (unlike projecting the surface's
/// full theoretical extent): every expansion is bounded by the region the
/// face actually occupies, never the whole surface its geometry sits on.
///
/// # Errors
///
/// Returns an error if the solid has no vertices or a topology lookup fails.
pub fn solid_bounding_box(
    topo: &Topology,
    solid: SolidId,
) -> Result<Aabb3, crate::OperationsError> {
    let points = collect_solid_vertex_points(topo, solid)?;
    let mut aabb = Aabb3::try_from_points(points.iter().copied()).ok_or_else(|| {
        crate::OperationsError::InvalidInput {
            reason: "solid has no vertices".into(),
        }
    })?;

    // Expand AABB for non-planar faces by sampling edge midpoints on the
    // actual surface. This captures curvature (e.g., the arc midpoint of a
    // fillet cylinder) without over-expanding to the surface's full extent.
    let solid_data = topo.solid(solid)?;
    let shell = topo.shell(solid_data.outer_shell())?;
    for &fid in shell.faces() {
        if let Ok(face) = topo.face(fid) {
            expand_aabb_for_face(topo, &mut aabb, fid, face.surface());
        }
    }

    Ok(aabb)
}

/// Compute a conservative axis-aligned bounding box over an arbitrary set of
/// faces (e.g. one connected component of a multi-region solid).
///
/// Like [`solid_bounding_box`], the box starts from the faces' vertex
/// positions and is then expanded for surface curvature, so the returned box
/// is a conservative *outer* bound of every face in the set. Used by the
/// disjoint-fuse fast path to test whether two operands' components are
/// spatially separated.
///
/// # Errors
///
/// Returns an error if the face set is empty (no vertices) or a topology
/// lookup fails.
pub fn face_set_bounding_box(
    topo: &Topology,
    faces: &[FaceId],
) -> Result<Aabb3, crate::OperationsError> {
    let mut vertex_ids = HashSet::new();
    for &fid in faces {
        let face = topo.face(fid)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id)?;
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge())?;
                vertex_ids.insert(edge.start());
                vertex_ids.insert(edge.end());
            }
        }
    }

    let mut points = Vec::with_capacity(vertex_ids.len());
    for vid in vertex_ids {
        points.push(topo.vertex(vid)?.point());
    }
    let mut aabb = Aabb3::try_from_points(points.iter().copied()).ok_or_else(|| {
        crate::OperationsError::InvalidInput {
            reason: "face set has no vertices".into(),
        }
    })?;

    for &fid in faces {
        if let Ok(face) = topo.face(fid) {
            expand_aabb_for_face(topo, &mut aabb, fid, face.surface());
        }
    }

    Ok(aabb)
}

/// Expand an AABB to include a point.
fn aabb_include(aabb: &mut Aabb3, p: Point3) {
    *aabb = aabb.union(Aabb3 { min: p, max: p });
}

/// Expand an AABB for a face, accounting for surface curvature.
///
/// Uses different strategies based on surface type:
/// - **Sphere/Torus**: analytic expansion over the face's *trimmed* parameter
///   region, recovered from its boundary (see [`ring_patch_box`])
/// - **Cylinder/Cone**: wire-bounded expansion (sample edge midpoints
///   to avoid over-expanding for partial arcs like fillets)
/// - **NURBS**: sparse interior grid sampling
/// - **Plane**: no expansion needed
#[allow(clippy::too_many_lines)]
fn expand_aabb_for_face(
    topo: &Topology,
    aabb: &mut Aabb3,
    face_id: brepkit_topology::face::FaceId,
    surface: &FaceSurface,
) {
    // Always sample wire midpoints — captures curvature of curved boundary
    // edges (Circle, Ellipse, NurbsCurve) regardless of surface type.
    // Critical for: cone base discs (Plane face with circle edge), partial
    // arcs whose extremes lie between vertices, and any curved edge on a
    // planar face.
    sample_face_wire_midpoints(topo, aabb, face_id);

    match surface {
        FaceSurface::Plane { .. } => {}

        // Sphere and torus: analytic expansion, but only over the parameter
        // region the face actually occupies. Sampling the whole surface is
        // what made an imported part's box twice its true size — a 4 mm blend
        // riding a 270 mm ring reported the entire ring (issue: imported CATIA
        // part misframes Fit View). A face that genuinely wraps its surface
        // still gets the full extent, because the domain recovery below falls
        // back to the full period whenever the boundary does not bound.
        FaceSurface::Sphere(s) => {
            let (lo, hi) = ring_patch_box(
                s.center(),
                [s.x_axis(), s.y_axis(), s.z_axis()],
                0.0,
                s.radius(),
                sphere_patch_domain(topo, face_id, s),
            );
            aabb_include(aabb, lo);
            aabb_include(aabb, hi);
        }
        FaceSurface::Torus(t) => {
            let (lo, hi) = ring_patch_box(
                t.center(),
                [t.x_axis(), t.y_axis(), t.z_axis()],
                t.major_radius(),
                t.minor_radius(),
                torus_patch_domain(topo, face_id, t),
            );
            aabb_include(aabb, lo);
            aabb_include(aabb, hi);
        }

        // Cylinder: expand radially at each face vertex's axis projection.
        // Unlike the old approach that used AABB corners (which over-expands
        // for fillet cylinders), this uses the face's own vertices to
        // constrain the expansion to the actual face extent.
        FaceSurface::Cylinder(c) => {
            expand_cylinder_at_vertices(topo, aabb, face_id, c);
        }

        // Cone: expand radially at each face vertex (the radius varies per
        // axial position). Uses the vertex's own distance-from-axis as the
        // local radius, then projects to a full circle at that axial slice.
        FaceSurface::Cone(c) => {
            expand_cone_at_vertices(topo, aabb, face_id, c);
        }

        // NURBS: sample the surface at a sparse interior grid.
        //
        // KNOWN GAP: this grid spans the surface's whole knot domain, so a face
        // trimmed to a corner of a larger patch still reports the rest of it —
        // the same defect the sphere/torus arms above just lost. Closing it
        // needs the face's parameter region, and a NURBS surface only yields
        // that through `project_point` (a coarse grid search plus Newton,
        // ~7.5 us a sample against the analytic surfaces' closed-form atan2),
        // plus a periodicity guard so a closed patch whose seam bounds nothing
        // does not collapse. Left for its own change rather than folded in
        // here on a performance and soundness profile this one does not share.
        FaceSurface::Nurbs(nurbs) => {
            let (u_min, u_max) = nurbs.domain_u();
            let (v_min, v_max) = nurbs.domain_v();
            let n_samples = 4;
            #[allow(clippy::cast_precision_loss)]
            for iu in 1..n_samples {
                let u = u_min + (u_max - u_min) * (iu as f64) / (n_samples as f64);
                for iv in 1..n_samples {
                    let v = v_min + (v_max - v_min) * (iv as f64) / (n_samples as f64);
                    aabb_include(aabb, nurbs.evaluate(u, v));
                }
            }
        }
    }
}

/// Samples taken along each boundary edge when recovering a face's trimmed
/// parameter region. 16 keeps consecutive parameter samples on a quarter-turn
/// arc under 6° apart, comfortably below the gap that
/// [`compute_angular_range`] reads as "the face stops here".
const TRIM_SAMPLES_PER_EDGE: usize = 16;

/// The `(u, v)` rectangle a face occupies on its analytic surface.
///
/// `u.1 - u.0 == TAU` (and likewise for `v`) means "not bounded by the
/// boundary in this direction" — the face wraps, or its boundary degenerates
/// to a seam — so the caller must assume the full period.
struct PatchDomain {
    u: (f64, f64),
    v: (f64, f64),
}

/// Sample every boundary edge of a face — outer wire and inner wires — at
/// [`TRIM_SAMPLES_PER_EDGE`] intervals, endpoints included.
fn face_boundary_samples(topo: &Topology, face_id: FaceId) -> Vec<Point3> {
    let mut pts = Vec::new();
    let Ok(face) = topo.face(face_id) else {
        return pts;
    };
    for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let Ok(wire) = topo.wire(wid) else {
            continue;
        };
        for oe in wire.edges() {
            let Ok(edge) = topo.edge(oe.edge()) else {
                continue;
            };
            let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end())) else {
                continue;
            };
            let (p_start, p_end) = (sv.point(), ev.point());
            let (t0, t1) = edge.curve().domain_with_endpoints(p_start, p_end);
            for i in 0..=TRIM_SAMPLES_PER_EDGE {
                #[allow(clippy::cast_precision_loss)]
                let frac = (i as f64) / (TRIM_SAMPLES_PER_EDGE as f64);
                let t = (t1 - t0).mul_add(frac, t0);
                pts.push(edge.curve().evaluate_with_endpoints(t, p_start, p_end));
            }
        }
    }
    pts
}

/// Recover the `(u, v)` rectangle a toroidal face occupies.
///
/// Both torus directions are periodic and free of degeneracies, so a face is
/// bounded in a direction exactly when its boundary leaves a gap there —
/// which is what [`compute_angular_range`] tests, returning the full period
/// when it finds none.
fn torus_patch_domain(topo: &Topology, face_id: FaceId, t: &ToroidalSurface) -> PatchDomain {
    let pts = face_boundary_samples(topo, face_id);
    let mut us = Vec::with_capacity(pts.len());
    let mut vs = Vec::with_capacity(pts.len());
    for p in &pts {
        let (u, v) = t.project_point(*p);
        us.push(u);
        vs.push(v);
    }
    PatchDomain {
        u: compute_angular_range(&mut us),
        v: compute_angular_range(&mut vs),
    }
}

/// Recover the `(u, v)` rectangle a spherical face occupies.
///
/// Longitude is periodic and handled like the torus. Latitude is not: a polar
/// cap's only boundary is one latitude circle, so its sampled latitude range
/// collapses to that circle while the face runs on to the pole. A face bounded
/// in longitude cannot contain a pole (every longitude meets there), so the
/// sampled latitude range is trusted only in that case; a face that wraps
/// longitude keeps the full latitude span, exactly as before this was
/// trim-aware.
fn sphere_patch_domain(topo: &Topology, face_id: FaceId, s: &SphericalSurface) -> PatchDomain {
    let pts = face_boundary_samples(topo, face_id);
    let mut us = Vec::with_capacity(pts.len());
    let mut vs = Vec::with_capacity(pts.len());
    for p in &pts {
        let (u, v) = s.project_point(*p);
        us.push(u);
        vs.push(v);
    }
    let u = compute_angular_range(&mut us);
    let wraps_longitude = u.1 - u.0 >= TAU - 1e-12;
    let v = if wraps_longitude || vs.is_empty() {
        (-FRAC_PI_2, FRAC_PI_2)
    } else {
        (
            vs.iter().copied().fold(f64::INFINITY, f64::min),
            vs.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        )
    };
    PatchDomain { u, v }
}

/// World-space corners of the analytic patch
/// `center + (R + r·cos v)·(x̂·cos u + ŷ·sin u) + ẑ·r·sin v`
/// over `domain`. A sphere of radius `r` is this family with `R = 0`, so both
/// surfaces share one routine.
fn ring_patch_box(
    center: Point3,
    frame: [Vec3; 3],
    major: f64,
    minor: f64,
    domain: PatchDomain,
) -> (Point3, Point3) {
    let [xa, ya, za] = frame;
    let axis =
        |a: f64, b: f64, k: f64| ring_patch_axis_extent(a, b, k, major, minor, domain.u, domain.v);
    let (x_lo, x_hi) = axis(xa.x(), ya.x(), za.x());
    let (y_lo, y_hi) = axis(xa.y(), ya.y(), za.y());
    let (z_lo, z_hi) = axis(xa.z(), ya.z(), za.z());
    (
        Point3::new(center.x() + x_lo, center.y() + y_lo, center.z() + z_lo),
        Point3::new(center.x() + x_hi, center.y() + y_hi, center.z() + z_hi),
    )
}

/// Exact extent of the patch along one world axis, relative to the centre.
///
/// `a`, `b` and `k` are that world axis expressed in the surface frame
/// (`x̂·ê`, `ŷ·ê`, `ẑ·ê`). With `A = ‖(a, b)‖`, `φ = atan2(b, a)` and
/// `C = cos(u − φ)`, the component along `ê` is
/// `R·A·C + r·(A·C·cos v + k·sin v)`. For a fixed `C` the bracketed term is
/// `m·cos(v − ψ)` with `m = ‖(A·C, k)‖`, whose extremes over the `v` interval
/// are exact. Seen as a function of `C`, the maximum is a pointwise maximum of
/// affine functions plus an affine term — convex — so it is attained at an end
/// of `C`'s range; the minimum is concave for the same reason. Testing both
/// ends of `C` is therefore exact, not a sampling approximation.
///
/// A face that wraps both directions reduces to `R·A + r`, the tight bound for
/// a whole torus (and to `r` for a whole sphere).
fn ring_patch_axis_extent(
    a: f64,
    b: f64,
    k: f64,
    major: f64,
    minor: f64,
    (u0, u1): (f64, f64),
    (v0, v1): (f64, f64),
) -> (f64, f64) {
    let amp = a.hypot(b);
    let phi = b.atan2(a);
    let (c_lo, c_hi) = cos_range(u0 - phi, u1 - phi);

    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for c in [c_lo, c_hi] {
        let ring = amp * c;
        let m = ring.hypot(k);
        let psi = k.atan2(ring);
        let (g_lo, g_hi) = cos_range(v0 - psi, v1 - psi);
        let base = major * ring;
        lo = lo.min(minor.mul_add(m * g_lo, base));
        hi = hi.max(minor.mul_add(m * g_hi, base));
    }
    (lo, hi)
}

/// Exact `(min, max)` of `cos` over `[t0, t1]`.
///
/// The endpoints bound it unless the interval contains a peak (a multiple of
/// `2π`) or a trough (`π` plus a multiple).
fn cos_range(t0: f64, t1: f64) -> (f64, f64) {
    if t1 - t0 >= TAU {
        return (-1.0, 1.0);
    }
    let (e0, e1) = (t0.cos(), t1.cos());
    let mut lo = e0.min(e1);
    let mut hi = e0.max(e1);
    // The first `target + n·2π` at or above `t0`; the interval is shorter than
    // a period, so if that one overshoots `t1` no other lands inside either.
    let contains = |target: f64| target + TAU * ((t0 - target) / TAU).ceil() <= t1;
    if contains(0.0) {
        hi = 1.0;
    }
    if contains(PI) {
        lo = -1.0;
    }
    (lo, hi)
}

/// Sample edge midpoints along a face's outer wire to expand the AABB.
///
/// Returns `true` if any curved (non-Line) edges were found. For curved
/// edges (Circle, Ellipse, NurbsCurve), sampling at 0.25, 0.5, 0.75
/// captures the curvature.
fn sample_face_wire_midpoints(
    topo: &Topology,
    aabb: &mut Aabb3,
    face_id: brepkit_topology::face::FaceId,
) -> bool {
    let Ok(face) = topo.face(face_id) else {
        return false;
    };
    let Ok(wire) = topo.wire(face.outer_wire()) else {
        return false;
    };
    let mut has_curved = false;
    for oe in wire.edges() {
        let Ok(edge) = topo.edge(oe.edge()) else {
            continue;
        };
        if !matches!(edge.curve(), brepkit_topology::edge::EdgeCurve::Line) {
            has_curved = true;
        }
        let Ok(sv) = topo.vertex(edge.start()) else {
            continue;
        };
        let Ok(ev) = topo.vertex(edge.end()) else {
            continue;
        };
        let p_start = sv.point();
        let p_end = ev.point();
        let (t0, t1) = edge.curve().domain_with_endpoints(p_start, p_end);
        for &frac in &[0.25, 0.5, 0.75] {
            let t = t0 + (t1 - t0) * frac;
            let pt = edge.curve().evaluate_with_endpoints(t, p_start, p_end);
            aabb_include(aabb, pt);
        }
    }
    has_curved
}

/// Expand AABB for a cylinder face by projecting each vertex onto the
/// cylinder axis and adding the full radial extent at that axial position.
fn expand_cylinder_at_vertices(
    topo: &Topology,
    aabb: &mut Aabb3,
    face_id: brepkit_topology::face::FaceId,
    cyl: &brepkit_math::surfaces::CylindricalSurface,
) {
    let Ok(face) = topo.face(face_id) else {
        return;
    };
    let Ok(wire) = topo.wire(face.outer_wire()) else {
        return;
    };
    let axis = cyl.axis();
    let origin = cyl.origin();
    let r = cyl.radius();
    let rx = r * (1.0 - axis.x() * axis.x()).max(0.0).sqrt();
    let ry = r * (1.0 - axis.y() * axis.y()).max(0.0).sqrt();
    let rz = r * (1.0 - axis.z() * axis.z()).max(0.0).sqrt();
    for oe in wire.edges() {
        let Ok(edge) = topo.edge(oe.edge()) else {
            continue;
        };
        for vid in [edge.start(), edge.end()] {
            let Ok(v) = topo.vertex(vid) else {
                continue;
            };
            let rel = brepkit_math::vec::Vec3::new(
                v.point().x() - origin.x(),
                v.point().y() - origin.y(),
                v.point().z() - origin.z(),
            );
            let t = axis.dot(rel);
            let coa = Point3::new(
                origin.x() + axis.x() * t,
                origin.y() + axis.y() * t,
                origin.z() + axis.z() * t,
            );
            aabb_include(aabb, Point3::new(coa.x() - rx, coa.y() - ry, coa.z() - rz));
            aabb_include(aabb, Point3::new(coa.x() + rx, coa.y() + ry, coa.z() + rz));
        }
    }
}

/// Expand AABB for a cone face by computing each face vertex's radial
/// distance from the axis (the local cone radius at that axial slice),
/// then including a full circle of that radius at that slice.
fn expand_cone_at_vertices(
    topo: &Topology,
    aabb: &mut Aabb3,
    face_id: brepkit_topology::face::FaceId,
    cone: &brepkit_math::surfaces::ConicalSurface,
) {
    use brepkit_math::vec::Vec3;
    let Ok(face) = topo.face(face_id) else {
        return;
    };
    let Ok(wire) = topo.wire(face.outer_wire()) else {
        return;
    };
    let axis = cone.axis();
    let apex = cone.apex();
    // Axis-perpendicular projection scales for a full ring at slice centre.
    let sx = (1.0 - axis.x() * axis.x()).max(0.0).sqrt();
    let sy = (1.0 - axis.y() * axis.y()).max(0.0).sqrt();
    let sz = (1.0 - axis.z() * axis.z()).max(0.0).sqrt();
    for oe in wire.edges() {
        let Ok(edge) = topo.edge(oe.edge()) else {
            continue;
        };
        for vid in [edge.start(), edge.end()] {
            let Ok(v) = topo.vertex(vid) else {
                continue;
            };
            let rel = Vec3::new(
                v.point().x() - apex.x(),
                v.point().y() - apex.y(),
                v.point().z() - apex.z(),
            );
            let t = axis.dot(rel);
            let coa = Point3::new(
                apex.x() + axis.x() * t,
                apex.y() + axis.y() * t,
                apex.z() + axis.z() * t,
            );
            // Local radius is the perpendicular distance from axis to vertex.
            let perp = Vec3::new(
                rel.x() - axis.x() * t,
                rel.y() - axis.y() * t,
                rel.z() - axis.z() * t,
            );
            let r = perp.length();
            aabb_include(
                aabb,
                Point3::new(coa.x() - r * sx, coa.y() - r * sy, coa.z() - r * sz),
            );
            aabb_include(
                aabb,
                Point3::new(coa.x() + r * sx, coa.y() + r * sy, coa.z() + r * sz),
            );
        }
    }
}
