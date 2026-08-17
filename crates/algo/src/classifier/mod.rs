//! Face classification -- determines if a sub-face is inside/outside
//! the opposing solid.
//!
//! Two strategies:
//! - **Analytic**: O(1) point-in-solid for convex analytic solids.
//! - **Ray cast**: Multi-ray fallback for general solids.

mod analytic;
mod ray_cast;

pub use analytic::{AnalyticClassifier, classify_analytic, try_build_analytic_classifier};
pub use ray_cast::{
    RayCastGeoms, classify_ray_cast, classify_ray_cast_cached, compute_solid_bbox,
    planar_face_polygons, point_in_face_3d, point_in_planar_region, ray_cast_inside_votes,
    ray_cast_inside_votes_cached,
};
pub(crate) use ray_cast::{largest_u_gap, u_in_gap};

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

use crate::builder::FaceClass;
use crate::error::AlgoError;

/// Classify a point relative to a solid -- dispatch to the best available
/// strategy.
///
/// Tries the analytic classifier first (O(1) for convex analytic solids),
/// then falls back to ray casting.
///
/// # Errors
///
/// Returns [`AlgoError::ClassificationFailed`] if classification is
/// indeterminate.
pub fn classify_point(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
) -> Result<FaceClass, AlgoError> {
    if let Some(class) = classify_analytic(topo, solid, point) {
        return Ok(class);
    }

    classify_ray_cast(topo, solid, point)
}

/// Like [`classify_point`], but reuses pre-collected ray-cast geometry for the
/// solid when available (`Some`).
///
/// The analytic fast path is tried first exactly as in [`classify_point`]; only
/// the ray-cast fallback consults the cache. Passing `None` reproduces
/// [`classify_point`] verbatim (geometry collected per call), so a caller that
/// failed to build the cache degrades to identical behaviour.
///
/// # Errors
///
/// Returns [`AlgoError::ClassificationFailed`] if classification is
/// indeterminate.
pub fn classify_point_cached(
    topo: &Topology,
    solid: SolidId,
    geoms: Option<&ray_cast::RayCastGeoms>,
    point: Point3,
) -> Result<FaceClass, AlgoError> {
    if let Some(class) = classify_analytic(topo, solid, point) {
        return Ok(class);
    }

    match geoms {
        Some(g) => ray_cast::classify_ray_cast_cached(g, point),
        None => classify_ray_cast(topo, solid, point),
    }
}

/// Classify a planar sub-face that is coincident-coplanar with a face of the
/// opposing solid by 2D containment, bypassing the unstable grazing ray-cast.
///
/// When a split sub-face's supporting plane is coincident (coplanar within
/// `tol`, ignoring normal sign) with a planar face of the opposing solid, the
/// sub-face's interior point necessarily lies *on* that opposing face's plane.
/// A cardinal ray-cast from such a point grazes the coincident cap and its wall
/// top-edges and can vote wrongly Inside (and a single interior sample is
/// itself unreliable on a thin corner wedge).
///
/// The override fires only for the *wholly-exterior wedge* signature: the
/// sub-face has at least one vertex strictly outside the opposing region and
/// **no** vertex strictly inside it (every vertex is outside or on the shared
/// boundary) — the clipped-away corner orphan whose only contact with the
/// opposing region is along the shared boundary.
///
/// To stay sound it additionally runs a *depth probe* at the wedge tip: a 2D
/// point outside the opposing face's region is outside the opposing *solid*
/// only when this coincident plane is the local outer boundary there. Stepping
/// off the plane to both sides of the tip and finding the solid absent on both
/// sides confirms the plane is a local boundary → the wedge is exterior
/// ([`FaceClass::Outside`]). If the solid persists on either side (a plane
/// shared with an interior feature, e.g. the honeycomb's stacked caps), the
/// genuinely-inside coincident face is left to the regular classifier.
///
/// Returns `None` when there is no coincident opposing face, the sub-face is
/// not a wholly-exterior wedge, or the depth probe finds the plane is internal.
///
/// # Errors
///
/// Returns [`AlgoError`] on a topology lookup failure.
#[allow(clippy::too_many_arguments)]
pub fn classify_coincident_coplanar(
    topo: &Topology,
    opposing_solid: SolidId,
    geoms: Option<&ray_cast::RayCastGeoms>,
    sub_face_id: remus_topology::face::FaceId,
    sub_normal: Vec3,
    sub_d: f64,
    interior_hint: Option<Point3>,
    tol: remus_math::tolerance::Tolerance,
) -> Result<Option<FaceClass>, AlgoError> {
    let plane_tol = tol.linear.max(1e-7);
    let n_tol = 1e-6_f64;
    let faces = remus_topology::explorer::solid_faces(topo, opposing_solid)?;
    for fid in faces {
        let face = topo.face(fid)?;
        let FaceSurface::Plane {
            normal: fn_raw,
            d: fd_raw,
        } = face.surface()
        else {
            continue;
        };
        // The stored (normal, d) define the plane regardless of face
        // orientation; coincidence is sign-agnostic.
        let fnv = *fn_raw;
        let coplanar_same =
            (fnv - sub_normal).length() < n_tol && (fd_raw - sub_d).abs() < plane_tol;
        let coplanar_flip =
            (fnv + sub_normal).length() < n_tol && (fd_raw + sub_d).abs() < plane_tol;
        if !(coplanar_same || coplanar_flip) {
            continue;
        }
        let Some((outer, holes, region_normal)) = planar_face_polygons(topo, fid)? else {
            continue;
        };
        let Some(sub_verts) = sub_face_outer_vertices(topo, sub_face_id)? else {
            return Ok(None);
        };

        // Classify each sub-face vertex against the opposing region with a
        // boundary band: a vertex on the shared boundary (within `plane_tol`)
        // is neither strictly inside nor strictly outside. Track the deepest
        // strictly-outside vertex (farthest from the opposing boundary) — that
        // is the wedge tip, the most reliable place to probe.
        // A hole rim is polygonised as an inscribed polygon, so a vertex lying
        // on the TRUE arc sits up to the sagitta inside that polygon. Widen the
        // "on the boundary" band for holes to the polygon's own chord length,
        // which bounds the sagitta — otherwise an annular cap dropped into a
        // matching bore reports every rim vertex as strictly inside the
        // opposing region and the straddler guard defers a face that does not
        // overlap it at all. The band only ever demotes a vertex to "on", and
        // the decision still rests on the interior probe below.
        let hole_bands: Vec<f64> = holes
            .iter()
            .map(|h| {
                let n = h.len();
                let max_chord = (0..n)
                    .map(|i| (h[(i + 1) % n] - h[i]).length())
                    .fold(0.0_f64, f64::max);
                // Sagitta of the widest chord: how far inside the inscribed
                // polygon a point on the true arc can sit. Bound it from the
                // polygon alone via c^2 / 8R, taking R as the mean centroid
                // distance (exact for a circular rim). The chord ITSELF is far
                // too wide a band — on a r=8.8 rim it is 4.6mm, which would
                // swallow genuinely-interior vertices metres from the arc.
                #[allow(clippy::cast_precision_loss)]
                let inv_n = 1.0 / n as f64;
                let cx = h.iter().map(|p| p.x()).sum::<f64>() * inv_n;
                let cy = h.iter().map(|p| p.y()).sum::<f64>() * inv_n;
                let cz = h.iter().map(|p| p.z()).sum::<f64>() * inv_n;
                let centroid = Point3::new(cx, cy, cz);
                let radius = h.iter().map(|p| (*p - centroid).length()).sum::<f64>() * inv_n;
                if radius <= plane_tol {
                    return plane_tol;
                }
                // 1.5x for polygons that are not exactly circular.
                1.5 * max_chord * max_chord / (8.0 * radius)
            })
            .collect();
        let near_hole_rim = |p: Point3| -> bool {
            holes.iter().zip(&hole_bands).any(|(h, &band)| {
                dist_to_polygon_boundary(p, h, &region_normal) <= band.max(plane_tol)
            })
        };

        let mut any_strictly_inside = false;
        let mut deepest_outside: Option<(f64, Point3)> = None;
        let mut all_verts_on_rim = true;
        for &v in &sub_verts {
            let dist = dist_to_polygon_boundary(v, &outer, &region_normal);
            if dist <= plane_tol {
                continue;
            }
            if near_hole_rim(v) {
                continue;
            }
            all_verts_on_rim = false;
            if point_in_planar_region(v, &outer, &holes, &region_normal) {
                any_strictly_inside = true;
            } else if deepest_outside.is_none_or(|(d, _)| dist > d) {
                deepest_outside = Some((dist, v));
            }
        }

        // A sub-face whose whole boundary runs along the opposing region's rim
        // yields no wedge tip: an annular cap dropped into a matching bore has
        // every rim vertex ON the shared circle, and the opposing hole is
        // polygonised as an inscribed polygon, so those vertices read as inside
        // the region by up to the sagitta. Its interior point settles it — it
        // sits mid-material, far from either rim, where containment is not a
        // near-tie.
        let mut tip_is_interior = false;
        let deepest_outside = deepest_outside.or_else(|| {
            // Only for the flush-rim signature: EVERY boundary vertex rides the
            // opposing region's rim, so the sub-face abuts it without crossing
            // it. A sub-face with vertices genuinely off the rim is a different
            // configuration and stays with the wedge logic.
            if !all_verts_on_rim {
                return None;
            }
            let hint = interior_hint?;
            if point_in_planar_region(hint, &outer, &holes, &region_normal) {
                return None;
            }
            let clearance = std::iter::once(&outer)
                .chain(holes.iter())
                .map(|poly| dist_to_polygon_boundary(hint, poly, &region_normal))
                .fold(f64::INFINITY, f64::min);
            tip_is_interior = clearance > plane_tol;
            tip_is_interior.then_some((clearance, hint))
        });

        // Wholly-exterior wedge: outside-or-on everywhere, with real exterior
        // extent. A straddler (any strictly-inside vertex) is deferred.
        let Some((depth, tip)) = deepest_outside else {
            return Ok(None);
        };
        if any_strictly_inside {
            return Ok(None);
        }

        // Depth probe: a 2D point outside the opposing face's region is outside
        // the opposing *solid* only if this coincident plane is the local outer
        // boundary there — i.e. stepping off the plane to *both* sides leaves
        // the solid. (A plane shared with an interior feature, e.g. the
        // honeycomb's stacked caps, has solid on one side → defer to ray-cast,
        // which correctly keeps the genuinely-inside coincident face.)
        //
        // The wedge tip sits at the sub-face's outermost corner, which lies on
        // the shared walls — ray-cast grazes there. Nudge the probe location
        // off the tip toward the wedge centroid so it clears the walls, while
        // keeping it strictly outside the opposing 2D region.
        let nlen = region_normal.length();
        if nlen < 1e-12 {
            return Ok(None);
        }
        let np = region_normal * (1.0 / nlen);
        let (mut cx, mut cy, mut cz) = (0.0, 0.0, 0.0);
        for &v in &sub_verts {
            cx += v.x();
            cy += v.y();
            cz += v.z();
        }
        let inv = 1.0 / sub_verts.len() as f64;
        let centroid = Point3::new(cx * inv, cy * inv, cz * inv);
        let probe = (100.0 * plane_tol).max(1e-3);
        // Candidate probe locations along tip → centroid. The centroid fractions
        // cover the wedge from rim to interior (a partially-internal coincident
        // plane can persist at either end — the honeycomb's stacked cap persists
        // near the RIM); the small ABSOLUTE nudges scaled to the wedge's own
        // outside-extent `depth` stay near the tip inside the band and are the
        // ONLY valid probes on a thin annulus (a ~1.2mm lip on a ~125mm face),
        // where every centroid fraction jumps clear across the band into the hole
        // (the opposing 2D region) and is rejected — without them the band face
        // found no valid probe and was dropped.
        let mut candidates: Vec<Point3> = Vec::with_capacity(7);
        // A tip taken from the sub-face's OWN interior (the flush-rim case: an
        // annular cap dropped into a matching bore) already sits mid-material,
        // clear of both rims — probe it where it stands. A wedge corner does
        // not, and still needs the nudges below.
        if tip_is_interior {
            candidates.push(tip);
        }
        for frac in [0.25_f64, 0.4, 0.55] {
            candidates.push(tip + (centroid - tip) * frac);
        }
        let dir = centroid - tip;
        let dl = dir.length();
        if dl > 1e-12 {
            let dir_unit = dir * (1.0 / dl);
            for scale in [0.5_f64, 0.25, 0.1] {
                candidates.push(tip + dir_unit * (depth * scale).min(0.9 * dl));
            }
        }

        // Decide from ALL valid probes, order-independently: if ANY strictly-
        // outside probe finds the opposing solid persisting on a side, the plane
        // is internal there → defer (keep). Only when at least one probe is valid
        // and NONE show persistence is the plane a genuine local outer boundary →
        // Outside. (A first-valid-wins scan was order-fragile: it could accept a
        // both-sides-empty rim probe before reaching the honeycomb cap's interior
        // persistence, or vice-versa.)
        let mut any_valid = false;
        for probe_xy in candidates {
            // Must still be strictly outside the opposing region and clear of
            // its boundary, else the probe is meaningless.
            if point_in_planar_region(probe_xy, &outer, &holes, &region_normal)
                || dist_to_polygon_boundary(probe_xy, &outer, &region_normal) <= probe
            {
                continue;
            }
            any_valid = true;
            let probe_a = probe_xy + np * probe;
            let probe_b = probe_xy - np * probe;
            let (av, bv) = match geoms {
                Some(g) => (
                    ray_cast::ray_cast_inside_votes_cached(g, probe_a)?,
                    ray_cast::ray_cast_inside_votes_cached(g, probe_b)?,
                ),
                None => (
                    ray_cast_inside_votes(topo, opposing_solid, probe_a)?,
                    ray_cast_inside_votes(topo, opposing_solid, probe_b)?,
                ),
            };
            if av >= 2 || bv >= 2 {
                // Solid persists on a side: internal plane → keep (defer).
                return Ok(None);
            }
        }
        return Ok(any_valid.then_some(FaceClass::Outside));
    }
    Ok(None)
}

/// Minimum distance from `p` to the closed polyline `poly` (edges + wrap).
fn dist_to_polygon_boundary(p: Point3, poly: &[Point3], _normal: &Vec3) -> f64 {
    let n = poly.len();
    if n < 2 {
        return f64::INFINITY;
    }
    let mut best = f64::INFINITY;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let ab = b - a;
        let len2 = ab.dot(ab);
        let t = if len2 > 1e-18 {
            ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let proj = a + ab * t;
        best = best.min((p - proj).length());
    }
    best
}

/// Collect a planar sub-face's outer-wire vertices (3D), de-duplicated.
fn sub_face_outer_vertices(
    topo: &Topology,
    face_id: remus_topology::face::FaceId,
) -> Result<Option<Vec<Point3>>, AlgoError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut verts = Vec::new();
    for oe in wire.edges() {
        let e = topo.edge(oe.edge())?;
        verts.push(topo.vertex(e.start())?.point());
        // Walk curved edges. A boundary that is ONE closed circle — an annular
        // cap's rim, a disc — has start == end, so endpoints alone give two
        // coincident points and the caller gives up on a face that is
        // perfectly classifiable. Sampling the arc yields a real polygon, and
        // its centroid is the circle centre, which is what the wedge probe
        // needs to step inward from the rim.
        if !matches!(e.curve(), remus_topology::edge::EdgeCurve::Line) {
            let sp = topo.vertex(e.start())?.point();
            let ep = topo.vertex(e.end())?.point();
            let (t0, t1) = e.curve().domain_with_endpoints(sp, ep);
            for k in 1..CURVE_SAMPLES {
                let t = f64::from(k).mul_add((t1 - t0) / f64::from(CURVE_SAMPLES), t0);
                verts.push(e.curve().evaluate_with_endpoints(t, sp, ep));
            }
        }
        verts.push(topo.vertex(e.end())?.point());
    }
    if verts.len() < 3 {
        return Ok(None);
    }
    Ok(Some(verts))
}

/// Points used to polygonise a curved boundary edge.
const CURVE_SAMPLES: u32 = 12;
