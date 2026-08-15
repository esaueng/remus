//! Bounding box computation for B-rep solids.

use std::collections::HashSet;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use brepkit_math::aabb::Aabb3;
use brepkit_math::nurbs::projection::{project_point_to_surface, project_point_to_surface_seeded};
use brepkit_math::nurbs::surface::NurbsSurface;
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
/// - **NURBS**: sparse interior grid sampling, over the face's *trimmed*
///   parameter box (see [`nurbs_patch_domain`])
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

        // NURBS: grid-sample the surface over the region the face is trimmed
        // to, not the whole knot domain — a face cut from the corner of a big
        // patch must not report the rest of the patch, same as the analytic
        // arms above.
        FaceSurface::Nurbs(nurbs) => {
            // The surface lies inside its control points' convex hull (the
            // weights are positive by construction), so if that hull is
            // already inside the box, no sample of this face can move it.
            // Skips the projection below outright — and the final box is the
            // same either way, since `aabb` only ever grows.
            if control_hull_within(nurbs, aabb) {
                return;
            }
            let dom = nurbs_patch_domain(topo, face_id, nurbs);
            let ((u_min, u_max), (v_min, v_max)) = (dom.u, dom.v);
            // Sampled closed, endpoints included: the extremes of a region sit
            // anywhere in its parameter box, commonly on an edge of it, and
            // that box is now the face's own rather than the whole surface's.
            // Interior-only sampling was tuned for the full domain, where the
            // boundary edges covered the rim; on a trimmed region it misses.
            let n_samples = 4;
            #[allow(clippy::cast_precision_loss)]
            for iu in 0..=n_samples {
                let u = u_min + (u_max - u_min) * (f64::from(iu) / f64::from(n_samples));
                for iv in 0..=n_samples {
                    let v = v_min + (v_max - v_min) * (f64::from(iv) / f64::from(n_samples));
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

/// Samples per boundary edge when recovering a NURBS face's region. Lower than
/// [`TRIM_SAMPLES_PER_EDGE`] because a NURBS surface has no closed-form
/// inversion: each sample costs a coarse grid search plus Newton (~4.5 us,
/// against an `atan2` on the analytic surfaces). 8 resolves a trim curve's
/// parameter span well inside [`NURBS_DOMAIN_PAD`].
const NURBS_TRIM_SAMPLES_PER_EDGE: usize = 8;

/// Maximum number of boundary samples one NURBS face may spend on recovery.
///
/// Per-edge sampling alone makes the cost scale with edge count, and a face
/// carved up by neighbouring features can carry dozens of edges — the same
/// per-edge density that costs 36 projections on a four-sided face costs 432 on
/// a forty-eight-sided one. Spreading a budget instead keeps a face's recovery
/// bounded, and costs such a face nothing: its own edge endpoints already
/// resolve the boundary finely.
const NURBS_TRIM_SAMPLE_BUDGET: usize = 64;

/// Evenly reduce boundary samples to the fixed NURBS inversion budget.
///
/// Sampling first and then reducing keeps coverage spread over the outer and
/// inner wires while placing a hard bound on both the duplicate filtering and
/// surface inversion performed by [`nurbs_patch_domain`].
fn limit_nurbs_trim_samples(samples: Vec<Point3>) -> Vec<Point3> {
    if samples.len() <= NURBS_TRIM_SAMPLE_BUDGET {
        return samples;
    }

    let last = samples.len() - 1;
    (0..NURBS_TRIM_SAMPLE_BUDGET)
        .map(|i| samples[i * last / (NURBS_TRIM_SAMPLE_BUDGET - 1)])
        .collect()
}

/// Newton tolerance for inverting a boundary sample onto its NURBS surface.
///
/// The value [`ParametricSurface::project_point`](brepkit_math::traits::ParametricSurface::project_point)
/// hardcodes, which is the solve this replaced.
const NURBS_PROJECT_TOL: f64 = 1e-7;

/// Floor for the residual a seeded inversion may leave, as a fraction of the
/// surface's own size.
///
/// It cannot be [`NURBS_PROJECT_TOL`]: a boundary sample is a point of the
/// *edge* curve, which lies on the face's surface only to the tolerance the
/// edge was built or imported at, so demanding the Newton tolerance itself
/// would reject every healthy-but-inexact edge and fall back on all of them —
/// slower than never having seeded at all. Nor can it be an absolute slack,
/// which would mean different things on a part drawn in metres and one drawn
/// in millimetres.
///
/// This is only the floor. How far a face's edges actually sit off its surface
/// is a property of that face, not a constant, so the real budget is learned
/// per face — see [`SeedBudget`].
const NURBS_SEED_FLOOR_FRAC: f64 = 1e-5;

/// Margin over a face's own observed edge-to-surface gap within which a seeded
/// inversion is still believed.
///
/// The failure being screened out is a seed whose basin led somewhere else on
/// the patch, which lands a good fraction of the patch away — far outside a
/// few times the gap the face's own edges already show.
const NURBS_SEED_SLACK: f64 = 4.0;

/// Slack added to each side of a recovered NURBS parameter span, as a fraction
/// of that span.
///
/// A region bounded by a closed loop has exactly its boundary's parameter box,
/// so the only error is that a finite set of samples can miss where the trim
/// curve reaches furthest between them. This covers that, and the residue of
/// Newton's distance-space convergence.
const NURBS_DOMAIN_PAD: f64 = 0.1;

/// The `(u, v)` rectangle a face occupies on its surface.
///
/// A span equal to the surface's full extent in that direction — a period of
/// `TAU` on the analytic surfaces, the knot domain on a NURBS one — means "the
/// boundary does not bound the face here": it wraps, or it degenerates to a
/// seam. Recovery returns exactly that whenever it cannot prove otherwise.
struct PatchDomain {
    u: (f64, f64),
    v: (f64, f64),
}

/// Sample every boundary edge of a face — outer wire and inner wires — at
/// [`TRIM_SAMPLES_PER_EDGE`] intervals, endpoints included.
fn face_boundary_samples(topo: &Topology, face_id: FaceId, per_edge: usize) -> Vec<Point3> {
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
            for i in 0..=per_edge {
                #[allow(clippy::cast_precision_loss)]
                let frac = (i as f64) / (per_edge as f64);
                let t = (t1 - t0).mul_add(frac, t0);
                pts.push(edge.curve().evaluate_with_endpoints(t, p_start, p_end));
            }
        }
    }
    pts
}

/// Total edges across a face's outer and inner wires.
fn face_boundary_edge_count(topo: &Topology, face_id: FaceId) -> usize {
    let Ok(face) = topo.face(face_id) else {
        return 0;
    };
    std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .filter_map(|wid| topo.wire(wid).ok())
        .map(|wire| wire.edges().len())
        .sum()
}

/// Choose a boundary sampling density without letting attacker-controlled
/// topology exceed the fixed trim-recovery budget.
fn nurbs_trim_samples_per_edge(edge_count: usize) -> Option<usize> {
    if edge_count > NURBS_TRIM_SAMPLE_BUDGET {
        return None;
    }
    Some(
        NURBS_TRIM_SAMPLE_BUDGET
            .checked_div(edge_count)
            .unwrap_or(NURBS_TRIM_SAMPLES_PER_EDGE)
            .clamp(1, NURBS_TRIM_SAMPLES_PER_EDGE),
    )
}

/// Recover the `(u, v)` rectangle a toroidal face occupies.
///
/// Both torus directions are periodic and free of degeneracies, so a face is
/// bounded in a direction exactly when its boundary leaves a gap there —
/// which is what [`compute_angular_range`] tests, returning the full period
/// when it finds none.
fn torus_patch_domain(topo: &Topology, face_id: FaceId, t: &ToroidalSurface) -> PatchDomain {
    // Inner wires bound holes, not the occupied patch. Their angular samples
    // can therefore describe the complement of the face and must never be
    // used to shrink a conservative box. Until domain recovery can classify
    // periodic complements, retain the full analytic extent for holed faces.
    if topo
        .face(face_id)
        .is_ok_and(|face| !face.inner_wires().is_empty())
    {
        return PatchDomain {
            u: (0.0, TAU),
            v: (0.0, TAU),
        };
    }
    let pts = face_boundary_samples(topo, face_id, TRIM_SAMPLES_PER_EDGE);
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
    // As for a torus, an inner loop encloses excluded geometry. Falling back
    // to the whole surface is conservative and prevents a hole near one side
    // of the sphere from hiding the occupied face on the opposite side.
    if topo
        .face(face_id)
        .is_ok_and(|face| !face.inner_wires().is_empty())
    {
        return PatchDomain {
            u: (0.0, TAU),
            v: (-FRAC_PI_2, FRAC_PI_2),
        };
    }
    let pts = face_boundary_samples(topo, face_id, TRIM_SAMPLES_PER_EDGE);
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

/// Whether every control point of `nurbs` already lies inside `aabb`.
///
/// A NURBS surface lies within the convex hull of its control points, and
/// [`NurbsSurface::new`](brepkit_math::nurbs::surface::NurbsSurface::new)
/// rejects non-positive weights, so this holds for rational surfaces too. When
/// it is true no point of the surface — trimmed or not — can grow the box.
fn control_hull_within(nurbs: &brepkit_math::nurbs::surface::NurbsSurface, aabb: &Aabb3) -> bool {
    nurbs.control_points().iter().flatten().all(|p| {
        p.x() >= aabb.min.x()
            && p.x() <= aabb.max.x()
            && p.y() >= aabb.min.y()
            && p.y() <= aabb.max.y()
            && p.z() >= aabb.min.z()
            && p.z() <= aabb.max.z()
    })
}

/// Recover the `(u, v)` box a NURBS face is trimmed to within its surface's
/// knot domain.
///
/// A NURBS surface has no closed-form inversion, so the boundary samples are
/// projected onto it. That is the whole cost of this function, and it buys the
/// same guarantee the analytic surfaces get for free: a region bounded by a
/// closed loop occupies exactly its boundary's parameter box.
///
/// A direction the surface closes on keeps its full domain. There the seam
/// bounds nothing — the face may wrap straight through it, and the two sides of
/// the domain are the same points in space, so projection cannot tell which one
/// a boundary sample came from. Same reasoning as a whole torus's seam, and the
/// same conservative answer.
fn nurbs_patch_domain(
    topo: &Topology,
    face_id: FaceId,
    nurbs: &brepkit_math::nurbs::surface::NurbsSurface,
) -> PatchDomain {
    let full_u = nurbs.domain_u();
    let full_v = nurbs.domain_v();
    let (closed_u, closed_v) = (nurbs.is_periodic_u(), nurbs.is_periodic_v());
    if closed_u && closed_v {
        return PatchDomain {
            u: full_u,
            v: full_v,
        };
    }

    let Some(per_edge) = nurbs_trim_samples_per_edge(face_boundary_edge_count(topo, face_id))
    else {
        // Sampling every edge would make projection and de-duplication costs
        // depend without bound on imported topology. The complete surface
        // domain is conservative and keeps bounding-box work bounded.
        return PatchDomain {
            u: full_u,
            v: full_v,
        };
    };
    let pts = limit_nurbs_trim_samples(face_boundary_samples(topo, face_id, per_edge));
    if pts.is_empty() {
        return PatchDomain {
            u: full_u,
            v: full_v,
        };
    }

    // Adjacent edges meet at a shared vertex, so a closed wire hands back each
    // of its corners twice — and at one sample per edge, which is what a
    // many-edged face gets, that is every second projection. Inverting a point
    // costs orders of magnitude more than comparing two, so drop the repeats.
    let mut distinct: Vec<Point3> = Vec::with_capacity(pts.len());
    for p in &pts {
        if !distinct
            .iter()
            .any(|q: &Point3| (*q - *p).length_squared() < 1e-14)
        {
            distinct.push(*p);
        }
    }

    // Walk the boundary carrying the last verified parameter forward as the
    // next Newton seed. `distinct` preserves wire order, so consecutive
    // entries are neighbours along the boundary and land within a step or two
    // of one another in parameter — which is the grid search's whole job, had
    // for free. A seed that leads elsewhere is caught and paid for below.
    let mut budget = SeedBudget::for_surface(nurbs);
    let mut seed: Option<(f64, f64)> = None;
    let (mut u_lo, mut u_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut v_lo, mut v_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &distinct {
        let Some((u, v)) = invert_boundary_sample(nurbs, *p, &mut seed, &mut budget) else {
            // Neither the seed nor the grid search could place this sample.
            // The recovered span would be missing a piece of the boundary, and
            // a span that is too small is a box that is too small, so give up
            // on trimming this face rather than report a region we cannot
            // stand behind.
            return PatchDomain {
                u: full_u,
                v: full_v,
            };
        };
        u_lo = u_lo.min(u);
        u_hi = u_hi.max(u);
        v_lo = v_lo.min(v);
        v_hi = v_hi.max(v);
    }

    PatchDomain {
        u: if closed_u {
            full_u
        } else {
            padded_span((u_lo, u_hi), full_u)
        },
        v: if closed_v {
            full_v
        } else {
            padded_span((v_lo, v_hi), full_v)
        },
    }
}

/// Invert one boundary sample onto its NURBS surface, seeding Newton from the
/// previous sample's parameters and carrying the answer forward.
///
/// The sample lies *on* the surface by construction — it is a point of the
/// face's own boundary — so a correct inversion leaves a residual near zero,
/// and one that does not is not this point's parameter. That is what makes a
/// seed safe to use at all: Newton solves perpendicularity rather than
/// minimising, so it settles wherever the seed's basin leads, and a seeded
/// answer is only ever as good as its seed. Accepted, the result is a verified
/// preimage; rejected, the call falls back to the grid search that ran here
/// before, so the recovered span is never worse for having tried.
///
/// Only verified parameters are carried forward — seeding the next solve from
/// an unchecked basin would let one bad inversion walk down the whole
/// boundary.
///
/// Returns `None` when even the grid search cannot place the sample.
/// [`ParametricSurface::project_point`](brepkit_math::traits::ParametricSurface::project_point)
/// answers the domain midpoint there, which would fold a point the face may
/// not contain into the span; the caller needs to know instead.
fn invert_boundary_sample(
    nurbs: &NurbsSurface,
    p: Point3,
    seed: &mut Option<(f64, f64)>,
    budget: &mut SeedBudget,
) -> Option<(f64, f64)> {
    let seeded = seed.and_then(|s| {
        project_point_to_surface_seeded(nurbs, p, NURBS_PROJECT_TOL, s)
            .ok()
            .filter(|proj| budget.accepts(proj.distance))
    });
    if let Some(proj) = seeded {
        *seed = Some((proj.u, proj.v));
        return Some((proj.u, proj.v));
    }

    let proj = project_point_to_surface(nurbs, p, NURBS_PROJECT_TOL).ok()?;
    budget.observe(proj.distance);
    *seed = budget.accepts(proj.distance).then_some((proj.u, proj.v));
    Some((proj.u, proj.v))
}

/// How large a residual a seeded inversion may leave on this face and still be
/// believed.
///
/// A fixed threshold cannot work. Set it at the Newton tolerance and every
/// face whose edges sit slightly off their surface — which is most imported
/// geometry — rejects every seed and pays for both paths, ending up slower
/// than never seeding. Set it loose enough for those and it stops screening
/// anything on a face whose edges are exact.
///
/// So it is learned. The grid search runs on the first sample of every face
/// and on every rejection, and what it leaves is this face's own edge-to-
/// surface gap, measured rather than assumed; the budget is a few times that,
/// never below a floor scaled to the patch. A seed that landed in the wrong
/// basin is a good fraction of the patch away and still fails it.
struct SeedBudget {
    limit: f64,
}

impl SeedBudget {
    /// Start from the floor alone: the patch's size times
    /// [`NURBS_SEED_FLOOR_FRAC`], before any edge has been measured.
    fn for_surface(nurbs: &NurbsSurface) -> Self {
        Self {
            limit: surface_extent(nurbs) * NURBS_SEED_FLOOR_FRAC,
        }
    }

    /// Fold in a residual left by the grid search — evidence of how far this
    /// face's edges genuinely lie off its surface.
    fn observe(&mut self, grid_residual: f64) {
        if grid_residual.is_finite() {
            self.limit = self.limit.max(grid_residual * NURBS_SEED_SLACK);
        }
    }

    fn accepts(&self, residual: f64) -> bool {
        residual <= self.limit
    }
}

/// Diagonal of the surface's control-point box — a cheap stand-in for how big
/// the patch is, and an upper bound on it, since the surface lies inside that
/// hull.
fn surface_extent(nurbs: &NurbsSurface) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in nurbs.control_points().iter().flatten() {
        for (axis, c) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
            lo[axis] = lo[axis].min(c);
            hi[axis] = hi[axis].max(c);
        }
    }
    if lo[0] > hi[0] {
        return 0.0;
    }
    (hi[0] - lo[0]).hypot(hi[1] - lo[1]).hypot(hi[2] - lo[2])
}

/// Widen a recovered parameter span by [`NURBS_DOMAIN_PAD`] on each side and
/// clamp it to `full`. A span that came back empty or inverted falls back to
/// the whole domain.
fn padded_span((lo, hi): (f64, f64), full: (f64, f64)) -> (f64, f64) {
    if !lo.is_finite() || !hi.is_finite() || hi < lo {
        return full;
    }
    let pad = (hi - lo) * NURBS_DOMAIN_PAD;
    ((lo - pad).max(full.0), (hi + pad).min(full.1))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nurbs_trim_sampling_is_bounded() {
        for edge_count in 0..=NURBS_TRIM_SAMPLE_BUDGET {
            let Some(per_edge) = nurbs_trim_samples_per_edge(edge_count) else {
                unreachable!("edge count within budget must be sampled");
            };
            assert!(edge_count.saturating_mul(per_edge + 1) <= 2 * NURBS_TRIM_SAMPLE_BUDGET);
        }
        assert!(nurbs_trim_samples_per_edge(NURBS_TRIM_SAMPLE_BUDGET + 1).is_none());
        assert!(nurbs_trim_samples_per_edge(usize::MAX).is_none());
    }

    #[test]
    fn nurbs_trim_samples_are_strictly_bounded_and_span_input() {
        let samples: Vec<_> = (0..1_024)
            .map(|i| Point3::new(f64::from(i), 0.0, 0.0))
            .collect();

        let limited = limit_nurbs_trim_samples(samples);

        assert_eq!(limited.len(), NURBS_TRIM_SAMPLE_BUDGET);
        assert_eq!(limited.first().map(|point| point.x()), Some(0.0));
        assert_eq!(limited.last().map(|point| point.x()), Some(1_023.0));
    }

    #[test]
    fn nurbs_trim_samples_below_budget_are_preserved() {
        let samples = vec![Point3::new(1.0, 2.0, 3.0), Point3::new(4.0, 5.0, 6.0)];

        assert_eq!(limit_nurbs_trim_samples(samples.clone()), samples);
    }
}
