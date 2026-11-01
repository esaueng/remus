//! Phase EF: Edge-face intersection detection.
//!
//! For each (edge, face) pair across solids, finds points where the
//! edge crosses or touches the face surface. Records EF interferences
//! and adds extra paves to the edge for later splitting.

use remus_math::det_hash::{DetHashMap, DetHashSet};

use crate::builder::classify_2d::{distance_to_polygon_boundary, point_in_polygon_2d};
use crate::builder::plane_frame::PlaneFrame;
use crate::ds::{GfaArena, Interference, Pave};
use crate::error::AlgoError;
use remus_math::aabb::Aabb3;
use remus_math::nurbs::projection::{SurfaceSeedGrid, project_point_to_surface_with_grid};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;

use super::helpers::{add_pave_to_edge, find_nearby_pave_vertex as find_nearby_vertex};

#[path = "phase_ef/line_sphere.rs"]
mod line_sphere;

/// Number of samples along each edge for sign-change detection.
const N_SAMPLES: usize = 64;

/// Newton tolerance for NURBS point inversion here, matching what
/// [`ParametricSurface::project_point`](remus_math::traits::ParametricSurface::project_point)
/// hardcodes — the call the pre-gridded path stands in for.
const NURBS_PROJECT_TOL: f64 = 1e-7;

/// Number of samples per boundary edge for face containment polygons.
const N_BOUNDARY_SAMPLES: usize = 32;

/// Detect edge-face intersections between the two solids.
///
/// Checks edges of A against faces of B, and edges of B against
/// faces of A. When an edge crosses a face surface (within tolerance),
/// an EF interference is recorded and an extra pave is added to the
/// edge's pave block.
///
/// # Errors
///
/// Returns [`AlgoError`] if any topology lookup fails.
pub fn perform(
    topo: &mut Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
) -> Result<(), AlgoError> {
    perform_with(topo, solid_a, solid_b, tol, arena, true)
}

/// [`perform`], with the gates that prune analytic pairs before their
/// sampled scan switchable so tests can prove they change nothing but the
/// work done.
pub(super) fn perform_with(
    topo: &mut Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
    analytic_gates: bool,
) -> Result<(), AlgoError> {
    perform_inner(
        topo,
        solid_a,
        solid_b,
        tol,
        arena,
        analytic_gates,
        &mut FaceContainments::default(),
    )
}

/// Each face's sampled containment, built on the face's first use.
///
/// A containment depends only on the face's boundary and the tolerance, and
/// phase EF only adds the vertices its crossings mint, never touching a face
/// boundary. One cache therefore serves a whole N-way EF pass, where each
/// solid's faces were sampled again for every solid pair they are in.
#[derive(Default)]
pub(super) struct FaceContainments(DetHashMap<FaceId, FaceContainment>);

/// [`perform`] reading face containments through `containments`.
pub(super) fn perform_cached(
    topo: &mut Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
    containments: &mut FaceContainments,
) -> Result<(), AlgoError> {
    perform_inner(topo, solid_a, solid_b, tol, arena, true, containments)
}

fn perform_inner(
    topo: &mut Topology,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
    arena: &mut GfaArena,
    analytic_gates: bool,
    containments: &mut FaceContainments,
) -> Result<(), AlgoError> {
    let _phase = crate::perf::phase(crate::perf::PhaseKind::Ef);
    let bbox_a = crate::classifier::compute_solid_bbox(topo, solid_a)?;
    let bbox_b = crate::classifier::compute_solid_bbox(topo, solid_b)?;
    if !bbox_a
        .expanded(tol.linear)
        .intersects(bbox_b.expanded(tol.linear))
    {
        log::debug!("EF: solids are disjoint, skipping");
        return Ok(());
    }

    let edges_a = remus_topology::explorer::solid_edges(topo, solid_a)?;
    let edges_b = remus_topology::explorer::solid_edges(topo, solid_b)?;
    let faces_a = remus_topology::explorer::solid_faces(topo, solid_a)?;
    let faces_b = remus_topology::explorer::solid_faces(topo, solid_b)?;

    super::helpers::validate_edge_domains(topo, &edges_a, "edge-face interference")?;
    super::helpers::validate_edge_domains(topo, &edges_b, "edge-face interference")?;

    // Collect face boundary edge sets to skip edges that are already
    // on the face boundary.
    let face_boundary_edges_b = collect_face_boundary_edges(topo, &faces_b)?;
    let face_boundary_edges_a = collect_face_boundary_edges(topo, &faces_a)?;

    check_edge_face_pairs(
        topo,
        &edges_a,
        &faces_b,
        &face_boundary_edges_b,
        tol,
        arena,
        analytic_gates,
        containments,
    )?;
    check_edge_face_pairs(
        topo,
        &edges_b,
        &faces_a,
        &face_boundary_edges_a,
        tol,
        arena,
        analytic_gates,
        containments,
    )?;

    Ok(())
}

/// Collect the set of boundary edge IDs for each face.
fn collect_face_boundary_edges(
    topo: &Topology,
    faces: &[FaceId],
) -> Result<Vec<DetHashSet<EdgeId>>, AlgoError> {
    let mut result = Vec::with_capacity(faces.len());
    for &fid in faces {
        let edges = remus_topology::explorer::face_edges(topo, fid)?;
        result.push(edges.into_iter().collect());
    }
    Ok(result)
}

/// Spatial containment test for a face, built from sampled boundary edges.
///
/// Surface crossings are found against infinite surfaces; this rejects
/// crossing points that lie outside the trimmed face region.
struct FaceContainment {
    bbox: Option<Aabb3>,
    planar: Option<PlanarContainment>,
    /// Chart-polygon containment for a NURBS face that is open in both
    /// directions. A curved face's sampled boundary box is not conservative,
    /// so such faces used to accept EVERY crossing of their carrier — a peg
    /// cap rim crossing a converted bar wall's carrier 3 mm above the wall
    /// paved a vertex in open space.
    nurbs: Option<NurbsContainment>,
}

struct NurbsContainment {
    surface: remus_math::nurbs::surface::NurbsSurface,
    grid: SurfaceSeedGrid,
    /// The plane the (coplanar) control net lies in, for exact crossings.
    /// `None` for a recognized exact-rational cylinder wall, whose signed
    /// distance is measured against the recovered analytic cylinder instead.
    plane: Option<(Vec3, f64)>,
    /// Recovered analytic cylinder for a recognized converted wall; the
    /// signed distance is radial (`|p - axis| - r`), exact for Line edges.
    wall: Option<remus_math::surfaces::CylindricalSurface>,
    /// Outer outline inverted into the surface's `(u, v)` chart.
    polygon: Vec<Point2>,
    /// Inner-wire outlines in the same chart.
    holes: Vec<Vec<Point2>>,
    /// Model-space boundary sagitta margin; converted to chart units at the
    /// tested point through the local metric.
    margin: f64,
}

impl NurbsContainment {
    /// Chart coordinates of `pt` and the length of one chart unit there.
    fn invert(&self, pt: Point3) -> Option<(Point2, f64)> {
        let proj =
            project_point_to_surface_with_grid(&self.surface, pt, NURBS_PROJECT_TOL, &self.grid)
                .ok()?;
        let d = self.surface.derivatives(proj.u, proj.v, 1);
        let scale = d[1][0].length().min(d[0][1].length());
        let scale = if scale.is_finite() && scale > 1e-12 {
            scale
        } else {
            1.0
        };
        Some((Point2::new(proj.u, proj.v), scale))
    }

    fn accepts(&self, pt: Point3) -> bool {
        // Conservative keep when the inversion fails, as the curved-face
        // fallback always was.
        let Some((uv, scale)) = self.invert(pt) else {
            return true;
        };
        let margin = self.margin / scale;
        let within_outer = point_in_polygon_2d(uv, &self.polygon)
            || distance_to_polygon_boundary(uv, &self.polygon) <= margin;
        if !within_outer {
            return false;
        }
        !self.holes.iter().any(|hole| {
            point_in_polygon_2d(uv, hole) && distance_to_polygon_boundary(uv, hole) > margin
        })
    }
}

struct PlanarContainment {
    frame: PlaneFrame,
    polygon: Vec<Point2>,
    /// Inner-wire outlines, in the same frame as `polygon`. A point strictly
    /// inside one of these is in a HOLE, so it is not on the face.
    holes: Vec<Vec<Point2>>,
    margin: f64,
}

impl FaceContainment {
    fn accepts(&self, pt: Point3) -> bool {
        if self
            .bbox
            .as_ref()
            .is_some_and(|bbox| !bbox.contains_point(pt))
        {
            return false;
        }
        if let Some(nurbs) = &self.nurbs {
            return nurbs.accepts(pt);
        }
        let Some(planar) = &self.planar else {
            return true;
        };
        let p2 = planar.frame.project(pt);
        let within_outer = point_in_polygon_2d(p2, &planar.polygon)
            || distance_to_polygon_boundary(p2, &planar.polygon) <= planar.margin;
        if !within_outer {
            return false;
        }
        // A face with holes is not the disc its outer wire bounds. Without
        // this, the flange rim's z=10 cap (an annulus r24..45) accepted the
        // hub bore's seam at r=12 — a point in open space — and paved a
        // spurious vertex that split the bore seam in two.
        //
        // Points ON a hole rim stay accepted: that rim is real face boundary,
        // and a tool edge genuinely meeting it must still pave. Only the
        // strict interior, beyond the same sagitta margin the outer test
        // uses, is rejected.
        !planar.holes.iter().any(|hole| {
            point_in_polygon_2d(p2, hole) && distance_to_polygon_boundary(p2, hole) > planar.margin
        })
    }
}

/// Sample one wire into an ordered 3D outline, honouring traversal direction
/// and dropping the duplicate closing vertex.
///
/// Returns the outline and the longest chord contributed by a CURVED edge.
/// Only curved edges contribute to that sagitta margin: a straight Line edge's
/// sampled chords coincide with the edge exactly (zero sagitta), so it must not
/// inflate the chord. Basing the margin on a long straight edge would
/// over-extend a thin face's boundary band (a 123mm-wide × 1mm-tall ramp strip
/// got a 1.9mm margin, admitting EF crossings well outside it — the 3×3
/// scoop+label lip-corner fallback).
fn sample_wire_outline(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
    tol: Tolerance,
) -> Result<(Vec<Point3>, f64), AlgoError> {
    let mut max_boundary_error = 0.0_f64;
    let wire = topo.wire(wire_id)?;
    let mut points = Vec::with_capacity(wire.edges().len() * (N_BOUNDARY_SAMPLES + 1));
    let mut prev: Option<Point3> = None;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let start_pos = topo.vertex(edge.start())?.point();
        let end_pos = topo.vertex(edge.end())?.point();
        let (t0, t1) = super::helpers::authoritative_edge_domain(
            edge,
            oe.edge(),
            "edge-face boundary sampling",
        )?;
        let is_curved = !matches!(edge.curve(), EdgeCurve::Line);
        let n = N_BOUNDARY_SAMPLES;
        let mut prev_t: Option<f64> = None;
        // Sample inclusive of the edge's end vertex (0..=n) so the closing
        // segment of a closed wire reaches the true endpoint; consecutive
        // edges share a vertex, so dedup against the previous point.
        for i in 0..=n {
            let frac = i as f64 / n as f64;
            let frac = if oe.is_forward() { frac } else { 1.0 - frac };
            let t = t0 + (t1 - t0) * frac;
            let pt = edge.curve().evaluate_with_endpoints(t, start_pos, end_pos);
            if let Some(p) = prev {
                if (pt - p).length() <= tol.linear {
                    prev_t = Some(t);
                    continue;
                }
                if is_curved {
                    let error = if matches!(edge.curve(), EdgeCurve::Circle(_)) {
                        let mid = edge.curve().evaluate_with_endpoints(
                            f64::midpoint(prev_t.unwrap_or(t), t),
                            start_pos,
                            end_pos,
                        );
                        let chord = pt - p;
                        let chord_len_sq = chord.length_squared();
                        if chord_len_sq <= tol.linear * tol.linear {
                            0.0
                        } else {
                            let along = ((mid - p).dot(chord) / chord_len_sq).clamp(0.0, 1.0);
                            (mid - (p + chord * along)).length()
                        }
                    } else {
                        // A half-chord is conservative for boundary types whose
                        // curvature extrema are not available here.
                        (pt - p).length() * 0.5
                    };
                    max_boundary_error = max_boundary_error.max(error);
                }
            }
            prev = Some(pt);
            prev_t = Some(t);
            points.push(pt);
        }
    }
    // The last edge's end vertex coincides with the first edge's start
    // vertex on a closed wire; drop the duplicate so the closing polygon
    // segment isn't degenerate.
    if points.len() >= 2
        && let (Some(&first), Some(&last)) = (points.first(), points.last())
        && (last - first).length() <= tol.linear
    {
        points.pop();
    }
    Ok((points, max_boundary_error))
}

/// Sample a face's boundary into an AABB plus, for planar faces, in-plane
/// outer-wire and hole polygons for exact containment testing.
fn build_face_containment(
    topo: &Topology,
    fid: FaceId,
    tol: Tolerance,
) -> Result<FaceContainment, AlgoError> {
    let face = topo.face(fid)?;
    let surface = face.surface().clone();

    // An analytic curved face takes no containment from its outline (a
    // sampled boundary box is not conservative on a curved patch; see the
    // end of this function), so sampling it is wasted: every hole wall of a
    // drilled plate paid for it on every cut. The sampling cannot fail
    // either — `perform` validated every edge domain of both solids first —
    // so returning early changes nothing.
    if matches!(
        surface,
        FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
    ) {
        return Ok(FaceContainment {
            bbox: None,
            planar: None,
            nurbs: None,
        });
    }

    crate::perf::bump_face_outline_sampling();
    let (outer_points, mut max_boundary_error) = sample_wire_outline(topo, face.outer_wire(), tol)?;
    let mut all_points = outer_points.clone();

    // Hole outlines get the same treatment as the outer wire — they are face
    // boundary too, and their sagitta feeds the same margin.
    let mut hole_points: Vec<Vec<Point3>> = Vec::new();
    for &inner_wid in face.inner_wires() {
        let (pts, boundary_error) = sample_wire_outline(topo, inner_wid, tol)?;
        max_boundary_error = max_boundary_error.max(boundary_error);
        all_points.extend_from_slice(&pts);
        if pts.len() >= 3 {
            hole_points.push(pts);
        }
    }

    let Some(bbox) = Aabb3::try_from_points(all_points) else {
        return Ok(FaceContainment {
            bbox: None,
            planar: None,
            nurbs: None,
        });
    };
    let diag = (bbox.max - bbox.min).length();

    if let FaceSurface::Plane { normal, .. } = &surface {
        if outer_points.len() >= 3 {
            // Sampled chords undercut curved boundaries. Circle segments use
            // their measured midpoint sagitta; other curves retain the
            // conservative half-chord bound. Using half a circle chord here
            // inflated an r=30.4 disc by nearly 3 mm, so a nearby r=32.9
            // cylinder seam was falsely classified inside the disc and split
            // at the blind-bore floor.
            let margin = max_boundary_error.max(tol.linear * 10.0);
            let frame = PlaneFrame::from_normal_and_point(*normal, outer_points[0]);
            let polygon: Vec<Point2> = outer_points.iter().map(|&p| frame.project(p)).collect();
            let holes: Vec<Vec<Point2>> = hole_points
                .iter()
                .map(|pts| pts.iter().map(|&p| frame.project(p)).collect())
                .collect();
            return Ok(FaceContainment {
                bbox: Some(bbox.expanded(margin)),
                planar: Some(PlanarContainment {
                    frame,
                    polygon,
                    holes,
                    margin,
                }),
                nurbs: None,
            });
        }
        return Ok(FaceContainment {
            bbox: Some(bbox.expanded((diag * 0.5).max(tol.linear * 10.0))),
            planar: None,
            nurbs: None,
        });
    }

    // A PLANAR NURBS carrier (a coplanar control net) has an affine chart:
    // its outline inverts to a polygon there, tested like the planar one.
    // Any failed inversion declines to the accept-everything fallback below.
    // A recognized exact-rational cylinder wall gets the same chart-polygon
    // containment PLUS an exact signed distance (radial, against the
    // recovered analytic cylinder) for transversal crossing detection.
    // Genuinely curved NURBS faces keep that fallback on purpose: paving
    // their crossings fed an imported freeform body's splitter sections it
    // could not consume.
    if let FaceSurface::Nurbs(nurbs) = &surface
        && outer_points.len() >= 3
    {
        let planar = super::helpers::planar_nurbs_as_plane(&surface, tol).and_then(|p| match p {
            FaceSurface::Plane { normal, d } => Some((normal, d)),
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => None,
        });
        let wall = super::helpers::rational_cylinder_wall(nurbs, tol);
        if planar.is_some() || wall.is_some() {
            let grid = SurfaceSeedGrid::for_surface(nurbs);
            let invert = |pts: &[Point3]| -> Option<Vec<Point2>> {
                pts.iter()
                    .map(|&p| {
                        project_point_to_surface_with_grid(nurbs, p, NURBS_PROJECT_TOL, &grid)
                            .ok()
                            .map(|proj| Point2::new(proj.u, proj.v))
                    })
                    .collect()
            };
            if let Some(polygon) = invert(&outer_points)
                && let Some(holes) = hole_points
                    .iter()
                    .map(|pts| invert(pts))
                    .collect::<Option<Vec<_>>>()
            {
                // A wall and a plane reading are mutually exclusive: the
                // wall net is curved, so coplanar detection must have
                // declined.
                debug_assert!(planar.is_none() || wall.is_none());
                return Ok(FaceContainment {
                    bbox: None,
                    planar: None,
                    nurbs: Some(NurbsContainment {
                        surface: nurbs.clone(),
                        grid: SurfaceSeedGrid::for_surface(nurbs),
                        plane: planar,
                        wall,
                        polygon,
                        holes,
                        margin: max_boundary_error.max(tol.linear * 10.0),
                    }),
                });
            }
        }
    }

    // A sampled boundary box is not conservative for a curved surface patch.
    Ok(FaceContainment {
        bbox: None,
        planar: None,
        nurbs: None,
    })
}

/// Check each edge against each face.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
fn check_edge_face_pairs(
    topo: &mut Topology,
    edges: &[EdgeId],
    faces: &[FaceId],
    face_boundary_edges: &[DetHashSet<EdgeId>],
    tol: Tolerance,
    arena: &mut GfaArena,
    analytic_gates: bool,
    cache: &mut FaceContainments,
) -> Result<(), AlgoError> {
    // Surface crossings are found against INFINITE surfaces; without a bounds
    // check an edge "crosses" a face far outside its trimmed region, creating
    // spurious paves that propagate bogus edge splits. The containment test
    // bounds-checks every crossing against the face's sampled boundary (bbox
    // for all faces, in-plane outer + inner-wire polygon for planar faces).
    for &fid in faces {
        if let std::collections::hash_map::Entry::Vacant(slot) = cache.0.entry(fid) {
            slot.insert(build_face_containment(topo, fid, tol)?);
        }
    }
    let containments: Vec<&FaceContainment> =
        faces.iter().filter_map(|fid| cache.0.get(fid)).collect();
    debug_assert_eq!(containments.len(), faces.len());

    // Pre-expand each face's containment AABB by the linear tolerance so the
    // broad-phase reject below is conservative (never skips a real crossing,
    // which by definition lies inside the face's boundary region).
    let face_aabbs: Vec<Option<Aabb3>> = containments
        .iter()
        .map(|c| c.bbox.map(|bbox| bbox.expanded(tol.linear)))
        .collect();

    // Inverting a point onto a NURBS surface starts with an 81-point coarse
    // grid, and that grid depends only on the surface — yet the loop below
    // rebuilds it for every sample of every edge against the same face. On a
    // fused bin socket that is ~77% of the phase's time. Evaluate each face's
    // grid once here and the whole nest reuses it; the seed it yields is the
    // same node, so every crossing is unchanged.
    let mut seed_grids: Vec<Option<SurfaceSeedGrid>> = Vec::with_capacity(faces.len());
    // Conservative box of each NURBS carrier: the convex hull of the control
    // net contains every point `distance_to_surface` can measure against
    // (the projection clamps to the surface domain), so a curved edge whose
    // own conservative box stays farther than the crossing thresholds from
    // it can produce no crossing, no tangency and no on-surface verdict.
    // Both boxes carry a coordinate-scaled slack for evaluation rounding.
    // Analytic carriers are unbounded, so a box says nothing about them; a
    // cylinder or sphere carrier is gated by a certified distance bound
    // instead (`carrier_gates`, below). Cones and tori are not gated.
    let mut carrier_aabbs: Vec<Option<Aabb3>> = Vec::with_capacity(faces.len());
    let mut carrier_gates: Vec<Option<super::helpers::CarrierGate>> =
        Vec::with_capacity(faces.len());
    for &fid in faces {
        let surface = topo.face(fid)?.surface().clone();
        seed_grids.push(match &surface {
            FaceSurface::Nurbs(nurbs) => Some(SurfaceSeedGrid::for_surface(nurbs)),
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => None,
        });
        carrier_aabbs.push(match &surface {
            FaceSurface::Nurbs(nurbs) => {
                super::helpers::finite_aabb(nurbs.control_points().iter().flatten().copied())
                    .map(|a| a.expanded(super::helpers::coordinate_slack(a)))
            }
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => None,
        });
        carrier_gates.push(super::helpers::CarrierGate::of(&surface));
    }
    // Every threshold the crossing scan compares a distance against is at
    // most 4 x linear tolerance (the tangent-contact trigger); gate at twice
    // that so float slack in the box arithmetic can never matter.
    let gate_margin = tol.linear * 8.0;

    for &eid in edges {
        // Snapshot edge data to avoid holding immutable borrow across add_vertex
        let (curve, start_pos, end_pos, t0, t1) = {
            let edge = topo.edge(eid)?;
            let sp = topo.vertex(edge.start())?.point();
            let ep = topo.vertex(edge.end())?.point();
            let (t0, t1) =
                super::helpers::authoritative_edge_domain(edge, eid, "edge-face intersection")?;
            (edge.curve().clone(), sp, ep, t0, t1)
        };

        // Broad-phase AABB for the edge, reused across all faces. The vast
        // majority of (edge, face) pairs are spatially disjoint; testing each
        // edge sample against every face surface (an iterative projection for
        // curved faces) is the dominant cost in booleans on solids with many
        // curved faces. Sampling the edge into an AABB once and rejecting
        // disjoint faces collapses that quadratic to the pairs that actually
        // overlap. Sampled densely enough that the inter-sample sagitta is
        // negligible for the analytic edge curves used here.
        let edge_aabb = matches!(curve, EdgeCurve::Line)
            .then(|| Aabb3::try_from_points([start_pos, end_pos]))
            .flatten()
            .map(|a| a.expanded(tol.linear));
        let curved_gate_aabb = super::helpers::conservative_curve_aabb(&curve, start_pos, end_pos)
            .map(|a| a.expanded(gate_margin + super::helpers::coordinate_slack(a)));
        // The NURBS-carrier gate predates the frame-tight box. Keep it at
        // least as wide as its original reach-based box so that tightening an
        // ellipse along its minor axis prunes no pair it used to scan: the
        // scan of a planar- or wall-containment NURBS face can still accept an
        // off-patch carrier crossing, and changing that is a separate fix.
        let carrier_gate_aabb =
            nurbs_carrier_gate_aabb(&curve, start_pos, end_pos, curved_gate_aabb, gate_margin);
        let edge_reach = super::helpers::EdgeReach::of(&curve, start_pos, end_pos);

        for (face_idx, &fid) in faces.iter().enumerate() {
            if face_boundary_edges[face_idx].contains(&eid) {
                continue;
            }

            // Broad-phase: skip faces whose region cannot reach this edge.
            if let (Some(ea), Some(fa)) = (&edge_aabb, &face_aabbs[face_idx])
                && !ea.intersects(*fa)
            {
                continue;
            }

            if let (Some(eb), Some(cb)) = (&carrier_gate_aabb, &carrier_aabbs[face_idx])
                && !eb.intersects(*cb)
            {
                continue;
            }

            let face = topo.face(fid)?;
            let surface = face.surface();

            // Every crossing a curved edge yields against a plane is one of
            // its own evaluations, inside `curved_gate_aabb`, and must pass
            // the face's containment, which first tests the box `face_aabbs`
            // contains. (Line edges took the exact test above.)
            if analytic_gates
                && !matches!(curve, EdgeCurve::Line)
                && matches!(surface, FaceSurface::Plane { .. })
                && let (Some(eb), Some(fa)) = (&curved_gate_aabb, &face_aabbs[face_idx])
                && !eb.intersects(*fa)
            {
                crate::perf::bump_ef_analytic_pair_gated();
                continue;
            }

            // Against a cylinder or sphere the scan emits a crossing only at
            // a point it measures within 4 x tolerance of the carrier, and it
            // never measures a point closer than its distance to the infinite
            // carrier. An edge whose certified clearance from that carrier
            // exceeds the gate margin can therefore produce no crossing, no
            // tangency and no on-surface verdict.
            if analytic_gates
                && let (Some(reach), Some(gate)) = (&edge_reach, &carrier_gates[face_idx])
                && gate
                    .clearance(reach)
                    .is_some_and(|clearance| clearance > gate_margin)
            {
                crate::perf::bump_ef_analytic_pair_gated();
                continue;
            }

            if takes_sampled_analytic_scan(&curve, surface) {
                crate::perf::bump_ef_analytic_pair_scan();
            }
            let grid = seed_grids[face_idx].as_ref();
            if grid.is_some() {
                crate::perf::bump_ef_nurbs_pair_probe();
            }

            // An edge lying entirely ON the face's surface is a coincidence
            // handled by the FF/same-domain machinery, not a set of
            // crossings; sampling it here would emit dozens of fake paves
            // (e.g. a cap circle lying in the opposing cap's plane).
            let n_chk = 16;
            let edge_on_surface = (0..=n_chk).all(|i| {
                let t = t0 + (t1 - t0) * (f64::from(i) / f64::from(n_chk));
                let pt = curve.evaluate_with_endpoints(t, start_pos, end_pos);
                distance_to_surface(pt, surface, grid) < tol.linear
            });
            if edge_on_surface {
                continue;
            }

            let crossings = match (surface, &containments[face_idx].nurbs) {
                (FaceSurface::Plane { normal, d }, _) => {
                    find_edge_plane_crossings(&curve, start_pos, end_pos, t0, t1, *normal, *d, tol)
                }
                // The unsigned sampler below only notices a crossing when a
                // sample lands within tolerance of the surface, which a
                // transversal crossing between two samples never does — so
                // a peg's cap rim passing through a converted bar wall went
                // unpaved and the rim never split. The carrier is a plane, so
                // the algebraic plane crossing is exact.
                //
                // An edge that HUGS the carrier — never departing it on both
                // sides beyond the band `fill_face_info` treats as lying in
                // the face — is that phase's coincidence, not a crossing:
                // a gridfinity cavity's rim riding a compartment wall's
                // planar carrier paved the wall at every noise flip and left
                // the cut with an edge shared by three faces.
                //
                // A recognized exact-rational cylinder wall measures the
                // same signed test radially: Line edges get the closed-form
                // line-cylinder solve; curved edges sample the radial field
                // for sign changes like the planar arm. The hug gate is
                // identical in structure (both sides beyond the band), and
                // the containment check below still bounds every crossing
                // to the face's trimmed chart polygon — the carrier alone
                // never admits a crossing.
                (FaceSurface::Nurbs(_), Some(nurbs)) => {
                    let signed: Box<dyn Fn(Point3) -> f64> = match (&nurbs.plane, &nurbs.wall) {
                        (Some((normal, d)), None) => {
                            let (normal, d) = (*normal, *d);
                            Box::new(move |pt: Point3| {
                                normal.dot(Vec3::new(pt.x(), pt.y(), pt.z())) - d
                            })
                        }
                        (None, Some(wall)) => {
                            let wall = wall.clone();
                            Box::new(move |pt: Point3| {
                                let to_pt = pt - wall.origin();
                                let axial = wall.axis() * wall.axis().dot(to_pt);
                                (to_pt - axial).length() - wall.radius()
                            })
                        }
                        // Structurally unreachable: the containment builder
                        // sets exactly one of the two readings (wall nets are
                        // curved, so coplanar detection declines on them).
                        _ => {
                            return Err(AlgoError::IntersectionFailed(
                                "EF NURBS containment carries neither a plane nor a wall reading"
                                    .to_string(),
                            ));
                        }
                    };
                    let chord = (end_pos - start_pos).length();
                    let hug = (super::fill_face_info::ON_SURFACE_BAND_FACTOR * tol.linear).max(
                        (super::fill_face_info::IN_FACE_MAX_DEVIATION_RATIO * chord)
                            .min(super::fill_face_info::IN_FACE_MAX_DEVIATION_ABS),
                    );
                    let (mut above, mut below) = (0.0_f64, 0.0_f64);
                    for i in 0..=N_SAMPLES {
                        #[allow(clippy::cast_precision_loss)]
                        let t = t0 + (t1 - t0) * (i as f64 / N_SAMPLES as f64);
                        let sd = signed(curve.evaluate_with_endpoints(t, start_pos, end_pos));
                        above = above.max(sd);
                        below = below.min(sd);
                    }
                    if above > hug && -below > hug {
                        match (&nurbs.plane, &nurbs.wall) {
                            (Some((normal, d)), None) => find_edge_plane_crossings_legacy(
                                &curve, start_pos, end_pos, t0, t1, *normal, *d, tol,
                            ),
                            (None, Some(wall)) => find_edge_cylinder_crossings(
                                &curve, start_pos, end_pos, t0, t1, wall, &signed, tol,
                            ),
                            _ => {
                                return Err(AlgoError::IntersectionFailed(
                                    "EF NURBS containment carries neither a plane nor a wall reading"
                                        .to_string(),
                                ));
                            }
                        }
                    } else {
                        Vec::new()
                    }
                }
                (FaceSurface::Sphere(sphere), _) if matches!(curve, EdgeCurve::Line) => {
                    line_sphere::crossings(start_pos, end_pos, t0, t1, sphere, tol.linear)
                        .unwrap_or_else(|| {
                            find_edge_surface_crossings(
                                &curve, start_pos, end_pos, t0, t1, surface, tol, grid,
                            )
                        })
                }
                (FaceSurface::Nurbs(_), None)
                | (
                    FaceSurface::Cylinder(_)
                    | FaceSurface::Cone(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_),
                    _,
                ) => find_edge_surface_crossings(
                    &curve, start_pos, end_pos, t0, t1, surface, tol, grid,
                ),
            };
            let crossings =
                if let (FaceSurface::Cylinder(wall), EdgeCurve::Line) = (surface, &curve) {
                    settle_line_cylinder_crossings(crossings, start_pos, end_pos, t0, t1, wall, tol)
                } else {
                    crossings
                };

            // Endpoint-drop windows, one per crossing and per endpoint,
            // computed while the face's surface borrow is still live (the
            // loop below mutates `topo`). A TANGENTIAL (grazing) contact's
            // position along the curve is only accurate to sqrt-of-residual —
            // an arc grazing a coplanar wall at its endpoint solves to a
            // point microns along the arc from the true endpoint despite a
            // ~1e-12 residual. A fixed `tol.linear` window misses that point
            // and mints a near-duplicate vertex next to the edge's own
            // endpoint (the gridfinity lip-corner non-manifold STL family).
            //
            // The widened window (tol / |tangent·normal|, capped at 1e-3)
            // applies ONLY toward an endpoint that itself lies ON the
            // surface: then the contact IS that endpoint's vertex-face
            // incidence and the solver merely mislocated it. An endpoint off
            // the surface keeps the tight `tol.linear` window — a shallow
            // crossing near (but not at) an off-surface endpoint is genuine
            // topology and must keep its pave (dropping those regressed the
            // honeycomb wall-cut raw residual).
            let on_surface = |p: Point3| distance_to_surface(p, surface, grid) <= tol.linear;
            let start_on_surface = on_surface(start_pos);
            let end_on_surface = on_surface(end_pos);
            let endpoint_windows: Vec<(f64, f64, f64)> = crossings
                .iter()
                .map(|&(t, pt)| {
                    let tangent = curve.tangent_with_endpoints(t, start_pos, end_pos);
                    let normal = match surface {
                        FaceSurface::Plane { normal, .. } => Some(*normal),
                        FaceSurface::Nurbs(_)
                        | FaceSurface::Cylinder(_)
                        | FaceSurface::Cone(_)
                        | FaceSurface::Sphere(_)
                        | FaceSurface::Torus(_) => {
                            surface.project_point(pt).map(|(u, v)| surface.normal(u, v))
                        }
                    };
                    let sin_angle = match (tangent.normalize(), normal) {
                        (Ok(tangent_unit), Some(n)) => tangent_unit.dot(n).abs(),
                        _ => 1.0,
                    };
                    let widened = (tol.linear / sin_angle.max(1e-9)).min(1e-3);
                    if !start_on_surface && !end_on_surface {
                        return (tol.linear, tol.linear, widened);
                    }
                    (
                        if start_on_surface {
                            widened
                        } else {
                            tol.linear
                        },
                        if end_on_surface { widened } else { tol.linear },
                        widened,
                    )
                })
                .collect();

            // Mid-edge tangential junction snap. A grazing contact solved to
            // within the tolerance WELL (distance to the surface grows only
            // quadratically away from a tangency, so a whole ~sqrt(2r*tol)
            // band of the edge sits "on" the surface) lands microns from the
            // true junction, minting a near-duplicate vertex next to an
            // exact one that already exists in an operand (a socket outline
            // arc ending where the outline's straight run continues along
            // the bin wall). Snap the crossing to an existing pave vertex
            // within the angle-scaled window when that vertex genuinely lies
            // on BOTH the crossed surface and this edge's curve — the
            // incidence checks are what make the widened radius safe. The
            // pave parameter is recomputed for Line edges (exact foot);
            // other curve types keep the tight path.
            let snaps: Vec<Option<(remus_topology::vertex::VertexId, f64)>> = crossings
                .iter()
                .zip(&endpoint_windows)
                .map(|(&(t, pt), &(_, _, snap_window))| {
                    // Only lines can use the exact parameter recomputation
                    // below. Reject other curves before the spatial lookup.
                    if !matches!(curve, remus_topology::edge::EdgeCurve::Line)
                        || snap_window <= tol.linear
                        || find_nearby_vertex(topo, arena, pt, tol).is_some()
                    {
                        return None;
                    }
                    let _ = t;
                    super::helpers::find_nearby_pave_vertex_widened(
                        arena,
                        pt,
                        snap_window,
                        tol.linear,
                        // The candidate itself must lie on the crossed surface
                        // AND inside the face's boundary region — the solved
                        // point passed containment, but the vertex sits up to
                        // the window away from it.
                        |p| {
                            distance_to_surface(p, surface, grid) <= tol.linear
                                && containments[face_idx].accepts(p)
                        },
                    )
                    // B53: the grazing contact can also coincide with a vertex
                    // an earlier phase minted ON THIS EDGE — EE solves the
                    // edge against the face's own boundary rim with a
                    // well-conditioned double root, while this scan's distance
                    // minimum lands sqrt-of-residual along the edge (1.6e-7 on
                    // the finding-17 box edge tangent to the cone's top rim).
                    // The operand-vertex index never holds those, so look at
                    // the edge's own extra paves under the same incidence
                    // check.
                    .or_else(|| {
                        edge_extra_pave_within(topo, arena, eid, pt, snap_window, tol.linear, |p| {
                            distance_to_surface(p, surface, grid) <= tol.linear
                        })
                    })
                    .and_then(|vid| {
                        let vp = topo.vertex(vid).ok()?.point();
                        let d = end_pos - start_pos;
                        let len_sq = d.length_squared();
                        if len_sq < tol.linear * tol.linear {
                            return None;
                        }
                        let s = ((vp - start_pos).dot(d) / len_sq).clamp(0.0, 1.0);
                        let t_new = (t1 - t0).mul_add(s, t0);
                        let on_curve = (curve.evaluate_with_endpoints(t_new, start_pos, end_pos)
                            - vp)
                            .length()
                            <= tol.linear;
                        on_curve.then_some((vid, t_new))
                    })
                })
                .collect();

            for (((t, pt), (start_window, end_window, _)), snap) in
                crossings.into_iter().zip(endpoint_windows).zip(snaps)
            {
                if !containments[face_idx].accepts(pt) {
                    log::debug!(
                        "EF: dropping crossing of edge {eid:?} at t={t:.6} — outside face {fid:?} boundary",
                    );
                    continue;
                }

                // A contact at the edge's own endpoint is a vertex-face
                // incidence (VF territory), not an edge crossing. Recording
                // it as EF marks the adjacent pave block as lying inside the
                // face even though the edge merely touches the face there
                // (e.g. a cap-rim arc tangent to a coplanar wall corner).
                if (pt - start_pos).length() <= start_window
                    || (pt - end_pos).length() <= end_window
                {
                    log::debug!(
                        "EF: dropping endpoint contact of edge {eid:?} at t={t:.6} on face {fid:?} (windows {start_window:.2e}/{end_window:.2e})",
                    );
                    continue;
                }

                let existing = find_nearby_vertex(topo, arena, pt, tol);

                let (vertex_id, t) = if let Some(vid) = existing {
                    (vid, t)
                } else if let Some((vid, t_new)) = snap {
                    log::debug!(
                        "EF: snapping tangential crossing of edge {eid:?} at t={t:.6} to \
                         existing vertex {vid:?} (t={t_new:.6})",
                    );
                    (vid, t_new)
                } else {
                    (topo.add_vertex(Vertex::new(pt, tol.linear)), t)
                };

                let pave = Pave::new(vertex_id, t);
                add_pave_to_edge(arena, eid, pave);

                arena.interference.ef.push(Interference::EF {
                    edge: eid,
                    face: fid,
                    new_vertex: Some(vertex_id),
                    parameter: Some(t),
                });

                arena.face_info_mut(fid).vertices_in.insert(vertex_id);

                log::debug!("EF: edge {eid:?} crosses face {fid:?} at t={t:.6}");
            }
        }
    }

    Ok(())
}

/// Whether the pair runs one of the sampled scans the analytic gates exist to
/// avoid: a curved edge against a plane, or any edge against an analytic
/// curved carrier.
fn takes_sampled_analytic_scan(curve: &EdgeCurve, surface: &FaceSurface) -> bool {
    match surface {
        FaceSurface::Plane { .. } => !matches!(curve, EdgeCurve::Line),
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => true,
        FaceSurface::Nurbs(_) => false,
    }
}

/// The vertex of an extra pave already on `edge` that lies within `radius` of
/// `point` and passes `accept`; `None` when there is none, or when two
/// candidates farther than `distinct_tol` apart both qualify (the same
/// unambiguity rule as the operand-vertex index).
fn edge_extra_pave_within(
    topo: &Topology,
    arena: &GfaArena,
    edge: EdgeId,
    point: Point3,
    radius: f64,
    distinct_tol: f64,
    accept: impl Fn(Point3) -> bool,
) -> Option<remus_topology::vertex::VertexId> {
    let mut best: Option<(f64, Point3, remus_topology::vertex::VertexId)> = None;
    let mut ambiguous = false;
    for &pb_id in arena.edge_pave_blocks.get(&edge)? {
        let Some(pb) = arena.pave_blocks.get(pb_id) else {
            continue;
        };
        for pave in &pb.extra_paves {
            let vertex = arena.resolve_vertex(pave.vertex);
            let Ok(v) = topo.vertex(vertex) else {
                continue;
            };
            let pos = v.point();
            let distance = (pos - point).length();
            if distance > radius || !accept(pos) {
                continue;
            }
            match best {
                Some((best_distance, best_pos, _)) => {
                    ambiguous |= (pos - best_pos).length() > distinct_tol;
                    if distance < best_distance {
                        best = Some((distance, pos, vertex));
                    }
                }
                None => best = Some((distance, pos, vertex)),
            }
        }
    }
    (!ambiguous)
        .then(|| best.map(|(_, _, vertex)| vertex))
        .flatten()
}

/// Find edge-cylinder crossings against a recognized converted wall.
///
/// Line edges get the closed-form line-cylinder solve (quadratic in the
/// chord parameter — exact up to roundoff, mirroring the algebraic
/// line-plane arm above). Every other curve samples the caller-supplied
/// signed radial field for sign changes, like the planar arm's curved-edge
/// path. Both paths keep the 1e-7 endpoint slack the plane arm uses.
#[allow(clippy::too_many_arguments)]
fn find_edge_cylinder_crossings(
    curve: &EdgeCurve,
    start_pos: Point3,
    end_pos: Point3,
    t0: f64,
    t1: f64,
    wall: &remus_math::surfaces::CylindricalSurface,
    signed: &dyn Fn(Point3) -> f64,
    tol: Tolerance,
) -> Vec<(f64, Point3)> {
    if matches!(curve, EdgeCurve::Line) {
        // Solve |radial(start + s*dir)|^2 = r^2 for the chord fraction s.
        let axis = wall.axis();
        let dir = end_pos - start_pos;
        let axial_dir = axis * axis.dot(dir);
        let radial_dir = dir - axial_dir;
        let to_start = start_pos - wall.origin();
        let axial_start = axis * axis.dot(to_start);
        let radial_start = to_start - axial_start;
        let a = radial_dir.length_squared();
        // 1e-15 checks for mathematical degeneracy (line parallel to the
        // axis — radial-cross-section blind), not geometric tolerance.
        if a < 1e-30 {
            return Vec::new();
        }
        let b = 2.0 * radial_start.dot(radial_dir);
        let c = radial_start.length_squared() - wall.radius() * wall.radius();
        let disc = b * b - 4.0 * a * c;
        if !disc.is_finite() || disc < 0.0 {
            return Vec::new();
        }
        let sqrt_disc = disc.sqrt();
        let mut out = Vec::new();
        for s in [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)] {
            if !s.is_finite() || !(-1e-7..=1.0 + 1e-7).contains(&s) {
                continue;
            }
            let s_clamped = s.clamp(0.0, 1.0);
            let pt = start_pos + dir * s_clamped;
            let t = s_clamped.mul_add(t1 - t0, t0);
            out.push((t, pt));
        }
        out
    } else {
        find_crossings_by_sampling(curve, start_pos, end_pos, t0, t1, signed, tol.linear)
    }
}

/// Settle the sampled scan's line-cylinder crossings on the closed-form roots.
///
/// The scan's ternary search on the unsigned distance narrows its bracket to
/// (2/3)^30 of a sample interval, 1.6e-6 along a 40 mm edge. A transversal
/// crossing hides that, but along a near-tangent line the distance barely
/// changes: a block edge 1e-9 inside a boss wall left its crossing 6e-7 off,
/// beside the exact vertex the wall's rim crossing minted, and the fuse kept
/// both. Each crossing moves to the nearest root of |radial|² = r² within its
/// sample bracket, or to the closest approach when the line passes the wall
/// within tolerance without crossing it. Which crossings exist is the scan's
/// decision; only their positions change.
fn settle_line_cylinder_crossings(
    crossings: Vec<(f64, Point3)>,
    start_pos: Point3,
    end_pos: Point3,
    t0: f64,
    t1: f64,
    wall: &remus_math::surfaces::CylindricalSurface,
    tol: Tolerance,
) -> Vec<(f64, Point3)> {
    let axis = wall.axis();
    let dir = end_pos - start_pos;
    let radial_dir = dir - axis * axis.dot(dir);
    let to_start = start_pos - wall.origin();
    let radial_start = to_start - axis * axis.dot(to_start);
    let a = radial_dir.length_squared();
    if a < 1e-30 || (t1 - t0).abs() <= 0.0 {
        return crossings;
    }
    let half_b = radial_start.dot(radial_dir);
    let c = radial_start.length_squared() - wall.radius() * wall.radius();
    let quarter_disc = half_b.mul_add(half_b, -a * c);
    let roots: Vec<f64> = if quarter_disc >= 0.0 {
        // Cancellation-free pair: q carries the sign of -half_b.
        let q = -half_b - quarter_disc.sqrt().copysign(half_b);
        if q == 0.0 {
            vec![0.0]
        } else {
            vec![q / a, c / q]
        }
    } else {
        vec![-half_b / a]
    };
    #[allow(clippy::cast_precision_loss)]
    let bracket = 2.0 / N_SAMPLES as f64;
    let mut settled: Vec<(f64, Point3)> = Vec::with_capacity(crossings.len());
    for (t, pt) in crossings {
        let s = (t - t0) / (t1 - t0);
        let nearest = roots
            .iter()
            .copied()
            .filter(|root| (0.0..=1.0).contains(root) && (root - s).abs() <= bracket)
            .min_by(|x, y| (x - s).abs().total_cmp(&(y - s).abs()));
        let (t, pt) = nearest.map_or((t, pt), |root| {
            let p = start_pos + dir * root;
            let radial = p - wall.origin();
            let off = ((radial - axis * axis.dot(radial)).length() - wall.radius()).abs();
            if off <= tol.linear {
                (root.mul_add(t1 - t0, t0), p)
            } else {
                (t, pt)
            }
        });
        if !settled
            .iter()
            .any(|&(_, q): &(f64, Point3)| (q - pt).length() <= tol.linear)
        {
            settled.push((t, pt));
        }
    }
    settled
}

/// Algebraic events for an actual analytic Plane×Circle pair. All root
/// coordinates are outward enclosed; a parameter is only a locator for a
/// native point whose whole root image fits within the reserved tolerance.
mod circle_plane_events {
    use super::{Point3, Vec3};
    use remus_math::compact_sort::CompactSort;
    use remus_math::curves::Circle3D;
    use std::f64::consts::{PI, TAU};

    #[derive(Clone, Copy)]
    struct I {
        lo: f64,
        hi: f64,
    }
    impl I {
        const fn point(value: f64) -> Self {
            Self {
                lo: value,
                hi: value,
            }
        }
        const fn unknown() -> Self {
            Self {
                lo: f64::NEG_INFINITY,
                hi: f64::INFINITY,
            }
        }
        fn finite(self) -> bool {
            self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi
        }
        fn point_value(self) -> Option<f64> {
            (self.lo.to_bits() == self.hi.to_bits()).then_some(self.lo)
        }
        fn is_zero(self) -> bool {
            self.point_value()
                .is_some_and(|value| value.to_bits().trailing_zeros() >= 63)
        }
        fn add(self, other: Self) -> Self {
            if !self.finite() || !other.finite() {
                return Self::unknown();
            }
            if let (Some(a), Some(b)) = (self.point_value(), other.point_value()) {
                let sum = a + b;
                if !sum.is_finite() {
                    return Self::unknown();
                }
                // Error-free TwoSum, including gradual underflow for binary64
                // addition. A known exact zero remains a singleton interval.
                let b_virtual = sum - a;
                let error = (a - (sum - b_virtual)) + (b - b_virtual);
                return if error > 0.0 {
                    Self {
                        lo: sum,
                        hi: sum.next_up(),
                    }
                } else if error < 0.0 {
                    Self {
                        lo: sum.next_down(),
                        hi: sum,
                    }
                } else {
                    Self::point(sum)
                };
            }
            let result = Self {
                lo: (self.lo + other.lo).next_down(),
                hi: (self.hi + other.hi).next_up(),
            };
            if result.finite() {
                result
            } else {
                Self::unknown()
            }
        }
        fn neg(self) -> Self {
            Self {
                lo: -self.hi,
                hi: -self.lo,
            }
        }
        fn sub(self, other: Self) -> Self {
            self.add(other.neg())
        }
        fn mul(self, other: Self) -> Self {
            if !self.finite() || !other.finite() {
                return Self::unknown();
            }
            if self.is_zero() || other.is_zero() {
                return Self::point(0.0);
            }
            if let (Some(a), Some(b)) = (self.point_value(), other.point_value()) {
                let product = a * b;
                // FMA's exact residual is representable above this conservative
                // threshold. Below it, residual==0 must NOT certify a product:
                // the product and its error can both underflow.
                let safe_floor = f64::MIN_POSITIVE / (f64::EPSILON * f64::EPSILON);
                if product.is_finite() && product.abs() >= safe_floor {
                    let error = a.mul_add(b, -product);
                    if error.is_finite() {
                        return Self::point(product).add(Self::point(error));
                    }
                }
            }
            let products = [
                self.lo * other.lo,
                self.lo * other.hi,
                self.hi * other.lo,
                self.hi * other.hi,
            ];
            if products.iter().any(|v| !v.is_finite()) {
                return Self::unknown();
            }
            Self {
                lo: products
                    .into_iter()
                    .fold(f64::INFINITY, f64::min)
                    .next_down(),
                hi: products
                    .into_iter()
                    .fold(f64::NEG_INFINITY, f64::max)
                    .next_up(),
            }
        }
        fn square(self) -> Self {
            if !self.finite() {
                return Self::unknown();
            }
            if self.point_value().is_some() {
                return self.mul(self);
            }
            let upper = self.lo.abs().max(self.hi.abs());
            let lower = if self.lo <= 0.0 && self.hi >= 0.0 {
                0.0
            } else {
                self.lo.abs().min(self.hi.abs())
            };
            Self {
                lo: (lower * lower).next_down().max(0.0),
                hi: (upper * upper).next_up(),
            }
        }
        fn div_positive(self, denominator: Self) -> Self {
            if !denominator.finite() || denominator.lo <= 0.0 {
                return Self::unknown();
            }
            self.mul(Self {
                lo: (1.0 / denominator.hi).next_down(),
                hi: (1.0 / denominator.lo).next_up(),
            })
        }
        fn abs_max(self) -> f64 {
            self.lo.abs().max(self.hi.abs())
        }
        fn midpoint(self) -> f64 {
            f64::midpoint(self.lo, self.hi)
        }
        fn hull(self, other: Self) -> Self {
            Self {
                lo: self.lo.min(other.lo),
                hi: self.hi.max(other.hi),
            }
        }
    }
    fn dot(a: Vec3, b: Vec3) -> I {
        (0..3)
            .map(|j| I::point(a.0[j]).mul(I::point(b.0[j])))
            .fold(I::point(0.0), I::add)
    }
    fn root_image(circle: &Circle3D, cosine: I, sine: I) -> [I; 3] {
        std::array::from_fn(|j| {
            I::point(circle.center().0[j]).add(
                I::point(circle.radius()).mul(
                    I::point(circle.u_axis().0[j])
                        .mul(cosine)
                        .add(I::point(circle.v_axis().0[j]).mul(sine)),
                ),
            )
        })
    }
    fn qualifies_point(
        circle: &Circle3D,
        root: [I; 2],
        parameter: f64,
        normal: Vec3,
        d: f64,
        budget: f64,
    ) -> Option<Point3> {
        let point = circle.evaluate(parameter);
        let image = root_image(circle, root[0], root[1]);
        let radius = image
            .into_iter()
            .zip(point.0)
            .map(|(bound, coordinate)| {
                let difference = bound.sub(I::point(coordinate));
                if difference.finite() {
                    I::point(difference.abs_max())
                } else {
                    I::unknown()
                }
            })
            .fold(I::point(0.0), I::add);
        let residual = dot(normal, Vec3::new(point.x(), point.y(), point.z())).sub(I::point(d));
        (radius.finite()
            && radius.hi <= budget
            && residual.finite()
            && residual.abs_max() <= budget)
            .then_some(point)
    }

    /// Outward quadrant reduction and degree39/38 Taylor bounds on |x|≤1.
    /// The omitted terms are below2e-48;1e-30 covers both remainders.
    fn sin_cos(parameter: f64) -> Option<(I, I)> {
        use std::f64::consts::FRAC_PI_2;
        if !parameter.is_finite() || parameter.abs() > 64.0 * PI {
            return None;
        }
        let turns = (parameter / FRAC_PI_2).round();
        let x = I::point(parameter).sub(
            I {
                lo: FRAC_PI_2.next_down(),
                hi: FRAC_PI_2.next_up(),
            }
            .mul(I::point(turns)),
        );
        if !x.finite() || x.abs_max() > 1.0 {
            return None;
        }
        let square = x.square();
        let (mut s_term, mut c_term) = (x, I::point(1.0));
        let (mut sine, mut cosine) = (s_term, c_term);
        for k in 1..20 {
            s_term = s_term
                .mul(square)
                .neg()
                .div_positive(I::point(f64::from((2 * k) * (2 * k + 1))));
            c_term = c_term
                .mul(square)
                .neg()
                .div_positive(I::point(f64::from((2 * k) * (2 * k - 1))));
            sine = sine.add(s_term);
            cosine = cosine.add(c_term);
        }
        let remainder = I {
            lo: -1e-30,
            hi: 1e-30,
        };
        let sine = sine.add(remainder);
        let cosine = cosine.add(remainder);
        let quadrant = turns.rem_euclid(4.0);
        Some(if quadrant < 0.5 {
            (sine, cosine)
        } else if quadrant < 1.5 {
            (cosine, sine.neg())
        } else if quadrant < 2.5 {
            (sine.neg(), cosine.neg())
        } else {
            (cosine.neg(), sine)
        })
    }

    /// Native sector containment uses only outward endpoint trigonometry.
    /// atan2/libm never decides that a possible carrier root is outside a trim.
    fn in_native_span(root: [I; 2], lo: f64, hi: f64) -> Option<bool> {
        let (sin_lo, cos_lo) = sin_cos(lo)?;
        let (sin_hi, cos_hi) = sin_cos(hi)?;
        let left = root[1].mul(cos_lo).sub(root[0].mul(sin_lo));
        let right = sin_hi.mul(root[0]).sub(cos_hi.mul(root[1]));
        let minor = if left.hi < 0.0 || right.hi < 0.0 {
            Some(false)
        } else if left.lo > 0.0 && right.lo > 0.0 {
            Some(true)
        } else {
            None
        };
        let major = if left.lo > 0.0 || right.lo > 0.0 {
            Some(true)
        } else if left.hi < 0.0 && right.hi < 0.0 {
            Some(false)
        } else {
            None
        };
        let span = I::point(hi).sub(I::point(lo));
        if span.hi < PI.next_down() {
            minor
        } else if span.lo > PI.next_up() {
            major
        } else if minor == major {
            minor
        } else {
            None
        }
    }

    /// `None` means no certified algebraic reading; the historical scan then
    /// retains its existing authority. `Some(empty)` proves strict separation.
    #[allow(clippy::too_many_lines)]
    pub(super) fn crossings(
        circle: &Circle3D,
        t0: f64,
        t1: f64,
        normal: Vec3,
        d: f64,
        tolerance: f64,
    ) -> Option<Vec<(f64, Point3)>> {
        if !tolerance.is_finite()
            || tolerance <= 0.0
            || !d.is_finite()
            || !t0.is_finite()
            || !t1.is_finite()
            || t0.abs().max(t1.abs()) > 64.0 * PI
            || (t1 - t0).abs() <= 0.0
            || (t1 - t0).abs() > TAU.next_up()
            || !circle.radius().is_finite()
            || circle.radius() <= 0.0
            || circle
                .center()
                .0
                .iter()
                .chain(normal.0.iter())
                .chain(circle.u_axis().0.iter())
                .chain(circle.v_axis().0.iter())
                .any(|v| !v.is_finite())
        {
            return None;
        }
        let n2 = dot(normal, normal);
        if n2.lo < 1.0 - 1e-12 || n2.hi > 1.0 + 1e-12 {
            return None;
        }
        let radius = I::point(circle.radius());
        let a = dot(normal, circle.u_axis()).mul(radius);
        let b = dot(normal, circle.v_axis()).mul(radius);
        let center = circle.center();
        let c = dot(normal, Vec3::new(center.x(), center.y(), center.z())).sub(I::point(d));
        let amplitude2 = a.square().add(b.square());
        let exact_tangent = (a.is_zero()
            && matches!((b.point_value(), c.point_value()), (Some(b), Some(c)) if b.abs().to_bits() == c.abs().to_bits()))
            || (b.is_zero()
                && matches!((a.point_value(), c.point_value()), (Some(a), Some(c)) if a.abs().to_bits() == c.abs().to_bits()));
        // Preserve exact coefficient dependency for axial tangencies instead
        // of subtracting independent outward squares of the same value.
        let discriminant = if exact_tangent {
            I::point(0.0)
        } else {
            amplitude2.sub(c.square())
        };
        if !discriminant.finite() {
            return None;
        }
        if discriminant.hi < 0.0 {
            return Some(Vec::new());
        }
        if amplitude2.lo <= 0.0 {
            return None;
        } // parallel/coplanar: FF owns coincidence
        let root = I {
            lo: discriminant.lo.max(0.0).sqrt().next_down().max(0.0),
            hi: discriminant.hi.sqrt().next_up(),
        };
        let x = c.neg().mul(a);
        let y = c.neg().mul(b);
        let plus = [
            x.sub(b.mul(root)).div_positive(amplitude2),
            y.add(a.mul(root)).div_positive(amplitude2),
        ];
        let minus = [
            x.add(b.mul(root)).div_positive(amplitude2),
            y.sub(a.mul(root)).div_positive(amplitude2),
        ];
        if plus.iter().chain(minus.iter()).any(|value| !value.finite()) {
            return None;
        }
        let roots = if discriminant.lo <= 0.0 {
            // An arithmetic-ambiguous tangent is one contact only when the
            // ENTIRE possible two-root cluster fits the same physical budget.
            vec![[plus[0].hull(minus[0]), plus[1].hull(minus[1])]]
        } else {
            vec![plus, minus]
        };
        let (lo, hi) = (t0.min(t1), t0.max(t1));
        let full_turn = (hi - lo).to_bits() == TAU.to_bits();
        if !full_turn && I::point(hi).sub(I::point(lo)).hi >= TAU.next_down() {
            return None;
        }
        let budget = tolerance / 8.0;
        let mut events: Vec<(f64, Point3)> = Vec::new();
        for root in roots {
            let membership = if full_turn {
                Some(true)
            } else {
                in_native_span(root, lo, hi)
            };
            if membership == Some(false) {
                continue;
            }
            let event = if membership.is_none() {
                // An unresolved endpoint event is represented only by an
                // authoritative native endpoint with the same physical proof.
                [lo, hi].into_iter().find_map(|native| {
                    qualifies_point(circle, root, native, normal, d, budget)
                        .map(|point| (native, point))
                })
            } else {
                let base = root[1].midpoint().atan2(root[0].midpoint());
                if !base.is_finite() {
                    return None;
                }
                // Locator failure is UNKNOWN, never a proof of no event.
                // Every retained root must emit a certified native point.
                (-34..=34).find_map(|turn| {
                    let parameter = f64::from(turn).mul_add(TAU, base);
                    if parameter < lo || parameter > hi {
                        return None;
                    }
                    let native = if full_turn && parameter.to_bits() == hi.to_bits() {
                        lo
                    } else {
                        parameter
                    };
                    qualifies_point(circle, root, native, normal, d, budget)
                        .map(|point| (native, point))
                })
            }?;
            if !events
                .iter()
                .any(|(existing, _)| existing.to_bits() == event.0.to_bits())
            {
                events.push(event);
            }
        }
        events.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
        Some(events)
    }

    #[cfg(test)]
    mod arithmetic_tests {
        use super::I;
        #[test]
        fn underflow_never_becomes_a_certified_exact_zero_product() {
            let tiny = I::point(1e-200).mul(I::point(1e-200));
            assert!(tiny.finite() && tiny.lo < 0.0 && tiny.hi > 0.0);
            assert!(tiny.point_value().is_none());
            assert!(I::point(0.0).mul(I::point(1e-200)).is_zero());
            assert!(
                I::point(1000.0)
                    .mul(I::point(1000.0))
                    .point_value()
                    .is_some()
            );
            assert!(!I::point(f64::NAN).mul(I::point(0.0)).finite());
        }
    }
}

/// Crossings for an actual analytic plane. NURBS plane aliases deliberately
/// retain the historical solver below; the new certificate owns Circle only.
#[allow(clippy::too_many_arguments)]
fn find_edge_plane_crossings(
    curve: &EdgeCurve,
    start_pos: Point3,
    end_pos: Point3,
    t0: f64,
    t1: f64,
    normal: Vec3,
    d: f64,
    tol: Tolerance,
) -> Vec<(f64, Point3)> {
    if let EdgeCurve::Circle(circle) = curve
        && let Some(events) = circle_plane_events::crossings(circle, t0, t1, normal, d, tol.linear)
    {
        return events;
    }
    find_edge_plane_crossings_legacy(curve, start_pos, end_pos, t0, t1, normal, d, tol)
}

/// Find edge-plane crossings using algebraic ray-plane intersection.
#[allow(clippy::too_many_arguments)]
fn find_edge_plane_crossings_legacy(
    curve: &EdgeCurve,
    start_pos: Point3,
    end_pos: Point3,
    t0: f64,
    t1: f64,
    normal: Vec3,
    d: f64,
    tol: Tolerance,
) -> Vec<(f64, Point3)> {
    if matches!(curve, EdgeCurve::Line) {
        let dir = end_pos - start_pos;
        let denom = dir.dot(normal);

        // 1e-15 checks for mathematical degeneracy (line parallel to
        // plane), not geometric tolerance.
        if denom.abs() < 1e-15 {
            // Line parallel to plane — no single crossing
            return Vec::new();
        }

        let origin_dot =
            start_pos.x() * normal.x() + start_pos.y() * normal.y() + start_pos.z() * normal.z();
        let s = (d - origin_dot) / denom;

        // s is in [0, 1] parameterization of start..end
        if !(-1e-7..=1.0 + 1e-7).contains(&s) {
            return Vec::new();
        }

        let s_clamped = s.clamp(0.0, 1.0);
        let pt = start_pos + dir * s_clamped;
        let t = s_clamped.mul_add(t1 - t0, t0);
        vec![(t, pt)]
    } else {
        find_crossings_by_sampling(
            curve,
            start_pos,
            end_pos,
            t0,
            t1,
            &|pt: Point3| pt.x() * normal.x() + pt.y() * normal.y() + pt.z() * normal.z() - d,
            tol.linear,
        )
    }
}

/// Find edge-surface crossings by sampling signed distance and refining.
#[allow(clippy::too_many_arguments)]
fn find_edge_surface_crossings(
    curve: &EdgeCurve,
    start_pos: Point3,
    end_pos: Point3,
    t0: f64,
    t1: f64,
    surface: &FaceSurface,
    tol: Tolerance,
    grid: Option<&SurfaceSeedGrid>,
) -> Vec<(f64, Point3)> {
    let n = N_SAMPLES;
    let parameter_epsilon = (t1 - t0).abs() / n as f64 * 2.0;
    let mut crossings = Vec::new();
    let mut prev_dist = f64::MAX;
    let mut prev_t = t0;

    for i in 0..=n {
        let t = t0 + (t1 - t0) * (i as f64 / n as f64);
        let pt = curve.evaluate_with_endpoints(t, start_pos, end_pos);
        let dist = distance_to_surface(pt, surface, grid);

        if i > 0 && dist < tol.linear {
            let is_dup = crossings
                .iter()
                .any(|&(ct, _): &(f64, Point3)| (t - ct).abs() < parameter_epsilon);
            if !is_dup {
                let refined =
                    refine_crossing(curve, start_pos, end_pos, prev_t, t, surface, tol, grid);
                crossings.push(refined);
            }
        } else if i > 0 && prev_dist > tol.linear && dist > tol.linear {
            let mid_t = f64::midpoint(prev_t, t);
            let mid_pt = curve.evaluate_with_endpoints(mid_t, start_pos, end_pos);
            let mid_dist = distance_to_surface(mid_pt, surface, grid);
            if mid_dist < prev_dist.min(dist) && mid_dist < tol.linear * 2.0 {
                let refined =
                    refine_crossing(curve, start_pos, end_pos, prev_t, t, surface, tol, grid);
                if distance_to_surface(refined.1, surface, grid) < tol.linear {
                    crossings.push(refined);
                }
            }

            // Tangent contact: near-surface sample triggers golden section minimum search
            if prev_dist < 4.0 * tol.linear || dist < 4.0 * tol.linear {
                let phi = 0.5 * (5.0_f64.sqrt() - 1.0);
                let mut lo = prev_t;
                let mut hi = t;
                for _ in 0..30 {
                    let m1 = hi - phi * (hi - lo);
                    let m2 = lo + phi * (hi - lo);
                    let d1 = distance_to_surface(
                        curve.evaluate_with_endpoints(m1, start_pos, end_pos),
                        surface,
                        grid,
                    );
                    let d2 = distance_to_surface(
                        curve.evaluate_with_endpoints(m2, start_pos, end_pos),
                        surface,
                        grid,
                    );
                    if d1 < d2 {
                        hi = m2;
                    } else {
                        lo = m1;
                    }
                }
                let t_min = f64::midpoint(lo, hi);
                let pt_min = curve.evaluate_with_endpoints(t_min, start_pos, end_pos);
                if distance_to_surface(pt_min, surface, grid) < tol.linear {
                    let is_dup = crossings
                        .iter()
                        .any(|&(ct, _): &(f64, Point3)| (t_min - ct).abs() < parameter_epsilon);
                    if !is_dup {
                        let refined =
                            refine_crossing(curve, start_pos, end_pos, lo, hi, surface, tol, grid);
                        crossings.push(refined);
                    }
                }
            }
        }

        prev_dist = dist;
        prev_t = t;
    }

    crossings
}

/// Find crossings by sampling a signed distance function and detecting sign changes.
fn find_crossings_by_sampling(
    curve: &EdgeCurve,
    start_pos: Point3,
    end_pos: Point3,
    t0: f64,
    t1: f64,
    signed_dist: &dyn Fn(Point3) -> f64,
    tol_linear: f64,
) -> Vec<(f64, Point3)> {
    let n = N_SAMPLES;
    let parameter_epsilon = (t1 - t0).abs() / n as f64 * 2.0;
    let mut crossings = Vec::new();

    let mut samples: Vec<(f64, f64)> = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = t0 + (t1 - t0) * (i as f64 / n as f64);
        let pt = curve.evaluate_with_endpoints(t, start_pos, end_pos);
        let sd = signed_dist(pt);
        samples.push((t, sd));
    }

    for i in 0..n {
        let (t_a, sd_a) = samples[i];
        let (t_b, sd_b) = samples[i + 1];

        if sd_a * sd_b < 0.0 {
            let mut lo = t_a;
            let mut hi = t_b;
            let mut sd_lo = sd_a;

            for _ in 0..30 {
                let mid = f64::midpoint(lo, hi);
                let pt_mid = curve.evaluate_with_endpoints(mid, start_pos, end_pos);
                let sd_mid = signed_dist(pt_mid);

                if sd_mid * sd_lo < 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                    sd_lo = sd_mid;
                }
            }

            let t = f64::midpoint(lo, hi);
            let pt = curve.evaluate_with_endpoints(t, start_pos, end_pos);
            crossings.push((t, pt));
        }
        // Tangent contact: minimum approaches zero without sign change
        else if sd_a.abs() < 4.0 * tol_linear || sd_b.abs() < 4.0 * tol_linear {
            let phi = 0.5 * (5.0_f64.sqrt() - 1.0);
            let mut lo = t_a;
            let mut hi = t_b;
            for _ in 0..30 {
                let m1 = hi - phi * (hi - lo);
                let m2 = lo + phi * (hi - lo);
                let d1 = signed_dist(curve.evaluate_with_endpoints(m1, start_pos, end_pos)).abs();
                let d2 = signed_dist(curve.evaluate_with_endpoints(m2, start_pos, end_pos)).abs();
                if d1 < d2 {
                    hi = m2;
                } else {
                    lo = m1;
                }
            }
            let t_min = f64::midpoint(lo, hi);
            let pt_min = curve.evaluate_with_endpoints(t_min, start_pos, end_pos);
            let d_min = signed_dist(pt_min).abs();
            if d_min < tol_linear {
                let is_dup = crossings
                    .iter()
                    .any(|&(ct, _): &(f64, Point3)| (t_min - ct).abs() < parameter_epsilon);
                if !is_dup {
                    crossings.push((t_min, pt_min));
                }
            }
        }
    }

    crossings
}

/// Compute distance from point to surface.
fn distance_to_surface(pt: Point3, surface: &FaceSurface, grid: Option<&SurfaceSeedGrid>) -> f64 {
    if let FaceSurface::Plane { normal, d } = surface {
        (pt.x() * normal.x() + pt.y() * normal.y() + pt.z() * normal.z() - d).abs()
    } else if let (FaceSurface::Nurbs(nurbs), Some(grid)) = (surface, grid) {
        // Identical to the generic arm below, with the coarse grid handed in
        // rather than rebuilt: same nodes, same nearest node, same Newton
        // start. Mirrors `ParametricSurface::project_point` exactly, midpoint
        // fallback included — this is a speed change, not a behaviour one.
        let (u, v) = if let Ok(proj) =
            project_point_to_surface_with_grid(nurbs, pt, NURBS_PROJECT_TOL, grid)
        {
            (proj.u, proj.v)
        } else {
            let (u0, u1) = nurbs.domain_u();
            let (v0, v1) = nurbs.domain_v();
            ((u0 + u1) * 0.5, (v0 + v1) * 0.5)
        };
        (pt - nurbs.evaluate(u, v)).length()
    } else if let Some((u, v)) = surface.project_point(pt) {
        if let Some(surf_pt) = surface.evaluate(u, v) {
            (pt - surf_pt).length()
        } else {
            f64::MAX
        }
    } else {
        f64::MAX
    }
}

/// Refine a crossing between two parameter values using ternary search.
#[allow(clippy::too_many_arguments)]
fn refine_crossing(
    curve: &EdgeCurve,
    start_pos: Point3,
    end_pos: Point3,
    t_lo: f64,
    t_hi: f64,
    surface: &FaceSurface,
    _tol: Tolerance,
    grid: Option<&SurfaceSeedGrid>,
) -> (f64, Point3) {
    let mut lo = t_lo.min(t_hi);
    let mut hi = t_lo.max(t_hi);

    for _ in 0..30 {
        let m1 = lo + (hi - lo) / 3.0;
        let m2 = hi - (hi - lo) / 3.0;
        let d1 = distance_to_surface(
            curve.evaluate_with_endpoints(m1, start_pos, end_pos),
            surface,
            grid,
        );
        let d2 = distance_to_surface(
            curve.evaluate_with_endpoints(m2, start_pos, end_pos),
            surface,
            grid,
        );
        if d1 < d2 {
            hi = m2;
        } else {
            lo = m1;
        }
    }

    // The search only narrows the bracket to (2/3)^30 of its width, so a
    // minimum AT a bracket end (an edge endpoint lying on the surface) would
    // come back that half-width short of it: 2e-7 along a 5 mm edge sampled
    // 64 times, past the endpoint window, minting a near-duplicate vertex.
    let t = f64::midpoint(lo, hi);
    let pt = curve.evaluate_with_endpoints(t, start_pos, end_pos);
    let mut best = (distance_to_surface(pt, surface, grid), t, pt);
    for end in [t_lo, t_hi] {
        let end_pt = curve.evaluate_with_endpoints(end, start_pos, end_pos);
        let d = distance_to_surface(end_pt, surface, grid);
        if d <= best.0 {
            best = (d, end, end_pt);
        }
    }
    (best.1, best.2)
}

/// The edge box for the NURBS-carrier gate: the frame-tight `tight` box
/// widened to cover [`legacy_reach_aabb`] (grown by `gate_margin`, as the
/// gate originally used it). `None`, so nothing is gated, whenever `tight` is.
fn nurbs_carrier_gate_aabb(
    curve: &EdgeCurve,
    start: Point3,
    end: Point3,
    tight: Option<Aabb3>,
    gate_margin: f64,
) -> Option<Aabb3> {
    let tight = tight?;
    match legacy_reach_aabb(curve, start, end).map(|a| a.expanded(gate_margin)) {
        Some(legacy) => Aabb3::try_from_points([tight.min, tight.max, legacy.min, legacy.max]),
        None => Some(tight),
    }
}

/// The NURBS-carrier gate's original circle/ellipse box: the carrier inside
/// the disc of radius `max(semi-axes)`, boxed per axis from its normal as
/// `reach * sqrt(1 - (n . e)^2)`. Narrower than the frame-tight box where the
/// normal is within ~1e-8 of a world axis or the frame is not orthonormal, so
/// it only ever widens that box (see `carrier_gate_aabb`). `None` for every
/// other carrier.
fn legacy_reach_aabb(curve: &EdgeCurve, start: Point3, end: Point3) -> Option<Aabb3> {
    let (center, reach, normal) = match curve {
        EdgeCurve::Circle(circle) => (circle.center(), circle.radius(), circle.normal()),
        EdgeCurve::Ellipse(ellipse) => (
            ellipse.center(),
            ellipse.semi_major().max(ellipse.semi_minor()),
            ellipse.normal(),
        ),
        EdgeCurve::Line
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Parabola(_)
        | EdgeCurve::Hyperbola(_) => return None,
    };
    if !reach.is_finite() {
        return None;
    }
    let n = normal.normalize().ok()?;
    let extent = |component: f64| -> f64 {
        let s = (1.0 - component * component).max(0.0).sqrt();
        reach * (s + 4.0 * f64::EPSILON)
    };
    let r = Vec3::new(extent(n.x()), extent(n.y()), extent(n.z()));
    Aabb3::try_from_points([center - r, center + r, start, end])
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    use remus_math::vec::Point3;
    use remus_topology::edge::EdgeCurve;

    /// Tightening the curved-edge box along an ellipse's minor axis must not
    /// let the NURBS-carrier gate prune a pair its reach-based box scanned:
    /// that box is a lower bound of the gate box, as is the tight box.
    #[test]
    fn nurbs_carrier_gate_box_covers_the_reach_box_and_the_tight_box() {
        use remus_math::curves::{Circle3D, Ellipse3D};
        let margin = 4e-7;
        let curves = [
            EdgeCurve::Ellipse(
                Ellipse3D::with_axes(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(0.0, 0.0, 1.0),
                    3.0,
                    1.0,
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
            ),
            EdgeCurve::Circle(
                Circle3D::new(Point3::new(5.0, -2.0, 1.0), Vec3::new(0.6, 0.0, 0.8), 2.5).unwrap(),
            ),
        ];
        let zero = Point3::new(0.0, 0.0, 0.0);
        for curve in &curves {
            let start = curve.evaluate_with_endpoints(0.3, zero, zero);
            let end = curve.evaluate_with_endpoints(2.0, zero, zero);
            let tight = super::super::helpers::conservative_curve_aabb(curve, start, end)
                .map(|a| a.expanded(margin));
            let gate = nurbs_carrier_gate_aabb(curve, start, end, tight, margin).unwrap();
            let legacy = legacy_reach_aabb(curve, start, end)
                .unwrap()
                .expanded(margin);
            for b in [legacy, tight.unwrap()] {
                assert!(gate.contains_point(b.min) && gate.contains_point(b.max));
            }
            assert!(nurbs_carrier_gate_aabb(curve, start, end, None, margin).is_none());
        }
        // The 3 x 1 ellipse: the tight box is 1 deep in y, the gate box 3.
        let (start, end) = (Point3::new(3.0, 0.0, 0.0), Point3::new(-3.0, 0.0, 0.0));
        let tight = super::super::helpers::conservative_curve_aabb(&curves[0], start, end).unwrap();
        let gate = nurbs_carrier_gate_aabb(&curves[0], start, end, Some(tight), 0.0).unwrap();
        assert!(tight.max.y() < 1.1 && gate.max.y() > 2.9);
    }

    /// The circle's normal is 1e-8 off the z axis, which rounds to exactly
    /// `(1e-8, 0, 1)`, so a box sized from `normal()` alone is flat in z to
    /// within ulps while the circle itself reaches `r * 1e-8 = 1e-4` above and
    /// below its centre. A planar NURBS face at `z = 3e-5` is crossed twice;
    /// the curved-edge gate must not prune the pair (it did, and EF recorded
    /// nothing).
    #[test]
    fn ef_nurbs_gate_keeps_a_near_axis_tilted_circle_crossing_its_carrier() {
        use remus_math::curves::Circle3D;
        use remus_math::nurbs::surface::NurbsSurface;
        use remus_topology::edge::Edge;
        use remus_topology::face::Face;
        use remus_topology::wire::{OrientedEdge, Wire};

        let mut topo = Topology::new();
        let normal = Vec3::new(1e-8, 0.0, 1.0).normalize().unwrap();
        assert_eq!(normal.z().to_bits(), 1.0_f64.to_bits());
        let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), normal, 1e4).unwrap();
        let seam = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
        let mut rim = Edge::new(seam, seam, EdgeCurve::Circle(circle));
        rim.set_trim(Some((0.0, std::f64::consts::TAU)));
        let rim = topo.add_edge(rim);

        let (h, z) = (2e4, 3e-5);
        let corners = [(-h, -h), (h, -h), (h, h), (-h, h)].map(|(x, y)| Point3::new(x, y, z));
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![vec![corners[0], corners[3]], vec![corners[1], corners[2]]],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .unwrap();
        let v = corners.map(|p| topo.add_vertex(Vertex::new(p, 1e-7)));
        let sides: Vec<_> = (0..4)
            .map(|i| {
                let e = topo.add_edge(Edge::new(v[i], v[(i + 1) % 4], EdgeCurve::Line));
                OrientedEdge::new(e, true)
            })
            .collect();
        let wire = topo.add_wire(Wire::new(sides, true).unwrap());
        let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface)));

        let mut arena = GfaArena::new();
        check_edge_face_pairs(
            &mut topo,
            &[rim],
            &[face],
            &[DetHashSet::default()],
            Tolerance::default(),
            &mut arena,
            true,
            &mut FaceContainments::default(),
        )
        .unwrap();
        let heights: Vec<f64> = arena
            .interference
            .ef
            .iter()
            .filter_map(|i| match i {
                Interference::EF {
                    new_vertex: Some(v),
                    ..
                } => Some(topo.vertex(*v).unwrap().point().z()),
                _ => None,
            })
            .collect();
        assert_eq!(heights.len(), 2, "{heights:?}");
        assert!(heights.iter().all(|h| (h - z).abs() < 1e-9), "{heights:?}");
    }

    /// Wherever the cylinder or sphere carrier gate fires, the scan's own
    /// `distance_to_surface` keeps every point it can evaluate on the edge at
    /// least 4 x tolerance (its largest trigger) from the carrier. Checked on
    /// a dense sweep plus the endpoints, for lines, full circles, arcs across
    /// the seam and reversed, an ellipse and a rational NURBS, against an
    /// axis-aligned and a tilted cylinder and a sphere placed at the origin,
    /// 1e6 and 1e13. At the origin the gate must also decide the adversarial
    /// placements as stated: a line through the wall or tangent to it at the
    /// radius, and a coaxial rim of the wall's own radius, stay ungated; a
    /// coaxial smaller rim, a line inside the wall and a rim 9e-7 clear of it
    /// are gated; a rim 7e-7 clear (inside the 8e-7 margin) is not.
    #[test]
    #[allow(clippy::too_many_lines)]
    fn carrier_gate_fires_only_where_every_scanned_point_is_clear() {
        use remus_math::curves::{Circle3D, Ellipse3D};
        use remus_math::nurbs::curve::NurbsCurve;
        use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
        use std::f64::consts::TAU;

        // (curve, t0, t1, start, end, expected gate decision at the origin)
        type Case = (EdgeCurve, f64, f64, Point3, Point3, Option<bool>);
        let tol = Tolerance::default();
        let gate_margin = 8.0 * tol.linear;
        let z = Vec3::new(0.0, 0.0, 1.0);
        let zero = Point3::new(0.0, 0.0, 0.0);
        let line = |a: Point3, b: Point3, gated: Option<bool>| -> Case {
            (EdgeCurve::Line, 0.0, 1.0, a, b, gated)
        };
        let closed = |curve: EdgeCurve, gated: Option<bool>| -> Case {
            let seam = curve.evaluate_with_endpoints(0.0, zero, zero);
            (curve, 0.0, TAU, seam, seam, gated)
        };
        let arc = |curve: EdgeCurve, t0: f64, t1: f64| -> Case {
            let (a, b) = (
                curve.evaluate_with_endpoints(t0, zero, zero),
                curve.evaluate_with_endpoints(t1, zero, zero),
            );
            (curve, t0, t1, a, b, Some(true))
        };
        let circle =
            |c: Point3, n: Vec3, r: f64| EdgeCurve::Circle(Circle3D::new(c, n, r).unwrap());
        let nurbs = |o: Point3| {
            let points = [
                (0.0, 0.0, 0.0),
                (1.0, 3.0, 0.0),
                (2.0, -3.0, 1.0),
                (3.0, 3.0, -1.0),
                (4.0, 0.0, 0.0),
            ]
            .map(|(x, y, w)| Point3::new(o.x() + x, o.y() + y, o.z() + w))
            .to_vec();
            let knots = vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0];
            let weights = vec![1.0, 2.0, 0.5, 2.0, 1.0];
            EdgeCurve::NurbsCurve(NurbsCurve::new(3, knots, points, weights).unwrap())
        };

        let mut decided = [0_usize; 2];
        for offset in [0.0, 1e6, 1e13] {
            let at = |x: f64, y: f64, w: f64| Point3::new(offset + x, y - offset, offset + w);
            let tilt = Vec3::new(1.0, 0.3, 2.0).normalize().unwrap();
            // A unit vector across the tilted axis.
            let u = Vec3::new(2.0, 0.0, -1.0).normalize().unwrap();
            let carriers: Vec<(FaceSurface, Vec<Case>)> = vec![
                (
                    FaceSurface::Cylinder(
                        CylindricalSurface::new(at(0.0, 0.0, 0.0), z, 2.0).unwrap(),
                    ),
                    vec![
                        line(at(5.0, 0.0, -1.0), at(5.0, 3.0, 4.0), Some(true)),
                        line(at(-1.0, 0.0, 0.0), at(1.0, 0.0, 3.0), Some(true)),
                        line(at(-3.0, 0.0, 0.0), at(3.0, 0.0, 0.0), Some(false)),
                        line(at(-3.0, 2.0, 0.0), at(3.0, 2.0, 0.0), Some(false)),
                        closed(circle(at(0.0, 0.0, 5.0), z, 2.0), Some(false)),
                        closed(circle(at(0.0, 0.0, 5.0), z, 1.0), Some(true)),
                        closed(circle(at(3.0 + 9e-7, 0.0, 1.0), z, 1.0), Some(true)),
                        closed(circle(at(3.0 + 7e-7, 0.0, 1.0), z, 1.0), Some(false)),
                        arc(circle(at(6.0, 0.0, 0.0), z, 1.0), 5.5, 7.0),
                        arc(circle(at(6.0, 0.0, 0.0), z, 1.0), 2.9, 0.3),
                        closed(
                            EdgeCurve::Ellipse(
                                Ellipse3D::new(at(0.0, 0.0, 1.0), z, 1.5, 1.0).unwrap(),
                            ),
                            Some(true),
                        ),
                        (
                            nurbs(at(10.0, 0.0, 0.0)),
                            0.0,
                            1.0,
                            at(10.0, 0.0, 0.0),
                            at(14.0, 0.0, 0.0),
                            Some(true),
                        ),
                        (
                            nurbs(at(-1.0, -1.0, 0.0)),
                            0.0,
                            1.0,
                            at(-1.0, -1.0, 0.0),
                            at(3.0, -1.0, 0.0),
                            Some(false),
                        ),
                    ],
                ),
                (
                    FaceSurface::Cylinder(
                        CylindricalSurface::new(at(1.0, 1.0, 1.0), tilt, 1.5).unwrap(),
                    ),
                    vec![
                        line(
                            at(1.0, 1.0, 1.0) + u * 3.0,
                            at(1.0, 1.0, 1.0) + u * 3.0 + tilt * 9.0,
                            Some(true),
                        ),
                        line(
                            at(1.0, 1.0, 1.0) + u * 1.5,
                            at(1.0, 1.0, 1.0) + u * 1.5 + tilt * 9.0,
                            Some(false),
                        ),
                        closed(
                            circle(at(1.0, 1.0, 1.0) + tilt * 4.0, tilt, 1.5),
                            Some(false),
                        ),
                        closed(
                            circle(at(1.0, 1.0, 1.0) + tilt * 4.0, tilt, 1.0),
                            Some(true),
                        ),
                        closed(circle(at(9.0, 1.0, 1.0), z, 1.0), None),
                    ],
                ),
                (
                    FaceSurface::Sphere(SphericalSurface::new(at(0.0, 0.0, 0.0), 3.0).unwrap()),
                    vec![
                        line(at(5.0, -1.0, 0.0), at(5.0, 1.0, 2.0), Some(true)),
                        line(at(-5.0, 0.0, 0.0), at(5.0, 0.0, 0.0), Some(false)),
                        line(at(-1.0, 0.5, 0.0), at(1.0, 0.0, 1.0), Some(true)),
                        line(at(-3.0, 3.0, 0.0), at(3.0, 3.0, 0.0), Some(false)),
                        closed(circle(at(0.0, 0.0, 0.5), z, 2.0), Some(true)),
                        closed(circle(at(0.0, 0.0, 0.0), z, 3.0), Some(false)),
                        arc(circle(at(0.0, 0.0, 9.0), z, 1.0), 5.5, 7.0),
                        (
                            nurbs(at(10.0, 0.0, 0.0)),
                            0.0,
                            1.0,
                            at(10.0, 0.0, 0.0),
                            at(14.0, 0.0, 0.0),
                            Some(true),
                        ),
                    ],
                ),
            ];
            for (surface, cases) in carriers {
                let gate = super::super::helpers::CarrierGate::of(&surface).expect("sound carrier");
                for (curve, t0, t1, start, end, expected) in cases {
                    let reach = super::super::helpers::EdgeReach::of(&curve, start, end).unwrap();
                    let gated = gate.clearance(&reach).is_some_and(|c| c > gate_margin);
                    if offset == 0.0
                        && let Some(expected) = expected
                    {
                        assert_eq!(gated, expected, "{curve:?} against {surface:?}");
                    }
                    decided[usize::from(gated)] += 1;
                    if !gated {
                        continue;
                    }
                    let nearest = (0..=4096)
                        .map(|i| t0 + (t1 - t0) * (f64::from(i) / 4096.0))
                        .map(|t| curve.evaluate_with_endpoints(t, start, end))
                        .chain([start, end])
                        .map(|p| distance_to_surface(p, &surface, None))
                        .fold(f64::INFINITY, f64::min);
                    assert!(
                        nearest >= 4.0 * tol.linear,
                        "gated {curve:?} comes {nearest} from {surface:?}"
                    );
                }
            }
        }
        assert!(decided[0] > 0 && decided[1] > 0, "{decided:?}");
    }

    /// The duplicate window is two sample spacings. Mutation testing found
    /// `span / n * 2` survives `span * n * 2` — a window wider than the whole
    /// edge, which folds every crossing after the first into it — because no
    /// test asked this function for two crossings on one edge.
    #[test]
    fn edge_surface_crossings_keep_both_crossings_of_a_through_line() {
        use remus_math::surfaces::CylindricalSurface;

        let cylinder =
            CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                .unwrap();
        let surface = FaceSurface::Cylinder(cylinder);
        // t = 0.25 and t = 0.75 sit exactly on the wall and on the sample grid.
        let start = Point3::new(-2.0, 0.0, 0.5);
        let end = Point3::new(2.0, 0.0, 0.5);
        let crossings = find_edge_surface_crossings(
            &EdgeCurve::Line,
            start,
            end,
            0.0,
            1.0,
            &surface,
            Tolerance::default(),
            None,
        );
        assert_eq!(
            crossings.len(),
            2,
            "a line through a cylinder crosses its wall twice, got {crossings:?}"
        );
        let mut xs: Vec<f64> = crossings.iter().map(|(_, p)| p.x()).collect();
        xs.sort_by(f64::total_cmp);
        assert!(
            (xs[0] + 1.0).abs() < 1e-3 && (xs[1] - 1.0).abs() < 1e-3,
            "{xs:?}"
        );
    }

    #[test]
    fn sampling_detects_tangent_touch() {
        // Signed distance: parabola touching zero at t=0.5 (exact tangent)
        let curve = EdgeCurve::Line;
        let start = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        let signed_dist = |pt: Point3| -> f64 {
            let t = pt.x();
            (t - 0.5) * (t - 0.5) // minimum = 0 at t=0.5
        };

        let crossings =
            find_crossings_by_sampling(&curve, start, end, 0.0, 1.0, &signed_dist, 1e-7);

        assert!(
            !crossings.is_empty(),
            "tangent touch (minimum=0) should be detected"
        );
        let (t, _) = crossings[0];
        assert!(
            (t - 0.5).abs() < 0.02,
            "tangent point should be near t=0.5, got {t}"
        );
    }

    #[test]
    fn sampling_detects_tangent_touch_on_descending_domain() {
        let curve = EdgeCurve::Line;
        let start = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(1.0, 0.0, 0.0);
        let signed_dist = |pt: Point3| -> f64 {
            let t = pt.x();
            (t - 0.5) * (t - 0.5)
        };

        let crossings =
            find_crossings_by_sampling(&curve, start, end, 1.0, 0.0, &signed_dist, 1e-7);

        assert!(!crossings.is_empty());
        assert!((crossings[0].0 - 0.5).abs() < 0.02);
    }

    /// An edge whose single pave block carries `extra` paves at the given
    /// positions, in order; returns the edge and the extra-pave vertices.
    fn edge_with_extra_paves(
        topo: &mut Topology,
        arena: &mut GfaArena,
        extra: &[Point3],
    ) -> (EdgeId, Vec<remus_topology::vertex::VertexId>) {
        let v0 = topo.add_vertex(Vertex::new(Point3::new(-10.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(10.0, 0.0, 0.0), 1e-7));
        let edge = topo.add_edge(remus_topology::edge::Edge::new(v0, v1, EdgeCurve::Line));
        let pb = arena.init_edge_pave_block(edge, v0, 0.0, v1, 1.0);
        let vertices: Vec<_> = extra
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, 1e-7)))
            .collect();
        for (i, v) in vertices.iter().enumerate() {
            let t = (f64::from(u32::try_from(i).unwrap()) + 1.0) / 8.0;
            arena
                .pave_blocks
                .get_mut(pb)
                .unwrap()
                .extra_paves
                .push(Pave::new(*v, t));
        }
        (edge, vertices)
    }

    /// Distances are 3-4-5 exact: from the origin, (3,0,0) is 3, (0,3,0) is
    /// 3, (3,4,0) is 5, and (3,0,0)-(3,4,0) is 4.
    #[test]
    fn extra_pave_within_keeps_the_radius_and_the_acceptance_test() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let mut topo = Topology::new();
        let mut arena = GfaArena::new();
        let (edge, v) = edge_with_extra_paves(&mut topo, &mut arena, &[Point3::new(3.0, 4.0, 0.0)]);
        let query = |radius: f64, accept: bool| {
            edge_extra_pave_within(&topo, &arena, edge, origin, radius, 1e-7, |_| accept)
        };
        assert_eq!(query(6.0, true), Some(v[0]));
        // Exactly on the radius is within it.
        assert_eq!(query(5.0, true), Some(v[0]));
        assert_eq!(query(4.0, true), None);
        assert_eq!(query(6.0, false), None);
        // An edge without pave blocks has no extra pave.
        let other = topo.add_edge(remus_topology::edge::Edge::new(v[0], v[0], EdgeCurve::Line));
        assert_eq!(
            edge_extra_pave_within(&topo, &arena, other, origin, 6.0, 1e-7, |_| true),
            None
        );
    }

    #[test]
    fn extra_pave_within_returns_the_nearest_unless_candidates_are_distinct() {
        let origin = Point3::new(0.0, 0.0, 0.0);
        let near = Point3::new(3.0, 0.0, 0.0);
        let far = Point3::new(3.0, 4.0, 0.0);
        for order in [[near, far], [far, near]] {
            let mut topo = Topology::new();
            let mut arena = GfaArena::new();
            let (edge, v) = edge_with_extra_paves(&mut topo, &mut arena, &order);
            let nearest = if order[0] == near { v[0] } else { v[1] };
            let query = |distinct_tol: f64| {
                edge_extra_pave_within(&topo, &arena, edge, origin, 10.0, distinct_tol, |_| true)
            };
            // The candidates are 4 apart: only a tolerance below 4 makes them
            // distinct, and two distinct candidates are ambiguous.
            assert_eq!(query(5.0), Some(nearest));
            assert_eq!(query(4.0), Some(nearest));
            assert_eq!(query(1.0), None);
        }
        // Equidistant candidates: the first one found stays.
        let mut topo = Topology::new();
        let mut arena = GfaArena::new();
        let (edge, v) =
            edge_with_extra_paves(&mut topo, &mut arena, &[near, Point3::new(0.0, 3.0, 0.0)]);
        assert_eq!(
            edge_extra_pave_within(&topo, &arena, edge, origin, 10.0, 5.0, |_| true),
            Some(v[0])
        );
    }

    #[test]
    fn circle_plane_tangent_uses_the_true_endpoint_not_the_tolerance_well() {
        use remus_math::curves::Circle3D;
        use std::f64::consts::{FRAC_PI_2, PI};
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.001,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let (from, to) = (3.0 * PI / 8.0, FRAC_PI_2);
        let start = circle.evaluate(from);
        let end = circle.evaluate(to);
        let roots = find_edge_plane_crossings(
            &EdgeCurve::Circle(circle),
            start,
            end,
            from,
            to,
            Vec3::new(0.0, 1.0, 0.0),
            0.001,
            Tolerance::default(),
        );
        assert_eq!(roots.len(), 1, "one tangent event, got {roots:?}");
        assert!(
            (roots[0].0 - to).abs() < 1e-10,
            "phantom tolerance-well pave: {roots:?}"
        );
        assert!(
            (roots[0].1 - end).length() < 1e-7 / 8.0,
            "contact must retain endpoint authority"
        );
    }

    fn circle_plane_pose(v: Vec3) -> Vec3 {
        let axis = Vec3::new(1.0, 2.0, 3.0).normalize().unwrap();
        let (s, c) = 0.37_f64.sin_cos();
        v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
    }

    #[test]
    fn circle_plane_axis_tangency_is_one_event_at_small_unit_and_large_scales() {
        use remus_math::curves::Circle3D;
        use std::f64::consts::{FRAC_PI_2, TAU};
        for r in [0.001, 1.0, 1000.0] {
            for center in [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(10.0 * r, -7.0 * r, 3.0 * r),
            ] {
                let circle = Circle3D::new_with_ref(
                    center,
                    Vec3::new(0.0, 0.0, 1.0),
                    r,
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap();
                let events = circle_plane_events::crossings(
                    &circle,
                    0.0,
                    TAU,
                    Vec3::new(0.0, 1.0, 0.0),
                    center.y() + r,
                    1e-7,
                )
                .unwrap();
                assert_eq!(events.len(), 1, "r={r} center={center:?}: {events:?}");
                assert!((events[0].0 - FRAC_PI_2).abs() < 1e-10);
                let expected = center + Vec3::new(0.0, r, 0.0);
                assert!((events[0].1 - expected).length() < 1e-7 / 8.0);
            }
        }
    }

    #[test]
    fn circle_plane_secants_and_misses_match_rigid_closed_form_oracles() {
        use remus_math::curves::Circle3D;
        use std::f64::consts::TAU;
        for r in [0.001, 1.0, 1000.0] {
            for placed in [false, true] {
                let pose = |v| if placed { circle_plane_pose(v) } else { v };
                let center = Point3::new(10.0 * r, -7.0 * r, 3.0 * r);
                let normal = pose(Vec3::new(0.0, 1.0, 0.0));
                let circle = Circle3D::new_with_ref(
                    center,
                    pose(Vec3::new(0.0, 0.0, 1.0)),
                    r,
                    pose(Vec3::new(1.0, 0.0, 0.0)),
                )
                .unwrap();
                let offset = normal.dot(Vec3::new(center.x(), center.y(), center.z()));
                let events = circle_plane_events::crossings(
                    &circle,
                    0.0,
                    TAU,
                    normal,
                    offset + 0.5 * r,
                    1e-7,
                )
                .unwrap();
                assert_eq!(events.len(), 2, "r={r} placed={placed}");
                for x in [-3.0_f64.sqrt() / 2.0, 3.0_f64.sqrt() / 2.0] {
                    let expected = center + pose(Vec3::new(x * r, 0.5 * r, 0.0));
                    assert!(
                        events
                            .iter()
                            .any(|(_, point)| (*point - expected).length() < 1e-7 / 8.0),
                        "r={r} placed={placed}: {events:?}"
                    );
                }
                assert!(
                    circle_plane_events::crossings(
                        &circle,
                        0.0,
                        TAU,
                        normal,
                        offset + 2.0 * r,
                        1e-7
                    )
                    .unwrap()
                    .is_empty()
                );
                // A genuine miss inside the old geometric distance band is
                // still separated algebraically, not promoted to a contact.
                assert!(
                    circle_plane_events::crossings(
                        &circle,
                        0.0,
                        TAU,
                        normal,
                        offset + (1.0 + 1e-5) * r,
                        1e-7
                    )
                    .unwrap()
                    .is_empty()
                );
            }
        }
    }

    #[test]
    fn circle_plane_native_trim_periods_descending_ranges_and_seams_are_authoritative() {
        use remus_math::curves::Circle3D;
        use std::f64::consts::{PI, TAU};
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let normal = Vec3::new(0.0, 1.0, 0.0);
        for (from, to, expected) in [
            (0.0, 0.2, 0),
            (0.0, 0.7, 1),
            (PI, 0.0, 2),
            (TAU, TAU + PI, 2),
        ] {
            let events =
                circle_plane_events::crossings(&circle, from, to, normal, 0.5, 1e-7).unwrap();
            assert_eq!(events.len(), expected, "[{from},{to}]: {events:?}");
            assert!(
                events
                    .iter()
                    .all(|(t, _)| *t >= from.min(to) && *t <= from.max(to))
            );
        }
        assert!(
            circle_plane_events::crossings(&circle, 0.0, 1.4, normal, 1.0, 1e-7)
                .unwrap()
                .is_empty()
        );
        let events =
            circle_plane_events::crossings(&circle, 0.0, TAU, Vec3::new(1.0, 0.0, 0.0), 1.0, 1e-7)
                .unwrap();
        assert_eq!(events.len(), 1, "full-circle seam is one contact");
        assert!(events[0].0.abs() < 1e-12);
    }

    #[test]
    fn circle_plane_posed_small_tangent_is_certified_and_uncertainty_declines() {
        use remus_math::curves::Circle3D;
        use std::f64::consts::{FRAC_PI_2, TAU};
        let radius = 0.001;
        let center = Point3::new(0.01, -0.007, 0.003);
        let normal = circle_plane_pose(Vec3::new(0.0, 1.0, 0.0));
        let circle = Circle3D::new_with_ref(
            center,
            circle_plane_pose(Vec3::new(0.0, 0.0, 1.0)),
            radius,
            circle_plane_pose(Vec3::new(1.0, 0.0, 0.0)),
        )
        .unwrap();
        let d = normal.dot(Vec3::new(center.x(), center.y(), center.z())) + radius;
        let events = circle_plane_events::crossings(
            &circle,
            3.0 * std::f64::consts::PI / 8.0,
            FRAC_PI_2,
            normal,
            d,
            1e-7,
        )
        .unwrap();
        assert_eq!(events.len(), 1);
        assert!((events[0].1 - circle.evaluate(FRAC_PI_2)).length() < 1e-7 / 8.0);
        assert!(circle_plane_events::crossings(&circle, 0.0, TAU, normal, d, 1e-30).is_none());
        assert!(circle_plane_events::crossings(&circle, f64::NAN, TAU, normal, d, 1e-7).is_none());
        assert!(
            circle_plane_events::crossings(&circle, 0.0, TAU, Vec3::new(0.0, 0.0, 0.0), d, 1e-7)
                .is_none()
        );
    }

    #[test]
    fn circle_plane_trim_exclusion_uses_certified_native_sectors() {
        use remus_math::curves::Circle3D;
        use std::f64::consts::PI;
        let circle = Circle3D::new_with_ref(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let angle = PI / 6.0;
        let normal = Vec3::new(0.0, 1.0, 0.0);
        // Both offsets are much less than the operation tolerance. Their
        // opposite exact native membership still determines0 versus1 event.
        assert!(
            circle_plane_events::crossings(&circle, angle + 1e-9, 0.7, normal, 0.5, 1e-7)
                .unwrap()
                .is_empty()
        );
        assert_eq!(
            circle_plane_events::crossings(&circle, angle - 1e-9, 0.7, normal, 0.5, 1e-7)
                .unwrap()
                .len(),
            1
        );
        assert_eq!(
            circle_plane_events::crossings(&circle, 0.0, angle, normal, 0.5, 1e-7)
                .unwrap()
                .len(),
            1
        );
        // A major arc retains the opposite secant and excludes the first.
        let events =
            circle_plane_events::crossings(&circle, 0.7, 2.0 * PI - 0.7, normal, 0.5, 1e-7)
                .unwrap();
        assert_eq!(events.len(), 1);
        assert!((events[0].0 - 5.0 * PI / 6.0).abs() < 1e-10);
    }
}
