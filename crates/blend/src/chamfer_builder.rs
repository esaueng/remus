//! Chamfer builder: orchestrates the full chamfer pipeline.
//!
//! Supports symmetric, asymmetric, and distance-angle chamfer modes on
//! planar face pairs (v1). Reuses the analytic fast path and face trimming
//! infrastructure from the fillet pipeline.

use std::collections::HashSet;

use remus_math::curves::Circle3D;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::analytic;
use crate::builder_utils::{
    add_certified_curve_edge, project_onto_axis, radial_distance, refuse_non_line_rim_neighbors,
    sample_nurbs_endpoints, wire_axial_range, wire_radial_extremum,
};
use crate::spine::Spine;
use crate::stripe::{Stripe, StripeResult};
use crate::trimmer::{self, TrimKeep, TrimSide};
use crate::{BlendError, BlendFaceOrigins, BlendResult};

/// Internal representation of a chamfer edge set with its distance parameters.
enum ChamferEdgeSet {
    /// Two explicit distances (d1 on face 1, d2 on face 2).
    TwoDistance {
        /// Edges to chamfer.
        edges: Vec<EdgeId>,
        /// Distance on face 1.
        d1: f64,
        /// Distance on face 2.
        d2: f64,
    },
    /// Distance on face 1 plus angle from face 1 toward face 2.
    DistanceAngle {
        /// Edges to chamfer.
        edges: Vec<EdgeId>,
        /// Distance on face 1.
        distance: f64,
        /// Angle from face 1 (radians).
        angle: f64,
    },
}

/// Builder for chamfer (bevel) operations on solid edges.
///
/// Collects edge sets with their distance parameters, then computes and
/// assembles the chamfered solid in a single `build()` call.
pub struct ChamferBuilder<'a> {
    topo: &'a mut Topology,
    solid: SolidId,
    edge_sets: Vec<ChamferEdgeSet>,
}

impl<'a> ChamferBuilder<'a> {
    /// Create a new chamfer builder for the given solid.
    #[must_use]
    pub fn new(topo: &'a mut Topology, solid: SolidId) -> Self {
        Self {
            topo,
            solid,
            edge_sets: Vec::new(),
        }
    }

    /// Add edges with symmetric chamfer distance (d1 = d2 = d).
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn add_edges_symmetric(&mut self, edges: &[EdgeId], d: f64) -> &mut Self {
        self.edge_sets.push(ChamferEdgeSet::TwoDistance {
            edges: edges.to_vec(),
            d1: d,
            d2: d,
        });
        self
    }

    /// Add edges with asymmetric chamfer distances.
    ///
    /// `d1` is the distance on face 1, `d2` on face 2.
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn add_edges_asymmetric(&mut self, edges: &[EdgeId], d1: f64, d2: f64) -> &mut Self {
        self.edge_sets.push(ChamferEdgeSet::TwoDistance {
            edges: edges.to_vec(),
            d1,
            d2,
        });
        self
    }

    /// Add edges with distance-angle chamfer.
    ///
    /// `distance` is measured on face 1; `angle` (radians) determines
    /// the depth on face 2 as `distance * tan(angle)`.
    ///
    /// Returns `&mut Self` for method chaining.
    pub fn add_edges_distance_angle(
        &mut self,
        edges: &[EdgeId],
        distance: f64,
        angle: f64,
    ) -> &mut Self {
        self.edge_sets.push(ChamferEdgeSet::DistanceAngle {
            edges: edges.to_vec(),
            distance,
            angle,
        });
        self
    }

    /// Compute and build the chamfered solid.
    ///
    /// # Algorithm
    ///
    /// 1. Build adjacency index for the solid.
    /// 2. For each target edge, find the two adjacent faces.
    /// 3. Build single-edge spines (no chain propagation in v1).
    /// 4. Compute stripes via analytic fast path or record failure.
    /// 5. Trim adjacent faces along contact curves.
    /// 6. Assemble new solid from trimmed faces, blend faces, and untouched
    ///    original faces.
    ///
    /// # Errors
    ///
    /// Returns [`BlendError`] if no edges were specified, or if topology
    /// lookups fail. Individual edge failures are recorded in
    /// [`BlendResult::failed`] rather than aborting the whole operation.
    #[allow(clippy::too_many_lines)]
    pub fn build(self) -> Result<BlendResult, BlendError> {
        let all_edges: Vec<(EdgeId, f64, f64)> = self
            .edge_sets
            .into_iter()
            .flat_map(|set| {
                let (edges, d1, d2) = match set {
                    ChamferEdgeSet::TwoDistance { edges, d1, d2 } => (edges, d1, d2),
                    ChamferEdgeSet::DistanceAngle {
                        edges,
                        distance,
                        angle,
                    } => {
                        let d2 = distance * angle.tan();
                        (edges, distance, d2)
                    }
                };
                edges.into_iter().map(move |eid| (eid, d1, d2))
            })
            .collect();

        if all_edges.is_empty() {
            return Err(BlendError::Topology(remus_topology::TopologyError::Empty {
                entity: "chamfer edge set",
            }));
        }

        let topo = self.topo;

        let adjacency = topo.build_adjacency(self.solid)?;

        let solid_data = topo.solid(self.solid)?;
        let shell_id = solid_data.outer_shell();
        let inner_shells = solid_data.inner_shells().to_vec();
        let original_faces: Vec<FaceId> = topo.shell(shell_id)?.faces().to_vec();

        let mut touched_faces: HashSet<FaceId> = HashSet::new();

        let mut succeeded: Vec<EdgeId> = Vec::new();
        let mut failed: Vec<(EdgeId, BlendError)> = Vec::new();
        let mut stripe_results: Vec<StripeResult> = Vec::new();

        for (edge_id, d1, d2) in &all_edges {
            let result = compute_chamfer_stripe(topo, &adjacency, *edge_id, *d1, *d2);
            match result {
                Ok(sr) => {
                    touched_faces.insert(sr.stripe.face1);
                    touched_faces.insert(sr.stripe.face2);
                    stripe_results.push(sr);
                    succeeded.push(*edge_id);
                }
                Err(e) => {
                    failed.push((*edge_id, e));
                }
            }
        }

        // If no stripes succeeded, return the original solid with all failures.
        if stripe_results.is_empty() {
            let is_partial = !failed.is_empty();
            return Ok(BlendResult {
                solid: self.solid,
                succeeded: Vec::new(),
                failed,
                is_partial,
                // Nothing was chamfered, so the input solid is the result and
                // every face is itself.
                face_origins: Some(BlendFaceOrigins {
                    survived: original_faces.iter().map(|&f| (f, f)).collect(),
                    deleted: Vec::new(),
                    created: Vec::new(),
                    created_unattributed: Vec::new(),
                }),
            });
        }

        let mut face_replacements: std::collections::HashMap<FaceId, FaceId> =
            std::collections::HashMap::new();
        // Every chamfer face beside the two base faces it was built between —
        // exact provenance, taken from the stripe that produced it.
        let mut blend_face_origins: Vec<(FaceId, Vec<FaceId>)> = Vec::new();

        // Partition out closed-revolution rim stripes (a full circular rim
        // between a disc cap and an axisymmetric wall). Those need an annular
        // rebuild the per-face line-based trimmer cannot produce — a closed
        // interior contact loop has no endpoints for it to cut at. Everything
        // else goes through the trim + blend-face path below.
        let mut rim_band_faces: Vec<FaceId> = Vec::new();
        let mut regular: Vec<&StripeResult> = Vec::new();
        let mut stripe_contact_edges: Vec<(
            Option<remus_topology::edge::EdgeId>,
            Option<remus_topology::edge::EdgeId>,
        )> = Vec::new();
        for sr in &stripe_results {
            if let Some(rim) = closed_rim_info(topo, &sr.stripe)? {
                match assemble_closed_rim(topo, &sr.stripe, &rim, &mut face_replacements) {
                    Ok(band) => {
                        rim_band_faces.push(band);
                        blend_face_origins.push((band, vec![sr.stripe.face1, sr.stripe.face2]));
                        continue;
                    }
                    // A setback the geometry cannot accommodate is a verdict,
                    // not a reason to try another assembler.
                    Err(e @ BlendError::RadiusTooLarge { .. }) => return Err(e),
                    Err(e) => {
                        log::warn!("closed-rim chamfer assembly failed: {e}, falling back to trim");
                    }
                }
            }
            regular.push(sr);
        }

        for sr in &regular {
            let stripe = &sr.stripe;
            stripe_contact_edges.push((None, None));

            let contact1_pts = sample_nurbs_endpoints(&stripe.contact1);
            let contact2_pts = sample_nurbs_endpoints(&stripe.contact2);

            let keep_side1 =
                if let (Some(sec), Ok(face)) = (stripe.sections.first(), topo.face(stripe.face1)) {
                    let n = face.surface().normal(0.0, 0.0);
                    if n.dot(sec.center - sec.p1) > 0.0 {
                        TrimSide::Right
                    } else {
                        TrimSide::Left
                    }
                } else {
                    TrimSide::Right
                };
            let keep_side2 =
                if let (Some(sec), Ok(face)) = (stripe.sections.first(), topo.face(stripe.face2)) {
                    let n = face.surface().normal(0.0, 0.0);
                    if n.dot(sec.center - sec.p2) > 0.0 {
                        TrimSide::Right
                    } else {
                        TrimSide::Left
                    }
                } else {
                    TrimSide::Right
                };

            let current_face1 = face_replacements
                .get(&stripe.face1)
                .copied()
                .unwrap_or(stripe.face1);
            let trim1 = trimmer::trim_face(
                topo,
                current_face1,
                &contact1_pts,
                &[(0.0, 0.0), (1.0, 0.0)],
                TrimKeep::Side(keep_side1),
                stripe.spine.edges(),
            );

            match trim1 {
                Ok(tr) if tr.trimmed_face != current_face1 => {
                    if let Some(slot) = stripe_contact_edges.last_mut() {
                        slot.0 = tr.contact_edge;
                    }
                    face_replacements.insert(stripe.face1, tr.trimmed_face);
                }
                Ok(_) | Err(_) => {
                    return Err(BlendError::TrimmingFailure { face: stripe.face1 });
                }
            }

            let current_face2 = face_replacements
                .get(&stripe.face2)
                .copied()
                .unwrap_or(stripe.face2);
            let trim2 = trimmer::trim_face(
                topo,
                current_face2,
                &contact2_pts,
                &[(0.0, 0.0), (1.0, 0.0)],
                TrimKeep::Side(keep_side2),
                stripe.spine.edges(),
            );

            match trim2 {
                Ok(tr) if tr.trimmed_face != current_face2 => {
                    if let Some(slot) = stripe_contact_edges.last_mut() {
                        slot.1 = tr.contact_edge;
                    }
                    face_replacements.insert(stripe.face2, tr.trimmed_face);
                }
                Ok(_) | Err(_) => {
                    return Err(BlendError::TrimmingFailure { face: stripe.face2 });
                }
            }
        }

        let mut blend_face_ids: Vec<FaceId> = rim_band_faces;

        for (si, sr) in regular.iter().enumerate() {
            // Reuse the trimmed neighbours' contact edges (mirrors the fillet
            // builder): a freshly minted duplicate leaves both copies use-1
            // and opens the shell along the chamfer flanks.
            let (c1, c2) = stripe_contact_edges
                .get(si)
                .copied()
                .unwrap_or((None, None));
            let blend_face_id =
                crate::builder_utils::create_blend_face_with_contacts(topo, &sr.stripe, c1, c2)?
                    .face;
            blend_face_ids.push(blend_face_id);
            blend_face_origins.push((blend_face_id, vec![sr.stripe.face1, sr.stripe.face2]));
        }

        let mut result_faces: Vec<FaceId> = Vec::new();

        for &fid in &original_faces {
            if !touched_faces.contains(&fid) {
                result_faces.push(fid);
            }
        }

        for &fid in &touched_faces {
            let replacement = face_replacements.get(&fid).copied();
            result_faces.push(replacement.unwrap_or(fid));
        }

        result_faces.extend(&blend_face_ids);

        // Provenance, straight from the bookkeeping above: an untouched face is
        // itself, a trimmed one is its replacement, and each chamfer band names
        // the two base faces its stripe ran between.
        let mut survived: Vec<(FaceId, FaceId)> = Vec::with_capacity(original_faces.len());
        for &fid in &original_faces {
            survived.push((fid, face_replacements.get(&fid).copied().unwrap_or(fid)));
        }
        let face_origins = BlendFaceOrigins {
            survived,
            deleted: Vec::new(),
            created: blend_face_origins,
            created_unattributed: Vec::new(),
        };

        let new_shell = Shell::new(result_faces)?;
        // Preserve the fork's fail-closed modifier contract. Upstream's shared
        // contact-edge path can close both chamfer flanks while still leaving
        // a free end edge on a regular finite stripe. Never return that open
        // shell as a successful modifier result.
        if (remus_topology::validation::validate_shell_closed(&new_shell, topo).is_err()
            || remus_topology::validation::validate_shell_manifold(&new_shell, topo).is_err())
            && let Some(sr) = stripe_results.first()
        {
            return Err(BlendError::TrimmingFailure {
                face: sr.stripe.face1,
            });
        }
        let new_shell_id = topo.add_shell(new_shell);
        let new_solid = Solid::new(new_shell_id, inner_shells);
        let new_solid_id = topo.add_solid(new_solid);

        let is_partial = !failed.is_empty();
        Ok(BlendResult {
            solid: new_solid_id,
            succeeded,
            failed,
            is_partial,
            face_origins: Some(face_origins),
        })
    }
}

/// Compute a chamfer stripe for a single edge using the adjacency index.
///
/// # Errors
///
/// Returns [`BlendError`] if the edge is non-manifold, if topology lookups
/// fail, or if the analytic path cannot produce a result.
fn compute_chamfer_stripe(
    topo: &Topology,
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    edge_id: EdgeId,
    d1: f64,
    d2: f64,
) -> Result<StripeResult, BlendError> {
    let adj_faces = adjacency.faces_for_edge(edge_id);
    if adj_faces.len() != 2 {
        log::warn!(
            "edge {edge_id:?} has {} adjacent faces (expected 2) — cannot chamfer non-manifold or boundary edges",
            adj_faces.len()
        );
        return Err(BlendError::StartSolutionFailure {
            edge: edge_id,
            t: 0.0,
        });
    }
    let face1 = adj_faces[0];
    let face2 = adj_faces[1];

    let surf1 = topo.face(face1)?.surface().clone();
    let surf2 = topo.face(face2)?.surface().clone();

    let spine = Spine::from_single_edge(topo, edge_id)?;

    if let Some(result) =
        analytic::try_analytic_chamfer(&surf1, &surf2, &spine, topo, d1, d2, face1, face2)?
    {
        return Ok(result);
    }

    log::debug!(
        target: "remus_approx",
        "chamfer: analytic path unavailable for {}+{} — v1 has no walker fallback, returning UnsupportedSurface",
        surf1.type_tag(),
        surf2.type_tag()
    );
    // v1: no walker fallback for non-analytic surface pairs.
    Err(BlendError::UnsupportedSurface {
        face: face1,
        surface_tag: format!(
            "{}+{} (walker not yet integrated)",
            surf1.type_tag(),
            surf2.type_tag()
        ),
    })
}

/// Geometry of a full-revolution rim chamfer (a closed circular edge between a
/// bounded disc cap and an axisymmetric wall), recovered from a stripe whose
/// blend surface is a cone.
///
/// This mirrors the rim-fillet path in `fillet_builder`; the only geometric
/// differences are that the band is a cone rather than a torus and the seam
/// joining the two contact circles is a straight line rather than a minor arc
/// (a chamfer band is ruled).
struct ClosedRimInfo {
    /// The planar cap face.
    plane_face: FaceId,
    /// The axisymmetric wall face (`Cylinder` or `Cone`).
    wall_face: FaceId,
    /// The original closed rim edge on the wall, replaced by the wall contact.
    rim_edge: EdgeId,
    /// Contact circle on the plate, in the plane.
    plate_circle: Circle3D,
    /// Contact circle on the wall, one chamfer setback along the axis.
    wall_circle: Circle3D,
    /// Which of the cap's boundaries the rim forms, and so which one the plate
    /// contact replaces.
    cap_rim_wire: CapRimWire,
}

/// Which boundary of the cap face the rim edge forms.
#[derive(Clone, Copy, PartialEq, Eq)]
enum CapRimWire {
    /// The cap's outer wire: a disc or annulus rim. The chamfer eats inward, so
    /// the boundary shrinks.
    Outer,
    /// Inner wire `i`: the rim is a hole through the cap, as at a bore mouth.
    /// The chamfer eats outward, so the hole widens.
    Inner(usize),
}

/// Detect a full-revolution rim-chamfer stripe and recover its annular geometry.
///
/// Returns `Some` when the blend surface is a cone, the spine is a single
/// closed circular edge, and the two adjacent faces are a plane (the disc cap)
/// and a cylinder/cone (the wall). Every other configuration returns `None`, so
/// the caller uses the normal trim path.
///
/// # Errors
///
/// Returns [`BlendError`] if topology lookups or circle construction fail.
fn closed_rim_info(topo: &Topology, stripe: &Stripe) -> Result<Option<ClosedRimInfo>, BlendError> {
    if !matches!(stripe.surface, FaceSurface::Cone(_)) {
        return Ok(None);
    }

    // Spine must be a single closed circular edge.
    let edges = stripe.spine.edges();
    if edges.len() != 1 {
        return Ok(None);
    }
    let rim_edge = edges[0];
    let rim_normal = {
        let e = topo.edge(rim_edge)?;
        if e.start() != e.end() {
            return Ok(None);
        }
        let EdgeCurve::Circle(circle) = e.curve() else {
            return Ok(None);
        };
        circle.normal()
    };

    // One side is the plane (cap), the other the cylinder/cone wall.
    let s1 = topo.face(stripe.face1)?.surface().clone();
    let s2 = topo.face(stripe.face2)?.surface().clone();
    let (plane_face, wall_face) = match (&s1, &s2) {
        (FaceSurface::Plane { .. }, FaceSurface::Cylinder(_) | FaceSurface::Cone(_)) => {
            (stripe.face1, stripe.face2)
        }
        (FaceSurface::Cylinder(_) | FaceSurface::Cone(_), FaceSurface::Plane { .. }) => {
            (stripe.face2, stripe.face1)
        }
        _ => return Ok(None),
    };

    // The annular rebuild swaps ONE cap boundary — whichever the rim edge forms
    // all by itself — for the plate contact, and carries the rest verbatim.
    // That boundary is the outer wire on a disc or annulus rim (the drilled
    // flange's rim cap is an annulus with a central opening plus six bolt
    // holes), and an inner wire at a bore mouth, where the rim is a hole
    // through the cap. Anything else falls back to the trim path.
    let cap_rim_wire = {
        let cap = topo.face(plane_face)?;
        let sole_rim = |wire_id| -> Result<bool, BlendError> {
            let edges = topo.wire(wire_id)?.edges();
            Ok(edges.len() == 1 && edges[0].edge() == rim_edge)
        };
        if sole_rim(cap.outer_wire())? {
            CapRimWire::Outer
        } else {
            let mut found = None;
            for (i, &wid) in cap.inner_wires().iter().enumerate() {
                if sole_rim(wid)? {
                    found = Some(CapRimWire::Inner(i));
                    break;
                }
            }
            match found {
                Some(w) => w,
                None => return Ok(None),
            }
        }
    };

    let (plate_contact, wall_contact) = if plane_face == stripe.face1 {
        (&stripe.contact1, &stripe.contact2)
    } else {
        (&stripe.contact2, &stripe.contact1)
    };

    // Recover the wall axis line from the wall surface.
    let wall_surf = topo.face(wall_face)?.surface().clone();
    let (axis, axis_origin) = match &wall_surf {
        FaceSurface::Cylinder(c) => (c.axis(), c.origin()),
        FaceSurface::Cone(c) => (c.axis(), c.apex()),
        _ => return Ok(None),
    };

    // Each contact is a full circle perpendicular to the axis; recover centre
    // and radius from one sampled point.
    let (pt0, _) = plate_contact.domain();
    let plate_pt = plate_contact.evaluate(pt0);
    let plate_center = project_onto_axis(plate_pt, axis_origin, axis);
    let plate_radius = radial_distance(plate_pt, axis_origin, axis);

    let (wt0, _) = wall_contact.domain();
    let wall_pt = wall_contact.evaluate(wt0);
    let wall_center = project_onto_axis(wall_pt, axis_origin, axis);
    let wall_radius = radial_distance(wall_pt, axis_origin, axis);

    // Pin both contact circles' `evaluate(0)` to the ray the rim's own seam
    // vertex sits on, exactly as the fillet twin does. The rebuild re-points
    // the wall's seam edge at the new circle's seam vertex while keeping the
    // seam's curve, so a circle seamed wherever `Frame3::from_normal` happens
    // to land leaves that edge running as a chord through the inside of the
    // wall — an edge of the wall face that is not on the wall surface.
    let seam_dir = {
        let v = topo.vertex(topo.edge(rim_edge)?.start())?.point() - axis_origin;
        v - axis * axis.dot(v)
    };
    // Keep the source rim's parameter direction. The cap and wall retain the
    // source edge-use flags, so rebuilding around the unsigned wall axis can
    // silently reverse a bore loop whose source circle uses the opposite axis.
    let plate_circle = Circle3D::new_with_ref(plate_center, rim_normal, plate_radius, seam_dir)?;
    let wall_circle = Circle3D::new_with_ref(wall_center, rim_normal, wall_radius, seam_dir)?;

    // At a bore mouth the chamfer must widen the hole. If the contact came out
    // the other way the configuration is not the one this rebuild models, so
    // leave it alone rather than build something plausible-looking.
    if matches!(cap_rim_wire, CapRimWire::Inner(_)) && plate_radius <= wall_radius {
        return Ok(None);
    }

    // The boundaries carried through unchanged must still clear the one that
    // moves. An outer rim shrinks the cap to `plate_radius`, so every hole has
    // to stay inside it; a bore mouth widens the hole to `plate_radius`, so
    // everything else has to stay outside. A setback big enough to reach
    // another boundary would need the two to merge — real geometry the annular
    // rebuild cannot express — and the cap wire would cross its own boundary
    // while still passing every topological check.
    //
    // The measurement is exact rather than sampled at nine points per edge:
    // a nine-point sample of a straight plate edge can miss its nearest
    // approach by more than the clearance being tested.
    {
        let cap = topo.face(plane_face)?;
        let mut others: Vec<remus_topology::wire::WireId> = Vec::new();
        match cap_rim_wire {
            CapRimWire::Outer => others.extend(cap.inner_wires().iter().copied()),
            CapRimWire::Inner(i) => {
                others.push(cap.outer_wire());
                others.extend(
                    cap.inner_wires()
                        .iter()
                        .enumerate()
                        .filter(|&(k, _)| k != i)
                        .map(|(_, &w)| w),
                );
            }
        }
        let widening = matches!(cap_rim_wire, CapRimWire::Inner(_));
        for wid in others {
            let clearance = wire_radial_extremum(topo, wid, axis_origin, axis, widening)?;
            let collides = if widening {
                clearance <= plate_radius
            } else {
                clearance >= plate_radius
            };
            if !collides {
                continue;
            }
            if widening {
                // The cause is the setback: the same rim chamfers fine below
                // the clearance, so report the achievable distance.
                return Err(BlendError::RadiusTooLarge {
                    edge: rim_edge,
                    max_radius: (clearance - wall_radius).max(0.0),
                });
            }
            // The shrinking direction keeps its established behaviour: defer to
            // the trim path rather than change how existing shapes report.
            return Ok(None);
        }
    }

    // When the contact moves INTO the wall, the rebuild shortens the wall to
    // meet it, and only the wall's own axial extent says whether it has that
    // much material to give — the shell would still close, and the tessellation
    // would still be watertight, with the contact hanging off the end of the
    // bore. A concave rim band extends the wall instead, which has no such
    // bound; that case shows up as no extent in the setback direction.
    {
        let setback = axis.dot(wall_center - plate_center);
        let wall_wire = topo.face(wall_face)?.outer_wire();
        let (s_min, s_max) = wire_axial_range(topo, wall_wire, plate_center, axis)?;
        let available = if setback >= 0.0 { s_max } else { -s_min };
        let shortening = available > 1e-9 * (1.0 + setback.abs());
        if shortening && setback.abs() >= available {
            return Err(BlendError::RadiusTooLarge {
                edge: rim_edge,
                max_radius: available.max(0.0),
            });
        }
    }

    Ok(Some(ClosedRimInfo {
        plane_face,
        wall_face,
        rim_edge,
        plate_circle,
        wall_circle,
        cap_rim_wire,
    }))
}

/// Assemble a full-revolution rim chamfer: rebuild the disc cap bounded by the
/// plate contact, shorten the wall to the wall contact, and emit the conical
/// band between them. Cap and wall edges are shared with the band so the result
/// is watertight.
///
/// # Errors
///
/// Returns [`BlendError`] if topology lookups or wire/face construction fail.
fn assemble_closed_rim(
    topo: &mut Topology,
    stripe: &Stripe,
    rim: &ClosedRimInfo,
    face_replacements: &mut std::collections::HashMap<FaceId, FaceId>,
) -> Result<FaceId, BlendError> {
    const TOL: f64 = 1e-7;

    let plane_surf = topo.face(rim.plane_face)?.surface().clone();
    let plane_reversed = topo.face(rim.plane_face)?.is_reversed();

    let current_wall = face_replacements
        .get(&rim.wall_face)
        .copied()
        .unwrap_or(rim.wall_face);
    let wall_surf = topo.face(current_wall)?.surface().clone();
    let wall_reversed = topo.face(current_wall)?.is_reversed();
    let wall_outer_wire = topo.face(current_wall)?.outer_wire();
    let wall_inner = topo.face(current_wall)?.inner_wires().to_vec();
    let wall_oriented: Vec<OrientedEdge> = topo.wire(wall_outer_wire)?.edges().to_vec();
    let old_rim_vertex = topo.edge(rim.rim_edge)?.start();

    refuse_non_line_rim_neighbors(
        topo,
        &wall_oriented,
        rim.rim_edge,
        old_rim_vertex,
        rim.wall_face,
    )?;

    // Vertices for the two closed contact circles (start == end → degenerate).
    let plate_point = rim.plate_circle.evaluate(0.0);
    let wall_point = rim.wall_circle.evaluate(0.0);
    let plate_v = topo.add_vertex(Vertex::new(plate_point, TOL));
    let wall_v = topo.add_vertex(Vertex::new(wall_point, TOL));

    let plate_edge = add_certified_curve_edge(
        topo,
        plate_v,
        plate_v,
        EdgeCurve::Circle(rim.plate_circle.clone()),
        (0.0, std::f64::consts::TAU),
    )?;
    let wall_edge = add_certified_curve_edge(
        topo,
        wall_v,
        wall_v,
        EdgeCurve::Circle(rim.wall_circle.clone()),
        (0.0, std::f64::consts::TAU),
    )?;
    // The chamfer band is ruled, so its seam is the straight generator between
    // the two contacts — unlike the fillet's minor-circle arc. `EdgeCurve::Line`
    // takes its geometry from the endpoints, so no explicit curve is needed.
    let seam_edge = topo.add_edge(Edge::new(plate_v, wall_v, EdgeCurve::Line));

    // --- Move the cap's rim boundary to the plate contact. ---
    // Only the boundary the rim forms moves; the cap's other wires are carried
    // through verbatim. Handing the rebuilt face an empty inner-wire list would
    // fill in every hole — the drilled flange's rim cap would lose its bore and
    // six bolt openings, and each bore wall would lose the face it pairs with,
    // opening the shell. A prior stripe may already have replaced this cap; a
    // rebuild preserves wire count and order, so the recorded index still names
    // the right wire.
    let cap_orig = topo.face(
        face_replacements
            .get(&rim.plane_face)
            .copied()
            .unwrap_or(rim.plane_face),
    )?;
    let cap_outer_wire_id = cap_orig.outer_wire();
    let cap_inner_wire_ids = cap_orig.inner_wires().to_vec();
    let rim_wire_id = match rim.cap_rim_wire {
        CapRimWire::Outer => cap_outer_wire_id,
        CapRimWire::Inner(i) => *cap_inner_wire_ids
            .get(i)
            .ok_or(BlendError::TrimmingFailure {
                face: rim.plane_face,
            })?,
    };
    let cap_forward = topo
        .wire(rim_wire_id)?
        .edges()
        .iter()
        .find(|oe| oe.edge() == rim.rim_edge)
        .is_some_and(OrientedEdge::is_forward);
    let cap_wire = Wire::new(vec![OrientedEdge::new(plate_edge, cap_forward)], true)?;
    let cap_wire_id = topo.add_wire(cap_wire);
    let (new_cap_outer, new_cap_inner) = match rim.cap_rim_wire {
        CapRimWire::Outer => (cap_wire_id, cap_inner_wire_ids),
        CapRimWire::Inner(i) => {
            let mut inner = cap_inner_wire_ids;
            inner[i] = cap_wire_id;
            (cap_outer_wire_id, inner)
        }
    };
    let mut cap_face = Face::new(new_cap_outer, new_cap_inner, plane_surf);
    cap_face.set_reversed(plane_reversed);
    let cap_face_id = topo.add_face(cap_face);
    face_replacements.insert(rim.plane_face, cap_face_id);

    // --- Shorten the wall to the wall contact. ---
    // The wall's outer wire references the rim circle plus (for the cylinder /
    // cone primitive) a seam line whose endpoint is the rim vertex. Replace the
    // rim circle with the wall contact, and rebuild any seam edge touching the
    // old rim vertex so it starts at the new wall vertex — otherwise the wire
    // no longer closes.
    // A seam edge may appear twice in the wall wire (fwd + rev); rebuild each
    // distinct edge once so both references share the new edge.
    let mut rebuilt: std::collections::HashMap<EdgeId, EdgeId> = std::collections::HashMap::new();
    let mut new_wall_edges: Vec<OrientedEdge> = Vec::with_capacity(wall_oriented.len());
    let mut wall_forward = None;
    for oe in &wall_oriented {
        if oe.edge() == rim.rim_edge {
            new_wall_edges.push(OrientedEdge::new(wall_edge, oe.is_forward()));
            wall_forward = Some(oe.is_forward());
            continue;
        }
        let e = topo.edge(oe.edge())?;
        let touches_rim = e.start() == old_rim_vertex || e.end() == old_rim_vertex;
        if touches_rim {
            let new_eid = if let Some(&id) = rebuilt.get(&oe.edge()) {
                id
            } else {
                let new_start = if e.start() == old_rim_vertex {
                    wall_v
                } else {
                    e.start()
                };
                let new_end = if e.end() == old_rim_vertex {
                    wall_v
                } else {
                    e.end()
                };
                let id = topo.add_edge(Edge::new(new_start, new_end, EdgeCurve::Line));
                rebuilt.insert(oe.edge(), id);
                id
            };
            new_wall_edges.push(OrientedEdge::new(new_eid, oe.is_forward()));
        } else {
            new_wall_edges.push(*oe);
        }
    }
    let Some(wall_forward) = wall_forward else {
        return Err(BlendError::TrimmingFailure {
            face: rim.wall_face,
        });
    };
    let new_wall_wire = Wire::new(new_wall_edges, true)?;
    let new_wall_wire_id = topo.add_wire(new_wall_wire);
    let mut new_wall_face = Face::new(new_wall_wire_id, wall_inner, wall_surf);
    new_wall_face.set_reversed(wall_reversed);
    let new_wall_face_id = topo.add_face(new_wall_face);
    face_replacements.insert(rim.wall_face, new_wall_face_id);

    // --- Conical band between the two contact circles. ---
    // Degenerate-seam wire (plate circle, seam up, wall circle reversed, seam
    // down). The seam runs plate_v → wall_v, so this fixed order always closes.
    // The shared circle edges are used opposite to the standard-wound cap and
    // wall, keeping the shell manifold.
    let band_reversed = cone_band_needs_reversal(&stripe.surface, rim);
    let cap_effective_forward = cap_forward != plane_reversed;
    let wall_effective_forward = wall_forward != wall_reversed;
    let band_plate_forward = cap_effective_forward == band_reversed;
    let band_wall_forward = wall_effective_forward == band_reversed;
    let band_wire = Wire::new(
        vec![
            OrientedEdge::new(plate_edge, band_plate_forward),
            OrientedEdge::new(seam_edge, true),
            OrientedEdge::new(wall_edge, band_wall_forward),
            OrientedEdge::new(seam_edge, false),
        ],
        true,
    )?;
    let band_wire_id = topo.add_wire(band_wire);
    let mut band_face = Face::new(band_wire_id, Vec::new(), stripe.surface.clone());
    if band_reversed {
        band_face.set_reversed(true);
    }
    let band_face_id = topo.add_face(band_face);

    Ok(band_face_id)
}

/// Decide whether a rim-chamfer cone band must carry `reversed` so its outward
/// normal points away from the solid.
///
/// Mirrors `torus_band_needs_reversal` in `fillet_builder`: the band's outward
/// axial direction is the one pointing from the wall contact back toward the
/// plate, and the band is reversed when the surface's geometric normal at the
/// mid-generator point opposes it.
fn cone_band_needs_reversal(surface: &FaceSurface, rim: &ClosedRimInfo) -> bool {
    let FaceSurface::Cone(cone) = surface else {
        return false;
    };
    let axis = cone.axis();
    let to_plate = rim.plate_circle.center() - rim.wall_circle.center();
    let outward_axial = axis * axis.dot(to_plate);
    // Midpoint of the straight generator between the two contacts.
    let plate_pt = rim.plate_circle.evaluate(0.0);
    let wall_pt = rim.wall_circle.evaluate(0.0);
    let mid = Point3::new(
        (plate_pt.x() + wall_pt.x()) * 0.5,
        (plate_pt.y() + wall_pt.y()) * 0.5,
        (plate_pt.z() + wall_pt.z()) * 0.5,
    );
    let (u, v) = remus_math::traits::ParametricSurface::project_point(cone, mid);
    let n = remus_math::traits::ParametricSurface::normal(cone, u, v);
    n.dot(outward_axial) < 0.0
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use super::*;
    use remus_topology::adjacency::AdjacencyIndex;
    use remus_topology::test_utils::make_unit_cube_manifold;

    /// Find the first manifold edge of the solid (shared by exactly 2 faces).
    fn find_manifold_edge(topo: &Topology, solid: SolidId) -> EdgeId {
        let adjacency = AdjacencyIndex::build(topo, solid).unwrap();
        let shell_id = topo.solid(solid).unwrap().outer_shell();
        let faces = topo.shell(shell_id).unwrap().faces().to_vec();

        for &fid in &faces {
            let face = topo.face(fid).unwrap();
            let wire = topo.wire(face.outer_wire()).unwrap();
            for oe in wire.edges() {
                let adj = adjacency.faces_for_edge(oe.edge());
                if adj.len() == 2 {
                    return oe.edge();
                }
            }
        }
        panic!("cube should have manifold edges");
    }

    #[test]
    fn chamfer_builder_symmetric() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let target_edge = find_manifold_edge(&topo, solid);

        let shell_id = topo.solid(solid).unwrap().outer_shell();
        let original_face_count = topo.shell(shell_id).unwrap().faces().len();

        let mut builder = ChamferBuilder::new(&mut topo, solid);
        builder.add_edges_symmetric(&[target_edge], 0.1);
        let result = builder.build();
        assert!(matches!(result, Err(BlendError::TrimmingFailure { .. })));
        assert_eq!(original_face_count, 6);
    }

    #[test]
    fn chamfer_builder_distance_angle() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);
        let target_edge = find_manifold_edge(&topo, solid);

        let shell_id = topo.solid(solid).unwrap().outer_shell();
        let original_face_count = topo.shell(shell_id).unwrap().faces().len();

        // 45-degree angle means d2 = distance * tan(45deg) = distance.
        let distance = 0.15;
        let angle = std::f64::consts::FRAC_PI_4;

        let mut builder = ChamferBuilder::new(&mut topo, solid);
        builder.add_edges_distance_angle(&[target_edge], distance, angle);
        let result = builder.build();
        assert!(matches!(result, Err(BlendError::TrimmingFailure { .. })));
        assert_eq!(original_face_count, 6);
    }

    #[test]
    fn chamfer_builder_empty_edges_error() {
        let mut topo = Topology::new();
        let solid = make_unit_cube_manifold(&mut topo);

        let builder = ChamferBuilder::new(&mut topo, solid);
        let result = builder.build();
        assert!(result.is_err(), "empty edge set should produce an error");
    }
}
