//! Resize a through-bore at a chamfered thread mouth. Reconstruct the screw
//! form from its lands, and admit it only if the generator reproduces the
//! complete trimmed wall. This works on a standalone boundary representation.
mod certificate;
mod fit;
mod recognize;
use super::{
    Frame, Ring, SEAM_DEPTH, ThreadEnd, ThreadEndSide, ThreadError, ThreadFaceRole, ThreadForm,
    ThreadHand, ThreadKind, default_tolerance, require_storable_tolerances, thread_ring,
};
use crate::push_pull::{DirectEditEvolution, MoveFacesResult};
use crate::{
    OperationsError,
    boolean::{BooleanOp, boolean},
};
use remus_math::{tolerance::Tolerance, vec::Point3};
use remus_topology::journal::EntityKey;
use remus_topology::{
    Topology,
    builder::make_circle_edge_with_ref,
    edge::EdgeCurve,
    explorer::{face_edges, solid_faces},
    face::{Face, FaceId, FaceSurface},
    solid::SolidId,
    wire::{OrientedEdge, Wire},
};
use std::f64::consts::TAU;

struct Mouth {
    cap: FaceId,
    cone: FaceId,
    center: Point3,
    angle: f64,
    rim: remus_topology::edge::EdgeId,
}

fn mouths(
    topo: &Topology,
    adj: &remus_topology::adjacency::AdjacencyIndex,
    cap: FaceId,
) -> Vec<Mouth> {
    let Ok(face) = topo.face(cap) else {
        return vec![];
    };
    let Some(normal) = face.effective_plane_normal() else {
        return vec![];
    };
    face.inner_wires()
        .iter()
        .filter_map(|&wire| {
            let [rim] = topo.wire(wire).ok()?.edges() else {
                return None;
            };
            let edge = topo.edge(rim.edge()).ok()?;
            let EdgeCurve::Circle(circle) = edge.curve() else {
                return None;
            };
            if !edge.is_closed() {
                return None;
            }
            let &cone = adj.faces_for_edge(rim.edge()).iter().find(|&&f| f != cap)?;
            let FaceSurface::Cone(surface) = topo.face(cone).ok()?.surface() else {
                return None;
            };
            if normal.cross(surface.axis()).length() > Tolerance::new().angular
                || !face_edges(topo, cone).ok()?.iter().any(|&e| {
                    topo.edge(e)
                        .is_ok_and(|e| matches!(e.curve(), EdgeCurve::NurbsCurve(_)))
                })
            {
                return None;
            }
            Some(Mouth {
                cap,
                rim: rim.edge(),
                cone,
                center: circle.center(),
                angle: std::f64::consts::FRAC_PI_2 - surface.half_angle(),
            })
        })
        .collect()
}

/// A possible chamfered screw mouth needs the full move path during a drag.
pub fn is_mouth(topo: &Topology, solid: SolidId, faces: &[FaceId]) -> bool {
    let [face] = faces else { return false };
    topo.face(*face)
        .is_ok_and(|f| f.effective_plane_normal().is_some() && !f.inner_wires().is_empty())
        && topo
            .build_adjacency(solid)
            .is_ok_and(|adj| !mouths(topo, &adj, *face).is_empty())
}

struct Bore {
    ends: [Mouth; 2],
    wall: Vec<FaceId>,
    frame: Frame,
    length: f64,
    form: ThreadForm,
    crest: f64,
    root: f64,
}

impl Bore {
    fn ring(
        &self,
        topo: &mut Topology,
        frame: &Frame,
        length: f64,
        phase: f64,
        tolerance: f64,
    ) -> Result<Ring, ThreadError> {
        thread_ring(
            topo,
            frame,
            &self.form,
            self.crest,
            self.root,
            0.0,
            phase,
            ThreadKind::Internal,
            self.root + SEAM_DEPTH * self.form.depth,
            length,
            tolerance,
            (
                ThreadEnd::Chamfer {
                    angle: self.ends[0].angle,
                },
                ThreadEnd::Chamfer {
                    angle: self.ends[1].angle,
                },
            ),
        )
    }
}

/// Move a cap of a recognized through-thread by rebuilding its screw section.
pub fn rebuild(
    topo: &mut Topology,
    solid: SolidId,
    faces: &[FaceId],
    distance: f64,
) -> Result<Option<DirectEditEvolution>, OperationsError> {
    let [cap] = faces else { return Ok(None) };
    if !distance.is_finite()
        || distance.abs() <= Tolerance::new().linear
        || topo.face(*cap)?.effective_plane_normal().is_none()
        || topo.face(*cap)?.inner_wires().is_empty()
    {
        return Ok(None);
    }
    let adjacency = topo.build_adjacency(solid)?;
    let mut candidates = mouths(topo, &adjacency, *cap).into_iter();
    let Some(first) = candidates.next() else {
        return Ok(None);
    };
    if candidates.next().is_some() {
        return Err(failure("the moving cap has multiple threaded mouths"));
    }
    let bore = Bore::recognize(topo, &adjacency, first)
        .ok_or_else(|| failure("could not reconstruct the complete thread form"))?;
    let verifies = |tolerance| {
        let mut template = Topology::new();
        bore.ring(&mut template, &bore.frame, bore.length, 0.0, tolerance)
            .is_ok_and(|ring| certificate::matches(topo, &bore, &template, &ring).is_some())
    };
    let inferred = fit::tolerance(topo, &bore);
    let tolerance = if let Some(t) = inferred
        && verifies(t)
    {
        t
    } else {
        let default = 0.5 * default_tolerance(bore.form.pitch);
        if !verifies(default) {
            return Err(failure(
                "the original thread walls do not match the reconstructed screw form",
            ));
        }
        default
    };
    let new_length = bore.length + distance;
    if new_length <= Tolerance::new().linear {
        return Err(failure("the thread has no remaining span"));
    }
    let (filled, map) = crate::defeature::defeature_with_history(topo, solid, &bore.wall)?;
    let filled_cap = *filled
        .face_map
        .get(&cap.index())
        .ok_or_else(|| failure("missing filled cap"))?;
    let (resized, moved_boundary) = crate::push_pull::move_faces_with_entity_evolution(
        topo,
        filled.solid,
        &[filled_cap],
        distance,
    )?;
    let moved_boundary: std::collections::BTreeMap<_, _> = moved_boundary.into_iter().collect();
    let mut boundary = filled
        .boundary_history
        .into_iter()
        .flatten()
        .filter_map(|(source, filled)| Some((source, *moved_boundary.get(&filled?)?)))
        .collect();
    let mut frame = bore.frame;
    let mut phase = 0.0;
    if bore.ends[0].cap == *cap {
        frame.origin = frame.origin - frame.axis * distance;
        let hand = if bore.form.hand == ThreadHand::Right {
            1.
        } else {
            -1.
        };
        phase -= hand * TAU * distance / bore.form.lead();
    }
    let seam = bore.root + SEAM_DEPTH * bore.form.depth;
    let rim = make_circle_edge_with_ref(
        topo,
        frame.origin,
        frame.axis,
        seam,
        frame.x,
        Tolerance::new().linear,
    )?;
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true)?);
    let disk = topo.add_face(Face::new(wire, vec![], frame.plane()));
    let cutter = crate::extrude::extrude(topo, disk, frame.axis, new_length)?;
    // Inspect the entire common solid. A small intersecting pocket must not
    // disappear below a relative-volume threshold on a long thread.
    let common = boolean(topo, BooleanOp::Intersect, resized.solid, cutter)?;
    if !full_barrel(topo, common, &frame, seam, new_length) {
        return Err(failure("the resized bore opens into another feature"));
    }
    let ring = bore
        .ring(topo, &frame, new_length, phase, tolerance)
        .map_err(|e| failure(&e.to_string()))?;
    require_storable_tolerances(topo, ring.solid, 2.0 * tolerance)
        .map_err(|e| failure(&e.to_string()))?;
    // The ring's two cap holes are the exact edges of its cone mouths.
    // Add them to the enlarged/shortened blank's caps and sew on the wall.
    // These cap handles belong to the temporary defeatured copy.
    let mut evolution = crate::blend_cluster::reblend::compose_evolution(&map, &resized.evolution);
    attach_rims(topo, &bore.ends, ring.solid, &evolution, &mut boundary)?;
    let mut shell_faces = solid_faces(topo, resized.solid)?;
    for (face, role) in ring.face_roles {
        shell_faces.push(face);
        if let ThreadFaceRole::Chamfer(side) = role {
            let index = usize::from(side == ThreadEndSide::End);
            let source = bore.ends[index].cone.index();
            evolution.deleted.remove(&source);
            evolution.add_modified(source, face.index());
        } else {
            // The generator repartitions the flanks and lands at the new length.
            evolution.add_generated(cap.index(), face.index());
        }
    }
    let shell = topo.add_shell(remus_topology::shell::Shell::new(shell_faces)?);
    let result = topo.add_solid(remus_topology::solid::Solid::new(shell, vec![]));
    crate::push_pull::validate_moved_solid(topo, faces, result)?;
    Ok(Some((
        MoveFacesResult {
            solid: result,
            evolution,
        },
        boundary,
    )))
}

fn attach_rims(
    topo: &mut Topology,
    ends: &[Mouth; 2],
    ring: SolidId,
    evolution: &crate::evolution::EvolutionMap,
    boundary: &mut Vec<(EntityKey, EntityKey)>,
) -> Result<(), OperationsError> {
    let ring_faces = solid_faces(topo, ring)?;
    for end in ends {
        let new_cap = evolution
            .modified
            .get(&end.cap.index())
            .and_then(|ids| match ids.as_slice() {
                [id] => topo.face_id_from_index(*id),
                _ => None,
            })
            .ok_or_else(|| failure("could not track the resized cap"))?;
        let normal = topo
            .face(new_cap)?
            .effective_plane_normal()
            .ok_or_else(|| failure("nonplanar resized cap"))?;
        let rim = ring_faces
            .iter()
            .find_map(|&f| {
                let face = topo.face(f).ok()?;
                let [rim] = face.inner_wires() else {
                    return None;
                };
                (face.effective_plane_normal()?.dot(normal) > 1.0 - Tolerance::new().angular)
                    .then_some(*rim)
            })
            .ok_or_else(|| failure("missing thread end rim"))?;
        let [opening] = topo.wire(rim)?.edges() else {
            return Err(failure("the thread mouth is not a single circle"));
        };
        let EdgeCurve::Circle(circle) = topo.edge(opening.edge())?.curve() else {
            return Err(failure("the thread mouth is not circular"));
        };
        let center = circle.center();
        let tol = Tolerance::new().linear;
        // Other material on the end plane could cover the cutter. Establish
        // that this exact cap contains the disk and no existing trim enters it.
        let on_cap = remus_check::distance::point_to_face(topo, center, new_cap)?
            .is_some_and(|(d, _)| d <= tol);
        let clear = face_edges(topo, new_cap)?.into_iter().all(|e| {
            remus_check::distance::point_to_edge(topo, center, e)
                .is_ok_and(|(d, _)| d > circle.radius() + tol)
        });
        if !on_cap || !clear {
            return Err(failure("the thread mouth crosses a cap boundary"));
        }
        let face = topo.face(new_cap)?;
        let outer = face.outer_wire();
        let mut inner = face.inner_wires().to_vec();
        inner.push(rim);
        let old_edge = topo.edge(end.rim)?;
        boundary.push((
            EntityKey::edge(end.rim.index()),
            EntityKey::edge(opening.edge().index()),
        ));
        boundary.push((
            EntityKey::vertex(old_edge.start().index()),
            EntityKey::vertex(topo.edge(opening.edge())?.start().index()),
        ));
        topo.set_face_boundary_wires(new_cap, outer, inner)?;
    }
    Ok(())
}

/// The intersection must have one straight cylindrical wall joining exactly
/// two complete circular disks at the requested ends.
fn full_barrel(topo: &Topology, solid: SolidId, frame: &Frame, radius: f64, length: f64) -> bool {
    let tol = Tolerance::new();
    let Ok(faces) = solid_faces(topo, solid) else {
        return false;
    };
    if faces.len() != 3 {
        return false;
    }
    let mut caps = [false; 2];
    let mut wall = false;
    for f in faces {
        let Ok(face) = topo.face(f) else { return false };
        if !face.inner_wires().is_empty() {
            return false;
        }
        match face.surface() {
            FaceSurface::Cylinder(c) => {
                if wall
                    || face.is_reversed()
                    || (c.radius() - radius).abs() > tol.linear
                    || c.axis().cross(frame.axis).length() > tol.angular
                    || (c.origin() - frame.origin).cross(frame.axis).length() > tol.linear
                {
                    return false;
                }
                wall = true;
            }
            FaceSurface::Plane { .. } => {
                let Ok(w) = topo.wire(face.outer_wire()) else {
                    return false;
                };
                let [oe] = w.edges() else { return false };
                let Ok(e) = topo.edge(oe.edge()) else {
                    return false;
                };
                let EdgeCurve::Circle(c) = e.curve() else {
                    return false;
                };
                let Ok((a, b)) = e.strict_domain() else {
                    return false;
                };
                if !e.is_closed()
                    || ((b - a).abs() - TAU).abs() > tol.angular
                    || (c.radius() - radius).abs() > tol.linear
                    || c.normal().cross(frame.axis).length() > tol.angular
                {
                    return false;
                }
                let Some(i) = [frame.origin, frame.origin + frame.axis * length]
                    .iter()
                    .position(|p| (*p - c.center()).length() <= tol.linear)
                else {
                    return false;
                };
                caps[i] = true;
            }
            FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => return false,
        }
    }
    wall && caps == [true, true]
}

fn failure(reason: &str) -> OperationsError {
    OperationsError::Unsupported {
        operation: "move thread mouth",
        reason: reason.into(),
    }
}
