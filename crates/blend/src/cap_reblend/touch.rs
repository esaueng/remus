//! Analytic shared-boundary topology at the instant a shoulder reaches its rim.

use std::collections::BTreeSet;
use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::Point3;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{face_edges, face_vertices, solid_edges, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{FaceId, SolidId, Topology};

use super::{CapReblendApproximation, CapReblendOutcome, height, refusal};
use crate::builder_utils::add_certified_curve_edge;
use crate::{BlendEngine, BlendError, BlendFaceOrigins, BlendResult};

/// Recover the complete lower support wire from an owned touching candidate.
///
/// The old cap bands remain disposable; their shared contact with the boss
/// is replaced by the existing shoulder spring before rebuilding the cap.
/// Returns `None` for a cap without the touching shoulder/rim incidence.
///
/// # Errors
/// Refuses ambiguous support roles or a non-contiguous old rim contact,
/// restoring the candidate atomically. No deliverable source may share it.
pub fn restore_touching_support_limits_in_place(
    topo: &mut Topology,
    solid: SolidId,
    rims: &[FaceId],
    axis: remus_math::vec::Vec3,
) -> Result<Option<super::SupportExtension>, BlendError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = restore_touching(topo, solid, rims, axis);
    if result.is_ok() {
        snapshot.commit(topo);
    } else {
        snapshot.restore(topo);
    }
    result
}

fn restore_touching(
    topo: &mut Topology,
    solid: SolidId,
    rims: &[FaceId],
    axis: remus_math::vec::Vec3,
) -> Result<Option<super::SupportExtension>, BlendError> {
    let [rim] = rims else { return Ok(None) };
    let FaceSurface::Torus(cap_torus) = topo.face(*rim)?.surface() else {
        return Ok(None);
    };
    let boss_radius = cap_torus.major_radius() + cap_torus.minor_radius();
    let adjacency = topo.build_adjacency(solid)?;
    let mut springs = Vec::new();
    let mut bosses = BTreeSet::new();
    for edge in face_edges(topo, *rim)? {
        for &face in adjacency.faces_for_edge(edge) {
            if face == *rim {
                continue;
            }
            match topo.face(face)?.surface() {
                FaceSurface::Torus(torus)
                    if torus.z_axis().dot(axis) > 1.0 - 1e-10
                        && torus.minor_radius() > cap_torus.minor_radius()
                        && (torus.major_radius() - torus.minor_radius() - boss_radius).abs()
                            < 1e-7 =>
                {
                    if let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve()
                        && (circle.radius() - boss_radius).abs() < 1e-7
                        && circle.normal().dot(axis).abs() > 1.0 - 1e-10
                    {
                        springs.push((edge, face));
                    }
                }
                FaceSurface::Cylinder(cylinder)
                    if cylinder.axis().dot(axis) > 1.0 - 1e-10
                        && (cylinder.radius() - boss_radius).abs() < 1e-7 =>
                {
                    bosses.insert(face);
                }
                _ => {}
            }
        }
    }
    if springs.is_empty() {
        return Ok(None);
    }
    let [(spring, shoulder)] = springs.as_slice() else {
        return Err(refusal("touching cap has ambiguous shoulder contacts"));
    };
    let [boss] = bosses.into_iter().collect::<Vec<_>>()[..] else {
        return Err(refusal("touching cap has ambiguous cylindrical supports"));
    };
    let (spring, shoulder) = (*spring, *shoulder);
    let face = topo.face(boss)?;
    if !face.inner_wires().is_empty() {
        return Err(refusal("touching cap support has holes"));
    }
    let old = topo.wire(face.outer_wire())?.edges().to_vec();
    let contacts: Vec<_> = old
        .iter()
        .enumerate()
        .filter(|(_, oe)| adjacency.faces_for_edge(oe.edge()).contains(rim))
        .map(|(i, _)| i)
        .collect();
    if contacts.is_empty() || contacts.len() == old.len() {
        return Err(refusal("touching cap has no bounded cylindrical contact"));
    }
    let starts: Vec<_> = contacts
        .iter()
        .copied()
        .filter(|&i| !contacts.contains(&((i + old.len() - 1) % old.len())))
        .collect();
    let [start] = starts.as_slice() else {
        return Err(refusal("touching cap contact is not contiguous"));
    };
    let start = *start;
    if !(0..contacts.len()).all(|i| contacts.contains(&((start + i) % old.len()))) {
        return Err(refusal("touching cap contact is not one path"));
    }
    let first = old[start];
    let last = old[(start + contacts.len() - 1) % old.len()];
    let from = first.oriented_start(topo.edge(first.edge())?);
    let to = last.oriented_end(topo.edge(last.edge())?);
    let spring_data = topo.edge(spring)?;
    let forward = if spring_data.start() == from && spring_data.end() == to {
        true
    } else if spring_data.end() == from && spring_data.start() == to {
        false
    } else {
        return Err(refusal(
            "touching cap contact endpoints differ from the shoulder spring",
        ));
    };
    let mut wire = vec![OrientedEdge::new(spring, forward)];
    wire.extend((contacts.len()..old.len()).map(|i| old[(start + i) % old.len()]));
    let wire = topo.add_wire(Wire::new(wire, true)?);
    topo.set_face_boundary_wires(boss, wire, vec![])?;
    Ok(Some(super::SupportExtension {
        edge_images: vec![],
        spring,
        boss,
        shoulder,
    }))
}

/// Rebuild contact onset on a caller-owned disposable translation candidate.
///
/// Returns `None` when no qualified shoulder spring is at the cap's lower
/// contact plane. A qualified contact shares the existing spring between
/// shoulder and rim, instead of constructing collapsed corner bands.
///
/// # Errors
/// Ambiguous springs, periodic seams, unsupported boundaries or inconsistent
/// analytic carriers refuse atomically. The input candidate must be owned.
pub fn rebuild_touching_cap_in_place(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    permission: CapReblendApproximation,
) -> Result<Option<CapReblendOutcome>, BlendError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = rebuild(topo, solid, cap, rims, radius, permission);
    if result.is_ok() {
        snapshot.commit(topo);
    } else {
        snapshot.restore(topo);
    }
    result
}

fn rebuild(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    permission: CapReblendApproximation,
) -> Result<Option<CapReblendOutcome>, BlendError> {
    let cap_data = topo.face(cap)?;
    let Some(axis) = cap_data.effective_plane_normal() else {
        return Ok(None);
    };
    let FaceSurface::Plane { normal, d } = cap_data.surface() else {
        return Ok(None);
    };
    let level = *d * normal.dot(axis) - radius;
    let adjacency = topo.build_adjacency(solid)?;
    let mut contacts = Vec::new();
    for edge in solid_edges(topo, solid)? {
        let data = topo.edge(edge)?;
        let EdgeCurve::Circle(circle) = data.curve() else {
            continue;
        };
        if circle.normal().dot(axis).abs() < 1.0 - 1e-10
            || (height(circle.center(), axis) - level).abs() > 1e-9
        {
            continue;
        }
        let neighbors = adjacency.faces_for_edge(edge);
        if neighbors.len() != 2 || neighbors[0] == neighbors[1] {
            continue;
        }
        for &(boss, shoulder) in &[(neighbors[0], neighbors[1]), (neighbors[1], neighbors[0])] {
            let (FaceSurface::Cylinder(cylinder), FaceSurface::Torus(torus)) =
                (topo.face(boss)?.surface(), topo.face(shoulder)?.surface())
            else {
                continue;
            };
            if !rims.contains(&shoulder)
                && cylinder.axis().dot(axis) > 1.0 - 1e-10
                && torus.z_axis().dot(axis) > 1.0 - 1e-10
                && torus.minor_radius() > radius
                && (height(torus.center(), axis) - level).abs() < 1e-9
                && (torus.major_radius() - torus.minor_radius() - cylinder.radius()).abs() < 1e-7
                && (circle.radius() - cylinder.radius()).abs() < 1e-7
            {
                contacts.push((edge, boss, shoulder));
            }
        }
    }
    if contacts.is_empty() {
        return Ok(None);
    }
    let [(spring, boss, shoulder)] = contacts.as_slice() else {
        return Err(refusal("contact onset has ambiguous shoulder springs"));
    };
    let (spring, boss, shoulder) = (*spring, *boss, *shoulder);
    let source_faces = solid_faces(topo, solid)?;
    let cap_neighbors: BTreeSet<_> = face_edges(topo, cap)?
        .into_iter()
        .flat_map(|e| adjacency.faces_for_edge(e).iter().copied())
        .filter(|&f| f != cap)
        .collect();
    if rims.iter().copied().collect::<BTreeSet<_>>() != cap_neighbors
        || rims.len() != cap_neighbors.len()
        || !topo.solid(solid)?.inner_shells().is_empty()
        || [cap, boss, shoulder]
            .iter()
            .any(|&f| topo.face(f).is_ok_and(|f| !f.inner_wires().is_empty()))
    {
        return Err(refusal(
            "contact onset requires complete cap bands and outer supports",
        ));
    }
    let FaceSurface::Cylinder(cylinder) = topo.face(boss)?.surface() else {
        return Err(refusal("contact onset lost its cylindrical support"));
    };
    if topo.face(boss)?.is_reversed() || cylinder.radius() <= radius {
        return Err(refusal(
            "contact onset requires an outer cylindrical support",
        ));
    }
    let cylinder = cylinder.clone();
    let data = topo.edge(spring)?;
    let ends = [data.start(), data.end()];
    let points = [topo.vertex(ends[0])?.point(), topo.vertex(ends[1])?.point()];
    let radial = |point: Point3| {
        let delta = point - cylinder.origin();
        (delta - axis * delta.dot(axis)).normalize()
    };
    let front = (radial(points[0])? + radial(points[1])?).normalize()?;
    let reference = face_vertices(topo, boss)?
        .into_iter()
        .find_map(|v| {
            let p = topo.vertex(v).ok()?.point();
            let radial = radial(p).ok()?;
            (height(p, axis) < level - 1e-7 && radial.dot(front) < -0.1).then_some(radial)
        })
        .ok_or_else(|| refusal("contact onset has no independent back-side seam"))?;
    let center = cylinder.origin() + axis * (level - height(cylinder.origin(), axis));
    let circle = Circle3D::new_with_ref(center, axis, cylinder.radius(), reference)?;
    let mut parameters = [
        circle.project(points[0]).rem_euclid(TAU),
        circle.project(points[1]).rem_euclid(TAU),
    ];
    let mut vertices = ends;
    if parameters[0] > parameters[1] {
        parameters.swap(0, 1);
        vertices.swap(0, 1);
    }
    if parameters[0] <= 1e-8
        || parameters[1] >= TAU - 1e-8
        || parameters[1] - parameters[0] >= PI
        || parameters[1] - parameters[0] < 1e-8
        || points
            .iter()
            .any(|&p| (height(p, axis) - level).abs() > 1e-7)
    {
        return Err(refusal(
            "contact onset is not one nondegenerate minor spring",
        ));
    }
    let old_wire = topo.wire(topo.face(boss)?.outer_wire())?.edges().to_vec();
    let mut removed = BTreeSet::new();
    let mut detours = 0;
    for start in 0..old_wire.len() {
        let seam = old_wire[start];
        let e = topo.edge(seam.edge())?;
        if !matches!(e.curve(), EdgeCurve::Line) {
            continue;
        }
        for count in 3..old_wire.len() {
            let end = (start + count - 1) % old_wire.len();
            if old_wire[end].edge() == seam.edge()
                && old_wire[end].is_forward() != seam.is_forward()
                && (1..count - 1).all(|i| {
                    adjacency
                        .faces_for_edge(old_wire[(start + i) % old_wire.len()].edge())
                        .iter()
                        .any(|f| rims.contains(f))
                })
            {
                detours += 1;
                removed.extend((0..count).map(|i| (start + i) % old_wire.len()));
            }
        }
    }
    if detours > 1 {
        return Err(refusal("contact onset has ambiguous cap-ring excursions"));
    }
    let lower = topo.add_vertex(Vertex::new(circle.evaluate(0.0), 1e-7));
    let first = add_certified_curve_edge(
        topo,
        lower,
        vertices[0],
        EdgeCurve::Circle(circle.clone()),
        (0.0, parameters[0]),
    )?;
    let last = add_certified_curve_edge(
        topo,
        vertices[1],
        lower,
        EdgeCurve::Circle(circle),
        (parameters[1], TAU),
    )?;
    let mut boss_wire = Vec::new();
    let mut found = false;
    for (i, oe) in old_wire.into_iter().enumerate() {
        if removed.contains(&i) {
            continue;
        }
        if oe.edge() == spring {
            if found {
                return Err(refusal("contact onset repeats its shoulder spring"));
            }
            found = true;
            let forward = oe.oriented_start(topo.edge(spring)?) == vertices[1];
            boss_wire.extend(if forward {
                [
                    OrientedEdge::new(last, true),
                    OrientedEdge::new(first, true),
                ]
            } else {
                [
                    OrientedEdge::new(first, false),
                    OrientedEdge::new(last, false),
                ]
            });
        } else {
            boss_wire.push(oe);
        }
    }
    if !found {
        return Err(refusal("contact onset lost its boss spring"));
    }
    let boss_wire = topo.add_wire(Wire::new(boss_wire, true)?);
    topo.face_mut(boss)?
        .set_surface(FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            cylinder.radius(),
            front,
        )?));
    topo.set_face_boundary_wires(boss, boss_wire, vec![])?;
    let major = cylinder.radius() - radius;
    let upper = topo.add_vertex(Vertex::new(
        center + reference * major + axis * radius,
        1e-7,
    ));
    let upper_circle = Circle3D::new_with_ref(center + axis * radius, axis, major, reference)?;
    let upper_edge = add_certified_curve_edge(
        topo,
        upper,
        upper,
        EdgeCurve::Circle(upper_circle),
        (0.0, TAU),
    )?;
    let arc = Circle3D::new_with_ref(
        center + reference * major,
        reference.cross(axis),
        radius,
        reference,
    )?;
    let arc =
        add_certified_curve_edge(topo, lower, upper, EdgeCurve::Circle(arc), (0.0, PI * 0.5))?;
    let cap_wire = topo.add_wire(Wire::new(
        vec![OrientedEdge::new(
            upper_edge,
            !topo.face(cap)?.is_reversed(),
        )],
        true,
    )?);
    topo.set_face_boundary_wires(cap, cap_wire, vec![])?;
    let rim_wire = topo.add_wire(Wire::new(
        vec![
            OrientedEdge::new(first, true),
            OrientedEdge::new(spring, topo.edge(spring)?.start() == vertices[0]),
            OrientedEdge::new(last, true),
            OrientedEdge::new(arc, true),
            OrientedEdge::new(upper_edge, false),
            OrientedEdge::new(arc, false),
        ],
        true,
    )?);
    let rim = topo.add_face(Face::new(
        rim_wire,
        vec![],
        FaceSurface::Torus(ToroidalSurface::with_axis_and_ref_dir(
            center, major, radius, axis, reference,
        )?),
    ));
    let kept: Vec<_> = source_faces
        .into_iter()
        .filter(|f| !rims.contains(f))
        .collect();
    let shell = topo.add_shell(Shell::new(kept.iter().copied().chain([rim]).collect())?);
    let result = topo.add_solid(Solid::new(shell, vec![]));
    let mut parents = vec![cap, boss, shoulder];
    parents.extend_from_slice(rims);
    Ok(Some(CapReblendOutcome {
        corner_stations: permission.stations,
        edge_images: vec![],
        result: BlendResult {
            solid: result,
            succeeded: vec![],
            failed: vec![],
            is_partial: false,
            engine: BlendEngine::Walking,
            face_origins: Some(BlendFaceOrigins {
                survived: kept.into_iter().map(|f| (f, f)).collect(),
                deleted: rims.to_vec(),
                created: vec![(rim, parents)],
                created_unattributed: vec![],
            }),
        },
    }))
}
