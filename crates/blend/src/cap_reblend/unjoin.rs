//! Restore the full analytic cap after its shoulder separates from the rim.

use remus_math::curves::Circle3D;
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::Point3;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{face_vertices, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{FaceId, SolidId, Topology};

use super::{CapReblendApproximation, CapReblendOutcome, SupportExtension, height, refusal};
use crate::builder_utils::add_certified_curve_edge;
use crate::{BlendEngine, BlendError, BlendFaceOrigins, BlendResult};

/// Restore a full rounded cap on an owned candidate whose lower limits were restored.
///
/// The complete shoulder spring must lie strictly below the cap's lower
/// contact plane. A back-side boundary vertex supplies the periodic seam;
/// its position is kept unchanged. The transient candidate is caller-owned.
///
/// # Errors
/// Refuses missing support roles, an ambiguous seam or a shoulder that still
/// meets the rim. A refusal restores the candidate atomically.
pub fn restore_unjoined_cap_in_place(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    extension: &SupportExtension,
    permission: CapReblendApproximation,
) -> Result<CapReblendOutcome, BlendError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = restore(topo, solid, cap, rims, radius, extension, permission);
    if result.is_ok() {
        snapshot.commit(topo);
    } else {
        snapshot.restore(topo);
    }
    result
}

fn restore(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    extension: &SupportExtension,
    permission: CapReblendApproximation,
) -> Result<CapReblendOutcome, BlendError> {
    if !topo.solid(solid)?.inner_shells().is_empty() {
        return Err(refusal("cap separation does not support cavity shells"));
    }
    let cap_data = topo.face(cap)?;
    let axis = cap_data
        .effective_plane_normal()
        .ok_or_else(|| refusal("separated cap is not planar"))?;
    let FaceSurface::Plane { normal, d } = cap_data.surface() else {
        return Err(refusal("separated cap changed carrier"));
    };
    let top_level = *d * normal.dot(axis);
    let level = top_level - radius;
    let FaceSurface::Torus(shoulder) = topo.face(extension.shoulder)?.surface() else {
        return Err(refusal("separated shoulder changed carrier"));
    };
    if height(shoulder.center(), axis) >= level - 1e-7 {
        return Err(refusal("shoulder still meets its fixed rim"));
    }
    let boss = extension.boss;
    let data = topo.face(boss)?;
    let FaceSurface::Cylinder(cylinder) = data.surface() else {
        return Err(refusal("separated boss changed carrier"));
    };
    if data.is_reversed() || !data.inner_wires().is_empty() || cylinder.radius() <= radius {
        return Err(refusal(
            "separated cap requires one outer cylindrical support",
        ));
    }
    let cylinder = cylinder.clone();
    let spring = topo.edge(extension.spring)?;
    let radial = |point: Point3| {
        let delta = point - cylinder.origin();
        (delta - axis * delta.dot(axis)).normalize()
    };
    let front = (radial(topo.vertex(spring.start())?.point())?
        + radial(topo.vertex(spring.end())?.point())?)
    .normalize()?;
    let vertices = face_vertices(topo, boss)?;
    let bottom = vertices
        .iter()
        .map(|&v| topo.vertex(v).map(|v| height(v.point(), axis)))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    let mut candidates = Vec::new();
    for vertex in vertices {
        let point = topo.vertex(vertex)?.point();
        if (height(point, axis) - bottom).abs() < 1e-7 && radial(point)?.dot(front) < -0.1 {
            candidates.push(vertex);
        }
    }
    let seam = candidates
        .into_iter()
        .min_by_key(|vertex| vertex.index())
        .ok_or_else(|| refusal("separated cap has no existing back-side seam vertex"))?;
    let reference = radial(topo.vertex(seam)?.point())?;
    let center = cylinder.origin() + axis * (level - height(cylinder.origin(), axis));
    let major = cylinder.radius() - radius;
    let lower = topo.add_vertex(Vertex::new(center + reference * cylinder.radius(), 1e-7));
    let upper = topo.add_vertex(Vertex::new(
        center + reference * major + axis * radius,
        1e-7,
    ));
    let lower_curve = Circle3D::new_with_ref(center, axis, cylinder.radius(), reference)?;
    let upper_curve = Circle3D::new_with_ref(center + axis * radius, axis, major, reference)?;
    let lower_edge = add_certified_curve_edge(
        topo,
        lower,
        lower,
        EdgeCurve::Circle(lower_curve),
        (0.0, std::f64::consts::TAU),
    )?;
    let upper_edge = add_certified_curve_edge(
        topo,
        upper,
        upper,
        EdgeCurve::Circle(upper_curve),
        (0.0, std::f64::consts::TAU),
    )?;
    let arc = Circle3D::new_with_ref(
        center + reference * major,
        reference.cross(axis),
        radius,
        reference,
    )?;
    let arc = add_certified_curve_edge(
        topo,
        lower,
        upper,
        EdgeCurve::Circle(arc),
        (0.0, std::f64::consts::FRAC_PI_2),
    )?;
    let vertical = topo.add_edge(Edge::new(seam, lower, EdgeCurve::Line));
    let wire = topo.wire(topo.face(boss)?.outer_wire())?.edges().to_vec();
    let position = wire
        .iter()
        .position(|oe| {
            topo.edge(oe.edge())
                .is_ok_and(|e| oe.oriented_start(e) == seam)
        })
        .ok_or_else(|| refusal("back-side seam vertex is absent from its support wire"))?;
    let mut rewritten = wire[..position].to_vec();
    rewritten.extend([
        OrientedEdge::new(vertical, true),
        OrientedEdge::new(lower_edge, false),
        OrientedEdge::new(vertical, false),
    ]);
    rewritten.extend_from_slice(&wire[position..]);
    let wire = topo.add_wire(Wire::new(rewritten, true)?);
    topo.face_mut(boss)?
        .set_surface(FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            cylinder.radius(),
            reference,
        )?));
    topo.set_face_boundary_wires(boss, wire, vec![])?;
    let wire = topo.add_wire(Wire::new(
        vec![OrientedEdge::new(
            upper_edge,
            !topo.face(cap)?.is_reversed(),
        )],
        true,
    )?);
    topo.set_face_boundary_wires(cap, wire, vec![])?;
    let wire = topo.add_wire(Wire::new(
        vec![
            OrientedEdge::new(lower_edge, true),
            OrientedEdge::new(arc, true),
            OrientedEdge::new(upper_edge, false),
            OrientedEdge::new(arc, false),
        ],
        true,
    )?);
    let rim = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Torus(ToroidalSurface::with_axis_and_ref_dir(
            center, major, radius, axis, reference,
        )?),
    ));
    let originals = solid_faces(topo, solid)?;
    let kept: Vec<_> = originals
        .iter()
        .copied()
        .filter(|f| !rims.contains(f))
        .collect();
    let shell = topo.add_shell(Shell::new(kept.iter().copied().chain([rim]).collect())?);
    let result = topo.add_solid(Solid::new(shell, vec![]));
    let mut parents = vec![cap, boss];
    parents.extend_from_slice(rims);
    Ok(CapReblendOutcome {
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
    })
}
