//! Topology-preserving radial edits of closed analytic surfaces of revolution.
//!
//! A cylinder's tangent torus bands keep their minor radius while their major
//! radius changes by the cylinder's radial displacement. A planar support
//! holds the band's axial level fixed. A conical support holds its meridian
//! line fixed, so the band also translates axially by `dr / tan(angle)` with
//! the cone's axis sign. Existing circles and seam arcs retain their signed
//! parameter spans, and copied entities retain their construction identity.
//!
//! The bounded cell is one shell of at most 64 coaxial plane/cylinder/cone/
//! torus faces, with complete circular rims and rectangular torus strips.
//! An exact meridian arrangement rejects new intersections, collapsed or
//! reversed spans, and lost wall thickness before the result is returned.
//! Its closed-form revolution integral independently checks the B-rep volume.

mod profile;
use crate::{
    OperationsError,
    copy::{CopyReplacements, copy_solid_replacing_with_entity_map},
    push_pull::{DirectEditEvolution, MoveFacesResult, exact_face_evolution},
};
use remus_math::{
    curves::Circle3D,
    surfaces::{CylindricalSurface, ToroidalSurface},
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::{EdgeCurve, EdgeId},
    explorer::{edge_to_face_map, face_edges, solid_faces},
    face::{FaceId, FaceSurface},
    solid::SolidId,
};
use std::collections::BTreeSet;

use super::reconstruction;

pub fn resize_cylindrical_strip(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    radius: f64,
) -> Result<Option<DirectEditEvolution>, OperationsError> {
    let FaceSurface::Cylinder(wall) = topo.face(face)?.surface().clone() else {
        return Ok(None);
    };
    let faces = solid_faces(topo, solid)?;
    if faces.len() > 64 || !topo.solid(solid)?.inner_shells().is_empty() {
        return Ok(None);
    }
    let axis = wall.axis();
    let origin = wall.origin();
    let tolerance = Tolerance::new();
    let coaxial = |point: Point3, direction: Vec3| {
        direction.cross(axis).length() <= tolerance.angular
            && (point - origin).cross(axis).length() <= tolerance.linear
    };
    for &candidate in &faces {
        let qualified = match topo.face(candidate)?.surface() {
            FaceSurface::Plane { normal, .. } => normal.cross(axis).length() <= tolerance.angular,
            FaceSurface::Cylinder(c) => coaxial(c.origin(), c.axis()),
            FaceSurface::Cone(c) => coaxial(c.apex(), c.axis()),
            FaceSurface::Torus(t) => coaxial(t.center(), t.z_axis()),
            FaceSurface::Sphere(_) | FaceSurface::Nurbs(_) => false,
        };
        if !qualified {
            return Ok(None);
        }
    }
    let adjacency = edge_to_face_map(topo, solid)?;
    let delta = radius - wall.radius();
    let mut replacements = CopyReplacements::default();
    replacements.surfaces.insert(
        face.index(),
        FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
            origin,
            axis,
            radius,
            wall.x_axis(),
        )?),
    );
    let mut bands = BTreeSet::new();
    let mut rims = 0;
    for edge in face_edges(topo, face)? {
        let neighbors = &adjacency[&edge.index()];
        let Some(&neighbor) = neighbors.iter().find(|&&other| other != face) else {
            continue;
        };
        let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve() else {
            return Ok(None);
        };
        if !coaxial(circle.center(), circle.normal()) {
            return Ok(None);
        }
        rims += 1;
        match topo.face(neighbor)?.surface() {
            FaceSurface::Plane { .. } => {
                replace_circle(topo, edge, delta, 0.0, origin, axis, &mut replacements)?;
            }
            FaceSurface::Torus(torus) => {
                if !bands.insert(neighbor.index()) {
                    continue;
                }
                if ((torus.major_radius() - wall.radius()).abs() - torus.minor_radius()).abs()
                    > tolerance.linear
                {
                    return Ok(None);
                }
                let band = super::describe_band(topo, solid, neighbor)?;
                if band.faces != [neighbor]
                    || band.supports.len() != 2
                    || !band.supports.contains(&face)
                {
                    return Ok(None);
                }
                let support = *band
                    .supports
                    .iter()
                    .find(|&&support| support != face)
                    .ok_or_else(|| reconstruction("radial band has no other support"))?;
                let axial = match topo.face(support)?.surface() {
                    FaceSurface::Plane { .. } => 0.0,
                    FaceSurface::Cone(cone) => {
                        delta / (cone.axis().dot(axis) * cone.half_angle().tan())
                    }
                    FaceSurface::Cylinder(_)
                    | FaceSurface::Sphere(_)
                    | FaceSurface::Torus(_)
                    | FaceSurface::Nurbs(_) => return Ok(None),
                };
                let major = torus.major_radius() + delta;
                if major <= torus.minor_radius() + tolerance.linear {
                    return Err(reconstruction("radial resize collapses a torus ring"));
                }
                replacements.surfaces.insert(
                    neighbor.index(),
                    FaceSurface::Torus(ToroidalSurface::with_axis_and_ref_dir(
                        torus.center() + axis * axial,
                        major,
                        torus.minor_radius(),
                        torus.z_axis(),
                        torus.x_axis(),
                    )?),
                );
                for boundary in face_edges(topo, neighbor)? {
                    replace_circle(
                        topo,
                        boundary,
                        delta,
                        axial,
                        origin,
                        axis,
                        &mut replacements,
                    )?;
                }
            }
            FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Nurbs(_) => return Ok(None),
        }
    }
    if rims != 2 || bands.is_empty() {
        return Ok(None);
    }
    let Some(source_profile) = profile::Profile::read(topo, solid, origin, axis)? else {
        return Ok(None);
    };
    source_profile.verify_result(topo, solid)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        let copied = copy_solid_replacing_with_entity_map(topo, solid, replacements)?;
        let profile = profile::Profile::read(topo, copied.solid, origin, axis)?
            .ok_or_else(|| reconstruction("radial result lost its meridian profile"))?;
        source_profile.preserve_directions(
            &profile,
            copied
                .face_map
                .iter()
                .map(|(&source, &target)| (source, target)),
        )?;
        profile.verify_result(topo, copied.solid)?;
        let report = remus_check::validate::validate_solid(
            topo,
            copied.solid,
            &remus_check::validate::ValidateOptions::default(),
        )?;
        if !report.is_valid() {
            return Err(reconstruction(format!(
                "radial strip failed validation: {:?}",
                report.issues
            )));
        }
        let boundary = copied
            .edge_map
            .iter()
            .map(|(&source, &target)| {
                (
                    remus_topology::journal::EntityKey::edge(source),
                    remus_topology::journal::EntityKey::edge(target.index()),
                )
            })
            .chain(copied.vertex_map.iter().map(|(&source, &target)| {
                (
                    remus_topology::journal::EntityKey::vertex(source),
                    remus_topology::journal::EntityKey::vertex(target.index()),
                )
            }))
            .collect();
        let evolution = exact_face_evolution(topo, &faces, copied.solid, copied.face_map)?;
        Ok(Some((
            MoveFacesResult {
                solid: copied.solid,
                evolution,
            },
            boundary,
        )))
    })
}

fn replace_circle(
    topo: &Topology,
    edge: EdgeId,
    delta: f64,
    axial: f64,
    origin: Point3,
    axis: Vec3,
    replacements: &mut CopyReplacements,
) -> Result<(), OperationsError> {
    if replacements.edges.contains_key(&edge.index()) {
        return Ok(());
    }
    let source = topo.edge(edge)?;
    let EdgeCurve::Circle(circle) = source.curve() else {
        // A periodic band's duplicated seam is circular. A noncircular
        // boundary has not established the meridian transport contract.
        return Err(reconstruction(
            "radial band requires circular rims and meridian seam",
        ));
    };
    let curve = if circle.normal().cross(axis).length() <= Tolerance::new().angular {
        Circle3D::new_with_ref(
            circle.center() + axis * axial,
            circle.normal(),
            circle.radius() + delta,
            circle.u_axis(),
        )?
    } else {
        let offset = circle.center() - origin;
        let radial = offset - axis * offset.dot(axis);
        if radial.length() <= Tolerance::new().linear {
            return Err(reconstruction("radial meridian seam has no radial center"));
        }
        Circle3D::new_with_ref(
            circle.center() + radial * (delta / radial.length()) + axis * axial,
            circle.normal(),
            circle.radius(),
            circle.u_axis(),
        )?
    };
    let range = source.strict_domain().map_err(|error| {
        reconstruction(format!("radial rim has no parameter authority: {error}"))
    })?;
    for (vertex, t) in [(source.start(), range.0), (source.end(), range.1)] {
        let point = curve.evaluate(t);
        if let Some(previous) = replacements.vertices.insert(vertex.index(), point)
            && (point - previous).length() > Tolerance::new().linear
        {
            return Err(reconstruction(
                "radial rim transports disagree at a shared vertex",
            ));
        }
    }
    replacements
        .edges
        .insert(edge.index(), (EdgeCurve::Circle(curve), source.trim()));
    Ok(())
}
