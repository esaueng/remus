//! Face geometric validation checks.

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::face::FaceId;

use super::checks::{CheckId, EntityRef, Severity, ValidationIssue};
use crate::CheckError;

/// Check that a face has a valid surface (always true in current model,
/// but validates the face can be resolved).
pub fn check_face_has_surface(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let _face = topo.face(face_id)?;
    // In the current model, FaceSurface is always present (it's a required field).
    // This check validates the face entity exists and is resolvable.
    Ok(vec![])
}

/// Check face orientation consistency: the STORED outer-wire winding should
/// be consistent with the STORED surface normal.
///
/// Uses Newell's method on the outer wire polygon to determine winding,
/// then compares with the face surface normal at the polygon centroid.
///
/// The face's reversal flag plays no part: it mirrors the effective normal
/// and the effective edge traversal (`is_forward != is_reversed`) TOGETHER,
/// so a correctly emitted reversed face keeps its stored winding matched to
/// its stored normal (the convention every emitter follows — see the
/// orientation-emission campaign in remus-operations). Comparing against
/// the reversal-corrected normal instead made every correctly wound
/// reversed OPEN face score dot = −1 by construction (extruded ruled-NURBS
/// hole walls), while reversed WRAPPED walls stayed silent only because
/// Newell on a wrapped polygon is near-degenerate.
pub fn check_face_orientation(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let mut issues = Vec::new();
    let polygon = crate::util::face_polygon(topo, face_id)?;
    if polygon.len() >= 3 {
        let wire_normal = newell_normal(&polygon);
        if wire_normal.length() >= 1e-15 {
            let face = topo.face(face_id)?;
            let centroid = polygon_centroid(&polygon);

            let surface_normal = if let Some((u, v)) = face.surface().project_point(centroid) {
                face.surface().normal(u, v)
            } else {
                // Plane: use stored normal directly
                face.surface().normal(0.0, 0.0)
            };

            // Check if normals agree (dot product > 0 means same direction)
            let dot = wire_normal.dot(surface_normal);
            if dot < -0.1 {
                // Allow some tolerance for curved surfaces
                issues.push(ValidationIssue {
                    check: CheckId::FaceOrientationConsistency,
                    severity: Severity::Warning,
                    entity: EntityRef::Face(face_id),
                    description: format!(
                        "face normal inconsistent with wire winding (dot={dot:.3})"
                    ),
                    deviation: Some(dot.abs()),
                });
            }
        }
    }

    issues.extend(check_face_inner_wire_orientation(topo, face_id)?);
    Ok(issues)
}

/// Check that every planar inner wire traverses opposite to its outer wire.
///
/// A same-wound hole reverses the material-side convention and can poison the
/// orientation of a later boolean even when the source solid otherwise passes
/// shell and volume checks.
///
/// # Errors
///
/// Returns an error if the face, one of its wires, or an edge or vertex used by
/// those wires cannot be resolved.
pub fn check_face_inner_wire_orientation(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let face = topo.face(face_id)?;
    if !matches!(
        face.surface(),
        remus_topology::face::FaceSurface::Plane { .. }
    ) || face.inner_wires().is_empty()
    {
        return Ok(Vec::new());
    }

    let outer = crate::util::wire_polygon(topo, face.outer_wire())?;
    let Some(outer_normal) = raw_newell_normal(&outer) else {
        return Ok(Vec::new());
    };
    let mut issues = Vec::new();
    for &wire_id in face.inner_wires() {
        let inner = crate::util::wire_polygon(topo, wire_id)?;
        let Some(inner_normal) = raw_newell_normal(&inner) else {
            continue;
        };
        let alignment =
            outer_normal.dot(inner_normal) / (outer_normal.length() * inner_normal.length());
        if alignment > 0.1 {
            issues.push(ValidationIssue {
                check: CheckId::FaceOrientationConsistency,
                severity: Severity::Error,
                entity: EntityRef::Wire(wire_id),
                description: format!(
                    "inner wire {} on planar face {} has the same winding as its outer wire",
                    wire_id.index(),
                    face_id.index()
                ),
                deviation: Some(alignment),
            });
        }
    }
    Ok(issues)
}

/// Compute polygon normal via Newell's method.
fn newell_normal(verts: &[Point3]) -> Vec3 {
    crate::util::polygon_normal(verts)
}

fn raw_newell_normal(verts: &[Point3]) -> Option<Vec3> {
    if verts.len() < 3 {
        return None;
    }
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for (current, next) in verts
        .iter()
        .zip(verts.iter().cycle().skip(1))
        .take(verts.len())
    {
        normal += Vec3::new(
            (current.y() - next.y()) * (current.z() + next.z()),
            (current.z() - next.z()) * (current.x() + next.x()),
            (current.x() - next.x()) * (current.y() + next.y()),
        );
    }
    let length = normal.length();
    (length.is_finite() && length > 1e-30).then_some(normal)
}

/// Compute polygon centroid.
fn polygon_centroid(verts: &[Point3]) -> Point3 {
    let n = verts.len() as f64;
    let sx: f64 = verts.iter().map(|v| v.x()).sum();
    let sy: f64 = verts.iter().map(|v| v.y()).sum();
    let sz: f64 = verts.iter().map(|v| v.z()).sum();
    Point3::new(sx / n, sy / n, sz / n)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

    /// A unit square face in the XY plane. `ccw` picks the winding of the
    /// stored wire; `normal_z` picks the stored plane normal; `reversed`
    /// sets the face flag.
    fn square_face(topo: &mut Topology, ccw: bool, normal_z: f64, reversed: bool) -> FaceId {
        let mut pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        if !ccw {
            pts.reverse();
        }
        let vids: Vec<_> = pts
            .into_iter()
            .map(|p| topo.add_vertex(Vertex::new(p, 1e-7)))
            .collect();
        let mut oes = Vec::new();
        for i in 0..4 {
            let e = topo.add_edge(Edge::new(vids[i], vids[(i + 1) % 4], EdgeCurve::Line));
            oes.push(OrientedEdge::new(e, true));
        }
        let wire = topo.add_wire(Wire::new(oes, true).unwrap());
        let surface = FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, normal_z),
            d: 0.0,
        };
        if reversed {
            topo.add_face(Face::new_reversed(wire, vec![], surface))
        } else {
            topo.add_face(Face::new(wire, vec![], surface))
        }
    }

    /// The convention: STORED winding must match the STORED surface normal;
    /// the reversal flag never changes the verdict (it mirrors the effective
    /// normal and the effective traversal together — a correctly emitted
    /// reversed face keeps its stored pairing).
    #[test]
    fn stored_winding_vs_stored_normal_decides_regardless_of_flag() {
        let mut topo = Topology::new();
        // Consistent: CCW winding, +z normal — clean either flag value.
        for reversed in [false, true] {
            let fid = square_face(&mut topo, true, 1.0, reversed);
            assert!(
                check_face_orientation(&topo, fid).unwrap().is_empty(),
                "CCW winding with +z stored normal is consistent (reversed={reversed})"
            );
        }
        // Flipped: CCW winding, -z stored normal — warns either flag value.
        for reversed in [false, true] {
            let fid = square_face(&mut topo, true, -1.0, reversed);
            let issues = check_face_orientation(&topo, fid).unwrap();
            assert_eq!(
                issues.len(),
                1,
                "CCW winding with -z stored normal is flipped (reversed={reversed})"
            );
            assert_eq!(issues[0].check, CheckId::FaceOrientationConsistency);
        }
        // Flipped the other way: CW winding, +z stored normal.
        let fid = square_face(&mut topo, false, 1.0, false);
        assert_eq!(check_face_orientation(&topo, fid).unwrap().len(), 1);
    }

    #[test]
    fn planar_inner_wire_must_wind_opposite_to_outer_wire() {
        let mut topo = Topology::new();
        let outer = remus_topology::builder::make_polygon_wire(
            &mut topo,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
                Point3::new(4.0, 4.0, 0.0),
                Point3::new(0.0, 4.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let same_wound_inner = remus_topology::builder::make_polygon_wire(
            &mut topo,
            &[
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(3.0, 1.0, 0.0),
                Point3::new(3.0, 3.0, 0.0),
                Point3::new(1.0, 3.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let face = topo.add_face(Face::new(
            outer,
            vec![same_wound_inner],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));

        let issues = check_face_orientation(&topo, face).unwrap();
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].severity, Severity::Error);
        assert!(issues[0].description.contains("same winding"));
    }
}
