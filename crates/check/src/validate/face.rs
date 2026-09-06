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

/// Check that every inner wire traverses opposite to its outer wire.
///
/// A same-wound hole reverses the material-side convention and can poison the
/// orientation of a later boolean even when the source solid otherwise passes
/// shell and volume checks.
///
/// Planar faces are judged in 3D (Newell normals). The analytic periodic
/// surfaces (cylinder, cone, sphere, torus) are judged in the surface's
/// (u, v) parameter space with the seam unwrapped the way the ray-cast
/// classifier unwraps it (`classify::boundary`), so a hole crossing the seam
/// still gets a coherent winding. NURBS faces are skipped: their parameter
/// direction carries no handedness guarantee relative to the surface normal.
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
    if face.inner_wires().is_empty() {
        return Ok(Vec::new());
    }
    match face.surface() {
        remus_topology::face::FaceSurface::Plane { .. } => {
            check_planar_inner_wire_orientation(topo, face_id)
        }
        remus_topology::face::FaceSurface::Cylinder(_)
        | remus_topology::face::FaceSurface::Cone(_)
        | remus_topology::face::FaceSurface::Sphere(_)
        | remus_topology::face::FaceSurface::Torus(_) => {
            check_periodic_inner_wire_orientation(topo, face_id)
        }
        remus_topology::face::FaceSurface::Nurbs(_) => Ok(Vec::new()),
    }
}

/// Planar inner-wire winding, compared through Newell normals.
fn check_planar_inner_wire_orientation(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    let face = topo.face(face_id)?;

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

/// How a wire's UV polygon carries its winding verdict. The payload is the
/// winding's handedness (`true` = positive/CCW).
enum UvWinding {
    /// A full-turn ring around a periodic coordinate (one rim of a
    /// two-ring band face): the sampled polygon is a period-long sliver whose
    /// shoelace sign is an artifact of the closing chord, so the traversal
    /// direction of the winding coordinate is the verdict instead.
    Ring(bool),
    /// A genuine area-enclosing loop: the shoelace sign is the verdict.
    Area(bool),
    /// Degenerate or unprojectable: no verdict.
    Unknown,
}

/// Inner-wire winding on the analytic periodic surfaces, compared in the
/// surface's seam-unwrapped UV parameter space.
fn check_periodic_inner_wire_orientation(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Vec<ValidationIssue>, CheckError> {
    use crate::classify::boundary::{
        unwrap_periodic, uv_boundary_is_degenerate, uv_polygon_double_area,
    };

    let face = topo.face(face_id)?;
    let surface = face.surface();
    // Every analytic surface here parameterizes u as an angle with period
    // 2pi; v is angular only for the torus (mirrors `count_analytic_crossings`).
    let u_period = std::f64::consts::TAU;
    let v_period = match surface {
        remus_topology::face::FaceSurface::Torus(_) => Some(std::f64::consts::TAU),
        _ => None,
    };

    // Project a wire's 3D polygon into (u, v), unwrapping the periodic
    // coordinates so consecutive samples stay within half a turn. Returns
    // `None` when a sample cannot be projected.
    let wire_uv =
        |wire_id: remus_topology::wire::WireId| -> Result<Option<Vec<(f64, f64)>>, CheckError> {
            let polygon = crate::util::wire_polygon(topo, wire_id)?;
            let mut uv: Vec<(f64, f64)> = Vec::with_capacity(polygon.len());
            for &point in &polygon {
                let Some(pair) = surface.project_point(point) else {
                    return Ok(None);
                };
                uv.push(pair);
            }
            for i in 1..uv.len() {
                uv[i].0 = unwrap_periodic(uv[i - 1].0, uv[i].0, u_period);
                if let Some(period) = v_period {
                    uv[i].1 = unwrap_periodic(uv[i - 1].1, uv[i].1, period);
                }
            }
            Ok(Some(uv))
        };

    let classify = |uv: &[(f64, f64)]| -> UvWinding {
        if uv_boundary_is_degenerate(uv) {
            return UvWinding::Unknown;
        }
        let n = uv.len();
        // A ring's samples progress a full turn in one periodic coordinate
        // and the polygon's closing edge jumps straight back across it. A
        // genuine area loop returns near its start, so requiring BOTH the
        // progression and the closing jump to span the period keeps a tall
        // oval window from misclassifying as a ring.
        let closing_du = (uv[0].0 - uv[n - 1].0).abs();
        let progress_u = uv[n - 1].0 - uv[0].0;
        if closing_du > 0.75 * u_period && progress_u.abs() > 0.75 * u_period {
            return UvWinding::Ring(progress_u > 0.0);
        }
        if let Some(period) = v_period {
            let closing_dv = (uv[0].1 - uv[n - 1].1).abs();
            let progress_v = uv[n - 1].1 - uv[0].1;
            if closing_dv > 0.75 * period && progress_v.abs() > 0.75 * period {
                return UvWinding::Ring(progress_v > 0.0);
            }
        }
        UvWinding::Area(uv_polygon_double_area(uv) > 0.0)
    };

    let Some(outer_uv) = wire_uv(face.outer_wire())? else {
        return Ok(Vec::new());
    };
    let outer_winding = classify(&outer_uv);
    if matches!(outer_winding, UvWinding::Unknown) {
        return Ok(Vec::new());
    }

    let mut issues = Vec::new();
    for &wire_id in face.inner_wires() {
        let Some(inner_uv) = wire_uv(wire_id)? else {
            continue;
        };
        let inner_winding = classify(&inner_uv);
        // A ring and an area loop have no consistent sign relation — which
        // side of a ring carries the material is invisible in UV (that
        // ambiguity is the two-ring representation problem itself) — so only
        // like-for-like pairs are judged.
        let same_wound = match (&outer_winding, &inner_winding) {
            (UvWinding::Ring(outer), UvWinding::Ring(inner))
            | (UvWinding::Area(outer), UvWinding::Area(inner)) => outer == inner,
            _ => false,
        };
        if same_wound {
            issues.push(ValidationIssue {
                check: CheckId::FaceOrientationConsistency,
                severity: Severity::Error,
                entity: EntityRef::Wire(wire_id),
                description: format!(
                    "inner wire {} on periodic face {} has the same winding as its outer wire",
                    wire_id.index(),
                    face_id.index()
                ),
                deviation: None,
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

    /// A cylindrical patch face (u ∈ [0, π/2], v ∈ [0, 1] on a unit cylinder
    /// around +z) with one rectangular hole loop. `inner_ccw` picks the hole's
    /// UV winding; CCW here matches the outer wire's winding.
    fn cylinder_face_with_hole(topo: &mut Topology, inner_ccw: bool) -> FaceId {
        let line_wire = |topo: &mut Topology, pts: &[Point3]| {
            let vids: Vec<_> = pts
                .iter()
                .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
                .collect();
            let oes: Vec<_> = (0..pts.len())
                .map(|i| {
                    let e = topo.add_edge(Edge::new(
                        vids[i],
                        vids[(i + 1) % pts.len()],
                        EdgeCurve::Line,
                    ));
                    OrientedEdge::new(e, true)
                })
                .collect();
            topo.add_wire(Wire::new(oes, true).unwrap())
        };
        let outer = line_wire(
            topo,
            &[
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 1.0),
                Point3::new(1.0, 0.0, 1.0),
            ],
        );
        let (u0, u1) = (0.4_f64, 0.9_f64);
        let corners = [
            Point3::new(u0.cos(), u0.sin(), 0.25),
            Point3::new(u1.cos(), u1.sin(), 0.25),
            Point3::new(u1.cos(), u1.sin(), 0.75),
            Point3::new(u0.cos(), u0.sin(), 0.75),
        ];
        let inner_pts: Vec<_> = if inner_ccw {
            corners.to_vec()
        } else {
            corners.iter().rev().copied().collect()
        };
        let inner = line_wire(topo, &inner_pts);
        let surface = FaceSurface::Cylinder(
            remus_math::surfaces::CylindricalSurface::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(outer, vec![inner], surface))
    }

    // Issue #270: an inverted hole loop on a cylindrical (or conical) face used
    // to pass validation entirely — the check ran on planar faces only.
    #[test]
    fn cylinder_inner_wire_winding_is_checked_in_uv() {
        let mut topo = Topology::new();
        let bad = cylinder_face_with_hole(&mut topo, true);
        let issues = check_face_inner_wire_orientation(&topo, bad).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "same-wound hole on a cylinder must fire: {issues:?}"
        );
        assert_eq!(issues[0].severity, Severity::Error);
        assert!(issues[0].description.contains("same winding"));

        let mut topo = Topology::new();
        let good = cylinder_face_with_hole(&mut topo, false);
        let issues = check_face_inner_wire_orientation(&topo, good).unwrap();
        assert!(
            issues.is_empty(),
            "opposite-wound hole on a cylinder must pass: {issues:?}"
        );
    }

    /// Same as above, but the hole straddles the u = 0 seam: the unwrap must
    /// keep the polygon coherent instead of tearing it across the chart.
    #[test]
    fn cylinder_hole_crossing_the_seam_keeps_its_winding_verdict() {
        let mut topo = Topology::new();
        let line_wire = |topo: &mut Topology, pts: &[Point3]| {
            let vids: Vec<_> = pts
                .iter()
                .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
                .collect();
            let oes: Vec<_> = (0..pts.len())
                .map(|i| {
                    let e = topo.add_edge(Edge::new(
                        vids[i],
                        vids[(i + 1) % pts.len()],
                        EdgeCurve::Line,
                    ));
                    OrientedEdge::new(e, true)
                })
                .collect();
            topo.add_wire(Wire::new(oes, true).unwrap())
        };
        let outer = line_wire(
            &mut topo,
            &[
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 1.0),
                Point3::new(1.0, 0.0, 1.0),
            ],
        );
        // u runs from −0.2 to +0.2 across the seam.
        let corners = [
            Point3::new((-0.2_f64).cos(), (-0.2_f64).sin(), 0.25),
            Point3::new(0.2_f64.cos(), 0.2_f64.sin(), 0.25),
            Point3::new(0.2_f64.cos(), 0.2_f64.sin(), 0.75),
            Point3::new((-0.2_f64).cos(), (-0.2_f64).sin(), 0.75),
        ];
        let surface = FaceSurface::Cylinder(
            remus_math::surfaces::CylindricalSurface::new(
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                1.0,
            )
            .unwrap(),
        );

        let same_wound = line_wire(&mut topo, &corners);
        let bad = topo.add_face(Face::new(outer, vec![same_wound], surface.clone()));
        let issues = check_face_inner_wire_orientation(&topo, bad).unwrap();
        assert_eq!(
            issues.len(),
            1,
            "seam-crossing same-wound hole must fire: {issues:?}"
        );

        let opposite: Vec<_> = corners.iter().rev().copied().collect();
        let opposite = line_wire(&mut topo, &opposite);
        let good = topo.add_face(Face::new(outer, vec![opposite], surface));
        let issues = check_face_inner_wire_orientation(&topo, good).unwrap();
        assert!(
            issues.is_empty(),
            "seam-crossing opposite-wound hole must pass: {issues:?}"
        );
    }
}
