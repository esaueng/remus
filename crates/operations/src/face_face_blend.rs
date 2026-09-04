//! Exact standalone face-face blend sheets.

use remus_blend::face_face::{FaceFaceBlendBand, build_face_face_blend_band};
use remus_math::vec::Point3;
use remus_topology::BodyClass;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::face::FaceId;
use remus_topology::shell::{Shell, ShellId};

pub use remus_blend::face_face::FaceFaceHoldLine;

/// Exact face-face blend result, represented as a first-class sheet body.
#[derive(Debug, Clone, Copy)]
pub struct FaceFaceBlendResult {
    /// Sheet body containing the blend band.
    pub sheet: ShellId,
    /// Exact cylindrical band face.
    pub band: FaceId,
    /// Longitudinal contact edges, in support-set order.
    pub contact_edges: [EdgeId; 2],
    /// Start of the synthetic carrier-intersection spine.
    pub spine_start: Point3,
    /// End of the synthetic carrier-intersection spine.
    pub spine_end: Point3,
    /// Prescribed constant radius.
    pub radius: f64,
}

/// Build an exact constant-radius band between two face selections.
///
/// The result is a standalone sheet and does not alter either support body.
/// The currently qualified subset contains one convex, straight-edged planar
/// face per selection. Their carrier planes must intersect, their edge sets
/// must be disjoint, and both contact segments must remain inside the bounded
/// support patches. A supplied hold line must match one exact contact segment.
///
/// # Errors
///
/// Returns a typed blend refusal outside the qualified subset. Construction
/// and sheet validation are transactional, so any error leaves the topology
/// unchanged.
pub fn face_face_blend(
    topo: &mut Topology,
    first_faces: &[FaceId],
    second_faces: &[FaceId],
    radius: f64,
    hold_line: Option<FaceFaceHoldLine>,
) -> Result<FaceFaceBlendResult, crate::OperationsError> {
    remus_topology::transaction::run_validated(
        topo,
        |topo| -> Result<_, crate::OperationsError> {
            let FaceFaceBlendBand {
                face,
                contact_edges,
                spine_start,
                spine_end,
                radius,
            } = build_face_face_blend_band(topo, first_faces, second_faces, radius, hold_line)?;
            let sheet = topo.add_shell(Shell::new(vec![face])?);
            topo.set_shell_body_class(sheet, BodyClass::Sheet)?;
            Ok(FaceFaceBlendResult {
                sheet,
                band: face,
                contact_edges,
                spine_start,
                spine_end,
                radius,
            })
        },
        |topo, result| {
            let report = remus_check::validate::validate_sheet_body(
                topo,
                result.sheet,
                &remus_check::validate::ValidateOptions::default(),
            )?;
            if report.is_valid() {
                Ok(())
            } else {
                Err(crate::OperationsError::BodyValidationFailed {
                    body_class: BodyClass::Sheet.as_str(),
                    error_count: report.error_count(),
                })
            }
        },
    )
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use remus_blend::BlendError;
    use remus_math::vec::Vec3;
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::face::{Face, FaceSurface};
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

    fn quad(topo: &mut Topology, points: [Point3; 4], normal: Vec3) -> FaceId {
        let vertices = points.map(|point| topo.add_vertex(Vertex::new(point, 1.0e-7)));
        let edges = [
            topo.add_edge(Edge::new(vertices[0], vertices[1], EdgeCurve::Line)),
            topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line)),
            topo.add_edge(Edge::new(vertices[2], vertices[3], EdgeCurve::Line)),
            topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line)),
        ];
        let wire = topo.add_wire(
            Wire::new(
                edges
                    .into_iter()
                    .map(|edge| OrientedEdge::new(edge, true))
                    .collect(),
                true,
            )
            .unwrap(),
        );
        topo.add_face(Face::new(
            wire,
            Vec::new(),
            FaceSurface::Plane {
                normal,
                d: normal.dot(Vec3::new(points[0].x(), points[0].y(), points[0].z())),
            },
        ))
    }

    fn disjoint_supports(topo: &mut Topology) -> (FaceId, FaceId) {
        disjoint_supports_at(topo, 1.0, Point3::new(0.0, 0.0, 0.0))
    }

    fn disjoint_supports_at(topo: &mut Topology, scale: f64, offset: Point3) -> (FaceId, FaceId) {
        let point = |x: f64, y: f64, z: f64| {
            Point3::new(
                offset.x() + x * scale,
                offset.y() + y * scale,
                offset.z() + z * scale,
            )
        };
        let horizontal = quad(
            topo,
            [
                point(0.5, 0.0, 0.0),
                point(0.5, 10.0, 0.0),
                point(4.0, 10.0, 0.0),
                point(4.0, 0.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, -1.0),
        );
        let vertical = quad(
            topo,
            [
                point(0.0, 0.0, 0.5),
                point(0.0, 0.0, 4.0),
                point(0.0, 10.0, 4.0),
                point(0.0, 10.0, 0.5),
            ],
            Vec3::new(-1.0, 0.0, 0.0),
        );
        (horizontal, vertical)
    }

    fn edge_midpoint(topo: &Topology, edge: EdgeId) -> Point3 {
        let edge = topo.edge(edge).unwrap();
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let range = edge.strict_domain().unwrap();
        edge.curve()
            .evaluate_with_endpoints(f64::midpoint(range.0, range.1), start, end)
    }

    fn mesh_area(mesh: &crate::tessellate::TriangleMesh) -> f64 {
        mesh.indices
            .chunks_exact(3)
            .map(|triangle| {
                let a = mesh.positions[triangle[0] as usize];
                let b = mesh.positions[triangle[1] as usize];
                let c = mesh.positions[triangle[2] as usize];
                (b - a).cross(c - a).length() * 0.5
            })
            .sum()
    }

    #[test]
    fn disjoint_planar_faces_build_an_exact_radius_band() {
        let mut topo = Topology::new();
        let (horizontal, vertical) = disjoint_supports(&mut topo);
        let result = face_face_blend(&mut topo, &[horizontal], &[vertical], 1.0, None).unwrap();

        assert_eq!(
            topo.shell(result.sheet).unwrap().body_class(),
            BodyClass::Sheet
        );
        assert_eq!(topo.shell(result.sheet).unwrap().faces(), &[result.band]);
        let report = remus_check::validate::validate_sheet_body(
            &topo,
            result.sheet,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert!(report.is_valid(), "{:#?}", report.issues);
        let FaceSurface::Cylinder(cylinder) = topo.face(result.band).unwrap().surface() else {
            panic!("face-face band must retain an exact cylinder carrier");
        };
        assert!((cylinder.radius() - 1.0).abs() < 1.0e-12);
        assert_eq!(
            topo.wire(topo.face(result.band).unwrap().outer_wire())
                .unwrap()
                .edges()
                .len(),
            4
        );

        let first_contact = edge_midpoint(&topo, result.contact_edges[0]);
        let second_contact = edge_midpoint(&topo, result.contact_edges[1]);
        assert!((first_contact.x() - 1.0).abs() < 1.0e-9);
        assert!(first_contact.z().abs() < 1.0e-9);
        assert!(second_contact.x().abs() < 1.0e-9);
        assert!((second_contact.z() - 1.0).abs() < 1.0e-9);
        assert!((result.spine_start.y() - 0.0).abs() < 1.0e-9);
        assert!((result.spine_end.y() - 10.0).abs() < 1.0e-9);

        let expected_area = 5.0 * std::f64::consts::PI;
        let exact_area = crate::measure::sheet_surface_area(&topo, result.sheet, 0.01).unwrap();
        assert!(
            (exact_area - expected_area).abs() < 1.0e-8,
            "area={exact_area}"
        );
        let mesh = crate::tessellate::tessellate_sheet(&topo, result.sheet, 0.001).unwrap();
        let tessellated_area = mesh_area(&mesh);
        assert!(
            (tessellated_area - expected_area).abs() < 0.05,
            "mesh area={tessellated_area}, exact area={exact_area}"
        );
    }

    #[test]
    fn exact_band_is_scale_and_translation_stable() {
        let offset = Point3::new(3.0, -4.0, 7.0);
        for scale in [1.0e-3, 1.0, 1.0e3] {
            let mut topo = Topology::new();
            let (horizontal, vertical) = disjoint_supports_at(&mut topo, scale, offset);
            let result =
                face_face_blend(&mut topo, &[horizontal], &[vertical], scale, None).unwrap();
            let first_contact = edge_midpoint(&topo, result.contact_edges[0]);
            let second_contact = edge_midpoint(&topo, result.contact_edges[1]);
            let coordinate_tolerance = scale * 1.0e-9 + 1.0e-10;
            assert!((first_contact.x() - (offset.x() + scale)).abs() < coordinate_tolerance);
            assert!((first_contact.z() - offset.z()).abs() < coordinate_tolerance);
            assert!((second_contact.x() - offset.x()).abs() < coordinate_tolerance);
            assert!((second_contact.z() - (offset.z() + scale)).abs() < coordinate_tolerance);

            let expected_area = 5.0 * std::f64::consts::PI * scale * scale;
            let area =
                crate::measure::sheet_surface_area(&topo, result.sheet, scale * 0.01).unwrap();
            assert!(
                (area - expected_area).abs() <= (expected_area * 1.0e-9).max(1.0e-12),
                "scale={scale}, area={area}, expected={expected_area}"
            );
        }
    }

    #[test]
    fn exact_hold_line_is_verified_and_mismatch_rolls_back() {
        let mut topo = Topology::new();
        let (horizontal, vertical) = disjoint_supports(&mut topo);
        let hold = FaceFaceHoldLine {
            support: horizontal,
            start: Point3::new(1.0, 0.0, 0.0),
            end: Point3::new(1.0, 10.0, 0.0),
        };
        let result =
            face_face_blend(&mut topo, &[horizontal], &[vertical], 1.0, Some(hold)).unwrap();
        assert_eq!(topo.shell(result.sheet).unwrap().faces(), &[result.band]);

        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
        );
        let mismatch = FaceFaceHoldLine {
            support: horizontal,
            start: Point3::new(2.0, 0.0, 0.0),
            end: Point3::new(2.0, 10.0, 0.0),
        };
        let error = face_face_blend(&mut topo, &[horizontal], &[vertical], 1.0, Some(mismatch))
            .unwrap_err();
        assert!(matches!(
            error,
            crate::OperationsError::Blend(BlendError::UnsupportedFaceFaceBlend { .. })
        ));
        assert_eq!(
            before,
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
            )
        );
    }

    #[test]
    fn support_cliff_and_multi_face_sets_fail_closed() {
        let mut topo = Topology::new();
        let (horizontal, vertical) = disjoint_supports(&mut topo);
        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
        );
        for outcome in [
            face_face_blend(&mut topo, &[horizontal], &[vertical], 5.0, None),
            face_face_blend(&mut topo, &[horizontal, horizontal], &[vertical], 1.0, None),
        ] {
            assert!(matches!(
                outcome,
                Err(crate::OperationsError::Blend(
                    BlendError::UnsupportedFaceFaceBlend { .. }
                ))
            ));
            assert_eq!(
                before,
                (
                    topo.num_vertices(),
                    topo.num_edges(),
                    topo.num_wires(),
                    topo.num_faces(),
                    topo.num_shells(),
                )
            );
        }
    }

    #[test]
    fn malformed_support_polygons_fail_closed() {
        let mut topo = Topology::new();
        let (_, vertical) = disjoint_supports(&mut topo);
        let off_plane = quad(
            &mut topo,
            [
                Point3::new(0.5, 0.0, 0.0),
                Point3::new(0.5, 10.0, 0.0),
                Point3::new(4.0, 10.0, 0.1),
                Point3::new(4.0, 0.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, -1.0),
        );
        let self_crossing = quad(
            &mut topo,
            [
                Point3::new(0.5, 0.0, 0.0),
                Point3::new(4.0, 10.0, 0.0),
                Point3::new(0.5, 10.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, -1.0),
        );
        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
        );

        for support in [off_plane, self_crossing] {
            let error = face_face_blend(&mut topo, &[support], &[vertical], 1.0, None)
                .expect_err("malformed support must refuse");
            assert!(matches!(
                error,
                crate::OperationsError::Blend(BlendError::UnsupportedFaceFaceBlend { .. })
            ));
            assert_eq!(
                before,
                (
                    topo.num_vertices(),
                    topo.num_edges(),
                    topo.num_wires(),
                    topo.num_faces(),
                    topo.num_shells(),
                )
            );
        }
    }
}
