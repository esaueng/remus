//! Exact support-surface replacement with topology-preserving re-limitation.

use remus_algo::compute_pcurve_on_surface_in_domain;
use remus_math::curves2d::Curve2D;
use remus_math::det_hash::DetHashMap;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;

use crate::OperationsError;

/// A re-limited solid and exact source-to-result face correspondence.
#[derive(Debug)]
pub struct ReplaceSurfaceResult {
    /// Edited solid.
    pub solid: SolidId,
    /// Source face index to the one result face derived from it.
    ///
    /// Deterministically hashed: the offset layer builds this with a std map,
    /// whose iteration order varies run to run.
    pub face_map: DetHashMap<usize, FaceId>,
}

/// Replace one face's support surface and rebuild every affected boundary.
///
/// The qualified first cell accepts plane-to-plane replacement and coaxial
/// cylinder radius replacement when every neighbor is planar or cylindrical.
/// It preserves the source adjacency graph, reconstructs intersection-curve
/// trims, and derives a fresh p-curve for every coedge in the result.
///
/// # Errors
///
/// Returns a typed offset error naming the offending source face or edge when
/// re-intersection cannot preserve the adjacency graph. Any error, including
/// p-curve projection or final validation, restores the exact input topology.
pub fn replace_surface(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    replacement: FaceSurface,
) -> Result<ReplaceSurfaceResult, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let result = remus_offset::replace_surface_with_face_map(topo, solid, face, replacement)?;
        let result_faces = solid_faces(topo, result.solid)?;
        register_fresh_pcurves(topo, &result_faces)?;

        let report = crate::validate::validate_solid(topo, result.solid)?;
        if !report.is_valid() {
            return Err(remus_offset::OffsetError::TopologyChange {
                face: Some(face),
                edge: None,
                reason: format!(
                    "replace-surface validation failed with {} error(s)",
                    report.error_count()
                ),
            }
            .into());
        }
        Ok(ReplaceSurfaceResult {
            solid: result.solid,
            face_map: result.face_map.into_iter().collect(),
        })
    })
}

fn register_fresh_pcurves(topo: &mut Topology, faces: &[FaceId]) -> Result<(), OperationsError> {
    for &face_id in faces {
        let face = topo.face(face_id)?;
        let surface = face.surface().clone();
        let outer_wire = face.outer_wire();
        let wire_ids: Vec<_> = std::iter::once(outer_wire)
            .chain(face.inner_wires().iter().copied())
            .collect();
        let wire_points: Vec<Point3> = topo
            .wire(outer_wire)?
            .edges()
            .iter()
            .map(|oriented| {
                let edge = topo.edge(oriented.edge())?;
                Ok(topo.vertex(edge.start())?.point())
            })
            .collect::<Result<_, remus_topology::TopologyError>>()?;

        for wire_id in wire_ids {
            let uses = topo.wire(wire_id)?.edges().to_vec();
            for oriented in uses {
                let edge_id = oriented.edge();
                let edge = topo.edge(edge_id)?;
                let start = topo.vertex(edge.start())?.point();
                let end = topo.vertex(edge.end())?.point();
                let domain = edge.strict_domain().map_err(|error| {
                    remus_offset::OffsetError::TopologyChange {
                        face: Some(face_id),
                        edge: Some(edge_id),
                        reason: format!("result edge has no trim authority: {error}"),
                    }
                })?;
                let curve = compute_pcurve_on_surface_in_domain(
                    edge.curve(),
                    start,
                    end,
                    domain,
                    &surface,
                    &wire_points,
                    None,
                )?;
                let (start_parameter, end_parameter) = match &curve {
                    Curve2D::Line(_) if edge.start() == edge.end() => (0.0, std::f64::consts::TAU),
                    Curve2D::Line(_) => (0.0, (end - start).length()),
                    Curve2D::Circle(_) | Curve2D::Ellipse(_) | Curve2D::Nurbs(_) => (0.0, 1.0),
                };
                let pcurve = PCurve::new(curve, start_parameter, end_parameter);
                topo.set_pcurve_oriented(edge_id, face_id, oriented.is_forward(), pcurve)?;
            }
        }
    }
    Ok(())
}
