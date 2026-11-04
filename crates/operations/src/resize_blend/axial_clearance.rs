//! Clearance between a translating shoulder or chamfer and a fixed full rim.

use std::f64::consts::TAU;

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::face_edges;
use remus_topology::face::{FaceId, FaceSurface};

use crate::OperationsError;

pub(super) struct AxialBlendClearance {
    moving: FaceId,
    fixed: FaceId,
    gap: f64,
}

impl AxialBlendClearance {
    pub(super) fn reached(&self, distance: f64) -> bool {
        distance * self.gap > 0.0 && distance.abs() + Tolerance::new().linear >= self.gap.abs()
    }

    pub(super) const fn fixed(&self) -> FaceId {
        self.fixed
    }

    pub(super) fn check(&self, distance: f64) -> Result<(), OperationsError> {
        if self.reached(distance) {
            return Err(remus_offset::OffsetError::TopologyChange {
                face: Some(self.moving),
                edge: None,
                reason: format!(
                    "moving blend reaches fixed rim face {} before completing the move",
                    self.fixed.index()
                ),
            }
            .into());
        }
        Ok(())
    }
}

struct Contact {
    level: f64,
    length: f64,
}

/// An exact circular spring on one cylindrical carrier, all at one level.
fn contact(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    band: FaceId,
    carrier: FaceId,
    direction: Vec3,
) -> Result<Option<Contact>, OperationsError> {
    let mut contact: Option<Contact> = None;
    for edge in face_edges(topo, band)? {
        if !adjacency.faces_for_edge(edge).contains(&carrier) {
            continue;
        }
        let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve() else {
            return Ok(None);
        };
        if circle.normal().dot(direction).abs() < 1.0 - Tolerance::new().angular
            || (!matches!(
                topo.face(band)?.surface(),
                FaceSurface::Cone(_) | FaceSurface::Plane { .. }
            ) && !super::tangent_across(topo, edge, band, carrier)?)
        {
            return Ok(None);
        }
        let level = (circle.center() - Point3::new(0.0, 0.0, 0.0)).dot(direction);
        let entry = contact.get_or_insert(Contact { level, length: 0.0 });
        if !Tolerance::new().approx_eq(entry.level, level) {
            return Ok(None);
        }
        entry.length += crate::measure::edge_length(topo, edge)?;
    }
    Ok(contact)
}

/// A full fixed spring covers every angular position of a moving spring on
/// the same cylinder. Their axial levels therefore give an exact travel
/// bound without using a periodic face's deliberately conservative box.
pub(super) fn axial_blend_clearances(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    source_faces: &[FaceId],
    is_moved: impl Fn(FaceId) -> bool,
    is_invariant_carrier: impl Fn(FaceId) -> bool,
    direction: Vec3,
) -> Result<Vec<AxialBlendClearance>, OperationsError> {
    let mut limits = Vec::new();
    for &carrier in source_faces {
        if !is_invariant_carrier(carrier) {
            continue;
        }
        let FaceSurface::Cylinder(cylinder) = topo.face(carrier)?.surface() else {
            continue;
        };
        if cylinder.axis().dot(direction).abs() < 1.0 - Tolerance::new().angular {
            continue;
        }
        let mut moving = Vec::new();
        let mut fixed = Vec::new();
        for &face in source_faces {
            if !matches!(
                topo.face(face)?.surface(),
                FaceSurface::Torus(_) | FaceSurface::Cone(_) | FaceSurface::Plane { .. }
            ) {
                continue;
            }
            let Some(contact) = contact(topo, adjacency, face, carrier, direction)? else {
                continue;
            };
            if is_moved(face) {
                moving.push((face, contact.level));
            } else if (contact.length - TAU * cylinder.radius()).abs()
                <= Tolerance::new().linear * TAU
            {
                fixed.push((face, contact.level));
            }
        }
        for &(moving, from) in &moving {
            for &(fixed, to) in &fixed {
                limits.push(AxialBlendClearance {
                    moving,
                    fixed,
                    gap: to - from,
                });
            }
        }
    }
    Ok(limits)
}
