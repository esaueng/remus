//! Clearance for coaxially resizing a general partial-turn boss wall.
//!
//! [`super::quarter_cylinder`] certifies the single quarter-sector prism. This
//! module certifies any outward cylinder wall bounded by axial lines and
//! coaxial circles, such as a boss fused into one corner of a block whose wall
//! is a partial turn below the block top and a full turn above it. Each axial
//! side must lie on a plane through the axis, so its contact slides radially
//! along that plane, and each rim must lie on a plane perpendicular to the
//! axis, so it re-trims as a coaxial circle.
//!
//! The swept region is over-approximated by the full annular slab between the
//! old and new radii over the wall's axial span. Every source edge outside the
//! wall boundary must clear that slab, except a straight radial edge that
//! leaves a wall vertex outward and ends beyond the slab: the moving vertex
//! slides along it. Every face not adjacent to the wall must clear the slab as
//! a whole, which also covers a face pierced by the slab with no boundary edge
//! inside it. A violation is a typed [`OffsetError::TopologyChange`]; the
//! over-approximation can only refuse a valid edit, never accept a contact.

use std::collections::{BTreeMap, BTreeSet};

use remus_math::{surfaces::CylindricalSurface, tolerance::Tolerance, vec::Vec3};
use remus_topology::{
    Topology,
    edge::{EdgeCurve, EdgeId},
    face::{FaceId, FaceSurface},
};

use crate::{OffsetError, UnsupportedMoveCause};

fn unsupported(face: FaceId, reason: &str) -> OffsetError {
    OffsetError::UnsupportedMoveFace {
        face,
        surface_type: "cylinder",
        cause: UnsupportedMoveCause::Unclassified,
        reason: reason.into(),
    }
}

fn contact(face: Option<FaceId>, edge: Option<EdgeId>, reason: &str) -> OffsetError {
    OffsetError::TopologyChange {
        face,
        edge,
        reason: reason.into(),
    }
}

/// Cylinder-frame coordinates `(u, v, w)`, with `w` along the axis.
struct Frame {
    origin: remus_math::vec::Point3,
    u: Vec3,
    v: Vec3,
    w: Vec3,
}

impl Frame {
    fn coordinates(&self, point: remus_math::vec::Point3) -> [f64; 3] {
        let delta = point - self.origin;
        [delta.dot(self.u), delta.dot(self.v), delta.dot(self.w)]
    }

    fn radius(&self, point: remus_math::vec::Point3) -> f64 {
        let [u, v, _] = self.coordinates(point);
        u.hypot(v)
    }
}

/// An axis-aligned box in cylinder-frame coordinates.
#[derive(Clone, Copy)]
struct FrameBox {
    lo: [f64; 3],
    hi: [f64; 3],
}

impl FrameBox {
    const EMPTY: Self = Self {
        lo: [f64::INFINITY; 3],
        hi: [f64::NEG_INFINITY; 3],
    };

    fn include(&mut self, coordinates: [f64; 3], margin: [f64; 3]) {
        for axis in 0..3 {
            self.lo[axis] = self.lo[axis].min(coordinates[axis] - margin[axis]);
            self.hi[axis] = self.hi[axis].max(coordinates[axis] + margin[axis]);
        }
    }

    fn merge(&mut self, other: Self) {
        for axis in 0..3 {
            self.lo[axis] = self.lo[axis].min(other.lo[axis]);
            self.hi[axis] = self.hi[axis].max(other.hi[axis]);
        }
    }

    fn is_finite(&self) -> bool {
        self.lo
            .iter()
            .chain(self.hi.iter())
            .all(|value| value.is_finite())
    }

    /// Whether the box may meet `radius ∈ [radius_lo, radius_hi]`,
    /// `w ∈ [w_lo, w_hi]`, all widened by `linear`.
    fn may_meet_slab(
        &self,
        radius_lo: f64,
        radius_hi: f64,
        w_lo: f64,
        w_hi: f64,
        linear: f64,
    ) -> bool {
        if self.hi[2] < w_lo - linear || self.lo[2] > w_hi + linear {
            return false;
        }
        let gap = |lo: f64, hi: f64| {
            if lo > 0.0 {
                lo
            } else if hi < 0.0 {
                -hi
            } else {
                0.0
            }
        };
        let nearest = gap(self.lo[0], self.hi[0]).hypot(gap(self.lo[1], self.hi[1]));
        let farthest = self.lo[0]
            .abs()
            .max(self.hi[0].abs())
            .hypot(self.lo[1].abs().max(self.hi[1].abs()));
        nearest <= radius_hi + linear && farthest >= radius_lo - linear
    }
}

fn edge_box(
    topo: &Topology,
    frame: &Frame,
    edge_id: EdgeId,
) -> Result<Option<FrameBox>, OffsetError> {
    let edge = topo.edge(edge_id)?;
    let mut bounds = FrameBox::EMPTY;
    match edge.curve() {
        EdgeCurve::Line => {
            for vertex in [edge.start(), edge.end()] {
                bounds.include(frame.coordinates(topo.vertex(vertex)?.point()), [0.0; 3]);
            }
        }
        EdgeCurve::Circle(circle) => {
            // The whole circle bounds any arc of it.
            let normal = circle.normal();
            let extent = |direction: Vec3| {
                let along = normal.dot(direction);
                circle.radius() * (1.0 - along * along).max(0.0).sqrt()
            };
            bounds.include(
                frame.coordinates(circle.center()),
                [extent(frame.u), extent(frame.v), extent(frame.w)],
            );
        }
        _ => return Ok(None),
    }
    Ok(bounds.is_finite().then_some(bounds))
}

/// Certify that coaxially resizing the outward partial wall `selected` to
/// `replacement` keeps the source adjacency graph and meets no other face.
#[allow(clippy::too_many_lines)]
pub(super) fn validate_clearance<V: std::ops::Deref<Target = [FaceId]>>(
    topo: &Topology,
    selected: FaceId,
    replacement: &CylindricalSurface,
    edge_faces: &BTreeMap<usize, V>,
    tolerance: Tolerance,
) -> Result<(), OffsetError> {
    let face = topo.face(selected)?;
    let FaceSurface::Cylinder(source) = face.surface() else {
        return Err(unsupported(
            selected,
            "partial-wall source is not cylindrical",
        ));
    };
    let axis = source.axis().normalize()?;
    let u = source.x_axis().normalize()?;
    let frame = Frame {
        origin: source.origin(),
        u,
        v: axis.cross(u),
        w: axis,
    };
    let radius_lo = source.radius().min(replacement.radius());
    let radius_hi = source.radius().max(replacement.radius());

    // Wall boundary: axial lines and coaxial source-radius circles.
    let mut wall_edges = BTreeSet::new();
    let mut wall_vertices = BTreeSet::new();
    let mut w_lo = f64::INFINITY;
    let mut w_hi = f64::NEG_INFINITY;
    let mut neighbors: BTreeMap<usize, (FaceId, bool, bool)> = BTreeMap::new();
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge_id = oriented.edge();
            let edge = topo.edge(edge_id)?;
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            let axial = match edge.curve() {
                EdgeCurve::Line => {
                    if (end - start).cross(axis).length() > tolerance.linear {
                        return Err(unsupported(
                            selected,
                            "partial-wall side edges must be axial lines",
                        ));
                    }
                    true
                }
                EdgeCurve::Circle(circle) => {
                    let offset = circle.center() - source.origin();
                    let level = offset.dot(axis);
                    if circle.normal().cross(axis).length() > tolerance.angular
                        || (circle.radius() - source.radius()).abs() > tolerance.linear
                        || (offset - axis * level).length() > tolerance.linear
                    {
                        return Err(unsupported(
                            selected,
                            "partial-wall rims must be coaxial circles on the wall",
                        ));
                    }
                    false
                }
                _ => {
                    return Err(unsupported(
                        selected,
                        "partial-wall boundaries must be lines and circles",
                    ));
                }
            };
            for point in [start, end] {
                let w = frame.coordinates(point)[2];
                w_lo = w_lo.min(w);
                w_hi = w_hi.max(w);
            }
            wall_edges.insert(edge_id.index());
            wall_vertices.insert(edge.start().index());
            wall_vertices.insert(edge.end().index());
            let adjacent = edge_faces
                .get(&edge_id.index())
                .ok_or_else(|| unsupported(selected, "partial-wall edge has no adjacency"))?;
            for &neighbor in adjacent.iter() {
                if neighbor != selected {
                    let entry = neighbors
                        .entry(neighbor.index())
                        .or_insert((neighbor, false, false));
                    if axial {
                        entry.1 = true;
                    } else {
                        entry.2 = true;
                    }
                }
            }
        }
    }
    if !w_lo.is_finite() || !w_hi.is_finite() || w_hi - w_lo <= tolerance.linear {
        return Err(unsupported(selected, "partial-wall has no axial span"));
    }
    if neighbors.is_empty() {
        return Err(unsupported(
            selected,
            "partial-wall has no supporting faces",
        ));
    }

    // Supports: planes through the axis along the sides, perpendicular planes
    // along the rims. A plane meeting the wall in both ways cannot exist.
    for &(neighbor, along_side, along_rim) in neighbors.values() {
        let FaceSurface::Plane { normal, d } = topo.face(neighbor)?.surface() else {
            return Err(unsupported(
                selected,
                "partial-wall supports must be planar",
            ));
        };
        let normal = normal.normalize()?;
        let alignment = normal.dot(axis);
        let through_axis = alignment.abs() <= tolerance.angular
            && (normal.dot(source.origin() - remus_math::vec::Point3::new(0.0, 0.0, 0.0)) - d)
                .abs()
                <= tolerance.linear;
        let perpendicular = alignment.abs() >= 1.0 - tolerance.angular;
        if (along_side && !through_axis) || (along_rim && !perpendicular) {
            return Err(unsupported(
                selected,
                "partial-wall sides must lie on planes through its axis and rims on planes perpendicular to it",
            ));
        }
    }

    let linear = tolerance.linear;
    let slab = |bounds: &FrameBox| bounds.may_meet_slab(radius_lo, radius_hi, w_lo, w_hi, linear);

    // Edges: the wall's own are rebuilt; a radial edge leaving a wall vertex
    // outward guides that vertex; every other edge must clear the slab.
    let mut face_boxes: BTreeMap<usize, (FaceId, FrameBox)> = BTreeMap::new();
    for (&edge_index, faces) in edge_faces {
        if wall_edges.contains(&edge_index) {
            continue;
        }
        let edge_id = topo
            .edge_id_from_index(edge_index)
            .ok_or_else(|| contact(Some(selected), None, "source edge is unavailable"))?;
        let edge = topo.edge(edge_id)?;
        let start_on_wall = wall_vertices.contains(&edge.start().index());
        let end_on_wall = wall_vertices.contains(&edge.end().index());
        let bounds = edge_box(topo, &frame, edge_id)?;
        for &owner in faces.iter() {
            let entry = face_boxes
                .entry(owner.index())
                .or_insert((owner, FrameBox::EMPTY));
            match bounds {
                Some(bounds) => entry.1.merge(bounds),
                None => {
                    entry.1 = FrameBox {
                        lo: [f64::NEG_INFINITY; 3],
                        hi: [f64::INFINITY; 3],
                    }
                }
            }
        }
        if start_on_wall || end_on_wall {
            let (anchor, far) = if start_on_wall {
                (edge.start(), edge.end())
            } else {
                (edge.end(), edge.start())
            };
            let anchor = topo.vertex(anchor)?.point();
            let far = topo.vertex(far)?.point();
            let direction = far - anchor;
            let radial = frame.coordinates(anchor);
            let radial = frame.u * radial[0] + frame.v * radial[1];
            let guides = matches!(edge.curve(), EdgeCurve::Line)
                && !(start_on_wall && end_on_wall)
                && direction.dot(axis).abs() <= linear
                && direction.length() > linear
                && radial.length() > linear
                && direction.cross(radial).length()
                    <= tolerance.angular * direction.length() * radial.length()
                && direction.dot(radial) > 0.0
                && frame.radius(far) > radius_hi + linear;
            if !guides {
                return Err(contact(
                    faces.first().copied(),
                    Some(edge_id),
                    "partial-wall vertex does not slide along an outward radial edge that clears the sweep",
                ));
            }
            continue;
        }
        let Some(bounds) = bounds else {
            return Err(contact(
                faces.first().copied(),
                Some(edge_id),
                "partial-wall clearance requires line and circle edges",
            ));
        };
        if slab(&bounds) {
            return Err(contact(
                faces.first().copied(),
                Some(edge_id),
                "partial-wall sweep may contact a source edge",
            ));
        }
    }

    // Nonadjacent faces must clear the slab as a whole. A planar face lies in
    // the box of its boundary, so the box also covers its interior; a curved
    // face can bulge past its boundary box and is refused.
    for (&index, &(other, bounds)) in &face_boxes {
        if index == selected.index() || neighbors.contains_key(&index) {
            continue;
        }
        if !matches!(topo.face(other)?.surface(), FaceSurface::Plane { .. }) {
            return Err(contact(
                Some(other),
                None,
                "partial-wall clearance currently requires planar nonadjacent faces",
            ));
        }
        if !bounds.is_finite() || slab(&bounds) {
            return Err(contact(
                Some(other),
                None,
                "partial-wall sweep may contact a nonadjacent source face",
            ));
        }
    }
    Ok(())
}
