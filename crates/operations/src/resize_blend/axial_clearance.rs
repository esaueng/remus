//! Clearance between a translating blend and a fixed blend on one cylinder.

use remus_math::det_hash::DetHashSet;
use std::f64::consts::TAU;

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{face_edges, face_vertices};
use remus_topology::face::{FaceId, FaceSurface};

use crate::OperationsError;

pub(super) struct AxialBlendClearance {
    moving: FaceId,
    fixed: FaceId,
    gap: f64,
    /// Whether the fixed blend's contact is the carrier's full circle.
    fixed_is_full: bool,
}

impl AxialBlendClearance {
    pub(super) fn reached(&self, distance: f64) -> bool {
        distance * self.gap > 0.0 && distance.abs() + Tolerance::new().linear >= self.gap.abs()
    }

    pub(super) const fn fixed(&self) -> FaceId {
        self.fixed
    }

    pub(super) const fn moving(&self) -> FaceId {
        self.moving
    }

    /// Whether the fixed blend is a full rim of the carrier: a moving partial
    /// spring then meets it at whichever angular positions it covers.
    pub(super) const fn fixed_is_full_rim(&self) -> bool {
        self.fixed_is_full
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

/// Whether a contact of `length` covers the whole circle of `radius`.
fn covers_full_circle(length: f64, radius: f64) -> bool {
    (length - TAU * radius).abs() <= Tolerance::new().linear * TAU
}

/// The exact axial travel bounds of a translation, and the carriers they
/// prove cannot fold.
pub(super) struct AxialClearances {
    pub(super) limits: Vec<AxialBlendClearance>,
    /// Cylindrical carriers whose every moving edge belongs to a full moving
    /// spring, each bounded against every fixed boundary edge of the carrier:
    /// no move inside those bounds folds the carrier's wire.
    pub(super) certified: DetHashSet<FaceId>,
}

/// Travel bounds between moving and fixed geometry on one cylindrical carrier.
///
/// Two springs on the same cylinder meet when their axial levels meet,
/// provided they share an angular position. A spring that covers the whole
/// circle shares every angular position with any other, so a pair is bounded
/// whenever either side is full: a moving partial shoulder against a fixed
/// full rim, or a moving full rim against a fixed partial shoulder. Their
/// axial levels give an exact travel bound without using a periodic face's
/// deliberately conservative box. A pair of partial springs is not bounded
/// here: their arcs may never overlap.
///
/// A full moving spring sweeps every angular position of its carrier, so
/// every fixed boundary edge of the carrier bounds it too, at the edge's
/// nearest level: a fixed partial blend's arc, a freeform corner patch's
/// contact, a seam line. Such a carrier is certified when every fixed edge
/// is bounded this way; any carrier left uncertified keeps the translation's
/// closed-mesh postcondition.
pub(super) fn axial_blend_clearances(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    source_faces: &[FaceId],
    is_moved: impl Fn(FaceId) -> bool,
    is_invariant_carrier: impl Fn(FaceId) -> bool,
    direction: Vec3,
) -> Result<AxialClearances, OperationsError> {
    let mut moved_vertices = DetHashSet::default();
    for &face in source_faces {
        if is_moved(face) {
            moved_vertices.extend(face_vertices(topo, face)?);
        }
    }
    let mut limits = Vec::new();
    let mut certified = DetHashSet::default();
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
            let spring = Spring {
                face,
                level: contact.level,
                full: covers_full_circle(contact.length, cylinder.radius()),
            };
            if is_moved(face) {
                moving.push(spring);
            } else {
                fixed.push(spring);
            }
        }
        limits.extend(pair_limits(&moving, &fixed));
        let full: Vec<Spring> = moving
            .iter()
            .copied()
            .filter(|spring| spring.full)
            .collect();
        if full.is_empty() {
            continue;
        }
        let mut proven = true;
        for edge in face_edges(topo, carrier)? {
            let data = topo.edge(edge)?;
            let start_moves = moved_vertices.contains(&data.start());
            let end_moves = moved_vertices.contains(&data.end());
            if start_moves && end_moves {
                // A moving edge outside every full spring may sweep
                // anywhere: leave the carrier to the mesh postcondition.
                proven &= full_spring_edge(topo, edge, &full, direction)?;
                continue;
            }
            if start_moves || end_moves {
                // A stretched line, checked on its own for collapse.
                continue;
            }
            let Some((low, high)) = edge_levels(topo, edge, direction)? else {
                proven = false;
                continue;
            };
            let fixed = distinct_neighbour(adjacency, edge, carrier);
            for spring in &full {
                match level_limit(spring, fixed, low, high) {
                    Some(limit) => limits.push(limit),
                    None => proven = false,
                }
            }
        }
        if proven {
            certified.insert(carrier);
        }
    }
    Ok(AxialClearances { limits, certified })
}

/// Whether a moving `edge` of a carrier is an arc of one of its full springs.
fn full_spring_edge(
    topo: &Topology,
    edge: EdgeId,
    full: &[Spring],
    direction: Vec3,
) -> Result<bool, OperationsError> {
    let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve() else {
        return Ok(false);
    };
    let level = (circle.center() - Point3::new(0.0, 0.0, 0.0)).dot(direction);
    Ok(
        circle.normal().dot(direction).abs() >= 1.0 - Tolerance::new().angular
            && full
                .iter()
                .any(|spring| Tolerance::new().approx_eq(spring.level, level)),
    )
}

/// The face across `edge` from `carrier`, or the carrier itself for a seam.
fn distinct_neighbour(adjacency: &AdjacencyIndex, edge: EdgeId, carrier: FaceId) -> FaceId {
    adjacency
        .faces_for_edge(edge)
        .iter()
        .copied()
        .find(|&face| face != carrier)
        .unwrap_or(carrier)
}

/// The range of levels along `direction` an edge reaches, or `None` when the
/// edge's curve has no cheap exact bound. Circles bound by their whole circle
/// and NURBS by their control hull, both conservatively.
fn edge_levels(
    topo: &Topology,
    edge: EdgeId,
    direction: Vec3,
) -> Result<Option<(f64, f64)>, OperationsError> {
    let level = |point: Point3| (point - Point3::new(0.0, 0.0, 0.0)).dot(direction);
    let data = topo.edge(edge)?;
    let ends = [
        level(topo.vertex(data.start())?.point()),
        level(topo.vertex(data.end())?.point()),
    ];
    let (low, high) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
    Ok(match data.curve() {
        EdgeCurve::Line => Some((low, high)),
        EdgeCurve::Circle(circle) => {
            let tilt = circle.normal().dot(direction);
            let reach = circle.radius() * (1.0 - tilt * tilt).max(0.0).sqrt();
            let centre = level(circle.center());
            Some((low.min(centre - reach), high.max(centre + reach)))
        }
        EdgeCurve::NurbsCurve(curve) if curve.weights().iter().all(|&w| w > 0.0) => Some(
            curve
                .control_points()
                .iter()
                .fold((low, high), |(lo, hi), &p| {
                    (lo.min(level(p)), hi.max(level(p)))
                }),
        ),
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => None,
    })
}

/// The bound a fixed edge spanning `[low, high]` puts on a full spring, or
/// `None` when the edge straddles the spring's level.
fn level_limit(spring: &Spring, fixed: FaceId, low: f64, high: f64) -> Option<AxialBlendClearance> {
    let linear = Tolerance::new().linear;
    let gap = if high < spring.level - linear {
        high - spring.level
    } else if low > spring.level + linear {
        low - spring.level
    } else {
        return None;
    };
    Some(AxialBlendClearance {
        moving: spring.face,
        fixed,
        gap,
        fixed_is_full: false,
    })
}

/// A circular contact of a blend on one carrier.
#[derive(Clone, Copy)]
struct Spring {
    face: FaceId,
    level: f64,
    full: bool,
}

/// The bounded pairs of moving and fixed springs on one carrier.
fn pair_limits(moving: &[Spring], fixed: &[Spring]) -> Vec<AxialBlendClearance> {
    let mut limits = Vec::new();
    for from in moving {
        for to in fixed {
            if from.full || to.full {
                limits.push(AxialBlendClearance {
                    moving: from.face,
                    fixed: to.face,
                    gap: to.level - from.level,
                    fixed_is_full: to.full,
                });
            }
        }
    }
    limits
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]

    use super::*;

    fn faces() -> (FaceId, FaceId) {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        (faces[0], faces[1])
    }

    fn spring(face: FaceId, level: f64, full: bool) -> Spring {
        Spring { face, level, full }
    }

    #[test]
    fn full_rim_records_a_fixed_partial_shoulder_below_it() {
        // An R1 cap rim (full circle at z=35) moving down a boss whose R4
        // shoulder wraps only the quarter inside the block (arc at z=28).
        let (rim, shoulder) = faces();
        let limits = pair_limits(&[spring(rim, 35.0, true)], &[spring(shoulder, 28.0, false)]);
        let [limit] = limits.as_slice() else {
            panic!("one bound expected, got {}", limits.len());
        };
        assert_eq!(limit.fixed(), shoulder);
        assert!(!limit.fixed_is_full_rim());
        assert!(!limit.reached(-6.9));
        assert!(limit.reached(-7.0));
        assert!(limit.reached(-10.0));
        assert!(limit.check(-10.0).is_err());
        // Moving away from the shoulder never reaches it.
        assert!(!limit.reached(25.2));
        assert!(limit.check(25.2).is_ok());
    }

    #[test]
    fn partial_shoulder_still_records_a_fixed_full_rim() {
        let (shoulder, rim) = faces();
        let limits = pair_limits(&[spring(shoulder, 31.0, false)], &[spring(rim, 33.2, true)]);
        let [limit] = limits.as_slice() else {
            panic!("one bound expected, got {}", limits.len());
        };
        assert!(limit.fixed_is_full_rim());
        assert!(!limit.reached(2.1));
        assert!(limit.reached(2.3));
        assert!(!limit.reached(-7.0));
    }

    #[test]
    fn two_partial_springs_are_not_bounded() {
        // Two arcs on one cylinder may sit at disjoint angles.
        let (a, b) = faces();
        assert!(pair_limits(&[spring(a, 35.0, false)], &[spring(b, 28.0, false)]).is_empty());
    }

    #[test]
    fn fixed_boundary_levels_bound_a_full_spring_on_either_side() {
        let (rim, patch) = faces();
        let full = spring(rim, 35.0, true);
        // A corner-patch contact reaching z=28 below the rim.
        let below = level_limit(&full, patch, 20.0, 28.0).unwrap();
        assert_eq!(below.fixed(), patch);
        assert!(!below.fixed_is_full_rim());
        assert!(!below.reached(-6.9));
        assert!(below.reached(-7.0));
        assert!(!below.reached(25.2));
        // An edge above the rim bounds an upward move instead.
        let above = level_limit(&full, patch, 40.0, 41.0).unwrap();
        assert!(above.reached(5.0));
        assert!(!above.reached(-30.0));
        // An edge straddling the rim's level cannot be bounded.
        assert!(level_limit(&full, patch, 30.0, 35.0).is_none());
        assert!(level_limit(&full, patch, 35.0, 36.0).is_none());
    }

    #[test]
    fn edge_levels_bound_lines_circles_and_control_hulls() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_cylinder(&mut topo, 6.0, 10.0).unwrap();
        let up = Vec3::new(0.0, 0.0, 1.0);
        let mut seen_line = false;
        let mut seen_circle = false;
        for edge in remus_topology::explorer::solid_edges(&topo, solid).unwrap() {
            let (low, high) = edge_levels(&topo, edge, up).unwrap().unwrap();
            match topo.edge(edge).unwrap().curve() {
                EdgeCurve::Line => {
                    seen_line = true;
                    assert!(low.abs() < 1e-12 && (high - 10.0).abs() < 1e-12);
                }
                EdgeCurve::Circle(_) => {
                    seen_circle = true;
                    assert!((high - low).abs() < 1e-12);
                }
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => {}
            }
            // Sideways, a rim circle spans its whole diameter.
            let (low, high) = edge_levels(&topo, edge, Vec3::new(1.0, 0.0, 0.0))
                .unwrap()
                .unwrap();
            assert!(low >= -6.0 - 1e-12 && high <= 6.0 + 1e-12);
        }
        assert!(seen_line && seen_circle);
    }

    #[test]
    fn full_circle_coverage_uses_the_carrier_radius() {
        assert!(covers_full_circle(TAU * 6.0, 6.0));
        assert!(!covers_full_circle(TAU * 6.0 / 4.0, 6.0));
        assert!(!covers_full_circle(TAU * 6.0 - 1e-3, 6.0));
    }
}
