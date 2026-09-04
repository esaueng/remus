//! Global self-intersection detection and removal.
//!
//! The qualified cell is deliberately narrow: a closed, straight-edged,
//! all-planar uniform-width L-prism may contain one disconnected inner region
//! whose two cap wires have crossed the local-collapse boundary and inverted.
//! When that whole component lies strictly inside one sound outer component, the inner
//! region is empty and is removed. A connected/partial fold, curved geometry,
//! or an ambiguous component layout refuses instead of discarding material.

use std::collections::BTreeMap;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::{Solid, SolidId};

use crate::error::OffsetError;

/// Exact result of the bounded self-intersection-removal pass.
#[derive(Debug)]
pub struct SelfIntersectionRemoval {
    /// Result solid. This is the input handle when no fold was detected.
    pub solid: SolidId,
    /// Faces belonging to the fully collapsed component that was excised.
    pub removed_faces: Vec<FaceId>,
}

#[derive(Clone, Copy)]
struct Bounds {
    min: Point3,
    max: Point3,
}

impl Bounds {
    fn from_points(points: impl Iterator<Item = Point3>) -> Option<Self> {
        let mut points = points.peekable();
        points.peek()?;
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut min_z = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut max_z = f64::NEG_INFINITY;
        for point in points {
            min_x = min_x.min(point.x());
            min_y = min_y.min(point.y());
            min_z = min_z.min(point.z());
            max_x = max_x.max(point.x());
            max_y = max_y.max(point.y());
            max_z = max_z.max(point.z());
        }
        Some(Self {
            min: Point3::new(min_x, min_y, min_z),
            max: Point3::new(max_x, max_y, max_z),
        })
    }

    fn scale(self) -> f64 {
        (self.max.x() - self.min.x())
            .abs()
            .max((self.max.y() - self.min.y()).abs())
            .max((self.max.z() - self.min.z()).abs())
            .max(1.0)
    }

    fn strictly_inside(self, outer: Self, tolerance: f64) -> bool {
        self.min.x() > outer.min.x() + tolerance
            && self.min.y() > outer.min.y() + tolerance
            && self.min.z() > outer.min.z() + tolerance
            && self.max.x() < outer.max.x() - tolerance
            && self.max.y() < outer.max.y() - tolerance
            && self.max.z() < outer.max.z() - tolerance
    }
}

struct FaceGeometry {
    face: FaceId,
    points: Vec<Point3>,
    effective_normal: Vec3,
    signed_area: f64,
    is_planar: bool,
    has_inner_wires: bool,
    all_edges_straight: bool,
}

struct Component {
    faces: Vec<FaceId>,
    geometry: Vec<FaceGeometry>,
    bounds: Bounds,
    non_positive_faces: Vec<usize>,
}

/// Detect and remove the qualified fully collapsed uniform-L-prism component.
///
/// An unchanged result means every inspected face retained positive boundary
/// orientation; it is not a claim that arbitrary curved global
/// self-intersections were searched for. If a non-positive face is observed,
/// the complete qualified proof is required before any face is removed.
/// `removable_faces` must be the construction-proven complete inner component;
/// a geometric guess is not accepted as deletion authority.
///
/// # Errors
///
/// Returns [`OffsetError::SelfIntersection`] when a detected fold is partial,
/// connected to the retained boundary, non-planar, non-prismatic, or otherwise
/// outside the declared exact cell.
pub fn remove_folded_uniform_l_prism_region(
    topo: &mut Topology,
    solid: SolidId,
    tolerance: f64,
    removable_faces: &[FaceId],
) -> Result<SelfIntersectionRemoval, OffsetError> {
    let solid_data = topo.solid(solid)?;
    if !solid_data.inner_shells().is_empty() {
        inspect_without_removal(topo, solid, tolerance)?;
        return Ok(SelfIntersectionRemoval {
            solid,
            removed_faces: Vec::new(),
        });
    }

    let shell_id = solid_data.outer_shell();
    let components = shell_components(topo, shell_id, tolerance)?;
    let folded: Vec<_> = components
        .iter()
        .enumerate()
        .filter(|(_, component)| !component.non_positive_faces.is_empty())
        .map(|(index, _)| index)
        .collect();
    if folded.is_empty() {
        return Ok(SelfIntersectionRemoval {
            solid,
            removed_faces: Vec::new(),
        });
    }

    if components.len() != 2 || folded.len() != 1 {
        return Err(unqualified(
            "the shell does not contain exactly one folded and one retained component",
        ));
    }

    let folded_index = folded[0];
    let retained_index = usize::from(folded_index == 0);
    let collapsed = &components[folded_index];
    let retained = &components[retained_index];
    let mut removable: Vec<_> = removable_faces.iter().map(|face| face.index()).collect();
    removable.sort_unstable();
    removable.dedup();
    let collapsed_indices: Vec<_> = collapsed.faces.iter().map(|face| face.index()).collect();
    if removable != collapsed_indices {
        return Err(unqualified(
            "the folded component does not exactly match the caller-proven removable face set",
        ));
    }
    if !retained.non_positive_faces.is_empty() {
        return Err(unqualified(
            "the component selected as the retained outer boundary is itself folded",
        ));
    }
    validate_closed_component(topo, collapsed)?;
    validate_closed_component(topo, retained)?;
    let collapsed_caps = validate_collapsed_prism(collapsed, tolerance)?;
    let retained_caps = validate_retained_l_prism(retained, tolerance)?;
    if collapsed.geometry[collapsed_caps.0]
        .effective_normal
        .dot(retained.geometry[retained_caps.0].effective_normal)
        .abs()
        < 1.0 - 1e-8
    {
        return Err(unqualified(
            "the collapsed and retained prism axes are not parallel",
        ));
    }
    let containment_tolerance = tolerance.max(collapsed.bounds.scale() * 1e-10);
    if !collapsed
        .bounds
        .strictly_inside(retained.bounds, containment_tolerance)
    {
        return Err(unqualified(
            "the folded prism is not strictly inside the retained boundary",
        ));
    }

    let rebuilt_shell = topo.add_shell(Shell::new(retained.faces.clone())?);
    remus_topology::validation::validate_shell_closed(topo.shell(rebuilt_shell)?, topo)?;
    let result = topo.add_solid(Solid::new(rebuilt_shell, Vec::new()));
    let mut removed_faces = collapsed.faces.clone();
    removed_faces.sort_unstable_by_key(|face| face.index());
    Ok(SelfIntersectionRemoval {
        solid: result,
        removed_faces,
    })
}

/// Detect and remove global self-intersections in the offset solid.
///
/// Construction-proven callers can use the bounded uniform-L remover above.
/// This general entry point has no such provenance, so any detected fold
/// returns a typed error rather than authorizing deletion.
///
/// # Errors
///
/// Returns [`OffsetError::SelfIntersection`] when a fold is detected outside
/// the qualified cell.
pub fn remove_self_intersections(
    topo: &mut Topology,
    solid: SolidId,
) -> Result<SolidId, OffsetError> {
    Ok(remove_folded_uniform_l_prism_region(
        topo,
        solid,
        remus_math::tolerance::Tolerance::new().linear,
        &[],
    )?
    .solid)
}

fn inspect_without_removal(
    topo: &Topology,
    solid: SolidId,
    tolerance: f64,
) -> Result<(), OffsetError> {
    let solid_data = topo.solid(solid)?;
    for shell_id in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        if shell_components(topo, shell_id, tolerance)?
            .iter()
            .any(|component| !component.non_positive_faces.is_empty())
        {
            return Err(unqualified(
                "a folded component was detected on an unsupported shell kind",
            ));
        }
    }
    Ok(())
}

fn shell_components(
    topo: &Topology,
    shell_id: ShellId,
    tolerance: f64,
) -> Result<Vec<Component>, OffsetError> {
    let mut faces = topo.shell(shell_id)?.faces().to_vec();
    faces.sort_unstable_by_key(|face| face.index());
    let mut edge_owner: BTreeMap<usize, usize> = BTreeMap::new();
    let mut parent: Vec<_> = (0..faces.len()).collect();
    for (position, face_id) in faces.iter().copied().enumerate() {
        let face = topo.face(face_id)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                if let Some(&other) = edge_owner.get(&oriented.edge().index()) {
                    union(&mut parent, position, other);
                } else {
                    edge_owner.insert(oriented.edge().index(), position);
                }
            }
        }
    }

    let mut grouped: BTreeMap<usize, Vec<FaceId>> = BTreeMap::new();
    for (position, face) in faces.into_iter().enumerate() {
        grouped
            .entry(root(&parent, position))
            .or_default()
            .push(face);
    }
    grouped
        .into_values()
        .map(|component| component_geometry(topo, component, tolerance))
        .collect()
}

fn root(parent: &[usize], mut index: usize) -> usize {
    while parent[index] != index {
        index = parent[index];
    }
    index
}

fn union(parent: &mut [usize], left: usize, right: usize) {
    let left = root(parent, left);
    let right = root(parent, right);
    if left != right {
        parent[right] = left;
    }
}

fn component_geometry(
    topo: &Topology,
    faces: Vec<FaceId>,
    tolerance: f64,
) -> Result<Component, OffsetError> {
    let mut geometry = Vec::with_capacity(faces.len());
    for face_id in &faces {
        geometry.push(face_geometry(topo, *face_id)?);
    }
    let bounds = Bounds::from_points(geometry.iter().flat_map(|face| face.points.iter().copied()))
        .ok_or_else(|| unqualified("a shell component has no boundary points"))?;
    let scale = bounds.scale();
    let area_tolerance = tolerance.max(scale * 1e-12) * scale;
    let non_positive_faces = geometry
        .iter()
        .enumerate()
        .filter(|(_, face)| face.signed_area <= area_tolerance)
        .map(|(index, _)| index)
        .collect();
    Ok(Component {
        faces,
        geometry,
        bounds,
        non_positive_faces,
    })
}

fn face_geometry(topo: &Topology, face_id: FaceId) -> Result<FaceGeometry, OffsetError> {
    let face = topo.face(face_id)?;
    let (effective_normal, is_planar) = match face.surface() {
        FaceSurface::Plane { normal, .. } => (
            if face.is_reversed() {
                -*normal
            } else {
                *normal
            },
            true,
        ),
        FaceSurface::Nurbs(_)
        | FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => (Vec3::new(0.0, 0.0, 0.0), false),
    };
    let wire = topo.wire(face.outer_wire())?;
    let mut points = Vec::with_capacity(wire.edges().len());
    let mut all_edges_straight = true;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        all_edges_straight &= matches!(edge.curve(), EdgeCurve::Line);
        points.push(topo.vertex(oriented.oriented_start(edge))?.point());
    }
    let signed_area = if is_planar {
        signed_area_along(&points, effective_normal)
    } else {
        f64::INFINITY
    };
    Ok(FaceGeometry {
        face: face_id,
        points,
        effective_normal,
        signed_area,
        is_planar,
        has_inner_wires: !face.inner_wires().is_empty(),
        all_edges_straight,
    })
}

fn signed_area_along(points: &[Point3], normal: Vec3) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let mut area = Vec3::new(0.0, 0.0, 0.0);
    for (point, next) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
    {
        area += Vec3::new(
            point.y() * next.z() - point.z() * next.y(),
            point.z() * next.x() - point.x() * next.z(),
            point.x() * next.y() - point.y() * next.x(),
        );
    }
    0.5 * area.dot(normal)
}

fn validate_closed_component(topo: &Topology, component: &Component) -> Result<(), OffsetError> {
    let mut uses: BTreeMap<usize, usize> = BTreeMap::new();
    for face_id in &component.faces {
        let face = topo.face(*face_id)?;
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                *uses.entry(oriented.edge().index()).or_default() += 1;
            }
        }
    }
    if uses.values().any(|count| *count != 2) {
        return Err(unqualified(
            "a candidate removal component is not independently closed and manifold",
        ));
    }
    Ok(())
}

fn validate_collapsed_prism(
    component: &Component,
    tolerance: f64,
) -> Result<(usize, usize), OffsetError> {
    if component
        .geometry
        .iter()
        .any(|face| !face.is_planar || face.has_inner_wires || !face.all_edges_straight)
    {
        return Err(unqualified(
            "the folded component has a curved edge or an inner wire",
        ));
    }
    let max_edges = component
        .geometry
        .iter()
        .map(|face| face.points.len())
        .max()
        .unwrap_or(0);
    let mut cap_candidates = [usize::MAX; 2];
    let mut cap_count = 0;
    for &index in &component.non_positive_faces {
        if component.geometry[index].points.len() == max_edges {
            if cap_count == cap_candidates.len() {
                return Err(unqualified("the folded component has too many cap faces"));
            }
            cap_candidates[cap_count] = index;
            cap_count += 1;
        }
    }
    // A uniform L collapses as its two caps and the two short arm-end walls
    // reverse together. Any other fold signature is outside this cell.
    if max_edges != 6 || cap_count != 2 || component.non_positive_faces.len() != 4 {
        return Err(unqualified(
            "the folded component does not have the four-face uniform-L collapse signature",
        ));
    }
    let cap_a = &component.geometry[cap_candidates[0]];
    let cap_b = &component.geometry[cap_candidates[1]];
    let edge_count = cap_a.points.len();
    if edge_count < 4
        || cap_b.points.len() != edge_count
        || component.geometry.len() != edge_count + 2
    {
        return Err(unqualified(
            "the folded component does not have the two-cap plus one-side-per-edge topology of a prism",
        ));
    }
    if cap_a.effective_normal.dot(cap_b.effective_normal) > -1.0 + 1e-8 {
        return Err(unqualified(
            "the two inverted candidate cap faces are not oppositely parallel",
        ));
    }
    let area_scale = cap_a
        .signed_area
        .abs()
        .max(cap_b.signed_area.abs())
        .max(component.bounds.scale() * tolerance);
    if (cap_a.signed_area - cap_b.signed_area).abs() > area_scale * 1e-8 {
        return Err(unqualified(
            "the two inverted candidate cap faces do not carry the same collapsed section",
        ));
    }
    for face in &component.geometry {
        if face.face == cap_a.face || face.face == cap_b.face {
            continue;
        }
        if face.points.len() != 4 {
            return Err(unqualified("a folded prism side is not a quadrilateral"));
        }
        let on_a = face
            .points
            .iter()
            .filter(|point| {
                point_plane_distance(**point, &cap_a.points, cap_a.effective_normal) <= tolerance
            })
            .count();
        let on_b = face
            .points
            .iter()
            .filter(|point| {
                point_plane_distance(**point, &cap_b.points, cap_b.effective_normal) <= tolerance
            })
            .count();
        if on_a != 2 || on_b != 2 {
            return Err(unqualified(
                "a folded prism side does not join corresponding cap edges",
            ));
        }
    }
    Ok((cap_candidates[0], cap_candidates[1]))
}

fn validate_retained_l_prism(
    component: &Component,
    tolerance: f64,
) -> Result<(usize, usize), OffsetError> {
    if component
        .geometry
        .iter()
        .any(|face| !face.is_planar || face.has_inner_wires || !face.all_edges_straight)
    {
        return Err(unqualified(
            "the retained component is not hole-free, straight-edged, and planar",
        ));
    }
    let mut caps = [usize::MAX; 2];
    let mut cap_count = 0;
    for (index, face) in component.geometry.iter().enumerate() {
        if face.points.len() == 6 {
            if cap_count == caps.len() {
                return Err(unqualified("the retained component has too many cap faces"));
            }
            caps[cap_count] = index;
            cap_count += 1;
        } else if face.points.len() != 4 {
            return Err(unqualified("the retained component has a non-quad side"));
        }
    }
    if cap_count != 2 || component.geometry.len() != 8 {
        return Err(unqualified(
            "the retained component is not a six-edge prism",
        ));
    }
    let cap_a = &component.geometry[caps[0]];
    let cap_b = &component.geometry[caps[1]];
    if cap_a.effective_normal.dot(cap_b.effective_normal) > -1.0 + 1e-8
        || (cap_a.signed_area - cap_b.signed_area).abs()
            > cap_a.signed_area.abs().max(cap_b.signed_area.abs()) * 1e-8
    {
        return Err(unqualified(
            "the retained component does not have matching opposite caps",
        ));
    }
    validate_uniform_l_profile(cap_a, tolerance)?;
    Ok((caps[0], caps[1]))
}

fn validate_uniform_l_profile(cap: &FaceGeometry, tolerance: f64) -> Result<(), OffsetError> {
    let mut edges = [Vec3::new(0.0, 0.0, 0.0); 6];
    let mut lengths = [0.0; 6];
    let mut scale = 0.0_f64;
    for index in 0..6 {
        edges[index] = cap.points[(index + 1) % 6] - cap.points[index];
        lengths[index] = edges[index].length();
        scale = scale.max(lengths[index]);
        if lengths[index] <= tolerance {
            return Err(unqualified("the retained profile has a degenerate edge"));
        }
    }
    let mut reflex = None;
    for index in 0..6 {
        let next = (index + 1) % 6;
        if edges[index].dot(edges[next]).abs() > lengths[index] * lengths[next] * 1e-8 {
            return Err(unqualified("the retained profile is not orthogonal"));
        }
        if edges[index].cross(edges[next]).dot(cap.effective_normal) < 0.0 {
            if reflex.is_some() {
                return Err(unqualified(
                    "the retained profile has multiple reflex corners",
                ));
            }
            reflex = Some(index);
        }
    }
    let Some(corner) = reflex else {
        return Err(unqualified(
            "the retained profile is not an L with exactly one reflex corner",
        ));
    };
    let close = |a: f64, b: f64| (a - b).abs() <= tolerance.max(scale * 1e-9);
    if !close(
        lengths[(corner + 4) % 6],
        lengths[corner] + lengths[(corner + 2) % 6],
    ) || !close(
        lengths[(corner + 3) % 6],
        lengths[(corner + 1) % 6] + lengths[(corner + 5) % 6],
    ) || !close(lengths[(corner + 5) % 6], lengths[(corner + 2) % 6])
    {
        return Err(unqualified(
            "the retained L profile does not have one uniform arm width",
        ));
    }
    Ok(())
}

fn point_plane_distance(point: Point3, plane: &[Point3], normal: Vec3) -> f64 {
    plane
        .first()
        .map_or(f64::INFINITY, |origin| normal.dot(point - *origin).abs())
}

fn unqualified(_detail: &'static str) -> OffsetError {
    OffsetError::SelfIntersection {
        reason: "outside the qualified uniform-L collapse cell".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::signed_area_along;
    use remus_math::vec::{Point3, Vec3};

    #[test]
    fn signed_area_tracks_requested_normal() {
        let square = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        assert!((signed_area_along(&square, Vec3::new(0.0, 0.0, 1.0)) - 1.0).abs() < 1e-12);
        assert!((signed_area_along(&square, Vec3::new(0.0, 0.0, -1.0)) + 1.0).abs() < 1e-12);
    }
}
