//! Exact supported profile regions and separate caps for material islands.
use super::{Face, FaceId, FaceSurface, OrientedEdge, Topology, Vec3, Wire, WireId};
use remus_check::validate::planar_regions::{PlanarRegions, planar_regions};

pub(super) fn profile_regions(
    topo: &Topology,
    face: FaceId,
) -> Result<PlanarRegions, crate::OperationsError> {
    let count = topo.face(face)?.inner_wires().len();
    if count < 2 {
        return Ok(PlanarRegions {
            depths: vec![0; count],
            parents: vec![None; count],
        });
    }
    match planar_regions(topo, face) {
        Ok(regions) => Ok(regions),
        Err(_) if has_other_carriers(topo, face)? && separated_inner_bounds(topo, face)? => {
            Ok(PlanarRegions {
                depths: vec![0; count],
                parents: vec![None; count],
            })
        }
        Err(error) => Err(crate::OperationsError::Unsupported {
            operation: "profile sweep",
            reason: format!("profile nesting could not be certified: {error}"),
        }),
    }
}

#[allow(clippy::redundant_pub_crate)]
pub(crate) fn refuse_nested_profile(
    topo: &Topology,
    face: FaceId,
    operation: &str,
) -> Result<(), crate::OperationsError> {
    if profile_regions(topo, face)?
        .depths
        .iter()
        .any(|&depth| depth > 0)
    {
        return Err(crate::OperationsError::Unsupported {
            operation: "profile sweep",
            reason: format!("{operation} does not support nested inner wires"),
        });
    }
    Ok(())
}

pub(super) fn orient_profile_wire(
    topo: &mut Topology,
    wire: WireId,
    offset: Vec3,
    cw: bool,
) -> Result<WireId, crate::OperationsError> {
    let (normal, _) = remus_topology::builder::wire_plane(topo, wire)?;
    if (normal.dot(offset) < 0.) == cw {
        return Ok(wire);
    }
    let edges = topo
        .wire(wire)?
        .edges()
        .iter()
        .rev()
        .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
        .collect();
    Ok(topo.add_wire(Wire::new(edges, true)?))
}

pub(super) fn region_caps(
    topo: &mut Topology,
    outer: WireId,
    wires: &[WireId],
    regions: &PlanarRegions,
    surface: &FaceSurface,
    islands: &mut Vec<FaceId>,
) -> FaceId {
    let holes = wires
        .iter()
        .enumerate()
        .filter(|(i, _)| regions.depths[*i] == 0)
        .map(|(_, &wire)| wire)
        .collect();
    let main = topo.add_face(Face::new(outer, holes, surface.clone()));
    for (index, &wire) in wires.iter().enumerate() {
        if regions.depths[index].is_multiple_of(2) {
            continue;
        }
        let holes = wires
            .iter()
            .enumerate()
            .filter(|(i, _)| regions.parents[*i] == Some(index))
            .map(|(_, &wire)| wire)
            .collect();
        islands.push(topo.add_face(Face::new(wire, holes, surface.clone())));
    }
    main
}

// Preserve the existing unnested curved-profile path only when conservative
// whole-carrier boxes prove every inner boundary separate from every other.
// Overlapping boxes never prove anything: those profiles need the exact reader.
fn separated_inner_bounds(topo: &Topology, face: FaceId) -> Result<bool, crate::OperationsError> {
    use remus_geometry::bounds::curve::{
        circle_arc_bounds, ellipse_arc_bounds, line_segment_bounds, nurbs_curve_bounds,
    };
    use remus_math::aabb::Aabb3;
    use remus_topology::edge::EdgeCurve;
    let face_id = face;
    let face = topo.face(face)?;
    if !matches!(face.surface(), FaceSurface::Plane { .. }) || face.inner_wires().len() > 63 {
        return Ok(false);
    }
    let mut boxes: Vec<Aabb3> = Vec::new();
    for &wire in face.inner_wires() {
        let wire = topo.wire(wire)?;
        if wire.edges().is_empty() || wire.edges().len() > 256 {
            return Ok(false);
        }
        let mut box_: Option<Aabb3> = None;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let bound = match edge.curve() {
                EdgeCurve::Line => line_segment_bounds(
                    topo.vertex(edge.start())?.point(),
                    topo.vertex(edge.end())?.point(),
                ),
                EdgeCurve::Circle(c) => circle_arc_bounds(c, 0., std::f64::consts::TAU),
                EdgeCurve::Ellipse(c) => ellipse_arc_bounds(c, 0., std::f64::consts::TAU),
                EdgeCurve::NurbsCurve(c) => {
                    let (a, b) = c.domain();
                    nurbs_curve_bounds(c, a, b)
                }
                EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => return Ok(false),
            };
            if !bound.is_prunable() {
                return Ok(false);
            }
            box_ = Some(box_.map_or_else(|| bound.aabb(), |b| b.union(bound.aabb())));
        }
        let Some(box_) = box_ else {
            return Ok(false);
        };
        let box_ = box_.expanded(remus_math::tolerance::Tolerance::new().linear);
        if !remus_check::validate::planar_regions::planar_outer_contains_box(topo, face_id, box_)? {
            return Ok(false);
        }
        if boxes.iter().any(|other| other.intersects(box_)) {
            return Ok(false);
        }
        boxes.push(box_);
    }
    Ok(true)
}

// Structural failures on fully supported boundaries must never take the
// compatibility path (in particular, a self-crossing polygon is not a hole).
fn has_other_carriers(topo: &Topology, face: FaceId) -> Result<bool, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;
    for &wire in topo.face(face)?.inner_wires() {
        let wire = topo.wire(wire)?;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            if !(matches!(edge.curve(), EdgeCurve::Line)
                || wire.edges().len() == 1
                    && edge.start() == edge.end()
                    && matches!(edge.curve(), EdgeCurve::Circle(_)))
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}
