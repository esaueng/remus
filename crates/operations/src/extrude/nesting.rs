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
