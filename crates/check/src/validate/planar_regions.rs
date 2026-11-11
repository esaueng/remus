//! Profile construction's adapter to the shared certified loop-nesting reader.
//!
//! No sampling ray or carrier-domain reconstruction is used here. Parent
//! indices come from the same decision that certified the nesting depths.
use crate::CheckError;
use crate::properties::loop_nesting::{
    InnerLoopNesting, inner_loop_nesting_report_with_domains, nesting_of_with_domains,
};
use remus_topology::edge::EdgeId;
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    face::{FaceId, FaceSurface},
    wire::WireId,
};
use std::collections::BTreeMap;

/// Inner-wire depths and immediate parents, parallel to `Face::inner_wires`.
/// Depth zero is a hole; depth one is an island. `None` means the outer wire.
pub struct PlanarRegions {
    /// Number of enclosing inner wires.
    pub depths: Vec<usize>,
    /// Index of the nearest enclosing inner wire.
    pub parents: Vec<Option<usize>>,
}

fn refuse() -> CheckError {
    CheckError::ValidationFailed("profile nesting requires certified separated boundaries".into())
}

/// Preserve the earlier structural checks on polygon/full-circle profiles.
/// Geometry and containment are still read only by the shared reader. Other
/// outer carriers never gate the independent inner-loop decision.
fn simple_boundary(topo: &Topology, wire: WireId) -> Result<bool, CheckError> {
    let wire = topo.wire(wire)?;
    let mut linear = true;
    for oe in wire.edges() {
        linear &= crate::distance::edge::is_linear_curve(topo.edge(oe.edge())?.curve());
    }
    if linear {
        if !wire.is_closed() || wire.edges().is_empty() {
            return Err(refuse());
        }
        return Ok(true);
    }
    let [oe] = wire.edges() else {
        return Ok(false);
    };
    let edge = topo.edge(oe.edge())?;
    Ok(edge.is_closed() && matches!(edge.curve(), EdgeCurve::Circle(_)))
}

/// Read certified inner-wire containment independently of the outer carrier.
///
/// Flat profiles retain their existing construction; nested profiles use the
/// certified tree. An invalid or undecided reading never supplies guessed
/// depths or a partially certified parent list.
///
/// # Errors
/// Refuses invalid, non-planar or undecided nesting and propagates missing
/// topology handles. Existing polygon/full-circle structural checks remain.
pub fn planar_regions(topo: &Topology, face: FaceId) -> Result<PlanarRegions, CheckError> {
    planar_regions_with_domains(topo, face, &BTreeMap::new())
}

/// [`planar_regions`] over the ranges the raw profile normalizer has certified
/// but has not written yet. Stored trims take precedence.
///
/// # Errors
/// The same refusals as [`planar_regions`].
pub fn planar_regions_with_domains(
    topo: &Topology,
    face: FaceId,
    domains: &BTreeMap<EdgeId, (f64, f64)>,
) -> Result<PlanarRegions, CheckError> {
    let data = topo.face(face)?;
    let FaceSurface::Plane { normal, d } = data.surface() else {
        return Err(refuse());
    };
    if !normal.length().is_finite() || (normal.length() - 1.).abs() > 1e-10 || !d.is_finite() {
        return Err(refuse());
    }
    let count = data.inner_wires().len();
    let report = inner_loop_nesting_report_with_domains(topo, face, domains);
    let regions = match report.nesting {
        InnerLoopNesting::Flat => PlanarRegions {
            depths: vec![0; count],
            parents: vec![None; count],
        },
        InnerLoopNesting::Nested(depths) => PlanarRegions {
            depths,
            parents: report.parents.ok_or_else(refuse)?,
        },
        InnerLoopNesting::Invalid | InnerLoopNesting::Undecided(_) => return Err(refuse()),
    };
    let wires: Vec<_> = std::iter::once(data.outer_wire())
        .chain(data.inner_wires().iter().copied())
        .collect();
    let mut simple = true;
    for &wire in &wires {
        let is_simple = simple_boundary(topo, wire)?;
        simple &= is_simple;
        if is_simple
            && !crate::validate::check_wire_self_intersection(
                topo,
                wire,
                remus_math::tolerance::Tolerance::new().linear,
            )?
            .is_empty()
        {
            return Err(refuse());
        }
    }
    if simple && count > 0 {
        // Include the outline as the root to keep outside, touching and
        // crossing polygon/full-circle profiles refused. This uses exactly
        // the same enclosures, contact checks and work budget as the inner
        // decision; there is no second containment algorithm.
        match nesting_of_with_domains(topo, &wires, *normal, domains).nesting {
            InnerLoopNesting::Nested(depths)
                if depths[0] == 0
                    && depths[1..]
                        .iter()
                        .zip(&regions.depths)
                        .all(|(&with_outer, &inner)| with_outer == inner + 1) => {}
            _ => return Err(refuse()),
        }
    }
    Ok(regions)
}

#[cfg(test)]
mod trim_bounds_tests;

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::{builder::make_polygon_wire, face::Face};
    fn square(topo: &mut Topology, r: f64, cw: bool) -> WireId {
        let mut p = vec![
            Point3::new(-r, -r, 5.),
            Point3::new(r, -r, 5.),
            Point3::new(r, r, 5.),
            Point3::new(-r, r, 5.),
        ];
        if cw {
            p.reverse();
        }
        make_polygon_wire(topo, &p, 1e-7).unwrap()
    }
    fn face(topo: &mut Topology, wrong_island: bool) -> FaceId {
        let outer = square(topo, 10., false);
        let hole = square(topo, 5., true);
        let island = square(topo, 3., wrong_island);
        let bore = square(topo, 1., true);
        // Deliberately not sorted by depth.
        topo.add_face(Face::new(
            outer,
            vec![island, bore, hole],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ))
    }
    #[test]
    fn planar_regions_find_parents_independent_of_wire_order() {
        let mut topo = Topology::new();
        let face = face(&mut topo, false);
        let r = planar_regions(&topo, face).unwrap();
        assert_eq!(r.depths, vec![1, 2, 0]);
        assert_eq!(r.parents, vec![Some(2), Some(0), None]);
    }
    #[test]
    fn planar_orientation_accepts_alternating_depth_and_face_reversal() {
        let mut topo = Topology::new();
        let face = face(&mut topo, false);
        for reversed in [false, true] {
            topo.face_mut(face).unwrap().set_reversed(reversed);
            assert!(
                crate::validate::check_face_inner_wire_orientation(&topo, face)
                    .unwrap()
                    .is_empty()
            );
        }
    }
    #[test]
    fn planar_orientation_rejects_wrong_island_winding() {
        let mut topo = Topology::new();
        let face = face(&mut topo, true);
        let issues = crate::validate::check_face_inner_wire_orientation(&topo, face).unwrap();
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].severity, crate::validate::Severity::Error);
    }
    #[test]
    fn planar_regions_refuse_self_crossing_and_budget_exhaustion() {
        let mut topo = Topology::new();
        let outer = square(&mut topo, 10., false);
        let bow = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(-2., -2., 5.),
                Point3::new(2., 2., 5.),
                Point3::new(-2., 2., 5.),
                Point3::new(2., -2., 5.),
            ],
            1e-7,
        )
        .unwrap();
        let id = topo.add_face(Face::new(
            outer,
            vec![bow],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ));
        assert!(planar_regions(&topo, id).is_err());
        let id = topo.add_face(Face::new(
            outer,
            vec![bow; 64],
            FaceSurface::Plane {
                normal: Vec3::new(0., 0., 1.),
                d: 5.,
            },
        ));
        assert!(planar_regions(&topo, id).is_err());
    }
}
