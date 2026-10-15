//! A spliced copy restores coedge authority by position, skipping the
//! splice's inserted hole loops, exactly as the per-snapshot search does:
//! the same drilled plate, with stored pcurves and periodic windings on every
//! face, is built both ways and every coedge compared.

use std::fmt::Write as _;

use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Vec2};
use remus_topology::Topology;
use remus_topology::coedge::PeriodicWinding;
use remus_topology::explorer::solid_faces;
use remus_topology::pcurve::PCurve;

use super::super::{ALIGNMENT_USES, SEARCH_ONLY, SNAPSHOT_SEARCHES, face_coedges};

/// A plate drilled with `side × side` through holes one cut at a time, every
/// face's coedges given authority before each cut, dumped coedge by coedge.
fn drilled_plate(side: u32, search_only: bool) -> (String, usize, usize) {
    SEARCH_ONLY.set(search_only);
    ALIGNMENT_USES.set(0);
    SNAPSHOT_SEARCHES.set(0);
    let mut topo = Topology::new();
    let extent = f64::from(side) * 12.0 + 4.0;
    let mut plate = crate::primitives::make_box(&mut topo, extent, extent, 10.0).unwrap();
    for row in 0..side {
        for col in 0..side {
            for (index, face) in solid_faces(&topo, plate).unwrap().into_iter().enumerate() {
                for (k, coedge) in face_coedges(&topo, face).unwrap().into_iter().enumerate() {
                    let offset = f64::from(u32::try_from(index * 7 + k).unwrap());
                    let pcurve = PCurve::new(
                        Curve2D::Line(
                            Line2D::new(Point2::new(offset, 1.0), Vec2::new(1.0, 0.0)).unwrap(),
                        ),
                        0.0,
                        offset + 1.0,
                    );
                    topo.set_coedge_pcurve(coedge, pcurve).unwrap();
                    let turns = i32::try_from(k % 3).unwrap() - 1;
                    topo.set_coedge_periodic_winding(coedge, PeriodicWinding::new(turns, 1))
                        .unwrap();
                }
            }
            let tool = crate::primitives::make_cylinder(&mut topo, 2.0, 20.0).unwrap();
            let (x, y) = (6.0 + f64::from(col) * 12.0, 6.0 + f64::from(row) * 12.0);
            crate::transform::transform_solid(&mut topo, tool, &Mat4::translation(x, y, -5.0))
                .unwrap();
            plate = crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, plate, tool)
                .unwrap();
        }
    }
    let mut dump = String::new();
    for face in solid_faces(&topo, plate).unwrap() {
        writeln!(dump, "face {}", face.index()).unwrap();
        for coedge in face_coedges(&topo, face).unwrap() {
            let coedge = topo.coedge(coedge).unwrap();
            writeln!(
                dump,
                "  {} {} {:?} {:?}",
                coedge.edge().index(),
                coedge.is_forward(),
                coedge.periodic_winding(),
                coedge.pcurve()
            )
            .unwrap();
        }
    }
    SEARCH_ONLY.set(false);
    (dump, ALIGNMENT_USES.get(), SNAPSHOT_SEARCHES.get())
}

#[test]
fn aligned_authority_restore_matches_the_per_snapshot_search() {
    let (aligned, uses, _) = drilled_plate(4, false);
    let (searched, _, searches) = drilled_plate(4, true);
    assert!(uses > 0, "no visit skipped inserted coedges");
    assert!(
        aligned.contains("Some(PCurve"),
        "premise: authority survives"
    );
    assert!(searches > 0, "premise: the reference searched");
    assert_eq!(aligned, searched);
}

/// The per-snapshot search scans every coedge of the face for each
/// snapshot, so drilling a plate one hole at a time through it is cubic in
/// the hole count. With the splice aligned no drilled face needs it.
#[test]
fn drilling_a_plate_never_falls_back_to_the_per_snapshot_search() {
    for side in [2, 5] {
        let (_, uses, searches) = drilled_plate(side, false);
        assert!(uses > 0, "{side}x{side}: no visit skipped inserted coedges");
        assert_eq!(searches, 0, "{side}x{side}: snapshots searched");
    }
}
