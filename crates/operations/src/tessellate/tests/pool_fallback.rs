//! Full-scan fallback of the circle contact index (`pool_index`).
//!
//! `PoolIndex::circle_candidates` returns `None`, and the caller scans the
//! whole shared pool, when the scan is estimated to cost at most 3/5 of the
//! walk: first from the walk's point count (`2700 · pool ≤ 9000 · points`,
//! i.e. `3 · pool ≤ 10 · (steps + 1)`), then from its distinct cells
//! (`2700 · pool ≤ 22500 · cells`, i.e. `3 · pool ≤ 25 · cells`). Both paths
//! feed the same acceptance test, in ascending id order, a superset of the
//! vertices it accepts, so the mesh must not depend on which one runs. These
//! tests pin both decision boundaries and compare meshes with the fallback
//! forced on (every circle scans the pool), forced off (the index is always
//! walked, exactly as before the fallback existed) and left to the rule.

#![allow(clippy::expect_used, clippy::unwrap_used)]

use std::f64::consts::PI;

use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use super::super::pool_index::PoolIndex;
use super::super::pool_index::test_hooks::{Verdicts, with_fallback};
use super::super::tessellate_solid;
use super::mesh_passes::{assert_bit_identical, grouped};

/// Acceptance distance the synthetic pools are indexed with (far below
/// their cell, so the cell is set by the pool extent alone).
const ACCEPT_TOL: f64 = 1e-6;

/// `finite` vertices spanning exactly 128 along x, so the grid cell is 1,
/// followed by `non_finite` vertices with a NaN coordinate. Vertex 22 sits
/// at (11, 0, 0), on [`unit_circle`].
fn pool(finite: usize, non_finite: usize) -> Vec<Point3> {
    assert!(finite > 22, "the pool must hold the on-circle vertex");
    let mut out: Vec<Point3> = (0..finite - 1)
        .map(|i| {
            #[allow(clippy::cast_precision_loss)]
            Point3::new((i % 100) as f64 * 0.5, (i / 100) as f64 * 0.25, 0.0)
        })
        .collect();
    out.push(Point3::new(128.0, 0.0, 0.0));
    out.extend((0..non_finite).map(|_| Point3::new(f64::NAN, 0.0, 0.0)));
    out
}

/// Unit circle about +z centred at (10, 0, 0).
fn unit_circle() -> Circle3D {
    Circle3D::new(Point3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
}

/// Candidates of `circle` over `range`, with the rule's verdicts.
fn decide(
    positions: &[Point3],
    circle: &Circle3D,
    range: Option<(f64, f64)>,
) -> (Option<Vec<u32>>, Verdicts) {
    let index = PoolIndex::new(positions, ACCEPT_TOL).expect("finite cell");
    with_fallback(None, || index.circle_candidates(circle, range))
}

/// Distinct cells the walk of `circle` visits in a pool of `positions`.
fn walk_cells(positions: &[Point3], circle: &Circle3D) -> usize {
    let index = PoolIndex::new(positions, ACCEPT_TOL).expect("finite cell");
    let (walked, seen) = with_fallback(Some(false), || index.circle_candidates(circle, None));
    assert!(walked.is_some(), "forced walk");
    seen.cells
}

const POINT_SCAN: Verdicts = Verdicts {
    point_scans: 1,
    cell_scans: 0,
    walks: 0,
    cells: 0,
};

const fn cell_scan(cells: usize) -> Verdicts {
    Verdicts {
        point_scans: 0,
        cell_scans: 1,
        walks: 0,
        cells,
    }
}

const fn walked(cells: usize) -> Verdicts {
    Verdicts {
        point_scans: 0,
        cell_scans: 0,
        walks: 1,
        cells,
    }
}

/// With a cell of 1 the full unit circle is walked in 15 steps
/// (`ceil((2π + 1) / 0.5)`): 16 points, so the point check scans pools of
/// up to 53 vertices (`3 · 53 ≤ 160 < 3 · 54`); its four quadrant cells
/// keep the cell check (`3 · pool ≤ 100`) below that. The arc `[0, 3]`
/// takes exactly `(3 + 1) / 0.5 = 8` steps: 9 points, a tie at 30 vertices.
#[test]
fn point_check_falls_back_at_the_boundary() {
    let circle = unit_circle();
    assert_eq!(
        walk_cells(&pool(54, 0), &circle),
        4,
        "one cell per quadrant"
    );
    assert_eq!(decide(&pool(53, 0), &circle, None), (None, POINT_SCAN));
    let (candidates, seen) = decide(&pool(54, 0), &circle, None);
    let candidates = candidates.expect("walked");
    assert_eq!(seen, walked(4));
    assert!(
        candidates.windows(2).all(|w| w[0] < w[1]),
        "ascending, unique"
    );
    assert!(
        candidates.contains(&22),
        "the on-circle vertex is a candidate"
    );
    assert!(candidates.len() < 54, "the walk narrows the scan");

    // Vertices with a non-finite coordinate count towards the pool size and
    // are candidates of every walk.
    assert_eq!(decide(&pool(52, 1), &circle, None), (None, POINT_SCAN));
    let (candidates, seen) = decide(&pool(53, 1), &circle, None);
    assert_eq!(seen, walked(4));
    assert!(candidates.expect("walked").contains(&53), "NaN vertex kept");

    // An integral step count whose estimates tie exactly scans.
    let arc = Some((0.0, 3.0));
    assert_eq!(decide(&pool(30, 0), &circle, arc), (None, POINT_SCAN));
    let (_, seen) = decide(&pool(31, 0), &circle, arc);
    assert_eq!(seen.point_scans, 0, "31 vertices pass the point check");
    assert_eq!(seen.cell_scans + seen.walks, 1);

    // An empty pool has nothing to scan.
    assert_eq!(decide(&[], &circle, Some((0.0, 0.01))), (None, POINT_SCAN));

    // Arcs too long to walk and non-finite radii never reach the rule.
    let z = Vec3::new(0.0, 0.0, 1.0);
    for radius in [1e5, f64::NAN] {
        let circle = Circle3D::new(Point3::new(0.0, 0.0, 0.0), z, radius).unwrap();
        assert_eq!(
            decide(&pool(54, 0), &circle, None),
            (None, Verdicts::default())
        );
    }
}

/// A radius-10 circle in the same cell-1 pool walks 129 points (128 steps
/// of 0.05 rad) through 64 distinct cells, more than 0.4 per point, so once
/// its cells are known the walk is dearer than the point count implied:
/// beyond the point check's 430 vertices, pools of up to 533 vertices
/// (`3 · 533 ≤ 25 · 64 < 3 · 534`) scan on the cell check.
#[test]
fn cell_check_falls_back_at_the_boundary() {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let circle = Circle3D::new(Point3::new(20.0, 0.0, 0.0), z, 10.0).unwrap();
    assert_eq!(walk_cells(&pool(431, 0), &circle), 64);
    assert_eq!(decide(&pool(430, 0), &circle, None), (None, POINT_SCAN));
    assert_eq!(decide(&pool(431, 0), &circle, None), (None, cell_scan(64)));
    assert_eq!(decide(&pool(533, 0), &circle, None), (None, cell_scan(64)));
    let (candidates, seen) = decide(&pool(534, 0), &circle, None);
    assert_eq!(seen, walked(64));
    let candidates = candidates.expect("walked");
    assert!(candidates.windows(2).all(|w| w[0] < w[1]));
    assert!(
        candidates.contains(&20) && candidates.contains(&60),
        "the on-circle vertices are candidates"
    );
}

/// At the bench plates' scale (a 100 × 100 × 10 pool, cell 100 / 128) a
/// radius-2 hole rim takes 35 steps, 36 points, through 17 cells: pools of
/// up to 120 vertices scan on the point check, up to 141 (`3 · 141 ≤
/// 25 · 17`) on the cell check. The 64-hole plate's pool and the partial
/// plates where a scan costs more than the walk (24 holes: 344-872
/// vertices at deflection 0.1-0.5) are far above that and walk.
#[test]
fn rule_at_hole_plate_scale() {
    let rim = |cx: f64, cy: f64, z: f64, n: u32| {
        (0..n).map(move |k| {
            let a = f64::from(k) * 2.0 * PI / f64::from(n);
            Point3::new(cx + 2.0 * a.cos(), cy + 2.0 * a.sin(), z)
        })
    };
    let mut plate = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 100.0, 10.0)];
    for row in 0..8_u32 {
        for col in 0..8_u32 {
            let (cx, cy) = (6.0 + f64::from(col) * 12.0, 6.0 + f64::from(row) * 12.0);
            plate.extend(rim(cx, cy, 0.0, 10));
            plate.extend(rim(cx, cy, 10.0, 10));
        }
    }
    let hole = Circle3D::new(Point3::new(6.0, 6.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    assert_eq!(walk_cells(&plate, &hole), 17);
    let (candidates, seen) = decide(&plate, &hole, None);
    assert_eq!(
        seen,
        walked(17),
        "a {}-vertex plate pool walks",
        plate.len()
    );
    let candidates = candidates.expect("walked");
    assert!(
        (2..12).all(|gid| candidates.contains(&gid)),
        "the rim is found"
    );

    plate.truncate(142);
    let (candidates, seen) = decide(&plate, &hole, None);
    assert_eq!(seen, walked(17));
    let candidates = candidates.expect("walked");
    assert!(
        (2..12).all(|gid| candidates.contains(&gid)),
        "the rim is found"
    );
    plate.truncate(141);
    assert_eq!(decide(&plate, &hole, None), (None, cell_scan(17)));
    plate.truncate(121);
    assert_eq!(decide(&plate, &hole, None), (None, cell_scan(17)));
    plate.truncate(120);
    assert_eq!(decide(&plate, &hole, None), (None, POINT_SCAN));
}

/// The test switch overrides the rule both ways; the verdicts keep
/// recording the rule's own checks.
#[test]
fn forced_modes_override_the_rule() {
    let circle = unit_circle();
    let small = pool(53, 0);
    let index = PoolIndex::new(&small, ACCEPT_TOL).expect("finite cell");
    let (forced_walk, seen) = with_fallback(Some(false), || index.circle_candidates(&circle, None));
    // The point check would scan; forced on, the walk reaches the cell check.
    let rule = Verdicts {
        point_scans: 1,
        ..walked(4)
    };
    assert_eq!(seen, rule);
    let forced_walk = forced_walk.expect("forced walk");
    assert!(forced_walk.contains(&22) && forced_walk.len() < 53);

    let large = pool(54, 0);
    let index = PoolIndex::new(&large, ACCEPT_TOL).expect("finite cell");
    let forced_scan = with_fallback(Some(true), || index.circle_candidates(&circle, None));
    // Forced to scan, the circle stops at the point check, which walks.
    assert_eq!(forced_scan, (None, Verdicts::default()));
}

/// Box ∩ sphere (the PERF-D03 curved bench body).
fn box_sphere(topo: &mut Topology) -> SolidId {
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::{make_box, make_sphere};
    let bx = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let sp = make_sphere(topo, 7.0, 16).unwrap();
    boolean(topo, BooleanOp::Intersect, bx, sp).unwrap()
}

/// A square profile revolved a full turn about an axis clear of it: a tube
/// whose four rims are full circles.
fn revolved(topo: &mut Topology) -> SolidId {
    let profile = remus_topology::builder::make_polygon_wire(
        topo,
        &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(remus_topology::face::Face::new(
        profile,
        vec![],
        remus_topology::face::FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ));
    crate::revolve::revolve(
        topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        2.0 * PI,
    )
    .unwrap()
}

/// A 20 × 10 × 10 box with every edge filleted at r = 1.5: quarter-circle
/// blend rims meeting at the corner patches.
fn filleted(topo: &mut Topology) -> SolidId {
    let sharp = crate::primitives::make_box(topo, 20.0, 10.0, 10.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(topo, sharp).unwrap();
    crate::blend_ops::fillet_v2(topo, sharp, &edges, 1.5)
        .unwrap()
        .solid
}

/// `body` with the through holes `(x, y, radius)` cut along z.
fn drilled(topo: &mut Topology, mut body: SolidId, holes: &[(f64, f64, f64)]) -> SolidId {
    use crate::boolean::{BooleanOp, boolean};
    use crate::primitives::make_cylinder;
    use crate::transform::transform_solid;
    for &(x, y, radius) in holes {
        let cyl = make_cylinder(topo, radius, 20.0).unwrap();
        transform_solid(topo, cyl, &Mat4::translation(x, y, -5.0)).unwrap();
        body = boolean(topo, BooleanOp::Cut, body, cyl).unwrap();
    }
    body
}

/// Grouped and ungrouped meshes of `body` with the fallback left to the
/// rule and forced on are byte-identical to those with it forced off (the
/// pre-fallback walk). Returns the rule's verdicts over the grouped rule
/// run so callers can check which path it took.
fn assert_modes_identical(
    topo: &Topology,
    body: SolidId,
    deflection: f64,
    angular: f64,
    label: &str,
) -> Verdicts {
    let context = format!("{label} at {deflection}/{angular}");
    let (rule, seen) = with_fallback(None, || grouped(topo, body, deflection, angular));
    let (scan, _) = with_fallback(Some(true), || grouped(topo, body, deflection, angular));
    let (walk, _) = with_fallback(Some(false), || grouped(topo, body, deflection, angular));
    assert_bit_identical(&rule, &walk, &format!("{context}: rule vs walk"));
    assert_bit_identical(&scan, &walk, &format!("{context}: scan vs walk"));
    let plain = |force| {
        with_fallback(force, || {
            (
                tessellate_solid(topo, body, deflection).unwrap(),
                Vec::new(),
            )
        })
        .0
    };
    let walk = plain(Some(false));
    assert_bit_identical(
        &plain(None),
        &walk,
        &format!("{context} ungrouped: rule vs walk"),
    );
    assert_bit_identical(
        &plain(Some(true)),
        &walk,
        &format!("{context} ungrouped: scan vs walk"),
    );
    seen
}

/// Small curved bodies mesh identically whichever path each circle takes;
/// the long rims of box ∩ sphere and the tube over their tiny pools always
/// scan.
#[test]
fn meshes_identical_with_fallback_forced_on_and_off() {
    let mut topo = Topology::new();
    let curved = [
        ("box-sphere", box_sphere(&mut topo), true),
        ("revolved", revolved(&mut topo), true),
        ("filleted", filleted(&mut topo), false),
    ];
    for (label, body, always_scans) in curved {
        for (deflection, angular) in [(0.1, 0.5), (0.01, 0.1)] {
            let seen = assert_modes_identical(&topo, body, deflection, angular, label);
            assert!(
                seen.walks + seen.scans() > 0,
                "{label} at {deflection}: no circle"
            );
            if always_scans {
                assert_eq!(seen.walks, 0, "{label} at {deflection}: a circle walked");
            }
        }
    }
}

/// The 64-hole bench plate and its first 24 holes alone: every hole rim
/// walks the index (scanning the 24-hole plate's pool would cost 2.6x the
/// walk at 0.1/0.35 and 1.05x at 0.5/1.0) and meshes identically.
#[test]
fn hole_plates_walk_the_index_with_identical_meshes() {
    let mut topo = Topology::new();
    let holes: Vec<(f64, f64, f64)> = (0..64_u32)
        .map(|i| {
            let (row, col) = (f64::from(i / 8), f64::from(i % 8));
            (6.0 + col * 12.0, 6.0 + row * 12.0, 2.0)
        })
        .collect();
    let block = crate::primitives::make_box(&mut topo, 100.0, 100.0, 10.0).unwrap();
    let partial = drilled(&mut topo, block, &holes[..24]);
    let full = drilled(&mut topo, partial, &holes[24..]);
    for (label, body, deflection, angular) in [
        ("24-hole plate", partial, 0.1, 0.35),
        ("24-hole plate", partial, 0.5, 1.0),
        ("64-hole plate", full, 0.1, 0.5),
    ] {
        let seen = assert_modes_identical(&topo, body, deflection, angular, label);
        assert!(
            seen.walks > 0,
            "{label} at {deflection}: the plate walks the index"
        );
        assert_eq!(
            seen.scans(),
            0,
            "{label} at {deflection}: a plate circle fell back"
        );
    }
}

/// Hole radii from 0.25 to 8 in one 60 × 60 × 10 block: at display
/// tolerance the large rims scan the pool and the small ones walk, in the
/// same refinement pass, with identical meshes.
#[test]
fn mixed_hole_sizes_take_both_paths_with_identical_meshes() {
    let mut topo = Topology::new();
    let block = crate::primitives::make_box(&mut topo, 60.0, 60.0, 10.0).unwrap();
    let body = drilled(
        &mut topo,
        block,
        &[
            (15.0, 15.0, 8.0),
            (45.0, 15.0, 4.0),
            (15.0, 45.0, 2.0),
            (45.0, 45.0, 1.0),
            (30.0, 30.0, 0.5),
            (30.0, 8.0, 0.25),
            (8.0, 30.0, 3.0),
            (52.0, 30.0, 1.5),
        ],
    );
    let seen = assert_modes_identical(&topo, body, 0.1, 0.35, "mixed holes");
    assert!(seen.walks > 0, "the small rims walk");
    assert!(
        seen.point_scans > 0,
        "the large rims scan on their point count"
    );
    assert!(seen.cell_scans > 0, "a mid-size rim scans on its cells");
}
