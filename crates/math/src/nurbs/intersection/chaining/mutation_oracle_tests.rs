//! Hand-placed oracles for the grid-accelerated nearest-unused walk
//! (B19 F3a).
//!
//! `nearest_unused_ring` must return exactly what the full scan returns:
//! the nearest unused component member, lowest component rank on a tie.
//! Each case places a handful of points so that the answer is obvious by
//! hand and a wrong stop, filter, distance or tie-break would pick a
//! different, also hand-known, point. The full scan is asserted alongside
//! as the specification.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::float_cmp
)]

use super::*;

fn ip(x: f64, y: f64, z: f64) -> IntersectionPoint {
    IntersectionPoint {
        point: Point3::new(x, y, z),
        param1: (x, y),
        param2: (z, 0.0),
    }
}

/// Run one walk step from `current` over `comp` (in rank order) with the
/// given adjacency and already-used members, through both the ring search
/// and the full scan. Returns `(ring, scan)`.
fn pick(
    points: &[IntersectionPoint],
    width: f64,
    comp: &[usize],
    adj: &[Vec<usize>],
    current: usize,
    used_ids: &[usize],
) -> (Option<usize>, Option<usize>) {
    let n = points.len();
    let grid = ChainGrid::build(points, width);
    let mut membership = StampSet::new(n);
    let epoch = membership.next_epoch();
    for (rank, &i) in comp.iter().enumerate() {
        membership.mark(i, epoch, rank);
    }
    let is_member = |i: usize| membership.is_marked(i, epoch);
    let rank_of = |i: usize| membership.value(i, epoch);
    let mut used = StampSet::new(n);
    let used_epoch = used.next_epoch();
    used.mark(current, used_epoch, 0);
    for &i in used_ids {
        used.mark(i, used_epoch, 0);
    }
    let mut cmax = 0.0f64;
    for &i in comp {
        let q = points[i].point;
        cmax = cmax.max(q.x().abs()).max(q.y().abs()).max(q.z().abs());
    }
    let ring = nearest_unused_ring(
        points,
        comp,
        adj,
        &grid,
        current,
        cmax * cmax,
        used_epoch,
        &used,
        &is_member,
        &rank_of,
    );
    let scan = nearest_unused_scan(points, comp, current, used_epoch, &used);
    (ring, scan)
}

fn no_edges(n: usize) -> Vec<Vec<usize>> {
    vec![Vec::new(); n]
}

/// Width 1/2, walk from the centre of cell (0,0,0). A decoy on the cell
/// diagonal at ring 5 is 2.5 sqrt(3) ~ 4.33 away; the true nearest lies on
/// the x axis in ring 9, 4.3 away. Rings past the incumbent are only
/// provably empty once (ring - 1) * width exceeds 4.33, i.e. at ring 10:
/// stopping any earlier returns the decoy.
#[test]
fn ring_stop_waits_for_a_closer_point_in_a_higher_ring() {
    let points = [
        ip(0.25, 0.25, 0.25),
        ip(2.75, 2.75, 2.75),
        ip(4.55, 0.25, 0.25),
    ];
    let (ring, scan) = pick(&points, 0.5, &[0, 1, 2], &no_edges(3), 0, &[]);
    assert_eq!(scan, Some(2));
    assert_eq!(ring, Some(2));
}

/// Width 1/8: the decoy sits at ring 2 (squared distance 3/16), the true
/// nearest at ring 3 (squared distance ~0.1008). The stop must compare
/// squared lengths with squared lengths: a decoy distance below one model
/// unit must not end the search at ring 2.
#[test]
fn ring_stop_compares_squared_reach_with_squared_distance() {
    let points = [
        ip(0.0625, 0.0625, 0.0625),
        ip(0.3125, 0.3125, 0.3125),
        ip(0.38, 0.0625, 0.0625),
    ];
    let (ring, scan) = pick(&points, 0.125, &[0, 1, 2], &no_edges(3), 0, &[]);
    assert_eq!(scan, Some(2));
    assert_eq!(ring, Some(2));
}

/// The same layout scaled by 2^-500: squared distances (~1e-302) sit far
/// below the 1e-280 rounding floor of the stop tolerance, so the stop rule
/// never fires and the full scan decides. A tolerance that went negative
/// at this scale would stop at ring 2 on the decoy.
#[test]
fn ring_stop_floor_holds_at_tiny_scale() {
    let s = 2.0f64.powi(-500);
    let points = [
        ip(0.0625 * s, 0.0625 * s, 0.0625 * s),
        ip(0.3125 * s, 0.3125 * s, 0.3125 * s),
        ip(0.38 * s, 0.0625 * s, 0.0625 * s),
    ];
    let (ring, scan) = pick(&points, 0.125 * s, &[0, 1, 2], &no_edges(3), 0, &[]);
    assert_eq!(scan, Some(2));
    assert_eq!(ring, Some(2));
}

/// Width 1, walk from (0.9, 0.5, 0.5): an unused member in the home cell
/// is 0.8 away, one across the cell wall (ring 1) is 0.7 away. Ring 1 must
/// be searched before any stop.
#[test]
fn ring_search_never_stops_before_ring_two() {
    let points = [ip(0.9, 0.5, 0.5), ip(0.1, 0.5, 0.5), ip(1.6, 0.5, 0.5)];
    let (ring, scan) = pick(&points, 1.0, &[0, 1, 2], &no_edges(3), 0, &[]);
    assert_eq!(scan, Some(2));
    assert_eq!(ring, Some(2));
}

/// Out-of-component points and used members are skipped even when they
/// are the nearest, and distances use all three axes: the x-offset member
/// (squared 1.5625) beats the z-offset one (squared 2.25).
#[test]
fn ring_search_skips_foreign_and_used_points_and_measures_z() {
    let points = [
        ip(-3.5, 2.5, 0.5),  // 0: current
        ip(-3.5, 2.5, 1.0),  // 1: nearest, not in the component
        ip(-3.0, 2.5, 0.5),  // 2: nearest member, already used
        ip(-2.25, 2.5, 0.5), // 3: unused member, 1.25 along x
        ip(-3.5, 2.5, 2.0),  // 4: unused member, 1.5 along z
    ];
    let comp = [0, 2, 3, 4];
    let (ring, scan) = pick(&points, 1.0, &comp, &no_edges(5), 0, &[2]);
    assert_eq!(scan, Some(3));
    assert_eq!(ring, Some(3));
}

/// Two unused members tie at distance 2 in ring 2; the ring visits the one
/// at -x first, but the +x one has the lower component rank and wins, as
/// in the scan. Reversing the ranks reverses the answer.
#[test]
fn ring_search_breaks_ties_by_component_rank() {
    let points = [ip(0.5, 0.5, 0.5), ip(2.5, 0.5, 0.5), ip(-1.5, 0.5, 0.5)];
    let edges = no_edges(3);
    let (ring, scan) = pick(&points, 1.0, &[0, 1, 2], &edges, 0, &[]);
    assert_eq!((ring, scan), (Some(1), Some(1)));
    let (ring, scan) = pick(&points, 1.0, &[0, 2, 1], &edges, 0, &[]);
    assert_eq!((ring, scan), (Some(2), Some(2)));
}

/// A farther member with a lower rank, found after the incumbent, must not
/// displace it: the rank only breaks exact ties.
#[test]
fn ring_search_rank_never_beats_distance() {
    let points = [
        ip(0.5, 0.5, 0.5),  // 0: current
        ip(-1.5, 0.5, 0.5), // 1: nearest (2.0), visited first
        ip(2.75, 0.5, 0.5), // 2: farther (2.25), lower rank, visited later
    ];
    let (ring, scan) = pick(&points, 1.0, &[0, 2, 1], &no_edges(3), 0, &[]);
    assert_eq!((ring, scan), (Some(1), Some(1)));
}

/// Tier one (unused adjacency members) keeps the same rules: nearest by
/// the full 3D distance, exact ties to the lower rank, regardless of the
/// adjacency list's index order.
#[test]
fn adjacency_tier_measures_z_and_breaks_ties_by_rank() {
    // 1 is 0.3 along x (squared 0.09); 2 is 0.25 along z (squared 0.0625).
    let points = [ip(2.5, 1.5, 0.5), ip(2.8, 1.5, 0.5), ip(2.5, 1.5, 0.25)];
    let mut adj = no_edges(3);
    adj[0] = vec![1, 2];
    let (ring, scan) = pick(&points, 1.0, &[0, 1, 2], &adj, 0, &[]);
    assert_eq!((ring, scan), (Some(2), Some(2)));
    // Exact tie at 0.25 along +z and -z: rank decides, either order.
    let points = [ip(2.5, 1.5, 0.5), ip(2.5, 1.5, 0.75), ip(2.5, 1.5, 0.25)];
    let (ring, scan) = pick(&points, 1.0, &[0, 2, 1], &adj, 0, &[]);
    assert_eq!((ring, scan), (Some(2), Some(2)));
    let (ring, scan) = pick(&points, 1.0, &[0, 1, 2], &adj, 0, &[]);
    assert_eq!((ring, scan), (Some(1), Some(1)));
    // A used neighbour is skipped even though it is the nearest.
    let (ring, scan) = pick(&points, 1.0, &[0, 1, 2], &adj, 0, &[1]);
    assert_eq!((ring, scan), (Some(2), Some(2)));
}

/// The ring walk terminates: on a ring-heavy input it finishes far inside
/// a generous deadline. A ring counter that never advances (or runs
/// backwards) spins forever; the deadline turns that into a failure
/// instead of a hang.
#[test]
fn ring_walk_terminates_within_a_deadline() {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let points = [
            ip(0.25, 0.25, 0.25),
            ip(2.75, 2.75, 2.75),
            ip(4.55, 0.25, 0.25),
        ];
        let got = pick(&points, 0.5, &[0, 1, 2], &no_edges(3), 0, &[]);
        let _ = tx.send(got);
    });
    let got = rx
        .recv_timeout(std::time::Duration::from_secs(30))
        .expect("ring walk did not finish within 30 s");
    assert_eq!(got, (Some(2), Some(2)));
}

/// Regression (found triaging B19 survivor `chaining.rs` 245:12): a
/// threshold whose square overflows connects a pair only when the pair's
/// squared distance stays finite, exactly as the all-pairs `d^2 < t^2`
/// comparison does. Two finite points 1e200 apart have an infinite
/// squared distance, so they stay apart at every such threshold. The
/// clique shortcut used to join them, and the walk's scan (which can never
/// select an infinitely distant point) then dropped the far point from the
/// output altogether.
#[test]
fn overflowing_threshold_keeps_overflowing_pairs_apart() {
    let points = [ip(0.0, 0.0, 0.0), ip(1e200, 0.0, 0.0), ip(1.0, 0.0, 0.0)];
    // 2^1023 * 0.9 takes the cell path; 1.5e308 and infinity take the
    // overflow shortcut. All three must agree with the comparison.
    for threshold in [0.9 * 2.0f64.powi(1023), 1.5e308, f64::INFINITY] {
        let chains = chain_intersection_points(&points, threshold);
        let sizes: Vec<usize> = chains.iter().map(Vec::len).collect();
        assert_eq!(sizes, vec![2, 1], "threshold {threshold:e}");
        assert_eq!(chains[1][0].point.x(), 1e200, "threshold {threshold:e}");
    }
}

// ---------------------------------------------------------------------------
// chain_intersection_points: adjacency on both the scan and the grid path
// ---------------------------------------------------------------------------

/// Six points at threshold 3 (squared 9): a pair 4 apart along z (squared
/// 16, apart), a pair exactly 3 apart (squared 9, apart: the comparison is
/// strict), and a pair 2.6 apart (squared 6.76, joined).
fn threshold_cases() -> Vec<IntersectionPoint> {
    vec![
        ip(0.0, 0.0, 0.0),
        ip(0.0, 0.0, 4.0),
        ip(10.0, 0.0, 0.0),
        ip(13.0, 0.0, 0.0),
        ip(20.0, 0.0, 0.0),
        ip(22.6, 0.0, 0.0),
    ]
}

fn chain_xs(chains: &[Vec<IntersectionPoint>]) -> Vec<Vec<(f64, f64)>> {
    chains
        .iter()
        .map(|c| c.iter().map(|q| (q.point.x(), q.point.z())).collect())
        .collect()
}

#[test]
fn scan_path_joins_exactly_the_pairs_under_the_threshold() {
    let chains = chain_intersection_points(&threshold_cases(), 3.0);
    assert_eq!(
        chain_xs(&chains),
        vec![
            vec![(0.0, 0.0)],
            vec![(0.0, 4.0)],
            vec![(10.0, 0.0)],
            vec![(13.0, 0.0)],
            vec![(20.0, 0.0), (22.6, 0.0)],
        ]
    );
}

/// The same six points plus 250 isolated fillers take the grid path (256
/// points) and must join exactly the same pairs.
#[test]
fn grid_path_joins_exactly_the_pairs_under_the_threshold() {
    let mut points = threshold_cases();
    for k in 1..=250 {
        points.push(ip(0.0, 100.0 * f64::from(k), 0.0));
    }
    assert_eq!(points.len(), GRID_THRESHOLD);
    let chains = chain_intersection_points(&points, 3.0);
    assert_eq!(chains.len(), 255);
    let xs = chain_xs(&chains);
    assert_eq!(
        xs[..5],
        [
            vec![(0.0, 0.0)],
            vec![(0.0, 4.0)],
            vec![(10.0, 0.0)],
            vec![(13.0, 0.0)],
            vec![(20.0, 0.0), (22.6, 0.0)],
        ]
    );
    assert!(chains[5..].iter().all(|c| c.len() == 1));
}

/// A threshold a few ulps above 2^100 has a `log2` that rounds down to
/// 100, so the cell width must be doubled to 2^101. A pair 2^100 (1 +
/// 2^-49) apart straddling the cell boundary at 2^101 lies two 2^100-wide
/// cells apart; it is within the threshold and must still be joined.
#[test]
fn grid_width_covers_a_threshold_just_above_a_power_of_two() {
    let unit = 2.0f64.powi(100);
    let threshold = unit * (1.0 + 2.0f64.powi(-48));
    let mut points = vec![
        ip(unit * (1.0 - 2.0f64.powi(-49)), 0.0, 0.0),
        ip(2.0 * unit, 0.0, 0.0),
    ];
    for k in 1..=254 {
        points.push(ip(0.0, 8.0 * unit * f64::from(k), 0.0));
    }
    let chains = chain_intersection_points(&points, threshold);
    assert_eq!(chains.len(), 255);
    assert_eq!(chains[0].len(), 2);
    assert!(chains[1..].iter().all(|c| c.len() == 1));
}

/// A subnormal threshold (cell width from the subnormal exponent branch)
/// squares to zero, so no pair connects, even two coincident points.
#[test]
fn subnormal_threshold_connects_nothing() {
    let points = [ip(0.0, 0.0, 0.0), ip(0.0, 0.0, 0.0), ip(1e-311, 0.0, 0.0)];
    let chains = chain_intersection_points(&points, 1e-310);
    assert_eq!(chains.len(), 3);
    assert!(chains.iter().all(|c| c.len() == 1));
}
