//! B12 annular-Coons cell: certified rectangular iso-holes on n-sided
//! (Coons-carrier) non-planar sweep/pipe caps.
//!
//! Roadmap row B12 (`docs/kernel-maturity/roadmap.md`) names two candidate
//! designs against the extruded-annulus ground truth (outer circle minus
//! inner circle profile, closed-form volume and area): an annular Coons
//! patch, or cap-then-subtract. The comparison was measured during
//! development with uncommitted scratch probes and decided for the single
//! trimmed face on the cap carrier:
//!
//! - Ground truth (this file, `ground_truth_extruded_annulus`): the planar
//!   circular annulus extruded straight is 4 faces (2 planar annulus caps +
//!   2 cylinders), validate-clean, watertight, volume and area exact to
//!   ~1e-16 at 1e-3/1/1e3.
//! - Cap-then-subtract (hole-free saddle sweep, then exact-only boolean cut
//!   of a box/cylinder tool through the hole footprint): REFUSED by the
//!   exact-only policy — the NURBS cap carrier has no exact boolean arm
//!   against the tool, and touching `algo` to add one is out of scope. The
//!   disclosed mesh fallback costs analytic exactness (it measured 16
//!   all-planar faces at 72.000000000000 on the saddle case) and would route
//!   an exact op through approximation, contradicting B21 — so it loses on
//!   exact-or-typed despite fewer faces.
//! - Annular Coons (this file, `sweep`/`pipe` legs): one trimmed NURBS cap
//!   per end — 2 holed faces, exact carrier, plane walls — validate-clean,
//!   watertight, volume within 1e-6 of the Cavalieri closed form
//!   `L * (A_outer - A_hole)` at all three scales.
//!
//! A TRUE circular trim on a bilinear/Coons carrier stays a typed refusal
//! (`curved_hole_stays_typed_refusal` pins it): a planar section of a
//! genuinely bilinear patch is a hyperbola or a ruling line, never a
//! circle, so no exact circular trim exists to certify — and the boolean
//! route refuses exact for the same carrier. The refusal rolls back without
//! emitting a solid.
//!
//! Fixture doubles as the regression proof for the `cap.rs` gate change
//! (n-sided holed rings route to the Coons carrier instead of refusing):
//! on the pre-fix tree the sweep leg fails at `build_cap_face` with
//! "non-planar cap holes currently require a four-sided outer ring".

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::{Point3, Vec3};
use remus_operations::extrude::extrude;
use remus_operations::nonplanar_ring_surface;
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_polygon_wire};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const PATH_LEN: f64 = 6.0;
/// D2-symmetric hexagon outer boundary (180-degree rotation symmetry keeps
/// the section Newell normal exactly +Z, so straight-sweep rings are pure
/// translates and the Cavalieri oracle below is exact).
const OUTER_XY: [[f64; 2]; 6] = [
    [-2.0, -1.0],
    [0.0, -2.0],
    [2.0, -1.0],
    [2.0, 1.0],
    [0.0, 2.0],
    [-2.0, 1.0],
];
/// Hole rectangle in carrier (u, v): inside one knot span in each direction
/// (the hexagon Coons carrier is 3x3-spanned with interior knots at 0.5 in
/// both directions: u-spans [0,0.5],[0.5,1], v-spans likewise), strictly
/// interior, non-degenerate. Iso-edges inside a single span are straight
/// carrier lines, which is what the certifier verifies; a hole edge
/// straddling a span boundary kinks with the carrier and leaves it.
const HOLE_U: (f64, f64) = (0.55, 0.8);
const HOLE_V: (f64, f64) = (0.55, 0.8);

fn saddle_z(x: f64, y: f64) -> f64 {
    0.1 * x * y
}

fn shoelace(pts: &[[f64; 2]]) -> f64 {
    // Close the loop explicitly; |area| so winding never matters.
    let sum: f64 = pts
        .iter()
        .zip(pts.iter().cycle().skip(1))
        .map(|(w, v)| w[0] * v[1] - v[0] * w[1])
        .sum();
    (sum / 2.0).abs()
}

fn outer_area() -> f64 {
    shoelace(&OUTER_XY)
}

/// Rightmost outer-boundary x at height `y` (unit coordinates), by scanning
/// the outer polygon edges. Locates the band probe robustly no matter where
/// the carrier maps the hole.
fn outer_max_x_at(y: f64) -> f64 {
    let mut best = f64::NEG_INFINITY;
    for i in 0..OUTER_XY.len() {
        let [x0, y0] = OUTER_XY[i];
        let [x1, y1] = OUTER_XY[(i + 1) % OUTER_XY.len()];
        if (y - y0) * (y - y1) <= 0.0 && (y1 - y0).abs() > 1e-12 {
            let t = (y - y0) / (y1 - y0);
            best = best.max(x0 + t * (x1 - x0));
        }
    }
    best
}

fn straight_path(len: f64) -> remus_math::nurbs::curve::NurbsCurve {
    remus_math::nurbs::curve::NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, len)],
        vec![1.0, 1.0],
    )
    .unwrap()
}

/// Hexagon-outer / iso-rect-hole saddle profile at `scale` (uniform scaling
/// about the origin keeps the shape similar at every scale). Returns the
/// profile face plus the hole loop's (x, y) polygon for the area oracle —
/// read back from the built wire vertices (B-Rep facts), since the loop
/// sits at carrier-evaluated positions by construction.
fn hexagon_profile(
    topo: &mut Topology,
    scale: f64,
) -> (remus_topology::face::FaceId, Vec<[f64; 2]>) {
    let corners: Vec<Point3> = OUTER_XY
        .iter()
        .map(|&[x, y]| Point3::new(x * scale, y * scale, saddle_z(x, y) * scale))
        .collect();
    let carrier = nonplanar_ring_surface(&corners).unwrap();
    // Wound opposite the outer boundary per B-Rep convention (mirroring the
    // saddle-annulus precedent): inner loops oppose the outer boundary.
    let hole_pts: Vec<Point3> = [
        (HOLE_U.0, HOLE_V.0),
        (HOLE_U.0, HOLE_V.1),
        (HOLE_U.1, HOLE_V.1),
        (HOLE_U.1, HOLE_V.0),
    ]
    .iter()
    .map(|&(u, v)| carrier.evaluate(u, v))
    .collect();
    let hole_xy: Vec<[f64; 2]> = hole_pts.iter().map(|p| [p.x(), p.y()]).collect();
    let outer = make_polygon_wire(topo, &corners, 1e-7).unwrap();
    let hole = make_polygon_wire(topo, &hole_pts, 1e-7).unwrap();
    let face = topo.add_face(Face::new(outer, vec![hole], FaceSurface::Nurbs(carrier)));
    (face, hole_xy)
}

/// Closed-form volume for a straight sweep: every interior cross-section
/// perpendicular to the path is a rigid translate of the profile boundary
/// (rotation-minimizing frames along a straight path differ by translation
/// only, up to an in-plane twist that preserves areas), so the volume is
/// path length times enclosed chord-polygon area, outer minus hole.
fn straight_sweep_volume(len: f64, outer_area: f64, hole_area: f64) -> f64 {
    len * (outer_area - hole_area)
}

fn surface_census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for fid in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(fid).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

fn edge_use_counts(topo: &Topology, solid: SolidId) -> BTreeMap<usize, usize> {
    let mut counts = BTreeMap::new();
    for fid in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *counts.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    counts
}

#[allow(clippy::too_many_lines)]
fn assert_annular_solid(
    topo: &Topology,
    solid: SolidId,
    expected_volume: f64,
    scale: f64,
    label: &str,
) {
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let holed_nurbs_caps = faces
        .iter()
        .filter(|&&fid| {
            let face = topo.face(fid).unwrap();
            matches!(face.surface(), FaceSurface::Nurbs(_)) && face.inner_wires().len() == 1
        })
        .count();
    assert_eq!(
        holed_nurbs_caps, 2,
        "{label} at scale {scale:e}: both caps must be single trimmed holed faces"
    );
    // Fallback tell (solid-verification rung 1): a mesh fallback would be
    // hundreds of all-planar faces, not tens of analytic ones.
    assert!(
        faces.len() < 100,
        "{label} at scale {scale:e}: {} faces looks like a mesh fallback",
        faces.len()
    );

    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert_eq!(
        report.error_count(),
        0,
        "{label} at scale {scale:e}: invalid solid: {report:?}"
    );
    assert!(
        edge_use_counts(topo, solid).values().all(|&c| c == 2),
        "{label} at scale {scale:e}: every edge must have exactly two face uses"
    );

    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, 0.01 * scale).unwrap();
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(
        quality.is_watertight(),
        "{label} at scale {scale:e}: mesh has {} boundary and {} non-manifold edges",
        quality.boundary_edges,
        quality.non_manifold_edges
    );

    // Volume at two deflections finer than the tessellation clamp, plus the
    // closed form (solid-verification rung 6). Deflections scale with the
    // model (mirroring the saddle-annulus precedent at scale 1); an
    // absolute fine deflection would explode the tessellation grid at 1e-3
    // scale, while coarse requests are clamped to the same value and would
    // agree trivially instead of proving convergence.
    let coarse = remus_operations::measure::solid_volume(topo, solid, scale * 2e-4).unwrap();
    let fine = remus_operations::measure::solid_volume(topo, solid, scale * 1e-4).unwrap();
    assert!(
        (coarse - fine).abs() / expected_volume < 1e-9,
        "{label} at scale {scale:e}: volume must converge across deflections ({coarse} vs {fine})"
    );
    for (name, volume) in [("coarse", coarse), ("fine", fine)] {
        assert!(
            (volume - expected_volume).abs() / expected_volume < 1e-6,
            "{label} at scale {scale:e}: {name} volume {volume} vs closed form {expected_volume}"
        );
    }
}

fn assert_annular_classification(
    topo: &Topology,
    solid: SolidId,
    hole_xy: &[[f64; 2]],
    len: f64,
    label: &str,
) {
    let options = ClassifyOptions::default();
    let mid_x: f64 = hole_xy.iter().map(|p| p[0]).sum::<f64>() / hole_xy.len() as f64;
    let mid_y: f64 = hole_xy.iter().map(|p| p[1]).sum::<f64>() / hole_xy.len() as f64;
    let hole_probe = Point3::new(mid_x, mid_y, len / 2.0);
    assert_eq!(
        classify_point(topo, solid, hole_probe, &options).unwrap(),
        PointClassification::Outside,
        "{label}: the annular opening must remain void"
    );
    // Band probe midway between the hole's right side and the outer wall at
    // the hole's mid-height; convexity keeps the segment inside material.
    // (Outer x from the input polygon, hole x read back: the probe stays
    // strictly between the two regardless of carrier mapping.)
    let hole_max_x: f64 = hole_xy
        .iter()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max);
    let unit_mid_y = mid_y / (len / PATH_LEN);
    let outer_x = outer_max_x_at(unit_mid_y) * (len / PATH_LEN);
    assert!(
        outer_x.is_finite() && outer_x > hole_max_x,
        "{label}: band probe has no material span (hole_max_x={hole_max_x}, outer_x={outer_x})"
    );
    let band_probe = Point3::new(f64::midpoint(hole_max_x, outer_x), mid_y, len / 2.0);
    assert_eq!(
        classify_point(topo, solid, band_probe, &options).unwrap(),
        PointClassification::Inside,
        "{label}: the annular band must remain material"
    );
}

#[test]
fn ground_truth_extruded_annulus() {
    // The row's named oracle: outer circle minus inner circle, extruded.
    // Exact analytics: 2 planar annulus caps + inner/outer cylinders.
    for scale in SCALES {
        let (r_out, r_in, len) = (2.0 * scale, 1.0 * scale, PATH_LEN * scale);
        let expected_volume = PI * (r_out * r_out - r_in * r_in) * len;
        let expected_area =
            2.0 * PI * (r_out * r_out - r_in * r_in) + 2.0 * PI * (r_out + r_in) * len;
        let mut topo = Topology::new();
        let tol = 1e-7;
        let outer = make_circle_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            r_out,
            tol,
        )
        .unwrap();
        let inner = make_circle_edge(
            &mut topo,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            r_in,
            tol,
        )
        .unwrap();
        let ow = topo.add_wire(Wire::new(vec![OrientedEdge::new(outer, true)], true).unwrap());
        let iw = topo.add_wire(Wire::new(vec![OrientedEdge::new(inner, false)], true).unwrap());
        let profile = topo.add_face(Face::new(
            ow,
            vec![iw],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ));
        let solid = extrude(&mut topo, profile, Vec3::new(0.0, 0.0, 1.0), len).unwrap();
        let census = surface_census(&topo, solid);
        assert_eq!(
            census.get("plane"),
            Some(&2),
            "ground truth caps must stay planar at scale {scale:e}: {census:?}"
        );
        assert_eq!(
            census.get("cylinder"),
            Some(&2),
            "ground truth walls must stay exact cylinders at scale {scale:e}: {census:?}"
        );
        let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
        assert_eq!(report.error_count(), 0, "ground truth invalid: {report:?}");
        let mesh =
            remus_operations::tessellate::tessellate_solid(&topo, solid, 0.01 * scale).unwrap();
        assert!(
            remus_operations::tessellate::welded_mesh_quality(&mesh).is_watertight(),
            "ground truth mesh must be watertight at scale {scale:e}"
        );
        let volume = remus_operations::measure::solid_volume(&topo, solid, scale * 1e-4).unwrap();
        assert!(
            (volume - expected_volume).abs() / expected_volume < 1e-9,
            "ground truth volume {volume} vs closed form {expected_volume} at scale {scale:e}"
        );
        let area =
            remus_operations::measure::solid_surface_area(&topo, solid, scale * 1e-4).unwrap();
        assert!(
            (area - expected_area).abs() / expected_area < 1e-6,
            "ground truth area {area} vs closed form {expected_area} at scale {scale:e}"
        );
    }
}

#[test]
fn sweep_annular_coons_cap_matches_ground_truth_shape() {
    for scale in SCALES {
        let mut topo = Topology::new();
        let (profile, hole_xy) = hexagon_profile(&mut topo, scale);
        let len = PATH_LEN * scale;
        let solid =
            remus_operations::sweep::sweep(&mut topo, profile, &straight_path(len)).unwrap();
        let expected = straight_sweep_volume(len, outer_area() * scale * scale, shoelace(&hole_xy));
        assert_annular_solid(&topo, solid, expected, scale, "sweep");
        assert_annular_classification(&topo, solid, &hole_xy, len, "sweep");
        // Deterministic rebuild: same inputs must give the same volume bits.
        let mut topo2 = Topology::new();
        let (profile2, hole_xy2) = hexagon_profile(&mut topo2, scale);
        let solid2 =
            remus_operations::sweep::sweep(&mut topo2, profile2, &straight_path(len)).unwrap();
        let expected2 =
            straight_sweep_volume(len, outer_area() * scale * scale, shoelace(&hole_xy2));
        let v1 = remus_operations::measure::solid_volume(&topo, solid, scale * 1e-4).unwrap();
        let v2 = remus_operations::measure::solid_volume(&topo2, solid2, scale * 1e-4).unwrap();
        assert_eq!(
            expected.to_bits(),
            expected2.to_bits(),
            "oracle must rebuild identically"
        );
        assert_eq!(
            v1.to_bits(),
            v2.to_bits(),
            "sweep rebuild must be bit-stable at scale {scale:e}"
        );
    }
}

#[test]
fn pipe_annular_coons_cap_matches_ground_truth_shape() {
    for scale in SCALES {
        let mut topo = Topology::new();
        let (profile, hole_xy) = hexagon_profile(&mut topo, scale);
        let len = PATH_LEN * scale;
        let solid =
            remus_operations::pipe::pipe(&mut topo, profile, &straight_path(len), None).unwrap();
        let expected = straight_sweep_volume(len, outer_area() * scale * scale, shoelace(&hole_xy));
        assert_annular_solid(&topo, solid, expected, scale, "pipe");
        assert_annular_classification(&topo, solid, &hole_xy, len, "pipe");
    }
}

#[test]
fn curved_hole_stays_typed_refusal() {
    // A true circular trim has no exact carrier on a genuinely bilinear cap
    // (a planar section of one is a hyperbola or a ruling line, never a
    // circle), so the cap certifier must refuse typed — and roll back
    // without emitting a solid.
    let mut topo = Topology::new();
    let saddle = |x: f64, y: f64| Point3::new(x, y, saddle_z(x, y));

    let outer_pts = [
        saddle(-2.0, -2.0),
        saddle(2.0, -2.0),
        saddle(2.0, 2.0),
        saddle(-2.0, 2.0),
    ];
    let outer = make_polygon_wire(&mut topo, &outer_pts, 1e-7).unwrap();
    let circ = make_circle_edge(
        &mut topo,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        1.0,
        1e-7,
    )
    .unwrap();
    let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(circ, false)], true).unwrap());
    let carrier = nonplanar_ring_surface(&outer_pts).unwrap();
    let profile = topo.add_face(Face::new(outer, vec![hole], FaceSurface::Nurbs(carrier)));
    let solids_before = topo.num_solids();
    let err = remus_operations::sweep::sweep(&mut topo, profile, &straight_path(6.0)).unwrap_err();
    assert!(
        matches!(err, remus_operations::OperationsError::InvalidInput { .. }),
        "curved hole must refuse typed, got {err:?}"
    );
    assert_eq!(
        topo.num_solids(),
        solids_before,
        "a refused cap must not emit a solid"
    );
}
