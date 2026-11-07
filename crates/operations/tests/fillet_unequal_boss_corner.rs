//! Unequal concave seam / shoulder corners on the Gentle-Gecko part (spec
//! step A2, concave half; video step 6: a vertical seam R5 -> R2.6 while the
//! shoulder and the top rounds stay).
//!
//! Gentle-Gecko is a 30 x 18 x 24 block with an R6 boss (axis +Z, height 36)
//! on its x = 0, y = 0 corner edge. Its filleted form rounds the two
//! vertical boss seams, the shoulder round the boss on the block top and
//! the two convex top edges at R4. Where a seam, the shoulder and a top
//! round meet, a ball rolls on the boss over the top round from the seam to
//! the shoulder. With a seam of a different radius than the shoulder the
//! ball's radius runs from one to the other along a Hermite ramp, and the
//! corner patch is the envelope of that sphere family (`remus_blend`'s boss
//! corner module): tangent to the boss and the round along its contacts,
//! and meeting the seam cylinder and the shoulder torus along their own
//! sections. Before, neither stage order built it (the W3 spike): the seam
//! and shoulder now go in one concave call, each run at its own radius,
//! mitring where they meet; the convex rounds then close the corner.
//!
//! Every result meets the solid-verification bar: both validators, zero
//! mesh boundary and non-manifold edges at two deflections, API volume
//! within 1e-4 of an independent mesh volume, classification probes on
//! both sides of every corner patch, and an exact census (only the corner
//! patches are sampled). Each corner patch's boundaries are measured for G1
//! continuity against the face across them. Group edits match a fresh
//! construction from the sharp part in the planner's stage order.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_cluster::{Convexity, recognize_blend_cluster};
use remus_operations::blend_ops::{fillet_cascade, fillet_cascade_radii};
use remus_operations::measure::solid_volume;
use remus_operations::resize_blend::{resize_blend, resize_blend_failure_code};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

// ---------------------------------------------------------------------------
// Fixtures.

fn import(text: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(text, &mut topo).unwrap()[0];
    (topo, solid)
}

fn gecko() -> (Topology, SolidId) {
    import(include_str!("data/gentle_gecko_filleted.step"))
}

fn gecko_sharp() -> (Topology, SolidId) {
    import(include_str!("data/gentle_gecko_sharp.step"))
}

/// The block face a seam runs up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    /// The y = 0 face (its top edge runs along X).
    Y0,
    /// The x = 0 face (its top edge runs along Y).
    X0,
}

impl Side {
    const BOTH: [Self; 2] = [Self::Y0, Self::X0];

    /// A point `out` outside the block face and `along` the face from the
    /// boss axis, at height `z`.
    fn point(self, along: f64, out: f64, z: f64) -> Point3 {
        match self {
            Self::Y0 => Point3::new(along, -out, z),
            Self::X0 => Point3::new(-out, along, z),
        }
    }

    /// Whether `p` is on this side's block face plane.
    fn on_face(self, p: Point3) -> bool {
        match self {
            Self::Y0 => p.y().abs() < 1e-7,
            Self::X0 => p.x().abs() < 1e-7,
        }
    }
}

fn edge_ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

/// The sharp part's seam up `side`'s block face.
fn sharp_seam(topo: &Topology, solid: SolidId, side: Side) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let (a, b) = edge_ends(topo, edge);
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line)
                && side.on_face(a)
                && side.on_face(b)
                && (a.x().hypot(a.y()) - 6.0).abs() < 1e-7
                && (b.x().hypot(b.y()) - 6.0).abs() < 1e-7
                && a.z().min(b.z()) < 1e-9
        })
        .collect();
    assert_eq!(found.len(), 1, "one sharp seam on {side:?}");
    found[0]
}

/// The sharp part's shoulder arc.
fn sharp_shoulder(topo: &Topology, solid: SolidId) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            matches!(topo.edge(edge).unwrap().curve(),
                EdgeCurve::Circle(c) if (c.center().z() - 24.0).abs() < 1e-7)
        })
        .collect();
    assert_eq!(found.len(), 1, "one sharp shoulder arc");
    found[0]
}

/// The straight convex top edges left on a body whose concave run is built.
fn top_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let (a, b) = edge_ends(topo, edge);
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line)
                && (a.z() - 24.0).abs() < 1e-7
                && (b.z() - 24.0).abs() < 1e-7
                && Side::BOTH.iter().any(|s| s.on_face(a) && s.on_face(b))
        })
        .collect();
    assert_eq!(found.len(), 2, "two convex top edges");
    found
}

/// The boss cap's rim circle(s).
fn cap_rim(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            matches!(topo.edge(edge).unwrap().curve(),
                EdgeCurve::Circle(c) if (c.center().z() - 36.0).abs() < 1e-7)
        })
        .collect()
}

/// The sharp Gecko with its seams at `seams` (y = 0 face, x = 0 face) under
/// an R4 shoulder in one concave call, then both top edges at `round`, then
/// (optionally) an R1 cap rim.
fn build_from_sharp(seams: [f64; 2], round: f64, rim: bool) -> (Topology, SolidId) {
    let (mut topo, sharp) = gecko_sharp();
    let sets = vec![
        (vec![sharp_seam(&topo, sharp, Side::Y0)], seams[0]),
        (vec![sharp_shoulder(&topo, sharp)], 4.0),
        (vec![sharp_seam(&topo, sharp, Side::X0)], seams[1]),
    ];
    let concave = fillet_cascade_radii(&mut topo, sharp, &sets).unwrap().solid;
    let tops = top_edges(&topo, concave);
    let mut solid = fillet_cascade(&mut topo, concave, &tops, round)
        .unwrap()
        .solid;
    if rim {
        let rim = cap_rim(&topo, solid);
        solid = fillet_cascade(&mut topo, solid, &rim, 1.0).unwrap().solid;
    }
    (topo, solid)
}

// ---------------------------------------------------------------------------
// Verification.

fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let origin = mesh.positions[0];
    mesh.indices
        .chunks(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize] - origin;
            let b = mesh.positions[tri[1] as usize] - origin;
            let c = mesh.positions[tri[2] as usize] - origin;
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

fn bbox_diagonal(topo: &Topology, solid: SolidId) -> f64 {
    let mesh = tessellate_solid(topo, solid, 0.1).unwrap();
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in &mesh.positions {
        for (axis, value) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
            lo[axis] = lo[axis].min(value);
            hi[axis] = hi[axis].max(value);
        }
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

/// Both validators clean, a watertight manifold mesh at two deflections at
/// or below `bbox_diag * 5e-5`, and the API volume within 1e-4 of the
/// independent mesh volume at each. Returns the API volume.
fn verify(topo: &Topology, solid: SolidId, label: &str) -> f64 {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(report.is_valid(), "{label}: {:?}", report.issues);
    let strict = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(strict.is_valid(), "{label}: {:?}", strict.issues);
    let diagonal = bbox_diagonal(topo, solid);
    let volume = solid_volume(topo, solid, diagonal * 1e-5).unwrap();
    for deflection in [diagonal * 5e-5, diagonal * 2e-5] {
        let mesh = tessellate_solid(topo, solid, deflection).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label} at {deflection}");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "{label} at {deflection}");
        let independent = mesh_volume(&mesh);
        assert!(
            (independent - volume).abs() <= 1e-4 * volume,
            "{label}: mesh volume {independent} vs {volume} at {deflection}"
        );
    }
    volume
}

/// Face census by exact carrier: `"plane"`, `"cylinder R"`, `"torus M/m"`,
/// and `"sampled"` for everything else.
fn census(topo: &Topology, solid: SolidId) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    for face in solid_faces(topo, solid).unwrap() {
        let key = match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => "plane".to_owned(),
            FaceSurface::Cylinder(c) => format!("cylinder {:.3}", c.radius()),
            FaceSurface::Torus(t) => {
                format!("torus {:.3}/{:.3}", t.major_radius(), t.minor_radius())
            }
            _ => "sampled".to_owned(),
        };
        *counts.entry(key).or_insert(0) += 1;
    }
    counts
}

/// The census of a Gecko with its seams at `seams`, both top rounds at
/// `round` and an R1 cap rim when `rim`: everything exact but the two
/// corner patches.
fn expected_census(seams: [f64; 2], round: f64, rim: bool) -> BTreeMap<String, usize> {
    let mut counts = BTreeMap::new();
    counts.insert("plane".to_owned(), 7);
    counts.insert("sampled".to_owned(), 2);
    counts.insert("torus 10.000/4.000".to_owned(), 1);
    if rim {
        counts.insert("torus 5.000/1.000".to_owned(), 1);
    }
    for radius in [seams[0], seams[1], round, round, 6.0] {
        *counts.entry(format!("cylinder {radius:.3}")).or_insert(0) += 1;
    }
    counts
}

fn classify(topo: &Topology, solid: SolidId, point: Point3) -> PointClassification {
    classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap()
}

/// The sampled faces: the corner patches.
fn corner_patches(topo: &Topology, solid: SolidId) -> Vec<FaceId> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
        .collect()
}

/// Outward unit normal of `face` at the projection of `point`, and how far
/// `point` lies off its surface.
fn outward(topo: &Topology, face: FaceId, point: Point3) -> (Vec3, f64) {
    let data = topo.face(face).unwrap();
    let surface = data.surface();
    if let FaceSurface::Plane { normal, d } = surface {
        let off = (normal.dot(point - Point3::new(0.0, 0.0, 0.0)) - d).abs();
        return (
            if data.is_reversed() {
                -*normal
            } else {
                *normal
            },
            off,
        );
    }
    let (u, v) = surface
        .project_point(point)
        .unwrap_or_else(|| panic!("{point:?} does not project onto {surface:?}"));
    let normal = surface.normal(u, v).normalize().unwrap();
    let off = (surface.evaluate(u, v).unwrap() - point).length();
    (if data.is_reversed() { -normal } else { normal }, off)
}

/// For every boundary edge of `patch`: the face across it, the largest angle
/// (degrees) between the two faces' normals and the largest distance of the
/// edge off either surface, over 63 interior stations.
fn boundary_continuity(topo: &Topology, solid: SolidId, patch: FaceId) -> Vec<(String, f64, f64)> {
    let adjacency = topo.build_adjacency(solid).unwrap();
    let wire = topo
        .wire(topo.face(patch).unwrap().outer_wire())
        .unwrap()
        .edges()
        .to_vec();
    let mut rows = Vec::new();
    for oe in wire {
        let edge = topo.edge(oe.edge()).unwrap();
        let Some(&neighbour) = adjacency
            .faces_for_edge(oe.edge())
            .iter()
            .find(|&&f| f != patch)
        else {
            continue;
        };
        let (start, end) = edge_ends(topo, oe.edge());
        let (t0, t1) = edge.strict_domain().unwrap();
        let (mut angle, mut gap) = (0.0f64, 0.0f64);
        for i in 1..64 {
            let t = (t1 - t0).mul_add(f64::from(i) / 64.0, t0);
            let q = edge.curve().evaluate_with_endpoints(t, start, end);
            let (a, off_a) = outward(topo, patch, q);
            let (b, off_b) = outward(topo, neighbour, q);
            angle = angle.max(a.cross(b).length().atan2(a.dot(b)).to_degrees());
            gap = gap.max(off_a).max(off_b);
        }
        let kind = match topo.face(neighbour).unwrap().surface() {
            FaceSurface::Plane { .. } => "plane".to_owned(),
            FaceSurface::Cylinder(c) => format!("cylinder R{}", c.radius()),
            FaceSurface::Torus(t) => format!("torus {}/{}", t.major_radius(), t.minor_radius()),
            _ => "sampled".to_owned(),
        };
        rows.push((kind, angle, gap));
    }
    rows
}

/// The boss-corner patches (`corners` of them) meet every neighbour G1
/// and lie on both surfaces along their boundaries; points just off every
/// sampled patch classify on the right side of it.
///
/// A boss corner borders the boss (split where the boss's own seam crosses
/// it), the seam cylinder, the round and the shoulder torus, and no plane.
/// A round of its own radius between an equal seam and shoulder is walked
/// into the seam band instead (`fillet_into_seam_band`): that transition
/// borders the block top and is not this module's construction.
fn assert_corners(topo: &Topology, solid: SolidId, corners: usize, label: &str) {
    let patches = corner_patches(topo, solid);
    assert_eq!(patches.len(), 2, "{label}: two sampled patches");
    let mut found = 0;
    for &patch in &patches {
        let rows = boundary_continuity(topo, solid, patch);
        if rows.iter().any(|(kind, _, _)| kind == "plane") {
            check_sides(topo, solid, patch, label);
            continue;
        }
        found += 1;
        assert!(rows.len() >= 4, "{label}: {rows:?}");
        // The patch is ruled between 513 exact sections, so its normal
        // turns in steps of about 0.09 degrees along every boundary: the
        // same as the equal-radius corner (measured 0.04-0.11 degrees and
        // 5e-6 against the very same neighbours); these corners measure
        // 0.02-0.13 degrees.
        for (kind, angle, gap) in &rows {
            assert!(*angle < 0.15, "{label}: G1 against {kind}: {angle} degrees");
            assert!(*gap < 1e-5, "{label}: boundary off {kind} by {gap}");
        }
        check_sides(topo, solid, patch, label);
    }
    assert_eq!(found, corners, "{label}: boss corners");
}

/// Points just off `patch` are outside the solid, points just behind it
/// inside.
fn check_sides(topo: &Topology, solid: SolidId, patch: FaceId, label: &str) {
    let data = topo.face(patch).unwrap();
    let reversed = data.is_reversed();
    let FaceSurface::Nurbs(nurbs) = data.surface() else {
        unreachable!()
    };
    let ((u0, u1), (v0, v1)) = (nurbs.domain_u(), nurbs.domain_v());
    for fu in [0.25, 0.5, 0.75] {
        for fv in [0.1, 0.5, 0.9] {
            let (u, v) = ((u1 - u0).mul_add(fu, u0), (v1 - v0).mul_add(fv, v0));
            let p = data.surface().evaluate(u, v).unwrap();
            let n = data.surface().normal(u, v).normalize().unwrap();
            let n = if reversed { -n } else { n };
            assert_eq!(
                classify(topo, solid, p + n * 0.05),
                PointClassification::Outside,
                "{label}: off the patch at ({fu}, {fv})"
            );
            assert_eq!(
                classify(topo, solid, p - n * 0.05),
                PointClassification::Inside,
                "{label}: behind the patch at ({fu}, {fv})"
            );
        }
    }
}

/// A point beside the wall at mid height, near the sharp seam: filled by
/// a seam of `radius` exactly when it lies outside that seam's rolling ball
/// (an R4 or R3 seam fills it, an R2 or R2.6 one does not).
fn assert_seam(topo: &Topology, solid: SolidId, side: Side, radius: f64, label: &str) {
    let (along, out) = (6.55, 0.65);
    let probe = side.point(along, out, 10.0);
    let centre = ((6.0 + radius).powi(2) - radius * radius).sqrt();
    let expected = if (along - centre).hypot(radius - out) > radius {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    };
    assert_eq!(
        classify(topo, solid, probe),
        expected,
        "{label}: {side:?} seam at R{radius}"
    );
}

fn fingerprint(topo: &Topology, solid: SolidId) -> String {
    remus_io::step::writer::write_step(topo, &[solid]).unwrap()
}

fn assert_matches_oracle(
    topo: &Topology,
    solid: SolidId,
    oracle_topo: &Topology,
    oracle: SolidId,
    label: &str,
) {
    assert_eq!(
        solid_entity_counts(topo, solid).unwrap(),
        solid_entity_counts(oracle_topo, oracle).unwrap(),
        "{label}: entity counts"
    );
    assert_eq!(
        census(topo, solid),
        census(oracle_topo, oracle),
        "{label}: census"
    );
    let volume = solid_volume(topo, solid, 1e-3).unwrap();
    let expected = solid_volume(oracle_topo, oracle, 1e-3).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "{label}: volume {volume} vs oracle {expected}"
    );
}

/// The concave seam band on `side`.
fn seam_band(topo: &Topology, solid: SolidId, side: Side) -> FaceId {
    let boss = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if (c.radius() - 6.0).abs() < 1e-9 && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .unwrap();
    let cluster = recognize_blend_cluster(topo, solid, boss).unwrap();
    let seams: Vec<FaceId> = cluster
        .members
        .iter()
        .filter(|member| member.convexity == Convexity::Concave)
        .flat_map(|member| member.faces.iter().copied())
        .filter(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(c) => match side {
                Side::Y0 => c.origin().y() < -1.0,
                Side::X0 => c.origin().x() < -1.0,
            },
            _ => false,
        })
        .collect();
    assert_eq!(seams.len(), 1, "one seam band on {side:?}");
    seams[0]
}

// ---------------------------------------------------------------------------
// Construction from the sharp part.

/// The concave run alone: each band exact at its own radius, the seams and
/// the shoulder mitred.
#[test]
fn concave_runs_of_different_radii_mitre_in_one_call() {
    for seams in [[2.0, 4.0], [2.0, 2.0], [2.6, 4.0]] {
        let label = format!("concave run {seams:?}");
        let (mut topo, sharp) = gecko_sharp();
        let sets = vec![
            (vec![sharp_seam(&topo, sharp, Side::Y0)], seams[0]),
            (vec![sharp_shoulder(&topo, sharp)], 4.0),
            (vec![sharp_seam(&topo, sharp, Side::X0)], seams[1]),
        ];
        let solid = fillet_cascade_radii(&mut topo, sharp, &sets).unwrap().solid;
        verify(&topo, solid, &label);
        let mut expected = BTreeMap::new();
        expected.insert("plane".to_owned(), 7);
        expected.insert("torus 10.000/4.000".to_owned(), 1);
        for radius in [seams[0], seams[1], 6.0] {
            *expected.entry(format!("cylinder {radius:.3}")).or_insert(0) += 1;
        }
        assert_eq!(census(&topo, solid), expected, "{label}");
        for (side, radius) in Side::BOTH.into_iter().zip(seams) {
            assert_seam(&topo, solid, side, radius, &label);
        }
    }
}

/// Seam R2 (either side, then both) and R2.6 under the R4 shoulder, with R4
/// top rounds and the R1 cap rim, and with R1.9 top rounds: every corner is
/// a G1 patch and every band stays exact.
#[test]
fn unequal_seam_and_shoulder_corners_build_from_the_sharp_part() {
    for (seams, round, rim) in [
        ([2.0, 4.0], 4.0, true),
        ([4.0, 2.0], 4.0, true),
        ([2.0, 2.0], 4.0, true),
        ([2.6, 4.0], 4.0, true),
        ([2.0, 4.0], 1.9, false),
    ] {
        let label = format!("seams {seams:?}, rounds R{round}, rim {rim}");
        let (topo, solid) = build_from_sharp(seams, round, rim);
        verify(&topo, solid, &label);
        assert_eq!(
            census(&topo, solid),
            expected_census(seams, round, rim),
            "{label}"
        );
        // Every side is a boss corner unless its seam equals the shoulder
        // under a round of its own radius.
        let corners = seams
            .iter()
            .filter(|&&seam| (seam - 4.0).abs() > 1e-9 || (round - 4.0).abs() < 1e-9)
            .count();
        assert_corners(&topo, solid, corners, &label);
        for (side, radius) in Side::BOTH.into_iter().zip(seams) {
            assert_seam(&topo, solid, side, radius, &label);
        }
        // The boss, the block and the shoulder's fill stay solid.
        for inside in [
            Point3::new(-1.0, -1.0, 30.0),
            Point3::new(15.0, 9.0, 12.0),
            Point3::new(4.0, 4.0, 24.5),
        ] {
            assert_eq!(
                classify(&topo, solid, inside),
                PointClassification::Inside,
                "{label}: {inside:?}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// The A2 concave edit through the blend group.

/// Seam R4 -> R2 on either side and R4 -> R2.6 on the filleted Gecko: the
/// group is removed to sharp and rebuilt with the seam at its new radius,
/// matching a fresh construction; the source is untouched.
#[test]
fn concave_seam_radius_edits_rebuild_the_group() {
    for (side, radius) in [(Side::Y0, 2.0), (Side::X0, 2.0), (Side::X0, 2.6)] {
        let label = format!("{side:?} seam R4 -> R{radius}");
        let (mut topo, solid) = gecko();
        let before = fingerprint(&topo, solid);
        let seam = seam_band(&topo, solid, side);
        assert_seam(&topo, solid, side, 4.0, &label);
        let result = resize_blend(&mut topo, solid, seam, 4.0, radius).unwrap();
        assert_eq!(fingerprint(&topo, solid), before, "{label}: source");
        verify(&topo, result.solid, &label);
        let seams = match side {
            Side::Y0 => [radius, 4.0],
            Side::X0 => [4.0, radius],
        };
        assert_eq!(
            census(&topo, result.solid),
            expected_census(seams, 4.0, false),
            "{label}"
        );
        assert_corners(&topo, result.solid, 2, &label);
        for (side, radius) in Side::BOTH.into_iter().zip(seams) {
            assert_seam(&topo, result.solid, side, radius, &label);
        }
        let (oracle_topo, oracle) = build_from_sharp(seams, 4.0, false);
        assert_matches_oracle(&topo, result.solid, &oracle_topo, oracle, &label);
        assert_eq!(
            result.evolution.modified.get(&seam.index()).map(Vec::len),
            Some(1),
            "{label}: {:?}",
            result.evolution
        );
        let outputs: Vec<usize> = solid_faces(&topo, result.solid)
            .unwrap()
            .into_iter()
            .map(remus_topology::arena::Id::index)
            .collect();
        assert!(
            result.evolution.accounts_for_result(outputs),
            "{label}: {:?}",
            result.evolution
        );
    }
}

/// The convex top band on `side` (its axis along the side's face).
fn convex_band(topo: &Topology, solid: SolidId, side: Side) -> FaceId {
    let boss = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if (c.radius() - 6.0).abs() < 1e-9 && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .unwrap();
    let along = match side {
        Side::Y0 => Vec3::new(1.0, 0.0, 0.0),
        Side::X0 => Vec3::new(0.0, 1.0, 0.0),
    };
    let cluster = recognize_blend_cluster(topo, solid, boss).unwrap();
    let bands: Vec<FaceId> = cluster
        .members
        .iter()
        .filter(|member| member.convexity == Convexity::Convex)
        .flat_map(|member| member.faces.iter().copied())
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(along).length() < 1e-9)
        })
        .collect();
    assert_eq!(bands.len(), 1, "one convex band along the {side:?} face");
    bands[0]
}

/// Material just under the top edge of `side`'s face, mid length: removed
/// by an R4 round, kept by an R1.9 or R1.6 one.
fn assert_round(topo: &Topology, solid: SolidId, side: Side, radius: f64, label: &str) {
    let probe = match side {
        Side::Y0 => Point3::new(20.0, 0.5, 22.5),
        Side::X0 => Point3::new(0.5, 14.0, 22.5),
    };
    let expected = if radius > 3.0 {
        PointClassification::Outside
    } else {
        PointClassification::Inside
    };
    assert_eq!(
        classify(topo, solid, probe),
        expected,
        "{label}: {side:?} round at R{radius}"
    );
}

/// One edit of a sequence: its label, the face it picks, the band radius
/// from and to, then the seams and rounds (y = 0, x = 0 faces) after it.
type Step = (
    &'static str,
    fn(&Topology, SolidId) -> FaceId,
    f64,
    f64,
    [f64; 2],
    [f64; 2],
);

/// The video's A2 edits in order on one body, each through the group:
/// step 4, the near top round R4 -> 1.9 (here the x = 0 face's); step 6,
/// the opposite seam R4 -> 2.6; step 7, the opposite top round R4 -> 1.6,
/// whose corner then rides an R1.6 round from an R2.6 seam to the R4
/// shoulder; and the seam once more, R2.6 -> 3. Each corner patch with
/// unequal radii is recognized as the group's corner, removed and rebuilt.
/// Over the R1.6 round an R2 seam has no G1 corner (the growing ball's
/// envelope folds): that edit is a typed refusal that leaves the body
/// alone.
#[test]
fn video_a2_edits_rebuild_through_the_group_in_sequence() {
    let (mut topo, mut solid) = gecko();
    // (step, face picker, from, to, seams [y0, x0], rounds [y0, x0])
    let steps: [Step; 4] = [
        (
            "step 4",
            |t, s| convex_band(t, s, Side::X0),
            4.0,
            1.9,
            [4.0, 4.0],
            [4.0, 1.9],
        ),
        (
            "step 6",
            |t, s| seam_band(t, s, Side::Y0),
            4.0,
            2.6,
            [2.6, 4.0],
            [4.0, 1.9],
        ),
        (
            "step 7",
            |t, s| convex_band(t, s, Side::Y0),
            4.0,
            1.6,
            [2.6, 4.0],
            [1.6, 1.9],
        ),
        (
            "seam again",
            |t, s| seam_band(t, s, Side::Y0),
            2.6,
            3.0,
            [3.0, 4.0],
            [1.6, 1.9],
        ),
    ];
    for (label, pick, from, to, seams, rounds) in steps {
        let before = fingerprint(&topo, solid);
        let face = pick(&topo, solid);
        let result = resize_blend(&mut topo, solid, face, from, to).unwrap();
        assert_eq!(fingerprint(&topo, solid), before, "{label}: source");
        solid = result.solid;
        verify(&topo, solid, label);
        let mut expected = BTreeMap::new();
        expected.insert("plane".to_owned(), 7);
        expected.insert("sampled".to_owned(), 2);
        expected.insert("torus 10.000/4.000".to_owned(), 1);
        for radius in [seams[0], seams[1], rounds[0], rounds[1], 6.0] {
            *expected.entry(format!("cylinder {radius:.3}")).or_insert(0) += 1;
        }
        assert_eq!(census(&topo, solid), expected, "{label}");
        // A side is a boss corner unless its seam equals the shoulder under
        // a round of its own radius (then the round is walked into it).
        let corners = seams
            .iter()
            .zip(rounds)
            .filter(|&(&seam, round)| (seam - 4.0).abs() > 1e-9 || (round - 4.0).abs() < 1e-9)
            .count();
        assert_corners(&topo, solid, corners, label);
        for (side, (seam, round)) in Side::BOTH.into_iter().zip(seams.into_iter().zip(rounds)) {
            assert_seam(&topo, solid, side, seam, label);
            assert_round(&topo, solid, side, round, label);
        }
    }
    let before = fingerprint(&topo, solid);
    let seam = seam_band(&topo, solid, Side::Y0);
    let error = resize_blend(&mut topo, solid, seam, 3.0, 2.0).unwrap_err();
    assert_eq!(
        resize_blend_failure_code(&error),
        "blend-cluster-reblend-failed",
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before, "refused seam edit");
}
