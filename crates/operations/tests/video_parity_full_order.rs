//! The reference video's full order on one body: the filleted Gentle-Gecko
//! with its R1 cap rim (A0), the cap lengthened to z61.2 and shortened to z26
//! into the R4 shoulder with sampled permission (A1), then the edits that
//! follow it.
//!
//! A1's shortening keeps the shoulder and trims it, and the corner patches
//! beside it, by the cap plane; the rim re-rolls over them as exact tori on
//! the boss and shoulder and as sampled bands over the corner patches. Before
//! the joined-cap route those sampled rim bands (rolling on sampled corner
//! patches, so no closed form explains them) made every later group edit
//! refuse: A2 with `blend end face .. is a nurbs surface`, A7 with
//! `move-face does not support face .. (nurbs)`. The group route now edits
//! such a body with its cap lifted clear of the shoulder and lowers it back
//! with the permission it was lowered with (`blend_cluster/joined_cap.rs`).
//!
//! The steps at the acceptance spec's values, in the order the OpenZCAD
//! application runs them on one cumulative body:
//!
//! - **A2 convex**: the convex top band along the x = 0 face, R4 -> 1.5.
//!   The cap lowered back past the band's walked corner patch used to fold
//!   (`regress_cap_descent_walked_corner`).
//! - **A2 concave**: the concave vertical seam R4 -> 2 on the opposite side,
//!   the y = 0 face, as the video does it (its step 6). The seam beside the
//!   R1.5 band would put its corner in the unequal corner's fold domain and
//!   refuses typed, editing nothing.
//! - **A3**: R1 on the x = 0 / y = 18 vertical edge (22.5 mm long once the
//!   band is R1.5, as in the video).
//! - **A5** (the video's step 9, before its A4 edits): a negative fillet
//!   radius on the free-end top edge refuses and leaves the body unchanged.
//! - **A4**: the one picked free-end top edge filleted R1.5 and R3.7 (built,
//!   inspected and discarded, as the video undoes them). Both radii exceed
//!   A3's R1 band the edge's tangent chain runs over, so they build through
//!   the blend group, the R1 band re-filleted after the new round. Then the
//!   edge chamfered 0.7, which is kept: picked alone it does not chamfer (its
//!   corners lie on the band end arcs) and `chamfer_v2` names the five-edge
//!   tangent chain; the chain builds. The broad chamfer the video builds and
//!   undoes (5) is the ignored `broad_free_end_chamfer_consumes_the_rounds`:
//!   no chamfer engine yet builds a setback larger than the rounds its chain
//!   crosses.
//! - **A6**: the 0.8 bottom chamfer from one seed edge (its seven-edge tangent
//!   chain), and the application's twelve-edge closed bottom outline, which
//!   adds the free end's bottom edge and the A4 stripes' feet, mitred.
//! - **A7**: the boss Ø12 -> 14.6 -> 17 -> (21.1 previewed through the
//!   prepared API, then cancelled) -> 14.7, and an oversize Ø60 refusing.
//!
//! Every result meets PLAN.md's verification bar (both validators, watertight
//! manifold meshes at two deflections and at the application's display
//! deflection, API against independent mesh volume), is probed at the edited
//! feature and keeps planes, cylinders, cones and tori analytic. Every group
//! edit goes through the prepared API as the application drives it: a
//! preview the kernel returns must commit (the preview/commit contract), and
//! the commit is the unprepared call's result.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_cluster::{BlendClusterError, GroupEditKind, PreparedGroupEdit};
use remus_operations::blend_ops::{chamfer_v2, chamfer_v2_tangent_chain, fillet_cascade};
use remus_operations::face_move_quality::{
    FaceMoveApproximation, FaceMoveQuality, move_faces_with_quality,
};
use remus_operations::measure::solid_volume;
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::resize_blend::resize_blend;
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{
    face_edges, face_vertices, solid_edges, solid_entity_counts, solid_faces,
};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::OrientedEdge;

/// A1's sampled permission, as the application passes it.
const SAMPLED: Option<FaceMoveApproximation> = Some(FaceMoveApproximation { stations: 256 });
/// The video's free-end chamfer distance.
const FREE_END: f64 = 0.7;
/// The video's bottom chamfer distance.
const BOTTOM: f64 = 0.8;
/// A2's convex band radius (the spec's R4 -> 1.5).
const BAND: f64 = 1.5;
/// A2's concave seam radius (the spec's R4 -> 2).
const SEAM: f64 = 2.0;
/// The free-end fillet radii A4 builds and discards.
const END_FILLETS: [f64; 2] = [1.5, 3.7];
/// The broad free-end chamfer A4 builds and discards.
const END_BROAD: f64 = 5.0;
/// The application's display deflection (`tessellateSolidGroupedBinary`).
const DISPLAY: f64 = 0.005;

// ---------------------------------------------------------------------------
// Fixtures and picks.

fn import(text: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(text, &mut topo).unwrap()[0];
    (topo, solid)
}

fn top_at(topo: &Topology, solid: SolidId, height: f64) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.dot(Vec3::new(0.0, 0.0, 1.0)) > 1.0 - 1e-9)
                && face_vertices(topo, face)
                    .unwrap()
                    .into_iter()
                    .all(|vertex| (topo.vertex(vertex).unwrap().point().z() - height).abs() < 1e-7)
        })
        .expect("planar top at requested height")
}

/// A0: the filleted Gecko with its R1 cap rim.
fn a0() -> (Topology, SolidId) {
    let (mut topo, solid) = import(include_str!("data/gentle_gecko_filleted.step"));
    let cap = top_at(&topo, solid, 36.0);
    let rim = face_edges(&topo, cap).unwrap();
    let solid = fillet_cascade(&mut topo, solid, &rim, 1.0).unwrap().solid;
    (topo, solid)
}

/// A0, then A1 lengthened to z61.2 and shortened to z26, each verified.
fn a1() -> (Topology, SolidId) {
    let (mut topo, solid) = a0();
    verify(&topo, solid, "A0");
    let cap = top_at(&topo, solid, 36.0);
    let long = move_faces_with_quality(&mut topo, solid, &[cap], 25.2, SAMPLED).unwrap();
    verify(&topo, long.result.solid, "A1 lengthen");
    assert_eq!(census(&topo, long.result.solid), census(&topo, solid));
    let cap = top_at(&topo, long.result.solid, 61.2);
    let short = move_faces_with_quality(&mut topo, long.result.solid, &[cap], 26.0 - 61.2, SAMPLED)
        .unwrap();
    assert_eq!(
        short.quality,
        FaceMoveQuality::SampledCapJoin { stations: 256 }
    );
    verify(&topo, short.result.solid, "A1 shorten");
    (topo, short.result.solid)
}

/// The boss wall: the cylinder on the Z axis.
fn boss_of(topo: &Topology, solid: SolidId) -> FaceId {
    let walls: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .collect();
    assert_eq!(walls.len(), 1, "one boss wall");
    walls[0]
}

/// The convex top band along the block's x = 0 face (its axis along Y).
fn x0_band(topo: &Topology, solid: SolidId) -> FaceId {
    let bands: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9
                    && c.origin().x() > 0.0)
        })
        .collect();
    assert_eq!(bands.len(), 1, "one convex band along the x = 0 face");
    bands[0]
}

/// The concave seam round on the block's x = 0 face.
fn x0_seam(topo: &Topology, solid: SolidId) -> FaceId {
    let seams: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x() < -1.0)
        })
        .collect();
    assert_eq!(seams.len(), 1, "one seam round on the x = 0 face");
    seams[0]
}

/// The concave seam round on the block's y = 0 face.
fn y0_seam(topo: &Topology, solid: SolidId) -> FaceId {
    let seams: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().y() < -1.0)
        })
        .collect();
    assert_eq!(seams.len(), 1, "one seam round on the y = 0 face");
    seams[0]
}

fn edge_ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

fn edge_length(topo: &Topology, edge: EdgeId) -> f64 {
    let (a, b) = edge_ends(topo, edge);
    (b - a).length()
}

/// The single straight edge with both ends satisfying `on`.
fn line_where(topo: &Topology, solid: SolidId, on: impl Fn(Point3) -> bool, what: &str) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line))
        .filter(|&edge| {
            let (a, b) = edge_ends(topo, edge);
            on(a) && on(b)
        })
        .collect();
    assert_eq!(found.len(), 1, "one {what}");
    found[0]
}

/// The A3 pick: the block's x = 0 / y = 18 vertical edge.
fn a3_pick(topo: &Topology, solid: SolidId) -> EdgeId {
    line_where(
        topo,
        solid,
        |p| p.x().abs() < 1e-7 && (p.y() - 18.0).abs() < 1e-7,
        "x = 0 / y = 18 vertical edge",
    )
}

/// The A4 pick: the free end's top edge.
fn a4_pick(topo: &Topology, solid: SolidId) -> EdgeId {
    line_where(
        topo,
        solid,
        |p| (p.x() - 30.0).abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7,
        "free-end top edge",
    )
}

/// The A6 seed: the bottom edge along the block's y = 0 face.
fn bottom_pick(topo: &Topology, solid: SolidId) -> EdgeId {
    line_where(
        topo,
        solid,
        |p| p.y().abs() < 1e-7 && p.z().abs() < 1e-7,
        "bottom edge along y = 0",
    )
}

/// The bottom face.
fn bottom_face(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|normal| normal.z() < -1.0 + 1e-9)
        })
        .unwrap()
}

// ---------------------------------------------------------------------------
// Verification.

fn bbox_diagonal(topo: &Topology, solid: SolidId) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mesh = tessellate_solid(topo, solid, 0.1).unwrap();
    for p in &mesh.positions {
        for (axis, value) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
            lo[axis] = lo[axis].min(value);
            hi[axis] = hi[axis].max(value);
        }
    }
    ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt()
}

/// Signed mesh volume summed about a point of the mesh.
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

/// PLAN.md's bar: both validators clean, a watertight manifold mesh at two
/// deflections at or below `bbox_diag * 5e-5`, and the API volume within
/// 1e-4 of the independent mesh volume. Returns the API volume.
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
    // The display mesh must close too; its chordal volume error is not
    // held to the measuring bar.
    let display = tessellate_solid(topo, solid, DISPLAY).unwrap();
    assert_eq!(
        boundary_edge_count(&display),
        0,
        "{label} at display {DISPLAY}"
    );
    assert_eq!(
        non_manifold_edge_count(&display),
        0,
        "{label} at display {DISPLAY}"
    );
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

/// Face count per surface type.
fn census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for face in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(face).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

/// Points that must be inside (`true`) or outside the body.
fn probe(topo: &Topology, solid: SolidId, points: &[(Point3, bool)], label: &str) {
    for &(point, inside) in points {
        let expected = if inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(
            classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap(),
            expected,
            "{label}: {point:?}"
        );
    }
}

fn fingerprint(topo: &Topology, solid: SolidId) -> String {
    remus_io::step::writer::write_step(topo, &[solid]).unwrap()
}

/// The result matches its fresh construction in face count, face census and
/// volume (1e-6 relative).
fn assert_matches_oracle(
    topo: &Topology,
    solid: SolidId,
    oracle_topo: &Topology,
    oracle: SolidId,
    label: &str,
) {
    assert_eq!(
        census(topo, solid),
        census(oracle_topo, oracle),
        "{label}: census"
    );
    assert_eq!(
        solid_entity_counts(topo, solid).unwrap().0,
        solid_entity_counts(oracle_topo, oracle).unwrap().0,
        "{label}: faces"
    );
    let volume = solid_volume(topo, solid, 1e-3).unwrap();
    let expected = solid_volume(oracle_topo, oracle, 1e-3).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "{label}: volume {volume} vs oracle {expected}"
    );
}

/// Planes, cylinders, cones and tori stay analytic: the only sampled faces
/// are the `sampled` this body is allowed.
fn assert_sampled(topo: &Topology, solid: SolidId, sampled: usize, label: &str) {
    let census = census(topo, solid);
    assert_eq!(
        census.get("nurbs").copied().unwrap_or(0),
        sampled,
        "{label}: {census:?}; sampled faces span {:?}",
        sampled_extents(topo, solid)
    );
    for tag in census.keys() {
        assert!(
            ["plane", "cylinder", "cone", "torus", "nurbs"].contains(tag),
            "{label}: {census:?}"
        );
    }
}

/// The vertex bounding box of every sampled face, rounded to 0.01 mm.
fn sampled_extents(topo: &Topology, solid: SolidId) -> Vec<[f64; 6]> {
    let round = |value: f64| (value * 100.0).round() / 100.0;
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Nurbs(_)))
        .map(|face| {
            let mut extent = [
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ];
            for vertex in face_vertices(topo, face).unwrap() {
                let point = topo.vertex(vertex).unwrap().point();
                for (axis, value) in [point.x(), point.y(), point.z()].into_iter().enumerate() {
                    extent[axis] = extent[axis].min(round(value));
                    extent[axis + 3] = extent[axis + 3].max(round(value));
                }
            }
            extent
        })
        .collect()
}

/// The cap at z26 with its R1 rim round the boss of radius `radius`: just
/// under the cap is material, just above is air, and the rim rounds the cap
/// edge.
fn assert_cap_at_26(topo: &Topology, solid: SolidId, radius: f64, label: &str) {
    let back = Vec3::new(-1.0, -1.0, 0.0).normalize().unwrap();
    let at = |r: f64, z: f64| Point3::new(0.0, 0.0, z) + back * r;
    probe(
        topo,
        solid,
        &[
            (at(radius - 2.0, 25.9), true),
            (at(radius - 2.0, 26.1), false),
            // The rim's ball removes the cap edge's corner.
            (at(radius - 0.1, 25.9), false),
            (at(radius - 1.0, 25.5), true),
        ],
        label,
    );
}

/// The boss wall at radius `r`: material just inside it, air just outside.
fn assert_boss_wall(topo: &Topology, solid: SolidId, r: f64, label: &str) {
    let back = Vec3::new(-1.0, -1.0, 0.0).normalize().unwrap();
    let at = |radius: f64| Point3::new(0.0, 0.0, 12.0) + back * radius;
    probe(
        topo,
        solid,
        &[(at(r - 0.05), true), (at(r + 0.05), false)],
        label,
    );
}

// ---------------------------------------------------------------------------
// Fresh constructions.

fn gecko_sharp() -> (Topology, SolidId) {
    import(include_str!("data/gentle_gecko_sharp.step"))
}

/// The four sharp edges of the sharp Gecko that stay at R4: the two seams,
/// the shoulder and the y = 0 top edge.
fn sharp_r4_edges(topo: &Topology, solid: SolidId) -> Vec<EdgeId> {
    let r = 6.0;
    let mut found = Vec::new();
    for edge in solid_edges(topo, solid).unwrap() {
        let (a, b) = edge_ends(topo, edge);
        match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle)
                if (circle.radius() - r).abs() < 1e-7
                    && (circle.center().z() - 24.0).abs() < 1e-7 =>
            {
                found.push(edge);
            }
            EdgeCurve::Line => {
                let vertical = (a.x() - b.x()).abs() < 1e-9 && (a.y() - b.y()).abs() < 1e-9;
                let seam = vertical
                    && (a.x().hypot(a.y()) - r).abs() < 1e-7
                    && a.z().min(b.z()) < 1e-9
                    && (a.z().max(b.z()) - 24.0).abs() < 1e-7;
                let top = |p: Point3| (p.z() - 24.0).abs() < 1e-7;
                let along_x = top(a) && top(b) && a.y().abs() < 1e-7 && b.y().abs() < 1e-7;
                if seam || (along_x && a.x().min(b.x()) > 1e-7) {
                    found.push(edge);
                }
            }
            _ => {}
        }
    }
    assert_eq!(
        found.len(),
        4,
        "two seams, the shoulder and the y = 0 top edge"
    );
    found
}

/// Fresh construction of A2's convex edit in the planner's order: the sharp
/// Gecko, the R4 network, the x = 0 top band at `band`, the cap rim, then
/// the cap lowered from z36 to z26 with A1's permission.
fn oracle(band: f64) -> (Topology, SolidId) {
    let (mut topo, sharp) = gecko_sharp();
    let r4 = sharp_r4_edges(&topo, sharp);
    let mut solid = fillet_cascade(&mut topo, sharp, &r4, 4.0).unwrap().solid;
    let x0 = line_where(
        &topo,
        solid,
        |p| p.x().abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7,
        "x = 0 top edge",
    );
    solid = fillet_cascade(&mut topo, solid, &[x0], band).unwrap().solid;
    let cap = top_at(&topo, solid, 36.0);
    let rim = face_edges(&topo, cap).unwrap();
    solid = fillet_cascade(&mut topo, solid, &rim, 1.0).unwrap().solid;
    let cap = top_at(&topo, solid, 36.0);
    let lowered = move_faces_with_quality(&mut topo, solid, &[cap], -10.0, SAMPLED).unwrap();
    assert_eq!(
        lowered.quality,
        FaceMoveQuality::SampledCapJoin { stations: 256 }
    );
    (topo, lowered.result.solid)
}

// ---------------------------------------------------------------------------
// Group edits as the application drives them.

/// A group edit's prepared preview at `value`, then its commit.
///
/// The application previews every drag frame through the prepared API and
/// keeps the value with a commit. A preview the kernel returns must commit:
/// the contract fails if the preview builds and the commit refuses (or the
/// reverse). The commit is checked against the unprepared call, and the
/// source stays byte-identical throughout. Returns the committed solid, or
/// the refusal both agree on.
fn group_edit(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    kind: GroupEditKind,
    value: f64,
    label: &str,
) -> Result<SolidId, OperationsError> {
    let before = fingerprint(topo, solid);
    let prepared = match kind {
        GroupEditKind::BandRadius { expected_radius } => {
            PreparedGroupEdit::prepare_band(topo, solid, face, expected_radius)
        }
        GroupEditKind::CylinderRadius { .. } => {
            PreparedGroupEdit::prepare_cylinder(topo, solid, face)
        }
    };
    let unprepared = |topo: &mut Topology| match kind {
        GroupEditKind::BandRadius { expected_radius } => {
            resize_blend(topo, solid, face, expected_radius, value).map(|result| result.solid)
        }
        GroupEditKind::CylinderRadius { .. } => resize_cylindrical_face(topo, solid, face, value),
    };
    let result = match &prepared {
        Ok(prepared) => {
            let preview = prepared.preview(topo, value);
            assert_eq!(fingerprint(topo, solid), before, "{label}: preview source");
            let committed = prepared.commit(topo, value);
            assert_eq!(fingerprint(topo, solid), before, "{label}: commit source");
            match (&preview, &committed) {
                (Ok(_), Err(error)) => {
                    panic!("{label}: the preview built but the commit refused: {error}")
                }
                (Err(error), Ok(_)) => {
                    panic!("{label}: the commit built but the preview refused: {error}")
                }
                _ => {}
            }
            committed
        }
        Err(error) => {
            assert!(
                matches!(
                    error,
                    OperationsError::BlendCluster(BlendClusterError::NotPrepared { .. })
                ),
                "{label}: preparation: {error}"
            );
            unprepared(topo)
        }
    };
    if prepared.is_ok() {
        // The commit is the unprepared call's answer.
        let direct = unprepared(topo);
        match (&result, &direct) {
            (Ok(committed), Ok(direct)) => {
                assert_eq!(
                    census(topo, *committed),
                    census(topo, *direct),
                    "{label}: prepared and unprepared census"
                );
                let a = solid_volume(topo, *committed, 1e-3).unwrap();
                let b = solid_volume(topo, *direct, 1e-3).unwrap();
                assert!(
                    (a - b).abs() <= 1e-9 * b,
                    "{label}: prepared {a} vs unprepared {b}"
                );
            }
            (Err(_), Err(_)) => {}
            (Ok(_), Err(error)) => panic!("{label}: the unprepared call refused: {error}"),
            (Err(error), Ok(_)) => panic!("{label}: the prepared commit refused: {error}"),
        }
    }
    assert_eq!(fingerprint(topo, solid), before, "{label}: source");
    result
}

/// [`group_edit`] that must build.
fn group_edit_ok(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    kind: GroupEditKind,
    value: f64,
    label: &str,
) -> SolidId {
    group_edit(topo, solid, face, kind, value, label)
        .unwrap_or_else(|error| panic!("{label}: {error}"))
}

// ---------------------------------------------------------------------------
// The order.

/// A1 then A2 and A3 on one body at the spec's values, each step verified.
fn a1_to_a3() -> (Topology, SolidId) {
    let (mut topo, solid) = a1();
    assert_sampled(&topo, solid, 4, "A1");
    assert_cap_at_26(&topo, solid, 6.0, "A1");

    // A2 convex: the band R4 -> 1.5 through the joined cap.
    let band = x0_band(&topo, solid);
    let a2 = group_edit_ok(
        &mut topo,
        solid,
        band,
        GroupEditKind::BandRadius {
            expected_radius: 4.0,
        },
        BAND,
        "A2 convex",
    );
    verify(&topo, a2, "A2 convex");
    // The R1.5 band leaves material the R4 band took.
    probe(
        &topo,
        a2,
        &[
            (Point3::new(0.6, 14.0, 23.4), true),
            (Point3::new(0.2, 14.0, 23.8), false),
        ],
        "A2 convex",
    );
    assert_cap_at_26(&topo, a2, 6.0, "A2 convex");
    assert_sampled(&topo, a2, 4, "A2 convex");
    let (oracle_topo, oracle_solid) = oracle(BAND);
    assert_matches_oracle(&topo, a2, &oracle_topo, oracle_solid, "A2 convex");

    // The seam on the band's own side cannot go R2: its corner joins the R2
    // seam, the R4 shoulder and the R1.5 band, inside the unequal corner's
    // fold domain (`docs/design/unequal-radius-boss-corner.md`). It refuses
    // typed, its preview refuses too, and the body is unchanged.
    let same_side = x0_seam(&topo, a2);
    let error = group_edit(
        &mut topo,
        a2,
        same_side,
        GroupEditKind::BandRadius {
            expected_radius: 4.0,
        },
        SEAM,
        "A2 concave beside the band",
    )
    .unwrap_err();
    assert!(
        matches!(
            &error,
            OperationsError::BlendCluster(BlendClusterError::ReblendFailed { .. })
        ),
        "{error}"
    );

    // A2 concave: as the video does it (its step 6), the vertical seam on
    // the opposite side, the y = 0 face, R4 -> 2.
    let seam = y0_seam(&topo, a2);
    let a2 = group_edit_ok(
        &mut topo,
        a2,
        seam,
        GroupEditKind::BandRadius {
            expected_radius: 4.0,
        },
        SEAM,
        "A2 concave",
    );
    verify(&topo, a2, "A2 concave");
    // Air inside the R2 ball that the R4 round filled; material near the
    // seam corner both rounds fill.
    probe(
        &topo,
        a2,
        &[
            (Point3::new(7.3, -0.3, 12.0), false),
            (Point3::new(6.3, -0.1, 12.0), true),
        ],
        "A2 concave",
    );
    assert_cap_at_26(&topo, a2, 6.0, "A2 concave");
    assert_sampled(&topo, a2, 4, "A2 concave");

    // A3: R1 on the corrected vertical edge, turning through the band.
    let pick = a3_pick(&topo, a2);
    assert!((edge_length(&topo, pick) - (24.0 - BAND)).abs() < 1e-9);
    let a3 = fillet_cascade(&mut topo, a2, &[pick], 1.0).unwrap().solid;
    verify(&topo, a3, "A3");
    probe(
        &topo,
        a3,
        &[
            (Point3::new(0.15, 17.85, 12.0), false),
            (Point3::new(0.5, 17.5, 12.0), true),
        ],
        "A3",
    );
    assert_sampled(&topo, a3, 5, "A3");
    (topo, a3)
}

/// [`a1_to_a3`], then A5, A4 and A6 on the same body, each step verified.
/// Returns the A6 body and the A4 body before it (for the application's
/// twelve-edge A6 selection).
fn a1_to_a6() -> (Topology, SolidId, SolidId) {
    let (mut topo, a3) = a1_to_a3();

    // A5 (the video's step 9, before its A4 edits): a negative radius on
    // the free-end top edge refuses and leaves the body as it was.
    let before = fingerprint(&topo, a3);
    let pick = a4_pick(&topo, a3);
    assert!(
        fillet_cascade(&mut topo, a3, &[pick], -1.0).is_err(),
        "A5 built"
    );
    assert_eq!(fingerprint(&topo, a3), before, "A5: source");

    // A4: the free-end fillets the video builds and undoes, from the one
    // picked edge as the application sends it. Both radii are larger than
    // the A3 band its tangent chain runs over, so they build through the
    // blend group: the R1 band is re-filleted after the new round.
    let before = fingerprint(&topo, a3);
    for radius in END_FILLETS {
        let label = format!("A4 fillet R{radius}");
        let pick = a4_pick(&topo, a3);
        let filleted = fillet_cascade(&mut topo, a3, &[pick], radius)
            .unwrap_or_else(|error| panic!("{label}: {error}"))
            .solid;
        assert_eq!(fingerprint(&topo, a3), before, "{label}: source");
        verify(&topo, filleted, &label);
        // The edge's corner is gone; the ball's side of the round is not.
        let inside = 30.0 - 0.4 * radius;
        probe(
            &topo,
            filleted,
            &[
                (Point3::new(29.8, 9.0, 23.8), false),
                (Point3::new(inside, 9.0, 24.0 - 0.4 * radius), true),
            ],
            &label,
        );
        // One sampled face more than A3: the corner where the re-filleted
        // R1 band meets the new round at the free end.
        assert_sampled(&topo, filleted, 6, &label);
    }

    // A4's broad chamfer (5) is `broad_free_end_chamfer_consumes_the_rounds`.
    let pick = a4_pick(&topo, a3);

    // A4: the retained 0.7 chamfer. The edge alone names its chain; the
    // chain builds.
    let chain = match chamfer_v2(&mut topo, a3, &[pick], FREE_END, FREE_END) {
        Err(OperationsError::ChamferTangentChainRequired {
            selected, chain, ..
        }) => {
            assert_eq!(selected, vec![pick.index()]);
            chain
        }
        Err(other) => panic!("A4 single edge: {other}"),
        Ok(_) => panic!("A4 single edge built"),
    };
    assert_eq!(fingerprint(&topo, a3), before, "A4 single: source");
    assert_eq!(chain.len(), 5, "A4: the five-edge tangent chain");
    let chain: Vec<EdgeId> = chain
        .iter()
        .map(|&index| topo.edge_id_from_index(index).unwrap())
        .collect();
    let named = chamfer_v2(&mut topo, a3, &chain, FREE_END, FREE_END)
        .unwrap()
        .solid;
    let a4 = chamfer_v2_tangent_chain(&mut topo, a3, &[pick], FREE_END, FREE_END)
        .unwrap()
        .solid;
    verify(&topo, a4, "A4");
    assert_eq!(census(&topo, named), census(&topo, a4));
    assert!(
        (solid_volume(&topo, named, 1e-3).unwrap() - solid_volume(&topo, a4, 1e-3).unwrap()).abs()
            < 1e-9
    );
    probe(
        &topo,
        a4,
        &[
            (Point3::new(29.8, 9.0, 23.8), false),
            (Point3::new(29.5, 9.0, 23.5), true),
        ],
        "A4",
    );
    assert_sampled(&topo, a4, 5, "A4");

    // A6 from its seed: the seven-edge bottom chain.
    let pick = bottom_pick(&topo, a4);
    let a6 = chamfer_v2_tangent_chain(&mut topo, a4, &[pick], BOTTOM, BOTTOM)
        .unwrap()
        .solid;
    verify(&topo, a6, "A6");
    probe(
        &topo,
        a6,
        &[
            (Point3::new(15.0, 0.2, 0.2), false),
            (Point3::new(15.0, 0.5, 0.5), true),
        ],
        "A6",
    );
    assert_sampled(&topo, a6, 5, "A6");
    (topo, a6, a4)
}

/// A1 -> A2 convex 1.5 -> A2 concave 2 -> A3 -> A4 -> A5 -> A6 -> A7
/// Ø14.6 -> 17 -> (21.1 previewed) -> 14.7 on one body, then an oversize Ø60:
/// every step builds and verifies (the oversize refuses typed, editing
/// nothing); the joined cap stays at z26 with its rim throughout.
#[test]
fn full_video_order_at_spec_values() {
    let (mut topo, mut solid, _) = a1_to_a6();
    let census_before = census(&topo, solid);
    for diameter in [14.6_f64, 17.0, 21.1, 14.7] {
        let label = format!("A7 diameter {diameter}");
        let radius = diameter / 2.0;
        let before = fingerprint(&topo, solid);
        let boss = boss_of(&topo, solid);
        let kind = GroupEditKind::CylinderRadius {
            radius: boss_radius(&topo, boss),
        };
        if (diameter - 21.1).abs() < 1e-12 {
            // Previewed through the prepared API, then cancelled.
            let prepared = PreparedGroupEdit::prepare_cylinder(&topo, solid, boss)
                .unwrap_or_else(|error| panic!("{label}: {error}"));
            let preview = prepared
                .preview(&mut topo, radius)
                .unwrap_or_else(|error| panic!("{label}: {error}"));
            assert_eq!(fingerprint(&topo, solid), before, "{label}: source");
            assert_boss_wall(&topo, preview, radius, &label);
            assert_eq!(census(&topo, preview), census_before, "{label}: census");
            let mesh = tessellate_solid(&topo, preview, 0.05).unwrap();
            assert_eq!(boundary_edge_count(&mesh), 0, "{label}");
            assert_eq!(non_manifold_edge_count(&mesh), 0, "{label}");
            // The cancelled preview commits too.
            let committed = prepared
                .commit(&mut topo, radius)
                .unwrap_or_else(|error| panic!("{label}: previewed, commit refused: {error}"));
            verify(&topo, committed, &label);
            continue;
        }
        let result = group_edit_ok(&mut topo, solid, boss, kind, radius, &label);
        verify(&topo, result, &label);
        assert_eq!(census(&topo, result), census_before, "{label}: census");
        assert_boss_wall(&topo, result, radius, &label);
        assert_cap_at_26(&topo, result, radius, &label);
        // The bottom chain, the free-end chain and the A3 round are re-cut.
        probe(
            &topo,
            result,
            &[
                (Point3::new(15.0, 0.2, 0.2), false),
                (Point3::new(29.8, 9.0, 23.8), false),
                (Point3::new(0.15, 17.85, 12.0), false),
                (Point3::new(15.0, 9.0, 12.0), true),
            ],
            &label,
        );
        solid = result;
    }

    // An oversize diameter refuses with the diameter-fit refusal, editing
    // nothing.
    let before = fingerprint(&topo, solid);
    let boss = boss_of(&topo, solid);
    let kind = GroupEditKind::CylinderRadius {
        radius: boss_radius(&topo, boss),
    };
    let error = group_edit(&mut topo, solid, boss, kind, 30.0, "A7 oversize").unwrap_err();
    assert!(
        matches!(
            &error,
            OperationsError::BlendCluster(BlendClusterError::DiameterUnbuildable { .. })
        ),
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
}

fn boss_radius(topo: &Topology, boss: FaceId) -> f64 {
    match topo.face(boss).unwrap().surface() {
        FaceSurface::Cylinder(cylinder) => cylinder.radius(),
        other => panic!("boss wall is {}", other.type_tag()),
    }
}

/// The application's A6 selection after A4: the bottom face's twelve-edge
/// closed outline. It holds the seven-edge chain, the free end's bottom edge
/// and the two A4 stripes' feet; the band runs round all of it, mitred at
/// the four 135° corners the stripes make. Removed volume against the closed
/// form: `d²/2` per unit length of each straight edge, Pappus on each arc,
/// and `−cot(α/2)·d³/3` per convex corner of interior angle `α`.
#[test]
fn closed_bottom_outline_after_the_free_end_chamfer() {
    for shortened in [false, true] {
        let label = if shortened { "after A1" } else { "without A1" };
        let (mut topo, a4) = if shortened {
            let (topo, _, a4) = a1_to_a6();
            (topo, a4)
        } else {
            let (mut topo, mut solid) = a0();
            let band = x0_band(&topo, solid);
            solid = resize_blend(&mut topo, solid, band, 4.0, BAND)
                .unwrap()
                .solid;
            let pick = a3_pick(&topo, solid);
            solid = fillet_cascade(&mut topo, solid, &[pick], 1.0)
                .unwrap()
                .solid;
            let pick = a4_pick(&topo, solid);
            solid = chamfer_v2_tangent_chain(&mut topo, solid, &[pick], FREE_END, FREE_END)
                .unwrap()
                .solid;
            (topo, solid)
        };
        let floor = bottom_face(&topo, a4);
        let outline = face_edges(&topo, floor).unwrap();
        assert_eq!(outline.len(), 12, "{label}");
        let expected_removed = removed_round(&topo, a4, floor, BOTTOM);
        let before = fingerprint(&topo, a4);
        let result = chamfer_v2(&mut topo, a4, &outline, BOTTOM, BOTTOM).unwrap();
        assert_eq!(fingerprint(&topo, a4), before, "{label}: source");
        verify(&topo, result.solid, label);
        // Both bodies are measured alike. Their volumes come from meshes (the
        // corner patches are sampled), so the measured removal converges on
        // the closed form as the deflection shrinks.
        let removed = |deflection: f64| {
            solid_volume(&topo, a4, deflection).unwrap()
                - solid_volume(&topo, result.solid, deflection).unwrap()
        };
        let errors: Vec<f64> = [1e-3, 3e-4, 1e-4]
            .into_iter()
            .map(|deflection| (removed(deflection) - expected_removed).abs())
            .collect();
        assert!(
            errors.windows(2).all(|pair| pair[1] < pair[0]) && errors[2] <= 2e-4 * expected_removed,
            "{label}: removal errors {errors:?} against closed form {expected_removed}"
        );
        // Every new face is an exact plane or cone.
        let before_census = census(&topo, a4);
        for (tag, count) in census(&topo, result.solid) {
            let had = before_census.get(tag).copied().unwrap_or(0);
            assert!(
                count == had || (count > had && ["plane", "cone"].contains(&tag)),
                "{label}: {tag} {had} -> {count}"
            );
        }
        // The free end's bottom edge and the stripes' feet are bevelled too.
        probe(
            &topo,
            result.solid,
            &[
                (Point3::new(29.8, 9.0, 0.2), false),
                (Point3::new(29.5, 9.0, 0.5), true),
                (Point3::new(29.4, 0.7, 0.1), false),
                (Point3::new(15.0, 0.2, 0.2), false),
                (Point3::new(15.0, 17.8, 0.2), false),
            ],
            label,
        );
    }
}

/// A4's broad free-end chamfer (the video builds 5.2 and undoes it), from
/// the body after A3.
///
/// Open: the chain's setback runs over A3's R1 band, whose radius is below
/// the chord ball's (`d/√2` ≈ 3.5), so no ball rolls on it and the bevel would
/// have to mitre across the band and trim it; past 4.5 even the R4 band ends
/// fail to trim. The flat bevel refuses to re-trim curved neighbours. Both
/// need a construction the chamfer engines do not have.
#[test]
#[ignore = "open: a chamfer larger than the rounds its chain crosses (A4 chamfer 5)"]
fn broad_free_end_chamfer_consumes_the_rounds() {
    let (mut topo, a3) = a1_to_a3();
    let before = fingerprint(&topo, a3);
    let pick = a4_pick(&topo, a3);
    let broad = chamfer_v2_tangent_chain(&mut topo, a3, &[pick], END_BROAD, END_BROAD)
        .unwrap_or_else(|error| panic!("A4 chamfer {END_BROAD}: {error}"))
        .solid;
    assert_eq!(fingerprint(&topo, a3), before, "A4 broad: source");
    verify(&topo, broad, "A4 chamfer 5");
    probe(
        &topo,
        broad,
        &[
            (Point3::new(28.0, 9.0, 22.5), false),
            (Point3::new(27.0, 9.0, 21.5), true),
        ],
        "A4 chamfer 5",
    );
}

/// Closed-form material removed by an equal-distance `d` chamfer round the
/// whole outline of `floor`.
fn removed_round(topo: &Topology, solid: SolidId, floor: FaceId, d: f64) -> f64 {
    let section = 0.5 * d * d;
    let adjacency = topo.build_adjacency(solid).unwrap();
    let wire = topo
        .wire(topo.face(floor).unwrap().outer_wire())
        .unwrap()
        .edges()
        .to_vec();
    let mut removed = 0.0;
    for oe in &wire {
        let edge = oe.edge();
        let data = topo.edge(edge).unwrap();
        match data.curve() {
            EdgeCurve::Line => removed += section * edge_length(topo, edge),
            EdgeCurve::Circle(circle) => {
                let (t0, t1) = data.strict_domain().unwrap();
                let sweep = (t1 - t0).abs();
                let wall = adjacency
                    .faces_for_edge(edge)
                    .iter()
                    .copied()
                    .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
                    .unwrap();
                // A reversed wall is a concave round: material outside it.
                let concave = topo.face(wall).unwrap().is_reversed();
                let centroid = if concave {
                    circle.radius() + d / 3.0
                } else {
                    circle.radius() - d / 3.0
                };
                removed += section * sweep * centroid;
            }
            other => panic!("unexpected bottom edge {}", other.type_tag()),
        }
    }
    // Each convex corner between straight edges: each band loses the wedge
    // between its square end and the mitre, `cot(α/2)·d³/6`.
    let direction = |oe: OrientedEdge| {
        let (a, b) = edge_ends(topo, oe.edge());
        let along = if oe.is_forward() { b - a } else { a - b };
        along.normalize().unwrap()
    };
    for (i, &oe) in wire.iter().enumerate() {
        let next = wire[(i + 1) % wire.len()];
        if !matches!(topo.edge(oe.edge()).unwrap().curve(), EdgeCurve::Line)
            || !matches!(topo.edge(next.edge()).unwrap().curve(), EdgeCurve::Line)
        {
            continue;
        }
        let turn = direction(oe).dot(direction(next)).clamp(-1.0, 1.0).acos();
        if turn < 1e-9 {
            continue;
        }
        let alpha = std::f64::consts::PI - turn;
        removed -= d.powi(3) / 3.0 / (alpha / 2.0).tan();
    }
    removed
}

/// A single free-end edge after A2/A3, without A1: `chamfer_v2` chamfers
/// exactly its selection, so the lone edge refuses and the refusal names the
/// tangent chain whose chamfer builds; the body is unchanged. A selection
/// that is its whole chain keeps its own refusal.
#[test]
fn single_free_end_edge_names_its_tangent_chain() {
    let (mut topo, mut solid) = a0();
    let band = x0_band(&topo, solid);
    solid = resize_blend(&mut topo, solid, band, 4.0, BAND)
        .unwrap()
        .solid;
    let pick = a3_pick(&topo, solid);
    solid = fillet_cascade(&mut topo, solid, &[pick], 1.0)
        .unwrap()
        .solid;
    let before = fingerprint(&topo, solid);
    let pick = a4_pick(&topo, solid);
    let Err(error) = chamfer_v2(&mut topo, solid, &[pick], FREE_END, FREE_END) else {
        panic!("the lone free-end edge built");
    };
    assert_eq!(
        remus_operations::blend_ops::blend_failure_code(&error),
        "chamfer-tangent-chain-required"
    );
    let OperationsError::ChamferTangentChainRequired { chain, reason, .. } = error else {
        unreachable!()
    };
    assert!(reason.contains("curved"), "{reason}");
    assert_eq!(fingerprint(&topo, solid), before);
    let chain: Vec<EdgeId> = chain
        .iter()
        .map(|&index| topo.edge_id_from_index(index).unwrap())
        .collect();
    let expanded: Vec<EdgeId> = remus_blend::g1_chain::g1_chains(
        &topo,
        solid,
        &[pick],
        remus_math::tolerance::Tolerance::new(),
    )
    .unwrap()
    .concat();
    assert_eq!(chain.len(), expanded.len());
    assert!(expanded.iter().all(|edge| chain.contains(edge)));
    let result = chamfer_v2(&mut topo, solid, &chain, FREE_END, FREE_END).unwrap();
    verify(&topo, result.solid, "A4 chain");
    // An oversized chamfer of the whole chain keeps its own refusal.
    match chamfer_v2(&mut topo, solid, &chain, 30.0, 30.0) {
        Err(OperationsError::ChamferTangentChainRequired { .. }) => {
            panic!("a whole chain cannot require its own chain")
        }
        Err(_) => {}
        Ok(_) => panic!("a 30 mm chamfer built"),
    }
    assert_eq!(fingerprint(&topo, solid), before);
}
