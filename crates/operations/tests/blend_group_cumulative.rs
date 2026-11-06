//! The reference video's cumulative workflow on the filleted Gentle-Gecko
//! part: edits made through a recognized blend group on a body that already
//! carries later features.
//!
//! Gentle-Gecko is a 30 x 18 x 24 block with an R6 boss (axis +Z, height 36)
//! on its x = 0, y = 0 corner edge; the filleted part rounds the two concave
//! boss seams, the concave shoulder and the two convex block top edges at
//! R4, with two sampled corner patches where they meet. The video's steps,
//! mapped onto it (`VIDEO_FRAME_SPEC.md`):
//!
//! - **A2**: the convex top band along the x = 0 face R4 -> 1.9 (its re-fillet
//!   walks the smaller band into the seam round: a sampled transition).
//! - **A3** (video step 8, 34.4-37.1 s): R1 on the vertical boundary near the
//!   boss seen from the back, the block's x = 0 / y = 18 corner edge. Its
//!   tangent chain turns through the A2 band's end arc onto the long upper
//!   y = 18 edge. The picked edge is `24 - r` long under an R`r` band: 22.1
//!   after A2 at R1.9, and the video pill's 22.5 after A2 at R1.5.
//! - **A4**: the 0.7 chamfer of the free-end top edge, run down its tangent
//!   chain (the band end arcs and the free end's vertical edges).
//! - **A6**: a 0.8 chamfer picked on one bottom edge runs the tangent chain
//!   round the bottom outline (block edges, seam feet, boss foot, the A3
//!   round's foot).
//! - **A7**: the boss diameter 12 -> 14.6 -> 17 -> 14.7. The group the boss
//!   seeds holds every blend reached from it: the R4 network, the A2 band
//!   with its walked transition, the A3 round, and the chamfer stripes —
//!   planes along block edges and cones round the boss foot (bands restoring
//!   a sharp edge), cones round a round's foot or a band's end arc (corners
//!   collapsing with that round). The whole group is removed to the exact
//!   sharp part, the sharp wall resized, the fillets re-built and then the
//!   chamfer chains re-cut.
//!
//! The oracle for every edit is a fresh construction from the edited sharp
//! body with the same feature list in the planner's order (fillet stages
//! concave before convex and larger radii first, then the chamfer chains):
//! the result must match it in entity counts, face census and volume (1e-6
//! relative), and meet the full verification bar of `blend_group_edit.rs`.
//!
//! A4 and A6 build on one body in either order: each chain's run ends at a
//! free-end bottom corner on the other's 45° stripe, and the floor-chain
//! assembler runs the band on until its plane leaves the corner (across the
//! narrower stripe into the free end face for the wider bottom band). The
//! result is the union of the two chamfers, checked against each chain's own
//! volume less the two corner overlaps, and A7 then carries both chains.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_cluster::{
    BlendMemberForm, BlendMemberKind, Convexity, MemberSharp, ReblendStage, SharpCarrier,
    StageBlend, fillet_through_blend_group, plan_reblend, recognize_blend_cluster,
    remove_blend_cluster, resize_cylinder_through_group,
};
use remus_operations::blend_ops::{chamfer_v2_tangent_chain, fillet_cascade};
use remus_operations::measure::solid_volume;
use remus_operations::push_pull::resize_cylindrical_face;
use remus_operations::resize_blend::{resize_blend, resize_blend_failure_code};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// The video's bottom chamfer distance.
const BOTTOM: f64 = 0.8;

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

fn edge_ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

/// Straight edges with both ends satisfying `on`.
fn lines_where(topo: &Topology, solid: SolidId, on: impl Fn(Point3) -> bool) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line))
        .filter(|&edge| {
            let (a, b) = edge_ends(topo, edge);
            on(a) && on(b)
        })
        .collect()
}

/// The bottom edge along the block's y = 0 face: the video's single pick.
fn bottom_pick(topo: &Topology, solid: SolidId) -> EdgeId {
    let found = lines_where(topo, solid, |p| p.y().abs() < 1e-7 && p.z().abs() < 1e-7);
    assert_eq!(found.len(), 1, "one bottom edge along y = 0");
    found[0]
}

/// A6 on `solid`: the 0.8 chamfer chain from the bottom pick.
fn bottom_chamfer(topo: &mut Topology, solid: SolidId) -> SolidId {
    let pick = bottom_pick(topo, solid);
    chamfer_v2_tangent_chain(topo, solid, &[pick], BOTTOM, BOTTOM)
        .unwrap()
        .solid
}

/// The five sharp R4 edges of a (resized) sharp Gecko with boss radius `r`.
fn sharp_fillet_edges(topo: &Topology, solid: SolidId, r: f64) -> Vec<EdgeId> {
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
                let along_y = top(a) && top(b) && a.x().abs() < 1e-7 && b.x().abs() < 1e-7;
                if seam
                    || (along_x && a.x().min(b.x()) > 1e-7)
                    || (along_y && a.y().min(b.y()) > 1e-7)
                {
                    found.push(edge);
                }
            }
            _ => {}
        }
    }
    assert_eq!(found.len(), 5, "two seams, the shoulder and two top edges");
    found
}

/// The oracle for A7 after A6: the sharp Gecko resized exactly, the five
/// edges at R4 in one call, then the bottom chamfer chain.
fn boss_after_bottom_oracle(radius: f64) -> (Topology, SolidId) {
    let (mut topo, sharp) = gecko_sharp();
    let boss = boss_of(&topo, sharp);
    let resized = resize_cylindrical_face(&mut topo, sharp, boss, radius).unwrap();
    let edges = sharp_fillet_edges(&topo, resized, radius);
    let filleted = fillet_cascade(&mut topo, resized, &edges, 4.0)
        .unwrap()
        .solid;
    let solid = bottom_chamfer(&mut topo, filleted);
    (topo, solid)
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

fn classify(topo: &Topology, solid: SolidId, point: Point3) -> PointClassification {
    classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap()
}

/// The result matches its oracle in entity counts, face census and volume
/// (1e-6 relative).
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

fn fingerprint(topo: &Topology, solid: SolidId) -> String {
    remus_io::step::writer::write_step(topo, &[solid]).unwrap()
}

/// The bottom chamfer is cut along the block's y = 0 edge and round the
/// boss foot (radius `r`); just past it is material.
fn assert_bottom_chamfer(topo: &Topology, solid: SolidId, r: f64, label: &str) {
    let foot = -std::f64::consts::FRAC_PI_2 - 0.6;
    let (sin, cos) = foot.sin_cos();
    let at_foot = |inward: f64, z: f64| Point3::new((r - inward) * cos, (r - inward) * sin, z);
    for (point, expected, what) in [
        (
            Point3::new(20.0, 0.2, 0.2),
            PointClassification::Outside,
            "edge wedge",
        ),
        (
            Point3::new(20.0, 0.6, 0.6),
            PointClassification::Inside,
            "past the edge wedge",
        ),
        (
            at_foot(0.2, 0.2),
            PointClassification::Outside,
            "boss foot wedge",
        ),
        (
            at_foot(0.6, 0.6),
            PointClassification::Inside,
            "past the boss foot wedge",
        ),
        (at_foot(1.2, 0.1), PointClassification::Inside, "boss floor"),
    ] {
        assert_eq!(classify(topo, solid, point), expected, "{label}: {what}");
    }
}

/// The resized wall and the untouched block.
fn assert_boss_wall(topo: &Topology, solid: SolidId, r: f64, label: &str) {
    for angle in [200.0f64, 225.0, 250.0] {
        let (sin, cos) = angle.to_radians().sin_cos();
        for (offset, expected) in [
            (-0.05, PointClassification::Inside),
            (0.05, PointClassification::Outside),
        ] {
            let at = Point3::new((r + offset) * cos, (r + offset) * sin, 32.0);
            assert_eq!(
                classify(topo, solid, at),
                expected,
                "{label}: wall at {angle}"
            );
        }
    }
    assert_eq!(
        classify(topo, solid, Point3::new(0.0, 0.0, 35.95)),
        PointClassification::Inside,
        "{label}: cap"
    );
    assert_eq!(
        classify(topo, solid, Point3::new(29.0, 17.0, 23.0)),
        PointClassification::Inside,
        "{label}: block"
    );
}

// ---------------------------------------------------------------------------
// W10: chamfer stripes as group members.

/// The boss seeds the five fillets, their two corners and the bottom
/// chamfer's five stripes; removing the group returns the sharp part.
#[test]
fn bottom_chamfer_stripes_join_the_boss_group() {
    let (mut topo, filleted) = gecko();
    let solid = bottom_chamfer(&mut topo, filleted);
    let cluster = recognize_blend_cluster(&topo, solid, boss_of(&topo, solid)).unwrap();
    let count = |kind| cluster.members.iter().filter(|m| m.kind == kind).count();
    assert_eq!(count(BlendMemberKind::Band), 5);
    assert_eq!(count(BlendMemberKind::CornerPatch), 2);
    // The stripes along the two block edges and round the boss foot restore
    // sharp edges; the cones round the seam feet collapse with their seams.
    assert_eq!(count(BlendMemberKind::ChamferBand), 3);
    assert_eq!(count(BlendMemberKind::ChamferCorner), 2);
    for member in cluster.members.iter().filter(|m| m.kind.is_chamfer()) {
        assert!((member.radius - BOTTOM).abs() < 1e-9, "{member:?}");
        assert!(member.form.is_exact());
    }
    let mut chamfer_sharps: Vec<String> = cluster
        .members
        .iter()
        .filter(|m| m.kind == BlendMemberKind::ChamferBand)
        .map(|m| {
            let MemberSharp::Edge(index) = m.sharp else {
                panic!("a chamfer band restores an edge");
            };
            match (cluster.sharp_edges[index].carrier, m.form) {
                (SharpCarrier::Line { direction, .. }, BlendMemberForm::Plane) => {
                    format!("line {:.0} {:.0}", direction.x().abs(), direction.y().abs())
                }
                (SharpCarrier::Circle { radius, center, .. }, BlendMemberForm::Cone) => {
                    format!("circle {radius:.1} at z {:.1}", center.z())
                }
                other => panic!("unexpected chamfer sharp {other:?}"),
            }
        })
        .collect();
    chamfer_sharps.compact_sort_unstable();
    assert_eq!(
        chamfer_sharps,
        vec!["circle 6.0 at z 0.0", "line 0 1", "line 1 0"],
        "the block's two bottom edges and the boss foot"
    );
    // Planner order: the R4 fillets first, then one tangent-chain chamfer.
    let last = cluster.reblend_order.last().unwrap();
    assert_eq!(
        last.blend,
        StageBlend::Chamfer {
            tangent_chain: true
        }
    );
    assert!(
        cluster.reblend_order[..cluster.reblend_order.len() - 1]
            .iter()
            .all(|step| step.blend == StageBlend::Fillet)
    );

    let restored = remove_blend_cluster(&mut topo, solid, &cluster).unwrap();
    verify(&topo, restored.solid, "A6 group removed");
    // The exact sharp part: its eight faces on their carriers. (The STEP
    // fixture splits two closed boss circles at the wall's seam; the
    // restoration keeps the boss foot as one arc between the block faces.)
    let (sharp_topo, sharp) = gecko_sharp();
    assert_eq!(solid_entity_counts(&topo, restored.solid).unwrap().0, 8);
    assert_eq!(census(&topo, restored.solid), census(&sharp_topo, sharp));
    let volume = solid_volume(&topo, restored.solid, 1e-3).unwrap();
    let expected = solid_volume(&sharp_topo, sharp, 1e-3).unwrap();
    assert!(
        (volume - expected).abs() <= 1e-9 * expected,
        "{volume} vs {expected}"
    );
}

/// A7 after A6: the boss diameter 12 -> 14.6 -> 17 -> 14.7 chained on one
/// body; each step re-cuts the bottom chamfer round the new foot.
#[test]
fn boss_diameter_edits_carry_the_bottom_chamfer() {
    let (mut topo, filleted) = gecko();
    let mut solid = bottom_chamfer(&mut topo, filleted);
    verify(&topo, solid, "A6");
    for diameter in [14.6, 17.0, 14.7] {
        let label = format!("A7 diameter {diameter} after A6");
        let radius = diameter / 2.0;
        let before = fingerprint(&topo, solid);
        let boss = boss_of(&topo, solid);
        let result = resize_cylindrical_face(&mut topo, solid, boss, radius).unwrap();
        assert_eq!(fingerprint(&topo, solid), before, "{label}: source");
        verify(&topo, result, &label);
        assert_eq!(
            census(&topo, result),
            BTreeMap::from([
                ("cone", 3),
                ("cylinder", 5),
                ("nurbs", 2),
                ("plane", 9),
                ("torus", 1)
            ]),
            "{label}"
        );
        assert_bottom_chamfer(&topo, result, radius, &label);
        assert_boss_wall(&topo, result, radius, &label);
        let (oracle_topo, oracle) = boss_after_bottom_oracle(radius);
        assert_matches_oracle(&topo, result, &oracle_topo, oracle, &label);
        solid = result;
    }
}

/// The group route's plan and history on the chamfered body.
#[test]
fn boss_route_rebuilds_fillets_then_the_chamfer_chain() {
    let (mut topo, filleted) = gecko();
    let solid = bottom_chamfer(&mut topo, filleted);
    let boss = boss_of(&topo, solid);
    let cluster = recognize_blend_cluster(&topo, solid, boss).unwrap();
    let edit = resize_cylinder_through_group(&mut topo, solid, boss, 7.3)
        .unwrap()
        .unwrap();
    assert_eq!(edit.stages.len(), 2, "{:?}", edit.stages);
    assert_eq!(edit.stages[0].blend, StageBlend::Fillet);
    assert!((edit.stages[0].radius - 4.0).abs() < 1e-12);
    assert_eq!(edit.stages[0].sharp_edges.len(), 5);
    assert_eq!(
        edit.stages[1].blend,
        StageBlend::Chamfer {
            tangent_chain: true
        }
    );
    assert!((edit.stages[1].radius - BOTTOM).abs() < 1e-9);
    assert_eq!(edit.stages[1].sharp_edges.len(), 3);
    let outputs: Vec<usize> = solid_faces(&topo, edit.solid)
        .unwrap()
        .into_iter()
        .map(remus_topology::arena::Id::index)
        .collect();
    assert!(
        edit.evolution.accounts_for_result(outputs),
        "{:?}",
        edit.evolution
    );
    assert_eq!(
        edit.evolution.modified.get(&boss.index()),
        Some(&vec![boss_of(&topo, edit.solid).index()])
    );
    for face in cluster.faces() {
        assert!(
            edit.evolution.deleted.contains(&face.index()),
            "face {}",
            face.index()
        );
    }
}

/// Oversized diameters on the chamfered body are typed refusals that leave
/// it byte-identical.
#[test]
fn oversized_boss_after_the_bottom_chamfer_is_refused() {
    let (mut topo, filleted) = gecko();
    let solid = bottom_chamfer(&mut topo, filleted);
    let before = fingerprint(&topo, solid);
    let boss = boss_of(&topo, solid);
    let error = resize_cylindrical_face(&mut topo, solid, boss, 20.0).unwrap_err();
    assert!(
        matches!(&error, OperationsError::Offset(inner) if format!("{inner:?}").starts_with("TopologyChange")),
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
    let error = resize_cylindrical_face(&mut topo, solid, boss, 17.0).unwrap_err();
    assert!(
        matches!(&error, OperationsError::BlendCluster(inner) if inner.code() == "blend-cluster-reblend-failed"),
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
}

// ---------------------------------------------------------------------------
// The cumulative workflow: A2, A3 (W11), A4 / A6, A7.

/// The free-end chamfer distance.
const FREE_END: f64 = 0.7;

/// The single straight edge whose ends both satisfy `on`.
fn line_where(topo: &Topology, solid: SolidId, on: impl Fn(Point3) -> bool, what: &str) -> EdgeId {
    let found = lines_where(topo, solid, on);
    assert_eq!(found.len(), 1, "one {what}");
    found[0]
}

/// The convex top band along the block's x = 0 face (its axis along Y).
fn x0_band(topo: &Topology, solid: SolidId) -> FaceId {
    let cluster = recognize_blend_cluster(topo, solid, boss_of(topo, solid)).unwrap();
    let bands: Vec<FaceId> = cluster
        .members
        .iter()
        .filter(|m| m.convexity == Convexity::Convex)
        .flat_map(|m| m.faces.iter().copied())
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
            if c.axis().cross(Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9)
        })
        .collect();
    assert_eq!(bands.len(), 1, "one convex band along the x = 0 face");
    bands[0]
}

/// The concave seam round on the block's x = 0 face.
fn x0_seam(topo: &Topology, solid: SolidId) -> FaceId {
    let cluster = recognize_blend_cluster(topo, solid, boss_of(topo, solid)).unwrap();
    let seams: Vec<FaceId> = cluster
        .members
        .iter()
        .filter(|m| m.convexity == Convexity::Concave)
        .flat_map(|m| m.faces.iter().copied())
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
            if c.origin().x() < -1.0)
        })
        .collect();
    assert_eq!(seams.len(), 1, "one seam round on the x = 0 face");
    seams[0]
}

/// The A3 pick: the block's x = 0 / y = 18 vertical corner edge.
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

fn edge_length(topo: &Topology, edge: EdgeId) -> f64 {
    let (a, b) = edge_ends(topo, edge);
    (b - a).length()
}

/// The video's steps on one body, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    A2,
    A3,
    A4,
    A6,
}

/// The filleted Gecko taken through `steps` (A2 at `band`), each step
/// verified.
fn workflow(steps: &[Step], band: f64) -> (Topology, SolidId) {
    let (mut topo, mut solid) = gecko();
    for step in steps {
        solid = match step {
            Step::A2 => {
                let face = x0_band(&topo, solid);
                resize_blend(&mut topo, solid, face, 4.0, band)
                    .unwrap()
                    .solid
            }
            Step::A3 => {
                let pick = a3_pick(&topo, solid);
                fillet_cascade(&mut topo, solid, &[pick], 1.0)
                    .unwrap()
                    .solid
            }
            Step::A4 => {
                let pick = a4_pick(&topo, solid);
                chamfer_v2_tangent_chain(&mut topo, solid, &[pick], FREE_END, FREE_END)
                    .unwrap()
                    .solid
            }
            Step::A6 => bottom_chamfer(&mut topo, solid),
        };
        verify(&topo, solid, &format!("{step:?}"));
    }
    (topo, solid)
}

/// Fresh construction of `steps` from the sharp Gecko with its boss at
/// `radius`, in the planner's order: the R4 seams, shoulder and y = 0 top
/// edge in one call, the x = 0 top edge at `band`, the A3 edges at R1, then
/// the chamfer chains.
fn workflow_oracle(steps: &[Step], band: f64, radius: f64) -> (Topology, SolidId) {
    assert_eq!(steps.first(), Some(&Step::A2));
    let (mut topo, sharp) = gecko_sharp();
    let boss = boss_of(&topo, sharp);
    let mut solid = if (radius - 6.0).abs() < 1e-12 {
        sharp
    } else {
        resize_cylindrical_face(&mut topo, sharp, boss, radius).unwrap()
    };
    let edges = sharp_fillet_edges(&topo, solid, radius);
    let x0_top = |p: Point3| p.x().abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7;
    let r4: Vec<EdgeId> = edges
        .iter()
        .copied()
        .filter(|&edge| {
            let (a, b) = edge_ends(&topo, edge);
            !(x0_top(a) && x0_top(b))
        })
        .collect();
    assert_eq!(r4.len(), 4);
    solid = fillet_cascade(&mut topo, solid, &r4, 4.0).unwrap().solid;
    let x0 = line_where(&topo, solid, x0_top, "x = 0 top edge");
    solid = fillet_cascade(&mut topo, solid, &[x0], band).unwrap().solid;
    if steps.contains(&Step::A3) {
        let vertical = a3_pick(&topo, solid);
        let top = line_where(
            &topo,
            solid,
            |p| (p.y() - 18.0).abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7,
            "y = 18 top edge",
        );
        solid = fillet_cascade(&mut topo, solid, &[vertical, top], 1.0)
            .unwrap()
            .solid;
    }
    // Chamfer chains, larger distance first.
    if steps.contains(&Step::A6) {
        solid = bottom_chamfer(&mut topo, solid);
    }
    if steps.contains(&Step::A4) {
        let pick = a4_pick(&topo, solid);
        solid = chamfer_v2_tangent_chain(&mut topo, solid, &[pick], FREE_END, FREE_END)
            .unwrap()
            .solid;
    }
    (topo, solid)
}

/// A7 Ø14.6 -> 17 -> 14.7 chained on the body after `steps`: each result
/// verified, probed and matched against its fresh construction; the source
/// stays byte-identical.
fn boss_sequence_after(steps: &[Step], band: f64) {
    let (mut topo, mut solid) = workflow(steps, band);
    let census_before = census(&topo, solid);
    for diameter in [14.6, 17.0, 14.7] {
        let label = format!("A7 diameter {diameter} after {steps:?} (A2 R{band})");
        let radius = diameter / 2.0;
        let before = fingerprint(&topo, solid);
        let boss = boss_of(&topo, solid);
        let result = resize_cylindrical_face(&mut topo, solid, boss, radius).unwrap();
        assert_eq!(fingerprint(&topo, solid), before, "{label}: source");
        verify(&topo, result, &label);
        assert_eq!(census(&topo, result), census_before, "{label}: census");
        assert_boss_wall(&topo, result, radius, &label);
        if steps.contains(&Step::A6) {
            assert_bottom_chamfer(&topo, result, radius, &label);
        }
        let (oracle_topo, oracle) = workflow_oracle(steps, band, radius);
        assert_matches_oracle(&topo, result, &oracle_topo, oracle, &label);
        solid = result;
    }
}

/// A7 after A2: the edited band's walked transition into its seam joins the
/// group (as part of the band it continues), so the boss still resizes.
#[test]
fn boss_diameter_sequence_after_a2() {
    boss_sequence_after(&[Step::A2], 1.9);
}

/// A3 on the post-A2 body: the picked edge is `24 - r` long (the video's
/// 22.5 at R1.5), its round turns through the band's end arc onto the y = 18
/// top edge, and the result is the planner's order: the same as the group
/// route builds.
#[test]
fn a3_round_turns_through_the_a2_band() {
    for band in [1.9, 1.5] {
        let label = format!("A3 after A2 R{band}");
        let (mut topo, a2) = workflow(&[Step::A2], band);
        let pick = a3_pick(&topo, a2);
        assert!(
            (edge_length(&topo, pick) - (24.0 - band)).abs() < 1e-9,
            "{label}"
        );
        if (band - 1.5).abs() < 1e-12 {
            assert!(
                (edge_length(&topo, pick) - 22.5).abs() < 1e-9,
                "the video's pill"
            );
        }
        let before = fingerprint(&topo, a2);
        let direct = fillet_cascade(&mut topo, a2, &[pick], 1.0).unwrap();
        assert_eq!(fingerprint(&topo, a2), before);
        verify(&topo, direct.solid, &label);
        // Two exact R1 cylinders and the turn over the band's end arc (a
        // spindle-torus patch, R1 inside R`band`, so lofted).
        let mut expected = census(&topo, a2);
        *expected.entry("cylinder").or_insert(0) += 2;
        *expected.entry("nurbs").or_insert(0) += 1;
        assert_eq!(census(&topo, direct.solid), expected, "{label}");
        for (point, inside) in [
            (Point3::new(0.15, 17.85, 12.0), false),
            (Point3::new(0.5, 17.5, 12.0), true),
            (Point3::new(15.0, 17.85, 23.85), false),
            (Point3::new(15.0, 17.5, 23.5), true),
        ] {
            let expected = if inside {
                PointClassification::Inside
            } else {
                PointClassification::Outside
            };
            assert_eq!(
                classify(&topo, direct.solid, point),
                expected,
                "{label}: {point:?}"
            );
        }
        let routed = fillet_through_blend_group(&mut topo, a2, &[pick], 1.0)
            .unwrap()
            .unwrap();
        assert_eq!(fingerprint(&topo, a2), before);
        verify(&topo, routed.solid, &format!("{label} through the group"));
        let radii: Vec<f64> = routed.stages.iter().map(|stage| stage.radius).collect();
        assert_eq!(radii, vec![4.0, band, 1.0], "{label}: planner's order");
        assert_matches_oracle(&topo, routed.solid, &topo, direct.solid, &label);
        let (oracle_topo, oracle) = workflow_oracle(&[Step::A2, Step::A3], band, 6.0);
        assert_matches_oracle(&topo, direct.solid, &oracle_topo, oracle, &label);
    }
}

/// W11: a round larger than the band its chain crosses. No engine rolls an
/// R2 ball along the R1.9 band, and the band ends on a sampled transition, so
/// it cannot be taken off alone; the group route removes the whole network
/// and re-fillets it with the new edge as one more stage, largest first.
#[test]
fn a_round_larger_than_the_band_it_crosses_builds_through_the_group() {
    let (mut topo, a2) = workflow(&[Step::A2], 1.9);
    let pick = a3_pick(&topo, a2);
    let before = fingerprint(&topo, a2);
    let result = fillet_cascade(&mut topo, a2, &[pick], 2.0).unwrap();
    assert_eq!(fingerprint(&topo, a2), before);
    assert!(result.face_origins.is_none(), "rebuilt through the group");
    verify(&topo, result.solid, "R2 after A2");
    assert_eq!(
        census(&topo, result.solid),
        BTreeMap::from([("cylinder", 6), ("nurbs", 2), ("plane", 7), ("torus", 1)])
    );
    // The R2 round removes the corner; the R1.9 band still rounds the x = 0
    // top edge.
    assert_eq!(
        classify(&topo, result.solid, Point3::new(0.3, 17.7, 12.0)),
        PointClassification::Outside
    );
    assert_eq!(
        classify(&topo, result.solid, Point3::new(0.2, 12.0, 23.8)),
        PointClassification::Outside
    );
    let routed = fillet_through_blend_group(&mut topo, a2, &[pick], 2.0)
        .unwrap()
        .unwrap();
    let radii: Vec<f64> = routed.stages.iter().map(|stage| stage.radius).collect();
    assert_eq!(radii, vec![4.0, 2.0, 1.9], "largest first");
    // Oracle: the sharp part, R4, the corner edge at R2, the x = 0 top edge
    // at R1.9.
    let (mut oracle_topo, sharp) = gecko_sharp();
    let edges = sharp_fillet_edges(&oracle_topo, sharp, 6.0);
    let x0_top = |p: Point3| p.x().abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7;
    let r4: Vec<EdgeId> = edges
        .iter()
        .copied()
        .filter(|&edge| {
            let (a, b) = edge_ends(&oracle_topo, edge);
            !(x0_top(a) && x0_top(b))
        })
        .collect();
    let mut oracle = fillet_cascade(&mut oracle_topo, sharp, &r4, 4.0)
        .unwrap()
        .solid;
    let corner = a3_pick(&oracle_topo, oracle);
    oracle = fillet_cascade(&mut oracle_topo, oracle, &[corner], 2.0)
        .unwrap()
        .solid;
    let x0 = line_where(&oracle_topo, oracle, x0_top, "x = 0 top edge");
    oracle = fillet_cascade(&mut oracle_topo, oracle, &[x0], 1.9)
        .unwrap()
        .solid;
    assert_matches_oracle(&topo, result.solid, &oracle_topo, oracle, "R2 after A2");
}

/// A2 -> A3 -> A6 -> A7 Ø14.6 -> 17 -> 14.7.
#[test]
fn cumulative_a2_a3_a6_then_boss_diameters() {
    boss_sequence_after(&[Step::A2, Step::A3, Step::A6], 1.9);
}

/// A2 -> A3 -> A4 -> A7 Ø14.6 -> 17 -> 14.7.
#[test]
fn cumulative_a2_a3_a4_then_boss_diameters() {
    boss_sequence_after(&[Step::A2, Step::A3, Step::A4], 1.9);
}

/// Steps with no construction are typed refusals that leave the cumulative
/// body byte-identical: the concave seam R4 -> 2 under the R4 shoulder,
/// and oversized boss diameters.
#[test]
fn unbuildable_cumulative_steps_are_refused() {
    let (mut topo, solid) = workflow(&[Step::A2, Step::A3, Step::A6], 1.9);
    let before = fingerprint(&topo, solid);
    let seam = x0_seam(&topo, solid);
    let error = resize_blend(&mut topo, solid, seam, 4.0, 2.0).unwrap_err();
    assert_eq!(
        resize_blend_failure_code(&error),
        "blend-cluster-reblend-failed",
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
    let boss = boss_of(&topo, solid);
    let error = resize_cylindrical_face(&mut topo, solid, boss, 20.0).unwrap_err();
    assert!(
        matches!(&error, OperationsError::Offset(inner) if format!("{inner:?}").starts_with("TopologyChange")),
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
    let error = resize_cylindrical_face(&mut topo, solid, boss, 17.0).unwrap_err();
    assert!(
        matches!(&error, OperationsError::BlendCluster(inner) if inner.code() == "blend-cluster-reblend-failed"),
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
}

// ---------------------------------------------------------------------------
// W9b: the free-end and bottom chamfer chains on one body.

/// Material both chamfers would remove at one block corner where an `a`
/// chamfer up a vertical edge meets a `b` chamfer along a bottom edge (the
/// corner tetrahedron-like wedge counted twice by the two chains):
/// `∫₀^min (a − y)(b − y) dy` with `min = min(a, b)`.
fn corner_overlap(a: f64, b: f64) -> f64 {
    let (small, large) = if a <= b { (a, b) } else { (b, a) };
    small * small * large / 2.0 - small.powi(3) / 6.0
}

/// A4 and A6 on one body, in either order. Each chain's run ends at a block
/// corner of the free end on the other's 45° stripe: the free-end band on
/// the bottom stripe, wholly; the wider bottom band crosses the free-end
/// stripe and runs on into the free end face. Both orders give one body:
/// the union of the two chamfers, so the removed volume is each chain's own
/// less the two corner wedges both would take.
#[test]
fn free_end_and_bottom_chamfers_meet_in_either_order() {
    let (topo_a3, a3) = workflow(&[Step::A2, Step::A3], 1.9);
    let (topo_a4, a4) = workflow(&[Step::A2, Step::A3, Step::A4], 1.9);
    let (topo_a6, a6) = workflow(&[Step::A2, Step::A3, Step::A6], 1.9);
    let volume = |topo: &Topology, solid| solid_volume(topo, solid, 1e-3).unwrap();
    let expected = volume(&topo_a4, a4) + volume(&topo_a6, a6) - volume(&topo_a3, a3)
        + 2.0 * corner_overlap(FREE_END, BOTTOM);
    let mut bodies = Vec::new();
    for steps in [
        [Step::A2, Step::A3, Step::A4, Step::A6],
        [Step::A2, Step::A3, Step::A6, Step::A4],
    ] {
        let label = format!("{steps:?}");
        let (mut topo, solid) = workflow(&steps, 1.9);
        let measured = volume(&topo, solid);
        assert!(
            (measured - expected).abs() <= 1e-9 * expected,
            "{label}: volume {measured} vs closed form {expected}"
        );
        // Five more faces than A4 alone (the bottom chain's planes and cones)
        // and every one of them exact; the free-end corner closes on planes.
        let mut grown = census(&topo_a4, a4);
        *grown.entry("plane").or_insert(0) += 3;
        *grown.entry("cone").or_insert(0) += 4;
        assert_eq!(census(&topo, solid), grown, "{label}");
        for (point, expected, what) in [
            (
                Point3::new(29.95, 0.75, 0.02),
                PointClassification::Outside,
                "bottom band past the free-end stripe",
            ),
            (
                Point3::new(29.99, 0.85, 0.01),
                PointClassification::Inside,
                "free end just above the bottom band",
            ),
            (
                Point3::new(29.6, 0.1, 0.9),
                PointClassification::Outside,
                "free-end stripe above the bottom band",
            ),
            (
                Point3::new(29.5, 0.5, 0.5),
                PointClassification::Inside,
                "inside both bands' corner",
            ),
            (
                Point3::new(29.95, 17.25, 0.02),
                PointClassification::Outside,
                "far corner, bottom band",
            ),
            (
                Point3::new(29.5, 17.5, 0.5),
                PointClassification::Inside,
                "inside the far corner",
            ),
        ] {
            assert_eq!(classify(&topo, solid, point), expected, "{label}: {what}");
        }
        // The boss group holds both chains (the free-end stripes end on the
        // bottom stripes, which stay chamfers with crossing ends) and comes
        // off whole to the exact sharp part.
        let cluster = recognize_blend_cluster(&topo, solid, boss_of(&topo, solid)).unwrap();
        let chamfers: Vec<f64> = cluster
            .members
            .iter()
            .filter(|m| m.kind.is_chamfer())
            .map(|m| m.radius)
            .collect();
        assert_eq!(chamfers.len(), 12, "{label}: {chamfers:?}");
        assert_eq!(
            chamfers
                .iter()
                .filter(|&&d| (d - BOTTOM).abs() < 1e-9)
                .count(),
            7,
            "{label}"
        );

        let restored = remove_blend_cluster(&mut topo, solid, &cluster).unwrap();
        let (sharp_topo, sharp) = gecko_sharp();
        assert_eq!(census(&topo, restored.solid), census(&sharp_topo, sharp));
        let restored_volume = solid_volume(&topo, restored.solid, 1e-3).unwrap();
        let sharp_volume = solid_volume(&sharp_topo, sharp, 1e-3).unwrap();
        assert!(
            (restored_volume - sharp_volume).abs() <= 1e-9 * sharp_volume,
            "{label}: removed group {restored_volume} vs sharp {sharp_volume}"
        );
        // Re-blend plans: the chains larger distance first, then the same
        // fillet plans with the chains reversed.
        let removed: Vec<usize> = (0..cluster.members.len()).collect();
        let plans = plan_reblend(&cluster, &removed, &BTreeMap::new()).unwrap();
        let chain_order = |plan: &[ReblendStage]| -> Vec<f64> {
            plan.iter()
                .filter(|stage| matches!(stage.blend, StageBlend::Chamfer { .. }))
                .map(|stage| stage.radius)
                .collect()
        };
        let orders: Vec<Vec<f64>> = plans.iter().map(|plan| chain_order(plan)).collect();
        assert_eq!(orders.len() % 2, 0, "{label}: {orders:?}");
        let half = orders.len() / 2;
        for order in &orders[..half] {
            assert!((order[0] - BOTTOM).abs() < 1e-9 && (order[1] - FREE_END).abs() < 1e-9);
        }
        for order in &orders[half..] {
            assert!((order[0] - FREE_END).abs() < 1e-9 && (order[1] - BOTTOM).abs() < 1e-9);
        }
        bodies.push((topo, solid, label));
    }
    let [(topo_a, a, label_a), (topo_b, b, _)] = bodies.as_slice() else {
        unreachable!()
    };
    assert_eq!(
        solid_entity_counts(topo_a, *a).unwrap(),
        solid_entity_counts(topo_b, *b).unwrap(),
        "{label_a}: both orders"
    );
}

/// A2 -> A3 -> A4 -> A6 -> A7 Ø14.6 -> 17 -> 14.7: the group holds both
/// chamfer chains and re-cuts them after every diameter.
#[test]
fn cumulative_a2_a3_a4_a6_then_boss_diameters() {
    boss_sequence_after(&[Step::A2, Step::A3, Step::A4, Step::A6], 1.9);
}

/// The same with the chamfers made in the other order.
#[test]
fn cumulative_a2_a3_a6_a4_then_boss_diameters() {
    boss_sequence_after(&[Step::A2, Step::A3, Step::A6, Step::A4], 1.9);
}
