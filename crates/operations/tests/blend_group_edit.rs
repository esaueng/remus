//! Direct edits through a recognized blend group on the filleted
//! Gentle-Gecko part: recognize the group, remove it to sharp, edit the sharp
//! body, re-fillet in planned order.
//!
//! Gentle-Gecko is a 30 x 18 x 24 block with an R6 boss (axis +Z, height 36)
//! on its x = 0, y = 0 corner edge. The filleted part rounds the two
//! concave boss seams, the concave shoulder and the two convex block top
//! edges that run into them at R4, with two sampled corner patches where
//! they meet. Neither edit below has a per-band path on it: `resize_blend`
//! refuses the convex bands (they end on the sampled corners) and the
//! partial-turn wall resize refuses the boss (its neighbours are sampled).
//!
//! - **A2 convex half** (`resize_blend`): each longitudinal top band
//!   R4 -> 1.5 and R4 -> 1.9, the seams, shoulder and the other band staying
//!   R4.
//! - **A2 concave half, out of scope**: a seam R4 -> 2 under an R4 shoulder
//!   has no construction (the W3 spike); it must be a typed refusal that
//!   leaves the body byte-identical.
//! - **A7** (`resize_cylindrical_face`): the boss diameter 12 -> 14.6 -> 17
//!   -> 21.1 -> 14.7 chained on one body; oversized diameters are typed
//!   refusals with the body unchanged.
//!
//! The oracle for every edit is a fresh `fillet_cascade` of the edited sharp
//! body (`gentle_gecko_sharp.step`, exactly resized where the edit is a
//! diameter) with the same radii, in the planner's stage order: the result
//! must match it in entity counts, face census and volume (1e-6 relative).
//! Every result also meets the full solid-verification bar.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::blend_cluster::{
    Convexity, recognize_blend_cluster, resize_band_through_group, resize_cylinder_through_group,
};
use remus_operations::blend_ops::fillet_cascade;
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

/// Which convex top band: the one along the y = 0 face (running along X) or
/// the one along the x = 0 face (running along Y).
#[derive(Clone, Copy, Debug)]
enum Side {
    AlongX,
    AlongY,
}

impl Side {
    /// Unit direction of the band's sharp edge.
    fn direction(self) -> Vec3 {
        match self {
            Self::AlongX => Vec3::new(1.0, 0.0, 0.0),
            Self::AlongY => Vec3::new(0.0, 1.0, 0.0),
        }
    }

    /// A station along the side's top edge, clear of both of its ends.
    const fn station(self) -> f64 {
        match self {
            Self::AlongX => 20.0,
            Self::AlongY => 14.0,
        }
    }

    /// A point `inward` into the block from the sharp top edge and `down`
    /// below the top, at `along` on the edge.
    fn probe(self, along: f64, inward: f64, down: f64) -> Point3 {
        match self {
            Self::AlongX => Point3::new(along, inward, 24.0 - down),
            Self::AlongY => Point3::new(inward, along, 24.0 - down),
        }
    }

    /// Whether a straight sharp edge of the top (z = 24) lies on this side's
    /// block face.
    fn holds(self, a: Point3, b: Point3) -> bool {
        let on = |p: Point3| {
            (p.z() - 24.0).abs() < 1e-7
                && match self {
                    Self::AlongX => p.y().abs() < 1e-7,
                    Self::AlongY => p.x().abs() < 1e-7,
                }
        };
        on(a) && on(b)
    }
}

/// The convex band face on `side`.
fn convex_band(topo: &Topology, solid: SolidId, side: Side) -> FaceId {
    let cluster = recognize_blend_cluster(topo, solid, boss_of(topo, solid)).unwrap();
    let bands: Vec<FaceId> = cluster
        .members
        .iter()
        .filter(|member| member.convexity == Convexity::Convex)
        .flat_map(|member| member.faces.iter().copied())
        .filter(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(c) => c.axis().cross(side.direction()).length() < 1e-9,
            _ => false,
        })
        .collect();
    assert_eq!(bands.len(), 1, "one convex band along {side:?}");
    bands[0]
}

/// The concave seam band on the block face of `side`.
fn seam_band(topo: &Topology, solid: SolidId, side: Side) -> FaceId {
    let cluster = recognize_blend_cluster(topo, solid, boss_of(topo, solid)).unwrap();
    let seams: Vec<FaceId> = cluster
        .members
        .iter()
        .filter(|member| member.convexity == Convexity::Concave)
        .flat_map(|member| member.faces.iter().copied())
        .filter(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(c) => {
                let o = c.origin();
                match side {
                    Side::AlongX => o.y() < -1.0,
                    Side::AlongY => o.x() < -1.0,
                }
            }
            _ => false,
        })
        .collect();
    assert_eq!(seams.len(), 1, "one seam band on the {side:?} face");
    seams[0]
}

fn edge_ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

/// The five sharp edges of a (resized) sharp Gecko with boss radius `r`:
/// the two vertical seams, the shoulder arc, and the two convex top edges.
struct SharpEdges {
    seams: Vec<EdgeId>,
    shoulder: EdgeId,
    along_x: EdgeId,
    along_y: EdgeId,
}

fn sharp_edges(topo: &Topology, solid: SolidId, r: f64) -> SharpEdges {
    let mut seams = Vec::new();
    let mut shoulder = Vec::new();
    let mut along_x = Vec::new();
    let mut along_y = Vec::new();
    for edge in solid_edges(topo, solid).unwrap() {
        let (a, b) = edge_ends(topo, edge);
        match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Circle(circle) => {
                if (circle.radius() - r).abs() < 1e-7 && (circle.center().z() - 24.0).abs() < 1e-7 {
                    shoulder.push(edge);
                }
            }
            EdgeCurve::Line => {
                let vertical = (a.x() - b.x()).abs() < 1e-9 && (a.y() - b.y()).abs() < 1e-9;
                let on_wall = (a.x().hypot(a.y()) - r).abs() < 1e-7;
                if vertical
                    && on_wall
                    && a.z().min(b.z()) < 1e-9
                    && (a.z().max(b.z()) - 24.0).abs() < 1e-7
                {
                    seams.push(edge);
                } else if Side::AlongX.holds(a, b) && a.x().min(b.x()) > 1e-7 {
                    along_x.push(edge);
                } else if Side::AlongY.holds(a, b) && a.y().min(b.y()) > 1e-7 {
                    along_y.push(edge);
                }
            }
            _ => {}
        }
    }
    assert_eq!(seams.len(), 2, "two seams");
    assert_eq!(shoulder.len(), 1, "one shoulder arc");
    assert_eq!(along_x.len(), 1, "one top edge along X");
    assert_eq!(along_y.len(), 1, "one top edge along Y");
    SharpEdges {
        seams,
        shoulder: shoulder[0],
        along_x: along_x[0],
        along_y: along_y[0],
    }
}

/// The straight convex top edge on `side` of a partly filleted body.
fn top_edge(topo: &Topology, solid: SolidId, side: Side) -> EdgeId {
    let found: Vec<EdgeId> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line) && {
                let (a, b) = edge_ends(topo, edge);
                side.holds(a, b)
            }
        })
        .collect();
    assert_eq!(found.len(), 1, "one remaining top edge along {side:?}");
    found[0]
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

/// PLAN.md's bar: both validators clean, no free edges, a watertight
/// manifold mesh at two deflections at or below `bbox_diag * 5e-5`, and the
/// API volume within 1e-4 of the independent mesh volume. Returns the API
/// volume.
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

/// (planes, cylinders, tori, other) faces.
fn census(topo: &Topology, solid: SolidId) -> (usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0);
    for face in solid_faces(topo, solid).unwrap() {
        match topo.face(face).unwrap().surface() {
            FaceSurface::Plane { .. } => counts.0 += 1,
            FaceSurface::Cylinder(_) => counts.1 += 1,
            FaceSurface::Torus(_) => counts.2 += 1,
            _ => counts.3 += 1,
        }
    }
    counts
}

/// Radii of the exact cylinders whose axis is parallel to `direction`,
/// ascending.
fn cylinder_radii(topo: &Topology, solid: SolidId, direction: Vec3) -> Vec<f64> {
    let mut radii: Vec<f64> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(c) if c.axis().cross(direction).length() < 1e-9 => {
                Some(c.radius())
            }
            _ => None,
        })
        .collect();
    radii.compact_sort_by(f64::total_cmp);
    radii
}

/// (major, minor) radii of every torus.
fn tori(topo: &Topology, solid: SolidId) -> Vec<(f64, f64)> {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Torus(t) => Some((t.major_radius(), t.minor_radius())),
            _ => None,
        })
        .collect()
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

fn fingerprint(topo: &Topology, solid: SolidId) -> (String, usize) {
    (
        remus_io::step::writer::write_step(topo, &[solid]).unwrap(),
        topo.allocated_slot_count(),
    )
}

// ---------------------------------------------------------------------------
// A2, convex half.

/// The oracle for a convex band edit: the sharp Gecko, the seams, shoulder
/// and the other top edge at R4 in one call, then the edited top edge at
/// `radius` into the seam band it runs into.
fn convex_oracle(side: Side, radius: f64) -> (Topology, SolidId) {
    let (mut topo, sharp) = gecko_sharp();
    let edges = sharp_edges(&topo, sharp, 6.0);
    let other = match side {
        Side::AlongX => edges.along_y,
        Side::AlongY => edges.along_x,
    };
    let mut first = edges.seams.clone();
    first.extend([edges.shoulder, other]);
    let staged = fillet_cascade(&mut topo, sharp, &first, 4.0).unwrap().solid;
    let edited = top_edge(&topo, staged, side);
    let solid = fillet_cascade(&mut topo, staged, &[edited], radius)
        .unwrap()
        .solid;
    (topo, solid)
}

#[test]
fn convex_top_band_radius_edits_rebuild_the_group() {
    for side in [Side::AlongX, Side::AlongY] {
        let other = match side {
            Side::AlongX => Side::AlongY,
            Side::AlongY => Side::AlongX,
        };
        for radius in [1.5, 1.9] {
            let label = format!("{side:?} R4 -> R{radius}");
            let (mut topo, solid) = gecko();
            let source = fingerprint(&topo, solid);
            let band = convex_band(&topo, solid, side);
            // Material the R4 band removed and the smaller band keeps.
            let regained = side.probe(side.station(), 0.5, 1.5);
            assert_eq!(
                classify(&topo, solid, regained),
                PointClassification::Outside
            );

            let result = resize_blend(&mut topo, solid, band, 4.0, radius).unwrap();
            assert_eq!(fingerprint(&topo, solid).0, source.0, "{label}: source");
            verify(&topo, result.solid, &label);

            // Exact census: 7 planes; the boss, two seams and both top bands
            // as cylinders; the shoulder torus; the remaining corner patch
            // and the edited band's walked transition into its seam.
            assert_eq!(census(&topo, result.solid), (7, 5, 1, 2), "{label}");
            assert_eq!(
                cylinder_radii(&topo, result.solid, side.direction()),
                vec![radius],
                "{label}: the edited band's straight run is an exact R{radius} cylinder"
            );
            assert_eq!(
                cylinder_radii(&topo, result.solid, other.direction()),
                vec![4.0],
                "{label}: the other band stays R4"
            );
            assert_eq!(
                cylinder_radii(&topo, result.solid, Vec3::new(0.0, 0.0, 1.0)),
                vec![4.0, 4.0, 6.0],
                "{label}: seams stay R4 on the R6 boss"
            );
            assert_eq!(tori(&topo, result.solid), vec![(10.0, 4.0)], "{label}");

            // Probes: the edited corner keeps material it lost at R4; the
            // other band's corner is still rounded at R4; the seams and
            // shoulder are unchanged.
            assert_eq!(
                classify(&topo, result.solid, regained),
                PointClassification::Inside,
                "{label}: regained corner"
            );
            assert_eq!(
                classify(&topo, result.solid, side.probe(side.station(), 0.2, 0.2)),
                PointClassification::Outside,
                "{label}: the R{radius} round still removes the very corner"
            );
            assert_eq!(
                classify(&topo, result.solid, other.probe(other.station(), 0.5, 1.5)),
                PointClassification::Outside,
                "{label}: the other band is still R4"
            );
            assert_eq!(
                classify(&topo, result.solid, Point3::new(-0.5, -0.5, 30.0)),
                PointClassification::Inside,
                "{label}: boss"
            );

            let (oracle_topo, oracle) = convex_oracle(side, radius);
            assert_matches_oracle(&topo, result.solid, &oracle_topo, oracle, &label);

            // History: the edited band has a unique successor inferred from
            // its recognized supports; every result face is accounted for. The ridge walk
            // reports no face origins, so this map is a geometric inference
            // (and says so).
            assert_eq!(
                result.evolution.modified.get(&band.index()).map(Vec::len),
                Some(1),
                "{label}"
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
}

// ---------------------------------------------------------------------------
// A2, concave half: out of scope, refused.

#[test]
fn concave_seam_radius_edit_is_refused_and_leaves_the_body_alone() {
    for side in [Side::AlongX, Side::AlongY] {
        let (mut topo, solid) = gecko();
        let before = fingerprint(&topo, solid);
        let seam = seam_band(&topo, solid, side);
        let error = resize_blend(&mut topo, solid, seam, 4.0, 2.0).unwrap_err();
        assert_eq!(
            resize_blend_failure_code(&error),
            "blend-cluster-reblend-failed",
            "{side:?}: {error}"
        );
        // The body is byte-identical. (The per-band attempt `resize_blend`
        // makes first retires handle slots inside its rolled-back
        // transaction, as it does without the group route.)
        assert_eq!(fingerprint(&topo, solid).0, before.0, "{side:?}");
        // The group route itself runs on a private arena: not even a slot.
        let slots = topo.allocated_slot_count();
        let error = resize_band_through_group(&mut topo, solid, seam, 4.0, 2.0).unwrap_err();
        assert!(
            matches!(&error, OperationsError::BlendCluster(inner) if inner.code() == "blend-cluster-reblend-failed"),
            "{error}"
        );
        assert_eq!(topo.allocated_slot_count(), slots, "{side:?}");
        assert_eq!(fingerprint(&topo, solid).0, before.0, "{side:?}");
    }
}

#[test]
fn malformed_band_edits_keep_their_own_refusals() {
    let (mut topo, solid) = gecko();
    let before = fingerprint(&topo, solid);
    let band = convex_band(&topo, solid, Side::AlongX);
    // A wrong radius witness and a negative radius never reach the group.
    let mismatch = resize_blend(&mut topo, solid, band, 3.0, 1.5).unwrap_err();
    assert_eq!(
        resize_blend_failure_code(&mismatch),
        "blend-radius-mismatch"
    );
    assert!(resize_blend(&mut topo, solid, band, 4.0, -1.0).is_err());
    assert_eq!(fingerprint(&topo, solid), before);
}

// ---------------------------------------------------------------------------
// A7.

/// The oracle for a boss diameter edit: the sharp Gecko resized exactly,
/// then all five edges at R4 in one call (the group's only stage plan).
fn boss_oracle(radius: f64) -> (Topology, SolidId) {
    let (mut topo, sharp) = gecko_sharp();
    let boss = boss_of(&topo, sharp);
    let resized = resize_cylindrical_face(&mut topo, sharp, boss, radius).unwrap();
    let edges = sharp_edges(&topo, resized, radius);
    let mut all = edges.seams.clone();
    all.extend([edges.shoulder, edges.along_x, edges.along_y]);
    let solid = fillet_cascade(&mut topo, resized, &all, 4.0).unwrap().solid;
    (topo, solid)
}

fn assert_boss_result(topo: &Topology, solid: SolidId, radius: f64, label: &str) {
    verify(topo, solid, label);
    assert_eq!(census(topo, solid), (7, 5, 1, 2), "{label}");
    assert_eq!(
        cylinder_radii(topo, solid, Vec3::new(0.0, 0.0, 1.0)),
        vec![4.0, 4.0, radius],
        "{label}: boss and seams"
    );
    for side in [Side::AlongX, Side::AlongY] {
        assert_eq!(
            cylinder_radii(topo, solid, side.direction()),
            vec![4.0],
            "{label}: top band along {side:?}"
        );
    }
    let shoulder = tori(topo, solid);
    assert_eq!(shoulder.len(), 1, "{label}");
    assert!(
        (shoulder[0].0 - (radius + 4.0)).abs() < 1e-9 && (shoulder[0].1 - 4.0).abs() < 1e-12,
        "{label}: shoulder {shoulder:?}"
    );
    // Across the new wall, above the shoulder and outside the block.
    for angle in [200.0f64, 225.0, 250.0] {
        let (sin, cos) = angle.to_radians().sin_cos();
        for (offset, expected) in [
            (-0.05, PointClassification::Inside),
            (0.05, PointClassification::Outside),
        ] {
            let at = Point3::new((radius + offset) * cos, (radius + offset) * sin, 32.0);
            assert_eq!(
                classify(topo, solid, at),
                expected,
                "{label}: wall at {angle}"
            );
        }
    }
    // The cap and the block corner far from the boss are untouched.
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

#[test]
fn boss_diameter_sequence_rebuilds_the_group() {
    let (mut topo, mut solid) = gecko();
    let mut previous = fingerprint(&topo, solid);
    for diameter in [14.6, 17.0, 21.1, 14.7] {
        let label = format!("diameter {diameter}");
        let radius = diameter / 2.0;
        let boss = boss_of(&topo, solid);
        let result = resize_cylindrical_face(&mut topo, solid, boss, radius).unwrap();
        assert_eq!(fingerprint(&topo, solid).0, previous.0, "{label}: source");
        assert_boss_result(&topo, result, radius, &label);
        let (oracle_topo, oracle) = boss_oracle(radius);
        assert_matches_oracle(&topo, result, &oracle_topo, oracle, &label);
        solid = result;
        previous = fingerprint(&topo, solid);
    }
}

/// The group route's own result: the plan (one five-edge R4 call), and a
/// inferred face history that accounts for every result face and carries
/// the boss wall and uniquely recognized blends to their successors.
#[test]
fn boss_group_route_reports_its_plan_and_history() {
    let (mut topo, solid) = gecko();
    let boss = boss_of(&topo, solid);
    let cluster = recognize_blend_cluster(&topo, solid, boss).unwrap();
    let edit = resize_cylinder_through_group(&mut topo, solid, boss, 7.3)
        .unwrap()
        .unwrap();
    assert_eq!(edit.stages.len(), 1);
    assert!((edit.stages[0].radius - 4.0).abs() < 1e-12);
    assert_eq!(edit.stages[0].sharp_edges, vec![0, 1, 2, 3, 4]);
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
    let new_boss = boss_of(&topo, edit.solid);
    assert_eq!(
        edit.evolution.modified.get(&boss.index()),
        Some(&vec![new_boss.index()])
    );
    for face in cluster.faces() {
        assert_eq!(
            edit.evolution.modified.get(&face.index()).map(Vec::len),
            Some(1)
        );
    }
}

#[test]
fn oversized_boss_diameters_are_refused_and_leave_the_body_alone() {
    let (mut topo, solid) = gecko();
    let before = fingerprint(&topo, solid);
    let boss = boss_of(&topo, solid);
    // Diameter 40 meets the block's back face y = 18: the exact sharp resize
    // refuses the topology change.
    let error = resize_cylindrical_face(&mut topo, solid, boss, 20.0).unwrap_err();
    assert!(
        matches!(&error, OperationsError::BlendCluster(inner) if inner.code() == "cylinder-diameter-unbuildable"),
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
    // Diameter 34 still resizes sharp, but leaves a 1 mm top edge on the
    // x = 0 face that no R4 round fits: the re-fillet refuses.
    let error = resize_cylindrical_face(&mut topo, solid, boss, 17.0).unwrap_err();
    assert!(
        matches!(&error, OperationsError::BlendCluster(inner) if inner.code() == "cylinder-diameter-unbuildable"),
        "{error}"
    );
    assert_eq!(fingerprint(&topo, solid), before);
    for radius in [20.0, 17.0] {
        assert!(resize_cylinder_through_group(&mut topo, solid, boss, radius).is_err());
        assert_eq!(fingerprint(&topo, solid), before, "group route at {radius}");
    }
}

/// PLAN.md's A0 reference body: the filleted Gecko plus an R1 cap rim. The
/// boss seeds an eight-member group (the rim too), rebuilt last.
#[test]
fn boss_diameter_edit_carries_the_cap_rim() {
    let (mut topo, filleted) = gecko();
    let rim: Vec<EdgeId> = solid_edges(&topo, filleted)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(c)
                if (c.center().z() - 36.0).abs() < 1e-9)
        })
        .collect();
    assert!(!rim.is_empty());
    let solid = fillet_cascade(&mut topo, filleted, &rim, 1.0)
        .unwrap()
        .solid;
    let cluster = recognize_blend_cluster(&topo, solid, boss_of(&topo, solid)).unwrap();
    assert_eq!(cluster.members.len(), 8);
    let before = fingerprint(&topo, solid);
    let radius = 7.3;
    let boss = boss_of(&topo, solid);
    let result = resize_cylindrical_face(&mut topo, solid, boss, radius).unwrap();
    assert_eq!(fingerprint(&topo, solid).0, before.0);
    let label = "rimmed diameter 14.6";
    verify(&topo, result, label);
    let mut rims = tori(&topo, result);
    rims.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
    assert_eq!(rims.len(), 2, "{label}: {rims:?}");
    assert!((rims[0].0 - (radius - 1.0)).abs() < 1e-9 && (rims[0].1 - 1.0).abs() < 1e-12);
    assert!((rims[1].0 - (radius + 4.0)).abs() < 1e-9 && (rims[1].1 - 4.0).abs() < 1e-12);

    // Oracle: the resized sharp body, the five edges at R4, then the rim.
    let (mut oracle_topo, oracle) = boss_oracle(radius);
    let cap_rim: Vec<EdgeId> = solid_edges(&oracle_topo, oracle)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            matches!(oracle_topo.edge(edge).unwrap().curve(), EdgeCurve::Circle(c)
                if (c.center().z() - 36.0).abs() < 1e-9)
        })
        .collect();
    let oracle = fillet_cascade(&mut oracle_topo, oracle, &cap_rim, 1.0)
        .unwrap()
        .solid;
    assert_matches_oracle(&topo, result, &oracle_topo, oracle, label);
}
