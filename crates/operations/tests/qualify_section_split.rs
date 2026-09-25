//! Qualification matrix for planar sections and plane splits of box-derived
//! solids, including internal cavities and through-holes (B6 in
//! `docs/kernel-maturity/roadmap.md`).
//!
//! Axes covered: fixture (plain box, box with one fully enclosed box cavity,
//! plate with one rectangular through-hole), scale (`1e-3 / 1 / 1e3`),
//! placement (origin, rigid rotation + translation), cut position (through
//! material, through cavity/hole, wholly outside, coincident with a face,
//! through a vertex/edge), plane-normal sense (forward, reversed), and repeat
//! determinism.
//!
//! Contracts under test (declared before running; see the probe notes in the
//! PR description for the starting-SHA reproduction):
//!
//! * `section` returns an empty face list as a successful empty result when
//!   the plane misses the solid entirely. A plane coincident with a face
//!   returns that face's outline (1 face). A plane that only touches the
//!   solid at a single vertex returns an empty face list; a plane through an
//!   existing edge has no closed wire and returns `InvalidInput`
//!   ("no closed cross-section could be assembled").
//! * `split` decomposes the body into two validated halves whose volumes sum
//!   to the input. A plane that misses the body is `InvalidInput`
//!   ("entirely on one side"). A plane that contains an existing edge, that
//!   crosses an inner wire, or that meets a solid with cavity shells is a
//!   typed `Unsupported` refusal naming the configuration; refusals roll back
//!   transactionally and preserve the input solid.
//! * Section tolerance bands are fixed physical values (`coplanar 1e-5`,
//!   endpoint `1e-6`, chain `1e-4` from `Tolerance::linear = 1e-7`); split
//!   side classification uses the scaled slack
//!   `eps = max(1e-7, span * 1e-10)`. Both are recorded, never widened.
//!
//! Successes are checked against hand closed forms (rectangle areas,
//! rectangle-minus-hole areas, half volumes), an independent shoelace polygon
//! oracle over the section wires (so a missing hole cannot hide behind a
//! matching area), plane-membership of every section vertex, ordered closed
//! boundaries, material probes (`classify_point`) inside/outside each loop,
//! dual solid validators, closed B-Rep topology, and an independently
//! integrated watertight-mesh volume. Rebuilding any cell must be
//! deterministic.
//!
//! Explicit denominators are stated on each test. Curved/freeform sections
//! (cylinders, spheres, NURBS) are out of scope and listed as uncovered in
//! the B6 disposition; this matrix promotes only the planar box-derived
//! subset.
//!
//! Cell count (native): inputs 18 + transverse sections 30 + empty/coincident
//! 36 + degenerates 21 + reversed/repeat sections 9 + split successes 18 +
//! split refusals 15 + split reversed/repeat 6 = 153 cells.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{tessellate_solid_with_tolerance, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
const ROT_ANGLE: f64 = 0.5;

/// Plain box edge length multiplier: the unit box is 4 x 4 x 4.
const BOX: f64 = 4.0;
/// Hollow-box cavity: 2 x 2 x 2 at offset (1, 1, 1) inside the 4-box.
const CAVITY: f64 = 2.0;
/// Holed plate: 10 x 10 x 2 with a 2 x 2 through-hole at (4, 4) in XY.
const PLATE_XY: f64 = 10.0;
const PLATE_Z: f64 = 2.0;
const HOLE: f64 = 2.0;

fn assert_relative(label: &str, actual: f64, expected: f64, limit: f64) {
    let relative = (actual - expected).abs() / expected.abs().max(1e-300);
    assert!(
        relative <= limit,
        "{label}: expected {expected:.12e}, got {actual:.12e}, relative error {relative:.3e} > {limit:.3e}"
    );
}

/// Rigid placement: rotate 30-ish degrees about Z, then translate.
fn rigid_matrix(scale: f64) -> Mat4 {
    Mat4::translation(10.0 * scale, -3.0 * scale, 5.0 * scale) * Mat4::rotation_z(ROT_ANGLE)
}

fn rotate_normal_z(normal: Vec3) -> Vec3 {
    let (c, s) = (ROT_ANGLE.cos(), ROT_ANGLE.sin());
    Vec3::new(
        c * normal.x() - s * normal.y(),
        s * normal.x() + c * normal.y(),
        normal.z(),
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Placement {
    Origin,
    Rigid,
}

impl Placement {
    const ALL: [Self; 2] = [Self::Origin, Self::Rigid];

    fn apply_to_solid(self, topo: &mut Topology, solid: SolidId, scale: f64) {
        if matches!(self, Self::Rigid) {
            transform_solid(topo, solid, &rigid_matrix(scale)).unwrap();
        }
    }

    /// Map a plane defined in the origin frame into this placement.
    fn map_plane(self, point: Point3, normal: Vec3, scale: f64) -> (Point3, Vec3) {
        match self {
            Self::Origin => (point, normal),
            Self::Rigid => (
                rigid_matrix(scale).mul_point(point),
                rotate_normal_z(normal),
            ),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fixture {
    PlainBox,
    HollowBox,
    HoledPlate,
}

impl Fixture {
    const ALL: [Self; 3] = [Self::PlainBox, Self::HollowBox, Self::HoledPlate];

    /// Build the fixture at `scale` in the origin frame. Returns the solid
    /// and its hand closed-form volume (outer minus void, no kernel
    /// measurement involved).
    fn build(self, topo: &mut Topology, scale: f64) -> (SolidId, f64) {
        match self {
            Self::PlainBox => {
                let solid = make_box(topo, BOX * scale, BOX * scale, BOX * scale).unwrap();
                (solid, BOX.powi(3) * scale.powi(3))
            }
            Self::HollowBox => {
                let outer = make_box(topo, BOX * scale, BOX * scale, BOX * scale).unwrap();
                let inner = make_box(topo, CAVITY * scale, CAVITY * scale, CAVITY * scale).unwrap();
                transform_solid(topo, inner, &Mat4::translation(scale, scale, scale)).unwrap();
                let hollow = boolean(topo, BooleanOp::Cut, outer, inner).unwrap();
                let expected = (BOX.powi(3) - CAVITY.powi(3)) * scale.powi(3);
                (hollow, expected)
            }
            Self::HoledPlate => {
                let plate =
                    make_box(topo, PLATE_XY * scale, PLATE_XY * scale, PLATE_Z * scale).unwrap();
                let tool =
                    make_box(topo, HOLE * scale, HOLE * scale, (PLATE_Z + 2.0) * scale).unwrap();
                transform_solid(
                    topo,
                    tool,
                    &Mat4::translation(4.0 * scale, 4.0 * scale, -scale),
                )
                .unwrap();
                let holed = boolean(topo, BooleanOp::Cut, plate, tool).unwrap();
                let expected =
                    (PLATE_XY * PLATE_XY * PLATE_Z - HOLE * HOLE * PLATE_Z) * scale.powi(3);
                (holed, expected)
            }
        }
    }

    fn expected_shells(self) -> usize {
        match self {
            Self::PlainBox | Self::HoledPlate => 1,
            Self::HollowBox => 2,
        }
    }
}

fn assert_input_valid(
    topo: &Topology,
    solid: SolidId,
    expected_volume: f64,
    expected_shells: usize,
    label: &str,
    scale: f64,
) {
    let operations_report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        operations_report.is_valid(),
        "{label}: L3 validation issues: {:?}",
        operations_report.issues
    );
    let check_report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        check_report.is_valid(),
        "{label}: check validation issues: {:?}",
        check_report.issues
    );
    let data = topo.solid(solid).unwrap();
    let shells = 1 + data.inner_shells().len();
    assert_eq!(shells, expected_shells, "{label}: shell count");
    let shell = topo.shell(data.outer_shell()).unwrap();
    validate_shell_closed(shell, topo).unwrap();
    let volume = solid_volume(topo, solid, 0.01 * scale).unwrap();
    assert_relative(
        &format!("{label}: input volume"),
        volume,
        expected_volume,
        1e-9,
    );
}

/// Scale-relative membership epsilon for assertions (not kernel tolerance).
fn membership_eps(scale: f64) -> f64 {
    (scale * 1e-7).max(1e-12)
}

fn plane_d(normal: Vec3, point: Point3) -> f64 {
    normal.x() * point.x() + normal.y() * point.y() + normal.z() * point.z()
}

/// Every vertex of every section wire lies on the cutting plane; the section
/// faces themselves carry the cutting plane as their surface.
fn assert_section_on_plane(
    topo: &Topology,
    faces: &[remus_topology::face::FaceId],
    normal: Vec3,
    point: Point3,
    scale: f64,
    label: &str,
) {
    let unit = normal.normalize().unwrap();
    let d = plane_d(unit, point);
    let eps = membership_eps(scale).max(scale * 1e-9);
    for &fid in faces {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            FaceSurface::Plane { normal: sn, d: sd } => {
                let parallel = sn.dot(unit).abs();
                assert!(
                    (parallel - 1.0).abs() < 1e-12,
                    "{label}: section face normal {sn:?} must be parallel to cut normal {unit:?}"
                );
                assert!(
                    (sd - d).abs() <= eps.max(1e-9 * scale.max(1.0))
                        || (sd + d).abs() <= eps.max(1e-9 * scale.max(1.0)),
                    "{label}: section face d {sd} must match cut d {d} (eps {eps:e})"
                );
            }
            other => panic!("{label}: section face must be planar, got {other:?}"),
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            let wire = topo.wire(wid).unwrap();
            assert!(
                wire.is_closed(),
                "{label}: section wires must be ordered closed boundaries"
            );
            assert!(
                wire.edges().len() >= 3,
                "{label}: section wire must have at least 3 edges"
            );
            for oe in wire.edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                for vid in [edge.start(), edge.end()] {
                    let p = topo.vertex(vid).unwrap().point();
                    let residual = (plane_d(unit, p) - d).abs();
                    assert!(
                        residual <= eps.max(1e-9 * scale.max(1.0)) + 1e-12,
                        "{label}: section vertex {p:?} off plane by {residual:e} (eps {eps:e})"
                    );
                }
            }
        }
    }
}

/// Independent shoelace area of a section face's outer/inner polygons,
/// projected onto the cutting plane. Shares no code with `face_area`.
fn shoelace_face_area(
    topo: &Topology,
    fid: remus_topology::face::FaceId,
    normal: Vec3,
) -> (f64, f64, usize) {
    let face = topo.face(fid).unwrap();
    let mut inner_area = 0.0;
    let (u, v) = plane_basis(normal);
    let project = |p: Point3| {
        (
            u.x() * p.x() + u.y() * p.y() + u.z() * p.z(),
            v.x() * p.x() + v.y() * p.y() + v.z() * p.z(),
        )
    };
    let polygon_area = |wid: remus_topology::wire::WireId| {
        let wire = topo.wire(wid).unwrap();
        let mut pts = Vec::new();
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            let vid = if oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            pts.push(project(topo.vertex(vid).unwrap().point()));
        }
        let mut acc = 0.0;
        for i in 0..pts.len() {
            let j = (i + 1) % pts.len();
            acc += pts[i].0 * pts[j].1 - pts[j].0 * pts[i].1;
        }
        acc.abs() * 0.5
    };
    let outer_area = polygon_area(face.outer_wire());
    for &wid in face.inner_wires() {
        inner_area += polygon_area(wid);
    }
    (outer_area, inner_area, face.inner_wires().len())
}

fn plane_basis(normal: Vec3) -> (Vec3, Vec3) {
    let seed = if normal.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let u = normal.cross(seed);
    let len = u.length();
    let u = Vec3::new(u.x() / len, u.y() / len, u.z() / len);
    (u, normal.cross(u))
}

fn surface_census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut result = BTreeMap::new();
    for face_id in solid_faces(topo, solid).unwrap() {
        let tag = topo.face(face_id).unwrap().surface().type_tag();
        *result.entry(tag).or_default() += 1;
    }
    result
}

fn hole_count(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .sum()
}

fn signed_mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let a = mesh.positions[t[0] as usize];
            let b = mesh.positions[t[1] as usize];
            let c = mesh.positions[t[2] as usize];
            (a.x() * (b.y() * c.z() - b.z() * c.y()) - a.y() * (b.x() * c.z() - b.z() * c.x())
                + a.z() * (b.x() * c.y() - b.y() * c.x()))
                / 6.0
        })
        .sum::<f64>()
        .abs()
}

fn assert_half_valid_closed_mesh(
    topo: &Topology,
    solid: SolidId,
    expected_volume: f64,
    scale: f64,
    label: &str,
) {
    let operations_report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        operations_report.is_valid(),
        "{label}: L3 issues: {:?}",
        operations_report.issues
    );
    let check_report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        check_report.is_valid(),
        "{label}: check issues: {:?}",
        check_report.issues
    );
    let data = topo.solid(solid).unwrap();
    validate_shell_closed(topo.shell(data.outer_shell()).unwrap(), topo).unwrap();
    let volume = solid_volume(topo, solid, 0.01 * scale).unwrap();
    assert!(
        volume.is_finite() && volume > 0.0,
        "{label}: half must enclose positive volume, got {volume}"
    );
    assert_relative(
        &format!("{label}: half volume"),
        volume,
        expected_volume,
        1e-9,
    );
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01 * scale, 0.1).unwrap();
    let quality = welded_mesh_quality(&mesh);
    assert!(
        quality.is_watertight(),
        "{label}: welded mesh must be watertight, got {quality:?}"
    );
    assert_relative(
        &format!("{label}: independent mesh volume"),
        signed_mesh_volume(&mesh),
        expected_volume,
        1e-9,
    );
}

fn classify(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    scale: f64,
) -> remus_operations::classify::PointClassification {
    remus_operations::classify::classify_point(topo, solid, point, 0.01 * scale, 1e-7).unwrap()
}

// ── Section cut definitions ─────────────────────────────────────────────

/// One transverse section case in the origin frame: the plane, the expected
/// face/hole counts, and the hand rectangle-minus-hole area at unit scale.
struct SectionCase {
    fixture: Fixture,
    name: &'static str,
    point: Point3,
    normal: Vec3,
    expected_faces: usize,
    expected_holes: usize,
    expected_area_unit: f64,
}

fn transverse_section_cases() -> Vec<SectionCase> {
    vec![
        SectionCase {
            fixture: Fixture::PlainBox,
            name: "box through material z=2",
            point: Point3::new(0.0, 0.0, 2.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            expected_faces: 1,
            expected_holes: 0,
            expected_area_unit: 16.0,
        },
        SectionCase {
            fixture: Fixture::HollowBox,
            name: "hollow below cavity z=0.5",
            point: Point3::new(0.0, 0.0, 0.5),
            normal: Vec3::new(0.0, 0.0, 1.0),
            expected_faces: 1,
            expected_holes: 0,
            expected_area_unit: 16.0,
        },
        SectionCase {
            fixture: Fixture::HollowBox,
            name: "hollow through cavity z=2",
            point: Point3::new(0.0, 0.0, 2.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            expected_faces: 1,
            expected_holes: 1,
            expected_area_unit: 12.0,
        },
        SectionCase {
            fixture: Fixture::HoledPlate,
            name: "plate through hole z=1",
            point: Point3::new(0.0, 0.0, 1.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            expected_faces: 1,
            expected_holes: 1,
            expected_area_unit: 96.0,
        },
        SectionCase {
            fixture: Fixture::HoledPlate,
            name: "plate clear of hole x=1",
            point: Point3::new(1.0, 0.0, 0.0),
            normal: Vec3::new(1.0, 0.0, 0.0),
            expected_faces: 1,
            expected_holes: 0,
            expected_area_unit: 20.0,
        },
    ]
}

// ── Tests ───────────────────────────────────────────────────────────────

/// 18 cells: 3 fixtures x 3 scales x 2 placements. Every input is proven
/// valid (dual validators, closed shell, closed-form volume) before any
/// section or split runs.
#[test]
fn section_split_inputs_are_valid() {
    for fixture in Fixture::ALL {
        for scale in SCALES {
            for placement in Placement::ALL {
                let label = format!("{fixture:?} at scale {scale:e} {placement:?}");
                let mut topo = Topology::new();
                let (solid, expected_volume) = fixture.build(&mut topo, scale);
                placement.apply_to_solid(&mut topo, solid, scale);
                assert_input_valid(
                    &topo,
                    solid,
                    expected_volume,
                    fixture.expected_shells(),
                    &label,
                    scale,
                );
            }
        }
    }
}

/// 30 cells: 5 transverse cuts x 3 scales x 2 placements. Each asserts plane
/// membership, face/hole counts, hand rectangle-minus-hole area, an
/// independent shoelace oracle (outer minus inner), ordered closed
/// boundaries, and material probes inside/outside each loop.
#[test]
fn transverse_sections_match_rectangle_minus_hole() {
    for case in transverse_section_cases() {
        for scale in SCALES {
            for placement in Placement::ALL {
                let label = format!("{} at scale {scale:e} {placement:?}", case.name);
                let mut topo = Topology::new();
                let (solid, _) = case.fixture.build(&mut topo, scale);
                placement.apply_to_solid(&mut topo, solid, scale);
                let origin_point = Point3::new(
                    case.point.x() * scale,
                    case.point.y() * scale,
                    case.point.z() * scale,
                );
                let (plane_point, plane_normal) =
                    placement.map_plane(origin_point, case.normal, scale);
                // Rigid placements move the solid; the plane follows it.
                let section =
                    remus_operations::section::section(&mut topo, solid, plane_point, plane_normal)
                        .unwrap_or_else(|e| {
                            panic!("{label}: transverse cut must succeed, got {e:?}")
                        });
                assert_eq!(
                    section.faces.len(),
                    case.expected_faces,
                    "{label}: face count"
                );
                assert_section_on_plane(
                    &topo,
                    &section.faces,
                    plane_normal,
                    plane_point,
                    scale,
                    &label,
                );
                let expected_area = case.expected_area_unit * scale * scale;
                let mut total_shoelace = 0.0;
                let mut total_holes = 0;
                for &fid in &section.faces {
                    let measured = face_area(&topo, fid, 0.01 * scale).unwrap();
                    let (outer, inner, holes) =
                        shoelace_face_area(&topo, fid, plane_normal.normalize().unwrap());
                    total_holes += holes;
                    total_shoelace += outer - inner;
                    // The kernel area and the independent polygon oracle agree,
                    // and the hole count is explicit: an area match alone
                    // must not hide a missing hole.
                    assert_relative(
                        &format!("{label}: kernel face area"),
                        measured,
                        expected_area,
                        1e-9,
                    );
                    assert_relative(
                        &format!("{label}: independent shoelace area"),
                        outer - inner,
                        expected_area,
                        1e-9,
                    );
                    assert!(
                        outer > inner,
                        "{label}: outer {outer} must exceed inner {inner}"
                    );
                }
                assert_eq!(total_holes, case.expected_holes, "{label}: hole count");
                assert_relative(
                    &format!("{label}: total shoelace"),
                    total_shoelace,
                    expected_area,
                    1e-9,
                );
                // Material probes: a point in the material ring is Inside the
                // input solid; the cavity/hole centre is Outside it.
                match (case.fixture, case.name) {
                    (Fixture::HollowBox, "hollow through cavity z=2") => {
                        let in_material = placement
                            .map_plane(
                                Point3::new(0.5 * scale, 2.0 * scale, 2.0 * scale),
                                case.normal,
                                scale,
                            )
                            .0;
                        let in_cavity = placement
                            .map_plane(
                                Point3::new(2.0 * scale, 2.0 * scale, 2.0 * scale),
                                case.normal,
                                scale,
                            )
                            .0;
                        assert_eq!(
                            classify(&topo, solid, in_material, scale),
                            remus_operations::classify::PointClassification::Inside,
                            "{label}: material probe"
                        );
                        assert_eq!(
                            classify(&topo, solid, in_cavity, scale),
                            remus_operations::classify::PointClassification::Outside,
                            "{label}: cavity probe"
                        );
                    }
                    (Fixture::HoledPlate, "plate through hole z=1") => {
                        let in_material = placement
                            .map_plane(
                                Point3::new(1.0 * scale, 1.0 * scale, 1.0 * scale),
                                case.normal,
                                scale,
                            )
                            .0;
                        let in_hole = placement
                            .map_plane(
                                Point3::new(5.0 * scale, 5.0 * scale, 1.0 * scale),
                                case.normal,
                                scale,
                            )
                            .0;
                        assert_eq!(
                            classify(&topo, solid, in_material, scale),
                            remus_operations::classify::PointClassification::Inside,
                            "{label}: material probe"
                        );
                        assert_eq!(
                            classify(&topo, solid, in_hole, scale),
                            remus_operations::classify::PointClassification::Outside,
                            "{label}: hole probe"
                        );
                    }
                    _ => {}
                }
            }
        }
    }
}

/// 36 cells: 3 fixtures x (outside + coincident) x 3 scales x 2 placements.
/// Outside yields the documented empty-face-list success; coincident yields
/// the face outline.
#[test]
fn empty_and_coincident_sections_hold_their_contracts() {
    for fixture in Fixture::ALL {
        for scale in SCALES {
            for placement in Placement::ALL {
                // Wholly outside: above the body.
                {
                    let label = format!("{fixture:?} outside at scale {scale:e} {placement:?}");
                    let mut topo = Topology::new();
                    let (solid, _) = fixture.build(&mut topo, scale);
                    placement.apply_to_solid(&mut topo, solid, scale);
                    let miss_point = Point3::new(0.0, 0.0, 50.0 * scale);
                    let (pp, nn) = placement.map_plane(miss_point, Vec3::new(0.0, 0.0, 1.0), scale);
                    let section =
                        remus_operations::section::section(&mut topo, solid, pp, nn).unwrap();
                    assert!(
                        section.faces.is_empty(),
                        "{label}: miss must be an empty success"
                    );
                }
                // Coincident with the bottom face (z=0).
                {
                    let label = format!("{fixture:?} coincident at scale {scale:e} {placement:?}");
                    let mut topo = Topology::new();
                    let (solid, _) = fixture.build(&mut topo, scale);
                    placement.apply_to_solid(&mut topo, solid, scale);
                    let (pp, nn) = placement.map_plane(
                        Point3::new(0.0, 0.0, 0.0),
                        Vec3::new(0.0, 0.0, 1.0),
                        scale,
                    );
                    let section =
                        remus_operations::section::section(&mut topo, solid, pp, nn).unwrap();
                    assert_eq!(section.faces.len(), 1, "{label}: coincident outline");
                    assert_section_on_plane(&topo, &section.faces, nn, pp, scale, &label);
                    // Bottom-face outline areas: box 16, hollow 16 (cavity
                    // does not touch the bottom), plate 96 (hole mouth).
                    let expected_unit = match fixture {
                        Fixture::PlainBox | Fixture::HollowBox => 16.0,
                        Fixture::HoledPlate => 96.0,
                    };
                    let expected_holes = match fixture {
                        Fixture::HoledPlate => 1,
                        _ => 0,
                    };
                    let measured = face_area(&topo, section.faces[0], 0.01 * scale).unwrap();
                    assert_relative(
                        &format!("{label}: coincident area"),
                        measured,
                        expected_unit * scale * scale,
                        1e-9,
                    );
                    let (_, _, holes) =
                        shoelace_face_area(&topo, section.faces[0], nn.normalize().unwrap());
                    assert_eq!(holes, expected_holes, "{label}: coincident holes");
                }
            }
        }
    }
}

/// 21 cells: vertex touch (empty success), edge touch (`InvalidInput`), and
/// both-sided face-boundary offsets (3 outside, 2 inside) on the plain box,
/// each at 3 scales, plus fixed-tolerance documentation. The degenerate bands use fixed physical
/// tolerance in `section` (coplanar `1e-5`), so the same offsets behave the
/// same at every scale; `split` uses the scaled `eps` and is pinned in the
/// refusal test.
#[test]
fn degenerate_sections_hold_both_sided_contracts() {
    for scale in SCALES {
        // Through a single vertex: touches at one point, no closed wire.
        {
            let label = format!("vertex touch at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, _) = Fixture::PlainBox.build(&mut topo, scale);
            let section = remus_operations::section::section(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 1.0),
            )
            .unwrap();
            assert!(
                section.faces.is_empty(),
                "{label}: vertex touch is an empty success, got {} faces",
                section.faces.len()
            );
        }
        // Through an existing vertical edge: segments exist but no closed
        // wire assembles.
        {
            let label = format!("edge touch at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, _) = Fixture::PlainBox.build(&mut topo, scale);
            let err = remus_operations::section::section(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 2.0 * scale),
                Vec3::new(1.0, 1.0, 0.0),
            )
            .unwrap_err();
            assert!(
                matches!(err, OperationsError::InvalidInput { .. }),
                "{label}: edge touch must be InvalidInput, got {err:?}"
            );
        }
        // Both-sided boundary around the top face (z=4*scale): within the
        // fixed 1e-5 coplanar band the plane still reads as coincident;
        // beyond it the plane misses.
        for (dz, expect_faces) in [(1e-9, 1), (1e-6, 1), (1e-4, 0)] {
            let label = format!("top face +{dz:e} at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, _) = Fixture::PlainBox.build(&mut topo, scale);
            let section = remus_operations::section::section(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 4.0 * scale + dz),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap();
            assert_eq!(
                section.faces.len(),
                expect_faces,
                "{label}: fixed coplanar band 1e-5 decides coincidence"
            );
        }
        for dz in [-1e-9, -1e-6] {
            let label = format!("top face {dz:e} at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, _) = Fixture::PlainBox.build(&mut topo, scale);
            let section = remus_operations::section::section(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 4.0 * scale + dz),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap();
            assert_eq!(section.faces.len(), 1, "{label}: just inside still cuts");
        }
    }
}

/// 9 cells: 3 fixtures x 3 scales. Reversed normals return the same area;
/// repeated calls return bit-identical areas.
#[test]
fn sections_are_normal_reversible_and_repeatable() {
    for fixture in Fixture::ALL {
        for scale in SCALES {
            let label = format!("{fixture:?} reversed/repeat at scale {scale:e}");
            let (point, normal) = match fixture {
                Fixture::PlainBox => (Point3::new(0.0, 0.0, 2.0 * scale), Vec3::new(0.0, 0.0, 1.0)),
                Fixture::HollowBox => {
                    (Point3::new(0.0, 0.0, 2.0 * scale), Vec3::new(0.0, 0.0, 1.0))
                }
                Fixture::HoledPlate => {
                    (Point3::new(0.0, 0.0, 1.0 * scale), Vec3::new(0.0, 0.0, 1.0))
                }
            };
            let mut topo = Topology::new();
            let (solid, _) = fixture.build(&mut topo, scale);
            let forward =
                remus_operations::section::section(&mut topo, solid, point, normal).unwrap();
            let backward = remus_operations::section::section(
                &mut topo,
                solid,
                point,
                Vec3::new(-normal.x(), -normal.y(), -normal.z()),
            )
            .unwrap();
            assert_eq!(
                forward.faces.len(),
                backward.faces.len(),
                "{label}: face count"
            );
            let area_sum = |topo: &Topology, faces: &[remus_topology::face::FaceId]| {
                faces
                    .iter()
                    .map(|&f| face_area(topo, f, 0.01 * scale).unwrap())
                    .sum::<f64>()
            };
            let forward_area = area_sum(&topo, &forward.faces);
            let backward_area = area_sum(&topo, &backward.faces);
            assert_relative(
                &format!("{label}: reversed area"),
                backward_area,
                forward_area,
                1e-12,
            );
            let forward_count = forward.faces.len();
            let again =
                remus_operations::section::section(&mut topo, solid, point, normal).unwrap();
            assert_eq!(again.faces.len(), forward_count, "{label}: repeat count");
            let again_area = area_sum(&topo, &again.faces);
            assert!(
                again_area.to_bits() == forward_area.to_bits(),
                "{label}: repeat must be bit-identical"
            );
        }
    }
}

// ── Split definitions ───────────────────────────────────────────────────

struct SplitCase {
    fixture: Fixture,
    name: &'static str,
    point: Point3,
    normal: Vec3,
    expected_pos_unit: f64,
    expected_neg_unit: f64,
    /// Expected inner-wire totals on (positive, negative) halves.
    expected_holes: (usize, usize),
}

fn successful_split_cases() -> Vec<SplitCase> {
    vec![
        SplitCase {
            fixture: Fixture::PlainBox,
            name: "box mid z=2",
            point: Point3::new(0.0, 0.0, 2.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            expected_pos_unit: 32.0,
            expected_neg_unit: 32.0,
            expected_holes: (0, 0),
        },
        SplitCase {
            fixture: Fixture::HoledPlate,
            name: "plate clear of hole x=1",
            point: Point3::new(1.0, 0.0, 0.0),
            normal: Vec3::new(1.0, 0.0, 0.0),
            expected_pos_unit: 172.0,
            expected_neg_unit: 20.0,
            expected_holes: (2, 0),
        },
        SplitCase {
            fixture: Fixture::HoledPlate,
            name: "plate mid-thickness z=1",
            point: Point3::new(0.0, 0.0, 1.0),
            normal: Vec3::new(0.0, 0.0, 1.0),
            expected_pos_unit: 96.0,
            expected_neg_unit: 96.0,
            expected_holes: (2, 2),
        },
    ]
}

/// 18 cells: 3 successful splits x 3 scales x 2 placements. Each half is
/// verified for closed-form volume, volume-sum identity, cap holes, material
/// on the correct side, dual validation, closed B-Rep, watertight welded
/// mesh with independent volume, all-planar census, and input preservation.
#[test]
fn splits_decompose_volumes_and_preserve_material() {
    for case in successful_split_cases() {
        for scale in SCALES {
            for placement in Placement::ALL {
                let label = format!("{} at scale {scale:e} {placement:?}", case.name);
                let mut topo = Topology::new();
                let (solid, input_volume) = case.fixture.build(&mut topo, scale);
                placement.apply_to_solid(&mut topo, solid, scale);
                let origin_point = Point3::new(
                    case.point.x() * scale,
                    case.point.y() * scale,
                    case.point.z() * scale,
                );
                let (plane_point, plane_normal) =
                    placement.map_plane(origin_point, case.normal, scale);
                let input_entities = solid_entity_counts(&topo, solid).unwrap();
                let result =
                    remus_operations::split::split(&mut topo, solid, plane_point, plane_normal)
                        .unwrap_or_else(|e| {
                            panic!("{label}: transverse split must succeed, got {e:?}")
                        });
                let expected_pos = case.expected_pos_unit * scale.powi(3);
                let expected_neg = case.expected_neg_unit * scale.powi(3);
                assert_half_valid_closed_mesh(
                    &topo,
                    result.positive,
                    expected_pos,
                    scale,
                    &format!("{label} positive"),
                );
                assert_half_valid_closed_mesh(
                    &topo,
                    result.negative,
                    expected_neg,
                    scale,
                    &format!("{label} negative"),
                );
                let pos_vol = solid_volume(&topo, result.positive, 0.01 * scale).unwrap();
                let neg_vol = solid_volume(&topo, result.negative, 0.01 * scale).unwrap();
                let slack = input_volume.abs() * 1e-9;
                assert!(
                    (pos_vol + neg_vol - input_volume).abs() <= slack,
                    "{label}: halves {pos_vol} + {neg_vol} must sum to {input_volume}"
                );
                assert_eq!(
                    hole_count(&topo, result.positive),
                    case.expected_holes.0,
                    "{label}: positive cap/preserved holes"
                );
                assert_eq!(
                    hole_count(&topo, result.negative),
                    case.expected_holes.1,
                    "{label}: negative cap/preserved holes"
                );
                // Box-derived splits stay all-planar (caps are planes, walls
                // are planes, rectangular-hole walls are planes).
                assert_eq!(
                    surface_census(&topo, result.positive),
                    BTreeMap::from([("plane", solid_faces(&topo, result.positive).unwrap().len())]),
                    "{label}: positive stays analytic-planar"
                );
                assert_eq!(
                    surface_census(&topo, result.negative),
                    BTreeMap::from([("plane", solid_faces(&topo, result.negative).unwrap().len())]),
                    "{label}: negative stays analytic-planar"
                );
                // Material on the correct side: probes at the body interior
                // on each side of the plane are Inside their half and
                // Outside the sibling. Origin-frame probes avoid the
                // through-hole; the placement maps them rigidly.
                let (above_origin, below_origin): (Point3, Point3) = match case.name {
                    "box mid z=2" => (
                        Point3::new(2.0 * scale, 2.0 * scale, 3.0 * scale),
                        Point3::new(2.0 * scale, 2.0 * scale, 1.0 * scale),
                    ),
                    "plate clear of hole x=1" => (
                        Point3::new(2.0 * scale, 2.0 * scale, 1.0 * scale),
                        Point3::new(0.5 * scale, 5.0 * scale, 1.0 * scale),
                    ),
                    _ => (
                        Point3::new(2.0 * scale, 2.0 * scale, 1.5 * scale),
                        Point3::new(2.0 * scale, 2.0 * scale, 0.5 * scale),
                    ),
                };
                let (above, _) = placement.map_plane(above_origin, case.normal, scale);
                let (below, _) = placement.map_plane(below_origin, case.normal, scale);
                assert_eq!(
                    classify(&topo, result.positive, above, scale),
                    remus_operations::classify::PointClassification::Inside,
                    "{label}: above-plane probe must be in the positive half"
                );
                assert_eq!(
                    classify(&topo, result.negative, above, scale),
                    remus_operations::classify::PointClassification::Outside,
                    "{label}: above-plane probe must be outside the negative half"
                );
                assert_eq!(
                    classify(&topo, result.negative, below, scale),
                    remus_operations::classify::PointClassification::Inside,
                    "{label}: below-plane probe must be in the negative half"
                );
                assert_eq!(
                    classify(&topo, result.positive, below, scale),
                    remus_operations::classify::PointClassification::Outside,
                    "{label}: below-plane probe must be outside the positive half"
                );
                // Input preservation: the source solid is still valid at its
                // closed-form volume with its entity census intact.
                assert_input_valid(
                    &topo,
                    solid,
                    input_volume,
                    case.fixture.expected_shells(),
                    &format!("{label}: input preserved"),
                    scale,
                );
                assert_eq!(
                    solid_entity_counts(&topo, solid).unwrap(),
                    input_entities,
                    "{label}: input census preserved"
                );
            }
        }
    }
}

/// 15 cells: 5 refusal configurations x 3 scales. Misses are `InvalidInput`;
/// cavity shells, inner-wire crossings, and edge-contained planes are typed
/// `Unsupported`. Every refusal preserves the full logical session: the
/// input solid stays valid at its closed-form volume and its handles still
/// resolve (allocated-slot counts may grow because rollback tombstones).
#[test]
fn split_refusals_are_typed_and_preserve_the_session() {
    for scale in SCALES {
        // Hollow box: cavity shells are outside the qualified split subset.
        {
            let label = format!("hollow-box cavity refusal at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, expected_volume) = Fixture::HollowBox.build(&mut topo, scale);
            let before_entities = solid_entity_counts(&topo, solid).unwrap();
            let before_volume = solid_volume(&topo, solid, 0.01 * scale).unwrap();
            let err = remus_operations::split::split(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 0.5 * scale),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap_err();
            assert!(
                matches!(
                    err,
                    OperationsError::Unsupported {
                        operation: "split",
                        ..
                    }
                ),
                "{label}: must be typed split refusal, got {err:?}"
            );
            assert!(
                format!("{err:?}").contains("cavity"),
                "{label}: refusal must name the cavity, got {err:?}"
            );
            assert_input_valid(&topo, solid, expected_volume, 2, &label, scale);
            assert_eq!(
                solid_entity_counts(&topo, solid).unwrap(),
                before_entities,
                "{label}: input census preserved"
            );
            assert_relative(
                &format!("{label}: input volume preserved"),
                solid_volume(&topo, solid, 0.01 * scale).unwrap(),
                before_volume,
                1e-12,
            );
        }
        // Holed plate split through the hole mouth: the hole stops being a
        // hole and becomes a notch, which has no exact construction.
        {
            let label = format!("plate inner-wire refusal at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, expected_volume) = Fixture::HoledPlate.build(&mut topo, scale);
            let before_entities = solid_entity_counts(&topo, solid).unwrap();
            let err = remus_operations::split::split(
                &mut topo,
                solid,
                Point3::new(5.0 * scale, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap_err();
            assert!(
                matches!(
                    err,
                    OperationsError::Unsupported {
                        operation: "split",
                        ..
                    }
                ),
                "{label}: must be typed split refusal, got {err:?}"
            );
            assert!(
                format!("{err:?}").contains("inner wire"),
                "{label}: refusal must name the inner wire, got {err:?}"
            );
            assert_input_valid(&topo, solid, expected_volume, 1, &label, scale);
            assert_eq!(
                solid_entity_counts(&topo, solid).unwrap(),
                before_entities,
                "{label}: input census preserved"
            );
        }
        // Plane wholly outside the body.
        {
            let label = format!("miss refusal at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, expected_volume) = Fixture::PlainBox.build(&mut topo, scale);
            let before_entities = solid_entity_counts(&topo, solid).unwrap();
            let err = remus_operations::split::split(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 50.0 * scale),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap_err();
            assert!(
                matches!(err, OperationsError::InvalidInput { .. }),
                "{label}: miss must be InvalidInput, got {err:?}"
            );
            assert_input_valid(&topo, solid, expected_volume, 1, &label, scale);
            assert_eq!(
                solid_entity_counts(&topo, solid).unwrap(),
                before_entities,
                "{label}: input census preserved"
            );
        }
        // Plane contains an existing edge: no unambiguous answer.
        {
            let label = format!("edge-contained refusal at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, expected_volume) = Fixture::PlainBox.build(&mut topo, scale);
            let before_entities = solid_entity_counts(&topo, solid).unwrap();
            let err = remus_operations::split::split(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap_err();
            assert!(
                matches!(
                    err,
                    OperationsError::Unsupported {
                        operation: "split",
                        ..
                    }
                ),
                "{label}: must be typed split refusal, got {err:?}"
            );
            assert!(
                format!("{err:?}").contains("contains edge"),
                "{label}: refusal must name the contained edge, got {err:?}"
            );
            assert_input_valid(&topo, solid, expected_volume, 1, &label, scale);
            assert_eq!(
                solid_entity_counts(&topo, solid).unwrap(),
                before_entities,
                "{label}: input census preserved"
            );
        }
        // Plane coincident with a side face contains that face's edges.
        {
            let label = format!("coincident-face refusal at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, expected_volume) = Fixture::HoledPlate.build(&mut topo, scale);
            let before_entities = solid_entity_counts(&topo, solid).unwrap();
            let err = remus_operations::split::split(
                &mut topo,
                solid,
                Point3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap_err();
            assert!(
                matches!(
                    err,
                    OperationsError::Unsupported { .. } | OperationsError::InvalidInput { .. }
                ),
                "{label}: coincident face must refuse typed, got {err:?}"
            );
            assert_input_valid(&topo, solid, expected_volume, 1, &label, scale);
            assert_eq!(
                solid_entity_counts(&topo, solid).unwrap(),
                before_entities,
                "{label}: input census preserved"
            );
        }
    }
}

/// 6 cells: 2 representative splits x 3 scales. Reversing the normal swaps
/// the halves; repeating the call is deterministic.
#[test]
fn splits_are_normal_reversible_and_repeatable() {
    for scale in SCALES {
        for (fixture, point, normal, name) in [
            (
                Fixture::PlainBox,
                Point3::new(0.0, 0.0, 2.0 * scale),
                Vec3::new(0.0, 0.0, 1.0),
                "box",
            ),
            (
                Fixture::HoledPlate,
                Point3::new(1.0 * scale, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                "plate",
            ),
        ] {
            let label = format!("{name} reversed/repeat at scale {scale:e}");
            let mut topo = Topology::new();
            let (solid, _) = fixture.build(&mut topo, scale);
            let forward = remus_operations::split::split(&mut topo, solid, point, normal).unwrap();
            let forward_pos = solid_volume(&topo, forward.positive, 0.01 * scale).unwrap();
            let forward_neg = solid_volume(&topo, forward.negative, 0.01 * scale).unwrap();
            let backward = remus_operations::split::split(
                &mut topo,
                solid,
                point,
                Vec3::new(-normal.x(), -normal.y(), -normal.z()),
            )
            .unwrap();
            let backward_pos = solid_volume(&topo, backward.positive, 0.01 * scale).unwrap();
            let backward_neg = solid_volume(&topo, backward.negative, 0.01 * scale).unwrap();
            assert_relative(
                &format!("{label}: reversed swaps halves (pos)"),
                backward_pos,
                forward_neg,
                1e-12,
            );
            assert_relative(
                &format!("{label}: reversed swaps halves (neg)"),
                backward_neg,
                forward_pos,
                1e-12,
            );
            let again = remus_operations::split::split(&mut topo, solid, point, normal).unwrap();
            let again_pos = solid_volume(&topo, again.positive, 0.01 * scale).unwrap();
            assert!(
                again_pos.to_bits() == forward_pos.to_bits(),
                "{label}: repeat must be bit-identical"
            );
        }
    }
}
