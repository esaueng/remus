//! OpenZCAD issue 1244: exact Intersect of a torus with a holed block kept
//! material inside the block's through-hole.
//!
//! On the original STEP representation (see
//! `data/issue_1242_1244_fixtures.md`) both operand orders were accepted and
//! the result held (6, 1, 0.5), a point inside the torus and inside the hole.
//! The torus face's `v` band, read from five samples of its seam circle,
//! missed a quarter of the tube, so the section on the hole wall stopped at
//! the torus crown; the splitter dropped it as an orphaned arc and the wall
//! was kept or discarded whole. With the band whole the section closes, and
//! the doubly periodic torus face is partitioned as a certified arrangement
//! in its cut-open chart (a closed section crossing its seam is cut there,
//! and the hole wall's copy of that loop is refined at the same vertices).
//!
//! The imported operands and their Remus STEP round trip (a reload) must
//! intersect exactly, in both orders, through every entry point a stored
//! operation can replay through, with no section chain dropped. The native
//! primitive torus is bounded by degenerate seams, which the chart does not
//! qualify; it must refuse with the arena unchanged or be exact.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, boolean, boolean_with_entity_evolution};
use remus_operations::classify::{PointClassification, classify_point};
use remus_operations::journal_ops::boolean_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

const TORUS: &str = include_str!("data/issue_1244_torus.step");
const HOLED_BLOCK: &str = include_str!("data/issue_1244_holed_block.step");

/// Inside the torus tube and inside the block's hole.
const HOLE_WITNESS: [f64; 3] = [6.0, 1.0, 0.5];
/// Inside the torus and the block's material (x = 9.5 is 3.5 from the hole
/// axis, beyond the 2.46 hole radius; |(9.5, 1)| = 9.55 is within 3 of R 7).
const MATERIAL_WITNESS: [f64; 3] = [9.5, 1.0, 0.5];
/// The exact intersection volume: a second B-rep kernel at a 1e-9
/// integration tolerance and the closed-form angular integral over the
/// tube's annulus both give 570.78893 (data file notes).
const INTERSECTION_VOLUME: f64 = 570.788_93;
/// Remus's face integrator reads this result 0.0245 high (4.3e-5), the same
/// in both orders and at any quadrature setting, while the tessellated
/// volume converges to the exact value; the integrator bound is that bias
/// plus margin, and the mesh extrapolation below carries the tight check.
const INTEGRATOR_TOL: f64 = 0.05;
/// Bound on the first-order Richardson extrapolation of the mesh volume.
const MESH_TOL: f64 = 0.005;

fn point(p: [f64; 3]) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

fn class(topo: &Topology, solid: SolidId, p: [f64; 3]) -> PointClassification {
    classify_point(topo, solid, point(p), 0.01, 1e-7).unwrap()
}

fn live_counts(topo: &Topology) -> [usize; 4] {
    [
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_faces(),
        topo.num_solids(),
    ]
}

fn imported() -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let torus = remus_io::step::reader::read_step(TORUS, &mut topo).unwrap()[0];
    let block = remus_io::step::reader::read_step(HOLED_BLOCK, &mut topo).unwrap()[0];
    (topo, torus, block)
}

/// The operands written out and read back through Remus's own STEP, as a
/// reload of a stored document replays them.
fn reloaded() -> (Topology, SolidId, SolidId) {
    let (topo, torus, block) = imported();
    let text = remus_io::step::writer::write_step(&topo, &[torus, block]).unwrap();
    let mut reloaded = Topology::new();
    let solids = remus_io::step::reader::read_step(&text, &mut reloaded).unwrap();
    assert_eq!(solids.len(), 2);
    (reloaded, solids[0], solids[1])
}

/// The same operands built from primitives: a torus, and a box less a
/// cylinder, scaled and moved as in the issue.
fn native() -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, 7.0, 3.0, 64).unwrap();
    let block = make_box(&mut topo, 24.0, 18.0, 12.0).unwrap();
    transform_solid(&mut topo, block, &Mat4::translation(-12.0, -9.0, -6.0)).unwrap();
    let drill = make_cylinder(&mut topo, 3.0, 20.0).unwrap();
    transform_solid(&mut topo, drill, &Mat4::translation(0.0, 0.0, -10.0)).unwrap();
    let holed = boolean(&mut topo, BooleanOp::Cut, block, drill).unwrap();
    let place = Mat4::translation(6.0, 1.0, 0.5) * Mat4::scale(0.82, 0.82, 0.82);
    transform_solid(&mut topo, holed, &place).unwrap();
    (topo, torus, holed)
}

fn check_operands(topo: &Topology, torus: SolidId, block: SolidId, what: &str) {
    assert_eq!(
        class(topo, torus, HOLE_WITNESS),
        PointClassification::Inside,
        "{what}"
    );
    assert_eq!(
        class(topo, block, HOLE_WITNESS),
        PointClassification::Outside,
        "{what}"
    );
    assert_eq!(
        class(topo, torus, MATERIAL_WITNESS),
        PointClassification::Inside,
        "{what}"
    );
    assert_eq!(
        class(topo, block, MATERIAL_WITNESS),
        PointClassification::Inside,
        "{what}"
    );
}

type Entry = fn(&mut Topology, SolidId, SolidId) -> Result<SolidId, OperationsError>;

const ENTRIES: [(&str, Entry); 3] = [
    ("boolean", |t, a, b| boolean(t, BooleanOp::Intersect, a, b)),
    ("entity evolution", |t, a, b| {
        boolean_with_entity_evolution(t, BooleanOp::Intersect, a, b).map(|(s, _)| s)
    }),
    ("journaled", |t, a, b| {
        boolean_journaled(t, remus_algo::bop::BooleanOp::Intersect, a, b).map(|r| r.solid)
    }),
];

/// Volume of a closed tessellation, by signed tetrahedra.
fn mesh_volume(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
    let mesh = remus_operations::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
    let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
    assert!(
        quality.is_watertight(),
        "volume requires a closed mesh at deflection {deflection}: {quality:?}"
    );
    assert_eq!(
        remus_operations::tessellate::boundary_edge_count(&mesh),
        0,
        "volume requires consistent triangle winding at deflection {deflection}"
    );
    let corner = |i: u32| {
        let p = mesh.positions[i as usize];
        Vec3::new(p.x(), p.y(), p.z())
    };
    mesh.indices
        .chunks(3)
        .map(|t| corner(t[0]).dot(corner(t[1]).cross(corner(t[2]))) / 6.0)
        .sum()
}

/// The exact volume from two tessellations: an inscribed mesh falls short
/// by an amount first order in the deflection, so the pair extrapolates.
fn extrapolated_volume(topo: &Topology, solid: SolidId) -> f64 {
    let (coarse, fine) = (0.004, 0.002);
    let (v_coarse, v_fine) = (
        mesh_volume(topo, solid, coarse),
        mesh_volume(topo, solid, fine),
    );
    v_fine + (v_fine - v_coarse) * fine / (coarse - fine)
}

fn check_exact(topo: &Topology, result: SolidId, what: &str) {
    assert_ne!(
        class(topo, result, HOLE_WITNESS),
        PointClassification::Inside,
        "{what}: kept material inside the through-hole"
    );
    assert_eq!(
        class(topo, result, MATERIAL_WITNESS),
        PointClassification::Inside,
        "{what}: lost material both operands hold"
    );
    let v = solid_volume(topo, result, 0.005).unwrap();
    assert!(
        (v - INTERSECTION_VOLUME).abs() < INTEGRATOR_TOL,
        "{what}: volume {v}, reference {INTERSECTION_VOLUME}"
    );
    let v = extrapolated_volume(topo, result);
    assert!(
        (v - INTERSECTION_VOLUME).abs() < MESH_TOL,
        "{what}: extrapolated mesh volume {v}, reference {INTERSECTION_VOLUME}"
    );
    let faces = remus_topology::explorer::solid_faces(topo, result).unwrap();
    assert!(faces.len() < 20, "{what}: {} faces", faces.len());
    for face in faces {
        let tag = topo.face(face).unwrap().surface().type_tag();
        assert!(
            matches!(tag, "plane" | "cylinder" | "torus"),
            "{what}: {tag} face"
        );
    }
    let report = remus_check::validate::validate_solid(
        topo,
        result,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(report.is_valid(), "{what}: {report:?}");
}

/// `required`: the family is qualified and must intersect exactly; when
/// false, a typed refusal with the arena unchanged is also accepted.
fn check_intersections(build: fn() -> (Topology, SolidId, SolidId), source: &str, required: bool) {
    for (name, entry) in ENTRIES {
        for swap in [false, true] {
            let what = format!("{source} {name} swap={swap}");
            let (mut topo, torus, block) = build();
            check_operands(&topo, torus, block, &what);
            let before = live_counts(&topo);
            let (a, b) = if swap { (block, torus) } else { (torus, block) };
            let drops = remus_algo::gfa::SectionDropScope::enter();
            match entry(&mut topo, a, b) {
                Ok(result) => {
                    if required {
                        assert_eq!(drops.dropped(), 0, "{what}: a section chain was dropped");
                    }
                    check_exact(&topo, result, &what);
                }
                Err(e) => {
                    assert!(!required, "{what}: refused: {e:?}");
                    assert!(
                        matches!(
                            e,
                            OperationsError::ExactOnlyUnattainable | OperationsError::Algo(_)
                        ),
                        "{what}: unexpected refusal {e:?}"
                    );
                    assert_eq!(live_counts(&topo), before, "{what}: refusal left entities");
                }
            }
            check_operands(&topo, torus, block, &what);
        }
    }
}

#[test]
fn imported_intersection_is_exact() {
    check_intersections(imported, "imported", true);
}

#[test]
fn reloaded_intersection_is_exact() {
    check_intersections(reloaded, "reloaded", true);
}

#[test]
fn native_intersection_is_exact_or_refuses() {
    check_intersections(native, "native", false);
}

/// The hole witness really is in the hole's axis line: a sanity check on the
/// fixture placement the witnesses above rely on.
#[test]
fn hole_axis_matches_the_placement() {
    let (topo, _, block) = imported();
    for dz in [-4.0, 0.0, 4.0] {
        let p = point(HOLE_WITNESS) + Vec3::new(0.0, 0.0, dz);
        assert_eq!(
            classify_point(&topo, block, p, 0.01, 1e-7).unwrap(),
            PointClassification::Outside
        );
    }
}
