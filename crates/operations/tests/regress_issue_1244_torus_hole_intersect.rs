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

use remus_math::compact_sort::CompactSort;
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

// A consumer unifies the Boolean result's faces before displaying it. That
// merges the two torus faces the section split along the meridian seam into
// one face whose boundary is four closed wires meeting at vertices: the two
// band boundaries, each winding the tube period, and two zero-area slits
// along the equator seam that joined them. The chart tessellator read every
// inner wire as a strictly interior hole and refused the winding band
// boundary ("torus hole winds a parameter period"), at every deflection and
// through every mesh entry point, though the unified solid validates and
// classifies exactly. It now joins touching cycles into one boundary walk.

/// The display tessellation a consumer requests for a prepared drag.
const DISPLAY: (f64, f64) = (0.08, 0.24);

fn unified(
    build: fn() -> (Topology, SolidId, SolidId),
    swap: bool,
    entry: Entry,
) -> (Topology, SolidId) {
    let (mut topo, torus, block) = build();
    let (a, b) = if swap { (block, torus) } else { (torus, block) };
    let result = entry(&mut topo, a, b).unwrap();
    let report = remus_operations::heal::unify_faces_checked(&mut topo, result).unwrap();
    assert_eq!(report.input_errors, 0);
    assert_eq!(report.result_errors, 0);
    assert!(!report.reverted);
    assert!(report.faces_merged >= 1, "nothing merged: {report:?}");
    (topo, result)
}

/// The merged torus face, and the precondition that it is the touching-cycle
/// shape: an inner wire sharing a vertex with the outer wire.
fn merged_torus_face(topo: &Topology, solid: SolidId) -> remus_topology::face::FaceId {
    let starts = |wire| {
        topo.wire(wire)
            .unwrap()
            .edges()
            .iter()
            .map(|oe| oe.oriented_start(topo.edge(oe.edge()).unwrap()))
            .collect::<Vec<_>>()
    };
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let tori: Vec<_> = faces
        .into_iter()
        .filter(|&f| topo.face(f).unwrap().surface().type_tag() == "torus")
        .collect();
    assert_eq!(tori.len(), 1, "the torus faces were not merged");
    let face = topo.face(tori[0]).unwrap();
    let outer = starts(face.outer_wire());
    assert!(
        face.inner_wires()
            .iter()
            .any(|&w| starts(w).iter().any(|v| outer.contains(v))),
        "no inner wire touches the outer wire"
    );
    tori[0]
}

fn closed(mesh: &remus_operations::tessellate::TriangleMesh, what: &str) {
    assert!(!mesh.indices.is_empty(), "{what}: empty mesh");
    let quality = remus_operations::tessellate::welded_mesh_quality(mesh);
    assert!(quality.is_watertight(), "{what}: {quality:?}");
    assert_eq!(
        remus_operations::tessellate::boundary_edge_count(mesh),
        0,
        "{what}: inconsistent winding"
    );
}

#[test]
fn unified_intersection_meshes_exactly() {
    for (source, build) in [("imported", imported as fn() -> _), ("reloaded", reloaded)] {
        for (name, entry) in ENTRIES {
            for swap in [false, true] {
                let what = format!("unified {source} {name} swap={swap}");
                let (topo, result) = unified(build, swap, entry);
                let torus_face = merged_torus_face(&topo, result);
                // Every original oracle: witnesses, both volumes, analytic
                // faces, strict validity.
                check_exact(&topo, result, &what);
                for (deflection, angular) in
                    [DISPLAY, (0.004, remus_math::chord::DEFAULT_ANGULAR_TOL)]
                {
                    let (grouped, offsets) =
                        remus_operations::tessellate::tessellate_solid_grouped_with_tolerance(
                            &topo, result, deflection, angular,
                        )
                        .unwrap_or_else(|e| panic!("{what} grouped @{deflection}: {e:?}"));
                    closed(&grouped, &format!("{what} grouped @{deflection}"));
                    let plain = remus_operations::tessellate::tessellate_solid_with_tolerance(
                        &topo, result, deflection, angular,
                    )
                    .unwrap();
                    // Grouping only reorders triangles by face.
                    assert_eq!(grouped.positions, plain.positions, "{what} @{deflection}");
                    let mut a: Vec<_> = grouped.indices.chunks(3).map(<[u32]>::to_vec).collect();
                    let mut b: Vec<_> = plain.indices.chunks(3).map(<[u32]>::to_vec).collect();
                    a.compact_sort_unstable();
                    b.compact_sort_unstable();
                    assert_eq!(a, b, "{what} @{deflection}: grouped and plain differ");
                    assert_eq!(*offsets.last().unwrap() as usize, grouped.indices.len());
                }
                // The face's own entry point charts the same boundary walk.
                let face = remus_operations::tessellate::tessellate(&topo, torus_face, 0.004)
                    .unwrap_or_else(|e| panic!("{what} per-face: {e:?}"));
                assert!(!face.indices.is_empty(), "{what}: per-face mesh empty");
            }
        }
    }
}

/// Builds a free torus face on the merged face's carrier from chosen wires
/// of it, for the per-face entry point.
fn torus_face_from(
    topo: &mut Topology,
    source: remus_topology::face::FaceId,
    outer: Vec<remus_topology::wire::OrientedEdge>,
    inner: Vec<Vec<remus_topology::wire::OrientedEdge>>,
) -> remus_topology::face::FaceId {
    use remus_topology::wire::Wire;
    let surface = topo.face(source).unwrap().surface().clone();
    let outer = topo.add_wire(Wire::new(outer, true).unwrap());
    let inner = inner
        .into_iter()
        .map(|w| topo.add_wire(Wire::new(w, true).unwrap()))
        .collect();
    topo.add_face(remus_topology::face::Face::new(outer, inner, surface))
}

/// Negative controls on the same carrier and edges: cycles that wind the tube
/// period without a seam path joining them bound no chart region and must
/// still be refused, whether they touch the outer wire or not.
#[test]
fn winding_cycles_without_a_seam_path_still_refuse() {
    let (mut topo, result) = unified(imported, false, ENTRIES[0].1);
    let face_id = merged_torus_face(&topo, result);
    let face = topo.face(face_id).unwrap();
    let wires: Vec<Vec<_>> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .map(|w| topo.wire(w).unwrap().edges().to_vec())
        .collect();
    let twice = |wire: &[remus_topology::wire::OrientedEdge]| {
        wire.iter()
            .filter(|oe| wire.iter().filter(|o| o.edge() == oe.edge()).count() == 2)
            .count()
    };
    // The outer wire is one band boundary between the two uses of its slit;
    // the inner wires are the drilled hole, the other slit and the other band
    // boundary (face order is not assumed).
    let outer = &wires[0];
    assert_eq!(twice(outer), 2);
    let band_a: Vec<_> = outer
        .iter()
        .filter(|oe| outer.iter().filter(|o| o.edge() == oe.edge()).count() == 1)
        .copied()
        .collect();
    let slit = wires[1..]
        .iter()
        .find(|w| twice(w) == w.len())
        .unwrap()
        .clone();
    let starts = |wire: &[remus_topology::wire::OrientedEdge]| {
        wire.iter()
            .map(|oe| oe.oriented_start(topo.edge(oe.edge()).unwrap()))
            .collect::<Vec<_>>()
    };
    let slit_vertices = starts(&slit);
    // The other band boundary is the cycle the slit touches; the drilled
    // hole touches nothing.
    let (band_b, drilled): (Vec<_>, Vec<_>) = wires[1..]
        .iter()
        .filter(|w| twice(w) == 0)
        .cloned()
        .partition(|w| starts(w).iter().any(|v| slit_vertices.contains(v)));
    assert_eq!((band_b.len(), drilled.len()), (1, 1));

    let expect_refusal =
        |topo: &Topology, face, reason: &str| match remus_operations::tessellate::tessellate(
            topo, face, 0.004,
        ) {
            Err(OperationsError::InvalidInput { reason: got }) => assert_eq!(got, reason),
            other => panic!("expected refusal {reason:?}, got {other:?}"),
        };
    // Both band boundaries and the hole, no slits: an annulus, nothing touches.
    let annulus = torus_face_from(
        &mut topo,
        face_id,
        band_a,
        vec![drilled[0].clone(), band_b[0].clone()],
    );
    expect_refusal(&topo, annulus, "torus hole winds a parameter period");
    // One band boundary and its slit to the other slit, without the other
    // band boundary: the touching slit splices, the walk still winds.
    let open = torus_face_from(
        &mut topo,
        face_id,
        outer.clone(),
        vec![slit, drilled[0].clone()],
    );
    expect_refusal(&topo, open, "torus boundary winds a parameter period");
}
