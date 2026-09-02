//! OpenZCAD parity-corpus STEP solids must import as valid analytic B-Reps.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeMap;

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Vec2};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_cylinder;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::PeriodicWinding;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_boundary_authority;
use remus_topology::validation::validate_shell_closed;

const BORED_PLATE: &str = include_str!("data/openzcad_a_export_bored_plate.step");
const FILLETED_PLATE: &str = include_str!("data/openzcad_e_analytic_fillet_plate.step");
const NURBS_PLATE: &str = include_str!("data/openzcad_e_nurbs_fillet_plate.step");

fn import_one(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).expect("import STEP fixture");
    assert_eq!(solids.len(), 1, "fixture must contain exactly one solid");
    (topo, solids[0])
}

fn surface_census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for face in solid_faces(topo, solid).expect("solid faces") {
        *census
            .entry(topo.face(face).expect("face").surface().type_tag())
            .or_default() += 1;
    }
    census
}

fn assert_valid_round_trip(
    step: &str,
    expected_volume: f64,
    expected_census: &[(&'static str, usize)],
) {
    let (topo, solid) = import_one(step);
    let shell = topo
        .shell(topo.solid(solid).expect("solid").outer_shell())
        .expect("outer shell");
    validate_shell_closed(shell, &topo).expect("closed imported shell");

    let report = validate_solid(&topo, solid).expect("validate imported solid");
    assert!(
        report.is_valid(),
        "valid STEP fixture was rejected: {:?}",
        report.issues
    );
    let volume = solid_volume(&topo, solid, 0.05).expect("imported volume");
    assert!(
        (volume - expected_volume).abs() <= expected_volume * 1e-10,
        "imported volume {volume:.12} != {expected_volume:.12}"
    );
    let expected: BTreeMap<_, _> = expected_census.iter().copied().collect();
    assert_eq!(surface_census(&topo, solid), expected);

    let exported = write_step(&topo, &[solid]).expect("write round-trip STEP");
    assert!(exported.contains("MANIFOLD_SOLID_BREP"));
    let (round_topo, round_solid) = import_one(&exported);
    let round_report = validate_solid(&round_topo, round_solid).expect("validate round-trip solid");
    assert!(
        round_report.is_valid(),
        "round-trip STEP was rejected: {:?}",
        round_report.issues
    );
    let round_volume = solid_volume(&round_topo, round_solid, 0.05).expect("round-trip volume");
    assert!(
        (round_volume - volume).abs() <= volume.abs().max(1.0) * 1e-10,
        "round-trip volume {round_volume:.12} != source {volume:.12}"
    );
    assert_eq!(surface_census(&round_topo, round_solid), expected);
}

#[test]
fn openzcad_bored_plate_imports_as_one_valid_analytic_solid() {
    assert_valid_round_trip(
        BORED_PLATE,
        8_814.601_836_602_553,
        &[("cylinder", 1), ("plane", 6)],
    );
}

#[test]
fn openzcad_bored_plate_remains_valid_after_a_second_drill() {
    let (mut topo, solid) = import_one(BORED_PLATE);
    let tool = make_cylinder(&mut topo, 2.0, 10.4).expect("drill tool");
    transform_solid(&mut topo, tool, &Mat4::translation(8.0, 4.8, -0.2))
        .expect("position drill tool");

    let drilled = boolean(&mut topo, BooleanOp::Cut, solid, tool).expect("drill imported plate");
    let report = validate_solid(&topo, drilled).expect("validate drilled plate");
    assert!(
        report.is_valid(),
        "a conforming cut must not inherit inconsistent orientation from STEP: {:?}",
        report.issues
    );
}

#[test]
fn openzcad_analytic_fillet_plate_imports_as_one_valid_analytic_solid() {
    let (topo, solid) = import_one(FILLETED_PLATE);
    assert_eq!(
        topo.num_pcurves(),
        48,
        "fixture carries 48 positioned PCURVEs"
    );
    let imported_boundary =
        validate_boundary_authority(&topo).expect("imported whole-topology authority");
    assert_eq!(imported_boundary.faces, topo.num_faces());
    assert_eq!(imported_boundary.loops, topo.num_loops());
    assert_eq!(imported_boundary.coedges, topo.num_coedges());
    let first = write_step(&topo, &[solid]).expect("write pcurve-bearing STEP");
    assert_eq!(first.matches("PCURVE(").count(), 48);
    let (round_topo, round_solid) = import_one(&first);
    assert_eq!(round_topo.num_pcurves(), 48);
    let round_boundary =
        validate_boundary_authority(&round_topo).expect("round-trip whole-topology authority");
    assert_eq!(round_boundary.faces, round_topo.num_faces());
    assert_eq!(round_boundary.loops, round_topo.num_loops());
    assert_eq!(round_boundary.coedges, round_topo.num_coedges());
    let second = write_step(&round_topo, &[round_solid]).expect("write second STEP");
    assert_eq!(second, first, "write/read/write must be deterministic");

    assert_valid_round_trip(
        FILLETED_PLATE,
        9_522.606_928_409_188,
        &[("cylinder", 4), ("plane", 6)],
    );
}

#[test]
fn step_pcurve_branch_count_and_endpoints_fail_closed() {
    for (label, malformed) in [
        (
            "unused duplicate branch",
            FILLETED_PLATE.replacen(
                "#26 = SURFACE_CURVE('',#27,(#31,#43),.PCURVE_S1.);",
                "#26 = SURFACE_CURVE('',#27,(#31,#31,#43),.PCURVE_S1.);",
                1,
            ),
        ),
        (
            "off-edge pcurve",
            FILLETED_PLATE.replacen(
                "#39 = CARTESIAN_POINT('',(0.,0.));",
                "#39 = CARTESIAN_POINT('',(1.,0.));",
                1,
            ),
        ),
    ] {
        let mut topo = Topology::new();
        let before = (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_faces(),
            topo.num_coedges(),
            topo.num_pcurves(),
        );
        let error = read_step(&malformed, &mut topo)
            .expect_err("malformed per-use authority must fail the whole import");
        assert!(
            error.to_string().contains("PCURVE"),
            "{label} produced the wrong diagnostic: {error}"
        );
        assert_eq!(
            (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_faces(),
                topo.num_coedges(),
                topo.num_pcurves(),
            ),
            before,
            "{label} left partial topology behind"
        );
    }
}

#[test]
fn cylinder_seam_pcurves_round_trip_by_loop_position() {
    let mut topo = Topology::new();
    let solid = make_cylinder(&mut topo, 2.0, 5.0).expect("cylinder");
    let face = solid_faces(&topo, solid)
        .expect("faces")
        .into_iter()
        .find(|&face| {
            matches!(
                topo.face(face).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .expect("lateral face");
    let cylinder = match topo.face(face).unwrap().surface() {
        remus_topology::face::FaceSurface::Cylinder(cylinder) => cylinder.clone(),
        _ => unreachable!(),
    };
    let loop_id = topo.face(face).unwrap().outer_loop().expect("outer loop");
    let coedges = topo.face_loop(loop_id).unwrap().coedges().to_vec();
    let seam_edge = coedges
        .iter()
        .map(|&coedge| topo.coedge(coedge).unwrap().edge())
        .find(|&edge| {
            coedges
                .iter()
                .filter(|&&coedge| topo.coedge(coedge).unwrap().edge() == edge)
                .count()
                == 2
        })
        .expect("seam edge");
    let seam_uses: Vec<_> = coedges
        .into_iter()
        .filter(|&coedge| topo.coedge(coedge).unwrap().edge() == seam_edge)
        .collect();
    assert_eq!(seam_uses.len(), 2);
    let first_use = topo.coedge(seam_uses[0]).unwrap();
    let first_edge = topo.edge(first_use.edge()).unwrap();
    let first_vertex = if first_use.is_forward() {
        first_edge.start()
    } else {
        first_edge.end()
    };
    let base_u = cylinder
        .project_point(topo.vertex(first_vertex).unwrap().point())
        .0;
    for (index, coedge_id) in seam_uses.into_iter().enumerate() {
        let coedge = topo.coedge(coedge_id).unwrap();
        let edge = topo.edge(coedge.edge()).unwrap();
        let (start, end) = if coedge.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        let start = topo.vertex(start).unwrap().point();
        let end = topo.vertex(end).unwrap().point();
        let direction = (end.z() - start.z()).signum();
        let u = base_u
            + if index == 0 {
                0.0
            } else {
                std::f64::consts::TAU
            };
        let line =
            Line2D::new(Point2::new(u, start.z()), Vec2::new(0.0, direction)).expect("seam pcurve");
        topo.set_coedge_pcurve(
            coedge_id,
            PCurve::new(Curve2D::Line(line), 0.0, (end.z() - start.z()).abs()),
        )
        .unwrap();
        topo.set_coedge_periodic_winding(
            coedge_id,
            if index == 0 {
                PeriodicWinding::ZERO
            } else {
                PeriodicWinding::new(1, 0)
            },
        )
        .unwrap();
    }

    let first = write_step(&topo, &[solid]).expect("write seam STEP");
    assert_eq!(first.matches("PCURVE(").count(), 2);
    let (mut round_topo, round_solid) = import_one(&first);
    assert_eq!(round_topo.num_pcurves(), 2);
    let round_face = solid_faces(&round_topo, round_solid)
        .unwrap()
        .into_iter()
        .find(|&face| {
            matches!(
                round_topo.face(face).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder(_)
            )
        })
        .unwrap();
    let mut branches: Vec<_> = round_topo
        .face(round_face)
        .unwrap()
        .boundary_loops()
        .iter()
        .flat_map(|&loop_id| round_topo.face_loop(loop_id).unwrap().coedges())
        .filter_map(|&coedge_id| {
            round_topo.coedge_pcurve(coedge_id).unwrap().map(|pcurve| {
                (
                    pcurve.evaluate(pcurve.t_start()).x(),
                    round_topo.coedge(coedge_id).unwrap().periodic_winding().u(),
                )
            })
        })
        .collect();
    branches.sort_by(|left, right| left.0.total_cmp(&right.0));
    assert_eq!(branches.len(), 2);
    assert_eq!(branches[0].0.to_bits(), base_u.to_bits());
    assert_eq!(branches[0].1, 0);
    assert_eq!(
        branches[1].0.to_bits(),
        (base_u + std::f64::consts::TAU).to_bits()
    );
    assert_eq!(branches[1].1, 1);
    let second = write_step(&round_topo, &[round_solid]).expect("write seam STEP again");
    assert_eq!(second, first);

    let lifted_use = round_topo
        .face(round_face)
        .unwrap()
        .boundary_loops()
        .iter()
        .flat_map(|&loop_id| round_topo.face_loop(loop_id).unwrap().coedges())
        .copied()
        .find(|&coedge_id| round_topo.coedge(coedge_id).unwrap().periodic_winding().u() == 1)
        .expect("lifted seam use");
    round_topo
        .set_coedge_periodic_winding(lifted_use, PeriodicWinding::ZERO)
        .unwrap();
    let error = write_step(&round_topo, &[round_solid])
        .expect_err("inconsistent winding metadata must not be exported");
    assert!(
        error
            .to_string()
            .contains("disagrees with its pcurve branch")
    );
}

/// The K0.1 "spline accuracy" question, resolved: Remus reports the FILE, not
/// the design intent. The corpus's `e-nurbs-fillet-plate` encodes its four
/// corner fillet bands as degree-2 NON-RATIONAL B-splines — a quadratic
/// Bezier cannot carry a circular arc, and with its middle control point at
/// the corner-tangent intersection each band is a parabola. Per corner the
/// parabola removes 1.5 mm² (tangent triangle 4.5 minus the parabola-chord
/// area (2/3)·4.5 = 3) where the true r=3 arc removes 9·(1 − π/4) ≈ 1.9314
/// mm², so the file's exact content is 40·24·10 − 4·1.5·10 = 9540.0 mm³ —
/// +0.181% above the closed-form intent 9522.7433388. The corpus pin's
/// +0.16% for Remus is this file deviation minus a small inscribed-mesh
/// undercount; OCCT's 9500.0 matches neither the file nor the intent.
#[test]
fn openzcad_nurbs_fillet_plate_volume_reports_the_files_parabolic_content() {
    use remus_operations::tessellate::tessellate_solid_with_tolerance;
    use remus_topology::face::FaceSurface;

    const FILE_CONTENT: f64 = 9_540.0; // 9600 − 4 corners · 1.5 mm² · 10 mm
    const CLOSED_FORM_INTENT: f64 = 9_522.743_338_8;

    let (topo, solid) = import_one(NURBS_PLATE);
    let expected: BTreeMap<_, _> = [("nurbs", 4), ("plane", 6)].into_iter().collect();
    assert_eq!(surface_census(&topo, solid), expected);

    // The fixture stores one band with ADVANCED_FACE.same_sense = .F.
    // and its loop in face sense per ISO 10303-42; strict validation
    // guards the reader's same_sense composition on B-spline faces.
    let report = validate_solid(&topo, solid).expect("validate imported solid");
    assert!(
        report.is_valid(),
        "reversed-NURBS import failed strict validation: {:?}",
        report.issues
    );

    // The premise: four degree-(2,1) non-rational bands — parabolas, not arcs.
    for fid in solid_faces(&topo, solid).expect("faces") {
        if let FaceSurface::Nurbs(surf) = topo.face(fid).expect("face").surface() {
            assert_eq!(
                (surf.degree_u(), surf.degree_v()),
                (2, 1),
                "corner band degree changed; re-derive the parabolic arithmetic"
            );
            assert!(!surf.is_rational(), "a rational band could be an exact arc");
        }
    }

    // Fine mesh converges on the file's content...
    let mesh = tessellate_solid_with_tolerance(&topo, solid, 1e-3, 0.5).expect("mesh");
    let mut fine = 0.0;
    for tri in mesh.indices.chunks(3) {
        let p0 = mesh.positions[tri[0] as usize];
        let p1 = mesh.positions[tri[1] as usize];
        let p2 = mesh.positions[tri[2] as usize];
        let a = remus_math::vec::Vec3::new(p0.x(), p0.y(), p0.z());
        let b = remus_math::vec::Vec3::new(p1.x(), p1.y(), p1.z());
        let c = remus_math::vec::Vec3::new(p2.x(), p2.y(), p2.z());
        fine += a.dot(b.cross(c)) / 6.0;
    }
    assert!(
        (fine - FILE_CONTENT).abs() <= FILE_CONTENT * 3e-4,
        "fine-mesh volume {fine:.4} strayed from the file's exact content {FILE_CONTENT}"
    );
    // ...which is distinctly ABOVE the design intent: measuring ~9522.74 here
    // would mean the reader refit the bands to arcs instead of keeping the file.
    assert!(
        fine > CLOSED_FORM_INTENT + 10.0,
        "volume {fine:.4} no longer separates file content from design intent"
    );

    // The wasm-facing measure path stays within 0.05% of the file content.
    let measured = solid_volume(&topo, solid, 0.01).expect("solid_volume");
    assert!(
        (measured - FILE_CONTENT).abs() <= FILE_CONTENT * 5e-4,
        "solid_volume {measured:.4} strayed from the file content {FILE_CONTENT}"
    );
}
