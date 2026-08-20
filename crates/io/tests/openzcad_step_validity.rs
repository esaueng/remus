//! OpenZCAD parity-corpus STEP solids must import as valid analytic B-Reps.

#![allow(clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_operations::measure::solid_volume;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;
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
fn openzcad_analytic_fillet_plate_imports_as_one_valid_analytic_solid() {
    assert_valid_round_trip(
        FILLETED_PLATE,
        9_522.606_928_409_188,
        &[("cylinder", 4), ("plane", 6)],
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
