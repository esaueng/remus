//! STEP regressions for exact analytic blend resizing and refusal.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeMap;

use remus_check::validate::{CheckId, ValidateOptions, validate_solid};
use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_operations::blend_ops::fillet_v2;
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::resize_blend::{blend_region, resize_blend, resize_blend_failure_code};
use remus_operations::tessellate::{
    is_watertight, tessellate_solid_with_tolerance, welded_mesh_quality,
};
use remus_operations::validate::validate_solid as validate_operations_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

const DEFLECTION: f64 = 0.01;
const HAMMER_HOLDER_VOLUME: f64 = 50_240.482_852_844_82;

fn assert_valid(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid, &ValidateOptions::default()).expect("validate solid");
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
}

fn assert_valid_with_unproved_step_pcurves(topo: &Topology, solid: SolidId) {
    let mut options = ValidateOptions::default();
    options.disabled_checks.insert(CheckId::EdgeSameParameter);
    let report = validate_solid(topo, solid, &options).expect("validate STEP solid");
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
}

fn volume(topo: &Topology, solid: SolidId) -> f64 {
    solid_volume(topo, solid, DEFLECTION).expect("measure volume")
}

fn blend_face(topo: &Topology, solid: SolidId, radius: f64) -> FaceId {
    let matches: Vec<FaceId> = solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .filter(|face| match topo.face(*face).expect("face").surface() {
            FaceSurface::Cylinder(cylinder) => {
                Tolerance::new().approx_eq(cylinder.radius(), radius)
            }
            FaceSurface::Torus(torus) => Tolerance::new().approx_eq(torus.minor_radius(), radius),
            _ => false,
        })
        .collect();
    assert_eq!(matches.len(), 1, "one analytic r={radius} blend face");
    matches[0]
}

fn surface_census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut census = BTreeMap::new();
    for face in solid_faces(topo, solid).expect("solid faces") {
        *census
            .entry(topo.face(face).expect("face").surface().type_tag())
            .or_insert(0) += 1;
    }
    census
}

fn assert_hammer_holder_census(topo: &Topology, solid: SolidId) {
    assert_eq!(
        surface_census(topo, solid),
        BTreeMap::from([
            ("cone", 2),
            ("cylinder", 42),
            ("nurbs", 42),
            ("plane", 52),
            ("sphere", 8),
            ("torus", 14),
        ])
    );
    let faces = solid_faces(topo, solid).expect("solid faces");
    assert_eq!(
        faces
            .iter()
            .filter(|face| matches!(
                topo.face(**face).expect("face").surface(),
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), 3.0)
            ))
            .count(),
        32,
        "fixture's radius-3 cylinder census"
    );
    assert_eq!(
        faces
            .iter()
            .filter(|face| matches!(
                topo.face(**face).expect("face").surface(),
                FaceSurface::Torus(torus)
                    if Tolerance::new().approx_eq(torus.minor_radius(), 3.0)
            ))
            .count(),
        12,
        "fixture's radius-3 torus census"
    );
}

fn assert_hammer_holder_strict(topo: &Topology, solid: SolidId) {
    let report = validate_operations_solid(topo, solid).expect("strict operations validation");
    assert!(report.is_valid(), "validation issues: {:?}", report.issues);
    assert_eq!(report.warning_count(), 0, "strict validation warnings");

    let solid_data = topo.solid(solid).expect("solid");
    for shell in
        std::iter::once(solid_data.outer_shell()).chain(solid_data.inner_shells().iter().copied())
    {
        let shell_data = topo.shell(shell).expect("shell");
        remus_topology::validation::validate_shell_closed(shell_data, topo)
            .expect("strict shell closure");
        remus_topology::validation::validate_shell_manifold(shell_data, topo)
            .expect("strict shell manifoldness");
    }
}

fn hammer_holder_resize_seed(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .expect("solid faces")
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).expect("face").surface(),
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), 3.0)
            )
        })
        .expect("radius-3 Shapr3D blend seed")
}

fn imported_box_fillet() -> String {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, 10.0, 10.0, 10.0).expect("box");
    let edge = solid_edges(&topo, sharp).expect("box edges")[0];
    let filleted = fillet_v2(&mut topo, sharp, &[edge], 1.0)
        .expect("fillet box")
        .solid;
    write_step(&topo, &[filleted]).expect("write STEP")
}

fn imported_trihedral_fillet() -> String {
    let mut topo = Topology::new();
    let sharp = make_box(&mut topo, 40.0, 40.0, 10.0).expect("box");
    let corner = Point3::new(40.0, 40.0, 10.0);
    let edges: Vec<EdgeId> = solid_edges(&topo, sharp)
        .expect("box edges")
        .into_iter()
        .filter(|edge| {
            let edge = topo.edge(*edge).expect("edge");
            [edge.start(), edge.end()].into_iter().any(|vertex| {
                (topo.vertex(vertex).expect("vertex").point() - corner).length()
                    <= Tolerance::new().linear
            })
        })
        .collect();
    assert_eq!(edges.len(), 3);
    let filleted = fillet_v2(&mut topo, sharp, &edges, 3.0)
        .expect("trihedral fillet")
        .solid;
    write_step(&topo, &[filleted]).expect("write STEP")
}

#[test]
fn imported_step_box_blend_grows_shrinks_and_removes_exactly() {
    let step = imported_box_fillet();

    for new_radius in [2.0, 0.5, 0.0] {
        let mut topo = Topology::new();
        let input = read_step(&step, &mut topo).expect("read STEP")[0];
        assert_valid(&topo, input);
        let band = blend_face(&topo, input, 1.0);
        let before = volume(&topo, input);

        let result = resize_blend(&mut topo, input, band, 1.0, new_radius)
            .expect("resize imported blend")
            .solid;
        assert_valid(&topo, result);
        let after = volume(&topo, result);

        if Tolerance::new().approx_eq(new_radius, 0.0) {
            assert_eq!(solid_entity_counts(&topo, result).expect("counts").0, 6);
            assert!(Tolerance::new().approx_eq(after, 1000.0));
        } else {
            let _ = blend_face(&topo, result, new_radius);
            if new_radius > 1.0 {
                assert!(after < before, "growing a convex blend removes volume");
            } else {
                assert!(after > before, "shrinking a convex blend restores volume");
            }
        }
    }
}

#[test]
fn imported_step_trihedral_region_resizes_with_corner_patch() {
    let step = imported_trihedral_fillet();

    for new_radius in [2.0, 4.0] {
        let mut topo = Topology::new();
        let input = read_step(&step, &mut topo).expect("read STEP")[0];
        assert_valid(&topo, input);
        let seed = solid_faces(&topo, input)
            .expect("faces")
            .into_iter()
            .find(|face| {
                matches!(
                    topo.face(*face).expect("face").surface(),
                    FaceSurface::Cylinder(cylinder)
                        if Tolerance::new().approx_eq(cylinder.radius(), 3.0)
                )
            })
            .expect("imported trihedral band");
        assert_eq!(
            blend_region(&topo, input, seed)
                .expect("region")
                .faces
                .len(),
            4
        );

        let result = resize_blend(&mut topo, input, seed, 3.0, new_radius)
            .expect("resize imported trihedral region")
            .solid;
        assert_valid(&topo, result);
        let mesh =
            tessellate_solid_with_tolerance(&topo, result, 0.01, 0.1).expect("tessellate result");
        assert!(is_watertight(&mesh));
        let faces = solid_faces(&topo, result).expect("result faces");
        assert_eq!(faces.len(), 10);
        assert_eq!(
            faces
                .iter()
                .filter(|face| matches!(
                    topo.face(**face).expect("face").surface(),
                    FaceSurface::Cylinder(cylinder)
                        if Tolerance::new().approx_eq(cylinder.radius(), new_radius)
                ))
                .count(),
            3
        );
        assert_eq!(
            faces
                .iter()
                .filter(|face| matches!(
                    topo.face(**face).expect("face").surface(),
                    FaceSurface::Sphere(sphere)
                        if Tolerance::new().approx_eq(sphere.radius(), new_radius)
                ))
                .count(),
            1
        );
    }
}

#[test]
fn shapr3d_hammer_holder_blend_refusal_preserves_exact_acceptance_contract() {
    let mut topo = Topology::new();
    let solids = read_step(include_str!("data/shapr3d_hammer_holder.step"), &mut topo)
        .expect("read Shapr3D hammer holder");
    assert_eq!(solids.len(), 1, "fixture must contain one solid");
    let input = solids[0];

    let seed = hammer_holder_resize_seed(&topo, input);
    let discovery_error = blend_region(&topo, input, seed).unwrap_err();
    assert_eq!(
        resize_blend_failure_code(&discovery_error),
        "band-touches-freeform"
    );
    let error = resize_blend(&mut topo, input, seed, 3.0, 2.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "band-touches-freeform");

    assert_hammer_holder_strict(&topo, input);
    assert_eq!(
        solid_entity_counts(&topo, input).expect("entity counts after refusal"),
        (160, 386, 238)
    );
    assert_hammer_holder_census(&topo, input);
    let after = volume(&topo, input);
    assert!(
        (after - HAMMER_HOLDER_VOLUME).abs() <= 0.01,
        "fixture volume {after} differs from {HAMMER_HOLDER_VOLUME}"
    );
    let mesh = tessellate_solid_with_tolerance(&topo, input, 0.1, 0.1)
        .expect("tessellate hammer holder after refusal");
    let quality = welded_mesh_quality(&mesh);
    assert!(
        quality.is_watertight(),
        "refusal must preserve a watertight manifold mesh ({} boundary, {} non-manifold edges)",
        quality.boundary_edges,
        quality.non_manifold_edges
    );
}

#[test]
fn shapr3d_periodic_step_unfillets_exactly_and_refuses_unimplemented_resize() {
    let mut topo = Topology::new();
    let input = read_step(
        include_str!("data/shapr3d_walking_stick_foot.step"),
        &mut topo,
    )
    .expect("read Shapr3D STEP")[0];
    assert_valid(&topo, input);
    let band = solid_faces(&topo, input)
        .expect("faces")
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).expect("face").surface(),
                FaceSurface::Torus(torus)
                    if Tolerance::new().approx_eq(torus.minor_radius(), 4.0)
            )
        })
        .expect("Shapr3D torus band");
    let counts = solid_entity_counts(&topo, input).expect("counts");
    let before = volume(&topo, input);

    let error = resize_blend(&mut topo, input, band, 4.0, 3.0).unwrap_err();
    assert_eq!(
        resize_blend_failure_code(&error),
        "unsupported-support-pair"
    );
    assert_eq!(solid_entity_counts(&topo, input).expect("counts"), counts);
    assert!(Tolerance::new().approx_eq(volume(&topo, input), before));

    let removed =
        resize_blend(&mut topo, input, band, 4.0, 0.0).expect("remove Shapr3D cylinder-cone band");
    assert_valid(&topo, removed.solid);
    assert!(removed.evolution.origin.is_exact());
    assert!(removed.evolution.deleted.contains(&band.index()));
    assert_eq!(
        solid_entity_counts(&topo, removed.solid)
            .expect("result counts")
            .0,
        counts.0 - 1
    );

    let removed_counts = solid_entity_counts(&topo, removed.solid).expect("removed counts");
    let removed_volume = volume(&topo, removed.solid);
    let step = write_step(&topo, &[removed.solid]).expect("write unfilleted Shapr3D STEP");
    let mut roundtrip_topo = Topology::new();
    let roundtrip = read_step(&step, &mut roundtrip_topo).expect("re-read unfilleted STEP")[0];
    assert_valid(&roundtrip_topo, roundtrip);
    assert_eq!(
        solid_entity_counts(&roundtrip_topo, roundtrip)
            .expect("round-trip counts")
            .0,
        removed_counts.0
    );
    assert!(Tolerance::new().approx_eq(volume(&roundtrip_topo, roundtrip), removed_volume));
}

#[test]
fn occt_multi_fillet_step_refuses_without_mutating_input() {
    let mut topo = Topology::new();
    let input = read_step(
        include_str!("data/openzcad_e_analytic_fillet_plate.step"),
        &mut topo,
    )
    .expect("read Open CASCADE STEP")[0];
    assert_valid_with_unproved_step_pcurves(&topo, input);
    let band = solid_faces(&topo, input)
        .expect("faces")
        .into_iter()
        .find(|face| {
            matches!(
                topo.face(*face).expect("face").surface(),
                FaceSurface::Cylinder(cylinder)
                    if Tolerance::new().approx_eq(cylinder.radius(), 3.0)
            )
        })
        .expect("Open CASCADE fillet band");
    let counts = solid_entity_counts(&topo, input).expect("counts");
    let before = volume(&topo, input);

    let error = resize_blend(&mut topo, input, band, 3.0, 2.0).unwrap_err();
    assert_eq!(resize_blend_failure_code(&error), "resize-blend-failed");
    assert_eq!(solid_entity_counts(&topo, input).expect("counts"), counts);
    assert!(Tolerance::new().approx_eq(volume(&topo, input), before));
    assert_valid_with_unproved_step_pcurves(&topo, input);
}
