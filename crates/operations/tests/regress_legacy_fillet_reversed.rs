//! Legacy bevel and variable-radius rebuilds must use outward plane normals
//! while preserving the original reversal and wire order of a cut's faces.
//!
//! A groove supplies both reversed support planes and reversed planes far
//! from the selected edge. Plain-box controls pin the unreversed behavior.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, deprecated)]

use std::f64::consts::FRAC_PI_4;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::BlendError;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::fillet::{FilletRadiusLaw, fillet, fillet_variable};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::{tessellate_solid, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const RADIUS: f64 = 0.5;
const LENGTH: f64 = 20.0;
const BOX_VOLUME: f64 = 20.0 * 10.0 * 40.0;
const CUT_VOLUME: f64 = BOX_VOLUME - 20.0 * 1.5 * 3.0;

#[derive(Clone, Copy, Debug)]
enum Target {
    Remote,
    LowerLip,
    UpperLip,
}

impl Target {
    fn yz(self) -> (f64, f64) {
        match self {
            Self::Remote => (0.0, 0.0),
            Self::LowerLip => (10.0, 26.0),
            Self::UpperLip => (10.0, 29.0),
        }
    }

    fn point_in_corner(self, x: f64, distance: f64) -> Point3 {
        match self {
            Self::Remote => Point3::new(x, distance, distance),
            Self::LowerLip => Point3::new(x, 10.0 - distance, 26.0 - distance),
            Self::UpperLip => Point3::new(x, 10.0 - distance, 29.0 + distance),
        }
    }
}

fn fixture(topo: &mut Topology, cut: bool, target: Target) -> (SolidId, EdgeId) {
    let blank = make_box(topo, LENGTH, 10.0, 40.0).unwrap();
    let solid = if cut {
        let tool = make_box(topo, 22.0, 2.5, 3.0).unwrap();
        transform_solid(topo, tool, &Mat4::translation(-1.0, 8.5, 26.0)).unwrap();
        boolean(topo, BooleanOp::Cut, blank, tool).unwrap()
    } else {
        blank
    };
    let reversed = reversed_plane_count(topo, solid);
    assert_eq!(reversed, if cut { 3 } else { 0 });
    let (y, z) = target.yz();
    let candidates: Vec<_> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let edge = topo.edge(id).unwrap();
            [edge.start(), edge.end()].into_iter().all(|vertex| {
                let p = topo.vertex(vertex).unwrap().point();
                (p.y() - y).abs() < 1e-7 && (p.z() - z).abs() < 1e-7
            })
        })
        .collect();
    assert_eq!(candidates.len(), 1, "{target:?}: unique straight spine");
    (solid, candidates[0])
}

fn reversed_plane_count(topo: &Topology, solid: SolidId) -> usize {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&id| {
            let face = topo.face(id).unwrap();
            face.is_reversed() && matches!(face.surface(), FaceSurface::Plane { .. })
        })
        .count()
}

fn assert_classification(
    topo: &Topology,
    solid: SolidId,
    point: Point3,
    expected: PointClassification,
) {
    assert_eq!(
        classify_point(topo, solid, point, &ClassifyOptions::default()).unwrap(),
        expected,
        "probe {point:?}"
    );
}

fn assert_result(
    topo: &Topology,
    input: SolidId,
    result: SolidId,
    cut: bool,
    target: Target,
    removed: f64,
    variable: bool,
) {
    assert_ne!(input, result);
    let before_faces = solid_faces(topo, input).unwrap();
    let after_faces = solid_faces(topo, result).unwrap();
    assert_eq!(after_faces.len(), before_faces.len() + 1);
    assert_eq!(
        reversed_plane_count(topo, result),
        reversed_plane_count(topo, input),
        "rebuilding a face must preserve its reversal"
    );
    let nurbs = after_faces
        .iter()
        .filter(|&&id| matches!(topo.face(id).unwrap().surface(), FaceSurface::Nurbs(_)))
        .count();
    assert_eq!(nurbs, usize::from(variable));

    for solid in [input, result] {
        let report = validate_solid(topo, solid).unwrap();
        assert!(report.is_valid(), "invalid solid: {:?}", report.issues);
        let quality = welded_mesh_quality(&tessellate_solid(topo, solid, 0.01).unwrap());
        assert_eq!(quality.boundary_edges, 0, "mesh not watertight");
        assert_eq!(quality.non_manifold_edges, 0, "mesh non-manifold");
    }
    let original = if cut { CUT_VOLUME } else { BOX_VOLUME };
    for deflection in [1e-3, 5e-4] {
        let before = solid_volume(topo, input, deflection).unwrap();
        assert!((before - original).abs() < 1e-7, "input changed");
        let actual_removed = before - solid_volume(topo, result, deflection).unwrap();
        // The variable path retains its existing chord-trimmed end caps
        // (B61). Bound that known approximation to 5% of the small removed
        // sliver, independently of the much larger body's total volume.
        let error = if variable { 0.05 * removed } else { 1e-7 };
        assert!(
            (actual_removed - removed).abs() < error,
            "{target:?}: removed {actual_removed}, closed form {removed} at {deflection}"
        );
    }
    let removed_probe = target.point_in_corner(10.0, 0.05);
    let kept_probe = target.point_in_corner(10.0, 0.45);
    assert_classification(topo, input, removed_probe, PointClassification::Inside);
    assert_classification(topo, result, removed_probe, PointClassification::Outside);
    assert_classification(topo, result, kept_probe, PointClassification::Inside);
    if cut {
        for solid in [input, result] {
            assert_classification(
                topo,
                solid,
                Point3::new(10.0, 9.0, 27.5),
                PointClassification::Outside,
            );
            assert_classification(
                topo,
                solid,
                Point3::new(10.0, 8.0, 27.5),
                PointClassification::Inside,
            );
        }
    }
}

#[test]
fn flat_bevel_preserves_reversed_planes_on_cut_bodies() {
    for target in [Target::Remote, Target::LowerLip, Target::UpperLip] {
        let mut topo = Topology::new();
        let (input, edge) = fixture(&mut topo, true, target);
        let result = fillet(&mut topo, input, &[edge], RADIUS).unwrap();
        assert_result(
            &topo,
            input,
            result,
            true,
            target,
            0.5 * RADIUS * RADIUS * LENGTH,
            false,
        );
    }
}

fn variable_case(cut: bool, target: Target, start: f64, end: f64) {
    let mut topo = Topology::new();
    let (input, edge) = fixture(&mut topo, cut, target);
    let law = if (start - end).abs() < 1e-12 {
        FilletRadiusLaw::Constant(start)
    } else {
        FilletRadiusLaw::Linear { start, end }
    };
    let result = fillet_variable(&mut topo, input, &[(edge, law)]).unwrap();
    // Integral of the square of a linear radius law over the spine.
    let removed = (1.0 - FRAC_PI_4) * LENGTH * (start * start + start * end + end * end) / 3.0;
    assert_result(&topo, input, result, cut, target, removed, true);

    // Read the spine's actual parameter direction: the upper and lower
    // groove lips need not have the same start/end order. At each end, a
    // diagonal point crosses a quarter-circle fillet at (1 - 1/sqrt(2)) r.
    // The 0.2r/0.4r pair also detects an accidentally swapped linear law.
    let spine = topo.edge(edge).unwrap();
    let x_start = topo.vertex(spine.start()).unwrap().point().x();
    let x_end = topo.vertex(spine.end()).unwrap().point().x();
    for x in [1.0, 19.0] {
        let fraction = (x - x_start) / (x_end - x_start);
        let local_radius = (end - start).mul_add(fraction, start);
        assert_classification(
            &topo,
            result,
            target.point_in_corner(x, 0.2 * local_radius),
            PointClassification::Outside,
        );
        assert_classification(
            &topo,
            result,
            target.point_in_corner(x, 0.4 * local_radius),
            PointClassification::Inside,
        );
    }
}

#[test]
fn constant_variable_fillet_preserves_reversed_planes_on_cut_bodies() {
    for target in [Target::Remote, Target::LowerLip, Target::UpperLip] {
        variable_case(true, target, RADIUS, RADIUS);
    }
}

#[test]
fn linear_variable_fillet_preserves_reversed_planes_on_cut_bodies() {
    for target in [Target::Remote, Target::LowerLip, Target::UpperLip] {
        variable_case(true, target, 0.3, 0.7);
    }
}

#[test]
fn variable_fillet_still_refuses_reversed_support_cliffs_without_mutation() {
    for target in [Target::LowerLip, Target::UpperLip] {
        for radius in [1.5, 2.0] {
            let mut topo = Topology::new();
            let (input, edge) = fixture(&mut topo, true, target);
            let counts = (
                topo.num_vertices(),
                topo.num_edges(),
                topo.num_wires(),
                topo.num_faces(),
                topo.num_shells(),
                topo.num_solids(),
                topo.num_loops(),
                topo.num_coedges(),
                topo.num_pcurves(),
            );
            let error = fillet_variable(
                &mut topo,
                input,
                &[(edge, FilletRadiusLaw::Constant(radius))],
            )
            .unwrap_err();
            let OperationsError::Blend(BlendError::CliffEncountered {
                edge: cliff_edge,
                face,
                requested_radius,
                available_radius,
            }) = error
            else {
                panic!("expected reversed-support cliff, got {error}");
            };
            assert_eq!(cliff_edge, edge);
            assert!(topo.face(face).unwrap().is_reversed());
            assert!((requested_radius - radius).abs() < 1e-12);
            assert!((available_radius - 1.5).abs() < 1e-9);
            assert_eq!(
                counts,
                (
                    topo.num_vertices(),
                    topo.num_edges(),
                    topo.num_wires(),
                    topo.num_faces(),
                    topo.num_shells(),
                    topo.num_solids(),
                    topo.num_loops(),
                    topo.num_coedges(),
                    topo.num_pcurves(),
                ),
                "refusal must roll back the arena"
            );
            assert!(validate_solid(&topo, input).unwrap().is_valid());
            assert!((solid_volume(&topo, input, 1e-3).unwrap() - CUT_VOLUME).abs() < 1e-7);
        }
    }
}

#[test]
fn legacy_unreversed_box_controls() {
    let mut topo = Topology::new();
    let (input, edge) = fixture(&mut topo, false, Target::Remote);
    let result = fillet(&mut topo, input, &[edge], RADIUS).unwrap();
    assert_result(
        &topo,
        input,
        result,
        false,
        Target::Remote,
        0.5 * RADIUS * RADIUS * LENGTH,
        false,
    );
    variable_case(false, Target::Remote, RADIUS, RADIUS);
    variable_case(false, Target::Remote, 0.3, 0.7);
}
