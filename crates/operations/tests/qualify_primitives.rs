//! Qualification matrix for the analytic primitive constructors.
//!
//! Axes covered (B6 in `docs/kernel-maturity/roadmap.md`): primitive kind
//! (box, cylinder, pointed cone, frustum, sphere, torus, ellipsoid), scale
//! (`1e-3 / 1 / 1e3`), invalid-input boundaries, and the complete solid
//! postcondition set. Successes are checked against closed-form volume and
//! bounding-box oracles, exact analytic surface/entity censuses, both solid
//! validators, closed/oriented B-Rep topology, and an independently measured
//! closed/manifold tessellation. Rebuilding the matrix must be bit-for-bit
//! deterministic.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_math::tolerance::Tolerance;
use remus_operations::OperationsError;
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::tessellate::{tessellate_solid_with_tolerance, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];

#[derive(Clone, Copy, Debug)]
enum Primitive {
    Box,
    Cylinder,
    PointedCone,
    Frustum,
    Sphere,
    Torus,
    Ellipsoid,
}

impl Primitive {
    const ALL: [Self; 7] = [
        Self::Box,
        Self::Cylinder,
        Self::PointedCone,
        Self::Frustum,
        Self::Sphere,
        Self::Torus,
        Self::Ellipsoid,
    ];

    fn build(self, topo: &mut Topology, scale: f64) -> Result<SolidId, OperationsError> {
        match self {
            Self::Box => make_box(topo, 2.0 * scale, 3.0 * scale, 4.0 * scale),
            Self::Cylinder => make_cylinder(topo, 2.0 * scale, 5.0 * scale),
            Self::PointedCone => make_cone(topo, 3.0 * scale, 0.0, 4.0 * scale),
            Self::Frustum => make_cone(topo, 3.0 * scale, scale, 4.0 * scale),
            Self::Sphere => make_sphere(topo, 2.0 * scale, 16),
            Self::Torus => make_torus(topo, 4.0 * scale, scale, 16),
            Self::Ellipsoid => {
                let solid = make_sphere(topo, 1.0, 16)?;
                transform_solid(
                    topo,
                    solid,
                    &Mat4::scale(1.5 * scale, 2.0 * scale, 2.5 * scale),
                )?;
                Ok(solid)
            }
        }
    }

    fn expected_volume(self, scale: f64) -> f64 {
        let unit = match self {
            Self::Box => 24.0,
            Self::Cylinder => 20.0 * PI,
            Self::PointedCone => 12.0 * PI,
            Self::Frustum => 52.0 * PI / 3.0,
            Self::Sphere => 32.0 * PI / 3.0,
            Self::Torus => 8.0 * PI * PI,
            Self::Ellipsoid => 10.0 * PI,
        };
        unit * scale.powi(3)
    }

    fn expected_bounds(self, scale: f64) -> [f64; 6] {
        let unit = match self {
            Self::Box => [0.0, 0.0, 0.0, 2.0, 3.0, 4.0],
            Self::Cylinder => [-2.0, -2.0, 0.0, 2.0, 2.0, 5.0],
            Self::PointedCone | Self::Frustum => [-3.0, -3.0, 0.0, 3.0, 3.0, 4.0],
            Self::Sphere => [-2.0, -2.0, -2.0, 2.0, 2.0, 2.0],
            Self::Torus => [-5.0, -5.0, -1.0, 5.0, 5.0, 1.0],
            Self::Ellipsoid => [-1.5, -2.0, -2.5, 1.5, 2.0, 2.5],
        };
        unit.map(|value| value * scale)
    }

    fn expected_entities(self) -> (usize, usize, usize) {
        match self {
            Self::Box => (6, 12, 8),
            Self::Cylinder | Self::Frustum => (3, 3, 2),
            Self::PointedCone => (2, 2, 2),
            Self::Sphere | Self::Ellipsoid => (2, 16, 16),
            Self::Torus => (1, 2, 1),
        }
    }

    fn expected_surfaces(self) -> BTreeMap<&'static str, usize> {
        let entries: &[(&str, usize)] = match self {
            Self::Box => &[("plane", 6)],
            Self::Cylinder => &[("cylinder", 1), ("plane", 2)],
            Self::PointedCone => &[("cone", 1), ("plane", 1)],
            Self::Frustum => &[("cone", 1), ("plane", 2)],
            Self::Sphere => &[("sphere", 2)],
            Self::Torus => &[("torus", 1)],
            Self::Ellipsoid => &[("nurbs", 2)],
        };
        entries.iter().copied().collect()
    }

    const fn expected_euler(self) -> i64 {
        match self {
            Self::Torus => 0,
            _ => 2,
        }
    }

    const fn volume_tolerance(self) -> f64 {
        match self {
            Self::Ellipsoid => 1e-3,
            _ => 1e-9,
        }
    }
}

#[derive(Debug, Eq, PartialEq)]
struct Snapshot {
    entities: (usize, usize, usize),
    surfaces: BTreeMap<&'static str, usize>,
    volume: u64,
    bounds: [u64; 6],
    mesh_positions: Vec<[u64; 3]>,
    mesh_normals: Vec<[u64; 3]>,
    mesh_indices: Vec<u32>,
}

fn assert_relative(label: &str, actual: f64, expected: f64, limit: f64) {
    let relative = (actual - expected).abs() / expected.abs();
    assert!(
        relative <= limit,
        "{label}: expected {expected:.12e}, got {actual:.12e}, relative error {relative:.3e} > {limit:.3e}"
    );
}

fn surface_census(topo: &Topology, solid: SolidId) -> BTreeMap<&'static str, usize> {
    let mut result = BTreeMap::new();
    for face_id in solid_faces(topo, solid).unwrap() {
        let tag = topo.face(face_id).unwrap().surface().type_tag();
        *result.entry(tag).or_default() += 1;
    }
    result
}

fn signed_mesh_volume(mesh: &remus_operations::tessellate::TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|triangle| {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            let determinant = a.x() * (b.y() * c.z() - b.z() * c.y())
                - a.y() * (b.x() * c.z() - b.z() * c.x())
                + a.z() * (b.x() * c.y() - b.y() * c.x());
            determinant / 6.0
        })
        .sum::<f64>()
        .abs()
}

fn qualify(case: Primitive, scale: f64) -> Snapshot {
    let label = format!("{case:?} at scale {scale}");
    let mut topo = Topology::new();
    let solid = case.build(&mut topo, scale).unwrap();

    let operations_report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(
        operations_report.is_valid(),
        "{label}: L3 validation issues: {:?}",
        operations_report.issues
    );
    let check_report = remus_check::validate::validate_solid(
        &topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        check_report.is_valid(),
        "{label}: check validation issues: {:?}",
        check_report.issues
    );

    let shell_id = topo.solid(solid).unwrap().outer_shell();
    let shell = topo.shell(shell_id).unwrap();
    validate_shell_closed(shell, &topo).unwrap();
    assert!(
        remus_check::validate::shell::check_shell_orientation(&topo, shell_id)
            .unwrap()
            .is_empty(),
        "{label}: shared edges must have opposite effective senses"
    );

    let entities = solid_entity_counts(&topo, solid).unwrap();
    assert_eq!(entities, case.expected_entities(), "{label}: entity census");
    #[allow(clippy::cast_possible_wrap)]
    let euler = entities.2 as i64 - entities.1 as i64 + entities.0 as i64;
    assert_eq!(euler, case.expected_euler(), "{label}: B-Rep Euler");

    let surfaces = surface_census(&topo, solid);
    assert_eq!(
        surfaces,
        case.expected_surfaces(),
        "{label}: surface census"
    );

    let expected_volume = case.expected_volume(scale);
    let volume = solid_volume(&topo, solid, 0.01 * scale).unwrap();
    assert_relative(
        &format!("{label}: B-Rep volume"),
        volume,
        expected_volume,
        case.volume_tolerance(),
    );

    let bounds = solid_bounding_box(&topo, solid).unwrap();
    let actual_bounds = [
        bounds.min.x(),
        bounds.min.y(),
        bounds.min.z(),
        bounds.max.x(),
        bounds.max.y(),
        bounds.max.z(),
    ];
    for (axis, (actual, expected)) in actual_bounds
        .iter()
        .zip(case.expected_bounds(scale))
        .enumerate()
    {
        let limit = 1e-10 * scale.max(1.0);
        assert!(
            (actual - expected).abs() <= limit,
            "{label}: bound {axis} expected {expected:.12e}, got {actual:.12e}"
        );
    }

    let mesh = tessellate_solid_with_tolerance(&topo, solid, 0.01 * scale, 0.1).unwrap();
    assert!(
        mesh.positions
            .iter()
            .all(|point| point.x().is_finite() && point.y().is_finite() && point.z().is_finite()),
        "{label}: mesh positions must be finite"
    );
    assert!(
        mesh.normals.iter().all(|normal| normal.x().is_finite()
            && normal.y().is_finite()
            && normal.z().is_finite()),
        "{label}: mesh normals must be finite"
    );
    assert!(
        mesh.indices
            .iter()
            .all(|&index| (index as usize) < mesh.positions.len()),
        "{label}: mesh indices must be in range"
    );
    let quality = welded_mesh_quality(&mesh);
    assert!(quality.is_watertight(), "{label}: mesh quality {quality:?}");
    assert_eq!(
        quality.euler_characteristic,
        case.expected_euler(),
        "{label}: mesh Euler"
    );
    assert_relative(
        &format!("{label}: independently integrated mesh volume"),
        signed_mesh_volume(&mesh),
        expected_volume,
        0.02,
    );

    Snapshot {
        entities,
        surfaces,
        volume: volume.to_bits(),
        bounds: actual_bounds.map(f64::to_bits),
        mesh_positions: mesh
            .positions
            .iter()
            .map(|point| {
                [
                    point.x().to_bits(),
                    point.y().to_bits(),
                    point.z().to_bits(),
                ]
            })
            .collect(),
        mesh_normals: mesh
            .normals
            .iter()
            .map(|normal| {
                [
                    normal.x().to_bits(),
                    normal.y().to_bits(),
                    normal.z().to_bits(),
                ]
            })
            .collect(),
        mesh_indices: mesh.indices,
    }
}

#[test]
fn primitive_family_is_qualified_across_kind_and_scale() {
    for case in Primitive::ALL {
        for scale in SCALES {
            let first = qualify(case, scale);
            let second = qualify(case, scale);
            assert_eq!(
                first, second,
                "{case:?} at scale {scale}: rebuild must be deterministic"
            );
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum InvalidCase {
    BoxZero,
    BoxNegative,
    BoxNan,
    BoxInfinite,
    CylinderZeroRadius,
    CylinderZeroHeight,
    CylinderNan,
    ConeBothRadiiZero,
    ConeNegativeRadius,
    ConeNan,
    ConeInfiniteHeight,
    SphereZeroRadius,
    SphereNan,
    SphereTooFewSegments,
    TorusZeroMajor,
    TorusNan,
    TorusInfinite,
    TorusEqualRadii,
    TorusInvertedRadii,
    TorusTooFewSegments,
}

impl InvalidCase {
    const ALL: [Self; 20] = [
        Self::BoxZero,
        Self::BoxNegative,
        Self::BoxNan,
        Self::BoxInfinite,
        Self::CylinderZeroRadius,
        Self::CylinderZeroHeight,
        Self::CylinderNan,
        Self::ConeBothRadiiZero,
        Self::ConeNegativeRadius,
        Self::ConeNan,
        Self::ConeInfiniteHeight,
        Self::SphereZeroRadius,
        Self::SphereNan,
        Self::SphereTooFewSegments,
        Self::TorusZeroMajor,
        Self::TorusNan,
        Self::TorusInfinite,
        Self::TorusEqualRadii,
        Self::TorusInvertedRadii,
        Self::TorusTooFewSegments,
    ];

    fn build(self, topo: &mut Topology) -> Result<SolidId, OperationsError> {
        match self {
            Self::BoxZero => make_box(topo, 0.0, 1.0, 1.0),
            Self::BoxNegative => make_box(topo, 1.0, -1.0, 1.0),
            Self::BoxNan => make_box(topo, f64::NAN, 1.0, 1.0),
            Self::BoxInfinite => make_box(topo, 1.0, f64::INFINITY, 1.0),
            Self::CylinderZeroRadius => make_cylinder(topo, 0.0, 1.0),
            Self::CylinderZeroHeight => make_cylinder(topo, 1.0, 0.0),
            Self::CylinderNan => make_cylinder(topo, f64::NAN, 1.0),
            Self::ConeBothRadiiZero => make_cone(topo, 0.0, 0.0, 1.0),
            Self::ConeNegativeRadius => make_cone(topo, -1.0, 1.0, 1.0),
            Self::ConeNan => make_cone(topo, 1.0, f64::NAN, 1.0),
            Self::ConeInfiniteHeight => make_cone(topo, 1.0, 0.5, f64::INFINITY),
            Self::SphereZeroRadius => make_sphere(topo, 0.0, 16),
            Self::SphereNan => make_sphere(topo, f64::NAN, 16),
            Self::SphereTooFewSegments => make_sphere(topo, 1.0, 3),
            Self::TorusZeroMajor => make_torus(topo, 0.0, 1.0, 16),
            Self::TorusNan => make_torus(topo, f64::NAN, 1.0, 16),
            Self::TorusInfinite => make_torus(topo, f64::INFINITY, 1.0, 16),
            Self::TorusEqualRadii => make_torus(topo, 2.0, 2.0, 16),
            Self::TorusInvertedRadii => make_torus(topo, 1.0, 2.0, 16),
            Self::TorusTooFewSegments => make_torus(topo, 3.0, 1.0, 3),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ArenaCounts {
    vertices: usize,
    edges: usize,
    wires: usize,
    faces: usize,
    shells: usize,
    solids: usize,
    loops: usize,
    coedges: usize,
}

fn arena_counts(topo: &Topology) -> ArenaCounts {
    ArenaCounts {
        vertices: topo.num_vertices(),
        edges: topo.num_edges(),
        wires: topo.num_wires(),
        faces: topo.num_faces(),
        shells: topo.num_shells(),
        solids: topo.num_solids(),
        loops: topo.num_loops(),
        coedges: topo.num_coedges(),
    }
}

#[test]
fn invalid_input_matrix_is_typed_and_non_mutating() {
    let mut topo = Topology::new();
    let existing = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    let before = arena_counts(&topo);
    let before_volume = solid_volume(&topo, existing, 0.01).unwrap();

    for case in InvalidCase::ALL {
        assert!(
            matches!(
                case.build(&mut topo),
                Err(OperationsError::InvalidInput { .. })
            ),
            "{case:?} must return the stable InvalidInput variant"
        );
        assert_eq!(
            arena_counts(&topo),
            before,
            "{case:?} must not allocate partial topology"
        );
        assert_eq!(
            solid_volume(&topo, existing, 0.01).unwrap().to_bits(),
            before_volume.to_bits(),
            "{case:?} must not change an existing solid"
        );
    }
}

#[test]
fn declared_input_boundaries_are_tested_from_both_sides() {
    let minimum = Tolerance::new().linear;

    let mut topo = Topology::new();
    assert!(make_box(&mut topo, minimum, 1.0, 1.0).is_err());
    assert!(make_box(&mut topo, 2.0 * minimum, 1.0, 1.0).is_ok());

    let mut topo = Topology::new();
    assert!(make_cylinder(&mut topo, minimum, 1.0).is_err());
    assert!(make_cylinder(&mut topo, 2.0 * minimum, 1.0).is_ok());

    let mut topo = Topology::new();
    assert!(make_cone(&mut topo, minimum, 0.0, 1.0).is_err());
    assert!(make_cone(&mut topo, 2.0 * minimum, 0.0, 1.0).is_ok());

    let mut topo = Topology::new();
    assert!(make_sphere(&mut topo, minimum, 16).is_err());
    assert!(make_sphere(&mut topo, 2.0 * minimum, 16).is_ok());
    assert!(make_sphere(&mut Topology::new(), 1.0, 3).is_err());
    assert!(make_sphere(&mut Topology::new(), 1.0, 4).is_ok());

    let mut topo = Topology::new();
    assert!(make_torus(&mut topo, 1.0, 1.0, 16).is_err());
    assert!(make_torus(&mut topo, 1.0, 1.0 - 2.0 * minimum, 16).is_ok());
    assert!(make_torus(&mut Topology::new(), 3.0, 1.0, 3).is_err());
    assert!(make_torus(&mut Topology::new(), 3.0, 1.0, 4).is_ok());
}
