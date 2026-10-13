//! OpSeq F5/F12: synthesized conic caps retain material and oppositely
//! directed shared boundaries, including major arcs and thin crescents.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::prepared::PreparedSolid;
use remus_check::classify::{ClassifyOptions, PointClassification};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_surface_area, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const RADIUS: f64 = 1.5;
const HEIGHT: f64 = 6.5;

#[derive(Clone, Copy, Debug)]
enum Region {
    CylinderCut,
    BoxCut,
    Intersection,
    Union,
}

impl Region {
    fn contains(self, cylinder: bool, rectangle: bool) -> bool {
        match self {
            Self::CylinderCut => cylinder && !rectangle,
            Self::BoxCut => rectangle && !cylinder,
            Self::Intersection => cylinder && rectangle,
            Self::Union => cylinder || rectangle,
        }
    }

    /// All fixtures are prisms over independent Cartesian cross sections.
    /// For 0 < b < r and a rectangle [0,r] x [0,b],
    /// A_overlap = integral_0^b sqrt(r^2-y^2) dy
    ///           = (b sqrt(r^2-b^2) + r^2 asin(b/r))/2.
    /// At b=1 this is sqrt(5)/4 + 9 asin(2/3)/8.
    /// Boundary lengths follow from the circle arc r asin(b/r) and the
    /// straight segments; volume=A*H and exterior area=2*A+perimeter*H.
    fn quantities(self, b: f64) -> (f64, f64) {
        let x = RADIUS.mul_add(RADIUS, -b * b).sqrt();
        let theta = (b / RADIUS).asin();
        let overlap = 0.5 * b.mul_add(x, RADIUS * RADIUS * theta);
        let disc = PI * RADIUS * RADIUS;
        let rectangle = RADIUS * b;
        let arc = RADIUS * theta;
        let (area, perimeter) = match self {
            Self::CylinderCut => (disc - overlap, 2.0 * PI * RADIUS - arc + b + x + RADIUS),
            Self::BoxCut => (rectangle - overlap, b + RADIUS - x + arc),
            Self::Intersection => (overlap, arc + b + x + RADIUS),
            Self::Union => (
                disc + rectangle - overlap,
                2.0 * PI * RADIUS - arc + b + RADIUS - x,
            ),
        };
        (area * HEIGHT, 2.0 * area + perimeter * HEIGHT)
    }
}

fn classification(inside: bool) -> PointClassification {
    if inside {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    }
}

fn assert_closed_boundary(topo: &Topology, solid: SolidId, label: &str) {
    let mut uses = BTreeMap::new();
    let faces = remus_topology::explorer::solid_faces(topo, solid).unwrap();
    let mut cylinders = 0;
    for fid in faces {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            FaceSurface::Cylinder(_) => cylinders += 1,
            FaceSurface::Plane { .. } => {}
            unsupported @ (FaceSurface::Nurbs(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_)) => {
                panic!("{label}: unexpected carrier {}", unsupported.type_tag());
            }
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                uses.entry(edge.edge())
                    .or_insert_with(Vec::new)
                    .push(edge.is_forward() != face.is_reversed());
            }
        }
    }
    assert!(cylinders > 0, "{label}: cylinder carrier lost");
    for (edge, directions) in uses {
        assert_eq!(
            directions.len(),
            2,
            "{label}: open/non-manifold edge {edge:?}"
        );
        assert_ne!(
            directions[0], directions[1],
            "{label}: same-sense shared edge {edge:?}"
        );
    }
    assert!(
        remus_operations::validate::validate_solid(topo, solid)
            .unwrap()
            .is_valid(),
        "{label}: operations validator"
    );
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        !report
            .issues
            .iter()
            .any(|issue| issue.severity == remus_check::validate::Severity::Error),
        "{label}: {:?}",
        report.issues
    );
}

#[allow(clippy::too_many_lines)]
fn check_fixture(region: Region, b: f64, scale: f64, placed: bool) {
    let label = format!("F5 {region:?} b={b} scale={scale} placed={placed}");
    let mut topo = Topology::new();
    // Keep the literal repro's creation order: box 0, cylinder 1.
    let rectangle = make_box(&mut topo, RADIUS * scale, b * scale, HEIGHT * scale).unwrap();
    let cylinder = make_cylinder(&mut topo, RADIUS * scale, HEIGHT * scale).unwrap();
    let placement = if placed {
        Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    transform_solid(&mut topo, rectangle, &placement).unwrap();
    transform_solid(&mut topo, cylinder, &placement).unwrap();
    let before = remus_io::arena_io::serialize_solids(&topo, &[rectangle, cylinder]).unwrap();
    let (op, a, b_solid) = match region {
        Region::CylinderCut => (BooleanOp::Cut, cylinder, rectangle),
        Region::BoxCut => (BooleanOp::Cut, rectangle, cylinder),
        Region::Intersection => (BooleanOp::Intersect, cylinder, rectangle),
        Region::Union => (BooleanOp::Fuse, rectangle, cylinder),
    };
    let outcome = boolean_with_context(
        &mut topo,
        op,
        a,
        b_solid,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    );
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[rectangle, cylinder]).unwrap(),
        before,
        "{label}: operands changed, including on refusal"
    );
    let outcome = outcome.unwrap_or_else(|error| panic!("{label}: {error:?}"));
    assert!(matches!(outcome.quality, BooleanQuality::Exact), "{label}");
    let solid = outcome.solid;
    assert_closed_boundary(&topo, solid, &label);

    let options = ClassifyOptions {
        tolerance: 1e-7,
        ..ClassifyOptions::default()
    };
    let source_cylinder = PreparedSolid::prepare(&topo, cylinder).unwrap();
    let source_rectangle = PreparedSolid::prepare(&topo, rectangle).unwrap();
    let result = PreparedSolid::prepare(&topo, solid).unwrap();
    let mut membership_classes = [0; 4];
    for x in [-1.65_f64, -1.25, -0.5, 0.25, 0.75, 1.25, 1.4, 1.65] {
        for y in [-1.25_f64, -0.5, 0.25, 0.6, 0.9, 1.1, 1.4] {
            // Keep probes away from every analytic boundary independently
            // of kernel tolerance. The box-only wedge is sampled as well.
            if (x * x + y * y - RADIUS * RADIUS).abs() < 0.02
                || x.abs() < 0.02
                || (x - RADIUS).abs() < 0.02
                || y.abs() < 0.02
                || (y - b).abs() < 0.02
            {
                continue;
            }
            for z in [-0.5_f64, 0.5, 3.25, 6.0, 7.0] {
                let axial = z > 0.0 && z < HEIGHT;
                let in_cylinder = axial && x * x + y * y < RADIUS * RADIUS;
                let in_rectangle = axial && x > 0.0 && x < RADIUS && y > 0.0 && y < b;
                membership_classes[usize::from(in_cylinder) * 2 + usize::from(in_rectangle)] += 1;
                let point = placement.mul_point(Point3::new(x * scale, y * scale, z * scale));
                let where_at = format!("{label}: local=({x}, {y}, {z})");
                assert_eq!(
                    source_cylinder.classify_point(point, &options).unwrap(),
                    classification(in_cylinder),
                    "{where_at}: source cylinder"
                );
                assert_eq!(
                    source_rectangle.classify_point(point, &options).unwrap(),
                    classification(in_rectangle),
                    "{where_at}: source box"
                );
                assert_eq!(
                    result.classify_point(point, &options).unwrap(),
                    classification(region.contains(in_cylinder, in_rectangle)),
                    "{where_at}: result material"
                );
            }
        }
    }
    assert!(
        membership_classes.iter().all(|&count| count > 0),
        "{label}: grid coverage"
    );
    let (expected_volume, expected_area) = region.quantities(b);
    for deflection in [0.0001, 0.00001] {
        let volume = solid_volume(&topo, solid, deflection * scale).unwrap();
        assert!(
            (volume - expected_volume * scale.powi(3)).abs() < 1e-4 * scale.powi(3),
            "{label}: measured volume {volume} vs {expected_volume} * scale^3"
        );
    }
    let gauss = mass_properties(&topo, solid).unwrap().mass;
    assert!(
        (gauss - expected_volume * scale.powi(3)).abs() < 1e-4 * scale.powi(3),
        "{label}: Gauss volume {gauss} vs {expected_volume} * scale^3"
    );
    let area = solid_surface_area(&topo, solid, 0.0001 * scale).unwrap();
    assert!(
        (area - expected_area * scale.powi(2)).abs() < 1e-4 * scale.powi(2),
        "{label}: area {area} vs {expected_area} * scale^2"
    );
    for deflection in [0.1, 0.01] {
        let mesh = tessellate_solid(&topo, solid, deflection * scale).unwrap();
        assert!(!mesh.indices.is_empty(), "{label}: empty mesh");
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{label}: raw boundary at {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{label}: raw non-manifold mesh"
        );
    }
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[rectangle, cylinder]).unwrap(),
        before,
        "{label}: operands changed after read-only checks"
    );
}

#[test]
fn f5_cut_unit_origin() {
    check_fixture(Region::CylinderCut, 1.0, 1.0, false);
}

#[test]
fn f5_cut_unit_placed() {
    check_fixture(Region::CylinderCut, 1.0, 1.0, true);
}

#[test]
fn f5_cut_large_origin() {
    check_fixture(Region::CylinderCut, 1.0, 1e3, false);
}

#[test]
fn f5_cut_large_placed() {
    check_fixture(Region::CylinderCut, 1.0, 1e3, true);
}

#[test]
fn f5_related_operations_and_operand_order() {
    for region in [Region::BoxCut, Region::Intersection, Region::Union] {
        check_fixture(region, 1.0, 1.0, false);
    }
}

#[test]
fn f5_nearby_notch_heights() {
    for b in [0.75, 1.25] {
        check_fixture(Region::CylinderCut, b, 1.0, false);
    }
}

/// The r6 stock disc minus an r8 disc whose centre is sqrt(5) away.
/// With gamma=acos(23/(12 sqrt(5))) and beta=acos(33/(16 sqrt(5))),
/// the crescent has area 36 gamma - 64 beta + sqrt(191)/2 and boundary
/// length 12 gamma + 16 beta. The solid is a height-2 prism, so V=2A
/// and exterior area=2A+2P. This uses circle-sector geometry, independent
/// of the kernel's face integrators and tessellations.
fn translated_cylinder_quantities() -> (f64, f64) {
    let gamma = (23.0 / (12.0 * 5.0_f64.sqrt())).acos();
    let beta = (33.0 / (16.0 * 5.0_f64.sqrt())).acos();
    let area = 36.0 * gamma - 64.0 * beta + 191.0_f64.sqrt() / 2.0;
    let perimeter = 12.0 * gamma + 16.0 * beta;
    (2.0 * area, 2.0 * area + 2.0 * perimeter)
}

#[allow(clippy::too_many_lines)]
fn check_translated_cylinders(scale: f64, placed: bool) {
    let label = format!("F12 translated cylinder cut scale={scale} placed={placed}");
    let mut topo = Topology::new();
    // Literal bundle order: tool 0, stock 1, translate tool, stock minus tool.
    let tool = make_cylinder(&mut topo, 8.0 * scale, 5.0 * scale).unwrap();
    let stock = make_cylinder(&mut topo, 6.0 * scale, 2.0 * scale).unwrap();
    let placement = if placed {
        Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    transform_solid(
        &mut topo,
        tool,
        &(placement * Mat4::translation(scale, 2.0 * scale, 0.0)),
    )
    .unwrap();
    transform_solid(&mut topo, stock, &placement).unwrap();
    let before = remus_io::arena_io::serialize_solids(&topo, &[tool, stock]).unwrap();
    let outcome = boolean_with_context(
        &mut topo,
        BooleanOp::Cut,
        stock,
        tool,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    );
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[tool, stock]).unwrap(),
        before,
        "{label}: operands changed, including on refusal"
    );
    let outcome = outcome.unwrap_or_else(|error| panic!("{label}: {error:?}"));
    assert!(matches!(outcome.quality, BooleanQuality::Exact), "{label}");
    let solid = outcome.solid;
    assert_closed_boundary(&topo, solid, &label);
    let cylinders = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .count();
    assert!(cylinders >= 2, "{label}: both cylinder walls must survive");

    let options = ClassifyOptions {
        tolerance: 1e-7,
        ..ClassifyOptions::default()
    };
    let source_stock = PreparedSolid::prepare(&topo, stock).unwrap();
    let source_tool = PreparedSolid::prepare(&topo, tool).unwrap();
    let result = PreparedSolid::prepare(&topo, solid).unwrap();
    // u points toward the translated tool centre; v is perpendicular. This
    // independent Cartesian basis lets a rectangular grid sample the thin
    // crescent as well as stock-only, tool-only, overlap and neither.
    let d = 5.0_f64.sqrt();
    let mut probes = Vec::new();
    for u in [-6.25, -5.95, -5.85, -5.7, -5.4, -5.0, 0.0, 5.0, 7.0, 11.0] {
        for v in [-3.5, -2.5, -1.5, -0.5, 0.0, 0.5, 1.5, 2.5, 3.5] {
            for z in [-0.5, 0.25, 1.0, 1.75, 2.25, 4.5, 5.5] {
                probes.push(((u - 2.0 * v) / d, (2.0 * u + v) / d, z, false));
            }
        }
    }
    // At each v the kept interval is between the two left circle roots.
    // Fractions across that interval resolve the crescent's thickness rather
    // than relying on a volume reading or probes at the centre of the stock.
    for v in [-2.75_f64, -2.0, -1.0, 0.0, 1.0, 2.0, 2.75] {
        let stock_left = -(36.0 - v * v).sqrt();
        let tool_left = d - (64.0 - v * v).sqrt();
        assert!(
            tool_left > stock_left,
            "{label}: analytic crescent interval"
        );
        for fraction in [0.25, 0.5, 0.75] {
            let u = stock_left + fraction * (tool_left - stock_left);
            for z in [0.25, 1.0, 1.75] {
                probes.push(((u - 2.0 * v) / d, (2.0 * u + v) / d, z, true));
            }
        }
    }
    let mut classes = [0; 4];
    let mut crescent_probes = 0;
    for (x, y, z, thin_crescent) in probes {
        let stock_radius_squared = x * x + y * y;
        let tool_radius_squared = (x - 1.0).powi(2) + (y - 2.0).powi(2);
        if !thin_crescent
            && ((stock_radius_squared - 36.0).abs() < 0.04
                || (tool_radius_squared - 64.0).abs() < 0.04)
        {
            continue;
        }
        let in_stock = stock_radius_squared < 36.0 && z > 0.0 && z < 2.0;
        let in_tool = tool_radius_squared < 64.0 && z > 0.0 && z < 5.0;
        if thin_crescent {
            assert!(
                in_stock && !in_tool,
                "{label}: independent thin-crescent guard"
            );
            crescent_probes += 1;
        }
        classes[usize::from(in_stock) * 2 + usize::from(in_tool)] += 1;
        let point = placement.mul_point(Point3::new(x * scale, y * scale, z * scale));
        let at = format!("{label}: local=({x}, {y}, {z})");
        assert_eq!(
            source_stock.classify_point(point, &options).unwrap(),
            classification(in_stock),
            "{at}: stock self-membership"
        );
        assert_eq!(
            source_tool.classify_point(point, &options).unwrap(),
            classification(in_tool),
            "{at}: tool self-membership"
        );
        assert_eq!(
            result.classify_point(point, &options).unwrap(),
            classification(in_stock && !in_tool),
            "{at}: result material"
        );
    }
    assert_eq!(crescent_probes, 63, "{label}: thin-crescent coverage");
    assert!(
        classes.iter().all(|&count| count > 0),
        "{label}: grid coverage"
    );

    let (expected_volume, expected_area) = translated_cylinder_quantities();
    for volume in [
        solid_volume(&topo, solid, 0.0001 * scale).unwrap(),
        solid_volume(&topo, solid, 0.00001 * scale).unwrap(),
        mass_properties(&topo, solid).unwrap().mass,
    ] {
        assert!(
            (volume - expected_volume * scale.powi(3)).abs() < 1e-4 * scale.powi(3),
            "{label}: volume {volume} vs {expected_volume} * scale^3"
        );
    }
    let area = solid_surface_area(&topo, solid, 0.0001 * scale).unwrap();
    assert!(
        (area - expected_area * scale.powi(2)).abs() < 1e-4 * scale.powi(2),
        "{label}: area {area} vs {expected_area} * scale^2"
    );
    for deflection in [0.1, 0.01] {
        let mesh = tessellate_solid(&topo, solid, deflection * scale).unwrap();
        assert!(!mesh.indices.is_empty(), "{label}: empty mesh");
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{label}: raw boundary at {deflection}"
        );
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{label}: raw non-manifold mesh"
        );
    }
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[tool, stock]).unwrap(),
        before,
        "{label}: operands changed after read-only checks"
    );
}

#[test]
fn f12_cut_unit_origin() {
    check_translated_cylinders(1.0, false);
}

#[test]
fn f12_cut_unit_placed() {
    check_translated_cylinders(1.0, true);
}

#[test]
fn f12_cut_large_origin() {
    check_translated_cylinders(1e3, false);
}

#[test]
fn f12_cut_large_placed() {
    check_translated_cylinders(1e3, true);
}
