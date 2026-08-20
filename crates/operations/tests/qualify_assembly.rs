//! Qualification evidence for the assembly family.
//!
//! Axes covered (see `docs/kernel-maturity/stabilization-plan.md`, item A3):
//! hierarchy depth, transform composition (rotation + translation, verified
//! against directly transformed geometry), instance sharing, sub-assembly
//! nodes with their own solids, BOM determinism, bounding boxes, and typed
//! errors for empty assemblies and invalid parents.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::assembly::Assembly;
use remus_operations::primitives::make_box;
use remus_topology::Topology;

/// A five-level chain of translations composes exactly: the world transform
/// equals the product of every ancestor transform, and flatten reports the
/// same matrix.
#[test]
fn deep_hierarchy_transform_composition() {
    let mut topo = Topology::new();
    let unit = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

    let mut asm = Assembly::new("chain");
    let mut parent = asm.add_root_component("l0", unit, Mat4::translation(1.0, 0.0, 0.0));
    let mut expected = Mat4::translation(1.0, 0.0, 0.0);
    for level in 1..5 {
        let step = Mat4::translation(0.0, f64::from(level), 0.0);
        parent = asm
            .add_child_component(parent, format!("l{level}"), unit, step)
            .unwrap();
        expected = expected * step;
    }

    let world = asm.world_transform(parent).unwrap();
    let origin = world.mul_point(Point3::new(0.0, 0.0, 0.0));
    let want = expected.mul_point(Point3::new(0.0, 0.0, 0.0));
    assert!((origin - want).length() < 1e-12);
    // y = 1 + 2 + 3 + 4 = 10, x = 1.
    assert!((origin.x() - 1.0).abs() < 1e-12 && (origin.y() - 10.0).abs() < 1e-12);

    // Flatten reports every component, parent before child, with the same
    // accumulated matrices.
    let flat = asm.flatten();
    assert_eq!(flat.len(), 5);
    let last = flat.last().unwrap();
    let flat_origin = last.1.mul_point(Point3::new(0.0, 0.0, 0.0));
    assert!((flat_origin - want).length() < 1e-12);
}

/// Rotation composed with translation places geometry where directly
/// transforming the solid places it: the assembly bounding box matches the
/// transformed body's box.
#[test]
fn rotation_translation_bbox_matches_direct_transform() {
    let mut topo = Topology::new();
    let slab = make_box(&mut topo, 4.0, 1.0, 1.0).unwrap();

    // Rotate the 4-long slab 90 deg about Z, then translate +X: it now spans
    // x in [10-1? ...] — compute via the same matrix applied to the corner
    // points instead of trusting a hand calculation.
    let m = Mat4::translation(10.0, 0.0, 0.0) * Mat4::rotation_z(std::f64::consts::FRAC_PI_2);

    let mut asm = Assembly::new("placed");
    asm.add_root_component("slab", slab, m);
    let bbox = asm.bounding_box(&topo).unwrap();

    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for corner in [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(4.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(4.0, 0.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(4.0, 1.0, 1.0),
    ] {
        let p = m.mul_point(corner);
        for (i, v) in [p.x(), p.y(), p.z()].into_iter().enumerate() {
            lo[i] = lo[i].min(v);
            hi[i] = hi[i].max(v);
        }
    }
    assert!((bbox.min.x() - lo[0]).abs() < 1e-9 && (bbox.max.x() - hi[0]).abs() < 1e-9);
    assert!((bbox.min.y() - lo[1]).abs() < 1e-9 && (bbox.max.y() - hi[1]).abs() < 1e-9);
    assert!((bbox.min.z() - lo[2]).abs() < 1e-9 && (bbox.max.z() - hi[2]).abs() < 1e-9);
}

/// A sub-assembly node's own solid is an instance: flatten and the BOM agree
/// on the count.
#[test]
fn parent_solid_is_an_instance() {
    let mut topo = Topology::new();
    let housing = make_box(&mut topo, 4.0, 4.0, 2.0).unwrap();
    let bolt = make_box(&mut topo, 0.2, 0.2, 1.0).unwrap();

    let mut asm = Assembly::new("housing+bolts");
    let h = asm.add_root_component("housing", housing, Mat4::identity());
    for i in 0..4 {
        asm.add_child_component(
            h,
            format!("bolt_{i}"),
            bolt,
            Mat4::translation(1.0 + f64::from(i) * 0.5, 1.0, 2.0),
        )
        .unwrap();
    }

    let flat = asm.flatten();
    assert_eq!(flat.len(), 5, "housing + 4 bolts are 5 instances");

    let bom = asm.bill_of_materials();
    assert_eq!(bom.len(), 2);
    let total: usize = bom.iter().map(|e| e.instance_count).sum();
    assert_eq!(total, 5, "the BOM counts the same 5 instances");
}

/// BOM output is deterministic: ordered by solid index, named by the lowest-
/// ID component, identical across rebuilds.
#[test]
fn bom_is_deterministic() {
    let build = |topo: &mut Topology| {
        let a = make_box(topo, 1.0, 1.0, 1.0).unwrap();
        let b = make_box(topo, 2.0, 1.0, 1.0).unwrap();
        let c = make_box(topo, 3.0, 1.0, 1.0).unwrap();
        let mut asm = Assembly::new("mix");
        for (i, &s) in [b, a, c, a, b, a].iter().enumerate() {
            asm.add_root_component(format!("c{i}"), s, Mat4::identity());
        }
        asm.bill_of_materials()
            .into_iter()
            .map(|e| (e.solid_index, e.name, e.instance_count))
            .collect::<Vec<_>>()
    };
    let mut t1 = Topology::new();
    let mut t2 = Topology::new();
    let bom1 = build(&mut t1);
    let bom2 = build(&mut t2);
    assert_eq!(bom1, bom2);
    // Ordered by solid arena index; counts: a=3, b=2, c=1.
    let counts: Vec<usize> = bom1.iter().map(|e| e.2).collect();
    assert_eq!(counts, vec![3, 2, 1]);
    assert!(bom1.windows(2).all(|w| w[0].0 < w[1].0));
}

/// Flatten order is deterministic and depth-first across rebuilds.
#[test]
fn flatten_order_is_deterministic() {
    let build = |topo: &mut Topology| {
        let s = make_box(topo, 1.0, 1.0, 1.0).unwrap();
        let mut asm = Assembly::new("tree");
        let r1 = asm.add_root_component("r1", s, Mat4::translation(1.0, 0.0, 0.0));
        let r2 = asm.add_root_component("r2", s, Mat4::translation(2.0, 0.0, 0.0));
        asm.add_child_component(r1, "r1c1", s, Mat4::translation(0.0, 1.0, 0.0))
            .unwrap();
        asm.add_child_component(r2, "r2c1", s, Mat4::translation(0.0, 2.0, 0.0))
            .unwrap();
        asm.add_child_component(r1, "r1c2", s, Mat4::translation(0.0, 3.0, 0.0))
            .unwrap();
        asm.flatten()
            .into_iter()
            .map(|(sid, m)| {
                let p = m.mul_point(Point3::new(0.0, 0.0, 0.0));
                (sid.index(), (p.x() * 10.0) as i64, (p.y() * 10.0) as i64)
            })
            .collect::<Vec<_>>()
    };
    let mut t1 = Topology::new();
    let mut t2 = Topology::new();
    let f1 = build(&mut t1);
    assert_eq!(f1, build(&mut t2));
    // Depth-first, roots in insertion order: r1, r1c1, r1c2, r2, r2c1.
    let ys: Vec<i64> = f1.iter().map(|e| e.2).collect();
    assert_eq!(ys, vec![0, 10, 30, 0, 20]);
}

/// Empty assemblies and invalid parents are typed errors.
#[test]
fn empty_and_invalid_are_typed() {
    let topo = Topology::new();
    let asm = Assembly::new("empty");
    assert!(asm.flatten().is_empty());
    assert!(matches!(
        asm.bounding_box(&topo),
        Err(OperationsError::InvalidInput { .. })
    ));

    let mut topo = Topology::new();
    let s = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let mut asm = Assembly::new("bad-parent");
    assert!(matches!(
        asm.add_child_component(42, "orphan", s, Mat4::identity()),
        Err(OperationsError::InvalidInput { .. })
    ));
}
