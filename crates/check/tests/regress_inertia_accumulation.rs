//! Parallel-axis shifts retain the transverse inertia of slender assemblies.

use remus_check::properties::GProps;
use remus_math::vec::Point3;

#[test]
fn slender_assembly_keeps_transverse_parallel_axis_terms() {
    for long_axis in 0..3 {
        let mut offset = [1.0; 3];
        offset[long_axis] = 1e8;
        let mut a = GProps {
            mass: 1.0,
            center: Point3::new(offset[0], offset[1], offset[2]),
            inertia: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        };
        let b = GProps {
            mass: 1.0,
            center: Point3::new(-offset[0], -offset[1], -offset[2]),
            inertia: [1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
        };
        a.add(&b);
        assert!((a.mass - 2.0).abs() < 1e-15);
        assert!(a.center.0.iter().all(|coordinate| coordinate.abs() < 1e-15));
        // About the long axis, each body's contribution is its own unit
        // inertia plus the squared distances along the other TWO axes.
        assert!((a.inertia[long_axis] - 6.0).abs() < 1e-12, "{a:?}");
    }
}
