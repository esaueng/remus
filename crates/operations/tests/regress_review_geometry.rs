//! Independent regressions for trim-aware measurement, oriented offsets, and NURBS bounds.
#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]
    use remus_math::{
        curves::Circle3D,
        nurbs::{NurbsCurve, NurbsSurface},
        vec::{Point3, Vec3},
    };
    use remus_operations::{measure::edge_length, offset_face::offset_face_with_quality};
    use remus_topology::{
        Topology,
        edge::{Edge, EdgeCurve},
        face::FaceSurface,
        vertex::Vertex,
    };

    #[test]
    fn trimmed_nurbs_length() {
        let mut t = Topology::new();
        let c = NurbsCurve::new(
            1,
            vec![0., 0., 1., 1.],
            vec![Point3::new(0., 0., 0.), Point3::new(10., 0., 0.)],
            vec![1., 1.],
        )
        .unwrap();
        for trim in [(0.2, 0.4), (0.4, 0.2)] {
            let a = t.add_vertex(Vertex::new(c.evaluate(trim.0), 1e-7));
            let b = t.add_vertex(Vertex::new(c.evaluate(trim.1), 1e-7));
            let mut e = Edge::new(a, b, EdgeCurve::NurbsCurve(c.clone()));
            e.set_trim(Some(trim));
            e.strict_domain().unwrap();
            let id = t.add_edge(e);
            let got = edge_length(&t, id).unwrap();
            assert!(
                (got - 2.).abs() < 1e-9,
                "trim {trim:?} on length-10 carrier: expected 2, got {got}"
            );
        }
    }
    #[test]
    fn clockwise_circle_length() {
        let mut t = Topology::new();
        let c = Circle3D::new(Point3::new(0., 0., 0.), Vec3::new(0., 0., 1.), 2.).unwrap();
        let a = t.add_vertex(Vertex::new(c.evaluate(std::f64::consts::FRAC_PI_2), 1e-7));
        let b = t.add_vertex(Vertex::new(c.evaluate(0.), 1e-7));
        let mut e = Edge::new(a, b, EdgeCurve::Circle(c));
        e.set_trim(Some((std::f64::consts::FRAC_PI_2, 0.)));
        e.strict_domain().unwrap();
        let id = t.add_edge(e);
        let got = edge_length(&t, id).unwrap();
        assert!(
            (got - std::f64::consts::PI).abs() < 1e-9,
            "clockwise quarter circle r=2: expected pi, got {got}"
        );
    }
    #[test]
    fn reversed_face_offset() {
        let mut t = Topology::new();
        let f = remus_topology::test_utils::make_unit_square_face(&mut t);
        t.face_mut(f).unwrap().set_reversed(true);
        let out = offset_face_with_quality(&mut t, f, 1., None).unwrap();
        let got = t.face(out.face).unwrap();
        let FaceSurface::Plane { d, .. } = got.surface() else {
            panic!()
        };
        assert!(
            (*d + 1.).abs() < 1e-9 && got.is_reversed(),
            "reversed +Z plane outward offset expected d=-1 and reversed=true; got d={d}, reversed={}",
            got.is_reversed()
        );
    }
    #[test]
    fn nonunit_nurbs_offset_domain() {
        let mut t = Topology::new();
        let f = remus_topology::test_utils::make_unit_square_face(&mut t);
        let k = vec![2., 2., 4., 4.];
        let s = NurbsSurface::new(
            1,
            1,
            k.clone(),
            k,
            vec![
                vec![Point3::new(0., 0., 0.), Point3::new(0., 1., 0.)],
                vec![Point3::new(1., 0., 0.), Point3::new(1., 1., 0.)],
            ],
            vec![vec![1.; 2]; 2],
        )
        .unwrap();
        t.face_mut(f).unwrap().set_surface(FaceSurface::Nurbs(s));
        let out = offset_face_with_quality(&mut t, f, 0.5, Some(4));
        assert!(
            out.is_ok(),
            "valid plane NURBS on [2,4]^2 must offset: {out:?}"
        );
        let FaceSurface::Nurbs(s) = t.face(out.unwrap().face).unwrap().surface() else {
            panic!()
        };
        let p = s.evaluate(0.5, 0.5);
        assert!(
            (p.x() - 0.5).abs() < 1e-6 && (p.y() - 0.5).abs() < 1e-6 && (p.z() - 0.5).abs() < 1e-6,
            "offset plane center expected (0.5,0.5,0.5), got {p:?}"
        );
    }
    fn bumped_box(t: &mut Topology) -> remus_topology::solid::SolidId {
        let solid = remus_operations::primitives::make_box(t, 1., 1., 1.).unwrap();
        let faces = remus_topology::explorer::solid_faces(t, solid).unwrap();
        let top = faces
            .into_iter()
            .find(|&face| {
                matches!(t.face(face).unwrap().surface(), FaceSurface::Plane { normal, d }
                if normal.z() > 0.9 && *d > 0.9)
            })
            .unwrap();
        let cp: Vec<Vec<Point3>> = (0..4)
            .map(|i| {
                (0..3)
                    .map(|j| {
                        Point3::new(
                            f64::from(i) / 3.,
                            f64::from(j) / 2.,
                            if i == 1 && j == 1 { 101. } else { 1. },
                        )
                    })
                    .collect()
            })
            .collect();
        let surf = NurbsSurface::new(
            3,
            2,
            vec![0., 0., 0., 0., 1., 1., 1., 1.],
            vec![0., 0., 0., 1., 1., 1.],
            cp,
            vec![vec![1.; 3]; 4],
        )
        .unwrap();
        t.face_mut(top)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(surf));
        solid
    }
    #[test]
    fn nurbs_box_conservativeness() {
        let mut t = Topology::new();
        let solid = bumped_box(&mut t);
        let report = remus_check::validate::validate_solid(
            &t,
            solid,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert!(report.is_valid(), "fixture validation failed: {report:?}");
        let bb = remus_operations::measure::solid_bounding_box(&t, solid).unwrap();
        let max = 1. + 100. * (2. / 9.);
        assert!(
            bb.max.z() >= max - 1e-9,
            "valid NURBS solid top reaches z={max}, returned bbox max z={}",
            bb.max.z()
        );
    }
    #[test]
    fn compound_intersection_false_empty() {
        use remus_operations::boolean::{BooleanOp, boolean_compound_regions};
        use remus_topology::compound::Compound;
        let mut t = Topology::new();
        let a = bumped_box(&mut t);
        let b = remus_operations::primitives::make_box(&mut t, 0.02, 0.02, 0.05).unwrap();
        remus_operations::transform::transform_solid(
            &mut t,
            b,
            &remus_math::mat::Mat4::translation(0.32, 0.49, 22.5),
        )
        .unwrap();
        let ca = t.add_compound(Compound::new(vec![a]));
        let cb = t.add_compound(Compound::new(vec![b]));
        let got = boolean_compound_regions(&mut t, BooleanOp::Intersect, ca, cb);
        assert!(
            !matches!(
                got,
                Err(remus_operations::OperationsError::EmptyResult { .. })
            ),
            "box B is within NURBS bulge but intersection reports {got:?}"
        );
    }
}
