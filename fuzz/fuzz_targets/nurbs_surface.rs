//! Structured fuzzing for NURBS surface construction, evaluation and SSI.
//!
//! Every accepted input builds a bounded rational height-field patch and a
//! horizontal NURBS plane. Half of the byte space deliberately corrupts one
//! constructor invariant and requires a typed rejection. Valid cases exercise
//! evaluation, derivatives and surface-surface intersection. Returned section
//! points are checked against the independently known plane equation and by
//! re-evaluation on both parameterized surfaces.

#![no_main]

use libfuzzer_sys::fuzz_target;
use remus_math::context::{OperationContext, WorkBudgets};
use remus_math::nurbs::intersection::intersect_nurbs_nurbs_with_context;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};

const MAX_ROWS: usize = 4;
const MAX_COLS: usize = 4;

struct Bytes<'a> {
    data: &'a [u8],
    index: usize,
}

impl<'a> Bytes<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, index: 0 }
    }

    fn next(&mut self) -> u8 {
        if self.data.is_empty() {
            return 0;
        }
        let value = self.data[self.index % self.data.len()];
        self.index += 1;
        value
    }

    fn unit(&mut self) -> f64 {
        f64::from(self.next()) / f64::from(u8::MAX)
    }

    fn signed(&mut self) -> f64 {
        self.unit().mul_add(2.0, -1.0)
    }
}

struct SurfaceSpec {
    degree_u: usize,
    degree_v: usize,
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    control_points: Vec<Vec<Point3>>,
    weights: Vec<Vec<f64>>,
}

fn open_uniform_knots(count: usize, degree: usize) -> Vec<f64> {
    let mut knots = vec![0.0; count + degree + 1];
    let interior = count - degree - 1;
    for (index, knot) in knots.iter_mut().enumerate().take(count).skip(degree + 1) {
        *knot = (index - degree) as f64 / (interior + 1) as f64;
    }
    for knot in knots.iter_mut().skip(count) {
        *knot = 1.0;
    }
    knots
}

fn generated_spec(bytes: &mut Bytes<'_>, scale: f64) -> SurfaceSpec {
    let rows = 2 + usize::from(bytes.next()) % (MAX_ROWS - 1);
    let cols = 2 + usize::from(bytes.next()) % (MAX_COLS - 1);
    let degree_u = 1 + usize::from(bytes.next()) % (rows - 1);
    let degree_v = 1 + usize::from(bytes.next()) % (cols - 1);

    let mut control_points = Vec::with_capacity(rows);
    let mut weights = Vec::with_capacity(rows);
    for row in 0..rows {
        let mut point_row = Vec::with_capacity(cols);
        let mut weight_row = Vec::with_capacity(cols);
        for col in 0..cols {
            let u = row as f64 / (rows - 1) as f64;
            let v = col as f64 / (cols - 1) as f64;
            point_row.push(Point3::new(u * scale, v * scale, bytes.signed() * scale));
            weight_row.push(0.25 + bytes.unit() * 3.75);
        }
        control_points.push(point_row);
        weights.push(weight_row);
    }

    // The plane height below always lies between these two points, so every
    // valid surface has a non-vacuous section by continuity.
    control_points[0][0] = Point3::new(0.0, 0.0, -scale);
    control_points[rows - 1][cols - 1] = Point3::new(scale, scale, scale);

    SurfaceSpec {
        degree_u,
        degree_v,
        knots_u: open_uniform_knots(rows, degree_u),
        knots_v: open_uniform_knots(cols, degree_v),
        control_points,
        weights,
    }
}

fn corrupt(spec: &mut SurfaceSpec, mode: u8) {
    match mode % 7 {
        0 => spec.weights[0][0] = 0.0,
        1 => spec.weights[0][0] = f64::NAN,
        2 => {
            spec.knots_u[spec.degree_u] = 0.75;
            spec.knots_u[spec.degree_u + 1] = 0.25;
        }
        3 => {
            spec.knots_v.pop();
        }
        4 => spec.control_points[0][0] = Point3::new(f64::INFINITY, 0.0, 0.0),
        5 => spec.degree_u = spec.control_points.len(),
        _ => {
            spec.weights.pop();
        }
    }
}

fn construct(spec: SurfaceSpec) -> Result<NurbsSurface, remus_math::MathError> {
    NurbsSurface::new(
        spec.degree_u,
        spec.degree_v,
        spec.knots_u,
        spec.knots_v,
        spec.control_points,
        spec.weights,
    )
}

fn plane(scale: f64, height: f64) -> NurbsSurface {
    let result = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![
                Point3::new(0.0, 0.0, height),
                Point3::new(0.0, scale, height),
            ],
            vec![
                Point3::new(scale, 0.0, height),
                Point3::new(scale, scale, height),
            ],
        ],
        vec![vec![1.0, 1.0], vec![1.0, 1.0]],
    );
    assert!(
        result.is_ok(),
        "the fixed plane fixture must be constructible"
    );
    result.unwrap_or_else(|error| unreachable!("validated plane failed: {error}"))
}

fn point_is_finite(point: Point3) -> bool {
    point.x().is_finite() && point.y().is_finite() && point.z().is_finite()
}

fn vector_is_finite(vector: Vec3) -> bool {
    vector.x().is_finite() && vector.y().is_finite() && vector.z().is_finite()
}

fn assert_evaluation_is_finite(surface: &NurbsSurface, u: f64, v: f64) {
    let point = surface.evaluate(u, v);
    assert!(
        point_is_finite(point),
        "surface evaluation returned {point:?}"
    );
    for derivative in surface.derivatives(u, v, 2).into_iter().flatten() {
        assert!(
            vector_is_finite(derivative),
            "surface derivative returned {derivative:?}"
        );
    }
}

fn assert_in_domain(value: f64, domain: (f64, f64), name: &str) {
    let slack = f64::EPSILON * 32.0;
    assert!(
        value.is_finite() && value >= domain.0 - slack && value <= domain.1 + slack,
        "{name}={value} lies outside [{}, {}]",
        domain.0,
        domain.1
    );
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 2 {
        return;
    }

    let valid = data[0] & 1 == 0;
    let mut bytes = Bytes::new(&data[2..]);
    let scale = [0.1, 1.0, 10.0][usize::from(bytes.next()) % 3];
    let mut spec = generated_spec(&mut bytes, scale);

    if !valid {
        corrupt(&mut spec, data[1]);
        assert!(
            construct(spec).is_err(),
            "NURBS constructor accepted a deliberately invalid surface"
        );
        return;
    }

    let surface = construct(spec)
        .unwrap_or_else(|error| panic!("generated valid NURBS surface was rejected: {error}"));
    for (u, v) in [
        (0.0, 0.0),
        (0.5, 0.5),
        (1.0, 1.0),
        (bytes.unit(), bytes.unit()),
    ] {
        assert_evaluation_is_finite(&surface, u, v);
    }

    let plane_height = bytes.signed() * scale * 0.5;
    let section_plane = plane(scale, plane_height);
    let budgets = WorkBudgets::new()
        .with_march_steps(48)
        .with_queue_size(16)
        .with_segments(8)
        .with_branches_per_direction(4)
        .with_newton_iterations(12)
        .with_subdivision_depth(6);
    let context = OperationContext::new().with_budgets(budgets);
    let samples = 5 + usize::from(bytes.next()) % 4;
    let march_step = if bytes.next() & 1 == 0 {
        0.0
    } else {
        0.02 + bytes.unit() * 0.06
    };

    let curves =
        intersect_nurbs_nurbs_with_context(&surface, &section_plane, samples, march_step, &context)
            .unwrap_or_else(|error| panic!("bounded valid NURBS SSI failed: {error}"));
    assert!(
        !curves.is_empty(),
        "bounded NURBS SSI missed a section guaranteed by opposite surface corners"
    );

    let oracle_tolerance = scale.mul_add(1.0e-3, 1.0e-6);
    for intersection in curves {
        for sample in &intersection.points {
            assert!(
                point_is_finite(sample.point),
                "SSI returned a non-finite point: {:?}",
                sample.point
            );
            assert_in_domain(sample.param1.0, surface.domain_u(), "u1");
            assert_in_domain(sample.param1.1, surface.domain_v(), "v1");
            assert_in_domain(sample.param2.0, section_plane.domain_u(), "u2");
            assert_in_domain(sample.param2.1, section_plane.domain_v(), "v2");

            let on_surface = surface.evaluate(sample.param1.0, sample.param1.1);
            let on_plane = section_plane.evaluate(sample.param2.0, sample.param2.1);
            assert!(
                (on_surface - sample.point).length() <= oracle_tolerance,
                "SSI point missed the generated surface"
            );
            assert!(
                (on_plane - sample.point).length() <= oracle_tolerance,
                "SSI point missed the plane surface"
            );
            assert!(
                (sample.point.z() - plane_height).abs() <= oracle_tolerance,
                "SSI point violated the independent plane equation"
            );
        }

        for control_point in intersection.curve.control_points() {
            assert!(
                (control_point.z() - plane_height).abs() <= oracle_tolerance * 20.0,
                "fitted SSI control point violated the independent plane equation: \
                 control_point={control_point:?}, plane_height={plane_height}, \
                 tolerance={oracle_tolerance}"
            );
        }

        let (start, end) = intersection.curve.domain();
        for index in 0..=4 {
            let t = start + (end - start) * f64::from(index) / 4.0;
            let point = intersection.curve.evaluate(t);
            assert!(point_is_finite(point), "SSI curve returned {point:?}");
            assert!(
                (point.z() - plane_height).abs() <= oracle_tolerance * 20.0,
                "fitted SSI curve violated the independent plane equation: \
                 point={point:?}, plane_height={plane_height}, tolerance={oracle_tolerance}"
            );
        }
    }
});
