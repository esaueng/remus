//! opseq81 seed 1146645986253340484: a torus piercing a sphere, fused
//! exact-only, leaves two disjoint NURBS section holes in one sphere
//! hemisphere. The holed-hemisphere mesher accepted only one hole, so the
//! face fell back to a path that skinned both holes and resampled the
//! equator: 1320 open mesh edges at the campaign deflection (CI run
//! 41959012115, artifact `opseq81-campaign-findings`).
//!
//! The fused B-Rep was already valid. This file pins it independently
//! (validation, section edges on both carriers, Cartesian material probes,
//! an inclusion-exclusion volume integral) before checking raw indexed and
//! geometrically welded mesh closure, manifoldness and orientation at two
//! deflections, in both operand orders, under rigid placement and scale.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_bounding_box};
use remus_operations::mirror::mirror;
use remus_operations::primitives::{make_sphere, make_torus};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count,
    tessellate_solid_grouped_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

const MAJOR: f64 = 4.0;
const MINOR: f64 = 2.0;
const BALL: f64 = 8.0;
/// Torus centre in the seed's frame: mirrored about x = -4, then moved by
/// (1, 0, -3).
const TORUS_X: f64 = -7.0;
const TORUS_Z: f64 = -3.0;

/// Area of the intersection of two discs of radii `a`, `b` whose centres
/// are `d` apart.
fn lens(a: f64, b: f64, d: f64) -> f64 {
    if d >= a + b {
        return 0.0;
    }
    if d <= (a - b).abs() {
        return PI * a.min(b).powi(2);
    }
    let ca = (d.mul_add(d, a * a) - b * b) / (2.0 * d * a);
    let cb = (d.mul_add(d, b * b) - a * a) / (2.0 * d * b);
    let k = (-d + a + b) * (d + a - b) * (d - a + b) * (d + a + b);
    a * a * ca.clamp(-1.0, 1.0).acos() + b * b * cb.clamp(-1.0, 1.0).acos() - 0.5 * k.sqrt()
}

/// Volume of torus ∩ ball, independent of the kernel: each horizontal slice
/// is an annulus (centred `|TORUS_X|` from the ball axis) meeting a disc,
/// so its area is a difference of two closed-form lenses. The substitution
/// `z = TORUS_Z - MINOR cos t` removes the tube's endpoint square roots.
fn torus_ball_overlap() -> f64 {
    let n = 20_000;
    let mut total = 0.0;
    for i in 0..n {
        let t = (f64::from(i) + 0.5) * PI / f64::from(n);
        let z = MINOR.mul_add(-t.cos(), TORUS_Z);
        let h = (MINOR * MINOR - (z - TORUS_Z).powi(2)).max(0.0).sqrt();
        let s = (BALL * BALL - z * z).max(0.0).sqrt();
        let area = lens(MAJOR + h, s, TORUS_X.abs()) - lens(MAJOR - h, s, TORUS_X.abs());
        total += area * MINOR * t.sin() * PI / f64::from(n);
    }
    total
}

fn torus_distance(p: Point3) -> f64 {
    let (x, y, z) = (p.x() - TORUS_X, p.y(), p.z() - TORUS_Z);
    (x.hypot(y) - MAJOR).hypot(z) - MINOR
}

fn topology_counts(t: &Topology) -> [usize; 8] {
    [
        t.num_vertices(),
        t.num_edges(),
        t.num_wires(),
        t.num_faces(),
        t.num_shells(),
        t.num_solids(),
        t.num_loops(),
        t.num_coedges(),
    ]
}

fn origin() -> Point3 {
    Point3::new(0.0, 0.0, 0.0)
}

/// Check raw index closure, then the same after welding coincident
/// positions: every edge used exactly twice with opposite directions.
fn assert_closed_oriented(mesh: &TriangleMesh, weld: Option<f64>, label: &str) {
    let mut ids: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    if let Some(grid) = weld {
        let mut first = BTreeMap::new();
        for (i, p) in mesh.positions.iter().enumerate() {
            let key = [p.x(), p.y(), p.z()].map(|c| (c / grid).round() as i64);
            ids[i] = *first.entry(key).or_insert(i as u32);
        }
    }
    let mut uses: BTreeMap<(u32, u32), Vec<bool>> = BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let t = [tri[0], tri[1], tri[2]].map(|i| ids[i as usize]);
        assert!(
            t[0] != t[1] && t[1] != t[2] && t[0] != t[2],
            "{label}: collapsed triangle"
        );
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            uses.entry((a.min(b), a.max(b))).or_default().push(a < b);
        }
    }
    for (edge, directions) in uses {
        assert_eq!(directions.len(), 2, "{label}: incidence {edge:?}");
        assert_ne!(directions[0], directions[1], "{label}: winding {edge:?}");
    }
}

/// Which cell of the matrix a fixture run is.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Cell {
    /// The seed itself: also checked at the campaign's reading deflection.
    Seed,
    /// A control that must fuse exactly and mesh closed.
    Control,
    /// A far scale that may refuse exact-only upstream of meshing.
    FarScale,
}

#[allow(clippy::too_many_lines)]
fn check_fixture(scale: f64, placed: bool, swapped: bool, cell: Cell) {
    let label = format!("scale={scale} placed={placed} swap={swapped}");
    let mut topo = Topology::new();
    // The seed's own construction, in its own order.
    let source = make_torus(&mut topo, MAJOR * scale, MINOR * scale, 16).unwrap();
    let sphere = make_sphere(&mut topo, BALL * scale, 16).unwrap();
    let torus = mirror(
        &mut topo,
        source,
        Point3::new(-4.0 * scale, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    transform_solid(
        &mut topo,
        torus,
        &Mat4::translation(scale, 0.0, -3.0 * scale),
    )
    .unwrap();
    let placement = if placed {
        Mat4::translation(10.0 * scale, -7.0 * scale, 3.0 * scale)
            * Mat4::rotation_x(0.37)
            * Mat4::rotation_z(0.23)
    } else {
        Mat4::identity()
    };
    if placed {
        for solid in [torus, sphere] {
            transform_solid(&mut topo, solid, &placement).unwrap();
        }
    }
    let inverse = placement.inverse().unwrap();
    let local = |p: Point3| {
        let q = inverse.mul_point(p);
        Point3::new(q.x() / scale, q.y() / scale, q.z() / scale)
    };
    let operands = remus_io::arena_io::serialize_solids(&topo, &[torus, sphere]).unwrap();
    let (lhs, rhs) = if swapped {
        (sphere, torus)
    } else {
        (torus, sphere)
    };
    let counts = topology_counts(&topo);
    let result = boolean_with_context(
        &mut topo,
        BooleanOp::Fuse,
        lhs,
        rhs,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    );
    let result = match result {
        // Outside 0.01..=10 the exact pipeline currently declines this pair
        // upstream of meshing. Pin only that the refusal is typed and rolls
        // back; an exact result there must pass every check below.
        Err(error) if cell == Cell::FarScale => {
            assert!(
                matches!(error, OperationsError::ExactOnlyUnattainable),
                "{label}: {error:?}"
            );
            assert_eq!(topology_counts(&topo), counts, "{label}: rollback");
            assert_eq!(
                remus_io::arena_io::serialize_solids(&topo, &[torus, sphere]).unwrap(),
                operands,
                "{label}: operands unchanged"
            );
            return;
        }
        result => result.unwrap_or_else(|e| panic!("{label}: {e:?}")),
    };
    assert!(matches!(result.quality, BooleanQuality::Exact), "{label}");
    let solid = result.solid;
    assert_eq!(
        remus_io::arena_io::serialize_solids(&topo, &[torus, sphere]).unwrap(),
        operands,
        "{label}: operands unchanged"
    );

    // --- B-Rep, before any mesh ---
    let validation = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(validation.is_valid(), "{label}: {:?}", validation.issues);
    let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
    let mut census = [0_usize; 2];
    let mut holed = 0;
    let mut uses = BTreeMap::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        match face.surface() {
            FaceSurface::Torus(_) => census[0] += 1,
            FaceSurface::Sphere(_) => {
                census[1] += 1;
                holed += usize::from(face.inner_wires().len() == 2);
            }
            other => panic!("{label}: unexpected {} carrier", other.type_tag()),
        }
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wid).unwrap().edges() {
                uses.entry(edge.edge())
                    .or_insert_with(Vec::new)
                    .push(edge.is_forward() != face.is_reversed());
            }
        }
    }
    assert_eq!(census, [1, 2], "{label}: one torus piece, two hemispheres");
    assert_eq!(holed, 1, "{label}: one hemisphere carries both holes");
    for (edge, directions) in &uses {
        assert_eq!(directions.len(), 2, "{label}: incidence {edge:?}");
        assert_ne!(directions[0], directions[1], "{label}: same-sense {edge:?}");
    }
    // Every torus boundary edge is a section curve: on both carriers within
    // its own stored tolerance.
    let torus_face = faces
        .iter()
        .copied()
        .find(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .unwrap();
    let tf = topo.face(torus_face).unwrap();
    for wid in std::iter::once(tf.outer_wire()).chain(tf.inner_wires().iter().copied()) {
        for oriented in topo.wire(wid).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let tolerance = edge.tolerance().unwrap_or(1e-7) * scale.max(1.0);
            let (a, b) = edge.strict_domain().unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            for i in 0..=256 {
                let t = a + (b - a) * f64::from(i) / 256.0;
                let p = local(edge.curve().evaluate_with_endpoints(t, start, end));
                assert!(
                    ((p - origin()).length() - BALL).abs() * scale <= tolerance,
                    "{label}: section off sphere"
                );
                assert!(
                    torus_distance(p).abs() * scale <= tolerance,
                    "{label}: section off torus"
                );
            }
        }
    }
    // Cartesian material: inside the ball or the torus, away from both.
    let options = ClassifyOptions::default();
    let mut coverage = [0_usize; 4];
    for x in [-12.0_f64, -10.5, -7.0, -4.5, -1.0, 3.0, 7.0] {
        for y in [-5.5_f64, -1.5, 0.0, 2.5, 6.0] {
            for z in [-5.5_f64, -4.0, -3.0, -1.5, 0.5, 4.0, 7.5] {
                let p = Point3::new(x, y, z);
                let to_ball = (p - origin()).length() - BALL;
                let to_torus = torus_distance(p);
                if to_ball.abs() < 0.2 || to_torus.abs() < 0.2 {
                    continue;
                }
                let (ball, tube) = (to_ball < 0.0, to_torus < 0.0);
                coverage[2 * usize::from(ball) + usize::from(tube)] += 1;
                let point = placement.mul_point(Point3::new(x * scale, y * scale, z * scale));
                for (name, id, inside) in [
                    ("sphere", sphere, ball),
                    ("torus", torus, tube),
                    ("union", solid, ball || tube),
                ] {
                    assert_eq!(
                        classify_point(&topo, id, point, &options).unwrap(),
                        if inside {
                            PointClassification::Inside
                        } else {
                            PointClassification::Outside
                        },
                        "{label}: {name} at {x},{y},{z}"
                    );
                }
            }
        }
    }
    assert!(coverage.iter().all(|&n| n > 0), "{label}: {coverage:?}");
    // Inclusion-exclusion against closed-form primitive volumes.
    let expected = 4.0 / 3.0 * PI * BALL.powi(3) + 2.0 * PI * PI * MAJOR * MINOR * MINOR
        - torus_ball_overlap();
    // The face integrator reads 1.06e-5 high on this trimmed torus (the
    // section curves themselves sit within 3.3e-10 of both carriers); this
    // bound pins the B-Rep's material, not the integrator's accuracy.
    let gauss = mass_properties(&topo, solid).unwrap().mass / scale.powi(3);
    assert!(
        (gauss - expected).abs() < 2e-5 * expected,
        "{label}: Gauss {gauss} expected {expected}"
    );

    // --- Mesh ---
    let diagonal = {
        let b = solid_bounding_box(&topo, solid).unwrap();
        (b.max - b.min).length()
    };
    if cell == Cell::Seed {
        // The campaign's own reading deflection (1.4M triangles): the
        // recorded 1320 open edges, now none.
        let (mesh, _) =
            tessellate_solid_grouped_with_tolerance(&topo, solid, diagonal * 1e-5, 0.35).unwrap();
        assert_eq!(boundary_edge_count(&mesh), 0, "{label}: campaign boundary");
        assert_eq!(
            non_manifold_edge_count(&mesh),
            0,
            "{label}: campaign incidence"
        );
    }
    for deflection in [0.01 * scale, 0.002 * scale] {
        let (mesh, offsets) =
            tessellate_solid_grouped_with_tolerance(&topo, solid, deflection, 0.35).unwrap();
        let tag = format!("{label} deflection={deflection}");
        assert_eq!(boundary_edge_count(&mesh), 0, "{tag}: boundary");
        assert_eq!(non_manifold_edge_count(&mesh), 0, "{tag}: non-manifold");
        assert_closed_oriented(&mesh, None, &format!("{tag} raw"));
        assert_closed_oriented(&mesh, Some(1e-9 * scale), &format!("{tag} welded"));
        let mut volume = 0.0;
        for (index, &fid) in faces.iter().enumerate() {
            assert!(offsets[index + 1] > offsets[index], "{tag}: empty face");
            let spherical = matches!(topo.face(fid).unwrap().surface(), FaceSurface::Sphere(_));
            for tri in
                mesh.indices[offsets[index] as usize..offsets[index + 1] as usize].chunks_exact(3)
            {
                let [a, b, c] =
                    [tri[0], tri[1], tri[2]].map(|i| local(mesh.positions[i as usize]) - origin());
                volume += a.dot(b.cross(c)) / 6.0;
                if spherical {
                    let normal = (b - a).cross(c - a);
                    assert!(
                        normal.dot((a + b + c) * (1.0 / 3.0)) > 0.0,
                        "{tag}: inward sphere triangle"
                    );
                    // The skinned-hole defect left sphere facets across the
                    // tube. A retained sphere facet stays outside the torus
                    // (within its chord sag).
                    let centroid = origin() + (a + b + c) * (1.0 / 3.0);
                    assert!(
                        torus_distance(centroid) > -2.0 * deflection / scale,
                        "{tag}: sphere facet inside the torus"
                    );
                }
            }
        }
        // A closed inscribed mesh converges on the exact volume from its
        // chord sag; 1e-3 relative is far tighter than a skinned hole.
        assert!(
            (volume - expected).abs() < 1e-3 * expected,
            "{tag}: mesh volume {volume} expected {expected}"
        );
    }
}

/// The seed itself at its campaign deflection, in the recorded operand order.
#[test]
fn opseq81_seed_1146645986253340484_fuse_mesh_is_closed() {
    check_fixture(1.0, false, false, Cell::Seed);
}

#[test]
fn torus_sphere_fuse_mesh_closed_under_order_placement_and_scale() {
    // Scale 10 also exceeded the hull-only NURBS hole certificate's work cap.
    for scale in [0.01, 0.1, 1.0, 10.0] {
        for placed in [false, true] {
            for swapped in [false, true] {
                check_fixture(scale, placed, swapped, Cell::Control);
            }
        }
    }
}

#[test]
fn torus_sphere_fuse_far_scales_close_or_refuse_with_rollback() {
    for scale in [0.001, 100.0, 1000.0] {
        for swapped in [false, true] {
            check_fixture(scale, true, swapped, Cell::FarScale);
        }
    }
}
