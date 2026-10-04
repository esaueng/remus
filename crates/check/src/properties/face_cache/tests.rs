#![allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]

use super::*;
use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_topology::edge::{Edge, EdgeId};
use remus_topology::face::Face;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use std::f64::consts::TAU;

fn bits(c: &FaceContribution) -> [u64; 14] {
    [
        c.area,
        c.volume,
        c.volume_moment_x,
        c.volume_moment_y,
        c.volume_moment_z,
        c.volume_second_x,
        c.volume_second_y,
        c.volume_second_z,
        c.volume_product_xy,
        c.volume_product_xz,
        c.volume_product_yz,
        c.centroid_x,
        c.centroid_y,
        c.centroid_z,
    ]
    .map(f64::to_bits)
}

/// A torus tube band between two rim circles: a curved, trimmed face with an
/// inner wire and closed single-vertex edges. `pad` dummy vertices shift
/// every handle so two copies differ in handle values only.
fn torus_band(topo: &mut Topology, center: Point3, pad: usize) -> FaceId {
    for _ in 0..pad {
        topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
    }
    let torus = ToroidalSurface::new(center, 10.0, 3.0).unwrap();
    let mut edges = Vec::new();
    for u in [0.2_f64, 1.2] {
        let radial = Vec3::new(u.cos(), u.sin(), 0.0);
        let circle = Circle3D::new_with_ref(
            center + radial * 10.0,
            Vec3::new(u.sin(), -u.cos(), 0.0),
            3.0,
            radial,
        )
        .unwrap();
        let vertex = topo.add_vertex(Vertex::new(center + radial * 13.0, 1e-7));
        let mut edge = Edge::new(vertex, vertex, EdgeCurve::Circle(circle));
        edge.set_trim(Some((0.0, TAU)));
        edges.push(topo.add_edge(edge));
    }
    let outer = topo.add_wire(Wire::new(vec![OrientedEdge::new(edges[0], false)], true).unwrap());
    let inner = topo.add_wire(Wire::new(vec![OrientedEdge::new(edges[1], true)], true).unwrap());
    topo.add_face(Face::new(outer, vec![inner], FaceSurface::Torus(torus)))
}

/// A bicubic NURBS "pillow" over the unit square: straight boundary rows
/// and columns, raised interior, bounded by four line edges.
fn nurbs_pillow(topo: &mut Topology, lift: f64) -> FaceId {
    let cps: Vec<Vec<Point3>> = (0..4)
        .map(|i| {
            (0..4)
                .map(|j| {
                    let (x, y) = (f64::from(i) / 3.0, f64::from(j) / 3.0);
                    let inside = (1..3).contains(&i) && (1..3).contains(&j);
                    Point3::new(x, y, if inside { lift } else { 0.0 })
                })
                .collect()
        })
        .collect();
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let surface =
        NurbsSurface::new(3, 3, knots.clone(), knots, cps, vec![vec![1.0; 4]; 4]).unwrap();
    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let vertices: Vec<_> = corners
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect();
    let edges: Vec<_> = (0..4)
        .map(|k| {
            let edge = topo.add_edge(Edge::new(
                vertices[k],
                vertices[(k + 1) % 4],
                EdgeCurve::Line,
            ));
            OrientedEdge::new(edge, true)
        })
        .collect();
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface)))
}

fn cube_faces(topo: &mut Topology) -> Vec<FaceId> {
    let solid = remus_topology::test_utils::make_unit_cube_manifold(topo);
    remus_topology::explorer::solid_faces(topo, solid).unwrap()
}

fn origin() -> Point3 {
    Point3::new(0.0, 0.0, 0.0)
}

#[test]
fn hits_are_bit_identical_to_integrating() {
    let mut topo = Topology::new();
    let mut faces = cube_faces(&mut topo);
    faces.push(torus_band(&mut topo, Point3::new(13.0, -7.0, 5.0), 0));
    faces.push(nurbs_pillow(&mut topo, 0.4));
    let reference = Point3::new(0.5, 0.25, -0.75);
    let mut cache = FaceIntegralCache::new();
    for pass in 0..2 {
        for &face in &faces {
            let direct =
                face_integrator::integrate_face_fixed_about(&topo, face, 5, reference).unwrap();
            let cached = cache
                .integrate_face_fixed_about(&topo, face, 5, reference)
                .unwrap();
            assert_eq!(bits(&cached), bits(&direct), "pass {pass}");
            let area = face_integrator::integrate_face_area(&topo, face, 8).unwrap();
            let cached_area = cache.integrate_face_area(&topo, face, 8).unwrap();
            assert_eq!(cached_area.to_bits(), area.to_bits(), "pass {pass}");
        }
    }
    let stats = cache.stats();
    let n = 2 * faces.len() as u64;
    assert_eq!((stats.misses, stats.insertions, stats.hits), (n, n, n));
    assert_eq!(stats.len, 2 * faces.len());
}

/// A face rebuilt with the same content under different handles — in the
/// same topology or another one — is the same entry.
#[test]
fn equal_content_under_other_handles_hits() {
    let center = Point3::new(1.0, 2.0, 3.0);
    let mut first = Topology::new();
    let a = torus_band(&mut first, center, 0);
    let b = torus_band(&mut first, center, 3);
    let mut second = Topology::new();
    let c = torus_band(&mut second, center, 11);
    let mut cache = FaceIntegralCache::new();
    let reference = Point3::new(1.0, 2.0, 3.0);
    let fa = cache
        .integrate_face_fixed_about(&first, a, 5, reference)
        .unwrap();
    let fb = cache
        .integrate_face_fixed_about(&first, b, 5, reference)
        .unwrap();
    let fc = cache
        .integrate_face_fixed_about(&second, c, 5, reference)
        .unwrap();
    assert_eq!(bits(&fa), bits(&fb));
    assert_eq!(bits(&fa), bits(&fc));
    let stats = cache.stats();
    assert_eq!((stats.misses, stats.hits, stats.len), (1, 2, 1));
}

type Edit = Box<dyn Fn(&mut Topology)>;

fn key_of(topo: &Topology, face: FaceId, request: Request) -> Vec<u64> {
    face_key(topo, face, request).unwrap().1
}

fn fixed(order: usize, reference: Point3) -> Request {
    Request::FixedAbout { order, reference }
}

fn bump(x: f64) -> f64 {
    f64::from_bits(x.to_bits() + 1)
}

/// Anything the integrators read, changed by one ulp or one flag, changes
/// the key; so does the request.
#[test]
fn any_change_to_what_is_integrated_misses() {
    let mut topo = Topology::new();
    let face = nurbs_pillow(&mut topo, 0.4);
    let base = key_of(&topo, face, fixed(5, origin()));
    assert_eq!(base, key_of(&topo, face, fixed(5, origin())));

    assert_ne!(base, key_of(&topo, face, fixed(6, origin())));
    assert_ne!(
        base,
        key_of(&topo, face, fixed(5, Point3::new(0.0, 0.0, bump(0.0))))
    );
    assert_ne!(base, key_of(&topo, face, Request::Area { order: 5 }));

    let mut edits: Vec<Edit> = Vec::new();
    edits.push(Box::new(move |t| {
        let f = t.face_mut(face).unwrap();
        f.set_reversed(!f.is_reversed());
    }));
    edits.push(Box::new(move |t| {
        let lifted = nurbs_pillow(t, bump(0.4));
        let surface = t.face(lifted).unwrap().surface().clone();
        t.face_mut(face).unwrap().set_surface(surface);
    }));
    let wire = topo.face(face).unwrap().outer_wire();
    let first_edge = topo.wire(wire).unwrap().edges()[0].edge();
    let first_vertex = topo.edge(first_edge).unwrap().start();
    edits.push(Box::new(move |t| {
        let v = t.vertex_mut(first_vertex).unwrap();
        let p = v.point();
        v.set_point(Point3::new(bump(p.x()), p.y(), p.z()));
    }));
    edits.push(Box::new(move |t| {
        t.vertex_mut(first_vertex)
            .unwrap()
            .set_tolerance(2e-7)
            .unwrap();
    }));
    edits.push(Box::new(move |t| {
        t.edge_mut(first_edge).unwrap().set_trim(Some((0.0, 1.0)));
    }));
    edits.push(Box::new(move |t| {
        t.edge_mut(first_edge)
            .unwrap()
            .set_tolerance(Some(1e-6))
            .unwrap();
    }));
    edits.push(Box::new(move |t| {
        let curve = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![origin(), Point3::new(1.0, 0.0, 0.0)],
            vec![1.0, 1.0],
        )
        .unwrap();
        t.edge_mut(first_edge)
            .unwrap()
            .set_curve(EdgeCurve::NurbsCurve(curve));
    }));
    for (k, edit) in edits.iter().enumerate() {
        let mut edited = topo.clone();
        edit(&mut edited);
        assert_ne!(
            base,
            key_of(&edited, face, fixed(5, origin())),
            "edit {k} must change the key"
        );
    }
}

/// The key records which uses share an entity: one closed edge walked by
/// both wires is not two equal edges.
#[test]
fn shared_entities_are_part_of_the_key() {
    let center = Point3::new(0.0, 0.0, 0.0);
    let mut topo = Topology::new();
    let face = torus_band(&mut topo, center, 0);
    let outer = topo.face(face).unwrap().outer_wire();
    let outer_edge: EdgeId = topo.wire(outer).unwrap().edges()[0].edge();
    let twin = topo.edge(outer_edge).unwrap().clone();
    let twin_edge = topo.add_edge(twin);
    let separate =
        topo.add_wire(Wire::new(vec![OrientedEdge::new(twin_edge, true)], true).unwrap());
    let shared = topo.add_wire(Wire::new(vec![OrientedEdge::new(outer_edge, true)], true).unwrap());
    let surface = topo.face(face).unwrap().surface().clone();
    let with_twin = topo.add_face(Face::new(outer, vec![separate], surface.clone()));
    let with_shared = topo.add_face(Face::new(outer, vec![shared], surface));
    assert_ne!(
        key_of(&topo, with_twin, fixed(5, center)),
        key_of(&topo, with_shared, fixed(5, center))
    );
}

#[test]
fn bounds_evict_oldest_first_and_skip_oversized_entries() {
    let mut topo = Topology::new();
    let faces = cube_faces(&mut topo);
    let mut cache = FaceIntegralCache::with_limits(2, usize::MAX);
    for &face in &faces[..3] {
        cache.integrate_face_area(&topo, face, 5).unwrap();
    }
    let stats = cache.stats();
    assert_eq!((stats.len, stats.evictions, stats.insertions), (2, 1, 3));
    // The first face was evicted (FIFO); the last two hit.
    cache.integrate_face_area(&topo, faces[2], 5).unwrap();
    cache.integrate_face_area(&topo, faces[1], 5).unwrap();
    assert_eq!(cache.stats().hits, 2);
    cache.integrate_face_area(&topo, faces[0], 5).unwrap();
    assert_eq!(cache.stats().misses, 4);

    // A byte budget below one entry retains nothing but still answers.
    let mut tiny = FaceIntegralCache::with_limits(16, 8);
    let area = tiny.integrate_face_area(&topo, faces[0], 5).unwrap();
    assert_eq!(
        area.to_bits(),
        face_integrator::integrate_face_area(&topo, faces[0], 5)
            .unwrap()
            .to_bits()
    );
    assert_eq!((tiny.stats().len, tiny.stats().insertions), (0, 0));

    // The byte budget evicts as well.
    let one = {
        let (_, key) = face_key(&topo, faces[0], Request::Area { order: 5 }).unwrap();
        key.len() * 8 + ENTRY_OVERHEAD_BYTES
    };
    let mut budget = FaceIntegralCache::with_limits(16, one * 2);
    for &face in &faces {
        budget.integrate_face_area(&topo, face, 5).unwrap();
    }
    assert!(budget.stats().retained_bytes <= one * 2);
    assert_eq!(budget.stats().len, 2);

    // Shrinking evicts, clearing keeps the counters, disabled retains nothing.
    budget.set_limits(1, usize::MAX);
    assert_eq!(budget.stats().len, 1);
    let before = budget.stats();
    budget.clear();
    let after = budget.stats();
    assert_eq!((after.len, after.retained_bytes), (0, 0));
    assert_eq!((after.hits, after.misses), (before.hits, before.misses));
    let mut off = FaceIntegralCache::disabled();
    off.integrate_face_area(&topo, faces[0], 5).unwrap();
    off.integrate_face_area(&topo, faces[0], 5).unwrap();
    assert_eq!(off.stats(), FaceIntegralCache::disabled().stats());
}

#[test]
fn errors_are_reported_and_never_cached() {
    let mut topo = Topology::new();
    let face = cube_faces(&mut topo)[0];
    let mut cache = FaceIntegralCache::new();
    let bad = Point3::new(f64::NAN, 0.0, 0.0);
    assert!(
        cache
            .integrate_face_fixed_about(&topo, face, 5, bad)
            .is_err()
    );
    assert!(
        cache
            .integrate_face_fixed_about(&topo, face, 5, bad)
            .is_err()
    );
    assert_eq!(cache.stats().len, 0);
}

/// The per-thread instance is off until enabled, then serves the memoized
/// functions; its results match the plain integrators bit for bit.
#[test]
fn thread_cache_is_opt_in() {
    // Tests share threads with each other only through this function.
    set_thread_face_cache_limits(0, 0);
    let mut topo = Topology::new();
    let face = torus_band(&mut topo, Point3::new(2.0, 0.0, 0.0), 0);
    let direct = face_integrator::integrate_face_fixed_about(&topo, face, 5, origin()).unwrap();
    let off = integrate_face_fixed_about_memoized(&topo, face, 5, origin()).unwrap();
    assert_eq!(bits(&off), bits(&direct));
    assert_eq!(thread_face_cache_stats(), FaceCacheStats::default());

    enable_thread_face_cache();
    let stats = thread_face_cache_stats();
    assert_eq!(
        (stats.capacity, stats.byte_budget),
        (DEFAULT_FACE_CACHE_CAPACITY, DEFAULT_FACE_CACHE_BYTE_BUDGET)
    );
    let first = integrate_face_fixed_about_memoized(&topo, face, 5, origin()).unwrap();
    let second = integrate_face_fixed_about_memoized(&topo, face, 5, origin()).unwrap();
    assert_eq!(bits(&first), bits(&direct));
    assert_eq!(bits(&second), bits(&direct));
    let area = integrate_face_area_memoized(&topo, face, 8).unwrap();
    assert_eq!(
        area.to_bits(),
        face_integrator::integrate_face_area(&topo, face, 8)
            .unwrap()
            .to_bits()
    );
    let stats = thread_face_cache_stats();
    assert_eq!((stats.misses, stats.hits, stats.len), (2, 1, 2));
    clear_thread_face_cache();
    assert_eq!(thread_face_cache_stats().len, 0);
    set_thread_face_cache_limits(0, 0);
}

/// Tripwire for the key's completeness: each geometry and topology type the
/// key reads field by field is pinned at its current size, so adding a field
/// fails here until `KeyWriter` (and this pin) learn about it.
#[cfg(target_pointer_width = "64")]
#[test]
fn key_reads_every_geometry_field() {
    use std::mem::size_of;
    const F: usize = size_of::<f64>();
    assert_eq!(size_of::<CylindricalSurface>(), 13 * F);
    assert_eq!(size_of::<ConicalSurface>(), 13 * F);
    assert_eq!(size_of::<SphericalSurface>(), 13 * F);
    assert_eq!(size_of::<ToroidalSurface>(), 14 * F);
    assert_eq!(size_of::<Circle3D>(), 13 * F);
    assert_eq!(size_of::<Ellipse3D>(), 14 * F);
    assert_eq!(size_of::<Hyperbola3D>(), 14 * F);
    assert_eq!(size_of::<Parabola3D>(), 10 * F);
    // Degrees, knots, control points and weights, plus the derived caches.
    assert_eq!(size_of::<NurbsSurface>(), 144);
    assert_eq!(size_of::<NurbsCurve>(), 96);
    // Start, end, curve, tolerance, trim; point, tolerance.
    assert_eq!(size_of::<Edge>(), 176);
    assert_eq!(size_of::<Vertex>(), 4 * F);
}
