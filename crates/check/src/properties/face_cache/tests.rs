#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic
)]

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

fn key_of(topo: &Topology, face: FaceId, request: Request) -> Key {
    face_key(topo, face, request).unwrap().1
}

fn fixed(order: usize, reference: Point3) -> Request {
    Request::FixedAbout { order, reference }
}

fn bump(x: f64) -> f64 {
    f64::from_bits(x.to_bits() + 1)
}

/// Anything the integrators read, changed by one ulp or one flag, makes the
/// key differ from the exact one — so the full contribution, which only an
/// exact match serves, is integrated afresh. A discrete change (a flag, a
/// trim, a tolerance, a curve kind, the order or request family) matches
/// nothing at all; an ulp of geometry is still the same face up to a
/// translation. The reference point is not part of the key.
#[test]
fn any_change_to_what_is_integrated_misses() {
    let mut topo = Topology::new();
    let face = nurbs_pillow(&mut topo, 0.4);
    let base = key_of(&topo, face, fixed(5, origin()));
    assert_eq!(
        base.matches(&key_of(&topo, face, fixed(5, origin()))),
        Some(Match::Exact)
    );
    assert_eq!(
        base.matches(&key_of(
            &topo,
            face,
            fixed(5, Point3::new(0.0, 0.0, bump(0.0)))
        )),
        Some(Match::Exact),
        "the reference is recorded in the entry, not the key"
    );
    assert_eq!(
        base.matches(&key_of(
            &topo,
            face,
            Request::Volume {
                order: 5,
                reference: origin()
            }
        )),
        Some(Match::Exact),
        "full contributions and volume terms share entries"
    );
    assert_eq!(base.matches(&key_of(&topo, face, fixed(6, origin()))), None);
    assert_eq!(
        base.matches(&key_of(&topo, face, Request::Area { order: 5 })),
        None
    );

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
    // Edits 1 and 2 move geometry by one ulp: within the translation
    // tolerance. Every other edit is discrete.
    for (k, edit) in edits.iter().enumerate() {
        let mut edited = topo.clone();
        edit(&mut edited);
        let found = base.matches(&key_of(&edited, face, fixed(5, origin())));
        let expected = if matches!(k, 1 | 2) {
            Some(Match::Translated)
        } else {
            None
        };
        assert_eq!(found, expected, "edit {k}");
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
    assert_eq!(
        key_of(&topo, with_twin, fixed(5, center)).matches(&key_of(
            &topo,
            with_shared,
            fixed(5, center)
        )),
        None
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
        key.retained_bytes() + ENTRY_OVERHEAD_BYTES
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
    const LOCK_F64: usize = size_of::<std::sync::OnceLock<f64>>();
    const LOCK_BOOL: usize = size_of::<std::sync::OnceLock<bool>>();
    assert_eq!(size_of::<CylindricalSurface>(), 13 * F);
    assert_eq!(size_of::<ConicalSurface>(), 13 * F);
    assert_eq!(size_of::<SphericalSurface>(), 13 * F);
    assert_eq!(size_of::<ToroidalSurface>(), 14 * F);
    assert_eq!(size_of::<Circle3D>(), 13 * F);
    assert_eq!(size_of::<Ellipse3D>(), 14 * F);
    assert_eq!(size_of::<Hyperbola3D>(), 14 * F);
    assert_eq!(size_of::<Parabola3D>(), 10 * F);
    // Degrees and shared immutable knots/control points/weights, plus derived caches.
    // The caches are `OnceLock`s, whose size follows the platform's `Once`
    // (8 bytes on macOS, a 4-byte futex on Linux), so they are pinned by type
    // rather than by number: a new field still fails here, a new platform
    // does not.
    assert_eq!(
        size_of::<NurbsSurface>(),
        2 * F + 4 * size_of::<std::sync::Arc<Vec<f64>>>() + LOCK_F64 + LOCK_BOOL
    );
    assert_eq!(
        size_of::<NurbsCurve>(),
        F + 3 * size_of::<std::sync::Arc<Vec<f64>>>() + LOCK_F64
    );
    // Start, end, curve, tolerance, trim; point, tolerance.
    assert_eq!(size_of::<Edge>(), 176);
    assert_eq!(size_of::<Vertex>(), 4 * F);
}

/// A planar face bounded by a line and a free-form cubic NURBS edge: the
/// sampled-polygon (fan) path, not the exact Green's-theorem one.
fn planar_fan_face(topo: &mut Topology) -> FaceId {
    let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
    let b = topo.add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), 1e-7));
    let line = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
    let bulge = NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.1, 1.5, 0.0),
            Point3::new(-0.3, 1.2, 0.0),
            Point3::new(0.0, 0.0, 0.0),
        ],
        vec![1.0, 1.0, 1.0, 1.0],
    )
    .unwrap();
    let mut curve = Edge::new(b, a, EdgeCurve::NurbsCurve(bulge));
    curve.set_trim(Some((0.0, 1.0)));
    let curve = topo.add_edge(curve);
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(line, true),
                OrientedEdge::new(curve, true),
            ],
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 0.0,
        },
    ))
}

/// One face of every integration path: exact planar (cube), sampled planar
/// (fan), the torus tube band (and the same band reversed), and a NURBS
/// patch.
fn path_faces(topo: &mut Topology) -> Vec<(&'static str, FaceId)> {
    let mut faces: Vec<(&'static str, FaceId)> = cube_faces(topo)
        .into_iter()
        .map(|face| ("cube", face))
        .collect();
    faces.push(("fan", planar_fan_face(topo)));
    let band = torus_band(topo, Point3::new(13.0, -7.0, 5.0), 0);
    faces.push(("torus band", band));
    let flipped = {
        let data = topo.face(band).unwrap().clone();
        let mut copy = Face::new(
            data.outer_wire(),
            data.inner_wires().to_vec(),
            data.surface().clone(),
        );
        copy.set_reversed(!data.is_reversed());
        topo.add_face(copy)
    };
    faces.push(("reversed band", flipped));
    faces.push(("nurbs", nurbs_pillow(topo, 0.4)));
    faces
}

fn fresh(topo: &Topology, face: FaceId, reference: Point3) -> FaceContribution {
    face_integrator::integrate_face_fixed_about(topo, face, 5, reference).unwrap()
}

/// Every fixture fits a 30-unit box, so `Σ |wᵢ (Pᵢ − R) · nᵢ| / 3` is at most
/// `area · (|R − anchor| + 30)`: the scale rounding is relative to.
fn rounding_scale(area: f64, reference: Point3, anchor: Point3) -> f64 {
    area.abs() * ((reference - anchor).length() + 30.0)
}

fn anchor_of(topo: &Topology, face: FaceId) -> Point3 {
    key_of(topo, face, fixed(5, origin())).anchor
}

/// Re-expressing a cached volume term about another reference matches
/// integrating about that reference afresh to rounding (~1e-13 of the
/// face's own scale), on every integration path, and so does the area; a
/// return to the stored reference is the stored bits.
#[test]
fn rereferenced_volume_terms_match_fresh_integration() {
    let mut topo = Topology::new();
    let faces = path_faces(&mut topo);
    let r0 = Point3::new(0.3, -0.2, 0.1);
    let far = [
        Point3::new(5.0, 7.0, -3.0),
        Point3::new(-120.5, 33.25, 870.0),
        Point3::new(13.0, -7.0, 5.0),
    ];
    let mut worst = 0.0_f64;
    for &(label, face) in &faces {
        let mut cache = FaceIntegralCache::new();
        let first = cache
            .integrate_face_volume_about(&topo, face, 5, r0)
            .unwrap();
        let at_r0 = fresh(&topo, face, r0);
        assert_eq!(first.volume.to_bits(), at_r0.volume.to_bits(), "{label}");
        assert_eq!(first.area.to_bits(), at_r0.area.to_bits(), "{label}");
        for (k, &r1) in far.iter().enumerate() {
            let moved = cache
                .integrate_face_volume_about(&topo, face, 5, r1)
                .unwrap();
            let expected = fresh(&topo, face, r1);
            // The sampled planar path forms its triangles about the
            // reference, so even its area moves in the last bits.
            assert!(
                (moved.area - expected.area).abs() <= expected.area.abs() * 1e-14,
                "{label}: area {} vs fresh {}",
                moved.area,
                expected.area
            );
            let scale = rounding_scale(expected.area, r1, anchor_of(&topo, face));
            let error = (moved.volume - expected.volume).abs() / scale;
            worst = worst.max(error);
            assert!(
                error <= 1e-13,
                "{label} about reference {k}: {} vs fresh {} ({error:e} of scale)",
                moved.volume,
                expected.volume
            );
        }
        let back = cache
            .integrate_face_volume_about(&topo, face, 5, r0)
            .unwrap();
        assert_eq!(back.volume.to_bits(), first.volume.to_bits(), "{label}");
        let stats = cache.stats();
        assert_eq!(
            (
                stats.misses,
                stats.hits,
                stats.rereferenced,
                stats.translated
            ),
            (1, 4, 3, 0),
            "{label}"
        );
    }
    assert!(worst > 0.0 || faces.is_empty());
}

/// The full contribution carries moments the vector area cannot re-express,
/// so it is never served across references or translations.
#[test]
fn full_contributions_are_served_only_bit_identically() {
    let mut topo = Topology::new();
    let face = torus_band(&mut topo, Point3::new(1.0, 2.0, 3.0), 0);
    let copy = translated_copy(&mut topo, face, Vec3::new(0.5, -0.25, 2.0), false);
    let mut cache = FaceIntegralCache::new();
    let r0 = Point3::new(0.0, 0.0, 0.0);
    let r1 = Point3::new(1.0, 2.0, 3.0);
    cache
        .integrate_face_volume_about(&topo, face, 5, r0)
        .unwrap();
    let other_reference = cache
        .integrate_face_fixed_about(&topo, face, 5, r1)
        .unwrap();
    assert_eq!(bits(&other_reference), bits(&fresh(&topo, face, r1)));
    let translated = cache
        .integrate_face_fixed_about(&topo, copy, 5, r0)
        .unwrap();
    assert_eq!(bits(&translated), bits(&fresh(&topo, copy, r0)));
    assert_eq!(cache.stats().misses, 3, "neither may hit");
    let same = cache
        .integrate_face_fixed_about(&topo, face, 5, r1)
        .unwrap();
    assert_eq!(bits(&same), bits(&other_reference));
    assert_eq!(cache.stats().hits, 1);
}

/// Deep-copy `face` with every vertex, edge curve and the surface moved by
/// `delta`, the way a rigid move builds them: `p + δ` for points, centres and
/// control points, `n · δ + d` for a plane. With `renormalize`, circle axes
/// are re-normalised and the normal rebuilt as `u × v` the way
/// `transform_edges` does, which moves them by an ulp.
fn translated_copy(topo: &mut Topology, face: FaceId, delta: Vec3, renormalize: bool) -> FaceId {
    let data = topo.face(face).unwrap().clone();
    let mut vertices = std::collections::HashMap::new();
    let mut edges = std::collections::HashMap::new();
    let mut wires = Vec::new();
    for wire_id in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id).unwrap().clone();
        let mut oriented = Vec::new();
        for use_ in wire.edges() {
            let edge_id = use_.edge();
            if let std::collections::hash_map::Entry::Vacant(slot) = edges.entry(edge_id) {
                let edge = topo.edge(edge_id).unwrap().clone();
                let mut ends = Vec::new();
                for vertex_id in [edge.start(), edge.end()] {
                    let copy = *vertices.entry(vertex_id).or_insert_with(|| {
                        let v = topo.vertex(vertex_id).unwrap().clone();
                        topo.add_vertex(Vertex::new(v.point() + delta, v.tolerance()))
                    });
                    ends.push(copy);
                }
                let curve = match edge.curve() {
                    EdgeCurve::Line => EdgeCurve::Line,
                    EdgeCurve::Circle(c) if renormalize => {
                        let u = c.u_axis().normalize().unwrap();
                        let v = c.v_axis().normalize().unwrap();
                        EdgeCurve::Circle(
                            Circle3D::with_axes(
                                c.center() + delta,
                                u.cross(v).normalize().unwrap(),
                                c.radius() * c.u_axis().length(),
                                u,
                                v,
                            )
                            .unwrap(),
                        )
                    }
                    EdgeCurve::Circle(c) => EdgeCurve::Circle(
                        Circle3D::with_axes(
                            c.center() + delta,
                            c.normal(),
                            c.radius(),
                            c.u_axis(),
                            c.v_axis(),
                        )
                        .unwrap(),
                    ),
                    EdgeCurve::NurbsCurve(c) => EdgeCurve::NurbsCurve(
                        NurbsCurve::new(
                            c.degree(),
                            c.knots().to_vec(),
                            c.control_points().iter().map(|&p| p + delta).collect(),
                            c.weights().to_vec(),
                        )
                        .unwrap(),
                    ),
                    other => panic!("fixture curve {} not translated here", other.type_tag()),
                };
                let mut copy = Edge::new(ends[0], ends[1], curve);
                copy.set_trim(edge.trim());
                copy.set_tolerance(edge.tolerance()).unwrap();
                slot.insert(topo.add_edge(copy));
            }
            oriented.push(OrientedEdge::new(edges[&edge_id], use_.is_forward()));
        }
        wires.push(topo.add_wire(Wire::new(oriented, wire.is_closed()).unwrap()));
    }
    let surface = match data.surface() {
        FaceSurface::Plane { normal, d } => FaceSurface::Plane {
            normal: *normal,
            d: normal.dot(delta).mul_add(1.0, *d),
        },
        FaceSurface::Torus(s) => FaceSurface::Torus(s.translated(delta)),
        FaceSurface::Nurbs(s) => FaceSurface::Nurbs(
            NurbsSurface::new(
                s.degree_u(),
                s.degree_v(),
                s.knots_u().to_vec(),
                s.knots_v().to_vec(),
                s.control_points()
                    .iter()
                    .map(|row| row.iter().map(|&p| p + delta).collect())
                    .collect(),
                s.weights().to_vec(),
            )
            .unwrap(),
        ),
        other => panic!("fixture surface {} not translated here", other.type_tag()),
    };
    let mut copy = Face::new(wires[0], wires[1..].to_vec(), surface);
    copy.set_reversed(data.is_reversed());
    topo.add_face(copy)
}

/// A rigidly translated face — built the way a move builds it, including the
/// ulp-moved axes `transform_edges` leaves on a circle — reads its source's
/// cached volume term shifted by `(1/3) δ · N`, which matches integrating the
/// copy afresh to rounding; its probe area is the source's. Area requests
/// (`face_area`) do not reuse translations.
#[test]
fn translated_faces_read_their_sources_integral() {
    let deltas = [
        Vec3::new(-6.0, 0.0, 0.0),
        Vec3::new(0.1, 0.2, 0.3),
        Vec3::new(-3.7e2, 1.25e3, -7.1e1),
    ];
    let reference = Point3::new(11.0, 33.0, 33.5);
    let mut topo = Topology::new();
    let faces = path_faces(&mut topo);
    for &(label, face) in &faces {
        for (k, &delta) in deltas.iter().enumerate() {
            for renormalize in [false, true] {
                let copy = translated_copy(&mut topo, face, delta, renormalize);
                let mut cache = FaceIntegralCache::new();
                let source = cache
                    .integrate_face_volume_about(&topo, face, 5, reference)
                    .unwrap();
                let read = cache
                    .integrate_face_volume_about(&topo, copy, 5, reference)
                    .unwrap();
                let stats = cache.stats();
                assert_eq!(
                    (stats.misses, stats.translated),
                    (1, 1),
                    "{label}, delta {k}, renormalize {renormalize}"
                );
                assert_eq!(read.area.to_bits(), source.area.to_bits());
                let expected = fresh(&topo, copy, reference);
                let scale = rounding_scale(expected.area, reference, anchor_of(&topo, copy));
                let error = (read.volume - expected.volume).abs() / scale;
                assert!(
                    error <= 1e-12,
                    "{label}, delta {k}: {} vs fresh {} ({error:e} of scale)",
                    read.volume,
                    expected.volume
                );
                assert!(
                    (read.area - expected.area).abs() <= expected.area.abs() * 1e-12,
                    "{label}, delta {k}: area {} vs fresh {}",
                    read.area,
                    expected.area
                );
                // `face_area` is a reported measurement: a translated copy
                // is integrated afresh, bit for bit.
                cache.integrate_face_area(&topo, face, 8).unwrap();
                let before = cache.stats();
                let area = cache.integrate_face_area(&topo, copy, 8).unwrap();
                assert_eq!(
                    area.to_bits(),
                    face_integrator::integrate_face_area(&topo, copy, 8)
                        .unwrap()
                        .to_bits()
                );
                assert_eq!(cache.stats().misses, before.misses + 1);
            }
        }
    }
}

/// Mirrored, rotated, scaled or locally edited copies are not translations:
/// they miss, so they integrate afresh.
#[test]
fn non_translations_miss() {
    let center = Point3::new(1.0, -2.0, 0.5);
    let mut topo = Topology::new();
    let band = torus_band(&mut topo, center, 0);
    let mut cache = FaceIntegralCache::new();
    let reference = Point3::new(0.0, 0.0, 0.0);
    cache
        .integrate_face_volume_about(&topo, band, 5, reference)
        .unwrap();

    // Same torus, rims at other tube angles: a different region.
    let torus = ToroidalSurface::new(center, 10.0, 3.0).unwrap();
    let mut rim_edges = Vec::new();
    for u in [0.3_f64, 1.2] {
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
        rim_edges.push(topo.add_edge(edge));
    }
    let outer =
        topo.add_wire(Wire::new(vec![OrientedEdge::new(rim_edges[0], false)], true).unwrap());
    let inner =
        topo.add_wire(Wire::new(vec![OrientedEdge::new(rim_edges[1], true)], true).unwrap());
    let other_region = topo.add_face(Face::new(outer, vec![inner], FaceSurface::Torus(torus)));

    // A bigger tube.
    let fatter = {
        let data = topo.face(band).unwrap().clone();
        let FaceSurface::Torus(t) = data.surface() else {
            unreachable!()
        };
        let grown = ToroidalSurface::new(t.center(), t.major_radius(), 3.0 * (1.0 + 1e-9)).unwrap();
        topo.add_face(Face::new(
            data.outer_wire(),
            data.inner_wires().to_vec(),
            FaceSurface::Torus(grown),
        ))
    };

    // A pillow with one corner moved by 1e-9 — far above rounding.
    let pillow = nurbs_pillow(&mut topo, 0.4);
    cache
        .integrate_face_volume_about(&topo, pillow, 5, reference)
        .unwrap();
    let nudged = translated_copy(&mut topo, pillow, Vec3::new(0.0, 0.0, 0.0), false);
    let corner = {
        let wire = topo.face(nudged).unwrap().outer_wire();
        let edge = topo.wire(wire).unwrap().edges()[1].edge();
        topo.edge(edge).unwrap().end()
    };
    let p = topo.vertex(corner).unwrap().point();
    topo.vertex_mut(corner)
        .unwrap()
        .set_point(Point3::new(p.x() + 1e-9, p.y(), p.z()));

    let before = cache.stats();
    for face in [other_region, fatter, nudged] {
        let read = cache
            .integrate_face_volume_about(&topo, face, 5, reference)
            .unwrap();
        let expected = fresh(&topo, face, reference);
        assert_eq!(read.volume.to_bits(), expected.volume.to_bits());
    }
    let after = cache.stats();
    assert_eq!(after.misses - before.misses, 3);
    assert_eq!(after.hits, before.hits);
}

/// The per-thread volume reads are off until enabled, then re-reference.
#[test]
fn thread_volume_reads_rereference_once_enabled() {
    set_thread_face_cache_limits(0, 0);
    let mut topo = Topology::new();
    let face = nurbs_pillow(&mut topo, 0.25);
    let (r0, r1) = (Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, -1.0, 2.0));
    let off = integrate_face_volume_about_memoized(&topo, face, 5, r1).unwrap();
    assert_eq!(
        off.volume.to_bits(),
        fresh(&topo, face, r1).volume.to_bits()
    );
    assert_eq!(thread_face_cache_stats(), FaceCacheStats::default());

    enable_thread_face_cache();
    integrate_face_volume_about_memoized(&topo, face, 5, r0).unwrap();
    let moved = integrate_face_volume_about_memoized(&topo, face, 5, r1).unwrap();
    let expected = fresh(&topo, face, r1);
    assert!(
        (moved.volume - expected.volume).abs()
            <= 1e-13 * rounding_scale(expected.area, r1, anchor_of(&topo, face))
    );
    let stats = thread_face_cache_stats();
    assert_eq!((stats.misses, stats.hits, stats.rereferenced), (1, 1, 1));
    clear_thread_face_cache();
    set_thread_face_cache_limits(0, 0);
}

/// A lightweight probe retains the full rule's area, volume and vector area
/// bits on each qualified path, including rational and reversed NURBS faces.
#[test]
fn volume_only_matches_full_rule_area_volume_and_flux_bits() {
    let mut topo = Topology::new();
    let mut faces = path_faces(&mut topo);
    let band = torus_band(&mut topo, Point3::new(13.0, -7.0, 5.0), 0);
    let data = topo.face(band).unwrap().clone();
    let FaceSurface::Torus(torus) = data.surface() else {
        panic!("torus fixture");
    };
    let periodic = remus_geometry::convert::surface_to_nurbs::torus_to_nurbs(torus).unwrap();
    assert!(periodic.is_periodic_v());
    let periodic = topo.add_face(Face::new(
        data.outer_wire(),
        data.inner_wires().to_vec(),
        FaceSurface::Nurbs(periodic),
    ));
    faces.push(("periodic rational NURBS", periodic));
    let original = nurbs_pillow(&mut topo, 0.4);
    let data = topo.face(original).unwrap().clone();
    let FaceSurface::Nurbs(surface) = data.surface() else {
        panic!("NURBS fixture");
    };
    let mut weights = surface.weights().to_vec();
    weights[1][1] = 1.7;
    weights[2][2] = 0.4;
    let rational = NurbsSurface::new(
        surface.degree_u(),
        surface.degree_v(),
        surface.knots_u().to_vec(),
        surface.knots_v().to_vec(),
        surface.control_points().to_vec(),
        weights,
    )
    .unwrap();
    let rational = topo.add_face(Face::new(
        data.outer_wire(),
        data.inner_wires().to_vec(),
        FaceSurface::Nurbs(rational),
    ));
    faces.push(("rational NURBS", rational));
    let reversed = {
        let mut copy = topo.face(rational).unwrap().clone();
        copy.set_reversed(true);
        topo.add_face(copy)
    };
    faces.push(("reversed rational NURBS", reversed));
    for order in [3, 5, 8] {
        for reference in [origin(), Point3::new(0.5, -0.25, 900.0)] {
            for &(label, face) in &faces {
                let (full, full_flux) =
                    face_integrator::integrate_face_fixed_flux_about(&topo, face, order, reference)
                        .unwrap();
                let (light, light_flux) = face_integrator::integrate_face_fixed_volume_flux_about(
                    &topo, face, order, reference,
                )
                .unwrap();
                assert_eq!(
                    light.area.to_bits(),
                    full.area.to_bits(),
                    "{label}, order {order}"
                );
                assert_eq!(
                    light.volume.to_bits(),
                    full.volume.to_bits(),
                    "{label}, order {order}"
                );
                let flux_bits = |v: Option<Vec3>| {
                    v.map(|v| [v.x().to_bits(), v.y().to_bits(), v.z().to_bits()])
                };
                assert_eq!(
                    flux_bits(light_flux),
                    flux_bits(full_flux),
                    "{label}, order {order}"
                );
            }
        }
    }
}

/// A volume-only reading at exactly the same reference must not populate
/// centroid/inertia answers; a complete entry may still serve volume reads.
#[test]
fn volume_only_cache_entry_cannot_answer_full_properties() {
    let mut topo = Topology::new();
    let face = nurbs_pillow(&mut topo, 0.4);
    let reference = Point3::new(0.2, -0.3, 0.4);
    let direct = fresh(&topo, face, reference);
    let mut cache = FaceIntegralCache::new();
    let light = cache
        .integrate_face_volume_about(&topo, face, 5, reference)
        .unwrap();
    assert_eq!(light.area.to_bits(), direct.area.to_bits());
    assert_eq!(light.volume.to_bits(), direct.volume.to_bits());
    assert_eq!(cache.stats().misses, 1);
    let full = cache
        .integrate_face_fixed_about(&topo, face, 5, reference)
        .unwrap();
    assert_eq!(bits(&full), bits(&direct));
    assert_eq!(
        cache.stats().misses,
        2,
        "partial entry must miss even at the same reference"
    );
    let again = cache
        .integrate_face_fixed_about(&topo, face, 5, reference)
        .unwrap();
    assert_eq!(bits(&again), bits(&direct));
    assert_eq!(cache.stats().hits, 1);

    let mut full_first = FaceIntegralCache::new();
    full_first
        .integrate_face_fixed_about(&topo, face, 5, reference)
        .unwrap();
    let light = full_first
        .integrate_face_volume_about(&topo, face, 5, reference)
        .unwrap();
    assert_eq!(light.area.to_bits(), direct.area.to_bits());
    assert_eq!(light.volume.to_bits(), direct.volume.to_bits());
    assert_eq!((full_first.stats().misses, full_first.stats().hits), (1, 1));
}
