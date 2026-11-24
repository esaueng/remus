//! Moving a face whose rim band has no history after a STEP round trip
//! (#1957).
//!
//! A STEP file carries no feature history, so a sloped band standing on the
//! walls a face move slides along could be a chamfer (carried with the face)
//! or a draft (re-limited against it): equal setbacks describe a 45° draft as
//! well as a symmetric chamfer, unequal ones a two-distance chamfer as well
//! as a steeper draft. The move refuses with `AmbiguousRimChamfer`, on the
//! direct and the prepared path, leaving the source untouched; the caller's
//! explicit reading builds that shape exactly (face position, footprint,
//! volume against closed forms).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::{read_step, write_step};
use remus_math::vec::Vec3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::chamfer_v2;
use remus_operations::measure::{face_area, solid_bounding_box, solid_volume};
use remus_operations::prepared_move::PreparedFaceMove;
use remus_operations::primitives::make_box;
use remus_operations::push_pull::{
    AmbiguousRimResolution, MoveFacesOptions, move_faces, move_faces_with_options,
};
use remus_operations::tessellate::{is_watertight, tessellate_solid_with_tolerance};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{face_vertices, solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

const X: f64 = 60.0;
const Y: f64 = 40.0;
const Z: f64 = 20.0;
const DEFLECTION: f64 = 1e-3;

const AS_CHAMFER: MoveFacesOptions =
    MoveFacesOptions::new().with_ambiguous_rim(AmbiguousRimResolution::TreatAsChamfer);
const AS_DRAFT: MoveFacesOptions =
    MoveFacesOptions::new().with_ambiguous_rim(AmbiguousRimResolution::TreatAsDraft);

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

fn edges_where(
    topo: &Topology,
    solid: SolidId,
    keep: impl Fn([f64; 3], [f64; 3]) -> bool,
) -> Vec<EdgeId> {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let edge = topo.edge(edge).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            keep([a.x(), a.y(), a.z()], [b.x(), b.y(), b.z()])
        })
        .collect()
}

fn bottom_rim(a: [f64; 3], b: [f64; 3]) -> bool {
    near(a[2], 0.0) && near(b[2], 0.0)
}

fn front_bottom(a: [f64; 3], b: [f64; 3]) -> bool {
    bottom_rim(a, b) && near(a[1], 0.0) && near(b[1], 0.0)
}

/// The largest planar face facing −z.
fn bottom(topo: &Topology, solid: SolidId) -> FaceId {
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(Vec3::new(0.0, 0.0, -1.0)) > 1.0 - 1e-9)
        })
        .max_by(|&a, &b| {
            face_area(topo, a, 0.01)
                .unwrap()
                .total_cmp(&face_area(topo, b, 0.01).unwrap())
        })
        .unwrap()
}

/// A 60 × 40 × 20 box chamfered along `keep` with setbacks `d1`/`d2`, written
/// to STEP and read back into a fresh topology: no history survives.
fn imported(keep: fn([f64; 3], [f64; 3]) -> bool, d1: f64, d2: f64) -> (Topology, SolidId) {
    let mut native = Topology::new();
    let blank = make_box(&mut native, X, Y, Z).unwrap();
    let edges = edges_where(&native, blank, keep);
    let solid = chamfer_v2(&mut native, blank, &edges, d1, d2)
        .unwrap()
        .solid;
    let text = write_step(&native, &[solid]).unwrap();
    let mut topo = Topology::new();
    let solids = read_step(&text, &mut topo).unwrap();
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

/// Simpson's rule, exact for the quadratic sections below.
fn simpson(f: impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    (b - a) / 6.0 * (4.0f64.mul_add(f(0.5 * (a + b)), f(a)) + f(b))
}

/// What a move by `distance` must build.
struct Expected {
    /// Volume change.
    volume: f64,
    /// Area of the moved bottom face.
    bottom_area: f64,
}

/// The default move refuses on both paths, naming a band of the bottom rim,
/// and leaves the source as it was. Returns the band's setbacks (cap, wall).
fn assert_refuses(topo: &mut Topology, solid: SolidId, label: &str) -> (f64, f64) {
    let face = bottom(topo, solid);
    let counts = solid_entity_counts(topo, solid).unwrap();
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    let mut prepared = PreparedFaceMove::prepare(topo, solid, &[face]).unwrap();
    let mut setbacks = None;
    for distance in [2.0, -0.1] {
        for error in [
            move_faces(topo, solid, &[face], distance).unwrap_err(),
            prepared.move_faces(topo, distance).unwrap_err(),
        ] {
            let OperationsError::AmbiguousRimChamfer(band) = error else {
                panic!("{label}, move {distance}: expected an ambiguous-rim refusal, got {error}");
            };
            assert_eq!(band.cap, face, "{label}");
            assert!(band.face != face && band.wall != face, "{label}");
            let edge = topo.edge(band.edge).unwrap();
            for vertex in [edge.start(), edge.end()] {
                assert!(
                    near(topo.vertex(vertex).unwrap().point().z(), 0.0),
                    "{label}"
                );
            }
            assert!(
                error_names_band(&band),
                "{label}: refusal text {}",
                OperationsError::AmbiguousRimChamfer(band)
            );
            setbacks.get_or_insert((band.cap_setback, band.wall_setback));
        }
    }
    assert_eq!(solid_entity_counts(topo, solid).unwrap(), counts, "{label}");
    assert!(
        (solid_volume(topo, solid, DEFLECTION).unwrap() - volume).abs() <= 1e-9 * volume,
        "{label}: source changed"
    );
    setbacks.unwrap()
}

fn error_names_band(band: &remus_operations::push_pull::AmbiguousRimBand) -> bool {
    let text = OperationsError::AmbiguousRimChamfer(*band).to_string();
    text.contains(&format!("face {}", band.face.index()))
        && text.contains(&format!("cap edge {}", band.edge.index()))
}

/// Moves the bottom by `distance` with `options`, directly and prepared, and
/// checks the result: valid, closed, the bottom face at `z = -distance` with
/// the expected area, the footprint in x and y unchanged, the volume changed
/// as expected, the source untouched.
fn assert_moves(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
    options: MoveFacesOptions,
    expected: &Expected,
    label: &str,
) {
    let face = bottom(topo, solid);
    let counts = solid_entity_counts(topo, solid).unwrap();
    let volume = solid_volume(topo, solid, DEFLECTION).unwrap();
    let before = solid_bounding_box(topo, solid).unwrap();
    let mut prepared =
        PreparedFaceMove::prepare_with_options(topo, solid, &[face], options).unwrap();
    let direct = move_faces_with_options(topo, solid, &[face], distance, options)
        .unwrap_or_else(|error| panic!("{label}, move {distance}: {error}"))
        .solid;
    let dragged = prepared
        .move_faces(topo, distance)
        .unwrap_or_else(|error| panic!("{label}, prepared move {distance}: {error}"))
        .solid;
    for (path, moved) in [("direct", direct), ("prepared", dragged)] {
        let label = format!("{label}, {path} move {distance}");
        assert!(validate_solid(topo, moved).unwrap().is_valid(), "{label}");
        assert_eq!(solid_entity_counts(topo, moved).unwrap(), counts, "{label}");
        let mesh = tessellate_solid_with_tolerance(topo, moved, 0.01, 0.08).unwrap();
        assert!(is_watertight(&mesh), "{label}: open mesh");

        let moved_bottom = bottom(topo, moved);
        for vertex in face_vertices(topo, moved_bottom).unwrap() {
            let z = topo.vertex(vertex).unwrap().point().z();
            assert!(near(z, -distance), "{label}: bottom vertex at z = {z}");
        }
        let area = face_area(topo, moved_bottom, DEFLECTION).unwrap();
        assert!(
            (area - expected.bottom_area).abs() <= 1e-9 * expected.bottom_area,
            "{label}: bottom area {area}, expected {}",
            expected.bottom_area
        );
        let after = solid_bounding_box(topo, moved).unwrap();
        for (a, b) in [
            (after.min.x(), before.min.x()),
            (after.max.x(), before.max.x()),
            (after.min.y(), before.min.y()),
            (after.max.y(), before.max.y()),
            (after.max.z(), before.max.z()),
            (after.min.z(), -distance),
        ] {
            assert!(near(a, b), "{label}: bounds {after:?} from {before:?}");
        }
        let change = solid_volume(topo, moved, DEFLECTION).unwrap() - volume;
        assert!(
            (change - expected.volume).abs() <= 1e-9 * expected.volume.abs().max(1.0),
            "{label}: volume changed by {change}, expected {}",
            expected.volume
        );
    }
    assert!(
        (solid_volume(topo, solid, DEFLECTION).unwrap() - volume).abs() <= 1e-9 * volume,
        "{label}: source changed"
    );
}

/// The equal-setback ring: a symmetric 0.35 chamfer, or a 45° draft.
#[test]
fn an_imported_equal_setback_rim_refuses_until_the_caller_reads_it() {
    const C: f64 = 0.35;
    let (mut topo, solid) = imported(bottom_rim, C, C);
    let (cap, wall) = assert_refuses(&mut topo, solid, "equal ring");
    assert!(near(cap, C) && near(wall, C), "setbacks {cap}/{wall}");

    // Read as a chamfer: the band translates, the walls lengthen.
    let cap_area = (2.0f64.mul_add(-C, X)) * (2.0f64.mul_add(-C, Y));
    for distance in [2.0, -2.0] {
        let expected = Expected {
            volume: distance * X * Y,
            bottom_area: cap_area,
        };
        assert_moves(
            &mut topo,
            solid,
            distance,
            AS_CHAMFER,
            &expected,
            "equal ring as chamfer",
        );
    }

    // Read as a draft: the band keeps its planes, the bottom is re-limited
    // against them. At height z the section is (X − 2C(1 − z/C)) by
    // (Y − 2C(1 − z/C)).
    let width = |z: f64, side: f64| 2.0f64.mul_add(-C * (1.0 - z / C), side);
    for distance in [2.0, -0.1] {
        let expected = Expected {
            volume: simpson(|z| width(z, X) * width(z, Y), -distance, 0.0),
            bottom_area: width(-distance, X) * width(-distance, Y),
        };
        assert_moves(
            &mut topo,
            solid,
            distance,
            AS_DRAFT,
            &expected,
            "equal ring as draft",
        );
    }
}

/// The unequal-setback band on one edge: a 0.3/0.6 chamfer, or a draft.
#[test]
fn an_imported_unequal_setback_rim_refuses_until_the_caller_reads_it() {
    let (mut topo, solid) = imported(front_bottom, 0.3, 0.6);
    let (c, w) = assert_refuses(&mut topo, solid, "unequal edge");
    let pair = [c.min(w), c.max(w)];
    assert!(near(pair[0], 0.3) && near(pair[1], 0.6), "setbacks {c}/{w}");

    for distance in [2.0, -2.0] {
        let expected = Expected {
            volume: distance * X * Y,
            bottom_area: X * (Y - c),
        };
        assert_moves(
            &mut topo,
            solid,
            distance,
            AS_CHAMFER,
            &expected,
            "unequal edge as chamfer",
        );
    }

    // As a draft the band's plane runs from (y = c, z = 0) to (y = 0, z = w):
    // at height z the bottom reaches y = c(1 − z/w).
    let depth = |z: f64| c.mul_add(-(1.0 - z / w), Y);
    for distance in [2.0, -0.1] {
        let expected = Expected {
            volume: X * simpson(depth, -distance, 0.0),
            bottom_area: X * depth(-distance),
        };
        assert_moves(
            &mut topo,
            solid,
            distance,
            AS_DRAFT,
            &expected,
            "unequal edge as draft",
        );
    }

    // The whole ring with unequal setbacks refuses too, and carries when read
    // as a chamfer.
    let (mut topo, solid) = imported(bottom_rim, 0.3, 0.6);
    assert_refuses(&mut topo, solid, "unequal ring");
    let cap_area = face_area(&topo, bottom(&topo, solid), DEFLECTION).unwrap();
    let expected = Expected {
        volume: -2.0 * X * Y,
        bottom_area: cap_area,
    };
    assert_moves(
        &mut topo,
        solid,
        -2.0,
        AS_CHAMFER,
        &expected,
        "unequal ring as chamfer",
    );
}
