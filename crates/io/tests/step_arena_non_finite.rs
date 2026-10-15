//! IO-8 (kernel review 2026-10-14): non-finite values crossed two exchange
//! boundaries silently.
//!
//! - The STEP reader checked every real for finiteness as it parsed it, then
//!   multiplied lengths by the file's unit scale. A `.METRE.` file states
//!   lengths in metres and remus works in millimetres, so a sphere of radius
//!   `1E306` imported Ok with radius ∞. Lengths are now checked again after
//!   scaling.
//! - The arena writer formatted a non-finite float the way `serde_json` does,
//!   as `null`, which the arena reader refuses: serialization succeeded and
//!   produced a document nothing could read back. The writer now refuses it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::IoError;
use remus_io::arena_io::{deserialize_solids, serialize_solids};
use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::vec::Point3;
use remus_operations::primitives::{make_box, make_sphere};
use remus_topology::Topology;
use remus_topology::explorer;

/// A remus-written STEP file re-declared in metres. The writer states
/// millimetres, so dropping the `.MILLI.` prefix scales every length by 1000.
fn metre_step(build: impl FnOnce(&mut Topology) -> remus_topology::solid::SolidId) -> String {
    let mut topo = Topology::new();
    let solid = build(&mut topo);
    let text = write_step(&topo, &[solid]).unwrap();
    assert_eq!(
        text.matches("SI_UNIT(.MILLI.,.METRE.)").count(),
        1,
        "{text}"
    );
    let text = text.replace("SI_UNIT(.MILLI.,.METRE.)", "SI_UNIT($,.METRE.)");
    // Control: the metre file imports at 1000× scale.
    let mut check = Topology::new();
    read_step(&text, &mut check).unwrap();
    text
}

fn replace_one(text: &str, from: &str, to: &str) -> String {
    assert_eq!(text.matches(from).count(), 1, "`{from}` must occur once");
    text.replacen(from, to, 1)
}

#[track_caller]
fn assert_refused_as_non_finite(text: &str, what: &str) {
    let mut topo = Topology::new();
    let error = read_step(text, &mut topo)
        .err()
        .unwrap_or_else(|| panic!("{what} imported"));
    assert!(
        error.to_string().contains("non-finite"),
        "{what} was refused for another reason: {error}"
    );
    assert_eq!(
        topo.num_faces(),
        0,
        "{what}: a refused import left faces behind"
    );
}

#[test]
fn metre_sphere_radius_overflowing_millimetres_is_refused() {
    let text = metre_step(|topo| make_sphere(topo, 2.0, 8).unwrap());
    let line = text
        .lines()
        .find(|line| line.contains("SPHERICAL_SURFACE("))
        .unwrap()
        .to_owned();
    // 1E306 is finite; 1E306 m is 1E309 mm, which is not.
    let (head, _radius) = line.rsplit_once(',').unwrap();
    let text = replace_one(&text, &line, &format!("{head}, 1.0E306);"));
    assert_refused_as_non_finite(&text, "a 1E306 m sphere radius");
}

#[test]
fn metre_point_overflowing_millimetres_is_refused() {
    let text = metre_step(|topo| make_box(topo, 1.0, 1.0, 1.0).unwrap());
    let line = text
        .lines()
        .find(|line| line.contains("CARTESIAN_POINT(") && line.contains("(0., 0., 0.)"))
        .unwrap()
        .to_owned();
    let poisoned = line.replacen("(0.,", "(1.0E306,", 1);
    let text = replace_one(&text, &line, &poisoned);
    assert_refused_as_non_finite(&text, "a 1E306 m coordinate");
}

/// Each input real can be finite after scaling while a quantity derived from
/// several of them is not: here a plane's offset `n·origin`, with every
/// coordinate 1.7E308 mm and an unnormalized `(1, 1, 1)` normal.
#[test]
fn metre_plane_whose_offset_overflows_is_refused() {
    let text = metre_step(|topo| make_box(topo, 1.0, 1.0, 1.0).unwrap());
    let plane = text
        .lines()
        .find(|line| line.contains("PLANE("))
        .unwrap()
        .to_owned();
    let (head, _placement) = plane.rsplit_once('#').unwrap();
    let mut text = replace_one(&text, &plane, &format!("{head}#999901);"));
    let data_end =
        text.find("DATA;").unwrap() + text[text.find("DATA;").unwrap()..].find("ENDSEC;").unwrap();
    text.insert_str(
        data_end,
        "#999901 = AXIS2_PLACEMENT_3D('', #999902, #999903, $);\n\
         #999902 = CARTESIAN_POINT('', (1.7E305, 1.7E305, 1.7E305));\n\
         #999903 = DIRECTION('', (1., 1., 1.));\n",
    );
    assert_refused_as_non_finite(&text, "a plane offset of 5.1E308 mm");
}

/// The writer no longer emits a document its own reader refuses.
#[test]
fn arena_writer_refuses_non_finite_geometry() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let vertex = explorer::solid_vertices(&topo, solid).unwrap()[0];
    let finite = serialize_solids(&topo, &[solid]).unwrap();
    deserialize_solids(&finite, &mut Topology::new()).unwrap();

    let at = topo.vertex(vertex).unwrap().point();
    topo.vertex_mut(vertex)
        .unwrap()
        .set_point(Point3::new(f64::INFINITY, at.y(), at.z()));
    let Err(error) = serialize_solids(&topo, &[solid]) else {
        panic!("a document with an infinite coordinate serialized");
    };
    assert!(
        matches!(error, IoError::InvalidTopology { .. })
            && error.to_string().contains("non-finite"),
        "{error}"
    );
}

/// A spent byte budget is reported as such, whichever write crossed it,
/// including the non-finite one.
#[test]
fn arena_byte_budget_wins_over_non_finite_refusal() {
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
    let finite_len = serialize_solids(&topo, &[solid]).unwrap().len();
    let vertex = explorer::solid_vertices(&topo, solid).unwrap()[0];
    let at = topo.vertex(vertex).unwrap().point();
    topo.vertex_mut(vertex)
        .unwrap()
        .set_point(Point3::new(f64::INFINITY, at.y(), at.z()));
    // `null` is never shorter than the finite coordinate it replaces, so every
    // budget below the finite length is exceeded.
    for limit in 0..finite_len {
        let limits = remus_io::ImportLimits {
            max_input_bytes: limit,
            ..remus_io::ImportLimits::default()
        };
        let Err(error) = remus_io::arena_io::serialize_solids_with_limits(&topo, &[solid], limits)
        else {
            panic!("a {limit}-byte budget accepted the document");
        };
        assert!(
            matches!(error, IoError::LimitExceeded { .. }),
            "budget {limit}: {error}"
        );
    }
}
