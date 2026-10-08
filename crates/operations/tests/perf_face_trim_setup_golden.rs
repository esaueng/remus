//! Bit pins for the face integrator's trim setup.
//!
//! Every curved face the integrator measures is outlined in UV (its outer
//! wire sampled and projected, its holes likewise), indexed for the
//! per-abscissa crossing and winding queries, and cut into v-spans at every
//! Gauss abscissa. Making that setup cheaper — sampling and projecting the
//! outer wire once, storing the trim index flat, reusing the span scratch —
//! may not move a single bit of any integral. These tests pin, per face, the
//! fixed rule at orders 1, 5 and 8 (every [`FaceContribution`] field, through
//! a digest of their bits), the area-only rule, and the option API at its
//! default and at a refining tolerance (the sliced adaptive path), plus the
//! solid-level volume and area, on booleans whose curved faces exercise each
//! surface arm: cylinder, cone, sphere, torus and NURBS walls, holes that
//! close and holes that wrap.
//!
//! The goldens were generated from the code before those changes and must
//! not be regenerated to make a performance change pass. On x86-64 Linux,
//! where they were generated and where CI runs, every number must match bit
//! for bit and every digest exactly. Elsewhere the platform `libm` may round
//! a transcendental differently in the last place, so numbers compare to
//! 1e-12 relative and digests are skipped.
//!
//! Regenerate a golden only for an intentional semantic change, with
//! `UPDATE_GOLDEN=1`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use remus_check::properties::PropertiesOptions;
use remus_check::properties::face_integrator::{
    FaceContribution, integrate_face_area, integrate_face_fixed_about, integrate_face_with_options,
};
use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::heal::convert_to_bspline;
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

/// Whether this target is the one the goldens were generated on, so numbers
/// and digests must match bit for bit.
const EXACT: bool = cfg!(all(target_os = "linux", target_arch = "x86_64"));

fn golden_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/golden/data/face_trim_setup")
        .join(name)
}

/// Whether two golden tokens agree: keys exactly, numbers bit for bit (or to
/// 1e-12 relative off the generating platform), digests exactly (or not at
/// all off it), anything else exactly.
fn tokens_agree(expected: &str, actual: &str) -> bool {
    match (expected.split_once('='), actual.split_once('=')) {
        (Some((ek, ev)), Some((ak, av))) => {
            if ek != ak {
                return false;
            }
            if ek == "digest" {
                return !EXACT || ev == av;
            }
            match (ev.parse::<f64>(), av.parse::<f64>()) {
                (Ok(e), Ok(a)) if EXACT => e.to_bits() == a.to_bits(),
                (Ok(e), Ok(a)) => {
                    let scale = 1.0_f64.max(e.abs()).max(a.abs());
                    (e - a).abs() <= 1e-12 * scale
                }
                _ => ev == av,
            }
        }
        (None, None) => expected == actual,
        _ => false,
    }
}

fn assert_golden(name: &str, actual: &str) {
    let path = golden_path(name);
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, actual).unwrap();
        return;
    }
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|_| {
        panic!(
            "golden file not found: {}\nRun with UPDATE_GOLDEN=1 to create it.",
            path.display()
        )
    });
    let expected = expected.replace("\r\n", "\n");
    let expected_lines: Vec<&str> = expected.trim().lines().collect();
    let actual_lines: Vec<&str> = actual.trim().lines().collect();
    assert_eq!(
        expected_lines.len(),
        actual_lines.len(),
        "{name}: line count differs\n--- actual ---\n{actual}"
    );
    for (index, (e, a)) in expected_lines.iter().zip(&actual_lines).enumerate() {
        let et: Vec<&str> = e.split_whitespace().collect();
        let at: Vec<&str> = a.split_whitespace().collect();
        assert!(
            et.len() == at.len() && et.iter().zip(&at).all(|(x, y)| tokens_agree(x, y)),
            "{name}: line {} differs\n  expected: {e}\n  actual:   {a}",
            index + 1
        );
    }
}

/// FNV-1a over the bits of every field, in declaration order.
fn digest(c: &FaceContribution) -> String {
    let fields = [
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
    ];
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for field in fields {
        for byte in field.to_bits().to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0100_0000_01b3);
        }
    }
    format!("{hash:016x}")
}

fn contribution(result: Result<FaceContribution, remus_check::CheckError>) -> String {
    match result {
        Ok(c) => format!(
            "area={:?} volume={:?} digest={}",
            c.area,
            c.volume,
            digest(&c)
        ),
        Err(e) => format!("err={}", format!("{e}").replace(char::is_whitespace, "_")),
    }
}

/// One face through each fixed order, the area-only rule and the option
/// API.
fn render_face(topo: &Topology, face: FaceId) -> String {
    let defaults = PropertiesOptions::default();
    let refined = PropertiesOptions {
        adaptive_eps: 1e-8,
        ..PropertiesOptions::default()
    };
    let mut out = String::new();
    let kind = topo.face(face).unwrap().surface().type_tag();
    for order in [1, 5, 8] {
        let fixed = integrate_face_fixed_about(topo, face, order, Point3::new(0.0, 0.0, 0.0));
        let area = match integrate_face_area(topo, face, order) {
            Ok(a) => format!("{a:?}"),
            Err(e) => format!("err:{}", format!("{e}").replace(char::is_whitespace, "_")),
        };
        writeln!(
            out,
            "face={} kind={kind} order={order} area_only={area} {}",
            face.index(),
            contribution(fixed)
        )
        .unwrap();
    }
    for (label, options) in [("default", &defaults), ("refined", &refined)] {
        writeln!(
            out,
            "face={} kind={kind} options={label} {}",
            face.index(),
            contribution(integrate_face_with_options(topo, face, options))
        )
        .unwrap();
    }
    out
}

/// Every face of `solid`, in shell order, through [`render_face`]; then the
/// solid's totals.
fn render(topo: &Topology, solid: SolidId) -> String {
    let defaults = PropertiesOptions::default();
    let mut out = String::new();
    let faces = solid_faces(topo, solid).unwrap();
    writeln!(out, "faces={}", faces.len()).unwrap();
    for &face in &faces {
        out.push_str(&render_face(topo, face));
    }
    let volume = remus_check::properties::solid_volume(topo, solid, &defaults).unwrap();
    let area = remus_check::properties::solid_area(topo, solid, &defaults).unwrap();
    let measured = remus_operations::measure::solid_volume(topo, solid, 0.01).unwrap();
    writeln!(
        out,
        "solid volume={volume:?} area={area:?} measured_volume={measured:?}"
    )
    .unwrap();
    out
}

fn translated(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) -> SolidId {
    transform_solid(topo, solid, &Mat4::translation(x, y, z)).unwrap();
    solid
}

fn box_minus_through_cylinder(topo: &mut Topology) -> SolidId {
    let block = make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let tool = make_cylinder(topo, 2.0, 20.0).unwrap();
    let tool = translated(topo, tool, 5.0, 5.0, -5.0);
    boolean(topo, BooleanOp::Cut, block, tool).unwrap()
}

#[test]
fn box_minus_through_cylinder_integrals_are_pinned() {
    let mut topo = Topology::new();
    let solid = box_minus_through_cylinder(&mut topo);
    assert_golden("box_minus_through_cylinder.golden", &render(&topo, solid));
}

/// The bore's two rims wrap the shaft wall, so it trims by bands.
#[test]
fn cross_drilled_shaft_integrals_are_pinned() {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, 3.0, 12.0).unwrap();
    let tool = make_cylinder(&mut topo, 1.0, 10.0).unwrap();
    transform_solid(
        &mut topo,
        tool,
        &Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
    )
    .unwrap();
    let tool = translated(&mut topo, tool, -5.0, 0.0, 6.0);
    let solid = boolean(&mut topo, BooleanOp::Cut, shaft, tool).unwrap();
    assert_golden("cross_drilled_shaft.golden", &render(&topo, solid));
}

/// A frustum with an off-axis bore through its lateral wall.
#[test]
fn drilled_cone_frustum_integrals_are_pinned() {
    let mut topo = Topology::new();
    let frustum = make_cone(&mut topo, 4.0, 2.0, 6.0).unwrap();
    let tool = make_cylinder(&mut topo, 0.8, 12.0).unwrap();
    transform_solid(
        &mut topo,
        tool,
        &Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
    )
    .unwrap();
    let tool = translated(&mut topo, tool, -6.0, 0.0, 3.0);
    let solid = boolean(&mut topo, BooleanOp::Cut, frustum, tool).unwrap();
    assert_golden("drilled_cone_frustum.golden", &render(&topo, solid));
}

/// A tunnel along the axis leaves each hemisphere a band ending at its rim.
#[test]
fn drilled_sphere_band_integrals_are_pinned() {
    let mut topo = Topology::new();
    let ball = make_sphere(&mut topo, 5.0, 16).unwrap();
    let tool = make_cylinder(&mut topo, 1.5, 20.0).unwrap();
    let tool = translated(&mut topo, tool, 0.0, 0.0, -10.0);
    let solid = boolean(&mut topo, BooleanOp::Cut, ball, tool).unwrap();
    assert_golden("drilled_sphere_band.golden", &render(&topo, solid));
}

/// The minor arc of `circle` from `a` to `b`, as an edge walked from `a`.
fn minor_arc(topo: &mut Topology, circle: &Circle3D, a: VertexId, b: VertexId) -> OrientedEdge {
    let (pa, pb) = (
        topo.vertex(a).unwrap().point(),
        topo.vertex(b).unwrap().point(),
    );
    let (ta, tb) = (circle.project(pa), circle.project(pb));
    let sweep = (tb - ta).rem_euclid(TAU);
    let (start, end, t0, t1, forward) = if sweep <= PI {
        (a, b, ta, ta + sweep, true)
    } else {
        (b, a, tb, tb + (TAU - sweep), false)
    };
    let mut edge = Edge::new(start, end, EdgeCurve::Circle(circle.clone()));
    edge.set_trim(Some((t0, t1)));
    OrientedEdge::new(topo.add_edge(edge), forward)
}

/// A torus patch between two latitude arcs and two meridian arcs that
/// straddles both seams. Built by hand: a box cutting a torus leaves the
/// exact path, so no boolean keeps a trimmed torus carrier to measure.
fn torus_patch_face(topo: &mut Topology, torus: &ToroidalSurface) -> FaceId {
    let (big, small) = (torus.major_radius(), torus.minor_radius());
    let (u0, u1, v0, v1) = (TAU - 0.4, TAU + 0.5, -0.5, 0.7);
    let corner = |u: f64, v: f64| {
        let radial = big + small * v.cos();
        Point3::new(radial * u.cos(), radial * u.sin(), small * v.sin())
    };
    let latitude = |v: f64| {
        Circle3D::new(
            Point3::new(0.0, 0.0, small * v.sin()),
            Vec3::new(0.0, 0.0, 1.0),
            big + small * v.cos(),
        )
        .unwrap()
    };
    let meridian = |u: f64| {
        Circle3D::new_with_ref(
            Point3::new(big * u.cos(), big * u.sin(), 0.0),
            Vec3::new(-u.sin(), u.cos(), 0.0),
            small,
            Vec3::new(u.cos(), u.sin(), 0.0),
        )
        .unwrap()
    };
    let vertices: Vec<VertexId> = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        .into_iter()
        .map(|(u, v)| topo.add_vertex(Vertex::new(corner(u, v), 1e-7)))
        .collect();
    let edges = vec![
        minor_arc(topo, &latitude(v0), vertices[0], vertices[1]),
        minor_arc(topo, &meridian(u1), vertices[1], vertices[2]),
        minor_arc(topo, &latitude(v1), vertices[2], vertices[3]),
        minor_arc(topo, &meridian(u0), vertices[3], vertices[0]),
    ];
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(torus.clone())))
}

/// A whole torus, and a hand-built patch of one trimmed by latitude and
/// meridian arcs across both seams.
#[test]
fn torus_patch_integrals_are_pinned() {
    let mut topo = Topology::new();
    let ring = make_torus(&mut topo, 10.0, 3.0, 16).unwrap();
    let mut rendered = render(&topo, ring);
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 3.0).unwrap();
    let patch = torus_patch_face(&mut topo, &torus);
    rendered.push_str(&render_face(&topo, patch));
    assert_golden("torus_patch.golden", &rendered);
}

/// The bored block with every surface and curve converted to B-splines:
/// NURBS faces bounded by NURBS rims take the sampled-outline path.
#[test]
fn nurbs_bounded_faces_integrals_are_pinned() {
    let mut topo = Topology::new();
    let solid = box_minus_through_cylinder(&mut topo);
    assert!(convert_to_bspline(&mut topo, solid).unwrap() > 0);
    assert_golden("nurbs_bounded_faces.golden", &render(&topo, solid));
}

/// `sequential_cylinder_cuts` at N = 16: a plate bored on a grid.
#[test]
fn sequential_cylinder_cuts_integrals_are_pinned() {
    let mut topo = Topology::new();
    let mut result = make_box(&mut topo, 100.0, 100.0, 10.0).unwrap();
    let (cols, rows) = (4_u32, 4_u32);
    let x_spacing = 100.0 / f64::from(cols + 1);
    let y_spacing = 100.0 / f64::from(rows + 1);
    for i in 0..cols * rows {
        let x = x_spacing * f64::from(i % cols + 1);
        let y = y_spacing * f64::from(i / cols + 1);
        let tool = make_cylinder(&mut topo, 2.0, 20.0).unwrap();
        let tool = translated(&mut topo, tool, x, y, -5.0);
        result = boolean(&mut topo, BooleanOp::Cut, result, tool).unwrap();
    }
    assert_golden("sequential_cylinder_cuts_16.golden", &render(&topo, result));
}

/// One gridfinity baseplate unit with its four blind magnet holes.
#[test]
fn gridfinity_unit_integrals_are_pinned() {
    let mut topo = Topology::new();
    let mut result = make_box(&mut topo, 42.0, 42.0, 4.65).unwrap();
    for (x, y) in [(4.0, 4.0), (38.0, 4.0), (4.0, 38.0), (38.0, 38.0)] {
        let tool = make_cylinder(&mut topo, 3.0, 5.0).unwrap();
        let tool = translated(&mut topo, tool, x, y, -0.5);
        result = boolean(&mut topo, BooleanOp::Cut, result, tool).unwrap();
    }
    assert_golden("gridfinity_unit.golden", &render(&topo, result));
}
