//! Fillets on the edges an earlier planar chamfer left on a box.
//!
//! Chamfering a box's four top edges leaves a mitred corner at each end of
//! every chamfer: the vertex where the chamfer's lower contact line meets the
//! side wall is bounded by four planes — two walls and two chamfers, the
//! chamfers meeting along the diagonal mitre edge. A fillet of that lower
//! line must end on both the neighbouring wall (below the chamfer) and the
//! neighbouring chamfer (above it). The rolling-ball rebuild knew only a
//! single transverse end plane, so it emitted an open shell there and every
//! engine refused the edge at every radius.
//!
//! The fillet of a straight convex edge between two planes removes exactly
//! the solid's intersection with the infinite prism over the corner region
//! of its cross-section (the region between the two faces and the ball's
//! arc), because every face meeting its ends is convex there. So each
//! result is held to that prism's volume, integrated in closed form over its
//! cross-section: the chord of the solid along the edge direction is a
//! piecewise-linear function of the section point, integrated as area times
//! centroid on each linear piece (a 1-D antiderivative where the piece
//! boundary is not a line through the corner). None of this uses kernel
//! volume code; the kernel's measured volume is only compared against it.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::f64::consts::{FRAC_1_SQRT_2, PI, SQRT_2};

use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::{fillet_cascade, planar_chamfer_with_origins};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::validate_shell_closed;

/// The box of the recording.
const BOX: (f64, f64, f64) = (45.0, 35.0, 15.0);
/// The chamfer distance.
const D: f64 = 1.0;
/// Fine enough to sit below the measurement clamp (`bbox_diag * 5e-5`).
const MEASURE_DEFLECTION: f64 = 1e-4;
const RADII: [f64; 3] = [0.25, 0.5, 1.0];

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn find_edge(topo: &Topology, solid: SolidId, a: Point3, b: Point3) -> EdgeId {
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            ((start - a).length() < 1e-6 && (end - b).length() < 1e-6)
                || ((start - b).length() < 1e-6 && (end - a).length() < 1e-6)
        })
        .unwrap_or_else(|| panic!("edge {a:?} -- {b:?}"))
}

/// The box with its four top edges chamfered at [`D`] by the planar bevel.
fn chamfered(topo: &mut Topology) -> SolidId {
    let (x, y, z) = BOX;
    let b = make_box(topo, x, y, z).unwrap();
    let top = [
        find_edge(topo, b, p(0.0, 0.0, z), p(x, 0.0, z)),
        find_edge(topo, b, p(x, 0.0, z), p(x, y, z)),
        find_edge(topo, b, p(x, y, z), p(0.0, y, z)),
        find_edge(topo, b, p(0.0, y, z), p(0.0, 0.0, z)),
    ];
    planar_chamfer_with_origins(topo, b, &top, D).unwrap().0
}

/// The chamfered box's volume: the box less four edge prisms of section
/// `D²/2`, plus back the four corner pyramids (`D³/3` each) two prisms share.
fn chamfered_volume() -> f64 {
    let (x, y, z) = BOX;
    x * y * z - D * D / 2.0 * 2.0 * (x + y) + 4.0 * D * D * D / 3.0
}

/// Signed area and centroid of one piece of a section region.
#[derive(Clone, Copy)]
struct Piece {
    area: f64,
    centroid: Point3,
}

fn combine(pieces: &[Piece]) -> Piece {
    let area: f64 = pieces.iter().map(|piece| piece.area).sum();
    let moment = pieces.iter().fold(Vec3::new(0.0, 0.0, 0.0), |acc, piece| {
        acc + Vec3::new(piece.centroid.x(), piece.centroid.y(), piece.centroid.z()) * piece.area
    });
    Piece {
        area,
        centroid: p(moment.x() / area, moment.y() / area, moment.z() / area),
    }
}

/// The half of a rolling-ball corner region next to the face along `own`:
/// the right triangle corner–contact–centre less the circular sector from
/// that contact to the arc's midpoint. `own` and `other` are unit directions
/// from the sharp corner `corner` into the two faces, in one section plane.
fn half_corner(corner: Point3, own: Vec3, other: Vec3, radius: f64) -> Piece {
    let theta = own.dot(other).clamp(-1.0, 1.0).acos();
    let setback = radius / (theta / 2.0).tan();
    let bisector = (own + other).normalize().unwrap();
    let centre = corner + bisector * (radius / (theta / 2.0).sin());
    let contact = corner + own * setback;
    let triangle = Piece {
        area: radius * setback / 2.0,
        centroid: p(
            (corner.x() + contact.x() + centre.x()) / 3.0,
            (corner.y() + contact.y() + centre.y()) / 3.0,
            (corner.z() + contact.z() + centre.z()) / 3.0,
        ),
    };
    // The sector turns through half the arc, (π − θ)/2, from the contact
    // radius to the radius toward the corner; its centroid sits on its own
    // bisector at 4 r sin(φ/2) / (3 φ).
    let phi = (PI - theta) / 2.0;
    let to_contact = (contact - centre).normalize().unwrap();
    let to_corner = (corner - centre).normalize().unwrap();
    let direction = (to_contact + to_corner).normalize().unwrap();
    let sector = Piece {
        area: -radius * radius * phi / 2.0,
        centroid: centre + direction * (4.0 * radius * (phi / 2.0).sin() / (3.0 * phi)),
    };
    combine(&[triangle, sector])
}

/// Material removed by a fillet of a chamfer's lower contact line of
/// `length`, between the wall below and the chamfer above (interior angle
/// 135°): `∫_C ℓ dA`, `ℓ(z) = length − 2 max(0, z − z_line)`, because below
/// the line the side walls end the chord and above it the side chamfers do.
/// With `t = z − z_c` above the ball centre, the region's width is
/// `y_arc(t) − max(0, t − h)`, `y_arc = r − √(r² − t²)`, `h = r(√2 − 1)`
/// the line's height, up to the chamfer contact at `t = r/√2`.
fn lower_line_removed(radius: f64, length: f64) -> f64 {
    let r = radius;
    let h = r * (SQRT_2 - 1.0);
    let top = r * FRAC_1_SQRT_2;
    // ∫ √(r² − t²) dt and ∫ t √(r² − t²) dt.
    let f0 = |t: f64| 0.5 * r.mul_add(r * (t / r).asin(), t * (r * r - t * t).sqrt());
    let f1 = |t: f64| -(r * r - t * t).powf(1.5) / 3.0;
    // ∫ y_arc dt, ∫ (t − h) y_arc dt, ∫ (t − h) dt, ∫ (t − h)² dt.
    let arc = |t: f64| r * t - f0(t);
    let arc_moment = |t: f64| r * (t - h).powi(2) / 2.0 - f1(t) + h * f0(t);
    let ramp = |t: f64| (t - h).powi(2) / 2.0;
    let ramp_moment = |t: f64| (t - h).powi(3) / 3.0;
    let area = arc(top) - arc(0.0) - (ramp(top) - ramp(h));
    let above_line = arc_moment(top) - arc_moment(h) - (ramp_moment(top) - ramp_moment(h));
    length * area - 2.0 * above_line
}

/// Material removed by a fillet of the front chamfer's upper contact line
/// (top `z = BOX.2`, chamfer below it): the whole region lies above the
/// walls' reach, so the side chamfers end the chord at both ends and
/// `ℓ = BOX.0 − 2 (z − (BOX.2 − D))` is linear: area times `ℓ(centroid)`.
fn upper_line_removed(radius: f64) -> f64 {
    let corner = p(0.0, D, BOX.2);
    let along_top = Vec3::new(0.0, 1.0, 0.0);
    let down_chamfer = Vec3::new(0.0, -FRAC_1_SQRT_2, -FRAC_1_SQRT_2);
    let region = combine(&[
        half_corner(corner, along_top, down_chamfer, radius),
        half_corner(corner, down_chamfer, along_top, radius),
    ]);
    region.area * (BOX.0 - 2.0 * (region.centroid.z() - (BOX.2 - D)))
}

/// Material removed by a fillet of the front-left mitre, from
/// `(0, 0, BOX.2 − D)` to `(D, D, BOX.2)` along `m = (1, 1, 1)/√3`
/// (interior angle 120°): the top plane ends the chord above; below, the
/// front wall `y = 0` ends it on the front chamfer's half of the region and
/// the left wall `x = 0` on the left chamfer's (the halves meet on the
/// symmetry plane `x = y`, where the two bounds agree). Each half's chord is
/// linear: area times chord at its centroid.
fn mitre_removed(radius: f64) -> f64 {
    let m = Vec3::new(1.0, 1.0, 1.0).normalize().unwrap();
    let corner = p(0.0, 0.0, BOX.2 - D);
    // Unit directions from the mitre into each chamfer, perpendicular to m:
    // into the front chamfer is toward +x along it, into the left one +y.
    let into = |normal: Vec3, positive: fn(Vec3) -> f64| {
        let dir = m.cross(normal).normalize().unwrap();
        if positive(dir) < 0.0 { -dir } else { dir }
    };
    let front = into(Vec3::new(0.0, -1.0, 1.0), Vec3::x);
    let left = into(Vec3::new(-1.0, 0.0, 1.0), Vec3::y);
    let to_top = |q: Point3| (BOX.2 - q.z()) / m.z();
    let past_front_wall = |q: Point3| -q.y() / m.y();
    let past_left_wall = |q: Point3| -q.x() / m.x();
    let front_half = half_corner(corner, front, left, radius);
    let left_half = half_corner(corner, left, front, radius);
    // Each half lies wholly on its side of `x = y`, so its centroid does.
    assert!(past_front_wall(front_half.centroid) > past_left_wall(front_half.centroid));
    assert!(past_left_wall(left_half.centroid) > past_front_wall(left_half.centroid));
    front_half.area * (to_top(front_half.centroid) - past_front_wall(front_half.centroid))
        + left_half.area * (to_top(left_half.centroid) - past_left_wall(left_half.centroid))
}

/// Fillet the edge `a`–`b` of the chamfered box and hold the result to the
/// closed-form `removed` volume (within `relative` of it), strict validity, a
/// closed shell and one exact cylinder band.
fn check(label: &str, a: Point3, b: Point3, radius: f64, removed: f64, relative: f64) {
    let mut topo = Topology::new();
    let solid = chamfered(&mut topo);
    let input = solid_volume(&topo, solid, MEASURE_DEFLECTION).unwrap();
    assert!(
        (input - chamfered_volume()).abs() < 1e-9 * chamfered_volume(),
        "chamfered box {input} against {}",
        chamfered_volume()
    );
    let before = solid_entity_counts(&topo, solid).unwrap();
    let edge = find_edge(&topo, solid, a, b);
    let result = fillet_cascade(&mut topo, solid, &[edge], radius)
        .unwrap_or_else(|error| panic!("{label} r={radius}: {error}"));
    let filleted = result.solid;

    let shell = topo
        .shell(topo.solid(filleted).unwrap().outer_shell())
        .unwrap();
    assert!(
        validate_shell_closed(shell, &topo).is_ok(),
        "{label} r={radius}: open shell"
    );
    let report = validate_solid(&topo, filleted).unwrap();
    assert!(report.is_valid(), "{label} r={radius}: {:?}", report.issues);
    let faces = solid_faces(&topo, filleted).unwrap();
    // One band and nothing else new: no closing facet stands in for the
    // band's run-out onto a neighbouring face.
    assert_eq!(faces.len(), before.0 + 1, "{label} r={radius}: face count");
    let bands: Vec<f64> = faces
        .iter()
        .filter_map(|&face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder) => Some(cylinder.radius()),
            _ => None,
        })
        .collect();
    assert_eq!(bands.len(), 1, "{label} r={radius}: cylinder bands");
    assert!(
        (bands[0] - radius).abs() < 1e-9,
        "{label} r={radius}: band radius {}",
        bands[0]
    );

    let expected = chamfered_volume() - removed;
    let volume = solid_volume(&topo, filleted, MEASURE_DEFLECTION).unwrap();
    // Held relative to the material removed, which is all the fillet decides.
    assert!(
        (volume - expected).abs() <= removed * relative,
        "{label} r={radius}: removed {}, closed form {removed}",
        chamfered_volume() - volume
    );
}

#[test]
fn lower_chamfer_line_fillet_ends_on_wall_and_chamfer() {
    let z = BOX.2 - D;
    for radius in RADII {
        check(
            "lower front",
            p(0.0, 0.0, z),
            p(BOX.0, 0.0, z),
            radius,
            lower_line_removed(radius, BOX.0),
            1e-4,
        );
    }
}

#[test]
fn lower_chamfer_line_fillet_on_the_short_side() {
    // The same corner cells with the stripe along y, which also flips which
    // side face ends which contact relative to the edge's face order.
    let z = BOX.2 - D;
    for radius in RADII {
        check(
            "lower left",
            p(0.0, 0.0, z),
            p(0.0, BOX.1, z),
            radius,
            lower_line_removed(radius, BOX.1),
            1e-4,
        );
    }
}

#[test]
fn upper_chamfer_line_fillet_ends_on_neighbour_chamfers() {
    for radius in RADII {
        check(
            "upper front",
            p(D, D, BOX.2),
            p(BOX.0 - D, D, BOX.2),
            radius,
            upper_line_removed(radius),
            1e-4,
        );
    }
}

#[test]
fn mitre_fillet_ends_on_both_walls_and_the_top() {
    for radius in [0.1, 0.25, 0.5] {
        check(
            "mitre",
            p(0.0, 0.0, BOX.2 - D),
            p(D, D, BOX.2),
            radius,
            mitre_removed(radius),
            // `solid_volume` measures this oblique, ellipse-trimmed band on
            // its per-face route about 6.8e-4 of the removed volume low at
            // every radius, while the whole-solid mesh converges onto the
            // closed form as its deflection shrinks (3.0e-3, 9.4e-4, 3.1e-4
            // high at 1e-3, 3e-4, 1e-4 for r = 0.5). The geometry is exact;
            // the reading is not. Wrong ends miss by percents.
            1e-3,
        );
    }
}
