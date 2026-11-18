//! Tight, tilted turns that the separating-plane certificate clears, and
//! the ones that truly meet themselves, against an independent oracle.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI};

use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::{make_circle_edge, make_line_edge};
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{OperationsError, sweep_exact};

const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

/// Turn `v` by `angle` about +Z.
fn turn_z(v: Vec3, angle: f64) -> Vec3 {
    let (s, c) = angle.sin_cos();
    Vec3::new(c * v.x() - s * v.y(), s * v.x() + c * v.y(), v.z())
}

/// The family: a circle of radius `s` tilted by `tilt` from square to the
/// path, swept along a run of `10 s` on +X, then a counter-clockwise turn of
/// `turn` and radius `radius · s` about `(10, radius, 0) s`, then (when
/// `back`) a run of `10 s` along the turn's end tangent.
#[derive(Debug, Clone, Copy)]
struct Case {
    tilt: f64,
    radius: f64,
    turn: f64,
    back: bool,
}

impl Case {
    fn build(&self, topo: &mut Topology, s: f64) -> Result<SolidId, OperationsError> {
        let (profile, path) = self.inputs(topo, s);
        sweep_exact(topo, profile, &path)
    }

    /// The profile face and the path edges.
    fn inputs(&self, topo: &mut Topology, s: f64) -> (FaceId, Vec<EdgeId>) {
        let normal = X * self.tilt.cos() + Y * self.tilt.sin();
        let edge = make_circle_edge(topo, p(0.0, 0.0, 0.0), normal, s, 1e-7).unwrap();
        let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap());
        let profile = topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane { normal, d: 0.0 },
        ));
        let centre = p(10.0 * s, self.radius * s, 0.0);
        let circle = Circle3D::with_axes(centre, Z, self.radius * s, X, Y).unwrap();
        let (a, b) = (-FRAC_PI_2, -FRAC_PI_2 + self.turn);
        let start = topo.add_vertex(Vertex::new(circle.evaluate(a), 1e-7));
        let end_point = circle.evaluate(b);
        let end = topo.add_vertex(Vertex::new(end_point, 1e-7));
        let mut arc = Edge::new(start, end, EdgeCurve::Circle(circle));
        arc.set_trim(Some((a, b)));
        let mut path = vec![
            make_line_edge(topo, p(0.0, 0.0, 0.0), p(10.0 * s, 0.0, 0.0), 1e-7).unwrap(),
            topo.add_edge(arc),
        ];
        if self.back {
            let tangent = Vec3::new(-b.sin(), b.cos(), 0.0);
            path.push(
                make_line_edge(topo, end_point, end_point + tangent * (10.0 * s), 1e-7).unwrap(),
            );
        }
        (profile, path)
    }

    /// The flux of the motion through the section: `π cos(tilt)` times the
    /// centre line's length, at unit scale.
    fn volume(&self) -> f64 {
        let length = self
            .radius
            .mul_add(self.turn, if self.back { 20.0 } else { 10.0 });
        PI * self.tilt.cos() * length
    }

    /// The oracle's model of the body at unit scale: its sections (unit
    /// discs) at path arc-length spacing at most `step`, each with its
    /// arc-length along the path.
    fn sections(&self, step: f64) -> Vec<(f64, Point3, Vec3)> {
        let n0 = X * self.tilt.cos() + Y * self.tilt.sin();
        let o = p(10.0, self.radius, 0.0);
        let mut out = Vec::new();
        let count = |length: f64| (length / step).ceil().max(1.0) as usize;
        let k = count(10.0);
        for i in 0..=k {
            let t = 10.0 * i as f64 / k as f64;
            out.push((t, p(t, 0.0, 0.0), n0));
        }
        let arc_length = self.radius * self.turn;
        let k = count(arc_length);
        for i in 1..=k {
            let alpha = self.turn * i as f64 / k as f64;
            let centre = o + turn_z(p(10.0, 0.0, 0.0) - o, alpha);
            out.push((10.0 + self.radius * alpha, centre, turn_z(n0, alpha)));
        }
        if self.back {
            let e = o + turn_z(p(10.0, 0.0, 0.0) - o, self.turn);
            let (d, n) = (turn_z(X, self.turn), turn_z(n0, self.turn));
            let k = count(10.0);
            for i in 1..=k {
                let t = 10.0 * i as f64 / k as f64;
                out.push((10.0 + arc_length + t, e + d * t, n));
            }
        }
        out
    }

    /// The independent oracle: a pair of distinct sections that share a
    /// point (unit discs, intersected exactly along their planes' common
    /// line) is a witness that the body meets itself. Sections closer than
    /// `0.5` along the path are skipped: a transversal profile's nearby
    /// sections never meet, and the two copies of a joint's section are one.
    fn witness(&self) -> Option<(f64, f64)> {
        let sections = self.sections(0.05);
        for (i, &(ta, ca, na)) in sections.iter().enumerate() {
            for &(tb, cb, nb) in &sections[i + 1..] {
                if tb - ta > 0.5 && discs_meet(ca, na, cb, nb) {
                    return Some((ta, tb));
                }
            }
        }
        None
    }
}

/// Whether two unit discs (centre, unit normal) share a point.
fn discs_meet(c1: Point3, n1: Vec3, c2: Point3, n2: Vec3) -> bool {
    let u = n1.cross(n2);
    let uu = u.dot(u);
    if uu < 1e-24 {
        // Parallel planes: only coplanar discs closer than a diameter meet.
        return n1.dot(c2 - c1).abs() < 1e-12 && (c2 - c1).length() <= 2.0;
    }
    let (h1, h2) = (
        n1.dot(Vec3::new(c1.x(), c1.y(), c1.z())),
        n2.dot(Vec3::new(c2.x(), c2.y(), c2.z())),
    );
    let k = n1.dot(n2);
    let base = (n1 * (h1 - h2 * k) + n2 * (h2 - h1 * k)) * (1.0 / uu);
    let origin = p(base.x(), base.y(), base.z());
    let dir = u * (1.0 / uu.sqrt());
    // Each disc cuts the common line in an interval about the foot of its
    // centre.
    let interval = |c: Point3| -> Option<(f64, f64)> {
        let t = (c - origin).dot(dir);
        let foot = origin + dir * t;
        let d = (c - foot).length();
        (d <= 1.0).then(|| {
            let half = (1.0 - d * d).sqrt();
            (t - half, t + half)
        })
    };
    match (interval(c1), interval(c2)) {
        (Some((a0, a1)), Some((b0, b1))) => a0 <= b1 && b0 <= a1,
        _ => false,
    }
}

fn rel(measured: f64, exact: f64) -> f64 {
    (measured - exact).abs() / exact.abs()
}

fn mesh_volume(mesh: &crate::tessellate::TriangleMesh) -> f64 {
    let o = p(0.0, 0.0, 0.0);
    let mut six = 0.0;
    for t in mesh.indices.chunks(3) {
        let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize]);
        six += (a - o).dot((b - o).cross(c - o));
    }
    six / 6.0
}

fn assert_probe(topo: &Topology, solid: SolidId, at: Point3, inside: bool, s: f64, label: &str) {
    let found = crate::classify::classify_point(topo, solid, at, 0.01 * s, 1e-9 * s).unwrap();
    let expected = if inside {
        crate::classify::PointClassification::Inside
    } else {
        crate::classify::PointClassification::Outside
    };
    assert_eq!(found, expected, "{label}: probe {at:?}");
}

/// The smallest turn radius at which the tilted unit circle still crosses
/// every arc transversally: a section point at in-plane offset `r` moves
/// through the section's plane at speed `R cos(tilt) − r`, so the fold sets
/// in at `R = 1 / cos(tilt)`. Below it the sweep is refused by the
/// transversality check, independently of the clearance certificate.
fn fold_radius(tilt: f64) -> f64 {
    1.0 / tilt.cos()
}

/// Tight, tilted turns the 2×reach distance check refused although they are
/// clean (#1860's declined review finding): the half turn of radius 1.2 at a
/// 30° tilt with a return run (built on main before #1860, volume 64.67),
/// and neighbours at other tilts down to within a few hundredths of the
/// fold radius; the 216° turn of radius 2 at 45°, which the distance check
/// already clears on this path, is a control. Each now builds valid,
/// watertight, with the flux volume `π cos(tilt) · length` (a tilted
/// section sweeps its area times `cos(tilt)` per unit of centre line), and
/// the independent oracle finds no two sections that meet.
#[test]
fn tight_tilted_turns_clear_by_a_separating_plane() {
    let cases = [
        (30.0, 1.2, PI, true),
        (-30.0, 1.2, PI, true),
        (15.0, 1.06, PI, true),
        (45.0, 1.52, PI, true),
        (-45.0, 1.52, PI, true),
        (60.0, 2.1, PI, true),
        (45.0, 2.0, 1.2 * PI, false),
        (30.0, 1.17, 1.2 * PI, false),
        (45.0, 1.43, 1.5 * PI, false),
        (60.0, 2.01, 1.5 * PI, false),
    ];
    for (degrees, radius, turn, back) in cases {
        let case = Case {
            tilt: f64::to_radians(degrees),
            radius,
            turn,
            back,
        };
        assert!(
            case.witness().is_none(),
            "{case:?}: the oracle finds the body meeting itself"
        );
        for s in [1e-3, 1.0, 1e3] {
            let label = format!("{case:?} at scale {s}");
            let mut topo = Topology::new();
            let solid = case
                .build(&mut topo, s)
                .unwrap_or_else(|e| panic!("{label}: {e:?}"));
            let report = crate::validate::validate_solid(&topo, solid).unwrap();
            assert!(report.is_valid(), "{label}: {report:?}");
            let exact = case.volume() * s * s * s;
            let measured = crate::measure::mass_properties(&topo, solid).unwrap().mass;
            // The NURBS integrator's absolute floor at 1e-3, as in
            // `oblique_profiles_into_a_turn_build_exactly`.
            let bound = if s < 1.0 { 1e-5 } else { 1e-8 };
            assert!(
                rel(measured, exact) < bound,
                "{label}: {measured} vs {exact}"
            );
            let mesh = crate::tessellate::tessellate_solid(&topo, solid, 0.02 * s).unwrap();
            assert!(crate::tessellate::is_watertight(&mesh), "{label}");
            let enclosed = mesh_volume(&mesh);
            assert!(
                rel(enclosed, exact) < 1e-2,
                "{label}: mesh {enclosed} vs {exact}"
            );
            let middle = -FRAC_PI_2 + 0.5 * turn;
            let mut probes = vec![
                (p(5.0, 0.0, 0.0), true),
                (
                    p(
                        radius.mul_add(middle.cos(), 10.0),
                        radius.mul_add(middle.sin(), radius),
                        0.0,
                    ),
                    true,
                ),
                // The turn's centre: the section's nearest point stays
                // `radius − cos(tilt)` from the axis.
                (p(10.0, radius, 0.0), false),
                (p(5.0, 0.0, 1.5), false),
            ];
            if back {
                // On the return run, and between the runs (each spans
                // `cos(tilt) < radius` either side of its centre line).
                probes.push((p(5.0, 2.0 * radius, 0.0), true));
                probes.push((p(5.0, radius, 0.0), false));
            }
            for (point, inside) in probes {
                let at = p(point.x() * s, point.y() * s, point.z() * s);
                assert_probe(&topo, solid, at, inside, s, &label);
            }
        }
    }
}

/// Tight, tilted turns that truly meet themselves stay refused, typed and
/// atomic, each with an exact witness: two sections of the body sharing a
/// point. The 270° turn of radius 1.2 at 30° is one #1860's review listed as
/// built on main; the oracle shows it overlaps.
#[test]
fn tight_turns_that_meet_themselves_stay_refused() {
    let cases = [
        (30.0, 1.2, 1.5 * PI, false),
        (-30.0, 1.3, 1.5 * PI, false),
        (15.0, 1.1, 1.5 * PI, false),
        (30.0, 1.5, 1.9 * PI, false),
        (45.0, 2.2, 1.9 * PI, false),
    ];
    for (degrees, radius, turn, back) in cases {
        let case = Case {
            tilt: f64::to_radians(degrees),
            radius,
            turn,
            back,
        };
        assert!(case.witness().is_some(), "{case:?}: no witness");
        let mut topo = Topology::new();
        let (profile, path) = case.inputs(&mut topo, 1.0);
        let slots = topo.allocated_slot_count();
        let refused = sweep_exact(&mut topo, profile, &path);
        assert!(
            matches!(&refused, Err(OperationsError::InvalidInput { reason })
                if reason.contains("meet itself")),
            "{case:?}: {refused:?}"
        );
        assert_eq!(topo.num_faces(), 1, "{case:?}: only the profile is left");
        assert_eq!(topo.allocated_slot_count(), slots, "{case:?}");
    }
}

/// Across the radius at which tight, tilted turns start to meet themselves,
/// the certificate never accepts a body the oracle shows overlapping. Each
/// sweep crosses from witnessed overlaps (or the fold) into accepted, clean
/// bodies, so it tests both sides of the true contact radius; the bodies in
/// between that are refused without a witness are the certificate's
/// conservative margin.
#[test]
fn radius_sweeps_across_contact_never_accept_an_overlap() {
    const STEPS: u32 = 20;
    let families = [
        (30.0, 1.5 * PI, false),
        (15.0, 1.5 * PI, false),
        (30.0, PI, true),
        (60.0, PI, true),
    ];
    for (degrees, turn, back) in families {
        let tilt = f64::to_radians(degrees);
        let (mut accepted, mut witnessed) = (0, 0);
        for k in 0..STEPS {
            let radius = fold_radius(tilt) * f64::from(k).mul_add(0.025, 0.98);
            let case = Case {
                tilt,
                radius,
                turn,
                back,
            };
            let mut topo = Topology::new();
            let built = case.build(&mut topo, 1.0);
            let witness = case.witness();
            witnessed += usize::from(witness.is_some());
            match built {
                Ok(solid) => {
                    accepted += 1;
                    assert!(
                        radius > fold_radius(tilt),
                        "{case:?}: accepted past the fold"
                    );
                    assert!(witness.is_none(), "{case:?}: accepted, but {witness:?}");
                    let report = crate::validate::validate_solid(&topo, solid).unwrap();
                    assert!(report.is_valid(), "{case:?}: {report:?}");
                }
                Err(OperationsError::InvalidInput { .. }) => {
                    assert_eq!(topo.num_faces(), 1, "{case:?}");
                }
                Err(other) => panic!("{case:?}: {other:?}"),
            }
        }
        assert!(
            accepted > 0,
            "tilt {degrees}, turn {turn}: nothing accepted"
        );
        assert!(
            accepted < STEPS as usize,
            "tilt {degrees}, turn {turn}: the sweep never reaches contact"
        );
        if !back {
            assert!(
                witnessed > 0,
                "tilt {degrees}, turn {turn}: no overlap seen"
            );
        }
    }
}
