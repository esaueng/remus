//! Line–torus contacts the solver must resolve, through their consumers:
//! the GFA and `remus-check` ray classifiers and the boolean section
//! trimmer.
//!
//! Review round 3 of the line–torus solve found two roots a resolvable
//! distance apart reported as one unresolved contact:
//!
//! - a point `1e-5` inside the tube of a large thin torus (`R = 1.5e6`,
//!   `r = 500`): along `+Z` and `+Y` the line leaves the tube `0.1` and
//!   `5.5` either side of the point, and the single contact reported at
//!   `t = 0` was skipped as behind the ray. The GFA classifier read the
//!   point Outside;
//! - a box edge a hair inside the tube: the two crossings the section
//!   trimmer splits a plane×torus oval at came back as one point between
//!   them, off the surface.
//!
//! The solver now separates such roots, and the consumers place a contact
//! by the whole window it may occupy. Points are checked against the tube's
//! signed distance, cuts against slicing integrals.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_algo::FaceClass;
use remus_algo::classifier::{RayCastGeoms, classify_ray_cast, ray_cast_rays_cached};
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_math::compact_sort::CompactSort;
use remus_math::curves::Circle3D;
use remus_math::mat::Mat4;
use remus_math::surfaces::ToroidalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_torus};
use remus_operations::tessellate::{boundary_edge_count, tessellate_solid};
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

/// A whole torus about `z` through `centre`: one face whose boundary is a
/// single closed circle on the outer equator. The GFA classifier reads this
/// construction as the whole torus (see `regress_torus_tangent_ray_parity`).
fn whole_torus(topo: &mut Topology, centre: Point3, big: f64, small: f64) -> SolidId {
    let surface = ToroidalSurface::new(centre, big, small).unwrap();
    let v0 = topo.add_vertex(Vertex::new(surface.evaluate(0.0, 0.0), 1e-7));
    let circle = Circle3D::new(centre, Vec3::new(0.0, 0.0, 1.0), big + small).unwrap();
    let mut edge = Edge::new(v0, v0, EdgeCurve::Circle(circle));
    edge.set_trim(Some((0.0, TAU)));
    let e = topo.add_edge(edge);
    let wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], true).unwrap());
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Torus(surface)));
    let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
    topo.add_solid(Solid::new(shell, vec![]))
}

/// `make_torus`'s whole torus about `z` (two point seams), moved to
/// `centre`: the construction the `remus-check` classifier reads whole.
fn made_torus(topo: &mut Topology, centre: Point3, big: f64, small: f64) -> SolidId {
    let solid = make_torus(topo, big, small, 32).unwrap();
    transform_solid(
        topo,
        solid,
        &Mat4::translation(centre.x(), centre.y(), centre.z()),
    )
    .unwrap();
    solid
}

fn torus_signed_distance(q: Point3, centre: Point3, big: f64, small: f64) -> f64 {
    let d = q - centre;
    let rho = d.x().hypot(d.y());
    (rho - big).hypot(d.z()) - small
}

/// The reviewed point (round 3, finding 1), through the GFA classifier, its
/// cardinal rays, and the `remus-check` classifier.
#[test]
fn point_just_inside_a_large_thin_torus_is_inside() {
    let (big, small) = (1_500_000.0, 500.0);
    let centre = Point3::new(0.0, 0.0, 0.0);
    let mut topo = Topology::default();
    let gfa = whole_torus(&mut topo, centre, big, small);
    let made = made_torus(&mut topo, centre, big, small);
    let q = Point3::new(1_500_499.999_99, 0.0, 0.0);
    let depth = torus_signed_distance(q, centre, big, small);
    assert!((depth + 1.000_007_6e-5).abs() < 1e-11, "{depth}");

    assert_eq!(classify_ray_cast(&topo, gfa, q).unwrap(), FaceClass::Inside);
    let geoms = RayCastGeoms::new(&topo, gfa).unwrap();
    let [cardinal, generic] = ray_cast_rays_cached(&geoms, q, Tolerance::default(), true);
    // Every ray leaves the tube once: +X after 1e-5, +Y after 5.5, +Z after
    // 0.1. None is a clean miss.
    assert!(cardinal.iter().all(|ray| ray.0), "{cardinal:?}");
    assert!(generic.iter().all(|ray| ray.0), "{generic:?}");
    assert_eq!(
        classify_point(&topo, made, q, &ClassifyOptions::default()).unwrap(),
        PointClassification::Inside
    );
}

/// Review round 4 of the line–torus solve, through the GFA classifier: tori
/// far along a ray, where the `f64` values of the ray's parameter are about
/// as far apart as two crossings.
///
/// - A unit torus `1e15` from the point: the `+X` ray crosses it four times,
///   eight spacings of `t` apart. The solve had merged them into two
///   tangential contacts, which flagged a clean ray; they are four clean
///   crossings.
/// - A tube `1.5e-7` across, `1500` below the point and crossed a hair
///   inside its outer equator: the two crossings lie in the `f64` cells
///   either side of `t = 1500`. The solve had returned them as two resolved
///   crossings, each window holding both. They are one even contact, which
///   adds no parity and flags the ray.
///
/// Either way the point is outside. (The `remus-check` classifier draws its
/// own ray directions; its face counts along these two rays are pinned in
/// its `torus_contact_tests`.)
#[test]
fn far_tori_classify_by_what_the_ray_parameter_can_separate() {
    let tol = Tolerance::default();
    let centre = Point3::new(1e15, 0.0, 0.0);
    let mut topo = Topology::default();
    let gfa = whole_torus(&mut topo, centre, 1.5, 0.5);
    let q = Point3::new(0.0, 0.0, 0.0);
    assert_eq!(
        classify_ray_cast(&topo, gfa, q).unwrap(),
        FaceClass::Outside
    );
    let geoms = RayCastGeoms::new(&topo, gfa).unwrap();
    let [cardinal, _] = ray_cast_rays_cached(&geoms, q, tol, true);
    // The cardinal rays are +Z, +X, +Y: +X runs through the torus.
    assert_eq!(cardinal[1], (false, false), "{cardinal:?}");
    // From the tube's core circle and from the hole, the same torus.
    let core = Point3::new(1e15 + 1.5, 0.0, 0.0);
    assert_eq!(
        classify_ray_cast(&topo, gfa, core).unwrap(),
        FaceClass::Inside
    );
    assert_eq!(
        classify_ray_cast(&topo, gfa, centre).unwrap(),
        FaceClass::Outside
    );

    let (big, small) = (0.0015, 0.000_000_15);
    let centre = Point3::new(0.0, 0.0, 0.0);
    let mut topo = Topology::default();
    let gfa = whole_torus(&mut topo, centre, big, small);
    let q = Point3::new(0.001_500_15, 0.0, -1500.0);
    assert_eq!(
        classify_ray_cast(&topo, gfa, q).unwrap(),
        FaceClass::Outside
    );
    let geoms = RayCastGeoms::new(&topo, gfa).unwrap();
    let [cardinal, _] = ray_cast_rays_cached(&geoms, q, tol, true);
    // +Z holds the unseparable pair: no parity, and the ray is flagged.
    assert_eq!(cardinal[0], (false, true), "{cardinal:?}");
    assert_eq!(cardinal[1], (false, false), "{cardinal:?}");
    assert_eq!(cardinal[2], (false, false), "{cardinal:?}");
}

/// Points a hair inside and outside the tube of large thin tori, at every
/// tube angle: from each, the cardinal rays run nearly along the surface, so
/// their roots come in close pairs about the point. Both classifiers agree
/// with the signed distance.
#[test]
fn points_next_to_thin_tubes_classify_by_their_side() {
    let mut failures = Vec::new();
    let mut checked = 0_usize;
    for (big, small) in [(1_500_000.0, 500.0), (1500.0, 0.5), (150.0, 1.5)] {
        let centre = Point3::new(0.25 * big, -0.5 * big, 0.125 * big);
        let mut topo = Topology::default();
        let gfa = whole_torus(&mut topo, centre, big, small);
        let made = made_torus(&mut topo, centre, big, small);
        for iu in 0..8 {
            // Cardinal directions in the torus's plane: the horizontal rays
            // are tangent to the tube's circles there.
            let u = f64::from(iu) * TAU / 8.0;
            for iv in 0..8 {
                let v = f64::from(iv) * TAU / 8.0;
                let radial = Vec3::new(u.cos(), u.sin(), 0.0);
                let normal = radial * v.cos() + Vec3::new(0.0, 0.0, v.sin());
                let on = centre + radial * big + normal * small;
                for depth in [-1e-2, -2e-5, -2e-6, 2e-6, 2e-5, 1e-2] {
                    let q = on + normal * depth;
                    let sd = torus_signed_distance(q, centre, big, small);
                    // The point's own coordinates round at this scale.
                    if sd.abs() < 1e-6 || (sd < 0.0) != (depth < 0.0) {
                        continue;
                    }
                    checked += 1;
                    let got = classify_ray_cast(&topo, gfa, q).unwrap();
                    if (got == FaceClass::Inside) != (sd < 0.0) {
                        failures.push(format!("gfa R {big} u {u} v {v} depth {depth}: {got:?}"));
                    }
                    let expected = if sd < 0.0 {
                        PointClassification::Inside
                    } else {
                        PointClassification::Outside
                    };
                    let got = classify_point(&topo, made, q, &ClassifyOptions::default()).unwrap();
                    if got != expected {
                        failures.push(format!("check R {big} u {u} v {v} depth {depth}: {got:?}"));
                    }
                }
            }
        }
    }
    assert!(checked > 900, "{checked}");
    assert!(
        failures.is_empty(),
        "{} of {checked}:\n{}",
        failures.len(),
        failures[..failures.len().min(20)].join("\n")
    );
}

fn torus_at(topo: &mut Topology, big: f64, small: f64, c: Point3) -> SolidId {
    let t = make_torus(topo, big, small, 32).unwrap();
    transform_solid(topo, t, &Mat4::translation(c.x(), c.y(), c.z())).unwrap();
    t
}

/// Volume of the torus (axis z, centre `(_, y0, h + z0)`) inside the box's
/// `y > 0, z < h` corner, for a torus clear of the box's other faces: the
/// torus sliced across its axis into annuli, each clipped by the half-plane
/// `y > 0` in closed form, integrated over the tube angle.
///
/// The clipped annulus changes form where one of its circles reaches the
/// line `y = 0` (`R ± r·sin θ = |y0|`), and a shallow notch lies wholly
/// within a sliver of `θ` past such a point. The midpoint rule is applied
/// to each piece between those points with as many nodes, so the sliver is
/// resolved however thin it is (as in `regress_torus_far_line_roots`, where
/// the notches are deep enough for one piece).
fn notch_volume(big: f64, small: f64, y0: f64, z0: f64) -> f64 {
    // The part of a disc of radius `rho` beyond a line `d ≥ 0` from its
    // centre: `rho²·(x − sin x)/2` with `x` the angle the chord subtends.
    // A notch's caps are slivers whose `x` is a few thousandths, where the
    // textbook `rho²·acos(d/rho) − d·sqrt(rho² − d²)` loses all but a few
    // digits to cancellation; `x − sin x` is taken from its series there.
    let cap = |rho: f64, d: f64| -> f64 {
        if d >= rho {
            return 0.0;
        }
        let x = 2.0 * ((rho - d) * (rho + d)).sqrt().atan2(d);
        let x2 = x * x;
        let x_minus_sin = if x < 0.5 {
            // x³/3! − x⁵/5! + … to x¹⁵/15!, nested.
            let mut tail = 1.0;
            for k in (2..=7).rev() {
                let k = f64::from(k);
                tail = 1.0 - x2 / ((2.0 * k) * (2.0 * k + 1.0)) * tail;
            }
            x * x2 / 6.0 * tail
        } else {
            x - x.sin()
        };
        0.5 * rho * rho * x_minus_sin
    };
    // The part of the disc about the torus's axis on the box's side,
    // `y > 0`.
    let half_disc = |rho: f64| -> f64 {
        if y0 < 0.0 {
            cap(rho, -y0)
        } else {
            PI.mul_add(rho * rho, -cap(rho, y0))
        }
    };
    let th_max = (z0 / small).clamp(-1.0, 1.0).acos();
    // Tube angles where `R + s` or `R − s` (with `s = r sin θ`) is `|y0|`.
    let mut breaks = vec![0.0, th_max];
    for s in [y0.abs() - big, big - y0.abs()] {
        let x = s / small;
        if (0.0..=1.0).contains(&x) {
            for th in [x.asin(), PI - x.asin()] {
                if th > 0.0 && th < th_max {
                    breaks.push(th);
                }
            }
        }
    }
    breaks.compact_sort_by(f64::total_cmp);
    let n = 200_000_u32;
    let mut total = 0.0;
    for piece in breaks.windows(2) {
        let (a, b) = (piece[0], piece[1]);
        let mut acc = 0.0;
        for i in 0..n {
            let th = (b - a).mul_add((f64::from(i) + 0.5) / f64::from(n), a);
            let s = small * th.sin();
            acc += (half_disc(big + s) - half_disc(big - s)) * s;
        }
        total += acc * (b - a) / f64::from(n);
    }
    total
}

/// What a grazing cut came to.
#[derive(Debug, PartialEq)]
enum Graze {
    /// A valid seven-face solid whose grazed edge is split at the two
    /// closed-form crossings, whose mesh is closed, whose material is where
    /// the tube's signed distance puts it, and whose notch (the intersection
    /// of the same operands) has the slicing integral's volume.
    Exact,
    /// The box, untouched.
    Unchanged,
    /// A typed refusal.
    Refused,
}

/// [`graze_cut_at`] at scale 1.
fn graze_cut(depth: f64) -> Graze {
    graze_cut_at(1.0, depth)
}

/// [`graze_cut_split`] on a plain box.
fn graze_cut_at(k: f64, depth: f64) -> Graze {
    graze_cut_split(k, depth, false)
}

/// A box `l × w × h`, with its top `y = 0` edge split at `x = split_x` on
/// both faces that use it when `split_x` is given: the same solid, its edge
/// in two collinear pieces (as an earlier operation or an import can leave
/// it).
fn box_with_split_top_edge(
    topo: &mut Topology,
    (l, w, h): (f64, f64, f64),
    split_x: Option<f64>,
) -> SolidId {
    let solid = make_box(topo, l, w, h).unwrap();
    let Some(x) = split_x else {
        return solid;
    };
    let on_edge = |p: Point3| p.y().abs() < 1e-12 && (p.z() - h).abs() < 1e-12;
    let edges = remus_topology::explorer::solid_edges(topo, solid).unwrap();
    let grazed = edges
        .into_iter()
        .find(|&e| {
            let edge = topo.edge(e).unwrap();
            on_edge(topo.vertex(edge.start()).unwrap().point())
                && on_edge(topo.vertex(edge.end()).unwrap().point())
        })
        .unwrap();
    let (start, end) = {
        let edge = topo.edge(grazed).unwrap();
        (edge.start(), edge.end())
    };
    let mid = topo.add_vertex(Vertex::new(Point3::new(x, 0.0, h), 1e-7));
    let first = topo.add_edge(Edge::new(start, mid, EdgeCurve::Line));
    let second = topo.add_edge(Edge::new(mid, end, EdgeCurve::Line));
    let shell = topo.solid(solid).unwrap().outer_shell();
    let faces = topo.shell(shell).unwrap().faces().to_vec();
    for (slot, face) in faces.into_iter().enumerate() {
        let wire_id = topo.face(face).unwrap().outer_wire();
        let wire = topo.wire(wire_id).unwrap();
        if !wire.edges().iter().any(|oe| oe.edge() == grazed) {
            continue;
        }
        let mut uses = Vec::new();
        for oe in wire.edges() {
            if oe.edge() != grazed {
                uses.push(*oe);
            } else if oe.is_forward() {
                uses.extend([
                    OrientedEdge::new(first, true),
                    OrientedEdge::new(second, true),
                ]);
            } else {
                uses.extend([
                    OrientedEdge::new(second, false),
                    OrientedEdge::new(first, false),
                ]);
            }
        }
        let wire = topo.add_wire(Wire::new(uses, true).unwrap());
        let (surface, reversed) = {
            let old = topo.face(face).unwrap();
            (old.surface().clone(), old.is_reversed())
        };
        let mut split_face = Face::new(wire, vec![], surface);
        split_face.set_reversed(reversed);
        let split_face = topo.add_face(split_face);
        topo.shell_mut(shell).unwrap().faces_mut()[slot] = split_face;
    }
    assert!(validate_solid(topo, solid).unwrap().is_valid());
    solid
}

/// Cut a box `10·k` on a side by a torus (axis z, `R = 1.5·k`, `r = 0.5·k`)
/// hung over the box's `y = 0` face, its centre `0.2·k` above the top face,
/// so that its outer flank at the top face's height reaches `depth` into
/// the box: the top edge `y = 0, z = h` runs a hair inside the tube. The
/// edge's two crossings are `2 sqrt(ρ² − y0²)` apart (`4e-3` at a depth of
/// `1e-6` and `k = 1`); the top-face circle and the side-face oval both end
/// on them.
///
/// Panics on a result that is neither exact, nor the unchanged box, nor a
/// refusal: a valid solid that is wrong.
/// With `split`, the grazed edge is split at `x = 5·k`, between its two
/// crossings ([`box_with_split_top_edge`]).
#[allow(clippy::too_many_lines)]
fn graze_cut_split(k: f64, depth: f64, split: bool) -> Graze {
    let (l, w, h) = (10.0 * k, 10.0 * k, 10.0 * k);
    let (big, small, sink): (f64, f64, f64) = (1.5 * k, 0.5 * k, 0.2 * k);
    let xc = 5.0 * k;
    // The torus's outer radius at the top face's height.
    let rho = big + (small * small - sink * sink).sqrt();
    let y0 = depth - rho;
    let half = (rho * rho - y0 * y0).sqrt();
    let centre = Point3::new(xc, y0, h + sink);
    let mut topo = Topology::new();
    let b = box_with_split_top_edge(&mut topo, (l, w, h), split.then_some(xc));
    let t = torus_at(&mut topo, big, small, centre);
    let expect = notch_volume(big, small, y0, sink);
    let Ok(s) = boolean(&mut topo, BooleanOp::Cut, b, t) else {
        return Graze::Refused;
    };
    let context = format!("scale {k} depth {depth:e} split {split}");
    assert!(validate_solid(&topo, s).unwrap().is_valid(), "{context}");
    let faces = remus_topology::explorer::solid_faces(&topo, s).unwrap();
    let torus_faces = faces
        .iter()
        .filter(|&&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .count();
    let removed = l * w * h - solid_volume(&topo, s, 5e-4).unwrap();
    // `solid_volume` of the box rounds at a few units in the last place of
    // its volume.
    let box_slack = 1e-14 * l * w * h + 1e-5;
    if (faces.len(), torus_faces) == (6, 0) {
        assert!(removed.abs() < box_slack, "{context}: {removed:e}");
        return Graze::Unchanged;
    }
    assert_eq!((faces.len(), torus_faces), (7, 1), "{context}");
    // Vertices strictly inside the grazed edge: the two crossings, where
    // the closed form puts them.
    let mut on_edge: Vec<f64> = Vec::new();
    for &f in &faces {
        let face = topo.face(f).unwrap();
        for oe in topo.wire(face.outer_wire()).unwrap().edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            for v in [edge.start(), edge.end()] {
                let p = topo.vertex(v).unwrap().point();
                let inside = p.x() > 1e-6 && p.x() < l - 1e-6;
                if inside
                    && p.y().abs() < 1e-7
                    && (p.z() - h).abs() < 1e-7
                    && !on_edge.iter().any(|x| (x - p.x()).abs() < 1e-9)
                {
                    on_edge.push(p.x());
                }
            }
        }
    }
    assert_eq!(on_edge.len(), 2, "{context}: {on_edge:?}");
    if on_edge[0] > on_edge[1] {
        on_edge.swap(0, 1);
    }
    for (got, want) in on_edge.iter().zip([xc - half, xc + half]) {
        assert!(
            (got - want).abs() < 1e-9,
            "{context}: edge crossing {got} vs {want}"
        );
    }
    // The side-face oval is a fitted curve, good to about 1e-5 in volume
    // here (as in `regress_torus_far_line_roots`).
    assert!(
        (removed - expect).abs() < 1e-4 * expect + box_slack,
        "{context}: removed {removed:e}, expected {expect:e}"
    );
    // A closed mesh at a display and a fine deflection.
    for deflection in [1e-2 * k, 1e-4 * k] {
        let mesh = tessellate_solid(&topo, s, deflection).unwrap();
        assert_eq!(
            boundary_edge_count(&mesh),
            0,
            "{context}: open mesh at {deflection:e}"
        );
    }
    // Material by the ray classifier, against the tube's signed distance:
    // in the notch (inside the torus), and beside it below the notch on the
    // side face, past its depth under the top face and past either crossing
    // (all outside the torus).
    let opts = ClassifyOptions::default();
    let probes = [
        (Point3::new(xc, 0.3 * depth, h - 0.3 * depth), true),
        (Point3::new(xc, 0.3 * depth, h - 3.0 * depth), false),
        (Point3::new(xc, 3.0 * depth, h - 0.3 * depth), false),
        (
            Point3::new(xc + 1.5 * half, 0.3 * depth, h - 0.3 * depth),
            false,
        ),
        (
            Point3::new(xc - 1.5 * half, 0.3 * depth, h - 0.3 * depth),
            false,
        ),
    ];
    for (q, in_notch) in probes {
        let sd = torus_signed_distance(q, centre, big, small);
        assert_eq!(sd < 0.0, in_notch, "{context}: probe {q:?} at {sd:e}");
        let expected = if in_notch {
            PointClassification::Outside
        } else {
            PointClassification::Inside
        };
        assert_eq!(
            classify_point(&topo, s, q, &opts).unwrap(),
            expected,
            "{context}: probe {q:?}"
        );
    }
    // The notch itself, as the intersection of the same operands: an exact
    // two-plane, one-torus solid whose volume is the slicing integral. (The
    // removed volume above is lost in the box's rounding at these depths.)
    let mut topo = Topology::new();
    let b = box_with_split_top_edge(&mut topo, (l, w, h), split.then_some(xc));
    let t = torus_at(&mut topo, big, small, centre);
    let notch = boolean(&mut topo, BooleanOp::Intersect, b, t)
        .unwrap_or_else(|e| panic!("{context}: the notch refused ({e:?}) where the cut built"));
    assert!(
        validate_solid(&topo, notch).unwrap().is_valid(),
        "{context}"
    );
    let notch_faces = remus_topology::explorer::solid_faces(&topo, notch).unwrap();
    assert_eq!(notch_faces.len(), 3, "{context}");
    let mesh = tessellate_solid(&topo, notch, 1e-4 * k).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0, "{context}: open notch mesh");
    let volume = solid_volume(&topo, notch, 1e-4 * k).unwrap();
    // Measured within 1.5e-5 of the integral from `1e-6` to `1e-2` of the
    // scale deep.
    let error = (volume - expect).abs() / expect;
    assert!(
        error < 5e-5,
        "{context}: notch volume {volume:e}, expected {expect:e} (relative {error:e})"
    );
    Graze::Exact
}

/// The crossings the section trimmer splits its ovals at: the grazed edge
/// of [`graze_cut`], from `1e-13` deep (crossings `1.3e-6` apart) to `1e-3`
/// deep. Each is two resolved roots, symmetric about the torus's centre, at
/// the closed-form half-chord; round 3 got one unresolved point between
/// them.
#[test]
fn grazed_edge_crossings_are_two_resolved_roots() {
    use remus_math::analytic_intersection::intersect_line_torus_contacts;
    let (l, h) = (10.0, 10.0);
    let (big, small, sink): (f64, f64, f64) = (1.5, 0.5, 0.2);
    let rho = big + (small * small - sink * sink).sqrt();
    for exponent in 3..=13 {
        let depth = 10f64.powi(-exponent);
        let y0 = depth - rho;
        let torus = ToroidalSurface::new(Point3::new(5.0, y0, h + sink), big, small).unwrap();
        // The edge as the trimmer passes it: its start and raw vector.
        let contacts =
            intersect_line_torus_contacts(&torus, Point3::new(0.0, 0.0, h), Vec3::new(l, 0.0, 0.0));
        let context = format!("depth {depth:e}: {contacts:?}");
        assert_eq!(contacts.len(), 2, "{context}");
        // (ρ − y0)(ρ + y0), from the represented `y0`.
        let half = ((rho - y0) * (rho + y0)).sqrt();
        for (contact, want) in contacts.iter().zip([5.0 - half, 5.0 + half]) {
            assert!(contact.resolved, "{context}");
            // `ρ + y0` carries the rounding of `y0`: a part in 1e3 of the
            // shallowest depth.
            assert!(
                (contact.t * l - want).abs() <= 1e-12 + 2e-3 * half * (1e-13 / depth),
                "{context}: {} vs {want}",
                contact.t * l
            );
            // Within the stated 128 ulps either side.
            assert!(contact.t_hi - contact.t_lo <= 1e-13, "{context}");
        }
        assert!(
            ((contacts[1].t - contacts[0].t) * l - 2.0 * half).abs() <= 1e-2 * half,
            "{context}"
        );
    }
}

/// A box edge a hair inside a torus (round 3, finding 3, through the
/// section trimmer): the cut is exact, with the grazed edge split at the
/// two closed-form crossings, or it is refused. An intrusion below the
/// linear tolerance is no intersection and may leave the box as it is.
#[test]
fn box_edge_grazing_a_torus_cuts_exactly_or_refuses() {
    for depth in [1e-8, 1e-7] {
        // Anything but a wrong solid.
        graze_cut(depth);
    }
    let mut exact = 0;
    for depth in [1e-4, 1e-3, 1e-2, 1e-1] {
        let outcome = graze_cut(depth);
        assert_ne!(outcome, Graze::Unchanged, "depth {depth:e}");
        exact += usize::from(outcome == Graze::Exact);
    }
    assert!(exact >= 3, "{exact}");
}

/// B266: a notch between the linear tolerance and `1e-5` deep came back as
/// the unchanged box, a valid solid that is wrong.
///
/// The side face's plane × torus oval enters the box face for an arc far
/// shorter than any sample pitch (`1.3e-2` of a `2.2` oval at `1e-5`), so
/// phase FF's box prefilter dropped it, and the section trimmer's dense
/// parameters for its crossings were too coarse to re-sample the arc
/// between them. Kept, the arc bounds a lens with the grazed edge as thin
/// as the notch: a lens thinner than the section tolerance model's
/// separation distance (`1e-5`) is refused by name, since later stages may
/// weld the arc onto the edge; a thicker one is cut exactly.
#[test]
fn shallow_torus_notch_on_a_box_edge_is_not_dropped() {
    for depth in [3e-7, 1e-6, 1e-5] {
        assert_ne!(graze_cut(depth), Graze::Unchanged, "depth {depth:e}");
    }
    // Lenses three and ten times the separation distance are cut.
    for depth in [3e-5, 1e-4] {
        assert_eq!(graze_cut(depth), Graze::Exact, "depth {depth:e}");
    }
}

/// [`shallow_torus_notch_on_a_box_edge_is_not_dropped`] with the whole
/// configuration scaled by `1e-1` to `1e2` at the same absolute depths:
/// exact or refused, never the unchanged box, and exact once the lens is
/// three times the separation distance.
#[test]
fn shallow_torus_notches_across_scales_are_exact_or_refused() {
    for k in [0.1, 10.0, 100.0] {
        for depth in [3e-7, 1e-6, 3e-6, 1e-5] {
            assert_ne!(
                graze_cut_at(k, depth),
                Graze::Unchanged,
                "scale {k} depth {depth:e}"
            );
        }
        for depth in [3e-5, 1e-4] {
            assert_eq!(
                graze_cut_at(k, depth),
                Graze::Exact,
                "scale {k} depth {depth:e}"
            );
        }
    }
}

/// Review of #1867, finding 1: the grazed edge split between its two
/// crossings (at `x = 5`, on both faces that use it) leaves the arc's ends
/// on two collinear edges. The lens is measured against the outline's
/// edges, so the split changes nothing: refused by the lens gate itself at
/// `1e-6` and `3e-6` (before the review fix the arc went on downstream and
/// assembly happened to fail on it), exact or refused at `1e-5`, exact at
/// `1e-4`. Never the unchanged box.
#[test]
fn shallow_notch_across_a_split_grazed_edge_is_exact_or_refused() {
    for depth in [1e-6, 3e-6] {
        assert_eq!(graze_cut_split(1.0, depth, true), Graze::Refused);
        // The refusal is the section's, by name.
        let (big, small, sink): (f64, f64, f64) = (1.5, 0.5, 0.2);
        let rho = big + (small * small - sink * sink).sqrt();
        let mut topo = Topology::new();
        let b = box_with_split_top_edge(&mut topo, (10.0, 10.0, 10.0), Some(5.0));
        let t = torus_at(
            &mut topo,
            big,
            small,
            Point3::new(5.0, depth - rho, 10.0 + sink),
        );
        let refusal = remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Cut, b, t)
            .expect_err("a lens thinner than the separation distance");
        assert!(
            matches!(&refusal, remus_algo::error::AlgoError::IntersectionFailed(why)
                if why.contains("separation distance")),
            "depth {depth:e}: {refusal:?}"
        );
    }
    assert_ne!(graze_cut_split(1.0, 1e-5, true), Graze::Unchanged);
    assert_eq!(graze_cut_split(1.0, 1e-4, true), Graze::Exact);
}
