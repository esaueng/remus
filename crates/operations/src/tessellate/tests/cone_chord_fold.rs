//! B289: cone faces whose CDT turns triangles over, and the never-worse gate
//! on their repair.
//!
//! A cone band bounded by two circular arcs and two straight generators,
//! much narrower along its generators than its rim chords are long, is
//! triangulated in a chart whose `v` axis is stretched by `du / dv`. Its
//! interior points sit in the lens between the far rim's chart chords and
//! their 3D chords, so the triangles the CDT joins them into turn over. The
//! repair triangulates again without them (see
//! `nonplanar::cone_fold_repair`), and its result replaces the CDT's first
//! mesh only when it turns strictly fewer triangles over and stays within
//! the deflection.
//!
//! Oracle, independent of the repair's own code: each band is meshed twice,
//! with the repair and without it (`without_repair`: the plain CDT mesh,
//! which is `main`'s, pinned by fingerprint below). The mesh must be the
//! plain one bit for bit, or turn strictly fewer triangles over against the
//! carrier's closed-form normal and keep every triangle the plain mesh does
//! not have within the deflection of the cone, read densely: the closed-form
//! distance at a
//! barycentric grid of `GRID` divisions and `EDGE_SAMPLES` more points on
//! each edge (the gate adds a curvature bound for the points between, so a
//! mesh it accepts passes this read). Either way the face mesh is manifold
//! with one boundary cycle. Each case prints its triangle and inversion
//! counts, deviation and area ratio with and without the repair
//! (`--no-capture`).

#![allow(clippy::print_stderr)]

use remus_math::curves::Circle3D;
use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::tessellate::nonplanar::cone_fold_repair::{EDGE_SAMPLES, GRID, without_repair};
use crate::tessellate::{TriangleMesh, tessellate_sheet};

/// The band `0 ≤ u ≤ span`, `v0 ≤ |v| ≤ v1` of the cone of apex `apex`,
/// axis `+ẑ` and half-angle `a`, on the upper nappe or (with `lower`) the
/// lower one, as a one-face sheet, its near rim split into `near_pieces`
/// equal arcs (so it is sampled on its own and more finely than the far
/// rim); with `reversed` the face is wound against the carrier. Returns the
/// topology, the sheet, and the carrier.
fn band_sheet_split(
    apex: Point3,
    a: f64,
    span: f64,
    (v0, v1): (f64, f64),
    near_pieces: usize,
    lower: bool,
    reversed: bool,
) -> (Topology, remus_topology::shell::ShellId, ConicalSurface) {
    let carrier =
        ConicalSurface::with_ref_dir(apex, Vec3::new(0.0, 0.0, 1.0), a, Vec3::new(1.0, 0.0, 0.0))
            .unwrap();
    let sign = if lower { -1.0 } else { 1.0 };
    let (near, far) = (sign * v0, sign * v1);
    let mut topo = Topology::new();
    // Corners counter-clockwise about ẑ along each rim: near rim then far.
    let corner = |u: f64, v: f64| carrier.evaluate(u, v);
    // Near rim corners 0 and 1, far rim 2 and 3, then the near rim's
    // splitting points in order.
    let mut points = vec![
        corner(0.0, near),
        corner(span, near),
        corner(span, far),
        corner(0.0, far),
    ];
    points.extend((1..near_pieces).map(|k| corner(span * k as f64 / near_pieces as f64, near)));
    let vertices: Vec<_> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-12)))
        .collect();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let (sin_a, cos_a) = a.sin_cos();
    let mut arc = |from: usize, to: usize, v: f64| {
        let circle = Circle3D::new(apex + z * (v * sin_a), z, v.abs() * cos_a).unwrap();
        let t0 = circle.project(points[from]);
        let mut t1 = circle.project(points[to]);
        if t1 <= t0 {
            t1 += std::f64::consts::TAU;
        }
        let mut edge = Edge::new(vertices[from], vertices[to], EdgeCurve::Circle(circle));
        edge.set_trim(Some((t0, t1)));
        topo.add_edge(edge)
    };
    let near_stops: Vec<usize> = std::iter::once(0)
        .chain(4..4 + near_pieces - 1)
        .chain(std::iter::once(1))
        .collect();
    let near_arcs: Vec<_> = near_stops
        .windows(2)
        .map(|pair| arc(pair[0], pair[1], near))
        .collect();
    let far_arc = arc(3, 2, far);
    let side = topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line));
    let start = topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line));
    // Counter-clockwise in the `(u, v)` chart on the upper nappe; `v`
    // falls outward below the apex, so there, and on a reversed face, the
    // wire runs the other way to keep the face on its left.
    let mut edges: Vec<_> = near_arcs
        .iter()
        .map(|&edge| OrientedEdge::new(edge, true))
        .collect();
    edges.extend([
        OrientedEdge::new(side, true),
        OrientedEdge::new(far_arc, false),
        OrientedEdge::new(start, true),
    ]);
    if lower != reversed {
        edges = edges
            .into_iter()
            .rev()
            .map(|e| OrientedEdge::new(e.edge(), !e.is_forward()))
            .collect();
    }
    let wire = topo.add_wire(Wire::new(edges, true).unwrap());
    let surface = FaceSurface::Cone(carrier.clone());
    let face = if reversed {
        Face::new_reversed(wire, vec![], surface)
    } else {
        Face::new(wire, vec![], surface)
    };
    let face = topo.add_face(face);
    let shell = topo.add_shell(remus_topology::shell::Shell::new(vec![face]).unwrap());
    topo.set_shell_body_class(shell, remus_topology::BodyClass::Sheet)
        .unwrap();
    (topo, shell, carrier)
}

/// Distance from `point` to the double cone `carrier`, in closed form: the
/// distance to the nearer of its two generator half-lines in the plane
/// through the axis and the point.
fn cone_distance(carrier: &ConicalSurface, point: Point3) -> f64 {
    let w = point - carrier.apex();
    let z = w.dot(carrier.axis());
    let r = (w - carrier.axis() * z).length();
    let (sin_a, cos_a) = carrier.half_angle().sin_cos();
    // Generator `(r, z) = t (cos a, ±sin a)`, `t ≥ 0`.
    let to_line = |sign: f64| {
        let along = r.mul_add(cos_a, sign * z * sin_a);
        if along >= 0.0 {
            r.mul_add(sin_a, -(sign * z * cos_a)).abs()
        } else {
            w.length()
        }
    };
    to_line(1.0).min(to_line(-1.0))
}

/// The largest closed-form distance from the cone over the triangle `p`,
/// read at the gate's sample points: a barycentric grid of `GRID` divisions
/// and `EDGE_SAMPLES` more points along each edge (no curvature bound: this
/// is a sample, and the gate's bound is at least it).
fn dense_deviation(carrier: &ConicalSurface, p: [Point3; 3]) -> f64 {
    let mut worst = 0.0_f64;
    for i in 0..=GRID {
        for j in 0..=GRID - i {
            let (l1, l2) = (i as f64 / GRID as f64, j as f64 / GRID as f64);
            let q = p[0] + (p[1] - p[0]) * l1 + (p[2] - p[0]) * l2;
            worst = worst.max(cone_distance(carrier, q));
        }
    }
    for k in 0..3 {
        let (a, b) = (p[k], p[(k + 1) % 3]);
        for step in 1..=EDGE_SAMPLES {
            let t = step as f64 / (EDGE_SAMPLES + 1) as f64;
            worst = worst.max(cone_distance(carrier, a + (b - a) * t));
        }
    }
    worst
}

/// What one face mesh reads against its carrier.
#[derive(Debug)]
struct Reading {
    triangles: usize,
    /// Triangles whose normal disagrees in sign with the face's (the
    /// carrier's, flipped for a reversed face), or degenerate.
    inverted: usize,
    /// Largest [`dense_deviation`] of any triangle.
    deviation: f64,
    /// Mesh area over the band's closed-form area.
    area_ratio: f64,
    /// Non-manifold or wrongly wound half-edges, or not one boundary cycle.
    structure: Vec<String>,
}

fn read_mesh(mesh: &TriangleMesh, carrier: &ConicalSurface, sign: f64, area: f64) -> Reading {
    let mut structure = Vec::new();
    let mut deviation = 0.0_f64;
    let mut inverted = 0;
    let mut mesh_area = 0.0;
    let mut half_edges = std::collections::BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let p = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize]);
        let cross = (p[1] - p[0]).cross(p[2] - p[0]);
        let centroid = p[0] + ((p[1] - p[0]) + (p[2] - p[0])) * (1.0 / 3.0);
        // The closed-form normal of the generator line through the
        // centroid (one vector along the whole line): `project_point` reads
        // a point below the apex at its own longitude, and the line through
        // it is the upper nappe's half a turn round.
        let (u, v) = carrier.project_point(centroid);
        let line = if v < 0.0 { u + std::f64::consts::PI } else { u };
        if cross.dot(carrier.normal(line, 0.0)) * sign <= 0.0 {
            inverted += 1;
        }
        mesh_area += 0.5 * cross.length();
        deviation = deviation.max(dense_deviation(carrier, p));
        for (i, j) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *half_edges.entry((i, j)).or_insert(0_usize) += 1;
        }
    }
    if mesh.indices.is_empty() {
        structure.push("empty mesh".to_owned());
    }
    if half_edges.values().any(|&n| n > 1) {
        structure.push("a half-edge used twice (non-manifold or folded)".to_owned());
    }
    let boundary: Vec<(u32, u32)> = half_edges
        .keys()
        .filter(|&&(i, j)| !half_edges.contains_key(&(j, i)))
        .copied()
        .collect();
    let next: std::collections::BTreeMap<u32, u32> = boundary.iter().copied().collect();
    if next.len() != boundary.len() {
        structure.push("a boundary vertex leaves twice".to_owned());
    } else if let Some(&(first, _)) = boundary.first() {
        let (mut at, mut steps) = (first, 0);
        while let Some(&to) = next.get(&at) {
            at = to;
            steps += 1;
            if at == first || steps > boundary.len() {
                break;
            }
        }
        if at != first || steps != boundary.len() {
            structure.push(format!(
                "boundary is not one cycle ({steps} of {} half-edges)",
                boundary.len()
            ));
        }
    } else if !mesh.indices.is_empty() {
        structure.push("no boundary".to_owned());
    }
    Reading {
        triangles: mesh.indices.len() / 3,
        inverted,
        deviation,
        area_ratio: mesh_area / area,
        structure,
    }
}

/// FNV-1a over a mesh's positions (bits) and indices: equal for meshes
/// equal bit for bit.
fn fingerprint(mesh: &TriangleMesh) -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    let mut eat = |word: u64| {
        for byte in word.to_le_bytes() {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0100_0000_01b3);
        }
    };
    eat(mesh.positions.len() as u64);
    for point in &mesh.positions {
        eat(point.x().to_bits());
        eat(point.y().to_bits());
        eat(point.z().to_bits());
    }
    eat(mesh.indices.len() as u64);
    for &index in &mesh.indices {
        eat(u64::from(index));
    }
    hash
}

/// One band variant meshed with the repair and without it.
struct Outcome {
    label: String,
    repaired: Reading,
    plain: Reading,
    /// The repaired mesh is the plain one bit for bit.
    unchanged: bool,
    /// Largest [`dense_deviation`] of a repaired triangle the plain mesh
    /// does not have (zero when unchanged).
    changed_deviation: f64,
    plain_fingerprint: u64,
}

/// A band (see [`band_sheet_split`]) meshed with and without the repair.
#[allow(clippy::too_many_arguments)]
fn mesh_band(
    apex: Point3,
    a: f64,
    span: f64,
    slant: (f64, f64),
    near_pieces: usize,
    deflection: f64,
    lower: bool,
    reversed: bool,
) -> Outcome {
    let label = format!(
        "apex {apex:?} a {a} span {span} slant {slant:?} pieces {near_pieces} deflection {deflection} lower {lower} reversed {reversed}"
    );
    let (topo, sheet, carrier) =
        band_sheet_split(apex, a, span, slant, near_pieces, lower, reversed);
    let repaired = tessellate_sheet(&topo, sheet, deflection).unwrap();
    let plain = without_repair(|| tessellate_sheet(&topo, sheet, deflection)).unwrap();
    // Area of the band: the developed annular sector of opening `span cos a`.
    let area = 0.5 * span * a.cos() * slant.1.mul_add(slant.1, -(slant.0 * slant.0));
    // A face's orientation normal is the chart's `S_u × S_v`: the line
    // normal on the upper nappe, its opposite below the apex (where `S_u`
    // turns over with `v`), flipped for a reversed face.
    let sign = if lower == reversed { 1.0 } else { -1.0 };
    let unchanged = repaired.positions == plain.positions && repaired.indices == plain.indices;
    let corners = |mesh: &TriangleMesh| -> Vec<[[u64; 3]; 3]> {
        mesh.indices
            .chunks_exact(3)
            .map(|t| {
                let mut key = [0, 1, 2].map(|k| {
                    let q = mesh.positions[t[k] as usize];
                    [q.x().to_bits(), q.y().to_bits(), q.z().to_bits()]
                });
                key.sort_unstable();
                key
            })
            .collect()
    };
    let old: std::collections::BTreeSet<_> = corners(&plain).into_iter().collect();
    let changed_deviation = repaired
        .indices
        .chunks_exact(3)
        .zip(corners(&repaired))
        .filter(|(_, key)| !old.contains(key))
        .map(|(t, _)| {
            dense_deviation(
                &carrier,
                [0, 1, 2].map(|k| repaired.positions[t[k] as usize]),
            )
        })
        .fold(0.0_f64, f64::max);
    Outcome {
        changed_deviation,
        label,
        repaired: read_mesh(&repaired, &carrier, sign, area),
        plain: read_mesh(&plain, &carrier, sign, area),
        unchanged,
        plain_fingerprint: fingerprint(&plain),
    }
}

/// The never-worse contract of one outcome: the plain mesh, or one with
/// strictly fewer inverted triangles whose every triangle the plain mesh
/// does not have lies within the deflection; a manifold mesh with one
/// boundary cycle either way.
fn contract_failures(outcome: &Outcome, deflection: f64) -> Vec<String> {
    let mut failures: Vec<String> = outcome
        .repaired
        .structure
        .iter()
        .map(|defect| format!("{}: {defect}", outcome.label))
        .collect();
    if !outcome.unchanged {
        if outcome.repaired.inverted >= outcome.plain.inverted {
            failures.push(format!(
                "{}: repaired mesh inverts {} triangles, the plain one {}",
                outcome.label, outcome.repaired.inverted, outcome.plain.inverted
            ));
        }
        if outcome.changed_deviation > deflection {
            failures.push(format!(
                "{}: a repaired triangle lies {} from the cone, {:.3} x the deflection",
                outcome.label,
                outcome.changed_deviation,
                outcome.changed_deviation / deflection
            ));
        }
    }
    failures
}

fn report(outcome: &Outcome, deflection: f64) {
    let (r, p) = (&outcome.repaired, &outcome.plain);
    eprintln!(
        "{}: {} {} triangles {} inverted, deviation {:.3} x, area {:.3} x (plain {} triangles {} inverted, deviation {:.3} x, area {:.3} x)",
        outcome.label,
        if outcome.unchanged {
            "plain"
        } else {
            "repaired"
        },
        r.triangles,
        r.inverted,
        r.deviation / deflection,
        r.area_ratio,
        p.triangles,
        p.inverted,
        p.deviation / deflection,
        p.area_ratio
    );
}

/// Mesh one band in all four (nappe, orientation) variants and collect
/// every contract failure, with the outcomes.
fn check_band(
    apex: Point3,
    a: f64,
    span: f64,
    slant: (f64, f64),
    deflection: f64,
) -> (Vec<String>, Vec<Outcome>) {
    let mut failures = Vec::new();
    let mut outcomes = Vec::new();
    for lower in [false, true] {
        for reversed in [false, true] {
            let outcome = mesh_band(apex, a, span, slant, 1, deflection, lower, reversed);
            report(&outcome, deflection);
            failures.extend(contract_failures(&outcome, deflection));
            outcomes.push(outcome);
        }
    }
    (failures, outcomes)
}

const ORIGIN: Point3 = Point3::new(0.0, 0.0, 0.0);

/// Round 1 of the review: half-angle 0.15, longitude `0..0.5`, slant
/// `1..1.0001`, deflection 0.005, on both nappes and wound either way. The
/// plain CDT turns triangles over; the repair removes every one, within
/// the deflection.
#[test]
fn a_narrow_cone_band_meshes_without_inverted_triangles() {
    let (failures, outcomes) = check_band(ORIGIN, 0.15, 0.5, (1.0, 1.0001), 0.005);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    for outcome in &outcomes {
        assert!(outcome.plain.inverted > 0, "{}", outcome.label);
        assert_eq!(outcome.repaired.inverted, 0, "{}", outcome.label);
    }
}

/// Round 2, case (a): a wide, narrow band at the origin whose rims take
/// three chords, at a coarse deflection.
#[test]
fn a_wide_narrow_band_at_coarse_deflection_meshes_within_its_deflection() {
    let (failures, _) = check_band(ORIGIN, 0.74, 3.0, (1.0, 1.016), 0.096);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Round 2, case (b): a band `1e8` out along each axis, `1e-5` wide.
#[test]
fn a_far_narrow_band_meshes_without_inverted_triangles() {
    let apex = Point3::new(1e8, 1e8, 1e8);
    let (failures, _) = check_band(apex, 0.05, 0.5, (1.0, 1.00001), 0.005);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Round 2, case (c): a steep band `1e9` out.
#[test]
fn a_far_steep_band_meshes_within_its_deflection() {
    let apex = Point3::new(1e9, 1e9, 1e9);
    let (failures, _) = check_band(apex, 1.1, 0.78, (1.0, 1.001), 0.001);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Round 3, case 1: half-angle 0.4, longitude `0..2.5`, slant
/// `1..1.0003`, deflection 0.001. Dropping the folded points left a facet
/// of area 0.38 lying 0.184 from the cone (the plain mesh: 0.00038); the
/// gate refuses that repair, so the face keeps a mesh within the deflection
/// (the plain one, or a repaired one that is).
#[test]
fn a_repair_that_leaves_the_deflection_is_refused() {
    let deflection = 0.001;
    let (failures, outcomes) = check_band(ORIGIN, 0.4, 2.5, (1.0, 1.0003), deflection);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    for outcome in &outcomes {
        assert!(
            outcome.unchanged || outcome.changed_deviation <= deflection,
            "{}",
            outcome.label
        );
        assert!(
            outcome.repaired.deviation <= outcome.plain.deviation.max(deflection),
            "{}",
            outcome.label
        );
    }
}

/// Round 3, case 2: half-angle 1.1, longitude `0..3`, slant `1..1.001`,
/// deflection 0.001. The plain CDT turns nothing over; the earlier
/// "standing" test stripped its vertices and left it 26% past the
/// deflection. Now it is the plain mesh, `main`'s, bit for bit.
#[test]
fn a_band_with_no_turned_over_triangle_keeps_its_mesh() {
    let (failures, outcomes) = check_band(ORIGIN, 1.1, 3.0, (1.0, 1.001), 0.001);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    for outcome in &outcomes {
        assert_eq!(outcome.plain.inverted, 0, "{}", outcome.label);
        assert!(outcome.unchanged, "{}", outcome.label);
    }
}

/// Round 3, case 4, a known limitation: half-angle 0.03, longitude
/// `0..1.25`, slant `1..1.0005`, deflection 0.001. The repair cannot clear
/// every turned-over triangle within the deflection; the gate keeps the
/// result no worse than the plain mesh. Left for a length-true chart (see
/// the B289 follow-up issue).
#[test]
fn a_shallow_band_keeps_no_more_inversions_than_the_plain_mesh() {
    let (failures, outcomes) = check_band(ORIGIN, 0.03, 1.25, (1.0, 1.0005), 0.001);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    for outcome in &outcomes {
        assert!(
            outcome.repaired.inverted <= outcome.plain.inverted,
            "{}",
            outcome.label
        );
    }
}

/// Band widths from 1e-1 down to 1e-5 of the slant, over half-angles 0.05
/// to 1.4 rad, three deflections and an apex at the origin and `1e8` out,
/// on both nappes and either orientation: every mesh keeps the contract,
/// and none keeps a turned-over triangle (the plain meshes turn 512 over).
#[test]
fn narrow_cone_bands_keep_the_contract_over_widths_and_angles() {
    let mut failures = Vec::new();
    let (mut plain, mut left, mut repaired) = (0, 0, 0);
    for apex in [ORIGIN, Point3::new(1e8, 1e8, 1e8)] {
        for a in [0.05, 0.15, 0.4, 0.8, 1.1, 1.4] {
            for width in [1e-1, 1e-2, 1e-3, 1e-4, 1e-5] {
                for deflection in [0.005, 0.05, 0.096] {
                    let (found, outcomes) =
                        check_band(apex, a, 0.5, (1.0, 1.0 + width), deflection);
                    failures.extend(found);
                    for outcome in outcomes {
                        plain += outcome.plain.inverted;
                        left += outcome.repaired.inverted;
                        repaired += usize::from(!outcome.unchanged);
                    }
                }
            }
        }
    }
    eprintln!("inverted triangles: {plain} plain, {left} left; {repaired} of 720 meshes repaired");
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    assert!(plain > 0);
    assert_eq!(left, 0, "{left} of {plain} inverted triangles left");
}

/// A band whose boundary forces a turned-over triangle: the near rim split
/// into 16 arcs, so its samples stand between the far rim's few, on a band
/// narrower than the far chords' lens. What turns over is made of boundary
/// samples alone, which the repair may not move. Reported, not fixed: the
/// mesh keeps the contract, and every turned-over triangle left has only
/// boundary vertices.
#[test]
fn a_boundary_that_forces_a_turned_over_triangle_is_left_and_reported() {
    let (a, span, slant, deflection) = (0.4, 0.5, (1.0, 1.0001), 0.005);
    let mut failures = Vec::new();
    for lower in [false, true] {
        for reversed in [false, true] {
            let outcome = mesh_band(ORIGIN, a, span, slant, 16, deflection, lower, reversed);
            report(&outcome, deflection);
            failures.extend(contract_failures(&outcome, deflection));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Cone faces no triangle of which turns over mesh bit for bit as on
/// `main`: the plain mesh's fingerprint for each, captured from `main`
/// (`bccdad52`, the plain path run there) before the repair existed, and the repaired mesh equal to
/// it. Wide bands on either nappe, a reversed one, case 2 above, and the
/// faces of a primitive cone solid.
#[test]
fn clean_cone_faces_mesh_bit_for_bit_as_on_main() {
    let cases = [
        (
            0.5,
            1.0,
            (1.0, 2.0),
            0.01,
            false,
            false,
            0x9fb1_495d_daa9_222e,
        ),
        (
            0.3,
            2.0,
            (0.5, 3.0),
            0.005,
            true,
            true,
            0xa4c4_7812_04d9_3734,
        ),
        (
            0.8,
            4.0,
            (2.0, 2.5),
            0.002,
            false,
            true,
            0x5783_c773_69a0_8b63,
        ),
        (
            1.1,
            3.0,
            (1.0, 1.001),
            0.001,
            false,
            false,
            0xbb52_fbe8_6d4c_29cd,
        ),
    ];
    let mut found = Vec::new();
    for (a, span, slant, deflection, lower, reversed, expected) in cases {
        let outcome = mesh_band(ORIGIN, a, span, slant, 1, deflection, lower, reversed);
        assert_eq!(outcome.plain.inverted, 0, "{}", outcome.label);
        assert!(outcome.unchanged, "{}", outcome.label);
        found.push((outcome.plain_fingerprint, expected));
    }
    let mut topo = Topology::new();
    let solid = crate::primitives::make_cone(&mut topo, 2.0, 0.5, 3.0).unwrap();
    for (deflection, expected) in [
        (0.01, 0x8b2b_c638_143c_fb7e_u64),
        (0.001, 0x73f3_af03_8652_fc05),
    ] {
        let mesh = crate::tessellate::tessellate_solid(&topo, solid, deflection).unwrap();
        found.push((fingerprint(&mesh), expected));
    }
    eprintln!("fingerprints (found, expected): {found:#x?}");
    assert!(found.iter().all(|(found, expected)| found == expected));
}
