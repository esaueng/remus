//! B289: narrow cone bands mesh without inverted triangles.
//!
//! A cone band bounded by two circular arcs and two straight generators,
//! much narrower along its generators than its rim chords are long, is
//! triangulated in a chart whose `v` axis is stretched by `du / dv`. Its
//! interior points sit in the lens between the far rim's chart chords and
//! their 3D chords, so the triangles the CDT joins them into turn over; the
//! CDT now checks for that and triangulates again without them (see
//! `nonplanar::cone_fold_repair`). The cases are the review's: the narrow
//! band of round 1 and the three of round 2, which defeated two predictive
//! guards.
//!
//! Oracle, independent of the repair: every triangle's normal against the
//! carrier's closed-form normal at the triangle's centroid (one sign for the
//! whole face, opposite for a reversed face), the distance from the cone of
//! every triangle's edge midpoints and centroid in closed form, the face
//! mesh's half-edge structure (manifold, one boundary cycle), and the band's
//! closed-form area. Each case prints its triangle count, deviation and
//! area ratio (`--no-capture`).

#![allow(clippy::print_stderr)]

use remus_math::curves::Circle3D;
use remus_math::surfaces::ConicalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::tessellate::{TriangleMesh, tessellate_sheet};

/// The band `0 ≤ u ≤ span`, `v0 ≤ |v| ≤ v1` of the cone of apex `0`, axis
/// `+ẑ` and half-angle `a`, on the upper nappe or (with `lower`) the lower
/// one, as a one-face sheet; with `reversed` the face is wound against the
/// carrier. Returns the topology, the sheet, and the carrier.
fn band_sheet(
    apex: Point3,
    a: f64,
    span: f64,
    slant: (f64, f64),
    lower: bool,
    reversed: bool,
) -> (Topology, remus_topology::shell::ShellId, ConicalSurface) {
    band_sheet_split(apex, a, span, slant, 1, lower, reversed)
}

/// [`band_sheet`] with its near rim split into `near_pieces` equal arcs, so
/// it is sampled on its own and more finely than the far rim.
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

/// How far past the deflection a triangle's edge midpoints and centroid may
/// lie from the cone: not at all. The boundary chords are sampled to the
/// deflection, and dropping an interior point only joins the boundary and
/// the points left by straighter triangles across a strip of the same
/// longitude span, whose flat triangles sag no more than the chords that
/// bound them; the interior grid is sized to the deflection too. Measured:
/// at most 0.22 of the deflection over the sweep, 0.86 on the steep band
/// `1e9` out (case (c)), against 1.23 there under the lens guard.
const DEFLECTION_FACTOR: f64 = 1.0;

/// Defects of one band's face mesh: triangles whose normal disagrees in sign
/// with the face's (the carrier's, flipped for a reversed face), degenerate triangles, a non-manifold or
/// inconsistently wound half-edge structure, more or fewer than one boundary
/// cycle, and an area off the closed form.
fn band_defects(
    mesh: &TriangleMesh,
    carrier: &ConicalSurface,
    expected_sign: f64,
    area: f64,
    deflection: f64,
) -> (Vec<String>, f64) {
    let mut defects = Vec::new();
    if mesh.indices.is_empty() {
        return (vec!["empty mesh".to_owned()], 0.0);
    }
    let mut deviation = 0.0_f64;
    let mut inverted = 0;
    let mut degenerate = 0;
    let mut mesh_area = 0.0;
    let mut half_edges = std::collections::BTreeMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let [p0, p1, p2] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize]);
        let cross = (p1 - p0).cross(p2 - p0);
        let centroid = Point3::new(
            (p0.x() + p1.x() + p2.x()) / 3.0,
            (p0.y() + p1.y() + p2.y()) / 3.0,
            (p0.z() + p1.z() + p2.z()) / 3.0,
        );
        // The closed-form normal of the generator line through the
        // centroid (one vector along the whole line): `project_point` reads
        // a point below the apex at its own longitude, and the line through
        // it is the upper nappe's half a turn round.
        let (u, v) = carrier.project_point(centroid);
        let line = if v < 0.0 { u + std::f64::consts::PI } else { u };
        let normal = carrier.normal(line, 0.0);
        let along = cross.dot(normal) * expected_sign;
        mesh_area += 0.5 * cross.length();
        for point in [
            centroid,
            p0 + (p1 - p0) * 0.5,
            p1 + (p2 - p1) * 0.5,
            p2 + (p0 - p2) * 0.5,
        ] {
            deviation = deviation.max(cone_distance(carrier, point));
        }
        if cross.length() == 0.0 {
            degenerate += 1;
        } else if along <= 0.0 {
            inverted += 1;
        }
        for (i, j) in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            *half_edges.entry((i, j)).or_insert(0_usize) += 1;
        }
    }
    let triangles = mesh.indices.len() / 3;
    if inverted > 0 {
        defects.push(format!("{inverted} of {triangles} triangles inverted"));
    }
    if degenerate > 0 {
        defects.push(format!("{degenerate} degenerate triangles"));
    }
    if half_edges.values().any(|&n| n > 1) {
        defects.push("a half-edge used twice (non-manifold or folded)".to_owned());
    }
    let boundary: std::collections::BTreeMap<u32, u32> = half_edges
        .keys()
        .filter(|&&(i, j)| !half_edges.contains_key(&(j, i)))
        .map(|&(i, j)| (i, j))
        .collect();
    let boundary_count = half_edges
        .keys()
        .filter(|&&(i, j)| !half_edges.contains_key(&(j, i)))
        .count();
    if boundary.len() != boundary_count {
        defects.push("a boundary vertex leaves twice".to_owned());
    } else if let Some((&first, _)) = boundary.iter().next() {
        let mut at = first;
        let mut steps = 0;
        while let Some(&next) = boundary.get(&at) {
            at = next;
            steps += 1;
            if at == first || steps > boundary.len() {
                break;
            }
        }
        if at != first || steps != boundary.len() {
            defects.push(format!(
                "boundary is not one cycle ({steps} of {} half-edges)",
                boundary.len()
            ));
        }
    } else {
        defects.push("no boundary".to_owned());
    }
    // Inscribed triangles fall a little short of a curved band's area, or
    // exceed it where they stand tilted off the surface (a Schwarz lantern:
    // a narrow band of a sharp cone, its interior points at mid-chord, reads
    // 1.10 at half-angle 1.4 and width 1e-3, every triangle within 37
    // degrees of the surface); the folded meshes overlapped to 20 times it.
    if !(mesh_area <= area * 1.25 && mesh_area >= area * 0.98) {
        defects.push(format!("area {mesh_area} against {area}"));
    }
    if deviation > DEFLECTION_FACTOR * deflection {
        defects.push(format!(
            "a triangle edge midpoint or centroid {deviation} from the cone, {:.3} x the deflection",
            deviation / deflection
        ));
    }
    (defects, deviation / deflection)
}

/// Mesh one band in all four (nappe, orientation) variants and collect
/// every defect, with the largest deviation over the deflection.
fn check_band(
    apex: Point3,
    a: f64,
    span: f64,
    slant: (f64, f64),
    deflection: f64,
) -> (Vec<String>, f64) {
    let mut failures = Vec::new();
    let mut worst = 0.0_f64;
    // Area of the band: the developed annular sector of opening `span cos a`.
    let area = 0.5 * span * a.cos() * slant.1.mul_add(slant.1, -(slant.0 * slant.0));
    for lower in [false, true] {
        for reversed in [false, true] {
            let (topo, sheet, carrier) = band_sheet(apex, a, span, slant, lower, reversed);
            let label = format!(
                "apex {apex:?} a {a} span {span} slant {slant:?} deflection {deflection} lower {lower} reversed {reversed}"
            );
            let mesh = match tessellate_sheet(&topo, sheet, deflection) {
                Ok(mesh) => mesh,
                Err(error) => {
                    failures.push(format!("{label}: {error}"));
                    continue;
                }
            };
            // A face's orientation normal is the chart's `S_u × S_v`: the
            // line normal on the upper nappe, its opposite below the apex
            // (where `S_u` turns over with `v`), flipped for a reversed face.
            let sign = if lower == reversed { 1.0 } else { -1.0 };
            let (defects, ratio) = band_defects(&mesh, &carrier, sign, area, deflection);
            worst = worst.max(ratio);
            let mesh_area: f64 = mesh
                .indices
                .chunks_exact(3)
                .map(|t| {
                    let [p0, p1, p2] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize]);
                    0.5 * (p1 - p0).cross(p2 - p0).length()
                })
                .sum();
            eprintln!(
                "{label}: {} triangles, deviation {ratio:.3} x deflection, area {:.4} x exact, {} defects",
                mesh.indices.len() / 3,
                mesh_area / area,
                defects.len()
            );
            failures.extend(
                defects
                    .into_iter()
                    .map(|defect| format!("{label}: {defect}")),
            );
        }
    }
    (failures, worst)
}

const ORIGIN: Point3 = Point3::new(0.0, 0.0, 0.0);

/// Round 1 of the review: half-angle 0.15, longitude `0..0.5`, slant
/// `1..1.0001`, deflection 0.005, on both nappes and wound either way.
#[test]
fn a_narrow_cone_band_meshes_without_inverted_triangles() {
    let (failures, _) = check_band(ORIGIN, 0.15, 0.5, (1.0, 1.0001), 0.005);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Round 2, case (a): a wide, narrow band at the origin whose rims take
/// three chords. The lens guard dropped points that did not fold and kept
/// one the CDT joined to a chord it lies beside, outside that chord's span:
/// 4 of 26 triangles turned over and the mesh read 12.8 times the area.
#[test]
fn a_wide_narrow_band_at_coarse_deflection_meshes_within_its_deflection() {
    let (failures, _) = check_band(ORIGIN, 0.74, 3.0, (1.0, 1.016), 0.096);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Round 2, case (b): a band `1e8` out along each axis, `1e-5` wide. The
/// lens guard's finite differences across a chord rounded to one point
/// there, so it skipped the chord and one triangle turned over.
#[test]
fn a_far_narrow_band_meshes_without_inverted_triangles() {
    let apex = Point3::new(1e8, 1e8, 1e8);
    let (failures, _) = check_band(apex, 0.05, 0.5, (1.0, 1.00001), 0.005);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Round 2, case (c): a steep band `1e9` out: no triangle turned over, but
/// an interior edge midpoint lay 23% past the deflection.
#[test]
fn a_far_steep_band_meshes_within_its_deflection() {
    let apex = Point3::new(1e9, 1e9, 1e9);
    let (failures, _) = check_band(apex, 1.1, 0.78, (1.0, 1.001), 0.001);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Band widths from 1e-1 down to 1e-5 of the slant, over half-angles 0.05
/// to 1.4 rad, three deflections and an apex at the origin and `1e8` out,
/// on both nappes and either orientation: no band turns a triangle over,
/// leaves the deflection, opens its boundary or misses its area.
#[test]
fn narrow_cone_bands_mesh_without_inverted_triangles_over_widths_and_angles() {
    let mut failures = Vec::new();
    let mut worst = 0.0_f64;
    for apex in [ORIGIN, Point3::new(1e8, 1e8, 1e8)] {
        for a in [0.05, 0.15, 0.4, 0.8, 1.1, 1.4] {
            for width in [1e-1, 1e-2, 1e-3, 1e-4, 1e-5] {
                for deflection in [0.005, 0.05, 0.096] {
                    let (found, ratio) = check_band(apex, a, 0.5, (1.0, 1.0 + width), deflection);
                    failures.extend(found);
                    worst = worst.max(ratio);
                }
            }
        }
    }
    eprintln!("largest deviation over the deflection: {worst:.4}");
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// A band whose boundary forces a turned-over triangle: the near rim split
/// into 16 arcs, so its samples stand between the far rim's few, on a band
/// narrower than the far chords' lens. Every interior point is dropped, and
/// what turns over is made of boundary samples alone, which the repair may
/// not move: it is left, and the repair ends. Reported, not fixed: the mesh
/// stays closed, each turned-over triangle has only boundary vertices, and
/// the deflection holds.
#[test]
fn a_boundary_that_forces_a_turned_over_triangle_is_left_and_reported() {
    let (a, span, slant, deflection) = (0.4, 0.5, (1.0, 1.0001), 0.005);
    let mut report = Vec::new();
    for lower in [false, true] {
        for reversed in [false, true] {
            let (topo, sheet, carrier) =
                band_sheet_split(ORIGIN, a, span, slant, 16, lower, reversed);
            let mesh = tessellate_sheet(&topo, sheet, deflection).unwrap();
            let sign = if lower == reversed { 1.0 } else { -1.0 };
            let mut half_edges = std::collections::BTreeSet::new();
            for t in mesh.indices.chunks_exact(3) {
                for (i, j) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                    assert!(half_edges.insert((i, j)), "half-edge used twice");
                }
            }
            let on_boundary: std::collections::BTreeSet<u32> = half_edges
                .iter()
                .filter(|&&(i, j)| !half_edges.contains(&(j, i)))
                .flat_map(|&(i, j)| [i, j])
                .collect();
            let mut turned_over = 0;
            for t in mesh.indices.chunks_exact(3) {
                let [p0, p1, p2] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize]);
                let centroid = p0 + ((p1 - p0) + (p2 - p0)) * (1.0 / 3.0);
                let (u, v) = carrier.project_point(centroid);
                let line = if v < 0.0 { u + std::f64::consts::PI } else { u };
                if (p1 - p0).cross(p2 - p0).dot(carrier.normal(line, 0.0)) * sign <= 0.0 {
                    turned_over += 1;
                    assert!(
                        t.iter().all(|id| on_boundary.contains(id)),
                        "a turned-over triangle keeps an interior vertex"
                    );
                }
                for point in [
                    centroid,
                    p0 + (p1 - p0) * 0.5,
                    p1 + (p2 - p1) * 0.5,
                    p2 + (p0 - p2) * 0.5,
                ] {
                    let deviation = cone_distance(&carrier, point);
                    assert!(deviation <= DEFLECTION_FACTOR * deflection, "{deviation}");
                }
            }
            report.push((lower, reversed, turned_over, mesh.indices.len() / 3));
        }
    }
    eprintln!("boundary-forced turned-over triangles (lower, reversed, count, of): {report:?}");
}
