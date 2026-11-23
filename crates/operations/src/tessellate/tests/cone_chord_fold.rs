//! B289 review: narrow cone bands mesh without inverted triangles.
//!
//! A cone band bounded by two circular arcs and two straight generators,
//! much narrower along its generators than its rim chords are long, is
//! triangulated in a chart whose `v` axis is stretched by `du / dv`. The
//! chord-fold guard's search band was a fixed fraction of a chord's chart
//! length, so a candidate a quarter of the stretched band away from the far
//! rim's chord was never tested, and 4 of the 22 triangles of the reviewed
//! band (half-angle 0.15, longitude `0..0.5`, slant `1..1.0001`, deflection
//! 0.005) turned over.
//!
//! Oracle, independent of the guard: every triangle's normal against the
//! carrier's closed-form normal at the triangle's centroid (one sign for the
//! whole face, opposite for a reversed face), the face mesh's half-edge
//! structure (manifold, one boundary cycle), and the band's closed-form area.

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
    a: f64,
    span: f64,
    (v0, v1): (f64, f64),
    lower: bool,
    reversed: bool,
) -> (Topology, remus_topology::shell::ShellId, ConicalSurface) {
    let carrier = ConicalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        a,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let sign = if lower { -1.0 } else { 1.0 };
    let (near, far) = (sign * v0, sign * v1);
    let mut topo = Topology::new();
    // Corners counter-clockwise about ẑ along each rim: near rim then far.
    let corner = |u: f64, v: f64| carrier.evaluate(u, v);
    let points = [
        corner(0.0, near),
        corner(span, near),
        corner(span, far),
        corner(0.0, far),
    ];
    let vertices: Vec<_> = points
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-12)))
        .collect();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let (sin_a, cos_a) = a.sin_cos();
    let mut arc = |from: usize, to: usize, v: f64| {
        let circle = Circle3D::new(Point3::new(0.0, 0.0, v * sin_a), z, v.abs() * cos_a).unwrap();
        let t0 = circle.project(points[from]);
        let mut t1 = circle.project(points[to]);
        if t1 <= t0 {
            t1 += std::f64::consts::TAU;
        }
        let mut edge = Edge::new(vertices[from], vertices[to], EdgeCurve::Circle(circle));
        edge.set_trim(Some((t0, t1)));
        topo.add_edge(edge)
    };
    let near_arc = arc(0, 1, near);
    let far_arc = arc(3, 2, far);
    let side = topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line));
    let start = topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line));
    // Counter-clockwise in the `(u, v)` chart on the upper nappe; `v`
    // falls outward below the apex, so there, and on a reversed face, the
    // wire runs the other way to keep the face on its left.
    let mut edges = vec![
        OrientedEdge::new(near_arc, true),
        OrientedEdge::new(side, true),
        OrientedEdge::new(far_arc, false),
        OrientedEdge::new(start, true),
    ];
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

/// Defects of one band's face mesh: triangles whose normal disagrees in sign
/// with the face's (the carrier's, flipped for a reversed face), degenerate triangles, a non-manifold or
/// inconsistently wound half-edge structure, more or fewer than one boundary
/// cycle, and an area off the closed form.
fn band_defects(
    mesh: &TriangleMesh,
    carrier: &ConicalSurface,
    expected_sign: f64,
    area: f64,
) -> Vec<String> {
    let mut defects = Vec::new();
    if mesh.indices.is_empty() {
        return vec!["empty mesh".to_owned()];
    }
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
    defects
}

/// Mesh one band in all four (nappe, orientation) variants and collect
/// every defect.
fn check_band(a: f64, span: f64, slant: (f64, f64), deflection: f64) -> Vec<String> {
    let mut failures = Vec::new();
    // Area of the band: the developed annular sector of opening `span cos a`.
    let area = 0.5 * span * a.cos() * slant.1.mul_add(slant.1, -(slant.0 * slant.0));
    for lower in [false, true] {
        for reversed in [false, true] {
            let (topo, sheet, carrier) = band_sheet(a, span, slant, lower, reversed);
            let label = format!(
                "a {a} span {span} slant {slant:?} deflection {deflection} lower {lower} reversed {reversed}"
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
            failures.extend(
                band_defects(&mesh, &carrier, sign, area)
                    .into_iter()
                    .map(|defect| format!("{label}: {defect}")),
            );
        }
    }
    failures
}

/// The reviewed band: half-angle 0.15, longitude `0..0.5`, slant
/// `1..1.0001`, deflection 0.005, on both nappes and wound either way.
#[test]
fn a_narrow_cone_band_meshes_without_inverted_triangles() {
    let failures = check_band(0.15, 0.5, (1.0, 1.0001), 0.005);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Band widths from 1e-1 down to 1e-5 of the slant, over half-angles 0.05
/// to 1.4 rad and two deflections: no band inverts a triangle.
#[test]
fn narrow_cone_bands_mesh_without_inverted_triangles_over_widths_and_angles() {
    let mut failures = Vec::new();
    for a in [0.05, 0.15, 0.4, 0.8, 1.1, 1.4] {
        for width in [1e-1, 1e-2, 1e-3, 1e-4, 1e-5] {
            for deflection in [0.005, 0.05] {
                failures.extend(check_band(a, 0.5, (1.0, 1.0 + width), deflection));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
