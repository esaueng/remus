//! A cone wall written on its carrier's lower nappe (`v < 0`) meshes the
//! same wall as on the upper nappe.
//!
//! `ConicalSurface` is a double cone: at `v < 0` its chart puts the point of
//! longitude `u` half a turn round the axis from where `project_point` says,
//! so a wall below its carrier's apex, charted by projection and read back
//! by evaluation, meshed its own mirror image across the axis: the half
//! frustum here enclosed 1.23 for 6.54 (scale 1, deflection 0.01).
//!
//! Bodies: a frustum of bottom radius `2s`, top radius `s`, height `2s`,
//! bored to radius `s/2` (so no face collapses onto the axis), revolved a
//! quarter, a half and a whole turn about `ẑ`, with its cone walls on
//! either carrier. Oracles, independent of `ConicalSurface`: the
//! physical wall `r = 2s − z/2`, `0 ≤ z ≤ 2s`, inside the revolved sector,
//! and its outward normal `∝ r̂ + ẑ/2`; the volume from `solid_volume`
//! (itself held to the closed form by `regress_cone_lower_nappe_measure`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::vec::{Point3, Vec3};
use remus_operations::measure;
use remus_operations::revolve::revolve;
use remus_operations::tessellate::{
    TriangleMesh, edge_census, tessellate, tessellate_solid_grouped_with_tolerance,
};
use remus_topology::Topology;
use remus_topology::builder::make_polygon_wire;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// The frustum's trapezoid profile revolved by `sweep` about `ẑ`; with
/// `lower` its cone walls are written on the reversed-axis carrier, where
/// they lie below the apex.
fn frustum(s: f64, sweep: f64, lower: bool) -> (Topology, SolidId, Vec<FaceId>) {
    let mut topo = Topology::new();
    let profile = make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.5 * s, 0.0, 0.0),
            Point3::new(2.0 * s, 0.0, 0.0),
            Point3::new(s, 0.0, 2.0 * s),
            Point3::new(0.5 * s, 0.0, 2.0 * s),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(
        profile,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, -1.0, 0.0),
            d: 0.0,
        },
    ));
    let solid = revolve(
        &mut topo,
        face,
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        sweep,
    )
    .unwrap();
    let walls: Vec<_> = solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cone(_)))
        .collect();
    assert!(!walls.is_empty(), "a cone wall");
    for &wall in &walls {
        let FaceSurface::Cone(cone) = topo.face(wall).unwrap().surface().clone() else {
            unreachable!()
        };
        let carrier = if lower {
            cone.with_reversed_axis()
        } else {
            cone
        };
        let wire = topo.wire(topo.face(wall).unwrap().outer_wire()).unwrap();
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            let (_, v) = carrier.project_point(topo.vertex(edge.start()).unwrap().point());
            assert_eq!(v < 0.0, lower, "the wall's nappe on its carrier");
        }
        topo.face_mut(wall)
            .unwrap()
            .set_surface(FaceSurface::Cone(carrier));
    }
    (topo, solid, walls)
}

/// What is wrong with one wall vertex, if anything: off the physical wall
/// `r = 2s − z/2` or outside its trim (`0 ≤ z ≤ 2s`, longitude in
/// `[0, sweep]`).
fn wall_vertex_defect(p: Point3, s: f64, sweep: f64) -> Option<String> {
    let tol = 1e-9 * s;
    let r = p.x().hypot(p.y());
    let off = (r - (2.0 * s - p.z() / 2.0)).abs();
    if off > tol {
        return Some(format!("{p:?} is {off} off the wall"));
    }
    if p.z() < -tol || p.z() > 2.0 * s + tol {
        return Some(format!("{p:?} is outside the wall's height"));
    }
    if sweep < TAU {
        let phi = p.y().atan2(p.x());
        // Longitude in [0, 2π), with the seam at 0 read on either side.
        let phi = if phi < -tol / r {
            phi + TAU
        } else {
            phi.max(0.0)
        };
        if phi > sweep + tol / r && phi < TAU - tol / r {
            return Some(format!("{p:?} at longitude {phi} is outside [0, {sweep}]"));
        }
    }
    None
}

/// The frustum wall's outward normal direction at (or radially beside) `p`.
fn outward_at(p: Point3) -> Vec3 {
    let radial = Vec3::new(p.x(), p.y(), 0.0).normalize().unwrap();
    radial + Vec3::new(0.0, 0.0, 0.5)
}

/// Signed volume and area of a closed triangle mesh.
fn volume_and_area(mesh: &TriangleMesh) -> (f64, f64) {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let (mut six, mut twice_area) = (0.0, 0.0);
    for tri in mesh.indices.chunks_exact(3) {
        let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize] - origin);
        six += a.dot(b.cross(c));
        twice_area += (b - a).cross(c - a).length();
    }
    (six / 6.0, twice_area / 2.0)
}

/// Every check on one body's grouped solid mesh at `deflection`.
fn solid_mesh_defects(
    label: &str,
    s: f64,
    sweep: f64,
    lower: bool,
    deflection: f64,
) -> Vec<String> {
    let mut failures = Vec::new();
    let (topo, solid, walls) = frustum(s, sweep, lower);
    let (mesh, offsets) =
        tessellate_solid_grouped_with_tolerance(&topo, solid, deflection, 0.5).unwrap();
    let census = edge_census(&mesh);
    if census.boundary_edges != 0 || census.non_manifold_edges != 0 {
        failures.push(format!(
            "{label}: {} boundary and {} non-manifold edges",
            census.boundary_edges, census.non_manifold_edges
        ));
    }
    let expected = measure::solid_volume(&topo, solid, 1e-3 * s).unwrap();
    let (volume, area) = volume_and_area(&mesh);
    // The same wall on the other carrier encloses the same volume.
    let (other_topo, other, _) = frustum(s, sweep, !lower);
    let (other_mesh, _) =
        tessellate_solid_grouped_with_tolerance(&other_topo, other, deflection, 0.5).unwrap();
    let other_volume = volume_and_area(&other_mesh).0;
    // Vertex for vertex, normals included (rim normals sum the faces'
    // carrier normals at the point).
    if other_mesh.positions.len() == mesh.positions.len() {
        let mut differ = 0;
        for k in 0..mesh.positions.len() {
            if (mesh.positions[k] - other_mesh.positions[k]).length() > 1e-12 * s
                || (mesh.normals[k] - other_mesh.normals[k]).length() > 1e-9
            {
                differ += 1;
            }
        }
        if differ != 0 {
            failures.push(format!(
                "{label}: {differ} vertices or normals differ from the other carrier's mesh"
            ));
        }
    } else {
        failures.push(format!(
            "{label}: {} vertices vs {} on the other carrier",
            mesh.positions.len(),
            other_mesh.positions.len()
        ));
    }
    if (volume - other_volume).abs() > 1e-9 * expected {
        failures.push(format!(
            "{label}: mesh volume {volume} vs {other_volume} on the other carrier"
        ));
    }
    // Pre-existing, on either carrier: a cone's interior grid is sized for
    // radius 1 (`estimate_surface_radius`), so a wall of radius 200 meshed at
    // 0.1 keeps a sparse middle row and encloses about 1% too little. That
    // case is held to the other carrier above, not to the chord slab.
    let sparse_grid = s > 1.0 && deflection < 1e-2 * s;
    if !sparse_grid && (volume - expected).abs() > deflection.mul_add(area, 1e-9 * expected) {
        failures.push(format!(
            "{label}: mesh volume {volume} vs solid_volume {expected} (slab {})",
            deflection * area
        ));
    }
    let faces = solid_faces(&topo, solid).unwrap();
    let origin = Point3::new(0.0, 0.0, 0.0);
    let (mut off_wall, mut inverted, mut wrong_normals, mut triangles) = (Vec::new(), 0, 0, 0);
    for (index, face) in faces.iter().enumerate() {
        if !walls.contains(face) {
            continue;
        }
        let (start, end) = (offsets[index] as usize, offsets[index + 1] as usize);
        for tri in mesh.indices[start..end].chunks_exact(3) {
            triangles += 1;
            let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize]);
            for p in [a, b, c] {
                if let Some(defect) = wall_vertex_defect(p, s, sweep) {
                    off_wall.push(defect);
                }
            }
            let centroid = origin + ((a - origin) + (b - origin) + (c - origin)) * (1.0 / 3.0);
            if (b - a).cross(c - a).dot(outward_at(centroid)) <= 0.0 {
                inverted += 1;
            }
            // A vertex off the wall's rims carries the wall's own normal.
            for &index in tri {
                let p = mesh.positions[index as usize];
                let rim = p.z().abs() < 1e-9 * s
                    || (p.z() - 2.0 * s).abs() < 1e-9 * s
                    || (sweep < TAU
                        && (p.y().abs() < 1e-9 * s || p.y().atan2(p.x()) >= sweep - 1e-9));
                let want = outward_at(p).normalize().unwrap();
                let got = mesh.normals[index as usize];
                if !rim && got.dot(want) < 0.999_999 * got.length() {
                    wrong_normals += 1;
                }
            }
        }
    }
    if wrong_normals != 0 {
        failures.push(format!(
            "{label}: {wrong_normals} wall vertex normals off the wall's normal"
        ));
    }
    if triangles == 0 {
        failures.push(format!("{label}: no wall triangles"));
    }
    if inverted != 0 {
        failures.push(format!(
            "{label}: {inverted} of {triangles} wall triangles wound inward"
        ));
    }
    if let Some(first) = off_wall.first() {
        failures.push(format!(
            "{label}: {} wall vertices off the wall, first {first}",
            off_wall.len()
        ));
    }
    failures
}

const SWEEPS: [(&str, f64); 3] = [("quarter", PI / 2.0), ("half", PI), ("whole", TAU)];

#[test]
fn a_lower_nappe_frustum_meshes_closed_on_its_own_wall() {
    let mut failures = Vec::new();
    for s in [1e-2, 1.0, 1e2] {
        for (name, sweep) in SWEEPS {
            for lower in [false, true] {
                for deflection in [1e-2 * s, 1e-3 * s] {
                    let label = format!(
                        "{name} frustum, scale {s}, lower nappe {lower}, deflection {deflection}"
                    );
                    failures.extend(solid_mesh_defects(&label, s, sweep, lower, deflection));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The standalone face mesher (`tessellate`): every vertex on the wall and
/// inside its trim, every triangle wound along the face's outward normal,
/// and the mesh area the closed-form wall area to its chords.
#[test]
fn a_lower_nappe_wall_meshes_alone_on_its_own_wall() {
    let mut failures = Vec::new();
    for s in [1e-2, 1.0, 1e2] {
        for (name, sweep) in SWEEPS {
            for lower in [false, true] {
                let label = format!("{name} wall, scale {s}, lower nappe {lower}");
                let (topo, _, walls) = frustum(s, sweep, lower);
                let mut area = 0.0;
                for &wall in &walls {
                    let mesh = tessellate(&topo, wall, 1e-3 * s).unwrap();
                    area += volume_and_area(&mesh).1;
                    let origin = Point3::new(0.0, 0.0, 0.0);
                    let mut inverted = 0;
                    for tri in mesh.indices.chunks_exact(3) {
                        let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize]);
                        for p in [a, b, c] {
                            if let Some(defect) = wall_vertex_defect(p, s, sweep) {
                                failures.push(format!("{label}: {defect}"));
                            }
                        }
                        let centroid =
                            origin + ((a - origin) + (b - origin) + (c - origin)) * (1.0 / 3.0);
                        if (b - a).cross(c - a).dot(outward_at(centroid)) <= 0.0 {
                            inverted += 1;
                        }
                    }
                    if inverted != 0 {
                        failures.push(format!("{label}: {inverted} triangles wound inward"));
                    }
                }
                // Lateral area of the swept frustum wall: sweep/2 (R + r) l.
                let want = sweep / 2.0 * (3.0 * s) * (2.0 * s).hypot(s);
                if (area - want).abs() > 1e-2 * want {
                    failures.push(format!("{label}: wall mesh area {area} vs {want}"));
                }
            }
        }
    }
    failures.truncate(40);
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// `effective_face_normal` reads a lower-nappe wall's own outward normal
/// (it read the axial part turned over), so the edge relations around the
/// wall match the upper carrier's, dihedral angles included.
#[test]
fn a_lower_nappe_wall_reads_its_own_normal_and_edge_relations() {
    use remus_operations::query::{effective_face_normal, solid_edge_relations};
    let mut failures = Vec::new();
    for s in [1e-2, 1.0, 1e2] {
        for (name, sweep) in SWEEPS {
            let (upper_topo, upper, _) = frustum(s, sweep, false);
            let upper_relations = solid_edge_relations(&upper_topo, upper, None).unwrap();
            for lower in [false, true] {
                let label = format!("{name} frustum, scale {s}, lower nappe {lower}");
                let (topo, solid, walls) = frustum(s, sweep, lower);
                for &wall in &walls {
                    for (phi, z) in [(0.3, 0.5), (1.2, 1.0), (0.05, 1.9)] {
                        let phi = phi * sweep / 1.5;
                        let r = (2.0 - z / 2.0) * s;
                        let point = Point3::new(r * phi.cos(), r * phi.sin(), z * s);
                        let got = effective_face_normal(&topo, wall, point).unwrap();
                        let want = outward_at(point).normalize().unwrap();
                        if (got - want).length() > 1e-9 {
                            failures.push(format!(
                                "{label}: normal at {point:?} is {got:?}, want {want:?}"
                            ));
                        }
                    }
                }
                let relations = solid_edge_relations(&topo, solid, None).unwrap();
                if relations.len() != upper_relations.len() {
                    failures.push(format!("{label}: edge count differs"));
                }
                for (got, want) in relations.iter().zip(&upper_relations) {
                    let angle_off = match (got.dihedral_angle, want.dihedral_angle) {
                        (Some(a), Some(b)) => (a - b).abs() > 1e-9,
                        (a, b) => a.is_some() != b.is_some(),
                    };
                    if got.concavity != want.concavity || angle_off {
                        failures.push(format!("{label}: {got:?} vs the upper carrier's {want:?}"));
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// A quarter annulus `r_in ≤ r ≤ r_out`, `0 ≤ φ ≤ π/2` on the lower nappe
/// of the cone of apex `0`, axis `+ẑ` and half-angle `a` (so `z = −r tan a`),
/// written on that carrier. With `pcurve` its generator edge along `φ = 0`
/// carries its exact pcurve in that carrier's chart (`u` half a turn from
/// the edge's longitude, `v = −r / cos a`).
fn shallow_lower_quarter_wall(
    a: f64,
    r_in: f64,
    r_out: f64,
    pcurve: bool,
) -> (Topology, FaceId, remus_math::surfaces::ConicalSurface) {
    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::vec::{Point2, Vec2};
    use remus_topology::edge::{Edge, EdgeCurve};
    use remus_topology::pcurve::PCurve;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};
    let carrier = remus_math::surfaces::ConicalSurface::with_ref_dir(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        a,
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let mut topo = Topology::new();
    let z = Vec3::new(0.0, 0.0, 1.0);
    let height = |r: f64| -r * a.tan();
    let at = |r: f64, phi: f64| Point3::new(r * phi.cos(), r * phi.sin(), height(r));
    let half = PI / 2.0;
    let corners = [
        at(r_out, 0.0),
        at(r_out, half),
        at(r_in, half),
        at(r_in, 0.0),
    ];
    let vertices: Vec<_> = corners
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-12)))
        .collect();
    let mut arc = |from: usize, to: usize, r: f64| {
        let circle = Circle3D::new(Point3::new(0.0, 0.0, height(r)), z, r).unwrap();
        let t0 = circle.project(corners[from]);
        let mut t1 = circle.project(corners[to]);
        if t1 <= t0 {
            t1 += TAU;
        }
        let mut edge = Edge::new(vertices[from], vertices[to], EdgeCurve::Circle(circle));
        edge.set_trim(Some((t0, t1)));
        topo.add_edge(edge)
    };
    let outer = arc(0, 1, r_out);
    let inner = arc(3, 2, r_in);
    let side = topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line));
    let generator = topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            // The generator first, so the wire's first corner is read off
            // its pcurve.
            vec![
                OrientedEdge::new(generator, true),
                OrientedEdge::new(outer, true),
                OrientedEdge::new(side, true),
                OrientedEdge::new(inner, false),
            ],
            true,
        )
        .unwrap(),
    );
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Cone(carrier.clone())));
    for (corner, &p) in corners.iter().enumerate() {
        let (_, v) = carrier.project_point(p);
        assert!(v < 0.0, "corner {corner} below the apex");
    }
    if pcurve {
        // At v < 0 the carrier puts longitude u at −ρ(u): the edge along
        // φ = 0 is the chart line u = π.
        let cos_a = a.cos();
        let line = Line2D::new(Point2::new(PI, -r_in / cos_a), Vec2::new(0.0, -1.0)).unwrap();
        let curve = PCurve::new(Curve2D::Line(line), 0.0, (r_out - r_in) / cos_a);
        for t in [0.0, 0.5, 1.0] {
            let uv = curve.evaluate(t * curve.t_end());
            let p = carrier.evaluate(uv.x(), uv.y());
            assert!(
                p.y().abs() < 1e-12 && p.x() > 0.0,
                "the pcurve is exact: {p:?}"
            );
        }
        topo.set_pcurve(generator, face, curve).unwrap();
        assert!(topo.pcurve_oriented(generator, face, true).is_some());
    }
    (topo, face, carrier)
}

/// One face as a sheet body, without the sheet validator (its edge checks
/// are not what these tests read).
fn raw_sheet(topo: &mut Topology, face: FaceId) -> remus_topology::shell::ShellId {
    let shell = topo.add_shell(remus_topology::shell::Shell::new(vec![face]).unwrap());
    topo.set_shell_body_class(shell, remus_topology::BodyClass::Sheet)
        .unwrap();
    shell
}

/// Review finding 1: a stale chart pcurve fooled the on-surface check.
/// Evaluated on the reversed-axis carrier the generator's exact old-chart
/// pcurve lands `2 |v| sin a ≈ 2e-8` off its edge, inside the 1e-7 check,
/// so the face CDT mixed one negative-`v` corner with projected rim
/// samples and emitted vertices near radius 5e-4, far outside the wall.
/// A charted face never reads its stored pcurves.
#[test]
fn a_stale_pcurve_does_not_steer_a_lower_nappe_chart() {
    use remus_operations::tessellate::tessellate_sheet;
    let (r_in, r_out) = (1.0, 1.001);
    let mut failures = Vec::new();
    for pcurve in [false, true] {
        let (mut topo, face, _) = shallow_lower_quarter_wall(1e-8, r_in, r_out, pcurve);
        let sheet = raw_sheet(&mut topo, face);
        for deflection in [1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6] {
            let mesh = tessellate_sheet(&topo, sheet, deflection).unwrap();
            if mesh.indices.is_empty() {
                failures.push(format!("pcurve {pcurve}, {deflection}: empty mesh"));
            }
            for p in &mesh.positions {
                let r = p.x().hypot(p.y());
                let phi = p.y().atan2(p.x());
                if r < r_in - 1e-9
                    || r > r_out + 1e-9
                    || !(-1e-9..=PI / 2.0 + 1e-9).contains(&phi)
                    || (p.z() + r * 1e-8_f64.tan()).abs() > 1e-12
                {
                    failures.push(format!(
                        "pcurve {pcurve}, deflection {deflection}: vertex {p:?} (r {r}) off the wall"
                    ));
                    break;
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Review finding 2: `tessellate_with_uvs` returns parameters on the face's
/// stored carrier, so evaluating them gives the mesh vertices back, on
/// either nappe and wherever the body is placed.
#[test]
fn lower_nappe_uvs_evaluate_to_their_vertices_on_the_stored_carrier() {
    use remus_math::mat::Mat4;
    use remus_operations::tessellate::tessellate_with_uvs;
    use remus_operations::transform::transform_solid;
    let placements = [
        Mat4::translation(0.0, 0.0, 0.0),
        Mat4::translation(3.0, -7.0, 11.0),
        Mat4::translation(-2.0, 5.0, 1.0) * Mat4::rotation_x(0.7) * Mat4::rotation_z(-1.9),
    ];
    let mut failures = Vec::new();
    for s in [1e-2, 1.0, 1e2] {
        for (name, sweep) in SWEEPS {
            for lower in [false, true] {
                for (index, placement) in placements.iter().enumerate() {
                    let (mut topo, solid, walls) = frustum(s, sweep, lower);
                    let mut placed = *placement;
                    for row in 0..3 {
                        placed.0[row][3] *= s;
                    }
                    transform_solid(&mut topo, solid, &placed).unwrap();
                    let label =
                        format!("{name} wall, scale {s}, lower nappe {lower}, placement {index}");
                    for &wall in &walls {
                        let FaceSurface::Cone(cone) = topo.face(wall).unwrap().surface().clone()
                        else {
                            unreachable!()
                        };
                        let meshed = tessellate_with_uvs(&topo, wall, 1e-2 * s).unwrap();
                        let worst = meshed
                            .mesh
                            .positions
                            .iter()
                            .zip(&meshed.uvs)
                            .map(|(&p, &[u, v])| (cone.evaluate(u, v) - p).length())
                            .fold(0.0_f64, f64::max);
                        if worst > 1e-12 * s.max(1.0) * 20.0 {
                            failures
                                .push(format!("{label}: a uv evaluates {worst} off its vertex"));
                        }
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Review finding 3: on a shallow cone (half-angle 5e-10, past the normal
/// helper's old 1e-9 floor) the chart and the normal readers agree: a wall
/// on the lower nappe reads its own normal, close to `+ẑ` here, in
/// `effective_face_normal` and in its mesh.
#[test]
fn a_shallow_lower_nappe_wall_reads_the_normal_it_is_meshed_with() {
    use remus_operations::query::effective_face_normal;
    use remus_operations::tessellate::tessellate_sheet;
    let a = 5e-10;
    let (mut topo, face, _) = shallow_lower_quarter_wall(a, 1.0, 2.0, false);
    let mut failures = Vec::new();
    for (r, phi) in [(2.0, 0.3), (1.5, PI / 4.0), (1.0001, 1.5)] {
        let point = Point3::new(r * f64::cos(phi), r * f64::sin(phi), -r * a.tan());
        let normal = effective_face_normal(&topo, face, point).unwrap();
        let radial = Vec3::new(phi.cos(), phi.sin(), 0.0);
        let want = radial * a.sin() + Vec3::new(0.0, 0.0, a.cos());
        if (normal - want).length() > 1e-9 {
            failures.push(format!("normal at {point:?} is {normal:?}, want {want:?}"));
        }
    }
    let sheet = raw_sheet(&mut topo, face);
    let mesh = tessellate_sheet(&topo, sheet, 1e-3).unwrap();
    let down = mesh.normals.iter().filter(|n| n.z() < 0.99).count();
    if down != 0 {
        failures.push(format!("{down} mesh normals off +z"));
    }
    let origin = Point3::new(0.0, 0.0, 0.0);
    let wound = mesh
        .indices
        .chunks_exact(3)
        .filter(|tri| {
            let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[tri[k] as usize] - origin);
            (b - a).cross(c - a).z() <= 0.0
        })
        .count();
    if wound != 0 {
        failures.push(format!("{wound} triangles wound against +z"));
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
