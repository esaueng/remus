#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs
)]
//! Cross-process determinism sweep.
//!
//! Prints one fingerprint line per scenario. Run the binary several times and
//! diff the output: any line that differs between PROCESSES is seed-dependent
//! (std `HashMap` iteration order leaking into branching).

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::measure::solid_volume;
use brepkit_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::solid::SolidId;

const DEFL: f64 = 0.05;

/// Strong, order-independent fingerprint: per-face surface type, orientation,
/// inner-wire COUNT (the field that exposed the shell rim bug), and the sorted
/// quantised vertex set of every wire.
fn fingerprint(topo: &Topology, solid: SolidId) -> String {
    let mut items: Vec<String> = Vec::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let f = topo.face(fid).unwrap();
        let mut wires: Vec<String> = Vec::new();
        for wid in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            let mut pts: Vec<(i64, i64, i64)> = Vec::new();
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                for v in [e.start(), e.end()] {
                    let p = topo.vertex(v).unwrap().point();
                    pts.push((
                        (p.x() * 1e4).round() as i64,
                        (p.y() * 1e4).round() as i64,
                        (p.z() * 1e4).round() as i64,
                    ));
                }
            }
            pts.sort_unstable();
            pts.dedup();
            wires.push(format!("{}#{:?}", pts.len(), pts));
        }
        wires.sort_unstable();
        items.push(format!(
            "{}/{}/{}/{}",
            f.surface().type_tag(),
            f.is_reversed(),
            f.inner_wires().len(),
            wires.join(",")
        ));
    }
    items.sort_unstable();
    let mut h: u64 = 1_469_598_103_934_665_603;
    for b in items.join("|").bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(1_099_511_628_211);
    }
    format!("{h:016x}")
}

fn report(name: &str, topo: &Topology, solid: SolidId) {
    let faces = solid_faces(topo, solid).unwrap();
    let inner: usize = faces
        .iter()
        .map(|&f| topo.face(f).unwrap().inner_wires().len())
        .sum();
    let vol = solid_volume(topo, solid, DEFL).unwrap_or(f64::NAN);
    println!(
        "{name:<34} fp={} faces={} inner={} vol={vol:.6}",
        fingerprint(topo, solid),
        faces.len(),
        inner
    );
}

/// An axis-aligned square face at height `z`, for lofting a NURBS-walled solid.
fn square_profile(topo: &mut Topology, half: f64, z: f64) -> brepkit_topology::face::FaceId {
    use brepkit_math::vec::{Point3, Vec3};
    use brepkit_topology::edge::{Edge, EdgeCurve};
    use brepkit_topology::face::{Face, FaceSurface};
    use brepkit_topology::vertex::Vertex;
    use brepkit_topology::wire::{OrientedEdge, Wire};

    let corners = [(-half, -half), (half, -half), (half, half), (-half, half)];
    let v: Vec<_> = corners
        .iter()
        .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, z), 1e-7)))
        .collect();
    let edges: Vec<_> = (0..4)
        .map(|i| {
            let e = topo.add_edge(Edge::new(v[i], v[(i + 1) % 4], EdgeCurve::Line));
            OrientedEdge::new(e, true)
        })
        .collect();
    let wid = topo.add_wire(Wire::new(edges, true).unwrap());
    topo.add_face(Face::new(
        wid,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: z,
        },
    ))
}

fn at(topo: &mut Topology, s: SolidId, x: f64, y: f64, z: f64) -> SolidId {
    transform_solid(topo, s, &Mat4::translation(x, y, z)).unwrap();
    s
}

fn main() {
    // --- primitives ---
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 10.0, 8.0, 6.0).unwrap();
        report("box", &t, b);
        let c = make_cylinder(&mut t, 4.0, 9.0).unwrap();
        report("cylinder", &t, c);
        let s = make_sphere(&mut t, 5.0, 24).unwrap();
        report("sphere", &t, s);
        let k = make_cone(&mut t, 5.0, 2.0, 8.0).unwrap();
        report("cone", &t, k);
        let tr = make_torus(&mut t, 8.0, 2.0, 24).unwrap();
        report("torus", &t, tr);
    }

    // --- booleans on primitives ---
    for (name, op) in [
        ("cut", BooleanOp::Cut),
        ("fuse", BooleanOp::Fuse),
        ("intersect", BooleanOp::Intersect),
    ] {
        let mut t = Topology::new();
        let b = make_box(&mut t, 20.0, 20.0, 10.0).unwrap();
        let c = make_cylinder(&mut t, 4.0, 20.0).unwrap();
        at(&mut t, c, 10.0, 10.0, -5.0);
        match boolean(&mut t, op, b, c) {
            Ok(r) => report(&format!("box_{name}_cylinder"), &t, r),
            Err(e) => println!("box_{name}_cylinder ERR {e}"),
        }
    }

    // --- sequential booleans (residue accumulates) ---
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 40.0, 40.0, 10.0).unwrap();
        let d1 = make_cylinder(&mut t, 3.0, 10.0).unwrap();
        at(&mut t, d1, 20.0, 20.0, 0.0);
        let s1 = boolean(&mut t, BooleanOp::Cut, b, d1).unwrap();
        report("drilled", &t, s1);
        let d2 = make_cylinder(&mut t, 5.0, 10.0).unwrap();
        at(&mut t, d2, 20.0, 20.0, 0.0);
        let s2 = boolean(&mut t, BooleanOp::Cut, s1, d2).unwrap();
        report("drilled_rebored", &t, s2);
    }

    // --- shell_op, closed and open ---
    for (name, radius, wall) in [("shell_cyl", 10.0, 1.2), ("shell_cyl_thin", 10.0, 0.4)] {
        let mut t = Topology::new();
        let c = make_cylinder(&mut t, radius, 16.0).unwrap();
        let top: Vec<_> = solid_faces(&t, c)
            .unwrap()
            .into_iter()
            .filter(|&f| {
                t.face(f)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| (n.z() - 1.0).abs() < 1e-6)
            })
            .collect();
        match brepkit_operations::shell_op::shell(&mut t, c, wall, &top) {
            Ok(s) => report(name, &t, s),
            Err(e) => println!("{name} ERR {e}"),
        }
    }
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 20.0, 14.0, 10.0).unwrap();
        let top: Vec<_> = solid_faces(&t, b)
            .unwrap()
            .into_iter()
            .filter(|&f| {
                t.face(f)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| (n.z() - 1.0).abs() < 1e-6)
            })
            .collect();
        match brepkit_operations::shell_op::shell(&mut t, b, 1.0, &top) {
            Ok(s) => report("shell_box", &t, s),
            Err(e) => println!("shell_box ERR {e}"),
        }
    }

    // --- fuse into a shelled cavity (the historically unstable case) ---
    {
        let mut t = Topology::new();
        let c = make_cylinder(&mut t, 10.0, 16.0).unwrap();
        let top: Vec<_> = solid_faces(&t, c)
            .unwrap()
            .into_iter()
            .filter(|&f| {
                t.face(f)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| (n.z() - 1.0).abs() < 1e-6)
            })
            .collect();
        let cup = brepkit_operations::shell_op::shell(&mut t, c, 1.2, &top).unwrap();
        let ro = make_cylinder(&mut t, 7.0, 3.0).unwrap();
        at(&mut t, ro, 0.0, 0.0, 13.0);
        let ri = make_cylinder(&mut t, 5.0, 3.0).unwrap();
        at(&mut t, ri, 0.0, 0.0, 13.0);
        let ring = boolean(&mut t, BooleanOp::Cut, ro, ri).unwrap();
        match boolean(&mut t, BooleanOp::Fuse, cup, ring) {
            Ok(r) => report("ring_in_cup", &t, r),
            Err(e) => println!("ring_in_cup ERR {e}"),
        }
    }

    // --- direct edits ---
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 40.0, 40.0, 10.0).unwrap();
        let d = make_cylinder(&mut t, 3.0, 10.0).unwrap();
        at(&mut t, d, 20.0, 20.0, 0.0);
        let drilled = boolean(&mut t, BooleanOp::Cut, b, d).unwrap();
        let bore = solid_faces(&t, drilled)
            .unwrap()
            .into_iter()
            .find(|&f| {
                matches!(
                    t.face(f).unwrap().surface(),
                    brepkit_topology::face::FaceSurface::Cylinder(_)
                )
            })
            .unwrap();
        match brepkit_operations::push_pull::resize_cylindrical_face(&mut t, drilled, bore, 5.0) {
            Ok(r) => report("resize_bore_up", &t, r),
            Err(e) => println!("resize_bore_up ERR {e}"),
        }
    }

    // --- blends ---
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 20.0, 20.0, 10.0).unwrap();
        let edges = brepkit_topology::explorer::solid_edges(&t, b).unwrap();
        match brepkit_operations::blend_ops::fillet_v2(&mut t, b, &edges[..1], 1.5) {
            Ok(r) => report("fillet_one_edge", &t, r.solid),
            Err(e) => println!("fillet_one_edge ERR {e}"),
        }
    }
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 20.0, 20.0, 10.0).unwrap();
        let edges = brepkit_topology::explorer::solid_edges(&t, b).unwrap();
        match brepkit_operations::blend_ops::chamfer_v2(&mut t, b, &edges[..1], 1.0, 1.0) {
            Ok(r) => report("chamfer_one_edge", &t, r.solid),
            Err(e) => println!("chamfer_one_edge ERR {e}"),
        }
    }

    // --- many sequential cuts (residue + assembly stress) ---
    {
        let mut t = Topology::new();
        let mut acc = make_box(&mut t, 60.0, 60.0, 10.0).unwrap();
        for row in 0..3 {
            for col in 0..3 {
                let c = make_cylinder(&mut t, 2.0, 20.0).unwrap();
                at(
                    &mut t,
                    c,
                    10.0 + f64::from(col) * 20.0,
                    10.0 + f64::from(row) * 20.0,
                    -5.0,
                );
                acc = boolean(&mut t, BooleanOp::Cut, acc, c).unwrap();
            }
        }
        report("nine_cuts", &t, acc);
    }

    // --- offsets, including the NURBS case that only reports an error ---
    //
    // The error TEXT is part of the fingerprint here on purpose. Offset names
    // the first face pair it fails to intersect, and it walked the face pairs
    // in `edge_to_face_map` order — so an unordered map made a failing offset
    // blame a different pair in each process while the verdict stayed put.
    for (name, distance) in [("offset_cyl_out", 0.4), ("offset_cyl_in", -0.4)] {
        let mut t = Topology::new();
        let c = make_cylinder(&mut t, 6.0, 12.0).unwrap();
        match brepkit_operations::offset_v2::offset_solid_v2(&mut t, c, distance) {
            Ok(r) => report(name, &t, r),
            Err(e) => println!("{name} ERR {e}"),
        }
    }
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 10.0, 8.0, 6.0).unwrap();
        match brepkit_operations::offset_v2::offset_solid_v2(&mut t, b, 0.5) {
            Ok(r) => report("offset_box", &t, r),
            Err(e) => println!("offset_box ERR {e}"),
        }
    }
    {
        let mut t = Topology::new();
        // Same three-square loft the approx_census "nurbs-loft" row builds.
        let profiles: Vec<_> = [(3.0, 0.0), (1.5, 5.0), (3.0, 10.0)]
            .iter()
            .map(|&(half, z)| square_profile(&mut t, half, z))
            .collect();
        match brepkit_operations::loft::loft_smooth(&mut t, &profiles) {
            Ok(s) => match brepkit_operations::offset_v2::offset_solid_v2(&mut t, s, 0.5) {
                Ok(r) => report("offset_nurbs_loft", &t, r),
                Err(e) => println!("offset_nurbs_loft ERR {e}"),
            },
            Err(e) => println!("offset_nurbs_loft LOFT_ERR {e}"),
        }
    }

    // --- tessellation ---
    {
        let mut t = Topology::new();
        let b = make_box(&mut t, 20.0, 20.0, 10.0).unwrap();
        let c = make_cylinder(&mut t, 4.0, 20.0).unwrap();
        at(&mut t, c, 10.0, 10.0, -5.0);
        let r = boolean(&mut t, BooleanOp::Cut, b, c).unwrap();
        let m = brepkit_operations::tessellate::tessellate_solid(&t, r, DEFL).unwrap();
        println!(
            "tessellate_drilled_box             tris={} verts={}",
            m.indices.len() / 3,
            m.positions.len() / 3
        );
    }
}
