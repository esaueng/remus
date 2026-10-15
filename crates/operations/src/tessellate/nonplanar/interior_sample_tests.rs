//! The CDT mesher evaluates each interior grid point once, at the merge-cell
//! filter, and emits that sample again instead of evaluating the point a
//! second time. These tests mesh a corpus of curved faces with the reuse on
//! and off and compare every position, normal and index bit for bit.

#![allow(clippy::unwrap_used, clippy::cast_precision_loss)]

use std::cell::Cell;

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::solid::SolidId;

use crate::boolean::{BooleanOp, boolean};
use crate::tessellate::TriangleMesh;

thread_local! {
    static RECOMPUTE: Cell<bool> = const { Cell::new(false) };
    static REUSED: Cell<usize> = const { Cell::new(0) };
}

/// The emission's reused sample, dropped (so the point is evaluated again)
/// while [`RECOMPUTE`] is set; counts the samples actually reused.
pub(super) fn observe<T>(reused: Option<T>) -> Option<T> {
    let reused = reused.filter(|_| !RECOMPUTE.with(Cell::get));
    if reused.is_some() {
        REUSED.with(|count| count.set(count.get() + 1));
    }
    reused
}

fn translate(topo: &mut Topology, solid: SolidId, x: f64, y: f64, z: f64) {
    crate::transform::transform_solid(topo, solid, &remus_math::mat::Mat4::translation(x, y, z))
        .unwrap();
}

fn corpus() -> (Topology, Vec<SolidId>) {
    let mut topo = Topology::new();
    let mut solids = vec![
        crate::primitives::make_sphere(&mut topo, 10.0, 16).unwrap(),
        crate::primitives::make_torus(&mut topo, 10.0, 3.0, 16).unwrap(),
        crate::primitives::make_cylinder(&mut topo, 4.0, 9.0).unwrap(),
        crate::primitives::make_cone(&mut topo, 5.0, 1.5, 7.0).unwrap(),
    ];
    let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let edges = remus_topology::explorer::solid_edges(&topo, sharp).unwrap();
    solids.push(
        crate::blend_ops::fillet_v2(&mut topo, sharp, &edges, 1.5)
            .unwrap()
            .solid,
    );
    let mut plate = crate::primitives::make_box(&mut topo, 60.0, 40.0, 10.0).unwrap();
    for (x, y) in [(10.0, 10.0), (30.0, 20.0)] {
        let tool = crate::primitives::make_cylinder(&mut topo, 4.0, 30.0).unwrap();
        translate(&mut topo, tool, x, y, -10.0);
        plate = boolean(&mut topo, BooleanOp::Cut, plate, tool).unwrap();
    }
    solids.push(plate);
    solids.push(nurbs_banded_cube(&mut topo));
    (topo, solids)
}

/// A cube with one edge blended, the blend's cylinder rewritten as the
/// equivalent NURBS patch.
fn nurbs_banded_cube(topo: &mut Topology) -> SolidId {
    use remus_topology::face::FaceSurface;
    let sharp = crate::primitives::make_box(topo, 10.0, 10.0, 10.0).unwrap();
    let edge = remus_topology::explorer::solid_edges(topo, sharp).unwrap()[0];
    let solid = crate::blend_ops::fillet_v2(topo, sharp, &[edge], 2.0)
        .unwrap()
        .solid;
    let band = remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(_)))
        .unwrap();
    let FaceSurface::Cylinder(cylinder) = topo.face(band).unwrap().surface().clone() else {
        unreachable!("the band is a cylinder");
    };
    let axial: Vec<f64> = remus_topology::explorer::face_vertices(topo, band)
        .unwrap()
        .iter()
        .map(|&vertex| {
            (topo.vertex(vertex).unwrap().point() - cylinder.origin()).dot(cylinder.axis())
        })
        .collect();
    let v_min = axial.iter().copied().fold(f64::INFINITY, f64::min) - 0.5;
    let v_max = axial.iter().copied().fold(f64::NEG_INFINITY, f64::max) + 0.5;
    let nurbs =
        remus_geometry::convert::surface_to_nurbs::cylinder_to_nurbs(&cylinder, (v_min, v_max))
            .unwrap();
    let (ku, kv) = (nurbs.knots_u(), nurbs.knots_v());
    let (u_mid, v_mid) = (
        f64::midpoint(ku[0], ku[ku.len() - 1]),
        f64::midpoint(kv[0], kv[kv.len() - 1]),
    );
    let point = nurbs.evaluate(u_mid, v_mid);
    let radial = {
        let offset = point - cylinder.origin();
        offset - cylinder.axis() * offset.dot(cylinder.axis())
    };
    let flip = nurbs.normal(u_mid, v_mid).unwrap().dot(radial) < 0.0;
    let face = topo.face_mut(band).unwrap();
    face.set_surface(FaceSurface::Nurbs(nurbs));
    face.compose_orientation(flip);
    solid
}

fn bits(mesh: &TriangleMesh) -> (Vec<[u64; 3]>, Vec<[u64; 3]>, Vec<u32>) {
    let point = |p: &Point3| [p.x().to_bits(), p.y().to_bits(), p.z().to_bits()];
    let vector = |v: &Vec3| [v.x().to_bits(), v.y().to_bits(), v.z().to_bits()];
    (
        mesh.positions.iter().map(point).collect(),
        mesh.normals.iter().map(vector).collect(),
        mesh.indices.clone(),
    )
}

fn meshes(topo: &Topology, solids: &[SolidId], recompute: bool) -> Vec<String> {
    RECOMPUTE.with(|flag| flag.set(recompute));
    let mut out = Vec::new();
    for &solid in solids {
        for (deflection, angle) in [(0.02, 0.35), (0.1, 0.5)] {
            let (mesh, offsets) = crate::tessellate::tessellate_solid_grouped_with_tolerance(
                topo, solid, deflection, angle,
            )
            .unwrap();
            out.push(format!("{:?} {offsets:?}", bits(&mesh)));
        }
        let quantity = crate::tessellate::tessellate_solid_for_analysis(topo, solid, 0.01, 0.2);
        out.push(format!("{:?}", quantity.map(|mesh| bits(&mesh))));
    }
    RECOMPUTE.with(|flag| flag.set(false));
    out
}

#[test]
fn reused_interior_samples_mesh_like_fresh_evaluations() {
    let (topo, solids) = corpus();
    let fresh = meshes(&topo, &solids, true);
    REUSED.with(|count| count.set(0));
    let reused = meshes(&topo, &solids, false);
    let hits = REUSED.with(Cell::get);
    assert!(hits > 100, "{hits} samples reused");
    assert_eq!(fresh.len(), reused.len());
    for (k, (fresh, reused)) in fresh.iter().zip(&reused).enumerate() {
        assert!(fresh == reused, "mesh {k} differs");
    }
}
