//! Exactness of blend NURBS faces is provenance (#1824).
//!
//! The detailed blend twins list a new NURBS face under `approximateFaces`,
//! and `exactOnly: true` refuses it, unless the engine that built the face
//! recorded the closed form it exactly is. The chamfer floor chain records
//! its elliptic cone bands over a leaning round (#1812), so a chamfer over
//! the rounds of a drafted box is exact. The fillet stripe over the same
//! rounds (#1778) is a loft through sampled sections with no closed form, so
//! it stays approximate and exact-only still refuses it.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashSet;

use remus_math::nurbs::surface::ExactForm;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use serde_json::{Value, json};

use crate::handles::{edge_id_to_u32, face_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;
use crate::types::SolidOperationDetailedResult;

const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

fn envelope(result: SolidOperationDetailedResult) -> Value {
    serde_json::to_value(result).unwrap()
}

fn batch_v2(kernel: &mut RemusKernel, ops: &Value) -> Value {
    serde_json::from_str(&kernel.execute_batch_v2(&ops.to_string())).unwrap()
}

fn handle(value: &Value) -> u32 {
    u32::try_from(value.as_u64().unwrap()).unwrap()
}

fn ends(topo: &Topology, edge: EdgeId) -> (Point3, Point3) {
    let data = topo.edge(edge).unwrap();
    (
        topo.vertex(data.start()).unwrap().point(),
        topo.vertex(data.end()).unwrap().point(),
    )
}

/// The #1743 body: a 40 x 40 x 20 box drafted 2 degrees on its four sides,
/// its uprights rounded R5 (leaning rounds), and the handles of the solid
/// and of its top loop (four lines and four ellipse arcs).
fn drafted_rounded_box(kernel: &mut RemusKernel) -> (u32, Vec<u32>, HashSet<FaceId>) {
    let topo = kernel.topo_mut();
    let body = remus_operations::primitives::make_box(topo, 40.0, 40.0, 20.0).unwrap();
    let sides: Vec<FaceId> = solid_faces(topo, body)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            topo.face(face)
                .unwrap()
                .effective_plane_normal()
                .is_some_and(|n| n.dot(Z).abs() < 1e-9)
        })
        .collect();
    assert_eq!(sides.len(), 4);
    let drafted = remus_operations::draft::draft(
        topo,
        body,
        &sides,
        Z,
        Point3::new(0.0, 0.0, 0.0),
        2f64.to_radians(),
    )
    .unwrap();
    let uprights: Vec<EdgeId> = solid_edges(topo, drafted)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let (a, b) = ends(topo, edge);
            (a.z() - b.z()).abs() > 2.5
        })
        .collect();
    assert_eq!(uprights.len(), 4);
    let rounded = remus_operations::blend_ops::fillet_cascade(topo, drafted, &uprights, 5.0)
        .unwrap()
        .solid;
    let top = top_loop(topo, rounded);
    let faces = solid_faces(topo, rounded).unwrap().into_iter().collect();
    (solid_id_to_u32(rounded), top, faces)
}

fn top_loop(topo: &Topology, solid: SolidId) -> Vec<u32> {
    let top: Vec<u32> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let (a, b) = ends(topo, edge);
            (a.z() - 20.0).abs() < 1e-6 && (b.z() - 20.0).abs() < 1e-6
        })
        .map(edge_id_to_u32)
        .collect();
    assert_eq!(top.len(), 8, "four lines and four ellipse arcs");
    top
}

/// The new NURBS faces of a committed result, with each one's recorded form.
fn new_nurbs(
    kernel: &RemusKernel,
    result: u32,
    input: &HashSet<FaceId>,
) -> Vec<(u32, Option<ExactForm>)> {
    let topo = kernel.topo();
    let solid = kernel.resolve_solid(result).unwrap();
    solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| !input.contains(face))
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Nurbs(surface) => Some((face_id_to_u32(face), surface.exact_form())),
            FaceSurface::Plane { .. }
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => None,
        })
        .collect()
}

/// `chamferDetailed(…, exactOnly: true)` over leaning rounds commits the
/// elliptic cone bands as an exact result, the same envelope as without
/// `exactOnly`, and the batch twins agree.
#[test]
fn exact_only_chamfer_accepts_the_elliptic_cone_bands_over_leaning_rounds() {
    let mut kernel = RemusKernel::new();
    let (solid, top, input) = drafted_rounded_box(&mut kernel);
    let strict = envelope(kernel.chamfer_detailed_impl(solid, &top, 0.2, true));
    assert_eq!(strict["status"], "ok", "{strict}");
    assert_eq!(strict["details"]["quality"], "exact", "{strict}");
    assert_eq!(strict["details"]["engine"], "walking", "{strict}");
    assert!(
        strict["details"].get("approximateFaces").is_none(),
        "{strict}"
    );
    let bands = new_nurbs(&kernel, handle(&strict["value"]), &input);
    assert_eq!(bands.len(), 4, "one band over each round: {bands:?}");
    assert!(
        bands
            .iter()
            .all(|(_, form)| *form == Some(ExactForm::EllipticCone)),
        "every band records its cone: {bands:?}"
    );

    // An identical kernel without exactOnly, and both batch twins.
    let mut permissive = RemusKernel::new();
    let (same_solid, same_top, _) = drafted_rounded_box(&mut permissive);
    assert_eq!((same_solid, &same_top), (solid, &top));
    let lenient = envelope(permissive.chamfer_detailed_impl(solid, &top, 0.2, false));
    assert_eq!(
        lenient, strict,
        "exactOnly changes nothing on an exact result"
    );
    let mut batch = RemusKernel::new();
    let _ = drafted_rounded_box(&mut batch);
    let batched = batch_v2(
        &mut batch,
        &json!([
            {"op": "chamferDetailed", "args": {"solid": solid, "edges": top, "distance": 0.2, "exactOnly": true}},
        ]),
    );
    assert_eq!(batched[0]["ok"], strict, "{batched}");
}

/// `chamferV2Detailed` reaches the same floor chain and reports it the same.
#[test]
fn exact_only_chamfer_v2_accepts_the_elliptic_cone_bands() {
    let mut kernel = RemusKernel::new();
    let (solid, top, _) = drafted_rounded_box(&mut kernel);
    let strict = envelope(kernel.chamfer_v2_detailed_impl(solid, &top, 1.0, 1.0, true));
    assert_eq!(strict["status"], "ok", "{strict}");
    assert_eq!(strict["details"]["quality"], "exact", "{strict}");
    assert!(
        strict["details"].get("approximateFaces").is_none(),
        "{strict}"
    );
}

/// The fillet stripe over the same leaning rounds is a loft through sampled
/// sections (#1778): approximate, listed, and refused under `exactOnly`.
#[test]
fn exact_only_fillet_still_refuses_the_lofted_stripes_over_leaning_rounds() {
    let mut kernel = RemusKernel::new();
    let (solid, top, input) = drafted_rounded_box(&mut kernel);
    let refused = envelope(kernel.fillet_detailed_impl(solid, &top, 2.0, true));
    assert_eq!(refused["status"], "error", "{refused}");
    assert_eq!(refused["code"], "exact_only_unattainable");
    assert_eq!(refused["category"], "quality_refused");
    assert_eq!(refused["details"]["approximateFaceCount"], 4, "{refused}");

    let disclosed = envelope(kernel.fillet_detailed_impl(solid, &top, 2.0, false));
    assert_eq!(disclosed["status"], "ok", "{disclosed}");
    assert_eq!(disclosed["details"]["quality"], "approximate");
    let stripes = new_nurbs(&kernel, handle(&disclosed["value"]), &input);
    assert_eq!(stripes.len(), 4, "{stripes:?}");
    assert!(
        stripes.iter().all(|(_, form)| form.is_none()),
        "a loft records no closed form: {stripes:?}"
    );
    let listed: HashSet<u32> = disclosed["details"]["approximateFaces"]
        .as_array()
        .unwrap()
        .iter()
        .map(handle)
        .collect();
    let expected: HashSet<u32> = stripes.iter().map(|(face, _)| *face).collect();
    assert_eq!(listed, expected, "every stripe is disclosed");
}
