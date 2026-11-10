//! Every edge of a filleted body lies on each face it bounds, read through
//! the bindings a consumer measures it with.
//!
//! OpenZCAD checks an approximate blend by sampling each edge — straight
//! between its ends when `getEdgeCurveType` says `LINE`, along `sampleEdge`
//! otherwise — and projecting the samples onto each face with
//! `projectPointOnSurface`. On three fillets that read as chords:
//!
//! * the four quarter-arc rim of a post fused onto a box corner, r ≥ R/2:
//!   real chords. The rolling-ball engine declined the spindle torus of each
//!   arc and lofted it between straight edges (fixed in `arc_torus`).
//! * the mixed seam (r1) and union junction (r5) corner fillets: no chord in
//!   the body. The walked contact between the NURBS corner patch and the
//!   exact band is a degree-1 polyline of ~500 points along a quarter circle,
//!   which `getEdgeCurveType` called `LINE`, so the consumer measured the
//!   straight segment between its ends, 0.29 r off both faces.
//!
//! Independently, `projectPointOnSurface` grid-searched analytic faces over a
//! fixed `v ∈ [−π, π]`, so a point on a cylinder wall's own rim read tens of
//! units off it, and no edge touching an exact face could be measured.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::vec::Point3;
use remus_operations::compound_ops::fuse_solids;
use remus_operations::heal::unify_faces_checked;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::solid::SolidId;

use crate::kernel::RemusKernel;

#[allow(clippy::cast_possible_truncation)]
const fn handle(index: usize) -> u32 {
    index as u32
}

/// Largest distance of any edge sample from any face that edge bounds, with
/// the edge and face handles that produced it.
fn worst_edge_face_deviation(kernel: &RemusKernel, solid: SolidId) -> (f64, u32, u32, String) {
    let mut worst = (0.0, 0, 0, String::new());
    for face in solid_faces(kernel.topo(), solid).unwrap() {
        let data = kernel.topo().face(face).unwrap();
        let wires: Vec<_> = std::iter::once(data.outer_wire())
            .chain(data.inner_wires().iter().copied())
            .collect();
        for wire in wires {
            for use_ in kernel.topo().wire(wire).unwrap().edges() {
                let edge = handle(use_.edge().index());
                let kind = kernel.get_edge_curve_type(edge).unwrap();
                let polyline = kernel.sample_edge_polyline(edge, 0.005).unwrap();
                let samples: Vec<Point3> = polyline
                    .chunks_exact(3)
                    .map(|p| Point3::new(p[0], p[1], p[2]))
                    .collect();
                let points: Vec<Point3> = if kind == "LINE" {
                    let (a, b) = (samples[0], samples[samples.len() - 1]);
                    (1..8).map(|i| a + (b - a) * (f64::from(i) / 8.0)).collect()
                } else {
                    samples
                };
                for p in points {
                    let projected = kernel
                        .project_point_on_surface(handle(face.index()), p.x(), p.y(), p.z())
                        .unwrap();
                    if projected[5] > worst.0 {
                        worst = (projected[5], edge, handle(face.index()), kind.clone());
                    }
                }
            }
        }
    }
    worst
}

fn edges_where(
    kernel: &RemusKernel,
    solid: SolidId,
    keep: impl Fn(Point3, Point3, &EdgeCurve, f64) -> bool,
) -> Vec<EdgeId> {
    solid_edges(kernel.topo(), solid)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = kernel.topo().edge(e).unwrap();
            let a = kernel.topo().vertex(edge.start()).unwrap().point();
            let b = kernel.topo().vertex(edge.end()).unwrap().point();
            let length = remus_operations::measure::edge_length(kernel.topo(), e).unwrap();
            keep(a, b, edge.curve(), length)
        })
        .collect()
}

fn at(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// Box 30 × 18 × `depth` fused with an R6 post on its corner, `cap` rounding
/// the post's top rim first, unified as a union gate leaves it.
fn post_on_corner(kernel: &mut RemusKernel, depth: f64, height: f64, cap: Option<f64>) -> SolidId {
    let topo = kernel.topo_mut();
    let block = make_box(topo, 30.0, 18.0, depth).unwrap();
    let mut post = make_cylinder(topo, 6.0, height).unwrap();
    if let Some(radius) = cap {
        let rim: Vec<EdgeId> = solid_edges(topo, post)
            .unwrap()
            .into_iter()
            .filter(|&e| {
                let edge = topo.edge(e).unwrap();
                matches!(edge.curve(), EdgeCurve::Circle(_))
                    && at(topo.vertex(edge.start()).unwrap().point().z(), height)
            })
            .collect();
        post = crate::helpers::fillet_whole_selection_result(topo, post, &rim, radius)
            .unwrap()
            .solid;
    }
    let fused = fuse_solids(topo, &[block, post]).unwrap();
    unify_faces_checked(topo, fused).unwrap();
    fused
}

fn fillet(kernel: &mut RemusKernel, solid: SolidId, edges: &[EdgeId], radius: f64) -> SolidId {
    crate::helpers::fillet_whole_selection_result(kernel.topo_mut(), solid, edges, radius)
        .unwrap()
        .solid
}

fn assert_coherent(kernel: &RemusKernel, solid: SolidId, label: &str) {
    let (deviation, edge, face, kind) = worst_edge_face_deviation(kernel, solid);
    assert!(
        deviation < 1e-4,
        "{label}: {kind} edge {edge} lies {deviation:.4} off face {face}"
    );
}

#[test]
fn quarter_arc_rim_fillet_edges_lie_on_their_faces() {
    for radius in [2.9, 3.8, 5.0] {
        let mut kernel = RemusKernel::new();
        let body = post_on_corner(&mut kernel, 24.0, 31.25, None);
        let rim = edges_where(&kernel, body, |a, b, _, _| {
            at(a.z(), 31.25) && at(b.z(), 31.25)
        });
        assert_eq!(rim.len(), 4, "the fuse splits the rim into quarter arcs");
        let rounded = fillet(&mut kernel, body, &rim, radius);
        assert_coherent(&kernel, rounded, &format!("rim r{radius}"));
    }
}

/// OpenZCAD `mixed-seam-fillet`: the junction arc, the vertical junction line
/// and the box's top edge, r1, on a post whose cap is rounded r2.
#[test]
fn mixed_seam_fillet_edges_lie_on_their_faces() {
    let mut kernel = RemusKernel::new();
    let body = post_on_corner(&mut kernel, 26.0, 35.0, Some(2.0));
    let picked = edges_where(&kernel, body, |a, b, curve, length| match curve {
        EdgeCurve::Circle(_) => {
            at(a.z(), 26.0) && at(b.z(), 26.0) && at(length, 3.0 * std::f64::consts::PI)
        }
        EdgeCurve::Line => {
            (at(a.x(), 6.0)
                && at(b.x(), 6.0)
                && at(a.y(), 0.0)
                && at(b.y(), 0.0)
                && at(length, 26.0))
                || (at(a.y(), 0.0)
                    && at(b.y(), 0.0)
                    && at(a.z(), 26.0)
                    && at(b.z(), 26.0)
                    && at(length, 24.0))
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => false,
    });
    assert_eq!(picked.len(), 3);
    let rounded = fillet(&mut kernel, body, &picked, 1.0);
    assert_coherent(&kernel, rounded, "mixed seam r1");
}

/// OpenZCAD `filleted-union-face-offset`: the post/box junction and the two
/// top edges that meet it, r5.
#[test]
fn union_junction_fillet_edges_lie_on_their_faces() {
    let mut kernel = RemusKernel::new();
    let body = post_on_corner(&mut kernel, 26.0, 35.0, None);
    let picked = edges_where(&kernel, body, |a, b, curve, _| match curve {
        EdgeCurve::Circle(_) => at(a.z(), 26.0) && at(b.z(), 26.0),
        EdgeCurve::Line => {
            let top = at(a.z(), 26.0) && at(b.z(), 26.0);
            let rise = (a.z() - b.z()).abs() > 25.99;
            (top && ((at(a.x(), 0.0) && at(b.x(), 0.0)) || (at(a.y(), 0.0) && at(b.y(), 0.0))))
                || (rise
                    && ((at(a.x(), 6.0) && at(b.x(), 6.0) && at(a.y(), 0.0) && at(b.y(), 0.0))
                        || (at(a.y(), 6.0) && at(b.y(), 6.0) && at(a.x(), 0.0) && at(b.x(), 0.0))))
        }
        EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => false,
    });
    assert_eq!(picked.len(), 5);
    let rounded = fillet(&mut kernel, body, &picked, 5.0);
    assert_coherent(&kernel, rounded, "union junction r5");
}
