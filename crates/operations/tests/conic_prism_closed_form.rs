//! Extrude a planar face bounded by a PARABOLIC arc and check the result
//! against a hand-derived closed form, plus full topology invariants.
//!
//! Why this shape: the region between `y = x^2` and the chord `y = 1` has an
//! elementary area (Archimedes' quadrature of the parabola), so the prism's
//! volume is known exactly without asking the kernel anything.
//!
//!     A = integral_{-1}^{1} (1 - x^2) dx = 2 - 2/3 = 4/3
//!     V = A * h
//!
//! Deliberately NOT verified by "`mass_properties` agrees with
//! `solid_volume`": those two share `integrate_face`, so their agreement is
//! structurally blind. The closed form above is the only reference used.
//!
//! Also swept over model scale. Volume carries L^3, so a defect in an
//! absolute tolerance shows up as a relative error that moves with scale.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_check::properties::{PropertiesOptions, solid_area, solid_volume};
use remus_math::curves::Parabola3D;
use remus_math::vec::{Point3, Vec3};
use remus_operations::extrude::extrude;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

const SCALES: [f64; 3] = [1.0, 1000.0, 0.001];

/// Exact area of the region between `y = x^2` and the chord `y = 1`,
/// in units where the half-width is 1.
const UNIT_SEGMENT_AREA: f64 = 4.0 / 3.0;

/// Build the closed planar wire: the parabolic arc from `(-w, w)` to
/// `(w, w)` in the XY plane (the curve `y = x^2 / w`), closed by a straight
/// chord back along `y = w`.
fn parabolic_segment_face(topo: &mut Topology, w: f64) -> remus_topology::face::FaceId {
    // focal length f = w/4 makes the parameterization
    //   P(t) = V + (t^2 / 4f) * axis + t * u = (t, t^2 / w)
    // so t = +-w gives y = w.
    let par = Parabola3D::with_axes(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        0.25 * w,
    )
    .unwrap();

    let vtol = 1e-9 * w;
    let left = par.evaluate(-w);
    let right = par.evaluate(w);

    let v_left = topo.add_vertex(Vertex::new(left, vtol));
    let v_right = topo.add_vertex(Vertex::new(right, vtol));

    let mut arc_edge = Edge::new(v_left, v_right, EdgeCurve::Parabola(par));
    arc_edge.set_trim(Some((-w, w)));
    let arc = topo.add_edge(arc_edge);
    let chord = topo.add_edge(Edge::new(v_right, v_left, EdgeCurve::Line));

    let wire = Wire::new(
        vec![OrientedEdge::new(arc, true), OrientedEdge::new(chord, true)],
        true,
    )
    .unwrap();
    let wid = topo.add_wire(wire);
    remus_topology::builder::make_planar_face_from_wire(topo, wid).unwrap()
}

/// Closed 2-manifold shell, zero free edges, zero non-manifold edges, and
/// Euler `V - E + F = 2` for the outer shell.
fn assert_topology_invariants(topo: &Topology, solid: SolidId, what: &str) {
    let adj = AdjacencyIndex::build(topo, solid).unwrap();
    assert!(
        adj.boundary_edges().is_empty(),
        "{what}: {} free edge(s) — shell is not closed",
        adj.boundary_edges().len()
    );
    assert!(
        adj.non_manifold_edges().is_empty(),
        "{what}: {} non-manifold edge(s)",
        adj.non_manifold_edges().len()
    );
    assert!(
        adj.is_manifold(),
        "{what}: shell is not a closed 2-manifold"
    );

    // Euler characteristic over the outer shell, counting DISTINCT entities.
    let shell_id = topo.solid(solid).unwrap().outer_shell();
    let faces = topo.shell(shell_id).unwrap().faces().to_vec();
    let mut verts = std::collections::HashSet::new();
    let mut edges = std::collections::HashSet::new();
    for &fid in &faces {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                let e = topo.edge(oe.edge()).unwrap();
                edges.insert(oe.edge().index());
                verts.insert(e.start().index());
                verts.insert(e.end().index());
            }
        }
    }
    #[allow(clippy::cast_possible_wrap)]
    let chi = verts.len() as i64 - edges.len() as i64 + faces.len() as i64;
    assert_eq!(
        chi,
        2,
        "{what}: Euler characteristic V - E + F = {} - {} + {} = {chi}, expected 2",
        verts.len(),
        edges.len(),
        faces.len()
    );
}

#[test]
fn parabolic_prism_volume_matches_archimedes_quadrature_at_every_scale() {
    for k in SCALES {
        let (w, h) = (k, 2.0 * k);
        let mut topo = Topology::new();
        let face = parabolic_segment_face(&mut topo, w);
        let solid = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), h).unwrap();

        assert_topology_invariants(&topo, solid, &format!("parabolic prism at {k}x"));

        // Archimedes: the parabolic segment has area 4/3 of the unit case,
        // scaled by w^2 (the region scales with w in both directions).
        let expected = UNIT_SEGMENT_AREA * w * w * h;
        let got = solid_volume(&topo, solid, &PropertiesOptions::default()).unwrap();
        let rel = (got - expected).abs() / expected;
        assert!(
            rel < 1e-6,
            "parabolic prism volume at {k}x: got {got}, closed form {expected} (rel {rel:.3e})"
        );

        // Guard the two silent-fallback failure modes explicitly. If the
        // parabolic edge collapsed to its chord the face would be empty
        // (the chord IS the closing edge), so the volume would go to zero;
        // if the trim were ignored the arc would run to infinity.
        assert!(
            got > 0.5 * expected,
            "volume {got} collapsed toward the chord (expected {expected})"
        );
    }
}

#[test]
fn parabolic_prism_area_matches_the_closed_form_at_every_scale() {
    // Surface area = 2 caps + parabolic wall + flat wall.
    //   caps:           2 * (4/3) * w^2
    //   flat wall:      (2w) * h
    //   parabolic wall: L * h, where L is the arc length of y = x^2/w from
    //                   x = -w to x = w. For the parameterization above
    //                   (focal length f = w/4) that is
    //                   L = w * (sqrt(5) + asinh(2)/2), i.e. twice the
    //                   half-arc  sqrt(5)/2 + asinh(2)/4  derived by hand in
    //                   `conic_edges_closed_form.rs`.
    let arc_unit = 2.0 * (5.0_f64.sqrt() / 2.0 + 2.0_f64.asinh() / 4.0);

    for k in SCALES {
        let (w, h) = (k, 2.0 * k);
        let mut topo = Topology::new();
        let face = parabolic_segment_face(&mut topo, w);
        let solid = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), h).unwrap();

        let expected = 2.0 * UNIT_SEGMENT_AREA * w * w + 2.0 * w * h + arc_unit * w * h;
        let got = solid_area(&topo, solid, &PropertiesOptions::default()).unwrap();
        let rel = (got - expected).abs() / expected;

        // ONE bound at every scale.
        //
        // This assertion used to be split — 1e-5 at and above unit scale,
        // 1e-3 below — because `face_integrator::patch_count` tiled a
        // quadrature axis by comparing the RAW parameter span against the
        // absolute constant PI/4. That is dimensionless only for an angular
        // parameter; this wall's NURBS v axis is the parabola's own
        // parameter, whose domain is exactly (-w, w) and so carries length.
        // The wall got 16 patches on a large model and 1 on a small one, and
        // its area error jumped 530x (8.5e-7 -> 4.5e-4) as the span crossed
        // PI/4. The split bound was stated as an UPPER bound so it would keep
        // holding once the defect was fixed, which is what happened.
        //
        // A NURBS axis is now tiled per KNOT SPAN, which is a property of the
        // surface rather than of the model's units, so the reading no longer
        // moves with scale: 9.885e-13, 9.889e-13 and 9.893e-13 at 1x, 1000x
        // and 0.001x. The planar caps were always exact to ~1e-16.
        let bound = 1e-11;
        assert!(
            rel < bound,
            "parabolic prism area at {k}x: got {got}, closed form {expected} (rel {rel:.3e})"
        );
    }
}

#[test]
fn the_parabolic_edge_survives_into_the_extruded_solid() {
    // If the extrude path had quietly replaced the conic with a chord, the
    // result would be a plain box-like prism with no curved wall. Assert the
    // curved wall is actually there.
    let mut topo = Topology::new();
    let face = parabolic_segment_face(&mut topo, 1.0);
    let solid = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();

    let mut has_curved_wall = false;
    for fid in solid_faces(&topo, solid).unwrap() {
        if !matches!(
            topo.face(fid).unwrap().surface(),
            remus_topology::face::FaceSurface::Plane { .. }
        ) {
            has_curved_wall = true;
        }
    }
    assert!(
        has_curved_wall,
        "extruding a parabolic arc produced only planar faces — the conic was flattened"
    );
}

#[test]
fn a_boolean_on_a_conic_edged_solid_refuses_by_name() {
    // The GFA's intersection phases, face splitter and classifier have no
    // hyperbola/parabola support. Letting such an input through would make
    // them fall back to a chord or a straight line and return a plausible
    // but geometrically wrong solid — the failure mode hardest to notice
    // downstream. `gfa::reject_unsupported_curves` fails closed instead,
    // and the message names the variant so a caller can act on it.
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::primitives::make_box;

    let mut topo = Topology::new();
    let face = parabolic_segment_face(&mut topo, 1.0);
    let prism = extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 2.0).unwrap();
    let cube = make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();

    let err = boolean(&mut topo, BooleanOp::Cut, prism, cube)
        .expect_err("boolean on a parabola-edged solid must refuse, not guess");
    let msg = err.to_string();
    assert!(
        msg.contains("parabola"),
        "refusal must name the offending variant, got: {msg}"
    );
}
