//! Whole-edge fillet of a flanged bracket through the facade.
//!
//! The body is a 4 mm plate with a front notch and two 40 mm tabs fused
//! flush with its front (y = 0) and outer sides, so the flush planes arrive as
//! coplanar-split faces and "every edge" includes the flat seams between them.
//! `Model::fillet` drops those seams, as the WASM binding does, and every
//! corner is one of the rectangular planar families:
//!
//! - three convex edges: sphere octant, `C = r³(1 − π/6)` past the stripes;
//! - two convex + one concave (plate notch, flush tab corners): notch torus,
//!   `t3 = (14π − 3π² − 12) r³ / 12` removed past the stripes;
//! - one convex + two concave (tab back corners on the plate): its mirror,
//!   `t3` added.
//!
//! Every stripe is inset one radius at each corner it ends in, so
//! `V = V0 − A·ΣL_convex + A·ΣL_concave − n_sphere·C − (n_notch − n_rib)·t3`
//! with `A = r²(1 − π/4)`, computed from the input's own edges below.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::PI;

use remus::prelude::*;
use remus_operations::query::{EdgeConcavity, edge_concavity, filter_filletable_edges};
use remus_operations::tessellate::welded_mesh_quality;
use remus_topology::face::FaceSurface;

const V0: f64 = 29_760.0;

fn bracket(model: &mut Model) -> SolidId {
    let profile = [
        (0.0, 0.0),
        (22.0, 0.0),
        (22.0, 8.0),
        (52.0, 8.0),
        (52.0, 0.0),
        (74.0, 0.0),
        (74.0, 60.0),
        (0.0, 60.0),
    ]
    .map(|(x, y)| Point3::new(x, y, 0.0));
    let face = model.make_planar_face(&profile).unwrap();
    let mut solid = model.extrude(face, Vec3::new(0.0, 0.0, 1.0), 4.0).unwrap();
    for x0 in [0.0, 56.0] {
        let tab = model.make_box(18.0, 10.0, 40.0).unwrap();
        model
            .transform(tab, &Mat4::translation(x0, 0.0, 0.0))
            .unwrap();
        solid = model.fuse(solid, tab).unwrap().solid;
    }
    solid
}

/// The closed form above, from the input body's own filletable edges.
fn expected_volume(model: &Model, solid: SolidId, r: f64) -> (f64, [usize; 3]) {
    let topo = model.topology();
    let all = model.solid_edges(solid).unwrap();
    let edges = filter_filletable_edges(topo, solid, &all).unwrap();
    let mut at_vertex: BTreeMap<usize, Vec<(Vec3, bool)>> = BTreeMap::new();
    let mut lines = Vec::new();
    for &edge_id in &edges {
        let edge = topo.edge(edge_id).unwrap();
        let a = topo.vertex(edge.start()).unwrap().point();
        let b = topo.vertex(edge.end()).unwrap().point();
        let convex = match edge_concavity(topo, solid, edge_id, 0.1).unwrap() {
            EdgeConcavity::Convex => true,
            EdgeConcavity::Concave => false,
            other => panic!("unexpected {other:?} edge"),
        };
        let dir = (b - a).normalize().unwrap();
        at_vertex
            .entry(edge.start().index())
            .or_default()
            .push((dir, convex));
        at_vertex
            .entry(edge.end().index())
            .or_default()
            .push((-dir, convex));
        lines.push((
            edge.start().index(),
            edge.end().index(),
            (b - a).length(),
            convex,
        ));
    }
    // A corner insets its stripes; two collinear pieces of one split edge
    // just continue.
    let corner = |vertex: usize| at_vertex[&vertex].len() >= 3;
    let mut census = [0usize; 3];
    for incident in at_vertex.values() {
        match incident.as_slice() {
            [(d0, c0), (d1, c1)] => {
                assert!(d0.dot(*d1) < -1.0 + 1e-12 && c0 == c1, "pass-through");
            }
            three @ [_, _, _] => {
                census[3 - three.iter().filter(|(_, c)| *c).count()] += 1;
            }
            other => panic!("unexpected {}-edge vertex", other.len()),
        }
    }
    let area = r * r * (1.0 - PI / 4.0);
    let mut volume = V0;
    for (start, end, length, convex) in lines {
        let insets = [start, end].into_iter().filter(|&v| corner(v)).count();
        #[allow(clippy::cast_precision_loss)]
        let active = length - r * insets as f64;
        volume += if convex { -area } else { area } * active;
    }
    let sphere = r.powi(3) * (1.0 - PI / 6.0);
    let t3 = (14.0 * PI - 3.0 * PI * PI - 12.0) * r.powi(3) / 12.0;
    #[allow(clippy::cast_precision_loss)]
    let corners = census[0] as f64 * sphere + (census[1] as f64 - census[2] as f64) * t3;
    (volume - corners, census)
}

#[test]
fn whole_edge_bracket_fillet_matches_closed_form() {
    for r in [0.5, 1.0] {
        let mut model = Model::new();
        let solid = bracket(&mut model);
        let (expected, census) = expected_volume(&model, solid, r);
        // 18 sphere corners (4 plate back, 2 plate front bottom, 4 notch
        // mouth, 8 tab top), 8 notch tori (4 plate notch, 4 flush tab), 2
        // rib-base tori.
        assert_eq!(census, [18, 8, 2]);
        let edges = model.solid_edges(solid).unwrap();
        let result = model
            .fillet(solid, &edges, r)
            .unwrap_or_else(|e| panic!("r={r}: whole-edge bracket must build: {e}"));
        assert!(!result.is_partial);
        let out = result.solid;
        assert!(model.validate(out).unwrap().is_valid(), "r={r}: invalid");
        let quality = welded_mesh_quality(&model.tessellate(out, 0.05).unwrap());
        assert!(quality.is_watertight(), "r={r}: mesh not watertight");
        assert_eq!(quality.non_manifold_edges, 0);
        let tori = remus_topology::explorer::solid_faces(model.topology(), out)
            .unwrap()
            .into_iter()
            .filter(|&f| {
                matches!(
                    model.topology().face(f).unwrap().surface(),
                    FaceSurface::Torus(_)
                )
            })
            .count();
        assert_eq!(tori, 10, "r={r}: one torus per mixed corner");
        let volume = model.volume(out, 0.005).unwrap();
        // The exact route samples curved trims at a fixed count, leaving an
        // r²-scaled residual on every corner family (5.7e-3 here at r = 1; a
        // plain notched plate shows 2.7e-3). The window is an order below
        // the smallest corner term, t3·r³ = 0.025 at r = 0.5.
        assert!(
            (volume - expected).abs() < 0.01 * r * r,
            "r={r}: volume {volume:.6} vs closed form {expected:.6}"
        );
    }
}

/// At r = 2 the plate's 4 mm sides would need two 2 mm setbacks: the stripes
/// meet edge to edge and the request is refused, input untouched.
#[test]
fn whole_edge_bracket_at_half_plate_thickness_refuses() {
    let mut model = Model::new();
    let solid = bracket(&mut model);
    let counts = remus_topology::explorer::solid_entity_counts(model.topology(), solid).unwrap();
    let edges = model.solid_edges(solid).unwrap();
    assert!(model.fillet(solid, &edges, 2.0).is_err());
    assert_eq!(
        remus_topology::explorer::solid_entity_counts(model.topology(), solid).unwrap(),
        counts
    );
}
