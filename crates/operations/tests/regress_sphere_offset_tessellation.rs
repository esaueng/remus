//! Regression: offsetting a sphere must produce a body you can actually see.
//!
//! `offset_solid` measured the offset ball perfectly (`4/3*pi*(r+d)^3`, to the
//! last digit) and kept both faces spherical, yet the body tessellated to ZERO
//! triangles and raised no warning — "no boundary edges" is vacuously true of
//! an empty mesh, so every watertightness check passed.
//!
//! Cause: `loops::try_direct_chain` walks the reconstructed boundary starting
//! from an arbitrary edge, which fixes the loop's traversal sense arbitrarily.
//! On a closed surface the sense IS the region — the same equatorial loop
//! bounds the northern hemisphere walked one way and the southern hemisphere
//! walked the other — so BOTH offset faces came out covering the northern
//! half, one of them inside out. `dedupe_coincident_triangles` then cancelled
//! the two opposite-winding copies against each other and the mesh emptied.
//!
//! The volume stayed right throughout because the face integrator reads the
//! surface and the face's own orientation, not the region the wire selects.
//! That is exactly why the assertions below are against the tessellated
//! geometry and hand-written closed forms rather than against another
//! integrator.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::type_complexity
)]

use std::collections::HashMap;
use std::f64::consts::PI;

use remus_operations::measure::solid_volume;
use remus_operations::offset_v2::offset_solid_v2;
use remus_operations::primitives::{make_box, make_cylinder, make_sphere, make_torus};
use remus_operations::tessellate::{TriangleMesh, tessellate_solid};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Model scales, ordered as a rotation of the natural small-to-large sweep so
/// a result that only holds at whichever scale runs first cannot pass.
///
/// The unfixed code emitted ZERO triangles at every one of these, for both
/// offset distances, while reporting the exact closed-form volume every time.
/// The fixed result is flat across all three for a stated reason: the fix is
/// the sign of a dot product between two vector areas — a pure orientation
/// decision with no length in it — so there is nothing for the scale to act on.
const SCALES: [f64; 3] = [1000.0, 0.001, 1.0];

/// Scales for the CONTROL bodies. `offset_solid` on a cylinder already fails
/// at 1000x on unfixed `main` ("offset face 0 has no reconstructed wire
/// loops"), verified by reverting this branch's source and re-running. That is
/// a pre-existing scale limitation of the offset engine, unrelated to the
/// orientation defect under test, so the controls sweep the two scales where
/// the operation is expected to succeed.
const CONTROL_SCALES: [f64; 2] = [0.001, 1.0];

fn sphere_volume(r: f64) -> f64 {
    4.0 / 3.0 * PI * r * r * r
}

/// Volume enclosed by a triangle mesh, by the divergence theorem, written out
/// here on purpose: it shares nothing with the face integrator that
/// `solid_volume` and `mass_properties` both call, so agreement between the
/// two is real evidence rather than a tautology.
fn mesh_enclosed_volume(mesh: &TriangleMesh) -> f64 {
    let mut total = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let a = mesh.positions[tri[0] as usize];
        let b = mesh.positions[tri[1] as usize];
        let c = mesh.positions[tri[2] as usize];
        total += (a.x() * (b.y() * c.z() - c.y() * b.z())
            - b.x() * (a.y() * c.z() - c.y() * a.z())
            + c.x() * (a.y() * b.z() - b.y() * a.z()))
            / 6.0;
    }
    total
}

/// Extent of the mesh along z. An offset ball must span the full diameter; a
/// single hemisphere covers only half of it.
fn mesh_z_span(mesh: &TriangleMesh) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for p in &mesh.positions {
        lo = lo.min(p.z());
        hi = hi.max(p.z());
    }
    (lo, hi)
}

fn assert_closed_two_manifold(topo: &Topology, solid: SolidId, what: &str) {
    let mut uses: HashMap<usize, usize> = HashMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *uses.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    assert!(!uses.is_empty(), "{what}: solid has no edges");
    let free = uses.values().filter(|&&c| c == 1).count();
    let non_manifold = uses.values().filter(|&&c| c > 2).count();
    assert_eq!(free, 0, "{what}: expected 0 free edges, got {free}");
    assert_eq!(
        non_manifold, 0,
        "{what}: expected 0 non-manifold edges, got {non_manifold}"
    );
}

#[test]
fn offsetting_a_sphere_produces_a_visible_body() {
    for scale in SCALES {
        let radius = 10.0 * scale;
        // Offsets stated as fractions of the radius so the same geometry is
        // tested at every scale. The tiny one reproduces the original report's
        // "+0.001 on r=10" case as a ratio rather than an absolute length.
        for ratio in [0.2, 1e-4] {
            let distance = radius * ratio;
            let outer = radius + distance;
            let exact = sphere_volume(outer);
            let what = format!("scale {scale} offset +{ratio}r");

            let mut topo = Topology::default();
            let sphere = make_sphere(&mut topo, radius, 32).unwrap();
            let result = offset_solid_v2(&mut topo, sphere, distance)
                .unwrap_or_else(|e| panic!("{what}: offset failed: {e}"));

            // Both spherical patches survive, and the shell is closed.
            let tags: Vec<_> = solid_faces(&topo, result)
                .unwrap()
                .iter()
                .map(|&f| topo.face(f).unwrap().surface().type_tag())
                .collect();
            assert_eq!(
                tags,
                vec!["sphere", "sphere"],
                "{what}: expected two spherical patches, got {tags:?}"
            );
            assert_closed_two_manifold(&topo, result, &what);

            // Measured volume against the closed form.
            let volume = solid_volume(&topo, result, outer * 0.005).unwrap();
            let rel = (volume - exact).abs() / exact;
            assert!(
                rel < 1e-9,
                "{what}: got {volume}, closed form 4/3*pi*(r+d)^3 = {exact} \
                 (relative error {rel:.3e})"
            );

            // The assertion the defect actually violated: the body must have
            // geometry. Zero triangles measured perfectly and warned about
            // nothing.
            let mesh = tessellate_solid(&topo, result, outer * 0.001).unwrap();
            let triangles = mesh.indices.len() / 3;
            assert!(
                triangles > 0,
                "{what}: offset body tessellated to ZERO triangles \
                 (volume measured {volume}, closed form {exact})"
            );

            // Both hemispheres must be present, not one of them twice. The
            // defect produced two copies of the NORTHERN half with opposite
            // winding, so the span was [0, outer] before they cancelled.
            let (lo, hi) = mesh_z_span(&mesh);
            let span_rel_error = ((hi - lo) - 2.0 * outer).abs() / (2.0 * outer);
            assert!(
                span_rel_error < 0.01,
                "{what}: mesh spans z in [{lo}, {hi}] — expected the full \
                 diameter {} (both hemispheres), relative error {span_rel_error:.3e}",
                2.0 * outer
            );

            // Independent volume, from the mesh, by hand. Positive (outward
            // winding) and within tessellation error of the same closed form.
            let from_mesh = mesh_enclosed_volume(&mesh);
            let mesh_rel = (from_mesh - exact).abs() / exact;
            assert!(
                mesh_rel < 0.01,
                "{what}: mesh encloses {from_mesh}, closed form {exact} \
                 (relative error {mesh_rel:.3e})"
            );
        }
    }
}

/// Controls: the same offset on bodies whose faces do NOT share their whole
/// boundary. These passed before the fix and must keep passing — the defect
/// was not about curvature, periodicity or seams. The torus is the sharp
/// control: closed, seamed, doubly periodic, and fine throughout.
///
/// Each control asserts the two things the defect broke on the sphere: the
/// closed-form volume, and a non-empty mesh. It deliberately does NOT assert
/// the mesh's enclosed volume — the tessellation of an offset cylinder is
/// itself well off its closed form (~57% at BOTH 1x and 0.001x, identical
/// triangle counts, so it is a fidelity gap rather than a scale one), which is
/// a separate matter reported alongside this change. Holding the controls to
/// volume-plus-non-empty keeps them a clean statement about which bodies the
/// orientation defect touched.
#[test]
fn offset_controls_box_cylinder_and_torus_still_tessellate() {
    for scale in CONTROL_SCALES {
        let s = 10.0 * scale;
        let d = s * 0.2;

        // (label, solid builder, closed-form volume of the offset body)
        let cases: [(&str, fn(&mut Topology, f64) -> SolidId, f64); 3] = [
            (
                // Box: side s offset by d gives (s + 2d)^3.
                "box",
                |topo, s| make_box(topo, s, s, s).unwrap(),
                (s + 2.0 * d).powi(3),
            ),
            (
                // Cylinder: r = s/2, h = s, offset d gives pi*(r+d)^2*(h+2d).
                "cylinder",
                |topo, s| make_cylinder(topo, s / 2.0, s).unwrap(),
                PI * (s / 2.0 + d) * (s / 2.0 + d) * (s + 2.0 * d),
            ),
            (
                // Torus: R = s, r = 0.3s, offset d gives 2*pi^2*R*(r+d)^2.
                "torus",
                |topo, s| make_torus(topo, s, 0.3 * s, 32).unwrap(),
                2.0 * PI * PI * s * (0.3 * s + d) * (0.3 * s + d),
            ),
        ];

        for (label, build, exact) in cases {
            let mut topo = Topology::default();
            let solid = build(&mut topo, s);
            let out = offset_solid_v2(&mut topo, solid, d)
                .unwrap_or_else(|e| panic!("scale {scale} {label}: offset failed: {e}"));

            let volume = solid_volume(&topo, out, s * 0.005).unwrap();
            let rel = (volume - exact).abs() / exact;
            assert!(
                rel < 1e-9,
                "scale {scale} {label}: offset volume {volume}, closed form {exact} \
                 (relative error {rel:.3e})"
            );

            let mesh = tessellate_solid(&topo, out, s * 0.001).unwrap();
            assert!(
                !mesh.indices.is_empty(),
                "scale {scale} {label}: offset body tessellated to ZERO triangles \
                 (volume measured {volume})"
            );
            assert_closed_two_manifold(&topo, out, &format!("scale {scale} {label}"));
        }
    }
}
