#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_math::vec::Vec3;
use remus_operations::blend_cluster::{Convexity, recognize_blend_cluster};
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::FaceSurface;
use serde_json::{Value, json};

use super::MAX_PREPARED_GROUP_EDITS;
use crate::handles::{face_id_to_u32, solid_id_to_u32};
use crate::kernel::RemusKernel;

fn fixture(k: &mut RemusKernel) -> u32 {
    let solid = remus_io::step::reader::read_step(
        include_str!("../../../../operations/tests/data/gentle_gecko_filleted.step"),
        k.topo_mut(),
    )
    .unwrap()[0];
    solid_id_to_u32(solid)
}

fn boss(k: &RemusKernel, solid: u32) -> u32 {
    let solid = k.resolve_solid(solid).unwrap();
    face_id_to_u32(
        solid_faces(k.topo(), solid)
            .unwrap()
            .into_iter()
            .find(|&f| {
                matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Cylinder(c)
                    if c.axis().z().abs() > 0.99
                        && c.origin().x().abs() < 1e-7
                        && c.origin().y().abs() < 1e-7)
            })
            .unwrap(),
    )
}

/// The convex top band along the x = 0 face.
fn x0_band(k: &RemusKernel, solid: u32) -> u32 {
    let solid_id = k.resolve_solid(solid).unwrap();
    let seed = k.resolve_face(boss(k, solid)).unwrap();
    let cluster = recognize_blend_cluster(k.topo(), solid_id, seed).unwrap();
    face_id_to_u32(
        cluster
            .members
            .iter()
            .filter(|m| m.convexity == Convexity::Convex)
            .flat_map(|m| m.faces.iter().copied())
            .find(|&f| {
                matches!(k.topo().face(f).unwrap().surface(), FaceSurface::Cylinder(c)
                    if c.axis().cross(Vec3::new(0.0, 1.0, 0.0)).length() < 1e-9)
            })
            .unwrap(),
    )
}

/// Entity counts, face census and volume of a solid.
fn body(k: &RemusKernel, solid: u32) -> (String, BTreeMap<&'static str, usize>, f64) {
    let id = k.resolve_solid(solid).unwrap();
    let mut census = BTreeMap::new();
    for face in solid_faces(k.topo(), id).unwrap() {
        *census
            .entry(k.topo().face(face).unwrap().surface().type_tag())
            .or_insert(0) += 1;
    }
    let volume = remus_operations::measure::solid_volume(k.topo(), id, 1e-3).unwrap();
    (
        format!("{:?}", solid_entity_counts(k.topo(), id).unwrap()),
        census,
        volume,
    )
}

fn assert_same(
    a: &(String, BTreeMap<&'static str, usize>, f64),
    b: &(String, BTreeMap<&'static str, usize>, f64),
    label: &str,
) {
    assert_eq!(a.0, b.0, "{label}: F/E/V");
    assert_eq!(a.1, b.1, "{label}: census");
    assert!(
        (a.2 - b.2).abs() <= 1e-9 * b.2,
        "{label}: volume {} vs {}",
        a.2,
        b.2
    );
}

fn batch(k: &mut RemusKernel, op: &str, args: Value) -> Value {
    let out: Value =
        serde_json::from_str(&k.execute_batch(&json!([{"op": op, "args": args}]).to_string()))
            .unwrap();
    out[0].clone()
}

/// Exercise the direct bindings' shared implementations: the batch adapter
/// adds its own transaction and would hide an operations-level leak.
#[test]
fn prepared_full_turn_sleeve_refusals_preserve_the_live_arena() {
    use remus_math::mat::Mat4;
    use remus_operations::blend_ops::fillet_v2;
    use remus_operations::boolean::{BooleanOp, boolean};
    use remus_operations::primitives::{make_box, make_cylinder};
    use remus_operations::transform::transform_solid;
    use remus_topology::edge::EdgeCurve;
    use remus_topology::explorer::solid_edges;

    let mut k = RemusKernel::new();
    let topo = k.topo_mut();
    let block = make_box(topo, 40.0, 40.0, 10.0).unwrap();
    let neighbor = make_box(topo, 10.0, 40.0, 20.0).unwrap();
    transform_solid(topo, neighbor, &Mat4::translation(30.0, 0.0, 0.0)).unwrap();
    let block = boolean(topo, BooleanOp::Fuse, block, neighbor).unwrap();
    let boss = make_cylinder(topo, 5.0, 10.0).unwrap();
    transform_solid(topo, boss, &Mat4::translation(20.0, 20.0, 10.0)).unwrap();
    let solid = boolean(topo, BooleanOp::Fuse, block, boss).unwrap();
    let shoulder = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&e| {
            matches!(topo.edge(e).unwrap().curve(), EdgeCurve::Circle(c)
            if (c.radius() - 5.0).abs() < 1e-7 && (c.center().z() - 10.0).abs() < 1e-7)
        })
        .unwrap();
    let blended = fillet_v2(topo, solid, &[shoulder], 1.0).unwrap();
    assert!(!blended.is_partial);
    let solid = blended.solid;
    let face = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .find(|&f| {
            matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(c)
            if (c.radius() - 5.0).abs() < 1e-7)
        })
        .unwrap();
    let prepared = k
        .prepare_group_edit_impl(solid_id_to_u32(solid), face_id_to_u32(face), None)
        .unwrap();
    let before = crate::bindings::transaction_tests::live_state(k.topo());
    let bytes = k.serialize_solids(&[solid_id_to_u32(solid)]).unwrap();
    for radius in [12.0, 25.0, 40.0] {
        for commit in [false, true] {
            let error = if commit {
                k.commit_prepared_group_edit_impl(prepared, radius)
            } else {
                k.preview_prepared_group_edit_impl(prepared, radius)
            }
            .unwrap_err();
            assert!(
                error.contains(if radius.to_bits() == 25.0_f64.to_bits() {
                    "exact-only"
                } else {
                    "cylindrical resize produced volume"
                }),
                "{error}"
            );
            assert_eq!(
                crate::bindings::transaction_tests::live_state(k.topo()),
                before,
                "R{radius}, commit={commit}"
            );
            assert_eq!(
                k.serialize_solids(&[solid_id_to_u32(solid)]).unwrap(),
                bytes
            );
        }
    }
    assert!(k.commit_prepared_group_edit_impl(prepared, 8.0).is_ok());
    assert!(k.release_prepared_group_edit(prepared));
}

/// A boss-diameter drag the way OpenZCAD runs it: prepare at drag start,
/// restore the checkpoint and preview each frame, commit journaled at the
/// end. Each frame equals `resizeCylindricalFace` in a second kernel at the
/// same value; the commit equals `resizeCylindricalFaceJournaled` (same JSON
/// shape, same body); cancel leaves the source document unchanged.
#[test]
fn boss_drag_previews_and_commits_like_the_unprepared_calls() {
    let mut k = RemusKernel::new();
    let solid = fixture(&mut k);
    let face = boss(&k, solid);
    let mut plain = RemusKernel::new();
    assert_eq!(fixture(&mut plain), solid);
    let source = k.serialize_solids(&[solid]).unwrap();

    let checkpoint = k.checkpoint_impl().unwrap();
    let plain_checkpoint = plain.checkpoint_impl().unwrap();
    let prepared = k.prepare_group_edit_impl(solid, face, None).unwrap();
    for diameter in [14.6, 17.0, 21.1, 14.7] {
        let radius = diameter / 2.0;
        k.restore_checkpoint_impl(checkpoint).unwrap();
        plain.restore_checkpoint_impl(plain_checkpoint).unwrap();
        let frame = batch(
            &mut k,
            "previewPreparedGroupEdit",
            json!({"prepared": prepared, "value": radius}),
        );
        let frame = u32::try_from(frame["ok"].as_u64().unwrap()).unwrap();
        let reference = batch(
            &mut plain,
            "resizeCylindricalFace",
            json!({"solid": solid, "face": face, "radius": radius}),
        );
        let reference = u32::try_from(reference["ok"].as_u64().unwrap()).unwrap();
        assert_same(
            &body(&k, frame),
            &body(&plain, reference),
            &format!("frame Ø{diameter}"),
        );
    }
    // Cancel: the drag's checkpoint is restored and the handle released.
    k.restore_checkpoint_impl(checkpoint).unwrap();
    assert_eq!(k.serialize_solids(&[solid]).unwrap(), source);
    assert!(k.release_prepared_group_edit(prepared));
    assert!(!k.release_prepared_group_edit(prepared));
    assert!(k.preview_prepared_group_edit_impl(prepared, 7.3).is_err());

    // A kept value: the journaled commit equals the unprepared journaled
    // call.
    let prepared = k.prepare_group_edit_impl(solid, face, None).unwrap();
    plain.restore_checkpoint_impl(plain_checkpoint).unwrap();
    let committed = batch(
        &mut k,
        "commitPreparedGroupEditJournaled",
        json!({"prepared": prepared, "value": 7.3}),
    );
    let reference = batch(
        &mut plain,
        "resizeCylindricalFaceJournaled",
        json!({"solid": solid, "face": face, "radius": 7.3}),
    );
    let (committed, reference) = (&committed["ok"], &reference["ok"]);
    assert_eq!(
        committed.as_object().unwrap().keys().collect::<Vec<_>>(),
        reference.as_object().unwrap().keys().collect::<Vec<_>>()
    );
    assert_same(
        &body(
            &k,
            u32::try_from(committed["solid"].as_u64().unwrap()).unwrap(),
        ),
        &body(
            &plain,
            u32::try_from(reference["solid"].as_u64().unwrap()).unwrap(),
        ),
        "journaled commit Ø14.6",
    );
    // The committed edit changed the source's session state: the
    // preparation is stale only if the source itself changed, and it did
    // not (a new solid was made).
    assert!(k.preview_prepared_group_edit_impl(prepared, 7.4).is_ok());
}

/// A band-radius drag through the direct methods, and the typed refusals.
#[test]
fn band_drag_and_refusals() {
    let mut k = RemusKernel::new();
    let solid = fixture(&mut k);
    let band = x0_band(&k, solid);
    let mut plain = RemusKernel::new();
    fixture(&mut plain);
    let checkpoint = k.checkpoint_impl().unwrap();
    let plain_checkpoint = plain.checkpoint_impl().unwrap();

    let error = k
        .prepare_group_edit_impl(solid, band, Some(3.0))
        .unwrap_err();
    assert!(error.starts_with("blend-radius-mismatch:"), "{error}");
    let plane = face_id_to_u32(
        solid_faces(k.topo(), k.resolve_solid(solid).unwrap())
            .unwrap()
            .into_iter()
            .find(|&f| {
                matches!(
                    k.topo().face(f).unwrap().surface(),
                    FaceSurface::Plane { .. }
                )
            })
            .unwrap(),
    );
    let error = k.prepare_group_edit_impl(solid, plane, None).unwrap_err();
    assert!(error.starts_with("blend-cluster-not-prepared:"), "{error}");

    let prepared = k.prepare_group_edit_impl(solid, band, Some(4.0)).unwrap();
    for radius in [1.5, 1.9] {
        k.restore_checkpoint_impl(checkpoint).unwrap();
        let frame = k
            .preview_prepared_group_edit_impl(prepared, radius)
            .unwrap();
        plain.restore_checkpoint_impl(plain_checkpoint).unwrap();
        let (solid_id, band_id) = (
            plain.resolve_solid(solid).unwrap(),
            plain.resolve_face(band).unwrap(),
        );
        let expected = remus_operations::resize_blend::resize_blend(
            plain.topo_mut(),
            solid_id,
            band_id,
            4.0,
            radius,
        )
        .unwrap();
        let expected = solid_id_to_u32(expected.solid);
        assert_same(
            &body(&k, frame),
            &body(&plain, expected),
            &format!("band frame R{radius}"),
        );
    }
    k.restore_checkpoint_impl(checkpoint).unwrap();
    let kept = k.commit_prepared_group_edit_impl(prepared, 1.5).unwrap();
    assert_ne!(kept, solid);

    // Ø40 is unbuildable: the frame refuses with the diameter's code and
    // edits nothing.
    k.restore_checkpoint_impl(checkpoint).unwrap();
    let source = k.serialize_solids(&[solid]).unwrap();
    let wall = k
        .prepare_group_edit_impl(solid, boss(&k, solid), None)
        .unwrap();
    let error = k.preview_prepared_group_edit_impl(wall, 20.0).unwrap_err();
    assert!(
        error.starts_with("cylinder-diameter-unbuildable:"),
        "{error}"
    );
    assert_eq!(k.serialize_solids(&[solid]).unwrap(), source);

    // The kernel holds at most 16 live preparations.
    for _ in 2..MAX_PREPARED_GROUP_EDITS {
        k.prepare_group_edit_impl(solid, band, Some(4.0)).unwrap();
    }
    let error = k
        .prepare_group_edit_impl(solid, band, Some(4.0))
        .unwrap_err();
    assert!(error.contains("release one first"), "{error}");
}

/// The single straight edge of `solid` whose ends both satisfy `on`.
fn line_where(
    k: &RemusKernel,
    solid: remus_topology::solid::SolidId,
    on: impl Fn(remus_math::vec::Point3) -> bool,
) -> remus_topology::edge::EdgeId {
    let topo = k.topo();
    let found: Vec<_> = remus_topology::explorer::solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&edge| {
            let data = topo.edge(edge).unwrap();
            matches!(data.curve(), remus_topology::edge::EdgeCurve::Line)
                && on(topo.vertex(data.start()).unwrap().point())
                && on(topo.vertex(data.end()).unwrap().point())
        })
        .collect();
    assert_eq!(found.len(), 1);
    found[0]
}

/// Writes the cumulative Gecko body (A2 R1.9, A3 R1, A4 0.7 and A6 0.8
/// chamfer chains) as an arena document to `$REMUS_CUMULATIVE_GECKO_DOC`,
/// for `scripts/perf/prepared-group-edit-bench.mjs` to load with
/// `deserializeSolids`. Not a contract test.
#[test]
#[ignore = "writes the cumulative Gecko document for the WASM prepared-edit benchmark"]
fn write_cumulative_gecko_document() {
    use remus_operations::blend_ops::{chamfer_v2_tangent_chain, fillet_cascade};
    let path = std::env::var("REMUS_CUMULATIVE_GECKO_DOC").unwrap();
    let mut k = RemusKernel::new();
    let handle = fixture(&mut k);
    let band = k.resolve_face(x0_band(&k, handle)).unwrap();
    let mut solid = k.resolve_solid(handle).unwrap();
    solid = remus_operations::resize_blend::resize_blend(k.topo_mut(), solid, band, 4.0, 1.9)
        .unwrap()
        .solid;
    let a3 = line_where(&k, solid, |p| {
        p.x().abs() < 1e-7 && (p.y() - 18.0).abs() < 1e-7
    });
    solid = fillet_cascade(k.topo_mut(), solid, &[a3], 1.0)
        .unwrap()
        .solid;
    let a4 = line_where(&k, solid, |p| {
        (p.x() - 30.0).abs() < 1e-7 && (p.z() - 24.0).abs() < 1e-7
    });
    solid = chamfer_v2_tangent_chain(k.topo_mut(), solid, &[a4], 0.7, 0.7)
        .unwrap()
        .solid;
    let a6 = line_where(&k, solid, |p| p.y().abs() < 1e-7 && p.z().abs() < 1e-7);
    solid = chamfer_v2_tangent_chain(k.topo_mut(), solid, &[a6], 0.8, 0.8)
        .unwrap()
        .solid;
    std::fs::write(path, k.serialize_solids(&[solid_id_to_u32(solid)]).unwrap()).unwrap();
}
