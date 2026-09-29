//! Partial PERF-R02 qualification: prepared-render assets.
//!
//! All GPU tests gate on [`probe_adapter`] and early-return when no adapter
//! exists; compilation alone is not rendering proof. Timings are reported via
//! `println!` for the PR body — no timing assertions, only counter and
//! correctness assertions.
//!
//! Runtime coverage available here: discrete GPU (Vulkan) only. Not covered:
//! the software-fallback backend, the window viewer, and physical device loss
//! (injected loss only) — reported explicitly, see the PR body.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    clippy::panic
)]

use std::collections::HashSet;
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_render::{
    Camera, OffscreenSession, PreparedDrawOpts, RenderOpts, RenderOutput, probe_adapter,
};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Serialize GPU tests within this binary: parallel device creation crashes
/// the Vulkan driver. Each GPU test holds this guard for its whole body.
fn gpu_guard() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(())).lock().unwrap()
}

fn iso_camera(target: Point3, radius: f64) -> Camera {
    let fov_y = 40.0_f64.to_radians();
    let dist = radius / (fov_y * 0.5).sin() * 2.0;
    let dir = Vec3::new(1.0, 0.9, 1.1).normalize().unwrap();
    let eye = target + Vec3::new(dir.x() * dist, dir.y() * dist, dir.z() * dist);
    Camera {
        eye,
        target,
        up: Vec3::new(0.0, 0.0, 1.0),
        fov_y,
        aspect: 1.0,
        near: (dist - radius).max(radius * 0.01),
        far: dist + radius * 4.0,
    }
}

fn linear_to_srgb_u8(c: f32) -> u8 {
    let c = c.clamp(0.0, 1.0);
    let s = if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    (s * 255.0).round() as u8
}

fn non_background_pixels(out: &RenderOutput, bg: [f32; 4]) -> usize {
    let bg_rgb = [
        linear_to_srgb_u8(bg[0]),
        linear_to_srgb_u8(bg[1]),
        linear_to_srgb_u8(bg[2]),
    ];
    let mut count = 0;
    for px in out.color.pixels() {
        let d = (i32::from(px[0]) - i32::from(bg_rgb[0])).abs()
            + (i32::from(px[1]) - i32::from(bg_rgb[1])).abs()
            + (i32::from(px[2]) - i32::from(bg_rgb[2])).abs();
        if d > 12 {
            count += 1;
        }
    }
    count
}

fn solid_aabb(topo: &Topology, solid: SolidId) -> (Point3, Point3) {
    let (mesh, _) = remus_operations::tessellate::tessellate_solid_grouped_with_tolerance(
        topo,
        solid,
        0.05,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .unwrap();
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for p in &mesh.positions {
        let c = [p.x(), p.y(), p.z()];
        for i in 0..3 {
            min[i] = min[i].min(c[i]);
            max[i] = max[i].max(c[i]);
        }
    }
    (
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], max[1], max[2]),
    )
}

fn frame_camera(topo: &Topology, solid: SolidId) -> Camera {
    let (min, max) = solid_aabb(topo, solid);
    let center = Point3::new(
        (min.x() + max.x()) * 0.5,
        (min.y() + max.y()) * 0.5,
        (min.z() + max.z()) * 0.5,
    );
    let radius =
        ((max.x() - min.x()).powi(2) + (max.y() - min.y()).powi(2) + (max.z() - min.z()).powi(2))
            .sqrt()
            * 0.5;
    iso_camera(center, radius)
}

fn valid_id_set(topo: &Topology, solid: SolidId) -> HashSet<u32> {
    solid_faces(topo, solid)
        .unwrap()
        .iter()
        .map(|f| u32::try_from(f.index()).unwrap() + 1)
        .collect()
}

/// Shared correctness bar for one frame: non-blank color, plausible id
/// silhouette, every id maps to a real face, corners are background.
fn check_frame(
    topo: &Topology,
    solid: SolidId,
    out: &RenderOutput,
    bg: [f32; 4],
    width: u32,
    height: u32,
    name: &str,
) {
    let valid_ids = valid_id_set(topo, solid);
    assert_eq!(out.width, width, "{name}: width");
    assert_eq!(out.height, height, "{name}: height");
    assert_eq!(
        out.id_buffer.len(),
        (width * height) as usize,
        "{name}: id buffer length"
    );

    let total = (width * height) as usize;
    let drawn = non_background_pixels(out, bg);
    let frac = drawn as f64 / total as f64;
    assert!(
        frac > 0.05,
        "{name}: only {drawn}/{total} ({frac:.3}) differ from background"
    );
    assert!(
        frac < 0.98,
        "{name}: {drawn}/{total} ({frac:.3}) fill the whole frame"
    );

    let mut seen: HashSet<u32> = HashSet::new();
    for &id in &out.id_buffer {
        if id != 0 {
            assert!(
                valid_ids.contains(&id),
                "{name}: id {id} is not a face of this solid (leak?)"
            );
            seen.insert(id);
        }
    }
    assert!(
        !seen.is_empty(),
        "{name}: no face-id pixels mapped to a real face"
    );
    assert_eq!(
        out.face_id_at(0, 0),
        None,
        "{name}: top-left corner should be background"
    );
}

/// Frame-option conversion drops `deflection` (baked at prepare time) and
/// keeps every per-frame knob. Runs without a GPU.
#[test]
#[allow(clippy::float_cmp)]
fn frame_opts_conversion_drops_deflection() {
    let render_opts = RenderOpts {
        deflection: 0.001,
        ..RenderOpts::new(320, 240)
    };
    let frame = PreparedDrawOpts::from(&render_opts);
    assert_eq!(frame.width, 320);
    assert_eq!(frame.height, 240);
    assert_eq!(frame.edges, render_opts.edges);
    assert_eq!(frame.background, render_opts.background);
    assert_eq!(frame.ambient, render_opts.ambient);
}

#[test]
#[allow(clippy::float_cmp)]
fn prepared_matches_session_and_convenience_bit_exact() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP prepared_matches_session_and_convenience_bit_exact: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 30.0, 20.0, 10.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(256, 256);

    let mut session = OffscreenSession::new().unwrap();
    let asset = session.prepare(&topo, solid, opts.deflection).unwrap();
    assert_eq!(session.prepare_calls(), 1);
    assert_eq!(session.prepared_draw_calls(), 0);
    assert_eq!(asset.session_id(), session.session_id());
    assert_eq!(asset.deflection(), opts.deflection);
    assert!(asset.has_edges());
    let stats = asset.stats();
    assert!(stats.triangles > 0 && stats.vertices > 0);
    // PERF-R03 per-face indexed: vertices are shared within a face (box: 12
    // tris, 24 verts = 6 faces x 4), never expanded to 3 per triangle.
    assert_eq!(stats.triangles, 12);
    assert_eq!(stats.vertices, 24);
    assert!(stats.vertices < stats.triangles * 3);
    assert!(stats.vertex_bytes > 0 && stats.index_bytes > 0);
    println!(
        "prepared box: {} triangles, {} vertices, {} edge segments, {} + {} + {} upload bytes",
        stats.triangles,
        stats.vertices,
        stats.edge_segments,
        stats.vertex_bytes,
        stats.index_bytes,
        stats.edge_bytes
    );

    let frame = PreparedDrawOpts::from(&opts);
    let prepared = session.draw_prepared(&asset, &cam, &frame).unwrap();
    assert_eq!(session.prepared_draw_calls(), 1);
    check_frame(
        &topo,
        solid,
        &prepared,
        opts.background,
        256,
        256,
        "prepared",
    );

    let direct = session.render(&topo, solid, &cam, &opts).unwrap();
    assert_eq!(
        prepared.id_buffer, direct.id_buffer,
        "prepared and session-render id buffers must match"
    );
    assert_eq!(
        prepared.color, direct.color,
        "prepared and session color must match"
    );

    let convenience = remus_render::render_solid_offscreen(&topo, solid, &cam, &opts).unwrap();
    assert_eq!(
        prepared.id_buffer, convenience.id_buffer,
        "prepared and convenience id buffers must match"
    );
    assert_eq!(
        prepared.color, convenience.color,
        "prepared and convenience color must match"
    );
    println!("parity: prepared == session == convenience, bit-exact");
}

#[test]
fn camera_only_draws_skip_tessellation_and_upload() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP camera_only_draws_skip_tessellation_and_upload: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(384, 384);
    let frame = PreparedDrawOpts::from(&opts);

    let mut session = OffscreenSession::new().unwrap();
    let asset = session.prepare(&topo, solid, opts.deflection).unwrap();

    // Five draws from five well-separated cameras on one asset.
    let mut moved_ids = None;
    for (i, shift) in [
        (45.0, -30.0, 15.0),
        (-50.0, 40.0, 0.0),
        (20.0, 60.0, -25.0),
        (-15.0, -55.0, 35.0),
        (60.0, 10.0, -40.0),
    ]
    .iter()
    .enumerate()
    {
        let moved = Camera {
            eye: Point3::new(
                cam.eye.x() + shift.0,
                cam.eye.y() + shift.1,
                cam.eye.z() + shift.2,
            ),
            ..cam
        };
        let out = session.draw_prepared(&asset, &moved, &frame).unwrap();
        check_frame(
            &topo,
            solid,
            &out,
            opts.background,
            384,
            384,
            &format!("orbit {i}"),
        );
        if i == 0 {
            moved_ids = Some(out.id_buffer);
        }
    }
    // Returning to the start camera reproduces its frame — and differs from
    // the moved frames, proving the camera is honored, not ignored.
    let home = session.draw_prepared(&asset, &cam, &frame).unwrap();
    check_frame(&topo, solid, &home, opts.background, 384, 384, "home");
    assert_ne!(
        home.id_buffer,
        moved_ids.unwrap(),
        "a moved camera must render differently from the start camera"
    );

    assert_eq!(session.prepare_calls(), 1, "one prepare total");
    assert_eq!(
        session.prepared_draw_calls(),
        6,
        "six draws, zero retessellations"
    );
    assert_eq!(
        session.target_rebuilds(),
        1,
        "same-size prepared draws must not rebuild targets"
    );
    let prep = session.last_prepare_timings().unwrap();
    let draw = session.last_draw_timings().unwrap();
    println!(
        "1 prepare (tessellate {:?} + upload {:?}) served 6 draws (last: submit {:?} + readback {:?})",
        prep.tessellate, prep.upload, draw.submit, draw.readback
    );
}

#[test]
fn frame_options_change_without_rebuild() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP frame_options_change_without_rebuild: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let mut session = OffscreenSession::new().unwrap();
    let asset = session.prepare(&topo, solid, 0.05).unwrap();

    // 1. Edge toggle: ids identical (edge pass masks id writes), color differs.
    let with_edges = PreparedDrawOpts::new(384, 384);
    let edged = session.draw_prepared(&asset, &cam, &with_edges).unwrap();
    check_frame(
        &topo,
        solid,
        &edged,
        with_edges.background,
        384,
        384,
        "edges on",
    );
    let mut no_edges = PreparedDrawOpts::new(384, 384);
    no_edges.edges = false;
    let plain = session.draw_prepared(&asset, &cam, &no_edges).unwrap();
    check_frame(
        &topo,
        solid,
        &plain,
        no_edges.background,
        384,
        384,
        "edges off",
    );
    assert_eq!(
        edged.id_buffer, plain.id_buffer,
        "edge toggle must not change ids"
    );
    let differing = edged
        .color
        .pixels()
        .zip(plain.color.pixels())
        .filter(|(a, b)| a != b)
        .count();
    assert!(differing > 100, "edge toggle should recolor edge pixels");
    assert_eq!(session.prepare_calls(), 1);

    // 2. Background change: corners carry the new clear color; ids unchanged.
    let mut red = PreparedDrawOpts::new(384, 384);
    red.background = [1.0, 0.0, 0.0, 1.0];
    let red_out = session.draw_prepared(&asset, &cam, &red).unwrap();
    assert_eq!(red_out.face_id_at(0, 0), None);
    assert_eq!(
        red_out.id_buffer, edged.id_buffer,
        "background must not change ids"
    );
    let px = red_out.color.get_pixel(0, 0);
    assert!(
        px[0] > 200 && px[1] < 60 && px[2] < 60,
        "corner should be red, got {px:?}"
    );

    // 3. Size change rebuilds targets once, then reuses; buffers match sizes.
    let wide = PreparedDrawOpts::new(512, 256);
    let wide_out = session.draw_prepared(&asset, &cam, &wide).unwrap();
    check_frame(&topo, solid, &wide_out, wide.background, 512, 256, "wide");
    assert_eq!(wide_out.id_buffer.len(), 512 * 256);
    let rebuilds = session.target_rebuilds();
    assert!(
        rebuilds >= 2,
        "size change must rebuild targets (saw {rebuilds})"
    );
    let wide_again = session.draw_prepared(&asset, &cam, &wide).unwrap();
    assert_eq!(
        wide_again.id_buffer, wide_out.id_buffer,
        "same frame must reproduce"
    );
    assert_eq!(
        session.target_rebuilds(),
        rebuilds,
        "same-size draw must not rebuild"
    );
    assert_eq!(session.prepare_calls(), 1, "no frame option retessellates");
    println!("frame options: edges/background/size all served from one prepare");
}

#[test]
fn independent_documents_with_identical_ids_do_not_alias() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP independent_documents_with_identical_ids_do_not_alias: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    // Two independent documents; each first solid has the same numeric id, but
    // assets are explicit handles — never a global SolidId-keyed cache.
    let mut topo_a = Topology::new();
    let box_a = remus_operations::primitives::make_box(&mut topo_a, 20.0, 20.0, 20.0).unwrap();
    let mut topo_b = Topology::new();
    let cyl_b = remus_operations::primitives::make_cylinder(&mut topo_b, 8.0, 20.0).unwrap();
    assert_eq!(
        box_a.index(),
        cyl_b.index(),
        "precondition: identical numeric ids"
    );

    let cam_box = frame_camera(&topo_a, box_a);
    let cam_cyl = frame_camera(&topo_b, cyl_b);
    let mut session = OffscreenSession::new().unwrap();
    let asset_box = session.prepare(&topo_a, box_a, 0.05).unwrap();
    let asset_cyl = session.prepare(&topo_b, cyl_b, 0.05).unwrap();
    assert_eq!(session.prepare_calls(), 2);

    let frame = PreparedDrawOpts::new(384, 384);
    let box_out = session.draw_prepared(&asset_box, &cam_box, &frame).unwrap();
    check_frame(
        &topo_a,
        box_a,
        &box_out,
        frame.background,
        384,
        384,
        "box doc",
    );
    let cyl_out = session.draw_prepared(&asset_cyl, &cam_cyl, &frame).unwrap();
    check_frame(
        &topo_b,
        cyl_b,
        &cyl_out,
        frame.background,
        384,
        384,
        "cylinder doc",
    );
    assert_ne!(
        box_out.id_buffer, cyl_out.id_buffer,
        "different documents must not alias through identical numeric ids"
    );
    // Alternating draws stay valid — no cross-talk between assets.
    let box_again = session.draw_prepared(&asset_box, &cam_box, &frame).unwrap();
    assert_eq!(box_again.id_buffer, box_out.id_buffer);
    assert_eq!(session.prepare_calls(), 2);
    println!("two documents, identical numeric ids, zero aliasing");
}

#[test]
fn replacement_picks_up_topology_edits_snapshot_stays_put() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP replacement_picks_up_topology_edits_snapshot_stays_put: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let frame = PreparedDrawOpts::new(384, 384);
    let mut session = OffscreenSession::new().unwrap();

    let asset = session.prepare(&topo, solid, 0.05).unwrap();
    let before = session.draw_prepared(&asset, &cam, &frame).unwrap();
    check_frame(
        &topo,
        solid,
        &before,
        frame.background,
        384,
        384,
        "before edit",
    );

    // Edit the topology after preparing: the snapshot must not move.
    remus_operations::transform::transform_solid(
        &mut topo,
        solid,
        &Mat4::translation(30.0, 0.0, 0.0),
    )
    .unwrap();
    let stale = session.draw_prepared(&asset, &cam, &frame).unwrap();
    assert_eq!(
        stale.id_buffer, before.id_buffer,
        "prepared asset is a snapshot: topology edits do not move it"
    );
    assert_eq!(stale.color, before.color);

    // Replacement is explicit: prepare again, drop the old asset.
    let mut replacement = session.prepare(&topo, solid, 0.05).unwrap();
    let after = session.draw_prepared(&replacement, &cam, &frame).unwrap();
    assert_ne!(
        after.id_buffer, before.id_buffer,
        "replacement prepare must pick up the topology edit"
    );
    drop(asset);
    // Reassigning releases the previous asset; the newest snapshot still draws.
    replacement = session.prepare(&topo, solid, 0.05).unwrap();
    let newest = session.draw_prepared(&replacement, &cam, &frame).unwrap();
    assert_eq!(newest.id_buffer, after.id_buffer);
    drop(replacement);
    // The session is unaffected by asset release: direct renders still work.
    let opts = RenderOpts::new(384, 384);
    let direct = session.render(&topo, solid, &cam, &opts).unwrap();
    check_frame(
        &topo,
        solid,
        &direct,
        opts.background,
        384,
        384,
        "post-release",
    );
    println!("snapshot held across edit; explicit replacement picked it up; release clean");
}

#[test]
#[allow(clippy::float_cmp)]
fn changed_preparation_options_change_geometry() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP changed_preparation_options_change_geometry: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    // A curved solid: deflection changes the tessellation census.
    let mut topo = Topology::new();
    let cyl = remus_operations::primitives::make_cylinder(&mut topo, 8.0, 20.0).unwrap();
    let cam = frame_camera(&topo, cyl);
    let mut session = OffscreenSession::new().unwrap();

    let fine = session.prepare(&topo, cyl, 0.02).unwrap();
    let coarse = session.prepare(&topo, cyl, 0.5).unwrap();
    assert_eq!(fine.deflection(), 0.02);
    assert_eq!(coarse.deflection(), 0.5);
    assert!(
        fine.stats().triangles > coarse.stats().triangles,
        "finer deflection must tessellate more triangles ({} vs {})",
        fine.stats().triangles,
        coarse.stats().triangles
    );
    println!(
        "cylinder census: fine {} tris / coarse {} tris",
        fine.stats().triangles,
        coarse.stats().triangles
    );

    let frame = PreparedDrawOpts::new(384, 384);
    let fine_out = session.draw_prepared(&fine, &cam, &frame).unwrap();
    check_frame(&topo, cyl, &fine_out, frame.background, 384, 384, "fine");
    let coarse_out = session.draw_prepared(&coarse, &cam, &frame).unwrap();
    check_frame(
        &topo,
        cyl,
        &coarse_out,
        frame.background,
        384,
        384,
        "coarse",
    );

    // Deflection is not a frame option: converting RenderOpts with a different
    // deflection draws the asset's baked geometry bit-exact.
    let mut other_deflection = RenderOpts::new(384, 384);
    other_deflection.deflection = 0.9;
    let converted = PreparedDrawOpts::from(&other_deflection);
    let via_converted = session.draw_prepared(&fine, &cam, &converted).unwrap();
    assert_eq!(via_converted.id_buffer, fine_out.id_buffer);
    assert_eq!(via_converted.color, fine_out.color);
    assert_eq!(session.prepare_calls(), 2);
}

#[test]
fn large_coordinate_prepared_render_matches() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP large_coordinate_prepared_render_matches: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let cube = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, cube);
    let frame = PreparedDrawOpts::new(384, 384);

    let offset = 1.0e6;
    let mut topo_far = Topology::new();
    let far_box = remus_operations::primitives::make_box(&mut topo_far, 20.0, 20.0, 20.0).unwrap();
    remus_operations::transform::transform_solid(
        &mut topo_far,
        far_box,
        &Mat4::translation(offset, offset, offset),
    )
    .unwrap();
    let shift = Vec3::new(offset, offset, offset);
    let cam_far = Camera {
        eye: cam.eye + shift,
        target: cam.target + shift,
        ..cam
    };

    let mut session = OffscreenSession::new().unwrap();
    let near_asset = session.prepare(&topo, cube, 0.05).unwrap();
    let far_asset = session.prepare(&topo_far, far_box, 0.05).unwrap();
    let near = session.draw_prepared(&near_asset, &cam, &frame).unwrap();
    let far = session.draw_prepared(&far_asset, &cam_far, &frame).unwrap();
    check_frame(
        &topo_far,
        far_box,
        &far,
        frame.background,
        384,
        384,
        "far 1e6",
    );
    assert_eq!(near.id_buffer, far.id_buffer, "RTC ids must match at +1e6");
    assert_eq!(near.color, far.color, "RTC color must match at +1e6");
    println!("RTC precision via prepared path: +1e6 bit-identical");
}

#[test]
fn prepared_invalid_released_and_device_loss() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP prepared_invalid_released_and_device_loss: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let cube = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let cam = frame_camera(&topo, cube);
    let mut session = OffscreenSession::new().unwrap();
    let asset = session.prepare(&topo, cube, 0.05).unwrap();
    let frame = PreparedDrawOpts::new(256, 256);

    // Invalid frame sizes never poison: the asset and session stay usable.
    let bad = PreparedDrawOpts::new(0, 256);
    assert!(matches!(
        session.draw_prepared(&asset, &cam, &bad),
        Err(remus_render::RenderError::InvalidSize { .. })
    ));
    let over = PreparedDrawOpts::new(4097, 4096);
    assert!(matches!(
        session.draw_prepared(&asset, &cam, &over),
        Err(remus_render::RenderError::PixelBudgetExceeded { .. })
    ));
    assert!(!session.is_failed());
    let max = session.max_texture_dimension_2d();
    if u64::from(max) < 16_777_216 {
        let too_big = PreparedDrawOpts::new(max + 1, 1);
        assert!(matches!(
            session.draw_prepared(&asset, &cam, &too_big),
            Err(remus_render::RenderError::SizeTooLarge { .. })
        ));
        assert!(!session.is_failed());

        // Direct rendering must reject size before tessellation or GPU upload.
        let empty = Topology::new();
        assert!(matches!(
            session.render(&empty, cube, &cam, &RenderOpts::new(max + 1, 1)),
            Err(remus_render::RenderError::SizeTooLarge { .. })
        ));
        assert!(!session.is_failed());
    }
    let ok = session.draw_prepared(&asset, &cam, &frame).unwrap();
    check_frame(&topo, cube, &ok, frame.background, 256, 256, "post-invalid");

    // Wrong session: refused before any GPU work, both sessions stay usable.
    let mut other = OffscreenSession::new().unwrap();
    assert_ne!(session.session_id(), other.session_id());
    match other.draw_prepared(&asset, &cam, &frame) {
        Err(remus_render::RenderError::WrongSession { expected, actual }) => {
            assert_eq!(expected, session.session_id());
            assert_eq!(actual, other.session_id());
        }
        Err(other_err) => panic!("expected WrongSession, got {other_err:?}"),
        Ok(_) => panic!("expected WrongSession, got Ok"),
    }
    assert!(!other.is_failed());
    assert!(!session.is_failed());
    // Preparing on the second session and drawing there works (affinity fix).
    let other_asset = other.prepare(&topo, cube, 0.05).unwrap();
    let other_out = other.draw_prepared(&other_asset, &cam, &frame).unwrap();
    check_frame(
        &topo,
        cube,
        &other_out,
        frame.background,
        256,
        256,
        "other session",
    );
    assert_eq!(
        other_out.id_buffer, ok.id_buffer,
        "same scene, either session: same ids"
    );

    // Released assets: dropping one asset leaves the session serving the rest.
    let extra = session.prepare(&topo, cube, 0.05).unwrap();
    drop(extra);
    drop(asset);
    let again = session.draw_prepared(&other_asset, &cam, &frame);
    assert!(
        matches!(again, Err(remus_render::RenderError::WrongSession { .. })),
        "cross-session asset still refused after local releases"
    );
    let opts = RenderOpts::new(256, 256);
    let direct = session.render(&topo, cube, &cam, &opts).unwrap();
    check_frame(
        &topo,
        cube,
        &direct,
        opts.background,
        256,
        256,
        "post-release",
    );

    // Existing device-loss behavior extends to the prepared paths: both fail
    // closed, and the session stays failed.
    session.inject_device_loss_for_test("injected loss for PERF-R02 test");
    assert!(session.is_failed());
    assert!(matches!(
        session.draw_prepared(&other_asset, &cam, &frame),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    assert!(matches!(
        session.prepare(&topo, cube, 0.05),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    assert!(matches!(
        session.render(&topo, cube, &cam, &opts),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    println!("invalid input recovers; wrong session refuses; loss fails closed");
}

#[test]
fn prepared_camera_only_measurements() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP prepared_camera_only_measurements: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(512, 512);
    let frame = PreparedDrawOpts::from(&opts);

    let mut session = OffscreenSession::new().unwrap();
    println!("session adapter: {}", session.adapter_info());

    // Preparation, split into tessellation vs. upload by the session itself.
    let t0 = Instant::now();
    let asset = session.prepare(&topo, solid, opts.deflection).unwrap();
    let prepare_total = t0.elapsed();
    let prep = session.last_prepare_timings().unwrap();
    let stats = asset.stats();
    println!(
        "prepare total {prepare_total:?} = tessellate {:?} + upload {:?} ({} tris, {} verts, {} + {} + {} bytes)",
        prep.tessellate,
        prep.upload,
        stats.triangles,
        stats.vertices,
        stats.vertex_bytes,
        stats.index_bytes,
        stats.edge_bytes
    );

    // First prepared draw, split into submit vs. readback.
    let t0 = Instant::now();
    let first = session.draw_prepared(&asset, &cam, &frame).unwrap();
    let first_total = t0.elapsed();
    let first_phases = session.last_draw_timings().unwrap();
    println!(
        "prepared first draw total {first_total:?} = submit {:?} + readback {:?}",
        first_phases.submit, first_phases.readback
    );
    let _ = first;

    // Warm camera-only draws: no tessellation, no upload, same targets.
    let t0 = Instant::now();
    let n: u32 = 5;
    for i in 0..n {
        let moved = Camera {
            eye: Point3::new(
                cam.eye.x() + f64::from(i) * 7.0,
                cam.eye.y() - f64::from(i) * 5.0,
                cam.eye.z(),
            ),
            ..cam
        };
        let _ = session.draw_prepared(&asset, &moved, &frame).unwrap();
    }
    let warm_total = t0.elapsed();
    let warm_phases = session.last_draw_timings().unwrap();
    println!(
        "prepared {n}x warm camera-only draws: {warm_total:?} (avg {:?}; last submit {:?} + readback {:?})",
        warm_total / n,
        warm_phases.submit,
        warm_phases.readback
    );
    assert_eq!(
        session.prepare_calls(),
        1,
        "warm draws must not retessellate"
    );
    assert_eq!(
        session.prepared_draw_calls(),
        1 + u64::from(n),
        "every prepared draw is counted"
    );
    assert_eq!(session.target_rebuilds(), 1, "warm draws must not rebuild");

    // Reference: direct session renders retessellate + re-upload per frame.
    let t0 = Instant::now();
    for _ in 0..2 {
        let _ = session.render(&topo, solid, &cam, &opts).unwrap();
    }
    let direct_total = t0.elapsed();
    println!(
        "direct 2x session renders (tess + upload each): {direct_total:?} (avg {:?})",
        direct_total / 2
    );

    println!(
        "adapter: {} | prepares: {} | prepared draws: {} | target rebuilds: {}",
        session.adapter_info(),
        session.prepare_calls(),
        session.prepared_draw_calls(),
        session.target_rebuilds()
    );
    println!("note: renderer reuse only — not an exact-kernel speedup claim");
}
