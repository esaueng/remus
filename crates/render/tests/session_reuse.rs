//! PERF-R01 session qualification: reuse, correctness, failure policy, timings.
//!
//! All GPU tests gate on [`probe_adapter`] and early-return when no adapter
//! exists; compilation alone is not rendering proof. Timings are reported via
//! `println!` for the PR body — no timing assertions, only reuse-count and
//! correctness assertions.

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
use remus_render::{Camera, OffscreenSession, RenderOpts, RenderOutput, probe_adapter};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

/// Serialize GPU tests within this binary: parallel device creation crashes
/// the Vulkan driver (SIGSEGV with 32 test threads). Each GPU test holds this
/// guard for its whole body. Cross-binary parallelism (nextest, CI) still
/// applies; keep the per-test GPU load modest.
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

fn id_silhouette_bbox(out: &RenderOutput) -> Option<(u32, u32, u32, u32)> {
    let mut bbox: Option<(u32, u32, u32, u32)> = None;
    for y in 0..out.height {
        for x in 0..out.width {
            if out.face_id_at(x, y).is_some() {
                bbox = Some(match bbox {
                    None => (x, y, x, y),
                    Some((minx, miny, maxx, maxy)) => {
                        (minx.min(x), miny.min(y), maxx.max(x), maxy.max(y))
                    }
                });
            }
        }
    }
    bbox
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
fn check_frame(topo: &Topology, solid: SolidId, out: &RenderOutput, opts: &RenderOpts, name: &str) {
    let valid_ids = valid_id_set(topo, solid);
    assert_eq!(out.width, opts.width, "{name}: width");
    assert_eq!(out.height, opts.height, "{name}: height");
    assert_eq!(
        out.id_buffer.len(),
        (out.width * out.height) as usize,
        "{name}: id buffer length"
    );

    let total = (out.width * out.height) as usize;
    let drawn = non_background_pixels(out, opts.background);
    let frac = drawn as f64 / total as f64;
    assert!(
        frac > 0.05,
        "{name}: only {drawn}/{total} ({frac:.3}) differ from background"
    );
    assert!(
        frac < 0.98,
        "{name}: {drawn}/{total} ({frac:.3}) fill the whole frame"
    );

    let bbox = id_silhouette_bbox(out).expect("expected at least one face-id pixel");
    let (minx, miny, maxx, maxy) = bbox;
    let bw = maxx - minx + 1;
    let bh = maxy - miny + 1;
    assert!(
        bw >= 16 && bh >= 16,
        "{name}: id silhouette too small: {bw}x{bh}"
    );
    assert!(
        bw < out.width && bh < out.height,
        "{name}: id silhouette fills the entire frame: {bw}x{bh}"
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
    assert_eq!(
        out.face_id_at(out.width - 1, 0),
        None,
        "{name}: top-right corner should be background"
    );
}

#[test]
fn session_reuses_device_and_targets() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP session_reuses_device_and_targets: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let cube = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, cube);

    let mut session = OffscreenSession::new().unwrap();
    println!("session adapter: {}", session.adapter_info());
    assert!(
        session.adapter_info().contains("Vulkan")
            || session.adapter_info().contains("Dx12")
            || session.adapter_info().contains("Metal")
            || session.adapter_info().to_lowercase().contains("lavapipe")
            || session.adapter_info().to_lowercase().contains("llvmpipe")
            || !session.adapter_info().is_empty(),
        "adapter info should be non-empty: {}",
        session.adapter_info()
    );
    assert_eq!(session.target_rebuilds(), 0);
    assert_eq!(session.cached_size(), None);
    assert!(!session.is_failed());

    // Cold: first render builds targets.
    let opts = RenderOpts::new(512, 512);
    let out = session.render(&topo, cube, &cam, &opts).unwrap();
    check_frame(&topo, cube, &out, &opts, "cold 512");
    assert_eq!(session.target_rebuilds(), 1);
    assert_eq!(session.cached_size(), Some((512, 512)));
    let adapter_first = session.adapter_info().to_string();

    // Warm: same size, new camera — no target rebuild, same device.
    let mut cam2 = cam;
    cam2.eye = Point3::new(cam.eye.x() + 30.0, cam.eye.y() - 20.0, cam.eye.z() + 10.0);
    let out2 = session.render(&topo, cube, &cam2, &opts).unwrap();
    check_frame(&topo, cube, &out2, &opts, "warm camera change");
    assert_eq!(
        session.target_rebuilds(),
        1,
        "same-size render must not rebuild targets"
    );
    assert_eq!(
        session.adapter_info(),
        adapter_first,
        "device must be stable"
    );

    // Size change rebuilds once, then reuses.
    let opts_small = RenderOpts::new(256, 256);
    let out3 = session.render(&topo, cube, &cam, &opts_small).unwrap();
    check_frame(&topo, cube, &out3, &opts_small, "size change 256");
    assert_eq!(session.target_rebuilds(), 2);
    assert_eq!(session.cached_size(), Some((256, 256)));
    let out4 = session.render(&topo, cube, &cam2, &opts_small).unwrap();
    check_frame(&topo, cube, &out4, &opts_small, "warm 256");
    assert_eq!(session.target_rebuilds(), 2);
}

#[test]
fn session_matches_convenience_bit_exact() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP session_matches_convenience_bit_exact: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 30.0, 20.0, 10.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(256, 256);

    let mut session = OffscreenSession::new().unwrap();
    let a = session.render(&topo, solid, &cam, &opts).unwrap();
    let b = remus_render::render_solid_offscreen(&topo, solid, &cam, &opts).unwrap();
    assert_eq!(
        a.id_buffer, b.id_buffer,
        "session and convenience id buffers must match"
    );
    assert_eq!(a.color, b.color, "session and convenience color must match");
    println!("session and convenience agree bit-exact on 256x256 box");
}

#[test]
fn session_camera_size_edge_solid_topology_qualification() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP session_camera_size_edge_solid_topology_qualification: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    // Two independent topology documents.
    let mut topo_a = Topology::new();
    let box_a = remus_operations::primitives::make_box(&mut topo_a, 20.0, 20.0, 20.0).unwrap();
    let mut topo_b = Topology::new();
    let cyl_b = remus_operations::primitives::make_cylinder(&mut topo_b, 8.0, 20.0).unwrap();

    let cam_box = frame_camera(&topo_a, box_a);
    let cam_cyl = frame_camera(&topo_b, cyl_b);
    let mut session = OffscreenSession::new().unwrap();

    // 1. Box, edges on.
    let mut opts = RenderOpts::new(384, 384);
    opts.edges = true;
    let box_edges = session.render(&topo_a, box_a, &cam_box, &opts).unwrap();
    check_frame(&topo_a, box_a, &box_edges, &opts, "box edges on");

    // 2. Edge toggle: same scene, edges off. Id buffer must be identical
    //    (edge pass masks id writes); color must differ (edge lines darken).
    let mut opts_no_edge = opts;
    opts_no_edge.edges = false;
    let box_no_edges = session
        .render(&topo_a, box_a, &cam_box, &opts_no_edge)
        .unwrap();
    check_frame(
        &topo_a,
        box_a,
        &box_no_edges,
        &opts_no_edge,
        "box edges off",
    );
    assert_eq!(
        box_edges.id_buffer, box_no_edges.id_buffer,
        "edge toggle must not change face ids"
    );
    let differing = box_edges
        .color
        .pixels()
        .zip(box_no_edges.color.pixels())
        .filter(|(a, b)| a != b)
        .count();
    assert!(
        differing > 100,
        "edge toggle should recolor edge pixels, only {differing} bytes differ"
    );
    println!("edge toggle: {differing} color bytes differ, ids identical");
    assert_eq!(
        session.target_rebuilds(),
        1,
        "edge toggle must not rebuild targets"
    );

    // 3. Camera change on the same solid.
    let mut cam_box2 = cam_box;
    cam_box2.eye = Point3::new(
        cam_box.eye.x() - 40.0,
        cam_box.eye.y() + 25.0,
        cam_box.eye.z(),
    );
    let box_cam2 = session.render(&topo_a, box_a, &cam_box2, &opts).unwrap();
    check_frame(&topo_a, box_a, &box_cam2, &opts, "box camera 2");
    assert_eq!(session.target_rebuilds(), 1);

    // 4. Different solid in an independent document: no id leak.
    let cyl_out = session.render(&topo_b, cyl_b, &cam_cyl, &opts).unwrap();
    check_frame(&topo_b, cyl_b, &cyl_out, &opts, "cylinder other doc");
    let cyl_valid = valid_id_set(&topo_b, cyl_b);
    for &id in &cyl_out.id_buffer {
        if id != 0 {
            assert!(cyl_valid.contains(&id), "cylinder frame leaked box id {id}");
        }
    }

    // 5. Alternate back to the first document: box ids valid again.
    let box_again = session.render(&topo_a, box_a, &cam_box, &opts).unwrap();
    check_frame(
        &topo_a,
        box_a,
        &box_again,
        &opts,
        "box again after cylinder",
    );
    assert_eq!(
        box_again.id_buffer, box_edges.id_buffer,
        "returning to the same scene must reproduce the same ids (no leak)"
    );

    // 6. Background change: corners must carry the new clear color, not the old.
    let mut opts_red = opts;
    opts_red.background = [1.0, 0.0, 0.0, 1.0];
    let red_out = session.render(&topo_a, box_a, &cam_box, &opts_red).unwrap();
    assert_eq!(
        red_out.face_id_at(0, 0),
        None,
        "corner must stay background id 0"
    );
    let px = red_out.color.get_pixel(0, 0);
    // Rgba8UnormSrgb encode of linear red (1,0,0) is (255,0,0).
    assert!(
        px[0] > 200 && px[1] < 60 && px[2] < 60,
        "corner pixel should be red background, got {px:?}"
    );
    println!("background switch verified: corner pixel {px:?}");

    // 7. Size change then back: buffers resize, no stale ids.
    let opts_big = RenderOpts::new(512, 256);
    let big = session.render(&topo_a, box_a, &cam_box, &opts_big).unwrap();
    check_frame(&topo_a, box_a, &big, &opts_big, "wide 512x256");
    assert_eq!(big.id_buffer.len(), 512 * 256);
    let rebuilds_after_big = session.target_rebuilds();
    let small_again = session.render(&topo_b, cyl_b, &cam_cyl, &opts).unwrap();
    check_frame(
        &topo_b,
        cyl_b,
        &small_again,
        &opts,
        "cylinder 384 after wide",
    );
    assert_eq!(
        small_again.id_buffer.len(),
        (384 * 384) as usize,
        "id buffer must match the new size, not the old frame"
    );
    assert!(
        session.target_rebuilds() > rebuilds_after_big,
        "size change must rebuild targets"
    );
}

#[test]
fn session_depth_and_large_coordinate_precision() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP session_depth_and_large_coordinate_precision: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    // Depth: an iso box shows exactly the three front faces; back faces stay
    // hidden behind the depth-tested front shell.
    let mut topo = Topology::new();
    let cube = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, cube);
    let opts = RenderOpts::new(384, 384);
    let mut session = OffscreenSession::new().unwrap();
    let out = session.render(&topo, cube, &cam, &opts).unwrap();
    check_frame(&topo, cube, &out, &opts, "depth box");
    let seen: HashSet<u32> = out
        .id_buffer
        .iter()
        .copied()
        .filter(|&id| id != 0)
        .collect();
    assert_eq!(
        seen.len(),
        3,
        "iso box must show exactly 3 front faces with depth testing, saw {}: {seen:?}",
        seen.len()
    );
    println!("depth: visible faces {seen:?}");

    // Precision: the same box translated +1e6 must render identically when the
    // camera translates with it (render-relative-to-center keeps f32 exact).
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
    let near = session.render(&topo, cube, &cam, &opts).unwrap();
    let far = session.render(&topo_far, far_box, &cam_far, &opts).unwrap();
    check_frame(&topo_far, far_box, &far, &opts, "far box 1e6");
    assert_eq!(
        near.id_buffer, far.id_buffer,
        "RTC render at +1e6 must reproduce near-origin face ids"
    );
    assert_eq!(
        near.color, far.color,
        "RTC render at +1e6 must reproduce near-origin color"
    );
    println!("RTC precision: +1e6 render bit-identical to origin");
}

#[test]
fn session_failure_policy() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP session_failure_policy: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let cube = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let cam = frame_camera(&topo, cube);
    let mut session = OffscreenSession::new().unwrap();
    let opts = RenderOpts::new(256, 256);

    // Validation failures never poison.
    let bad = RenderOpts::new(0, 256);
    assert!(matches!(
        session.render(&topo, cube, &cam, &bad),
        Err(remus_render::RenderError::InvalidSize { .. })
    ));
    assert!(!session.is_failed());

    let over = RenderOpts::new(4097, 4096);
    assert!(matches!(
        session.render(&topo, cube, &cam, &over),
        Err(remus_render::RenderError::PixelBudgetExceeded { .. })
    ));
    assert!(!session.is_failed());

    // SizeTooLarge without tripping the pixel budget: max+1 by 1.
    let max = session.max_texture_dimension_2d();
    if u64::from(max) < 16_777_216 {
        let too_big = RenderOpts::new(max + 1, 1);
        assert!(matches!(
            session.render(&topo, cube, &cam, &too_big),
            Err(remus_render::RenderError::SizeTooLarge { .. })
        ));
        assert!(!session.is_failed());
    } else {
        println!("skip SizeTooLarge probe: max {max} already exceeds pixel budget");
    }

    // A valid render still succeeds after the validation failures.
    let out = session.render(&topo, cube, &cam, &opts).unwrap();
    check_frame(&topo, cube, &out, &opts, "post-validation-failure");
    assert!(!session.is_failed());

    // Controlled fault injection poisons the session; later calls fail closed.
    session.inject_device_loss_for_test("injected loss for PERF-R01 test");
    assert!(session.is_failed());
    assert_eq!(
        session.failure_reason(),
        Some("injected loss for PERF-R01 test")
    );
    assert!(matches!(
        session.render(&topo, cube, &cam, &opts),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    // Poisoned sessions stay failed even for otherwise-valid renders.
    assert!(matches!(
        session.render(&topo, cube, &cam, &opts),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    println!("failure policy: validation errors recover, injected loss fails closed");
}

#[test]
fn session_cold_warm_measurements() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP session_cold_warm_measurements: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(512, 512);

    // Cold device/pipeline setup (1 device + 2 pipeline variants, no targets yet).
    let t0 = Instant::now();
    let mut session = OffscreenSession::new().unwrap();
    let cold_setup = t0.elapsed();
    println!("cold OffscreenSession::new (1 device + 2 pipelines): {cold_setup:?}");
    assert_eq!(session.target_rebuilds(), 0);

    // Tessellation alone (CPU proxy via the public grouped tessellator; the
    // session tessellates per frame at the same deflection — no geometry is
    // cached across frames, per PERF-R02 out-of-scope).
    let t0 = Instant::now();
    let (_grouped, _offsets) =
        remus_operations::tessellate::tessellate_solid_grouped_with_tolerance(
            &topo,
            solid,
            opts.deflection,
            remus_math::chord::DEFAULT_ANGULAR_TOL,
        )
        .unwrap();
    let tess = t0.elapsed();
    println!("tessellation grouped (512 box, CPU proxy): {tess:?}");

    // First session render: tessellation + target build + upload/render/readback.
    let t0 = Instant::now();
    let first = session.render(&topo, solid, &cam, &opts).unwrap();
    let first_total = t0.elapsed();
    println!("session first render (tess + target build + frame): {first_total:?}");
    assert_eq!(session.target_rebuilds(), 1);
    let _ = first;

    // Warm session renders: same device, pipelines, and targets.
    let t0 = Instant::now();
    let n = 5;
    for _ in 0..n {
        let _ = session.render(&topo, solid, &cam, &opts).unwrap();
    }
    let warm_total = t0.elapsed();
    println!(
        "session {n}x warm renders same device/targets: {warm_total:?} (avg {:?})",
        warm_total / n
    );
    assert_eq!(
        session.target_rebuilds(),
        1,
        "warm renders must not rebuild"
    );

    // Convenience path: cold device + pipelines per render (N=2 to bound time).
    let t0 = Instant::now();
    for _ in 0..2 {
        let _ = remus_render::render_solid_offscreen(&topo, solid, &cam, &opts).unwrap();
    }
    let convenience_total = t0.elapsed();
    println!(
        "convenience 2x renders (cold device each): {convenience_total:?} (avg {:?})",
        convenience_total / 2
    );

    println!(
        "adapter: {} | target rebuilds after {} warm: {}",
        session.adapter_info(),
        n,
        session.target_rebuilds()
    );
    println!("note: renderer reuse only — not an exact-kernel speedup claim");
}
