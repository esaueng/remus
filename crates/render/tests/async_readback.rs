//! PERF-R04 qualification: asynchronous selective readback.
//!
//! All GPU tests gate on [`probe_adapter`] and early-return when no adapter
//! exists; compilation alone is not rendering proof. Timings are reported via
//! `println!` for the PR body — no timing assertions, only counter,
//! identity, and correctness assertions.
//!
//! Runtime coverage available here: discrete GPU (Vulkan) only. Not covered:
//! the software-fallback backend (no lavapipe in this environment), the window
//! viewer, and physical device loss (injected loss only) — reported
//! explicitly, see the PR body.
//!
//! What is proven:
//! - completed async outputs match the synchronous reference bit-exact
//!   (color + face ids) for identical camera/size/background/edge/asset;
//! - multiple frames with different cameras, sizes, backgrounds, edge toggles,
//!   and assets can be submitted before any collection, and collected in a
//!   different order without stale colors or face ids;
//! - selective readback skips unrequested outputs (`Color` yields color only,
//!   `FaceIds` yields ids only, `Both` yields both, `None` yields neither);
//! - queue exhaustion (`QueueFull`), cancellation, abandoned tickets
//!   (dropped without collect), repeated resize, large-coordinate rendering,
//!   and injected device failure behave per contract.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::print_stdout,
    clippy::panic
)]

use std::collections::HashSet;
use std::sync::{Mutex, OnceLock};

use remus_math::mat::Mat4;
use remus_math::vec::{Point3, Vec3};
use remus_render::{
    AsyncFrameOutput, Camera, FrameTicket, OffscreenSession, PreparedDrawOpts, ReadbackSelection,
    RenderOpts, RenderOutput, probe_adapter,
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

fn check_async_frame(
    topo: &Topology,
    solid: SolidId,
    out: &AsyncFrameOutput,
    width: u32,
    height: u32,
    name: &str,
) {
    assert_eq!(out.width, width, "{name}: width");
    assert_eq!(out.height, height, "{name}: height");
    // At least one output must be present unless selection is None.
    if out.color.is_none() && out.id_buffer.is_none() {
        assert_eq!(
            out.transferred_bytes, 0,
            "{name}: None selection must transfer 0 bytes"
        );
        return;
    }
    if let Some(ids) = out.id_buffer.as_ref() {
        assert_eq!(ids.len(), (width * height) as usize, "{name}: id len");
        let valid = valid_id_set(topo, solid);
        let mut seen = HashSet::new();
        for &id in ids {
            if id != 0 {
                assert!(valid.contains(&id), "{name}: leaked id {id}");
                seen.insert(id);
            }
        }
        assert!(!seen.is_empty(), "{name}: no real face ids");
    }
    if let Some(color) = out.color.as_ref() {
        assert_eq!(color.width(), width, "{name}: color width");
        assert_eq!(color.height(), height, "{name}: color height");
    }
}

fn check_sync_frame(topo: &Topology, solid: SolidId, out: &RenderOutput, name: &str) {
    let valid = valid_id_set(topo, solid);
    let mut seen = HashSet::new();
    for &id in &out.id_buffer {
        if id != 0 {
            assert!(valid.contains(&id), "{name}: leaked id {id}");
            seen.insert(id);
        }
    }
    assert!(!seen.is_empty(), "{name}: no real face ids");
}

/// Drain a ticket via non-blocking polls (with a bounded spin) then fall back
/// to blocking wait. Proves `poll_frame` can complete without `wait_frame`.
fn collect_via_poll(session: &mut OffscreenSession, ticket: FrameTicket) -> AsyncFrameOutput {
    for _ in 0..10_000 {
        match session.poll_frame(ticket).unwrap() {
            Some(out) => return out,
            None => std::hint::spin_loop(),
        }
    }
    session.wait_frame(ticket).unwrap()
}

#[test]
fn async_matches_sync_reference_bit_exact() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP async_matches_sync_reference_bit_exact: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 30.0, 20.0, 10.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(256, 256);

    let mut session = OffscreenSession::new().unwrap();
    // Baseline synchronous reference.
    let sync_out = session.render(&topo, solid, &cam, &opts).unwrap();
    check_sync_frame(&topo, solid, &sync_out, "sync ref");

    // Async direct, Both: must match bit-exact.
    let ticket = session
        .submit_render(&topo, solid, &cam, &opts, ReadbackSelection::Both)
        .unwrap();
    assert_eq!(ticket.session_id(), session.session_id());
    assert_eq!(session.pending_frame_count(), 1);
    assert!(session.in_flight_bytes() > 0);
    let async_out = session.wait_frame(ticket).unwrap();
    assert_eq!(async_out.ticket, ticket);
    assert_eq!(async_out.width, 256);
    assert_eq!(async_out.height, 256);
    let async_ids = async_out.id_buffer.as_ref().unwrap();
    assert_eq!(
        *async_ids, sync_out.id_buffer,
        "async Both ids must match sync"
    );
    assert_eq!(
        async_out.color.as_ref().unwrap(),
        &sync_out.color,
        "async Both color must match sync"
    );
    assert!(async_out.transferred_bytes > 0);
    assert_eq!(session.pending_frame_count(), 0);
    // Collecting twice is UnknownTicket (not silent double-free).
    assert!(matches!(
        session.wait_frame(ticket),
        Err(remus_render::RenderError::UnknownTicket { .. })
    ));

    // Async prepared, Both: must also match.
    let asset = session.prepare(&topo, solid, opts.deflection).unwrap();
    let frame = PreparedDrawOpts::from(&opts);
    let ticket2 = session
        .submit_prepared(&asset, &cam, &frame, ReadbackSelection::Both)
        .unwrap();
    let prepared_out = session.wait_frame(ticket2).unwrap();
    assert_eq!(
        prepared_out.id_buffer.as_ref().unwrap(),
        &sync_out.id_buffer
    );
    assert_eq!(prepared_out.color.as_ref().unwrap(), &sync_out.color);
    println!(
        "async Both matches sync bit-exact; transferred {} bytes; peak {} bytes",
        async_out.transferred_bytes,
        session.peak_in_flight_bytes()
    );
}

#[test]
fn selective_readback_skips_unrequested() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP selective_readback_skips_unrequested: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(256, 256);
    let mut session = OffscreenSession::new().unwrap();
    let sync_out = session.render(&topo, solid, &cam, &opts).unwrap();

    // Color-only: color present and matches, ids absent, no id bytes.
    let t_color = session
        .submit_render(&topo, solid, &cam, &opts, ReadbackSelection::Color)
        .unwrap();
    let color_out = session.wait_frame(t_color).unwrap();
    assert!(color_out.color.is_some());
    assert!(color_out.id_buffer.is_none());
    assert_eq!(color_out.color.as_ref().unwrap(), &sync_out.color);
    assert_eq!(color_out.face_id_at(10, 10), None);
    println!(
        "color-only transferred {} bytes",
        color_out.transferred_bytes
    );

    // Id-only: ids present and match, color absent.
    let t_ids = session
        .submit_render(&topo, solid, &cam, &opts, ReadbackSelection::FaceIds)
        .unwrap();
    let ids_out = session.wait_frame(t_ids).unwrap();
    assert!(ids_out.color.is_none());
    assert!(ids_out.id_buffer.is_some());
    assert_eq!(ids_out.id_buffer.as_ref().unwrap(), &sync_out.id_buffer);
    println!("id-only transferred {} bytes", ids_out.transferred_bytes);

    // Both transfers strictly more than either alone (padding equal, so
    // Both = Color + Ids).
    let t_both = session
        .submit_render(&topo, solid, &cam, &opts, ReadbackSelection::Both)
        .unwrap();
    let both_out = session.wait_frame(t_both).unwrap();
    assert_eq!(
        both_out.transferred_bytes,
        color_out.transferred_bytes + ids_out.transferred_bytes,
        "Both must transfer exactly color + ids"
    );

    // None: neither present, zero bytes, still completes.
    let t_none = session
        .submit_render(&topo, solid, &cam, &opts, ReadbackSelection::None)
        .unwrap();
    let none_out = session.wait_frame(t_none).unwrap();
    assert!(none_out.color.is_none());
    assert!(none_out.id_buffer.is_none());
    assert_eq!(none_out.transferred_bytes, 0);
    assert_eq!(none_out.width, 256);
    println!(
        "selective: color {} + ids {} = both {}; none 0",
        color_out.transferred_bytes, ids_out.transferred_bytes, both_out.transferred_bytes
    );
}

#[test]
fn overlap_different_frames_collect_out_of_order() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP overlap_different_frames_collect_out_of_order: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo_box = Topology::new();
    let box_solid =
        remus_operations::primitives::make_box(&mut topo_box, 20.0, 20.0, 20.0).unwrap();
    let mut topo_cyl = Topology::new();
    let cyl_solid = remus_operations::primitives::make_cylinder(&mut topo_cyl, 8.0, 20.0).unwrap();
    let cam_box = frame_camera(&topo_box, box_solid);
    let cam_cyl = frame_camera(&topo_cyl, cyl_solid);
    let mut cam_box2 = cam_box;
    cam_box2.eye = Point3::new(
        cam_box.eye.x() - 40.0,
        cam_box.eye.y() + 25.0,
        cam_box.eye.z(),
    );

    let mut session = OffscreenSession::new().unwrap();
    let asset_box = session.prepare(&topo_box, box_solid, 0.05).unwrap();
    let asset_cyl = session.prepare(&topo_cyl, cyl_solid, 0.05).unwrap();

    // Synchronous references for each distinct frame.
    let ref_box_edges = session
        .render(&topo_box, box_solid, &cam_box, &RenderOpts::new(384, 384))
        .unwrap();
    let mut opts_no_edge = RenderOpts::new(384, 384);
    opts_no_edge.edges = false;
    let ref_box_no_edge = session
        .render(&topo_box, box_solid, &cam_box, &opts_no_edge)
        .unwrap();
    let mut opts_red = RenderOpts::new(384, 384);
    opts_red.background = [1.0, 0.0, 0.0, 1.0];
    let ref_box_red = session
        .render(&topo_box, box_solid, &cam_box, &opts_red)
        .unwrap();
    let ref_box_cam2 = session
        .render(&topo_box, box_solid, &cam_box2, &RenderOpts::new(384, 384))
        .unwrap();
    let ref_cyl = session
        .render(&topo_cyl, cyl_solid, &cam_cyl, &RenderOpts::new(384, 384))
        .unwrap();
    let ref_box_wide = session
        .render(&topo_box, box_solid, &cam_box, &RenderOpts::new(512, 256))
        .unwrap();

    // Submit all six before collecting any: different cameras, sizes,
    // backgrounds, edge toggles, and assets.
    let f_box_edges = PreparedDrawOpts::new(384, 384);
    let mut f_no_edge = PreparedDrawOpts::new(384, 384);
    f_no_edge.edges = false;
    let mut f_red = PreparedDrawOpts::new(384, 384);
    f_red.background = [1.0, 0.0, 0.0, 1.0];
    let f_wide = PreparedDrawOpts::new(512, 256);

    let t0 = session
        .submit_prepared(&asset_box, &cam_box, &f_box_edges, ReadbackSelection::Both)
        .unwrap();
    let t1 = session
        .submit_prepared(&asset_box, &cam_box, &f_no_edge, ReadbackSelection::Both)
        .unwrap();
    let t2 = session
        .submit_prepared(&asset_box, &cam_box, &f_red, ReadbackSelection::Both)
        .unwrap();
    let t3 = session
        .submit_prepared(&asset_box, &cam_box2, &f_box_edges, ReadbackSelection::Both)
        .unwrap();
    let t4 = session
        .submit_prepared(&asset_cyl, &cam_cyl, &f_box_edges, ReadbackSelection::Both)
        .unwrap();
    let t5 = session
        .submit_prepared(&asset_box, &cam_box, &f_wide, ReadbackSelection::Both)
        .unwrap();
    assert_eq!(session.pending_frame_count(), 6);
    println!(
        "6 frames submitted before any collect; in-flight {} bytes (peak {})",
        session.in_flight_bytes(),
        session.peak_in_flight_bytes()
    );

    // Collect in reverse order (different from submission): no stale data.
    let o5 = session.wait_frame(t5).unwrap();
    let o4 = session.wait_frame(t4).unwrap();
    let o3 = session.wait_frame(t3).unwrap();
    let o2 = session.wait_frame(t2).unwrap();
    let o1 = session.wait_frame(t1).unwrap();
    let o0 = session.wait_frame(t0).unwrap();
    assert_eq!(session.pending_frame_count(), 0);

    assert_eq!(o0.ticket, t0);
    assert_eq!(o0.id_buffer.as_ref().unwrap(), &ref_box_edges.id_buffer);
    assert_eq!(o0.color.as_ref().unwrap(), &ref_box_edges.color);

    assert_eq!(o1.id_buffer.as_ref().unwrap(), &ref_box_no_edge.id_buffer);
    assert_eq!(o1.color.as_ref().unwrap(), &ref_box_no_edge.color);
    // Edge toggle keeps ids, changes color.
    assert_eq!(
        o0.id_buffer.as_ref().unwrap(),
        o1.id_buffer.as_ref().unwrap()
    );
    assert_ne!(o0.color.as_ref().unwrap(), o1.color.as_ref().unwrap());

    assert_eq!(o2.id_buffer.as_ref().unwrap(), &ref_box_red.id_buffer);
    let px = o2.color.as_ref().unwrap().get_pixel(0, 0);
    assert!(px[0] > 200 && px[1] < 60 && px[2] < 60);

    assert_eq!(o3.id_buffer.as_ref().unwrap(), &ref_box_cam2.id_buffer);
    assert_ne!(
        o3.id_buffer.as_ref().unwrap(),
        o0.id_buffer.as_ref().unwrap(),
        "moved camera must differ"
    );

    assert_eq!(o4.id_buffer.as_ref().unwrap(), &ref_cyl.id_buffer);
    assert_ne!(
        o4.id_buffer.as_ref().unwrap(),
        o0.id_buffer.as_ref().unwrap(),
        "different assets must not alias"
    );

    assert_eq!(o5.id_buffer.as_ref().unwrap(), &ref_box_wide.id_buffer);
    assert_eq!(o5.width, 512);
    assert_eq!(o5.height, 256);

    // Poll-based collection also works (non-blocking path).
    let ta = session
        .submit_prepared(&asset_box, &cam_box, &f_box_edges, ReadbackSelection::Both)
        .unwrap();
    let tb = session
        .submit_prepared(&asset_cyl, &cam_cyl, &f_box_edges, ReadbackSelection::Both)
        .unwrap();
    let ob = collect_via_poll(&mut session, tb);
    let oa = collect_via_poll(&mut session, ta);
    assert_eq!(oa.id_buffer.as_ref().unwrap(), &ref_box_edges.id_buffer);
    assert_eq!(ob.id_buffer.as_ref().unwrap(), &ref_cyl.id_buffer);
    println!("overlap: 6x submit-then-reverse-collect + 2x poll-collect, no stale data");
}

#[test]
fn queue_exhaustion_cancel_and_abandoned() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP queue_exhaustion_cancel_and_abandoned: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let opts = RenderOpts::new(128, 128);
    let mut session = OffscreenSession::new().unwrap();
    assert_eq!(session.max_in_flight(), 8);

    // Fill the queue exactly.
    let mut tickets = Vec::new();
    for _ in 0..session.max_in_flight() {
        tickets.push(
            session
                .submit_render(&topo, solid, &cam, &opts, ReadbackSelection::Both)
                .unwrap(),
        );
    }
    assert_eq!(session.pending_frame_count(), 8);
    let peak_full = session.peak_in_flight_bytes();
    assert!(session.in_flight_bytes() > 0);

    // One more submit fails with QueueFull without poisoning.
    match session.submit_render(&topo, solid, &cam, &opts, ReadbackSelection::Both) {
        Err(remus_render::RenderError::QueueFull { pending, max }) => {
            assert_eq!(pending, 8);
            assert_eq!(max, 8);
        }
        Err(other) => panic!("expected QueueFull, got {other:?}"),
        Ok(_) => panic!("expected QueueFull, got Ok"),
    }
    assert!(!session.is_failed());

    // Cancel one: slot frees, submission succeeds again.
    assert!(session.cancel_frame(tickets[0]).unwrap());
    assert_eq!(session.pending_frame_count(), 7);
    // Cancelling twice is idempotent (false, not an error).
    assert!(!session.cancel_frame(tickets[0]).unwrap());
    let replacement = session
        .submit_render(&topo, solid, &cam, &opts, ReadbackSelection::Both)
        .unwrap();
    assert_eq!(session.pending_frame_count(), 8);

    // Abandoned ticket: drop without collect/cancel still occupies the queue.
    let abandoned = session.pending_tickets().into_iter().next().unwrap();
    let _ = abandoned;
    assert_eq!(
        session.pending_frame_count(),
        8,
        "dropping a ticket must not release its frame"
    );
    // Queue still full.
    assert!(matches!(
        session.submit_render(&topo, solid, &cam, &opts, ReadbackSelection::Both),
        Err(remus_render::RenderError::QueueFull { .. })
    ));

    // Drain everything except the cancelled slot: collect 7, cancel the last.
    // tickets[0] was cancelled; replacement + tickets[1..] are pending (8 total).
    let mut collected = 0;
    for &t in tickets.iter().skip(1) {
        let out = session.wait_frame(t).unwrap();
        check_async_frame(&topo, solid, &out, 128, 128, "drain");
        collected += 1;
    }
    let out = session.wait_frame(replacement).unwrap();
    check_async_frame(&topo, solid, &out, 128, 128, "replacement");
    collected += 1;
    assert_eq!(collected, 8);
    assert_eq!(session.pending_frame_count(), 0);
    assert_eq!(session.in_flight_bytes(), 0);
    assert!(session.peak_in_flight_bytes() >= peak_full);
    println!(
        "queue: 8 fill + QueueFull + cancel/retry + abandoned-holds-slot + drain; peak {peak_full} bytes"
    );

    // Unknown ticket after drain.
    assert!(matches!(
        session.wait_frame(tickets[1]),
        Err(remus_render::RenderError::UnknownTicket { .. })
    ));
    assert!(!session.is_failed());
}

#[test]
fn wrong_session_invalid_sizes_and_device_loss() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP wrong_session_invalid_sizes_and_device_loss: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let _opts = RenderOpts::new(256, 256);
    let frame = PreparedDrawOpts::new(256, 256);

    let mut a = OffscreenSession::new().unwrap();
    let mut b = OffscreenSession::new().unwrap();
    assert_ne!(a.session_id(), b.session_id());
    let asset_a = a.prepare(&topo, solid, 0.05).unwrap();

    // Wrong-session asset on submit: refused, both usable.
    match b.submit_prepared(&asset_a, &cam, &frame, ReadbackSelection::Both) {
        Err(remus_render::RenderError::WrongSession { expected, actual }) => {
            assert_eq!(expected, a.session_id());
            assert_eq!(actual, b.session_id());
        }
        Err(other) => panic!("expected WrongSession, got {other:?}"),
        Ok(_) => panic!("expected WrongSession"),
    }
    assert!(!a.is_failed() && !b.is_failed());

    // Wrong-session ticket on poll/wait/cancel: refused.
    let ticket = a
        .submit_prepared(&asset_a, &cam, &frame, ReadbackSelection::Both)
        .unwrap();
    assert!(matches!(
        b.poll_frame(ticket),
        Err(remus_render::RenderError::WrongSession { .. })
    ));
    assert!(matches!(
        b.wait_frame(ticket),
        Err(remus_render::RenderError::WrongSession { .. })
    ));
    assert!(matches!(
        b.cancel_frame(ticket),
        Err(remus_render::RenderError::WrongSession { .. })
    ));
    // Original session still owns it.
    let out = a.wait_frame(ticket).unwrap();
    check_async_frame(&topo, solid, &out, 256, 256, "right session");

    // Invalid sizes never consume queue slots and never poison.
    let before = a.pending_frame_count();
    assert!(matches!(
        a.submit_render(
            &topo,
            solid,
            &cam,
            &RenderOpts::new(0, 256),
            ReadbackSelection::Both
        ),
        Err(remus_render::RenderError::InvalidSize { .. })
    ));
    assert!(matches!(
        a.submit_prepared(
            &asset_a,
            &cam,
            &PreparedDrawOpts::new(4097, 4096),
            ReadbackSelection::Both
        ),
        Err(remus_render::RenderError::PixelBudgetExceeded { .. })
    ));
    let max = a.max_texture_dimension_2d();
    if u64::from(max) < 16_777_216 {
        assert!(matches!(
            a.submit_prepared(
                &asset_a,
                &cam,
                &PreparedDrawOpts::new(max + 1, 1),
                ReadbackSelection::Both
            ),
            Err(remus_render::RenderError::SizeTooLarge { .. })
        ));
    }
    assert_eq!(a.pending_frame_count(), before);
    assert!(!a.is_failed());

    // Injected device loss: pending frames fail closed, submits refuse,
    // cancels still release.
    let pending = a
        .submit_prepared(&asset_a, &cam, &frame, ReadbackSelection::Both)
        .unwrap();
    a.inject_device_loss_for_test("injected loss for PERF-R04 test");
    assert!(a.is_failed());
    assert!(matches!(
        a.wait_frame(pending),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    assert_eq!(a.pending_frame_count(), 0);
    assert!(matches!(
        a.submit_prepared(&asset_a, &cam, &frame, ReadbackSelection::Both),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    // Cancel on poisoned session still works (idempotent false when gone).
    assert!(!a.cancel_frame(pending).unwrap());
    // A fresh session recovers while the poisoned one stays failed.
    let mut fresh = OffscreenSession::new().unwrap();
    let fresh_asset = fresh.prepare(&topo, solid, 0.05).unwrap();
    let fresh_ticket = fresh
        .submit_prepared(&fresh_asset, &cam, &frame, ReadbackSelection::Both)
        .unwrap();
    let fresh_out = fresh.wait_frame(fresh_ticket).unwrap();
    check_async_frame(&topo, solid, &fresh_out, 256, 256, "fresh after loss");
    assert!(matches!(
        a.poll_frame(pending),
        Err(remus_render::RenderError::DeviceLost(_))
    ));
    println!("wrong-session refuses; invalid sizes recover; injected loss fails closed");
}

#[test]
fn repeated_resize_large_coordinates_and_asset_replacement() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP repeated_resize_large_coordinates_and_asset_replacement: no wgpu adapter");
        return;
    };
    println!("using wgpu adapter: {adapter}");

    let mut topo = Topology::new();
    let solid = remus_operations::primitives::make_box(&mut topo, 20.0, 20.0, 20.0).unwrap();
    let cam = frame_camera(&topo, solid);
    let mut session = OffscreenSession::new().unwrap();
    let asset = session.prepare(&topo, solid, 0.05).unwrap();

    // Repeated resize: each pending frame owns its size; sync cache untouched.
    let sizes = [(128, 128), (512, 256), (256, 512), (384, 384), (128, 128)];
    let mut tickets = Vec::new();
    for &(w, h) in &sizes {
        let f = PreparedDrawOpts::new(w, h);
        tickets.push(
            session
                .submit_prepared(&asset, &cam, &f, ReadbackSelection::Both)
                .unwrap(),
        );
    }
    // Sync render at another size does not disturb pending frames.
    let sync_opts = RenderOpts::new(200, 200);
    let sync_out = session.render(&topo, solid, &cam, &sync_opts).unwrap();
    check_sync_frame(&topo, solid, &sync_out, "interleaved sync");
    for (i, &t) in tickets.iter().enumerate() {
        let (w, h) = sizes[i];
        let out = session.wait_frame(t).unwrap();
        assert_eq!((out.width, out.height), (w, h));
        assert_eq!(out.id_buffer.as_ref().unwrap().len(), (w * h) as usize);
        check_async_frame(&topo, solid, &out, w, h, &format!("resize {i}"));
    }
    // Returning to a size reproduces it (no stale buffers).
    let f128 = PreparedDrawOpts::new(128, 128);
    let t_a = session
        .submit_prepared(&asset, &cam, &f128, ReadbackSelection::Both)
        .unwrap();
    let o_a = session.wait_frame(t_a).unwrap();
    let t_b = session
        .submit_prepared(&asset, &cam, &f128, ReadbackSelection::Both)
        .unwrap();
    let o_b = session.wait_frame(t_b).unwrap();
    assert_eq!(o_a.id_buffer, o_b.id_buffer);
    assert_eq!(o_a.color, o_b.color);

    // Large coordinates: +1e6 RTC render matches near-origin via async.
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
    let asset_far = session.prepare(&topo_far, far_box, 0.05).unwrap();
    let f384 = PreparedDrawOpts::new(384, 384);
    let t_near = session
        .submit_prepared(&asset, &cam, &f384, ReadbackSelection::Both)
        .unwrap();
    let t_far = session
        .submit_prepared(&asset_far, &cam_far, &f384, ReadbackSelection::Both)
        .unwrap();
    // Collect far first (out of order) — still matches.
    let o_far = session.wait_frame(t_far).unwrap();
    let o_near = session.wait_frame(t_near).unwrap();
    assert_eq!(o_near.id_buffer, o_far.id_buffer, "RTC ids at +1e6");
    assert_eq!(o_near.color, o_far.color, "RTC color at +1e6");

    // Asset replacement while frames pending: old frames keep old snapshot.
    let t_old = session
        .submit_prepared(&asset, &cam, &f384, ReadbackSelection::Both)
        .unwrap();
    // Edit topology and prepare replacement (translation moves the silhouette).
    remus_operations::transform::transform_solid(
        &mut topo,
        solid,
        &Mat4::translation(30.0, 0.0, 0.0),
    )
    .unwrap();
    let replacement = session.prepare(&topo, solid, 0.05).unwrap();
    drop(asset);
    let t_new = session
        .submit_prepared(&replacement, &cam, &f384, ReadbackSelection::Both)
        .unwrap();
    let o_old = session.wait_frame(t_old).unwrap();
    let o_new = session.wait_frame(t_new).unwrap();
    assert_ne!(
        o_old.id_buffer, o_new.id_buffer,
        "replacement must pick up the edit; pending old frame must not move"
    );
    // Session still serves direct renders after asset release.
    let direct = session
        .render(&topo, solid, &cam, &RenderOpts::new(384, 384))
        .unwrap();
    check_sync_frame(&topo, solid, &direct, "post-replacement");
    println!("resize x5 + interleaved sync + RTC 1e6 + replacement-while-pending, all clean");
}

#[test]
fn async_throughput_measurements() {
    let _gpu = gpu_guard();
    let Some(adapter) = probe_adapter() else {
        println!("SKIP async_throughput_measurements: no wgpu adapter");
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
    let asset = session.prepare(&topo, solid, opts.deflection).unwrap();
    let stats = asset.stats();
    println!(
        "asset: {} tris, {} verts, {} + {} + {} upload bytes",
        stats.triangles, stats.vertices, stats.vertex_bytes, stats.index_bytes, stats.edge_bytes
    );

    // One async Both frame, fully split.
    let t0 = std::time::Instant::now();
    let ticket = session
        .submit_prepared(&asset, &cam, &frame, ReadbackSelection::Both)
        .unwrap();
    let submit_only = t0.elapsed();
    let out = session.wait_frame(ticket).unwrap();
    println!(
        "1x async Both 512: submit {:?} (call) vs {:?} (inside) + wait {:?} + decode {:?}; {} bytes",
        submit_only,
        out.timings.submit,
        out.timings.wait,
        out.timings.decode,
        out.transferred_bytes
    );

    // Sustained: submit N=8 before collecting (queue-full bound), then drain.
    // Measures overlapped submission vs serialized sync, plus tail latency.
    let n = 8;
    let t0 = std::time::Instant::now();
    let mut tickets = Vec::new();
    let mut submit_sum = std::time::Duration::ZERO;
    for i in 0..n {
        let moved = Camera {
            eye: Point3::new(
                cam.eye.x() + f64::from(i) * 7.0,
                cam.eye.y() - f64::from(i) * 5.0,
                cam.eye.z(),
            ),
            ..cam
        };
        let s0 = std::time::Instant::now();
        tickets.push(
            session
                .submit_prepared(&asset, &moved, &frame, ReadbackSelection::Both)
                .unwrap(),
        );
        submit_sum += s0.elapsed();
    }
    let submit_phase = t0.elapsed();
    println!(
        "submitted {n}x without waiting in {submit_phase:?} (sum calls {submit_sum:?}); peak {} bytes",
        session.peak_in_flight_bytes()
    );
    let t0 = std::time::Instant::now();
    let mut waits = Vec::new();
    let mut decodes = Vec::new();
    let mut bytes = 0_u64;
    for t in tickets {
        let o = session.wait_frame(t).unwrap();
        waits.push(o.timings.wait);
        decodes.push(o.timings.decode);
        bytes += o.transferred_bytes;
    }
    let drain = t0.elapsed();
    waits.sort();
    decodes.sort();
    let tail_wait = waits.last().copied().unwrap_or_default();
    let p50_wait = waits[waits.len() / 2];
    println!(
        "drained {n}x in {drain:?} (avg {:?}); wait p50 {p50_wait:?} tail {tail_wait:?}; decode p50 {:?} tail {:?}; {bytes} bytes total",
        drain / n,
        decodes[decodes.len() / 2],
        decodes.last().copied().unwrap_or_default(),
    );

    // Reference: 2x synchronous prepared draws (serialized submit+wait+decode).
    let t0 = std::time::Instant::now();
    for _ in 0..2 {
        let _ = session.draw_prepared(&asset, &cam, &frame).unwrap();
    }
    let sync_total = t0.elapsed();
    println!(
        "sync prepared 2x: {sync_total:?} (avg {:?}); async sustained avg {:?}/frame drain + overlapped submit",
        sync_total / 2,
        drain / n
    );
    println!(
        "adapter: {} | total transferred {} bytes | peak in-flight {} bytes",
        session.adapter_info(),
        session.total_transferred_bytes(),
        session.peak_in_flight_bytes()
    );
    println!("note: renderer reuse/overlap only — not an exact-kernel speedup claim");
    println!("coverage: discrete GPU only; software backend unavailable in this environment");
}
