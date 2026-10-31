//! Warm-cache workloads (RFC 0009 stage 0): the same edit sequences the
//! `wasm_warm_*` scenarios of `scripts/performance/wasm.cjs` drive through the
//! kernel package, run natively through the library with the kernel's cache
//! switches on.
//!
//! Every sample starts from emptied caches and records, per cache, the hit,
//! miss and eviction counts its timed phases added. Those counts are a pure
//! function of the call sequence, so they must be identical across samples,
//! processes and builds that keep the same work (`cache_stats.py`).

use std::time::Instant;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::feature_recognition::recognize_features;
use remus_operations::measure::{face_area, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::push_pull::move_faces;
use remus_operations::query::solid_edge_relations;
use remus_operations::tessellate::tessellate_solid_grouped_with_tolerance;
use remus_operations::transform::transform_solid;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_wasm::kernel::RemusKernel;
use serde_json::{Map, Value, json};

/// Display deflection of the consumer's mesh refresh (PERF-D01).
const DISPLAY_DEFLECTION: f64 = 0.0148;
/// Measurement deflection of the PERF-V02 reads.
const MEASURE_DEFLECTION: f64 = 0.08;
/// Area of the holder's +X cap face that the edit moves (PERF-V02).
const MOVED_FACE_AREA: f64 = 1045.93;
const MOVE_DISTANCE: f64 = -6.0;
const RETICK_DISTANCE: f64 = -5.0;

/// Exact arena document of the hammer holder, the input of every holder case
/// in both workers, so both start from the same handles.
pub fn hammer_arena() -> Vec<u8> {
    let mut topo = Topology::new();
    let solids = remus_io::step::reader::read_step(
        include_str!("../../../io/tests/data/shapr3d_hammer_holder.step"),
        &mut topo,
    )
    .expect("hammer-holder import");
    assert_eq!(solids.len(), 1);
    remus_io::arena_io::serialize_solids(&topo, &solids).expect("arena document")
}

fn load(arena: &[u8]) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = remus_io::arena_io::deserialize_solids(arena, &mut topo).expect("arena");
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

/// Turn the kernel's module-wide caches on (the constructor's switches) and
/// empty them, so a sample never reuses an earlier sample's entries.
fn fresh_caches() {
    drop(RemusKernel::new());
    remus_algo::pcurve_memo::set_thread_pcurve_memo_capacity(0);
    remus_algo::wire_area_memo::set_thread_wire_area_memo_capacity(0);
    remus_algo::ray_geom_memo::set_thread_ray_geom_memo_capacity(0);
    remus_check::properties::face_cache::clear_thread_face_cache();
    remus_operations::measure::clear_thread_volume_memo();
    remus_operations::measure::clear_measurement_reuse();
    remus_operations::measure::clear_thread_nurbs_area_memo();
    remus_operations::measure::clear_thread_face_box_memo();
    remus_operations::tessellate::clear_face_mesh_cache();
    remus_operations::tessellate::clear_mesh_reuse();
    remus_operations::query::disable_thread_crossing_memo();
    drop(RemusKernel::new());
    let held = retained();
    assert!(held.values().all(|n| *n == 0), "caches not empty: {held:?}");
}

/// Entries each cache holds.
fn retained() -> Map<String, Value> {
    let mut out = Map::new();
    let mut put = |name: &str, n: usize| {
        out.insert(name.into(), json!(n));
    };
    put(
        "pcurve",
        remus_algo::pcurve_memo::thread_pcurve_memo_stats().len,
    );
    put(
        "wire_area",
        remus_algo::wire_area_memo::thread_wire_area_memo_stats().len,
    );
    put(
        "ray_geom",
        remus_algo::ray_geom_memo::thread_ray_geom_memo_stats().len,
    );
    put(
        "face_integrals",
        remus_check::properties::face_cache::thread_face_cache_stats().len,
    );
    put(
        "volume",
        remus_operations::measure::thread_volume_memo_stats().len,
    );
    put(
        "measurement_reuse",
        remus_operations::measure::measurement_reuse_stats().area_entries,
    );
    put(
        "nurbs_area",
        remus_operations::measure::thread_nurbs_area_memo_stats().map_or(0, |s| s.len),
    );
    put(
        "face_box",
        remus_operations::measure::thread_face_box_memo_stats().map_or(0, |s| s.len),
    );
    put(
        "face_mesh",
        remus_operations::tessellate::face_mesh_cache_stats().map_or(0, |s| s.entries),
    );
    put(
        "mesh_stage",
        remus_operations::tessellate::mesh_reuse_stats().map_or(0, |s| s.entries),
    );
    put(
        "crossing",
        remus_operations::query::thread_crossing_memo_stats().map_or(0, |s| s.len),
    );
    out
}

/// Cumulative per-cache counters; [`delta`] turns two snapshots into the work
/// a phase sequence did.
fn counters() -> Value {
    let pcurve = remus_algo::pcurve_memo::thread_pcurve_memo_stats();
    let wire = remus_algo::wire_area_memo::thread_wire_area_memo_stats();
    let ray = remus_algo::ray_geom_memo::thread_ray_geom_memo_stats();
    let face = remus_check::properties::face_cache::thread_face_cache_stats();
    let volume = remus_operations::measure::thread_volume_memo_stats();
    let reuse = remus_operations::measure::measurement_reuse_stats();
    let nurbs = remus_operations::measure::thread_nurbs_area_memo_stats().unwrap_or_default();
    let boxes = remus_operations::measure::thread_face_box_memo_stats().unwrap_or_default();
    let mesh = remus_operations::tessellate::face_mesh_cache_stats().unwrap_or_default();
    let stage = remus_operations::tessellate::mesh_reuse_stats().unwrap_or_default();
    let crossing = remus_operations::query::thread_crossing_memo_stats()
        .map_or((0, 0), |s| (s.hits, s.misses));
    json!({
        "pcurve": {"hits": pcurve.hits, "misses": pcurve.misses},
        "wire_area": {"hits": wire.hits, "misses": wire.misses},
        "ray_geom": {"hits": ray.hits, "misses": ray.misses},
        "face_integrals": {"hits": face.hits, "rereferenced": face.rereferenced,
            "translated": face.translated, "misses": face.misses, "evictions": face.evictions},
        "volume": {"hits": volume.hits, "content_hits": volume.content_hits, "misses": volume.misses},
        "measurement_reuse": {"area_identity_hits": reuse.area_identity_hits,
            "area_content_hits": reuse.area_content_hits, "area_misses": reuse.area_misses,
            "area_evictions": reuse.area_evictions, "volume_identity_hits": reuse.volume_identity_hits,
            "volume_content_hits": reuse.volume_content_hits},
        "nurbs_area": {"hits": nurbs.hits, "misses": nurbs.misses, "evictions": nurbs.evictions},
        "face_box": {"hits": boxes.hits, "misses": boxes.misses, "evictions": boxes.evictions},
        "face_mesh": {"exact_hits": mesh.exact_hits, "translated_hits": mesh.translated_hits,
            "misses": mesh.misses, "translation_refused": mesh.translation_refused,
            "replay_conflicts": mesh.replay_conflicts, "uncacheable": mesh.uncacheable,
            "evictions": mesh.evictions},
        "mesh_stage": {"boundary_hits": stage.boundary_hits, "edge_hits": stage.edge_hits,
            "planar_hits": stage.planar_hits, "weld_hits": stage.weld_hits,
            "cleanup_hits": stage.cleanup_hits, "evictions": stage.evictions},
        "crossing": {"hits": crossing.0, "misses": crossing.1},
    })
}

fn delta(before: &Value, after: &Value) -> Value {
    match (before, after) {
        (Value::Object(b), Value::Object(a)) => Value::Object(
            a.iter()
                .map(|(k, v)| (k.clone(), delta(&b[k], v)))
                .collect(),
        ),
        _ => json!(after.as_u64().unwrap() - before.as_u64().unwrap()),
    }
}

/// Times named phases and keeps their order.
struct Phases(Vec<(&'static str, u128)>);

impl Phases {
    fn time<T>(&mut self, name: &'static str, f: impl FnOnce() -> T) -> T {
        let start = Instant::now();
        let out = f();
        self.0.push((name, start.elapsed().as_nanos()));
        out
    }

    fn finish(self, start: &Value, mut metrics: Value) -> (u128, Value) {
        let total = self.0.iter().map(|(_, ns)| ns).sum();
        let phases: Map<String, Value> = self
            .0
            .iter()
            .map(|(name, ns)| ((*name).to_string(), json!(*ns as f64 / 1e6)))
            .collect();
        metrics["phases_ms"] = Value::Object(phases);
        metrics["cache_stats"] = delta(start, &counters());
        metrics["cache_entries"] = Value::Object(retained());
        (total, metrics)
    }
}

fn near_rel(actual: f64, expected: f64, rel: f64) {
    assert!(
        actual.is_finite() && (actual - expected).abs() <= rel * expected.abs(),
        "{actual} != {expected} (rel {rel})"
    );
}

fn valid(topo: &Topology, solid: SolidId) {
    let report = validate_solid(topo, solid).expect("strict validation");
    assert!(report.is_valid(), "{report:?}");
}

fn areas(topo: &Topology, faces: &[FaceId]) -> f64 {
    faces
        .iter()
        .map(|&f| face_area(topo, f, MEASURE_DEFLECTION).expect("face area"))
        .sum()
}

/// The planar face facing +X whose area is closest to [`MOVED_FACE_AREA`],
/// chosen as the WASM worker chooses it.
fn moved_face(topo: &Topology, solid: SolidId) -> FaceId {
    let mut best = None;
    for face in solid_faces(topo, solid).unwrap() {
        let data = topo.face(face).unwrap();
        let FaceSurface::Plane { normal, .. } = data.surface() else {
            continue;
        };
        let nx = if data.is_reversed() {
            -normal.x()
        } else {
            normal.x()
        };
        if nx <= 0.99 {
            continue;
        }
        let gap = (face_area(topo, face, 0.05).unwrap() - MOVED_FACE_AREA).abs();
        if best.is_none_or(|(g, _)| gap < g) {
            best = Some((gap, face));
        }
    }
    let (gap, face) = best.expect("+X planar face");
    assert!(gap < 1.0, "+X face area off by {gap}");
    face
}

fn mesh_triangles(topo: &Topology, solid: SolidId) -> usize {
    let (mesh, offsets) = tessellate_solid_grouped_with_tolerance(
        topo,
        solid,
        DISPLAY_DEFLECTION,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .expect("display mesh");
    assert!(!offsets.is_empty());
    mesh.indices.len() / 3
}

/// Strict validation, face areas and volume, each cold then repeated, plus
/// validation of a deserialized copy in a second document (the probe kernel).
pub fn measure(arena: &[u8]) -> (u128, Value) {
    let (topo, solid) = load(arena);
    let faces = solid_faces(&topo, solid).unwrap();
    fresh_caches();
    let start = counters();
    let mut p = Phases(Vec::new());
    p.time("validate_cold", || valid(&topo, solid));
    p.time("validate_warm", || valid(&topo, solid));
    p.time("validate_copy", || {
        let (copy, copied) = load(arena);
        valid(&copy, copied);
    });
    let cold = p.time("areas_cold", || areas(&topo, &faces));
    let warm = p.time("areas_warm", || areas(&topo, &faces));
    let v_cold = p.time("volume_cold", || {
        solid_volume(&topo, solid, MEASURE_DEFLECTION).unwrap()
    });
    let v_warm = p.time("volume_warm", || {
        solid_volume(&topo, solid, MEASURE_DEFLECTION).unwrap()
    });
    assert_eq!(cold.to_bits(), warm.to_bits(), "warm face areas differ");
    assert_eq!(v_cold.to_bits(), v_warm.to_bits(), "warm volume differs");
    near_rel(v_cold, 50_245.22, 1e-6);
    p.finish(
        &start,
        json!({"faces": faces.len(), "area_sum": cold, "volume": v_cold}),
    )
}

/// The consumer's edit loop: display mesh cold and warm, edge relations and
/// feature recognition, a move of the +X face and a second drag tick of the same source face, then
/// the first moved body's mesh, relations, recognition and strict validation.
pub fn edit(arena: &[u8]) -> (u128, Value) {
    let (mut topo, solid) = load(arena);
    let face = moved_face(&topo, solid);
    fresh_caches();
    let start = counters();
    let mut p = Phases(Vec::new());
    let tris = p.time("mesh_cold", || mesh_triangles(&topo, solid));
    let tris_warm = p.time("mesh_warm", || mesh_triangles(&topo, solid));
    assert_eq!(tris, tris_warm);
    let relations = p.time("relations_source", || {
        solid_edge_relations(&topo, solid, None).unwrap().len()
    });
    let source_features = p.time("recognize_source", || {
        recognize_features(&topo, solid, MEASURE_DEFLECTION)
            .unwrap()
            .len()
    });
    let moved = p.time("move", || {
        move_faces(&mut topo, solid, &[face], MOVE_DISTANCE).expect("move +X face")
    });
    // A later drag tick moves the same source face to another distance.
    let retick = p.time("move_retick", || {
        move_faces(&mut topo, solid, &[face], RETICK_DISTANCE).expect("move +X face again")
    });
    let moved_tris = p.time("mesh_moved", || mesh_triangles(&topo, moved));
    let moved_relations = p.time("relations_moved", || {
        solid_edge_relations(&topo, moved, None).unwrap().len()
    });
    let features = p.time("recognize_moved", || {
        recognize_features(&topo, moved, MEASURE_DEFLECTION)
            .unwrap()
            .len()
    });
    p.time("validate_moved", || valid(&topo, moved));
    let volume = solid_volume(&topo, moved, MEASURE_DEFLECTION).unwrap();
    let retick_volume = solid_volume(&topo, retick, MEASURE_DEFLECTION).unwrap();
    assert!(
        volume < retick_volume && retick_volume < 50_245.22,
        "moving +X inward must remove material"
    );
    p.finish(
        &start,
        json!({"triangles": tris, "moved_triangles": moved_tris, "relations": relations, "features": source_features,
            "moved_relations": moved_relations, "moved_features": features,
            "moved_volume": volume,
            "retick_volume": retick_volume}),
    )
}

fn placed(topo: &mut Topology, solid: SolidId, at: [f64; 3]) -> SolidId {
    transform_solid(topo, solid, &Mat4::translation(at[0], at[1], at[2])).unwrap();
    solid
}

/// Booleans against the holder's blended rims: a slot, a hole, then the same
/// slot again on the original body.
pub fn cuts(arena: &[u8]) -> (u128, Value) {
    let (mut topo, solid) = load(arena);
    fresh_caches();
    let start = counters();
    let mut p = Phases(Vec::new());
    let mut cut = |topo: &mut Topology, name, tool: fn(&mut Topology) -> SolidId| {
        p.time(name, || {
            let tool = tool(topo);
            boolean(topo, BooleanOp::Cut, solid, tool).expect("holder cut")
        })
    };
    let slot = |t: &mut Topology| {
        let b = make_box(t, 4.0, 12.0, 20.0).unwrap();
        placed(t, b, [10.0, 44.0, 0.0])
    };
    let hole = |t: &mut Topology| {
        let c = make_cylinder(t, 2.0, 30.0).unwrap();
        placed(t, c, [20.0, 50.0, -5.0])
    };
    let a = cut(&mut topo, "cut_slot", slot);
    let b = cut(&mut topo, "cut_hole", hole);
    let c = cut(&mut topo, "cut_slot_repeat", slot);
    let volumes: Vec<f64> = [a, b, c]
        .iter()
        .map(|&s| solid_volume(&topo, s, MEASURE_DEFLECTION).unwrap())
        .collect();
    assert_eq!(
        volumes[0].to_bits(),
        volumes[2].to_bits(),
        "repeat cut differs"
    );
    assert!(volumes.iter().all(|&v| v < 50_245.22));
    p.finish(&start, json!({"volumes": volumes}))
}

/// Plate geometry shared with the WASM worker.
pub const PLATE: [f64; 3] = [100.0, 100.0, 5.0];
pub const HOLE_RADIUS: f64 = 1.5;
pub const SLOTS: usize = 6;

/// A plate drilled with a `grid × grid` pattern of holes one cut at a time,
/// then cut by [`SLOTS`] slots that run between hole columns and meet only
/// the two faces carrying every hole (`perf-local-boolean.md`, stage 3).
pub fn plate(grid: usize) -> (u128, Value) {
    let mut topo = Topology::new();
    fresh_caches();
    let start = counters();
    let mut p = Phases(Vec::new());
    let pitch = PLATE[0] / (grid as f64 + 1.0);
    let mut body = p.time("drill", || {
        let mut body = make_box(&mut topo, PLATE[0], PLATE[1], PLATE[2]).unwrap();
        for i in 1..=grid {
            for j in 1..=grid {
                let c = make_cylinder(&mut topo, HOLE_RADIUS, PLATE[2] + 2.0).unwrap();
                let c = placed(&mut topo, c, [i as f64 * pitch, j as f64 * pitch, -1.0]);
                body = boolean(&mut topo, BooleanOp::Cut, body, c).expect("drill");
            }
        }
        body
    });
    for k in 0..SLOTS {
        let name = if k == 0 { "slot_first" } else { "slots_rest" };
        body = p.time(name, || {
            let s = make_box(&mut topo, 1.0, 0.8 * PLATE[1], PLATE[2] + 2.0).unwrap();
            let x = (k as f64 + 1.5) * pitch - 0.5;
            let s = placed(&mut topo, s, [x, 0.1 * PLATE[1], -1.0]);
            boolean(&mut topo, BooleanOp::Cut, body, s).expect("slot")
        });
    }
    let volume = solid_volume(&topo, body, MEASURE_DEFLECTION).unwrap();
    let expected = PLATE[0] * PLATE[1] * PLATE[2]
        - (grid * grid) as f64 * std::f64::consts::PI * HOLE_RADIUS * HOLE_RADIUS * PLATE[2]
        - SLOTS as f64 * 0.8 * PLATE[1] * PLATE[2];
    near_rel(volume, expected, 1e-9);
    let mut timed = p;
    // `slots_rest` was pushed once per slot; merge so the phase is their sum.
    let rest: u128 = timed
        .0
        .iter()
        .filter(|(n, _)| *n == "slots_rest")
        .map(|(_, ns)| ns)
        .sum();
    timed.0.retain(|(n, _)| *n != "slots_rest");
    timed.0.push(("slots_rest", rest));
    timed.finish(
        &start,
        json!({"holes": grid * grid, "slots": SLOTS, "volume": volume}),
    )
}
