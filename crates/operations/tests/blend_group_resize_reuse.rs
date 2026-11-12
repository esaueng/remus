//! The boss-diameter edit through a blend group, as an interactive CAD
//! session makes it: the journaled entry (`resize_cylindrical_face_journaled`)
//! on the filleted Gentle-Gecko part, whose boss wall carries concave seam
//! bands, a shoulder and sampled corner patches, so every resize takes the
//! group route (remove the group to sharp, resize the sharp wall, re-fillet).
//!
//! - **Unchanged results.** Each recognition reads its sampled carriers once
//!   and removal trusts a cluster its caller has just recognized on the same
//!   arena; neither may change a result. The chained diameters 12 -> 14.6 ->
//!   17 -> 21.1 -> 14.7 are pinned to the results the route produced before
//!   (entity counts, face census, volume to 1e-9 relative).
//! - **Work.** The route recognizes the group once on its private copy and
//!   once on the caller's solid (identity carry), never again inside the
//!   removal.
//! - **Measured carrier reads.** Many queries on one
//!   [`MeasuredCarrier`] answer bit for bit what one fresh read per query
//!   answers.
//! - **Display-only frames.** With `set_display_only_blends` on, the same
//!   resizes build the same bodies (counts, census, volume) and skip the
//!   group's mesh and volume postconditions; the bodies still pass strict
//!   validation when checked afterwards.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_blend::cap_reblend::{
    MeasuredCarrier, sampled_canal_measured, sampled_project_measured,
    sampled_section_centre_measured,
};
use remus_math::compact_sort::CompactSort;
use remus_math::vec::Vec3;
use remus_operations::blend_ops::{display_only_blends, set_display_only_blends};
use remus_operations::journal_ops::resize_cylindrical_face_journaled;
use remus_operations::measure::solid_volume;
use remus_operations::performance;
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn gecko() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solid = remus_io::step::reader::read_step(
        include_str!("data/gentle_gecko_filleted.step"),
        &mut topo,
    )
    .unwrap()[0];
    (topo, solid)
}

/// The boss wall: the cylinder on the Z axis.
fn boss_of(topo: &Topology, solid: SolidId) -> FaceId {
    let walls: Vec<FaceId> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&face| {
            matches!(topo.face(face).unwrap().surface(), FaceSurface::Cylinder(c)
                if c.axis().cross(Vec3::new(0.0, 0.0, 1.0)).length() < 1e-9
                    && c.origin().x().abs() < 1e-9
                    && c.origin().y().abs() < 1e-9)
        })
        .collect();
    assert_eq!(walls.len(), 1, "one boss wall");
    walls[0]
}

/// Entity counts and the sorted surface types of `solid`.
fn census(topo: &Topology, solid: SolidId) -> (String, Vec<&'static str>) {
    let counts = format!("{:?}", solid_entity_counts(topo, solid).unwrap());
    let mut types: Vec<&'static str> = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|face| topo.face(face).unwrap().surface().type_tag())
        .collect();
    types.compact_sort_unstable();
    (counts, types)
}

/// The boss radii of the chained edit: diameters 14.6, 17, 21.1, 14.7.
const RADII: [f64; 4] = [7.3, 8.5, 10.55, 7.35];

/// What the journaled route produced for each step of [`RADII`] before
/// the recognition reuse: entity counts `(F, E, V)`, face census and
/// volume.
const PINNED: [(&str, &str, f64); 4] = [
    ("(15, 37, 24)", CENSUS, 18_001.190_268_393_137),
    ("(15, 37, 24)", CENSUS, 19_808.976_432_619_187),
    ("(15, 37, 24)", CENSUS, 23_523.128_256_833_133),
    ("(15, 37, 24)", CENSUS, 18_071.117_682_090_964),
];

/// The face census every step keeps.
const CENSUS: &str = "cylinder,cylinder,cylinder,cylinder,cylinder,nurbs,nurbs,plane,plane,plane,plane,plane,plane,plane,torus";

/// Recognitions of the group one journaled resize runs: on the private copy,
/// and on the caller's solid for the identity carry. (Before the reuse the
/// removal recognized the copy a second time: 7 in all, counting the fillet
/// cascade's own cheap probes.)
const RECOGNITIONS_PER_RESIZE: usize = 5;

const VOLUME_DEFLECTION: f64 = 0.01;

/// One step of the chain: the result and its census and volume.
fn step(topo: &mut Topology, solid: SolidId, radius: f64) -> (SolidId, String, String, f64) {
    let boss = boss_of(topo, solid);
    let result = resize_cylindrical_face_journaled(topo, solid, boss, radius)
        .unwrap_or_else(|error| panic!("resize to radius {radius}: {error}"));
    let (counts, types) = census(topo, result.solid);
    let volume = solid_volume(topo, result.solid, VOLUME_DEFLECTION).unwrap();
    (result.solid, counts, types.join(","), volume)
}

fn assert_volume(actual: f64, expected: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= 1e-9 * expected.abs(),
        "{what}: volume {actual}, expected {expected}"
    );
}

/// Resets the display-only switch however a test ends.
struct DisplayOnly;

impl DisplayOnly {
    fn on() -> Self {
        set_display_only_blends(true);
        Self
    }
}

impl Drop for DisplayOnly {
    fn drop(&mut self) {
        set_display_only_blends(false);
    }
}

#[test]
fn journaled_boss_resizes_match_the_pinned_results() {
    let (mut topo, mut solid) = gecko();
    for (index, &radius) in RADII.iter().enumerate() {
        let (result, counts, types, volume) = step(&mut topo, solid, radius);
        let (pinned_counts, pinned_types, pinned_volume) = PINNED[index];
        assert_eq!(counts, pinned_counts, "step {index} counts");
        assert_eq!(types, pinned_types, "step {index} census");
        assert_volume(volume, pinned_volume, &format!("step {index}"));
        solid = result;
    }
}

#[test]
fn a_group_resize_does_not_recognize_the_group_again_to_remove_it() {
    fn clock() -> f64 {
        0.0
    }
    let (mut topo, solid) = gecko();
    let boss = boss_of(&topo, solid);
    performance::configure(Some(clock));
    let result = resize_cylindrical_face_journaled(&mut topo, solid, boss, RADII[0]);
    let recognitions = performance::drain()
        .iter()
        .filter(|phase| phase.name == "blend_group.recognize")
        .count();
    performance::configure(None);
    result.unwrap();
    assert_eq!(recognitions, RECOGNITIONS_PER_RESIZE);
}

#[test]
fn measured_carrier_reads_answer_as_fresh_reads_do() {
    let (topo, solid) = gecko();
    let mut read = 0;
    for face in solid_faces(&topo, solid).unwrap() {
        let data = topo.face(face).unwrap();
        let FaceSurface::Nurbs(surface) = data.surface() else {
            continue;
        };
        let Ok(carrier) = MeasuredCarrier::read(data) else {
            assert!(sampled_canal_measured(data).is_err());
            continue;
        };
        read += 1;
        assert_eq!(
            format!("{:?}", carrier.canal()),
            format!("{:?}", sampled_canal_measured(data))
        );
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        // Points on the carrier and either side of it, queried in an order
        // that revisits the carrier's span ends.
        for i in 0..=4_u32 {
            for j in (0..=16_u32).rev() {
                let u = (u1 - u0).mul_add(f64::from(i) / 4.0, u0);
                let v = (v1 - v0).mul_add(f64::from(j) / 16.0, v0);
                let on = surface.evaluate(u, v);
                let normal = surface.normal(u, v).unwrap_or(Vec3::new(0.0, 0.0, 1.0));
                for offset in [0.0, 0.3, -0.2] {
                    let point = on + normal * offset;
                    let cached = carrier.project(point);
                    let fresh = sampled_project_measured(data, point).unwrap();
                    assert_eq!(
                        (cached.0.to_bits(), cached.1.to_bits()),
                        (fresh.0.to_bits(), fresh.1.to_bits()),
                        "projection of {point:?} on face {}",
                        face.index()
                    );
                    assert_eq!(
                        format!("{:?}", carrier.section_centre(point)),
                        format!("{:?}", sampled_section_centre_measured(data, point)),
                        "section centre of {point:?} on face {}",
                        face.index()
                    );
                }
            }
        }
    }
    assert!(read > 0, "the part has sampled carriers to read");
}

#[test]
fn display_only_resizes_build_the_checked_bodies() {
    let (topo, solid) = gecko();
    let mut checked_topo = topo.clone();
    let mut display_topo = topo;
    let (mut checked, mut display) = (solid, solid);
    for (index, &radius) in RADII.iter().enumerate() {
        let (result, counts, types, volume) = step(&mut checked_topo, checked, radius);
        checked = result;
        let shown = {
            let _display = DisplayOnly::on();
            assert!(display_only_blends());
            step(&mut display_topo, display, radius)
        };
        assert!(!display_only_blends());
        display = shown.0;
        assert_eq!(shown.1, counts, "step {index} counts");
        assert_eq!(shown.2, types, "step {index} census");
        assert_volume(shown.3, volume, &format!("display-only step {index}"));
        let report = validate_solid(&display_topo, display).unwrap();
        assert!(report.is_valid(), "display-only step {index}: {report:?}");
    }
}
