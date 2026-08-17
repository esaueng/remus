//! A full-revolution quadric band whose rim was split by a boolean.
//!
//! `tessellate_solid` shares one polyline per edge so neighbouring faces meet on
//! identical vertices. For a CLOSED circle edge it sampled that polyline from the
//! curve's own parameter origin, `t = 0`, which is wherever the underlying
//! `Circle3D` happens to start — not where the edge's own vertex is. On a rim
//! that closes on a seam vertex the two differ by an arbitrary angle, and the
//! CDT boundary walk that consumes the polyline then jumps by that angle when it
//! crosses from the seam line onto the rim. On a periodic surface the jump
//! unwraps into an extra turn: the walk reported a `u` span of 2.5 turns for a
//! band that is one turn around, the CDT tiled the sheared domain, and the
//! triangles folded back over the cylinder.
//!
//! The body is the one OpenZCAD reports on: a 40 x 24 x 10 plate fused with an
//! r6 h20 boss seated so that part of it overhangs the plate's `x = 0` wall. The
//! fuse splits the boss wall into a tab (below the plate top, over the arc that
//! is outside the wall) and a full ring above it — and the ring's lower rim is
//! three arcs meeting the seam, so it takes the CDT path rather than the
//! structured two-rim band.
//!
//! The mesh stayed closed and 2-manifold throughout, which is why nothing
//! caught it: the folded band is a watertight surface that simply encloses
//! 34 mm3 it should not. `mass_properties` (exact per-face integrals, no mesh)
//! read the body correctly to 6e-15 the whole time, so the two routes disagreed
//! by 0.30 % and only the meshed one was wrong.
//!
//! A SEPARATE defect sits next door and shapes what can be asserted where. At
//! `d >= R/2` — the axis at or past half the radius — the fuse stops splitting
//! the boss wall against the plate and hands back one undivided cylinder face
//! of area exactly `2 pi R H`, buried portion included. The body then carries
//! `(2 pi - 2 acos(d/R)) R * PLATE_Z` of interior sheet, and no mesh of it can
//! be watertight. `validate_solid` calls that body valid and both volume routes
//! return the exact closed form on it, so mesh watertightness is its only
//! witness. It is pinned below, and it is not this fix's.
//!
//! The seating OpenZCAD reports on stands exactly ON that cliff, at `0.5 R`,
//! where which side the kernel lands on is decided by rounding: on x86_64 the
//! body splits at unit scale and does not at 1000x, and CI's arm64 runner does
//! not split it at unit scale either. So `the_reported_body...` below asserts
//! BOTH sides in closed form, and a reader should know that on whichever
//! platform lands on the un-split side, that case does not exercise this fix at
//! all — there is no split rim on that body. The `<= 0.4 R` sweep and the
//! `tessellate::tests` unit test are what pin this fix everywhere.
//!
//! Everything asserted here is written out from the construction's own
//! dimensions. Nothing is a recorded measurement, and nothing compares one
//! kernel route against another.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{PI, TAU};

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{TriangleMesh, tessellate_solid};
use remus_operations::transform::transform_solid;
use remus_operations::validate;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// Plate and boss in units of the model's own scale factor.
const PLATE_X: f64 = 40.0;
const PLATE_Y: f64 = 24.0;
const PLATE_Z: f64 = 10.0;
const R: f64 = 6.0;
const H: f64 = 20.0;

/// Scale factors, coarsest FIRST so a fix that only holds at unit scale cannot
/// hide behind being swept first. Every length below — the model, the
/// deflection — carries this factor, and every tolerance asserted is relative,
/// so each row is the same problem in different units.
const SCALES: [f64; 3] = [1000.0, 1.0, 0.001];

/// Axis offsets `d` as a FRACTION of the radius: the boss axis stands at
/// `x = d`, so `d < R` leaves `R - d` of it hanging past the `x = 0` wall.
/// Swept rather than pinned at one seating, because the split that triggers the
/// defect is the same at every overhang and a fix has to hold across the range.
///
/// Capped at `0.4 R` because of a SEPARATE defect in the fuse, pinned by
/// `the_fuse_stops_trimming_the_tool_wall_at_half_the_radius` below: at
/// `d >= R/2` the boolean stops splitting the boss wall against the plate and
/// returns it whole, so the body carries the buried part as an interior sheet
/// and no mesh of it can be watertight. The sweep stays on the side of that
/// cliff where the body this fix is about — a rim split by the boolean —
/// actually exists.
const DEPTH_FRACTIONS: [f64; 4] = [0.4, 0.25, 0.1, 0.05];

/// The seating OpenZCAD reports on: the axis at `x = 3`, half the radius, so
/// 3 mm of the boss hangs past the wall.
const REPORTED_FRACTION: f64 = 0.5;

/// Build the fused body: plate `[0,PLATE_X] x [0,PLATE_Y] x [0,PLATE_Z]` and an
/// `R` x `H` boss on a vertical axis at `(d, PLATE_Y/2)`, both scaled by `s`.
fn build(topo: &mut Topology, s: f64, d: f64) -> SolidId {
    let plate = make_box(topo, PLATE_X * s, PLATE_Y * s, PLATE_Z * s).unwrap();
    let boss = make_cylinder(topo, R * s, H * s).unwrap();
    transform_solid(
        topo,
        boss,
        &Mat4::translation(d * s, PLATE_Y * s / 2.0, 0.0),
    )
    .unwrap();
    boolean(topo, BooleanOp::Fuse, plate, boss).unwrap()
}

/// Area of the boss footprint outside the wall: the circular segment of the
/// disc (centre `x = d`, radius `R`) beyond `x = 0`.
fn segment_outside(d: f64, s: f64) -> f64 {
    let (r, d) = (R * s, d * s);
    r * r * (d / r).acos() - d * (r * r - d * d).sqrt()
}

/// `plate + whole boss - the part of the boss buried in the plate`.
fn closed_form_volume(d: f64, s: f64) -> f64 {
    let (r, h) = (R * s, H * s);
    let (px, py, pz) = (PLATE_X * s, PLATE_Y * s, PLATE_Z * s);
    px * py * pz + PI * r * r * h - (PI * r * r - segment_outside(d, s)) * pz
}

/// Surface area of the same body, face by face: the two plate faces the boss
/// interrupts, the four plate walls (one of them notched by the boss passing
/// through it), the boss's top disc, the full ring of boss wall above the plate,
/// and the tab of boss wall below it over the arc that is outside the wall.
fn closed_form_area(d: f64, s: f64) -> f64 {
    let (r, h, ds) = (R * s, H * s, d * s);
    let (px, py, pz) = (PLATE_X * s, PLATE_Y * s, PLATE_Z * s);
    let disc = PI * r * r;
    let segment = segment_outside(d, s);
    // Half-chord where the boss crosses the x = 0 plane.
    let half_chord = (r * r - ds * ds).sqrt();
    // Arc of the boss wall standing outside the wall plane.
    let exposed_arc = 2.0 * (ds / r).acos();

    let bottom = px * py + segment;
    let top = px * py - (disc - segment);
    let walls_y = 2.0 * px * pz;
    let wall_x_far = py * pz;
    let wall_x_near = py * pz - 2.0 * half_chord * pz;
    let boss_top = disc;
    let boss_ring = TAU * r * (h - pz);
    let boss_tab = exposed_arc * r * pz;

    bottom + top + walls_y + wall_x_far + wall_x_near + boss_top + boss_ring + boss_tab
}

fn mesh_area(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .chunks_exact(3)
        .map(|t| {
            let (a, b, c) = (
                mesh.positions[t[0] as usize],
                mesh.positions[t[1] as usize],
                mesh.positions[t[2] as usize],
            );
            (b - a).cross(c - a).length() * 0.5
        })
        .sum()
}

/// `(free edges, non-manifold edges)` of a triangle mesh: edges incident to one
/// triangle, and edges incident to three or more.
fn mesh_edge_defects(mesh: &TriangleMesh) -> (usize, usize) {
    use std::collections::HashMap;
    let mut counts: HashMap<(u32, u32), usize> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        for &(i, j) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if i < j { (i, j) } else { (j, i) };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    (
        counts.values().filter(|&&c| c == 1).count(),
        counts.values().filter(|&&c| c > 2).count(),
    )
}

fn assert_closed_solid(topo: &Topology, solid: SolidId, what: &str) {
    let report = validate::validate_solid(topo, solid).expect("validate");
    assert!(
        report.is_valid(),
        "{what}: not a closed 2-manifold solid: {}",
        report
            .issues
            .iter()
            .filter(|i| i.severity == validate::Severity::Error)
            .map(|i| i.description.clone())
            .collect::<Vec<_>>()
            .join("; ")
    );
}

/// The mesh of the fused body must have the body's own surface area.
///
/// This is the assertion that sees the fold. An inscribed triangle mesh can only
/// come in UNDER the exact area of a boundary that curves away from it, so any
/// excess at all is surface counted twice; the folded band ran 3.8 % over. The
/// bound above is one-sided for exactly that reason and holds at every scale.
///
/// The bound below is the chord deficit of a mesh sampled at 1e-4 of the model's
/// own extent, and it is tight only from unit scale up. Below unit scale a
/// separate, pre-existing defect on the TAB face costs another 0.75 - 1.2 % —
/// deflection-independent, the same at 0.1, 0.01 and 0.001, and present before
/// this fix too — so there the lower bound is only wide enough to catch a mesh
/// that lost a face.
#[test]
fn the_meshed_body_has_the_body_s_own_surface_area() {
    for s in SCALES {
        for f in DEPTH_FRACTIONS {
            let d = R * f;
            let mut topo = Topology::new();
            let solid = build(&mut topo, s, d);
            let what = format!("scale {s}, axis at {f} R");

            assert_closed_solid(&topo, solid, &what);

            let mesh = tessellate_solid(&topo, solid, 1e-4 * s).unwrap();
            let (free, non_manifold) = mesh_edge_defects(&mesh);
            assert_eq!(
                (free, non_manifold),
                (0, 0),
                "{what}: mesh has {free} free and {non_manifold} non-manifold edge(s)"
            );

            let want = closed_form_area(d, s);
            let got = mesh_area(&mesh);
            let rel = (got - want) / want;
            assert!(
                rel <= 1e-9,
                "{what}: mesh area {got:.6} EXCEEDS the closed form {want:.6} \
                 ({:+.4} %) — an inscribed mesh cannot, so this is doubled surface",
                rel * 100.0
            );
            let floor = if s >= 1.0 { -1e-3 } else { -2e-2 };
            assert!(
                rel >= floor,
                "{what}: mesh area {got:.6} against the closed form {want:.6} ({:+.4} %)",
                rel * 100.0
            );
        }
    }
}

/// And the volume that mesh carries is the body's volume.
///
/// The fold enclosed real extra space, so it read HIGH — the same one-sided
/// statement applies, and it is the one OpenZCAD sees: `kernel.volume` reported
/// +0.30 % on this body, converging to +0.3125 % as the deflection tightened,
/// while the exact per-face integral had it right all along.
#[test]
fn the_meshed_body_does_not_enclose_more_than_the_body() {
    for s in SCALES {
        for f in DEPTH_FRACTIONS {
            let d = R * f;
            let mut topo = Topology::new();
            let solid = build(&mut topo, s, d);
            let what = format!("scale {s}, axis at {f} R");

            let want = closed_form_volume(d, s);
            let got = solid_volume(&topo, solid, 1e-4 * s).unwrap();
            let rel = (got - want) / want;
            assert!(
                rel <= 1e-6,
                "{what}: volume {got:.6} EXCEEDS the closed form {want:.6} ({:+.5} %)",
                rel * 100.0
            );
        }
    }
}

/// At the scales where the mesh route is the one that answers, it answers
/// exactly. Held to 1e-4 relative — four orders tighter than the +0.3125 % the
/// fold converged to, and loose enough for the chord deficit at 1e-4 of extent.
///
/// Restricted to unit scale and up on purpose: below it a SEPARATE, pre-existing
/// defect takes over on the tab face, worth a steady -0.44 % at 0.1, 0.01 and
/// 0.001 alike and unmoved by deflection. It is not this fix's (the same -0.44 %
/// sits under the +0.31 % before the fix), and the two assertions above cover
/// every scale.
#[test]
fn the_meshed_volume_is_the_closed_form_at_and_above_unit_scale() {
    for s in [1000.0, 1.0] {
        for f in DEPTH_FRACTIONS {
            let d = R * f;
            let mut topo = Topology::new();
            let solid = build(&mut topo, s, d);
            let want = closed_form_volume(d, s);
            let got = solid_volume(&topo, solid, 1e-4 * s).unwrap();
            let rel = (got - want).abs() / want;
            assert!(
                rel < 1e-4,
                "scale {s}, axis at {f} R: volume {got:.6} against the closed form \
                 {want:.6} ({:.5} %)",
                rel * 100.0
            );
        }
    }
}

/// The cylindrical faces of the fused body, each with its own meshed area.
fn cylinder_face_areas(topo: &Topology, solid: SolidId, deflection: f64) -> Vec<f64> {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|fid| {
            matches!(
                topo.face(*fid).unwrap().surface(),
                remus_topology::face::FaceSurface::Cylinder { .. }
            )
        })
        .map(|fid| {
            mesh_area(&remus_operations::tessellate::tessellate(topo, fid, deflection).unwrap())
        })
        .collect()
}

/// The whole boundary's area, summed face by face rather than from the solid
/// mesh — the only way to see surface the solid mesh cannot stitch.
fn summed_face_area(topo: &Topology, solid: SolidId, deflection: f64) -> f64 {
    remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|fid| {
            mesh_area(&remus_operations::tessellate::tessellate(topo, fid, deflection).unwrap())
        })
        .sum()
}

/// Wall of the boss standing outside the plate, below its top: the tab.
fn tab_area(d: f64, s: f64) -> f64 {
    2.0 * ((d * s) / (R * s)).acos() * (R * s) * (PLATE_Z * s)
}

/// Wall of the boss above the plate: the full ring.
fn ring_area(s: f64) -> f64 {
    TAU * (R * s) * ((H - PLATE_Z) * s)
}

/// Wall of the boss buried inside the plate. Present on the boundary only when
/// the fuse fails to trim it, and interior surface when it is.
fn buried_wall_area(d: f64, s: f64) -> f64 {
    TAU * (R * s) * (PLATE_Z * s) - tab_area(d, s)
}

/// The body as the product builds it: a 40 x 24 x 10 plate and an r6 h20 boss
/// at `x = 3`. The app read 10984.864189375206 against the hand closed form
/// 10952.079901041901, +0.30 %.
///
/// `0.5 R` is exactly the cliff described at the top of this file, so the test
/// asserts both sides of it. Which side a platform lands on is not a tolerance
/// this test may set — it is decided by rounding inside the fuse — so the
/// branch is taken on an OBSERVABLE fact, the number of cylindrical faces the
/// boolean produced, and each branch then carries a full set of closed forms.
/// Neither branch can pass by accident and neither can quietly become the
/// other.
///
/// Swept over 1000x and 1x so BOTH branches are exercised on one machine: on
/// x86_64 the 1000x row lands on the un-split side and the 1x row splits.
///
/// 0.001x is left out, and only here. The split branch asserts the mesh area
/// from BELOW as well as above, and below unit scale a separate, pre-existing
/// defect on the tab face costs -0.61 % on this body — the same one worth
/// -0.44 % of volume at 0.1x, 0.01x and 0.001x alike, which is why
/// `the_meshed_volume_is_the_closed_form_at_and_above_unit_scale` stops at unit
/// scale too. The one-sided upper bound that actually detects the fold is
/// asserted at every scale by `the_meshed_body_has_the_body_s_own_surface_area`.
#[test]
fn the_reported_body_measures_its_closed_form() {
    let d = R * REPORTED_FRACTION;

    for s in [1000.0, 1.0] {
        let mut topo = Topology::new();
        let solid = build(&mut topo, s, d);
        let what = format!("the reported body at {s}x");
        assert_closed_solid(&topo, solid, &what);

        // True on BOTH sides: the exact per-face integral never saw any of
        // this. It is why the corruption below is invisible to measurement.
        let want_vol = closed_form_volume(d, s);
        let mp = remus_operations::measure::mass_properties(&topo, solid)
            .unwrap()
            .mass;
        let mp_rel = (mp - want_vol).abs() / want_vol;
        assert!(
            mp_rel < 1e-12,
            "{what}: mass_properties {mp:.6} against the closed form {want_vol:.6} ({mp_rel:e})"
        );

        let fine = 1e-6 * s;
        let cylinders = cylinder_face_areas(&topo, solid, fine);
        match cylinders.len() {
            // The fuse split the boss wall: tab below the plate top, ring
            // above. This is the body this fix is about, and the only one that
            // HAS the seam-split rim.
            2 => {
                let (want_tab, want_ring) = (tab_area(d, s), ring_area(s));
                for (got, want) in [(cylinders[0], want_tab), (cylinders[1], want_ring)] {
                    let rel = (got - want).abs() / want;
                    assert!(
                        rel < 1e-5,
                        "{what}: a boss wall face meshes {got:.6} against the closed form \
                         {want:.6} ({rel:e})"
                    );
                }

                let mesh = tessellate_solid(&topo, solid, 1e-4 * s).unwrap();
                assert_eq!(
                    mesh_edge_defects(&mesh),
                    (0, 0),
                    "{what}: mesh is not a closed 2-manifold"
                );

                let want_area = closed_form_area(d, s);
                let area_rel = (mesh_area(&mesh) - want_area) / want_area;
                assert!(
                    (-1e-3..=1e-9).contains(&area_rel),
                    "{what}: mesh area against the closed form {want_area:.6} ({:+.4} %)",
                    area_rel * 100.0
                );

                let got = solid_volume(&topo, solid, 1e-4 * s).unwrap();
                let rel = (got - want_vol).abs() / want_vol;
                assert!(
                    rel < 1e-4,
                    "{what}: volume {got:.12} against the closed form {want_vol:.12} ({rel:e})"
                );
            }
            // The fuse handed back the whole wall. Characterised, not accepted:
            // every number here is the closed form of a body that should not
            // exist, so if the fuse is fixed this branch simply stops running.
            1 => {
                let want_wall = TAU * (R * s) * (H * s);
                let rel = (cylinders[0] - want_wall).abs() / want_wall;
                assert!(
                    rel < 1e-5,
                    "{what}: the single boss wall face meshes {:.6}, which is neither the \
                     split pair nor the whole wall {want_wall:.6} ({rel:e})",
                    cylinders[0]
                );

                let excess = summed_face_area(&topo, solid, fine) - closed_form_area(d, s);
                let want_excess = buried_wall_area(d, s);
                let ex_rel = (excess - want_excess).abs() / want_excess;
                assert!(
                    ex_rel < 1e-4,
                    "{what}: the boundary carries {excess:.6} of surface beyond the closed \
                     form, against the buried wall's {want_excess:.6} ({ex_rel:e})"
                );

                let mesh = tessellate_solid(&topo, solid, 1e-4 * s).unwrap();
                let (free, _) = mesh_edge_defects(&mesh);
                assert!(
                    free > 0,
                    "{what}: the un-split body meshed watertight — if the fuse now trims \
                     the wall this branch should not have been taken at all"
                );
            }
            // Anything else is a third topology nobody has characterised, and
            // is not something to pass over quietly.
            n => panic!("{what}: {n} cylindrical faces, expected 1 or 2"),
        }
    }
}

/// The cliff itself, named: at `d >= R/2` the fuse stops trimming the tool wall.
///
/// Below the threshold the boss wall comes back SPLIT — a tab of
/// `2 acos(d/R) * R * PLATE_Z` outside the plate and a ring of
/// `2 pi R (H - PLATE_Z)` above it. At and past it the wall comes back WHOLE,
/// `2 pi R H`, buried portion and all, so the boundary carries
/// `(2 pi - 2 acos(d/R)) R * PLATE_Z` of interior sheet.
///
/// It is a cliff, not a gradient: `0.4999 R` splits and `0.5001 R` does not,
/// and it tracks the RATIO — the same threshold for radii sharing no factors.
/// Exactly `0.5 R` is left out on purpose; that is the coin flip
/// `the_reported_body_measures_its_closed_form` covers.
///
/// This is a defect in `boolean`, not in tessellation, and this file does not
/// fix it. The test is here so that a fix is noticed: it fails the moment the
/// wall starts being trimmed past the threshold.
#[test]
fn the_fuse_stops_trimming_the_tool_wall_at_half_the_radius() {
    for r in [R, 5.0, 7.0, 4.5] {
        for (f, want_split) in [(0.45, true), (0.4999, true), (0.5001, false), (0.55, false)] {
            let d = r * f;
            let mut topo = Topology::new();
            let plate = make_box(&mut topo, PLATE_X, PLATE_Y, PLATE_Z).unwrap();
            let boss = make_cylinder(&mut topo, r, H).unwrap();
            transform_solid(&mut topo, boss, &Mat4::translation(d, PLATE_Y / 2.0, 0.0)).unwrap();
            let solid = boolean(&mut topo, BooleanOp::Fuse, plate, boss).unwrap();
            let what = format!("r={r}, axis at {f} R");

            let cylinders = cylinder_face_areas(&topo, solid, 1e-6);
            assert_eq!(
                cylinders.len() == 2,
                want_split,
                "{what}: {} cylindrical wall face(s), expected {}",
                cylinders.len(),
                if want_split { 2 } else { 1 }
            );

            let (tab, ring) = (2.0 * (d / r).acos() * r * PLATE_Z, TAU * r * (H - PLATE_Z));
            let wants: Vec<f64> = if want_split {
                vec![tab, ring]
            } else {
                vec![TAU * r * H]
            };
            for (got, want) in cylinders.iter().zip(&wants) {
                let rel = (got - want).abs() / want;
                assert!(
                    rel < 1e-5,
                    "{what}: a wall face meshes {got:.6} against the closed form \
                     {want:.6} ({rel:e})"
                );
            }
        }
    }
}
