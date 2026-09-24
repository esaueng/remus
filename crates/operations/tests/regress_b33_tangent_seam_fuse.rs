//! B33 (B26 finding 2): a box fused with a cylinder whose wall is tangent to
//! a box wall ALONG THE CYLINDER'S OWN SEAM LINE.
//!
//! The box-bottom plane × cylinder-wall section is the cylinder's whole bottom
//! circle. It touches the box's bottom rectangle at a single point, where the
//! seam vertex sits on the rectangle's edge. Because the seam and the tangency
//! coincide, phase FF's boundary-crossing collector returned one hit instead of
//! two, the ≥ 2-hit arc split never ran, and the whole circle was emitted as a
//! section of the box face. The splitter carved it in as a "hole" lying OUTSIDE
//! the rectangle, wound the same way as the cylinder wall's use of that circle,
//! and the cylinder's own bottom cap vanished: one shared edge with same-sense
//! uses (`ShellOrientationConsistent` on the check crate, "inconsistent face
//! orientations" on the operations strict validator) and a mesh with hundreds
//! of boundary edges. Volume could not see it — every z = 0 face contributes
//! nothing to the divergence integral — which is why the oracles below are
//! topological and positional first.
//!
//! The union is genuinely non-manifold as a point set along the contact
//! segment, and the B-Rep shares that segment's two end vertices between the
//! lumps. A tessellation may therefore meet the segment from both lumps; the
//! mesh oracle demands a CLOSED mesh (zero boundary edges, raw and
//! position-welded) and allows branching mesh edges only ON the contact
//! segment, never anywhere else.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, PI};

use proptest::prelude::*;
use remus_check::classify::{ClassifyOptions, PointClassification, classify_point};
use remus_check::validate::{CheckId, Severity, ValidateOptions};
use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::tessellate::{boundary_edge_count, tessellate_solid, welded_mesh_quality};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::solid::SolidId;

// ── Oracles ──────────────────────────────────────────────────────────

/// Exact-only boolean: an `Approximate` disclosure is a failure, never a pass.
fn exact(
    topo: &mut Topology,
    op: BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Result<SolidId, OperationsError> {
    let outcome = boolean_with_context(
        topo,
        op,
        a,
        b,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )?;
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "ExactOnly disclosed {:?}",
        outcome.quality
    );
    Ok(outcome.solid)
}

/// Both validators. `touching` results are two lumps meeting along a line:
/// the by-edge-id shell-connectivity check sees two components by
/// construction, so only that check is disabled for them.
fn assert_strict_valid(topo: &Topology, solid: SolidId, touching: bool, what: &str) {
    let ops = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        ops.is_valid(),
        "{what}: operations strict validator: {:?}",
        ops.issues
            .iter()
            .map(|i| &i.description)
            .collect::<Vec<_>>()
    );
    let mut options = ValidateOptions::default();
    if touching {
        options.disabled_checks.insert(CheckId::ShellConnected);
    }
    let report = remus_check::validate::validate_solid(topo, solid, &options).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == Severity::Error)
        .map(|i| format!("{:?}: {}", i.check, i.description))
        .collect();
    assert!(errors.is_empty(), "{what}: check-crate errors {errors:?}");
}

/// Independent recount of the defect's signature: every edge is used exactly
/// twice, in opposite effective senses, and no face carries a hole (none of
/// these unions has one; the defect's phantom hole lay outside its face).
fn assert_orientation_and_no_holes(topo: &Topology, solid: SolidId, what: &str) {
    let mut uses: BTreeMap<usize, Vec<bool>> = BTreeMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        assert!(
            face.inner_wires().is_empty(),
            "{what}: face {} carries {} inner wire(s)",
            fid.index(),
            face.inner_wires().len()
        );
        for oe in topo.wire(face.outer_wire()).unwrap().edges() {
            uses.entry(oe.edge().index())
                .or_default()
                .push(oe.is_forward() != face.is_reversed());
        }
    }
    for (edge, senses) in &uses {
        assert!(
            senses.len() == 2 && senses[0] != senses[1],
            "{what}: edge {edge} has uses {senses:?}"
        );
    }
}

/// Position-quantized closure: keyed by quantized endpoints + curve
/// midpoint, every geometric edge has exactly two uses (the by-id count is
/// blind to position-duplicate free edges).
fn assert_position_closed(topo: &Topology, solid: SolidId, scale: f64, what: &str) {
    let grid = 1e-6 * scale;
    let key = |p: Point3| {
        let q = |v: f64| (v / grid).round() as i64;
        (q(p.x()), q(p.y()), q(p.z()))
    };
    let mut counts: BTreeMap<_, usize> = BTreeMap::new();
    for fid in solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for oe in topo.wire(face.outer_wire()).unwrap().edges() {
            let edge = topo.edge(oe.edge()).unwrap();
            let a = topo.vertex(edge.start()).unwrap().point();
            let b = topo.vertex(edge.end()).unwrap().point();
            let (t0, t1) = edge.strict_domain().unwrap_or_else(|e| {
                panic!(
                    "{what}: edge {} lacks domain authority: {e:?}",
                    oe.edge().index()
                )
            });
            let mid = edge.curve().evaluate_with_endpoints(0.5 * (t0 + t1), a, b);
            let (ka, kb) = (key(a), key(b));
            let ends = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *counts.entry((ends, key(mid))).or_default() += 1;
        }
    }
    let bad: Vec<_> = counts.iter().filter(|&(_, &c)| c != 2).collect();
    assert!(bad.is_empty(), "{what}: position-keyed edge uses {bad:?}");
}

/// Volume against a closed form through both the tessellation-aware and the
/// exact mass-property routes, plus a rigid-translation re-measure.
fn assert_volume(topo: &Topology, solid: SolidId, expected: f64, scale: f64, what: &str) {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    let fine = ((aabb.max - aabb.min).length() * 1e-5).max(1e-7);
    let rel = |v: f64| ((v - expected) / expected).abs();
    for d in [fine, fine * 0.5] {
        let v = solid_volume(topo, solid, d).unwrap();
        assert!(
            rel(v) < 1e-7,
            "{what}: solid_volume({d:e}) = {v} vs {expected}"
        );
    }
    let m = mass_properties(topo, solid).unwrap().mass;
    assert!(rel(m) < 1e-7, "{what}: mass_properties = {m} vs {expected}");
    let mut moved = topo.clone();
    transform_solid(
        &mut moved,
        solid,
        &Mat4::translation(13.0 * scale, -7.0 * scale, 5.0 * scale),
    )
    .unwrap();
    let v = solid_volume(&moved, solid, fine).unwrap();
    assert!(rel(v) < 1e-7, "{what}: translated volume {v} vs {expected}");
}

/// Ray-cast ground truth at intent-encoding probes.
fn assert_classified(topo: &Topology, solid: SolidId, probes: &[(Point3, bool)], what: &str) {
    for &(p, inside) in probes {
        let got = classify_point(topo, solid, p, &ClassifyOptions::default()).unwrap();
        let want = if inside {
            PointClassification::Inside
        } else {
            PointClassification::Outside
        };
        assert_eq!(got, want, "{what}: probe {p:?}");
    }
}

/// A vertical contact segment `x = cx, y = cy, z ∈ [z0, z1]`, where the two
/// lumps of a tangent union meet.
#[derive(Clone, Copy)]
struct Contact {
    cx: f64,
    cy: f64,
    z0: f64,
    z1: f64,
}

/// Closed mesh at 0.1 and 0.01 (in model units of the unit-scale case) and at
/// the harness deflection (1e-5 of the bounding-box diagonal). Branching mesh
/// edges are allowed only on `contact`; `None` demands a 2-manifold mesh.
fn assert_mesh_closed(
    topo: &Topology,
    solid: SolidId,
    scale: f64,
    contact: Option<Contact>,
    what: &str,
) {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    let harness = ((aabb.max - aabb.min).length() * 1e-5).max(1e-7);
    for d in [0.1 * scale, 0.01 * scale, harness] {
        let mesh = tessellate_solid(topo, solid, d).unwrap();
        let boundary = boundary_edge_count(&mesh);
        let welded = welded_mesh_quality(&mesh);
        assert!(
            boundary == 0 && welded.boundary_edges == 0,
            "{what}: mesh at {d:e} has {boundary} boundary edges ({} welded)",
            welded.boundary_edges
        );
        let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for t in mesh.indices.chunks_exact(3) {
            for (p, q) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *count.entry((p.min(q), p.max(q))).or_default() += 1;
            }
        }
        let band = 1e-9 * scale.max(1.0);
        let on_contact = |i: u32| {
            let v = mesh.positions[i as usize];
            contact.is_some_and(|c| {
                (v.x() - c.cx).abs() <= band
                    && (v.y() - c.cy).abs() <= band
                    && v.z() >= c.z0 - band
                    && v.z() <= c.z1 + band
            })
        };
        let stray: Vec<_> = count
            .iter()
            .filter(|&(&(p, q), &c)| c > 2 && !(on_contact(p) && on_contact(q)))
            .map(|(&(p, q), &c)| (mesh.positions[p as usize], mesh.positions[q as usize], c))
            .collect();
        assert!(
            stray.is_empty(),
            "{what}: branching mesh edges off the contact segment at {d:e}: {stray:?}"
        );
    }
}

fn box_cylinder(
    dims: (f64, f64, f64),
    radius: f64,
    height: f64,
    placement: &Mat4,
) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let a = make_box(&mut topo, dims.0, dims.1, dims.2).unwrap();
    let b = make_cylinder(&mut topo, radius, height).unwrap();
    transform_solid(&mut topo, b, placement).unwrap();
    (topo, a, b)
}

// ── The pinned witness geometry ──────────────────────────────────────

/// B26 finding 2 at every generation scale: box(1, 2.5, 1.5) ∪ cyl(2.5, 1)
/// rotated π about z (seam at −x) and moved to (3.5, 0.5, 0), so the seam line
/// lies on the box's x = 1 wall.
#[test]
fn tangent_seam_fuse_is_exact_valid_and_closed_at_every_scale() {
    for scale in [1e-3, 1.0, 1e3] {
        let what = format!("tangent-seam fuse @ {scale:e}");
        let placement = Mat4::translation(3.5 * scale, 0.5 * scale, 0.0) * Mat4::rotation_z(PI);
        let (mut topo, a, b) = box_cylinder(
            (scale, 2.5 * scale, 1.5 * scale),
            2.5 * scale,
            scale,
            &placement,
        );
        let fused = exact(&mut topo, BooleanOp::Fuse, a, b).unwrap();

        assert_strict_valid(&topo, fused, true, &what);
        assert_orientation_and_no_holes(&topo, fused, &what);
        assert_position_closed(&topo, fused, scale, &what);
        // The intersection is the contact segment (measure zero), so the
        // union is the disjoint sum of the closed forms.
        let expected = (1.0 * 2.5 * 1.5 + PI * 2.5 * 2.5 * 1.0) * scale.powi(3);
        assert_volume(&topo, fused, expected, scale, &what);
        let p = |x: f64, y: f64, z: f64| Point3::new(x * scale, y * scale, z * scale);
        assert_classified(
            &topo,
            fused,
            &[
                (p(0.5, 1.25, 0.75), true),  // box body
                (p(3.5, 0.5, 0.5), true),    // cylinder body
                (p(3.5, 0.5, -0.01), false), // just below the restored bottom cap
                (p(3.5, 0.5, 1.01), false),  // just above the top cap
                (p(1.1, 1.5, 0.5), false),   // the notch beside the contact line
                (p(0.5, 2.0, -0.01), false), // just below the box bottom
            ],
            &what,
        );
        assert_mesh_closed(
            &topo,
            fused,
            scale,
            Some(Contact {
                cx: scale,
                cy: 0.5 * scale,
                z0: 0.0,
                z1: scale,
            }),
            &what,
        );
    }
}

// ── General-position companions (the normal paths still work) ────────

/// The same tangency with the seam rotated away, a genuine 0.1 overlap, and
/// a 0.1 gap: each stays exact, strict-valid, 2-manifold-meshed, and matches
/// its closed form.
#[test]
fn general_position_companions_stay_exact() {
    let box_v = 1.0 * 2.5 * 1.5;
    let cyl_v = PI * 2.5 * 2.5;

    for (rot, tag) in [(0.0, "seam at +x"), (FRAC_PI_2, "seam at +y")] {
        let what = format!("tangent fuse, {tag}");
        let placement = Mat4::translation(3.5, 0.5, 0.0) * Mat4::rotation_z(rot);
        let (mut topo, a, b) = box_cylinder((1.0, 2.5, 1.5), 2.5, 1.0, &placement);
        let fused = exact(&mut topo, BooleanOp::Fuse, a, b).unwrap();
        assert_strict_valid(&topo, fused, true, &what);
        assert_orientation_and_no_holes(&topo, fused, &what);
        assert_position_closed(&topo, fused, 1.0, &what);
        assert_volume(&topo, fused, box_v + cyl_v, 1.0, &what);
        assert_mesh_closed(&topo, fused, 1.0, None, &what);
    }

    // Overlap by s = 0.1 at y = 1.25: the lens is a circular segment of
    // height s on the unit-height cylinder, wholly inside the box
    // (chord y = 1.25 ± 0.7). Inclusion–exclusion: fuse = box + cyl − lens.
    let (r, s) = (2.5_f64, 0.1_f64);
    let lens = r * r * ((r - s) / r).acos() - (r - s) * (2.0 * r * s - s * s).sqrt();
    let overlap = Mat4::translation(3.4, 1.25, 0.0) * Mat4::rotation_z(PI);
    let (mut topo, a, b) = box_cylinder((1.0, 2.5, 1.5), r, 1.0, &overlap);
    let fused = exact(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    assert_strict_valid(&topo, fused, false, "overlap fuse");
    assert_orientation_and_no_holes(&topo, fused, "overlap fuse");
    assert_position_closed(&topo, fused, 1.0, "overlap fuse");
    assert_volume(&topo, fused, box_v + cyl_v - lens, 1.0, "overlap fuse");
    assert_classified(
        &topo,
        fused,
        &[
            (Point3::new(0.95, 1.25, 0.5), true),   // inside the lens
            (Point3::new(3.4, 1.25, -0.01), false), // below the merged bottom
        ],
        "overlap fuse",
    );
    assert_mesh_closed(&topo, fused, 1.0, None, "overlap fuse");
    let (mut topo, a, b) = box_cylinder((1.0, 2.5, 1.5), r, 1.0, &overlap);
    let common = exact(&mut topo, BooleanOp::Intersect, a, b).unwrap();
    assert_volume(&topo, common, lens, 1.0, "overlap intersect");

    let gap = Mat4::translation(3.6, 0.5, 0.0) * Mat4::rotation_z(PI);
    let (mut topo, a, b) = box_cylinder((1.0, 2.5, 1.5), r, 1.0, &gap);
    let fused = exact(&mut topo, BooleanOp::Fuse, a, b).unwrap();
    assert_strict_valid(&topo, fused, true, "gap fuse");
    assert_orientation_and_no_holes(&topo, fused, "gap fuse");
    assert_volume(&topo, fused, box_v + cyl_v, 1.0, "gap fuse");
    assert_mesh_closed(&topo, fused, 1.0, None, "gap fuse");
}

// ── The quarter-turn family (stock cylinder's seam on a box wall) ────

/// The `is_finding2` family arm (seed e3ff8db5): a cylinder(1.5, 1) stock
/// whose seam line (+x) is tangent to a quarter-turned box tool's wall.
/// Before the fix, a 1,296-member sweep of this family (every box dim in
/// {1, 1.5, 2, 2.5, 3, 4}³, both quarter turns, lifts −0.5/0/0.5) answered 504
/// wrong (both validators) and refused 504; after it all 1,296 are exact,
/// strict-valid, and match the closed form.
#[test]
fn quarter_turn_family_tangent_at_the_stock_seam() {
    let cyl_v = PI * 1.5 * 1.5;
    // Every box reaches z ≥ 0.5 from each lift below, so the contact is a
    // segment, never the single point of a box merely touching the cap plane.
    for dims in [
        (1.0, 1.0, 1.0),
        (2.0, 1.5, 1.5),
        (1.0, 2.5, 3.0),
        (4.0, 3.0, 2.0),
    ] {
        let (dx, dy, dz) = dims;
        for quarter in [1_u8, 3] {
            // Quarter turn (+1): the box spans x ∈ [1.5, 1.5 + dy],
            // y ∈ [−0.5, dx − 0.5]; three quarters: y ∈ [0.5 − dx, 0.5].
            let (ox, oy, box_mid_y) = if quarter == 1 {
                (1.5 + dy, -0.5, 0.5 * dx - 0.5)
            } else {
                (1.5, 0.5, 0.5 - 0.5 * dx)
            };
            for oz in [-0.5, 0.0, 0.5] {
                let what = format!("family box{dims:?} q{quarter} oz={oz}");
                let placement = Mat4::translation(ox, oy, oz)
                    * Mat4::rotation_z(f64::from(quarter) * FRAC_PI_2);
                let mut topo = Topology::new();
                let a = make_cylinder(&mut topo, 1.5, 1.0).unwrap();
                let b = make_box(&mut topo, dx, dy, dz).unwrap();
                transform_solid(&mut topo, b, &placement).unwrap();
                let fused = exact(&mut topo, BooleanOp::Fuse, a, b).unwrap();
                assert_strict_valid(&topo, fused, true, &what);
                assert_orientation_and_no_holes(&topo, fused, &what);
                assert_position_closed(&topo, fused, 1.0, &what);
                assert_volume(&topo, fused, cyl_v + dx * dy * dz, 1.0, &what);
                assert_classified(
                    &topo,
                    fused,
                    &[
                        (Point3::new(0.0, 0.0, 0.5), true), // cylinder body
                        (Point3::new(1.5 + 0.5 * dy, box_mid_y, oz + 0.5 * dz), true), // box body
                        (Point3::new(0.0, 0.0, -0.01), false), // below the cap
                        (Point3::new(-1.6, 0.0, 0.5), false), // beside the wall
                    ],
                    &what,
                );
                assert_mesh_closed(
                    &topo,
                    fused,
                    1.0,
                    Some(Contact {
                        cx: 1.5,
                        cy: 0.0,
                        z0: oz.max(0.0),
                        z1: (oz + dz).min(1.0),
                    }),
                    &what,
                );
            }
        }
    }
}

// ── Property: any seam-on-wall tangency in the box's height ─────────

proptest! {
    #![proptest_config(ProptestConfig::with_cases(16))]

    /// Box × cylinder whose seam line lies on the box's +x wall, the whole
    /// cylinder within the box's height, at any contact position along the
    /// wall and any generation scale: the union is exact, strict-valid,
    /// position-closed, the disjoint sum of the closed forms, and meshes
    /// closed with branching edges only on the contact segment — never a
    /// wrong answer.
    ///
    /// One cell refuses instead, typed (`ExactOnlyUnattainable`), identically
    /// on main before B33: the seam line exactly on the wall's mid-line
    /// (`fy == 5`). A 31,752-member sweep of this domain (every dx, dy, r,
    /// fy and scale index; h ∈ {1, 4}, lift ∈ {0, 2}) refused the same 580
    /// inputs before and after the fix, every one of them in that cell;
    /// before the fix 11,392 of the others answered wrong, after it none do.
    /// A refusal anywhere else is a regression.
    #[test]
    fn prop_seam_on_wall_tangent_fuse(
        dx in 2u8..=8, dy in 2u8..=8,
        r in 1u8..=6, h in 1u8..=4, lift in 0u8..=2,
        fy in 1u8..=9,
        scale in prop_oneof![Just(1e-3), Just(1.0), Just(1e3)],
    ) {
        let (dx, dy) = (f64::from(dx) * 0.5, f64::from(dy) * 0.5);
        let (r, h) = (f64::from(r) * 0.5, f64::from(h) * 0.5);
        let z0 = f64::from(lift) * 0.25;
        let dz = z0 + h + 0.5;
        let y = f64::from(fy) * 0.1 * dy;
        let what = format!("seam-on-wall box({dx},{dy},{dz}) cyl({r},{h}) y={y} z0={z0} @ {scale:e}");
        let placement =
            Mat4::translation((dx + r) * scale, y * scale, z0 * scale) * Mat4::rotation_z(PI);
        let (mut topo, a, b) =
            box_cylinder((dx * scale, dy * scale, dz * scale), r * scale, h * scale, &placement);
        let fused = match exact(&mut topo, BooleanOp::Fuse, a, b) {
            Ok(fused) => fused,
            Err(OperationsError::ExactOnlyUnattainable) if fy == 5 => return Ok(()),
            Err(e) => panic!("{what}: refused {e:?}"),
        };
        assert_strict_valid(&topo, fused, true, &what);
        assert_orientation_and_no_holes(&topo, fused, &what);
        assert_position_closed(&topo, fused, scale, &what);
        let expected = (dx * dy * dz + PI * r * r * h) * scale.powi(3);
        assert_volume(&topo, fused, expected, scale, &what);
        assert_mesh_closed(
            &topo,
            fused,
            scale,
            Some(Contact { cx: dx * scale, cy: y * scale, z0: z0 * scale, z1: (z0 + h) * scale }),
            &what,
        );
    }
}
