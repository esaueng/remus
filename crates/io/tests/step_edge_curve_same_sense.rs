//! `EDGE_CURVE.same_sense` — ISO 10303-42's flag for an edge that runs
//! start → end *against* its curve's own parameterization.
//!
//! The reader used to drop it, which is only harmless when the two vertices
//! pin down the traversal on their own. On a periodic curve they do not: a
//! `.F.` arc came back as the complement of the intended one, so a millimetre
//! of fillet imported as very nearly a whole circle. Real CATIA and NX
//! exports are full of `.F.` edges, so this quietly deformed most imported
//! solids while leaving the face, edge and vertex counts looking perfect.
//!
//! The source part that exposed this is proprietary customer geometry and is
//! deliberately not vendored here. Instead the fixture below is generated,
//! then rewritten into the equivalent `.F.` formulation: each CIRCLE's
//! placement axis is negated and the referencing EDGE_CURVE's `same_sense` is
//! flipped to `.F.`. Negating the axis reverses the circle's parameterization,
//! so declaring `.F.` against it describes the *identical* point set — the two
//! files are two spellings of one solid, and any conformant reader has to
//! import them the same way. That is exactly the spelling CATIA emits.

#![allow(clippy::unwrap_used, clippy::panic)]

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;

use remus_io::step::{read_step, write_step};
use remus_math::aabb::Aabb3;
use remus_math::curves::Circle3D;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::{edge_length, solid_volume};
use remus_operations::primitives::make_box;
use remus_operations::tessellate::tessellate;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::solid::SolidId;

/// Tolerance for comparing two spellings of the same solid. They travel
/// through the same tessellator with the same deflection, so the only slack
/// needed is decimal round-tripping through the STEP text.
const SAME_GEOMETRY: f64 = 1e-6;

const DEFLECTION: f64 = 0.01;

// ---------------------------------------------------------------------------
// STEP text surgery
// ---------------------------------------------------------------------------

/// One entity per line, as `(id, type, attribute text)`.
fn entities(step: &str) -> Vec<(u64, String, String)> {
    step.lines()
        .filter_map(|line| {
            let rest = line.trim().strip_prefix('#')?;
            let (id, body) = rest.split_once('=')?;
            let (ty, attrs) = body.trim().split_once('(')?;
            Some((id.trim().parse().ok()?, ty.trim().to_string(), attrs.into()))
        })
        .collect()
}

/// Every `#NNN` reference in an attribute string, in order.
fn refs_in(attrs: &str) -> Vec<u64> {
    let bytes = attrs.as_bytes();
    let mut refs = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'#' {
            let start = i + 1;
            i = start;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            if let Ok(n) = attrs[start..i].parse() {
                refs.push(n);
            }
        } else {
            i += 1;
        }
    }
    refs
}

/// The three floats of a `DIRECTION('', (x, y, z))` — the inner tuple, not
/// the entity's own parentheses.
fn direction_components(body: &str) -> [f64; 3] {
    let entity_open = body.find('(').unwrap();
    let tuple_open = body[entity_open + 1..].find('(').unwrap() + entity_open + 1;
    let tuple_close = body[tuple_open..].find(')').unwrap() + tuple_open;
    let parts: Vec<f64> = body[tuple_open + 1..tuple_close]
        .split(',')
        .map(|p| p.trim().parse().unwrap())
        .collect();
    assert_eq!(parts.len(), 3, "DIRECTION needs three components");
    [parts[0], parts[1], parts[2]]
}

/// Rewrite a remus-written STEP into the equivalent `.F.` spelling.
///
/// Returns the new text and how many EDGE_CURVEs were flipped.
fn to_reversed_sense_formulation(step: &str) -> (String, usize) {
    let ents = entities(step);
    let by_id: HashMap<u64, (String, String)> = ents
        .iter()
        .map(|(id, ty, attrs)| (*id, (ty.clone(), attrs.clone())))
        .collect();

    let mut refcount: HashMap<u64, usize> = HashMap::new();
    for (_, _, attrs) in &ents {
        for r in refs_in(attrs) {
            *refcount.entry(r).or_default() += 1;
        }
    }

    let mut axes_to_negate = HashSet::new();
    let mut circles = HashSet::new();
    for (id, ty, attrs) in &ents {
        if ty != "CIRCLE" {
            continue;
        }
        let placement = refs_in(attrs)[0];
        let (placement_ty, placement_attrs) = &by_id[&placement];
        assert_eq!(placement_ty, "AXIS2_PLACEMENT_3D");
        let axis = refs_in(placement_attrs)[1];
        // The rewrite only stays semantics-preserving while the placement and
        // its axis belong to this CIRCLE alone. remus's writer never shares
        // them; assert it rather than trust it, so a future writer that
        // starts deduplicating turns this into a loud failure instead of a
        // silently corrupted fixture.
        assert_eq!(refcount[&placement], 1, "CIRCLE #{id} shares its placement");
        assert_eq!(refcount[&axis], 1, "CIRCLE #{id} shares its axis direction");
        axes_to_negate.insert(axis);
        circles.insert(*id);
    }
    assert!(
        !circles.is_empty(),
        "fixture has no CIRCLE geometry, so it cannot exercise same_sense"
    );

    let mut flipped = 0;
    let mut out = String::with_capacity(step.len());
    for line in step.lines() {
        let parsed = line
            .trim()
            .strip_prefix('#')
            .and_then(|rest| rest.split_once('='))
            .and_then(|(id, body)| Some((id.trim().parse::<u64>().ok()?, body.trim())));

        match parsed {
            Some((id, body)) if axes_to_negate.contains(&id) => {
                let [x, y, z] = direction_components(body);
                let _ = write!(out, "#{id} = DIRECTION('', ({}, {}, {}));", -x, -y, -z);
            }
            Some((id, body))
                if body.starts_with("EDGE_CURVE") && circles.contains(&refs_in(body)[2]) =>
            {
                assert!(body.contains(".T."), "EDGE_CURVE #{id} was not written .T.");
                let _ = write!(out, "#{id} = {}", body.replace(".T.", ".F."));
                flipped += 1;
            }
            _ => out.push_str(line),
        }
        out.push('\n');
    }
    (out, flipped)
}

// ---------------------------------------------------------------------------
// Fixture and measurements
// ---------------------------------------------------------------------------

/// A block with all four vertical edges rounded — the synthetic stand-in for
/// a filleted machined part, and the smallest shape that puts real circular
/// arcs on a solid whose exact volume is known in closed form.
fn filleted_block(topo: &mut Topology) -> SolidId {
    const DX: f64 = 40.0;
    const DY: f64 = 30.0;
    const DZ: f64 = 20.0;
    const RADIUS: f64 = 5.0;

    let block = make_box(topo, DX, DY, DZ).unwrap();
    let vertical: Vec<_> = solid_edges(topo, block)
        .unwrap()
        .into_iter()
        .filter(|&e| {
            let edge = topo.edge(e).unwrap();
            let (s, t) = (
                topo.vertex(edge.start()).unwrap().point(),
                topo.vertex(edge.end()).unwrap().point(),
            );
            (s.x() - t.x()).abs() < 1e-9 && (s.y() - t.y()).abs() < 1e-9
        })
        .collect();
    assert_eq!(vertical.len(), 4, "expected four vertical box edges");

    #[allow(deprecated)]
    let filleted =
        remus_operations::fillet::fillet_rolling_ball(topo, block, &vertical, RADIUS).unwrap();

    // Four rounded corners remove (4 − π)r² of cross-section.
    let expected = (DX * DY - (4.0 - std::f64::consts::PI) * RADIUS * RADIUS) * DZ;
    let measured = solid_volume(topo, filleted, DEFLECTION).unwrap();
    assert!(
        (measured - expected).abs() / expected < 1e-3,
        "fixture volume {measured} is not the closed-form {expected}"
    );
    filleted
}

/// Bounds of the tessellated surface, not of the vertices.
///
/// A complement arc keeps every vertex where it was — the wrong geometry
/// lives strictly *between* them — so a vertex-corner bounding box cannot see
/// it. Only the meshed surface shows the sweep escaping the part.
fn tessellated_bounds(topo: &Topology, solid: SolidId) -> Aabb3 {
    let points = solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .flat_map(|f| tessellate(topo, f, DEFLECTION).unwrap().positions);
    Aabb3::try_from_points(points).unwrap()
}

/// Every edge length in the solid, sorted, so two imports can be compared
/// without depending on edge ordering.
fn sorted_edge_lengths(topo: &Topology, solid: SolidId) -> Vec<f64> {
    let mut lengths: Vec<f64> = solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .map(|e| edge_length(topo, e).unwrap())
        .collect();
    lengths.sort_by(f64::total_cmp);
    lengths
}

fn import_one(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).unwrap();
    assert_eq!(solids.len(), 1, "fixture should import as a single solid");
    let solid = solids[0];
    (topo, solid)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// The two spellings of one solid must import to one solid.
///
/// Before `same_sense` was honoured this failed on volume: the flipped
/// spelling swept the fillet arcs the long way round and came back several
/// percent off, with a face/edge/vertex count still matching exactly.
#[test]
fn reversed_sense_spelling_imports_to_the_same_solid() {
    let mut authored = Topology::new();
    let solid = filleted_block(&mut authored);
    let step = write_step(&authored, &[solid]).unwrap();

    let (step_reversed, flipped) = to_reversed_sense_formulation(&step);
    assert!(
        flipped >= 8,
        "expected at least the eight fillet arcs to flip, got {flipped}"
    );

    let (forward_topo, forward) = import_one(&step);
    let (reversed_topo, reversed) = import_one(&step_reversed);

    let forward_volume = solid_volume(&forward_topo, forward, DEFLECTION).unwrap();
    let reversed_volume = solid_volume(&reversed_topo, reversed, DEFLECTION).unwrap();
    assert!(
        (forward_volume - reversed_volume).abs() / forward_volume < SAME_GEOMETRY,
        "same solid, two spellings, different volume: \
         .T. gave {forward_volume}, .F. gave {reversed_volume}"
    );

    let forward_bounds = tessellated_bounds(&forward_topo, forward);
    let reversed_bounds = tessellated_bounds(&reversed_topo, reversed);
    let corners = |b: &Aabb3| {
        [
            b.min.x(),
            b.min.y(),
            b.min.z(),
            b.max.x(),
            b.max.y(),
            b.max.z(),
        ]
    };
    let (forward_corners, reversed_corners) = (corners(&forward_bounds), corners(&reversed_bounds));
    for (a, b) in forward_corners.iter().zip(&reversed_corners) {
        assert!(
            (a - b).abs() < 1e-6,
            ".F. spelling sweeps outside the part — \
             .T. bounds {forward_corners:?}, .F. bounds {reversed_corners:?}"
        );
    }

    let forward_lengths = sorted_edge_lengths(&forward_topo, forward);
    let reversed_lengths = sorted_edge_lengths(&reversed_topo, reversed);
    assert_eq!(forward_lengths.len(), reversed_lengths.len());
    for (a, b) in forward_lengths.iter().zip(&reversed_lengths) {
        assert!(
            (a - b).abs() < 1e-6,
            "edge lengths diverge: {forward_lengths:?} vs {reversed_lengths:?}"
        );
    }
}

/// The mechanism, isolated: a short `.F.` arc must not come back as its
/// complement.
///
/// The dimensions mirror a fillet arc measured on the part that exposed this
/// — a 147.942 mm-radius fillet whose two vertices sit 1.28 mm apart, a true
/// sweep of about 0.0087 rad. Read against the curve's own parameterization
/// it becomes 2π − 0.0087 ≈ 6.2745 rad, sweeping the rounding all the way
/// around the far side of the circle.
#[test]
fn a_clockwise_arc_is_not_imported_as_its_complement() {
    const RADIUS: f64 = 147.942;
    let centre = Point3::new(38.0, 0.0, 0.0);
    let circle = Circle3D::new(centre, Vec3::new(1.0, 0.0, 0.0), RADIUS).unwrap();

    // Two points on the circle, a hair apart, taken in the clockwise order —
    // which is what a `.F.` EDGE_CURVE declares.
    let sweep = 0.008_66_f64;
    let start = circle.evaluate(sweep);
    let end = circle.evaluate(0.0);
    assert!(
        ((start - end).length() - 1.281).abs() < 1e-2,
        "fixture chord should be about 1.28 mm"
    );

    // Read as written, the topology layer can only offer the long way round.
    let (t0, t1) = EdgeCurve::Circle(circle.clone()).domain_with_endpoints(start, end);
    assert!(
        (t1 - t0 - (std::f64::consts::TAU - sweep)).abs() < 1e-6,
        "expected the complement sweep, got {}",
        t1 - t0
    );

    // Reversing the curve — what the reader now does for `.F.` — recovers the
    // arc the file meant.
    let (t0, t1) = EdgeCurve::Circle(circle.reversed()).domain_with_endpoints(start, end);
    assert!(
        (t1 - t0 - sweep).abs() < 1e-9,
        "expected a {sweep} rad arc, got {}",
        t1 - t0
    );

    // And it is the same arc, not merely one of the same length: every
    // sample sits on the short side.
    let reversed = circle.reversed();
    for i in 0..=8 {
        let t = t0 + (t1 - t0) * f64::from(i) / 8.0;
        let point = reversed.evaluate(t);
        assert!(
            (point - start).length() <= 1.3 && (point - end).length() <= 1.3,
            "sample at {t} left the short arc"
        );
    }
}

/// Reversal must not disturb a closed circle's identity.
///
/// OpenZCAD fingerprints a closed edge by its four-sample centre plus a
/// sign-canonicalized plane normal (ADR-011). Both survive a reversal here:
/// the point set is untouched and `u_axis` is deliberately left alone, so
/// `evaluate(0.0)` — the seam — does not move either. A closed `.F.` edge
/// therefore changes its winding without changing its fingerprint.
#[test]
fn reversing_a_closed_circle_keeps_its_identity() {
    let centre = Point3::new(3.0, -4.0, 11.0);
    let circle = Circle3D::new(centre, Vec3::new(0.3, -0.5, 0.81), 7.25).unwrap();
    let reversed = circle.reversed();

    assert!(
        (reversed.evaluate(0.0) - circle.evaluate(0.0)).length() < 1e-12,
        "the seam point moved"
    );
    assert!(
        (reversed.center() - circle.center()).length() < 1e-12,
        "the centre moved"
    );
    assert!(
        (reversed.normal() + circle.normal()).length() < 1e-12,
        "the normal should be exactly negated"
    );

    // Four-sample mean, the quantity ADR-011 actually hashes.
    let mean = |c: &Circle3D| {
        let mut acc = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..4 {
            let p = c.evaluate(std::f64::consts::TAU * f64::from(i) / 4.0);
            acc += Vec3::new(p.x(), p.y(), p.z());
        }
        acc * 0.25
    };
    assert!(
        (mean(&circle) - mean(&reversed)).length() < 1e-12,
        "the four-sample centre moved"
    );

    // Traversal reverses, which is the whole point.
    for i in 1..8 {
        let t = std::f64::consts::TAU * f64::from(i) / 8.0;
        assert!(
            (reversed.evaluate(t) - circle.evaluate(-t)).length() < 1e-12,
            "reversed(t) should equal original(-t)"
        );
    }
}
