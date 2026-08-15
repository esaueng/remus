//! STEP `FACE_BOUND` loops form an unordered set.  A file that omits the
//! optional `FACE_OUTER_BOUND` subtype must therefore import identically no
//! matter where its perimeter loop appears in an `ADVANCED_FACE` bound list.
//!
//! The fixtures are generated from an exact two-bore plate and an exact
//! cross-drilled shaft, then rewritten in memory.  No customer geometry is
//! redistributed.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::{BTreeMap, HashMap};

use brepkit_io::step::{read_step, write_step};
use brepkit_math::aabb::Aabb3;
use brepkit_math::mat::Mat4;
use brepkit_math::vec::Point3;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::measure::{solid_bounding_box, solid_volume};
use brepkit_operations::primitives::{make_box, make_cylinder};
use brepkit_operations::tessellate::{
    TriangleMesh, tessellate_solid_with_tolerance, welded_mesh_quality,
};
use brepkit_operations::transform::transform_solid;
use brepkit_operations::validate::{validate_solid, validate_solid_relaxed};
use brepkit_topology::Topology;
use brepkit_topology::edge::EdgeId;
use brepkit_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use brepkit_topology::solid::SolidId;

const DEFLECTION: f64 = 0.01;
const ANGULAR_TOLERANCE: f64 = 5.0_f64.to_radians();
const SAME_GEOMETRY: f64 = 1e-8;

#[derive(Debug, Clone, Copy)]
enum OuterPosition {
    First,
    Middle,
    Last,
}

fn refs_in(text: &str) -> Vec<u64> {
    let bytes = text.as_bytes();
    let mut refs = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] != b'#' {
            i += 1;
            continue;
        }
        let start = i + 1;
        i = start;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            i += 1;
        }
        if let Ok(id) = text[start..i].parse() {
            refs.push(id);
        }
    }
    refs
}

fn entity_types(step: &str) -> HashMap<u64, &str> {
    step.lines()
        .filter_map(|line| {
            let rest = line.trim().strip_prefix('#')?;
            let (id, body) = rest.split_once('=')?;
            let (entity_type, _) = body.trim().split_once('(')?;
            Some((id.trim().parse().ok()?, entity_type.trim()))
        })
        .collect()
}

fn entity_id(line: &str) -> Option<u64> {
    line.trim()
        .strip_prefix('#')?
        .split_once('=')?
        .0
        .trim()
        .parse()
        .ok()
}

fn refs_in_rhs(line: &str) -> Vec<u64> {
    line.split_once('=')
        .map_or_else(Vec::new, |(_, body)| refs_in(body))
}

fn repeat_first_face_bound(step: &str, count: usize) -> String {
    let mut replaced = false;
    step.lines()
        .map(|source_line| {
            if replaced || !source_line.contains("= ADVANCED_FACE(") {
                return source_line.to_string();
            }
            let list_start = source_line.find("(#").expect("ADVANCED_FACE bound list");
            let list_end = source_line[list_start..]
                .find(')')
                .map(|offset| list_start + offset)
                .expect("ADVANCED_FACE bound-list end");
            let bound = refs_in(&source_line[list_start..list_end])[0];
            let repeated = std::iter::repeat_n(format!("#{bound}"), count)
                .collect::<Vec<_>>()
                .join(",");
            replaced = true;
            format!(
                "{}({}){}",
                &source_line[..list_start],
                repeated,
                &source_line[list_end + 1..]
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Reorder every multi-bound face using the writer's explicit outer subtype
/// as the ground truth.  Optionally erase that subtype after reordering.
fn reorder_face_bounds(step: &str, outer_position: OuterPosition, make_generic: bool) -> String {
    let types = entity_types(step);
    let mut multi_bound_faces = 0;
    let mut three_bound_faces = 0;
    let mut out = String::with_capacity(step.len());

    for source_line in step.lines() {
        let mut line = source_line.to_string();
        if line.contains("= ADVANCED_FACE(") {
            let list_start = line.find("(#").expect("ADVANCED_FACE bound list");
            let list_end = line[list_start..]
                .find(')')
                .map(|offset| list_start + offset)
                .expect("ADVANCED_FACE bound-list end");
            let mut bounds = refs_in(&line[list_start..list_end]);
            if bounds.len() > 1 {
                multi_bound_faces += 1;
                three_bound_faces += usize::from(bounds.len() >= 3);
                let outer_index = bounds
                    .iter()
                    .position(|id| types.get(id) == Some(&"FACE_OUTER_BOUND"))
                    .expect("writer must identify one explicit outer bound");
                let outer = bounds.remove(outer_index);
                let target = match outer_position {
                    OuterPosition::First => 0,
                    OuterPosition::Middle => bounds.len().div_ceil(2),
                    OuterPosition::Last => bounds.len(),
                };
                bounds.insert(target, outer);
                let replacement = format!(
                    "({})",
                    bounds
                        .iter()
                        .map(|id| format!("#{id}"))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
                line.replace_range(list_start..=list_end, &replacement);
            }
        }
        if make_generic {
            line = line.replace("FACE_OUTER_BOUND", "FACE_BOUND");
        }
        out.push_str(&line);
        out.push('\n');
    }

    assert!(
        multi_bound_faces >= 2,
        "two-bore plate should have at least two multi-bound cap faces"
    );
    assert!(
        three_bound_faces >= 2,
        "two-bore plate should exercise first, middle, and last positions"
    );
    if make_generic {
        assert!(!out.contains("FACE_OUTER_BOUND"));
    }
    out
}

/// Reorder the first multi-bound face on `surface_type`, optionally erasing
/// only that face's explicit outer subtype.  Other faces remain byte-for-byte
/// equivalent so this isolates periodic outer-role inference.
fn reorder_surface_face_bounds(
    step: &str,
    surface_type: &str,
    outer_position: OuterPosition,
    make_generic: bool,
) -> String {
    let types = entity_types(step);
    let (face_id, outer_bound) = step
        .lines()
        .find_map(|line| {
            if !line.contains("= ADVANCED_FACE(") {
                return None;
            }
            let list_start = line.find("(#")?;
            let list_end = line[list_start..].find(')')? + list_start;
            let bounds = refs_in(&line[list_start..list_end]);
            let all_refs = refs_in(line);
            let surface = all_refs.iter().rev().find(|id| !bounds.contains(id))?;
            let outer = bounds
                .iter()
                .find(|id| types.get(id) == Some(&"FACE_OUTER_BOUND"))?;
            (bounds.len() > 1 && types.get(surface) == Some(&surface_type))
                .then_some((entity_id(line)?, *outer))
        })
        .unwrap_or_else(|| panic!("fixture needs a multi-bound {surface_type} face"));

    let mut rewritten_face = false;
    let mut rewritten_outer = !make_generic;
    let out = step
        .lines()
        .map(|source_line| {
            let mut line = source_line.to_string();
            if entity_id(&line) == Some(face_id) {
                let list_start = line.find("(#").expect("bound list");
                let list_end = line[list_start..].find(')').expect("bound list end") + list_start;
                let mut bounds = refs_in(&line[list_start..list_end]);
                let outer_index = bounds
                    .iter()
                    .position(|&id| id == outer_bound)
                    .expect("target outer bound");
                let outer = bounds.remove(outer_index);
                let target = match outer_position {
                    OuterPosition::First => 0,
                    OuterPosition::Middle => bounds.len().div_ceil(2),
                    OuterPosition::Last => bounds.len(),
                };
                bounds.insert(target, outer);
                let replacement = format!(
                    "({})",
                    bounds
                        .iter()
                        .map(|id| format!("#{id}"))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
                line.replace_range(list_start..=list_end, &replacement);
                rewritten_face = true;
            }
            if make_generic && entity_id(&line) == Some(outer_bound) {
                line = line.replace("FACE_OUTER_BOUND", "FACE_BOUND");
                rewritten_outer = true;
            }
            line
        })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n";
    assert!(rewritten_face, "target ADVANCED_FACE was not rewritten");
    assert!(rewritten_outer, "target FACE_OUTER_BOUND was not rewritten");
    out
}

/// Express every bound in the equivalent `.F.` form: reverse its EDGE_LOOP
/// and then declare the bound orientation false, which reverses it back.
fn reverse_bound_formulations(step: &str) -> String {
    let types = entity_types(step);
    let bound_loops: Vec<u64> = step
        .lines()
        .filter_map(|line| {
            let id = entity_id(line)?;
            matches!(types.get(&id), Some(&"FACE_BOUND" | &"FACE_OUTER_BOUND"))
                .then(|| refs_in_rhs(line).first().copied())?
        })
        .collect();
    let oriented_edges: Vec<u64> = step
        .lines()
        .filter_map(|line| {
            let id = entity_id(line)?;
            bound_loops.contains(&id).then(|| refs_in_rhs(line))
        })
        .flatten()
        .filter(|id| types.get(id) == Some(&"ORIENTED_EDGE"))
        .collect();

    let mut out = String::with_capacity(step.len());
    for source_line in step.lines() {
        let id = entity_id(source_line);
        let mut line = source_line.to_string();
        if id.is_some_and(|id| bound_loops.contains(&id)) {
            let list_start = line.find("(#").expect("EDGE_LOOP list");
            let list_end = line[list_start..]
                .find(')')
                .map(|offset| list_start + offset)
                .expect("EDGE_LOOP list end");
            let mut refs = refs_in(&line[list_start..list_end]);
            refs.reverse();
            let replacement = format!(
                "({})",
                refs.iter()
                    .map(|id| format!("#{id}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            line.replace_range(list_start..=list_end, &replacement);
        }
        if id.is_some_and(|id| oriented_edges.contains(&id))
            || id.is_some_and(|id| {
                matches!(types.get(&id), Some(&"FACE_BOUND" | &"FACE_OUTER_BOUND"))
            })
        {
            if let Some(flag) = line.rfind(".T.") {
                line.replace_range(flag..flag + 3, ".F.");
            } else if let Some(flag) = line.rfind(".F.") {
                line.replace_range(flag..flag + 3, ".T.");
            } else {
                panic!("oriented entity has no boolean orientation: {line}");
            }
        }
        out.push_str(&line);
        out.push('\n');
    }
    assert!(out.contains("FACE_BOUND('',"));
    assert!(out.contains(".F.)"));
    out
}

fn with_multiple_explicit_outer_bounds(step: &str) -> String {
    let types = entity_types(step);
    let second_outer = step
        .lines()
        .find_map(|line| {
            line.contains("= ADVANCED_FACE(").then(|| {
                let list_start = line.find("(#").expect("bound list");
                let list_end = line[list_start..].find(')').expect("bound list end") + list_start;
                refs_in(&line[list_start..list_end])
                    .into_iter()
                    .find(|id| types.get(id) == Some(&"FACE_BOUND"))
            })?
        })
        .expect("multi-bound face with an inner bound");
    step.lines()
        .map(|line| {
            if entity_id(line) == Some(second_outer) {
                line.replace("FACE_BOUND", "FACE_OUTER_BOUND")
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n"
}

fn without_enclosing_bound(step: &str) -> String {
    let types = entity_types(step);
    let mut changed = false;
    let rewritten = step
        .lines()
        .map(|source_line| {
            let mut line = source_line.to_string();
            if !changed && line.contains("= ADVANCED_FACE(") {
                let list_start = line.find("(#").expect("bound list");
                let list_end = line[list_start..].find(')').expect("bound list end") + list_start;
                let bounds = refs_in(&line[list_start..list_end]);
                let inner: Vec<_> = bounds
                    .into_iter()
                    .filter(|id| types.get(id) == Some(&"FACE_BOUND"))
                    .collect();
                if inner.len() >= 2 {
                    let replacement = format!(
                        "({})",
                        inner
                            .iter()
                            .map(|id| format!("#{id}"))
                            .collect::<Vec<_>>()
                            .join(", ")
                    );
                    line.replace_range(list_start..=list_end, &replacement);
                    changed = true;
                }
            }
            line.replace("FACE_OUTER_BOUND", "FACE_BOUND")
        })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n";
    assert!(changed, "fixture needs a face with two holes");
    rewritten
}

fn with_duplicate_generic_cylinder_bound(step: &str) -> String {
    let types = entity_types(step);
    let target_bound = step
        .lines()
        .find_map(|line| {
            if !line.contains("= ADVANCED_FACE(") {
                return None;
            }
            let list_start = line.find("(#")?;
            let list_end = line[list_start..].find(')')? + list_start;
            let bounds = refs_in(&line[list_start..list_end]);
            let all_refs = refs_in(line);
            let surface = all_refs.iter().rev().find(|id| !bounds.contains(id))?;
            (bounds.len() == 1 && types.get(surface) == Some(&"CYLINDRICAL_SURFACE"))
                .then_some(bounds[0])
        })
        .expect("fixture needs a cylindrical face");

    step.lines()
        .map(|source_line| {
            let mut line = source_line.to_string();
            if line.contains("= ADVANCED_FACE(") {
                let list_start = line.find("(#").expect("bound list");
                let list_end = line[list_start..].find(')').expect("bound list end") + list_start;
                let bounds = refs_in(&line[list_start..list_end]);
                if bounds == [target_bound] {
                    line.replace_range(
                        list_start..=list_end,
                        &format!("(#{target_bound}, #{target_bound})"),
                    );
                }
            }
            if entity_id(&line) == Some(target_bound) {
                line = line.replace("FACE_OUTER_BOUND", "FACE_BOUND");
            }
            line
        })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n"
}

fn without_periodic_enclosing_bound(step: &str, surface_type: &str) -> String {
    let types = entity_types(step);
    let face_id = step
        .lines()
        .find_map(|line| {
            if !line.contains("= ADVANCED_FACE(") {
                return None;
            }
            let list_start = line.find("(#")?;
            let list_end = line[list_start..].find(')')? + list_start;
            let bounds = refs_in(&line[list_start..list_end]);
            let all_refs = refs_in(line);
            let surface = all_refs.iter().rev().find(|id| !bounds.contains(id))?;
            let generic_count = bounds
                .iter()
                .filter(|id| types.get(id) == Some(&"FACE_BOUND"))
                .count();
            (generic_count >= 2 && types.get(surface) == Some(&surface_type))
                .then(|| entity_id(line))?
        })
        .unwrap_or_else(|| panic!("fixture needs a holed {surface_type} face"));

    step.lines()
        .map(|source_line| {
            let mut line = source_line.to_string();
            if entity_id(&line) == Some(face_id) {
                let list_start = line.find("(#").expect("bound list");
                let list_end = line[list_start..].find(')').expect("bound list end") + list_start;
                let inner: Vec<_> = refs_in(&line[list_start..list_end])
                    .into_iter()
                    .filter(|id| types.get(id) == Some(&"FACE_BOUND"))
                    .collect();
                let replacement = format!(
                    "({})",
                    inner
                        .iter()
                        .map(|id| format!("#{id}"))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
                line.replace_range(list_start..=list_end, &replacement);
            }
            line
        })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n"
}

fn cylinder_at(topo: &mut Topology, radius: f64, x: f64, y: f64) -> SolidId {
    let cylinder = make_cylinder(topo, radius, 10.0).expect("cylinder");
    transform_solid(topo, cylinder, &Mat4::translation(x, y, -2.0)).expect("place cylinder");
    cylinder
}

fn two_bore_plate() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let plate = make_box(&mut topo, 40.0, 30.0, 6.0).expect("plate");
    let left = cylinder_at(&mut topo, 3.0, 12.0, 15.0);
    let right = cylinder_at(&mut topo, 4.0, 28.0, 15.0);
    let one_bore = boolean(&mut topo, BooleanOp::Cut, plate, left).expect("left bore");
    let two_bores = boolean(&mut topo, BooleanOp::Cut, one_bore, right).expect("right bore");
    (topo, two_bores)
}

fn cross_drilled_shaft() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let shaft = make_cylinder(&mut topo, 3.0, 30.0).expect("shaft");
    let length = 42.0;
    let bore = make_cylinder(&mut topo, 2.0, length).expect("bore");
    transform_solid(
        &mut topo,
        bore,
        &Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
    )
    .expect("rotate bore");
    transform_solid(
        &mut topo,
        bore,
        &Mat4::translation(-length / 2.0, 0.0, 15.0),
    )
    .expect("place bore");
    let drilled = boolean(&mut topo, BooleanOp::Cut, shaft, bore).expect("cross drill");
    (topo, drilled)
}

fn import_one(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).expect("import rewritten STEP");
    assert_eq!(solids.len(), 1, "fixture must contain one solid");
    let solid = solids[0];

    let strict = validate_solid(&topo, solid).expect("strict validation");
    assert!(
        strict.is_valid(),
        "strict validation errors: {:?}",
        strict.issues
    );
    let relaxed = validate_solid_relaxed(&topo, solid).expect("relaxed validation");
    assert!(
        relaxed.is_valid(),
        "relaxed validation errors: {:?}",
        relaxed.issues
    );
    (topo, solid)
}

/// Import for regressions whose exact manifold, winding, and geometry contract
/// is audited by `snapshot` immediately after this helper returns.
fn import_one_manifold(step: &str) -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(step, &mut topo).expect("import rewritten STEP");
    assert_eq!(solids.len(), 1, "fixture must contain one solid");
    (topo, solids[0])
}

#[derive(Debug)]
struct Snapshot {
    counts: (usize, usize, usize, usize),
    surface_types: BTreeMap<&'static str, usize>,
    bounds: Aabb3,
    volume: f64,
    triangle_count: usize,
    mesh_bounds: Aabb3,
    mesh_signed_volume: f64,
}

fn signed_mesh_volume(mesh: &TriangleMesh) -> f64 {
    let origin = Point3::new(0.0, 0.0, 0.0);
    mesh.indices
        .chunks_exact(3)
        .map(|tri| {
            let a = mesh.positions[tri[0] as usize];
            let b = mesh.positions[tri[1] as usize];
            let c = mesh.positions[tri[2] as usize];
            (a - origin).dot((b - origin).cross(c - origin)) / 6.0
        })
        .sum()
}

fn snapshot(topo: &Topology, solid: SolidId) -> Snapshot {
    let faces = solid_faces(topo, solid).expect("faces");
    let shell_count = 1 + topo.solid(solid).expect("solid").inner_shells().len();
    assert_eq!(shell_count, 1, "fixture must remain one closed shell");
    let mut surface_types = BTreeMap::new();
    let mut edge_uses: HashMap<EdgeId, usize> = HashMap::new();
    for &face in &faces {
        let face = topo.face(face).expect("face");
        *surface_types.entry(face.surface().type_tag()).or_default() += 1;
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for edge in topo.wire(wire).expect("wire").edges() {
                *edge_uses.entry(edge.edge()).or_default() += 1;
            }
        }
    }
    assert_eq!(
        edge_uses.values().filter(|&&uses| uses == 1).count(),
        0,
        "B-Rep must have no open edges"
    );
    assert_eq!(
        edge_uses.values().filter(|&&uses| uses > 2).count(),
        0,
        "B-Rep must have no non-manifold edges"
    );

    let mesh = tessellate_solid_with_tolerance(topo, solid, DEFLECTION, ANGULAR_TOLERANCE)
        .expect("tessellate");
    let quality = welded_mesh_quality(&mesh);
    assert_eq!(quality.boundary_edges, 0, "mesh must be closed");
    assert_eq!(quality.non_manifold_edges, 0, "mesh must be manifold");
    let mesh_signed_volume = signed_mesh_volume(&mesh);
    assert!(
        mesh_signed_volume > 0.0,
        "mesh must keep outward-positive winding"
    );

    Snapshot {
        counts: (
            shell_count,
            faces.len(),
            solid_edges(topo, solid).expect("edges").len(),
            solid_vertices(topo, solid).expect("vertices").len(),
        ),
        surface_types,
        bounds: solid_bounding_box(topo, solid).expect("bounds"),
        volume: solid_volume(topo, solid, DEFLECTION).expect("volume"),
        triangle_count: mesh.indices.len() / 3,
        mesh_bounds: Aabb3::try_from_points(mesh.positions.iter().copied()).expect("mesh bounds"),
        mesh_signed_volume,
    }
}

fn assert_close(actual: f64, expected: f64, label: &str) {
    let scale = actual.abs().max(expected.abs()).max(1.0);
    assert!(
        (actual - expected).abs() <= SAME_GEOMETRY * scale,
        "{label}: {actual:.12} != {expected:.12}"
    );
}

fn assert_bounds_equal(actual: Aabb3, expected: Aabb3, label: &str) {
    for (axis, actual, expected) in [
        ("min.x", actual.min.x(), expected.min.x()),
        ("min.y", actual.min.y(), expected.min.y()),
        ("min.z", actual.min.z(), expected.min.z()),
        ("max.x", actual.max.x(), expected.max.x()),
        ("max.y", actual.max.y(), expected.max.y()),
        ("max.z", actual.max.z(), expected.max.z()),
    ] {
        assert_close(actual, expected, &format!("{label} {axis}"));
    }
}

fn assert_same_geometry(actual: &Snapshot, expected: &Snapshot, label: &str) {
    assert_eq!(actual.counts, expected.counts, "{label}: topology counts");
    assert_eq!(
        actual.surface_types, expected.surface_types,
        "{label}: face surface types"
    );
    assert_bounds_equal(actual.bounds, expected.bounds, &format!("{label}: B-Rep"));
    assert_close(actual.volume, expected.volume, &format!("{label}: volume"));
    assert_eq!(
        actual.triangle_count, expected.triangle_count,
        "{label}: triangle count"
    );
    assert_bounds_equal(
        actual.mesh_bounds,
        expected.mesh_bounds,
        &format!("{label}: mesh"),
    );
    assert_close(
        actual.mesh_signed_volume,
        expected.mesh_signed_volume,
        &format!("{label}: signed mesh volume"),
    );
}

#[test]
fn generic_planar_bounds_are_independent_of_list_order() {
    let (source_topo, source_solid) = two_bore_plate();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    assert!(
        canonical_step.matches("= CIRCLE(").count() >= 4,
        "closed circular hole loops must exercise curve sampling rather than vertex chords"
    );
    let (canonical_topo, canonical_solid) = import_one(&canonical_step);
    let expected = snapshot(&canonical_topo, canonical_solid);

    for position in [
        OuterPosition::First,
        OuterPosition::Middle,
        OuterPosition::Last,
    ] {
        let rewritten = reorder_face_bounds(&canonical_step, position, true);
        let (topo, solid) = import_one(&rewritten);
        let actual = snapshot(&topo, solid);
        assert_same_geometry(&actual, &expected, &format!("generic {position:?}"));

        let round_trip = write_step(&topo, &[solid]).expect("round-trip export");
        let (round_topo, round_solid) = import_one(&round_trip);
        assert_same_geometry(
            &snapshot(&round_topo, round_solid),
            &expected,
            &format!("generic {position:?} round trip"),
        );
    }
}

#[test]
fn explicit_outer_bound_keeps_precedence_when_listed_last() {
    let (source_topo, source_solid) = two_bore_plate();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let (canonical_topo, canonical_solid) = import_one(&canonical_step);
    let expected = snapshot(&canonical_topo, canonical_solid);

    let reordered = reorder_face_bounds(&canonical_step, OuterPosition::Last, false);
    assert!(reordered.contains("FACE_OUTER_BOUND"));
    let (topo, solid) = import_one(&reordered);
    assert_same_geometry(&snapshot(&topo, solid), &expected, "explicit outer last");
}

#[test]
fn false_face_bound_orientation_is_preserved_during_classification() {
    let (source_topo, source_solid) = two_bore_plate();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let (canonical_topo, canonical_solid) = import_one(&canonical_step);
    let expected = snapshot(&canonical_topo, canonical_solid);

    let false_form = reverse_bound_formulations(&canonical_step);
    let generic = reorder_face_bounds(&false_form, OuterPosition::Last, true);
    let (topo, solid) = import_one(&generic);
    assert_same_geometry(
        &snapshot(&topo, solid),
        &expected,
        "generic false-orientation bounds",
    );
}

#[test]
fn multiple_explicit_outer_bounds_are_rejected() {
    let (source_topo, source_solid) = two_bore_plate();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let malformed = with_multiple_explicit_outer_bounds(&canonical_step);
    let error = read_step(&malformed, &mut Topology::new()).expect_err("must reject two outers");
    assert!(
        error
            .to_string()
            .contains("multiple FACE_OUTER_BOUND entities"),
        "unexpected diagnostic: {error}"
    );
}

#[test]
fn disconnected_generic_bounds_fail_with_stable_diagnostic() {
    let (source_topo, source_solid) = two_bore_plate();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let malformed = without_enclosing_bound(&canonical_step);
    let error = read_step(&malformed, &mut Topology::new())
        .expect_err("two disjoint hole loops have no enclosing perimeter");
    assert!(
        error
            .to_string()
            .contains("do not have one enclosing outer boundary"),
        "unexpected diagnostic: {error}"
    );
}

#[test]
fn generic_cylindrical_bounds_are_order_independent_and_round_trip_exactly() {
    let (source_topo, source_solid) = cross_drilled_shaft();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let (canonical_topo, canonical_solid) = import_one_manifold(&canonical_step);
    let expected = snapshot(&canonical_topo, canonical_solid);

    for position in [
        OuterPosition::First,
        OuterPosition::Middle,
        OuterPosition::Last,
    ] {
        let rewritten =
            reorder_surface_face_bounds(&canonical_step, "CYLINDRICAL_SURFACE", position, true);
        let (topo, solid) = import_one_manifold(&rewritten);
        assert_same_geometry(
            &snapshot(&topo, solid),
            &expected,
            &format!("generic cylinder {position:?}"),
        );

        let round_trip = write_step(&topo, &[solid]).expect("round-trip export");
        assert!(
            round_trip.contains("FACE_OUTER_BOUND"),
            "writer must restore the explicit outer subtype"
        );
        let (round_topo, round_solid) = import_one_manifold(&round_trip);
        assert_same_geometry(
            &snapshot(&round_topo, round_solid),
            &expected,
            &format!("generic cylinder {position:?} round trip"),
        );
    }
}

#[test]
fn explicit_outer_bound_keeps_precedence_on_periodic_face() {
    let (source_topo, source_solid) = cross_drilled_shaft();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let (canonical_topo, canonical_solid) = import_one_manifold(&canonical_step);
    let expected = snapshot(&canonical_topo, canonical_solid);

    let reordered = reorder_surface_face_bounds(
        &canonical_step,
        "CYLINDRICAL_SURFACE",
        OuterPosition::Last,
        false,
    );
    let (topo, solid) = import_one_manifold(&reordered);
    assert_same_geometry(
        &snapshot(&topo, solid),
        &expected,
        "explicit periodic outer last",
    );
}

#[test]
fn false_bound_orientation_is_preserved_for_generic_periodic_face() {
    let (source_topo, source_solid) = cross_drilled_shaft();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let (canonical_topo, canonical_solid) = import_one_manifold(&canonical_step);
    let expected = snapshot(&canonical_topo, canonical_solid);

    let false_form = reverse_bound_formulations(&canonical_step);
    let generic = reorder_surface_face_bounds(
        &false_form,
        "CYLINDRICAL_SURFACE",
        OuterPosition::Middle,
        true,
    );
    let (topo, solid) = import_one_manifold(&generic);
    assert_same_geometry(
        &snapshot(&topo, solid),
        &expected,
        "generic periodic false-orientation bounds",
    );
}

#[test]
fn disconnected_generic_periodic_bounds_fail_closed() {
    let (source_topo, source_solid) = cross_drilled_shaft();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let malformed = without_periodic_enclosing_bound(&canonical_step, "CYLINDRICAL_SURFACE");
    let error = read_step(&malformed, &mut Topology::new())
        .expect_err("two breakout loops have no enclosing periodic perimeter");
    assert!(
        error.to_string().contains(
            "do not have one enclosing outer boundary in the unwrapped cylinder UV domain"
        ),
        "unexpected diagnostic: {error}"
    );
}

#[test]
fn duplicate_generic_periodic_bounds_fail_closed() {
    let (source_topo, source_solid) = two_bore_plate();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let malformed = with_duplicate_generic_cylinder_bound(&canonical_step);
    let error = read_step(&malformed, &mut Topology::new())
        .expect_err("duplicate generic periodic loops have no unique outer");
    assert!(
        error.to_string().contains(
            "do not have one enclosing outer boundary in the unwrapped cylinder UV domain"
        ),
        "unexpected diagnostic: {error}"
    );
}

#[test]
fn excessive_face_bound_references_are_rejected_before_classification() {
    let (source_topo, source_solid) = two_bore_plate();
    let canonical_step = write_step(&source_topo, &[source_solid]).expect("write fixture");
    let hostile = repeat_first_face_bound(&canonical_step, 129);

    let error = read_step(&hostile, &mut Topology::new())
        .expect_err("attacker-controlled face-bound lists must be capped");
    assert!(
        error
            .to_string()
            .contains("import limit exceeded for STEP bounds per ADVANCED_FACE"),
        "unexpected diagnostic: {error}"
    );
}
