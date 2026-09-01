//! Structured fuzzing of native arena serialization round-trips.
//!
//! Every accepted input builds a small, independently measurable topology —
//! boxes and cylinders with known census and closed-form volumes, duplicate
//! solid roots, shared-shell solid aliases, compounds with repeated or
//! aliased members, per-vertex and per-edge tolerances, cylinder rim trims,
//! and public entity attributes — serializes it with
//! [`serialize_document`], and replays it with
//! [`deserialize_document_with_limits`].
//!
//! The value is entirely in the oracles, not in reaching a panic:
//!
//! * every restored solid passes comprehensive validation and the
//!   closed-manifold census, and measures its closed-form volume
//!   (`dx * dy * dz` for boxes, `π * r² * h` for cylinders), checked per
//!   root/member position so a swap of equal-census bodies is caught;
//! * the round-trip preserves the exact public state: vertex/edge tolerance
//!   bit patterns, cylinder rim trims, and attribute contents all survive;
//! * duplicate roots and aliased compound members keep their identity
//!   relationships (duplicate roots deserialize to the same fresh handle;
//!   aliases share one shell);
//! * serialize → deserialize → serialize is byte-identical (this is the
//!   oracle that caught serde_json losing the last bit of arbitrary
//!   tolerance values without `float_roundtrip`);
//! * a deliberately corrupted root, member, wire, or version field in an
//!   otherwise valid document is refused with a typed error, leaves a
//!   pre-populated destination topology untouched, and leaks no staged
//!   allocations.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod common;
mod invariants;

use invariants as inv;
use remus_io::IoError;
use remus_io::arena_io::{
    DeserializedDocument, deserialize_document_with_limits, serialize_document,
};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::{make_box, make_cylinder};
use remus_operations::validate::validate_solid;
use remus_topology::Topology;
use remus_topology::attributes::{ColorRgb, EntityAttributes};
use remus_topology::compound::{Compound, CompoundId};
use remus_topology::explorer;
use remus_topology::solid::{Solid, SolidId};

struct Bytes<'a> {
    data: &'a [u8],
    index: usize,
}

impl<'a> Bytes<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, index: 0 }
    }

    fn next(&mut self) -> u8 {
        if self.data.is_empty() {
            return 0;
        }
        let value = self.data[self.index % self.data.len()];
        self.index += 1;
        value
    }

    fn unit(&mut self) -> f64 {
        f64::from(self.next()) / f64::from(u8::MAX)
    }

    /// A dimension in `[0.25, 4]`, quantized to sixteenths so closed-form
    /// volumes are exact in binary floating point.
    fn dim(&mut self) -> f64 {
        ((0.25 + self.unit() * 3.75) * 16.0).round() / 16.0
    }

    /// An arbitrary positive tolerance in `[1e-12, 1e-8]` whose bit pattern
    /// is hostile to lossy float printers.
    fn tolerance(&mut self) -> f64 {
        10f64.powf(-12.0 + self.unit() * 4.0)
    }
}

/// A solid whose census, volume, and diagonal are known by construction.
struct Known {
    faces: usize,
    edges: usize,
    vertices: usize,
    volume: f64,
    diag: f64,
}

fn build_box(topo: &mut Topology, bytes: &mut Bytes<'_>, scale: f64) -> (SolidId, Known) {
    let (dx, dy, dz) = (
        bytes.dim() * scale,
        bytes.dim() * scale,
        bytes.dim() * scale,
    );
    let id = make_box(topo, dx, dy, dz)
        .unwrap_or_else(|error| panic!("bounded box was rejected: {error}"));
    (
        id,
        Known {
            faces: 6,
            edges: 12,
            vertices: 8,
            volume: dx * dy * dz,
            diag: (dx * dx + dy * dy + dz * dz).sqrt(),
        },
    )
}

fn build_cylinder(topo: &mut Topology, bytes: &mut Bytes<'_>, scale: f64) -> (SolidId, Known) {
    let (r, h) = (bytes.dim() * scale, bytes.dim() * scale);
    let id = make_cylinder(topo, r, h)
        .unwrap_or_else(|error| panic!("bounded cylinder was rejected: {error}"));
    (
        id,
        Known {
            faces: 3,
            edges: 3,
            vertices: 2,
            volume: std::f64::consts::PI * r * r * h,
            diag: (4.0 * r * r + h * h).sqrt(),
        },
    )
}

/// Bit-pattern multisets of the serialized public state that must survive a
/// round-trip exactly: vertex tolerances, edge tolerances, and edge trims.
#[derive(Debug, Default, PartialEq)]
struct PrecisionState {
    vertex_tolerances: Vec<u64>,
    edge_tolerances: Vec<u64>,
    trims: Vec<(u64, u64)>,
}

fn precision_state(topo: &Topology, solids: &[SolidId]) -> PrecisionState {
    let mut state = PrecisionState::default();
    let mut solids_seen = std::collections::BTreeSet::new();
    let mut edges_seen = std::collections::BTreeSet::new();
    let mut vertices_seen = std::collections::BTreeSet::new();
    for &solid in solids {
        if !solids_seen.insert(solid.index()) {
            continue;
        }
        for face in explorer::solid_faces(topo, solid)
            .unwrap_or_else(|error| panic!("live solid has no faces: {error}"))
        {
            for vertex in explorer::face_vertices(topo, face)
                .unwrap_or_else(|error| panic!("live face has no vertices: {error}"))
            {
                if vertices_seen.insert(vertex.index()) {
                    let tolerance = topo
                        .vertex(vertex)
                        .unwrap_or_else(|error| panic!("live vertex: {error}"))
                        .tolerance();
                    state.vertex_tolerances.push(tolerance.to_bits());
                }
            }
            for edge in explorer::face_edges(topo, face)
                .unwrap_or_else(|error| panic!("live face has no edges: {error}"))
            {
                if edges_seen.insert(edge.index()) {
                    let data = topo
                        .edge(edge)
                        .unwrap_or_else(|error| panic!("live edge: {error}"));
                    if let Some(tolerance) = data.tolerance() {
                        state.edge_tolerances.push(tolerance.to_bits());
                    }
                    if let Some((start, end)) = data.trim() {
                        state.trims.push((start.to_bits(), end.to_bits()));
                    }
                }
            }
        }
    }
    state.vertex_tolerances.sort_unstable();
    state.edge_tolerances.sort_unstable();
    state.trims.sort_unstable();
    state
}

/// Attribute contents as value multisets: names and color channel bits.
/// Only attributes on the given (captured) solids and their faces count —
/// the document legitimately does not carry state of entities outside the
/// selected roots.
#[derive(Debug, Default, PartialEq)]
struct AttributeState {
    names: Vec<String>,
    colors: Vec<(u64, u64, u64)>,
}

fn attribute_state(topo: &Topology, solids: &[SolidId]) -> AttributeState {
    let mut state = AttributeState::default();
    let mut collect = |attributes: &EntityAttributes| {
        if let Some(name) = &attributes.name {
            state.names.push(name.clone());
        }
        if let Some(color) = attributes.color {
            state.colors.push((
                color.r().to_bits(),
                color.g().to_bits(),
                color.b().to_bits(),
            ));
        }
    };
    let mut solids_seen = std::collections::BTreeSet::new();
    let mut faces_seen = std::collections::BTreeSet::new();
    for &solid in solids {
        if !solids_seen.insert(solid.index()) {
            continue;
        }
        if let Some(attributes) = topo.attributes().solid(solid) {
            collect(attributes);
        }
        for face in explorer::solid_faces(topo, solid)
            .unwrap_or_else(|error| panic!("live solid has no faces: {error}"))
        {
            if faces_seen.insert(face.index())
                && let Some(attributes) = topo.attributes().face(face)
            {
                collect(attributes);
            }
        }
    }
    state.names.sort();
    state.colors.sort_unstable();
    state
}

fn make_attribute(name: Option<String>, color: (f64, f64, f64)) -> EntityAttributes {
    EntityAttributes {
        name,
        color: Some(
            ColorRgb::new(color.0, color.1, color.2)
                .unwrap_or_else(|error| panic!("generated color is in range: {error}")),
        ),
    }
}

/// The members of a compound as a repetition pattern: `true` where the
/// member appeared earlier in the same compound.
fn alias_pattern(members: &[SolidId]) -> Vec<bool> {
    members
        .iter()
        .enumerate()
        .map(|(i, member)| members[..i].contains(member))
        .collect()
}

/// Check every oracle that applies to a successfully round-tripped document.
#[allow(clippy::too_many_arguments)]
fn check_round_trip(
    source: &Topology,
    roots: &[SolidId],
    compounds: &[CompoundId],
    root_known: &[usize],
    compound_known: &[Vec<usize>],
    known: &[Known],
    duplicate_root: bool,
    aliased_shell: bool,
    bytes: &[u8],
) {
    let mut restored = Topology::new();
    let document = deserialize_document_with_limits(bytes, &mut restored, common::limits())
        .unwrap_or_else(|error| panic!("a valid generated document was refused: {error}"));

    // Root structure is preserved: same number of roots, and duplicate
    // roots alias to the same fresh handle.
    assert_eq!(
        document.solids.len(),
        roots.len(),
        "root count changed across the round-trip"
    );
    if duplicate_root {
        assert!(
            document.solids.len() >= 2 && document.solids[0] == document.solids[1],
            "duplicate roots lost their identity: {:?}",
            document.solids,
        );
    }
    if aliased_shell {
        let first = restored
            .solid(document.solids[0])
            .unwrap_or_else(|error| panic!("restored root resolves: {error}"))
            .outer_shell();
        let second = restored
            .solid(document.solids[1])
            .unwrap_or_else(|error| panic!("restored root resolves: {error}"))
            .outer_shell();
        assert_ne!(
            document.solids[0], document.solids[1],
            "the aliased solids must stay distinct roots"
        );
        assert_eq!(
            first, second,
            "the shared-shell alias did not survive the round-trip"
        );
    }

    // Compound roots replay with their repeated/aliased member pattern.
    assert_eq!(
        document.compounds.len(),
        compounds.len(),
        "compound root count changed across the round-trip"
    );
    for (&source_compound, &restored_compound) in compounds.iter().zip(&document.compounds) {
        let source_members = source
            .compound(source_compound)
            .unwrap_or_else(|error| panic!("source compound resolves: {error}"))
            .solids()
            .to_vec();
        let restored_members = restored
            .compound(restored_compound)
            .unwrap_or_else(|error| panic!("restored compound resolves: {error}"))
            .solids()
            .to_vec();
        assert_eq!(
            alias_pattern(&source_members),
            alias_pattern(&restored_members),
            "compound member alias pattern changed across the round-trip"
        );
    }

    // Every restored root and member validates, is a closed manifold, keeps
    // its census, and measures the closed-form volume of the body built at
    // its position — a position swap of equal-census bodies is caught here.
    let check_solid = |topo: &Topology, solid: SolidId, expectation: &Known| {
        let report = validate_solid(topo, solid)
            .unwrap_or_else(|error| panic!("restored solid failed to validate: {error}"));
        assert!(
            report.is_valid(),
            "comprehensive validation fails on a restored solid: {report:?}"
        );
        let census = inv::census(topo, solid)
            .unwrap_or_else(|error| panic!("census failed on a restored solid: {error}"));
        inv::assert_closed_manifold("arena_roundtrip", &census);
        assert_eq!(
            (census.faces, census.edges, census.vertices),
            (expectation.faces, expectation.edges, expectation.vertices),
            "round-trip changed the topology census"
        );
        let measured = solid_volume(topo, solid, inv::volume_deflection(expectation.diag))
            .unwrap_or_else(|error| panic!("restored solid lost its volume: {error}"));
        inv::assert_exact_volume("arena_roundtrip", expectation.volume, measured);
    };
    for (i, &solid) in document.solids.iter().enumerate() {
        check_solid(&restored, solid, &known[root_known[i]]);
    }
    for (j, &compound) in document.compounds.iter().enumerate() {
        let members = restored
            .compound(compound)
            .unwrap_or_else(|error| panic!("restored compound resolves: {error}"))
            .solids()
            .to_vec();
        for (k, member) in members.iter().enumerate() {
            check_solid(&restored, *member, &known[compound_known[j][k]]);
        }
    }

    // Tolerances and trims survive bit-exactly (the oracle the
    // `float_roundtrip` feature exists to satisfy).
    let source_captured = captured_source_solids(source, roots, compounds);
    let restored_captured = captured_restored_solids(&restored, &document);
    assert_eq!(
        precision_state(source, &source_captured),
        precision_state(&restored, &restored_captured),
        "tolerance or trim bits changed across the round-trip"
    );

    // Attributes survive: names and colors as value multisets over the
    // captured entities.
    assert_eq!(
        attribute_state(source, &source_captured),
        attribute_state(&restored, &restored_captured),
        "attributes changed across the round-trip"
    );

    // The strongest exactness statement: re-serializing the restored
    // document reproduces the input bytes.
    let rewritten = serialize_document(&restored, &document.solids, &document.compounds)
        .unwrap_or_else(|error| panic!("restored document failed to re-serialize: {error}"));
    assert_eq!(
        rewritten, bytes,
        "serialize → deserialize → serialize is not byte-identical"
    );
}

fn captured_source_solids(
    topo: &Topology,
    roots: &[SolidId],
    compounds: &[CompoundId],
) -> Vec<SolidId> {
    let mut out = roots.to_vec();
    for &compound in compounds {
        out.extend(
            topo.compound(compound)
                .unwrap_or_else(|error| panic!("source compound resolves: {error}"))
                .solids()
                .iter()
                .copied(),
        );
    }
    out
}

fn captured_restored_solids(topo: &Topology, document: &DeserializedDocument) -> Vec<SolidId> {
    let mut out = document.solids.clone();
    for &compound in &document.compounds {
        out.extend(
            topo.compound(compound)
                .unwrap_or_else(|error| panic!("restored compound resolves: {error}"))
                .solids()
                .iter()
                .copied(),
        );
    }
    out
}

/// A deliberately corrupted document must be refused with a typed error and
/// must leave a pre-populated destination topology untouched.
fn check_corruption(source_bytes: &[u8], mode: u8, salt: u8) {
    let mut value: serde_json::Value = serde_json::from_slice(source_bytes)
        .unwrap_or_else(|error| panic!("generated document parses as JSON: {error}"));
    let solids_len = value["solids"].as_array().map_or(0, Vec::len);
    let wires_len = value["wires"].as_array().map_or(0, Vec::len);
    let bogus_solid = solids_len + 1 + usize::from(salt % 3);
    let bogus_wire = wires_len + 1 + usize::from(salt % 3);
    match mode % 4 {
        0 => value["solid_roots"] = serde_json::json!([bogus_solid]),
        1 => {
            if value["compounds"].as_array().is_some_and(|c| !c.is_empty()) {
                value["compounds"][0]["solids"] = serde_json::json!([bogus_solid]);
            } else {
                value["solid_roots"] = serde_json::json!([bogus_solid]);
            }
        }
        2 => value["faces"][0]["outer_wire"] = serde_json::json!(bogus_wire),
        _ => value["version"] = serde_json::json!(99),
    }
    let corrupted = serde_json::to_vec(&value)
        .unwrap_or_else(|error| panic!("corrupted document re-serializes: {error}"));

    let mut destination = Topology::new();
    let guard = make_box(&mut destination, 1.0, 2.0, 3.0)
        .unwrap_or_else(|error| panic!("guard box builds: {error}"));
    let before = (
        destination.num_vertices(),
        destination.num_edges(),
        destination.num_wires(),
        destination.num_faces(),
        destination.num_shells(),
        destination.num_solids(),
        destination.num_compounds(),
        destination.allocated_slot_count(),
    );

    let error = deserialize_document_with_limits(&corrupted, &mut destination, common::limits())
        .err()
        .unwrap_or_else(|| panic!("a corrupted document was accepted"));
    assert!(
        matches!(
            error,
            IoError::ParseError { .. } | IoError::LimitExceeded { .. }
        ),
        "corruption produced a non-typed error: {error}"
    );

    let after = (
        destination.num_vertices(),
        destination.num_edges(),
        destination.num_wires(),
        destination.num_faces(),
        destination.num_shells(),
        destination.num_solids(),
        destination.num_compounds(),
        destination.allocated_slot_count(),
    );
    assert_eq!(
        before, after,
        "a rejected deserialization mutated or leaked into the destination topology"
    );
    let report = validate_solid(&destination, guard)
        .unwrap_or_else(|error| panic!("guard solid failed to validate: {error}"));
    assert!(
        report.is_valid(),
        "guard solid damaged by a rejected deserialization"
    );
    let measured = solid_volume(&destination, guard, 1.0e-6)
        .unwrap_or_else(|error| panic!("guard solid lost its volume: {error}"));
    inv::assert_exact_volume("arena_roundtrip guard", 6.0, measured);
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }
    let mut bytes = Bytes::new(data);
    let scale = [1.0e-3, 1.0, 1.0e3][usize::from(bytes.next()) % 3];
    let shape = bytes.next() % 3;

    let mut topo = Topology::new();
    let mut known: Vec<Known> = Vec::new();

    // Primary body: box or cylinder.
    let (a, primary) = if shape == 1 {
        build_cylinder(&mut topo, &mut bytes, scale)
    } else {
        build_box(&mut topo, &mut bytes, scale)
    };
    known.push(primary);

    // Optional second body: the other primitive, or a second box.
    let mut second: Option<SolidId> = None;
    if bytes.next() % 4 != 0 {
        let (id, extra) = if shape == 2 {
            build_cylinder(&mut topo, &mut bytes, scale)
        } else {
            build_box(&mut topo, &mut bytes, scale)
        };
        second = Some(id);
        known.push(extra);
    }
    let known_index_of = |id: SolidId| -> usize {
        // The primary is known[0]; the second body is known[1]; an alias
        // shares the primary's shell and therefore its geometry.
        if Some(id) == second { 1 } else { 0 }
    };

    // Root structure: plain, duplicated, two-body, or a shared-shell alias.
    let mut roots: Vec<SolidId> = Vec::new();
    let mut root_known: Vec<usize> = Vec::new();
    let mut duplicate_root = false;
    let mut aliased_shell = false;
    match bytes.next() % 4 {
        0 => {
            roots.push(a);
            root_known.push(0);
        }
        1 => {
            duplicate_root = true;
            roots.push(a);
            roots.push(a);
            root_known.push(0);
            root_known.push(0);
        }
        2 => {
            roots.push(a);
            root_known.push(0);
            if let Some(b) = second {
                roots.push(b);
                root_known.push(1);
            }
        }
        _ => {
            let shell = topo
                .solid(a)
                .unwrap_or_else(|error| panic!("primary resolves: {error}"))
                .outer_shell();
            let alias = topo.add_solid(Solid::new(shell, Vec::new()));
            aliased_shell = true;
            roots.push(a);
            root_known.push(0);
            roots.push(alias);
            root_known.push(known_index_of(a));
        }
    }

    // Compound structure: none, plain, or with repeated/aliased members.
    let mut compounds: Vec<CompoundId> = Vec::new();
    let mut compound_known: Vec<Vec<usize>> = Vec::new();
    match bytes.next() % 4 {
        1 => {
            compounds.push(topo.add_compound(Compound::new(vec![a])));
            compound_known.push(vec![0]);
        }
        2 => {
            let mut members = vec![a, a];
            let mut member_known = vec![0, 0];
            if let Some(b) = second {
                members.push(b);
                member_known.push(1);
            }
            compounds.push(topo.add_compound(Compound::new(members)));
            compound_known.push(member_known);
        }
        3 => {
            let mut members = vec![a];
            let mut member_known = vec![0];
            if let Some(b) = second {
                members.push(b);
                members.push(a);
                member_known.push(1);
                member_known.push(0);
            }
            compounds.push(topo.add_compound(Compound::new(members)));
            compound_known.push(member_known);
        }
        _ => {}
    }

    let captured: Vec<SolidId> = {
        let mut all = roots.clone();
        for &compound in &compounds {
            all.extend(
                topo.compound(compound)
                    .unwrap_or_else(|error| panic!("compound resolves: {error}"))
                    .solids()
                    .iter()
                    .copied(),
            );
        }
        all
    };

    // Precision-hostile tolerances on every vertex and up to two edges.
    if bytes.next() & 1 == 1 {
        let mut vertices_seen = std::collections::BTreeSet::new();
        let mut edges_seen = std::collections::BTreeSet::new();
        let mut edges_raised = 0;
        for &solid in &captured {
            for face in explorer::solid_faces(&topo, solid)
                .unwrap_or_else(|error| panic!("live solid has no faces: {error}"))
            {
                for vertex in explorer::face_vertices(&topo, face)
                    .unwrap_or_else(|error| panic!("live face has no vertices: {error}"))
                {
                    if vertices_seen.insert(vertex.index()) {
                        let tolerance = bytes.tolerance();
                        topo.vertex_mut(vertex)
                            .unwrap_or_else(|error| panic!("vertex resolves: {error}"))
                            .set_tolerance(tolerance)
                            .unwrap_or_else(|error| {
                                panic!("generated tolerance is valid: {error}")
                            });
                    }
                }
                for edge in explorer::face_edges(&topo, face)
                    .unwrap_or_else(|error| panic!("live face has no edges: {error}"))
                {
                    if edges_raised < 2 && edges_seen.insert(edge.index()) {
                        let tolerance = bytes.tolerance();
                        topo.edge_mut(edge)
                            .unwrap_or_else(|error| panic!("edge resolves: {error}"))
                            .set_tolerance(Some(tolerance))
                            .unwrap_or_else(|error| {
                                panic!("generated tolerance is valid: {error}")
                            });
                        edges_raised += 1;
                    }
                }
            }
        }
    }

    // Public attributes on the primary solid and one of its faces.
    match bytes.next() % 4 {
        1 => {
            let name = if bytes.next() & 1 == 0 {
                "fuzz-body".to_owned()
            } else {
                "fuzz-body-π".to_owned()
            };
            let color = (bytes.unit(), bytes.unit(), bytes.unit());
            topo.set_solid_attributes(a, make_attribute(Some(name), color))
                .unwrap_or_else(|error| panic!("solid is live: {error}"));
        }
        2 => {
            let color = (bytes.unit(), bytes.unit(), bytes.unit());
            let face = explorer::solid_faces(&topo, a)
                .unwrap_or_else(|error| panic!("live solid has no faces: {error}"))[0];
            topo.set_face_attributes(face, make_attribute(Some("fuzz-face".to_owned()), color))
                .unwrap_or_else(|error| panic!("face is live: {error}"));
        }
        3 => {
            let color = (bytes.unit(), bytes.unit(), bytes.unit());
            topo.set_solid_attributes(a, make_attribute(Some("fuzz-root".to_owned()), color))
                .unwrap_or_else(|error| panic!("solid is live: {error}"));
            if let Some(b) = second {
                topo.set_solid_attributes(b, make_attribute(Some("fuzz-second".to_owned()), color))
                    .unwrap_or_else(|error| panic!("second solid is live: {error}"));
            }
        }
        _ => {}
    }

    let serialized = serialize_document(&topo, &roots, &compounds)
        .unwrap_or_else(|error| panic!("generated document failed to serialize: {error}"));

    let mode = bytes.next();
    if mode % 8 >= 4 {
        check_corruption(&serialized, mode, bytes.next());
        return;
    }

    check_round_trip(
        &topo,
        &roots,
        &compounds,
        &root_known,
        &compound_known,
        &known,
        duplicate_root,
        aliased_shell,
        &serialized,
    );
});
