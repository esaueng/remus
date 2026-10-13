//! Differential oracle for the hand-written arena codec ([`super::codec`]).
//!
//! The codec replaced `serde_json` plus the derived impls of the `Ser*`
//! mirror types. That reader (with its journal preflight) is kept verbatim
//! at the end of this file, and the derived `Serialize` impls survive under
//! `cfg(test)`, so every test here compares the two implementations: bytes
//! written for generated topologies, fixtures and STEP imports, and the
//! outcome of reading the fixture corpus, hand-written edge cases, and
//! randomly mutated documents — the parsed document, or the [`IoError`] kind
//! (with the exact resource, limit and count for limit refusals).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::{Path, PathBuf};

use proptest::prelude::*;
use remus_algo::bop::BooleanOp;
use remus_operations::journal_ops::{boolean_journaled, fillet_journaled};
use remus_operations::primitives::{make_box, make_cone, make_cylinder, make_sphere, make_torus};
use remus_operations::sew::make_sheet_body;
use remus_topology::attributes::{ColorRgb, EntityAttributes};
use remus_topology::explorer::{solid_edges, solid_faces};
use serde::de::{DeserializeSeed, IgnoredAny, MapAccess, SeqAccess, Visitor};

use super::*;
use crate::json::mutation::{apply, mutation};

// ===========================================================================
// Comparison helpers
// ===========================================================================

/// The observable result of a read: the parsed document, or the error kind
/// (limit refusals with their exact payload; parse errors carry `serde_json`
/// messages, which are not part of the contract).
fn outcome(result: Result<ParsedDocument, IoError>) -> String {
    match result {
        Ok(document) => format!("{document:?}"),
        Err(IoError::LimitExceeded {
            resource,
            limit,
            actual,
        }) => format!("limit exceeded: {resource} {limit} {actual}"),
        Err(IoError::ParseError { .. }) => "parse error".to_owned(),
        Err(other) => format!("other error: {other}"),
    }
}

fn first_difference(a: &str, b: &str) -> String {
    let index = a
        .bytes()
        .zip(b.bytes())
        .position(|(x, y)| x != y)
        .unwrap_or_else(|| a.len().min(b.len()));
    let window = |text: &str| {
        let start = index.saturating_sub(80);
        let end = (index + 80).min(text.len());
        text.get(start..end)
            .unwrap_or("<non-char boundary>")
            .to_owned()
    };
    format!(
        "first difference at {index}:\n  codec:  {}\n  oracle: {}",
        window(a),
        window(b)
    )
}

fn preview(bytes: &[u8]) -> String {
    String::from_utf8_lossy(&bytes[..bytes.len().min(600)]).into_owned()
}

/// Reads `bytes` with both readers and asserts identical outcomes.
fn assert_same_read(bytes: &[u8], limits: ImportLimits) {
    let expected = outcome(oracle_parse_document(bytes, limits));
    let actual = outcome(parse_document(bytes, limits));
    assert!(
        actual == expected,
        "codec and oracle disagree for input {}\n{}",
        preview(bytes),
        first_difference(&actual, &expected)
    );
}

/// The old writer: `serde_json` over the derived view, streamed into the
/// byte budget when one applies.
fn oracle_encode<T: Serialize>(view: &T, limit: Option<usize>) -> Result<Vec<u8>, IoError> {
    match limit {
        Some(limit) => serialize_json_with_limit(view, limit),
        None => serde_json::to_vec(view).map_err(|e| IoError::ParseError {
            reason: format!("arena serialization failed: {e}"),
        }),
    }
}

fn written(result: Result<Vec<u8>, IoError>) -> String {
    match result {
        Ok(bytes) => String::from_utf8(bytes).unwrap(),
        Err(IoError::LimitExceeded {
            resource,
            limit,
            actual,
        }) => format!("limit exceeded: {resource} {limit} {actual}"),
        Err(other) => format!("error: {other}"),
    }
}

fn assert_same_bytes(actual: Result<Vec<u8>, IoError>, expected: Result<Vec<u8>, IoError>) {
    let (actual, expected) = (written(actual), written(expected));
    assert!(
        actual == expected,
        "{}",
        first_difference(&actual, &expected)
    );
}

/// Writes solid and compound roots with both writers, with no budget and at
/// the byte budget's boundaries, and returns the unbudgeted document.
fn assert_same_v3(topo: &Topology, solids: &[SolidId], compounds: &[CompoundId]) -> Vec<u8> {
    let expected = oracle_encode(
        &document_v3_view(topo, solids, compounds, None).unwrap(),
        None,
    );
    let actual = serialize_document(topo, solids, compounds);
    assert_same_bytes(actual, expected);
    let bytes = serialize_document(topo, solids, compounds).unwrap();
    for limit in [bytes.len(), bytes.len() - 1, bytes.len() / 2, 0] {
        let limits = ImportLimits {
            max_input_bytes: limit,
            ..ImportLimits::default()
        };
        let expected = document_v3_view(topo, solids, compounds, Some(limits))
            .and_then(|view| oracle_encode(&view, Some(limit)));
        let actual = serialize_document_impl(topo, solids, compounds, Some(limits));
        assert_same_bytes(actual, expected);
    }
    bytes
}

/// [`assert_same_v3`] for solid, sheet and compound roots (version 4).
fn assert_same_v4(
    topo: &Topology,
    solids: &[SolidId],
    sheets: &[ShellId],
    compounds: &[CompoundId],
) -> Vec<u8> {
    let expected = oracle_encode(
        &document_v4_view(topo, solids, sheets, compounds, None).unwrap(),
        None,
    );
    assert_same_bytes(
        serialize_body_document(topo, solids, sheets, compounds),
        expected,
    );
    let bytes = serialize_body_document(topo, solids, sheets, compounds).unwrap();
    for limit in [bytes.len(), bytes.len() - 1, 1] {
        let limits = ImportLimits {
            max_input_bytes: limit,
            ..ImportLimits::default()
        };
        let expected = document_v4_view(topo, solids, sheets, compounds, Some(limits))
            .and_then(|view| oracle_encode(&view, Some(limit)));
        let actual = serialize_body_document_impl(topo, solids, sheets, compounds, Some(limits));
        assert_same_bytes(actual, expected);
    }
    bytes
}

/// [`assert_same_v3`] for every root class (version 5, never budgeted).
fn assert_same_v5(
    topo: &Topology,
    solids: &[SolidId],
    sheets: &[ShellId],
    wires: &[WireId],
    compounds: &[CompoundId],
) -> Vec<u8> {
    let expected = oracle_encode(
        &document_v5_view(topo, solids, sheets, wires, compounds).unwrap(),
        None,
    );
    let actual = serialize_body_document_with_wires(topo, solids, sheets, wires, compounds);
    assert_same_bytes(actual, expected);
    serialize_body_document_with_wires(topo, solids, sheets, wires, compounds).unwrap()
}

/// Asserts both readers agree on `bytes` under default limits and under a
/// spread of budgets around the document's own size and entity count.
fn assert_same_reads(bytes: &[u8]) {
    assert_same_read(bytes, ImportLimits::default());
    let entities = [0, 1, 2, 3, 5, 8, 13, 40, 100, 1000];
    for max_model_entities in entities {
        assert_same_read(
            bytes,
            ImportLimits {
                max_model_entities,
                ..ImportLimits::default()
            },
        );
    }
    for max_input_bytes in [bytes.len(), bytes.len().saturating_sub(1)] {
        assert_same_read(
            bytes,
            ImportLimits {
                max_input_bytes,
                ..ImportLimits::default()
            },
        );
    }
}

// ===========================================================================
// Corpus
// ===========================================================================

fn data_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

fn fixture_documents() -> Vec<(String, Vec<u8>)> {
    let mut documents: Vec<(String, Vec<u8>)> = std::fs::read_dir(data_dir())
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| {
            matches!(
                path.extension().and_then(|extension| extension.to_str()),
                Some("bin" | "json")
            )
        })
        .map(|path| {
            (
                path.file_name().unwrap().to_string_lossy().into_owned(),
                std::fs::read(&path).unwrap(),
            )
        })
        .collect();
    documents.sort();
    documents
}

fn nurbs_sheet(topo: &mut Topology) -> ShellId {
    let surface = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            vec![
                Point3::new(-0.75, -0.25, 0.0),
                Point3::new(0.75, -0.25, 0.1),
            ],
            vec![
                Point3::new(-0.75, 0.0, 0.3),
                Point3::new(0.75, 0.0, 1.0 / 3.0),
            ],
            vec![Point3::new(-0.75, 0.25, 0.0), Point3::new(0.75, 0.25, -0.2)],
        ],
        vec![vec![1.0, 1.0], vec![0.7, 0.7], vec![1.0, 1.0]],
    )
    .unwrap();
    let face = remus_topology::builder::make_nurbs_face(topo, surface, 3.25e-8).unwrap();
    make_sheet_body(topo, &[face]).unwrap()
}

/// Names that exercise every `serde_json` string escape.
const TRICKY_NAMES: &[&str] = &[
    "plain",
    "quote \" and backslash \\",
    "controls \u{0}\u{1}\u{8}\u{9}\u{a}\u{b}\u{c}\u{d}\u{1f}\u{7f}",
    "unicode é 中 😀 \u{2028}",
    "slash / solidus",
    "",
];

/// Documents written by the codec for a spread of generated topologies,
/// each checked byte for byte against the old writer on the way.
fn generated_documents() -> Vec<(String, Vec<u8>)> {
    let mut documents = Vec::new();

    let mut topo = Topology::new();
    let primitives = [
        make_box(&mut topo, 10.0, 20.0, 30.0).unwrap(),
        make_cylinder(&mut topo, 1.5, 4.0).unwrap(),
        make_cone(&mut topo, 2.0, 0.5, 3.0).unwrap(),
        make_cone(&mut topo, 2.0, 0.0, 3.0).unwrap(),
        make_sphere(&mut topo, 1.25, 16).unwrap(),
        make_torus(&mut topo, 3.0, 0.75, 16).unwrap(),
    ];
    for (index, &solid) in primitives.iter().enumerate() {
        documents.push((
            format!("primitive {index}"),
            assert_same_v3(&topo, &[solid], &[]),
        ));
    }
    let empty = topo.add_empty_solid();
    let compound = topo.add_compound(Compound::new(vec![primitives[0], primitives[4], empty]));
    documents.push((
        "compound".to_owned(),
        assert_same_v3(&topo, &[primitives[1], primitives[1]], &[compound]),
    ));

    // Attributes with every escape, a cavity shell, and shared topology.
    let mut attributed = Topology::new();
    let outer = make_box(&mut attributed, 4.0, 4.0, 4.0).unwrap();
    let shell = attributed.solid(outer).unwrap().outer_shell();
    let cavity = attributed.add_solid(Solid::new(shell, vec![shell]));
    for (index, name) in TRICKY_NAMES.iter().enumerate() {
        let face = solid_faces(&attributed, outer).unwrap()[index];
        attributed
            .set_face_attributes(
                face,
                EntityAttributes {
                    name: Some((*name).to_owned()),
                    color: (index % 2 == 0)
                        .then(|| ColorRgb::new(0.1 * index as f64, 1.0 / 3.0, 0.0).unwrap()),
                },
            )
            .unwrap();
    }
    attributed
        .set_solid_attributes(
            cavity,
            EntityAttributes {
                name: None,
                color: Some(ColorRgb::new(0.2, 0.4, 0.6).unwrap()),
            },
        )
        .unwrap();
    documents.push((
        "attributes".to_owned(),
        assert_same_v3(&attributed, &[cavity, outer], &[]),
    ));

    // Journaled booleans and a journaled fillet.
    let mut journaled = Topology::new();
    let block = make_box(&mut journaled, 4.0, 3.0, 2.0).unwrap();
    let tool = make_cylinder(&mut journaled, 0.5, 5.0).unwrap();
    let cut = boolean_journaled(&mut journaled, BooleanOp::Cut, block, tool).unwrap();
    let other = make_box(&mut journaled, 1.0, 1.0, 1.0).unwrap();
    let fused = boolean_journaled(&mut journaled, BooleanOp::Fuse, cut.solid, other).unwrap();
    documents.push((
        "journaled booleans".to_owned(),
        assert_same_v3(&journaled, &[fused.solid], &[]),
    ));
    let mut blended = Topology::new();
    let block = make_box(&mut blended, 4.0, 3.0, 2.0).unwrap();
    let edges: Vec<_> = solid_edges(&blended, block)
        .unwrap()
        .into_iter()
        .take(2)
        .collect();
    if let Ok(fillet) = fillet_journaled(&mut blended, block, &edges, 0.25) {
        documents.push((
            "journaled fillet".to_owned(),
            assert_same_v3(&blended, &[fillet.result.solid], &[]),
        ));
    }

    // Sheets, wires and mixed version 4 and 5 documents.
    let mut bodies = Topology::new();
    let sheet = nurbs_sheet(&mut bodies);
    let wire = remus_topology::builder::make_polygon_wire(
        &mut bodies,
        &[
            Point3::new(-1.25, -0.75, 0.0),
            Point3::new(1.25, -0.75, 0.0),
            Point3::new(1.25, 0.75, 0.1),
        ],
        3.25e-8,
    )
    .unwrap();
    let solid = make_sphere(&mut bodies, 0.5, 8).unwrap();
    let compound = bodies.add_compound(Compound::new(vec![solid]));
    documents.push((
        "sheet".to_owned(),
        assert_same_v4(&bodies, &[], &[sheet, sheet], &[]),
    ));
    documents.push((
        "mixed v4".to_owned(),
        assert_same_v4(&bodies, &[solid], &[sheet], &[compound]),
    ));
    documents.push((
        "wire".to_owned(),
        assert_same_v5(&bodies, &[], &[], &[wire, wire], &[]),
    ));
    documents.push((
        "mixed v5".to_owned(),
        assert_same_v5(&bodies, &[solid], &[sheet], &[wire], &[compound]),
    ));

    // A many-solid compound.
    let mut many = Topology::new();
    let boxes: Vec<_> = (0..40)
        .map(|index| make_box(&mut many, 1.0 + index as f64 * 0.125, 2.0, 3.0).unwrap())
        .collect();
    let compound = many.add_compound(Compound::new(boxes.clone()));
    documents.push((
        "40-box compound".to_owned(),
        assert_same_v3(&many, &boxes[..3], &[compound]),
    ));
    documents
}

/// Documents for every STEP fixture that imports, checked byte for byte.
fn step_documents() -> Vec<(String, Vec<u8>)> {
    let mut paths: Vec<_> = std::fs::read_dir(data_dir())
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension().and_then(|e| e.to_str()) == Some("step"))
        .collect();
    paths.sort();
    let mut documents = Vec::new();
    for path in paths {
        let text = std::fs::read_to_string(&path).unwrap();
        let mut topo = Topology::new();
        let Ok(solids) = crate::step::reader::read_step(&text, &mut topo) else {
            continue;
        };
        if solids.is_empty() {
            continue;
        }
        let name = path.file_name().unwrap().to_string_lossy().into_owned();
        documents.push((name, assert_same_v3(&topo, &solids, &[])));
    }
    documents
}

// ===========================================================================
// Tests over the corpus
// ===========================================================================

#[test]
fn generated_documents_write_identically_and_read_identically() {
    for (name, bytes) in generated_documents() {
        assert!(!bytes.is_empty(), "{name}");
        assert_same_reads(&bytes);
        // Re-reading and re-writing is stable.
        let mut restored = Topology::new();
        let roots = deserialize_document(&bytes, &mut restored).unwrap();
        if bytes.starts_with(br#"{"version":3"#) {
            assert_eq!(
                serialize_document(&restored, &roots.solids, &roots.compounds).unwrap(),
                bytes,
                "{name}"
            );
        }
    }
}

#[test]
fn step_fixture_documents_write_identically_and_read_identically() {
    let documents = step_documents();
    assert!(
        documents.len() >= 10,
        "only {} STEP fixtures imported",
        documents.len()
    );
    for (_, bytes) in documents {
        assert_same_read(&bytes, ImportLimits::default());
    }
}

#[test]
fn fixture_corpus_reads_identically() {
    let documents = fixture_documents();
    assert!(documents.len() >= 100, "{}", documents.len());
    for (_, bytes) in &documents {
        assert_same_read(bytes, ImportLimits::default());
    }
    // Budgets against the smaller fixtures (the large ones only repeat work).
    for (_, bytes) in documents.iter().filter(|(_, bytes)| bytes.len() < 200_000) {
        assert_same_reads(bytes);
    }
}

#[test]
fn fixture_corpus_rewrites_byte_identically() {
    for (name, bytes) in fixture_documents() {
        let mut topo = Topology::new();
        let Ok(roots) = deserialize_document(&bytes, &mut topo) else {
            continue;
        };
        if !roots.sheets.is_empty() || !roots.wires.is_empty() {
            assert_same_v5(
                &topo,
                &roots.solids,
                &roots.sheets,
                &roots.wires,
                &roots.compounds,
            );
            continue;
        }
        let rewritten = assert_same_v3(&topo, &roots.solids, &roots.compounds);
        assert_same_read(&rewritten, ImportLimits::default());
        assert!(!rewritten.is_empty(), "{name}");
    }
}

// ===========================================================================
// Hand-written edge cases
// ===========================================================================

/// A minimal valid version 2 document whose `"pcurves":[]` is followed by
/// `extra` (a comma-led list of more top-level fields).
fn v2(extra: &str) -> String {
    format!(
        r#"{{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]{extra}}}"#
    )
}

fn v2_journal(entries: &str) -> String {
    v2(&format!(
        r#","journal":{{"next_op":9,"next_ordinal":3,"index":[{{"ordinal":0,"kind":"face","local":null}}],"entries":[{entries}]}}"#
    ))
}

fn nested(open: &str, close: &str, depth: usize, inner: &str) -> String {
    format!("{}{inner}{}", open.repeat(depth), close.repeat(depth))
}

/// One-edge, one-vertex version 2 document with `edge` and `vertex` spliced
/// in, to exercise geometry and struct forms.
fn v2_edge(vertex: &str, edge: &str) -> String {
    format!(
        r#"{{"version":2,"vertices":[{vertex}],"edges":[{edge}],"wires":[],"faces":[],"shells":[{{"faces":[]}}],"solids":[{{"outer_shell":0,"inner_shells":[]}}],"solid_roots":[0],"compounds":[],"pcurves":[]}}"#
    )
}

fn edge_cases() -> Vec<String> {
    let vertex = r#"{"point":[0,0,0],"tolerance":1e-9}"#;
    let circle =
        r#"{"center":[0,0,0],"normal":[0,0,1],"radius":1,"u_axis":[1,0,0],"v_axis":[0,1,0]}"#;
    let mut cases: Vec<String> = vec![
        // Roots, headers and trailing data.
        String::new(),
        " ".to_owned(),
        "null".to_owned(),
        "[]".to_owned(),
        "[1]".to_owned(),
        "[2]".to_owned(),
        "[1,[],[],[],[],[],0,[],[]]".to_owned(),
        "{}".to_owned(),
        r#"{"version":1}"#.to_owned(),
        r#"{"version":6}"#.to_owned(),
        r#"{"version":0}"#.to_owned(),
        r#"{"version":-1}"#.to_owned(),
        r#"{"version":2.0}"#.to_owned(),
        r#"{"version":"2"}"#.to_owned(),
        r#"{"version":4294967298}"#.to_owned(),
        r#"{"version":null}"#.to_owned(),
        r#"{"version":2,"version":2}"#.to_owned(),
        v2(""),
        format!("{} ", v2("")),
        format!("\t\r\n{}\n", v2("")),
        format!("{}x", v2("")),
        format!("{}{{}}", v2("")),
        v2(r#","journal":null,"attributes":null"#),
        v2(r#","unknown":[1,{"a":"\ud800"}],"other":"\u0000""#),
        v2(r#","vers\u0069on":2"#),
        v2(r#","journal":null,"journal":null"#),
        // Top-level keys must be valid UTF-8 even when unknown.
        r#"{"version":2,"\ud800":1}"#.to_owned(),
        r#"{"version":2,"\ud800\udc00":1}"#.to_owned(),
        // Struct forms.
        r#"{"version":2,"vertices":[[[1,2,3],0.5]],"edges":[],"wires":[],"faces":[],"shells":[[[]]],"solids":[[0,[]]],"solid_roots":[0],"compounds":[[[0]]],"pcurves":[]}"#.to_owned(),
        r#"{"version":2,"vertices":[[[1,2,3]]],"edges":[],"wires":[],"faces":[],"shells":[],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        r#"{"version":2,"vertices":[[[1,2,3],0.5,1]],"edges":[],"wires":[],"faces":[],"shells":[],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        r#"{"version":2,"vertices":[{"point":[1,2,3],"tolerance":0.5,"point":[1,2,3]}],"edges":[],"wires":[],"faces":[],"shells":[],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        "[2,[],[],[],[],[],[],[],[],[]]".to_owned(),
        // Shell body classes in every form.
        v2(r#","shells":[{"faces":[],"body_class":"sheet"}]"#),
        r#"{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[],"body_class":{"sheet":null}},[[],"general"],[[]],{"faces":[],"body_class":null}],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        r#"{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[],"body_class":{"sheet":1}}],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        r#"{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[],"body_class":"Sheet"}],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        r#"{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[],"body_class":{}}],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        r#"{"version":2,"vertices":[],"edges":[],"wires":[],"faces":[],"shells":[{"faces":[],"body_class":{"sheet":null,"wire":null}}],"solids":[],"solid_roots":[],"compounds":[],"pcurves":[]}"#.to_owned(),
        // Edge curves in string, object and positional forms.
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Line","tolerance":null}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Line":null}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Line":0}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Circle"}"#),
        v2_edge(vertex, &format!(r#"{{"start":0,"end":0,"curve":{{"Circle":{circle}}},"trim":[0,6.283185307179586]}}"#)),
        v2_edge(vertex, &format!(r#"[0,0,{{"Circle":{circle}}},1e-7,[0,1]]"#)),
        v2_edge(vertex, &format!(r#"[0,0,{{"Circle":{circle}}},1e-7]"#)),
        v2_edge(vertex, &format!(r#"[0,0,{{"Circle":{circle}}}]"#)),
        v2_edge(vertex, &format!(r#"[0,0,{{"Circle":{circle}}},null,null,1]"#)),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Circle":[[0,0,0],[0,0,1],-2,[1,0,0],[0,1,0]]}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Circle":[[0,0,0],[0,0,1],-2,[1,0,0]]}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Circle":{"center":[0,0,0],"normal":[0,0,1],"radius":1,"u_axis":[1,0,0],"v_axis":[0,1,0],"radius":2}}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Circle":{"center":[0,0,0],"normal":[0,0,1],"radius":1,"u_axis":[1,0,0]}}}"#),
        v2_edge(vertex, &format!(r#"{{"start":0,"end":0,"curve":{{"Circle":{circle},"Line":null}}}}"#)),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"NurbsCurve":{"degree":1,"knots":[0,0,1,1],"control_points":[[0,0,0],[1,0,0]],"weights":[1,1]}}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"NurbsCurve":[1,[0,0,1,1],[[0,0,0],[1,0,0]],[1,1]]}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"NurbsCurve":[18446744073709551615,[],[],[]]}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Parabola":{"vertex":[0,0,0],"axis_dir":[1,0,0],"focal_length":0.5,"u_axis":[0,1,0]}}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Hyperbola":{"center":[0,0,0],"normal":[0,0,1],"semi_major":2,"semi_minor":1,"u_axis":[1,0,0],"v_axis":[0,1,0]}}}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":{"Ellipse":{"center":[0,0,0],"normal":[0,0,1],"semi_major":2,"semi_minor":1,"u_axis":[1,0,0],"v_axis":[0,1,0]}},"tolerance":-0}"#),
        // Numbers.
        v2_edge(r#"{"point":[-0,0.0,-0.0],"tolerance":1e-400}"#, r#"{"start":0,"end":0,"curve":"Line"}"#),
        v2_edge(r#"{"point":[1e400,0,0],"tolerance":1}"#, r#"{"start":0,"end":0,"curve":"Line"}"#),
        v2_edge(r#"{"point":[18446744073709551616,-9223372036854775809,123456789012345678901234567890e-30],"tolerance":1}"#, r#"{"start":0,"end":0,"curve":"Line"}"#),
        v2_edge(vertex, r#"{"start":-0,"end":0,"curve":"Line"}"#),
        v2_edge(vertex, r#"{"start":0.0,"end":0,"curve":"Line"}"#),
        v2_edge(vertex, r#"{"start":4294967296,"end":0,"curve":"Line"}"#),
        v2_edge(vertex, r#"{"start":01,"end":0,"curve":"Line"}"#),
        v2_edge(vertex, r#"{"start":1.,"end":0,"curve":"Line"}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Line","tolerance":1e99999999999}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Line","tolerance":0e99999999999}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Line","tolerance":1e-99999999999}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Line","tolerance":1.7976931348623157e308}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Line","tolerance":1.7976931348623159e308}"#),
        v2_edge(vertex, r#"{"start":0,"end":0,"curve":"Line","tolerance":2.4703282292062328e-324}"#),
        // Strings.
        v2_journal(r#"{"op":1,"kind":"a\"\\\/\b\f\n\r\t\u00e9\ud83d\ude00","payload":"GlobalBarrier"}"#),
        v2_journal(r#"{"op":1,"kind":"\ud83d","payload":"GlobalBarrier"}"#),
        v2_journal(r#"{"op":1,"kind":"\ude00","payload":"GlobalBarrier"}"#),
        v2_journal(r#"{"op":1,"kind":"\ud83d\u0041","payload":"GlobalBarrier"}"#),
        v2_journal(r#"{"op":1,"kind":"\ud83dx","payload":"GlobalBarrier"}"#),
        v2_journal(r#"{"op":1,"kind":"\u00g0","payload":"GlobalBarrier"}"#),
        v2_journal(r#"{"op":1,"kind":"\q","payload":"GlobalBarrier"}"#),
        v2_journal("{\"op\":1,\"kind\":\"tab\there\",\"payload\":\"GlobalBarrier\"}"),
    ];

    // Journal entries: flattened payload and internally tagged events.
    for entry in [
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[1,2],"events":[]}"#,
        r#"{"payload":"Evolution","events":[],"scope":[],"construction":false,"kind":"k","op":1}"#,
        r#"{"op":1,"kind":"k","payload":0,"construction":true,"scope":[],"events":[]}"#,
        r#"{"op":1,"kind":"k","payload":1,"affected":[3]}"#,
        r#"{"op":1,"kind":"k","payload":2}"#,
        r#"{"op":1,"kind":"k","payload":3}"#,
        r#"{"op":1,"kind":"k","payload":-1}"#,
        r#"{"op":1,"kind":"k","payload":1.0}"#,
        r#"{"op":1,"kind":"k","payload":null}"#,
        r#"{"op":1,"kind":"k","payload":"GlobalBarrier","payload":"GlobalBarrier"}"#,
        r#"{"op":1,"kind":"k"}"#,
        r#"{"op":1,"payload":"GlobalBarrier"}"#,
        r#"{"op":1,"op":1,"kind":"k","payload":"GlobalBarrier"}"#,
        r#"{"op":1,"kind":"k","kind":"k","payload":"GlobalBarrier"}"#,
        r#"[1,"k","GlobalBarrier"]"#,
        r#"{"op":-1,"kind":"k","payload":"GlobalBarrier"}"#,
        r#"{"op":1,"kind":7,"payload":"GlobalBarrier"}"#,
        r#"{"op":1,"kind":"k","payload":"GlobalBarrier","junk":[{"x":"\ud800"}]}"#,
        r#"{"op":1,"kind":"k","payload":"GlobalBarrier","junk":1e999}"#,
        r#"{"op":1,"kind":"k","payload":"GlobalBarrier","junk":"\xff"}"#,
        r#"{"op":1,"kind":"k","payload":"Barrier","affected":[1],"affected":[2]}"#,
        r#"{"op":1,"kind":"k","payload":"Barrier","affected":[-1]}"#,
        r#"{"op":1,"kind":"k","payload":"Barrier","affected":[1.5]}"#,
        r#"{"op":1,"kind":"k","payload":"Barrier"}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":1,"scope":[],"events":[]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[],"events":[]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"event":"Preserved","from":2}],[3,{"from":4,"event":"Modified"}],[5,{"event":"Generated","sources":[1,2]}],[6,{"event":"Merged","from":[]}],[7,{"event":"Deleted","extra":[[]]}],[8,{"event":"Unresolved","candidates":[9]}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"event":0,"from":2}],[2,{"event":4}],[3,{"event":5,"candidates":[]}],[4,{"event":6}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,["Preserved",2]],[2,["Deleted"]],[3,[4]],[4,[5,[]]],[5,["Merged",[1],2]]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[2,["Deleted",1]]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[2,[]]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,["Preserved"]]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"event":"Preserved","from":2,"from":3}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"event":"Preserved","event":"Preserved","from":2}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"from":2}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"event":"Preserved"}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"event":"Preserved","from":2},3]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[-1,{"event":"Deleted"}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,"Deleted"]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":[[1,{"event":"deleted"}]]}"#,
        r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[],"events":{}}"#,
    ] {
        cases.push(v2_journal(entry));
    }
    // Journal index entries, including the positional form.
    for index in [
        r#"[0,"vertex",null]"#,
        r#"[0,"vertex"]"#,
        r#"{"ordinal":0,"kind":"edge"}"#,
        r#"{"ordinal":0,"kind":"edge","local":-1}"#,
        r#"{"ordinal":0,"kind":"edge","local":0,"local":0}"#,
    ] {
        cases.push(v2(&format!(
            r#","journal":{{"next_op":1,"next_ordinal":1,"index":[{index}],"entries":[]}}"#
        )));
    }
    cases.push(v2(r#","journal":[1,1,[],[]]"#));
    cases.push(v2(r#","journal":[1,1,[]]"#));
    cases.push(v2(
        r#","journal":{"next_op":1,"next_ordinal":1,"index":[],"entries":[],"x":[[[]]]}"#,
    ));
    // Attributes.
    for attributes in [
        r#"{"solids":[[0,{"name":"a"}]],"faces":[[1,{"color":[0.5,0.25,1]}]]}"#,
        r#"{"solids":[],"faces":[]}"#,
        "{}",
        "[]",
        "[[[0,{}]]]",
        r#"{"solids":[[0,{"name":null,"color":null}]]}"#,
        r#"{"solids":[[0,["a",[1,1,1]]]]}"#,
        r#"{"solids":[[0,["a",[1,1,1],1]]]}"#,
        r#"{"solids":[[0,{"name":"a","name":"b"}]]}"#,
        r#"{"solids":[[0,{"color":[1,1]}]]}"#,
        r#"{"solids":[[0]]}"#,
        r#"{"solids":[[0,{},1]]}"#,
    ] {
        cases.push(v2(&format!(r#","attributes":{attributes}"#)));
    }
    // Nesting: unbounded in skipped values, 127 levels in buffered data.
    for depth in [120, 123, 124, 125, 126, 127, 200] {
        cases.push(v2(&format!(
            r#","skipped":{}"#,
            nested("[", "]", depth, "")
        )));
        cases.push(v2(&format!(
            r#","journal":{{"next_op":1,"next_ordinal":1,"index":[],"entries":[],"x":{}}}"#,
            nested("[", "]", depth, "")
        )));
        cases.push(v2(&format!(
            r#","journal":{}"#,
            nested("{\"a\":", "}", depth, "1")
        )));
        cases.push(v2_journal(&format!(
            r#"{{"op":1,"kind":"k","payload":"GlobalBarrier","deep":{}}}"#,
            nested("[", "]", depth, "")
        )));
    }
    cases
}

#[test]
fn edge_cases_read_identically() {
    let mut cases: Vec<Vec<u8>> = edge_cases().into_iter().map(String::into_bytes).collect();
    // Invalid UTF-8 is refused only where a string is read as data: in keys
    // of objects read as data, in typed strings, and in journal values.
    let tail = |text: &[u8]| {
        let mut bytes = v2("").into_bytes();
        bytes.pop();
        bytes.extend_from_slice(text);
        bytes
    };
    cases.push(tail(b",\"\xc3\":1}"));
    cases.push(tail(b",\"unknown\":[\"\xff\",{\"\xfe\":\"\xc3\"}]}"));
    cases.push(tail(b",\"journal\":[\"\xff\"]}"));
    cases.push(tail(b",\"journal\":{\"\xff\":1}}"));
    cases.push(tail(
        b",\"attributes\":{\"solids\":[[0,{\"name\":\"\xff\"}]]}}",
    ));
    cases.push(tail(
        b",\"attributes\":{\"solids\":[[0,{\"x\":\"\xff\",\"\xfe\":1}]]}}",
    ));
    cases.push(tail(b",\"journal\":{\"next_op\":1,\"next_ordinal\":1,\"index\":[],\"entries\":[{\"op\":1,\"kind\":\"\xff\",\"payload\":\"GlobalBarrier\"}]}}"));
    for case in cases {
        assert_same_read(&case, ImportLimits::default());
        for max_model_entities in [0, 1, 2, 4] {
            assert_same_read(
                &case,
                ImportLimits {
                    max_model_entities,
                    ..ImportLimits::default()
                },
            );
        }
    }
}

#[test]
fn journal_budget_counts_like_the_preflight() {
    // Items over budget refuse as a limit even when a later part of the
    // document is malformed, but not when the syntax is broken.
    let many = format!("[{}]", vec!["0"; 50].join(","));
    for (document, max_model_entities) in [
        (
            v2_journal(&format!(
                r#"{{"op":1,"kind":"k","payload":"Barrier","affected":{many}}}"#
            )),
            20,
        ),
        (v2(&format!(r#","journal":{many},"journal":{many}"#)), 60),
        (v2(&format!(r#","journal":{many},"vertices":7"#)), 20),
        (v2(&format!(r#","journal":[{many},"\ud800"]"#)), 20),
        (v2(&format!(r#","journal":["\ud800",{many}]"#)), 20),
        (v2(&format!(r#","journal":{many},"x":[1,]"#)), 20),
        (format!(r#"{{"version":9,"journal":{many}}}"#), 20),
        (format!(r#"{{"version":1,"journal":{many}}}"#), 20),
    ] {
        assert_same_read(
            document.as_bytes(),
            ImportLimits {
                max_model_entities,
                ..ImportLimits::default()
            },
        );
    }
}

// ===========================================================================
// Mutation fuzzing
// ===========================================================================

/// Small valid documents of every version and feature to mutate.
fn seeds() -> Vec<Vec<u8>> {
    let mut seeds: Vec<Vec<u8>> = vec![
        v2_journal(
            r#"{"op":1,"kind":"k","payload":"Evolution","construction":true,"scope":[1],"events":[[1,{"event":"Preserved","from":2}],[2,{"event":"Deleted"}],[3,{"event":"Merged","from":[4]}]]},{"op":2,"kind":"b","payload":"Barrier","affected":[1]},{"op":3,"kind":"g","payload":"GlobalBarrier"}"#,
        )
        .into_bytes(),
        v2(r#","attributes":{"solids":[[0,{"name":"a\n","color":[0.5,0.25,1]}]],"faces":[[1,{"name":"f"}]]}"#)
            .into_bytes(),
    ];
    let mut topo = Topology::new();
    let solid = make_box(&mut topo, 1.0, 2.0, 3.0).unwrap();
    seeds.push(serialize_solid(&topo, solid).unwrap());
    let cylinder = make_cylinder(&mut topo, 0.5, 1.0).unwrap();
    seeds.push(serialize_solid(&topo, cylinder).unwrap());
    let mut bodies = Topology::new();
    let sheet = nurbs_sheet(&mut bodies);
    seeds.push(serialize_sheet(&bodies, sheet).unwrap());
    let mut journaled = Topology::new();
    let a = make_box(&mut journaled, 1.0, 1.0, 1.0).unwrap();
    let b = make_box(&mut journaled, 0.5, 0.5, 2.0).unwrap();
    let fused = boolean_journaled(&mut journaled, BooleanOp::Fuse, a, b).unwrap();
    seeds.push(serialize_solid(&journaled, fused.solid).unwrap());
    let mut smallest_fixtures: Vec<_> = fixture_documents()
        .into_iter()
        .map(|(_, bytes)| bytes)
        .collect();
    smallest_fixtures.sort_by_key(Vec::len);
    seeds.extend(smallest_fixtures.into_iter().take(3));
    seeds
}

fn seeds_cached() -> &'static [Vec<u8>] {
    static SEEDS: std::sync::OnceLock<Vec<Vec<u8>>> = std::sync::OnceLock::new();
    SEEDS.get_or_init(seeds)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: std::env::var("ARENA_CODEC_FUZZ_CASES")
            .ok()
            .and_then(|cases| cases.parse().ok())
            .unwrap_or(1500),
        ..ProptestConfig::default()
    })]

    #[test]
    fn mutated_documents_read_identically(
        seed in any::<usize>(),
        mutations in prop::collection::vec(mutation(), 1..4),
        entities in prop_oneof![Just(usize::MAX), 0_usize..64],
    ) {
        let seeds = seeds_cached();
        let mut bytes = seeds[seed % seeds.len()].clone();
        for mutation in &mutations {
            bytes = apply(&bytes, mutation);
        }
        let limits = if entities == usize::MAX {
            ImportLimits::default()
        } else {
            ImportLimits {
                max_model_entities: entities,
                ..ImportLimits::default()
            }
        };
        assert_same_read(&bytes, limits);
    }
}

// ===========================================================================
// Verbatim pre-change reader
// ===========================================================================

// A journal has nested sequences whose restoration allocates, copies and sorts.
// Count their encoded slots without materializing any vectors. Geometry is
// skipped; its established table budgets remain independent of history size.
struct JournalBudget {
    used: usize,
    limit: usize,
    exceeded: bool,
}

struct JournalItems<'a>(&'a mut JournalBudget);

impl<'de> DeserializeSeed<'de> for JournalItems<'_> {
    type Value = ();
    fn deserialize<D: serde::Deserializer<'de>>(self, deserializer: D) -> Result<(), D::Error> {
        deserializer.deserialize_any(self)
    }
}

impl<'de> Visitor<'de> for JournalItems<'_> {
    type Value = ();
    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("journal JSON")
    }
    fn visit_unit<E: serde::de::Error>(self) -> Result<(), E> {
        Ok(())
    }
    fn visit_bool<E: serde::de::Error>(self, _: bool) -> Result<(), E> {
        Ok(())
    }
    fn visit_u64<E: serde::de::Error>(self, _: u64) -> Result<(), E> {
        Ok(())
    }
    fn visit_i64<E: serde::de::Error>(self, _: i64) -> Result<(), E> {
        Ok(())
    }
    fn visit_f64<E: serde::de::Error>(self, _: f64) -> Result<(), E> {
        Ok(())
    }
    fn visit_str<E: serde::de::Error>(self, _: &str) -> Result<(), E> {
        Ok(())
    }
    fn visit_seq<A: SeqAccess<'de>>(self, mut sequence: A) -> Result<(), A::Error> {
        while sequence.next_element_seed(JournalItems(self.0))?.is_some() {
            let next = self.0.used.checked_add(1);
            self.0.used = next.unwrap_or(usize::MAX);
            if next.is_none() || self.0.used > self.0.limit {
                self.0.exceeded = true;
                return Err(serde::de::Error::custom("journal item limit exceeded"));
            }
        }
        Ok(())
    }
    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<(), A::Error> {
        while map.next_key::<IgnoredAny>()?.is_some() {
            map.next_value_seed(JournalItems(self.0))?;
        }
        Ok(())
    }
}

struct JournalPreflight<'a>(&'a mut JournalBudget);
impl<'de> Visitor<'de> for JournalPreflight<'_> {
    type Value = ();
    fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("arena document object")
    }
    fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<(), A::Error> {
        while let Some(key) = map.next_key::<std::borrow::Cow<'de, str>>()? {
            if key == "journal" {
                map.next_value_seed(JournalItems(self.0))?;
            } else {
                map.next_value::<IgnoredAny>()?;
            }
        }
        Ok(())
    }
}

fn check_encoded_journal(bytes: &[u8], limit: usize) -> Result<(), IoError> {
    let mut budget = JournalBudget {
        used: 0,
        limit,
        exceeded: false,
    };
    let mut deserializer = serde_json::Deserializer::from_slice(bytes);
    let result =
        serde::Deserializer::deserialize_map(&mut deserializer, JournalPreflight(&mut budget));
    match result {
        Err(_) if budget.exceeded => Err(IoError::LimitExceeded {
            resource: "arena journal items",
            limit,
            actual: budget.used,
        }),
        Err(error) => Err(IoError::ParseError {
            reason: format!("arena journal preflight failed: {error}"),
        }),
        Ok(()) => Ok(()),
    }
}

fn oracle_parse_document(bytes: &[u8], limits: ImportLimits) -> Result<ParsedDocument, IoError> {
    ensure_input_size(bytes.len(), limits)?;
    let header: VersionHeader = serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
        reason: format!("arena deserialization failed: {e}"),
    })?;
    if header.version != LEGACY_SINGLE_SOLID_VERSION {
        check_encoded_journal(bytes, limits.max_model_entities)?;
    }
    let document = match header.version {
        LEGACY_SINGLE_SOLID_VERSION => {
            let dump: SerializedSolidV1 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v1 deserialization failed: {e}"),
                })?;
            if dump.version != LEGACY_SINGLE_SOLID_VERSION {
                return Err(IoError::ParseError {
                    reason: format!("arena v1 document reported version {}", dump.version),
                });
            }
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: vec![SerSolid {
                    outer_shell: dump.outer_shell,
                    inner_shells: dump.inner_shells,
                }],
                solid_roots: vec![0],
                sheet_roots: Vec::new(),
                wire_roots: Vec::new(),
                compounds: Vec::new(),
                pcurves: dump.pcurves,
                boundary_authority: None,
                journal: None,
                attributes: None,
            }
        }
        LEGACY_MULTI_ROOT_VERSION => {
            let dump: SerializedDocumentV2 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v2 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                sheet_roots: Vec::new(),
                wire_roots: Vec::new(),
                compounds: dump.compounds,
                pcurves: dump.pcurves,
                boundary_authority: None,
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        FORMAT_VERSION => {
            let dump: SerializedDocumentV3 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v3 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                sheet_roots: Vec::new(),
                wire_roots: Vec::new(),
                compounds: dump.compounds,
                pcurves: Vec::new(),
                boundary_authority: Some(dump.boundary_authority),
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        SHEET_ROOT_FORMAT_VERSION => {
            let dump: SerializedDocumentV4 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v4 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                sheet_roots: dump.sheet_roots,
                wire_roots: Vec::new(),
                compounds: dump.compounds,
                pcurves: Vec::new(),
                boundary_authority: Some(dump.boundary_authority),
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        WIRE_ROOT_FORMAT_VERSION => {
            let dump: SerializedDocumentV5 =
                serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
                    reason: format!("arena v5 deserialization failed: {e}"),
                })?;
            ParsedDocument {
                vertices: dump.vertices,
                edges: dump.edges,
                wires: dump.wires,
                faces: dump.faces,
                shells: dump.shells,
                solids: dump.solids,
                solid_roots: dump.solid_roots,
                sheet_roots: dump.sheet_roots,
                wire_roots: dump.wire_roots,
                compounds: dump.compounds,
                pcurves: Vec::new(),
                boundary_authority: Some(dump.boundary_authority),
                journal: dump.journal,
                attributes: dump.attributes,
            }
        }
        version => {
            return Err(IoError::ParseError {
                reason: format!(
                    "unsupported arena dump version {version} (supported: {LEGACY_SINGLE_SOLID_VERSION}, {LEGACY_MULTI_ROOT_VERSION}, {FORMAT_VERSION}, {SHEET_ROOT_FORMAT_VERSION}, {WIRE_ROOT_FORMAT_VERSION})"
                ),
            });
        }
    };
    check_document_limits(&document, limits)?;
    Ok(document)
}
