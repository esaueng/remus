//! Differential oracle for the hand-written persistent-reference codec.
//!
//! The codec replaced `serde_json` plus the derived impls of the `Ser*`
//! mirror types, which survive under `cfg(test)`: every test compares the
//! old reader and writer (kept verbatim below) with the new ones on
//! generated references, hand-written edge cases, and mutated documents.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use proptest::prelude::*;
use remus_topology::journal::{EntityKind, OpId};
use remus_topology::naming::{
    AdjacencySignature, Anchor, Discriminator, EntitySignature, PersistentRef,
};

use super::*;
use crate::json::mutation::{apply, mutation};

/// The pre-change writer.
fn oracle_serialize(reference: &PersistentRef) -> Result<Vec<u8>, IoError> {
    serde_json::to_vec(&encode_ref(reference)).map_err(|e| IoError::ParseError {
        reason: format!("persistent reference serialization failed: {e}"),
    })
}

/// The pre-change reader.
fn oracle_deserialize(bytes: &[u8]) -> Result<PersistentRef, IoError> {
    let encoded: SerRefDocument =
        serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
            reason: format!("persistent reference deserialization failed: {e}"),
        })?;
    decode_ref(encoded)
}

/// Compares both readers on the mirror document and on the decoded
/// reference (parse errors carry `serde_json` messages, which are not part
/// of the contract, so only their kind is compared).
fn assert_same_read(bytes: &[u8]) {
    let mut reader = Reader::new(bytes);
    let actual = read_ref(&mut reader)
        .ok()
        .filter(|_| reader.at_end())
        .map(|document| format!("{document:?}"));
    let expected = serde_json::from_slice::<SerRefDocument>(bytes)
        .ok()
        .map(|document| format!("{document:?}"));
    assert_eq!(
        actual,
        expected,
        "mirror documents differ for {}",
        String::from_utf8_lossy(bytes)
    );
    let outcome = |result: Result<PersistentRef, IoError>| match result {
        Ok(reference) => format!("{reference:?}"),
        Err(IoError::ParseError { .. }) => "parse error".to_owned(),
        Err(other) => format!("other error: {other}"),
    };
    assert_eq!(
        outcome(deserialize_persistent_ref(bytes)),
        outcome(oracle_deserialize(bytes)),
        "references differ for {}",
        String::from_utf8_lossy(bytes)
    );
}

fn signature(depth: usize) -> EntitySignature {
    EntitySignature {
        kind: if depth.is_multiple_of(2) {
            EntityKind::Edge
        } else {
            EntityKind::Vertex
        },
        type_tag: format!("tag \"{depth}\"\n\u{1}é"),
        params: vec![-3, 0, i64::MAX, i64::MIN, i64::try_from(depth).unwrap()],
        quantum_bits: u64::MAX - depth as u64,
        adjacency: AdjacencySignature {
            primary: u32::MAX,
            secondary: depth as u32,
        },
        endpoints: (depth > 0).then(|| Box::new((signature(depth - 1), signature(depth - 1)))),
    }
}

fn references() -> Vec<PersistentRef> {
    let mut references = vec![
        PersistentRef::operation_output(OpId::from_value(7), EntityKind::Face, 3),
        PersistentRef::operation_output(OpId::from_value(u64::MAX), EntityKind::Vertex, 0),
        PersistentRef {
            entity_kind: EntityKind::Edge,
            anchor: Anchor::Signature {
                signature: signature(3),
            },
            discriminators: vec![
                Discriminator::SurfaceType("plane".to_owned()),
                Discriminator::CurveType("circle \\ \"".to_owned()),
            ],
        },
    ];
    let mut chained = references[0].clone();
    for depth in 0..70_usize {
        chained = PersistentRef {
            entity_kind: EntityKind::Face,
            anchor: Anchor::LineageOf {
                base: Box::new(chained),
            },
            discriminators: if depth.is_multiple_of(3) {
                vec![Discriminator::SurfaceType(format!("s{depth}"))]
            } else {
                Vec::new()
            },
        };
        if depth.is_multiple_of(7) || depth > 58 {
            references.push(chained.clone());
        }
    }
    references
}

#[test]
fn generated_references_write_and_read_identically() {
    for reference in references() {
        let bytes = serialize_persistent_ref(&reference).unwrap();
        let expected = oracle_serialize(&reference).unwrap();
        assert_eq!(
            String::from_utf8(bytes.clone()).unwrap(),
            String::from_utf8(expected).unwrap()
        );
        assert_same_read(&bytes);
    }
}

fn lineage(depth: usize, inner: &str) -> String {
    let mut text = inner.to_owned();
    for _ in 0..depth {
        text = format!(
            r#"{{"version":1,"entity_kind":"face","anchor":{{"anchor":"LineageOf","base":{text}}}}}"#
        );
    }
    text
}

#[test]
fn edge_cases_read_identically() {
    let operation = r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"OperationOutput","operation":3,"index":0}}"#;
    let mut cases: Vec<String> = vec![
        operation.to_owned(),
        format!("{operation} "),
        format!("{operation}x"),
        String::new(),
        "[]".to_owned(),
        r#"[1,"face",{"anchor":"OperationOutput","operation":3,"index":0}]"#.to_owned(),
        r#"[1,"face",["OperationOutput",3,0],[]]"#.to_owned(),
        r#"[1,"face",["OperationOutput",3,0],[],1]"#.to_owned(),
        r#"[1,"face",[0,3,0]]"#.to_owned(),
        r#"[1,"face",["OperationOutput",3]]"#.to_owned(),
        r#"[1,"face",["OperationOutput",3,0,1]]"#.to_owned(),
        r#"[1,"face"]"#.to_owned(),
        r#"{"version":2,"entity_kind":"face","anchor":{"anchor":"OperationOutput","operation":3,"index":0}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"solid","anchor":{"anchor":"OperationOutput","operation":3,"index":0}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"operation":3,"index":0,"anchor":"OperationOutput","x":[["\ud800"]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"OperationOutput","anchor":"OperationOutput","operation":3,"index":0}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":0,"operation":3,"index":0}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"OperationOutput","operation":-3,"index":0}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"OperationOutput","operation":3,"index":4294967296}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"OperationOutput","operation":3,"operation":3,"index":0}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"OperationOutput","operation":3}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":"OperationOutput"}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],[[0,"x"],{"discriminator":{"CurveType":{}},"tag":"y"},{"tag":"z","discriminator":"SurfaceType","w":1}]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],null]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":{"version":1,"entity_kind":"face","anchor":[0,3,0],"version":1}}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],[[2,"x"]]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],[{"discriminator":{"CurveType":1},"tag":"y"}]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],[{"discriminator":{"CurveType":null,"SurfaceType":null},"tag":"y"}]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],[{"discriminator":0,"tag":"y"}]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],[["SurfaceType","x",1]]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":{"kind":"edge","type_tag":"line","params":[1,-2],"quantum_bits":3,"adjacency":[1,2]}}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[1,-2],3,[1,2],null]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[1,-2],3,[1,2],[["vertex","p",[],0,[0,0]],["vertex","p",[],0,[0,0],null]]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[1,-2],3,[1,2],[["vertex","p",[],0,[0,0]]]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[1,-2],3,[1,2,3]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[9223372036854775808],3,[1,2]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[1.0],3,[1,2]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[],-3,[1,2]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":["edge","line",[],3,[4294967296,2]]}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":{"kind":"edge","type_tag":"line","params":[],"quantum_bits":3,"adjacency":[1,2],"endpoints":{}}}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":{"kind":"edge","kind":"edge","type_tag":"line","params":[],"quantum_bits":3,"adjacency":[1,2]}}}"#.to_owned(),
        r#"{"version":1,"entity_kind":"edge","anchor":{"anchor":"Signature","signature":{"kind":"bogus","type_tag":"line","params":[],"quantum_bits":3,"adjacency":[1,2]}}}"#.to_owned(),
    ];
    for discriminators in [
        r#"[{"discriminator":"SurfaceType","tag":"plane"}]"#,
        r#"[{"tag":"plane","discriminator":"SurfaceType"}]"#,
        r#"[{"discriminator":{"SurfaceType":null},"tag":"x"}]"#,
        r#"[{"discriminator":{"SurfaceType":{}},"tag":"x"}]"#,
        r#"[{"discriminator":{"SurfaceType":null,"CurveType":null},"tag":"x"}]"#,
        r#"[{"discriminator":{},"tag":"x"}]"#,
        r#"[["SurfaceType","x"]]"#,
        r#"[[0,"x"]]"#,
        r#"[["SurfaceType"]]"#,
        r#"[["SurfaceType","x","y"]]"#,
        r#"[{"discriminator":"SurfaceType"}]"#,
        r#"[{"tag":"x"}]"#,
        r#"[{"discriminator":"SurfaceType","tag":"x","tag":"y"}]"#,
        r#"[{"discriminator":"SurfaceType","discriminator":"SurfaceType","tag":"x"}]"#,
        r#"[{"tag":"x","tag":"y","discriminator":"SurfaceType"}]"#,
        r#"[{"tag":"x","discriminator":"SurfaceType","discriminator":"CurveType"}]"#,
        r#"[{"discriminator":"SurfaceType","tag":1}]"#,
        r#"[{"tag":[[[]]],"discriminator":"SurfaceType"}]"#,
        r#"[{"discriminator":"Volume","tag":"x"}]"#,
        r#"[{"discriminator":"SurfaceType","tag":"x","junk":["\ud800",1e999]}]"#,
        r#"[{"junk":["\ud800"],"discriminator":"SurfaceType","tag":"x"}]"#,
        r#"[{"tag":"\ud800","discriminator":"SurfaceType"}]"#,
        "[]",
        "null",
        "{}",
    ] {
        cases.push(format!(
            r#"{{"version":1,"entity_kind":"face","anchor":{{"anchor":"OperationOutput","operation":3,"index":0}},"discriminators":{discriminators}}}"#
        ));
        cases.push(format!(
            r#"{{"version":1,"entity_kind":"face","anchor":{{"anchor":"LineageOf","base":[1,"face",[0,3,0],{discriminators}]}}}}"#
        ));
    }
    // Buffered nesting is bounded; each lineage level nests two objects.
    for depth in [60, 61, 62, 63, 64, 70] {
        cases.push(lineage(depth, operation));
    }
    for case in cases {
        assert_same_read(case.as_bytes());
    }
    for raw in [
        &b"{\"version\":1,\"entity_kind\":\"face\",\"anchor\":{\"anchor\":\"OperationOutput\",\"operation\":3,\"index\":0},\"x\":\"\xff\"}"[..],
        b"{\"version\":1,\"entity_kind\":\"fa\xffce\",\"anchor\":{\"anchor\":\"OperationOutput\",\"operation\":3,\"index\":0}}",
        b"{\"version\":1,\"entity_kind\":\"face\",\"anchor\":{\"anchor\":\"OperationOutput\",\"operation\":3,\"index\":0,\"x\":\"\xff\"}}",
        b"{\"version\":1,\"entity_kind\":\"face\",\"anchor\":{\"anchor\":\"OperationOutput\",\"operation\":3,\"index\":0},\"discriminators\":[{\"discriminator\":\"SurfaceType\",\"tag\":\"x\",\"j\":\"\xff\"}]}",
    ] {
        assert_same_read(raw);
    }
}

fn seeds() -> Vec<Vec<u8>> {
    let mut seeds: Vec<Vec<u8>> = references()
        .iter()
        .take(6)
        .map(|reference| serialize_persistent_ref(reference).unwrap())
        .collect();
    seeds.push(
        br#"{"version":1,"entity_kind":"face","anchor":{"anchor":"LineageOf","base":[1,"face",[0,3,0],[[0,"x"],{"discriminator":{"CurveType":{}},"tag":"y"}]]},"discriminators":[{"discriminator":"SurfaceType","tag":"plane"},["CurveType","c"]]}"#
            .to_vec(),
    );
    seeds
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: std::env::var("NAMING_CODEC_FUZZ_CASES")
            .ok()
            .and_then(|cases| cases.parse().ok())
            .unwrap_or(1500),
        ..ProptestConfig::default()
    })]

    #[test]
    fn mutated_references_read_identically(
        seed in any::<usize>(),
        mutations in prop::collection::vec(mutation(), 1..4),
    ) {
        let seeds = seeds();
        let mut bytes = seeds[seed % seeds.len()].clone();
        for mutation in &mutations {
            bytes = apply(&bytes, mutation);
        }
        assert_same_read(&bytes);
    }
}
