//! Persistent-reference serialization (RFC 0003, Stage 5).
//!
//! A [`PersistentRef`] is a value object holding no arena ids — its
//! anchors are journal `OpId`s, wrapped references, or geometric
//! signatures — so it serializes context-free: a reference written from
//! one session resolves in any session holding the model's journal (see
//! `arena_io` for the journal's document form).
//!
//! The encoding is versioned JSON; version 1 documents are read forever,
//! and incompatible changes require a new version with a dedicated read
//! path.

use brepkit_topology::journal::{EntityKind, OpId};
use brepkit_topology::naming::{
    AdjacencySignature, Anchor, Discriminator, EntitySignature, PersistentRef,
};
use serde::{Deserialize, Serialize};

use crate::IoError;

const REF_FORMAT_VERSION: u32 = 1;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerRefDocument {
    version: u32,
    entity_kind: String,
    anchor: SerAnchor,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    discriminators: Vec<SerDiscriminator>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "anchor")]
enum SerAnchor {
    OperationOutput { operation: u64, index: usize },
    LineageOf { base: Box<SerRefDocument> },
    Signature { signature: SerSignature },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "discriminator", content = "tag")]
enum SerDiscriminator {
    SurfaceType(String),
    CurveType(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SerSignature {
    kind: String,
    type_tag: String,
    params: Vec<i64>,
    quantum_bits: u64,
    adjacency: (u32, u32),
    #[serde(default, skip_serializing_if = "Option::is_none")]
    endpoints: Option<Box<(Self, Self)>>,
}

fn kind_name(kind: EntityKind) -> String {
    kind.as_str().to_owned()
}

fn parse_kind(name: &str) -> Result<EntityKind, IoError> {
    match name {
        "vertex" => Ok(EntityKind::Vertex),
        "edge" => Ok(EntityKind::Edge),
        "face" => Ok(EntityKind::Face),
        other => Err(IoError::ParseError {
            reason: format!("persistent reference has unknown entity kind {other:?}"),
        }),
    }
}

fn encode_signature(signature: &EntitySignature) -> SerSignature {
    SerSignature {
        kind: kind_name(signature.kind),
        type_tag: signature.type_tag.clone(),
        params: signature.params.clone(),
        quantum_bits: signature.quantum_bits,
        adjacency: (signature.adjacency.primary, signature.adjacency.secondary),
        endpoints: signature
            .endpoints
            .as_ref()
            .map(|pair| Box::new((encode_signature(&pair.0), encode_signature(&pair.1)))),
    }
}

fn decode_signature(encoded: SerSignature) -> Result<EntitySignature, IoError> {
    let endpoints = match encoded.endpoints {
        Some(pair) => Some(Box::new((
            decode_signature(pair.0)?,
            decode_signature(pair.1)?,
        ))),
        None => None,
    };
    Ok(EntitySignature {
        kind: parse_kind(&encoded.kind)?,
        type_tag: encoded.type_tag,
        params: encoded.params,
        quantum_bits: encoded.quantum_bits,
        adjacency: AdjacencySignature {
            primary: encoded.adjacency.0,
            secondary: encoded.adjacency.1,
        },
        endpoints,
    })
}

fn encode_ref(reference: &PersistentRef) -> SerRefDocument {
    SerRefDocument {
        version: REF_FORMAT_VERSION,
        entity_kind: kind_name(reference.entity_kind),
        anchor: match &reference.anchor {
            Anchor::OperationOutput { operation, index } => SerAnchor::OperationOutput {
                operation: operation.value(),
                index: *index,
            },
            Anchor::LineageOf { base } => SerAnchor::LineageOf {
                base: Box::new(encode_ref(base)),
            },
            Anchor::Signature { signature } => SerAnchor::Signature {
                signature: encode_signature(signature),
            },
        },
        discriminators: reference
            .discriminators
            .iter()
            .map(|discriminator| match discriminator {
                Discriminator::SurfaceType(tag) => SerDiscriminator::SurfaceType(tag.clone()),
                Discriminator::CurveType(tag) => SerDiscriminator::CurveType(tag.clone()),
            })
            .collect(),
    }
}

fn decode_ref(encoded: SerRefDocument) -> Result<PersistentRef, IoError> {
    if encoded.version != REF_FORMAT_VERSION {
        return Err(IoError::ParseError {
            reason: format!(
                "unsupported persistent reference version {} (supported: {REF_FORMAT_VERSION})",
                encoded.version
            ),
        });
    }
    Ok(PersistentRef {
        entity_kind: parse_kind(&encoded.entity_kind)?,
        anchor: match encoded.anchor {
            SerAnchor::OperationOutput { operation, index } => Anchor::OperationOutput {
                operation: OpId::from_value(operation),
                index,
            },
            SerAnchor::LineageOf { base } => Anchor::LineageOf {
                base: Box::new(decode_ref(*base)?),
            },
            SerAnchor::Signature { signature } => Anchor::Signature {
                signature: decode_signature(signature)?,
            },
        },
        discriminators: encoded
            .discriminators
            .into_iter()
            .map(|discriminator| match discriminator {
                SerDiscriminator::SurfaceType(tag) => Discriminator::SurfaceType(tag),
                SerDiscriminator::CurveType(tag) => Discriminator::CurveType(tag),
            })
            .collect(),
    })
}

/// Serializes a persistent reference to versioned JSON.
///
/// # Errors
///
/// Returns [`IoError::ParseError`] if JSON serialization fails.
pub fn serialize_persistent_ref(reference: &PersistentRef) -> Result<Vec<u8>, IoError> {
    serde_json::to_vec(&encode_ref(reference)).map_err(|e| IoError::ParseError {
        reason: format!("persistent reference serialization failed: {e}"),
    })
}

/// Reconstructs a persistent reference from versioned JSON.
///
/// # Errors
///
/// Returns [`IoError::ParseError`] if the buffer is malformed, reports an
/// unsupported version, or names an unknown entity kind.
pub fn deserialize_persistent_ref(bytes: &[u8]) -> Result<PersistentRef, IoError> {
    let encoded: SerRefDocument =
        serde_json::from_slice(bytes).map_err(|e| IoError::ParseError {
            reason: format!("persistent reference deserialization failed: {e}"),
        })?;
    decode_ref(encoded)
}
