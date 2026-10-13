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

use remus_topology::journal::{EntityKind, OpId};
use remus_topology::naming::{
    AdjacencySignature, Anchor, Discriminator, EntitySignature, PersistentRef,
};
use std::borrow::Cow;

#[cfg(test)]
use serde::{Deserialize, Serialize};

use crate::IoError;
use crate::json::{Content, Num, Reader, Res, Writer};

const REF_FORMAT_VERSION: u32 = 1;

#[derive(Debug, Clone)]
#[cfg_attr(test, derive(Serialize, Deserialize))]
struct SerRefDocument {
    version: u32,
    entity_kind: String,
    anchor: SerAnchor,
    #[cfg_attr(test, serde(default, skip_serializing_if = "Vec::is_empty"))]
    discriminators: Vec<SerDiscriminator>,
}

#[derive(Debug, Clone)]
#[cfg_attr(test, derive(Serialize, Deserialize))]
#[cfg_attr(test, serde(tag = "anchor"))]
enum SerAnchor {
    OperationOutput { operation: u64, index: usize },
    LineageOf { base: Box<SerRefDocument> },
    Signature { signature: SerSignature },
}

#[derive(Debug, Clone)]
#[cfg_attr(test, derive(Serialize, Deserialize))]
#[cfg_attr(test, serde(tag = "discriminator", content = "tag"))]
enum SerDiscriminator {
    SurfaceType(String),
    CurveType(String),
}

#[derive(Debug, Clone)]
#[cfg_attr(test, derive(Serialize, Deserialize))]
struct SerSignature {
    kind: String,
    type_tag: String,
    params: Vec<i64>,
    quantum_bits: u64,
    adjacency: (u32, u32),
    #[cfg_attr(test, serde(default, skip_serializing_if = "Option::is_none"))]
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
// The `Result` is the public contract kept from the `serde_json` writer.
#[allow(clippy::unnecessary_wraps)]
pub fn serialize_persistent_ref(reference: &PersistentRef) -> Result<Vec<u8>, IoError> {
    let mut writer = Writer::new(None);
    write_ref(&mut writer, &encode_ref(reference));
    Ok(writer.finish())
}

/// Reconstructs a persistent reference from versioned JSON.
///
/// # Errors
///
/// Returns [`IoError::ParseError`] if the buffer is malformed, reports an
/// unsupported version, or names an unknown entity kind.
pub fn deserialize_persistent_ref(bytes: &[u8]) -> Result<PersistentRef, IoError> {
    let mut reader = Reader::new(bytes);
    let encoded = read_ref(&mut reader)
        .and_then(|encoded| {
            if reader.at_end() {
                Ok(encoded)
            } else {
                Err(reader.error("trailing characters"))
            }
        })
        .map_err(|e| IoError::ParseError {
            reason: format!("persistent reference deserialization failed: {e}"),
        })?;
    decode_ref(encoded)
}

// ---------------------------------------------------------------------------
// JSON encoding, byte for byte what `serde_json` wrote for the derived types
// ---------------------------------------------------------------------------

fn write_ref(w: &mut Writer, document: &SerRefDocument) {
    w.key_u64(b"{\"version\":", u64::from(document.version));
    w.raw(b",\"entity_kind\":");
    w.string(&document.entity_kind);
    match &document.anchor {
        SerAnchor::OperationOutput { operation, index } => {
            w.key_u64(
                b",\"anchor\":{\"anchor\":\"OperationOutput\",\"operation\":",
                *operation,
            );
            w.key_usize(b",\"index\":", *index);
        }
        SerAnchor::LineageOf { base } => {
            w.raw(b",\"anchor\":{\"anchor\":\"LineageOf\",\"base\":");
            write_ref(w, base);
        }
        SerAnchor::Signature { signature } => {
            w.raw(b",\"anchor\":{\"anchor\":\"Signature\",\"signature\":");
            write_signature(w, signature);
        }
    }
    w.byte(b'}');
    if !document.discriminators.is_empty() {
        w.raw(b",\"discriminators\":[");
        for (index, discriminator) in document.discriminators.iter().enumerate() {
            if index > 0 {
                w.byte(b',');
            }
            let (open, tag): (&[u8], _) = match discriminator {
                SerDiscriminator::SurfaceType(tag) => {
                    (b"{\"discriminator\":\"SurfaceType\",\"tag\":", tag)
                }
                SerDiscriminator::CurveType(tag) => {
                    (b"{\"discriminator\":\"CurveType\",\"tag\":", tag)
                }
            };
            w.raw(open);
            w.string(tag);
            w.byte(b'}');
        }
        w.byte(b']');
    }
    w.byte(b'}');
}

fn write_signature(w: &mut Writer, signature: &SerSignature) {
    w.raw(b"{\"kind\":");
    w.string(&signature.kind);
    w.raw(b",\"type_tag\":");
    w.string(&signature.type_tag);
    w.raw(b",\"params\":[");
    for (index, &param) in signature.params.iter().enumerate() {
        if index > 0 {
            w.byte(b',');
        }
        w.i64(param);
    }
    w.key_u64(b"],\"quantum_bits\":", signature.quantum_bits);
    w.key_u64(b",\"adjacency\":[", u64::from(signature.adjacency.0));
    w.key_u64(b",", u64::from(signature.adjacency.1));
    w.byte(b']');
    if let Some(endpoints) = &signature.endpoints {
        w.raw(b",\"endpoints\":[");
        write_signature(w, &endpoints.0);
        w.byte(b',');
        write_signature(w, &endpoints.1);
        w.byte(b']');
    }
    w.byte(b'}');
}

// ---------------------------------------------------------------------------
// JSON decoding, accepting exactly what `serde_json` accepted for the
// derived types. The top-level document and its discriminators are read
// directly; the internally tagged anchor is buffered (as `serde` buffers it)
// and everything inside it is decoded from that buffer, where `serde` also
// accepts variant indices for buffered tags.
// ---------------------------------------------------------------------------

const ANCHORS: &[&str] = &["OperationOutput", "LineageOf", "Signature"];
const DISCRIMINATORS: &[&str] = &["SurfaceType", "CurveType"];

fn discriminator(variant: usize, tag: String) -> SerDiscriminator {
    if variant == 0 {
        SerDiscriminator::SurfaceType(tag)
    } else {
        SerDiscriminator::CurveType(tag)
    }
}

fn read_ref(reader: &mut Reader<'_>) -> Res<SerRefDocument> {
    let (mut version, mut entity_kind, mut anchor, mut discriminators) = (None, None, None, None);
    reader.read_struct(
        &["version", "entity_kind", "anchor", "discriminators"],
        0b111,
        3,
        &mut |reader, index| {
            match index {
                0 => version = Some(reader.read_u32()?),
                1 => entity_kind = Some(reader.read_string()?.into_owned()),
                2 => {
                    let content = reader.read_content(2)?;
                    anchor = Some(
                        anchor_from_content(&content, false)
                            .ok_or_else(|| reader.error("invalid anchor"))?,
                    );
                }
                _ => discriminators = Some(reader.read_vec(read_discriminator)?),
            }
            Ok(())
        },
    )?;
    match (version, entity_kind, anchor) {
        (Some(version), Some(entity_kind), Some(anchor)) => Ok(SerRefDocument {
            version,
            entity_kind,
            anchor,
            discriminators: discriminators.unwrap_or_default(),
        }),
        _ => Err(reader.error("missing field")),
    }
}

/// Reads an adjacently tagged discriminator: `[variant, tag]`, or an
/// object holding exactly one `discriminator` (a unit variant) and exactly
/// one `tag` string among ignored keys.
fn read_discriminator(reader: &mut Reader<'_>) -> Res<SerDiscriminator> {
    if reader.peek() == Some(b'[') {
        reader.begin_array()?;
        let mut first = true;
        if !reader.next_element(&mut first)? {
            return Err(reader.error("invalid length"));
        }
        let name = reader.read_string()?;
        let variant = DISCRIMINATORS
            .iter()
            .position(|candidate| *candidate == name)
            .ok_or_else(|| reader.error("unknown variant"))?;
        if !reader.next_element(&mut first)? {
            return Err(reader.error("invalid length"));
        }
        let tag = reader.read_string()?.into_owned();
        if reader.next_element(&mut first)? {
            return Err(reader.error("trailing characters"));
        }
        return Ok(discriminator(variant, tag));
    }
    reader.begin_object()?;
    let (mut variant, mut tag) = (None, None);
    let mut first = true;
    while let Some(key) = reader.next_key(&mut first)? {
        match key.as_ref() {
            "discriminator" => {
                if variant.is_some() {
                    return Err(reader.error("duplicate field `discriminator`"));
                }
                let (index, object) = reader.read_variant(DISCRIMINATORS)?;
                reader.unit_variant(object)?;
                variant = Some(index);
            }
            "tag" => {
                if tag.is_some() {
                    return Err(reader.error("duplicate field `tag`"));
                }
                tag = Some(reader.read_string()?.into_owned());
            }
            _ => reader.skip_value()?,
        }
    }
    match (variant, tag) {
        (Some(variant), Some(tag)) => Ok(discriminator(variant, tag)),
        _ => Err(reader.error("missing field")),
    }
}

/// A buffered internally tagged enum: its variant and remaining fields.
enum Fields<'c, 'a> {
    Map(Vec<&'c (Cow<'a, str>, Content<'a>)>),
    Seq(&'c [Content<'a>]),
}

impl<'c, 'a> Fields<'c, 'a> {
    /// Field `index` named `name` of a struct with `count` fields, all
    /// required: by name (repeats refused) or by exact position.
    fn get(&self, name: &str, index: usize, count: usize) -> Option<&'c Content<'a>> {
        match self {
            Self::Map(entries) => {
                let mut found = None;
                for entry in entries {
                    if entry.0 == name {
                        if found.is_some() {
                            return None;
                        }
                        found = Some(&entry.1);
                    }
                }
                found
            }
            Self::Seq(items) => (items.len() == count).then(|| items.get(index)).flatten(),
        }
    }
}

/// A buffered variant tag: a name, or a variant index when `serde` reads the
/// tag from its buffer.
fn tag_of(content: &Content<'_>, names: &[&str], allow_index: bool) -> Option<usize> {
    if !allow_index && !matches!(content, Content::Str(_)) {
        return None;
    }
    content.variant(names)
}

fn anchor_from_content(content: &Content<'_>, allow_index: bool) -> Option<SerAnchor> {
    let (variant, fields) = match content {
        Content::Map(entries) => {
            let mut variant = None;
            let mut rest = Vec::new();
            for entry in entries {
                if entry.0 == "anchor" {
                    if variant.is_some() {
                        return None;
                    }
                    variant = Some(tag_of(&entry.1, ANCHORS, allow_index)?);
                } else {
                    rest.push(entry);
                }
            }
            (variant?, Fields::Map(rest))
        }
        Content::Seq(items) => {
            let (tag, rest) = items.split_first()?;
            (tag_of(tag, ANCHORS, allow_index)?, Fields::Seq(rest))
        }
        _ => return None,
    };
    Some(match variant {
        0 => SerAnchor::OperationOutput {
            operation: fields.get("operation", 0, 2)?.as_u64()?,
            index: usize::try_from(fields.get("index", 1, 2)?.as_u64()?).ok()?,
        },
        1 => SerAnchor::LineageOf {
            base: Box::new(ref_from_content(fields.get("base", 0, 1)?)?),
        },
        _ => SerAnchor::Signature {
            signature: signature_from_content(fields.get("signature", 0, 1)?)?,
        },
    })
}

/// A buffered derive-style struct: by name (unknown keys ignored, repeats
/// refused) or by position, where only the trailing `defaulted` fields may
/// be omitted. Returns the fields in declaration order.
fn struct_from_content<'c, 'a>(
    content: &'c Content<'a>,
    names: &[&str],
    defaulted: usize,
) -> Option<Vec<Option<&'c Content<'a>>>> {
    let mut values = vec![None; names.len()];
    match content {
        Content::Map(entries) => {
            for (key, value) in entries {
                if let Some(index) = names.iter().position(|name| key.as_ref() == *name) {
                    if values[index].is_some() {
                        return None;
                    }
                    values[index] = Some(value);
                }
            }
            if values[..names.len() - defaulted]
                .iter()
                .any(Option::is_none)
            {
                return None;
            }
        }
        Content::Seq(items) => {
            if items.len() > names.len() || items.len() < names.len() - defaulted {
                return None;
            }
            for (slot, item) in values.iter_mut().zip(items) {
                *slot = Some(item);
            }
        }
        _ => return None,
    }
    Some(values)
}

fn content_string(content: &Content<'_>) -> Option<String> {
    match content {
        Content::Str(text) => Some(text.as_ref().to_owned()),
        _ => None,
    }
}

fn ref_from_content(content: &Content<'_>) -> Option<SerRefDocument> {
    let fields = struct_from_content(
        content,
        &["version", "entity_kind", "anchor", "discriminators"],
        1,
    )?;
    let discriminators = match fields[3] {
        None => Vec::new(),
        Some(Content::Seq(items)) => items
            .iter()
            .map(discriminator_from_content)
            .collect::<Option<_>>()?,
        Some(_) => return None,
    };
    Some(SerRefDocument {
        version: u32::try_from(fields[0]?.as_u64()?).ok()?,
        entity_kind: content_string(fields[1]?)?,
        anchor: anchor_from_content(fields[2]?, true)?,
        discriminators,
    })
}

fn discriminator_from_content(content: &Content<'_>) -> Option<SerDiscriminator> {
    match content {
        Content::Seq(items) => match items.as_slice() {
            [variant, tag] => Some(discriminator(
                variant.variant(DISCRIMINATORS)?,
                content_string(tag)?,
            )),
            _ => None,
        },
        Content::Map(entries) => {
            let (mut variant, mut tag) = (None, None);
            for (key, value) in entries {
                if key.as_ref() == "discriminator" {
                    if variant.is_some() {
                        return None;
                    }
                    // A buffered unit variant: `"Name"`, or `{"Name": null}`
                    // or `{"Name": {}}`.
                    variant = Some(match value {
                        Content::Str(name) => DISCRIMINATORS
                            .iter()
                            .position(|candidate| *candidate == name.as_ref())?,
                        Content::Map(single) => match single.as_slice() {
                            [(name, Content::Null)] => DISCRIMINATORS
                                .iter()
                                .position(|candidate| *candidate == name.as_ref())?,
                            [(name, Content::Map(empty))] if empty.is_empty() => DISCRIMINATORS
                                .iter()
                                .position(|candidate| *candidate == name.as_ref())?,
                            _ => return None,
                        },
                        _ => return None,
                    });
                } else if key.as_ref() == "tag" {
                    if tag.is_some() {
                        return None;
                    }
                    tag = Some(content_string(value)?);
                }
            }
            Some(discriminator(variant?, tag?))
        }
        _ => None,
    }
}

fn content_i64(content: &Content<'_>) -> Option<i64> {
    match content {
        Content::Num(Num::U64(value)) => i64::try_from(*value).ok(),
        Content::Num(Num::I64(value)) => Some(*value),
        _ => None,
    }
}

fn signature_from_content(content: &Content<'_>) -> Option<SerSignature> {
    let fields = struct_from_content(
        content,
        &[
            "kind",
            "type_tag",
            "params",
            "quantum_bits",
            "adjacency",
            "endpoints",
        ],
        1,
    )?;
    let params = match fields[2]? {
        Content::Seq(items) => items.iter().map(content_i64).collect::<Option<_>>()?,
        _ => return None,
    };
    let adjacency = match fields[4]? {
        Content::Seq(items) => match items.as_slice() {
            [primary, secondary] => (
                u32::try_from(primary.as_u64()?).ok()?,
                u32::try_from(secondary.as_u64()?).ok()?,
            ),
            _ => return None,
        },
        _ => return None,
    };
    let endpoints = match fields[5] {
        None | Some(Content::Null) => None,
        Some(Content::Seq(items)) => match items.as_slice() {
            [start, end] => Some(Box::new((
                signature_from_content(start)?,
                signature_from_content(end)?,
            ))),
            _ => return None,
        },
        Some(_) => return None,
    };
    Some(SerSignature {
        kind: content_string(fields[0]?)?,
        type_tag: content_string(fields[1]?)?,
        params,
        quantum_bits: fields[3]?.as_u64()?,
        adjacency,
        endpoints,
    })
}

#[cfg(test)]
mod oracle_tests;
