//! Structure-aware mutations of JSON documents, shared by the codec
//! differential tests: token splices plus member duplication, removal,
//! reordering and object-to-array rewrites of syntactically valid input.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use proptest::prelude::*;
use serde::de::IgnoredAny;

/// Byte span of each JSON value in a syntactically valid document, with the
/// key spans of object members (test-side scanner, independent of the codec).
/// A container span: `(start, end, is_object, members)`, members being
/// `(key_start, value_start, value_end)`.
type Container = (usize, usize, bool, Vec<(usize, usize, usize)>);

pub struct Spans {
    /// `(start, end, is_object, members)`; members are `(key_start, value_start, value_end)`.
    containers: Vec<Container>,
    /// Scalars: `(start, end)`.
    scalars: Vec<(usize, usize)>,
}

pub fn scan(bytes: &[u8]) -> Option<Spans> {
    fn ws(bytes: &[u8], mut at: usize) -> usize {
        while matches!(bytes.get(at), Some(b' ' | b'\n' | b'\t' | b'\r')) {
            at += 1;
        }
        at
    }
    fn string_end(bytes: &[u8], mut at: usize) -> Option<usize> {
        at += 1;
        loop {
            match bytes.get(at)? {
                b'"' => return Some(at + 1),
                b'\\' => at += 2,
                _ => at += 1,
            }
        }
    }
    fn value(bytes: &[u8], at: usize, spans: &mut Spans) -> Option<usize> {
        let at = ws(bytes, at);
        match bytes.get(at)? {
            b'"' => {
                let end = string_end(bytes, at)?;
                spans.scalars.push((at, end));
                Some(end)
            }
            open @ (b'[' | b'{') => {
                let object = *open == b'{';
                let close = if object { b'}' } else { b']' };
                let slot = spans.containers.len();
                spans.containers.push((at, 0, object, Vec::new()));
                let mut cursor = ws(bytes, at + 1);
                let mut members = Vec::new();
                if bytes.get(cursor) == Some(&close) {
                    cursor += 1;
                } else {
                    loop {
                        let key = ws(bytes, cursor);
                        let value_start = if object {
                            let key_end = string_end(bytes, key)?;
                            ws(bytes, key_end) + 1
                        } else {
                            key
                        };
                        let value_start = ws(bytes, value_start);
                        let value_end = value(bytes, value_start, spans)?;
                        members.push((key, value_start, value_end));
                        cursor = ws(bytes, value_end);
                        match bytes.get(cursor)? {
                            b',' => cursor += 1,
                            b if *b == close => {
                                cursor += 1;
                                break;
                            }
                            _ => return None,
                        }
                    }
                }
                spans.containers[slot].1 = cursor;
                spans.containers[slot].3 = members;
                Some(cursor)
            }
            _ => {
                let mut end = at;
                while matches!(bytes.get(end), Some(b) if !b",]}: \n\t\r".contains(b)) {
                    end += 1;
                }
                spans.scalars.push((at, end));
                Some(end)
            }
        }
    }
    serde_json::from_slice::<IgnoredAny>(bytes).ok()?;
    let mut spans = Spans {
        containers: Vec::new(),
        scalars: Vec::new(),
    };
    value(bytes, 0, &mut spans)?;
    Some(spans)
}

/// Tokens worth splicing into documents.
pub const TOKENS: &[&str] = &[
    "",
    "null",
    "true",
    "false",
    "0",
    "-0",
    "1",
    "-1",
    "2",
    "4",
    "5",
    "1.5",
    "-2.5e-3",
    "1e400",
    "-1e400",
    "1e-400",
    "18446744073709551615",
    "18446744073709551616",
    "-9223372036854775808",
    "-9223372036854775809",
    "4294967295",
    "4294967296",
    "2147483648",
    "-2147483649",
    "0.0",
    "1E2",
    "1e+2",
    "01",
    "1.",
    "-",
    "[]",
    "{}",
    "[",
    "]",
    "{",
    "}",
    ",",
    ":",
    "\"",
    "\\",
    "\"\\ud800\"",
    "\"\\udc00\"",
    "\"\\ud800\\udc00\"",
    "\"\\u00e9\"",
    "\"\\x\"",
    "\"\u{7f}\"",
    "\"\u{0}\"",
    " ",
    "\n",
    "\"Line\"",
    "{\"Line\":null}",
    "\"NurbsCurve\"",
    "\"Plane\"",
    "\"Circle\"",
    "\"solid\"",
    "\"sheet\"",
    "{\"sheet\":null}",
    "\"Deleted\"",
    "\"payload\"",
    "\"event\"",
    "\"Evolution\"",
    "\"GlobalBarrier\"",
    "\"Barrier\"",
    "\"journal\"",
    "\"version\"",
    "\"trim\"",
    "\"tolerance\"",
    "\"body_class\"",
    "\"pcurve\"",
    "\"forward\"",
    "\"attributes\"",
    "\"name\"",
    "\"color\"",
    "[1,2]",
    "[1,2,3]",
    "[0,0]",
    "[0.5,0.5,0.5]",
    "{\"event\":4}",
    "[\"Deleted\"]",
    "[4]",
    "[\"Preserved\",1]",
    "[[0,{\"event\":\"Deleted\"}]]",
    "\"anchor\"",
    "\"OperationOutput\"",
    "\"LineageOf\"",
    "\"Signature\"",
    "\"base\"",
    "\"discriminator\"",
    "\"tag\"",
    "\"SurfaceType\"",
    "\"CurveType\"",
    "{\"SurfaceType\":null}",
    "{\"SurfaceType\":{}}",
    "\"endpoints\"",
    "\"params\"",
    "[0,\"x\"]",
    "[2,3]",
];

#[derive(Debug, Clone)]
pub enum Mutation {
    /// Replace `len` bytes at a position with a token.
    Splice { at: usize, len: usize, token: usize },
    /// Duplicate an object member (repeated key).
    DuplicateMember { container: usize, member: usize },
    /// Remove an object member or array element.
    RemoveMember { container: usize, member: usize },
    /// Rewrite an object as the array of its values (positional struct
    /// form), dropping `drop` trailing values and maybe adding one more.
    ToArray {
        container: usize,
        drop: usize,
        extra: bool,
    },
    /// Rotate an object's members.
    Rotate { container: usize, by: usize },
    /// Replace a scalar with a token.
    Scalar { scalar: usize, token: usize },
    /// Replace a container with a token.
    Container { container: usize, token: usize },
}

pub fn mutation() -> impl Strategy<Value = Mutation> {
    prop_oneof![
        (any::<usize>(), 0_usize..6, 0..TOKENS.len())
            .prop_map(|(at, len, token)| Mutation::Splice { at, len, token }),
        (any::<usize>(), any::<usize>())
            .prop_map(|(container, member)| Mutation::DuplicateMember { container, member }),
        (any::<usize>(), any::<usize>())
            .prop_map(|(container, member)| Mutation::RemoveMember { container, member }),
        (any::<usize>(), 0_usize..3, any::<bool>()).prop_map(|(container, drop, extra)| {
            Mutation::ToArray {
                container,
                drop,
                extra,
            }
        }),
        (any::<usize>(), 1_usize..4).prop_map(|(container, by)| Mutation::Rotate { container, by }),
        (any::<usize>(), 0..TOKENS.len())
            .prop_map(|(scalar, token)| Mutation::Scalar { scalar, token }),
        (any::<usize>(), 0..TOKENS.len())
            .prop_map(|(container, token)| Mutation::Container { container, token }),
    ]
}

pub fn splice(bytes: &[u8], start: usize, end: usize, with: &[u8]) -> Vec<u8> {
    let mut out = bytes[..start].to_vec();
    out.extend_from_slice(with);
    out.extend_from_slice(&bytes[end..]);
    out
}

pub fn apply(bytes: &[u8], mutation: &Mutation) -> Vec<u8> {
    if let Mutation::Splice { at, len, token } = *mutation {
        let at = at % (bytes.len() + 1);
        let end = (at + len).min(bytes.len());
        return splice(bytes, at, end, TOKENS[token].as_bytes());
    }
    let Some(spans) = scan(bytes) else {
        return bytes.to_vec();
    };
    let pick = |index: usize, count: usize| (count > 0).then(|| index % count);
    match *mutation {
        Mutation::Scalar { scalar, token } => match pick(scalar, spans.scalars.len()) {
            Some(index) => {
                let (start, end) = spans.scalars[index];
                splice(bytes, start, end, TOKENS[token].as_bytes())
            }
            None => bytes.to_vec(),
        },
        Mutation::Container { container, token } => match pick(container, spans.containers.len()) {
            Some(index) if index > 0 => {
                let (start, end, ..) = spans.containers[index];
                splice(bytes, start, end, TOKENS[token].as_bytes())
            }
            _ => bytes.to_vec(),
        },
        Mutation::DuplicateMember { container, member }
        | Mutation::RemoveMember { container, member } => {
            let Some(index) = pick(container, spans.containers.len()) else {
                return bytes.to_vec();
            };
            let (_, _, _, members) = &spans.containers[index];
            let Some(chosen) = pick(member, members.len()) else {
                return bytes.to_vec();
            };
            let (key, _, value_end) = members[chosen];
            let text = &bytes[key..value_end];
            if matches!(mutation, Mutation::DuplicateMember { .. }) {
                let mut with = text.to_vec();
                with.push(b',');
                splice(bytes, key, key, &with)
            } else if members.len() == 1 {
                splice(bytes, key, value_end, b"")
            } else if chosen + 1 < members.len() {
                splice(bytes, key, members[chosen + 1].0, b"")
            } else {
                let previous_end = members[chosen - 1].2;
                splice(bytes, previous_end, value_end, b"")
            }
        }
        Mutation::ToArray {
            container,
            drop,
            extra,
        } => {
            let objects: Vec<_> = spans.containers.iter().filter(|c| c.2 && c.0 > 0).collect();
            let Some(index) = pick(container, objects.len()) else {
                return bytes.to_vec();
            };
            let (start, end, _, members) = objects[index];
            let keep = members.len().saturating_sub(drop);
            let mut values: Vec<&[u8]> = members[..keep]
                .iter()
                .map(|&(_, value_start, value_end)| &bytes[value_start..value_end])
                .collect();
            if extra {
                values.push(b"0");
            }
            let mut with = b"[".to_vec();
            with.extend_from_slice(&values.join(&b","[..]));
            with.push(b']');
            splice(bytes, *start, *end, &with)
        }
        Mutation::Rotate { container, by } => {
            let objects: Vec<_> = spans
                .containers
                .iter()
                .filter(|c| c.2 && c.3.len() > 1)
                .collect();
            let Some(index) = pick(container, objects.len()) else {
                return bytes.to_vec();
            };
            let (start, end, _, members) = objects[index];
            let mut parts: Vec<&[u8]> = members
                .iter()
                .map(|&(key, _, value_end)| &bytes[key..value_end])
                .collect();
            let count = parts.len();
            parts.rotate_left(by % count);
            let mut with = b"{".to_vec();
            with.extend_from_slice(&parts.join(&b","[..]));
            with.push(b'}');
            splice(bytes, *start, *end, &with)
        }
        Mutation::Splice { .. } => unreachable!(),
    }
}
