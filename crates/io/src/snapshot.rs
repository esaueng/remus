//! Whole-session snapshots: an exact binary image of a [`Topology`] plus a
//! few caller-defined session counters, so a kernel session can be reopened
//! without replaying its modeling history.
//!
//! Unlike the arena documents ([`crate::arena_io`]), a snapshot is not a body
//! exchange format. It copies every arena slot verbatim — retired tombstones
//! and liveness flags included — together with the pcurve use index, the
//! attribute store, the whole journal and the journal-continuity counter
//! (see [`remus_topology::image`]). A restored topology resolves every
//! handle exactly as the source did, refuses every stale one, and gives the
//! next allocation in every arena the same index.
//!
//! # Format (version 1)
//!
//! All integers are little-endian. Every byte before the trailer is
//! covered by the digest.
//!
//! | Bytes | Content |
//! |-------|---------|
//! | 8 | magic `REMUSSNP` |
//! | 4 | format version (`u32`) |
//! | 1 + n | build identity: length `n` (1–255), then UTF-8 |
//! | 8 | payload length (`u64`) |
//! | … | payload |
//! | 16 | digest: FNV-1a 64 of every preceding byte, then a SplitMix64 chain over the same bytes as zero-padded 8-byte words |
//!
//! The payload encodes counts, indices and counters as LEB128 varints and
//! every `f64` as its raw IEEE-754 bits, so values round-trip exactly and
//! the bytes are the same on every platform. Its sections, in order: the
//! session words, the mutation counter, the ten arenas (vertices, edges,
//! wires, faces, shells, solids, compounds, comp-solids, loops, coedges;
//! each a slot count, a liveness bitset and every slot's value), the pcurve
//! use index, solid then face attributes, and the journal (counters, live
//! index, entries with their recorded ticks). The writer is canonical:
//! equal sessions give equal bytes.
//!
//! # Reading
//!
//! [`read_snapshot`] is all-or-nothing and checks, in order: the input size
//! limit, the magic, the format version, the framing (truncation, trailing
//! bytes), the digest, the build identity, the payload encoding, the
//! entity budget and the pcurve-copy budget, and finally the structural
//! invariants of
//! [`Topology::from_image`]. It returns a new topology and never touches an
//! existing one.

use std::hash::Hasher;

use remus_math::det_hash::DetHasher;
use remus_topology::Topology;

use crate::limits::ImportLimits;

mod payload;
#[cfg(test)]
mod tests;

/// The snapshot format version this build reads and writes.
pub const SNAPSHOT_FORMAT_VERSION: u32 = 1;

const MAGIC: &[u8; 8] = b"REMUSSNP";
const DIGEST_BYTES: usize = 16;
const MAX_BUILD_BYTES: usize = 255;
/// Most caller-defined session words a snapshot carries.
pub const MAX_SESSION_WORDS: usize = 64;

/// Why a snapshot could not be written or read. Every refusal leaves the
/// caller's state untouched.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SnapshotError {
    /// The bytes do not start with the snapshot magic.
    #[error("not a Remus session snapshot")]
    NotASnapshot,
    /// The snapshot uses a format version this build does not read.
    #[error("snapshot format {found} is not supported; this build reads format {expected}")]
    UnsupportedFormat {
        /// The version in the snapshot.
        found: u32,
        /// The version this build reads.
        expected: u32,
    },
    /// The snapshot was written by a different build.
    #[error("snapshot was written by {found}, not by this build ({expected})")]
    BuildMismatch {
        /// The build identity in the snapshot.
        found: String,
        /// This build's identity.
        expected: String,
    },
    /// The bytes are truncated, extended, fail the digest, or do not
    /// decode.
    #[error("snapshot is corrupt: {reason}")]
    Corrupt {
        /// What is wrong.
        reason: String,
    },
    /// The bytes decode but describe an inconsistent topology.
    #[error("snapshot is invalid: {reason}")]
    Invalid {
        /// The violated invariant.
        reason: String,
    },
    /// A size or entity budget was exceeded.
    #[error("snapshot limit exceeded for {resource}: {actual} > {limit}")]
    LimitExceeded {
        /// The bounded resource.
        resource: &'static str,
        /// Configured maximum.
        limit: usize,
        /// Observed value.
        actual: usize,
    },
    /// The caller's request cannot be encoded (for example an empty build
    /// identity or too many session words).
    #[error("snapshot request is invalid: {reason}")]
    InvalidRequest {
        /// What is wrong with the request.
        reason: String,
    },
}

impl SnapshotError {
    /// Stable machine-readable code of the refusal.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::NotASnapshot => "not_a_snapshot",
            Self::UnsupportedFormat { .. } => "unsupported_format",
            Self::BuildMismatch { .. } => "build_mismatch",
            Self::Corrupt { .. } => "corrupt",
            Self::Invalid { .. } => "invalid",
            Self::LimitExceeded { .. } => "limit_exceeded",
            Self::InvalidRequest { .. } => "invalid_request",
        }
    }

    fn corrupt(reason: impl Into<String>) -> Self {
        Self::Corrupt {
            reason: reason.into(),
        }
    }
}

/// A snapshot's 128-bit content digest. Probabilistic integrity (it
/// detects corruption and truncation), not a cryptographic signature.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SnapshotDigest(pub [u64; 2]);

impl SnapshotDigest {
    /// Lower-case hex, 32 characters.
    #[must_use]
    pub fn to_hex(self) -> String {
        format!("{:016x}{:016x}", self.0[0], self.0[1])
    }
}

/// A decoded snapshot: the topology and the caller's session words.
#[derive(Debug)]
pub struct SessionSnapshot {
    /// The restored topology (fresh cache and undo lineage).
    pub topology: Topology,
    /// The session words, as written.
    pub session: Vec<u64>,
}

/// Writes a snapshot of `topo` and `session`, stamped with `build`.
///
/// # Errors
///
/// [`SnapshotError::InvalidRequest`] for an empty or over-long build
/// identity or more than [`MAX_SESSION_WORDS`] session words, and
/// [`SnapshotError::LimitExceeded`] when the topology holds more slots than
/// `limits.max_model_entities`, or the encoding or the pcurve copies its
/// decoding makes exceed `limits.max_input_bytes` (a snapshot
/// [`read_snapshot`] would refuse).
pub fn write_snapshot(
    topo: &Topology,
    build: &str,
    session: &[u64],
    limits: ImportLimits,
) -> Result<Vec<u8>, SnapshotError> {
    if build.is_empty() || build.len() > MAX_BUILD_BYTES {
        return Err(SnapshotError::InvalidRequest {
            reason: format!("build identity must be 1 to {MAX_BUILD_BYTES} bytes"),
        });
    }
    if session.len() > MAX_SESSION_WORDS {
        return Err(SnapshotError::InvalidRequest {
            reason: format!("at most {MAX_SESSION_WORDS} session words"),
        });
    }
    let view = topo.image_view();
    let slots = payload::slot_count(&view);
    if slots > limits.max_model_entities {
        return Err(SnapshotError::LimitExceeded {
            resource: "snapshot entities",
            limit: limits.max_model_entities,
            actual: slots,
        });
    }
    let (body, pcurve_copies) = payload::encode(&view, session);
    if pcurve_copies > limits.max_input_bytes {
        return Err(SnapshotError::LimitExceeded {
            resource: "snapshot pcurve copies",
            limit: limits.max_input_bytes,
            actual: pcurve_copies,
        });
    }
    let mut bytes = Vec::with_capacity(body.len() + build.len() + 40);
    bytes.extend_from_slice(MAGIC);
    bytes.extend_from_slice(&SNAPSHOT_FORMAT_VERSION.to_le_bytes());
    // The length fits a byte: checked above.
    bytes.push(u8::try_from(build.len()).unwrap_or(u8::MAX));
    bytes.extend_from_slice(build.as_bytes());
    bytes.extend_from_slice(&(body.len() as u64).to_le_bytes());
    bytes.extend_from_slice(&body);
    drop(body);
    let digest = digest_of(&bytes);
    bytes.extend_from_slice(&digest.0[0].to_le_bytes());
    bytes.extend_from_slice(&digest.0[1].to_le_bytes());
    if bytes.len() > limits.max_input_bytes {
        return Err(SnapshotError::LimitExceeded {
            resource: "snapshot bytes",
            limit: limits.max_input_bytes,
            actual: bytes.len(),
        });
    }
    Ok(bytes)
}

/// Reads a snapshot written by the build `build`; see the module docs for
/// the order of checks.
///
/// # Errors
///
/// Any [`SnapshotError`] but `InvalidRequest`; nothing is built on error.
pub fn read_snapshot(
    bytes: &[u8],
    build: &str,
    limits: ImportLimits,
) -> Result<SessionSnapshot, SnapshotError> {
    let frame = frame(bytes, limits)?;
    if frame.build != build.as_bytes() {
        return Err(SnapshotError::BuildMismatch {
            found: String::from_utf8_lossy(frame.build).into_owned(),
            expected: build.to_owned(),
        });
    }
    let (image, session) = payload::decode(frame.payload, limits)?;
    let topology = Topology::from_image(image).map_err(|error| SnapshotError::Invalid {
        reason: error.reason,
    })?;
    Ok(SessionSnapshot { topology, session })
}

/// The verified digest of a snapshot, from any build: checks the magic,
/// the format version, the framing and the digest, not the build identity
/// or the payload.
///
/// # Errors
///
/// [`SnapshotError::NotASnapshot`], [`SnapshotError::UnsupportedFormat`],
/// [`SnapshotError::Corrupt`] or [`SnapshotError::LimitExceeded`].
pub fn snapshot_digest(bytes: &[u8]) -> Result<SnapshotDigest, SnapshotError> {
    frame(bytes, ImportLimits::default()).map(|frame| frame.digest)
}

/// The build identity a snapshot was written by, after the same checks as
/// [`snapshot_digest`].
///
/// # Errors
///
/// As [`snapshot_digest`], or [`SnapshotError::Corrupt`] for a non-UTF-8
/// identity.
pub fn snapshot_build(bytes: &[u8]) -> Result<String, SnapshotError> {
    let frame = frame(bytes, ImportLimits::default())?;
    String::from_utf8(frame.build.to_vec())
        .map_err(|_| SnapshotError::corrupt("build identity is not UTF-8"))
}

/// The framed parts of a snapshot whose digest has been verified.
struct Frame<'a> {
    build: &'a [u8],
    payload: &'a [u8],
    digest: SnapshotDigest,
}

fn frame(bytes: &[u8], limits: ImportLimits) -> Result<Frame<'_>, SnapshotError> {
    if bytes.len() > limits.max_input_bytes {
        return Err(SnapshotError::LimitExceeded {
            resource: "snapshot bytes",
            limit: limits.max_input_bytes,
            actual: bytes.len(),
        });
    }
    if bytes.get(..MAGIC.len()) != Some(MAGIC.as_slice()) {
        return Err(SnapshotError::NotASnapshot);
    }
    let truncated = || SnapshotError::corrupt(format!("truncated at {} bytes", bytes.len()));
    let version = bytes
        .get(8..12)
        .and_then(|word| <[u8; 4]>::try_from(word).ok())
        .map(u32::from_le_bytes)
        .ok_or_else(truncated)?;
    if version != SNAPSHOT_FORMAT_VERSION {
        return Err(SnapshotError::UnsupportedFormat {
            found: version,
            expected: SNAPSHOT_FORMAT_VERSION,
        });
    }
    let build_len = usize::from(*bytes.get(12).ok_or_else(truncated)?);
    if build_len == 0 {
        return Err(SnapshotError::corrupt("empty build identity"));
    }
    let build_end = 13 + build_len;
    let build = bytes.get(13..build_end).ok_or_else(truncated)?;
    let payload_len = bytes
        .get(build_end..build_end + 8)
        .and_then(|word| <[u8; 8]>::try_from(word).ok())
        .map(u64::from_le_bytes)
        .ok_or_else(truncated)?;
    let payload_start = build_end + 8;
    let expected = usize::try_from(payload_len)
        .ok()
        .and_then(|len| len.checked_add(payload_start))
        .and_then(|len| len.checked_add(DIGEST_BYTES))
        .ok_or_else(|| SnapshotError::corrupt("payload length overflows"))?;
    if bytes.len() < expected {
        return Err(SnapshotError::corrupt(format!(
            "truncated: {} of {expected} bytes",
            bytes.len()
        )));
    }
    if bytes.len() > expected {
        return Err(SnapshotError::corrupt(format!(
            "{} trailing bytes after the digest",
            bytes.len() - expected
        )));
    }
    let content_end = expected - DIGEST_BYTES;
    let (content, trailer) = bytes.split_at(content_end);
    let stored = SnapshotDigest([
        u64::from_le_bytes(trailer[..8].try_into().unwrap_or_default()),
        u64::from_le_bytes(trailer[8..].try_into().unwrap_or_default()),
    ]);
    let digest = digest_of(content);
    if digest != stored {
        return Err(SnapshotError::corrupt("digest mismatch"));
    }
    Ok(Frame {
        build,
        payload: &content[payload_start..],
        digest,
    })
}

/// FNV-1a 64 over the bytes, and a SplitMix64 chain over them as
/// zero-padded little-endian words seeded with their length: two
/// independent 64-bit hashes, platform independent.
fn digest_of(content: &[u8]) -> SnapshotDigest {
    let mut fnv = DetHasher::new();
    fnv.write(content);
    let mut mixed = 0x243f_6a88_85a3_08d3_u64 ^ content.len() as u64;
    let mut words = content.chunks_exact(8);
    for word in &mut words {
        let mut buffer = [0u8; 8];
        buffer.copy_from_slice(word);
        mixed = splitmix64(mixed ^ u64::from_le_bytes(buffer));
    }
    let tail = words.remainder();
    if !tail.is_empty() {
        let mut buffer = [0u8; 8];
        buffer[..tail.len()].copy_from_slice(tail);
        mixed = splitmix64(mixed ^ u64::from_le_bytes(buffer));
    }
    SnapshotDigest([fnv.finish(), mixed])
}

const fn splitmix64(state: u64) -> u64 {
    let mut z = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}
