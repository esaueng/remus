//! Whole-kernel session snapshots (`snapshot`, `fromSnapshot`).
//!
//! A snapshot lets a consumer reopen a session without replaying its
//! modeling history: it carries the whole topology verbatim (every arena
//! slot, retired tombstones and liveness included, the pcurve index, the
//! attributes and the whole journal with its continuity counters) plus the
//! high-water marks of the kernel's own handle namespaces (assemblies,
//! legacy sketches, GCS sketches, checkpoints). A restored kernel resolves
//! every handle as the original did, refuses every stale one, and issues
//! the same handle to the next allocation of every kind.
//!
//! Not carried: checkpoint contents (their IDs stay stale), prepared face
//! moves and blend-group edits, the classification cache and the PERF-Q02
//! cache lineage (a restored topology starts a fresh one, like a new
//! kernel). Live assemblies, legacy sketches and GCS sketches are refused
//! rather than dropped (`snapshot:transient_state`); their counters are
//! carried. The binary format and its checks are documented in
//! `remus_io::snapshot`; `docs/kernel-snapshots.md` is the consumer guide.
//!
//! Tier: core. Restore creates a live session (handles, journal), which no
//! lazy module may own.

#![allow(clippy::missing_errors_doc)]

use remus_io::snapshot::{SessionSnapshot, SnapshotError};
use wasm_bindgen::prelude::*;

use crate::kernel::RemusKernel;
use crate::state::{CheckpointStore, HandleStore};

/// The build identity stamped into snapshots: the kernel crate and its
/// version. A snapshot is restored only by the identical identity; the
/// consumer additionally keys stored snapshots by its package version.
#[must_use]
pub fn build_identity() -> String {
    format!(
        "remus-wasm {} / snapshot format {}",
        env!("CARGO_PKG_VERSION"),
        remus_io::snapshot::SNAPSHOT_FORMAT_VERSION
    )
}

/// Session words in format 1, in order.
const SESSION_WORDS: usize = 4;

/// Why a snapshot was refused. The message starts `snapshot:<code>:`.
#[derive(Debug, thiserror::Error)]
#[error("snapshot:{code}: {detail}", code = self.code(), detail = self.detail())]
pub enum KernelSnapshotError {
    /// The kernel panicked earlier; its state cannot be trusted.
    Poisoned,
    /// Live session state the format does not carry.
    TransientState(String),
    /// A framing, digest, build, decoding, validation or limit refusal.
    Snapshot(#[from] SnapshotError),
}

impl KernelSnapshotError {
    /// Stable machine-readable code, as in the message prefix.
    #[must_use]
    pub const fn code(&self) -> &'static str {
        match self {
            Self::Poisoned => "poisoned",
            Self::TransientState(_) => "transient_state",
            Self::Snapshot(error) => error.code(),
        }
    }

    fn detail(&self) -> String {
        match self {
            Self::Poisoned => {
                "a kernel poisoned by an earlier panic cannot be snapshotted".to_owned()
            }
            Self::TransientState(reason) => reason.clone(),
            Self::Snapshot(error) => error.to_string(),
        }
    }
}

fn invalid(reason: &str) -> KernelSnapshotError {
    KernelSnapshotError::Snapshot(SnapshotError::Invalid {
        reason: reason.to_owned(),
    })
}

impl RemusKernel {
    pub(crate) fn snapshot_impl(&self) -> Result<Vec<u8>, KernelSnapshotError> {
        if self.poisoned {
            return Err(KernelSnapshotError::Poisoned);
        }
        for (kind, live) in [
            ("assemblies", self.assemblies.has_live()),
            ("legacy sketches", self.sketches.has_live()),
            ("GCS sketches", self.gcs_sketches.has_live()),
        ] {
            if live {
                return Err(KernelSnapshotError::TransientState(format!(
                    "the kernel holds live {kind}, which snapshots do not carry; \
                     snapshot a kernel that holds only modeling state"
                )));
            }
        }
        let session = [
            self.assemblies.slot_len() as u64,
            self.sketches.slot_len() as u64,
            self.gcs_sketches.slot_len() as u64,
            u64::from(self.checkpoints.next_id()),
        ];
        Ok(remus_io::snapshot::write_snapshot(
            &self.topo,
            &build_identity(),
            &session,
            remus_io::ImportLimits::default(),
        )?)
    }

    pub(crate) fn from_snapshot_impl(bytes: &[u8]) -> Result<Self, KernelSnapshotError> {
        let SessionSnapshot { topology, session } = remus_io::snapshot::read_snapshot(
            bytes,
            &build_identity(),
            remus_io::ImportLimits::default(),
        )?;
        let [assemblies, sketches, gcs_sketches, next_checkpoint] =
            <[u64; SESSION_WORDS]>::try_from(session.as_slice())
                .map_err(|_| invalid("expected four session words"))?;
        // Every handle namespace is a `u32` space below `u32::MAX`.
        let handles = |count: u64| {
            u32::try_from(count)
                .ok()
                .filter(|count| *count < u32::MAX)
                .map(|count| count as usize)
                .ok_or_else(|| invalid("session handle counter out of range"))
        };
        let assemblies = handles(assemblies)?;
        let sketches = handles(sketches)?;
        let gcs_sketches = handles(gcs_sketches)?;
        let next_checkpoint = u32::try_from(next_checkpoint)
            .map_err(|_| invalid("checkpoint counter out of range"))?;
        let mut kernel = Self::with_module_caches(topology);
        kernel.assemblies = HandleStore::retired(assemblies);
        kernel.sketches = HandleStore::retired(sketches);
        kernel.gcs_sketches = HandleStore::retired(gcs_sketches);
        kernel.checkpoints = CheckpointStore::starting_at(next_checkpoint);
        Ok(kernel)
    }
}

#[wasm_bindgen]
impl RemusKernel {
    /// Snapshot the whole session for a later `RemusKernel.fromSnapshot`.
    ///
    /// Take it after a successful rebuild. The bytes carry the topology
    /// verbatim (retired handles included), the attributes, the whole
    /// journal and the handle counters, stamped with this build's identity
    /// (`RemusKernel.snapshotBuildId()`) and a content digest
    /// (`RemusKernel.snapshotDigest(bytes)`). Checkpoints, prepared drags
    /// and caches are not carried.
    ///
    /// Read-only. Throws `snapshot:poisoned` on a poisoned kernel,
    /// `snapshot:transient_state` while an assembly, legacy sketch or GCS
    /// sketch is live, and `snapshot:limit_exceeded` past the import limits
    /// (256 MiB, 3,000,000 arena slots) that `fromSnapshot` would refuse.
    #[wasm_bindgen(js_name = "snapshot")]
    pub fn snapshot(&self) -> Result<Vec<u8>, JsError> {
        Ok(self.snapshot_impl()?)
    }

    /// Restore a session from `snapshot()` bytes into a new kernel.
    ///
    /// The new kernel answers every handle, journal and evolution query as
    /// the snapshotted one did, and allocates the same next handles. It has
    /// no checkpoints (earlier checkpoint IDs stay stale), prepared drags or
    /// warm per-session caches. All or nothing: a refusal builds no kernel.
    /// Throws `snapshot:<code>: ...` with code `not_a_snapshot`,
    /// `unsupported_format`, `build_mismatch` (written by another build),
    /// `corrupt` (truncated, extended, digest mismatch, undecodable),
    /// `invalid` (inconsistent topology) or `limit_exceeded`. On any of
    /// them, rebuild from the modeling history instead.
    #[wasm_bindgen(js_name = "fromSnapshot")]
    pub fn from_snapshot(bytes: &[u8]) -> Result<Self, JsError> {
        Ok(Self::from_snapshot_impl(bytes)?)
    }

    /// The verified 128-bit content digest of snapshot bytes, as 32 hex
    /// characters, from any build. Store it beside the bytes to detect
    /// storage corruption before loading them.
    #[wasm_bindgen(js_name = "snapshotDigest")]
    pub fn snapshot_digest(bytes: &[u8]) -> Result<String, JsError> {
        remus_io::snapshot::snapshot_digest(bytes)
            .map(remus_io::snapshot::SnapshotDigest::to_hex)
            .map_err(|error| KernelSnapshotError::from(error).into())
    }

    /// The build identity this kernel stamps into snapshots and requires
    /// of the snapshots it restores.
    #[wasm_bindgen(js_name = "snapshotBuildId")]
    #[must_use]
    pub fn snapshot_build_id() -> String {
        build_identity()
    }
}
