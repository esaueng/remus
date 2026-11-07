//! Prepared blend-group edits: live preview of a band radius or boss
//! diameter drag on a blended body.
//!
//! A group edit ([`super::resize_band_through_group`],
//! [`super::resize_cylinder_through_group`]) spends most of its time on work
//! that does not depend on the dragged value: recognizing the group (three
//! times: the route, the removal's own check and the identity carry), removing
//! it to its sharp parent, and the per-band or partial-turn attempt that
//! refuses before the group route is tried. [`PreparedGroupEdit`] does that
//! once. Each frame then clones the private arena holding the sharp parent,
//! makes the exact sharp edit and re-blends the group in the planned order.
//!
//! # Frames and commits
//!
//! - [`PreparedGroupEdit::preview`] re-blends with display-only blends
//!   ([`crate::blend_ops::set_display_only_blends`]): every stage keeps its
//!   structural checks (each named edge rounded, a closed outer shell, no
//!   validation error the input did not have) and the finished body must
//!   have a closed outer shell and pass strict validation, but the blend
//!   volume oracles, the convex material guard and the group's mesh and
//!   volume postconditions are skipped. On the Gentle-Gecko bodies those
//!   measuring checks are nearly all of an edit's time. A frame is a display
//!   result; keep a value with a commit.
//! - [`PreparedGroupEdit::commit`] and
//!   `journal_ops::commit_prepared_group_edit_journaled` are the unprepared
//!   [`crate::resize_blend::resize_blend`] /
//!   [`crate::push_pull::resize_cylindrical_face`] and their journaled
//!   variants, with the group route served from the preparation: the same
//!   per-band or partial-turn attempt first, then the group route under every
//!   guard. They return the same result content and history, or the same
//!   refusal, as the unprepared call.
//!   For the last previewed value, an immutable private candidate can instead
//!   complete the measuring guards omitted by the frame and be copied out.
//!   Structural validation already passed on that exact candidate. Changed
//!   values, joined caps, chamfer groups, and ambiguous speculative branches
//!   use the full route. Journaled commits retain the initial local attempt
//!   and normal transaction so operation IDs and inferred history agree too.
//!
//! # Learned refusal
//!
//! The unprepared calls try the per-band path (a band) or the partial-turn
//! path (a boss wall) first and reach the group route only when it refuses.
//! Preparation runs that attempt once, on a private copy of the solid, at a
//! probe value. A refusal that does not depend on the value — the band's
//! description or removal refuses (a freeform neighbour, a sampled corner at
//! its end), or the wall borders a face the partial-turn move cannot
//! re-intersect — is learned, and frames go straight to the group route.
//! Otherwise every frame runs that attempt first, as the unprepared call
//! does. A frame at a value the group route never takes (the band's current
//! radius, a non-positive or non-finite value, a wall's current radius)
//! runs the unprepared call.
//!
//! # Staleness
//!
//! As for [`crate::prepared_move::PreparedFaceMove`]: preparation records the
//! source's content key, and every call refuses with an `invalid input:
//! prepared group edit is stale …` error, editing nothing, when the solid
//! changed since. A checkpoint restore that brings the same content back
//! under the same handles keeps the preparation valid.
//!
//! WASM module tier: `core` (session handles that edit the live topology;
//! `prepareResizeBlend`, `prepareResizeCylindricalFace`).

use std::cell::Cell;
use std::sync::{Mutex, MutexGuard, PoisonError};

use remus_math::tolerance::Tolerance;
use remus_offset::{OffsetError, UnsupportedMoveCause};
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use super::edit::{
    Built, SharpGroup, band_group, build_band, build_cylinder, check_band_radius, copy_out,
    cylinder_group,
};
use super::reblend::Gate;
use super::{BlendCluster, BlendClusterError, GroupEdit, recognize_blend_cluster};
use crate::OperationsError;
use crate::measure::volume_memo::{ContentKey, solid_move_key};
use crate::resize_blend::{ResizeBlendError, ResizeBlendResult};

/// Relative change of the probe value the learned-refusal check uses.
const BAND_PROBE: f64 = 0.75;
const WALL_PROBE: f64 = 1.05;

/// What a prepared group edit changes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GroupEditKind {
    /// A band's radius ([`crate::resize_blend::resize_blend`]); the value is
    /// the new radius.
    BandRadius {
        /// The band's current radius (the `expected_radius` witness).
        expected_radius: f64,
    },
    /// A cylindrical support's radius
    /// ([`crate::push_pull::resize_cylindrical_face`]); the value is the new
    /// radius.
    CylinderRadius {
        /// The wall's current radius.
        radius: f64,
    },
}

#[derive(Clone, Copy)]
enum Target {
    Band { member: usize },
    Wall { wall: FaceId },
}

/// A blend-group edit of one face of one source solid, prepared for
/// repeated values. See the module docs.
pub struct PreparedGroupEdit {
    solid: SolidId,
    face: FaceId,
    kind: GroupEditKind,
    source: ContentKey,
    group: SharpGroup,
    target: Target,
    /// The group recognized on the source, for the identity carry; absent
    /// when the source's cap was lowered into its shoulder (its group is
    /// recognized on the lifted copy only).
    before: Option<BlendCluster>,
    /// Whether the attempt in front of the group route refuses every value.
    learned: bool,
    /// Last private frame, independent of caller handles and checkpoint restores.
    preview: Mutex<Option<CachedPreview>>,
}

struct CachedPreview {
    value: u64,
    work: Topology,
    built: Built,
    checks: crate::blend_ops::preview_checks::Checks,
}

impl std::fmt::Debug for PreparedGroupEdit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PreparedGroupEdit")
            .field("solid", &self.solid)
            .field("face", &self.face)
            .field("kind", &self.kind)
            .field("members", &self.group.cluster.members.len())
            .field("learned_refusal", &self.learned)
            .finish_non_exhaustive()
    }
}

impl PreparedGroupEdit {
    /// Prepares radius edits of the band containing `face`. Reads the
    /// topology only.
    ///
    /// # Errors
    ///
    /// [`ResizeBlendError::RadiusMismatch`] for a wrong witness, and
    /// [`BlendClusterError::NotPrepared`] when `face` is not a band of a
    /// recognized group that removes to sharp (the unprepared
    /// [`crate::resize_blend::resize_blend`] answers those). Topology read
    /// errors propagate.
    pub fn prepare_band(
        topo: &Topology,
        solid: SolidId,
        face: FaceId,
        expected_radius: f64,
    ) -> Result<Self, OperationsError> {
        let _span = crate::performance::span("blend_group.prepare");
        let source = content_key(topo, solid)?;
        if !expected_radius.is_finite() || expected_radius <= 0.0 {
            return Err(not_prepared("expected_radius must be finite and positive"));
        }
        let Some((group, member)) = band_group(topo, solid, face, expected_radius, None)? else {
            return Err(not_prepared(
                "the face is not a band of a recognized blend group that removes to sharp",
            ));
        };
        let before = recognize_blend_cluster(topo, solid, face).ok();
        let learned = probe_band(topo, solid, face, expected_radius)?;
        Ok(Self {
            solid,
            face,
            kind: GroupEditKind::BandRadius { expected_radius },
            source,
            group,
            target: Target::Band { member },
            before,
            learned,
            preview: Mutex::new(None),
        })
    }

    /// Prepares radius edits of the cylindrical support `face`. Reads the
    /// topology only.
    ///
    /// # Errors
    ///
    /// [`BlendClusterError::NotPrepared`] when `face` is not a cylinder
    /// carrying a recognized group that removes to sharp (the unprepared
    /// [`crate::push_pull::resize_cylindrical_face`] answers those).
    /// Topology read errors propagate.
    pub fn prepare_cylinder(
        topo: &Topology,
        solid: SolidId,
        face: FaceId,
    ) -> Result<Self, OperationsError> {
        let _span = crate::performance::span("blend_group.prepare");
        let source = content_key(topo, solid)?;
        let FaceSurface::Cylinder(cylinder) = topo.face(face)?.surface() else {
            return Err(not_prepared("the face is not cylindrical"));
        };
        let radius = cylinder.radius();
        let Some((group, wall)) = cylinder_group(topo, solid, face)? else {
            return Err(not_prepared(
                "the face carries no recognized blend group that removes to sharp",
            ));
        };
        let before = recognize_blend_cluster(topo, solid, face).ok();
        let learned = probe_cylinder(topo, solid, face, radius)?;
        Ok(Self {
            solid,
            face,
            kind: GroupEditKind::CylinderRadius { radius },
            source,
            group,
            target: Target::Wall { wall },
            before,
            learned,
            preview: Mutex::new(None),
        })
    }

    /// The prepared source solid.
    #[must_use]
    pub const fn solid(&self) -> SolidId {
        self.solid
    }

    /// The prepared face.
    #[must_use]
    pub const fn face(&self) -> FaceId {
        self.face
    }

    /// What the preparation edits.
    #[must_use]
    pub const fn kind(&self) -> GroupEditKind {
        self.kind
    }

    /// Whether the attempt in front of the group route was learned to refuse
    /// every value, so frames skip it.
    #[must_use]
    pub const fn learned_refusal(&self) -> bool {
        self.learned
    }

    /// Whether `topo` still holds the prepared source, entity for entity.
    #[must_use]
    pub fn is_current(&self, topo: &Topology) -> bool {
        solid_move_key(topo, self.solid).is_some_and(|key| key == self.source)
    }

    /// A live preview frame at `value`: the result solid, copied into `topo`.
    ///
    /// The geometry is the group route's for `value`, built with display-only
    /// blends under the frame gate (see the module docs); commit the
    /// value to keep it.
    ///
    /// # Errors
    ///
    /// A stale-source [`OperationsError::InvalidInput`], or the refusal of the
    /// edit at `value` (typed as the unprepared call types it). The topology
    /// is unchanged on every error.
    pub fn preview(&self, topo: &mut Topology, value: f64) -> Result<SolidId, OperationsError> {
        let _span = crate::performance::span("blend_group.preview");
        self.require_current(topo)?;
        self.cached_preview().take();
        if self.learned && self.group_value(topo, value) {
            return self
                .group_edit(topo, value, Gate::Preview)
                .map(|edit| edit.solid);
        }
        self.unprepared(topo, value, Gate::Preview)
    }

    /// The edit at `value`, kept: what [`crate::resize_blend::resize_blend`]
    /// or [`crate::push_pull::resize_cylindrical_face`] returns for the
    /// prepared source, face and `value`, under every guard.
    ///
    /// # Errors
    ///
    /// A stale-source [`OperationsError::InvalidInput`], or exactly the
    /// unprepared call's refusal. The topology is unchanged on every error.
    pub fn commit(&self, topo: &mut Topology, value: f64) -> Result<SolidId, OperationsError> {
        let _span = crate::performance::span("blend_group.commit");
        self.require_current(topo)?;
        if let Some(edit) = self.cached_group_edit(topo, value)? {
            return Ok(edit.solid);
        }
        self.unprepared(topo, value, Gate::Full)
    }

    /// Adopt only a private, fully checked frame. The caller supplies the
    /// journal transaction, so no preview handles or history are retained.
    pub(crate) fn cached_group_edit(
        &self,
        topo: &mut Topology,
        value: f64,
    ) -> Result<Option<GroupEdit>, OperationsError> {
        let preview = self.cached_preview();
        let Some(cached) = preview
            .as_ref()
            .filter(|cached| cached.value == value.to_bits())
        else {
            return Ok(None);
        };
        if crate::blend_ops::display_only_blends() || cached.checks.validate().is_err() {
            return Ok(None);
        }
        remus_blend::governor::checkpoint()?;
        let _span = crate::performance::span("blend_group.adopt_preview");
        let result = copy_out(
            topo,
            self.solid,
            &cached.work,
            &self.group.inputs,
            &cached.built,
            self.before.as_ref(),
        )
        .map(Some);
        drop(preview);
        result
    }

    // Keep the preparation Send + Sync for native callers. A panic cannot
    // certify a candidate: checks mark success only after every guard passed.
    fn cached_preview(&self) -> MutexGuard<'_, Option<CachedPreview>> {
        self.preview.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// The group route at `value` under every guard, as the band route of
    /// the journaled commit (`journal_ops::commit_prepared_group_edit_journaled`)
    /// takes it.
    pub(crate) fn committed_band_route(
        &self,
        topo: &mut Topology,
        value: f64,
    ) -> Result<Option<ResizeBlendResult>, OperationsError> {
        if let Some(edit) = self.cached_group_edit(topo, value)? {
            return Ok(Some(ResizeBlendResult {
                solid: edit.solid,
                evolution: edit.evolution,
            }));
        }
        self.band_route(topo, value, Gate::Full)
    }

    /// The group route at `value` under every guard, as the cylinder route
    /// of the journaled commit takes it.
    pub(crate) fn committed_group_edit(
        &self,
        topo: &mut Topology,
        value: f64,
    ) -> Result<Option<GroupEdit>, OperationsError> {
        if let Some(edit) = self.cached_group_edit(topo, value)? {
            return Ok(Some(edit));
        }
        self.group_edit(topo, value, Gate::Full).map(Some)
    }

    /// Refuses, editing nothing, when the source changed since preparation.
    pub(crate) fn require_current(&self, topo: &Topology) -> Result<(), OperationsError> {
        if self.is_current(topo) {
            Ok(())
        } else {
            Err(OperationsError::InvalidInput {
                reason: format!(
                    "prepared group edit is stale: solid {} changed since it was prepared",
                    self.solid.index()
                ),
            })
        }
    }

    /// Whether the unprepared call reaches the group route at `value` when
    /// the attempt in front of it refuses: a value that edits.
    fn group_value(&self, topo: &Topology, value: f64) -> bool {
        let tolerance = Tolerance::new();
        match self.kind {
            GroupEditKind::BandRadius { expected_radius } => {
                value.is_finite()
                    && value > tolerance.linear
                    && !tolerance.approx_eq(value, expected_radius)
            }
            GroupEditKind::CylinderRadius { .. } => {
                crate::push_pull::validate_cylindrical_resize(topo, self.solid, self.face, value)
                    .is_ok()
            }
        }
    }

    /// The unprepared call at `value`, its group route served from the
    /// preparation under `gate`.
    fn unprepared(
        &self,
        topo: &mut Topology,
        value: f64,
        gate: Gate,
    ) -> Result<SolidId, OperationsError> {
        let (solid, face) = (self.solid, self.face);
        match self.kind {
            GroupEditKind::BandRadius { expected_radius } => {
                crate::resize_blend::resize_blend_with(
                    topo,
                    solid,
                    face,
                    expected_radius,
                    value,
                    |topo| self.band_route(topo, value, gate),
                )
                .map(|result| result.solid)
            }
            GroupEditKind::CylinderRadius { .. } => {
                crate::push_pull::resize_cylindrical_face_with(topo, solid, face, value, |topo| {
                    self.group_edit(topo, value, gate).map(Some)
                })
            }
        }
    }

    /// [`super::resize_band_through_group`]'s answer at a value the group
    /// route takes, from the preparation.
    fn band_route(
        &self,
        topo: &mut Topology,
        value: f64,
        gate: Gate,
    ) -> Result<Option<ResizeBlendResult>, OperationsError> {
        check_band_radius(value)?;
        let edit = self.group_edit(topo, value, gate)?;
        Ok(Some(ResizeBlendResult {
            solid: edit.solid,
            evolution: edit.evolution,
        }))
    }

    /// The group route at `value` from the preparation, copied into `topo`.
    fn group_edit(
        &self,
        topo: &mut Topology,
        value: f64,
        gate: Gate,
    ) -> Result<GroupEdit, OperationsError> {
        let work = self.group.work.clone();
        let build = || match self.target {
            Target::Band { member } => build_band(&self.group, work, member, value, gate),
            Target::Wall { wall } => build_cylinder(&self.group, work, wall, value, gate),
        };
        let (built, checks) = if gate == Gate::Preview
            && self.learned
            && self.group.lift.is_none()
            && self
                .group
                .cluster
                .members
                .iter()
                .all(|member| !member.kind.is_chamfer())
        {
            crate::blend_ops::preview_checks::capture(build)
        } else {
            (build(), None)
        };
        let (work, built) = built?;
        let result = copy_out(
            topo,
            self.solid,
            &work,
            &self.group.inputs,
            &built,
            self.before.as_ref(),
        )?;
        if let Some(checks) = checks {
            *self.cached_preview() = Some(CachedPreview {
                value: value.to_bits(),
                work,
                built,
                checks,
            });
        }
        Ok(result)
    }
}

fn content_key(topo: &Topology, solid: SolidId) -> Result<ContentKey, OperationsError> {
    topo.solid(solid)?;
    solid_move_key(topo, solid).ok_or_else(|| OperationsError::InvalidInput {
        reason: format!("solid {} has missing entities", solid.index()),
    })
}

fn not_prepared(reason: impl Into<String>) -> OperationsError {
    BlendClusterError::NotPrepared {
        reason: reason.into(),
    }
    .into()
}

/// `solid` copied into a scratch arena, with `face`'s copy.
fn scratch_copy(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
) -> Result<Option<(Topology, SolidId, FaceId)>, OperationsError> {
    let mut scratch = Topology::new();
    let copied = crate::copy::copy_solid_between_with_entity_map(topo, &mut scratch, solid)?;
    Ok(copied
        .face_map
        .get(&face.index())
        .copied()
        .map(|face| (scratch, copied.solid, face)))
}

/// Whether the per-band path refuses at a probe radius for a reason no
/// radius changes (the band's description or removal), on a scratch copy.
fn probe_band(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    expected_radius: f64,
) -> Result<bool, OperationsError> {
    let _span = crate::performance::span("blend_group.probe");
    let Some((mut scratch, solid, face)) = scratch_copy(topo, solid, face)? else {
        return Ok(false);
    };
    let probe = expected_radius * BAND_PROBE;
    let reached = Cell::new(false);
    let refusal = crate::resize_blend::resize_blend_with(
        &mut scratch,
        solid,
        face,
        expected_radius,
        probe,
        |_| {
            reached.set(true);
            Ok(None)
        },
    );
    Ok(reached.get()
        && refusal.as_ref().is_err_and(|error| {
            !matches!(
                error,
                OperationsError::ResizeBlend(ResizeBlendError::ReconstructionFailed { .. })
            )
        }))
}

/// Whether the attempts in front of the group route refuse a probe radius
/// for a reason no radius changes (the partial-turn move meets a neighbour
/// it cannot re-intersect), on a scratch copy.
fn probe_cylinder(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    radius: f64,
) -> Result<bool, OperationsError> {
    let _span = crate::performance::span("blend_group.probe");
    let Some((mut scratch, solid, face)) = scratch_copy(topo, solid, face)? else {
        return Ok(false);
    };
    let probe = radius * WALL_PROBE;
    let reached = Cell::new(false);
    let refusal =
        crate::push_pull::resize_cylindrical_face_with(&mut scratch, solid, face, probe, |_| {
            reached.set(true);
            Ok(None)
        });
    Ok(reached.get()
        && matches!(
            refusal,
            Err(OperationsError::Offset(OffsetError::UnsupportedMoveFace {
                cause: UnsupportedMoveCause::UnsupportedNeighbor { .. },
                ..
            }))
        ))
}
