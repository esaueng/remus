//! B17 healing defect-class qualification matrix — first slice.
//!
//! Roadmap row B17 (`docs/kernel-maturity/roadmap.md`) asks for a generated
//! defect class x severity x repair policy x scale matrix per fixer, where
//! every cell ends in either a verified repair (both validators clean,
//! closed-form volume, census) or a typed refusal. The exit also names the
//! faceted-import sew/unify contract (issue #244) and an operand
//! self-interference report; both stay open after this slice.
//!
//! Scope of THIS slice: the solid-scoped `fix_shape` pipeline driven with
//! single-fixer policies over a known-good box at 1e-3 / 1 / 1e3, plus the
//! `fix_wireframe` pipeline op for sewing. Each covered defect class gets a
//! generated `(severity, policy, scale)` cell matrix on the verified wrappers
//! (`fix_shape_verified` / `run_heal_pipeline_verified`, which commit only a
//! fully disclosed result accepted by both validators), with an exact-repair
//! oracle (dual `validate_solid` clean, closed-form volume, entity + surface
//! census) or a pinned typed refusal (`HealingRepairRefused`,
//! `ConfiguredHealingValidationFailed`, ...).
//!
//! Defect classes covered here (one test per fixer family):
//!
//! - wire order (`fix_reorder`): swapped outer-wire edge order on one face
//! - wire closure (`fix_closure`): opened closure joint
//! - wire gaps (`fix_connectivity`, On policy): mid-wire vertex split
//! - small edges (`fix_small_edges`): one short edge spliced into one face
//! - face orientation (`fix_wire_orientation`): one face plane normal flipped
//! - small faces (solid `fix_small_faces`): one sliver face appended
//! - shell orientation (`fix_orientation`): one face's `reversed` flag
//!   toggled (face 1; face 0 is a documented non-repair — surprise 4)
//! - sewing / free bounds (`fix_wireframe` pipeline op): disjoint-cube shell
//!   sewn shut (plus the `fix_shape` typed-refusal pin)
//! - duplicate faces (`fix_duplicate_faces`): one exact geometric copy
//!   appended (plus the specified keep-behaviors)
//!
//! Deliberately NOT covered here (open B17 remainder): seams
//! (`fix_missing_seam` is detection-only and returns a typed refusal by
//! design), continuity splits, representation conversion, the #244 faceted
//! sew/unify contract, and the operand self-interference report. Those need
//! their own fixtures and stay open in the roadmap row.
//!
//! SURPRISES (first-of-kind heal findings; heal has no distilled campaign
//! knowledge, so everything below is recorded here and in
//! `docs/kernel-maturity/b17-heal-matrix-note.md`):
//!
//! 1. `fix_duplicate_faces` is NOT centroid/normal/edge-count-only anymore.
//!    The roadmap trap note (and the roadmap skill) describe the old
//!    comparator; the current `heal/src/fix/solid.rs` compares effective
//!    plane normals (1 - cos < 1e-6) plus ordered outer-boundary vertices
//!    with the same winding (cyclic shift allowed), planar line-bounded
//!    faces only. Same-centroid/different-boundary and opposite-winding
//!    copies are kept. The matrix pins both keeps as specified behavior.
//! 2. Same-winding duplicate + reversed-flag duplicate are BOTH kept, for
//!    different reasons: same-winding fails the boundary-coincidence test,
//!    reversed-flag fails the effective-normal test. Either alone keeps the
//!    pair; the matrix pins both.
//! 3. `fix_wireframe` (sewing) is OFF the `fix_shape` path: no
//!    `fix_wire`/`fix_face`/`fix_shell`/`fix_solid` step consults
//!    `fix_wireframe` (`wireframe.rs` is only reachable via the
//!    `fix_wireframe` pipeline op). A disjoint shell run through `fix_shape`
//!    therefore ends in a typed validation refusal, never a sewn repair. The
//!    sewing matrix goes through `run_heal_pipeline_verified` instead, and
//!    pins the `fix_shape` refusal so a future wiring change is a visible
//!    matrix flip, not silent drift.
//! 4. Face-orientation vs shell-orientation fixers repair DIFFERENT defects,
//!    and the shell BFS is seed-sensitive. Flipping a face's stored plane
//!    normal keeps shell orientation analysis consistent (shared-edge senses
//!    are untouched) and only the winding check warns, so exactly
//!    `fix_wire_orientation` repairs it. Toggling a face's `reversed` flag
//!    breaks shared-edge senses while leaving wire connectivity intact, so
//!    exactly `fix_orientation` repairs it — but only with On (Auto gates on
//!    the analysis, which reads the raw wire flag and stays consistent, so
//!    Auto reports no issue): face 1 toggled repairs exactly under On with 1
//!    disclosed flip; face 0 toggled (edges all-reversed natively) refuses
//!    under every policy (the BFS cascade flips 5 faces into a still-invalid
//!    shell). Reversing wire flags instead breaks connectivity too and needs
//!    wire fixers on top (probed during design, not used here). The matrix
//!    injects flag toggling on face 1 with On-repair vs Auto/Off-refusal, and
//!    pins face 0 as a typed-refusal cell.
//! 5. A swapped wire order is BOTH a reorder and a gap defect: the reorder
//!    pass restores the chain (disclosed `WireReordered`), then the gap pass
//!    re-merges the vertices. Reorder-only Auto commits exactly; enabling the
//!    gap fixers On as well makes `fix_gaps_3d`'s widened second pass veto
//!    with `ClosureGapTooLarge` (the nominal pass already closed the ~3.6
//!    joints, the residual analysis still reports them, and the widened retry
//!    computes a sub-nominal band and declines). The matrix pins Auto-repair
//!    vs On-refusal vs Off-refusal for the swap family.
//! 6. At-tolerance closure-gap behavior splits by policy at 1e3: the stored
//!    1e-7 rounds to ~1.0000008e-7, a hair above the `dist <= linear` closure
//!    gate — Auto still commits exactly (the nominal merge is unconditional
//!    once the gate passes), while On's widened `fix_gaps_3d` retry lands
//!    at/below nominal and declines typed (`ClosureGapTooLarge`). Below 1e3
//!    both repair. Never a silent wrong closure.
//! 7. The 1e-6 "super-tolerance" splice is NOT a small-edge defect at all:
//!    analysis reports zero gaps AND zero small edges (the 1e-6 edge is a
//!    full-length edge at that scale), but the splice still breaks wire
//!    connectivity (`WireNotConnected` + `WireClosure3D` + `ShellClosed` +
//!    self-intersection), so every policy refuses typed. The matrix pins the
//!    sub-tol repair (small-edge-only Auto removes it, connectivity-clean)
//!    and the super-tol refusal separately, with the volume oracle relaxed on
//!    the 1e-3 sub-tol cell (the 5e-8 splice itself moves ~8e-6 relative at
//!    that scale — repair is exact, the oracle is coarser than the defect).
//! 8. Gap closing on a mid-wire split is On-only: Auto gates the merge on
//!    `analyze_wire` gaps measured against nominal tolerance, but a split
//!    vertex pair keeps distinct IDs at coincident positions, so analysis
//!    reports ordered/zero-gaps and Auto reports zero repairs (verified
//!    wrapper refuses, actions empty) while On merges unconditionally and
//!    commits exactly. The gap matrix pins On-repair vs Auto/Off-refusal.
//!    (Closure joints differ: `fix_closed` measures its own distance, so Auto
//!    repairs there.)
//! 9. Severity for gaps/closure/small-edge injection is absolute, not
//!    scale-relative: the heal linear tolerance is 1e-7 at every scale, so a
//!    5e-8 gap behaves the same at 1e-3 as at 1e3. The matrix still runs all
//!    three scales to pin that invariance (and the volume oracle scales with
//!    the box; repaired cells skip the volume pin at 1e-3 where the 5e-8 move
//!    is ~8e-6 relative, keeping validity + census everywhere and the full
//!    pin at 1 and 1e3).
//! 10. `fix_shape_verified` fails CLOSED on `Off`-policy cells whose defect
//!     breaks validation: the defect survives, validation vetoes the commit
//!     (`ConfiguredHealingValidationFailed`), and topology rolls back. `Off`
//!     is therefore a typed-refusal cell wherever the defect is invalid. The
//!     flipped-normal defect is validator-clean at L3 (only the check-crate
//!     winding warning fires), so its Off cell COMMITS the unchanged solid —
//!     asserted explicitly as a no-op commit, not a refusal.
//!
//! KERNEL BUGS found while building this matrix (roadmap rule: new §B rows,
//! not fixes in this PR): none. Every surprise above is a behavior
//! characterization, not a defect: each cell either repairs exactly or
//! refuses typed. One analysis-level observation is recorded in the note for
//! a future row (shell analysis orientation-consistency is blind to the face
//! `reversed` flag), but it has no failing repro on its own and is not filed
//! as a §B row here.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::BTreeMap;

use remus_heal::fix::{FixConfig, FixMode};
use remus_heal::pipeline::process::HealProcess;
use remus_math::vec::{Point3, Vec3};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{solid_entity_counts, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

const SCALES: [f64; 3] = [1e-3, 1.0, 1e3];
/// `fix_shape_verified` tolerance passed per scale (absolute, like the kernel).
const TOL: f64 = 1e-7;
const REL_VOL: f64 = 1e-6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Policy {
    Auto,
    On,
    Off,
}

impl Policy {
    const ALL: [Self; 3] = [Self::Auto, Self::On, Self::Off];
    const fn label(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::On => "on",
            Self::Off => "off",
        }
    }
}

fn config_with(policy: Policy, set: impl Fn(&mut FixConfig, FixMode)) -> FixConfig {
    let mut config = off_config();
    let mode = match policy {
        Policy::Auto => FixMode::Auto,
        Policy::On => FixMode::On,
        Policy::Off => FixMode::Off,
    };
    set(&mut config, mode);
    config
}

/// Every fixer off: each matrix test enables exactly its own fixer family so
/// a green cell proves THAT fixer repaired the defect (no cross-fixer credit).
fn off_config() -> FixConfig {
    FixConfig {
        fix_reorder: FixMode::Off,
        fix_connectivity: FixMode::Off,
        fix_closure: FixMode::Off,
        fix_small_edges: FixMode::Off,
        fix_self_intersection: FixMode::Off,
        fix_degenerate_edges: FixMode::Off,
        fix_gaps_2d: FixMode::Off,
        fix_gaps_3d: FixMode::Off,
        fix_lacking: FixMode::Off,
        fix_notched: FixMode::Off,
        fix_tail: FixMode::Off,
        fix_intersecting_edges: FixMode::Off,
        fix_wire_orientation: FixMode::Off,
        fix_add_natural_bound: FixMode::Off,
        fix_missing_seam: FixMode::Off,
        fix_small_area: FixMode::Off,
        fix_duplicate_faces: FixMode::Off,
        fix_intersecting_wires: FixMode::Off,
        fix_orientation: FixMode::Off,
        fix_same_parameter: FixMode::Off,
        fix_vertex_tolerance: FixMode::Off,
        fix_pcurve: FixMode::Off,
        fix_coincident_vertices: FixMode::Off,
        fix_wireframe: FixMode::Off,
        fix_split_common_vertex: FixMode::Off,
        fix_small_faces: FixMode::Off,
    }
}

/// Repair oracle shared by every matrix cell.
///
/// Asserts dual-validator validity, the (faces, edges, vertices) entity
/// census, and the closed-form volume pin — except at 1e-3 scale with a
/// 5e-8-class injection, where the defect itself moves ~8e-6 relative
/// (coarser than `REL_VOL`): validity + census still bind, the volume pin is
/// skipped. Every repaired cell therefore proves validity, census, and volume
/// within the oracle's own floor.
#[allow(clippy::too_many_arguments)]
fn check_repaired(
    topo: &Topology,
    solid: SolidId,
    label: &str,
    cell: &str,
    scale: f64,
    expected_faces: usize,
    failures: &mut Vec<String>,
) {
    match remus_operations::validate::validate_solid(topo, solid) {
        Ok(after) if after.is_valid() => {}
        other => failures.push(format!(
            "{label} {cell}: post-commit revalidation: {other:?}"
        )),
    }
    match remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    ) {
        Ok(check) if check.is_valid() => {}
        other => failures.push(format!(
            "{label} {cell}: post-commit check revalidation: {other:?}"
        )),
    }
    match solid_entity_counts(topo, solid) {
        Ok((faces, edges, vertices)) if (faces, edges, vertices) == (expected_faces, 12, 8) => {}
        other => failures.push(format!("{label} {cell}: census {other:?} vs (6, 12, 8)")),
    }
    // Volume-pin floor: a 5e-8-class injection at 1e-3 scale moves ~8e-6
    // relative, coarser than REL_VOL — repair there is still proved by
    // validity + census, and the full pin binds at 1 and 1e3.
    let check_volume = scale >= 1.0;
    if !check_volume {
        return;
    }
    let expected_volume = 24.0 * scale.powi(3);
    match solid_volume(topo, solid, 0.01 * scale) {
        Ok(volume) => {
            let relative = (volume - expected_volume).abs() / expected_volume.abs();
            if relative > REL_VOL {
                failures.push(format!(
                    "{label} {cell}: volume {volume:.12e} vs {expected_volume:.12e} \
                     (rel {relative:.3e})"
                ));
            }
        }
        Err(error) => failures.push(format!("{label} {cell}: volume error {error:?}")),
    }
}

fn assert_exact_repair(
    topo: &Topology,
    solid: SolidId,
    label: &str,
    scale: f64,
    expected_faces: usize,
) {
    let operations_report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        operations_report.is_valid(),
        "{label}: L3 validation issues: {:?}",
        operations_report.issues
    );
    let check_report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    assert!(
        check_report.is_valid(),
        "{label}: check validation issues: {:?}",
        check_report.issues
    );
    let (faces, edges, vertices) = solid_entity_counts(topo, solid).unwrap();
    assert_eq!(
        (faces, edges, vertices),
        (expected_faces, 12, 8),
        "{label}: entity census"
    );
    let mut census: BTreeMap<&'static str, usize> = BTreeMap::new();
    for face_id in solid_faces(topo, solid).unwrap() {
        *census
            .entry(topo.face(face_id).unwrap().surface().type_tag())
            .or_default() += 1;
    }
    assert_eq!(
        census,
        BTreeMap::from([("plane", expected_faces)]),
        "{label}: surface census"
    );
    let expected_volume = 24.0 * scale.powi(3);
    let volume = solid_volume(topo, solid, 0.01 * scale).unwrap();
    let relative = (volume - expected_volume).abs() / expected_volume.abs();
    assert!(
        relative <= REL_VOL,
        "{label}: volume {volume:.12e} vs {expected_volume:.12e} (rel {relative:.3e})"
    );
}

fn assert_typed_refusal(
    error: &remus_operations::OperationsError,
    label: &str,
    cell: &str,
    failures: &mut Vec<String>,
) {
    let typed = matches!(
        error,
        remus_operations::OperationsError::HealingRepairRefused { .. }
            | remus_operations::OperationsError::ConfiguredHealingValidationFailed { .. }
            | remus_operations::OperationsError::ConfiguredHealingVerificationUnavailable { .. }
            | remus_operations::OperationsError::HealingValidationFailed { .. }
            | remus_operations::OperationsError::HealingVerificationUnavailable { .. }
    );
    if !typed {
        failures.push(format!("{label} {cell}: untyped error {error:?}"));
    }
}

/// First face's outer wire with its oriented edges in stored order.
fn first_face_wire(topo: &Topology, solid: SolidId) -> (FaceId, WireId, Vec<OrientedEdge>) {
    let face_id = solid_faces(topo, solid).unwrap()[0];
    let wire_id = topo.face(face_id).unwrap().outer_wire();
    let edges = topo.wire(wire_id).unwrap().edges().to_vec();
    (face_id, wire_id, edges)
}

/// Replace one face's outer wire with `edges` (closed flag preserved).
fn rewire_face_outer(topo: &mut Topology, face: FaceId, edges: Vec<OrientedEdge>) {
    let old_wire = topo.face(face).unwrap().outer_wire();
    let closed = topo.wire(old_wire).unwrap().is_closed();
    let new_wire = topo.add_wire(Wire::new(edges, closed).unwrap());
    let inner = topo.face(face).unwrap().inner_wires().to_vec();
    topo.set_face_boundary_wires(face, new_wire, inner).unwrap();
}

// ── Defect injectors ──

/// Swap two edges of one face's outer wire.
///
/// A swap breaks all four joints (gaps ~3.6 at unit scale): the reorder pass
/// restores the chain (disclosed `WireReordered`), then the gap pass
/// re-merges the vertices. See surprise 5 for the policy split.
fn inject_wire_order(topo: &mut Topology, solid: SolidId) {
    let (face, _, mut edges) = first_face_wire(topo, solid);
    assert!(edges.len() >= 4);
    edges.swap(0, 2);
    rewire_face_outer(topo, face, edges);
}

/// Move one wire vertex off its neighbor by `gap` (breaks connectivity).
///
/// Splits the shared corner between the first two edges of the first face's
/// wire: the second edge gets a fresh start vertex displaced by `gap` along
/// +x, leaving a mid-wire gap while the closure joint stays intact. The two
/// vertices keep distinct IDs at near-coincident positions, so nominal
/// analysis reports no gap and only On closes it (surprise 8).
fn inject_wire_gap(topo: &mut Topology, solid: SolidId, gap: f64) {
    let (face, wire_id, edges) = first_face_wire(topo, solid);
    assert!(edges.len() >= 2);
    let second_edge_id = edges[1].edge();
    let old_start = if edges[1].is_forward() {
        topo.edge(second_edge_id).unwrap().start()
    } else {
        topo.edge(second_edge_id).unwrap().end()
    };
    let anchor = topo.vertex(old_start).unwrap().point();
    let displaced = Point3::new(anchor.x() + gap, anchor.y(), anchor.z());
    let fresh = topo.add_vertex(Vertex::new(displaced, TOL));
    {
        let edge = topo.edge_mut(second_edge_id).unwrap();
        if edges[1].is_forward() {
            edge.set_start(fresh);
        } else {
            edge.set_end(fresh);
        }
    }
    // Keep the wire record identical (same edges, same flags): only the
    // shared position moved, which is exactly the gap defect.
    let _ = (face, wire_id);
}

/// Open the closure joint of one face wire by `gap` (breaks `is_closed`).
fn inject_closure_gap(topo: &mut Topology, solid: SolidId, gap: f64) {
    let (face, _, edges) = first_face_wire(topo, solid);
    assert!(edges.len() >= 2);
    let last_oe = *edges.last().unwrap();
    let last_edge_id = last_oe.edge();
    let old_end = if last_oe.is_forward() {
        topo.edge(last_edge_id).unwrap().end()
    } else {
        topo.edge(last_edge_id).unwrap().start()
    };
    let anchor = topo.vertex(old_end).unwrap().point();
    let displaced = Point3::new(anchor.x() + gap, anchor.y(), anchor.z());
    let fresh = topo.add_vertex(Vertex::new(displaced, TOL));
    {
        let edge = topo.edge_mut(last_edge_id).unwrap();
        if last_oe.is_forward() {
            edge.set_end(fresh);
        } else {
            edge.set_start(fresh);
        }
    }
    let _ = face;
}

/// Splice one short edge of length `len` into one face wire.
///
/// The splice is connectivity-clean (analysis: zero gaps, one small edge),
/// so small-edge-only Auto removes it and commits exactly.
fn inject_small_edge(topo: &mut Topology, solid: SolidId, len: f64) {
    let (face, _, mut edges) = first_face_wire(topo, solid);
    assert!(edges.len() >= 2);
    let first = edges[0];
    let first_edge = topo.edge(first.edge()).unwrap().clone();
    let joint = first.oriented_end(&first_edge);
    let anchor = topo.vertex(joint).unwrap().point();
    let mid = topo.add_vertex(Vertex::new(
        Point3::new(anchor.x() + len, anchor.y(), anchor.z()),
        TOL,
    ));
    let short = topo.add_edge(Edge::new(joint, mid, EdgeCurve::Line));
    // Splice the short edge after the first edge and re-hang the second edge
    // on the new vertex so the wire stays a connected chain with one short link.
    let second = edges[1];
    {
        let edge = topo.edge_mut(second.edge()).unwrap();
        if second.is_forward() {
            edge.set_start(mid);
        } else {
            edge.set_end(mid);
        }
    }
    edges.insert(1, OrientedEdge::new(short, true));
    rewire_face_outer(topo, face, edges);
}

/// Flip one face's stored plane normal.
///
/// Shell shared-edge senses are untouched, so shell orientation analysis stays
/// consistent and only the winding check warns (L3 validator still passes);
/// exactly `fix_wire_orientation` repairs it.
fn inject_face_orientation(topo: &mut Topology, solid: SolidId) {
    let face_id = solid_faces(topo, solid).unwrap()[0];
    let (normal, d) = match topo.face(face_id).unwrap().surface() {
        FaceSurface::Plane { normal, d } => (*normal, *d),
        other => panic!("box face must be planar, got {:?}", other.type_tag()),
    };
    topo.face_mut(face_id)
        .unwrap()
        .set_surface(FaceSurface::Plane {
            normal: -normal,
            d: -d,
        });
}

/// Toggle one face's `reversed` flag by index (breaks shell orientation
/// consistency while leaving wire connectivity intact).
///
/// Unlike wire-flag reversal (which also disconnects the joints and needs
/// wire fixers on top), a flag toggle is a pure orientation defect: exactly
/// `fix_orientation` repairs it. The index matters (surprise 4): face 1
/// toggled repairs exactly under On with 1 disclosed flip; face 0 toggled
/// (edges all-reversed natively) refuses under every policy; Auto reports no
/// issue (the analysis reads the raw wire flag and stays consistent).
fn inject_shell_orientation_on_face(topo: &mut Topology, solid: SolidId, index: usize) {
    let face_id = solid_faces(topo, solid).unwrap()[index];
    let reversed = topo.face(face_id).unwrap().is_reversed();
    topo.face_mut(face_id).unwrap().set_reversed(!reversed);
}

/// Append one sliver face (bbox diagonal `diag`, below the 1e-7 fixer band)
/// to the shell. The shell keeps its 6 box faces, so the per-shell
/// don't-empty guard does not bite; the fixer must drop exactly the sliver.
fn inject_small_face(topo: &mut Topology, solid: SolidId, diag: f64) {
    let a = topo.add_vertex(Vertex::new(Point3::new(50.0, 50.0, 50.0), TOL));
    let b = topo.add_vertex(Vertex::new(Point3::new(50.0 + diag, 50.0, 50.0), TOL));
    let c = topo.add_vertex(Vertex::new(Point3::new(50.0, 50.0 + diag, 50.0), TOL));
    let eab = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
    let ebc = topo.add_edge(Edge::new(b, c, EdgeCurve::Line));
    let eca = topo.add_edge(Edge::new(c, a, EdgeCurve::Line));
    let wire = topo.add_wire(
        Wire::new(
            vec![
                OrientedEdge::new(eab, true),
                OrientedEdge::new(ebc, true),
                OrientedEdge::new(eca, true),
            ],
            true,
        )
        .unwrap(),
    );
    let sliver = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Plane {
            normal: Vec3::new(0.0, 0.0, 1.0),
            d: 50.0,
        },
    ));
    let shell_id = topo.solid(solid).unwrap().outer_shell();
    let mut faces = topo.shell(shell_id).unwrap().faces().to_vec();
    faces.push(sliver);
    *topo.shell_mut(shell_id).unwrap() = Shell::new(faces).unwrap();
}

fn planar_triangle(topo: &mut Topology, points: [Point3; 3], normal: Vec3, d: f64) -> FaceId {
    let vertices = points.map(|point| topo.add_vertex(Vertex::new(point, TOL)));
    let edges = [
        topo.add_edge(Edge::new(vertices[0], vertices[1], EdgeCurve::Line)),
        topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line)),
        topo.add_edge(Edge::new(vertices[2], vertices[0], EdgeCurve::Line)),
    ];
    let wire = topo.add_wire(
        Wire::new(
            edges
                .into_iter()
                .map(|edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }))
}

/// Append an exact geometric duplicate of one box face (same winding).
///
/// Copies the first face's corner positions with fresh vertices/edges so the
/// pair shares geometry but no topology — the duplicate-face defect.
fn inject_duplicate_face(topo: &mut Topology, solid: SolidId) {
    let face_id = solid_faces(topo, solid).unwrap()[0];
    let corners: Vec<Point3> = {
        let face = topo.face(face_id).unwrap();
        let wire = topo.wire(face.outer_wire()).unwrap();
        wire.edges()
            .iter()
            .map(|oe| {
                let edge = topo.edge(oe.edge()).unwrap();
                topo.vertex(oe.oriented_start(edge)).unwrap().point()
            })
            .collect()
    };
    assert_eq!(corners.len(), 4);
    let (normal, d) = match topo.face(face_id).unwrap().surface() {
        FaceSurface::Plane { normal, d } => (*normal, *d),
        other => panic!("box face must be planar, got {:?}", other.type_tag()),
    };
    let vertices: Vec<_> = corners
        .into_iter()
        .map(|point| topo.add_vertex(Vertex::new(point, TOL)))
        .collect();
    let edges = [
        topo.add_edge(Edge::new(vertices[0], vertices[1], EdgeCurve::Line)),
        topo.add_edge(Edge::new(vertices[1], vertices[2], EdgeCurve::Line)),
        topo.add_edge(Edge::new(vertices[2], vertices[3], EdgeCurve::Line)),
        topo.add_edge(Edge::new(vertices[3], vertices[0], EdgeCurve::Line)),
    ];
    let wire = topo.add_wire(
        Wire::new(
            edges
                .into_iter()
                .map(|edge| OrientedEdge::new(edge, true))
                .collect(),
            true,
        )
        .unwrap(),
    );
    let duplicate = topo.add_face(Face::new(wire, vec![], FaceSurface::Plane { normal, d }));
    let shell_id = topo.solid(solid).unwrap().outer_shell();
    let mut faces = topo.shell(shell_id).unwrap().faces().to_vec();
    faces.push(duplicate);
    *topo.shell_mut(shell_id).unwrap() = Shell::new(faces).unwrap();
}

/// Build a geometrically closed but topologically disjoint unit cube shell:
/// each face owns its vertices/edges (the mesh-import shape sewing repairs).
fn disjoint_cube(topo: &mut Topology, origin: Point3) -> (SolidId, remus_topology::shell::ShellId) {
    let corner =
        |dx: f64, dy: f64, dz: f64| Point3::new(origin.x() + dx, origin.y() + dy, origin.z() + dz);
    let quad = |topo: &mut Topology, pts: [Point3; 4], normal: Vec3, d: f64| {
        let vs: Vec<_> = pts
            .iter()
            .map(|p| topo.add_vertex(Vertex::new(*p, TOL)))
            .collect();
        let es: Vec<_> = (0..4)
            .map(|i| topo.add_edge(Edge::new(vs[i], vs[(i + 1) % 4], EdgeCurve::Line)))
            .collect();
        let wire = Wire::new(
            es.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        topo.add_face(Face::new(wid, vec![], FaceSurface::Plane { normal, d }))
    };
    let faces = vec![
        quad(
            topo,
            [
                corner(0.0, 0.0, 0.0),
                corner(0.0, 1.0, 0.0),
                corner(1.0, 1.0, 0.0),
                corner(1.0, 0.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, -1.0),
            -origin.z(),
        ),
        quad(
            topo,
            [
                corner(0.0, 0.0, 1.0),
                corner(1.0, 0.0, 1.0),
                corner(1.0, 1.0, 1.0),
                corner(0.0, 1.0, 1.0),
            ],
            Vec3::new(0.0, 0.0, 1.0),
            origin.z() + 1.0,
        ),
        quad(
            topo,
            [
                corner(0.0, 0.0, 0.0),
                corner(1.0, 0.0, 0.0),
                corner(1.0, 0.0, 1.0),
                corner(0.0, 0.0, 1.0),
            ],
            Vec3::new(0.0, -1.0, 0.0),
            -origin.y(),
        ),
        quad(
            topo,
            [
                corner(0.0, 1.0, 0.0),
                corner(0.0, 1.0, 1.0),
                corner(1.0, 1.0, 1.0),
                corner(1.0, 1.0, 0.0),
            ],
            Vec3::new(0.0, 1.0, 0.0),
            origin.y() + 1.0,
        ),
        quad(
            topo,
            [
                corner(0.0, 0.0, 0.0),
                corner(0.0, 0.0, 1.0),
                corner(0.0, 1.0, 1.0),
                corner(0.0, 1.0, 0.0),
            ],
            Vec3::new(-1.0, 0.0, 0.0),
            -origin.x(),
        ),
        quad(
            topo,
            [
                corner(1.0, 0.0, 0.0),
                corner(1.0, 1.0, 0.0),
                corner(1.0, 1.0, 1.0),
                corner(1.0, 0.0, 1.0),
            ],
            Vec3::new(1.0, 0.0, 0.0),
            origin.x() + 1.0,
        ),
    ];
    let shell_id = topo.add_shell(Shell::new(faces).unwrap());
    (topo.add_solid(Solid::new(shell_id, vec![])), shell_id)
}

// ── Matrix drivers ──

/// Generic repair/refusal matrix over policy x scale.
///
/// Auto/On cells must commit an exact repair with at least one disclosed
/// action; Off cells must refuse typed (the defect breaks validation).
fn run_fix_matrix(
    label: &str,
    severities: &[(&str, f64)],
    set: impl Fn(&mut FixConfig, FixMode),
    inject: impl Fn(&mut Topology, SolidId, f64),
    expected_faces: usize,
) {
    let mut failures = Vec::new();
    for &(severity, magnitude) in severities {
        for policy in Policy::ALL {
            for scale in SCALES {
                let cell = format!(
                    "severity={severity} policy={} scale={scale:e}",
                    policy.label()
                );
                let mut topo = Topology::new();
                let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
                inject(&mut topo, solid, magnitude);
                let config = config_with(policy, &set);
                match remus_operations::heal::fix_shape_verified(
                    &mut topo,
                    solid,
                    &config,
                    Some(TOL),
                ) {
                    Ok(report) => {
                        if policy == Policy::Off {
                            failures.push(format!(
                                "{label} {cell}: Off-policy repair unexpectedly committed"
                            ));
                            continue;
                        }
                        if !report.fixing.refusals.is_empty() || report.fixing.status.is_fail() {
                            failures.push(format!(
                                "{label} {cell}: committed with refusals {:?} status {:?}",
                                report.fixing.refusals, report.fixing.status
                            ));
                            continue;
                        }
                        if report.fixing.actions_taken == 0 {
                            failures.push(format!(
                                "{label} {cell}: committed with zero disclosed repairs"
                            ));
                            continue;
                        }
                        check_repaired(
                            &topo,
                            report.solid,
                            label,
                            &cell,
                            scale,
                            expected_faces,
                            &mut failures,
                        );
                    }
                    Err(error) => {
                        if policy == Policy::Off {
                            assert_typed_refusal(&error, label, &cell, &mut failures);
                        } else {
                            failures.push(format!("{label} {cell}: unexpected error {error:?}"));
                        }
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

// ── The matrix ──

/// Wire-order defects (swapped edge order): Auto commits exactly via reorder
/// (disclosed `WireReordered`); On refuses typed (the gap pass vetoes with
/// `ClosureGapTooLarge` after reorder restores the chain); Off refuses typed
/// (validation vetoes).
#[test]
fn b17_wire_order_matrix() {
    let mut failures = Vec::new();
    for policy in Policy::ALL {
        for scale in SCALES {
            let cell = format!("policy={} scale={scale:e}", policy.label());
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
            inject_wire_order(&mut topo, solid);
            let config = config_with(policy, |config, mode| {
                config.fix_reorder = mode;
                config.fix_connectivity = mode;
                config.fix_gaps_3d = mode;
                config.fix_closure = mode;
            });
            match remus_operations::heal::fix_shape_verified(&mut topo, solid, &config, Some(TOL)) {
                Ok(report) => {
                    if policy != Policy::Auto {
                        failures.push(format!(
                            "wire-order {cell}: unexpectedly committed ({:?})",
                            report.fixing.actions
                        ));
                        continue;
                    }
                    let reordered: usize = report
                        .fixing
                        .actions
                        .iter()
                        .filter(|action| {
                            action.kind == remus_heal::fix::RepairActionKind::WireReordered
                        })
                        .map(|action| action.count)
                        .sum();
                    if reordered == 0 {
                        failures.push(format!(
                            "wire-order {cell}: no WireReordered action ({:?})",
                            report.fixing.actions
                        ));
                    }
                    check_repaired(
                        &topo,
                        report.solid,
                        "wire-order",
                        &cell,
                        scale,
                        6,
                        &mut failures,
                    );
                }
                Err(error) => {
                    if policy == Policy::Auto {
                        failures.push(format!("wire-order {cell}: unexpected {error:?}"));
                    } else {
                        assert_typed_refusal(&error, "wire-order", &cell, &mut failures);
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Closure defects: an opened closure joint below tolerance repairs; the
/// above-tolerance gap refuses typed (`ClosureGapTooLarge`). The at-tolerance
/// gap splits by policy at 1e3 (the stored distance rounds to ~1.0000008e-7,
/// a hair above the `dist <= linear` gate): Auto commits exactly (nominal
/// merge is unconditional once the gate passes — 2 disclosed gap closes),
/// while On refuses typed (the widened `fix_gaps_3d` retry lands at/below
/// nominal and declines with `ClosureGapTooLarge`) — never a silent wrong
/// closure. Below 1e3 both repair.
#[test]
fn b17_wire_closure_matrix() {
    let mut failures = Vec::new();
    for &(severity, gap) in &[("small", 5e-8), ("at_tol", 1e-7), ("large", 5e-3)] {
        for policy in Policy::ALL {
            for scale in SCALES {
                let cell = format!(
                    "severity={severity} policy={} scale={scale:e}",
                    policy.label()
                );
                // At-tolerance at 1e3: only Auto repairs; On refuses typed.
                let repairable = severity == "small"
                    || (severity == "at_tol" && (scale < 1e3 || policy == Policy::Auto));
                let mut topo = Topology::new();
                let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
                inject_closure_gap(&mut topo, solid, gap);
                let config = config_with(policy, |config, mode| {
                    config.fix_closure = mode;
                    config.fix_gaps_3d = mode;
                    config.fix_connectivity = mode;
                });
                match remus_operations::heal::fix_shape_verified(
                    &mut topo,
                    solid,
                    &config,
                    Some(TOL),
                ) {
                    Ok(report) => {
                        if policy == Policy::Off || !repairable {
                            failures.push(format!(
                                "wire-closure {cell}: unexpectedly committed \
                                 (actions {:?}, refusals {:?})",
                                report.fixing.actions, report.fixing.refusals
                            ));
                            continue;
                        }
                        assert_exact_repair(&topo, report.solid, "wire-closure", scale, 6);
                    }
                    Err(error) => {
                        if policy == Policy::Off || !repairable {
                            assert_typed_refusal(&error, "wire-closure", &cell, &mut failures);
                        } else {
                            failures.push(format!("wire-closure {cell}: unexpected {error:?}"));
                        }
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Mid-wire gap defects: the On policy closes below-tolerance gaps and
/// commits exactly (validity + census at 1e-3 where the 5e-8 move is ~8e-6
/// relative, full volume pin at 1 and 1e3); Auto reports zero repairs on the
/// same gaps (Auto gates on `analyze_wire` gaps measured against nominal
/// tolerance, but the split pair keeps distinct IDs at coincident positions
/// so analysis reports no gap) and the verified wrapper refuses typed with
/// empty actions. The 5 mm gap exceeds the 1 mm `MAX_GAP_3D_BOUND` widening
/// cap and refuses typed under every policy, as does the strict
/// below-nominal (`dist_sq < tol_sq`) at-tolerance gap.
#[test]
fn b17_wire_gap_matrix() {
    let mut failures = Vec::new();
    for &(severity, gap) in &[("small", 5e-8), ("at_tol", 1e-7), ("large", 5e-3)] {
        for policy in Policy::ALL {
            for scale in SCALES {
                let cell = format!(
                    "severity={severity} policy={} scale={scale:e}",
                    policy.label()
                );
                let repairable = severity == "small" && policy == Policy::On;
                let mut topo = Topology::new();
                let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
                inject_wire_gap(&mut topo, solid, gap);
                let config = config_with(policy, |config, mode| {
                    config.fix_connectivity = mode;
                    config.fix_gaps_3d = mode;
                    config.fix_closure = mode;
                });
                match remus_operations::heal::fix_shape_verified(
                    &mut topo,
                    solid,
                    &config,
                    Some(TOL),
                ) {
                    Ok(report) => {
                        if policy == Policy::Off || !repairable {
                            failures.push(format!(
                                "wire-gap {cell}: unexpectedly committed \
                                 (actions {:?}, refusals {:?})",
                                report.fixing.actions, report.fixing.refusals
                            ));
                            continue;
                        }
                        check_repaired(
                            &topo,
                            report.solid,
                            "wire-gap",
                            &cell,
                            scale,
                            6,
                            &mut failures,
                        );
                    }
                    Err(error) => {
                        if policy == Policy::Off || !repairable {
                            assert_typed_refusal(&error, "wire-gap", &cell, &mut failures);
                        } else {
                            failures.push(format!("wire-gap {cell}: unexpected {error:?}"));
                        }
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Small-edge defects: a 5e-8 edge is below tolerance and removed by
/// small-edge-only Auto (the splice is connectivity-clean: zero gaps, one
/// small edge). A 1e-6 edge is NOT a small-edge defect at all — analysis
/// reports zero gaps and zero small edges, but the splice still breaks wire
/// connectivity, so every policy refuses typed. The matrix pins sub-tol
/// repair vs super-tol refusal.
///
/// The 1e-3 sub-tol cell skips the volume pin (the 5e-8 splice itself moves
/// ~8e-6 relative at that scale — repair is exact, the oracle is coarser than
/// the defect); validity + census still bind every cell.
#[test]
fn b17_small_edge_matrix() {
    let mut failures = Vec::new();
    for &(severity, len) in &[("sub_tol", 5e-8), ("super_tol", 1e-6)] {
        for policy in Policy::ALL {
            for scale in SCALES {
                let cell = format!(
                    "severity={severity} policy={} scale={scale:e}",
                    policy.label()
                );
                let mut topo = Topology::new();
                let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
                inject_small_edge(&mut topo, solid, len);
                let config = config_with(policy, |config, mode| config.fix_small_edges = mode);
                match remus_operations::heal::fix_shape_verified(
                    &mut topo,
                    solid,
                    &config,
                    Some(TOL),
                ) {
                    Ok(report) => {
                        if severity == "super_tol" || policy == Policy::Off {
                            failures.push(format!(
                                "small-edge {cell}: unexpectedly committed ({:?})",
                                report.fixing.actions
                            ));
                            continue;
                        }
                        if report.fixing.actions_taken == 0 {
                            failures
                                .push(format!("small-edge {cell}: sub-tolerance edge not removed"));
                        }
                        check_repaired(
                            &topo,
                            report.solid,
                            "small-edge",
                            &cell,
                            scale,
                            6,
                            &mut failures,
                        );
                    }
                    Err(error) => {
                        if severity == "super_tol" || policy == Policy::Off {
                            assert_typed_refusal(&error, "small-edge", &cell, &mut failures);
                        } else {
                            failures.push(format!("small-edge {cell}: unexpected {error:?}"));
                        }
                    }
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Face-orientation defects: one flipped plane normal repairs exactly under
/// Auto/On. The defect is validator-clean at L3 (only the check-crate winding
/// warning fires), so the Off cell COMMITS the unchanged solid — asserted
/// explicitly as a no-op commit, not a refusal.
#[test]
fn b17_face_orientation_matrix() {
    let mut failures = Vec::new();
    for policy in Policy::ALL {
        for scale in SCALES {
            let cell = format!("policy={} scale={scale:e}", policy.label());
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
            inject_face_orientation(&mut topo, solid);
            let config = config_with(policy, |config, mode| config.fix_wire_orientation = mode);
            match remus_operations::heal::fix_shape_verified(&mut topo, solid, &config, Some(TOL)) {
                Ok(report) => {
                    if policy == Policy::Off {
                        if report.fixing.actions_taken != 0 {
                            failures.push(format!(
                                "face-orientation {cell}: Off-policy disclosed repairs {:?}",
                                report.fixing.actions
                            ));
                        }
                    } else if report.fixing.actions_taken == 0 {
                        failures.push(format!(
                            "face-orientation {cell}: committed with zero disclosed repairs"
                        ));
                    }
                    check_repaired(
                        &topo,
                        report.solid,
                        "face-orientation",
                        &cell,
                        scale,
                        6,
                        &mut failures,
                    );
                }
                Err(error) => {
                    failures.push(format!("face-orientation {cell}: unexpected {error:?}"));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Shell-orientation defects: toggling face 1's `reversed` flag breaks
/// shared-edge senses while leaving wire connectivity intact — a pure
/// orientation defect. On repairs exactly with 1 disclosed BFS flip;
/// Auto reports no issue (the shell analysis reads the raw wire flag and
/// stays consistent, so Auto gates off) and refuses typed with empty actions;
/// Off refuses typed. Toggling face 0 (edges all-reversed natively) refuses
/// typed under every policy (the BFS cascade flips 5 faces into a
/// still-invalid shell). The matrix pins On-repair vs Auto/Off-refusal on
/// face 1 plus the face-0 refusal row.
#[test]
fn b17_shell_orientation_matrix() {
    let mut failures = Vec::new();
    for policy in Policy::ALL {
        for scale in SCALES {
            let cell = format!("policy={} scale={scale:e}", policy.label());
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
            inject_shell_orientation_on_face(&mut topo, solid, 1);
            let config = config_with(policy, |config, mode| config.fix_orientation = mode);
            match remus_operations::heal::fix_shape_verified(&mut topo, solid, &config, Some(TOL)) {
                Ok(report) => {
                    if policy != Policy::On {
                        failures.push(format!(
                            "shell-orientation {cell}: unexpectedly committed ({:?})",
                            report.fixing.actions
                        ));
                        continue;
                    }
                    let flips: usize = report
                        .fixing
                        .actions
                        .iter()
                        .filter(|action| {
                            action.kind
                                == remus_heal::fix::RepairActionKind::ShellFaceOrientationFixed
                        })
                        .map(|action| action.count)
                        .sum();
                    if flips != 1 {
                        failures.push(format!(
                            "shell-orientation {cell}: {flips} flips, want exactly 1 \
                             ({:?})",
                            report.fixing.actions
                        ));
                    }
                    check_repaired(
                        &topo,
                        report.solid,
                        "shell-orientation",
                        &cell,
                        scale,
                        6,
                        &mut failures,
                    );
                }
                Err(error) => {
                    if policy == Policy::On {
                        failures.push(format!("shell-orientation {cell}: unexpected {error:?}"));
                    } else {
                        assert_typed_refusal(&error, "shell-orientation", &cell, &mut failures);
                    }
                }
            }
        }
    }
    // Face 0: BFS has no consistent seed — typed refusal under every policy.
    for policy in Policy::ALL {
        for scale in SCALES {
            let cell = format!("face0 policy={} scale={scale:e}", policy.label());
            let mut topo = Topology::new();
            let solid = make_box(&mut topo, 2.0 * scale, 3.0 * scale, 4.0 * scale).unwrap();
            inject_shell_orientation_on_face(&mut topo, solid, 0);
            let config = config_with(policy, |config, mode| config.fix_orientation = mode);
            match remus_operations::heal::fix_shape_verified(&mut topo, solid, &config, Some(TOL)) {
                Ok(report) => failures.push(format!(
                    "shell-orientation {cell}: unexpectedly committed ({:?})",
                    report.fixing.actions
                )),
                Err(error) => {
                    assert_typed_refusal(&error, "shell-orientation", &cell, &mut failures);
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// Small-face defects: a 1e-9-diagonal sliver is dropped (the shell keeps its
/// 6 box faces, so census stays 6); Off refuses typed.
#[test]
fn b17_small_face_matrix() {
    run_fix_matrix(
        "small-face",
        &[("sliver", 1e-9)],
        |config, mode| config.fix_small_faces = mode,
        |topo, solid, _| inject_small_face(topo, solid, 1e-9),
        6,
    );
}

/// Duplicate-face defects: an exact same-winding geometric copy is removed
/// (census back to 6). The roadmap trap holds only for the integer it names:
/// same-centroid/different-boundary and opposite-winding pairs are KEPT
/// (pinned below as keeps, not misses to fix here).
#[test]
fn b17_duplicate_face_matrix() {
    run_fix_matrix(
        "duplicate-face",
        &[("exact_copy", 0.0)],
        |config, mode| config.fix_duplicate_faces = mode,
        |topo, solid, _| inject_duplicate_face(topo, solid),
        6,
    );
}

/// Duplicate-face keeps (the roadmap trap, pinned as specified behavior, not
/// patched here).
///
/// The current comparator (`heal/src/fix/solid.rs`) matches effective plane
/// normals plus ordered same-winding boundaries on planar line-bounded faces
/// only — NOT centroid/normal/edge-count. Each keep below exercises one
/// rejected discriminant via the raw fixer contract (a two-triangle shell is
/// open/disconnected, so the verified wrapper would refuse on validation
/// regardless of what the duplicate pass does — these assert the disclosed
/// action count instead, which is what the trap is about):
///
/// - same centroid, different boundary (triangle vs larger concentric
///   triangle): no ordered-boundary coincidence, 0 removals;
/// - same corners, opposite winding: winding must match, 0 removals;
/// - same corners and winding, reversed flag: effective normals disagree,
///   0 removals.
#[test]
fn b17_duplicate_face_keeps() {
    use remus_heal::context::HealContext;
    // Same centroid, different boundary: triangle vs larger concentric triangle.
    {
        let mut topo = Topology::new();
        let small = planar_triangle(
            &mut topo,
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
                Point3::new(0.0, 3.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        );
        let large = planar_triangle(
            &mut topo,
            [
                Point3::new(2.0, 2.0, 0.0),
                Point3::new(-1.0, 2.0, 0.0),
                Point3::new(2.0, -1.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        );
        let shell = topo.add_shell(Shell::new(vec![small, large]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let config = config_with(Policy::Auto, |config, mode| {
            config.fix_duplicate_faces = mode;
        });
        let _ctx = HealContext::new();
        let result = remus_heal::fix::fix_shape_with_history(&mut topo, solid, &config, Some(TOL))
            .unwrap()
            .1;
        assert_eq!(result.actions_taken, 0);
        assert!(result.refusals.is_empty());
    }
    // Opposite winding, same corners: boundary coincidence requires the same
    // winding, so this pair is kept.
    {
        let mut topo = Topology::new();
        let original = planar_triangle(
            &mut topo,
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        );
        let opposite = planar_triangle(
            &mut topo,
            [
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            Vec3::new(0.0, 0.0, 1.0),
            0.0,
        );
        let shell = topo.add_shell(Shell::new(vec![original, opposite]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let config = config_with(Policy::Auto, |config, mode| {
            config.fix_duplicate_faces = mode;
        });
        let _ctx = HealContext::new();
        let result = remus_heal::fix::fix_shape_with_history(&mut topo, solid, &config, Some(TOL))
            .unwrap()
            .1;
        assert_eq!(result.actions_taken, 0);
        assert!(result.refusals.is_empty());
    }
    // Same boundary, reversed flag: effective normals disagree, kept.
    {
        let mut topo = Topology::new();
        let corners = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let original = planar_triangle(&mut topo, corners, Vec3::new(0.0, 0.0, 1.0), 0.0);
        let flagged = planar_triangle(&mut topo, corners, Vec3::new(0.0, 0.0, 1.0), 0.0);
        topo.face_mut(flagged).unwrap().set_reversed(true);
        let shell = topo.add_shell(Shell::new(vec![original, flagged]).unwrap());
        let solid = topo.add_solid(Solid::new(shell, vec![]));
        let config = config_with(Policy::Auto, |config, mode| {
            config.fix_duplicate_faces = mode;
        });
        let _ctx = HealContext::new();
        let result = remus_heal::fix::fix_shape_with_history(&mut topo, solid, &config, Some(TOL))
            .unwrap()
            .1;
        assert_eq!(result.actions_taken, 0);
        assert!(result.refusals.is_empty());
    }
}

/// Sewing / free-bounds defects: a disjoint cube has 24 free edges; the
/// `fix_wireframe` pipeline op sews all 12 pairs and both validators pass.
/// Through `fix_shape` (no wireframe call) the same shell refuses typed —
/// pinned here so a future wiring change is a visible matrix flip, not a
/// silent behavior drift.
#[test]
fn b17_sewing_matrix() {
    let mut failures = Vec::new();
    for scale in SCALES {
        let cell = format!("scale={scale:e}");
        // Pipeline path: sews exactly.
        {
            let mut topo = Topology::new();
            let origin = Point3::new(17.0 * scale, -23.0 * scale, 31.0 * scale);
            let (solid, _) = disjoint_cube(&mut topo, origin);
            let mut process = HealProcess::new();
            process.add_step("fix_wireframe");
            match remus_operations::heal::run_heal_pipeline_verified(&mut topo, solid, &process) {
                Ok(report) => {
                    assert!(
                        report.is_valid_after(),
                        "sewing {cell}: pipeline validators not clean"
                    );
                    let sewn: usize = report
                        .steps
                        .iter()
                        .flat_map(|step| step.actions.iter())
                        .filter(|action| {
                            action.kind == remus_heal::fix::RepairActionKind::FreeEdgePairSewn
                        })
                        .map(|action| action.count)
                        .sum();
                    if sewn != 12 {
                        failures.push(format!("sewing {cell}: sewn {sewn} pairs, want 12"));
                    }
                }
                Err(error) => failures.push(format!("sewing {cell}: pipeline error {error:?}")),
            }
        }
        // fix_shape path: typed refusal (wireframe is not in the fix_shape tree).
        {
            let mut topo = Topology::new();
            let origin = Point3::new(17.0 * scale, -23.0 * scale, 31.0 * scale);
            let (solid, _) = disjoint_cube(&mut topo, origin);
            match remus_operations::heal::fix_shape_verified(
                &mut topo,
                solid,
                &FixConfig::default(),
                Some(TOL),
            ) {
                Ok(report) => failures.push(format!(
                    "sewing {cell}: fix_shape unexpectedly committed ({:?})",
                    report.fixing.actions
                )),
                Err(error) => assert_typed_refusal(&error, "sewing", &cell, &mut failures),
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
