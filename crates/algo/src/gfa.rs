//! Top-level GFA orchestrator.
//!
//! Runs the complete General Fuse Algorithm pipeline:
//! PaveFiller -> Builder -> BOP -> assemble.

use remus_math::context::OperationContext;
use remus_math::tolerance::Tolerance;
use remus_topology::BodyClass;
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::ShellId;
use remus_topology::solid::Solid;
use remus_topology::solid::SolidId;

use crate::bop::BooleanOp;
use crate::builder::Builder;
use crate::ds::{GfaArena, GfaShapeStoreN};
use crate::error::AlgoError;
use crate::pave_filler;

/// Result-face provenance in caller-topology face indices:
/// `(result face index, Some(input face index) | None)`.
pub type FaceOriginIndices = Vec<(usize, Option<usize>)>;

/// Run the complete GFA boolean operation with default tolerance.
///
/// This is the single entry point for boolean operations via the GFA.
///
/// # Errors
///
/// Returns [`AlgoError`] if any stage fails.
pub fn boolean(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<SolidId, AlgoError> {
    // Identical-solid fast path: avoid the full GFA pipeline when both
    // operands are the same topology entity.
    if solid_a == solid_b {
        return match op {
            BooleanOp::Fuse | BooleanOp::Intersect => {
                // A ∪ A = A, A ∩ A = A — return the original solid.
                // The caller (operations crate) copies if needed.
                Ok(solid_a)
            }
            BooleanOp::Cut => {
                // A \ A = empty
                Err(AlgoError::AssemblyFailed(
                    "Cut of identical solids produces empty result".into(),
                ))
            }
        };
    }
    boolean_with_context(topo, op, solid_a, solid_b, &OperationContext::new())
}

/// Run the complete GFA boolean operation under an explicit
/// [`OperationContext`].
///
/// This is the context-carrying entry point the pipeline converges on. The
/// context's tolerance drives every GFA stage, and its work budgets bound
/// NURBS surface-surface marching in the pave filler. The default context
/// reproduces [`boolean_with_tolerance`] with the default tolerance exactly;
/// like that function, this runs the full pipeline even
/// for identical operands ([`boolean`] carries the identical-solid fast
/// path).
///
/// # Errors
///
/// Returns [`AlgoError`] if any stage fails.
pub fn boolean_with_context(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        boolean_with_context_impl(topo, op, solid_a, solid_b, context)
    })
}

/// Run the complete GFA boolean operation with custom tolerance.
///
/// Stages:
/// 1. **PaveFiller** — intersect shapes, build pave blocks
/// 2. **Builder** — split faces, classify sub-faces
/// 3. **BOP + assemble** — select faces, build result solid
///
/// # Errors
///
/// Returns [`AlgoError`] if any stage fails.
pub fn boolean_with_tolerance(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
    tol: Tolerance,
) -> Result<SolidId, AlgoError> {
    let context = OperationContext::new().with_tolerance(tol);
    boolean_with_context(topo, op, solid_a, solid_b, &context)
}

fn boolean_with_context_impl(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    context.check_cancelled()?;
    let tol = context.tolerance;
    // Refuse unsupported curve types up front, by name. The pave filler,
    // face splitter and classifier all lack hyperbola/parabola support;
    // letting such an input through would make them fall back to a chord
    // or a straight line and return a plausible but geometrically wrong
    // solid. Failing closed here is the only honest option until the
    // intersection phases learn these conics.
    reject_unsupported_curves(topo, solid_a)?;
    reject_unsupported_curves(topo, solid_b)?;

    // Create an isolated shape store with deep-copied input solids.
    // The GFA pipeline operates entirely within the store, avoiding
    // vertex/edge identity conflicts with the caller's topology.
    let mut store = crate::ds::GfaShapeStore::new(topo, solid_a, solid_b)?;

    // Stage 1: PaveFiller — intersection + pave block construction
    context.check_cancelled()?;
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler_with_context(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        context,
        &mut arena,
    )?;

    // Stage 2: Builder — face splitting + classification
    context.check_cancelled()?;
    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tol,
    );
    builder.perform()?;

    // Stage 3: BOP selection + assembly
    context.check_cancelled()?;
    let (store_topo, store_result) = builder.build_result(op)?;
    store.topo = store_topo;

    // Export result solid back to the caller's topology
    context.check_cancelled()?;
    let result = store.export_solid(topo, store_result)?;

    context.check_cancelled()?;

    Ok(result)
}

/// Split a solid with a first-class sheet-body face set.
///
/// The sheet is installed in the isolated GFA topology behind a traversal-only
/// solid adapter. That adapter is never classified as a volume and its faces
/// are never passed through boolean selection: they participate only in pave
/// filling, face partitioning, and the two oppositely oriented cell closures.
/// The currently qualified exact subset is one cylindrical sheet face.
///
/// # Errors
///
/// Returns [`AlgoError::UnsupportedSheetSplit`] when the root is not a sheet,
/// the sheet is not the qualified single cylindrical face, or it does not
/// separate the solid into two cells. Other GFA and topology errors propagate.
pub fn split_by_sheet(
    topo: &mut Topology,
    solid: SolidId,
    sheet: ShellId,
) -> Result<Vec<SolidId>, AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        split_by_sheet_impl(topo, solid, sheet)
    })
}

fn split_by_sheet_impl(
    topo: &mut Topology,
    solid: SolidId,
    sheet: ShellId,
) -> Result<Vec<SolidId>, AlgoError> {
    topo.solid(solid)?;
    let sheet_data = topo.shell(sheet)?;
    if sheet_data.body_class() != BodyClass::Sheet {
        return Err(AlgoError::UnsupportedSheetSplit {
            reason: format!(
                "tool shell is tagged `{}` instead of `sheet`",
                sheet_data.body_class().as_str()
            ),
        });
    }
    let [sheet_face] = sheet_data.faces() else {
        return Err(AlgoError::UnsupportedSheetSplit {
            reason: format!(
                "qualified sheet split requires exactly one face, got {}",
                sheet_data.faces().len()
            ),
        });
    };
    let FaceSurface::Cylinder(cylinder) = topo.face(*sheet_face)?.surface() else {
        return Err(AlgoError::UnsupportedSheetSplit {
            reason: format!(
                "qualified sheet split requires a cylindrical face, got `{}`",
                topo.face(*sheet_face)?.surface().type_tag()
            ),
        });
    };
    let cylinder = cylinder.clone();

    reject_unsupported_curves(topo, solid)?;

    // A full clone is the isolation boundary. Existing handles remain valid in
    // it, so the sheet shell can be put behind a traversal-only adapter without
    // mutating or reclassifying the caller's first-class sheet body.
    let mut store_topo = topo.clone();
    let sheet_adapter = store_topo.add_solid(Solid::new(sheet, Vec::new()));
    reject_unsupported_curves(&store_topo, sheet_adapter)?;

    let tol = Tolerance::default();
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(&mut store_topo, solid, sheet_adapter, tol, &mut arena)?;
    let mut builder = Builder::with_tolerance(store_topo, arena, solid, sheet_adapter, tol);
    builder.perform_sheet_arrangement()?;
    let (store_topo, store_regions) = builder.build_cylindrical_sheet_regions(&cylinder)?;

    let mut regions = Vec::with_capacity(store_regions.len());
    for region in store_regions {
        regions.push(crate::ds::shape_store::deep_copy_solid(&store_topo, topo, region)?.0);
    }
    Ok(regions)
}

/// Trim a first-class sheet body against a solid, retaining either its inside
/// or outside face patches.
///
/// The sheet is a face-set operand: a traversal-only adapter feeds its faces
/// through pave filling and splitting, but it is classified only against the
/// real solid and is never interpreted as bounding material.
///
/// # Errors
///
/// Returns [`AlgoError::UnsupportedSheetTrim`] for an incorrectly tagged,
/// empty, coincident, or empty-result sheet configuration. Other exact GFA and
/// topology failures propagate.
pub fn trim_sheet_by_solid(
    topo: &mut Topology,
    sheet: ShellId,
    solid: SolidId,
    keep_inside: bool,
) -> Result<ShellId, AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        trim_sheet_by_solid_impl(topo, sheet, solid, keep_inside)
    })
}

fn trim_sheet_by_solid_impl(
    topo: &mut Topology,
    sheet: ShellId,
    solid: SolidId,
    keep_inside: bool,
) -> Result<ShellId, AlgoError> {
    topo.solid(solid)?;
    let sheet_data = topo.shell(sheet)?;
    if sheet_data.body_class() != BodyClass::Sheet {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: format!(
                "tool shell is tagged `{}` instead of `sheet`",
                sheet_data.body_class().as_str()
            ),
        });
    }
    if sheet_data.faces().is_empty() {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: "sheet contains no faces".into(),
        });
    }
    let sheet_faces = sheet_data.faces().to_vec();
    let solid_faces = remus_topology::explorer::solid_faces(topo, solid)?;
    if sheet_faces.iter().any(|face| solid_faces.contains(face)) {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: "sheet and solid share a face identity; keep-side classification is ambiguous"
                .into(),
        });
    }

    reject_unsupported_curves(topo, solid)?;
    let mut store_topo = topo.clone();
    let sheet_adapter = store_topo.add_solid(Solid::new(sheet, Vec::new()));
    reject_unsupported_curves(&store_topo, sheet_adapter)?;

    let tol = Tolerance::default();
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(&mut store_topo, solid, sheet_adapter, tol, &mut arena)?;
    let mut builder = Builder::with_tolerance(store_topo, arena, solid, sheet_adapter, tol);
    builder.perform_sheet_arrangement()?;
    let (store_topo, store_sheet) = builder.build_sheet_trim(keep_inside)?;
    crate::ds::shape_store::deep_copy_sheet(&store_topo, topo, store_sheet)
}

/// Mutually trim two first-class planar sheets by their oriented sides.
///
/// Positive is the side each sheet's effective face normal points toward;
/// negative is the opposite side. Both input sheets participate only as face
/// sets, and both returned handles are new first-class sheets.
///
/// # Errors
///
/// Returns [`AlgoError::UnsupportedSheetTrim`] unless both operands are
/// distinct, single-face planar sheets that split each other transversally.
pub fn mutual_trim_sheets(
    topo: &mut Topology,
    sheet_a: ShellId,
    sheet_b: ShellId,
    keep_a_positive: bool,
    keep_b_positive: bool,
) -> Result<(ShellId, ShellId), AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        mutual_trim_sheets_impl(topo, sheet_a, sheet_b, keep_a_positive, keep_b_positive)
    })
}

/// Trim one first-class planar sheet by one oriented side of another.
///
/// The tool sheet is used only to split and classify the target. It need not
/// itself be divided by the finite target intersection.
///
/// # Errors
///
/// Returns [`AlgoError::UnsupportedSheetTrim`] unless both operands are
/// distinct, single-face planar sheets and the tool splits the target.
pub fn trim_sheet_by_sheet(
    topo: &mut Topology,
    target: ShellId,
    tool: ShellId,
    keep_positive: bool,
) -> Result<ShellId, AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        trim_sheet_by_sheet_impl(topo, target, tool, keep_positive)
    })
}

fn planar_sheet(
    topo: &Topology,
    sheet: ShellId,
    label: &str,
) -> Result<(remus_math::vec::Vec3, f64), AlgoError> {
    let shell = topo.shell(sheet)?;
    if shell.body_class() != BodyClass::Sheet {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: format!(
                "{label} shell is tagged `{}` instead of `sheet`",
                shell.body_class().as_str()
            ),
        });
    }
    let [face_id] = shell.faces() else {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: format!(
                "qualified mutual trim requires one face per sheet; {label} has {}",
                shell.faces().len()
            ),
        });
    };
    let face = topo.face(*face_id)?;
    let FaceSurface::Plane { normal, d } = *face.surface() else {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: format!(
                "qualified mutual trim requires planar sheets; {label} is `{}`",
                face.surface().type_tag()
            ),
        });
    };
    let magnitude = normal.length();
    if !magnitude.is_finite() || magnitude <= f64::EPSILON || !d.is_finite() {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: format!("{label} has an invalid supporting plane"),
        });
    }
    let normal = normal * magnitude.recip();
    let d = d / magnitude;
    if face.is_reversed() {
        Ok((-normal, -d))
    } else {
        Ok((normal, d))
    }
}

fn mutual_trim_sheets_impl(
    topo: &mut Topology,
    sheet_a: ShellId,
    sheet_b: ShellId,
    keep_a_positive: bool,
    keep_b_positive: bool,
) -> Result<(ShellId, ShellId), AlgoError> {
    if sheet_a == sheet_b {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: "mutual trim requires two distinct sheet handles".into(),
        });
    }
    let (normal_a, d_a) = planar_sheet(topo, sheet_a, "sheet A")?;
    let (normal_b, d_b) = planar_sheet(topo, sheet_b, "sheet B")?;

    let mut store_topo = topo.clone();
    let adapter_a = store_topo.add_solid(Solid::new(sheet_a, Vec::new()));
    let adapter_b = store_topo.add_solid(Solid::new(sheet_b, Vec::new()));
    reject_unsupported_curves(&store_topo, adapter_a)?;
    reject_unsupported_curves(&store_topo, adapter_b)?;

    let tol = Tolerance::default();
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(&mut store_topo, adapter_a, adapter_b, tol, &mut arena)?;
    let mut builder = Builder::with_tolerance(store_topo, arena, adapter_a, adapter_b, tol);
    builder.perform_sheet_sheet_arrangement()?;
    let (store_topo, store_a, store_b) = builder.build_planar_sheet_sheet_trim(
        normal_a,
        d_a,
        normal_b,
        d_b,
        keep_a_positive,
        keep_b_positive,
    )?;
    let result_a = crate::ds::shape_store::deep_copy_sheet(&store_topo, topo, store_a)?;
    let result_b = crate::ds::shape_store::deep_copy_sheet(&store_topo, topo, store_b)?;
    Ok((result_a, result_b))
}

fn trim_sheet_by_sheet_impl(
    topo: &mut Topology,
    target: ShellId,
    tool: ShellId,
    keep_positive: bool,
) -> Result<ShellId, AlgoError> {
    if target == tool {
        return Err(AlgoError::UnsupportedSheetTrim {
            reason: "sheet-by-sheet trim requires distinct target and tool handles".into(),
        });
    }
    let _ = planar_sheet(topo, target, "target sheet")?;
    let (normal_tool, d_tool) = planar_sheet(topo, tool, "tool sheet")?;

    let mut store_topo = topo.clone();
    let adapter_a = store_topo.add_solid(Solid::new(target, Vec::new()));
    let adapter_b = store_topo.add_solid(Solid::new(tool, Vec::new()));
    reject_unsupported_curves(&store_topo, adapter_a)?;
    reject_unsupported_curves(&store_topo, adapter_b)?;

    let tol = Tolerance::default();
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(&mut store_topo, adapter_a, adapter_b, tol, &mut arena)?;
    let mut builder = Builder::with_tolerance(store_topo, arena, adapter_a, adapter_b, tol);
    builder.perform_sheet_sheet_arrangement()?;
    let (store_topo, store_result) =
        builder.build_planar_sheet_by_sheet_trim(normal_tool, d_tool, keep_positive)?;
    crate::ds::shape_store::deep_copy_sheet(&store_topo, topo, store_result)
}

/// Fuse **N** solids into one via a single GFA arrangement.
///
/// One pass over all operands instead of the sequential pairwise fuse's
/// re-processing of a growing accumulator (O(n²)). Runs the N-way pave filler
/// (pairwise intersection into one arena, spatially pruned) then the N-way
/// builder (split once, classify each sub-face against all other sources, keep
/// the union boundary, resolve coincident faces).
///
/// Fuse-only, and currently limited to the planar-coincidence cases the N-way
/// builder handles — it returns an error (for the caller to fall back to the
/// sequential fuse) on a non-planar coincident contact or an unresolved
/// coincidence. `sources` must be non-empty; a single source returns a copy.
///
/// # Errors
///
/// Returns [`AlgoError`] if `sources` is empty or any GFA stage fails.
pub fn fuse_n(topo: &mut Topology, sources: &[SolidId]) -> Result<SolidId, AlgoError> {
    fuse_n_with_context(topo, sources, &OperationContext::new())
}

/// Fuse **N** solids under an explicit [`OperationContext`].
///
/// # Errors
///
/// Returns [`AlgoError`] if `sources` is empty or any GFA stage fails.
pub fn fuse_n_with_context(
    topo: &mut Topology,
    sources: &[SolidId],
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        fuse_n_with_context_impl(topo, sources, context)
    })
}

fn fuse_n_with_context_impl(
    topo: &mut Topology,
    sources: &[SolidId],
    context: &OperationContext,
) -> Result<SolidId, AlgoError> {
    // Same up-front refusal as `boolean_with_tolerance`; see that function.
    for &sid in sources {
        reject_unsupported_curves(topo, sid)?;
    }
    match sources {
        [] => Err(AlgoError::AssemblyFailed(
            "fuse_n needs at least one source solid".into(),
        )),
        [only] => {
            // A copy so the caller owns a distinct result, mirroring the
            // store round-trip the multi-source path performs.
            let store = GfaShapeStoreN::new(topo, &[*only])?;
            store.export_solid(topo, store.sources[0])
        }
        _ => {
            let tol = context.tolerance;
            let mut store = GfaShapeStoreN::new(topo, sources)?;

            let mut arena = GfaArena::new();
            pave_filler::run_pave_filler_n_with_context(
                &mut store.topo,
                &store.sources,
                context,
                &mut arena,
            )?;

            let (store_topo, store_result) = crate::builder::build_fuse_n(
                std::mem::take(&mut store.topo),
                arena,
                &store.sources,
                &store.face_source,
                tol,
            )?;
            store.topo = store_topo;

            store.export_solid(topo, store_result)
        }
    }
}

/// Run a GFA boolean, also returning each result face's provenance.
///
/// Provenance is `(result face index, Some(input face index) | None)` in the
/// **caller's** topology — the basis for faithful shape-evolution tracking.
/// Indices are `FaceId::index()` values (matching the convention the evolution
/// map uses). `None` marks a synthesised face with no input origin. Unlike
/// [`boolean`], there is no identical-operand fast path; callers handle `A == B`.
///
/// # Errors
///
/// Returns [`AlgoError`] if any GFA stage fails.
pub fn boolean_with_face_origins(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<(SolidId, FaceOriginIndices), AlgoError> {
    // Same up-front refusal as `boolean_with_tolerance`; see that function.
    reject_unsupported_curves(topo, solid_a)?;
    reject_unsupported_curves(topo, solid_b)?;

    let tol = Tolerance::default();
    let mut store = crate::ds::GfaShapeStore::new(topo, solid_a, solid_b)?;

    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        tol,
        &mut arena,
    )?;

    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tol,
    );
    builder.perform()?;

    let (store_topo, store_result, store_origins, _lineage) =
        builder.build_result_with_origins(op)?;
    store.topo = store_topo;

    let (result, export_map) = store.export_solid_with_face_map(topo, store_result)?;

    // Translate store-space provenance to caller-space face indices. The export
    // map is total over result faces, so a miss is a real provenance desync and
    // is surfaced as an error rather than silently dropped (which would mark a
    // real input as `deleted`). A missing input source is `None` by design — a
    // synthesised face with no input origin.
    let mut origins = Vec::with_capacity(store_origins.len());
    for (store_out, store_src) in store_origins {
        let caller_out = export_map
            .get(&store_out.index())
            .ok_or_else(|| {
                AlgoError::AssemblyFailed(
                    "result face missing from export map (provenance desync)".into(),
                )
            })?
            .index();
        let caller_src =
            store_src.and_then(|s| store.input_face_to_caller.get(&s.index()).copied());
        origins.push((caller_out, caller_src));
    }

    Ok((result, origins))
}

/// Refuse a solid whose edges carry a curve type the GFA pipeline cannot
/// handle, naming the variant.
///
/// The pave filler's EE/EF/FF phases, the face splitter's split searches
/// and the analytic classifier all enumerate `EdgeCurve` explicitly and
/// have no hyperbola or parabola support. Without this gate those sites
/// would take their line/chord fallbacks and produce a result that looks
/// like a solid but has the wrong geometry — the failure mode that is
/// hardest to notice downstream. Refusing here fails closed instead.
fn reject_unsupported_curves(topo: &Topology, solid_id: SolidId) -> Result<(), AlgoError> {
    use remus_topology::edge::EdgeCurve;
    use remus_topology::explorer::solid_faces;

    for fid in solid_faces(topo, solid_id)? {
        let face = topo.face(fid)?;
        for &wid in std::iter::once(&face.outer_wire()).chain(face.inner_wires()) {
            let wire = topo.wire(wid)?;
            for oe in wire.edges() {
                let curve = topo.edge(oe.edge())?.curve();
                match curve {
                    EdgeCurve::Line
                    | EdgeCurve::Circle(_)
                    | EdgeCurve::Ellipse(_)
                    | EdgeCurve::NurbsCurve(_) => {}
                    EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => {
                        return Err(AlgoError::UnsupportedCurve {
                            variant: curve.type_tag(),
                        });
                    }
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod context_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_topology::test_utils::make_unit_cube_manifold_at;

    use super::*;

    #[test]
    fn boolean_with_default_context_matches_boolean() {
        // Two overlapping unit cubes, fused twice: once through the legacy
        // entry point, once through the context entry point with the default
        // context. Both results must have identical face counts (the
        // pipelines must be the same code path, not merely similar).
        let mut topo_a = Topology::new();
        let a1 = make_unit_cube_manifold_at(&mut topo_a, 0.0, 0.0, 0.0);
        let a2 = make_unit_cube_manifold_at(&mut topo_a, 0.5, 0.5, 0.5);
        let legacy = boolean(&mut topo_a, BooleanOp::Fuse, a1, a2).unwrap();

        let mut topo_b = Topology::new();
        let b1 = make_unit_cube_manifold_at(&mut topo_b, 0.0, 0.0, 0.0);
        let b2 = make_unit_cube_manifold_at(&mut topo_b, 0.5, 0.5, 0.5);
        let ctx = boolean_with_context(
            &mut topo_b,
            BooleanOp::Fuse,
            b1,
            b2,
            &OperationContext::new(),
        )
        .unwrap();

        let faces_legacy = remus_topology::explorer::solid_faces(&topo_a, legacy)
            .unwrap()
            .len();
        let faces_ctx = remus_topology::explorer::solid_faces(&topo_b, ctx)
            .unwrap()
            .len();
        assert_eq!(faces_legacy, faces_ctx);
    }

    #[test]
    fn boolean_with_context_carries_tolerance() {
        // Cubes separated by 5e-5 are distinct under the 1e-7 default but
        // coincident at the loose context's 1e-4 linear tolerance. This runs
        // GFA directly (no operations-layer disjoint shortcut): the default
        // keeps two closed regions, while the loose run treats the gap as a
        // coincident interface and refuses the resulting open growth shell.
        // That outcome change proves the caller tolerance reached the pave
        // filler and builder rather than being replaced with a default.
        let gap = 5e-5;

        let mut topo_a = Topology::new();
        let a1 = make_unit_cube_manifold_at(&mut topo_a, 0.0, 0.0, 0.0);
        let a2 = make_unit_cube_manifold_at(&mut topo_a, 1.0 + gap, 0.0, 0.0);
        let default = boolean_with_context(
            &mut topo_a,
            BooleanOp::Fuse,
            a1,
            a2,
            &OperationContext::new(),
        )
        .unwrap();

        let mut topo_b = Topology::new();
        let b1 = make_unit_cube_manifold_at(&mut topo_b, 0.0, 0.0, 0.0);
        let b2 = make_unit_cube_manifold_at(&mut topo_b, 1.0 + gap, 0.0, 0.0);
        let loose =
            OperationContext::new().with_tolerance(remus_math::tolerance::Tolerance::loose());
        let context_result = boolean_with_context(&mut topo_b, BooleanOp::Fuse, b1, b2, &loose);

        let default_faces = remus_topology::explorer::solid_faces(&topo_a, default)
            .unwrap()
            .len();
        assert_eq!(
            default_faces, 12,
            "default tolerance keeps both cube boundaries"
        );
        assert!(
            context_result.is_err(),
            "loose tolerance must materially change the sub-tolerance gap classification"
        );
    }
}

/// One result edge's construction-derived history event (Issue 12).
///
/// Claims follow the evolution discipline: `Preserved`/`Modified` bind a
/// result edge to a caller input edge and are made only from construction
/// records (copy maps and pave blocks); `Generated` names the caller faces
/// whose intersection created the edge; `Unresolved` is the honest bucket
/// for edges the builder rebuilt without construction records — never
/// guessed from geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EdgeEvent {
    /// The result edge IS a caller input edge, carried through unchanged
    /// (caller edge index).
    Preserved(usize),
    /// The result edge is a piece of a caller input edge, split by the pave
    /// filler (caller edge index of the parent).
    Modified(usize),
    /// The result edge was generated by a face–face intersection; the
    /// caller face indices of the generating pair (an entry is `None` when
    /// that face was itself synthesised in the store).
    Generated {
        /// Caller face index of the first generating face, when it maps.
        face_a: Option<usize>,
        /// Caller face index of the second generating face, when it maps.
        face_b: Option<usize>,
    },
    /// No construction record reaches this edge (builder rebuild paths are
    /// not yet recorded).
    Unresolved,
}

/// One result vertex's construction-derived history event (Issue 12).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexEvent {
    /// The result vertex IS a caller input vertex (caller vertex index).
    Preserved(usize),
    /// The vertex did not exist in either input: the pipeline created it.
    /// (Existence is construction-derived from the copy maps; the specific
    /// generating interference is not yet recorded.)
    Created,
}

/// Construction-derived vertex/edge/face history of one GFA boolean, in
/// caller-space indices.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EntityEvolution {
    /// Result-face provenance, as in [`boolean_with_face_origins`].
    pub faces: FaceOriginIndices,
    /// Result edge index → its event. Total over the result's edges.
    pub edges: Vec<(usize, EdgeEvent)>,
    /// Result vertex index → its event. Total over the result's vertices.
    pub vertices: Vec<(usize, VertexEvent)>,
}

/// One disconnected boolean result region and its construction-derived
/// entity evolution.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BooleanRegion {
    /// The independently valid result solid.
    pub solid: SolidId,
    /// Provenance total over this region's faces, edges, and vertices.
    pub evolution: EntityEvolution,
}

/// Store-local construction records that must be captured before the builder
/// consumes the pave-filler arena.
struct EntityLineage {
    split_parent: std::collections::HashMap<usize, usize>,
    pave_block_original: std::collections::HashMap<usize, usize>,
    section_origins: std::collections::BTreeMap<usize, (usize, usize)>,
}

impl EntityLineage {
    fn capture(arena: &GfaArena) -> Self {
        let mut split_parent = std::collections::HashMap::new();
        let mut pave_block_original = std::collections::HashMap::new();
        for (pave_block_id, pave_block) in arena.pave_blocks.iter() {
            pave_block_original.insert(pave_block_id.index(), pave_block.original_edge.index());
            if let Some(split) = pave_block.split_edge {
                split_parent.insert(split.index(), pave_block.original_edge.index());
            }
        }
        for (_, common_block) in arena.common_blocks.iter() {
            if let (Some(split), Some(&first_pave_block)) =
                (common_block.split_edge, common_block.pave_blocks.first())
                && let Some(pave_block) = arena.pave_blocks.get(first_pave_block)
            {
                split_parent.insert(split.index(), pave_block.original_edge.index());
            }
        }
        Self {
            split_parent,
            pave_block_original,
            section_origins: arena.section_edge_origins.clone(),
        }
    }
}

fn export_entity_evolution(
    topo: &mut Topology,
    store: &crate::ds::GfaShapeStore,
    store_result: SolidId,
    store_origins: crate::builder::FaceProvenance,
    builder_lineage: &crate::builder::split_types::EdgeLineageLog,
    lineage: &EntityLineage,
) -> Result<(SolidId, EntityEvolution), AlgoError> {
    let (result, export) = store.export_solid_with_entity_maps(topo, store_result)?;

    let mut faces = Vec::with_capacity(store_origins.len());
    for (store_out, store_src) in store_origins {
        let caller_out = export
            .faces
            .get(&store_out.index())
            .ok_or_else(|| {
                AlgoError::AssemblyFailed(
                    "result face missing from export map (provenance desync)".into(),
                )
            })?
            .index();
        let caller_src =
            store_src.and_then(|source| store.input_face_to_caller.get(&source.index()).copied());
        faces.push((caller_out, caller_src));
    }

    let resolve_edge = |store_index: usize| -> EdgeEvent {
        let mut current = store_index;
        let mut transformed = false;
        for _ in 0..64 {
            if let Some(&caller) = store.input_edge_to_caller.get(&current) {
                return if transformed {
                    EdgeEvent::Modified(caller)
                } else {
                    EdgeEvent::Preserved(caller)
                };
            }
            if let Some(&(face_a, face_b)) = lineage.section_origins.get(&current) {
                return EdgeEvent::Generated {
                    face_a: store.input_face_to_caller.get(&face_a).copied(),
                    face_b: store.input_face_to_caller.get(&face_b).copied(),
                };
            }
            if let Some(&old) = builder_lineage.rewrites.get(&current) {
                current = old;
                transformed = true;
                continue;
            }
            if let Some(&pave_block) = builder_lineage.to_pave_block.get(&current) {
                if let Some(&original) = lineage.pave_block_original.get(&pave_block) {
                    current = original;
                    transformed = true;
                    continue;
                }
                break;
            }
            if let Some(&parent) = lineage.split_parent.get(&current) {
                current = parent;
                transformed = true;
                continue;
            }
            break;
        }
        EdgeEvent::Unresolved
    };

    let mut edges: Vec<(usize, EdgeEvent)> = export
        .edges
        .iter()
        .map(|(&store_index, caller_edge)| (caller_edge.index(), resolve_edge(store_index)))
        .collect();
    edges.sort_by_key(|(index, _)| *index);

    let mut vertices: Vec<(usize, VertexEvent)> = export
        .vertices
        .iter()
        .map(|(&store_index, caller_vertex)| {
            let event = store
                .input_vertex_to_caller
                .get(&store_index)
                .map_or(VertexEvent::Created, |&caller| {
                    VertexEvent::Preserved(caller)
                });
            (caller_vertex.index(), event)
        })
        .collect();
    vertices.sort_by_key(|(index, _)| *index);

    Ok((
        result,
        EntityEvolution {
            faces,
            edges,
            vertices,
        },
    ))
}

/// Run a GFA boolean, returning construction-derived vertex, edge, and face
/// history alongside the result (Issue 12).
///
/// Lineage sources, all construction records: the store copy maps
/// (input identity across the isolation copy), the pave filler's pave
/// blocks (`original edge → split edge`), the FF phase's section-edge
/// origin table (`section edge → generating face pair`), and the export
/// maps. Result entities no record reaches are reported
/// [`EdgeEvent::Unresolved`] — never bound by geometric guessing.
///
/// Like [`boolean_with_face_origins`], there is no identical-operand fast
/// path; callers handle `A == B`.
///
/// # Errors
///
/// Returns [`AlgoError`] if any GFA stage fails.
pub fn boolean_with_entity_evolution(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<(SolidId, EntityEvolution), AlgoError> {
    reject_unsupported_curves(topo, solid_a)?;
    reject_unsupported_curves(topo, solid_b)?;

    let tol = Tolerance::default();
    let mut store = crate::ds::GfaShapeStore::new(topo, solid_a, solid_b)?;

    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        tol,
        &mut arena,
    )?;

    let lineage = EntityLineage::capture(&arena);

    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tol,
    );
    builder.perform()?;

    // The returned log includes both the perform-phase records and the
    // assembly-rebuild records (weld and collinear splits), so result
    // edges rebuilt during assembly chase back to their parents.
    let (store_topo, store_result, store_origins, builder_lineage) =
        builder.build_result_with_origins(op)?;
    store.topo = store_topo;

    export_entity_evolution(
        topo,
        &store,
        store_result,
        store_origins,
        &builder_lineage,
        &lineage,
    )
}

/// Run a GFA boolean and return each disconnected result region separately.
///
/// This is the cellular counterpart to [`boolean_with_entity_evolution`].
/// The builder never folds disconnected growth shells into one `Solid`, and
/// each returned region carries its own total construction history.
///
/// # Errors
///
/// Returns [`AlgoError`] if any GFA stage, region assembly, or export fails.
pub fn boolean_regions_with_entity_evolution(
    topo: &mut Topology,
    op: BooleanOp,
    solid_a: SolidId,
    solid_b: SolidId,
) -> Result<Vec<BooleanRegion>, AlgoError> {
    reject_unsupported_curves(topo, solid_a)?;
    reject_unsupported_curves(topo, solid_b)?;

    let tolerance = Tolerance::default();
    let mut store = crate::ds::GfaShapeStore::new(topo, solid_a, solid_b)?;
    let mut arena = GfaArena::new();
    pave_filler::run_pave_filler(
        &mut store.topo,
        store.solid_a,
        store.solid_b,
        tolerance,
        &mut arena,
    )?;
    let lineage = EntityLineage::capture(&arena);
    let mut builder = Builder::with_tolerance(
        std::mem::take(&mut store.topo),
        arena,
        store.solid_a,
        store.solid_b,
        tolerance,
    );
    builder.perform()?;
    let (store_topo, store_regions, store_origins, builder_lineage) =
        builder.build_result_regions_with_origins(op)?;
    store.topo = store_topo;

    let origins_by_face: std::collections::HashMap<FaceId, Option<FaceId>> =
        store_origins.into_iter().collect();
    let mut regions = Vec::with_capacity(store_regions.len());
    for store_region in store_regions {
        let region_origins = remus_topology::explorer::solid_faces(&store.topo, store_region)?
            .into_iter()
            .map(|face| (face, origins_by_face.get(&face).copied().flatten()))
            .collect();
        let (solid, evolution) = export_entity_evolution(
            topo,
            &store,
            store_region,
            region_origins,
            &builder_lineage,
            &lineage,
        )?;
        regions.push(BooleanRegion { solid, evolution });
    }
    Ok(regions)
}

/// Split the target solid's faces wherever they intersect the tool while
/// preserving every target patch and returning total construction lineage.
///
/// The tool participates only in the pave-filler arrangement. Its faces are
/// never selected into the result and no target patch is classified or
/// discarded. The currently qualified subset requires a real transversal
/// split, no same-domain face overlap, and complete edge lineage.
///
/// # Errors
///
/// Returns [`AlgoError::UnsupportedImprint`] when the operands are identical,
/// no target face is divided, a same-domain overlap is present, or any result
/// edge lacks construction lineage. Other GFA errors are propagated.
pub fn imprint_with_entity_evolution(
    topo: &mut Topology,
    target: SolidId,
    tool: SolidId,
) -> Result<(SolidId, EntityEvolution), AlgoError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        if target == tool {
            return Err(AlgoError::UnsupportedImprint {
                reason: "target and tool must be distinct solid handles".into(),
            });
        }
        for (label, solid) in [("target", target), ("tool", tool)] {
            for face_id in remus_topology::explorer::solid_faces(topo, solid)? {
                let surface = topo.face(face_id)?.surface();
                if !surface.is_planar() {
                    return Err(AlgoError::UnsupportedImprint {
                        reason: format!(
                            "{label} face {} has unqualified `{}` surface geometry",
                            face_id.index(),
                            surface.type_tag()
                        ),
                    });
                }
            }
        }
        reject_unsupported_curves(topo, target)?;
        reject_unsupported_curves(topo, tool)?;

        let tolerance = Tolerance::default();
        let mut store = crate::ds::GfaShapeStore::new(topo, target, tool)?;
        let mut arena = GfaArena::new();
        pave_filler::run_pave_filler(
            &mut store.topo,
            store.solid_a,
            store.solid_b,
            tolerance,
            &mut arena,
        )?;
        let lineage = EntityLineage::capture(&arena);

        let mut builder = Builder::with_tolerance(
            std::mem::take(&mut store.topo),
            arena,
            store.solid_a,
            store.solid_b,
            tolerance,
        );
        builder.perform_imprint_arrangement()?;
        let (store_topo, store_result, store_origins, builder_lineage) =
            builder.build_imprint_result_with_origins()?;
        store.topo = store_topo;

        let (result, evolution) = export_entity_evolution(
            topo,
            &store,
            store_result,
            store_origins,
            &builder_lineage,
            &lineage,
        )?;
        if evolution.faces.iter().any(|(_, source)| source.is_none())
            || evolution
                .edges
                .iter()
                .any(|(_, event)| matches!(event, EdgeEvent::Unresolved))
        {
            return Err(AlgoError::UnsupportedImprint {
                reason: "result construction lineage is incomplete".into(),
            });
        }
        Ok((result, evolution))
    })
}

#[cfg(test)]
mod entity_evolution_tests {
    #![allow(clippy::unwrap_used)]

    use remus_topology::test_utils::make_unit_cube_manifold_at;

    use super::*;

    fn count_events(evolution: &EntityEvolution) -> (usize, usize, usize, usize) {
        let mut preserved = 0;
        let mut modified = 0;
        let mut generated = 0;
        let mut unresolved = 0;
        for (_, event) in &evolution.edges {
            match event {
                EdgeEvent::Preserved(_) => preserved += 1,
                EdgeEvent::Modified(_) => modified += 1,
                EdgeEvent::Generated { .. } => generated += 1,
                EdgeEvent::Unresolved => unresolved += 1,
            }
        }
        (preserved, modified, generated, unresolved)
    }

    #[test]
    fn cube_fuse_history_is_total_and_construction_derived() {
        let mut topo = Topology::new();
        let a = make_unit_cube_manifold_at(&mut topo, 0.0, 0.0, 0.0);
        let b = make_unit_cube_manifold_at(&mut topo, 0.5, 0.5, 0.5);
        let (result, evolution) =
            boolean_with_entity_evolution(&mut topo, BooleanOp::Fuse, a, b).unwrap();

        // Totality: exactly the result's edge and vertex sets, each entity
        // once, every entity in some bucket.
        let faces = remus_topology::explorer::solid_faces(&topo, result).unwrap();
        let mut result_edges = std::collections::BTreeSet::new();
        let mut result_vertices = std::collections::BTreeSet::new();
        for &fid in &faces {
            let face = topo.face(fid).unwrap();
            for &wid in std::iter::once(&face.outer_wire()).chain(face.inner_wires().iter()) {
                for oe in topo.wire(wid).unwrap().edges() {
                    result_edges.insert(oe.edge().index());
                    let edge = topo.edge(oe.edge()).unwrap();
                    result_vertices.insert(edge.start().index());
                    result_vertices.insert(edge.end().index());
                }
            }
        }
        let mapped_edges: std::collections::BTreeSet<usize> =
            evolution.edges.iter().map(|(idx, _)| *idx).collect();
        let mapped_vertices: std::collections::BTreeSet<usize> =
            evolution.vertices.iter().map(|(idx, _)| *idx).collect();
        assert_eq!(mapped_edges, result_edges, "edge history must be total");
        assert_eq!(
            mapped_vertices, result_vertices,
            "vertex history must be total"
        );
        assert_eq!(evolution.edges.len(), mapped_edges.len(), "no duplicates");

        // The fuse of two offset cubes preserves far edges, splits crossing
        // edges, and generates section edges from crossing face pairs.
        let (preserved, modified, generated, unresolved) = count_events(&evolution);
        assert!(preserved > 0, "far cube edges must be preserved");
        assert!(modified > 0, "crossing cube edges must be split");
        assert!(
            generated > 0,
            "the intersection must generate section edges"
        );
        // Every Generated edge must name at least one caller face.
        for (_, event) in &evolution.edges {
            if let EdgeEvent::Generated { face_a, face_b } = event {
                assert!(
                    face_a.is_some() || face_b.is_some(),
                    "a section edge must name a generating input face"
                );
            }
        }
        // Every assembly-rebuild path records lineage (perform-phase wire
        // images, vertex-merge rebuilds, welds, collinear splits), so a cube
        // fuse's edge history is total construction fact: any regression
        // that reintroduces an unrecorded rebuild fails here.
        assert_eq!(
            unresolved, 0,
            "every result edge must chase to a construction record \
             (preserved={preserved} modified={modified} generated={generated})"
        );

        // Vertices: both preserved corners and created intersections exist.
        let created = evolution
            .vertices
            .iter()
            .filter(|(_, e)| matches!(e, VertexEvent::Created))
            .count();
        let kept = evolution.vertices.len() - created;
        assert!(kept > 0 && created > 0);

        // Determinism: an identical run reports identical history.
        let mut topo2 = Topology::new();
        let a2 = make_unit_cube_manifold_at(&mut topo2, 0.0, 0.0, 0.0);
        let b2 = make_unit_cube_manifold_at(&mut topo2, 0.5, 0.5, 0.5);
        let (_, evolution2) =
            boolean_with_entity_evolution(&mut topo2, BooleanOp::Fuse, a2, b2).unwrap();
        assert_eq!(evolution.edges, evolution2.edges);
        assert_eq!(evolution.vertices, evolution2.vertices);
    }
}
