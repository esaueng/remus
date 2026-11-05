//! V2 offset operations delegating to remus-offset.

use remus_offset::{JointType, OffsetError, OffsetOptions};
use remus_topology::Topology;
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::evolution::EvolutionMap;

/// Map an `OffsetError` to the most appropriate `OperationsError` variant,
/// preserving structured error information where possible.
fn map_offset_error(e: OffsetError) -> OperationsError {
    match e {
        OffsetError::Topology(t) => OperationsError::Topology(t),
        OffsetError::Math(m) => OperationsError::Math(m),
        // Construction has already accepted the arguments. Preserve the
        // producer's typed assembly failure instead of blaming caller input.
        error @ OffsetError::AssemblyFailed { .. } => OperationsError::Offset(error),

        other => OperationsError::InvalidInput {
            reason: format!("{other}"),
        },
    }
}

/// `distance` is the signed offset distance when the operation is a uniform
/// solid offset, and `None` for a shell, whose collapse stays the generic
/// refusal it was.
fn validate_offset_postcondition(
    topo: &Topology,
    operation: &'static str,
    solid: SolidId,
    distance: Option<f64>,
) -> Result<SolidId, OperationsError> {
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )?;
    if !report.is_valid() {
        let summary = report
            .issues
            .iter()
            .filter(|issue| issue.severity == remus_check::validate::Severity::Error)
            .take(3)
            .map(|issue| issue.description.as_str())
            .collect::<Vec<_>>()
            .join("; ");
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} postcondition validation failed with {} error(s): {summary}",
                report.error_count()
            ),
        });
    }
    ensure_not_collapsed(topo, operation, solid, distance)?;
    Ok(solid)
}

/// Reject an offset that pushed the boundary through itself.
///
/// An inward offset larger than the body's own half-thickness carries every
/// face past its opposite number. Nothing in the per-face offset notices:
/// the radius guards in `crates/offset/src/offset.rs` catch a cylinder,
/// cone, sphere, or torus whose radius goes non-positive, but a plane has no
/// radius — it is simply translated — so an all-planar solid has no
/// per-face collapse condition at all. Assembly then succeeds on the
/// inverted arrangement and returns a solid that is inside out.
///
/// On a 10 mm box: -4.9 gives the correct 0.008 mm^3 result, -5.0 happens to
/// fail in assembly, and -6, -10, and -1e6 all returned `Ok` with volumes of
/// 8, 1000 (the untouched input), and 8e18. `validate_solid` above does not
/// see it, because the check crate has no shell-orientation check.
///
/// A negative signed volume on the outer shell is exactly that inversion,
/// and it is the one signature every collapsed case shares, so it is checked
/// here rather than by widening the general validator.
///
/// For an inward solid offset the inversion is the typed
/// [`OperationsError::OffsetCollapsed`]; any other inversion (an outward
/// offset, a shell) is not a collapse the caller asked for and stays the
/// generic refusal.
fn ensure_not_collapsed(
    topo: &Topology,
    operation: &'static str,
    solid: SolidId,
    distance: Option<f64>,
) -> Result<(), OperationsError> {
    let gauss_order = remus_check::properties::PropertiesOptions::default().gauss_order;
    let Some(floor) = crate::measure::negligible_volume(topo, solid) else {
        return Ok(());
    };
    let outer = topo.solid(solid)?.outer_shell();
    let Some(signed) = crate::measure::shell_signed_volume(topo, outer, gauss_order) else {
        return Ok(());
    };
    if signed < -floor {
        if let Some(distance) = distance.filter(|d| *d < 0.0) {
            return Err(OperationsError::OffsetCollapsed {
                operation,
                distance,
                thickness: None,
            });
        }
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "{operation} collapsed the solid: the result's outer shell is inside out \
                 (signed volume {signed}), which means the offset carried the boundary \
                 through itself"
            ),
        });
    }
    Ok(())
}

/// Refuse, before construction, an inward offset that a certified thickness
/// says must collapse.
///
/// The qualified cell is an all-planar solid whose every edge is a straight
/// line (a box, a prism, any polyhedron). Its extent along a direction is
/// then attained at its vertices, so along each face normal `n` the solid
/// lies in the slab `min(n·v) <= n·x <= max(n·v)` of width `w`. An inward
/// offset by `|d|` keeps only points at least `|d|` inside the solid, which
/// lie inside that slab shrunk by `|d|` from both sides: empty when
/// `2|d| > w`, a zero-volume sheet when `2|d| = w`. That holds whether or not
/// the solid is convex, so the refusal is sound, never a guess.
///
/// Without it a box offset to exactly its half-thickness reached assembly
/// with coincident opposite faces and failed as a non-manifold topology
/// error, and one offset past it was caught only after construction, as an
/// untyped inside-out result.
///
/// Bodies outside the cell (any curved face or edge) are not judged here:
/// their per-face radius guards and the inversion postcondition still apply.
fn refuse_certified_collapse(
    topo: &Topology,
    operation: &'static str,
    solid: SolidId,
    distance: f64,
) -> Result<(), OperationsError> {
    if distance.is_nan() || distance >= 0.0 {
        return Ok(());
    }
    let Some(thickness) = planar_min_thickness(topo, solid)? else {
        return Ok(());
    };
    // Coordinates carry rounding relative to the model's size; the linear
    // tolerance covers small models.
    let tolerance = remus_math::tolerance::Tolerance::default()
        .linear
        .max(thickness * 1e-12);
    if 2.0 * distance.abs() >= thickness - tolerance {
        return Err(OperationsError::OffsetCollapsed {
            operation,
            distance,
            thickness: Some(thickness),
        });
    }
    Ok(())
}

/// Smallest width of an all-planar, straight-edged solid across its own face
/// normals, or `None` when the solid is outside that cell.
fn planar_min_thickness(topo: &Topology, solid: SolidId) -> Result<Option<f64>, OperationsError> {
    use remus_math::vec::Vec3;
    use remus_topology::edge::EdgeCurve;
    use remus_topology::face::FaceSurface;

    let outer = topo.solid(solid)?.outer_shell();
    let mut normals: Vec<Vec3> = Vec::new();
    let mut points: Vec<Vec3> = Vec::new();
    for &fid in topo.shell(outer)?.faces() {
        let face = topo.face(fid)?;
        let FaceSurface::Plane { normal, .. } = face.surface() else {
            return Ok(None);
        };
        normals.push(*normal);
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid)?.edges() {
                let edge = topo.edge(oe.edge())?;
                if !matches!(edge.curve(), EdgeCurve::Line) {
                    return Ok(None);
                }
                for vid in [edge.start(), edge.end()] {
                    let p = topo.vertex(vid)?.point();
                    points.push(Vec3::new(p.x(), p.y(), p.z()));
                }
            }
        }
    }
    let mut thickness: Option<f64> = None;
    for normal in normals {
        let length = normal.length();
        if !(length.is_finite() && length > 0.0) {
            return Ok(None);
        }
        let n = normal * (1.0 / length);
        let (lo, hi) = points
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                let s = n.dot(*p);
                (lo.min(s), hi.max(s))
            });
        let width = hi - lo;
        if !width.is_finite() {
            return Ok(None);
        }
        thickness = Some(thickness.map_or(width, |t: f64| t.min(width)));
    }
    Ok(thickness)
}

/// Offset all faces of a solid (V2 pipeline).
///
/// # Errors
///
/// Returns an error if the offset fails.
pub fn offset_solid_v2(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<SolidId, OperationsError> {
    refuse_certified_collapse(topo, "offset", solid, distance)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        let result = remus_offset::offset_solid(topo, solid, distance, OffsetOptions::default())
            .map_err(map_offset_error)?;
        validate_offset_postcondition(topo, "offset", result, Some(distance))
    })
}

/// Offset every face and return its construction-derived face evolution.
///
/// Each source face is carried to exactly one result face. The lower-level
/// offset engine validates that this correspondence covers both the complete
/// source and result face sets before exposing it.
///
/// # Errors
///
/// Returns an error if the offset fails, its result fails validation, or the
/// construction cannot prove a total one-to-one face map.
pub fn offset_solid_v2_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<(SolidId, EvolutionMap), OperationsError> {
    refuse_certified_collapse(topo, "offset", solid, distance)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        let result = remus_offset::offset_solid_with_face_map(
            topo,
            solid,
            distance,
            OffsetOptions::default(),
        )
        .map_err(map_offset_error)?;
        let solid = validate_offset_postcondition(topo, "offset", result.solid, Some(distance))?;
        let mut evolution = EvolutionMap::exact();
        for (source, result_face) in result.face_map {
            evolution.add_modified(source, result_face.index());
        }
        Ok((solid, evolution))
    })
}

/// An offset with result-aware face, edge and vertex history (B18).
#[derive(Debug, Clone)]
pub struct OffsetEntityEvolution {
    /// The offset solid.
    pub solid: SolidId,
    /// Construction-derived one-to-one face map.
    pub faces: EvolutionMap,
    /// Edge and vertex claims induced from `faces` by exact incidence.
    pub boundary: crate::boundary_evolution::BoundaryEvolution,
    /// `faces` and `boundary` compared against every result entity.
    pub completeness: crate::boundary_evolution::EntityCompletenessReport,
}

/// Offset every face and return construction-derived face, edge and vertex
/// history, checked against the actual result entity sets.
///
/// The geometry is exactly [`offset_solid_v2_with_evolution`]'s. Edge and
/// vertex claims are induced from its one-to-one face map by exact
/// incidence (see [`crate::boundary_evolution`]): every result edge and
/// vertex is `modified` from its unique source, `generated` where source
/// faces newly meet, or unresolved with a typed reason when several source
/// entities share its incidence (a torus's two seams, for example).
///
/// # Errors
///
/// Returns the errors of [`offset_solid_v2_with_evolution`], or
/// [`OperationsError::InvalidInput`] if the history fails to account for
/// every result face, edge and vertex. Any failure rolls the topology back.
pub fn offset_solid_v2_with_entity_evolution(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<OffsetEntityEvolution, OperationsError> {
    use crate::boundary_evolution::{
        SourceIncidence, completeness_for_result_solids, induce_boundary_evolution,
        require_accounted,
    };
    remus_topology::transaction::run_transacted(topo, |topo| {
        let source = SourceIncidence::capture(topo, solid)?;
        let (result, faces) = offset_solid_v2_with_evolution(topo, solid, distance)?;
        let boundary = induce_boundary_evolution(topo, &source, &faces, result)?;
        let completeness = completeness_for_result_solids(topo, &faces, &boundary, &[result])?;
        require_accounted("offset", &completeness)?;
        Ok(OffsetEntityEvolution {
            solid: result,
            faces,
            boundary,
            completeness,
        })
    })
}

/// Shell (hollow solid) operation (V2 pipeline).
///
/// # Errors
///
/// Returns an error if the offset fails.
pub fn shell_v2(
    topo: &mut Topology,
    solid: SolidId,
    thickness: f64,
    exclude: &[FaceId],
) -> Result<SolidId, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let result =
            remus_offset::thick_solid(topo, solid, thickness, exclude, OffsetOptions::default())
                .map_err(map_offset_error)?;
        validate_offset_postcondition(topo, "shell", result, None)
    })
}

/// Offset with arc joints (V2 pipeline).
///
/// # Errors
///
/// Returns an error if the offset fails.
pub fn offset_solid_arc_v2(
    topo: &mut Topology,
    solid: SolidId,
    distance: f64,
) -> Result<SolidId, OperationsError> {
    let options = OffsetOptions {
        joint: JointType::Arc,
        ..Default::default()
    };
    refuse_certified_collapse(topo, "arc offset", solid, distance)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        let result =
            remus_offset::offset_solid(topo, solid, distance, options).map_err(map_offset_error)?;
        validate_offset_postcondition(topo, "arc offset", result, Some(distance))
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::panic)]
    use super::*;
    use remus_math::det_hash::DetHashMap;
    use remus_topology::Topology;
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceSurface;

    /// The postcondition's inversion check, reached directly: only an
    /// inward solid offset makes it the typed collapse; an outward offset or
    /// a shell keeps the generic refusal.
    #[test]
    fn inside_out_postcondition_is_typed_only_for_inward_offsets() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        for face in solid_faces(&topo, solid).unwrap() {
            let reversed = topo.face(face).unwrap().is_reversed();
            topo.face_mut(face).unwrap().set_reversed(!reversed);
        }
        let error = validate_offset_postcondition(&topo, "offset", solid, Some(-0.5)).unwrap_err();
        assert!(
            matches!(
                error,
                OperationsError::OffsetCollapsed {
                    operation: "offset",
                    thickness: None,
                    ..
                }
            ),
            "{error:?}"
        );
        assert!(error.to_string().contains("inside out"), "{error}");
        for distance in [Some(0.5), None] {
            let error = validate_offset_postcondition(&topo, "shell", solid, distance).unwrap_err();
            assert!(
                matches!(error, OperationsError::InvalidInput { ref reason } if reason.contains("collapsed")),
                "{distance:?}: {error:?}"
            );
        }
    }

    #[test]
    fn planar_thickness_is_the_thinnest_face_normal_slab() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 5.5, 1.0).unwrap();
        let thickness = planar_min_thickness(&topo, solid).unwrap().unwrap();
        assert!((thickness - 1.0).abs() < 1e-15, "{thickness}");
        let cylinder = crate::primitives::make_cylinder(&mut topo, 1.0, 1.0).unwrap();
        assert!(planar_min_thickness(&topo, cylinder).unwrap().is_none());
        // Outward and curved bodies are never judged.
        assert!(refuse_certified_collapse(&topo, "offset", solid, 5.0).is_ok());
        assert!(refuse_certified_collapse(&topo, "offset", cylinder, -5.0).is_ok());
        assert!(refuse_certified_collapse(&topo, "offset", solid, -0.49).is_ok());
        assert!(matches!(
            refuse_certified_collapse(&topo, "offset", solid, -0.5),
            Err(OperationsError::OffsetCollapsed { .. })
        ));
    }

    #[test]
    fn offset_v2_box() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let result = offset_solid_v2(&mut topo, solid, 0.5).unwrap();
        let shell = topo
            .shell(topo.solid(result).unwrap().outer_shell())
            .unwrap();
        assert_eq!(shell.faces().len(), 6);
    }

    #[test]
    fn offset_evolution_is_total_exact_and_geometrically_true() {
        let mut topo = Topology::new();
        let source = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let source_faces = solid_faces(&topo, source).unwrap();
        let source_planes = source_faces
            .iter()
            .map(|face| match topo.face(*face).unwrap().surface() {
                FaceSurface::Plane { normal, d } => (
                    face.index(),
                    (*normal, *d, topo.face(*face).unwrap().is_reversed()),
                ),
                other => panic!("box face must be planar, got {}", other.type_tag()),
            })
            .collect::<DetHashMap<_, _>>();

        let (result, evolution) = offset_solid_v2_with_evolution(&mut topo, source, 0.5).unwrap();

        assert!(evolution.origin.is_exact());
        assert!(evolution.generated.is_empty());
        assert!(evolution.deleted.is_empty());
        assert!(evolution.unresolved.is_empty());
        assert_eq!(evolution.modified.len(), source_faces.len());

        let result_faces = solid_faces(&topo, result).unwrap();
        let claimed = evolution
            .modified
            .values()
            .flatten()
            .copied()
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(claimed.len(), result_faces.len());
        assert!(
            result_faces
                .iter()
                .all(|face| claimed.contains(&face.index()))
        );

        for (source_index, outputs) in &evolution.modified {
            assert_eq!(outputs.len(), 1, "offset face identity must be one-to-one");
            let (source_normal, source_d, source_reversed) = {
                let (normal, d, reversed) = source_planes[source_index];
                (normal, d, reversed)
            };
            let result_face = topo.face_id_from_index(outputs[0]).unwrap();
            let result_face_data = topo.face(result_face).unwrap();
            let FaceSurface::Plane {
                normal: result_normal,
                d: result_d,
            } = result_face_data.surface()
            else {
                panic!("mapped box face must remain planar");
            };
            assert!((*result_normal - source_normal).length() < 1e-12);
            assert_eq!(result_face_data.is_reversed(), source_reversed);
            let signed_distance = if source_reversed { -0.5 } else { 0.5 };
            assert!(
                (*result_d - source_d - signed_distance).abs() < 1e-12,
                "mapped face must lie exactly 0.5 model units outward"
            );
        }

        let volume = crate::measure::mass_properties(&topo, result).unwrap().mass;
        assert!((volume - 27.0).abs() < 1e-9, "3x3x3 volume: {volume}");
    }

    #[test]
    fn offset_v2_rejects_cavity_without_dropping_it() {
        let mut topo = Topology::new();
        let outer = crate::primitives::make_box(&mut topo, 4.0, 4.0, 4.0).unwrap();
        let inner = crate::primitives::make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let cavity_shell = topo.solid(inner).unwrap().outer_shell();
        topo.solid_mut(outer).unwrap().add_inner_shell(cavity_shell);

        let error = offset_solid_v2(&mut topo, outer, 0.5).unwrap_err();
        assert!(matches!(
            error,
            OperationsError::InvalidInput { ref reason }
                if reason.contains("cavity shells")
        ));
        assert_eq!(topo.solid(outer).unwrap().inner_shells(), &[cavity_shell]);
    }
}
