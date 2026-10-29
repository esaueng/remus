//! Exact, bounded interior polygon partition of one planar solid face.

use remus_math::context::OperationContext;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft, OpId};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::OperationsError;
use crate::journal_ops::{begin_scoped, solid_entity_keys};

mod interval;
use interval::{I, difference, dot, norm, side};

const MAX_RING: usize = 16;
const MAX_TARGET: usize = 32;

/// A planar partition and the construction entry recording its complete lineage.
#[derive(Debug, Clone, Copy)]
pub struct PlanarPolygonImprintResult {
    /// New solid, with all original material retained.
    pub solid: SolidId,
    /// New inner patch bounded by the polygon.
    pub inner_face: FaceId,
    /// New surrounding patch, with the polygon as its only hole.
    pub surround_face: FaceId,
    /// Construction journal operation with total F/E/V result scope.
    pub op: OpId,
}

fn invalid(reason: impl Into<String>) -> OperationsError {
    OperationsError::InvalidInput {
        reason: reason.into(),
    }
}

fn unsupported(reason: impl Into<String>) -> OperationsError {
    remus_algo::error::AlgoError::UnsupportedImprint {
        reason: reason.into(),
    }
    .into()
}

fn limit(resource: &'static str, actual: usize, maximum: usize) -> Result<(), OperationsError> {
    if actual > maximum {
        return Err(remus_math::MathError::ResourceLimitExceeded {
            resource,
            limit: maximum,
            actual,
        }
        .into());
    }
    Ok(())
}

fn finite_points(points: &[Point3]) -> Result<(), OperationsError> {
    if points
        .iter()
        .any(|point| point.0.iter().any(|coordinate| !coordinate.is_finite()))
    {
        return Err(invalid("imprint polygon coordinates must be finite"));
    }
    Ok(())
}

fn planar(
    points: &[Point3],
    normal: Vec3,
    d: f64,
    normal_length: I,
    linear: f64,
) -> Result<(), OperationsError> {
    let allowance = I::exact(linear).mul(I::exact(normal_length.lo)).lo;
    for point in points {
        let residual = dot(normal.0.map(I::exact), point.0.map(I::exact)).sub(I::exact(d));
        if !residual.finite() || residual.abs_max() > allowance {
            return Err(invalid(
                "imprint boundary must lie on the original plane within linear tolerance",
            ));
        }
    }
    Ok(())
}

fn margin(normal_length: I, a: Point3, b: Point3, linear: f64) -> Result<f64, OperationsError> {
    let length = norm(difference(b, a));
    if !length.finite() {
        return Err(unsupported("unresolved polygon edge-length arithmetic"));
    }
    if length.lo <= linear {
        return Err(invalid(
            "imprint polygon contains an unresolved or short edge",
        ));
    }
    let result = I::exact(linear)
        .mul(I::exact(normal_length.hi))
        .mul(I::exact(length.hi));
    if !result.finite() {
        return Err(unsupported("unresolved polygon halfspace arithmetic"));
    }
    Ok(result.hi)
}

fn clears(value: I, positive: bool, clearance: f64) -> bool {
    value.finite()
        && if positive {
            value.lo > clearance
        } else {
            value.hi < -clearance
        }
}

/// Every nonincident vertex must strictly clear every side. Unlike a local
/// turning-angle test, these complete halfspaces exclude star traversals.
fn convex(
    points: &[Point3],
    normal: Vec3,
    normal_length: I,
    linear: f64,
) -> Result<bool, OperationsError> {
    let first = side(normal, points[0], points[1], points[2]);
    let positive = first.lo > 0.0;
    for i in 0..points.len() {
        let next = (i + 1) % points.len();
        let clearance = margin(normal_length, points[i], points[next], linear)?;
        for (j, point) in points.iter().enumerate() {
            if j != i
                && j != next
                && !clears(
                    side(normal, points[i], points[next], *point),
                    positive,
                    clearance,
                )
            {
                return Err(invalid(
                    "imprint polygon must be simple and strictly convex with linear clearance",
                ));
            }
        }
    }
    Ok(positive)
}

fn target_points(topo: &Topology, face: FaceId) -> Result<Vec<Point3>, OperationsError> {
    let data = topo.face(face)?;
    if !data.inner_wires().is_empty() {
        return Err(unsupported(
            "target face holes are outside the planar polygon family",
        ));
    }
    remus_topology::validation::validate_face_loops(topo, face)?;
    let wire = topo.wire(data.outer_wire())?;
    limit("imprint_target_vertices", wire.edges().len(), MAX_TARGET)?;
    if !wire.is_closed() {
        return Err(invalid("target outer wire must be closed"));
    }
    if wire.edges().len() < 3 {
        return Err(unsupported(
            "target boundary requires at least three straight sides",
        ));
    }
    let mut seen = std::collections::BTreeSet::new();
    let mut points = Vec::with_capacity(wire.edges().len());
    for (i, oriented) in wire.edges().iter().enumerate() {
        let edge = topo.edge(oriented.edge())?;
        if !seen.insert(oriented.edge()) || !matches!(edge.curve(), EdgeCurve::Line) {
            return Err(unsupported(
                "target boundary must use distinct straight edges",
            ));
        }
        let domain = crate::authoritative_edge_domain(edge, "planar polygon imprint")?;
        if domain.0.abs() > 0.0 || (domain.1 - 1.0).abs() > 0.0 {
            return Err(unsupported(
                "target Line edge requires its complete native domain",
            ));
        }
        let (start, end) = if oriented.is_forward() {
            (edge.start(), edge.end())
        } else {
            (edge.end(), edge.start())
        };
        let next = wire.edges()[(i + 1) % wire.edges().len()];
        let next_edge = topo.edge(next.edge())?;
        let next_start = if next.is_forward() {
            next_edge.start()
        } else {
            next_edge.end()
        };
        if end != next_start {
            return Err(invalid(
                "target polygon must share exact consecutive vertices",
            ));
        }
        points.push(topo.vertex(start)?.point());
    }
    finite_points(&points)?;
    Ok(points)
}

fn validate(topo: &Topology, solid: SolidId) -> Result<(), OperationsError> {
    let errors = crate::validate::validate_solid(topo, solid)?.error_count()
        + remus_check::validate::validate_solid(
            topo,
            solid,
            &remus_check::validate::ValidateOptions::default(),
        )?
        .error_count();
    if errors != 0 {
        return Err(OperationsError::BodyValidationFailed {
            body_class: "solid",
            error_count: errors,
        });
    }
    Ok(())
}

/// Partition one hole-free planar convex all-Line face by a strictly interior
/// convex 3--16 point ring, retaining both patches and unchanged material.
///
/// # Errors
/// Invalid, unsupported or unproved geometry refuses atomically. The target
/// loop is capped at 32 vertices; default context budgets remain unchanged.
pub fn imprint_planar_polygon(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    points: &[Point3],
) -> Result<PlanarPolygonImprintResult, OperationsError> {
    imprint_planar_polygon_with_context(topo, solid, face, points, &OperationContext::new())
}

struct Qualified {
    source: Solid,
    original: Face,
    shells: Vec<remus_topology::shell::ShellId>,
    owner: usize,
    ring: Vec<Point3>,
}

struct Partition {
    solid: SolidId,
    inner_face: FaceId,
    surround_face: FaceId,
    inner_wire: remus_topology::wire::WireId,
}

fn owning_shell(
    topo: &Topology,
    source: &Solid,
    face: FaceId,
) -> Result<(Vec<remus_topology::shell::ShellId>, usize), OperationsError> {
    let shells: Vec<_> = std::iter::once(source.outer_shell())
        .chain(source.inner_shells().iter().copied())
        .collect();
    let mut owner = None;
    for (i, shell) in shells.iter().enumerate() {
        for &candidate in topo.shell(*shell)?.faces() {
            if candidate == face && owner.replace(i).is_some() {
                return Err(invalid("target face must have one owning shell use"));
            }
        }
    }
    let owner =
        owner.ok_or_else(|| invalid("selected face does not belong to the selected solid"))?;
    Ok((shells, owner))
}

fn plane(original: &Face) -> Result<(Vec3, f64, I), OperationsError> {
    let FaceSurface::Plane { normal, d } = original.surface() else {
        return Err(unsupported("selected face must retain a native plane"));
    };
    let (normal, d) = (*normal, *d);
    if !d.is_finite() || normal.0.iter().any(|value| !value.is_finite()) {
        return Err(invalid("target plane must be finite"));
    }
    let normal_length = norm(normal.0.map(I::exact));
    if !normal_length.finite()
        || normal_length.lo < 1.0 - 64.0 * f64::EPSILON
        || normal_length.hi > 1.0 + 64.0 * f64::EPSILON
    {
        return Err(unsupported(
            "target plane must have a qualified unit normal",
        ));
    }
    Ok((normal, d, normal_length))
}

fn qualify(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    points: &[Point3],
    context: &OperationContext,
) -> Result<Qualified, OperationsError> {
    context.check_cancelled()?;
    let linear = context.tolerance.linear;
    if !linear.is_finite()
        || linear <= 0.0
        || !context.max_entity_tolerance.is_finite()
        || linear > context.max_entity_tolerance
    {
        return Err(invalid(
            "imprint context requires a finite positive linear tolerance within its entity cap",
        ));
    }
    limit("imprint_polygon_vertices", points.len(), MAX_RING)?;
    if points.len() < 3 {
        return Err(invalid("imprint polygon requires at least three points"));
    }
    finite_points(points)?;
    let source = topo.solid(solid)?.clone();
    let (shells, owner) = owning_shell(topo, &source, face)?;
    let original = topo.face(face)?.clone();
    let (normal, d, normal_length) = plane(&original)?;
    let target = target_points(topo, face)?;
    limit(
        "imprint_segments",
        target.len() + points.len(),
        context.budgets.segments,
    )?;
    let pairs = target.len() * (target.len() - 2)
        + points.len() * (points.len() - 2)
        + target.len() * points.len();
    limit(
        "imprint_halfspace_tests",
        pairs,
        context.budgets.vertex_pairs,
    )?;
    planar(&target, normal, d, normal_length, linear)?;
    planar(points, normal, d, normal_length, linear)?;
    let target_positive = convex(&target, normal, normal_length, linear).map_err(|error| {
        unsupported(format!(
            "target polygon is outside the strict convex family: {error}"
        ))
    })?;
    let ring_positive = convex(points, normal, normal_length, linear)?;
    for i in 0..target.len() {
        context.check_cancelled()?;
        let next = (i + 1) % target.len();
        let clearance = margin(normal_length, target[i], target[next], linear)?;
        for &point in points {
            if !clears(
                side(normal, target[i], target[next], point),
                target_positive,
                clearance,
            ) {
                return Err(unsupported(
                    "imprint ring must stay strictly inside every target side with linear clearance",
                ));
            }
        }
    }
    let mut ring = points.to_vec();
    if ring_positive != target_positive {
        ring.reverse();
    }
    Ok(Qualified {
        source,
        original,
        shells,
        owner,
        ring,
    })
}

fn construct(
    topo: &mut Topology,
    face: FaceId,
    qualified: Qualified,
    linear: f64,
) -> Result<Partition, OperationsError> {
    let Qualified {
        source,
        original,
        shells,
        owner,
        ring,
    } = qualified;
    let inner_wire = remus_topology::builder::make_polygon_wire(topo, &ring, linear)?;
    let reverse_uses = topo
        .wire(inner_wire)?
        .edges()
        .iter()
        .rev()
        .map(|oriented| OrientedEdge::new(oriented.edge(), !oriented.is_forward()))
        .collect();
    let reverse_wire = topo.add_wire(Wire::new(reverse_uses, true)?);
    let mut inner = Face::new(inner_wire, vec![], original.surface().clone());
    inner.set_reversed(original.is_reversed());
    let inner_face = topo.add_face(inner);
    let mut surround = Face::new(
        original.outer_wire(),
        vec![reverse_wire],
        original.surface().clone(),
    );
    surround.set_reversed(original.is_reversed());
    let surround_face = topo.add_face(surround);
    let faces = topo
        .shell(shells[owner])?
        .faces()
        .iter()
        .copied()
        .flat_map(|candidate| {
            if candidate == face {
                vec![surround_face, inner_face]
            } else {
                vec![candidate]
            }
        })
        .collect();
    let replacement = topo.add_shell(Shell::new(faces)?);
    let mut outer = source.outer_shell();
    let mut cavities = source.inner_shells().to_vec();
    if owner == 0 {
        outer = replacement;
    } else {
        cavities[owner - 1] = replacement;
    }
    let result = topo.add_solid(Solid::new(outer, cavities));
    Ok(Partition {
        solid: result,
        inner_face,
        surround_face,
        inner_wire,
    })
}

fn record(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    pending: remus_topology::journal::PendingOp,
    partition: Partition,
) -> Result<PlanarPolygonImprintResult, OperationsError> {
    let mut generated = std::collections::BTreeSet::new();
    for oriented in topo.wire(partition.inner_wire)?.edges() {
        let edge = topo.edge(oriented.edge())?;
        generated.extend([
            EntityKey::edge(oriented.edge().index()),
            EntityKey::vertex(edge.start().index()),
            EntityKey::vertex(edge.end().index()),
        ]);
    }
    let keys = solid_entity_keys(topo, partition.solid)?;
    let mut draft = EvolutionDraft::construction();
    draft.add_scope(keys.clone());
    let source_face = EntityKey::face(face.index());
    for key in keys {
        let event = if key == EntityKey::face(partition.inner_face.index())
            || key == EntityKey::face(partition.surround_face.index())
        {
            EventDraft::Modified { from: source_face }
        } else if generated.contains(&key) {
            EventDraft::Generated {
                sources: vec![source_face],
            }
        } else {
            EventDraft::Preserved { from: key }
        };
        draft.push(key, event);
    }
    let op = topo.journal_record_evolution(pending, draft)?;
    topo.propagate_attributes_for_op(op, false)?;
    if let Some(attributes) = topo.attributes().solid(solid).cloned() {
        topo.set_solid_attributes(partition.solid, attributes)?;
    }
    Ok(PlanarPolygonImprintResult {
        solid: partition.solid,
        inner_face: partition.inner_face,
        surround_face: partition.surround_face,
        op,
    })
}

/// Context-aware planar polygon imprint, with cancellation and explicit work
/// admission before topology allocation. No approximation fallback is used.
///
/// # Errors
/// Refuses invalid tolerances, cancellation, exhausted segment/pair budgets,
/// unsupported geometry or result validation, restoring live state and history.
pub fn imprint_planar_polygon_with_context(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    points: &[Point3],
    context: &OperationContext,
) -> Result<PlanarPolygonImprintResult, OperationsError> {
    remus_topology::transaction::run_transacted(topo, |topo| {
        let qualified = qualify(topo, solid, face, points, context)?;
        // Before any private allocation, capture the pre-operation scope.
        let pending = begin_scoped(topo, "imprint_planar_polygon", &[solid])?;
        let partition = construct(topo, face, qualified, context.tolerance.linear)?;
        validate(topo, partition.solid)?;
        context.check_cancelled()?;
        record(topo, solid, face, pending, partition)
    })
}

#[cfg(test)]
mod tests;
