//! Draft angle operation for injection molding applications.
//!
//! Applies a taper to selected faces of a solid relative to a pull direction.
//!
//! Drafting a face moves its boundary, so the faces around it have to be
//! re-trimmed against the new plane. [`draft`] does that by relocating the
//! shared corners and rebuilding only the wires those corners sit on; every
//! other face — and every hole in every face — is carried through verbatim.
//! Anything it cannot re-trim exactly is refused by name rather than
//! approximated.

use std::collections::{BTreeMap, BTreeSet};

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;
use remus_topology::wire::WireId;

use crate::OperationsError;
use crate::boolean::{FaceSpec, assemble_solid_mixed_with_history};
use crate::dot_normal_point;
use crate::evolution::EvolutionMap;

/// Operation name carried by [`OperationsError::Unsupported`] refusals.
const OP: &str = "draft";

/// Smallest `|n1 · (n2 × n3)|` (unit normals) accepted for a three-plane
/// corner. Below this the planes are too near-parallel for the corner position
/// to be meaningful, and the draft is refused rather than emitting a far-away
/// intersection point.
const MIN_PLANE_TRIPLE_DET: f64 = 1e-6;

/// Smallest `|n × pull|` accepted for a drafted face. At zero the face lies on
/// the parting plane and there is no axis to tilt it about.
const MIN_DRAFT_AXIS_SINE: f64 = 1e-6;

/// A drafted corner may move at most this multiple of `span * |tan(angle)|`,
/// where `span` is the body's bounding-box diagonal. A genuine draft moves a
/// corner by its height above the neutral plane times `tan(angle)`; anything
/// far beyond that is a near-parallel plane solve blowing up, not a taper.
const MAX_DRAFT_DISPLACEMENT_FACTOR: f64 = 4.0;

/// Most distinct planes accepted at a single relocated corner. Past this the
/// corner is a vertex of a blended or faceted feature, not a box corner, and
/// its drafted position is not determined by plane intersection.
const MAX_PLANES_AT_CORNER: usize = 8;

/// Deflection for the closing volume check. Only the sign matters, so this is
/// deliberately coarse.
const VOLUME_DEFLECTION: f64 = 0.05;

fn unsupported(reason: impl Into<String>) -> OperationsError {
    OperationsError::Unsupported {
        operation: OP,
        reason: reason.into(),
    }
}

/// A face's plane in `normal · p = d` form, with `normal` the face's *outward*
/// normal (the stored surface normal flipped when the face is reversed).
#[derive(Clone, Copy)]
struct Plane {
    normal: Vec3,
    d: f64,
}

impl Plane {
    fn distance(self, p: Point3) -> f64 {
        dot_normal_point(self.normal, p) - self.d
    }

    /// The point of this plane closest to the origin.
    fn anchor(self) -> Point3 {
        let v = self.normal * self.d;
        Point3::new(v.x(), v.y(), v.z())
    }
}

/// Apply a draft angle to selected faces of a solid.
///
/// Tapers the specified faces by `angle_radians` relative to `pull_direction`.
/// The neutral plane is defined by `neutral_point` and `pull_direction`: a
/// point of a drafted face is displaced along that face's outward normal by
/// `height * tan(angle_radians)`, where `height` is its signed distance above
/// the neutral plane. Points on the neutral plane therefore stay put, points
/// above it move outward and points below it move inward. For a wall square to
/// the pull direction — the usual moulding case — this is exactly the wall
/// tilted by `angle_radians` about its intersection with the neutral plane.
///
/// That displacement is an affine map, so a drafted planar face stays planar.
/// Its corners are shared with its neighbours, so each one is relocated to the
/// intersection of the planes meeting there, with the drafted faces
/// contributing their new planes: the neighbours are re-trimmed against the
/// taper rather than left behind, and the shell closes.
///
/// Faces the draft does not move are carried through verbatim, keeping their
/// surface, their curved edges and all of their inner wires. A face the draft
/// does move keeps its inner wires too — and if the draft would move an inner
/// wire, the operation is refused, because a hole rim that moves off the wall
/// that owns it cannot be rebuilt from outer-wire positions.
///
/// The result is checked with [`crate::validate::validate_solid`] and required
/// to enclose positive volume before it is returned.
///
/// # Errors
///
/// Returns [`OperationsError::InvalidInput`] if `angle_radians` is zero,
/// `pull_direction` is zero-length, no face is selected, a selected face is
/// not part of the solid's outer shell, or a selected face is not planar.
///
/// Returns [`OperationsError::Unsupported`] if the taper has no exact
/// construction here — the solid has cavity shells, a drafted face carries an
/// inner wire, a drafted face lies on the parting plane, a relocated corner
/// also lies on a curved face or on fewer than three distinct planes, a wire
/// that must be rebuilt carries a curved edge, a relocated corner would leave
/// its own face's plane or travel implausibly far, the taper slides two
/// corners of a rebuilt wire past each other (the face would fold over its
/// neighbours), or the drafted shell fails validation.
pub fn draft(
    topo: &mut Topology,
    solid: SolidId,
    draft_faces: &[FaceId],
    pull_direction: Vec3,
    neutral_point: Point3,
    angle_radians: f64,
) -> Result<SolidId, OperationsError> {
    Ok(draft_with_evolution(
        topo,
        solid,
        draft_faces,
        pull_direction,
        neutral_point,
        angle_radians,
    )?
    .0)
}

/// [`draft`] with construction-derived face evolution.
///
/// Every result face derives from exactly one input face — drafted faces are
/// rebuilt on their tilted plane, re-trimmed neighbours keep their surface
/// with a substituted boundary, and untouched faces are copied — so the map
/// records each as `modified`, straight from the construction, never from
/// geometric matching.
///
/// # Errors
///
/// Exactly [`draft`]'s errors.
pub fn draft_with_evolution(
    topo: &mut Topology,
    solid: SolidId,
    draft_faces: &[FaceId],
    pull_direction: Vec3,
    neutral_point: Point3,
    angle_radians: f64,
) -> Result<(SolidId, EvolutionMap), OperationsError> {
    let tol = Tolerance::new();

    if angle_radians.abs() <= tol.angular {
        return Err(OperationsError::InvalidInput {
            reason: "draft angle must be non-zero".into(),
        });
    }
    if draft_faces.is_empty() {
        return Err(OperationsError::InvalidInput {
            reason: "must select at least one face to draft".into(),
        });
    }

    let pull = pull_direction.normalize()?;
    // Neutral plane: passes through neutral_point with normal = pull direction.
    let neutral_d = dot_normal_point(pull, neutral_point);

    let solid_data = topo.solid(solid)?;
    if !solid_data.inner_shells().is_empty() {
        return Err(unsupported(format!(
            "solid has {} cavity shell(s); draft only operates on the outer shell",
            solid_data.inner_shells().len()
        )));
    }
    let all_faces: Vec<FaceId> = topo.shell(solid_data.outer_shell())?.faces().to_vec();

    let position_of: BTreeMap<usize, usize> = all_faces
        .iter()
        .enumerate()
        .map(|(pos, f)| (f.index(), pos))
        .collect();

    let mut drafted = vec![false; all_faces.len()];
    for f in draft_faces {
        let Some(&pos) = position_of.get(&f.index()) else {
            return Err(OperationsError::InvalidInput {
                reason: format!("face {} is not part of the solid's outer shell", f.index()),
            });
        };
        drafted[pos] = true;
    }

    let mut planes = face_planes(topo, &all_faces)?;
    let tan_angle = angle_radians.tan();

    for (pos, &fid) in all_faces.iter().enumerate() {
        if !drafted[pos] {
            continue;
        }
        let face = topo.face(fid)?;
        // Vertex manipulation requires a plane.
        let Some(plane) = planes[pos] else {
            return Err(OperationsError::InvalidInput {
                reason: "draft target faces must be planar".into(),
            });
        };
        if !face.inner_wires().is_empty() {
            return Err(unsupported(format!(
                "face {} carries {} inner wire(s); drafting it moves each hole's \
                 rim off the wall that owns it, and re-trimming those walls \
                 against the drafted plane is not implemented",
                fid.index(),
                face.inner_wires().len()
            )));
        }
        require_line_wire(topo, face.outer_wire(), fid)?;
        planes[pos] = Some(tilt(plane, pull, neutral_d, tan_angle, fid)?);
    }

    let faces_at = faces_at_vertex(topo, &all_faces)?;
    let span = model_span(topo, &faces_at)?;
    // A corner solved from near-parallel planes lands far from anywhere; a real
    // taper cannot move a corner further than the body's own extent times the
    // draft slope.
    let max_travel = span * tan_angle.abs() * MAX_DRAFT_DISPLACEMENT_FACTOR + tol.linear;
    // Scale-relative slack for "is this point still on that plane", following
    // the crate's own `approx_eq` convention. Never loosened to admit a case.
    let eps = tol.linear.max(span * tol.relative);

    let mut moved: BTreeMap<usize, Point3> = BTreeMap::new();
    let mut resolved: BTreeSet<usize> = BTreeSet::new();
    for (pos, &fid) in all_faces.iter().enumerate() {
        if !drafted[pos] {
            continue;
        }
        for vid in wire_vertices(topo, topo.face(fid)?.outer_wire())? {
            if !resolved.insert(vid.index()) {
                continue;
            }
            let original = topo.vertex(vid)?.point();
            let at = faces_at
                .get(&vid.index())
                .ok_or_else(|| unsupported(format!("vertex {} belongs to no face", vid.index())))?;
            let corner = resolve_corner(at, &planes, vid.index(), eps)?;
            let travel = (corner - original).length();
            if travel > max_travel {
                return Err(unsupported(format!(
                    "the draft would move corner {} by {travel:.6}, far beyond the \
                     {max_travel:.6} a taper of this angle can reach; the faces \
                     meeting there are too near-parallel to re-trim",
                    vid.index()
                )));
            }
            // Leave untouched corners bit-identical to the input so the faces
            // that merely copy them still share exactly one vertex.
            if travel > tol.linear {
                moved.insert(vid.index(), corner);
            }
        }
    }

    let specs = build_specs(topo, &all_faces, &drafted, &planes, &moved, eps, tol)?;
    debug_assert_eq!(specs.len(), all_faces.len());
    let assembly = assemble_solid_mixed_with_history(topo, &specs, tol)?;
    let result = assembly.solid;
    let mut evolution = EvolutionMap::exact();
    for (i, out) in assembly.faces_by_spec.iter().enumerate() {
        if let (Some(out), Some(src)) = (out, all_faces.get(i)) {
            evolution.add_modified(src.index(), out.index());
        }
    }

    let report = crate::validate::validate_solid(topo, result)?;
    if !report.is_valid() {
        let detail: Vec<&str> = report
            .issues
            .iter()
            .filter(|i| i.severity == crate::validate::Severity::Error)
            .map(|i| i.description.as_str())
            .collect();
        return Err(unsupported(format!(
            "drafted shell failed validation ({})",
            detail.join("; ")
        )));
    }
    // A shell can pass the structural checks and still be turned inside out.
    let volume = crate::measure::solid_volume(topo, result, VOLUME_DEFLECTION)?;
    if !volume.is_finite() || volume <= 0.0 {
        return Err(unsupported(format!(
            "drafted shell encloses no volume ({volume}); the taper turned the \
             body inside out"
        )));
    }

    Ok((result, evolution))
}

/// Every face's outward plane, or `None` where the face is not planar.
///
/// `d` is recomputed from a boundary vertex rather than read from the stored
/// surface, which may be stale after earlier rebuilds.
fn face_planes(
    topo: &Topology,
    all_faces: &[FaceId],
) -> Result<Vec<Option<Plane>>, OperationsError> {
    let mut planes = Vec::with_capacity(all_faces.len());
    for &fid in all_faces {
        let face = topo.face(fid)?;
        let plane = match (face.surface(), face.effective_plane_normal()) {
            (FaceSurface::Plane { .. }, Some(normal)) => {
                let normal = normal.normalize()?;
                let first = *wire_vertices(topo, face.outer_wire())?
                    .first()
                    .ok_or_else(|| {
                        unsupported(format!("face {} has an empty outer wire", fid.index()))
                    })?;
                let anchor = topo.vertex(first)?.point();
                Some(Plane {
                    normal,
                    d: dot_normal_point(normal, anchor),
                })
            }
            _ => None,
        };
        planes.push(plane);
    }
    Ok(planes)
}

/// The drafted plane of `plane`.
///
/// The draft displaces a point `q` of the face to
/// `M(q) = q + n * tan(angle) * (pull · q - neutral_d)`. `M` is affine, so it
/// carries the face's plane to another plane, whose normal is `M^-T n`.
fn tilt(
    plane: Plane,
    pull: Vec3,
    neutral_d: f64,
    tan_angle: f64,
    fid: FaceId,
) -> Result<Plane, OperationsError> {
    let n = plane.normal;
    if n.cross(pull).length() <= MIN_DRAFT_AXIS_SINE {
        return Err(unsupported(format!(
            "face {} is square to the pull direction; a face on the parting plane \
             has no axis to draft about",
            fid.index()
        )));
    }
    // M = I + tan * n (x) pull, so M^-1 = I - tan * n (x) pull / denom
    // (Sherman-Morrison) and M^-T n = n - tan * pull / denom.
    let denom = 1.0 + tan_angle * n.dot(pull);
    if denom.abs() <= MIN_PLANE_TRIPLE_DET {
        return Err(unsupported(format!(
            "a draft of this angle folds face {} back onto itself",
            fid.index()
        )));
    }
    let tilted = (n - pull * (tan_angle / denom)).normalize()?;
    let anchor = plane.anchor();
    let displaced = anchor + n * (tan_angle * (dot_normal_point(pull, anchor) - neutral_d));
    Ok(Plane {
        normal: tilted,
        d: dot_normal_point(tilted, displaced),
    })
}

/// Shell positions of the faces meeting at each vertex, keyed by vertex index.
fn faces_at_vertex(
    topo: &Topology,
    all_faces: &[FaceId],
) -> Result<BTreeMap<usize, BTreeSet<usize>>, OperationsError> {
    let mut at: BTreeMap<usize, BTreeSet<usize>> = BTreeMap::new();
    for (pos, &fid) in all_faces.iter().enumerate() {
        let face = topo.face(fid)?;
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid)?.edges() {
                let edge = topo.edge(oe.edge())?;
                at.entry(edge.start().index()).or_default().insert(pos);
                at.entry(edge.end().index()).or_default().insert(pos);
            }
        }
    }
    Ok(at)
}

/// Bounding-box diagonal over every vertex the shell references.
fn model_span(
    topo: &Topology,
    faces_at: &BTreeMap<usize, BTreeSet<usize>>,
) -> Result<f64, OperationsError> {
    let mut bounds: Option<(Point3, Point3)> = None;
    for &vertex in faces_at.keys() {
        let vid = topo
            .vertex_id_from_index(vertex)
            .ok_or_else(|| unsupported(format!("vertex {vertex} disappeared")))?;
        let p = topo.vertex(vid)?.point();
        bounds = Some(match bounds {
            None => (p, p),
            Some((lo, hi)) => (
                Point3::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z())),
                Point3::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z())),
            ),
        });
    }
    Ok(bounds.map_or(0.0, |(lo, hi)| (hi - lo).length()))
}

/// The drafted position of a corner: the point common to every face meeting
/// there, with drafted faces contributing their tilted planes.
fn resolve_corner(
    at: &BTreeSet<usize>,
    planes: &[Option<Plane>],
    vertex: usize,
    eps: f64,
) -> Result<Point3, OperationsError> {
    let mut distinct: Vec<Plane> = Vec::new();
    for &pos in at {
        let Some(plane) = planes[pos] else {
            return Err(unsupported(format!(
                "corner {vertex} of a drafted face also lies on a curved face; \
                 re-trimming a curved neighbour against the drafted plane is not \
                 implemented"
            )));
        };
        if !distinct
            .iter()
            .any(|p| (p.normal - plane.normal).length() <= eps && (p.d - plane.d).abs() <= eps)
        {
            distinct.push(plane);
        }
        if distinct.len() > MAX_PLANES_AT_CORNER {
            return Err(unsupported(format!(
                "corner {vertex} lies on more than {MAX_PLANES_AT_CORNER} distinct \
                 planes; its drafted position is not determined by intersecting them"
            )));
        }
    }
    if distinct.len() < 3 {
        return Err(unsupported(format!(
            "corner {vertex} lies on only {} distinct plane(s); the draft does not \
             determine where it moves to",
            distinct.len()
        )));
    }

    // Solve from the best-conditioned triple, then require every other plane to
    // pass through the same point — otherwise the corner is over-constrained
    // and the faces around it cannot all be re-trimmed to meet there.
    let mut best: Option<(f64, Point3)> = None;
    for i in 0..distinct.len() {
        for j in (i + 1)..distinct.len() {
            for k in (j + 1)..distinct.len() {
                let bc = distinct[j].normal.cross(distinct[k].normal);
                let det = distinct[i].normal.dot(bc);
                if det.abs() <= MIN_PLANE_TRIPLE_DET
                    || best.is_some_and(|(best_det, _)| det.abs() <= best_det)
                {
                    continue;
                }
                let ca = distinct[k].normal.cross(distinct[i].normal);
                let ab = distinct[i].normal.cross(distinct[j].normal);
                let v =
                    (bc * distinct[i].d + ca * distinct[j].d + ab * distinct[k].d) * (1.0 / det);
                best = Some((det.abs(), Point3::new(v.x(), v.y(), v.z())));
            }
        }
    }

    let Some((_, corner)) = best else {
        return Err(unsupported(format!(
            "the planes at corner {vertex} are too near-parallel to intersect in a \
             well-conditioned point"
        )));
    };
    for plane in &distinct {
        if plane.distance(corner).abs() > eps {
            return Err(unsupported(format!(
                "the faces at corner {vertex} do not meet in a single point after \
                 drafting; re-trimming them would bend one off its own plane"
            )));
        }
    }
    Ok(corner)
}

/// One [`FaceSpec`] per face of the shell.
#[allow(clippy::too_many_arguments)]
fn build_specs(
    topo: &Topology,
    all_faces: &[FaceId],
    drafted: &[bool],
    planes: &[Option<Plane>],
    moved: &BTreeMap<usize, Point3>,
    eps: f64,
    tol: Tolerance,
) -> Result<Vec<FaceSpec>, OperationsError> {
    let mut specs = Vec::with_capacity(all_faces.len());

    for (pos, &fid) in all_faces.iter().enumerate() {
        let face = topo.face(fid)?;

        if drafted[pos] {
            // The drafted face was refused above if it had any inner wire, so
            // the empty `inner_wires` below states a fact rather than dropping
            // one. That is the whole difference from the version of this
            // function that silently filled every hole it touched.
            debug_assert!(face.inner_wires().is_empty());
            let outer = substitute_wire(topo, face.outer_wire(), fid, moved, tol)?;
            if outer.len() < 3 {
                return Err(unsupported(format!(
                    "the draft collapses face {} to {} corner(s)",
                    fid.index(),
                    outer.len()
                )));
            }
            let plane = planes[pos].ok_or_else(|| {
                unsupported(format!("drafted face {} lost its plane", fid.index()))
            })?;
            // `Plane::normal` is the outward normal; put the stored surface
            // normal back the way round the face carries it.
            let normal = if face.is_reversed() {
                -plane.normal
            } else {
                plane.normal
            };
            let d = dot_normal_point(normal, outer[0]);
            specs.push(if face.is_reversed() {
                FaceSpec::Surface {
                    vertices: outer,
                    surface: FaceSurface::Plane { normal, d },
                    reversed: true,
                    inner_wires: vec![],
                }
            } else {
                FaceSpec::Planar {
                    vertices: outer,
                    normal,
                    d,
                    inner_wires: vec![],
                }
            });
            continue;
        }

        for (slot, &wid) in face.inner_wires().iter().enumerate() {
            if wire_is_touched(topo, wid, moved)? {
                return Err(unsupported(format!(
                    "the draft moves inner wire {slot} of face {}; that hole lies \
                     inside the drafted region so its geometry genuinely changes, \
                     and re-trimming it against the taper is not implemented",
                    fid.index()
                )));
            }
        }

        if !wire_is_touched(topo, face.outer_wire(), moved)? {
            // Untouched: carry the face through with its exact surface, its
            // curved edges and every one of its inner wires.
            specs.push(FaceSpec::Existing {
                face: fid,
                outer: None,
            });
            continue;
        }

        require_line_wire(topo, face.outer_wire(), fid)?;
        let outer = substitute_wire(topo, face.outer_wire(), fid, moved, tol)?;
        if outer.len() < 3 {
            return Err(unsupported(format!(
                "the draft collapses face {} to {} corner(s)",
                fid.index(),
                outer.len()
            )));
        }
        let plane = planes[pos].ok_or_else(|| {
            unsupported(format!(
                "the draft moves a corner of curved face {}; re-trimming a curved \
                 neighbour against the taper is not implemented",
                fid.index()
            ))
        })?;
        for p in &outer {
            if plane.distance(*p).abs() > eps {
                return Err(unsupported(format!(
                    "re-trimming face {} against the taper would pull it off its \
                     own plane",
                    fid.index()
                )));
            }
        }
        // Replacement outer wire, inner wires still copied verbatim.
        specs.push(FaceSpec::Existing {
            face: fid,
            outer: Some(outer),
        });
    }

    Ok(specs)
}

/// The vertices of `wire`, in traversal order.
fn wire_vertices(topo: &Topology, wire: WireId) -> Result<Vec<VertexId>, OperationsError> {
    let mut ids = Vec::new();
    for oe in topo.wire(wire)?.edges() {
        ids.push(oe.oriented_start(topo.edge(oe.edge())?));
    }
    Ok(ids)
}

/// Whether the draft relocates any vertex of `wire`.
fn wire_is_touched(
    topo: &Topology,
    wire: WireId,
    moved: &BTreeMap<usize, Point3>,
) -> Result<bool, OperationsError> {
    for oe in topo.wire(wire)?.edges() {
        let edge = topo.edge(oe.edge())?;
        if moved.contains_key(&edge.start().index()) || moved.contains_key(&edge.end().index()) {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Refuse a wire the assembler would have to rebuild from straight chords when
/// it does not consist of straight edges to begin with.
fn require_line_wire(topo: &Topology, wire: WireId, fid: FaceId) -> Result<(), OperationsError> {
    for oe in topo.wire(wire)?.edges() {
        if !matches!(topo.edge(oe.edge())?.curve(), EdgeCurve::Line) {
            return Err(unsupported(format!(
                "the draft has to rebuild the boundary of face {}, which carries a \
                 curved edge; re-trimming a curved boundary against the taper is \
                 not implemented",
                fid.index()
            )));
        }
    }
    Ok(())
}

/// Walk a wire's vertices, substituting relocated corners and dropping corners
/// that collapsed onto their neighbour.
///
/// Every edge of the wire stays on its own support line (the intersection of
/// the two face planes that meet along it), so the only way the rebuilt wire
/// can degenerate is for an edge's two corners to slide past each other. That
/// happens whenever the taper pushes a face outward by more than its width
/// allows — on a convex body each corner moves toward the face's middle by
/// `δ / tan(dihedral)`, which for a narrow facet between shallow neighbours
/// exceeds the facet's own width — and the wire then winds against its normal.
/// Assembling that gives a shell that passes validation and encloses positive
/// volume while the folded face overlaps its neighbours, so a reversed edge
/// is refused here by name.
fn substitute_wire(
    topo: &Topology,
    wire: WireId,
    fid: FaceId,
    moved: &BTreeMap<usize, Point3>,
    tol: Tolerance,
) -> Result<Vec<Point3>, OperationsError> {
    let ids = wire_vertices(topo, wire)?;
    let mut original = Vec::with_capacity(ids.len());
    let mut relocated = Vec::with_capacity(ids.len());
    for vid in &ids {
        let p = topo.vertex(*vid)?.point();
        original.push(p);
        relocated.push(moved.get(&vid.index()).copied().unwrap_or(p));
    }

    for i in 0..ids.len() {
        let j = (i + 1) % ids.len();
        let before = original[j] - original[i];
        let after = relocated[j] - relocated[i];
        if after.length() > tol.linear && before.dot(after) < 0.0 {
            return Err(unsupported(format!(
                "the taper at this angle eliminates face {}: its corners {} and {} \
                 slide past each other, so the face would fold over its neighbours \
                 instead of tilting",
                fid.index(),
                ids[i].index(),
                ids[j].index()
            )));
        }
    }

    let mut points: Vec<Point3> = Vec::new();
    for p in relocated {
        if points
            .last()
            .is_none_or(|last| (*last - p).length() > tol.linear)
        {
            points.push(p);
        }
    }
    while points.len() >= 2 && (points[0] - points[points.len() - 1]).length() <= tol.linear {
        points.pop();
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::tolerance::Tolerance;
    use remus_math::vec::{Point3, Vec3};
    use remus_topology::Topology;
    use remus_topology::face::FaceSurface;
    use remus_topology::test_utils::make_unit_cube_manifold;

    use super::*;

    /// Helper: find faces whose normal is approximately equal to `target`.
    fn find_faces(topo: &Topology, solid: SolidId, target: Vec3) -> Vec<FaceId> {
        let tol = Tolerance::loose();
        let s = topo.solid(solid).unwrap();
        let sh = topo.shell(s.outer_shell()).unwrap();
        sh.faces()
            .iter()
            .filter(|&&fid| {
                let f = topo.face(fid).unwrap();
                if let FaceSurface::Plane { normal, .. } = f.surface() {
                    tol.approx_eq(normal.x(), target.x())
                        && tol.approx_eq(normal.y(), target.y())
                        && tol.approx_eq(normal.z(), target.z())
                } else {
                    false
                }
            })
            .copied()
            .collect()
    }

    #[test]
    fn draft_single_face() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let right_faces = find_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0));
        assert_eq!(right_faces.len(), 1);

        let result = draft(
            &mut topo,
            cube,
            &right_faces,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            5.0_f64.to_radians(),
        )
        .unwrap();

        let s = topo.solid(result).unwrap();
        let sh = topo.shell(s.outer_shell()).unwrap();
        assert_eq!(
            sh.faces().len(),
            6,
            "drafted solid should still have 6 faces"
        );

        // The +X wall leans out by z * tan(5 deg), adding a unit-wide wedge.
        let vol = crate::measure::solid_volume(&topo, result, 0.1).unwrap();
        let wedge = 5.0_f64.to_radians().tan() / 2.0;
        assert!(
            (vol - (1.0 + wedge)).abs() < 1e-6,
            "expected 1 + wedge {wedge}, got {vol}"
        );
    }

    #[test]
    fn draft_preserves_non_draft_faces() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        let right_faces = find_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0));
        let result = draft(
            &mut topo,
            cube,
            &right_faces,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            5.0_f64.to_radians(),
        )
        .unwrap();

        // The top and bottom faces should still be planar with ±Z normals.
        let top = find_faces(&topo, result, Vec3::new(0.0, 0.0, 1.0));
        let bottom = find_faces(&topo, result, Vec3::new(0.0, 0.0, -1.0));
        assert_eq!(top.len(), 1, "should still have top face");
        assert_eq!(bottom.len(), 1, "should still have bottom face");
    }

    #[test]
    fn draft_zero_angle_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let right = find_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0));

        assert!(
            draft(
                &mut topo,
                cube,
                &right,
                Vec3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 0.0),
                0.0,
            )
            .is_err()
        );
    }

    #[test]
    fn draft_zero_pull_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let right = find_faces(&topo, cube, Vec3::new(1.0, 0.0, 0.0));

        assert!(
            draft(
                &mut topo,
                cube,
                &right,
                Vec3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
                5.0_f64.to_radians(),
            )
            .is_err()
        );
    }

    #[test]
    fn draft_empty_selection_error() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);

        assert!(matches!(
            draft(
                &mut topo,
                cube,
                &[],
                Vec3::new(0.0, 0.0, 1.0),
                Point3::new(0.0, 0.0, 0.0),
                5.0_f64.to_radians(),
            ),
            Err(OperationsError::InvalidInput { .. })
        ));
    }

    /// A face square to the pull direction lies on the parting plane; there is
    /// no axis to tilt it about, so the draft is refused by name.
    #[test]
    fn draft_face_on_parting_plane_refused() {
        let mut topo = Topology::new();
        let cube = make_unit_cube_manifold(&mut topo);
        let top = find_faces(&topo, cube, Vec3::new(0.0, 0.0, 1.0));

        let err = draft(
            &mut topo,
            cube,
            &top,
            Vec3::new(0.0, 0.0, 1.0),
            Point3::new(0.0, 0.0, 0.0),
            5.0_f64.to_radians(),
        )
        .unwrap_err();

        assert!(
            matches!(err, OperationsError::Unsupported { operation, .. } if operation == "draft"),
            "expected a typed refusal, got {err:?}"
        );
    }

    /// Extrude a convex polygon with one deliberately narrow facet between two
    /// shallow neighbours: chord 0.42 at ~54° on a radius-6 prism, with
    /// dihedrals of 5.5° and 4° to the facets either side.
    fn narrow_facet_prism(topo: &mut Topology) -> (SolidId, FaceId) {
        let angles: [f64; 25] = [
            0.0, 15.0, 30.0, 45.0, 52.0, 56.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0, 150.0, 165.0,
            180.0, 195.0, 210.0, 225.0, 240.0, 255.0, 270.0, 285.0, 300.0, 315.0, 330.0,
        ];
        let points: Vec<Point3> = angles
            .iter()
            .map(|a| {
                let (s, c) = a.to_radians().sin_cos();
                Point3::new(6.0 * c, 6.0 * s, 0.0)
            })
            .collect();
        let base = remus_topology::builder::make_planar_face(topo, &points, tol_linear()).unwrap();
        let prism = crate::extrude::extrude(topo, base, Vec3::new(0.0, 0.0, 1.0), 6.0).unwrap();
        let (s, c) = 54.0_f64.to_radians().sin_cos();
        let narrow = find_faces(topo, prism, Vec3::new(c, s, 0.0));
        assert_eq!(
            narrow.len(),
            1,
            "expected exactly one facet with normal at 54°"
        );
        (prism, narrow[0])
    }

    fn tol_linear() -> f64 {
        Tolerance::new().linear
    }

    fn assert_watertight_at(topo: &Topology, solid: SolidId, deflection: f64) {
        let mesh = crate::tessellate::tessellate_solid(topo, solid, deflection).unwrap();
        let b = crate::tessellate::boundary_edge_count(&mesh);
        let n = crate::tessellate::non_manifold_edge_count(&mesh);
        assert!(
            b == 0 && n == 0,
            "tessellation at deflection {deflection} is not watertight: \
             {b} boundary edge(s), {n} non-manifold edge(s)"
        );
    }

    /// Pushing a narrow facet outward on a convex body slides each corner
    /// toward the facet's middle by `δ / tan(dihedral)`; here that exceeds the
    /// facet's width, the corners cross, and the face would wind against its
    /// own normal. The draft must refuse rather than emit the folded face.
    #[test]
    fn draft_refuses_facet_whose_corners_cross() {
        let mut topo = Topology::new();
        let (prism, narrow) = narrow_facet_prism(&mut topo);

        let err = draft(
            &mut topo,
            prism,
            &[narrow],
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 3.0),
            1.0_f64.to_radians(),
        )
        .unwrap_err();
        assert!(
            matches!(err, OperationsError::Unsupported { operation, .. } if operation == "draft"),
            "expected a typed refusal, got {err:?}"
        );
    }

    /// The same facet under a taper small enough that no corner reaches a
    /// neighbouring corner (0.25° about a neutral plane beyond the facet, so
    /// it moves inward by ~0.011 and widens by ~0.16 against neighbours 0.42
    /// wide) is a legitimate draft and must still succeed, proving the fold
    /// guard does not over-refuse.
    #[test]
    fn draft_widening_narrow_facet_succeeds() {
        let mut topo = Topology::new();
        let (prism, narrow) = narrow_facet_prism(&mut topo);

        let result = draft(
            &mut topo,
            prism,
            &[narrow],
            Vec3::new(1.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 3.0),
            0.25_f64.to_radians(),
        )
        .unwrap();
        assert_watertight_at(&topo, result, 0.003);
        assert_watertight_at(&topo, result, 0.1);
    }

    /// Fuzz finding (modifier_ops, corpus seed `draft-folded-facet`): a
    /// radius-6 cylinder cut by a tilted box comes back as a faceted prism,
    /// and a 1° outward taper of one of its facets crossed the corners. The
    /// folded face passed validation and the volume sign check, and only
    /// showed up as four wrongly-wound half-edges in the fine tessellation.
    ///
    /// Every facet of the body is drafted so the assertion does not depend on
    /// the boolean's face numbering: each accepted draft must tessellate
    /// watertight at the fuzz deflection and at a coarse one.
    #[test]
    fn draft_never_emits_folded_face_on_faceted_cylinder() {
        use std::f64::consts::FRAC_PI_6;

        use remus_math::mat::Mat4;
        use remus_topology::explorer::solid_faces;

        use crate::boolean::{BooleanOp, boolean};
        use crate::primitives::{make_box, make_cylinder};
        use crate::transform::transform_solid;

        let mut base = Topology::new();
        let stock = make_cylinder(&mut base, 6.0, 6.0).unwrap();
        let tool = make_box(&mut base, 6.0, 1.0, 6.5).unwrap();
        let placement = Mat4::translation(-4.0, -4.0, -4.0) * Mat4::rotation_x(FRAC_PI_6);
        transform_solid(&mut base, tool, &placement).unwrap();
        let body = boolean(&mut base, BooleanOp::Cut, stock, tool).unwrap();
        let faces = solid_faces(&base, body).unwrap();
        let center = crate::measure::solid_bounding_box(&base, body)
            .unwrap()
            .center();

        let mut refused = 0;
        for &fid in &faces {
            let mut topo = base.clone();
            match draft(
                &mut topo,
                body,
                &[fid],
                Vec3::new(1.0, 0.0, 0.0),
                center,
                1.0_f64.to_radians(),
            ) {
                Ok(result) => {
                    assert_watertight_at(&topo, result, 0.002_878_027_577_040_323);
                    assert_watertight_at(&topo, result, 0.1);
                }
                Err(OperationsError::Unsupported {
                    operation: "draft", ..
                }) => refused += 1,
                Err(err) => assert!(
                    matches!(err, OperationsError::InvalidInput { .. }),
                    "face {}: unexpected error {err:?}",
                    fid.index()
                ),
            }
        }
        assert!(refused > 0, "the fuzz facet is expected to be refused");
    }
}
