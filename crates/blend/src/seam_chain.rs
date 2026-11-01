//! A fillet or chamfer along the seam where a round boss meets a block.
//!
//! A cylinder fused against a block meets it in a concave seam that runs up
//! one wall of the block, across its top and down another wall: a chain of
//! edges that all lie on the boss (the *rail*), each between the boss and a
//! different plane of the block (its *support*). Consecutive supports meet in
//! a sharp convex edge of the block, so the chain turns a corner there rather
//! than running on tangentially (the tangent case is [`crate::rail_chain`]).
//!
//! Each run of the chain gets its own band, exact for the two support
//! families a block offers a boss: a plane along the boss's axis meets it in a
//! ruling, whose fillet is a cylinder and whose chamfer is a plane, and a
//! plane across the axis meets it in a circle, whose fillet is a torus and
//! whose chamfer is a cone. Where two runs meet, each band carries on past the
//! corner until it meets the other one: the two bands mitre along their
//! intersection, which runs from the boss down to the block's convex edge, and
//! the sharp edge of the block now starts where the later band leaves it.
//! The mitre is the only curve that is not exact; it is fitted through solved
//! points of the intersection and checked against both bands.
//!
//! At either end of the chain the band stops in the plane of the face the
//! chain runs into. That face is notched to the band's end section, or grows
//! to meet it where the boss carries on past the block (a band across the top
//! of a block ends above the block's side wall, so the wall rises to the
//! band's end section).

use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::{Shell, ShellId};
use remus_topology::solid::Solid;
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use crate::analytic::edge_material_frames_at;
use crate::body_edit::EditScope;
use crate::builder_utils::add_certified_curve_edge;
use crate::trimmer::{self, CutPiece};
use crate::{BlendError, BlendFaceOrigins, BlendResult};
use remus_math::compact_sort::CompactSort;

/// The blend to put along the seam.
#[derive(Debug, Clone, Copy)]
pub enum SeamBlend {
    /// A rolling-ball fillet of this radius.
    Fillet(f64),
    /// A chamfer setting back this far along both faces.
    Chamfer(f64),
}

/// Samples along a mitre or a fitted end section.
const SAMPLES: usize = 48;
/// Vertex tolerance of the points this module makes.
const VERTEX_TOL: f64 = 1e-7;

/// The boss face every chain edge lies on, with its frame.
#[derive(Debug, Clone)]
struct Rail {
    face: FaceId,
    origin: Point3,
    axis: Vec3,
    radius: f64,
    x: Vec3,
    y: Vec3,
}

impl Rail {
    fn height(&self, p: Point3) -> f64 {
        self.axis.dot(p - self.origin)
    }

    fn angle(&self, p: Point3) -> f64 {
        let d = p - self.origin;
        d.dot(self.y).atan2(d.dot(self.x))
    }

    fn radial(&self, theta: f64) -> Vec3 {
        let (s, c) = theta.sin_cos();
        self.x * c + self.y * s
    }

    fn off_axis(&self, p: Point3) -> f64 {
        let d = p - self.origin;
        (d - self.axis * self.axis.dot(d)).length()
    }

    /// The circle of the rail's frame at height `h`, radius `radius`, whose
    /// parameter is the rail's angle.
    fn circle(&self, h: f64, radius: f64) -> Result<Circle3D, BlendError> {
        Ok(Circle3D::with_axes(
            self.origin + self.axis * h,
            self.axis,
            radius,
            self.x,
            self.y,
        )?)
    }
}

/// A band's cross-section, in its own two coordinates: the rail's x and y at
/// height zero for a ruling band, radial distance and height for a band
/// round the axis. `t` runs from the rail contact (0) to the support (1).
#[derive(Debug, Clone, Copy)]
enum Profile {
    Arc {
        center: [f64; 2],
        from: [f64; 2],
        toward: [f64; 2],
        span: f64,
        radius: f64,
    },
    Segment {
        from: [f64; 2],
        to: [f64; 2],
        /// The sharp edge the chamfer replaces: its material is on this side.
        corner: [f64; 2],
    },
}

impl Profile {
    fn at(&self, t: f64) -> [f64; 2] {
        match *self {
            Self::Arc {
                center,
                from,
                toward,
                span,
                radius,
            } => {
                let (s, c) = (t * span).sin_cos();
                [
                    center[0] + radius * (c * from[0] + s * toward[0]),
                    center[1] + radius * (c * from[1] + s * toward[1]),
                ]
            }
            Self::Segment { from, to, .. } => [
                (to[0] - from[0]).mul_add(t, from[0]),
                (to[1] - from[1]).mul_add(t, from[1]),
            ],
        }
    }
}

/// Which way the support runs against the boss.
#[derive(Debug, Clone, Copy)]
enum Family {
    /// A plane along the axis: the spine is a ruling, `s` is height.
    Ruling,
    /// A plane across the axis: the spine is a circle, `s` is the angle.
    Cap,
}

/// One run of the chain: its support, its band and its spine.
#[derive(Debug, Clone)]
struct Run {
    support: FaceId,
    /// The support's outward normal and offset.
    normal: Vec3,
    offset: f64,
    family: Family,
    profile: Profile,
    /// The chain vertices it runs between, in chain order, and the spine
    /// parameter at each.
    from: VertexId,
    to: VertexId,
    s_from: f64,
    s_to: f64,
}

impl Run {
    fn eval(&self, rail: &Rail, s: f64, t: f64) -> Point3 {
        let [a, b] = self.profile.at(t);
        match self.family {
            Family::Ruling => rail.origin + rail.x * a + rail.y * b + rail.axis * s,
            Family::Cap => rail.origin + rail.radial(s) * a + rail.axis * b,
        }
    }

    /// The band's surface and whether the face on it is reversed.
    fn surface(&self, rail: &Rail) -> Result<(FaceSurface, bool), BlendError> {
        let surface = match (self.family, self.profile) {
            (Family::Ruling, Profile::Arc { center, radius, .. }) => {
                FaceSurface::Cylinder(CylindricalSurface::new(
                    rail.origin + rail.x * center[0] + rail.y * center[1],
                    rail.axis,
                    radius,
                )?)
            }
            (Family::Cap, Profile::Arc { center, radius, .. }) => {
                FaceSurface::Torus(ToroidalSurface::with_axis(
                    rail.origin + rail.axis * center[1],
                    center[0],
                    radius,
                    rail.axis,
                )?)
            }
            (Family::Ruling, Profile::Segment { .. }) => {
                let a = self.eval(rail, 0.0, 0.0);
                let b = self.eval(rail, 0.0, 1.0);
                let normal = (b - a).cross(rail.axis).normalize()?;
                FaceSurface::Plane {
                    normal,
                    d: normal.dot(a - Point3::new(0.0, 0.0, 0.0)),
                }
            }
            (Family::Cap, Profile::Segment { from, to, .. }) => {
                // The generator's line reaches the axis at the apex.
                let lambda = -from[0] / (to[0] - from[0]);
                let apex_h = (to[1] - from[1]).mul_add(lambda, from[1]);
                let rise = to[1] - apex_h;
                let axis = if rise >= 0.0 { rail.axis } else { -rail.axis };
                FaceSurface::Cone(ConicalSurface::new(
                    rail.origin + rail.axis * apex_h,
                    axis,
                    rise.abs().atan2(to[0]),
                )?)
            }
        };
        // The material is behind the band: a fillet's ball is in front of it,
        // a chamfer's sharp edge behind it.
        let s = 0.5 * (self.s_from + self.s_to);
        let point = self.eval(rail, s, 0.5);
        let outward = match self.profile {
            Profile::Arc { center, .. } => self.lift(rail, s, center) - point,
            Profile::Segment { corner, .. } => point - self.lift(rail, s, corner),
        };
        let normal = match &surface {
            FaceSurface::Plane { normal, .. } => *normal,
            other => {
                let (u, v) = other
                    .project_point(point)
                    .ok_or_else(|| invalid("band projection failed"))?;
                other.normal(u, v)
            }
        };
        let reversed = normal.dot(outward) < 0.0;
        Ok((surface, reversed))
    }

    /// A point of the profile's plane at spine parameter `s`.
    fn lift(&self, rail: &Rail, s: f64, p: [f64; 2]) -> Point3 {
        match self.family {
            Family::Ruling => rail.origin + rail.x * p[0] + rail.y * p[1] + rail.axis * s,
            Family::Cap => rail.origin + rail.radial(s) * p[0] + rail.axis * p[1],
        }
    }

    /// The contact curve at `t` (0 on the rail, 1 on the support), whose
    /// parameter is the spine parameter; `None` for a ruling's line.
    fn contact(&self, rail: &Rail, t: f64) -> Result<Option<Circle3D>, BlendError> {
        let [a, b] = self.profile.at(t);
        Ok(match self.family {
            Family::Ruling => None,
            Family::Cap => Some(rail.circle(b, a)?),
        })
    }

    /// The exact cross-section at spine parameter `s`; `None` for a line.
    fn section(&self, rail: &Rail, s: f64) -> Result<Option<Circle3D>, BlendError> {
        let Profile::Arc { center, radius, .. } = self.profile else {
            return refused(line!());
        };
        let c = self.lift(rail, s, center);
        Ok(Some(match self.family {
            Family::Ruling => Circle3D::with_axes(c, rail.axis, radius, rail.x, rail.y)?,
            Family::Cap => {
                let u = rail.radial(s);
                Circle3D::with_axes(c, u.cross(rail.axis), radius, u, rail.axis)?
            }
        }))
    }
}

/// A curve between two of the build's points that lies on a band.
#[derive(Debug, Clone)]
enum SideCurve {
    /// A line.
    Line,
    /// An arc of this circle, the short way.
    Arc(Circle3D),
    /// A fitted curve through these points, first to last.
    Fitted(Vec<Point3>),
}

/// Where the chain turns a corner between run `i` and run `i + 1`.
#[derive(Debug, Clone)]
struct Mitre {
    /// Where the two rail contacts meet.
    p: Point3,
    /// Where the mitre reaches a support contact.
    m: Point3,
    /// Where the other band's support contact leaves the sharp edge.
    k: Point3,
    /// The mitre, `p` to `m`.
    curve: Vec<Point3>,
    /// Whether `m` is on the earlier run's support contact: then the later
    /// band's end section `m` to `k` lies in the earlier support's plane.
    m_on_earlier: bool,
    /// From `m` to `k` along the band that carries on past `m`; `None` when
    /// the mitre reaches the sharp edge itself (`m` is `k`), as an equal
    /// chamfer's does.
    end: Option<SideCurve>,
    /// Spine parameters of the rail contacts at `p`, and of the support
    /// contacts where they stop (`m` or `k`), for the earlier and the later
    /// run.
    rail_s: [f64; 2],
    support_s: [f64; 2],
}

/// Where the chain stops against a face it runs into.
#[derive(Debug, Clone)]
struct End {
    /// The chain's end vertex.
    vertex: VertexId,
    /// The face the chain runs into.
    face: FaceId,
    /// The face of the end plane on the rail's side of the vertex: `face`
    /// itself, or the last of a fan of coplanar faces round the vertex (a
    /// boolean that left a boss's foot unmerged with the block's bottom).
    rail_side: FaceId,
    /// The band's end section, from the rail contact `x` to the support
    /// contact `k`, in the face's plane.
    x: Point3,
    k: Point3,
    section: SideCurve,
    rail_s: f64,
    support_s: f64,
    /// The rail's curve in the face's plane from `x` back to `vertex`.
    closure: SideCurve,
}

/// The whole plan, computed before anything is built.
#[derive(Debug, Clone)]
struct Plan {
    rail: Rail,
    runs: Vec<Run>,
    mitres: Vec<Mitre>,
    ends: [End; 2],
    chain: Vec<EdgeId>,
}

/// Decline the seam: the edges are not one this module builds. Logged with
/// the source line of the check that declined it.
#[allow(clippy::unnecessary_wraps)]
fn refused<T>(line: u32) -> Result<Option<T>, BlendError> {
    log::debug!("seam blend declined (seam_chain.rs:{line})");
    Ok(None)
}

fn invalid(reason: &str) -> BlendError {
    BlendError::InvalidInput {
        reason: format!("seam blend: {reason}"),
    }
}

/// Blend `edges` as one seam when they are a concave chain along a boss whose
/// other face turns sharp corners, or `None` when they are not one (nothing
/// has been built then).
///
/// # Errors
///
/// Returns [`BlendError`] when the chain qualifies but a trim or the assembly
/// fails.
#[allow(clippy::too_many_arguments)]
pub fn blend(
    topo: &mut Topology,
    adjacency: &AdjacencyIndex,
    edges: &[EdgeId],
    kind: SeamBlend,
    scope: &mut EditScope,
    original_faces: &[FaceId],
    inner_shells: &[ShellId],
    requested: &[EdgeId],
) -> Result<Option<BlendResult>, BlendError> {
    let Some(plan) = plan(topo, adjacency, edges, kind)? else {
        return refused(line!());
    };
    log::debug!(
        "seam blend: {} runs, {} mitres along face {:?}",
        plan.runs.len(),
        plan.mitres.len(),
        plan.rail.face
    );
    build(topo, &plan, scope, original_faces, inner_shells, requested).map(Some)
}

// ===========================================================================
// Planning
// ===========================================================================

/// The seam's plan, or `None` when the edges are not such a seam.
#[allow(clippy::too_many_lines)]
fn plan(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    edges: &[EdgeId],
    kind: SeamBlend,
) -> Result<Option<Plan>, BlendError> {
    let size = match kind {
        SeamBlend::Fillet(r) | SeamBlend::Chamfer(r) => r,
    };
    if edges.is_empty() || !size.is_finite() || size <= 0.0 {
        return refused(line!());
    }
    // Every edge between two distinct faces, one of them common to all.
    let mut pairs = Vec::with_capacity(edges.len());
    for &edge in edges {
        let &[a, b] = adjacency.faces_for_edge(edge) else {
            return refused(line!());
        };
        if a == b {
            return refused(line!());
        }
        pairs.push([a, b]);
    }
    let common: Vec<FaceId> = pairs[0]
        .iter()
        .copied()
        .filter(|face| pairs.iter().all(|pair| pair.contains(face)))
        .filter(|&face| {
            topo.face(face)
                .is_ok_and(|f| matches!(f.surface(), FaceSurface::Cylinder(_)) && !f.is_reversed())
        })
        .collect();
    let &[rail_face] = common.as_slice() else {
        return refused(line!());
    };
    let FaceSurface::Cylinder(cylinder) = topo.face(rail_face)?.surface() else {
        return refused(line!());
    };
    let rail = Rail {
        face: rail_face,
        origin: cylinder.origin(),
        axis: cylinder.axis(),
        radius: cylinder.radius(),
        x: cylinder.x_axis(),
        y: cylinder.y_axis(),
    };

    // The chain, in order: an open path.
    let Some((ordered, vertices)) = order_path(topo, edges)? else {
        return refused(line!());
    };
    // Group consecutive edges on one support into runs.
    let support_of = |edge: EdgeId| -> FaceId {
        let pair = pairs[edges.iter().position(|&e| e == edge).unwrap_or(0)];
        if pair[0] == rail_face {
            pair[1]
        } else {
            pair[0]
        }
    };
    let mut groups: Vec<(FaceId, usize, usize)> = Vec::new();
    for (index, &edge) in ordered.iter().enumerate() {
        let support = support_of(edge);
        match groups.last_mut() {
            Some(last) if last.0 == support => last.2 = index + 1,
            _ => groups.push((support, index, index + 1)),
        }
    }
    let ruling_or_cap = |face: FaceId| -> Result<Option<(Vec3, f64, Family)>, BlendError> {
        let face = topo.face(face)?;
        let FaceSurface::Plane { normal, d } = face.surface() else {
            return refused(line!());
        };
        let (normal, offset) = if face.is_reversed() {
            (-*normal, -*d)
        } else {
            (*normal, *d)
        };
        let along = normal.dot(rail.axis);
        Ok(if along.abs() < 1e-9 {
            Some((normal, offset, Family::Ruling))
        } else if along.abs() > 1.0 - 1e-12 {
            Some((normal, offset, Family::Cap))
        } else {
            None
        })
    };
    // A face can be a support only once; and the seam must take at least a
    // corner or a run round the axis, which the walking builder cannot.
    let mut runs: Vec<Run> = Vec::with_capacity(groups.len());
    for (index, &(support, first, last)) in groups.iter().enumerate() {
        if groups[..index].iter().any(|g| g.0 == support) {
            return refused(line!());
        }
        let Some((normal, offset, family)) = ruling_or_cap(support)? else {
            return refused(line!());
        };
        for &edge in &ordered[first..last] {
            let frames = edge_material_frames_at(topo, edge, rail_face, support)?;
            if frames.and_then(|f| f.concave()) != Some(true) {
                return refused(line!());
            }
        }
        let (from, to) = (vertices[first], vertices[last]);
        let Some(run) = make_run(
            topo,
            &rail,
            support,
            normal,
            offset,
            family,
            &ordered[first..last],
            from,
            to,
            kind,
        )?
        else {
            return refused(line!());
        };
        runs.push(run);
    }
    // A lone ruling fillet is the walking builder's (exact cylinder band);
    // its chamfer is not.
    if runs.len() == 1
        && matches!(runs[0].family, Family::Ruling)
        && matches!(kind, SeamBlend::Fillet(_))
    {
        return refused(line!());
    }

    let scale = rail.radius.max(size).max(1.0);
    let mut mitres = Vec::with_capacity(runs.len().saturating_sub(1));
    for pair in runs.windows(2) {
        let Some(sharp) = shared_edge_at(topo, pair[0].support, pair[1].support, pair[0].to)?
        else {
            return refused(line!());
        };
        // The block's edge between the supports must be convex: the corner
        // the seam turns round.
        let frames = edge_material_frames_at(topo, sharp, pair[0].support, pair[1].support)?;
        if frames.and_then(|f| f.concave()) != Some(false) {
            return refused(line!());
        }
        let Some(mitre) = plan_mitre(&rail, &pair[0], &pair[1], scale)? else {
            return refused(line!());
        };
        mitres.push(mitre);
    }

    let first = &runs[0];
    let last = &runs[runs.len() - 1];
    let (Some(start), Some(end)) = (
        plan_end(topo, adjacency, &rail, first, first.from, &ordered, scale)?,
        plan_end(topo, adjacency, &rail, last, last.to, &ordered, scale)?,
    ) else {
        return refused(line!());
    };
    Ok(Some(Plan {
        rail,
        runs,
        mitres,
        ends: [start, end],
        chain: ordered,
    }))
}

/// The edges as one open path, with its vertices (one more than the edges).
/// A chain's edges in path order, and its vertices (one more than edges).
type Path = (Vec<EdgeId>, Vec<VertexId>);

fn order_path(topo: &Topology, edges: &[EdgeId]) -> Result<Option<Path>, BlendError> {
    let mut ends: Vec<(EdgeId, VertexId, VertexId)> = Vec::with_capacity(edges.len());
    for &edge in edges {
        let e = topo.edge(edge)?;
        if e.start() == e.end() {
            return refused(line!());
        }
        ends.push((edge, e.start(), e.end()));
    }
    let degree = |v: VertexId| ends.iter().filter(|(_, a, b)| *a == v || *b == v).count();
    let mut starts = ends
        .iter()
        .flat_map(|(_, a, b)| [*a, *b])
        .filter(|&v| degree(v) == 1)
        .collect::<Vec<_>>();
    starts.compact_sort_unstable_by_key(|v| v.index());
    starts.dedup();
    if starts.len() != 2
        || ends
            .iter()
            .flat_map(|(_, a, b)| [*a, *b])
            .any(|v| degree(v) > 2)
    {
        return refused(line!());
    }
    let mut vertex = starts[0];
    let mut ordered = Vec::with_capacity(edges.len());
    let mut vertices = vec![vertex];
    let mut used = vec![false; ends.len()];
    while ordered.len() < ends.len() {
        let Some(index) =
            (0..ends.len()).find(|&i| !used[i] && (ends[i].1 == vertex || ends[i].2 == vertex))
        else {
            return refused(line!());
        };
        used[index] = true;
        ordered.push(ends[index].0);
        vertex = if ends[index].1 == vertex {
            ends[index].2
        } else {
            ends[index].1
        };
        vertices.push(vertex);
    }
    Ok(Some((ordered, vertices)))
}

fn wrap(angle: f64) -> f64 {
    let a = (angle + PI).rem_euclid(TAU) - PI;
    if a <= -PI { a + TAU } else { a }
}

/// A run's band and spine, or `None` when the geometry is not the family's.
#[allow(clippy::too_many_arguments)]
fn make_run(
    topo: &Topology,
    rail: &Rail,
    support: FaceId,
    normal: Vec3,
    offset: f64,
    family: Family,
    edges: &[EdgeId],
    from: VertexId,
    to: VertexId,
    kind: SeamBlend,
) -> Result<Option<Run>, BlendError> {
    let p_from = topo.vertex(from)?.point();
    let p_to = topo.vertex(to)?.point();
    let tol = 1e-7 * rail.radius.max(1.0);
    for p in [p_from, p_to] {
        if (rail.off_axis(p) - rail.radius).abs() > tol
            || (normal.dot(p - Point3::new(0.0, 0.0, 0.0)) - offset).abs() > tol
        {
            return refused(line!());
        }
    }
    let rho = rail.radius;
    let (profile, s_from, s_to) = match family {
        Family::Ruling => {
            // The cross-section plane: the rail's (x, y) at height zero.
            let n2 = [normal.dot(rail.x), normal.dot(rail.y)];
            let d2 = offset - normal.dot(rail.origin - Point3::new(0.0, 0.0, 0.0));
            let m2 = [-n2[1], n2[0]];
            let edge_angle = rail.angle(p_from);
            if (wrap(rail.angle(p_to) - edge_angle)).abs() > 1e-9 {
                return refused(line!());
            }
            let e2 = [rho * edge_angle.cos(), rho * edge_angle.sin()];
            let along_edge = e2[0] * m2[0] + e2[1] * m2[1];
            let side = if along_edge >= 0.0 { 1.0 } else { -1.0 };
            let profile = match kind {
                SeamBlend::Fillet(r) => {
                    let reach = (rho + r).powi(2) - (d2 + r).powi(2);
                    if reach <= 0.0 {
                        return refused(line!());
                    }
                    let lambda = side * reach.sqrt();
                    let center = [
                        (d2 + r).mul_add(n2[0], lambda * m2[0]),
                        (d2 + r).mul_add(n2[1], lambda * m2[1]),
                    ];
                    let c_len = center[0].hypot(center[1]);
                    let from = [-center[0] / c_len, -center[1] / c_len];
                    let to = [-n2[0], -n2[1]];
                    let cos = from[0].mul_add(to[0], from[1] * to[1]).clamp(-1.0, 1.0);
                    let span = cos.acos();
                    let toward = {
                        let v = [cos.mul_add(-from[0], to[0]), cos.mul_add(-from[1], to[1])];
                        let len = v[0].hypot(v[1]);
                        if len < 1e-12 {
                            return refused(line!());
                        }
                        [v[0] / len, v[1] / len]
                    };
                    Profile::Arc {
                        center,
                        from,
                        toward,
                        span,
                        radius: r,
                    }
                }
                SeamBlend::Chamfer(d) => {
                    // Along the support, away from the boss.
                    let along = if e2[0].mul_add(m2[0], e2[1] * m2[1]) >= 0.0 {
                        m2
                    } else {
                        [-m2[0], -m2[1]]
                    };
                    let to = [d.mul_add(along[0], e2[0]), d.mul_add(along[1], e2[1])];
                    // Round the boss toward the support's outward side.
                    let tangent = [-edge_angle.sin(), edge_angle.cos()];
                    let turn = if tangent[0].mul_add(n2[0], tangent[1] * n2[1]) >= 0.0 {
                        1.0
                    } else {
                        -1.0
                    };
                    let a = edge_angle + turn * d / rho;
                    Profile::Segment {
                        from: [rho * a.cos(), rho * a.sin()],
                        to,
                        corner: e2,
                    }
                }
            };
            (profile, rail.height(p_from), rail.height(p_to))
        }
        Family::Cap => {
            let sigma: f64 = if normal.dot(rail.axis) > 0.0 {
                1.0
            } else {
                -1.0
            };
            let h_t = rail.height(p_from);
            if (rail.height(p_to) - h_t).abs() > tol {
                return refused(line!());
            }
            let profile = match kind {
                SeamBlend::Fillet(r) => Profile::Arc {
                    center: [rho + r, sigma.mul_add(r, h_t)],
                    from: [-1.0, 0.0],
                    toward: [0.0, -sigma],
                    span: PI / 2.0,
                    radius: r,
                },
                SeamBlend::Chamfer(d) => Profile::Segment {
                    from: [rho, sigma.mul_add(d, h_t)],
                    to: [rho + d, h_t],
                    corner: [rho, h_t],
                },
            };
            // Unwrap the angle along the chain's own arcs.
            let mut theta = rail.angle(p_from);
            let s_from = theta;
            let mut at = from;
            for &edge in edges {
                let e = topo.edge(edge)?;
                let next = if e.start() == at { e.end() } else { e.start() };
                let mid = edge_midpoint(topo, edge)?;
                let p_next = topo.vertex(next)?.point();
                let theta_mid = theta + wrap(rail.angle(mid) - theta);
                theta = theta_mid + wrap(rail.angle(p_next) - theta_mid);
                at = next;
            }
            (profile, s_from, theta)
        }
    };
    Ok(Some(Run {
        support,
        normal,
        offset,
        family,
        profile,
        from,
        to,
        s_from,
        s_to,
    }))
}

fn edge_midpoint(topo: &Topology, edge: EdgeId) -> Result<Point3, BlendError> {
    let e = topo.edge(edge)?;
    let a = topo.vertex(e.start())?.point();
    let b = topo.vertex(e.end())?.point();
    if matches!(e.curve(), EdgeCurve::Line) {
        return Ok(a + (b - a) * 0.5);
    }
    let (t0, t1) = e.strict_domain().map_err(crate::edge_domain_input)?;
    Ok(e.curve().evaluate_with_endpoints(0.5 * (t0 + t1), a, b))
}

/// The edge two faces share at `vertex`.
fn shared_edge_at(
    topo: &Topology,
    a: FaceId,
    b: FaceId,
    vertex: VertexId,
) -> Result<Option<EdgeId>, BlendError> {
    let edges_of = |face: FaceId| -> Result<Vec<EdgeId>, BlendError> {
        Ok(topo
            .wire(topo.face(face)?.outer_wire())?
            .edges()
            .iter()
            .map(OrientedEdge::edge)
            .collect())
    };
    let (ea, eb) = (edges_of(a)?, edges_of(b)?);
    for edge in ea {
        if !eb.contains(&edge) {
            continue;
        }
        let e = topo.edge(edge)?;
        if e.start() == vertex || e.end() == vertex {
            return Ok(Some(edge));
        }
    }
    Ok(None)
}

// --- Small Newton solvers on the bands ---------------------------------------

/// Solve the square system `f(x) = 0` by Newton's method with a
/// finite-difference Jacobian. `None` on non-convergence.
fn newton<const N: usize>(
    mut x: [f64; N],
    scale: f64,
    f: impl Fn(&[f64; N]) -> [f64; N],
) -> Option<[f64; N]> {
    for _ in 0..60 {
        let fx = f(&x);
        let norm = fx.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !norm.is_finite() {
            return None;
        }
        if norm < 1e-13 * scale {
            return Some(x);
        }
        let mut jac = [[0.0; N]; N];
        for j in 0..N {
            let h = 1e-7 * x[j].abs().max(1.0);
            let mut xp = x;
            xp[j] += h;
            let mut xm = x;
            xm[j] -= h;
            let (fp, fm) = (f(&xp), f(&xm));
            for i in 0..N {
                jac[i][j] = (fp[i] - fm[i]) / (2.0 * h);
            }
        }
        let step = solve_linear(jac, fx)?;
        // Damped: never accept a step that makes the residual worse.
        let mut lambda = 1.0;
        loop {
            let mut trial = x;
            for j in 0..N {
                trial[j] -= lambda * step[j];
            }
            let tn = f(&trial).iter().map(|v| v * v).sum::<f64>().sqrt();
            if tn.is_finite() && (tn < norm || lambda < 1e-6) {
                x = trial;
                break;
            }
            lambda *= 0.5;
        }
    }
    let fx = f(&x);
    let norm = fx.iter().map(|v| v * v).sum::<f64>().sqrt();
    (norm < 1e-10 * scale).then_some(x)
}

/// Gaussian elimination with partial pivoting.
fn solve_linear<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for col in 0..N {
        let pivot = (col..N).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-300 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..N {
            let factor = a[row][col] / a[col][col];
            for k in col..N {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = [0.0; N];
    for row in (0..N).rev() {
        let mut sum = b[row];
        for k in row + 1..N {
            sum -= a[row][k] * x[k];
        }
        x[row] = sum / a[row][row];
    }
    Some(x)
}

fn diff(a: Point3, b: Point3) -> [f64; 3] {
    let d = a - b;
    [d.x(), d.y(), d.z()]
}

/// Where two runs' rail contacts meet: their spine parameters.
fn rail_meet(rail: &Rail, a: &Run, b: &Run, scale: f64) -> Option<(f64, f64)> {
    // Both contacts lie on the rail: meet in its (angle, height) chart.
    let chart = |p: Point3| [rail.angle(p), rail.height(p)];
    let guess = [a.s_to, b.s_from];
    let x = newton::<2>(guess, scale, |x| {
        let (pa, pb) = (
            chart(a.eval(rail, x[0], 0.0)),
            chart(b.eval(rail, x[1], 0.0)),
        );
        [wrap(pa[0] - pb[0]) * rail.radius, pa[1] - pb[1]]
    })?;
    Some((x[0], x[1]))
}

/// Spine parameter where `run`'s contact at `t` meets the plane `normal · p =
/// offset`, from `guess`.
fn plane_meet(
    rail: &Rail,
    run: &Run,
    t: f64,
    normal: Vec3,
    offset: f64,
    guess: f64,
    scale: f64,
) -> Option<f64> {
    newton::<1>([guess], scale, |x| {
        [normal.dot(run.eval(rail, x[0], t) - Point3::new(0.0, 0.0, 0.0)) - offset]
    })
    .map(|x| x[0])
}

/// The curve of `run`'s band in the plane `normal · p = offset`, from `t0` to
/// `t1`, starting from spine parameter `guess`: exact when the band's section
/// is that plane, fitted otherwise. Also returns the spine parameter at `t0`
/// and `t1`.
#[allow(clippy::too_many_arguments)]
fn band_in_plane(
    rail: &Rail,
    run: &Run,
    normal: Vec3,
    offset: f64,
    t0: f64,
    t1: f64,
    guess: f64,
    scale: f64,
) -> Result<Option<(SideCurve, f64, f64)>, BlendError> {
    let mut s = guess;
    let mut points = Vec::with_capacity(SAMPLES + 1);
    let mut params = Vec::with_capacity(SAMPLES + 1);
    for i in 0..=SAMPLES {
        #[allow(clippy::cast_precision_loss)]
        let t = (t1 - t0).mul_add(i as f64 / SAMPLES as f64, t0);
        let Some(next) = plane_meet(rail, run, t, normal, offset, s, scale) else {
            return refused(line!());
        };
        s = next;
        params.push(s);
        points.push(run.eval(rail, s, t));
    }
    let (s0, s1) = (params[0], params[SAMPLES]);
    let constant = params
        .iter()
        .all(|&p| (p - s0).abs() <= 1e-10 * s0.abs().max(1.0));
    let curve = if constant {
        match run.section(rail, s0)? {
            Some(circle) => SideCurve::Arc(circle),
            None => SideCurve::Line,
        }
    } else {
        SideCurve::Fitted(points)
    };
    Ok(Some((curve, s0, s1)))
}

/// The corner between two consecutive runs.
fn plan_mitre(rail: &Rail, a: &Run, b: &Run, scale: f64) -> Result<Option<Mitre>, BlendError> {
    let Some((sa, sb)) = rail_meet(rail, a, b, scale) else {
        return refused(line!());
    };
    let p = a.eval(rail, sa, 0.0);
    // March the intersection of the two bands from the rail, by the earlier
    // band's section parameter, until either band's support contact.
    let mitre_eq = |x: &[f64; 3], ta: f64| diff(a.eval(rail, x[0], ta), b.eval(rail, x[1], x[2]));
    let mut points = vec![p];
    // Each point's earlier-band section parameter and solved state.
    let mut samples: Vec<(f64, [f64; 3])> = vec![(0.0, [sa, sb, 0.0])];
    let mut state = [sa, sb, 0.0];
    let mut previous: Option<[f64; 3]> = None;
    let mut ta_prev = 0.0;
    let mut m_on_earlier = true;
    let mut end_state = None;
    for i in 1..=SAMPLES {
        #[allow(clippy::cast_precision_loss)]
        let ta = i as f64 / SAMPLES as f64;
        // Predict along the curve: at the rail both bands are tangent to it,
        // so start the later band's section level with the earlier one's.
        let guess = previous.map_or([state[0], state[1], ta], |before| {
            [
                2.0f64.mul_add(state[0], -before[0]),
                2.0f64.mul_add(state[1], -before[1]),
                2.0f64.mul_add(state[2], -before[2]),
            ]
        });
        let Some(next) = newton::<3>(guess, scale, |x| mitre_eq(x, ta)) else {
            return refused(line!());
        };
        if next[2] < -1e-9 {
            return refused(line!());
        }
        if next[2] > 1.0 {
            // The later band reached its support first: find where, with its
            // section parameter held at the support.
            let Some(x) = newton::<3>([state[0], state[1], ta_prev], scale, |x| {
                diff(a.eval(rail, x[0], x[2]), b.eval(rail, x[1], 1.0))
            }) else {
                return refused(line!());
            };
            if !(ta_prev - 1e-9..=ta + 1e-9).contains(&x[2]) {
                return refused(line!());
            }
            points.push(b.eval(rail, x[1], 1.0));
            samples.push((x[2], [x[0], x[1], 1.0]));
            m_on_earlier = false;
            end_state = Some((x[0], x[1], x[2], 1.0));
            break;
        }
        points.push(a.eval(rail, next[0], ta));
        samples.push((ta, next));
        previous = Some(state);
        state = next;
        ta_prev = ta;
        if i == SAMPLES {
            end_state = Some((next[0], next[1], 1.0, next[2]));
        }
    }
    let Some((sa_m, sb_m, ta_m, tb_m)) = end_state else {
        return refused(line!());
    };
    let m = *points.last().ok_or_else(|| invalid("empty mitre"))?;
    // Refine until the fitted mitre stays on both bands between samples.
    let Some(refined) = refine_mitre(rail, a, b, samples, scale)? else {
        return refused(line!());
    };
    points = refined;
    // The other band's end section in the plane of the support it stops on,
    // from `m` to its support contact `k`.
    let (end, k, support_s) = if m_on_earlier {
        let Some((curve, s_m, s_k)) =
            band_in_plane(rail, b, a.normal, a.offset, tb_m, 1.0, sb_m, scale)?
        else {
            return refused(line!());
        };
        if (s_m - sb_m).abs() > 1e-7 * scale {
            return refused(line!());
        }
        (curve, b.eval(rail, s_k, 1.0), [sa_m, s_k])
    } else {
        let Some((curve, s_m, s_k)) =
            band_in_plane(rail, a, b.normal, b.offset, ta_m, 1.0, sa_m, scale)?
        else {
            return refused(line!());
        };
        if (s_m - sa_m).abs() > 1e-7 * scale {
            return refused(line!());
        }
        (curve, a.eval(rail, s_k, 1.0), [s_k, sb_m])
    };
    log::debug!(
        "seam mitre candidate: p {p:?} m {m:?} k {k:?} m_on_earlier {m_on_earlier} points {}",
        points.len()
    );
    if (m - p).length() < 1e-9 * scale {
        return refused(line!());
    }
    let (end, k) = if (k - m).length() <= 1e-7 * scale {
        (None, m)
    } else {
        (Some(end), k)
    };
    log::debug!(
        "seam mitre: p {p:?} m {m:?} k {k:?} m_on_earlier {m_on_earlier} rail_s {sa} {sb} support_s {support_s:?}"
    );
    Ok(Some(Mitre {
        p,
        m,
        k,
        curve: points,
        m_on_earlier,
        end,
        rail_s: [sa, sb],
        support_s,
    }))
}

/// How far `q` lies off `surface`; `None` when it does not project.
fn off_surface(surface: &FaceSurface, q: Point3) -> Option<f64> {
    match surface {
        FaceSurface::Plane { normal, d } => {
            Some((normal.dot(q - Point3::new(0.0, 0.0, 0.0)) - d).abs())
        }
        other => {
            let (u, v) = other.project_point(q)?;
            Some((other.evaluate(u, v)? - q).length())
        }
    }
}

/// The mitre's points, refined until the curve [`side_edge`] fits through
/// them stays on both bands midway between every two (where a fit strays
/// furthest): a span that strays gets the exact intersection point at its
/// middle section parameter. `None` when it does not settle.
fn refine_mitre(
    rail: &Rail,
    a: &Run,
    b: &Run,
    mut samples: Vec<(f64, [f64; 3])>,
    scale: f64,
) -> Result<Option<Vec<Point3>>, BlendError> {
    let surfaces = [a.surface(rail)?.0, b.surface(rail)?.0];
    let point = |&(ta, state): &(f64, [f64; 3])| a.eval(rail, state[0], ta);
    for _ in 0..8 {
        let points: Vec<Point3> = samples.iter().map(point).collect();
        let curve = remus_math::nurbs::fitting::interpolate(&points, 3)?;
        let (t0, t1) = curve.domain();
        let mut chord = vec![0.0];
        for pair in points.windows(2) {
            chord.push(chord[chord.len() - 1] + (pair[1] - pair[0]).length());
        }
        let total = chord[chord.len() - 1];
        let mut straying = Vec::new();
        for i in 0..points.len() - 1 {
            let u = 0.5 * (chord[i] + chord[i + 1]) / total;
            let q = curve.evaluate((t1 - t0).mul_add(u, t0));
            let mut worst = 0.0f64;
            for surface in &surfaces {
                let Some(off) = off_surface(surface, q) else {
                    return Ok(None);
                };
                worst = worst.max(off);
            }
            if worst > 1e-7 * scale {
                straying.push(i);
            }
        }
        if straying.is_empty() {
            return Ok(Some(points));
        }
        if samples.len() + straying.len() > 4096 {
            return Ok(None);
        }
        for &i in straying.iter().rev() {
            let (ta0, s0) = samples[i];
            let (ta1, s1) = samples[i + 1];
            let ta = 0.5 * (ta0 + ta1);
            let guess = [
                0.5 * (s0[0] + s1[0]),
                0.5 * (s0[1] + s1[1]),
                0.5 * (s0[2] + s1[2]),
            ];
            let Some(state) = newton::<3>(guess, scale, |x| {
                diff(a.eval(rail, x[0], ta), b.eval(rail, x[1], x[2]))
            }) else {
                return Ok(None);
            };
            samples.insert(i + 1, (ta, state));
        }
    }
    Ok(None)
}

/// Where the chain stops at `vertex`, the end of `run`.
#[allow(clippy::too_many_arguments)]
fn plan_end(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    rail: &Rail,
    run: &Run,
    vertex: VertexId,
    chain: &[EdgeId],
    scale: f64,
) -> Result<Option<End>, BlendError> {
    // The face the chain runs into: it shares an edge at `vertex` with the
    // support and one with the rail.
    let Some(face) = across(topo, adjacency, run.support, vertex, chain, rail.face)? else {
        return refused(line!());
    };
    let end_face = topo.face(face)?;
    let FaceSurface::Plane { normal, d } = end_face.surface() else {
        return refused(line!());
    };
    let (normal, offset) = (*normal, *d);
    let reversed = end_face.is_reversed();
    // Round the vertex through coplanar faces to the one beside the rail.
    let mut rail_side = face;
    let mut previous = run.support;
    for _ in 0..8 {
        if let Some(with_rail) = shared_edge_at(topo, rail_side, rail.face, vertex)? {
            if chain.contains(&with_rail) {
                return refused(line!());
            }
            break;
        }
        let mut next = None;
        for oriented in topo.wire(topo.face(rail_side)?.outer_wire())?.edges() {
            let e = topo.edge(oriented.edge())?;
            if e.start() != vertex && e.end() != vertex {
                continue;
            }
            for &other in adjacency.faces_for_edge(oriented.edge()) {
                if other == rail_side || other == previous || other == run.support {
                    continue;
                }
                let f = topo.face(other)?;
                let coplanar = matches!(f.surface(), FaceSurface::Plane { normal: n, d: dd }
                    if (n.dot(normal) - 1.0).abs() < 1e-12
                        && (dd - offset).abs() < 1e-9 * scale)
                    && f.is_reversed() == reversed;
                if !coplanar || next.is_some_and(|n| n != other) {
                    return refused(line!());
                }
                next = Some(other);
            }
        }
        let Some(next) = next else {
            return refused(line!());
        };
        previous = rail_side;
        rail_side = next;
    }
    if shared_edge_at(topo, rail_side, rail.face, vertex)?.is_none() {
        return refused(line!());
    }
    let along = normal.dot(rail.axis).abs();
    // The band must stop square to its spine: a ruling run against a plane
    // across the axis, a run round the axis against a plane along it.
    match run.family {
        Family::Ruling if along > 1.0 - 1e-12 => {}
        Family::Cap if along < 1e-9 => {}
        Family::Ruling | Family::Cap => return Ok(None),
    }
    let s_end = if vertex == run.from {
        run.s_from
    } else {
        run.s_to
    };
    let Some((section, s_x, s_k)) =
        band_in_plane(rail, run, normal, offset, 0.0, 1.0, s_end, scale)?
    else {
        return refused(line!());
    };
    let x = run.eval(rail, s_x, 0.0);
    let k = run.eval(rail, s_k, 1.0);
    let v = topo.vertex(vertex)?.point();
    // The rail in the end plane, from `x` back to the vertex: a ruling of the
    // boss when the plane runs along it, a circle round it otherwise.
    let closure = match run.family {
        Family::Cap => {
            if wrap(rail.angle(x) - rail.angle(v)).abs() > 1e-9 {
                return refused(line!());
            }
            SideCurve::Line
        }
        Family::Ruling => {
            if (rail.height(x) - rail.height(v)).abs() > 1e-9 * scale {
                return refused(line!());
            }
            SideCurve::Arc(rail.circle(rail.height(v), rail.radius)?)
        }
    };
    Ok(Some(End {
        vertex,
        face,
        rail_side,
        x,
        k,
        section,
        rail_s: s_x,
        support_s: s_k,
        closure,
    }))
}

/// The face across the support's edge at `vertex` that is not part of the
/// chain: the face the chain runs into.
fn across(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    support: FaceId,
    vertex: VertexId,
    chain: &[EdgeId],
    rail: FaceId,
) -> Result<Option<FaceId>, BlendError> {
    let mut found = None;
    for oriented in topo.wire(topo.face(support)?.outer_wire())?.edges() {
        let edge = oriented.edge();
        if chain.contains(&edge) {
            continue;
        }
        let e = topo.edge(edge)?;
        if e.start() != vertex && e.end() != vertex {
            continue;
        }
        let faces = adjacency.faces_for_edge(edge);
        let Some(&other) = faces.iter().find(|&&f| f != support) else {
            continue;
        };
        if other == rail {
            continue;
        }
        if found.is_some() {
            return refused(line!());
        }
        found = Some(other);
    }
    Ok(found)
}

// ===========================================================================
// Building
// ===========================================================================

/// Add an edge along `curve` between two vertices; returns it oriented from
/// `from` to `to`.
fn side_edge(
    topo: &mut Topology,
    from: VertexId,
    to: VertexId,
    curve: &SideCurve,
) -> Result<OrientedEdge, BlendError> {
    let a = topo.vertex(from)?.point();
    let b = topo.vertex(to)?.point();
    match curve {
        SideCurve::Line => Ok(OrientedEdge::new(
            topo.add_edge(Edge::new(from, to, EdgeCurve::Line)),
            true,
        )),
        SideCurve::Arc(circle) => {
            let ta = circle.project(a);
            let delta = wrap(circle.project(b) - ta);
            arc_edge(topo, from, to, circle, ta, ta + delta)
        }
        SideCurve::Fitted(points) => {
            let mut points = points.clone();
            // Fitted from its own first point; the vertices are authoritative.
            let reversed = (points[0] - a).length() > (points[points.len() - 1] - a).length();
            if reversed {
                points.reverse();
            }
            let n = points.len();
            points[0] = a;
            points[n - 1] = b;
            let curve: NurbsCurve = remus_math::nurbs::fitting::interpolate(&points, 3)?;
            let domain = curve.domain();
            Ok(OrientedEdge::new(
                add_certified_curve_edge(topo, from, to, EdgeCurve::NurbsCurve(curve), domain)?,
                true,
            ))
        }
    }
}

/// An arc of `circle` from `from` (at parameter `t_from`) to `to` (at
/// `t_to`), stored over an increasing interval.
fn arc_edge(
    topo: &mut Topology,
    from: VertexId,
    to: VertexId,
    circle: &Circle3D,
    t_from: f64,
    t_to: f64,
) -> Result<OrientedEdge, BlendError> {
    let (lo, hi, forward) = if t_to > t_from {
        (t_from, t_to, true)
    } else {
        (t_to, t_from, false)
    };
    let shift = (lo.div_euclid(TAU)) * TAU;
    let (lo, hi) = (lo - shift, hi - shift);
    let (start, end) = if forward { (from, to) } else { (to, from) };
    let edge = add_certified_curve_edge(
        topo,
        start,
        end,
        EdgeCurve::Circle(circle.clone()),
        (lo, hi),
    )?;
    Ok(OrientedEdge::new(edge, forward))
}

fn reversed(oe: OrientedEdge) -> OrientedEdge {
    OrientedEdge::new(oe.edge(), !oe.is_forward())
}

/// The rail face's seam rulings: line edges its boundary uses twice, as
/// `(angle, low height, high height)`.
fn seam_rulings(
    topo: &Topology,
    rail: &Rail,
    face: FaceId,
) -> Result<Vec<(f64, f64, f64)>, BlendError> {
    let uses = topo.wire(topo.face(face)?.outer_wire())?.edges().to_vec();
    let mut seams = Vec::new();
    for (index, oriented) in uses.iter().enumerate() {
        if uses[..index]
            .iter()
            .any(|seen| seen.edge() == oriented.edge())
            || uses.iter().filter(|o| o.edge() == oriented.edge()).count() != 2
        {
            continue;
        }
        let edge = topo.edge(oriented.edge())?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            continue;
        }
        let a = topo.vertex(edge.start())?.point();
        let b = topo.vertex(edge.end())?.point();
        let (ha, hb) = (rail.height(a), rail.height(b));
        seams.push((rail.angle(a), ha.min(hb), ha.max(hb)));
    }
    Ok(seams)
}

/// The vertices one end of the chain gets in the build.
struct EndVertices {
    k: VertexId,
    x: VertexId,
    /// The rail's closure back to the chain vertex, oriented `x` to the
    /// vertex, when `x` is not on the end face's own rail edge.
    closure: Option<OrientedEdge>,
    /// Whether the rail itself carries the closure (it was made here).
    rail_closure: bool,
}

/// Build the planned seam.
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
fn build(
    topo: &mut Topology,
    plan: &Plan,
    scope: &mut EditScope,
    original_faces: &[FaceId],
    inner_shells: &[ShellId],
    requested: &[EdgeId],
) -> Result<BlendResult, BlendError> {
    let rail = &plan.rail;
    let runs = &plan.runs;
    let n = runs.len();
    let mut replacements = remus_math::det_hash::DetHashMap::default();
    let current = |replacements: &remus_math::det_hash::DetHashMap<FaceId, FaceId>,
                   face: FaceId| {
        replacements.get(&face).copied().unwrap_or(face)
    };
    let touch = 1e-6 * rail.radius.max(1.0);
    let failure = |face: FaceId| BlendError::TrimmingFailure { face };

    // --- 1. Every boundary vertex, split into the body before any cut. ---
    let mut split = |topo: &mut Topology,
                     replacements: &mut remus_math::det_hash::DetHashMap<FaceId, FaceId>,
                     face: FaceId,
                     point: Point3|
     -> Result<Option<VertexId>, BlendError> {
        let at = current(replacements, face);
        let mut edit = scope.edit(replacements);
        trimmer::split_body_boundary_at(topo, &mut edit, at, point, touch)
    };
    // Mitre corners: `k` on the sharp edge.
    let mut corner_k = Vec::with_capacity(plan.mitres.len());
    for (index, mitre) in plan.mitres.iter().enumerate() {
        let k = split(topo, &mut replacements, runs[index].support, mitre.k)?
            .ok_or_else(|| failure(runs[index].support))?;
        corner_k.push(k);
    }
    // Ends: `k` on the edge with the support; `x` on the edge with the rail,
    // or on another rail edge (a seam), or inside the rail.
    let mut end_vertices: Vec<EndVertices> = Vec::with_capacity(2);
    for end in &plan.ends {
        let k =
            split(topo, &mut replacements, end.face, end.k)?.ok_or_else(|| failure(end.face))?;
        // On the end face's own rail edge: the face is notched.
        let on_face = split(topo, &mut replacements, end.rail_side, end.x)?;
        if end.rail_side != end.face && on_face.is_none() {
            return Err(failure(end.rail_side));
        }
        let (x, closure, rail_closure) = if let Some(x) = on_face {
            if x == end.vertex {
                return Err(failure(end.face));
            }
            (x, None, false)
        } else if let Some(x) = split(topo, &mut replacements, rail.face, end.x)? {
            // On another rail boundary edge through the vertex: a seam ruling
            // the boss's parameter seam put there. Its piece below `x` is the
            // closure.
            let wire = topo
                .wire(topo.face(current(&replacements, rail.face))?.outer_wire())?
                .edges()
                .to_vec();
            let mut found = None;
            for oe in &wire {
                let e = topo.edge(oe.edge())?;
                if (e.start() == x && e.end() == end.vertex)
                    || (e.start() == end.vertex && e.end() == x)
                {
                    found = Some(OrientedEdge::new(oe.edge(), e.start() == x));
                }
            }
            let closure = found.ok_or_else(|| failure(rail.face))?;
            (x, Some(closure), false)
        } else {
            let x = topo.add_vertex(Vertex::new(end.x, VERTEX_TOL));
            let closure = side_edge(topo, x, end.vertex, &end.closure)?;
            (x, Some(closure), true)
        };
        end_vertices.push(EndVertices {
            k,
            x,
            closure,
            rail_closure,
        });
    }

    // Seam crossings of the rail contacts round the axis.
    let seams = seam_rulings(topo, rail, current(&replacements, rail.face))?;
    // Per run, the rail-contact spine range, in chain order.
    let rail_range = |i: usize| -> (f64, f64) {
        let s0 = if i == 0 {
            plan.ends[0].rail_s
        } else {
            plan.mitres[i - 1].rail_s[1]
        };
        let s1 = if i == n - 1 {
            plan.ends[1].rail_s
        } else {
            plan.mitres[i].rail_s[0]
        };
        (s0, s1)
    };
    let mut crossings: Vec<Vec<(f64, VertexId)>> = vec![Vec::new(); n];
    for (i, run) in runs.iter().enumerate() {
        if !matches!(run.family, Family::Cap) {
            continue;
        }
        let (s0, s1) = rail_range(i);
        let h = run.profile.at(0.0)[1];
        let (lo, hi) = (s0.min(s1), s0.max(s1));
        for &(angle, h_lo, h_hi) in &seams {
            if h <= h_lo + touch || h >= h_hi - touch {
                continue;
            }
            // Every turn of the seam's angle strictly inside the range.
            let first = angle + ((lo - angle) / TAU).ceil() * TAU;
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let turns = ((hi - first) / TAU).ceil().max(0.0) as usize;
            for turn in 0..turns {
                #[allow(clippy::cast_precision_loss)]
                let a = (turn as f64).mul_add(TAU, first);
                if a > lo + 1e-9 && a < hi - 1e-9 {
                    let point = run.eval(rail, a, 0.0);
                    let vertex = split(topo, &mut replacements, rail.face, point)?
                        .ok_or_else(|| failure(rail.face))?;
                    crossings[i].push((a, vertex));
                }
            }
        }
        crossings[i].compact_sort_unstable_by(|x, y| {
            if s1 >= s0 {
                x.0.total_cmp(&y.0)
            } else {
                y.0.total_cmp(&x.0)
            }
        });
    }
    // Ruling runs must not lie along a seam.
    for run in runs {
        if matches!(run.family, Family::Ruling) {
            let angle = rail.angle(run.eval(rail, run.s_from, 0.0));
            if seams.iter().any(|&(a, ..)| wrap(a - angle).abs() < 1e-9) {
                return Err(failure(rail.face));
            }
        }
    }

    // --- 2. The new vertices inside faces. ---
    let mitre_p: Vec<VertexId> = plan
        .mitres
        .iter()
        .map(|m| topo.add_vertex(Vertex::new(m.p, VERTEX_TOL)))
        .collect();
    let mitre_m: Vec<VertexId> = plan
        .mitres
        .iter()
        .zip(&corner_k)
        .map(|(m, &k)| {
            if m.end.is_some() {
                topo.add_vertex(Vertex::new(m.m, VERTEX_TOL))
            } else {
                k
            }
        })
        .collect();

    // --- 3. Edges. ---
    // Rail contacts, per run, as a list of edges from its first rail vertex
    // to its last, through the seam crossings.
    let mut rail_paths: Vec<Vec<OrientedEdge>> = Vec::with_capacity(n);
    for (i, run) in runs.iter().enumerate() {
        let (s0, s1) = rail_range(i);
        let v0 = if i == 0 {
            end_vertices[0].x
        } else {
            mitre_p[i - 1]
        };
        let v1 = if i == n - 1 {
            end_vertices[1].x
        } else {
            mitre_p[i]
        };
        let mut stops: Vec<(f64, VertexId)> = vec![(s0, v0)];
        stops.extend(crossings[i].iter().copied());
        stops.push((s1, v1));
        let mut edges = Vec::with_capacity(stops.len() - 1);
        for pair in stops.windows(2) {
            let ((sa, va), (sb, vb)) = (pair[0], pair[1]);
            let oe = match run.contact(rail, 0.0)? {
                None => OrientedEdge::new(topo.add_edge(Edge::new(va, vb, EdgeCurve::Line)), true),
                Some(circle) => arc_edge(topo, va, vb, &circle, sa, sb)?,
            };
            edges.push(oe);
        }
        rail_paths.push(edges);
    }
    // Support contacts, per run, from its first support vertex to its last.
    let mut support_edges: Vec<OrientedEdge> = Vec::with_capacity(n);
    let support_vertex_from = |i: usize| -> VertexId {
        if i == 0 {
            end_vertices[0].k
        } else if plan.mitres[i - 1].m_on_earlier {
            corner_k[i - 1]
        } else {
            mitre_m[i - 1]
        }
    };
    let support_vertex_to = |i: usize| -> VertexId {
        if i == n - 1 {
            end_vertices[1].k
        } else if plan.mitres[i].m_on_earlier {
            mitre_m[i]
        } else {
            corner_k[i]
        }
    };
    for (i, run) in runs.iter().enumerate() {
        let s0 = if i == 0 {
            plan.ends[0].support_s
        } else {
            plan.mitres[i - 1].support_s[1]
        };
        let s1 = if i == n - 1 {
            plan.ends[1].support_s
        } else {
            plan.mitres[i].support_s[0]
        };
        let (va, vb) = (support_vertex_from(i), support_vertex_to(i));
        let oe = match run.contact(rail, 1.0)? {
            None => OrientedEdge::new(topo.add_edge(Edge::new(va, vb, EdgeCurve::Line)), true),
            Some(circle) => arc_edge(topo, va, vb, &circle, s0, s1)?,
        };
        support_edges.push(oe);
    }
    // Mitres (p to m) and their end sections (m to k).
    let mut mitre_edges = Vec::with_capacity(plan.mitres.len());
    let mut mitre_ends = Vec::with_capacity(plan.mitres.len());
    for (index, mitre) in plan.mitres.iter().enumerate() {
        let curve = SideCurve::Fitted(mitre.curve.clone());
        mitre_edges.push(side_edge(topo, mitre_p[index], mitre_m[index], &curve)?);
        mitre_ends.push(match &mitre.end {
            Some(end) => Some(side_edge(topo, mitre_m[index], corner_k[index], end)?),
            None => None,
        });
    }
    // End sections (x to k).
    let mut end_sections = Vec::with_capacity(2);
    for (end, vertices) in plan.ends.iter().zip(&end_vertices) {
        end_sections.push(side_edge(topo, vertices.x, vertices.k, &end.section)?);
    }

    // --- 4. Cut the faces. ---
    // A piece over a face's current boundary from vertex `from` (on the side
    // of it `from_toward` lies) to `to`.
    let cut = |topo: &mut Topology,
               replacements: &mut remus_math::det_hash::DetHashMap<FaceId, FaceId>,
               face: FaceId,
               pieces: Vec<(VertexId, Point3, VertexId, Point3, Vec<OrientedEdge>)>,
               avoid: &[EdgeId]|
     -> Result<(), BlendError> {
        let at = current(replacements, face);
        let wire = topo.wire(topo.face(at)?.outer_wire())?.edges().to_vec();
        let mut cut_pieces = Vec::with_capacity(pieces.len());
        for (from, from_toward, to, to_toward, edges) in pieces {
            let (Some(from), Some(to)) = (
                trimmer::vertex_occurrence(topo, at, &wire, from, from_toward)?,
                trimmer::vertex_occurrence(topo, at, &wire, to, to_toward)?,
            ) else {
                return Err(failure(face));
            };
            cut_pieces.push(CutPiece { from, to, edges });
        }
        let trimmed = trimmer::cut_face(topo, at, &wire, &cut_pieces, avoid).map_err(|e| {
            log::warn!("seam blend: cutting {face:?} failed: {e}");
            failure(face)
        })?;
        replacements.insert(face, trimmed);
        Ok(())
    };
    // A point a little way along an edge use from its start.
    let ahead = |topo: &Topology, oe: OrientedEdge| -> Result<Point3, BlendError> {
        let e = topo.edge(oe.edge())?;
        let a = topo.vertex(oe.oriented_start(e))?.point();
        let mid = edge_midpoint(topo, oe.edge())?;
        Ok(a + (mid - a) * 0.25)
    };
    let behind = |topo: &Topology, oe: OrientedEdge| -> Result<Point3, BlendError> {
        let e = topo.edge(oe.edge())?;
        let b = topo.vertex(oe.oriented_end(e))?.point();
        let mid = edge_midpoint(topo, oe.edge())?;
        Ok(b + (mid - b) * 0.25)
    };
    let chain = &plan.chain;

    // Supports.
    for (i, run) in runs.iter().enumerate() {
        let mut edges = Vec::new();
        let start = if i > 0
            && !plan.mitres[i - 1].m_on_earlier
            && let Some(end) = mitre_ends[i - 1]
        {
            // The earlier band's end section lies in this support.
            edges.push(reversed(end));
            corner_k[i - 1]
        } else {
            support_vertex_from(i)
        };
        edges.push(support_edges[i]);
        let finish = if i < n - 1
            && plan.mitres[i].m_on_earlier
            && let Some(end) = mitre_ends[i]
        {
            edges.push(end);
            corner_k[i]
        } else {
            support_vertex_to(i)
        };
        let (first, last) = (edges[0], edges[edges.len() - 1]);
        let piece = (
            start,
            ahead(topo, first)?,
            finish,
            behind(topo, last)?,
            edges,
        );
        cut(topo, &mut replacements, run.support, vec![piece], chain)?;
    }
    // End faces.
    let mut lips: Vec<(FaceId, Vec<FaceId>)> = Vec::new();
    for (j, (end, vertices)) in plan.ends.iter().zip(&end_vertices).enumerate() {
        let face = end.face;
        if end.rail_side != face {
            // A fan of coplanar faces round the vertex: the band's end cap is
            // a face of its own, between the section and the two edges the
            // support and the rail give up at the vertex.
            let piece_of = |topo: &Topology,
                            replacements: &remus_math::det_hash::DetHashMap<FaceId, FaceId>,
                            face: FaceId,
                            other: VertexId|
             -> Result<(OrientedEdge, bool), BlendError> {
                let at = current(replacements, face);
                let f = topo.face(at)?;
                for oe in topo.wire(f.outer_wire())?.edges() {
                    let e = topo.edge(oe.edge())?;
                    if (e.start() == end.vertex && e.end() == other)
                        || (e.start() == other && e.end() == end.vertex)
                    {
                        // Oriented from `other` to the vertex, with the
                        // face's effective direction of travel.
                        return Ok((
                            OrientedEdge::new(oe.edge(), e.end() == end.vertex),
                            oe.is_forward() != f.is_reversed(),
                        ));
                    }
                }
                Err(failure(face))
            };
            let (to_vertex, support_way) = piece_of(topo, &replacements, face, vertices.k)?;
            let (to_x, _) = piece_of(topo, &replacements, end.rail_side, vertices.x)?;
            let from_vertex = reversed(to_x);
            let mut loop_edges = vec![end_sections[j], to_vertex, from_vertex];
            let end_face = topo.face(current(&replacements, face))?;
            let (surface, face_reversed) = (end_face.surface().clone(), end_face.is_reversed());
            // Run against the face beside it.
            if (to_vertex.is_forward() != face_reversed) == support_way {
                loop_edges = loop_edges.iter().rev().map(|&oe| reversed(oe)).collect();
            }
            let wire = topo.add_wire(Wire::new(loop_edges, true)?);
            let mut lip = Face::new(wire, Vec::new(), surface);
            lip.set_reversed(face_reversed);
            lips.push((topo.add_face(lip), vec![face, end.rail_side]));
            continue;
        }
        let at = current(&replacements, face);
        let wire = topo.wire(topo.face(at)?.outer_wire())?.edges().to_vec();
        // The edges between `k` and the chain vertex, and (notched) between
        // the vertex and `x`, are dropped.
        let mut avoid: Vec<EdgeId> = Vec::new();
        for oe in &wire {
            let e = topo.edge(oe.edge())?;
            let ends = [e.start(), e.end()];
            if ends.contains(&end.vertex)
                && (ends.contains(&vertices.k) || ends.contains(&vertices.x))
            {
                avoid.push(oe.edge());
            }
        }
        let mut edges = vec![reversed(end_sections[j])];
        let to = match vertices.closure {
            Some(closure) => {
                edges.push(closure);
                end.vertex
            }
            None => vertices.x,
        };
        let (first, last) = (edges[0], edges[edges.len() - 1]);
        let piece = (
            vertices.k,
            ahead(topo, first)?,
            to,
            behind(topo, last)?,
            edges,
        );
        cut(topo, &mut replacements, face, vec![piece], &avoid)?;
    }
    // The rail: one path from the first end to the last, cut at every seam
    // crossing.
    {
        let mut path: Vec<OrientedEdge> = Vec::new();
        if let (Some(closure), true) = (end_vertices[0].closure, end_vertices[0].rail_closure) {
            path.push(reversed(closure));
        }
        for rp in &rail_paths {
            path.extend(rp.iter().copied());
        }
        if let (Some(closure), true) = (end_vertices[1].closure, end_vertices[1].rail_closure) {
            path.push(closure);
        }
        let start = if end_vertices[0].rail_closure {
            plan.ends[0].vertex
        } else {
            end_vertices[0].x
        };
        let finish = if end_vertices[1].rail_closure {
            plan.ends[1].vertex
        } else {
            end_vertices[1].x
        };
        let crossing_vertices: Vec<VertexId> = crossings.iter().flatten().map(|c| c.1).collect();
        let mut pieces = Vec::new();
        let mut piece_edges: Vec<OrientedEdge> = Vec::new();
        let mut piece_start = start;
        for oe in path {
            piece_edges.push(oe);
            let e = topo.edge(oe.edge())?;
            let end_vertex = oe.oriented_end(e);
            if crossing_vertices.contains(&end_vertex) {
                let (first, last) = (piece_edges[0], piece_edges[piece_edges.len() - 1]);
                pieces.push((
                    piece_start,
                    ahead(topo, first)?,
                    end_vertex,
                    behind(topo, last)?,
                    std::mem::take(&mut piece_edges),
                ));
                piece_start = end_vertex;
            }
        }
        let (first, last) = (piece_edges[0], piece_edges[piece_edges.len() - 1]);
        pieces.push((
            piece_start,
            ahead(topo, first)?,
            finish,
            behind(topo, last)?,
            piece_edges,
        ));
        cut(topo, &mut replacements, rail.face, pieces, chain)?;
    }

    // --- 5. The bands. ---
    let mut created: Vec<(FaceId, Vec<FaceId>)> = Vec::with_capacity(n);
    for (i, run) in runs.iter().enumerate() {
        let mut loop_edges: Vec<OrientedEdge> = rail_paths[i].clone();
        // Far end: a mitre, or the chain's end section.
        if i < n - 1 {
            loop_edges.push(mitre_edges[i]);
            if !plan.mitres[i].m_on_earlier
                && let Some(end) = mitre_ends[i]
            {
                // This band carries on past `m` to `k` in the later support.
                loop_edges.push(end);
            }
        } else {
            loop_edges.push(end_sections[1]);
        }
        loop_edges.push(reversed(support_edges[i]));
        // Near end.
        if i > 0 {
            if plan.mitres[i - 1].m_on_earlier
                && let Some(end) = mitre_ends[i - 1]
            {
                loop_edges.push(reversed(end));
            }
            loop_edges.push(reversed(mitre_edges[i - 1]));
        } else {
            loop_edges.push(reversed(end_sections[0]));
        }
        let (surface, face_reversed) = run.surface(rail)?;
        // Each edge runs against its other face's use of it.
        let rail_face = current(&replacements, rail.face);
        let along_rail = effective_use(topo, rail_face, rail_paths[i][0].edge())?
            .ok_or_else(|| failure(rail.face))?;
        let ours = rail_paths[i][0].is_forward() != face_reversed;
        if ours == along_rail {
            loop_edges = loop_edges.iter().rev().map(|&oe| reversed(oe)).collect();
        }
        let wire = topo.add_wire(Wire::new(loop_edges, true)?);
        let mut face = Face::new(wire, Vec::new(), surface);
        face.set_reversed(face_reversed);
        let band = topo.add_face(face);
        created.push((band, vec![rail.face, run.support]));
    }

    let mut result_faces: Vec<FaceId> = original_faces
        .iter()
        .map(|&face| current(&replacements, face))
        .collect();
    created.extend(lips);
    result_faces.extend(created.iter().map(|(face, _)| *face));
    let shell = topo.add_shell(Shell::new(result_faces)?);
    let result = topo.add_solid(Solid::new(shell, inner_shells.to_vec()));
    Ok(BlendResult {
        solid: result,
        succeeded: requested.to_vec(),
        failed: Vec::new(),
        is_partial: false,
        face_origins: Some(BlendFaceOrigins {
            survived: original_faces
                .iter()
                .map(|&face| (face, current(&replacements, face)))
                .collect(),
            deleted: Vec::new(),
            created,
            created_unattributed: Vec::new(),
        }),
        engine: crate::BlendEngine::Walking,
    })
}

/// Whether `face` traverses `edge` forward (wire flag, turned round on a
/// reversed face); `None` when it does not use it.
fn effective_use(topo: &Topology, face: FaceId, edge: EdgeId) -> Result<Option<bool>, BlendError> {
    let face = topo.face(face)?;
    Ok(topo
        .wire(face.outer_wire())?
        .edges()
        .iter()
        .find(|oe| oe.edge() == edge)
        .map(|oe| oe.is_forward() != face.is_reversed()))
}
