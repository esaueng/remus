//! Blend-cluster recognition: the dependent group of rolling-ball blends a
//! direct edit has to remove to sharp, edit and re-fillet together.
//!
//! Given a seed face (a face being edited, or a blend face whose radius is
//! being edited) the recognizer returns every blend the seed supports, or the
//! seed's own blend, closed transitively over every blend that shares an edge
//! with a member: corner patches, same-ball continuations, mitres and bands
//! that roll on other bands. For each member it reports its kind, the faces it
//! rolls on, its radius and convexity, and the sharp edge (or, for a corner,
//! the sharp vertex) that removing it restores, in closed form from the
//! supports. It also suggests a re-blend order: concave runs first, then the
//! convex edges that ride into the seam bands, with each corner patch in the
//! step that produces it.
//!
//! # Recognition is measurement, not trust
//!
//! A face is a blend only when a rolling ball explains it:
//!
//! - **Exact bands** (cylinder, torus) and corners (sphere): every contact
//!   edge with a support is a generator of the band's rolling family (a
//!   cylinder's ruling, a torus's latitude), the ball centre at every sample
//!   lies at the ball radius from the support's closed-form carrier, on the
//!   side the band's convexity demands, and its foot there is the contact
//!   point. The band subtends less than a half turn (a boss or bore wall is
//!   tangent to its seam bands along rulings too, but subtends more). Other
//!   tangent edges must continue the same ball into another blend, or be the
//!   rail of a blend that rolls on this one. At least two distinct supports
//!   are required (a sphere corner instead continues into at least two
//!   bands).
//! - **Sampled corner patches**: freeform faces are accepted only on the
//!   bounded circular-section family the walking and boss-corner builders
//!   store ([`remus_blend::cap_reblend::sampled_canal`]). Every stored section
//!   is checked against the rolling-ball closed form of the support its rail
//!   lies on (the centre at the ball radius from the carrier, its foot at the
//!   section end), and the surface between stations is measured against the
//!   interpolated ball. A sampled face meets its neighbours only as tangent
//!   as its station spacing allows, so the tangency tolerance across its edges
//!   is measured from the stations (twice the largest turn of the section
//!   plane between neighbouring stations), never assumed.
//!
//! A face the group reaches that looks like a blend but fails these checks is
//! a typed refusal ([`BlendClusterError::UnrecognizedMember`]), never silently
//! dropped from the group. Read-only: the topology is not changed.
//!
//! WASM module tier: `core` once bound (it reads the live session's handles);
//! no binding yet.

mod edit;
mod reblend;
mod remove;
mod sharp;

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use remus_blend::cap_reblend::{SampledCanal, sampled_canal, sampled_section_centre};
use remus_math::compact_sort::CompactSort;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::EdgeId;
use remus_topology::explorer::{face_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use self::sharp::{Carrier, Section};
use crate::OperationsError;
use crate::query::{edge_normal_angle, effective_face_normal};

pub use crate::error_payloads::BlendClusterError;
pub use edit::{GroupEdit, resize_band_through_group, resize_cylinder_through_group};
pub use reblend::{
    GroupReblend, ReblendStage, compose_evolution, plan_reblend, reblend_restoration,
};
pub use remove::{SharpRestoration, remove_blend_cluster, remove_blend_members};

/// Normal angle (radians) up to which two exact analytic faces count as
/// tangent across an edge.
const EXACT_ANGLE: f64 = 1e-7;
/// Distance (relative to `max(1, radius)`) to which exact ball centres must
/// satisfy the rolling-ball closed form.
const EXACT_DISTANCE: f64 = 1e-7;
/// Largest accepted deviation of a sampled carrier from the rolling-ball
/// closed form, as a fraction of its ball radius.
const SAMPLED_DEVIATION_FRACTION: f64 = 1e-3;
/// Parametric distance from a sampled carrier's `U` boundary within which an
/// edge is read as one of its rails.
const RAIL_PARAMETER: f64 = 1e-3;
/// Normal angle (radians) below which an unproven freeform neighbour counts
/// as touching the group tangentially, so it might be an unrecognized blend
/// and is refused rather than treated as an ordinary support.
const TANGENT_SCREEN: f64 = 1e-2;
/// Interior samples taken along every classified edge.
const EDGE_SAMPLES: u32 = 7;

/// What a blend member is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum BlendMemberKind {
    /// A rolling-ball band along one sharp edge between two supports.
    Band,
    /// A patch closing a corner where bands meet; removing it restores a
    /// sharp vertex, not an edge.
    CornerPatch,
}

/// The carrier a member lies on.
#[derive(Debug, Clone, Copy, PartialEq)]
#[non_exhaustive]
pub enum BlendMemberForm {
    /// An exact circular cylinder (a straight band).
    Cylinder,
    /// An exact torus (a band along a circular edge).
    Torus,
    /// An exact sphere (a corner ball).
    Sphere,
    /// A sampled circular-section carrier, recognized by measurement.
    SampledSections {
        /// Stored rolling stations.
        stations: usize,
        /// Largest measured deviation from the rolling-ball closed form
        /// (model units): station centres against their supports, and the
        /// surface between stations against the interpolated ball.
        deviation: f64,
        /// Tangency tolerance derived from the station spacing (radians).
        angle_tolerance: f64,
    },
}

impl BlendMemberForm {
    /// Whether the member lies on an exact analytic carrier.
    #[must_use]
    pub const fn is_exact(self) -> bool {
        !matches!(self, Self::SampledSections { .. })
    }
}

/// Which side of its supports a blend rounds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Convexity {
    /// Material was added: the ball rolls outside the body.
    Concave,
    /// Material was removed: the ball rolls inside the body.
    Convex,
}

/// The sharp feature removing a member restores.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemberSharp {
    /// Index into [`BlendCluster::sharp_edges`].
    Edge(usize),
    /// Index into [`BlendCluster::sharp_vertices`].
    Vertex(usize),
}

/// One blend of the group.
#[derive(Debug, Clone, PartialEq)]
pub struct BlendMember {
    /// The member's faces (a band may be split across several faces of one
    /// carrier), in arena-index order.
    pub faces: Vec<FaceId>,
    /// Band or corner patch.
    pub kind: BlendMemberKind,
    /// Carrier the member lies on.
    pub form: BlendMemberForm,
    /// Rolling-ball radius.
    pub radius: f64,
    /// Concave or convex.
    pub convexity: Convexity,
    /// Faces the ball rolls on, in arena-index order. A support may itself be
    /// a member (a corner patch rolling on a band).
    pub supports: Vec<FaceId>,
    /// Supports that are not members: the faces the sharp feature lies on.
    pub root_supports: Vec<FaceId>,
    /// Largest measured normal angle across the member's tangent edges.
    pub contact_angle: f64,
    /// The sharp edge or vertex the member's removal restores.
    pub sharp: MemberSharp,
}

/// Closed-form carrier of a predicted sharp edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SharpCarrier {
    /// The line through `point` along unit `direction`.
    Line {
        /// A point of the line.
        point: Point3,
        /// Unit direction.
        direction: Vec3,
    },
    /// The circle about unit `axis` through `center`.
    Circle {
        /// Centre.
        center: Point3,
        /// Unit axis.
        axis: Vec3,
        /// Radius.
        radius: f64,
    },
}

/// The sharp edge a band's removal restores: the exact section of its two
/// root supports between two predicted vertices.
#[derive(Debug, Clone, PartialEq)]
pub struct SharpEdge {
    /// Index of the band in [`BlendCluster::members`].
    pub member: usize,
    /// The two faces whose section the edge is, in arena-index order.
    pub supports: [FaceId; 2],
    /// Closed-form carrier.
    pub carrier: SharpCarrier,
    /// Vertex indices into [`BlendCluster::sharp_vertices`]; `None` for a
    /// closed circle.
    pub ends: Option<[usize; 2]>,
    /// A point of the edge between its ends (the band's own span), which
    /// names the arc of a circle.
    pub midpoint: Point3,
}

/// A predicted sharp vertex.
#[derive(Debug, Clone, PartialEq)]
pub struct SharpVertex {
    /// Position in closed form.
    pub point: Point3,
    /// Non-member faces the vertex lies on, in arena-index order.
    pub faces: Vec<FaceId>,
    /// Corner members whose removal collapses to this vertex.
    pub corners: Vec<usize>,
}

/// One suggested re-fillet stage.
#[derive(Debug, Clone, PartialEq)]
pub struct ReblendStep {
    /// Side the stage rounds.
    pub convexity: Convexity,
    /// Ball radius of the stage.
    pub radius: f64,
    /// Bands the stage rebuilds, with the corner patches it produces.
    pub members: Vec<usize>,
    /// Sharp edges to fillet in this stage.
    pub sharp_edges: Vec<usize>,
}

/// The dependent blend group of a seed face.
#[derive(Debug, Clone, PartialEq)]
pub struct BlendCluster {
    /// The seed the group was recognized from.
    pub seed: FaceId,
    /// Whether the seed is itself a member.
    pub seed_is_member: bool,
    /// Members in a deterministic order (by their smallest face index).
    pub members: Vec<BlendMember>,
    /// Every non-member face some member rolls on, in arena-index order.
    pub supports: Vec<FaceId>,
    /// Non-member faces that only bound members (end caps a band runs into),
    /// in arena-index order. Removal re-trims them; they carry no blend.
    pub boundary_faces: Vec<FaceId>,
    /// Predicted sharp edges, one per band.
    pub sharp_edges: Vec<SharpEdge>,
    /// Predicted sharp vertices: every sharp-edge end and corner point.
    pub sharp_vertices: Vec<SharpVertex>,
    /// Suggested re-fillet order.
    pub reblend_order: Vec<ReblendStep>,
}

impl BlendCluster {
    /// Every member face, in arena-index order.
    #[must_use]
    pub fn faces(&self) -> Vec<FaceId> {
        let mut faces: Vec<FaceId> = self
            .members
            .iter()
            .flat_map(|member| member.faces.iter().copied())
            .collect();
        faces.compact_sort_unstable_by_key(|face| face.index());
        faces
    }

    /// Whether every member lies on an exact analytic carrier.
    #[must_use]
    pub fn is_exact(&self) -> bool {
        self.members.iter().all(|member| member.form.is_exact())
    }

    /// The member containing `face`, if any.
    #[must_use]
    pub fn member_of(&self, face: FaceId) -> Option<usize> {
        self.members
            .iter()
            .position(|member| member.faces.contains(&face))
    }
}

/// Recognize the dependent blend group of `seed`.
///
/// # Errors
///
/// [`BlendClusterError::InvalidSeed`] when the seed is not a face of the
/// solid; [`BlendClusterError::NoDependentBlends`] when nothing recognized
/// rolls on it and it is not a blend; [`BlendClusterError::UnrecognizedMember`]
/// when the group reaches a blend-like face the measurements do not prove;
/// [`BlendClusterError::NoSharpForm`] when a member's sharp feature has no
/// closed form. Topology read errors propagate.
pub fn recognize_blend_cluster(
    topo: &Topology,
    solid: SolidId,
    seed: FaceId,
) -> Result<BlendCluster, OperationsError> {
    let faces = solid_faces(topo, solid)?;
    if !faces.contains(&seed) {
        return Err(BlendClusterError::InvalidSeed {
            reason: format!(
                "face {} is not part of solid {}",
                seed.index(),
                solid.index()
            ),
        }
        .into());
    }
    let adjacency = topo.build_adjacency(solid)?;
    let mut recognizer = Recognizer {
        topo,
        adjacency: &adjacency,
        facts: BTreeMap::new(),
    };
    let start = recognizer.start(seed)?;
    let members = recognizer.close(start)?;
    assemble(&mut recognizer, seed, &members)
}

// ---------------------------------------------------------------------------
// Per-face facts.

#[derive(Clone, Debug)]
enum Form {
    Cylinder {
        origin: Point3,
        axis: Vec3,
        radius: f64,
    },
    Torus {
        center: Point3,
        axis: Vec3,
        major: f64,
        minor: f64,
    },
    Sphere {
        center: Point3,
        radius: f64,
    },
    Sampled {
        canal: SampledCanal,
        angle_tolerance: f64,
    },
}

impl Form {
    const fn radius(&self) -> f64 {
        match self {
            Self::Cylinder { radius, .. } | Self::Sphere { radius, .. } => *radius,
            Self::Torus { minor, .. } => *minor,
            Self::Sampled { canal, .. } => canal.radius,
        }
    }

    const fn angle_tolerance(&self) -> f64 {
        match self {
            Self::Sampled {
                angle_tolerance, ..
            } => *angle_tolerance,
            Self::Cylinder { .. } | Self::Torus { .. } | Self::Sphere { .. } => EXACT_ANGLE,
        }
    }

    fn distance_tolerance(&self) -> f64 {
        match self {
            Self::Sampled { canal, .. } => SAMPLED_DEVIATION_FRACTION * canal.radius,
            Self::Cylinder { .. } | Self::Torus { .. } | Self::Sphere { .. } => {
                EXACT_DISTANCE * self.radius().max(1.0)
            }
        }
    }

    /// Whether two faces lie on one exact carrier (a split band).
    fn same_carrier(&self, other: &Self) -> bool {
        let close = |a: f64, b: f64| (a - b).abs() <= EXACT_DISTANCE * a.abs().max(1.0);
        let parallel = |a: Vec3, b: Vec3| a.cross(b).length() <= EXACT_ANGLE;
        match (self, other) {
            (
                Self::Cylinder {
                    origin: o1,
                    axis: a1,
                    radius: r1,
                },
                Self::Cylinder {
                    origin: o2,
                    axis: a2,
                    radius: r2,
                },
            ) => {
                close(*r1, *r2)
                    && parallel(*a1, *a2)
                    && radial(*o2 - *o1, *a1).length() <= EXACT_DISTANCE * r1.max(1.0)
            }
            (
                Self::Torus {
                    center: c1,
                    axis: a1,
                    major: m1,
                    minor: n1,
                },
                Self::Torus {
                    center: c2,
                    axis: a2,
                    major: m2,
                    minor: n2,
                },
            ) => {
                close(*m1, *m2)
                    && close(*n1, *n2)
                    && parallel(*a1, *a2)
                    && (*c2 - *c1).length() <= EXACT_DISTANCE * m1.max(1.0)
            }
            (
                Self::Sphere {
                    center: c1,
                    radius: r1,
                },
                Self::Sphere {
                    center: c2,
                    radius: r2,
                },
            ) => close(*r1, *r2) && (*c2 - *c1).length() <= EXACT_DISTANCE * r1.max(1.0),
            _ => false,
        }
    }
}

fn radial(vector: Vec3, axis: Vec3) -> Vec3 {
    vector - axis * vector.dot(axis)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Relation {
    /// The ball rolls on the neighbour along this edge.
    Contact,
    /// The same ball continues into the neighbour (a blend).
    Continuation,
    /// The neighbour is a blend rolling on this face along this edge.
    RiddenBy,
    /// A non-tangent edge.
    End,
    /// A tangent edge no rolling ball explains.
    Foreign,
}

#[derive(Clone, Debug)]
struct Link {
    edge: EdgeId,
    other: FaceId,
    relation: Relation,
    angle: Option<f64>,
}

#[derive(Clone, Debug)]
struct Facts {
    form: Form,
    convex: bool,
    links: Vec<Link>,
    /// `Ok` when the face is a proven blend; `Err` names why not.
    verdict: Result<(), String>,
    /// Largest sampled-carrier deviation (zero for exact carriers).
    deviation: f64,
}

#[derive(Clone, Debug)]
enum Entry {
    /// Not blend-shaped at all (plane, cone, a wide cylinder or torus).
    Plain,
    /// A NURBS face outside the circular-section family.
    Freeform(String),
    /// A blend-shaped face, proven or not.
    Shaped(Box<Facts>),
}

struct Recognizer<'a> {
    topo: &'a Topology,
    adjacency: &'a AdjacencyIndex,
    facts: BTreeMap<FaceId, Entry>,
}

impl Recognizer<'_> {
    fn sample_edge(&self, edge: EdgeId) -> Result<Vec<Point3>, OperationsError> {
        let data = self.topo.edge(edge)?;
        let start = self.topo.vertex(data.start())?.point();
        let end = self.topo.vertex(data.end())?.point();
        let (lo, hi) = crate::authoritative_edge_domain(data, "blend cluster samples")?;
        Ok((1..=EDGE_SAMPLES)
            .map(|index| {
                let fraction = f64::from(index) / f64::from(EDGE_SAMPLES + 1);
                data.curve()
                    .evaluate_with_endpoints((hi - lo).mul_add(fraction, lo), start, end)
            })
            .collect())
    }

    fn neighbours(&self, face: FaceId) -> Result<Vec<(EdgeId, FaceId)>, OperationsError> {
        let mut result = Vec::new();
        for edge in face_edges(self.topo, face)? {
            let mut users: Vec<FaceId> = self
                .adjacency
                .faces_for_edge(edge)
                .iter()
                .copied()
                .filter(|other| *other != face)
                .collect();
            users.compact_sort_unstable_by_key(|other| other.index());
            users.dedup();
            if let [other] = users.as_slice() {
                result.push((edge, *other));
            }
        }
        Ok(result)
    }

    /// Blend-shaped carrier of a face, or why it is not one.
    fn form(&self, face: FaceId) -> Result<Result<Form, Option<String>>, OperationsError> {
        let data = self.topo.face(face)?;
        Ok(match data.surface() {
            FaceSurface::Cylinder(cylinder) => {
                let form = Form::Cylinder {
                    origin: cylinder.origin(),
                    axis: cylinder.axis(),
                    radius: cylinder.radius(),
                };
                if self.half_turn_or_more(face, &form)? {
                    Err(None)
                } else {
                    Ok(form)
                }
            }
            FaceSurface::Torus(torus) => {
                let form = Form::Torus {
                    center: torus.center(),
                    axis: torus.z_axis(),
                    major: torus.major_radius(),
                    minor: torus.minor_radius(),
                };
                if self.half_turn_or_more(face, &form)? {
                    Err(None)
                } else {
                    Ok(form)
                }
            }
            FaceSurface::Sphere(sphere) => Ok(Form::Sphere {
                center: sphere.center(),
                radius: sphere.radius(),
            }),
            FaceSurface::Nurbs(_) => match sampled_canal(data) {
                Ok(canal) => {
                    let angle_tolerance = station_turn(&canal).mul_add(2.0, EXACT_ANGLE);
                    Ok(Form::Sampled {
                        canal,
                        angle_tolerance,
                    })
                }
                Err(error) => Err(Some(error.to_string())),
            },
            // Planes and cones carry no rolling-ball radius; a future
            // variant declines explicitly rather than by a wildcard.
            FaceSurface::Plane { .. } | FaceSurface::Cone(_) => Err(None),
        })
    }

    /// Whether a cylinder or torus face subtends at least a half turn in its
    /// rolling cross-section, which no fillet between two supports does.
    fn half_turn_or_more(&self, face: FaceId, form: &Form) -> Result<bool, OperationsError> {
        let mut angles = Vec::new();
        for edge in face_edges(self.topo, face)? {
            for point in self.sample_edge(edge)? {
                let angle = match form {
                    Form::Cylinder { origin, axis, .. } => {
                        let offset = radial(point - *origin, *axis);
                        let (x, y) = frame(*axis);
                        offset.dot(y).atan2(offset.dot(x))
                    }
                    Form::Torus {
                        center,
                        axis,
                        major,
                        ..
                    } => {
                        let offset = point - *center;
                        let height = offset.dot(*axis);
                        height.atan2(radial(offset, *axis).length() - major)
                    }
                    Form::Sphere { .. } | Form::Sampled { .. } => return Ok(false),
                };
                angles.push(angle);
            }
            let data = self.topo.edge(edge)?;
            for vertex in [data.start(), data.end()] {
                let point = self.topo.vertex(vertex)?.point();
                if let Form::Cylinder { origin, axis, .. } = form {
                    let offset = radial(point - *origin, *axis);
                    let (x, y) = frame(*axis);
                    angles.push(offset.dot(y).atan2(offset.dot(x)));
                }
            }
        }
        if angles.len() < 2 {
            return Ok(true);
        }
        angles.compact_sort_unstable_by(f64::total_cmp);
        let mut largest_gap = std::f64::consts::TAU - (angles[angles.len() - 1] - angles[0]);
        for pair in angles.windows(2) {
            largest_gap = largest_gap.max(pair[1] - pair[0]);
        }
        Ok(std::f64::consts::TAU - largest_gap >= std::f64::consts::PI - 1e-6)
    }

    fn entry(&mut self, face: FaceId) -> Result<Entry, OperationsError> {
        if let Some(entry) = self.facts.get(&face) {
            return Ok(entry.clone());
        }
        let entry = match self.form(face)? {
            Err(None) => Entry::Plain,
            Err(Some(reason)) => Entry::Freeform(reason),
            Ok(form) => Entry::Shaped(Box::new(self.facts_of(face, form)?)),
        };
        self.facts.insert(face, entry.clone());
        Ok(entry)
    }

    fn centre(&self, face: FaceId, form: &Form, point: Point3) -> Option<Point3> {
        match form {
            Form::Cylinder { origin, axis, .. } => {
                Some(*origin + *axis * (point - *origin).dot(*axis))
            }
            Form::Torus {
                center,
                axis,
                major,
                ..
            } => {
                let planar = radial(point - *center, *axis);
                let length = planar.length();
                (length > EXACT_DISTANCE).then(|| *center + planar * (*major / length))
            }
            Form::Sphere { center, .. } => Some(*center),
            Form::Sampled { .. } => {
                let data = self.topo.face(face).ok()?;
                sampled_section_centre(data, point).ok()
            }
        }
    }

    /// Whether an edge is a generator of the face's rolling family: a
    /// cylinder's ruling, a torus's latitude, a sampled carrier's rail.
    fn generator(&self, face: FaceId, form: &Form, points: &[Point3]) -> bool {
        let Some(first) = points.first().copied() else {
            return false;
        };
        match form {
            Form::Cylinder { axis, radius, .. } => points.iter().all(|point| {
                radial(*point - first, *axis).length() <= EXACT_DISTANCE * radius.max(1.0)
            }),
            Form::Torus {
                center,
                axis,
                major,
                ..
            } => {
                let latitude = |point: Point3| {
                    let offset = point - *center;
                    offset
                        .dot(*axis)
                        .atan2(radial(offset, *axis).length() - major)
                };
                let reference = latitude(first);
                points
                    .iter()
                    .all(|point| (latitude(*point) - reference).abs() <= 1e-6)
            }
            Form::Sphere { .. } => true,
            Form::Sampled { .. } => self.rail_side(face, points).is_some(),
        }
    }

    /// Which `U` boundary (`false` = 0, `true` = 1) of a sampled carrier the
    /// points lie on.
    fn rail_side(&self, face: FaceId, points: &[Point3]) -> Option<bool> {
        let surface = self.topo.face(face).ok()?.surface();
        let mut side = None;
        for point in points {
            let (u, _) = surface.project_point(*point)?;
            let this = if u.abs() <= RAIL_PARAMETER {
                false
            } else if (u - 1.0).abs() <= RAIL_PARAMETER {
                true
            } else {
                return None;
            };
            if side.is_some_and(|side| side != this) {
                return None;
            }
            side = Some(this);
        }
        side
    }

    /// Signed offset of `point` from a support's closed-form carrier along
    /// its outward normal, with the carrier's closest point.
    fn support_offset(&self, support: FaceId, point: Point3) -> Option<(f64, Point3)> {
        let data = self.topo.face(support).ok()?;
        let sense = if data.is_reversed() { -1.0 } else { 1.0 };
        let (offset, foot) = match data.surface() {
            FaceSurface::Plane { normal, d } => {
                let length = normal.length();
                let unit = *normal * (1.0 / length);
                let height = unit.dot(Vec3::new(point.x(), point.y(), point.z())) - d / length;
                (height, point - unit * height)
            }
            FaceSurface::Cylinder(cylinder) => {
                let offset = radial(point - cylinder.origin(), cylinder.axis());
                let length = offset.length();
                if length <= EXACT_DISTANCE {
                    return None;
                }
                (
                    length - cylinder.radius(),
                    point - offset * ((length - cylinder.radius()) / length),
                )
            }
            FaceSurface::Sphere(sphere) => {
                let offset = point - sphere.center();
                let length = offset.length();
                if length <= EXACT_DISTANCE {
                    return None;
                }
                (
                    length - sphere.radius(),
                    point - offset * ((length - sphere.radius()) / length),
                )
            }
            FaceSurface::Torus(torus) => {
                let axis = torus.z_axis();
                let planar = radial(point - torus.center(), axis);
                let length = planar.length();
                if length <= EXACT_DISTANCE {
                    return None;
                }
                let spine = torus.center() + planar * (torus.major_radius() / length);
                let offset = point - spine;
                let distance = offset.length();
                if distance <= EXACT_DISTANCE {
                    return None;
                }
                (
                    distance - torus.minor_radius(),
                    point - offset * ((distance - torus.minor_radius()) / distance),
                )
            }
            FaceSurface::Cone(_) | FaceSurface::Nurbs(_) => return None,
        };
        Some((offset * sense, foot))
    }

    /// Largest rolling-ball residual of `face`'s ball at `points` against a
    /// support: the centre's offset from `ball` (signed by convexity) and the
    /// distance from the centre's foot to the contact point.
    fn contact_residual(
        &self,
        face: FaceId,
        form: &Form,
        convex: bool,
        support: FaceId,
        points: &[Point3],
    ) -> Option<f64> {
        let expected = if convex {
            -form.radius()
        } else {
            form.radius()
        };
        let mut residual = 0.0_f64;
        for point in points {
            let centre = self.centre(face, form, *point)?;
            let (offset, foot) = self.support_offset(support, centre)?;
            residual = residual
                .max((offset - expected).abs())
                .max((foot - *point).length());
        }
        Some(residual)
    }

    fn convex(&self, face: FaceId, form: &Form) -> Result<Option<bool>, OperationsError> {
        if let Form::Sampled { canal, .. } = form {
            return Ok(Some(canal.convex));
        }
        for edge in face_edges(self.topo, face)? {
            for point in self.sample_edge(edge)? {
                if let (Some(normal), Some(centre)) = (
                    effective_face_normal(self.topo, face, point),
                    self.centre(face, form, point),
                ) {
                    let side = (point - centre).dot(normal);
                    if side.abs() > 0.5 * form.radius() {
                        return Ok(Some(side > 0.0));
                    }
                }
            }
        }
        Ok(None)
    }

    #[allow(clippy::too_many_lines)]
    fn facts_of(&self, face: FaceId, form: Form) -> Result<Facts, OperationsError> {
        let Some(convex) = self.convex(face, &form)? else {
            return Ok(Facts {
                form,
                convex: false,
                links: Vec::new(),
                verdict: Err("its material side could not be measured".into()),
                deviation: 0.0,
            });
        };
        let mut links = Vec::new();
        let mut contacts = BTreeSet::new();
        let mut continuations = 0_usize;
        let mut verdict = Ok(());
        for (edge, other) in self.neighbours(face)? {
            let other_form = self.form(other)?;
            let tolerance = form.angle_tolerance().max(
                other_form
                    .as_ref()
                    .map_or(EXACT_ANGLE, Form::angle_tolerance),
            );
            let angle = edge_normal_angle(self.topo, edge, face, other)?;
            let tangent = angle.is_some_and(|angle| angle <= tolerance);
            let relation = if tangent {
                let points = self.sample_edge(edge)?;
                let distance = form
                    .distance_tolerance()
                    .max(other_form.as_ref().map_or(0.0, Form::distance_tolerance));
                let continues = other_form.as_ref().is_ok_and(|other_form| {
                    (other_form.radius() - form.radius()).abs() <= distance
                        && points.iter().all(|point| {
                            match (
                                self.centre(face, &form, *point),
                                self.centre(other, other_form, *point),
                            ) {
                                (Some(a), Some(b)) => (a - b).length() <= distance,
                                _ => false,
                            }
                        })
                });
                if continues {
                    Relation::Continuation
                } else if self.generator(face, &form, &points)
                    && self
                        .contact_residual(face, &form, convex, other, &points)
                        .is_some_and(|residual| residual <= form.distance_tolerance())
                {
                    Relation::Contact
                } else if other_form
                    .as_ref()
                    .is_ok_and(|other_form| self.generator(other, other_form, &points))
                {
                    Relation::RiddenBy
                } else {
                    Relation::Foreign
                }
            } else {
                Relation::End
            };
            match relation {
                Relation::Contact => {
                    contacts.insert(other);
                }
                Relation::Continuation => continuations += 1,
                Relation::Foreign if verdict.is_ok() => {
                    verdict = Err(format!(
                        "its tangent edge {} with face {} is neither a rolling contact nor a continuation",
                        edge.index(),
                        other.index()
                    ));
                }
                Relation::RiddenBy | Relation::End | Relation::Foreign => {}
            }
            links.push(Link {
                edge,
                other,
                relation,
                angle,
            });
        }
        if verdict.is_ok() {
            let enough =
                contacts.len() >= 2 || (matches!(form, Form::Sphere { .. }) && continuations >= 2);
            if !enough {
                verdict = Err(format!(
                    "it rolls on {} distinct supports and continues into {continuations} blends",
                    contacts.len()
                ));
            }
        }
        let mut deviation = 0.0;
        if verdict.is_ok()
            && let Form::Sampled { canal, .. } = &form
        {
            match self.sampled_deviation(face, canal, &links) {
                Ok(measured) => {
                    deviation = measured;
                    let limit = form.distance_tolerance();
                    if measured > limit {
                        verdict = Err(format!(
                            "its sections deviate {measured:.3e} from the rolling-ball closed form (limit {limit:.3e})"
                        ));
                    }
                }
                Err(reason) => verdict = Err(reason),
            }
        }
        Ok(Facts {
            form,
            convex,
            links,
            verdict,
            deviation,
        })
    }

    /// Measured deviation of a sampled carrier from the rolling-ball closed
    /// form: each stored section's centre at the ball radius from the support
    /// its rail lies on with its foot at the section end, the interpolated
    /// centre path between stations likewise, and the surface between
    /// stations on the interpolated ball.
    fn sampled_deviation(
        &self,
        face: FaceId,
        canal: &SampledCanal,
        links: &[Link],
    ) -> Result<f64, String> {
        let expected = if canal.convex {
            -canal.radius
        } else {
            canal.radius
        };
        let mut rails: [Vec<FaceId>; 2] = [Vec::new(), Vec::new()];
        for link in links
            .iter()
            .filter(|link| link.relation == Relation::Contact)
        {
            let points = self
                .sample_edge(link.edge)
                .map_err(|error| error.to_string())?;
            let side = self
                .rail_side(face, &points)
                .ok_or_else(|| format!("contact edge {} is not a rail", link.edge.index()))?;
            rails[usize::from(side)].push(link.other);
        }
        if rails.iter().any(Vec::is_empty) {
            return Err("a rail of the sampled carrier has no rolling support".into());
        }
        let data = self.topo.face(face).map_err(|error| error.to_string())?;
        let FaceSurface::Nurbs(surface) = data.surface() else {
            return Err("sampled carrier lost its surface".into());
        };
        // The support a section end touches is the rail support nearest it.
        let residual = |centre: Point3, contact: Option<Point3>, side: usize| -> Option<f64> {
            rails[side]
                .iter()
                .filter_map(|support| {
                    let (offset, foot) = self.support_offset(*support, centre)?;
                    let at = contact.map_or(0.0, |contact| (foot - contact).length());
                    Some((offset - expected).abs().max(at))
                })
                .min_by(f64::total_cmp)
        };
        let mut deviation = 0.0_f64;
        for section in &canal.sections {
            for (side, contact) in [(0, section.start), (1, section.end)] {
                let value = residual(section.centre, Some(contact), side)
                    .ok_or_else(|| "a section's support has no closed form".to_string())?;
                deviation = deviation.max(value);
            }
        }
        for pair in canal.sections.windows(2) {
            let v = 0.5 * (pair[0].v + pair[1].v);
            let probe = surface.evaluate(0.5, v);
            let centre = sampled_section_centre(data, probe).map_err(|error| error.to_string())?;
            for side in 0..2 {
                let value = residual(centre, None, side)
                    .ok_or_else(|| "a section's support has no closed form".to_string())?;
                deviation = deviation.max(value);
            }
            for step in 0..=4_u32 {
                let point = surface.evaluate(f64::from(step) / 4.0, v);
                deviation = deviation.max(((point - centre).length() - canal.radius).abs());
            }
        }
        Ok(deviation)
    }

    /// The face's facts when it is a proven blend.
    fn member_facts(&mut self, face: FaceId) -> Result<Option<Facts>, OperationsError> {
        Ok(match self.entry(face)? {
            Entry::Shaped(facts) if facts.verdict.is_ok() => Some(*facts),
            Entry::Shaped(_) | Entry::Plain | Entry::Freeform(_) => None,
        })
    }

    fn unrecognized(&mut self, face: FaceId) -> Result<OperationsError, OperationsError> {
        let reason = match self.entry(face)? {
            Entry::Freeform(reason) => {
                format!("a freeform face outside the circular-section family ({reason})")
            }
            Entry::Shaped(facts) => facts
                .verdict
                .err()
                .unwrap_or_else(|| "it is not a blend".into()),
            Entry::Plain => "it carries no rolling-ball radius".into(),
        };
        Ok(BlendClusterError::UnrecognizedMember {
            face: face.index(),
            reason,
        }
        .into())
    }

    /// Whether a non-member neighbour must be refused because it is tangent
    /// to the group yet blend-shaped and unproven.
    fn refuses_tangent(&mut self, face: FaceId) -> Result<bool, OperationsError> {
        Ok(match self.entry(face)? {
            Entry::Freeform(_) => true,
            Entry::Shaped(facts) => matches!(facts.form, Form::Sampled { .. }),
            Entry::Plain => false,
        })
    }

    /// The members the group starts from.
    fn start(&mut self, seed: FaceId) -> Result<Vec<FaceId>, OperationsError> {
        if self.member_facts(seed)?.is_some() {
            return Ok(vec![seed]);
        }
        if self.refuses_tangent(seed)? {
            // A freeform or sampled seed that is not a proven blend: refuse
            // only when it is tangent to a neighbour (it might be a blend we
            // cannot prove); otherwise it is an ordinary freeform support.
            for (edge, other) in self.neighbours(seed)? {
                if edge_normal_angle(self.topo, edge, seed, other)?
                    .is_some_and(|angle| angle <= TANGENT_SCREEN)
                {
                    return Err(self.unrecognized(seed)?);
                }
            }
        }
        let mut start = Vec::new();
        for (edge, other) in self.neighbours(seed)? {
            if let Some(facts) = self.member_facts(other)? {
                if facts
                    .links
                    .iter()
                    .any(|link| link.edge == edge && link.relation == Relation::Contact)
                {
                    start.push(other);
                }
            } else {
                let tangent = edge_normal_angle(self.topo, edge, seed, other)?
                    .is_some_and(|angle| angle <= TANGENT_SCREEN);
                if tangent && self.refuses_tangent(other)? {
                    return Err(self.unrecognized(other)?);
                }
            }
        }
        if start.is_empty() {
            return Err(BlendClusterError::NoDependentBlends { seed: seed.index() }.into());
        }
        start.compact_sort_unstable_by_key(|face| face.index());
        start.dedup();
        Ok(start)
    }

    /// Close the group over every blend sharing an edge with a member.
    fn close(&mut self, start: Vec<FaceId>) -> Result<BTreeSet<FaceId>, OperationsError> {
        let mut members: BTreeSet<FaceId> = start.iter().copied().collect();
        let mut queue: VecDeque<FaceId> = start.into();
        while let Some(face) = queue.pop_front() {
            let Some(facts) = self.member_facts(face)? else {
                return Err(self.unrecognized(face)?);
            };
            for link in &facts.links {
                let other_member = self.member_facts(link.other)?.is_some();
                match link.relation {
                    Relation::Continuation | Relation::RiddenBy if !other_member => {
                        return Err(self.unrecognized(link.other)?);
                    }
                    Relation::Contact | Relation::End if !other_member => {
                        let tangent = link.angle.is_some_and(|angle| angle <= TANGENT_SCREEN);
                        if tangent && self.refuses_tangent(link.other)? {
                            return Err(self.unrecognized(link.other)?);
                        }
                    }
                    _ => {}
                }
                if other_member && members.insert(link.other) {
                    queue.push_back(link.other);
                }
            }
        }
        Ok(members)
    }
}

/// Largest turn of the section plane between neighbouring stored stations.
fn station_turn(canal: &SampledCanal) -> f64 {
    let normal = |section: &remus_blend::cap_reblend::SampledSection| {
        (section.start - section.centre)
            .cross(section.end - section.centre)
            .normalize()
            .ok()
    };
    canal
        .sections
        .windows(2)
        .filter_map(|pair| {
            let (a, b) = (normal(&pair[0])?, normal(&pair[1])?);
            Some(a.cross(b).length().atan2(a.dot(b)))
        })
        .fold(0.0, f64::max)
}

fn frame(axis: Vec3) -> (Vec3, Vec3) {
    let seed = if axis.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let x = radial(seed, axis);
    let x = x * (1.0 / x.length());
    (x, axis.cross(x))
}

// ---------------------------------------------------------------------------
// Assembly: members, sharp features, re-blend order.

struct Draft {
    faces: Vec<FaceId>,
    form: Form,
    convex: bool,
    supports: BTreeSet<FaceId>,
    neighbours: BTreeSet<FaceId>,
    /// Neighbours the member rolls on or continues into.
    linked: BTreeSet<FaceId>,
    contact_angle: f64,
    deviation: f64,
}

fn no_sharp(face: FaceId, reason: impl Into<String>) -> OperationsError {
    BlendClusterError::NoSharpForm {
        face: face.index(),
        reason: reason.into(),
    }
    .into()
}

#[allow(clippy::too_many_lines)]
fn assemble(
    recognizer: &mut Recognizer<'_>,
    seed: FaceId,
    faces: &BTreeSet<FaceId>,
) -> Result<BlendCluster, OperationsError> {
    let topo = recognizer.topo;
    let mut facts = BTreeMap::new();
    for &face in faces {
        let Some(found) = recognizer.member_facts(face)? else {
            return Err(recognizer.unrecognized(face)?);
        };
        facts.insert(face, found);
    }

    // Split bands: faces of one exact carrier joined by a continuation edge.
    let mut owner: BTreeMap<FaceId, FaceId> = faces.iter().map(|face| (*face, *face)).collect();
    let find = |owner: &BTreeMap<FaceId, FaceId>, mut face: FaceId| {
        while owner[&face] != face {
            face = owner[&face];
        }
        face
    };
    for (&face, fact) in &facts {
        for link in &fact.links {
            if link.relation == Relation::Continuation
                && let Some(other) = facts.get(&link.other)
                && fact.form.same_carrier(&other.form)
            {
                let (a, b) = (find(&owner, face), find(&owner, link.other));
                if a != b {
                    owner.insert(a.max(b), a.min(b));
                }
            }
        }
    }
    let mut groups: BTreeMap<FaceId, Vec<FaceId>> = BTreeMap::new();
    for &face in faces {
        groups.entry(find(&owner, face)).or_default().push(face);
    }
    let mut drafts = Vec::new();
    let mut member_of = BTreeMap::new();
    for (root, group) in groups {
        let first = &facts[&root];
        let mut draft = Draft {
            faces: group.clone(),
            form: first.form.clone(),
            convex: first.convex,
            supports: BTreeSet::new(),
            neighbours: BTreeSet::new(),
            linked: BTreeSet::new(),
            contact_angle: 0.0,
            deviation: 0.0,
        };
        for face in &group {
            let fact = &facts[face];
            if fact.convex != draft.convex {
                return Err(no_sharp(*face, "faces of one band round opposite sides"));
            }
            draft.deviation = draft.deviation.max(fact.deviation);
            for link in &fact.links {
                if group.contains(&link.other) {
                    continue;
                }
                if matches!(
                    link.relation,
                    Relation::Contact | Relation::Continuation | Relation::RiddenBy
                ) {
                    draft.contact_angle = draft.contact_angle.max(link.angle.unwrap_or(0.0));
                }
                if link.relation == Relation::Contact {
                    draft.supports.insert(link.other);
                }
                if matches!(link.relation, Relation::Contact | Relation::Continuation) {
                    draft.linked.insert(link.other);
                }
                draft.neighbours.insert(link.other);
            }
        }
        for face in &group {
            member_of.insert(*face, drafts.len());
        }
        drafts.push(draft);
    }

    let roots = |draft: &Draft| -> BTreeSet<FaceId> {
        draft
            .supports
            .iter()
            .copied()
            .filter(|face| !member_of.contains_key(face))
            .collect()
    };
    let mut supports = BTreeSet::new();
    let mut boundary = BTreeSet::new();
    for draft in &drafts {
        supports.extend(roots(draft));
        for neighbour in &draft.neighbours {
            if !member_of.contains_key(neighbour) {
                boundary.insert(*neighbour);
            }
        }
    }
    let boundary: BTreeSet<FaceId> = boundary.difference(&supports).copied().collect();

    // Roots a corner (or any member) collapses onto, widened by the roots of
    // the members it rolls on or continues into.
    let expanded = |index: usize| -> BTreeSet<FaceId> {
        let draft = &drafts[index];
        let mut result = roots(draft);
        for neighbour in &draft.linked {
            if let Some(&other) = member_of.get(neighbour)
                && other != index
            {
                result.extend(roots(&drafts[other]));
            }
        }
        result
    };

    let mut vertices: Vec<SharpVertex> = Vec::new();
    let mut add_vertex = |point: Point3, on: &BTreeSet<FaceId>, corner: Option<usize>| -> usize {
        let scale = 1e-6 * (point - Point3::new(0.0, 0.0, 0.0)).length().max(1.0);
        let index = vertices
            .iter()
            .position(|vertex| (vertex.point - point).length() <= scale)
            .unwrap_or_else(|| {
                vertices.push(SharpVertex {
                    point,
                    faces: Vec::new(),
                    corners: Vec::new(),
                });
                vertices.len() - 1
            });
        let vertex = &mut vertices[index];
        for face in on {
            if !vertex.faces.contains(face) {
                vertex.faces.push(*face);
            }
        }
        if let Some(corner) = corner
            && !vertex.corners.contains(&corner)
        {
            vertex.corners.push(corner);
        }
        index
    };

    let mut members = Vec::with_capacity(drafts.len());
    let mut sharp_edges = Vec::new();
    let mut corner_vertex: BTreeMap<usize, usize> = BTreeMap::new();
    for (index, draft) in drafts.iter().enumerate() {
        let face = draft.faces[0];
        let root = roots(draft);
        match root.len() {
            2 => {}
            0 | 1 => continue,
            count => {
                return Err(no_sharp(
                    face,
                    format!("the band rolls on {count} non-blend supports"),
                ));
            }
        }
        let pair: Vec<FaceId> = root.iter().copied().collect();
        let carrier = |support: FaceId| -> Result<Carrier, OperationsError> {
            Carrier::of(topo.face(support)?.surface()).ok_or_else(|| {
                no_sharp(
                    face,
                    format!("support {} has no closed-form carrier", support.index()),
                )
            })
        };
        let (first, second) = (carrier(pair[0])?, carrier(pair[1])?);
        let reference = band_reference(recognizer, &draft.faces, &facts)?;
        let section =
            sharp::section(first, second, reference).map_err(|reason| no_sharp(face, reason))?;
        let midpoint = section
            .project(reference)
            .ok_or_else(|| no_sharp(face, "the band lies on its section's axis"))?;
        // Ends: every neighbour that is not one of the two roots names the
        // third face(s) the sharp edge runs into.
        let mut ends: Vec<(Point3, Option<usize>, BTreeSet<FaceId>)> = Vec::new();
        for neighbour in &draft.neighbours {
            if root.contains(neighbour) {
                continue;
            }
            let (thirds, corner) = match member_of.get(neighbour) {
                Some(&other) => {
                    let mut thirds = expanded(other);
                    for face in &root {
                        thirds.remove(face);
                    }
                    let corner = (roots(&drafts[other]).len() < 2).then_some(other);
                    (thirds, corner)
                }
                None if supports.contains(neighbour) || boundary.contains(neighbour) => {
                    (BTreeSet::from([*neighbour]), None)
                }
                None => continue,
            };
            if thirds.is_empty() {
                continue;
            }
            let near = shared_reference(recognizer, &draft.faces, *neighbour)?;
            for third in thirds {
                let third_carrier = carrier(third)?;
                let points =
                    sharp::meet(section, third_carrier).map_err(|reason| no_sharp(face, reason))?;
                let Some(point) = points
                    .into_iter()
                    .min_by(|a, b| (*a - near).length().total_cmp(&(*b - near).length()))
                else {
                    return Err(no_sharp(
                        face,
                        format!("its sharp edge misses face {}", third.index()),
                    ));
                };
                let mut on: BTreeSet<FaceId> = root.clone();
                on.insert(third);
                ends.push((point, corner, on));
            }
        }
        let mut distinct: Vec<usize> = Vec::new();
        let mut distinct_points: Vec<Point3> = Vec::new();
        for (point, corner, on) in &ends {
            let vertex = add_vertex(*point, on, *corner);
            if let Some(corner) = corner {
                match corner_vertex.get(corner) {
                    Some(existing) if *existing != vertex => {
                        return Err(no_sharp(
                            drafts[*corner].faces[0],
                            "the corner collapses to two different sharp vertices",
                        ));
                    }
                    _ => {
                        corner_vertex.insert(*corner, vertex);
                    }
                }
            }
            if !distinct.contains(&vertex) {
                distinct.push(vertex);
                distinct_points.push(*point);
            }
        }
        let midpoint = match (distinct_points.as_slice(), section) {
            ([a, b], Section::Line { .. }) => *a + (*b - *a) * 0.5,
            ([a, b], Section::Circle { .. }) => {
                let samples = band_samples(recognizer, &draft.faces, &facts)?;
                arc_midpoint(section, *a, *b, &samples)
                    .ok_or_else(|| no_sharp(face, "its sharp arc's ends coincide"))?
            }
            _ => midpoint,
        };
        let ends = match (distinct.as_slice(), section) {
            ([a, b], _) => Some([*a, *b]),
            ([], Section::Circle { .. }) => None,
            (found, _) => {
                return Err(no_sharp(
                    face,
                    format!("its sharp edge has {} predicted ends", found.len()),
                ));
            }
        };
        members.push((index, MemberSharp::Edge(sharp_edges.len())));
        sharp_edges.push(SharpEdge {
            member: index,
            supports: [pair[0], pair[1]],
            carrier: match section {
                Section::Line { point, direction } => SharpCarrier::Line { point, direction },
                Section::Circle {
                    center,
                    axis,
                    radius,
                } => SharpCarrier::Circle {
                    center,
                    axis,
                    radius,
                },
            },
            ends,
            midpoint,
        });
    }

    // Corners: each must have collapsed to exactly one predicted vertex that
    // lies on every face it rolls on or continues onto.
    for (index, draft) in drafts.iter().enumerate() {
        if roots(draft).len() >= 2 {
            continue;
        }
        let Some(&vertex) = corner_vertex.get(&index) else {
            return Err(no_sharp(
                draft.faces[0],
                "no band ends at the corner, so its sharp vertex is unknown",
            ));
        };
        let point = vertices[vertex].point;
        for support in expanded(index) {
            let Some(carrier) = Carrier::of(topo.face(support)?.surface()) else {
                return Err(no_sharp(
                    draft.faces[0],
                    format!("support {} has no closed-form carrier", support.index()),
                ));
            };
            if carrier.distance(point) > 1e-6 * draft.form.radius().max(1.0) {
                return Err(no_sharp(
                    draft.faces[0],
                    format!(
                        "the corner's sharp vertex is off its support {}",
                        support.index()
                    ),
                ));
            }
        }
        members.push((index, MemberSharp::Vertex(vertex)));
    }
    members.compact_sort_unstable_by_key(|(index, _)| *index);

    let out: Vec<BlendMember> = members
        .iter()
        .map(|(index, sharp)| {
            let draft = &drafts[*index];
            let mut faces = draft.faces.clone();
            faces.compact_sort_unstable_by_key(|face| face.index());
            BlendMember {
                faces,
                kind: if matches!(sharp, MemberSharp::Edge(_)) {
                    BlendMemberKind::Band
                } else {
                    BlendMemberKind::CornerPatch
                },
                form: match &draft.form {
                    Form::Cylinder { .. } => BlendMemberForm::Cylinder,
                    Form::Torus { .. } => BlendMemberForm::Torus,
                    Form::Sphere { .. } => BlendMemberForm::Sphere,
                    Form::Sampled {
                        canal,
                        angle_tolerance,
                    } => BlendMemberForm::SampledSections {
                        stations: canal.sections.len(),
                        deviation: draft.deviation,
                        angle_tolerance: *angle_tolerance,
                    },
                },
                radius: draft.form.radius(),
                convexity: if draft.convex {
                    Convexity::Convex
                } else {
                    Convexity::Concave
                },
                supports: draft.supports.iter().copied().collect(),
                root_supports: roots(draft).into_iter().collect(),
                contact_angle: draft.contact_angle,
                sharp: *sharp,
            }
        })
        .collect();
    for vertex in &mut vertices {
        vertex
            .faces
            .compact_sort_unstable_by_key(|face| face.index());
        vertex.corners.compact_sort_unstable();
    }

    let reblend_order = reblend_order(&out, &drafts, &member_of);
    Ok(BlendCluster {
        seed,
        seed_is_member: member_of.contains_key(&seed),
        members: out,
        supports: supports.into_iter().collect(),
        boundary_faces: boundary.into_iter().collect(),
        sharp_edges,
        sharp_vertices: vertices,
        reblend_order,
    })
}

/// A point on a band, near its sharp edge's span: the midpoint of its first
/// contact edge.
fn band_reference(
    recognizer: &Recognizer<'_>,
    faces: &[FaceId],
    facts: &BTreeMap<FaceId, Facts>,
) -> Result<Point3, OperationsError> {
    for face in faces {
        if let Some(link) = facts[face]
            .links
            .iter()
            .find(|link| link.relation == Relation::Contact)
        {
            let points = recognizer.sample_edge(link.edge)?;
            return Ok(points[points.len() / 2]);
        }
    }
    Err(no_sharp(faces[0], "the band has no contact edge"))
}

/// Samples of every contact edge of a band.
fn band_samples(
    recognizer: &Recognizer<'_>,
    faces: &[FaceId],
    facts: &BTreeMap<FaceId, Facts>,
) -> Result<Vec<Point3>, OperationsError> {
    let mut samples = Vec::new();
    for face in faces {
        for link in facts[face]
            .links
            .iter()
            .filter(|link| link.relation == Relation::Contact)
        {
            samples.extend(recognizer.sample_edge(link.edge)?);
        }
    }
    Ok(samples)
}

/// Midpoint of the arc from `a` to `b` on a circular section that holds
/// most of the band's projected contact samples.
fn arc_midpoint(section: Section, a: Point3, b: Point3, samples: &[Point3]) -> Option<Point3> {
    let Section::Circle {
        center,
        axis,
        radius,
    } = section
    else {
        return None;
    };
    let (x, y) = frame(axis);
    let angle = |point: Point3| {
        let offset = radial(point - center, axis);
        offset.dot(y).atan2(offset.dot(x))
    };
    let tau = std::f64::consts::TAU;
    let start = angle(a);
    let sweep = (angle(b) - start).rem_euclid(tau);
    if sweep <= 1e-12 {
        return None;
    }
    let inside = samples
        .iter()
        .filter(|point| (angle(**point) - start).rem_euclid(tau) <= sweep)
        .count();
    let middle = if 2 * inside >= samples.len() {
        0.5f64.mul_add(sweep, start)
    } else {
        0.5f64.mul_add(sweep - tau, start)
    };
    let (sin, cos) = middle.sin_cos();
    Some(center + (x * cos + y * sin) * radius)
}

/// Midpoint of an edge a member shares with `neighbour`.
fn shared_reference(
    recognizer: &Recognizer<'_>,
    faces: &[FaceId],
    neighbour: FaceId,
) -> Result<Point3, OperationsError> {
    for face in faces {
        for (edge, other) in recognizer.neighbours(*face)? {
            if other == neighbour {
                let points = recognizer.sample_edge(edge)?;
                return Ok(points[points.len() / 2]);
            }
        }
    }
    Err(no_sharp(
        faces[0],
        "the member shares no edge with its neighbour",
    ))
}

/// Concave stages first, then convex, larger radii first within each side;
/// a corner joins the latest stage among the bands it touches.
fn reblend_order(
    members: &[BlendMember],
    drafts: &[Draft],
    member_of: &BTreeMap<FaceId, usize>,
) -> Vec<ReblendStep> {
    let mut steps: Vec<ReblendStep> = Vec::new();
    let mut bands: Vec<usize> = members
        .iter()
        .enumerate()
        .filter(|(_, member)| member.kind == BlendMemberKind::Band)
        .map(|(index, _)| index)
        .collect();
    bands.compact_sort_by(|a, b| {
        let (a, b) = (&members[*a], &members[*b]);
        a.convexity
            .cmp(&b.convexity)
            .then(b.radius.total_cmp(&a.radius))
    });
    let mut stage_of = BTreeMap::new();
    for band in bands {
        let member = &members[band];
        let same = steps.last().is_some_and(|step| {
            step.convexity == member.convexity
                && (step.radius - member.radius).abs() <= EXACT_DISTANCE * member.radius.max(1.0)
        });
        if !same {
            steps.push(ReblendStep {
                convexity: member.convexity,
                radius: member.radius,
                members: Vec::new(),
                sharp_edges: Vec::new(),
            });
        }
        let step = steps.len() - 1;
        steps[step].members.push(band);
        if let MemberSharp::Edge(edge) = member.sharp {
            steps[step].sharp_edges.push(edge);
        }
        stage_of.insert(band, step);
    }
    for (index, member) in members.iter().enumerate() {
        if member.kind != BlendMemberKind::CornerPatch {
            continue;
        }
        let latest = drafts[index]
            .neighbours
            .iter()
            .filter_map(|face| member_of.get(face))
            .filter_map(|other| stage_of.get(other))
            .max()
            .copied();
        if let Some(step) = latest {
            steps[step].members.push(index);
        }
    }
    for step in &mut steps {
        step.members.compact_sort_unstable();
    }
    steps
}
