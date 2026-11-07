//! Rolling sections for a boss seam crossing a convex round.
//!
//! The boss is parallel to local Z. The convex strip has its axis parallel to
//! local X, at local Y = r, Z = 0. Its rolling-ball offset has radius 2r;
//! the boss offset has radius R+r. Their intersection determines each centre.
//! Unlike rounding the concave seam first and walking a convex ridge over it,
//! this construction keeps the boss as the rail throughout the corner.
//!
//! # Unequal radii
//!
//! The concave seam up the block wall (radius `r_s`), the concave shoulder
//! round the boss on the block top (`r_h`) and the convex round (`r_c`) need
//! not agree. The ball then still rolls on the boss over the convex round,
//! but its radius runs from `r_s` (where the round leaves the wall) to `r_h`
//! (where it reaches the top) by the Hermite ramp `r(t) = r_s + (r_h −
//! r_s)(3t² − 2t³)`, `t` the quarter turn about the round's axis: its centre
//! lies on the boss offset `ρ = R + r(t)` and on the round's offset at
//! `r_c + r(t)`. The patch is the envelope of that sphere family, the
//! surface the rolling ball actually sweeps: each section is the ball's
//! characteristic circle, cut by the plane `(X − c)·c′ = −r r′`. Both
//! contacts lie on it (the boss contact because `d|c_xy|/dt = r′`, the
//! round's because its offset grows at the same rate), so the patch is
//! tangent to the boss and the round along them. `r′` vanishes at both
//! ends, where the section is the seam cylinder's latitude and the shoulder
//! torus's meridian, so it meets both bands G1. (A great circle of each ball
//! through the same contacts, the evolving-radius section of a walked
//! variable fillet, would cut up to a sixth of the radius change into the
//! neighbouring balls on so short a spine.) Equal radii are the special
//! case `r′ ≡ 0`, built exactly as before.

use std::f64::consts::FRAC_PI_2;

use remus_math::compact_sort::CompactSort;
use remus_math::frame::Frame3;
use remus_math::vec::Point3;
use remus_math::vec::Vec3;

use crate::BlendError;
use crate::section::CircSection;

#[derive(Clone, Copy)]
pub struct CornerFrame {
    origin: Point3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl From<Frame3> for CornerFrame {
    fn from(frame: Frame3) -> Self {
        Self {
            origin: frame.origin,
            x: frame.x,
            y: frame.y,
            z: frame.z,
        }
    }
}

/// Circular rolling sections from the vertical seam to the top shoulder.
///
/// `frame.origin` is on the boss axis in the lower tangent plane of the
/// convex strip. X points along that strip into the part, Y points inward
/// from the block wall and Z points toward the block top. The final section
/// lies on the shoulder torus's meridian; the first lies on the vertical
/// seam cylinder's latitude.
pub fn sections(
    frame: CornerFrame,
    boss_radius: f64,
    radius: f64,
    count: usize,
) -> Result<Vec<CircSection>, BlendError> {
    if !boss_radius.is_finite()
        || boss_radius <= 0.0
        || !radius.is_finite()
        || radius <= 0.0
        || !(16..=2048).contains(&count)
    {
        return Err(BlendError::InvalidInput {
            reason: "invalid boss-corner radii or station count".into(),
        });
    }
    let mut sections = Vec::with_capacity(count);
    for index in 0..count {
        #[allow(clippy::cast_precision_loss)]
        let t = index as f64 / (count - 1) as f64;
        sections.push(section_at(frame, boss_radius, radius, t));
    }
    Ok(sections)
}

/// Whether the patch through `sections` folds: somewhere its normal turns
/// against the side the ball is on (a section point's motion from one
/// station to the next reverses against its neighbours').
///
/// The envelope of a growing ball rolled over a tightly bent spine is not
/// regular: when the round is small against the radius change (an R2 seam
/// under an R4 shoulder over a round below about R1.9), the characteristic
/// circles near the shoulder sweep back over themselves and the envelope
/// develops a cusp. No G1 patch rides that; the corner is refused instead.
fn folds(sections: &[CircSection]) -> bool {
    let arc = |section: &CircSection, u: f64| {
        let (cp0, cp1, cp2, w) = crate::walker::section_arc(section);
        let b = [(1.0 - u) * (1.0 - u), 2.0 * u * (1.0 - u) * w, u * u];
        let sum = b[0] + b[1] + b[2];
        let o = Point3::new(0.0, 0.0, 0.0);
        o + ((cp0 - o) * b[0] + (cp1 - o) * b[1] + (cp2 - o) * b[2]) * (1.0 / sum)
    };
    let mut sign: Option<bool> = None;
    for pair in sections.windows(2) {
        for k in 1..16 {
            let u = f64::from(k) / 16.0;
            let here = arc(&pair[0], u);
            let along = arc(&pair[0], u + 1e-4) - arc(&pair[0], u - 1e-4);
            let next = arc(&pair[1], u) - here;
            let side = along.cross(next).dot(here - pair[0].center);
            if side.abs() <= f64::MIN_POSITIVE {
                continue;
            }
            match sign {
                None => sign = Some(side > 0.0),
                Some(positive) if positive != (side > 0.0) => return true,
                Some(_) => {}
            }
        }
    }
    false
}

/// The three radii meeting at a boss corner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CornerRadii {
    /// The convex round the corner rides over (the call's radius).
    pub convex: f64,
    /// The concave seam up the block wall.
    pub seam: f64,
    /// The concave shoulder round the boss on the block top.
    pub shoulder: f64,
}

impl CornerRadii {
    /// All three the same.
    #[must_use]
    pub const fn equal(radius: f64) -> Self {
        Self {
            convex: radius,
            seam: radius,
            shoulder: radius,
        }
    }

    fn is_equal(self) -> bool {
        self.seam.to_bits() == self.convex.to_bits()
            && self.shoulder.to_bits() == self.convex.to_bits()
    }

    /// Ball radius and its derivative at `t`.
    fn ball(self, t: f64) -> (f64, f64) {
        let rise = self.shoulder - self.seam;
        let smooth = t * t * (-2.0f64).mul_add(t, 3.0);
        (rise.mul_add(smooth, self.seam), rise * 6.0 * t * (1.0 - t))
    }
}

/// Circular sections of the corner for any radii: the characteristic
/// circles of the rolling-ball envelope (see the module documentation).
pub fn sections_with_radii(
    frame: CornerFrame,
    boss_radius: f64,
    radii: CornerRadii,
    count: usize,
) -> Result<Vec<CircSection>, BlendError> {
    if radii.is_equal() {
        return sections(frame, boss_radius, radii.convex, count);
    }
    let valid = |r: f64| r.is_finite() && r > 0.0;
    if !valid(boss_radius)
        || !valid(radii.convex)
        || !valid(radii.seam)
        || !valid(radii.shoulder)
        || !(16..=2048).contains(&count)
    {
        return Err(BlendError::InvalidInput {
            reason: "invalid boss-corner radii or station count".into(),
        });
    }
    let mut sections = Vec::with_capacity(count);
    for index in 0..count {
        #[allow(clippy::cast_precision_loss)]
        let t = index as f64 / (count - 1) as f64;
        sections.push(section_with_radii(frame, boss_radius, radii, t)?);
    }
    Ok(sections)
}

/// The ball at corner station `t`: its centre and radius.
#[cfg(test)]
pub fn ball_at(frame: CornerFrame, boss_radius: f64, radii: CornerRadii, t: f64) -> (Point3, f64) {
    let ([x, y, z], _, r, _) = spine_with_radii(boss_radius, radii, t);
    (frame.origin + frame.x * x + frame.y * y + frame.z * z, r)
}

/// Local (x, y, z) of the ball centre at `t` and its derivative.
fn spine_with_radii(
    boss_radius: f64,
    radii: CornerRadii,
    t: f64,
) -> ([f64; 3], [f64; 3], f64, f64) {
    let (r, dr) = radii.ball(t);
    let angle = (t - 1.0) * FRAC_PI_2;
    let (sin, cos) = angle.sin_cos();
    let reach = radii.convex + r;
    let y = reach.mul_add(sin, radii.convex);
    let z = reach * cos;
    let dy = dr.mul_add(sin, reach * cos * FRAC_PI_2);
    let dz = dr.mul_add(cos, -reach * sin * FRAC_PI_2);
    let offset = boss_radius + r;
    let x = offset.mul_add(offset, -y * y).sqrt();
    let dx = offset.mul_add(dr, -y * dy) / x;
    ([x, y, z], [dx, dy, dz], r, dr)
}

fn section_with_radii(
    frame: CornerFrame,
    boss_radius: f64,
    radii: CornerRadii,
    t: f64,
) -> Result<CircSection, BlendError> {
    let point = |p: [f64; 3]| frame.origin + frame.x * p[0] + frame.y * p[1] + frame.z * p[2];
    let ([x, y, z], d, r, dr) = spine_with_radii(boss_radius, radii, t);
    if !x.is_finite() || x <= 0.0 {
        return Err(invalid("boss corner spine leaves the boss offset"));
    }
    let angle = (t - 1.0) * FRAC_PI_2;
    let (sin, cos) = angle.sin_cos();
    let boss = boss_radius / (boss_radius + r);
    let centre = point([x, y, z]);
    // The characteristic circle: the ball cut by (X − c)·c′ = −r r′. It
    // exists only while the ball grows slower than its centre moves.
    let speed2 = d[0].mul_add(d[0], d[1].mul_add(d[1], d[2] * d[2]));
    let (center, radius) = if dr == 0.0 {
        (centre, r)
    } else {
        let squared = r * r * (1.0 - dr * dr / speed2);
        if squared <= 0.0 || !squared.is_finite() {
            return Err(invalid(
                "boss corner ball radius changes faster than its spine",
            ));
        }
        let tangent = frame.x * d[0] + frame.y * d[1] + frame.z * d[2];
        (centre - tangent * (r * dr / speed2), squared.sqrt())
    };
    Ok(CircSection {
        p1: point([x * boss, y * boss, z]),
        p2: point([
            x,
            radii.convex.mul_add(sin, radii.convex),
            radii.convex * cos,
        ]),
        center,
        radius,
        uv1: (0.0, 0.0),
        uv2: (0.0, 0.0),
        t,
    })
}

fn section_at(frame: CornerFrame, boss_radius: f64, radius: f64, t: f64) -> CircSection {
    let point = |x, y, z| frame.origin + frame.x * x + frame.y * y + frame.z * z;
    let offset = boss_radius + radius;
    let angle = (t - 1.0) * FRAC_PI_2;
    let (sin, cos) = angle.sin_cos();
    let y = radius.mul_add(2.0 * sin, radius);
    let z = 2.0 * radius * cos;
    let x = offset.mul_add(offset, -y * y).sqrt();
    let boss = boss_radius / offset;
    CircSection {
        p1: point(x * boss, y * boss, z),
        p2: point(x, radius.mul_add(sin, radius), radius * cos),
        center: point(x, y, z),
        radius,
        uv1: (0.0, 0.0),
        uv2: (0.0, 0.0),
        t,
    }
}

use remus_math::curves::Circle3D;
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::surfaces::CylindricalSurface;
use remus_topology::Topology;
use remus_topology::adjacency::AdjacencyIndex;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::{face_edges, face_vertices, solid_faces};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, Wire};

use crate::body_edit::EditScope;
use crate::builder_utils::add_certified_curve_edge;
use crate::spine::Spine;
use crate::trimmer::{self, CutPiece};
use crate::{BlendEngine, BlendFaceOrigins, BlendResult};

struct Sources {
    top: FaceId,
    wall: FaceId,
    torus: FaceId,
    vertical: FaceId,
    boss: FaceId,
    end: FaceId,
    line: EdgeId,
    joint: VertexId,
    far: VertexId,
    frame: CornerFrame,
    boss_radius: f64,
    radii: CornerRadii,
    far_x: f64,
    mitre: Vec<EdgeId>,
}

fn invalid(reason: impl Into<String>) -> BlendError {
    BlendError::InvalidInput {
        reason: reason.into(),
    }
}
fn coord(point: Point3, frame: CornerFrame) -> Vec3 {
    let delta = point - frame.origin;
    Vec3::new(delta.dot(frame.x), delta.dot(frame.y), delta.dot(frame.z))
}

fn detect(
    topo: &Topology,
    solid: SolidId,
    adjacency: &AdjacencyIndex,
    spine: &Spine,
    radius: f64,
) -> Result<Option<Sources>, BlendError> {
    let source_faces = solid_faces(topo, solid)?;
    let mut touched = Vec::new();
    for &edge in spine.edges() {
        touched.extend_from_slice(adjacency.faces_for_edge(edge));
    }
    touched.compact_sort_unstable_by_key(|face| face.index());
    touched.dedup();
    // The shoulder and seam bands may have radii of their own (see the
    // module documentation); `radius` is the convex round's.
    let tori: Vec<_> = touched
        .iter()
        .copied()
        .filter(|face| {
            topo.face(*face)
                .is_ok_and(|f| f.is_reversed() && matches!(f.surface(), FaceSurface::Torus(_)))
        })
        .collect();
    let Some(torus) = single_or_equal(topo, &tori, radius)? else {
        return Ok(None);
    };
    let torus = &torus;
    let torus_data = topo.face(*torus)?;
    let FaceSurface::Torus(ts) = torus_data.surface() else {
        return Ok(None);
    };
    let mut found = None;
    for &line in spine.edges() {
        if !matches!(topo.edge(line)?.curve(), EdgeCurve::Line) {
            continue;
        }
        let [a, b] = adjacency.faces_for_edge(line) else {
            continue;
        };
        let (Some(na), Some(nb)) = (
            topo.face(*a)?.effective_plane_normal(),
            topo.face(*b)?.effective_plane_normal(),
        ) else {
            continue;
        };
        let (top, wall, z, y) =
            if na.cross(ts.z_axis()).length() < 1e-10 && na.dot(nb).abs() < 1e-10 {
                (*a, *b, na, -nb)
            } else if nb.cross(ts.z_axis()).length() < 1e-10 && na.dot(nb).abs() < 1e-10 {
                (*b, *a, nb, -na)
            } else {
                continue;
            };
        if found.is_some() {
            return Ok(None);
        }
        found = Some((line, top, wall, z, y));
    }
    let Some((line, top, wall, z, y)) = found else {
        return Ok(None);
    };
    let ts_edges = face_edges(topo, *torus)?;
    let mut verticals = Vec::new();
    for &face in &touched {
        let f = topo.face(face)?;
        if f.is_reversed()
            && matches!(f.surface(),FaceSurface::Cylinder(c) if c.axis().cross(z).length()<1e-10)
            && face_edges(topo, face)?
                .iter()
                .any(|edge| ts_edges.contains(edge))
        {
            verticals.push(face);
        }
    }
    let Some(vertical) = single_or_equal(topo, &verticals, radius)? else {
        return Ok(None);
    };
    let vertical = &vertical;
    let vertical_edges = face_edges(topo, *vertical)?;
    let FaceSurface::Cylinder(vertical_surface) = topo.face(*vertical)?.surface() else {
        return Ok(None);
    };
    let radii = CornerRadii {
        convex: radius,
        seam: vertical_surface.radius(),
        shoulder: ts.minor_radius(),
    };
    // Equal radii, or a seam and shoulder that differ (whatever the round):
    // a round of its own between an equal seam and shoulder rides into the
    // seam band instead (`fillet_into_seam_band`).
    let tolerance = 1e-7;
    let equal =
        (radii.seam - radius).abs() < tolerance && (radii.shoulder - radius).abs() < tolerance;
    if !equal && (radii.seam - radii.shoulder).abs() < tolerance {
        return Ok(None);
    }
    let radii = if equal {
        CornerRadii::equal(radius)
    } else {
        radii
    };
    let boss_radius = ts.major_radius() - radii.shoulder;
    if boss_radius <= 0.0 {
        return Ok(None);
    }
    let bosses:Vec<_>=source_faces.iter().copied().filter(|face|topo.face(*face).is_ok_and(|f|!f.is_reversed()&&matches!(f.surface(),FaceSurface::Cylinder(c) if (c.radius()-boss_radius).abs()<1e-7&&c.axis().cross(z).length()<1e-10))&&face_edges(topo,*face).is_ok_and(|edges|edges.iter().any(|edge|ts_edges.contains(edge))&&edges.iter().any(|edge|vertical_edges.contains(edge)))).collect();
    let [boss] = bosses.as_slice() else {
        return Ok(None);
    };
    let FaceSurface::Cylinder(boss_surface) = topo.face(*boss)?.surface() else {
        return Ok(None);
    };
    if (ts.center() - boss_surface.origin()).cross(z).length() > 1e-7 {
        return Ok(None);
    }
    let on_wall = vertical_surface.origin() - boss_surface.origin() + y * radii.seam;
    let along = on_wall - z * on_wall.dot(z);
    let x = along.normalize()?;
    if x.dot(y).abs() > 1e-10 || x.dot(z).abs() > 1e-10 {
        return Ok(None);
    }
    let edge = topo.edge(line)?;
    let pa = topo.vertex(edge.start())?.point();
    let pb = topo.vertex(edge.end())?.point();
    let h = (pa - boss_surface.origin()).dot(z);
    if (pb - pa).dot(y).abs() > 1e-7 || (pb - pa).dot(z).abs() > 1e-7 {
        return Ok(None);
    }
    let origin = boss_surface.origin() + z * (h - radius);
    let frame = CornerFrame { origin, x, y, z };
    let expected = ((boss_radius + radii.seam).powi(2) - radii.seam * radii.seam).sqrt();
    if (along.length() - expected).abs() > 1e-7
        || ((ts.center() - origin).dot(z) - (radius + radii.shoulder)).abs() > 1e-7
    {
        return Ok(None);
    }
    // Where the patch leaves the top: the round must reach past it.
    let top_reach = ((boss_radius + radii.shoulder).powi(2) - radius * radius).sqrt();
    if !top_reach.is_finite() {
        return Ok(None);
    }
    let (far, far_point) = if coord(pa, frame).x() > coord(pb, frame).x() {
        (edge.start(), pa)
    } else {
        (edge.end(), pb)
    };
    let far_x = coord(far_point, frame).x();
    if far_x <= expected.max(top_reach) + 1e-7 {
        return Ok(None);
    }
    let ends: Vec<_> = source_faces
        .iter()
        .copied()
        .filter(|face| {
            *face != top
                && *face != wall
                && topo.face(*face).is_ok_and(|f| {
                    f.effective_plane_normal()
                        .is_some_and(|n| n.cross(x).length() < 1e-10)
                })
                && face_vertices(topo, *face).is_ok_and(|vertices| vertices.contains(&far))
        })
        .collect();
    let [end] = ends.as_slice() else {
        return Ok(None);
    };
    for face in [top, wall, *torus, *vertical, *boss, *end] {
        if !topo.face(face)?.inner_wires().is_empty() {
            return Ok(None);
        }
    }
    let tv = face_vertices(topo, *torus)?;
    let vv = face_vertices(topo, *vertical)?;
    let joints: Vec<_> = face_vertices(topo, *boss)?
        .into_iter()
        .filter(|vertex| tv.contains(vertex) && vv.contains(vertex))
        .collect();
    let [joint] = joints.as_slice() else {
        return Ok(None);
    };
    let mitre = ts_edges
        .into_iter()
        .filter(|edge| vertical_edges.contains(edge))
        .collect::<Vec<_>>();
    if mitre.is_empty() {
        return Ok(None);
    }
    Ok(Some(Sources {
        top,
        wall,
        torus: *torus,
        vertical: *vertical,
        boss: *boss,
        end: *end,
        line,
        joint: *joint,
        far,
        frame,
        boss_radius,
        radii,
        far_x,
        mitre,
    }))
}

/// The one candidate band, or the one of the call's radius among several.
fn single_or_equal(
    topo: &Topology,
    candidates: &[FaceId],
    radius: f64,
) -> Result<Option<FaceId>, BlendError> {
    if let [only] = candidates {
        return Ok(Some(*only));
    }
    let mut equal = Vec::new();
    for &face in candidates {
        let surface = topo.face(face)?.surface();
        let band = if let FaceSurface::Torus(t) = surface {
            t.minor_radius()
        } else if let FaceSurface::Cylinder(c) = surface {
            c.radius()
        } else {
            continue;
        };
        if (band - radius).abs() < 1e-7 {
            equal.push(face);
        }
    }
    Ok(match equal.as_slice() {
        [only] => Some(*only),
        _ => None,
    })
}

fn contact_curve(points: Vec<Point3>) -> Result<NurbsCurve, BlendError> {
    let n = points.len();
    let mut knots = vec![0.0, 0.0];
    for i in 1..n - 1 {
        knots.push(i as f64 / (n - 1) as f64);
    }
    knots.extend([1.0, 1.0]);
    Ok(NurbsCurve::new(1, knots, points, vec![1.0; n])?)
}
fn minor_arc(
    topo: &mut Topology,
    a: VertexId,
    b: VertexId,
    center: Point3,
    radius: f64,
) -> Result<EdgeId, BlendError> {
    let va = topo.vertex(a)?.point() - center;
    let vb = topo.vertex(b)?.point() - center;
    let axis = va.cross(vb).normalize()?;
    let circle = Circle3D::new_with_ref(center, axis, radius, va)?;
    let angle = circle
        .project(topo.vertex(b)?.point())
        .rem_euclid(std::f64::consts::TAU);
    if angle <= 0.0 || angle >= std::f64::consts::PI {
        return Err(invalid("boss corner requires minor end arcs"));
    }
    add_certified_curve_edge(topo, a, b, EdgeCurve::Circle(circle), (0.0, angle))
}
fn current(map: &remus_math::det_hash::DetHashMap<FaceId, FaceId>, face: FaceId) -> FaceId {
    map.get(&face).copied().unwrap_or(face)
}
fn cut(
    topo: &mut Topology,
    map: &mut remus_math::det_hash::DetHashMap<FaceId, FaceId>,
    face: FaceId,
    a: VertexId,
    b: VertexId,
    edge: EdgeId,
    avoid: &[EdgeId],
) -> Result<(), BlendError> {
    let live = current(map, face);
    let wire = topo.wire(topo.face(live)?.outer_wire())?.edges().to_vec();
    let index = |vertex| {
        wire.iter()
            .position(|oe| {
                topo.edge(oe.edge())
                    .is_ok_and(|e| oe.oriented_start(e) == vertex)
            })
            .ok_or_else(|| invalid("boss-corner cut root missing from source wire"))
    };
    let (from, to) = (index(a)?, index(b)?);
    let image = trimmer::cut_face(
        topo,
        live,
        &wire,
        &[CutPiece {
            from,
            to,
            edges: vec![OrientedEdge::new(edge, true)],
        }],
        avoid,
    )?;
    map.insert(face, image);
    Ok(())
}
fn incident_edges(
    topo: &Topology,
    face: FaceId,
    vertex: VertexId,
) -> Result<Vec<EdgeId>, BlendError> {
    Ok(topo
        .wire(topo.face(face)?.outer_wire())?
        .edges()
        .iter()
        .filter_map(|oe| {
            topo.edge(oe.edge())
                .ok()
                .filter(|e| e.start() == vertex || e.end() == vertex)
                .map(|_| oe.edge())
        })
        .collect())
}
fn anchored_face(
    topo: &mut Topology,
    mut edges: Vec<OrientedEdge>,
    surface: FaceSurface,
    desired: Vec3,
    probe: Point3,
    anchor: EdgeId,
    neighbour: FaceId,
) -> Result<FaceId, BlendError> {
    let uv = surface
        .project_point(probe)
        .ok_or_else(|| invalid("new boss-corner face has no chart"))?;
    let reversed = surface.normal(uv.0, uv.1).dot(desired) < 0.0;
    let adjacent = topo.face(neighbour)?;
    let uses: Vec<_> = topo
        .wire(adjacent.outer_wire())?
        .edges()
        .iter()
        .filter(|oe| oe.edge() == anchor)
        .collect();
    let [use_] = uses.as_slice() else {
        return Err(invalid(
            "new boss-corner anchor is not a unique boundary use",
        ));
    };
    let opposite = use_.is_forward() == adjacent.is_reversed();
    let own = edges
        .iter()
        .find(|oe| oe.edge() == anchor)
        .ok_or_else(|| invalid("new boss-corner anchor is absent"))?;
    if (own.is_forward() != reversed) != opposite {
        edges = edges
            .into_iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
    }
    let wire = topo.add_wire(Wire::new(edges, true)?);
    let mut face = Face::new(wire, vec![], surface);
    face.set_reversed(reversed);
    Ok(topo.add_face(face))
}

/// Equivalent support-first construction for an equal-radius mixed ridge.
/// `None` leaves all other ridge families to their existing builder.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
pub fn blend(
    topo: &mut Topology,
    solid: SolidId,
    adjacency: &AdjacencyIndex,
    spine: &Spine,
    radius: f64,
) -> Result<Option<BlendResult>, BlendError> {
    if !topo.solid(solid)?.inner_shells().is_empty() {
        return Ok(None);
    }
    let Some(source) = detect(topo, solid, adjacency, spine, radius)? else {
        return Ok(None);
    };
    log::debug!(
        "{} boss corner qualified: seam={} shoulder={} convex={} top={} wall={} torus={} vertical={} boss={} end={}",
        if source.radii.is_equal() {
            "equal radius"
        } else {
            "unequal radius"
        },
        source.radii.seam,
        source.radii.shoulder,
        source.radii.convex,
        source.top.index(),
        source.wall.index(),
        source.torus.index(),
        source.vertical.index(),
        source.boss.index(),
        source.end.index()
    );
    let original = solid_faces(topo, solid)?;
    let seam = crossed_seam(topo, adjacency, &source, radius)?;
    let mut samples = sections_with_radii(source.frame, source.boss_radius, source.radii, 513)?;
    if !source.radii.is_equal() && folds(&samples) {
        return Err(invalid(format!(
            "boss corner envelope folds: a ball growing from R{} to R{} cannot ride an R{} round",
            source.radii.seam, source.radii.shoulder, source.radii.convex
        )));
    }
    let seam_station = if let Some((_, _, parameter)) = seam {
        samples.push(if source.radii.is_equal() {
            section_at(source.frame, source.boss_radius, radius, parameter)
        } else {
            section_with_radii(source.frame, source.boss_radius, source.radii, parameter)?
        });
        samples.compact_sort_by(|a, b| a.t.total_cmp(&b.t));
        samples.dedup_by(|a, b| (a.t - b.t).abs() < 1e-12);
        Some(
            samples
                .iter()
                .position(|section| (section.t - parameter).abs() < 1e-12)
                .ok_or_else(|| invalid("boss seam station missing"))?,
        )
    } else {
        None
    };
    let (first, last) = (&samples[0], &samples[samples.len() - 1]);
    let frame = source.frame;
    let ew = frame.origin + frame.x * source.far_x;
    let et = ew + frame.y * radius + frame.z * radius;
    let mut scope = EditScope::new(topo, original.iter().copied())?;
    let mut map = remus_math::det_hash::DetHashMap::default();
    let mut nodes = Vec::new();
    for (face, point) in [
        (source.wall, first.p2),
        (source.top, last.p2),
        (source.boss, first.p1),
        (source.boss, last.p1),
        (source.end, ew),
        (source.end, et),
    ] {
        let live = current(&map, face);
        let mut edit = scope.edit(&mut map);
        nodes.push(
            trimmer::split_body_boundary_at(topo, &mut edit, live, point, 1e-7)?.ok_or_else(
                || {
                    invalid(format!(
                        "boss-corner root could not be split on face {} at {:?}",
                        face.index(),
                        point
                    ))
                },
            )?,
        );
    }
    let [a, b, c, d, ew, et]: [VertexId; 6] = nodes
        .try_into()
        .map_err(|_| invalid("boss-corner node count"))?;
    let bottom = minor_arc(topo, c, a, first.center, first.radius)?;
    let top = minor_arc(topo, d, b, last.center, last.radius)?;
    let far_center = frame.origin + frame.x * source.far_x + frame.y * radius;
    let end = minor_arc(topo, ew, et, far_center, radius)?;
    let wall = topo.add_edge(Edge::new(a, ew, EdgeCurve::Line));
    let flat = topo.add_edge(Edge::new(b, et, EdgeCurve::Line));
    let boss_curve = contact_curve(samples.iter().map(|section| section.p1).collect())?;
    let (boss_edges, seam_vertex) = if let Some(index) = seam_station {
        let point = samples[index].p1;
        let vertex = topo.add_vertex(remus_topology::vertex::Vertex::new(point, 1e-7));
        let parameter = index as f64 / (samples.len() - 1) as f64;
        let before = add_certified_curve_edge(
            topo,
            c,
            vertex,
            EdgeCurve::NurbsCurve(boss_curve.clone()),
            (0.0, parameter),
        )?;
        let after = add_certified_curve_edge(
            topo,
            vertex,
            d,
            EdgeCurve::NurbsCurve(boss_curve),
            (parameter, 1.0),
        )?;
        (vec![before, after], Some(vertex))
    } else {
        (
            vec![add_certified_curve_edge(
                topo,
                c,
                d,
                EdgeCurve::NurbsCurve(boss_curve),
                (0.0, 1.0),
            )?],
            None,
        )
    };
    let strip = add_certified_curve_edge(
        topo,
        a,
        b,
        EdgeCurve::NurbsCurve(contact_curve(samples.iter().map(|s| s.p2).collect())?),
        (0.0, 1.0),
    )?;
    cut(topo, &mut map, source.wall, a, ew, wall, &[source.line])?;
    cut(topo, &mut map, source.top, b, et, flat, &[source.line])?;
    let avoid = incident_edges(topo, current(&map, source.end), source.far)?;
    cut(topo, &mut map, source.end, ew, et, end, &avoid)?;
    if let (Some((seam, lower, _)), Some(vertex)) = (seam, seam_vertex) {
        replace_boss_boundary_with_seam(
            topo,
            &mut map,
            &source,
            c,
            d,
            &boss_edges,
            seam,
            lower,
            vertex,
        )?;
    } else {
        let avoid = incident_edges(topo, current(&map, source.boss), source.joint)?;
        cut(topo, &mut map, source.boss, c, d, boss_edges[0], &avoid)?;
    }
    cut(topo, &mut map, source.vertical, c, a, bottom, &source.mitre)?;
    cut(topo, &mut map, source.torus, d, b, top, &source.mitre)?;
    let convex = FaceSurface::Cylinder(CylindricalSurface::new(
        frame.origin + frame.y * radius,
        frame.x,
        radius,
    )?);
    let desired = (-frame.y + frame.z).normalize()?;
    let probe = far_center - frame.x * ((source.far_x - coord(first.p2, frame).x()) * 0.5)
        + desired * radius;
    let convex = anchored_face(
        topo,
        vec![
            OrientedEdge::new(wall, true),
            OrientedEdge::new(end, true),
            OrientedEdge::new(flat, false),
            OrientedEdge::new(strip, false),
        ],
        convex,
        desired,
        probe,
        wall,
        current(&map, source.wall),
    )?;
    let surface = FaceSurface::Nurbs(crate::walker::approximate_blend_surface_linear_v(&samples)?);
    let mid = &samples[samples.len() / 2];
    let probe = surface
        .evaluate(0.5, 0.5)
        .ok_or_else(|| invalid("boss corner midpoint missing"))?;
    let desired = (mid.center
        - Point3::new(
            (mid.p1.x() + mid.p2.x()) * 0.5,
            (mid.p1.y() + mid.p2.y()) * 0.5,
            (mid.p1.z() + mid.p2.z()) * 0.5,
        ))
    .normalize()?;
    let corner = anchored_face(
        topo,
        boss_edges
            .iter()
            .map(|&edge| OrientedEdge::new(edge, true))
            .chain([
                OrientedEdge::new(top, true),
                OrientedEdge::new(strip, false),
                OrientedEdge::new(bottom, false),
            ])
            .collect(),
        surface,
        desired,
        probe,
        boss_edges[0],
        current(&map, source.boss),
    )?;
    let faces = original
        .iter()
        .map(|&face| current(&map, face))
        .chain([convex, corner])
        .collect();
    let shell = topo.add_shell(Shell::new(faces)?);
    let result = topo.add_solid(Solid::new(shell, vec![]));
    log::debug!(target:"remus_approx","fillet: equal-radius boss corner uses {} closed-form rolling sections and a sampled NURBS band",samples.len());
    Ok(Some(BlendResult {
        solid: result,
        succeeded: spine.edges().to_vec(),
        failed: vec![],
        is_partial: false,
        engine: BlendEngine::Walking,
        face_origins: Some(BlendFaceOrigins {
            survived: original
                .iter()
                .map(|&face| (face, current(&map, face)))
                .collect(),
            deleted: vec![],
            created: vec![
                (convex, vec![source.top, source.wall]),
                (
                    corner,
                    vec![
                        source.top,
                        source.wall,
                        source.boss,
                        source.torus,
                        source.vertical,
                    ],
                ),
            ],
            created_unattributed: vec![],
        }),
    }))
}

fn crossed_seam(
    topo: &Topology,
    adjacency: &AdjacencyIndex,
    source: &Sources,
    radius: f64,
) -> Result<Option<(EdgeId, VertexId, f64)>, BlendError> {
    let mut found = None;
    let mut seen = std::collections::BTreeSet::new();
    for edge in face_edges(topo, source.boss)? {
        if !seen.insert(edge) {
            continue;
        }
        let faces = adjacency.faces_for_edge(edge);
        if faces.len() != 2 || faces[0] != source.boss || faces[1] != source.boss {
            continue;
        }
        let data = topo.edge(edge)?;
        if !matches!(data.curve(), EdgeCurve::Line) {
            return Err(invalid("boss seam is not an axial ruling"));
        }
        let pa = topo.vertex(data.start())?.point();
        let pb = topo.vertex(data.end())?.point();
        if (pb - pa).cross(source.frame.z).length() > 1e-7 {
            return Err(invalid("boss seam is not parallel to its axis"));
        }
        let local = coord(pa, source.frame);
        if local.x() <= 0.0 {
            continue;
        }
        let t = if source.radii.is_equal() {
            let cy = local.y() * (source.boss_radius + radius) / source.boss_radius;
            if cy <= -radius + 1e-8 || cy >= radius - 1e-8 {
                continue;
            }
            (((cy - radius) / (2.0 * radius)).asin() + FRAC_PI_2) / FRAC_PI_2
        } else {
            let Some(t) = seam_station(source, local.y().atan2(local.x())) else {
                continue;
            };
            t
        };
        let lower = if coord(pa, source.frame).z() < coord(pb, source.frame).z() {
            data.start()
        } else {
            data.end()
        };
        if found.is_some() {
            return Err(invalid("multiple boss seams cross the corner"));
        }
        found = Some((edge, lower, t));
    }
    Ok(found)
}
/// The corner station whose boss contact lies at angle `target` about the
/// boss axis (from the frame's X towards its Y), strictly inside the corner;
/// the contact's angle is the centre's, and rises monotonically.
fn seam_station(source: &Sources, target: f64) -> Option<f64> {
    let angle = |t: f64| {
        let ([x, y, _], _, _, _) = spine_with_radii(source.boss_radius, source.radii, t);
        y.atan2(x)
    };
    let (lo, hi) = (angle(0.0), angle(1.0));
    let margin = 1e-8 / source.boss_radius;
    if !(lo + margin..=hi - margin).contains(&target) {
        return None;
    }
    let (mut a, mut b) = (0.0f64, 1.0f64);
    for _ in 0..200 {
        let m = 0.5 * (a + b);
        if angle(m) < target {
            a = m;
        } else {
            b = m;
        }
        if b - a < 1e-15 {
            break;
        }
    }
    Some(0.5 * (a + b))
}

fn boundary_arc(wire: &[OrientedEdge], a: usize, b: usize) -> Vec<OrientedEdge> {
    let mut arc = Vec::new();
    let mut pos = a;
    while pos != b {
        arc.push(wire[pos]);
        pos = (pos + 1) % wire.len();
    }
    arc
}
#[allow(clippy::too_many_arguments)]
fn replace_boss_boundary_with_seam(
    topo: &mut Topology,
    map: &mut remus_math::det_hash::DetHashMap<FaceId, FaceId>,
    source: &Sources,
    c: VertexId,
    d: VertexId,
    contacts: &[EdgeId],
    seam: EdgeId,
    lower: VertexId,
    vertex: VertexId,
) -> Result<(), BlendError> {
    let face_id = current(map, source.boss);
    let face = topo.face(face_id)?;
    let surface = face.surface().clone();
    let reversed = face.is_reversed();
    let inner = face.inner_wires().to_vec();
    let wire = topo.wire(face.outer_wire())?.edges().to_vec();
    let position = |vertex| {
        wire.iter()
            .position(|oe| {
                topo.edge(oe.edge())
                    .is_ok_and(|edge| oe.oriented_start(edge) == vertex)
            })
            .ok_or_else(|| invalid("boss contact root missing from periodic wire"))
    };
    let (pc, pd) = (position(c)?, position(d)?);
    let cd = boundary_arc(&wire, pc, pd);
    let dc = boundary_arc(&wire, pd, pc);
    let has_joint = |arc: &[OrientedEdge]| {
        arc.iter().any(|oe| {
            topo.edge(oe.edge())
                .is_ok_and(|edge| edge.start() == source.joint || edge.end() == source.joint)
        })
    };
    let (mut kept, consumed, forward) = match (has_joint(&cd), has_joint(&dc)) {
        (true, false) => (dc, cd, true),
        (false, true) => (cd, dc, false),
        _ => return Err(invalid("boss corner does not isolate its old joint")),
    };
    let occurrences: Vec<_> = consumed
        .iter()
        .enumerate()
        .filter_map(|(index, oe)| (oe.edge() == seam).then_some(index))
        .collect();
    let [first, last] = occurrences.as_slice() else {
        return Err(invalid("boss cap detour does not contain both seam uses"));
    };
    let old = topo.edge(seam)?;
    let ruling = if old.start() == lower {
        topo.add_edge(Edge::new(vertex, old.end(), EdgeCurve::Line))
    } else {
        topo.add_edge(Edge::new(old.start(), vertex, EdgeCurve::Line))
    };
    let detour = consumed[*first..=*last]
        .iter()
        .map(|oe| {
            OrientedEdge::new(
                if oe.edge() == seam { ruling } else { oe.edge() },
                oe.is_forward(),
            )
        })
        .collect::<Vec<_>>();
    if contacts.len() != 2 {
        return Err(invalid("cross-seam boss contour must have two branches"));
    }
    if forward {
        kept.push(OrientedEdge::new(contacts[0], true));
        kept.extend(detour);
        kept.push(OrientedEdge::new(contacts[1], true));
    } else {
        kept.push(OrientedEdge::new(contacts[1], false));
        kept.extend(detour);
        kept.push(OrientedEdge::new(contacts[0], false));
    }
    let wire = topo.add_wire(Wire::new(kept, true)?);
    let mut face = Face::new(wire, inner, surface);
    face.set_reversed(reversed);
    map.insert(source.boss, topo.add_face(face));
    Ok(())
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;
    use remus_math::vec::Vec3;

    fn local(frame: Frame3, p: Point3) -> Vec3 {
        let d = p - frame.origin;
        Vec3::new(d.dot(frame.x), d.dot(frame.y), d.dot(frame.z))
    }

    const UNEQUAL: [(f64, CornerRadii); 4] = [
        (
            6.0,
            CornerRadii {
                convex: 4.0,
                seam: 2.0,
                shoulder: 4.0,
            },
        ),
        (
            6.0,
            CornerRadii {
                convex: 4.0,
                seam: 2.6,
                shoulder: 4.0,
            },
        ),
        (
            6.0,
            CornerRadii {
                convex: 1.9,
                seam: 2.0,
                shoulder: 4.0,
            },
        ),
        (
            0.006,
            CornerRadii {
                convex: 0.004,
                seam: 0.005,
                shoulder: 0.002,
            },
        ),
    ];

    /// Every ball centre is on both offsets at its own radius, both
    /// contacts are on their supports and on the section, each section is
    /// the ball's characteristic circle (on the ball, its plane normal to
    /// the spine), and the end sections are the seam cylinder's latitude and
    /// the shoulder torus's meridian.
    #[test]
    fn unequal_sections_are_characteristic_circles_on_both_offsets() {
        let frame =
            Frame3::from_normal(Point3::new(3.0, -7.0, 2.0), Vec3::new(1.0, 2.0, 3.0)).unwrap();
        for (boss_radius, radii) in UNEQUAL {
            let sections = sections_with_radii(frame.into(), boss_radius, radii, 1025).unwrap();
            let scale = radii.convex.max(radii.seam).max(radii.shoulder);
            let tolerance = scale * 1e-10;
            for section in &sections {
                let ([x, y, z], d, r, _) = spine_with_radii(boss_radius, radii, section.t);
                let ball = frame.origin + frame.x * x + frame.y * y + frame.z * z;
                let tangent = (frame.x * d[0] + frame.y * d[1] + frame.z * d[2])
                    .normalize()
                    .unwrap();
                let p = local(frame, section.p1);
                let q = local(frame, section.p2);
                assert!((x.hypot(y) - (boss_radius + r)).abs() < tolerance);
                assert!(((y - radii.convex).hypot(z) - (radii.convex + r)).abs() < tolerance);
                assert!((p.x().hypot(p.y()) - boss_radius).abs() < tolerance);
                assert!(((q.y() - radii.convex).hypot(q.z()) - radii.convex).abs() < tolerance);
                for contact in [section.p1, section.p2] {
                    assert!(
                        ((contact - section.center).length() - section.radius).abs() < tolerance
                    );
                    assert!(((contact - ball).length() - r).abs() < tolerance);
                }
                assert!((section.center - ball).cross(tangent).length() < tolerance);
                let normal = (section.p1 - section.center)
                    .cross(section.p2 - section.center)
                    .normalize()
                    .unwrap();
                assert!(
                    normal.cross(tangent).length() < 1e-9,
                    "{normal:?} {tangent:?}"
                );
            }
            let (first, last) = (&sections[0], &sections[sections.len() - 1]);
            let c = local(frame, first.center);
            assert!((first.radius - radii.seam).abs() < tolerance);
            assert!((c.y() + radii.seam).abs() < tolerance && c.z().abs() < tolerance);
            let c = local(frame, last.center);
            assert!((last.radius - radii.shoulder).abs() < tolerance);
            assert!((c.y() - radii.convex).abs() < tolerance);
            assert!((c.z() - (radii.convex + radii.shoulder)).abs() < tolerance);
            assert!((c.x().hypot(c.y()) - (boss_radius + radii.shoulder)).abs() < tolerance);
        }
    }

    /// The sampled patch through 513 sections stays on the exact sections
    /// between them: midway between two stations it is within `2e-5` of the
    /// radius scale of the exact section there.
    #[test]
    fn unequal_patch_stays_on_the_envelope_between_sections() {
        let frame =
            Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        for (boss_radius, radii) in UNEQUAL {
            let sections = sections_with_radii(frame.into(), boss_radius, radii, 513).unwrap();
            let surface = crate::walker::approximate_blend_surface_linear_v(&sections).unwrap();
            let scale = radii.convex.max(radii.seam).max(radii.shoulder);
            let mut worst = 0.0f64;
            for i in 0..512 {
                let v = (f64::from(i) + 0.5) / 512.0;
                let exact = section_with_radii(frame.into(), boss_radius, radii, v).unwrap();
                let axis = (exact.p1 - exact.center)
                    .cross(exact.p2 - exact.center)
                    .normalize()
                    .unwrap();
                for k in 0..=8 {
                    let point = surface.evaluate(f64::from(k) / 8.0, v);
                    let d = point - exact.center;
                    let height = d.dot(axis);
                    let planar = (d - axis * height).length();
                    worst = worst.max((planar - exact.radius).hypot(height));
                }
            }
            assert!(worst < 2e-5 * scale, "{radii:?}: {worst}");
        }
    }

    /// A growing ball over a small round folds the envelope; the corner
    /// is refused there, and only there.
    #[test]
    fn folding_envelopes_are_detected() {
        let frame =
            Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        for (seam, convex, fold) in [
            (2.0, 4.0, false),
            (2.0, 1.9, false),
            (2.6, 1.6, false),
            (2.6, 4.0, false),
            (6.0, 1.0, false),
            (2.0, 1.6, true),
            (1.0, 1.0, true),
            (2.6, 1.0, true),
        ] {
            let radii = CornerRadii {
                convex,
                seam,
                shoulder: 4.0,
            };
            let sections = sections_with_radii(frame.into(), 6.0, radii, 513).unwrap();
            assert_eq!(folds(&sections), fold, "{radii:?}");
        }
        for radius in [1.0, 4.0] {
            let sections = sections(frame.into(), 6.0, radius, 513).unwrap();
            assert!(!folds(&sections), "equal R{radius}");
        }
    }

    /// Equal radii take the original closed form bit for bit.
    #[test]
    fn equal_radii_keep_the_equal_sections() {
        let frame =
            Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        let a = sections(frame.into(), 6.0, 4.0, 257).unwrap();
        let b = sections_with_radii(frame.into(), 6.0, CornerRadii::equal(4.0), 257).unwrap();
        for (a, b) in a.iter().zip(&b) {
            assert_eq!((a.p1, a.p2, a.center), (b.p1, b.p2, b.center));
            assert_eq!(a.radius.to_bits(), b.radius.to_bits());
        }
    }

    #[test]
    fn centres_and_contacts_obey_both_analytic_offsets() {
        let frame =
            Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).unwrap();
        for (boss_radius, radius) in [(6.0, 5.0), (3.0, 0.4), (0.006, 0.005), (6000.0, 5000.0)] {
            let sections = sections(frame.into(), boss_radius, radius, 257).unwrap();
            let tolerance = radius * 1e-10;
            for section in &sections {
                let local = |p: Point3| {
                    let d = p - frame.origin;
                    Vec3::new(d.dot(frame.x), d.dot(frame.y), d.dot(frame.z))
                };
                let c = local(section.center);
                let p = local(section.p1);
                let q = local(section.p2);
                assert!((c.x().hypot(c.y()) - (boss_radius + radius)).abs() < tolerance);
                assert!(((c.y() - radius).hypot(c.z()) - 2.0 * radius).abs() < tolerance);
                assert!((p.x().hypot(p.y()) - boss_radius).abs() < tolerance);
                assert!(((q.y() - radius).hypot(q.z()) - radius).abs() < tolerance);
                assert!(((section.p1 - section.center).length() - radius).abs() < tolerance);
                assert!(((section.p2 - section.center).length() - radius).abs() < tolerance);
            }
            let first = &sections[0];
            let last = &sections[sections.len() - 1];
            assert!(((first.p1 - frame.origin).dot(frame.z)).abs() < tolerance);
            assert!(((first.p2 - frame.origin).dot(frame.z)).abs() < tolerance);
            assert!(((last.p1 - frame.origin).dot(frame.z) - 2.0 * radius).abs() < tolerance);
            assert!(((last.p2 - frame.origin).dot(frame.z) - radius).abs() < tolerance);
        }
    }

    #[test]
    fn each_contact_is_normal_to_the_rolling_spine() {
        let frame =
            Frame3::from_normal(Point3::new(3.0, -7.0, 2.0), Vec3::new(1.0, 2.0, 3.0)).unwrap();
        let sections = sections(frame.into(), 6.0, 5.0, 1025).unwrap();
        for pair in sections.windows(3) {
            let tangent = (pair[2].center - pair[0].center).normalize().unwrap();
            for point in [pair[1].p1, pair[1].p2] {
                assert!(
                    tangent
                        .dot((point - pair[1].center).normalize().unwrap())
                        .abs()
                        < 1e-5
                );
            }
        }
    }
}
