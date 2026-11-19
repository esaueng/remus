//! Trim membership of a point on a sphere face whose trim holds a pole of the
//! sphere's own chart or winds its axis, and of a point in a spherical cap's
//! holes (B264).
//!
//! The `(longitude, latitude)` chart reads a sphere trim only when every loop
//! stays clear of both poles and closes in longitude. A piece that holds a
//! pole (the cap a sphere–torus fuse keeps above the tube) has a loop around
//! the axis, which bounds no polygon in that chart. The chorded 3D polygon,
//! projected onto the coordinate plane nearest its Newell normal, stood in
//! for it (and a cap's holes were projected the same way), and that
//! projection does not bound a strongly curved patch: a hit on the far side
//! of the cap tested inside the loop's shadow.
//!
//! These trims are read in a stereographic chart instead: the projection of
//! the sphere from a unit direction `N` chosen well clear of the loops. It is
//! a smooth, orientation-preserving chart of the whole sphere but `N`, so
//! every loop maps to a closed curve in the plane.
//!
//! Each loop is sampled along its own edges, and every chord between two
//! samples carries a band the true edge stays within. Stereographic
//! projection maps circles to circles, so a circle edge's band is the exact
//! sagitta of its image arc through the chord's ends and the image of the
//! parameter midpoint; a spline edge's band is twice the largest distance of
//! interior samples from the chord (measured, not certified). A point
//! farther than its band from every chord has the winding the true loops
//! give it; a point inside some band is undecided and the caller abstains.
//!
//! Which side of the loops is the face follows the stored winding the
//! assembly restores on every curved face (`orient_revolved_face_wires`): the
//! face lies left of its loops looking down the stored outward sphere
//! normal, whatever the face's reversal flag. In the plane the boundary of a
//! bounded region then has total winding `1` about its interior points and a
//! positive signed area; the boundary of the region holding `N` (which maps
//! to infinity) has winding `0` about its interior points, `-1` outside, and
//! a negative area. So the loops' total signed area says whether `N` is in
//! the face, and the winding number about a hit's image decides the hit.
//! A cap's holes need no winding convention: they are read against the
//! sphere point farthest off the cap, which lies in no hole.

use remus_math::predicates::winding_number_xy;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point2, Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::FaceId;
use remus_topology::wire::WireId;

use crate::CheckError;

/// Least distance, on the unit sphere, from the projection pole to every
/// sample of every loop. Within it the images of loop samples grow like the
/// inverse square of the distance; at this clearance they stay below `2e4`
/// (the chart is of the unit sphere).
const MIN_POLE_CLEARANCE: f64 = 1e-2;

/// A total signed area this small against the loops' own areas leaves the
/// side of the projection pole undecided.
const AREA_REL_EPS: f64 = 1e-9;

/// Hits this close to the projection pole (`1 - q . N`) take the pole's own
/// side: the pole is at least [`MIN_POLE_CLEARANCE`] from the boundary.
const AT_POLE: f64 = 1e-12;

/// Samples of a circle edge per full turn.
const CIRCLE_SAMPLES_PER_TURN: f64 = 256.0;

/// Samples of a spline edge, and interior samples per chord measuring its
/// band.
const SPLINE_SAMPLES: usize = 64;
const SPLINE_INTERIOR: usize = 4;

/// A spline chord's band is this multiple of its measured deviation.
const SPLINE_BAND_FACTOR: f64 = 2.0;

/// Relative rounding slack added to every band, against the chart scale of
/// the chord's ends.
const BAND_SLACK: f64 = 1e-9;

/// One loop sampled along its edges: unit directions about the centre, and
/// for the chord from sample `k` to sample `k + 1` (cyclically) the unit
/// directions of interior points of the edge between them.
#[derive(Debug, Clone)]
struct SampledLoop {
    points: Vec<Vec3>,
    interiors: Vec<Vec<Vec3>>,
    /// Whether the chord's edge is a circle, whose image is a circle.
    circular: Vec<bool>,
}

impl SampledLoop {
    /// Every sample and interior point, for clearance tests.
    fn all_points(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.points
            .iter()
            .copied()
            .chain(self.interiors.iter().flatten().copied())
    }
}

/// How a wire reads: as a sampled loop, as no loop at all (a degenerate
/// wire with no extent), or not at all.
enum WireReading {
    Loop(SampledLoop),
    NoExtent,
    Unsupported,
}

/// How one edge is walked: `(evaluate, first parameter, signed span,
/// samples, is a circle)`.
type EdgeWalk = (Box<dyn Fn(f64) -> Point3>, f64, f64, usize, bool);

/// Sample a wire of a sphere face about `center`, edge by edge in traversal
/// order (chaining edges as `util::wire_polygon` does).
#[allow(clippy::too_many_lines, clippy::cast_precision_loss)]
fn sample_wire(
    topo: &Topology,
    wire_id: WireId,
    center: Point3,
) -> Result<WireReading, CheckError> {
    let wire = topo.wire(wire_id)?;
    let mut points: Vec<Point3> = Vec::new();
    let mut interiors: Vec<Vec<Point3>> = Vec::new();
    let mut circular: Vec<bool> = Vec::new();
    let mut prev_end = None;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        let (start, end) = (edge.start(), edge.end());
        let start_pt = topo.vertex(start)?.point();
        let end_pt = topo.vertex(end)?.point();
        let forward = match prev_end {
            Some(previous) if start == previous && end != previous => true,
            Some(previous) if end == previous && start != previous => false,
            _ if start == end => oriented.is_forward(),
            Some(previous) => {
                let last = topo.vertex(previous)?.point();
                (start_pt - last).length_squared() <= (end_pt - last).length_squared()
            }
            None => oriented.is_forward(),
        };
        let Ok((t0, t1)) = edge.strict_domain() else {
            return Ok(WireReading::Unsupported);
        };
        let closed = start == end;
        let traversal_start = if forward { start_pt } else { end_pt };
        prev_end = Some(if closed || forward { end } else { start });
        let (evaluate, first, span, count, is_circle): EdgeWalk = match edge.curve() {
            EdgeCurve::Circle(circle) => {
                let circle = circle.clone();
                let span = t1 - t0;
                let count = (CIRCLE_SAMPLES_PER_TURN * span.abs() / std::f64::consts::TAU)
                    .ceil()
                    .max(4.0) as usize;
                let first = if closed { circle.project(start_pt) } else { t0 };
                (
                    Box::new(move |t| circle.evaluate(t)),
                    first,
                    span,
                    count,
                    true,
                )
            }
            EdgeCurve::NurbsCurve(curve) => {
                let (u0, u1) = curve.domain();
                if closed {
                    let span = u1 - u0;
                    if !(span.is_finite() && span > 0.0) {
                        return Ok(WireReading::Unsupported);
                    }
                    let seam = crate::util::nurbs_seam_parameter(curve, start_pt, u0, u1);
                    let curve = curve.clone();
                    (
                        Box::new(move |t: f64| curve.evaluate(u0 + (t - u0).rem_euclid(span))),
                        seam,
                        span,
                        SPLINE_SAMPLES,
                        false,
                    )
                } else {
                    let curve = curve.clone();
                    (
                        Box::new(move |t| curve.evaluate(t)),
                        t0,
                        t1 - t0,
                        SPLINE_SAMPLES,
                        false,
                    )
                }
            }
            // A degenerate edge (a collapsed pole edge) bounds nothing.
            EdgeCurve::Line if (end_pt - start_pt).length_squared() < super::COINCIDENT_SQ => {
                continue;
            }
            EdgeCurve::Line
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => return Ok(WireReading::Unsupported),
        };
        let at = |k: f64| span.mul_add(k / count as f64, first);
        // Walk the edge as the wire traverses it: a closed edge by its use's
        // direction, an open one positionally from the end the walk enters.
        let reversed = if closed {
            !forward
        } else {
            (evaluate(at(0.0)) - traversal_start).length_squared()
                > (evaluate(at(count as f64)) - traversal_start).length_squared()
        };
        let param = |k: f64| {
            if reversed {
                at(count as f64 - k)
            } else {
                at(k)
            }
        };
        let interior_count = if is_circle { 1 } else { SPLINE_INTERIOR };
        for k in 0..count {
            let base = k as f64;
            points.push(evaluate(param(base)));
            interiors.push(
                (1..=interior_count)
                    .map(|i| evaluate(param(base + i as f64 / (interior_count + 1) as f64)))
                    .collect(),
            );
            circular.push(is_circle);
        }
    }
    let Some(&first) = points.first() else {
        return Ok(WireReading::NoExtent);
    };
    if points.len() < 3
        || points
            .iter()
            .all(|p| (*p - first).length_squared() < super::COINCIDENT_SQ)
    {
        return Ok(WireReading::NoExtent);
    }
    let unit = |p: Point3| (p - center).normalize().ok();
    let Some(points) = points.into_iter().map(unit).collect::<Option<Vec<_>>>() else {
        return Ok(WireReading::Unsupported);
    };
    let Some(interiors) = interiors
        .into_iter()
        .map(|chord| chord.into_iter().map(unit).collect::<Option<Vec<_>>>())
        .collect::<Option<Vec<_>>>()
    else {
        return Ok(WireReading::Unsupported);
    };
    Ok(WireReading::Loop(SampledLoop {
        points,
        interiors,
        circular,
    }))
}

/// The stereographic projection of a sphere from one of its points.
#[derive(Debug, Clone)]
struct Chart {
    center: Point3,
    /// The unit projection pole `N`.
    pole: Vec3,
    /// Orthonormal plane frame with `e1 x e2 = -N`, so the chart preserves
    /// orientation about the outward normal.
    e1: Vec3,
    e2: Vec3,
}

impl Chart {
    fn new(center: Point3, pole: Vec3) -> Option<Self> {
        let reference = if pole.x().abs() < 0.6 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let e1 = reference.cross(pole).normalize().ok()?;
        let e2 = (-pole).cross(e1);
        Some(Self {
            center,
            pole,
            e1,
            e2,
        })
    }

    /// The chart image of a unit direction, `None` at the pole.
    fn image(&self, q: Vec3) -> Option<Point2> {
        let den = 1.0 - q.dot(self.pole);
        if den <= AT_POLE {
            return None;
        }
        Some(Point2::new(q.dot(self.e1) / den, q.dot(self.e2) / den))
    }

    /// The unit direction of a point about the centre; `None` at the centre.
    fn direction(&self, point: Point3) -> Option<Vec3> {
        (point - self.center).normalize().ok()
    }

    /// A sampled loop's image with the band of every chord; `None` when a
    /// sample is the pole or a circle chord's image arc is not minor.
    fn read(&self, sampled: &SampledLoop) -> Option<ChartLoop> {
        let points = sampled
            .points
            .iter()
            .map(|&q| self.image(q))
            .collect::<Option<Vec<_>>>()?;
        let n = points.len();
        let mut bands = Vec::with_capacity(n);
        for k in 0..n {
            let (a, b) = (points[k], points[(k + 1) % n]);
            let interior = sampled.interiors[k]
                .iter()
                .map(|&q| self.image(q))
                .collect::<Option<Vec<_>>>()?;
            let band = if sampled.circular[k] {
                arc_sagitta(a, *interior.first()?, b)?
            } else {
                SPLINE_BAND_FACTOR
                    * interior
                        .iter()
                        .map(|&m| segment_distance_2d(m, a, b))
                        .fold(0.0, f64::max)
            };
            let scale = 1.0 + a.x().abs().max(a.y().abs()) + b.x().abs().max(b.y().abs());
            bands.push(BAND_SLACK.mul_add(scale, band));
        }
        Some(ChartLoop {
            polygon: points.iter().map(|p| (p.x(), p.y())).collect(),
            points,
            bands,
        })
    }
}

/// The largest distance from chord `a b` of the circular arc from `a`
/// through `m` to `b`; `None` unless the arc is minor (less than half its
/// circle), whose points all lie over the chord.
fn arc_sagitta(a: Point2, m: Point2, b: Point2) -> Option<f64> {
    let (ma, mb) = (a - m, b - m);
    if ma.dot(mb) >= 0.0 {
        return None;
    }
    let chord = (b - a).length();
    let cross = (ma.x() * mb.y() - ma.y() * mb.x()).abs();
    let half = 0.5 * chord;
    if cross == 0.0 {
        return Some(0.0);
    }
    let radius = ma.length() * mb.length() * chord / (2.0 * cross);
    Some(half * half / (radius + half.mul_add(-half, radius * radius).max(0.0).sqrt()))
}

fn segment_distance_2d(p: Point2, a: Point2, b: Point2) -> f64 {
    let ab = b - a;
    let length_sq = ab.dot(ab);
    let t = if length_sq > 0.0 {
        ((p - a).dot(ab) / length_sq).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).length()
}

/// A loop's chart image: its polygon and the band of every chord.
#[derive(Debug, Clone)]
struct ChartLoop {
    points: Vec<Point2>,
    polygon: Vec<(f64, f64)>,
    bands: Vec<f64>,
}

impl ChartLoop {
    /// The loop's winding about `p`, or `None` within some chord's band.
    fn winding(&self, p: Point2) -> Option<i32> {
        let n = self.points.len();
        for k in 0..n {
            if segment_distance_2d(p, self.points[k], self.points[(k + 1) % n]) <= self.bands[k] {
                return None;
            }
        }
        Some(winding_number_xy(p, &self.polygon))
    }
}

/// How a face's trim reads in the stereographic chart.
#[derive(Debug, Clone)]
pub(super) enum StereoReading {
    /// Read: every hit away from the bands decides.
    Chart(StereoTrim),
    /// No loop has extent (a whole-sphere face): the caller keeps its path.
    NoBoundary,
    /// The trim cannot be read; every hit abstains.
    Undecided,
}

/// A sphere face's trim loops in the stereographic chart from a pole clear of
/// them.
#[derive(Debug, Clone)]
pub(super) struct StereoTrim {
    chart: Chart,
    loops: Vec<ChartLoop>,
    /// Whether the projection pole lies in the face.
    pole_inside: bool,
}

impl StereoTrim {
    /// Read a sphere face's outer and inner wires.
    ///
    /// # Errors
    ///
    /// Returns an error if a topology lookup fails.
    pub(super) fn build(
        topo: &Topology,
        face_id: FaceId,
        sphere: &SphericalSurface,
    ) -> Result<StereoReading, CheckError> {
        let face = topo.face(face_id)?;
        let mut loops = Vec::new();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            match sample_wire(topo, wire, sphere.center())? {
                WireReading::Loop(sampled) => loops.push(sampled),
                WireReading::NoExtent => {}
                WireReading::Unsupported => return Ok(StereoReading::Undecided),
            }
        }
        if loops.is_empty() {
            return Ok(StereoReading::NoBoundary);
        }
        Ok(Self::from_loops(sphere.center(), &loops)
            .map_or(StereoReading::Undecided, StereoReading::Chart))
    }

    fn from_loops(center: Point3, sampled: &[SampledLoop]) -> Option<Self> {
        let chart = Chart::new(center, clearest_pole(sampled)?)?;
        let loops = sampled
            .iter()
            .map(|sampled| chart.read(sampled))
            .collect::<Option<Vec<_>>>()?;
        let (mut total, mut magnitude) = (0.0_f64, 0.0_f64);
        for chart_loop in &loops {
            let area = double_area(&chart_loop.polygon);
            total += area;
            magnitude += area.abs();
        }
        if !total.is_finite() || total.abs() <= AREA_REL_EPS * magnitude {
            return None;
        }
        Some(Self {
            chart,
            loops,
            pole_inside: total < 0.0,
        })
    }

    /// Whether `point`, on the sphere, lies in the face; `None` within a
    /// chord's band or when the loops' winding about it is not that of a
    /// simple trim.
    pub(super) fn contains(&self, point: Point3) -> Option<bool> {
        let Some(hit) = self.chart.image(self.chart.direction(point)?) else {
            return Some(self.pole_inside);
        };
        let mut winding = 0;
        for chart_loop in &self.loops {
            winding += chart_loop.winding(hit)?;
        }
        match winding + i32::from(self.pole_inside) {
            1 => Some(true),
            0 => Some(false),
            _ => None,
        }
    }
}

/// One hole of a spherical cap in a chart from a pole clear of it, with the
/// parity of a point known to lie outside it.
#[derive(Debug, Clone)]
struct CapHole {
    chart: Chart,
    image: ChartLoop,
    /// Parity of the hole's winding about the reference point off the cap:
    /// odd exactly when the chart's pole lies in the hole.
    pole_parity: i32,
}

/// The holes of a spherical cap (a face whose outer loop is one plane
/// section), read without any winding convention.
///
/// Every hole lies in the cap, so `outside`, the sphere point farthest off
/// the cap, lies in none of them. Each hole is read in a chart from a pole
/// clear of that hole, and a hit is in the hole exactly when the hole's
/// image separates it from `outside`: their winding numbers differ in
/// parity.
#[derive(Debug, Clone)]
pub(super) struct CapHoles {
    holes: Vec<CapHole>,
}

impl CapHoles {
    /// Read every inner wire of `face_id`; `Ok(None)` when one cannot be
    /// read: an unsupported edge, no candidate pole clear of the hole, or
    /// `outside` within a band of the hole's image.
    ///
    /// # Errors
    ///
    /// Returns an error if a topology lookup fails.
    pub(super) fn build(
        topo: &Topology,
        face_id: FaceId,
        center: Point3,
        outside: Vec3,
    ) -> Result<Option<Self>, CheckError> {
        let mut holes = Vec::new();
        for &wire in topo.face(face_id)?.inner_wires() {
            let sampled = match sample_wire(topo, wire, center)? {
                WireReading::Loop(sampled) => sampled,
                WireReading::NoExtent => continue,
                WireReading::Unsupported => return Ok(None),
            };
            let Some(hole) = CapHole::read(center, &sampled, outside) else {
                return Ok(None);
            };
            holes.push(hole);
        }
        Ok(Some(Self { holes }))
    }

    /// Whether `point`, on the sphere, lies in a hole (an odd number of
    /// them, as nested inner wires alternate); `None` within a band.
    pub(super) fn in_hole(&self, point: Point3) -> Option<bool> {
        let mut containing = 0;
        for hole in &self.holes {
            containing += i32::from(hole.contains(point)?);
        }
        Some(containing % 2 == 1)
    }
}

impl CapHole {
    fn read(center: Point3, sampled: &SampledLoop, outside: Vec3) -> Option<Self> {
        let chart = Chart::new(center, clearest_pole(std::slice::from_ref(sampled))?)?;
        let image = chart.read(sampled)?;
        let pole_parity = match chart.image(outside) {
            Some(reference) => image.winding(reference)?.rem_euclid(2),
            // `outside` is the pole itself, which lies in no hole.
            None => 0,
        };
        Some(Self {
            chart,
            image,
            pole_parity,
        })
    }

    fn contains(&self, point: Point3) -> Option<bool> {
        let Some(hit) = self.chart.image(self.chart.direction(point)?) else {
            return Some(self.pole_parity == 1);
        };
        Some(self.image.winding(hit)?.rem_euclid(2) != self.pole_parity)
    }
}

/// Twice the signed area of a closed polygon.
fn double_area(polygon: &[(f64, f64)]) -> f64 {
    let n = polygon.len();
    (0..n)
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % n]);
            a.0.mul_add(b.1, -(b.0 * a.1))
        })
        .sum()
}

/// The candidate pole farthest from every sample of every loop: the 26
/// directions to the faces, edges and corners of a cube, first one on ties.
/// `None` when even the best clears the loops by less than
/// [`MIN_POLE_CLEARANCE`].
fn clearest_pole(loops: &[SampledLoop]) -> Option<Vec3> {
    let mut best: Option<(f64, Vec3)> = None;
    for i in -1_i32..=1 {
        for j in -1_i32..=1 {
            for k in -1_i32..=1 {
                if i == 0 && j == 0 && k == 0 {
                    continue;
                }
                let Ok(candidate) = Vec3::new(f64::from(i), f64::from(j), f64::from(k)).normalize()
                else {
                    continue;
                };
                let clearance = clearance(candidate, loops);
                if best.is_none_or(|(value, _)| clearance > value) {
                    best = Some((clearance, candidate));
                }
            }
        }
    }
    best.filter(|(clearance, _)| *clearance >= MIN_POLE_CLEARANCE)
        .map(|(_, pole)| pole)
}

/// Least distance from `p` to every chord of every loop and to every
/// interior sample (the edges stay near both).
fn clearance(p: Vec3, loops: &[SampledLoop]) -> f64 {
    loops
        .iter()
        .flat_map(|sampled| {
            let n = sampled.points.len();
            (0..n)
                .map(move |s| segment_distance(p, sampled.points[s], sampled.points[(s + 1) % n]))
                .chain(sampled.all_points().map(move |q| (p - q).length()))
        })
        .fold(f64::INFINITY, f64::min)
}

/// Distance from `p` to the segment `a b`.
fn segment_distance(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let length_sq = ab.dot(ab);
    let t = if length_sq > 0.0 {
        ((p - a).dot(ab) / length_sq).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).length()
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use crate::classify::boundary::FaceTrimData;

    /// The chart of sampled polygons standing for the loops (chords with no
    /// band beyond rounding).
    fn chart_of(trim: &FaceTrimData, sphere: &SphericalSurface) -> Option<StereoTrim> {
        let center = sphere.center();
        let loops: Vec<SampledLoop> = std::iter::once(&trim.outer)
            .chain(&trim.holes)
            .filter(|polygon| {
                polygon.len() >= 3
                    && polygon
                        .iter()
                        .any(|p| (*p - polygon[0]).length_squared() >= super::super::COINCIDENT_SQ)
            })
            .map(|polygon| SampledLoop {
                points: polygon
                    .iter()
                    .map(|&p| (p - center).normalize().unwrap())
                    .collect(),
                interiors: vec![Vec::new(); polygon.len()],
                circular: vec![false; polygon.len()],
            })
            .collect();
        if loops.is_empty() {
            return None;
        }
        StereoTrim::from_loops(center, &loops)
    }

    /// A circle of the unit sphere about `axis` at polar angle `theta`,
    /// counter-clockwise about `axis` looking down the outward normal.
    fn circle(center: Point3, radius: f64, axis: Vec3, theta: f64, n: usize) -> Vec<Point3> {
        let axis = axis.normalize().unwrap();
        let helper = if axis.x().abs() < 0.6 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        let a = helper.cross(axis).normalize().unwrap();
        let b = axis.cross(a);
        #[allow(clippy::cast_precision_loss)]
        (0..n)
            .map(|i| {
                let phi = std::f64::consts::TAU * i as f64 / n as f64;
                let dir = axis * theta.cos() + (a * phi.cos() + b * phi.sin()) * theta.sin();
                center + dir * radius
            })
            .collect()
    }

    fn on_sphere(center: Point3, radius: f64, dir: Vec3) -> Point3 {
        center + dir.normalize().unwrap() * radius
    }

    /// Polar cap of half-angle `theta` about `axis`: inside exactly when the
    /// angle to the axis is below `theta`, at any scale and placement, with
    /// the cap's own pole and its antipode decided too.
    #[test]
    fn polar_caps_match_the_angle_to_their_axis() {
        for (center, radius) in [
            (Point3::new(0.0, 0.0, 0.0), 1.0),
            (Point3::new(17.0, -23.0, 31.0), 1e-3),
            (Point3::new(-4e3, 2e3, 7e2), 1e3),
        ] {
            let sphere = SphericalSurface::new(center, radius).unwrap();
            for axis in [
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.3, -0.7, 0.2),
            ] {
                for theta in [0.4, 1.4, std::f64::consts::FRAC_PI_2, 2.6] {
                    let trim = FaceTrimData::from_parts(
                        circle(center, radius, axis, theta, 96),
                        Vec::new(),
                    );
                    let chart = chart_of(&trim, &sphere).unwrap();
                    for probe in [
                        axis,
                        -axis,
                        Vec3::new(0.2, 0.9, -0.1),
                        Vec3::new(-0.6, 0.1, 0.75),
                        Vec3::new(0.5, -0.5, -0.7),
                        Vec3::new(0.0, 0.0, 1.0),
                        Vec3::new(0.0, 0.0, -1.0),
                    ] {
                        let unit = probe.normalize().unwrap();
                        let angle = unit.dot(axis.normalize().unwrap()).acos();
                        if (angle - theta).abs() < 0.05 {
                            continue;
                        }
                        assert_eq!(
                            chart.contains(on_sphere(center, radius, unit)),
                            Some(angle < theta),
                            "axis {axis:?} theta {theta} probe {probe:?} scale {radius}"
                        );
                    }
                }
            }
        }
    }

    /// A band between two latitude circles (outer loop counter-clockwise
    /// about `+z` at the upper rim, inner loop the other way at the lower):
    /// both loops wind the axis, and the band holds neither pole.
    #[test]
    fn latitude_band_with_two_winding_rims() {
        let center = Point3::new(3.0, -1.0, 2.0);
        let sphere = SphericalSurface::new(center, 2.0).unwrap();
        let z = Vec3::new(0.0, 0.0, 1.0);
        // Upper rim at polar angle 0.8: counter-clockwise about -z bounds the
        // band below it. Lower rim at 2.0: counter-clockwise about +z bounds
        // what lies above it.
        let upper = circle(center, 2.0, -z, std::f64::consts::PI - 0.8, 64);
        let lower = circle(center, 2.0, z, 2.0, 64);
        let chart =
            chart_of(&FaceTrimData::from_parts(upper, vec![lower]), &sphere).expect("band chart");
        for (polar, inside) in [(0.3, false), (1.0, true), (1.6, true), (2.5, false)] {
            for azimuth in [0.1, 2.0, 4.0] {
                let dir = Vec3::new(
                    f64::sin(polar) * f64::cos(azimuth),
                    f64::sin(polar) * f64::sin(azimuth),
                    f64::cos(polar),
                );
                assert_eq!(
                    chart.contains(on_sphere(center, 2.0, dir)),
                    Some(inside),
                    "polar {polar} azimuth {azimuth}"
                );
            }
        }
    }

    /// A hemisphere minus a small disc around its pole: the hole loop runs
    /// against the rim, so the pole is outside and the rest of the cap in.
    #[test]
    fn holed_cap_excludes_its_hole() {
        let center = Point3::new(0.0, 0.0, 0.0);
        let sphere = SphericalSurface::new(center, 1.0).unwrap();
        let z = Vec3::new(0.0, 0.0, 1.0);
        let rim = circle(center, 1.0, z, std::f64::consts::FRAC_PI_2, 64);
        let mut hole = circle(center, 1.0, z, 0.3, 32);
        hole.reverse();
        let chart =
            chart_of(&FaceTrimData::from_parts(rim, vec![hole]), &sphere).expect("holed cap chart");
        assert_eq!(chart.contains(Point3::new(0.0, 0.0, 1.0)), Some(false));
        assert_eq!(
            chart.contains(on_sphere(center, 1.0, Vec3::new(0.5, 0.2, 0.6))),
            Some(true)
        );
        assert_eq!(
            chart.contains(on_sphere(center, 1.0, Vec3::new(0.5, 0.2, -0.6))),
            Some(false)
        );
    }

    /// A whole-sphere trim (no loop with extent) has no chart; nor has a loop
    /// sampled densely enough to leave no candidate pole clear of it.
    #[test]
    fn degenerate_and_unclearable_trims_decline() {
        let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).unwrap();
        let point = Point3::new(1.0, 0.0, 0.0);
        let degenerate = FaceTrimData::from_parts(vec![point; 4], Vec::new());
        assert!(chart_of(&degenerate, &sphere).is_none());
        // A spiral running pole to pole through every candidate direction.
        #[allow(clippy::cast_precision_loss)]
        let spiral: Vec<Point3> = (0..=20_000)
            .map(|i| {
                let t = i as f64 / 20_000.0;
                let polar = std::f64::consts::PI * t;
                let azimuth = 2000.0 * t;
                Point3::new(
                    polar.sin() * azimuth.cos(),
                    polar.sin() * azimuth.sin(),
                    polar.cos(),
                )
            })
            .collect();
        let mut closed = spiral.clone();
        closed.extend(
            spiral
                .iter()
                .rev()
                .skip(1)
                .map(|p| Point3::new(p.x(), -p.y(), p.z())),
        );
        assert!(chart_of(&FaceTrimData::from_parts(closed, Vec::new()), &sphere).is_none());
    }
}
