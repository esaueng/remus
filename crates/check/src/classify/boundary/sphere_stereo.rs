//! Trim membership of a point on a sphere face whose trim holds a pole of the
//! sphere's own chart or winds its axis (B264).
//!
//! The `(longitude, latitude)` chart reads a sphere trim only when every loop
//! stays clear of both poles and closes in longitude. A piece that holds a
//! pole (the cap a sphere–torus fuse keeps above the tube) has a loop around
//! the axis, which bounds no polygon in that chart. The chorded 3D polygon,
//! projected onto the coordinate plane nearest its Newell normal, stood in
//! for it (and a cap's holes were projected the same way), and that projection
//! does not bound a strongly curved patch: a hit on the far side of the cap
//! tested inside the loop's shadow.
//!
//! This trim reads the loops in a chart re-centred for the face: the
//! stereographic projection from a unit direction `N` chosen well clear of
//! every loop. The projection is a smooth, orientation-preserving chart of
//! the whole sphere but `N`, so every loop maps to a closed polygon in the
//! plane, and no loop winds anything it cannot bound.
//!
//! Which side of the loops is the face follows the stored winding the
//! assembly restores on every curved face (`orient_revolved_face_wires`): the
//! face lies left of its loops looking down the stored outward sphere
//! normal, whatever the face's reversal flag, so a positive loop bounds a cap
//! and a hole runs the other way. In the plane the boundary of a bounded
//! region then has total winding `1` about its interior points and a
//! positive signed area; the boundary of the region holding `N` (which maps
//! to infinity) has winding `0` about its interior points, `-1` outside, and
//! a negative area. So the loops' total signed area says whether `N` is in
//! the face, and the winding number about a hit's image decides the hit.
//! A sum that is neither `0` nor `1` cannot come from a simple, consistently
//! wound trim, and the hit is undecided.

use remus_math::predicates::winding_number_xy;
use remus_math::surfaces::SphericalSurface;
use remus_math::vec::{Point2, Point3, Vec3};

use super::{COINCIDENT_SQ, FaceTrimData};

/// Least distance, on the unit sphere, from the projection pole to every
/// chord of every loop. Within it the images of loop samples grow like the
/// inverse square of the distance; at this clearance they stay below `2e4`
/// (the chart is of the unit sphere).
const MIN_POLE_CLEARANCE: f64 = 1e-2;

/// A total signed area this small against the loops' own areas leaves the
/// side of the projection pole undecided.
const AREA_REL_EPS: f64 = 1e-9;

/// Hits this close to the projection pole (`1 - q . N`) take the pole's own
/// side: the pole is at least [`MIN_POLE_CLEARANCE`] from the boundary.
const AT_POLE: f64 = 1e-12;

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
    fn image(&self, q: Vec3) -> Option<(f64, f64)> {
        let den = 1.0 - q.dot(self.pole);
        if den <= AT_POLE {
            return None;
        }
        Some((q.dot(self.e1) / den, q.dot(self.e2) / den))
    }

    /// The unit direction of a point about the centre; `None` at the centre.
    fn direction(&self, point: Point3) -> Option<Vec3> {
        (point - self.center).normalize().ok()
    }
}

/// Whether a sampled loop has extent: three samples, not all coincident.
fn has_extent(polygon: &[Point3]) -> bool {
    polygon.first().is_some_and(|&first| {
        polygon.len() >= 3
            && polygon
                .iter()
                .any(|p| (*p - first).length_squared() >= COINCIDENT_SQ)
    })
}

/// Unit directions of a sampled loop about `center`; `None` when a sample is
/// the centre.
fn unit_directions(polygon: &[Point3], center: Point3) -> Option<Vec<Vec3>> {
    polygon
        .iter()
        .map(|&p| (p - center).normalize().ok())
        .collect()
}

/// A sphere face's trim loops in the stereographic chart from a pole clear of
/// them.
#[derive(Debug, Clone)]
pub(super) struct StereoTrim {
    chart: Chart,
    /// Every non-degenerate loop's image.
    loops: Vec<Vec<(f64, f64)>>,
    /// Whether the projection pole lies in the face.
    pole_inside: bool,
}

impl StereoTrim {
    /// The chart for a face's sampled trim, or `None` when the trim has no
    /// loop with extent (a whole-sphere face), a sample sits at the centre,
    /// no candidate pole clears every loop, or the pole's side is lost to
    /// cancellation.
    pub(super) fn build(trim: &FaceTrimData, sphere: &SphericalSurface) -> Option<Self> {
        let center = sphere.center();
        let mut unit_loops: Vec<Vec<Vec3>> = Vec::new();
        for polygon in std::iter::once(&trim.outer).chain(&trim.holes) {
            if has_extent(polygon) {
                unit_loops.push(unit_directions(polygon, center)?);
            }
        }
        if unit_loops.is_empty() {
            return None;
        }
        let chart = Chart::new(center, clearest_pole(&unit_loops)?)?;
        let mut loops = Vec::with_capacity(unit_loops.len());
        let (mut total, mut magnitude) = (0.0_f64, 0.0_f64);
        for directions in &unit_loops {
            let image = directions
                .iter()
                .map(|&q| chart.image(q))
                .collect::<Option<Vec<_>>>()?;
            let area = double_area(&image);
            total += area;
            magnitude += area.abs();
            loops.push(image);
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

    /// Whether `point`, on the sphere, lies in the face; `None` when the
    /// loops' winding about it is not that of a simple trim.
    pub(super) fn contains(&self, point: Point3) -> Option<bool> {
        let Some((x, y)) = self.chart.image(self.chart.direction(point)?) else {
            return Some(self.pole_inside);
        };
        let hit = Point2::new(x, y);
        let winding: i32 = self
            .loops
            .iter()
            .map(|image| winding_number_xy(hit, image))
            .sum();
        match winding + i32::from(self.pole_inside) {
            1 => Some(true),
            0 => Some(false),
            _ => None,
        }
    }
}

/// The holes of a spherical cap (a face whose outer loop is one plane
/// section) read in the stereographic chart from a point of the sphere off
/// the cap.
///
/// Every hole lies in the cap, so the projection pole is in no hole, each
/// hole maps to the bounded region of its image, and a hit is in a hole
/// exactly when that image winds about it. No winding convention is needed.
#[derive(Debug, Clone)]
pub(super) struct CapHoles {
    chart: Chart,
    holes: Vec<Vec<(f64, f64)>>,
}

impl CapHoles {
    /// The chart from `outside`, a unit direction about `center` that lies
    /// off the cap; `None` when a hole sample is the centre or `outside`
    /// comes closer than [`MIN_POLE_CLEARANCE`] to a hole.
    pub(super) fn build(holes: &[Vec<Point3>], center: Point3, outside: Vec3) -> Option<Self> {
        let mut unit_holes = Vec::with_capacity(holes.len());
        for hole in holes {
            if has_extent(hole) {
                unit_holes.push(unit_directions(hole, center)?);
            }
        }
        if clearance(outside, &unit_holes) < MIN_POLE_CLEARANCE {
            return None;
        }
        let chart = Chart::new(center, outside)?;
        let holes = unit_holes
            .iter()
            .map(|directions| {
                directions
                    .iter()
                    .map(|&q| chart.image(q))
                    .collect::<Option<Vec<_>>>()
            })
            .collect::<Option<Vec<_>>>()?;
        Some(Self { chart, holes })
    }

    /// Whether `point`, on the sphere, lies in one of the holes; `None` for
    /// the centre.
    pub(super) fn in_hole(&self, point: Point3) -> Option<bool> {
        let Some((x, y)) = self.chart.image(self.chart.direction(point)?) else {
            return Some(false);
        };
        let hit = Point2::new(x, y);
        Some(
            self.holes
                .iter()
                .any(|image| winding_number_xy(hit, image) != 0),
        )
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

/// The candidate pole farthest from every chord of every loop: the 26
/// directions to the faces, edges and corners of a cube, first one on ties.
/// `None` when even the best clears the loops by less than
/// [`MIN_POLE_CLEARANCE`].
fn clearest_pole(loops: &[Vec<Vec3>]) -> Option<Vec3> {
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

/// Least distance from `p` to every chord of every loop.
fn clearance(p: Vec3, loops: &[Vec<Vec3>]) -> f64 {
    loops
        .iter()
        .flat_map(|directions| {
            let n = directions.len();
            (0..n).map(move |s| (directions[s], directions[(s + 1) % n]))
        })
        .map(|(a, b)| segment_distance(p, a, b))
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
                    let chart = StereoTrim::build(&trim, &sphere).unwrap();
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
        let chart = StereoTrim::build(&FaceTrimData::from_parts(upper, vec![lower]), &sphere)
            .expect("band chart");
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
        let chart = StereoTrim::build(&FaceTrimData::from_parts(rim, vec![hole]), &sphere)
            .expect("holed cap chart");
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
        assert!(StereoTrim::build(&degenerate, &sphere).is_none());
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
        assert!(
            StereoTrim::build(&FaceTrimData::from_parts(closed, Vec::new()), &sphere).is_none()
        );
    }
}
