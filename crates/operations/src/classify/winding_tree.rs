//! Bounded solid-angle intervals for the robust classifier's existing thresholds.
//!
//! For F(x)=(x-p)/|x-p|³ the derivative's operator norm is 2/|x-p|³.
//! A triangle cluster in a ball (c,r), of total area A and vector area N,
//! therefore differs from F(c)·N by at most 2rA/(|c-p|-r)³ when |c-p|>r.
//! Near clusters use the original triangle formula. Disjoint cluster error
//! budgets sum to at most ANGULAR_ERROR_BUDGET, with roundoff padding. Only an
//! interval wholly beyond the existing decisive threshold is returned;
//! ambiguous intervals retain the original complete sum and ray fallback.

use remus_math::aabb::Aabb3;
use remus_math::vec::{Point3, Vec3};

use super::{AMBIGUOUS_BAND, INSIDE_THRESHOLD, PI, PointClassification, solid_angle};
use crate::tessellate::TriangleMesh;

const LEAF_TRIANGLES: usize = 8;
const MAX_TRIANGLES: usize = 131_072;
const ANGULAR_ERROR_BUDGET: f64 = 2.0;

struct Node {
    bounds: Aabb3,
    center: Point3,
    radius: f64,
    area: f64,
    vector_area: Vec3,
    range: std::ops::Range<usize>,
    children: Option<(usize, usize)>,
}

pub(super) struct WindingTree {
    nodes: Vec<Node>,
    root: usize,
    triangles: usize,
}

#[derive(Default)]
struct Reading {
    angle: f64,
    error: f64,
    exact_triangles: usize,
}

fn magnitude(p: Point3) -> f64 {
    p.x().abs().max(p.y().abs()).max(p.z().abs())
}

impl WindingTree {
    /// One operation-local tree over the classifier's immutable mesh. No
    /// global retention, topology handles, geometry approximation or remeshing.
    pub(super) fn new(mesh: &TriangleMesh) -> Option<Self> {
        let triangles = mesh.indices.len() / 3;
        if triangles == 0 || triangles > MAX_TRIANGLES {
            return None;
        }
        let mut tree = Self {
            nodes: Vec::new(),
            root: 0,
            triangles,
        };
        tree.root = tree.build(mesh, 0..triangles)?;
        Some(tree)
    }

    fn build(&mut self, mesh: &TriangleMesh, range: std::ops::Range<usize>) -> Option<usize> {
        let children = if range.len() > LEAF_TRIANGLES {
            let mid = range.start + range.len() / 2;
            Some((
                self.build(mesh, range.start..mid)?,
                self.build(mesh, mid..range.end)?,
            ))
        } else {
            None
        };
        let (bounds, vector_area, area) = if let Some((left, right)) = children {
            let a = &self.nodes[left];
            let b = &self.nodes[right];
            (
                a.bounds.union(b.bounds),
                a.vector_area + b.vector_area,
                a.area + b.area,
            )
        } else {
            let mut bounds: Option<Aabb3> = None;
            let mut vector_area = Vec3::new(0., 0., 0.);
            let mut area = 0.;
            for i in range.clone() {
                let tri = &mesh.indices[3 * i..3 * i + 3];
                let (a, b, c) = (
                    *mesh.positions.get(tri[0] as usize)?,
                    *mesh.positions.get(tri[1] as usize)?,
                    *mesh.positions.get(tri[2] as usize)?,
                );
                let bnd = Aabb3::try_from_points([a, b, c])?;
                bounds = Some(bounds.map_or(bnd, |old| old.union(bnd)));
                let n = (b - a).cross(c - a) * 0.5;
                vector_area += n;
                area += n.length();
            }
            (bounds?, vector_area, area)
        };
        let center = Point3::new(
            f64::midpoint(bounds.min.x(), bounds.max.x()),
            f64::midpoint(bounds.min.y(), bounds.max.y()),
            f64::midpoint(bounds.min.z(), bounds.max.z()),
        );
        let extent = Vec3::new(
            (bounds.min.x() - center.x())
                .abs()
                .max((bounds.max.x() - center.x()).abs()),
            (bounds.min.y() - center.y())
                .abs()
                .max((bounds.max.y() - center.y()).abs()),
            (bounds.min.z() - center.z())
                .abs()
                .max((bounds.max.z() - center.z()).abs()),
        );
        let radius = extent.length() * (1. + 256. * f64::EPSILON)
            + 256. * f64::EPSILON * magnitude(center).max(1.);
        if !area.is_finite() || !radius.is_finite() || !vector_area.length().is_finite() {
            return None;
        }
        let id = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            center,
            radius,
            area: area * (1. + 256. * f64::EPSILON),
            vector_area,
            range,
            children,
        });
        Some(id)
    }

    fn read(&self, id: usize, mesh: &TriangleMesh, point: Point3, reading: &mut Reading) {
        let node = &self.nodes[id];
        let delta = node.center - point;
        let distance = delta.length();
        let roundoff =
            256. * f64::EPSILON * (magnitude(point) + magnitude(node.center) + distance + 1.);
        let gap = distance - roundoff - node.radius;
        if gap > 0. && distance.is_finite() {
            let angle = node.vector_area.dot(delta) / distance.powi(3);
            let error = 2. * node.radius * node.area / gap.powi(3)
                + 256. * f64::EPSILON * (angle.abs() + node.area / gap.powi(2) + 1.);
            #[allow(clippy::cast_precision_loss)]
            let share = ANGULAR_ERROR_BUDGET * node.range.len() as f64 / self.triangles as f64;
            if angle.is_finite() && error.is_finite() && error <= share {
                reading.angle += angle;
                reading.error += error;
                return;
            }
        }
        if let Some((left, right)) = node.children {
            self.read(left, mesh, point, reading);
            self.read(right, mesh, point, reading);
        } else {
            for i in node.range.clone() {
                let tri = &mesh.indices[3 * i..3 * i + 3];
                reading.angle += solid_angle(
                    point,
                    mesh.positions[tri[0] as usize],
                    mesh.positions[tri[1] as usize],
                    mesh.positions[tri[2] as usize],
                );
                reading.exact_triangles += 1;
            }
        }
    }

    pub(super) fn retained_bytes(&self) -> usize {
        self.nodes.capacity() * std::mem::size_of::<Node>()
    }

    pub(super) fn decisive(
        &self,
        mesh: &TriangleMesh,
        point: Point3,
    ) -> Option<PointClassification> {
        self.decisive_with_work(mesh, point).0
    }

    fn decisive_with_work(
        &self,
        mesh: &TriangleMesh,
        point: Point3,
    ) -> (Option<PointClassification>, usize) {
        let mut reading = Reading::default();
        self.read(self.root, mesh, point, &mut reading);
        // Bound the regrouping and triangle-formula rounding far below the
        // classifier's 0.2-wide ambiguous band; a large-coordinate cluster's
        // expanded radius also forces the ordinary triangle path when needed.
        #[allow(clippy::cast_precision_loss)]
        let error = reading.error + 1e-9 * self.triangles as f64;
        let high = 4. * PI * (INSIDE_THRESHOLD + AMBIGUOUS_BAND);
        let low = 4. * PI * (INSIDE_THRESHOLD - AMBIGUOUS_BAND);
        let verdict = if reading.angle - error > high {
            Some(PointClassification::Inside)
        } else if reading.angle + error < low {
            Some(PointClassification::Outside)
        } else {
            None
        };
        (verdict, reading.exact_triangles)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::cast_precision_loss)]
    use super::*;
    use crate::classify::mesh_winding_number;

    fn subdivided_cube(n: u32, scale: f64, offset: f64) -> TriangleMesh {
        let mut mesh = TriangleMesh::default();
        // Each frame's du x dv is the face's outward normal.
        let frames = [
            (
                Point3::new(0., 0., 0.),
                Vec3::new(0., 0., 1.),
                Vec3::new(0., 1., 0.),
            ),
            (
                Point3::new(1., 0., 0.),
                Vec3::new(0., 1., 0.),
                Vec3::new(0., 0., 1.),
            ),
            (
                Point3::new(0., 0., 0.),
                Vec3::new(1., 0., 0.),
                Vec3::new(0., 0., 1.),
            ),
            (
                Point3::new(0., 1., 0.),
                Vec3::new(0., 0., 1.),
                Vec3::new(1., 0., 0.),
            ),
            (
                Point3::new(0., 0., 0.),
                Vec3::new(0., 1., 0.),
                Vec3::new(1., 0., 0.),
            ),
            (
                Point3::new(0., 0., 1.),
                Vec3::new(1., 0., 0.),
                Vec3::new(0., 1., 0.),
            ),
        ];
        for (origin, u, v) in frames {
            for i in 0..n {
                for j in 0..n {
                    let a = origin
                        + u * (f64::from(i) / f64::from(n))
                        + v * (f64::from(j) / f64::from(n));
                    let du = u * (1. / f64::from(n));
                    let dv = v * (1. / f64::from(n));
                    let index = u32::try_from(mesh.positions.len()).unwrap();
                    for p in [a, a + du, a + du + dv, a + dv] {
                        mesh.positions.push(Point3::new(
                            offset + scale * p.x(),
                            offset + scale * p.y(),
                            offset + scale * p.z(),
                        ));
                    }
                    mesh.indices
                        .extend([index, index + 1, index + 2, index, index + 2, index + 3]);
                }
            }
        }
        mesh
    }

    #[test]
    fn every_decisive_interval_matches_the_complete_sum() {
        let mut skipped = 0;
        for (scale, offset) in [(1e-3, 0.), (1., 0.), (1e3, 0.), (1., 1e9)] {
            let mut mesh = subdivided_cube(24, scale, offset);
            for inverted in [false, true] {
                if inverted {
                    for t in mesh.indices.chunks_exact_mut(3) {
                        t.swap(1, 2);
                    }
                }
                let tree = WindingTree::new(&mesh).unwrap();
                for i in 0..96 {
                    let t = f64::from(i) / 96.;
                    let point = Point3::new(
                        offset + scale * (3. * t - 1.),
                        offset + scale * (3. * (7. * t).fract() - 1.),
                        offset + scale * (3. * (13. * t).fract() - 1.),
                    );
                    let full = mesh_winding_number(&mesh, point);
                    let mut reading = Reading::default();
                    tree.read(tree.root, &mesh, point, &mut reading);
                    let padding = 1e-9 * tree.triangles as f64;
                    assert!(
                        (reading.angle - full * 4. * PI).abs() <= reading.error + padding,
                        "scale {scale} offset {offset} point {point:?}"
                    );
                    let (decision, exact) = tree.decisive_with_work(&mesh, point);
                    if let Some(decision) = decision {
                        let expected = if full > INSIDE_THRESHOLD + AMBIGUOUS_BAND {
                            PointClassification::Inside
                        } else {
                            assert!(full < INSIDE_THRESHOLD - AMBIGUOUS_BAND);
                            PointClassification::Outside
                        };
                        assert_eq!(decision, expected);
                        skipped += tree.triangles - exact;
                    }
                }
            }
        }
        assert!(
            skipped > 100_000,
            "the oracle must exercise clustered intervals"
        );
    }

    #[test]
    fn an_ambiguous_half_shell_keeps_the_full_sum() {
        let mut mesh = subdivided_cube(24, 1., 0.);
        mesh.indices.truncate(mesh.indices.len() / 2);
        let tree = WindingTree::new(&mesh).unwrap();
        let point = Point3::new(0.5, 0.5, 0.5);
        assert!((mesh_winding_number(&mesh, point) - 0.5).abs() < 1e-12);
        assert_eq!(tree.decisive(&mesh, point), None);
    }
}
