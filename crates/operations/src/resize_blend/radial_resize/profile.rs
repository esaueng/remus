//! Exact meridian collision and volume checks for the qualified radial cell.

use super::super::reconstruction;
use crate::OperationsError;
use remus_math::{
    tolerance::Tolerance,
    vec::{Point3, Vec3},
};
use remus_topology::{
    Topology,
    edge::EdgeCurve,
    explorer::{edge_to_face_map, face_edges, solid_faces},
    face::{FaceId, FaceSurface},
    solid::SolidId,
};
use std::collections::BTreeMap;
use std::f64::consts::{PI, TAU};

#[derive(Clone, Copy, Debug)]
struct P {
    r: f64,
    z: f64,
}
impl P {
    fn sub(self, other: Self) -> Self {
        Self {
            r: self.r - other.r,
            z: self.z - other.z,
        }
    }
    fn add(self, other: Self) -> Self {
        Self {
            r: self.r + other.r,
            z: self.z + other.z,
        }
    }
    fn scale(self, value: f64) -> Self {
        Self {
            r: self.r * value,
            z: self.z * value,
        }
    }
    fn dot(self, other: Self) -> f64 {
        self.r * other.r + self.z * other.z
    }
    fn cross(self, other: Self) -> f64 {
        self.r * other.z - self.z * other.r
    }
    fn length(self) -> f64 {
        self.r.hypot(self.z)
    }
}

#[derive(Clone, Copy, Debug)]
enum Curve {
    Line(P, P),
    Arc {
        center: P,
        radius: f64,
        first: f64,
        last: f64,
    },
}
impl Curve {
    fn point(self, t: f64) -> P {
        match self {
            Self::Line(a, b) => a.add(b.sub(a).scale(t)),
            Self::Arc {
                center,
                radius,
                first,
                last,
            } => {
                let angle = first + t * (last - first);
                center.add(
                    P {
                        r: angle.cos(),
                        z: angle.sin(),
                    }
                    .scale(radius),
                )
            }
        }
    }
    fn tangent(self) -> P {
        match self {
            Self::Line(a, b) => b.sub(a),
            Self::Arc {
                radius,
                first,
                last,
                ..
            } => {
                let angle = f64::midpoint(first, last);
                P {
                    r: -angle.sin(),
                    z: angle.cos(),
                }
                .scale(radius * (last - first))
            }
        }
    }
    fn contains(self, p: P) -> bool {
        let tol = Tolerance::new().linear;
        match self {
            Self::Line(a, b) => {
                let d = b.sub(a);
                let q = p.sub(a);
                let t = q.dot(d) / d.dot(d);
                q.cross(d).abs() <= tol * d.length()
                    && t >= -tol / d.length()
                    && t <= 1.0 + tol / d.length()
            }
            Self::Arc {
                center,
                radius,
                first,
                last,
            } => {
                let q = p.sub(center);
                if (q.length() - radius).abs() > tol {
                    return false;
                }
                let angle = q.z.atan2(q.r);
                let lo = first.min(last);
                let hi = first.max(last);
                let at = angle + ((lo - angle) / TAU).ceil() * TAU;
                at <= hi + tol / radius || at - TAU >= lo - tol / radius
            }
        }
    }
    fn integral(self) -> f64 {
        match self {
            Self::Line(a, b) => (b.z - a.z) * (a.r * a.r + a.r * b.r + b.r * b.r) / 3.0,
            Self::Arc {
                center,
                radius,
                first,
                last,
            } => {
                let primitive = |t: f64| {
                    let s = t.sin();
                    radius * center.r * center.r * s
                        + center.r * radius * radius * (t + s * t.cos())
                        + radius.powi(3) * (s - s.powi(3) / 3.0)
                };
                primitive(last) - primitive(first)
            }
        }
    }
}

pub(super) struct Profile {
    curves: BTreeMap<usize, Curve>,
    volume: f64,
}

impl Profile {
    pub(super) fn read(
        topo: &Topology,
        solid: SolidId,
        origin: Point3,
        axis: Vec3,
    ) -> Result<Option<Self>, OperationsError> {
        let tol = Tolerance::new();
        let mut curves = BTreeMap::new();
        let mut volume = 0.0;
        for face in solid_faces(topo, solid)? {
            let data = topo.face(face)?;
            let mut rims = Vec::new();
            let mut seam = None;
            for edge in face_edges(topo, face)? {
                let source = topo.edge(edge)?;
                match source.curve() {
                    EdgeCurve::Circle(circle) => {
                        let delta = circle.center() - origin;
                        if circle.normal().cross(axis).length() <= tol.angular
                            && delta.cross(axis).length() <= tol.linear
                        {
                            let (lo, hi) = source.strict_domain().map_err(|error| {
                                reconstruction(format!("radial profile rim: {error}"))
                            })?;
                            if ((hi - lo).abs() - TAU).abs() > tol.angular {
                                return Ok(None);
                            }
                            rims.push(P {
                                r: circle.radius(),
                                z: delta.dot(axis),
                            });
                        } else if let FaceSurface::Torus(torus) = data.surface() {
                            let offset = circle.center() - torus.center();
                            if offset.dot(axis).abs() > tol.linear
                                || (offset.length() - torus.major_radius()).abs() > tol.linear
                                || circle.normal().dot(axis).abs() > tol.angular
                                || circle.normal().dot(offset).abs() > tol.linear
                                || (circle.radius() - torus.minor_radius()).abs() > tol.linear
                            {
                                return Ok(None);
                            }
                            let (first, last) = source.strict_domain().map_err(|error| {
                                reconstruction(format!("radial profile seam: {error}"))
                            })?;
                            if (last - first).abs() > PI + tol.angular {
                                return Ok(None);
                            }
                            let radial = offset * offset.length().recip();
                            let a = circle.radius() * circle.u_axis().dot(radial);
                            let b = circle.radius() * circle.v_axis().dot(radial);
                            let c = circle.radius() * circle.u_axis().dot(axis);
                            let d = circle.radius() * circle.v_axis().dot(axis);
                            let point = circle.evaluate(first) - circle.center();
                            let angle = point.dot(axis).atan2(point.dot(radial));
                            seam = Some(Curve::Arc {
                                center: P {
                                    r: torus.major_radius(),
                                    z: (torus.center() - origin).dot(axis),
                                },
                                radius: torus.minor_radius(),
                                first: angle,
                                last: angle + (last - first) * (a * d - b * c).signum(),
                            });
                        } else {
                            return Ok(None);
                        }
                    }
                    EdgeCurve::Line => {
                        if !matches!(
                            data.surface(),
                            FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
                        ) {
                            return Ok(None);
                        }
                    }
                    _ => return Ok(None),
                }
            }
            let curve = match data.surface() {
                FaceSurface::Plane { .. } => match rims.as_slice() {
                    [rim] => Curve::Line(P { r: 0.0, z: rim.z }, *rim),
                    [a, b] if (a.z - b.z).abs() <= tol.linear => Curve::Line(*a, *b),
                    _ => return Ok(None),
                },
                FaceSurface::Cylinder(_) | FaceSurface::Cone(_) => match rims.as_slice() {
                    [a, b] => Curve::Line(*a, *b),
                    _ => return Ok(None),
                },
                FaceSurface::Torus(_) => {
                    if rims.len() != 2
                        || !data.inner_wires().is_empty()
                        || topo.wire(data.outer_wire())?.edges().len() != 4
                    {
                        return Ok(None);
                    }
                    let Some(curve) = seam else { return Ok(None) };
                    if !rims.iter().all(|rim| curve.contains(*rim)) {
                        return Ok(None);
                    }
                    curve
                }
                _ => return Ok(None),
            };
            let point = curve.point(0.5);
            let tangent = curve.tangent();
            if tangent.length() <= tol.linear {
                return Err(reconstruction("radial resize collapses a meridian segment"));
            }
            let normal = match data.surface() {
                FaceSurface::Plane { normal, .. } => P {
                    r: 0.0,
                    z: normal.dot(axis),
                },
                FaceSurface::Cylinder(_) => P { r: 1.0, z: 0.0 },
                FaceSurface::Cone(cone) => P {
                    r: cone.half_angle().cos(),
                    z: -cone.half_angle().sin() * cone.axis().dot(axis),
                },
                FaceSurface::Torus(torus) => P {
                    r: (point.r - torus.major_radius()) / torus.minor_radius(),
                    z: (point.z - (torus.center() - origin).dot(axis)) / torus.minor_radius(),
                },
                _ => return Ok(None),
            }
            .scale(if data.is_reversed() { -1.0 } else { 1.0 });
            let orientation = (normal.r * tangent.z - normal.z * tangent.r).signum();
            volume += PI * orientation * curve.integral();
            curves.insert(face.index(), curve);
        }
        let profile = Self { curves, volume };
        profile.validate(topo, solid, origin, axis)?;
        Ok(Some(profile))
    }

    fn validate(
        &self,
        topo: &Topology,
        solid: SolidId,
        origin: Point3,
        axis: Vec3,
    ) -> Result<(), OperationsError> {
        let adjacency = edge_to_face_map(topo, solid)?;
        let tol = Tolerance::new().linear;
        let entries: Vec<_> = self.curves.iter().collect();
        for (i, &(&first, &a)) in entries.iter().enumerate() {
            for &(&second, &b) in &entries[i + 1..] {
                let mut allowed = Vec::new();
                for (&index, faces) in &adjacency {
                    if !faces.iter().any(|face| face.index() == first)
                        || !faces.iter().any(|face| face.index() == second)
                    {
                        continue;
                    }
                    let edge = topo
                        .edge_id_from_index(index)
                        .ok_or_else(|| reconstruction("radial adjacency lost its edge"))?;
                    if let EdgeCurve::Circle(circle) = topo.edge(edge)?.curve() {
                        allowed.push(P {
                            r: circle.radius(),
                            z: (circle.center() - origin).dot(axis),
                        });
                    }
                }
                for point in intersections(a, b) {
                    if !allowed
                        .iter()
                        .any(|&shared| point.sub(shared).length() <= tol * 4.0)
                    {
                        return Err(reconstruction(format!(
                            "radial resize intersects nonincident meridian boundaries {first} and {second}"
                        )));
                    }
                }
            }
        }
        if !self.volume.is_finite() || self.volume <= tol.powi(3) {
            return Err(reconstruction(
                "radial profile has no positive finite volume",
            ));
        }
        Ok(())
    }

    pub(super) fn verify_result(
        &self,
        topo: &Topology,
        solid: SolidId,
    ) -> Result<(), OperationsError> {
        let actual = crate::measure::solid_volume(topo, solid, 0.01)?;
        if (actual - self.volume).abs() > self.volume.abs() * 1e-6 + 1e-6 {
            return Err(reconstruction(format!(
                "radial resize volume {actual} differs from exact meridian integral {}",
                self.volume
            )));
        }
        Ok(())
    }

    pub(super) fn preserve_directions(
        &self,
        result: &Self,
        map: &std::collections::HashMap<usize, FaceId>,
    ) -> Result<(), OperationsError> {
        for (&source, curve) in &self.curves {
            let target = map
                .get(&source)
                .and_then(|face| result.curves.get(&face.index()))
                .ok_or_else(|| reconstruction("radial profile has no construction successor"))?;
            if curve.tangent().dot(target.tangent()) <= Tolerance::new().linear_sq() {
                return Err(reconstruction("radial resize reverses a meridian segment"));
            }
        }
        Ok(())
    }
}

fn intersections(a: Curve, b: Curve) -> Vec<P> {
    let tol = Tolerance::new().linear;
    match (a, b) {
        (Curve::Line(p, q), Curve::Line(r, s)) => {
            let d = q.sub(p);
            let e = s.sub(r);
            let determinant = d.cross(e);
            if determinant.abs() <= tol * d.length().max(e.length()) {
                if r.sub(p).cross(d).abs() > tol * d.length() {
                    return Vec::new();
                }
                let mut points = Vec::new();
                for point in [p, q, r, s] {
                    if a.contains(point) && b.contains(point) {
                        points.push(point);
                    }
                }
                if points.len() > 1 {
                    let middle = points[0].add(points[1]).scale(0.5);
                    points.push(middle);
                }
                points
            } else {
                let point = p.add(d.scale(r.sub(p).cross(e) / determinant));
                if a.contains(point) && b.contains(point) {
                    vec![point]
                } else {
                    Vec::new()
                }
            }
        }
        (Curve::Line(p, q), Curve::Arc { center, radius, .. })
        | (Curve::Arc { center, radius, .. }, Curve::Line(p, q)) => {
            let d = q.sub(p);
            let f = p.sub(center);
            let aa = d.dot(d);
            let bb = 2.0 * f.dot(d);
            let cc = f.dot(f) - radius * radius;
            let discriminant = bb * bb - 4.0 * aa * cc;
            if discriminant < -tol * aa * radius.max(1.0) * 8.0 {
                return Vec::new();
            }
            let root = discriminant.max(0.0).sqrt();
            [(-bb - root) / (2.0 * aa), (-bb + root) / (2.0 * aa)]
                .into_iter()
                .map(|t| p.add(d.scale(t)))
                .filter(|&point| a.contains(point) && b.contains(point))
                .collect()
        }
        (
            Curve::Arc {
                center: p,
                radius: r,
                ..
            },
            Curve::Arc {
                center: q,
                radius: s,
                ..
            },
        ) => {
            let d = q.sub(p);
            let distance = d.length();
            if distance <= tol && (r - s).abs() <= tol {
                return [
                    a.point(0.0),
                    a.point(0.5),
                    a.point(1.0),
                    b.point(0.0),
                    b.point(0.5),
                    b.point(1.0),
                ]
                .into_iter()
                .filter(|&point| a.contains(point) && b.contains(point))
                .collect();
            }
            if distance <= tol || distance > r + s + tol || distance < (r - s).abs() - tol {
                return Vec::new();
            }
            let along = (r * r - s * s + distance * distance) / (2.0 * distance);
            let height = (r * r - along * along).max(0.0).sqrt();
            let center = p.add(d.scale(along / distance));
            let side = P { r: -d.z, z: d.r }.scale(height / distance);
            [center.add(side), center.sub(side)]
                .into_iter()
                .filter(|&point| a.contains(point) && b.contains(point))
                .collect()
        }
    }
}
