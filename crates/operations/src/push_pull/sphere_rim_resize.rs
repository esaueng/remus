//! Coaxial radius change of a full-turn cylindrical wall whose rims meet
//! spherical supports along non-circular (off-axis) intersection curves.
//!
//! The cell is a cylinder boss rising from a sphere, or a blind bore sunk
//! into one, after the boolean that made it: one cylindrical wall with an
//! axis-parallel line seam, one rim chain on a sphere face and the other on a
//! sphere face or on a perpendicular planar disk. The edit preserves the
//! adjacency graph exactly. The wall receives the new coaxial carrier, every
//! sphere rim edge is re-intersected with the new cylinder from the closed
//! form `t(θ) = c_t ± sqrt(R² − |ρ·e(θ) − c⊥|²)`, every disk rim circle takes
//! the new radius, and each rim vertex slides along its own azimuth onto the
//! new rim. The re-intersected rim is a cubic interpolant of closed-form
//! samples (an approximation, certified against the closed form at every
//! half step to the model linear tolerance); everything else is exact.
//!
//! The configuration is accepted only when it can be proved: the swept
//! annulus stays strictly inside each sphere's silhouette on the original
//! branch, the wall does not collapse axially, and every other edge of the
//! body clears the swept region (a sampled bound with a Lipschitz margin).
//! The result is self-checked by validation, a closed-shell check, carrier
//! residuals of every rebuilt rim, and the tessellated volume change against
//! a quadrature of the exact swept-sleeve integral.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;

use remus_math::curves::Circle3D;
use remus_math::det_hash::{DetHashMap, DetHashSet};
use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{edge_to_face_map, solid_edges, solid_faces};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::EntityKey;
use remus_topology::solid::SolidId;
use remus_topology::vertex::VertexId;

use super::DirectEditEvolution;
use crate::OperationsError;

/// Rim support: what bounds the wall along one rim chain.
#[derive(Clone, Copy)]
enum Support {
    /// A perpendicular planar disk at axial station `t`.
    Disk { t: f64 },
    /// A sphere carrier: axial center station, perpendicular center offset
    /// (in the wall frame), radius, and the root branch (`±1`).
    Sphere {
        c_t: f64,
        c_x: f64,
        c_y: f64,
        radius: f64,
        branch: f64,
    },
}

impl Support {
    /// Axial station of the rim at radius `rho` and azimuth `theta`, or
    /// `None` outside the sphere's silhouette.
    fn station(self, rho: f64, theta: f64) -> Option<f64> {
        match self {
            Self::Disk { t } => Some(t),
            Self::Sphere {
                c_t,
                c_x,
                c_y,
                radius,
                branch,
            } => {
                let (sin, cos) = theta.sin_cos();
                let dx = rho * cos - c_x;
                let dy = rho * sin - c_y;
                let disc = (radius - dx.hypot(dy)) * (radius + dx.hypot(dy));
                (disc >= 0.0).then(|| branch.mul_add(disc.sqrt(), c_t))
            }
        }
    }
}

/// Wall frame: origin, axis and the cylinder's own reference directions.
#[derive(Clone, Copy)]
struct Frame {
    origin: Point3,
    axis: Vec3,
    x: Vec3,
    y: Vec3,
}

impl Frame {
    fn point(self, rho: f64, theta: f64, t: f64) -> Point3 {
        let (sin, cos) = theta.sin_cos();
        self.origin + self.x * (rho * cos) + self.y * (rho * sin) + self.axis * t
    }

    /// `(rho, theta, t)` of a world point.
    fn local(self, point: Point3) -> (f64, f64, f64) {
        let delta = point - self.origin;
        let x = delta.dot(self.x);
        let y = delta.dot(self.y);
        (x.hypot(y), y.atan2(x), delta.dot(self.axis))
    }
}

/// One rim chain: the neighbour face across it, its support, and its edges.
struct Chain {
    neighbor: FaceId,
    support: Support,
    edges: Vec<EdgeId>,
}

struct Plan {
    frame: Frame,
    reversed: bool,
    old_radius: f64,
    wall_edges: BTreeSet<usize>,
    wall_vertices: BTreeSet<usize>,
    /// Vertex index to the index of its rim chain.
    vertex_chain: BTreeMap<usize, usize>,
    chains: Vec<Chain>,
    linear: f64,
}

fn refusal(face: FaceId, edge: Option<EdgeId>, reason: impl Into<String>) -> OperationsError {
    remus_offset::OffsetError::TopologyChange {
        face: Some(face),
        edge,
        reason: reason.into(),
    }
    .into()
}

/// Resize `face` to `new_radius` when it is a full-turn wall whose rims meet
/// a sphere along a non-circular intersection curve.
///
/// Returns `Ok(None)` when the wall is outside this cell, so the caller's
/// other constructions (and their typed refusals) apply unchanged.
pub(super) fn resize_sphere_rimmed_wall(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    cylinder: &CylindricalSurface,
    new_radius: f64,
) -> Result<Option<DirectEditEvolution>, OperationsError> {
    let Some(plan) = plan(topo, solid, face, cylinder)? else {
        return Ok(None);
    };
    prove_clearance(topo, solid, face, &plan, new_radius)?;
    let delta_volume = swept_volume(&plan, new_radius)?;
    let before = crate::measure::solid_volume(topo, solid, super::verify_deflection(topo, solid))?;
    let source_faces = solid_faces(topo, solid)?;
    remus_topology::transaction::run_transacted(topo, |topo| {
        let copied = crate::copy::copy_solid_with_entity_map(topo, solid)?;
        rebuild(topo, face, cylinder, &plan, new_radius, &copied)?;
        let result = copied.solid;
        let report = crate::validate::validate_solid(topo, result)?;
        if !report.is_valid() {
            return Err(refusal(
                face,
                None,
                format!(
                    "sphere-rimmed resize validation failed with {} error(s)",
                    report.error_count()
                ),
            ));
        }
        super::ensure_closed_shell(topo, result, "sphere-rimmed cylindrical resize")?;
        super::ensure_volume(
            topo,
            result,
            before + delta_volume,
            "sphere-rimmed cylindrical resize",
        )?;
        let evolution = super::exact_face_evolution(
            topo,
            &source_faces,
            result,
            copied
                .face_map
                .iter()
                .map(|(&source, &target)| (source, target)),
        )?;
        let pairs = copied
            .edge_map
            .iter()
            .map(|(&source, target)| (EntityKey::edge(source), EntityKey::edge(target.index())))
            .chain(copied.vertex_map.iter().map(|(&source, target)| {
                (EntityKey::vertex(source), EntityKey::vertex(target.index()))
            }))
            .collect();
        Ok(Some((
            super::MoveFacesResult {
                solid: result,
                evolution,
            },
            pairs,
        )))
    })
}

/// Classify the wall. `None` means "not this cell".
#[allow(clippy::too_many_lines)]
fn plan(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    cylinder: &CylindricalSurface,
) -> Result<Option<Plan>, OperationsError> {
    let tol = Tolerance::new();
    let Ok(axis) = cylinder.axis().normalize() else {
        return Ok(None);
    };
    let frame = Frame {
        origin: cylinder.origin(),
        axis,
        x: cylinder.x_axis(),
        y: cylinder.y_axis(),
    };
    let face_data = topo.face(face)?;
    let edge_faces = edge_to_face_map(topo, solid)?;
    let mut wall_edges = BTreeSet::new();
    let mut seam = false;
    let mut by_neighbor: BTreeMap<usize, (FaceId, Vec<EdgeId>)> = BTreeMap::new();
    for wire in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        for oriented in topo.wire(wire)?.edges() {
            let edge = oriented.edge();
            if !wall_edges.insert(edge.index()) {
                continue;
            }
            let Some(uses) = edge_faces.get(&edge.index()) else {
                return Ok(None);
            };
            if uses.len() != 2 {
                return Ok(None);
            }
            if uses[0] == face && uses[1] == face {
                let data = topo.edge(edge)?;
                let start = topo.vertex(data.start())?.point();
                let end = topo.vertex(data.end())?.point();
                if !matches!(data.curve(), EdgeCurve::Line)
                    || (end - start).cross(axis).length() > tol.linear
                {
                    return Ok(None);
                }
                seam = true;
                continue;
            }
            let Some(&neighbor) = uses.iter().find(|candidate| **candidate != face) else {
                return Ok(None);
            };
            by_neighbor
                .entry(neighbor.index())
                .or_insert_with(|| (neighbor, Vec::new()))
                .1
                .push(edge);
        }
    }
    // The cell claims only off-axis sphere rims: some sphere rim edge must be
    // a non-circular intersection curve. Coaxial sphere rims are circles and
    // keep their established constructions.
    let mut claims = false;
    for (neighbor, edges) in by_neighbor.values() {
        if matches!(topo.face(*neighbor)?.surface(), FaceSurface::Sphere(_)) {
            for &edge in edges {
                claims |= !matches!(topo.edge(edge)?.curve(), EdgeCurve::Circle(_));
            }
        }
    }
    if !seam || !claims || by_neighbor.len() != 2 {
        return Ok(None);
    }

    let mut scale = cylinder.radius().max(1.0);
    for value in frame.origin.0 {
        scale = scale.max(value.abs());
    }
    let linear = tol.linear.max(scale * tol.relative);
    let mut chains = Vec::new();
    let mut vertex_chain = BTreeMap::new();
    let mut wall_vertices = BTreeSet::new();
    for (neighbor, edges) in by_neighbor.into_values() {
        let index = chains.len();
        for &edge in &edges {
            let data = topo.edge(edge)?;
            for vertex in [data.start(), data.end()] {
                wall_vertices.insert(vertex.index());
                if vertex_chain
                    .insert(vertex.index(), index)
                    .is_some_and(|c| c != index)
                {
                    return Ok(None);
                }
            }
        }
        let support = match topo.face(neighbor)?.surface() {
            FaceSurface::Sphere(sphere) => {
                match sphere_support(topo, frame, sphere, &edges, linear)? {
                    Some(support) => support,
                    None => return Ok(None),
                }
            }
            FaceSurface::Plane { .. } => {
                match disk_support(topo, neighbor, frame, cylinder.radius(), &edges, linear)? {
                    Some(support) => support,
                    None => return Ok(None),
                }
            }
            FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Torus(_)
            | FaceSurface::Nurbs(_) => return Ok(None),
        };
        chains.push(Chain {
            neighbor,
            support,
            edges,
        });
    }
    // Every rim vertex is private to the wall's boundary: no edge outside the
    // wall meets it, so sliding it along its rim cannot drag another edge.
    for edge in solid_edges(topo, solid)? {
        if wall_edges.contains(&edge.index()) {
            continue;
        }
        let data = topo.edge(edge)?;
        if wall_vertices.contains(&data.start().index())
            || wall_vertices.contains(&data.end().index())
        {
            return Ok(None);
        }
    }
    Ok(Some(Plan {
        frame,
        reversed: face_data.is_reversed(),
        old_radius: cylinder.radius(),
        wall_edges,
        wall_vertices,
        vertex_chain,
        chains,
        linear,
    }))
}

/// Sample points of an edge over its authoritative domain, endpoints included.
#[allow(clippy::cast_precision_loss)]
fn edge_samples(
    topo: &Topology,
    edge: EdgeId,
    count: usize,
) -> Result<Vec<Point3>, OperationsError> {
    let data = topo.edge(edge)?;
    let start = topo.vertex(data.start())?.point();
    let end = topo.vertex(data.end())?.point();
    let (lo, hi) = crate::authoritative_edge_domain(data, "sphere-rim edge samples")?;
    Ok((0..=count)
        .map(|index| {
            let parameter = lo + (hi - lo) * index as f64 / count as f64;
            data.curve().evaluate_with_endpoints(parameter, start, end)
        })
        .collect())
}

fn sphere_support(
    topo: &Topology,
    frame: Frame,
    sphere: &SphericalSurface,
    edges: &[EdgeId],
    linear: f64,
) -> Result<Option<Support>, OperationsError> {
    let center = frame.local(sphere.center());
    let (sin, cos) = center.1.sin_cos();
    let (c_x, c_y, c_t) = (center.0 * cos, center.0 * sin, center.2);
    let mut branch: Option<bool> = None;
    for &edge in edges {
        for point in edge_samples(topo, edge, 16)? {
            let (_, _, t) = frame.local(point);
            let offset = t - c_t;
            // A rim at the sphere's silhouette has an ill-conditioned branch.
            if offset.abs() <= 1e3 * linear {
                return Ok(None);
            }
            let upper = offset > 0.0;
            if branch.is_some_and(|known| known != upper) {
                return Ok(None);
            }
            branch = Some(upper);
        }
    }
    Ok(Some(Support::Sphere {
        c_t,
        c_x,
        c_y,
        radius: sphere.radius(),
        branch: if branch == Some(false) { -1.0 } else { 1.0 },
    }))
}

fn disk_support(
    topo: &Topology,
    neighbor: FaceId,
    frame: Frame,
    radius: f64,
    edges: &[EdgeId],
    linear: f64,
) -> Result<Option<Support>, OperationsError> {
    let neighbor_data = topo.face(neighbor)?;
    let [edge] = edges else {
        return Ok(None);
    };
    if !neighbor_data.inner_wires().is_empty() {
        return Ok(None);
    }
    let wire = topo.wire(neighbor_data.outer_wire())?;
    let [oriented] = wire.edges() else {
        return Ok(None);
    };
    let data = topo.edge(*edge)?;
    let EdgeCurve::Circle(circle) = data.curve() else {
        return Ok(None);
    };
    let (rho, _, t) = frame.local(circle.center());
    if oriented.edge() != *edge
        || data.start() != data.end()
        || rho > linear
        || (circle.radius() - radius).abs() > linear
        || circle.normal().cross(frame.axis).length() > Tolerance::new().angular
    {
        return Ok(None);
    }
    Ok(Some(Support::Disk { t }))
}

/// Swept-region clearance: the new rim stays inside every sphere silhouette,
/// the wall keeps positive axial extent, and no edge off the wall's boundary
/// enters the annular region swept between the two radii.
fn prove_clearance(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
    plan: &Plan,
    new_radius: f64,
) -> Result<(), OperationsError> {
    let lo = plan.old_radius.min(new_radius);
    let hi = plan.old_radius.max(new_radius);
    for chain in &plan.chains {
        if let Support::Sphere {
            c_x, c_y, radius, ..
        } = chain.support
            && c_x.hypot(c_y) + hi >= radius - 1e3 * plan.linear
        {
            return Err(refusal(
                face,
                chain.edges.first().copied(),
                "resized wall would reach the sphere support's silhouette",
            ));
        }
    }
    // Axial bounds of the swept region over a dense azimuth/radius grid.
    let (mut t_min, mut t_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for step in 0..=8 {
        let rho = lo + (hi - lo) * f64::from(step) / 8.0;
        for index in 0..256 {
            let theta = TAU * f64::from(index) / 256.0;
            let mut stations = [0.0; 2];
            for (slot, chain) in stations.iter_mut().zip(&plan.chains) {
                *slot = chain
                    .support
                    .station(rho, theta)
                    .ok_or_else(|| refusal(face, None, "swept region leaves a sphere support"))?;
            }
            let height = (stations[0] - stations[1]).abs();
            if height <= 1e3 * plan.linear || {
                // The two rims must keep their axial order across the sweep.
                let original = plan.chains[0]
                    .support
                    .station(plan.old_radius, theta)
                    .zip(plan.chains[1].support.station(plan.old_radius, theta))
                    .map(|(a, b)| a > b);
                original != Some(stations[0] > stations[1])
            } {
                return Err(refusal(
                    face,
                    None,
                    "resized wall collapses between its rims",
                ));
            }
            t_min = t_min.min(stations[0].min(stations[1]));
            t_max = t_max.max(stations[0].max(stations[1]));
        }
    }
    for edge in solid_edges(topo, solid)? {
        if plan.wall_edges.contains(&edge.index()) {
            continue;
        }
        let samples = edge_samples(topo, edge, 128)?;
        let spacing = samples
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).length())
            .fold(0.0_f64, f64::max);
        // Distance to the swept region is 1-Lipschitz along the curve; the
        // margin covers the gap between samples (arc ≈ chord here).
        let margin = 0.55 * spacing + 1e3 * plan.linear;
        for point in samples {
            let (rho, _, t) = plan.frame.local(point);
            if rho > lo - margin && rho < hi + margin && t > t_min - margin && t < t_max + margin {
                return Err(refusal(
                    face,
                    Some(edge),
                    "resized wall sweeps across a source edge outside its boundary",
                ));
            }
        }
    }
    Ok(())
}

/// Gauss–Legendre nodes and weights on `[-1, 1]` (16 points).
const GAUSS_16: [(f64, f64); 8] = [
    (0.095_012_509_837_637_44, 0.189_450_610_455_068_5),
    (0.281_603_550_779_258_9, 0.182_603_415_044_923_6),
    (0.458_016_777_657_227_4, 0.169_156_519_395_002_5),
    (0.617_876_244_402_643_8, 0.149_595_988_816_576_7),
    (0.755_404_408_355_003, 0.124_628_971_255_533_9),
    (0.865_631_202_387_831_7, 0.095_158_511_682_492_78),
    (0.944_575_023_073_232_6, 0.062_253_523_938_647_89),
    (0.989_400_934_991_649_9, 0.027_152_459_411_754_09),
];

/// Signed volume change: `±∫θ∫ρ (t_top − t_bottom) ρ dρ dθ` over the swept
/// annulus, by 16-point Gauss–Legendre in ρ and the periodic trapezoid rule
/// in θ (spectrally accurate for this smooth periodic integrand). Positive
/// adds material (a growing boss), negative removes it (a growing bore).
fn swept_volume(plan: &Plan, new_radius: f64) -> Result<f64, OperationsError> {
    let half = 0.5 * (new_radius - plan.old_radius);
    let middle = 0.5 * (new_radius + plan.old_radius);
    let steps = 512_u32;
    let mut total = 0.0;
    for index in 0..steps {
        let theta = TAU * f64::from(index) / f64::from(steps);
        let mut radial = 0.0;
        for (node, weight) in GAUSS_16 {
            for rho in [middle + half * node, middle - half * node] {
                let a = plan.chains[0].support.station(rho, theta);
                let b = plan.chains[1].support.station(rho, theta);
                let (Some(a), Some(b)) = (a, b) else {
                    return Err(OperationsError::InvalidInput {
                        reason: "swept sleeve leaves a sphere support".into(),
                    });
                };
                radial += weight * (a - b).abs() * rho;
            }
        }
        total += radial * half;
    }
    let sleeve = total * TAU / f64::from(steps);
    Ok(if plan.reversed { -sleeve } else { sleeve })
}

#[allow(clippy::too_many_lines)]
fn rebuild(
    topo: &mut Topology,
    face: FaceId,
    cylinder: &CylindricalSurface,
    plan: &Plan,
    new_radius: f64,
    copied: &crate::copy::CopiedSolidEntities,
) -> Result<(), OperationsError> {
    let edge = |index: usize| {
        copied
            .edge_map
            .get(&index)
            .copied()
            .ok_or_else(|| refusal(face, None, "copied solid lost a wall boundary edge"))
    };
    let vertex = |index: usize| -> Result<VertexId, OperationsError> {
        copied
            .vertex_map
            .get(&index)
            .copied()
            .ok_or_else(|| refusal(face, None, "copied solid lost a wall boundary vertex"))
    };
    let wall = copied
        .face_map
        .get(&face.index())
        .copied()
        .ok_or_else(|| refusal(face, None, "copied solid lost the selected wall"))?;

    // Source rim directions are read before any vertex moves.
    let mut sweeps = DetHashMap::default();
    for chain in &plan.chains {
        if !matches!(chain.support, Support::Sphere { .. }) {
            continue;
        }
        for &source in &chain.edges {
            let data = topo.edge(source)?;
            let start = topo.vertex(data.start())?.point();
            let end = topo.vertex(data.end())?.point();
            let (lo, hi) = crate::authoritative_edge_domain(data, "sphere-rim edge samples")?;
            let probe = data
                .curve()
                .evaluate_with_endpoints(lo + 0.25 * (hi - lo), start, end);
            let theta_a = plan.frame.local(start).1;
            let theta_b = plan.frame.local(end).1;
            let theta_p = plan.frame.local(probe).1;
            let forward = (theta_b - theta_a).rem_euclid(TAU);
            let forward = if data.start() == data.end() || forward <= 1e-12 {
                TAU
            } else {
                forward
            };
            let sweep = if (theta_p - theta_a).rem_euclid(TAU) < forward {
                forward
            } else {
                forward - TAU
            };
            sweeps.insert(source.index(), (theta_a, sweep));
        }
    }

    // Slide every rim vertex along its azimuth onto the new rim.
    for &source in &plan.wall_vertices {
        let point = topo
            .vertex(
                topo.vertex_id_from_index(source)
                    .ok_or_else(|| refusal(face, None, "wall vertex is unavailable"))?,
            )?
            .point();
        let theta = plan.frame.local(point).1;
        let chain = plan.vertex_chain[&source];
        let t = plan.chains[chain]
            .support
            .station(new_radius, theta)
            .ok_or_else(|| refusal(face, None, "rim vertex leaves its sphere support"))?;
        let target = vertex(source)?;
        topo.vertex_mut(target)?
            .set_point(plan.frame.point(new_radius, theta, t));
    }

    for chain in &plan.chains {
        for &source in &chain.edges {
            let target = edge(source.index())?;
            match chain.support {
                Support::Disk { .. } => {
                    let data = topo.edge(target)?;
                    let EdgeCurve::Circle(circle) = data.curve() else {
                        return Err(refusal(face, Some(source), "disk rim is not a circle"));
                    };
                    let trim = data.trim();
                    let circle = Circle3D::with_axes(
                        circle.center(),
                        circle.normal(),
                        new_radius,
                        circle.u_axis(),
                        circle.v_axis(),
                    )?;
                    let data = topo.edge_mut(target)?;
                    data.set_curve(EdgeCurve::Circle(circle));
                    data.set_trim(trim);
                }
                Support::Sphere { .. } => {
                    let (theta_a, sweep) = sweeps[&source.index()];
                    let curve = fit_rim(
                        face,
                        source,
                        chain.support,
                        plan,
                        new_radius,
                        theta_a + sweep,
                        -sweep,
                    )?;
                    // Fitted from the end vertex back to the start, with a
                    // descending trim: the parameter convention the boolean
                    // section edges this replaces already carry.
                    let data = topo.edge_mut(target)?;
                    data.set_curve(EdgeCurve::NurbsCurve(curve));
                    data.set_trim(Some((1.0, 0.0)));
                }
            }
        }
    }

    let replacement = CylindricalSurface::with_ref_dir(
        cylinder.origin(),
        cylinder.axis(),
        new_radius,
        cylinder.x_axis(),
    )?;
    topo.face_mut(wall)?
        .set_surface(FaceSurface::Cylinder(replacement));

    let mut affected = vec![wall];
    for chain in &plan.chains {
        affected.push(
            copied
                .face_map
                .get(&chain.neighbor.index())
                .copied()
                .ok_or_else(|| refusal(face, None, "copied solid lost a rim support"))?,
        );
    }
    // Only the rebuilt boundary changes, and only coedges that carried a
    // stored p-curve get a fresh one: a support's other coedges (a sphere's
    // equator, say) and coedges whose p-curve the source left to be derived
    // keep exactly the source's state.
    let mut stored = DetHashSet::default();
    for &index in &plan.wall_edges {
        let copy = edge(index)?;
        for &face_id in &affected {
            for forward in [true, false] {
                if topo.pcurve_oriented(copy, face_id, forward).is_some() {
                    stored.insert((copy, face_id, forward));
                }
            }
        }
    }
    crate::replace_surface::register_fresh_pcurves_where(
        topo,
        &affected,
        |edge, face, forward| stored.contains(&(edge, face, forward)),
    )?;

    // Every rebuilt rim edge lies on both carriers.
    for chain in &plan.chains {
        let support = chain.support;
        for &source in &chain.edges {
            for point in edge_samples(topo, edge(source.index())?, 64)? {
                let (rho, theta, t) = plan.frame.local(point);
                let on_support = support
                    .station(new_radius, theta)
                    .map_or(f64::INFINITY, |s| (s - t).abs());
                if (rho - new_radius).abs() > 10.0 * plan.linear || on_support > 10.0 * plan.linear
                {
                    return Err(refusal(
                        face,
                        Some(source),
                        format!(
                            "rebuilt rim is off its carriers by {:.3e}",
                            (rho - new_radius).abs().max(on_support)
                        ),
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Cubic interpolant of the closed-form rim from azimuth `theta_a` through
/// `sweep`, refined until every half-step deviation is within the model
/// linear tolerance.
#[allow(clippy::cast_precision_loss)]
fn fit_rim(
    face: FaceId,
    source: EdgeId,
    support: Support,
    plan: &Plan,
    radius: f64,
    theta_a: f64,
    sweep: f64,
) -> Result<remus_math::nurbs::curve::NurbsCurve, OperationsError> {
    let rim = |fraction: f64| -> Result<Point3, OperationsError> {
        let theta = sweep.mul_add(fraction, theta_a);
        let t = support
            .station(radius, theta)
            .ok_or_else(|| refusal(face, Some(source), "rim leaves its sphere support"))?;
        Ok(plan.frame.point(radius, theta, t))
    };
    // At least 64 spans: a coarser rim is sampled so sparsely by the
    // tessellator's span-driven NURBS sampler that a small-scale (1e-3)
    // body's sphere face mesh no longer shares the wall's rim polyline.
    let mut segments = 64_usize;
    let mut worst = f64::INFINITY;
    while segments <= 4096 {
        let params: Vec<f64> = (0..=segments).map(|i| i as f64 / segments as f64).collect();
        let points = params
            .iter()
            .map(|&u| rim(u))
            .collect::<Result<Vec<_>, _>>()?;
        let curve = remus_math::nurbs::fitting::interpolate_with_params(&points, 3, &params)?;
        worst = 0.0;
        for index in 0..segments {
            let u = (index as f64 + 0.5) / segments as f64;
            let fitted = remus_math::traits::ParametricCurve::evaluate(&curve, u);
            worst = worst.max((fitted - rim(u)?).length());
        }
        if worst <= plan.linear {
            return Ok(curve);
        }
        segments *= 2;
    }
    Err(refusal(
        face,
        Some(source),
        format!("re-intersected rim fit deviates by {worst:.3e}"),
    ))
}
