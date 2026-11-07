//! Restore the full analytic cap after its shoulder separates from the rim.

use remus_math::curves::Circle3D;
use remus_math::surfaces::{CylindricalSurface, ToroidalSurface};
use remus_math::vec::Point3;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::explorer::{face_vertices, solid_faces};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};
use remus_topology::{FaceId, SolidId, Topology};

use super::{CapReblendApproximation, CapReblendOutcome, SupportExtension, height, live, refusal};
use crate::body_edit::EditScope;
use crate::builder_utils::add_certified_curve_edge;
use crate::trimmer::split_body_boundary_at;
use crate::{BlendEngine, BlendError, BlendFaceOrigins, BlendResult};
use remus_math::det_hash::DetHashMap;

/// Restore a full rounded cap on an owned candidate whose lower limits were restored.
///
/// The complete shoulder spring must lie strictly below the cap's lower
/// contact plane. A back-side boundary vertex supplies the periodic seam;
/// its position is kept unchanged. The transient candidate is caller-owned.
///
/// # Errors
/// Refuses missing support roles, an ambiguous seam or a shoulder that still
/// meets the rim. A refusal restores the candidate atomically.
pub fn restore_unjoined_cap_in_place(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    extension: &SupportExtension,
    permission: CapReblendApproximation,
) -> Result<CapReblendOutcome, BlendError> {
    let snapshot = remus_topology::transaction::RollbackSnapshot::capture(topo);
    let result = restore(topo, solid, cap, rims, radius, extension, permission);
    if result.is_ok() {
        snapshot.commit(topo);
    } else {
        snapshot.restore(topo);
    }
    result
}

fn restore(
    topo: &mut Topology,
    solid: SolidId,
    cap: FaceId,
    rims: &[FaceId],
    radius: f64,
    extension: &SupportExtension,
    permission: CapReblendApproximation,
) -> Result<CapReblendOutcome, BlendError> {
    if !topo.solid(solid)?.inner_shells().is_empty() {
        return Err(refusal("cap separation does not support cavity shells"));
    }
    let cap_data = topo.face(cap)?;
    let axis = cap_data
        .effective_plane_normal()
        .ok_or_else(|| refusal("separated cap is not planar"))?;
    let FaceSurface::Plane { normal, d } = cap_data.surface() else {
        return Err(refusal("separated cap changed carrier"));
    };
    let top_level = *d * normal.dot(axis);
    let level = top_level - radius;
    let FaceSurface::Torus(shoulder) = topo.face(extension.shoulder)?.surface() else {
        return Err(refusal("separated shoulder changed carrier"));
    };
    if height(shoulder.center(), axis) >= level - 1e-7 {
        return Err(refusal("shoulder still meets its fixed rim"));
    }
    let boss = extension.boss;
    let data = topo.face(boss)?;
    let FaceSurface::Cylinder(cylinder) = data.surface() else {
        return Err(refusal("separated boss changed carrier"));
    };
    if data.is_reversed() || !data.inner_wires().is_empty() || cylinder.radius() <= radius {
        return Err(refusal(
            "separated cap requires one outer cylindrical support",
        ));
    }
    let cylinder = cylinder.clone();
    let spring = topo.edge(extension.spring)?;
    let radial = |point: Point3| {
        let delta = point - cylinder.origin();
        (delta - axis * delta.dot(axis)).normalize()
    };
    let front = (radial(topo.vertex(spring.start())?.point())?
        + radial(topo.vertex(spring.end())?.point())?)
    .normalize()?;
    let vertices = face_vertices(topo, boss)?;
    let bottom = vertices
        .iter()
        .map(|&v| topo.vertex(v).map(|v| height(v.point(), axis)))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    let mut candidates = Vec::new();
    for vertex in vertices {
        let point = topo.vertex(vertex)?.point();
        if (height(point, axis) - bottom).abs() < 1e-7 && radial(point)?.dot(front) < -0.1 {
            candidates.push(vertex);
        }
    }
    // A boss whose foot a later feature runs round (a chamfer cone cut round
    // the foot) keeps no vertex on its back side. Its periodic seam then
    // stands where the wall's own parametric seam (`u = 0`) first meets its
    // boundary below the rim: on a corner's boss contact above the shoulder,
    // as the rounded part had it before its cap came down, never on the
    // foot a later band is cut round. The boundary is split there, on the
    // copy of every face that uses it.
    let mut replaced = DetHashMap::default();
    let seam = if let Some(seam) = candidates.into_iter().min_by_key(|vertex| vertex.index()) {
        seam
    } else {
        let point = parametric_seam_point(topo, solid, boss, rims, &cylinder, axis, level)?;
        let mut scope = EditScope::new(topo, solid_faces(topo, solid)?)?;
        let mut edit = scope.edit(&mut replaced);
        split_body_boundary_at(topo, &mut edit, boss, point, 1e-6)?
            .ok_or_else(|| refusal("separated cap's seam ruling misses its boss boundary"))?
    };
    let boss = live(&replaced, boss);
    let reference = radial(topo.vertex(seam)?.point())?;
    let center = cylinder.origin() + axis * (level - height(cylinder.origin(), axis));
    let major = cylinder.radius() - radius;
    let lower = topo.add_vertex(Vertex::new(center + reference * cylinder.radius(), 1e-7));
    let upper = topo.add_vertex(Vertex::new(
        center + reference * major + axis * radius,
        1e-7,
    ));
    let lower_curve = Circle3D::new_with_ref(center, axis, cylinder.radius(), reference)?;
    let upper_curve = Circle3D::new_with_ref(center + axis * radius, axis, major, reference)?;
    let lower_edge = add_certified_curve_edge(
        topo,
        lower,
        lower,
        EdgeCurve::Circle(lower_curve),
        (0.0, std::f64::consts::TAU),
    )?;
    let upper_edge = add_certified_curve_edge(
        topo,
        upper,
        upper,
        EdgeCurve::Circle(upper_curve),
        (0.0, std::f64::consts::TAU),
    )?;
    let arc = Circle3D::new_with_ref(
        center + reference * major,
        reference.cross(axis),
        radius,
        reference,
    )?;
    let arc = add_certified_curve_edge(
        topo,
        lower,
        upper,
        EdgeCurve::Circle(arc),
        (0.0, std::f64::consts::FRAC_PI_2),
    )?;
    let vertical = topo.add_edge(Edge::new(seam, lower, EdgeCurve::Line));
    let wire = topo.wire(topo.face(boss)?.outer_wire())?.edges().to_vec();
    let position = wire
        .iter()
        .position(|oe| {
            topo.edge(oe.edge())
                .is_ok_and(|e| oe.oriented_start(e) == seam)
        })
        .ok_or_else(|| refusal("back-side seam vertex is absent from its support wire"))?;
    let mut rewritten = wire[..position].to_vec();
    rewritten.extend([
        OrientedEdge::new(vertical, true),
        OrientedEdge::new(lower_edge, false),
        OrientedEdge::new(vertical, false),
    ]);
    rewritten.extend_from_slice(&wire[position..]);
    let wire = topo.add_wire(Wire::new(rewritten, true)?);
    topo.face_mut(boss)?
        .set_surface(FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
            cylinder.origin(),
            cylinder.axis(),
            cylinder.radius(),
            reference,
        )?));
    topo.set_face_boundary_wires(boss, wire, vec![])?;
    let wire = topo.add_wire(Wire::new(
        vec![OrientedEdge::new(
            upper_edge,
            !topo.face(cap)?.is_reversed(),
        )],
        true,
    )?);
    topo.set_face_boundary_wires(cap, wire, vec![])?;
    let wire = topo.add_wire(Wire::new(
        vec![
            OrientedEdge::new(lower_edge, true),
            OrientedEdge::new(arc, true),
            OrientedEdge::new(upper_edge, false),
            OrientedEdge::new(arc, false),
        ],
        true,
    )?);
    let rim = topo.add_face(Face::new(
        wire,
        vec![],
        FaceSurface::Torus(ToroidalSurface::with_axis_and_ref_dir(
            center, major, radius, axis, reference,
        )?),
    ));
    let originals = solid_faces(topo, solid)?;
    let kept: Vec<_> = originals
        .iter()
        .copied()
        .filter(|f| !rims.contains(f))
        .collect();
    let current: Vec<_> = kept.iter().map(|&f| live(&replaced, f)).collect();
    let shell = topo.add_shell(Shell::new(current.iter().copied().chain([rim]).collect())?);
    let result = topo.add_solid(Solid::new(shell, vec![]));
    let mut parents = vec![cap, extension.boss];
    parents.extend_from_slice(rims);
    Ok(CapReblendOutcome {
        corner_stations: permission.stations,
        edge_images: vec![],
        result: BlendResult {
            solid: result,
            succeeded: vec![],
            failed: vec![],
            is_partial: false,
            engine: BlendEngine::Walking,
            face_origins: Some(BlendFaceOrigins {
                survived: kept.into_iter().zip(current).collect(),
                deleted: rims.to_vec(),
                created: vec![(rim, parents)],
                created_unattributed: vec![],
            }),
        },
    })
}

/// Where the boss's parametric seam ruling (`u = 0`) meets the boss boundary
/// highest below the rim's lower contact `level`, ignoring the old rim
/// contacts.
fn parametric_seam_point(
    topo: &Topology,
    solid: SolidId,
    boss: FaceId,
    rims: &[FaceId],
    cylinder: &CylindricalSurface,
    axis: remus_math::vec::Vec3,
    level: f64,
) -> Result<Point3, BlendError> {
    const SAMPLES: u32 = 64;
    let x = cylinder.x_axis();
    let y = axis.cross(x);
    let side = |point: Point3| {
        let delta = point - cylinder.origin();
        (delta.dot(y), delta.dot(x))
    };
    let adjacency = topo.build_adjacency(solid)?;
    let mut best: Option<(f64, Point3)> = None;
    for oriented in topo.wire(topo.face(boss)?.outer_wire())?.edges() {
        let edge = oriented.edge();
        if adjacency
            .faces_for_edge(edge)
            .iter()
            .any(|f| rims.contains(f))
        {
            continue;
        }
        let data = topo.edge(edge)?;
        let (a, b) = data
            .strict_domain()
            .map_err(|error| refusal(format!("boss edge has no parameter authority: {error}")))?;
        let (start, end) = (
            topo.vertex(data.start())?.point(),
            topo.vertex(data.end())?.point(),
        );
        let at = |t: f64| data.curve().evaluate_with_endpoints(t, start, end);
        let mut previous = (a, side(at(a)).0);
        for i in 1..=SAMPLES {
            let t = a + (b - a) * f64::from(i) / f64::from(SAMPLES);
            let value = side(at(t)).0;
            if previous.1 * value <= 0.0 {
                let (mut lo, mut hi, flo) = (previous.0, t, previous.1);
                for _ in 0..80 {
                    let mid = f64::midpoint(lo, hi);
                    if side(at(mid)).0 * flo > 0.0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let point = at(f64::midpoint(lo, hi));
                let along = height(point, axis);
                if side(point).1 > 0.0
                    && along < level - 1e-7
                    && best.is_none_or(|(known, _)| along > known)
                {
                    best = Some((along, point));
                }
            }
            previous = (t, value);
        }
    }
    best.map(|(_, point)| point)
        .ok_or_else(|| refusal("separated cap's boss has no boundary on its seam ruling"))
}
