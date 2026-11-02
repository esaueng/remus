//! Cone chamfers at the ends of a thread, built into the ring directly.
//!
//! A chamfer is the cone `r_c(z)` through the end plane just inside the
//! root, opening `angle` from the axis into the thread. The thread's material
//! near that end is bounded by the smaller of the thread surface and the
//! cone, which splits the end into two zones:
//!
//! - from the end plane to where the cone reaches the root, the cone is
//!   inside the whole thread: a plain frustum (cone face, seam wall, end cap);
//! - from there to where the cone clears the crest, the cone cuts the thread:
//!   root faces stay whole, crest faces are gone, and each flank keeps the
//!   triangle below the cone.
//!
//! The exact boolean cannot cut a cone through the swept flanks (its point
//! classifier treats a swept face as a planar polygon), so the second zone is
//! the twisted extrusion `E` across it with that surgery done by hand.
//!
//! Where a flank meets the cone is closed form. A flank point at profile
//! angle `θ` has radius linear in `θ`; swept to height `z` it turns by `k·z`.
//! On the cone the radius is linear in `z`, so along the intersection the
//! radius, the height and the angle are all linear in one parameter: a
//! conical helix through the flank's root corner on the root-side plane of
//! `E` and its crest corner on the far plane. It is fitted with certified
//! cubic Hermite spans: for `g = r·e^{iφ}` with `r` and `φ` linear in `λ`,
//! `‖g⁗‖ = ω³·√(r²ω² + 16ρ²)` (`ω = dφ/dλ`, `ρ = dr/dλ`), and the height is
//! linear.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::nurbs::hermite_fit::fit_cubic_hermite;
use remus_math::surfaces::ConicalSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_circle_edge_with_ref;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::vertex::VertexId;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{Frame, ThreadError, ThreadKind};
use crate::extrude::twist::TwistedExtrusion;

/// Hermite spans allowed for one flank–cone curve.
const MAX_TRIM_SPANS: usize = 512;

/// Which plane of `E` the root-only side is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum RootSide {
    /// A start chamfer: `E` runs from the root plane up to full depth.
    Bottom,
    /// An end chamfer: `E` runs from full depth up to the root plane.
    Top,
}

/// The cone of a chamfer, in the thread's frame.
#[derive(Debug, Clone, Copy)]
pub(super) struct Cone {
    pub(super) kind: ThreadKind,
    /// The thread's crest and root radii.
    pub(super) crest: f64,
    pub(super) root: f64,
    /// Radius of the cone on the end plane: just inside the root.
    pub(super) at_end: f64,
    /// `tan` of the cone's angle from the axis.
    pub(super) slope: f64,
    /// The end plane's height along the frame axis.
    pub(super) end_z: f64,
    /// `+1` when the thread runs up the axis from the end plane (a start
    /// chamfer), `-1` when it runs down to it (an end chamfer).
    pub(super) inward: f64,
}

impl Cone {
    /// Height of the root plane: where the cone reaches the root.
    pub(super) fn root_z(&self) -> f64 {
        self.inward
            .mul_add((self.root - self.at_end).abs() / self.slope, self.end_z)
    }

    /// Height of the full plane: where the cone clears the crest.
    pub(super) fn full_z(&self) -> f64 {
        self.inward
            .mul_add((self.crest - self.at_end).abs() / self.slope, self.end_z)
    }

    /// The cone surface, its axis pointing the way it widens.
    fn surface(&self, frame: &Frame) -> Result<ConicalSurface, ThreadError> {
        // Radius grows `slope` per unit height toward the thread for an
        // external chamfer and shrinks for an internal one.
        let grows = if self.kind == ThreadKind::External {
            1.0
        } else {
            -1.0
        };
        let widening = frame.axis * (self.inward * grows);
        let apex = frame.origin + frame.axis * self.end_z - widening * (self.at_end / self.slope);
        let half_angle = FRAC_PI_2 - self.slope.atan();
        Ok(ConicalSurface::new(apex, widening, half_angle)?)
    }

    /// The outward normal of the chamfered material on the cone at `p`.
    fn outward(&self, frame: &Frame, p: Point3) -> Result<Vec3, ThreadError> {
        let offset = p - frame.origin;
        let radial = (offset - frame.axis * offset.dot(frame.axis)).normalize()?;
        // Material lies inside the cone (external) or outside it (internal);
        // the cone moves away from the end as it widens.
        let (radial_sign, grows) = if self.kind == ThreadKind::External {
            (1.0, 1.0)
        } else {
            (-1.0, -1.0)
        };
        Ok(
            (radial * radial_sign - frame.axis * (self.inward * grows * radial_sign * self.slope))
                .normalize()?,
        )
    }
}

/// What one chamfered end contributes to the ring.
pub(super) struct Zone {
    /// Faces of the ring, seam walls excluded.
    pub(super) faces: Vec<FaceId>,
    /// Seam walls ordered up the axis.
    pub(super) walls: Vec<FaceId>,
    /// Proven bound on the trim curves' distance from the exact ones.
    pub(super) trim_bound: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Role {
    Root,
    Crest,
    Flank,
}

/// An edge of the cone face's zigzag loop, with the effective direction the
/// cone face must use it in.
struct LoopEdge {
    edge: EdgeId,
    effective: bool,
}

fn effective_use(topo: &Topology, face: FaceId, edge: EdgeId) -> Result<Option<bool>, ThreadError> {
    let data = topo.face(face)?;
    for wire in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied()) {
        if let Some(oe) = topo.wire(wire)?.edges().iter().find(|oe| oe.edge() == edge) {
            return Ok(Some(oe.is_forward() != data.is_reversed()));
        }
    }
    Ok(None)
}

fn oriented_ends(topo: &Topology, oe: OrientedEdge) -> Result<(VertexId, VertexId), ThreadError> {
    let e = topo.edge(oe.edge())?;
    Ok(if oe.is_forward() {
        (e.start(), e.end())
    } else {
        (e.end(), e.start())
    })
}

/// Cut the chamfer into `e`, the twisted extrusion across the zone where the
/// cone crosses the thread, and add the frustum between its root plane and
/// the end plane. `e`'s cap on the root plane and its crest faces are
/// dropped; its cap on the full plane is left to the caller (it is the next
/// segment's profile, or the previous segment's cap).
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub(super) fn chamfer_zone(
    topo: &mut Topology,
    frame: &Frame,
    e: &TwistedExtrusion,
    cone: &Cone,
    root_side: RootSide,
    seam: f64,
    rate: f64,
    tolerance: f64,
) -> Result<Zone, ThreadError> {
    let tol = Tolerance::new().linear;
    let scale = cone.crest.max(cone.root).max(1.0);
    let near = (1e-9 * scale).max(tol);
    let invalid = |what: &str| ThreadError::invalid(format!("chamfer construction: {what}"));
    let c = &e.construction;
    let (wire, sides, seam_wall) = match cone.kind {
        ThreadKind::External => (
            &c.outer,
            &c.outer_sides,
            c.holes
                .first()
                .and_then(|h| h.sides.first())
                .copied()
                .ok_or_else(|| invalid("no seam wall"))?,
        ),
        ThreadKind::Internal => {
            let hole = c.holes.first().ok_or_else(|| invalid("no thread wire"))?;
            (
                &hole.inner,
                &hole.sides,
                c.outer_sides
                    .first()
                    .copied()
                    .ok_or_else(|| invalid("no seam wall"))?,
            )
        }
    };
    let n = sides.len();
    let (root_verts, full_verts, root_edges, full_edges) = match root_side {
        RootSide::Bottom => (
            wire.verts(),
            wire.top_verts(),
            wire.edge_ids(),
            wire.top_edge_ids(),
        ),
        RootSide::Top => (
            wire.top_verts(),
            wire.verts(),
            wire.top_edge_ids(),
            wire.edge_ids(),
        ),
    };
    let vertical = wire.vertical_edge_ids();
    if root_verts.len() != n || full_verts.len() != n || vertical.len() != n {
        return Err(invalid("mismatched construction record"));
    }
    let radius_of = |topo: &Topology, v: VertexId| -> Result<f64, ThreadError> {
        let offset = topo.vertex(v)?.point() - frame.origin;
        Ok((offset - frame.axis * offset.dot(frame.axis)).length())
    };
    let angle_of = |p: Point3| {
        let offset = p - frame.origin;
        offset.dot(frame.y).atan2(offset.dot(frame.x))
    };
    let height_of = |p: Point3| (p - frame.origin).dot(frame.axis);

    let mut faces = Vec::new();
    let mut zigzag: Vec<LoopEdge> = Vec::new();
    let mut trim_bound: f64 = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        let role = match topo.edge(wire.edge_ids()[i])?.curve() {
            EdgeCurve::Circle(circle) if (circle.radius() - cone.crest).abs() <= near => {
                Role::Crest
            }
            EdgeCurve::Circle(circle) if (circle.radius() - cone.root).abs() <= near => Role::Root,
            EdgeCurve::Circle(_) => return Err(invalid("an arc off the crest and root")),
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => Role::Flank,
        };
        let side = sides[i];
        match role {
            Role::Root => {
                faces.push(side);
                let used = effective_use(topo, side, root_edges[i])?
                    .ok_or_else(|| invalid("root face misses its arc"))?;
                zigzag.push(LoopEdge {
                    edge: root_edges[i],
                    effective: !used,
                });
            }
            Role::Crest => {
                // The cone takes the crest face's place against the full
                // plane's arc.
                let used = effective_use(topo, side, full_edges[i])?
                    .ok_or_else(|| invalid("crest face misses its arc"))?;
                zigzag.push(LoopEdge {
                    edge: full_edges[i],
                    effective: used,
                });
            }
            Role::Flank => {
                let ri = radius_of(topo, wire.verts()[i])?;
                let (root_at, crest_at) = if (ri - cone.root).abs() <= near {
                    (i, j)
                } else {
                    (j, i)
                };
                let a = root_verts[root_at];
                let cv = full_verts[crest_at];
                let (pa, pc) = (topo.vertex(a)?.point(), topo.vertex(cv)?.point());
                // The flank's own angular run, measured on one plane (the
                // vertices below `a` and `cv` on the profile plane), plus the
                // turn between the two planes.
                let (pa0, pc0) = (
                    topo.vertex(wire.verts()[root_at])?.point(),
                    topo.vertex(wire.verts()[crest_at])?.point(),
                );
                let span = (angle_of(pc0) - angle_of(pa0) + PI).rem_euclid(TAU) - PI;
                let rise = height_of(pc) - height_of(pa);
                let expected = rate.mul_add(rise, span);
                let raw = angle_of(pc) - angle_of(pa);
                let turn = raw + TAU * ((expected - raw) / TAU).round();
                let (fit, bound) = conical_helix(frame, pa, pc, turn, tolerance)?;
                trim_bound = trim_bound.max(bound);
                let edge_tol = tolerance + bound + e.deviation_bound;
                let mut trim =
                    Edge::with_tolerance(a, cv, EdgeCurve::NurbsCurve(fit.clone()), Some(edge_tol));
                trim.set_trim(Some(fit.domain()));
                let trim = topo.add_edge(trim);
                // Replace the two consecutive edges the cone removes (the
                // root-plane edge and the crest-side vertical) by the trim.
                let removed = [root_edges[i], vertical[crest_at]];
                let data = topo.face(side)?.clone();
                let old = topo.wire(data.outer_wire())?.edges().to_vec();
                let m = old.len();
                let Some(first) = (0..m).find(|&k| {
                    removed.contains(&old[k].edge()) && removed.contains(&old[(k + 1) % m].edge())
                }) else {
                    return Err(invalid(
                        "flank wire does not hold the removed pair together",
                    ));
                };
                let (from, _) = oriented_ends(topo, old[first])?;
                let (_, to) = oriented_ends(topo, old[(first + 1) % m])?;
                let forward = if from == a && to == cv {
                    true
                } else if from == cv && to == a {
                    false
                } else {
                    return Err(invalid("flank corners do not match the trim"));
                };
                let mut rebuilt = Vec::with_capacity(m - 1);
                for k in 0..m {
                    if k == first {
                        rebuilt.push(OrientedEdge::new(trim, forward));
                    } else if k != (first + 1) % m {
                        rebuilt.push(old[k]);
                    }
                }
                // Keep the cyclic order starting after the inserted edge.
                let wire_id = topo.add_wire(Wire::new(rebuilt, true)?);
                let surface = data.surface().clone();
                let face = topo.add_face(if data.is_reversed() {
                    Face::new_reversed(wire_id, vec![], surface)
                } else {
                    Face::new(wire_id, vec![], surface)
                });
                faces.push(face);
                let used = forward != data.is_reversed();
                zigzag.push(LoopEdge {
                    edge: trim,
                    effective: !used,
                });
            }
        }
    }

    // The cone face: its reversed flag from the outward normal, then the
    // zigzag walked in the direction each edge must run.
    let surface = cone.surface(frame)?;
    let probe_vertex = topo.edge(zigzag[0].edge)?.start();
    let probe = topo.vertex(probe_vertex)?.point();
    let cone_surface = FaceSurface::Cone(surface);
    let (u, v) = cone_surface
        .project_point(probe)
        .ok_or_else(|| invalid("cannot project onto the cone"))?;
    let reversed = cone_surface.normal(u, v).dot(cone.outward(frame, probe)?) < 0.0;
    let mut oriented: Vec<OrientedEdge> = zigzag
        .iter()
        .map(|le| OrientedEdge::new(le.edge, le.effective != reversed))
        .collect();
    // Walk the loop from an edge that starts on the root plane.
    let start_vertex = |topo: &Topology, oe: &OrientedEdge| oriented_ends(topo, *oe).map(|p| p.0);
    let root_z = cone.root_z();
    let mut walk: Vec<OrientedEdge> = Vec::with_capacity(oriented.len());
    let first = {
        let mut found = None;
        for (k, oe) in oriented.iter().enumerate() {
            let v = start_vertex(topo, oe)?;
            if (height_of(topo.vertex(v)?.point()) - root_z).abs() <= near {
                found = Some(k);
                break;
            }
        }
        found.ok_or_else(|| invalid("no zigzag edge starts on the root plane"))?
    };
    let mut current = oriented.swap_remove(first);
    let anchor = start_vertex(topo, &current)?;
    loop {
        let (_, end) = oriented_ends(topo, current)?;
        walk.push(current);
        if end == anchor && oriented.is_empty() {
            break;
        }
        let Some(next) =
            (0..oriented.len()).find(|&k| start_vertex(topo, &oriented[k]).is_ok_and(|s| s == end))
        else {
            return Err(invalid("the cone's upper loop does not close"));
        };
        current = oriented.swap_remove(next);
    }
    // Which way the zigzag turns about the axis.
    let mut turned = 0.0;
    for oe in &walk {
        let (s, t) = oriented_ends(topo, *oe)?;
        let (ps, pt) = (topo.vertex(s)?.point(), topo.vertex(t)?.point());
        let mid = sample_mid(topo, *oe)?;
        let d1 = (angle_of(mid) - angle_of(ps) + PI).rem_euclid(TAU) - PI;
        let d2 = (angle_of(pt) - angle_of(mid) + PI).rem_euclid(TAU) - PI;
        turned += d1 + d2;
    }
    let zigzag_ccw = turned > 0.0;

    // The end circle and the seam generator up to the anchor.
    let end_center = frame.origin + frame.axis * cone.end_z;
    let anchor_point = topo.vertex(anchor)?.point();
    let radial = {
        let offset = anchor_point - frame.origin;
        (offset - frame.axis * offset.dot(frame.axis)).normalize()?
    };
    let end_circle =
        make_circle_edge_with_ref(topo, end_center, frame.axis, cone.at_end, radial, tol)?;
    let w0 = topo.edge(end_circle)?.start();
    let generator = topo.add_edge(Edge::new(w0, anchor, EdgeCurve::Line));
    // In the face's parameter rectangle the end circle runs against the
    // zigzag.
    let mut cone_wire = vec![
        OrientedEdge::new(end_circle, !zigzag_ccw),
        OrientedEdge::new(generator, true),
    ];
    cone_wire.extend(walk);
    cone_wire.push(OrientedEdge::new(generator, false));
    let cone_wire = topo.add_wire(Wire::new(cone_wire, true)?);
    let cone_face = topo.add_face(if reversed {
        Face::new_reversed(cone_wire, vec![], cone_surface)
    } else {
        Face::new(cone_wire, vec![], cone_surface)
    });
    faces.push(cone_face);
    let end_circle_effective = zigzag_ccw == reversed;

    // The frustum's seam wall between the end plane and `e`'s seam circle on
    // the root plane.
    let wall_data = topo.face(seam_wall)?.clone();
    let wall_edges = topo.wire(wall_data.outer_wire())?.edges().to_vec();
    let mut root_seam_circle = None;
    for oe in &wall_edges {
        let edge = topo.edge(oe.edge())?;
        if matches!(edge.curve(), EdgeCurve::Circle(_))
            && (height_of(topo.vertex(edge.start())?.point()) - root_z).abs() <= near
        {
            root_seam_circle = Some(*oe);
        }
    }
    let root_seam_circle =
        root_seam_circle.ok_or_else(|| invalid("no seam circle on the root plane"))?;
    let root_seam_effective = root_seam_circle.is_forward() != wall_data.is_reversed();
    let end_seam_circle =
        make_circle_edge_with_ref(topo, end_center, frame.axis, seam, frame.x, tol)?;
    let seam_top = topo.edge(root_seam_circle.edge())?.start();
    let seam_bottom = topo.edge(end_seam_circle)?.start();
    let seam_line = topo.add_edge(Edge::new(seam_bottom, seam_top, EdgeCurve::Line));
    // The frustum wall uses the root circle against `e`'s wall, and the end
    // circle the other way round about the axis.
    let root_circle_forward = root_seam_effective == wall_data.is_reversed();
    let root_circle_ccw = {
        let EdgeCurve::Circle(circle) = topo.edge(root_seam_circle.edge())?.curve() else {
            return Err(invalid("seam circle is not a circle"));
        };
        (circle.normal().dot(frame.axis) > 0.0) == root_circle_forward
    };
    let end_circle_forward = !root_circle_ccw;
    let frustum_wall_edges = vec![
        OrientedEdge::new(end_seam_circle, end_circle_forward),
        OrientedEdge::new(seam_line, true),
        OrientedEdge::new(root_seam_circle.edge(), root_circle_forward),
        OrientedEdge::new(seam_line, false),
    ];
    let frustum_wire = topo.add_wire(Wire::new(frustum_wall_edges, true)?);
    let frustum_wall = topo.add_face(if wall_data.is_reversed() {
        Face::new_reversed(frustum_wire, vec![], wall_data.surface().clone())
    } else {
        Face::new(frustum_wire, vec![], wall_data.surface().clone())
    });
    let end_seam_effective = end_circle_forward != wall_data.is_reversed();

    // The end cap: the annulus between the cone's end circle and the seam
    // circle, facing out of the thread.
    let normal = frame.axis * (-cone.inward);
    let center = end_center;
    let plane = FaceSurface::Plane {
        normal,
        d: normal.dot(Vec3::new(center.x(), center.y(), center.z())),
    };
    let cone_loop = topo.add_wire(Wire::new(
        vec![OrientedEdge::new(end_circle, !end_circle_effective)],
        true,
    )?);
    let seam_loop = topo.add_wire(Wire::new(
        vec![OrientedEdge::new(end_seam_circle, !end_seam_effective)],
        true,
    )?);
    let (outer, inner) = match cone.kind {
        ThreadKind::External => (cone_loop, seam_loop),
        ThreadKind::Internal => (seam_loop, cone_loop),
    };
    faces.push(topo.add_face(Face::new(outer, vec![inner], plane)));

    let walls = match root_side {
        RootSide::Bottom => vec![frustum_wall, seam_wall],
        RootSide::Top => vec![seam_wall, frustum_wall],
    };
    Ok(Zone {
        faces,
        walls,
        trim_bound,
    })
}

fn sample_mid(topo: &Topology, oe: OrientedEdge) -> Result<Point3, ThreadError> {
    let e = topo.edge(oe.edge())?;
    let (sp, ep) = (
        topo.vertex(e.start())?.point(),
        topo.vertex(e.end())?.point(),
    );
    let (t0, t1) = e
        .strict_domain()
        .map_err(|error| ThreadError::invalid(format!("chamfer edge has no domain: {error}")))?;
    Ok(e.curve().evaluate_with_endpoints(0.5 * (t0 + t1), sp, ep))
}

/// The certified fit of the conical helix from `pa` to `pc` that turns
/// `turn` radians about the frame axis: radius, height and angle linear in
/// `λ ∈ [0, 1]`. Returns the curve, its ends snapped to `pa` and `pc`, and
/// its proven bound (with a rounding allowance).
fn conical_helix(
    frame: &Frame,
    pa: Point3,
    pc: Point3,
    turn: f64,
    tolerance: f64,
) -> Result<(remus_math::nurbs::NurbsCurve, f64), ThreadError> {
    let local = |p: Point3| {
        let o = p - frame.origin;
        let z = o.dot(frame.axis);
        let x = o.dot(frame.x);
        let y = o.dot(frame.y);
        (x.hypot(y), y.atan2(x), z)
    };
    let (r0, phi0, z0) = local(pa);
    let (r1, _, z1) = local(pc);
    let (rho, omega, dz) = (r1 - r0, turn, z1 - z0);
    let point = |l: f64| {
        let r = rho.mul_add(l, r0);
        let phi = omega.mul_add(l, phi0);
        let (s, c) = phi.sin_cos();
        frame.origin + frame.x * (r * c) + frame.y * (r * s) + frame.axis * dz.mul_add(l, z0)
    };
    let derivative = |l: f64| {
        let r = rho.mul_add(l, r0);
        let phi = omega.mul_add(l, phi0);
        let (s, c) = phi.sin_cos();
        frame.x * (rho * c - r * omega * s) + frame.y * (rho * s + r * omega * c) + frame.axis * dz
    };
    let r_max = r0.max(r1);
    let w = omega.abs();
    let bound = w.powi(3) * (r_max * w).hypot(4.0 * rho);
    let fit = fit_cubic_hermite(
        |l| (point(l), derivative(l)),
        0.0,
        1.0,
        bound,
        tolerance,
        MAX_TRIM_SPANS,
    )?;
    let mut points = fit.curve.control_points().to_vec();
    let last = points.len() - 1;
    points[0] = pa;
    points[last] = pc;
    let curve = remus_math::nurbs::NurbsCurve::new(
        fit.curve.degree(),
        fit.curve.knots().to_vec(),
        points,
        fit.curve.weights().to_vec(),
    )?;
    let allowance = 64.0 * f64::EPSILON * (r_max + z0.abs().max(z1.abs()));
    Ok((curve, fit.remainder_bound + allowance))
}
