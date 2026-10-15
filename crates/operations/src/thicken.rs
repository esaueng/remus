//! Thicken a face into a solid by offsetting it along its own normal field.
//!
//! A planar face extrudes along its outward normal. A curved analytic face
//! (cylinder, cone, sphere, torus) becomes the shell between the face and its
//! exact offset, closed by walls swept along the normals of its boundary.

use std::collections::BTreeMap;

use remus_math::curves::Circle3D;
use remus_math::nurbs::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::shell::Shell;
use remus_topology::solid::{Solid, SolidId};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;
use crate::extrude::extrude;

/// Thicken a face into a solid along its outward normal.
///
/// The outward normal is the face's own: the surface normal, negated on a
/// reversed face. Positive `thickness` grows the solid on that side of the
/// face, negative on the other.
///
/// - A planar face (or a NURBS face whose control net is planar) extrudes
///   along that normal.
/// - A cylinder, cone, sphere or torus face becomes the region between the
///   face and its exact offset by `thickness`, closed by walls swept along
///   the normals of its boundary: a plane, cylinder or cone per edge. Its
///   boundary must be lines and circles whose offsets are again lines and
///   circles (rulings, parallels, meridians, and any circle on a sphere);
///   both are certified on samples.
///
/// # Errors
///
/// Returns [`OperationsError::InvalidInput`] if `thickness` is zero or not
/// finite, and [`OperationsError::Unsupported`] for a curved NURBS face, a
/// boundary edge without an exact offset or wall, or an offset that
/// collapses the surface (a radius reaching zero, a cone crossing its apex,
/// a torus tube reaching its axis).
pub fn thicken(
    topo: &mut Topology,
    face: FaceId,
    thickness: f64,
) -> Result<SolidId, OperationsError> {
    let tol = Tolerance::new();
    if !thickness.is_finite() || tol.approx_eq(thickness, 0.0) {
        return Err(OperationsError::InvalidInput {
            reason: format!("thickness must be finite and non-zero, got {thickness}"),
        });
    }

    let face_data = topo.face(face)?;
    let side = if face_data.is_reversed() { -1.0 } else { 1.0 };
    let plane_normal = match face_data.surface() {
        FaceSurface::Plane { normal, .. } => Some(*normal),
        FaceSurface::Nurbs(nurbs) => Some(planar_nurbs_normal(nurbs, tol)?),
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => None,
    };
    if let Some(normal) = plane_normal {
        let outward = normal * side;
        return if thickness > 0.0 {
            extrude(topo, face, outward, thickness)
        } else {
            extrude(topo, face, -outward, -thickness)
        };
    }
    remus_topology::transaction::run_transacted(topo, |topo| {
        thicken_curved(topo, face, thickness, tol)
    })
}

/// The normal of a NURBS face whose control net lies in one plane, which the
/// surface then lies in too. A curved NURBS face has no exact offset.
fn planar_nurbs_normal(nurbs: &NurbsSurface, tol: Tolerance) -> Result<Vec3, OperationsError> {
    let (u0, u1) = nurbs.domain_u();
    let (v0, v1) = nurbs.domain_v();
    let (u, v) = (f64::midpoint(u0, u1), f64::midpoint(v0, v1));
    let normal = nurbs
        .normal(u, v)
        .map_err(|e| OperationsError::InvalidInput {
            reason: format!("NURBS normal computation failed: {e}"),
        })?;
    let origin = nurbs.evaluate(u, v);
    let planar = nurbs
        .control_points()
        .iter()
        .flatten()
        .all(|&p| normal.dot(p - origin).abs() <= tol.linear);
    if planar {
        Ok(normal)
    } else {
        Err(unsupported(
            "a curved NURBS face has no exact offset to thicken to".into(),
        ))
    }
}

fn unsupported(reason: String) -> OperationsError {
    OperationsError::Unsupported {
        operation: "thicken",
        reason,
    }
}

/// One boundary edge of the source face with its geometry.
struct SourceEdge {
    start: VertexId,
    end: VertexId,
    curve: EdgeCurve,
    trim: Option<(f64, f64)>,
    domain: (f64, f64),
    tolerance: Option<f64>,
}

/// The exact offset of a boundary edge.
enum OffsetCurve {
    Line,
    Circle(Circle3D),
}

/// The surface of the wall swept along the normals of one boundary edge.
enum WallSurface {
    Plane {
        normal: Vec3,
        d: f64,
    },
    Curved {
        surface: FaceSurface,
        reversed: bool,
    },
}

/// One wall: the edge use it stands on and its surface. `annulus` marks a
/// planar wall between two closed concentric circles, bounded by the two
/// circles alone rather than a seam.
struct Wall {
    wire: usize,
    position: usize,
    surface: WallSurface,
    annulus: bool,
}

/// The shell between a curved analytic face and its offset by `thickness`.
#[allow(clippy::too_many_lines)]
fn thicken_curved(
    topo: &mut Topology,
    face: FaceId,
    thickness: f64,
    tol: Tolerance,
) -> Result<SolidId, OperationsError> {
    let face_data = topo.face(face)?;
    let reversed = face_data.is_reversed();
    let surface = face_data.surface().clone();
    let wire_ids: Vec<WireId> = std::iter::once(face_data.outer_wire())
        .chain(face_data.inner_wires().iter().copied())
        .collect();
    // The offset along the CARRIER normal; the face's outward normal is the
    // carrier's negated on a reversed face.
    let delta = if reversed { -thickness } else { thickness };
    let side = if reversed { -1.0 } else { 1.0 };
    let grow = thickness > 0.0;
    let offset_surface = offset_carrier(&surface, delta)?;
    let check_tol = tol.linear;

    let offset_point = |p: Point3| -> Result<Point3, OperationsError> {
        let (u, v) = surface
            .project_point(p)
            .ok_or_else(|| unsupported("boundary point has no surface parameter".into()))?;
        let q = p + surface.normal(u, v) * delta;
        let (ou, ov) = offset_surface
            .project_point(q)
            .ok_or_else(|| unsupported("offset point has no surface parameter".into()))?;
        let on_offset = offset_surface.evaluate(ou, ov).map(|s| (s - q).length());
        if on_offset.is_none_or(|miss| !miss.is_finite() || miss > check_tol) {
            return Err(unsupported(format!(
                "offsetting the boundary by {thickness} leaves the offset surface (the \
                 offset folds the face)"
            )));
        }
        Ok(q)
    };
    let outward_at = |p: Point3| -> Result<Vec3, OperationsError> {
        let (u, v) = surface
            .project_point(p)
            .ok_or_else(|| unsupported("boundary point has no surface parameter".into()))?;
        Ok(surface.normal(u, v) * side)
    };

    // Snapshot the boundary.
    let mut uses: Vec<Vec<OrientedEdge>> = Vec::with_capacity(wire_ids.len());
    let mut sources: BTreeMap<EdgeId, SourceEdge> = BTreeMap::new();
    let mut use_count: BTreeMap<EdgeId, usize> = BTreeMap::new();
    let mut positions: BTreeMap<VertexId, (Point3, f64)> = BTreeMap::new();
    for &wire_id in &wire_ids {
        let wire = topo.wire(wire_id)?;
        if wire.edges().is_empty() {
            return Err(unsupported("face has an empty boundary wire".into()));
        }
        // In effective order: a reversed face reads its wires the other way
        // round.
        uses.push(stored(wire.edges().to_vec(), reversed));
        for oriented in wire.edges() {
            *use_count.entry(oriented.edge()).or_default() += 1;
            if sources.contains_key(&oriented.edge()) {
                continue;
            }
            let edge = topo.edge(oriented.edge())?;
            for vertex in [edge.start(), edge.end()] {
                let data = topo.vertex(vertex)?;
                positions.insert(vertex, (data.point(), data.tolerance()));
            }
            sources.insert(
                oriented.edge(),
                SourceEdge {
                    start: edge.start(),
                    end: edge.end(),
                    curve: edge.curve().clone(),
                    trim: edge.trim(),
                    domain: crate::authoritative_edge_domain(edge, "thicken boundary")?,
                    tolerance: edge.tolerance(),
                },
            );
        }
    }

    // Exact offsets of every vertex and edge, certified before anything is
    // allocated.
    let mut offset_positions: BTreeMap<VertexId, Point3> = BTreeMap::new();
    for (&vertex, &(p, _)) in &positions {
        // Measured along the generator: the axial height is only v·sin a,
        // which a shallow cone keeps under tolerance far from its apex.
        if let FaceSurface::Cone(cone) = &surface
            && cone.project_point(p).1 <= check_tol
        {
            return Err(unsupported(
                "a cone face reaching its apex has no offset normal there".into(),
            ));
        }
        offset_positions.insert(vertex, offset_point(p)?);
    }
    let mut offsets: BTreeMap<EdgeId, OffsetCurve> = BTreeMap::new();
    for (&edge, source) in &sources {
        let start = positions[&source.start].0;
        let end = positions[&source.end].0;
        let curve = match &source.curve {
            // A closed line seam (a torus's fundamental polygon) is a point,
            // copied as such; seams carry no wall.
            EdgeCurve::Line if source.start == source.end && use_count[&edge] > 1 => {
                OffsetCurve::Line
            }
            EdgeCurve::Line => {
                let (a, b) = (
                    offset_positions[&source.start],
                    offset_positions[&source.end],
                );
                if (b - a).length() <= check_tol || (end - start).length() <= check_tol {
                    return Err(unsupported("degenerate line boundary edge".into()));
                }
                for s in [0.25, 0.5, 0.75] {
                    let q = offset_point(start + (end - start) * s)?;
                    let along = (q - a).dot(b - a) / (b - a).length_squared();
                    if (q - (a + (b - a) * along)).length() > check_tol {
                        return Err(unsupported(
                            "a line boundary edge that is not a ruling has no straight offset"
                                .into(),
                        ));
                    }
                }
                OffsetCurve::Line
            }
            EdgeCurve::Circle(circle) => OffsetCurve::Circle(offset_circle(
                circle,
                source.domain,
                &offset_point,
                check_tol,
            )?),
            other => {
                return Err(unsupported(format!(
                    "`{}` boundary edges of a curved face have no exact offset",
                    other.type_tag()
                )));
            }
        };
        offsets.insert(edge, curve);
    }

    // Walls on every non-seam edge use: a seam is used twice by the face, and
    // its two walls would cancel.
    let mut walls = Vec::new();
    for (wire, wire_uses) in uses.iter().enumerate() {
        for (position, oriented) in wire_uses.iter().enumerate() {
            if use_count[&oriented.edge()] > 1 {
                continue;
            }
            let source = &sources[&oriented.edge()];
            let start = positions[&source.start].0;
            let end = positions[&source.end].0;
            let (t0, t1) = source.domain;
            let mid = f64::midpoint(t0, t1);
            let (point, mut tangent) = match &source.curve {
                EdgeCurve::Line => (start + (end - start) * 0.5, end - start),
                EdgeCurve::Circle(circle) => (
                    circle.evaluate(mid),
                    circle.tangent(mid) * (t1 - t0).signum(),
                ),
                EdgeCurve::Ellipse(_)
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Parabola(_)
                | EdgeCurve::Hyperbola(_) => unreachable_curve()?,
            };
            if !oriented.is_forward() {
                tangent = -tangent;
            }
            // Away from the face's material along its boundary, and with it
            // away from the solid's.
            let out = tangent.cross(outward_at(point)?);
            let (surface, annulus) = match &offsets[&oriented.edge()] {
                OffsetCurve::Line => {
                    let a = offset_positions[&source.start];
                    let b = offset_positions[&source.end];
                    (line_wall(start, end, a, b, out, check_tol)?, false)
                }
                OffsetCurve::Circle(offset) => {
                    let EdgeCurve::Circle(circle) = &source.curve else {
                        return unreachable_curve();
                    };
                    circle_wall(
                        circle,
                        offset,
                        source.domain,
                        source.start == source.end,
                        out,
                        check_tol,
                    )?
                }
            };
            walls.push(Wall {
                wire,
                position,
                surface,
                annulus,
            });
        }
    }

    // Allocate: copied and offset vertices and edges, rails, faces.
    let mut bottom_vertex: BTreeMap<VertexId, VertexId> = BTreeMap::new();
    let mut top_vertex: BTreeMap<VertexId, VertexId> = BTreeMap::new();
    for (&vertex, &(p, vertex_tol)) in &positions {
        bottom_vertex.insert(vertex, topo.add_vertex(Vertex::new(p, vertex_tol)));
        top_vertex.insert(
            vertex,
            topo.add_vertex(Vertex::new(offset_positions[&vertex], vertex_tol)),
        );
    }
    let mut bottom_edge: BTreeMap<EdgeId, EdgeId> = BTreeMap::new();
    let mut top_edge: BTreeMap<EdgeId, EdgeId> = BTreeMap::new();
    for (&edge, source) in &sources {
        let mut copy = Edge::with_tolerance(
            bottom_vertex[&source.start],
            bottom_vertex[&source.end],
            source.curve.clone(),
            source.tolerance,
        );
        copy.set_trim(source.trim);
        bottom_edge.insert(edge, topo.add_edge(copy));
        let curve = match &offsets[&edge] {
            OffsetCurve::Line => EdgeCurve::Line,
            OffsetCurve::Circle(circle) => EdgeCurve::Circle(circle.clone()),
        };
        let mut offset = Edge::with_tolerance(
            top_vertex[&source.start],
            top_vertex[&source.end],
            curve,
            source.tolerance,
        );
        offset.set_trim(source.trim);
        top_edge.insert(edge, topo.add_edge(offset));
    }

    let mut rails: BTreeMap<VertexId, EdgeId> = BTreeMap::new();
    let mut rail = |topo: &mut Topology, vertex: VertexId| -> EdgeId {
        *rails.entry(vertex).or_insert_with(|| {
            topo.add_edge(Edge::new(
                bottom_vertex[&vertex],
                top_vertex[&vertex],
                EdgeCurve::Line,
            ))
        })
    };

    // Every loop below is built in effective order (counter-clockwise about
    // the outward normal) and stored through `stored`. The copy faces away
    // from the thickening, the offset towards it.
    let copy_wire = |topo: &mut Topology,
                     wire_uses: &[OrientedEdge],
                     map: &BTreeMap<EdgeId, EdgeId>,
                     flip: bool|
     -> Result<WireId, OperationsError> {
        let edges = wire_uses
            .iter()
            .map(|o| OrientedEdge::new(map[&o.edge()], o.is_forward()))
            .collect();
        Ok(topo.add_wire(Wire::new(stored(edges, flip), true)?))
    };
    let mut faces = Vec::with_capacity(2 + walls.len());
    let cap = |topo: &mut Topology,
               map: &BTreeMap<EdgeId, EdgeId>,
               carrier: FaceSurface,
               flip: bool,
               reversed: bool|
     -> Result<FaceId, OperationsError> {
        let mut wires = Vec::with_capacity(uses.len());
        for wire_uses in &uses {
            wires.push(copy_wire(topo, wire_uses, map, flip != reversed)?);
        }
        let outer = wires.remove(0);
        Ok(topo.add_face(if reversed {
            Face::new_reversed(outer, wires, carrier)
        } else {
            Face::new(outer, wires, carrier)
        }))
    };
    // Bottom: outward against the thickening. Top: outward with it.
    faces.push(cap(
        topo,
        &bottom_edge,
        surface.clone(),
        grow,
        reversed != grow,
    )?);
    faces.push(cap(
        topo,
        &top_edge,
        offset_surface,
        !grow,
        reversed == grow,
    )?);

    for wall in &walls {
        let oriented = uses[wall.wire][wall.position];
        let source = &sources[&oriented.edge()];
        let forward = oriented.is_forward();
        let (a, b) = if forward {
            (source.start, source.end)
        } else {
            (source.end, source.start)
        };
        let bottom = bottom_edge[&oriented.edge()];
        let top = top_edge[&oriented.edge()];
        // Growing: along the bottom edge, up at its end, back along the top
        // edge, down at its start. Shrinking runs the other way round.
        let (bottom_use, top_use) = if grow {
            (
                OrientedEdge::new(bottom, forward),
                OrientedEdge::new(top, !forward),
            )
        } else {
            (
                OrientedEdge::new(bottom, !forward),
                OrientedEdge::new(top, forward),
            )
        };
        let (outer, inner) = if wall.annulus {
            let (EdgeCurve::Circle(circle), OffsetCurve::Circle(offset)) =
                (&source.curve, &offsets[&oriented.edge()])
            else {
                return unreachable_curve();
            };
            let (outer, inner) = if offset.radius() > circle.radius() {
                (top_use, bottom_use)
            } else {
                (bottom_use, top_use)
            };
            let flip = wall_reversed(&wall.surface);
            (
                topo.add_wire(Wire::new(stored(vec![outer], flip), true)?),
                vec![topo.add_wire(Wire::new(stored(vec![inner], flip), true)?)],
            )
        } else {
            let loop_ = if grow {
                vec![
                    bottom_use,
                    OrientedEdge::new(rail(topo, b), true),
                    top_use,
                    OrientedEdge::new(rail(topo, a), false),
                ]
            } else {
                vec![
                    bottom_use,
                    OrientedEdge::new(rail(topo, a), true),
                    top_use,
                    OrientedEdge::new(rail(topo, b), false),
                ]
            };
            let loop_ = stored(loop_, wall_reversed(&wall.surface));
            (topo.add_wire(Wire::new(loop_, true)?), Vec::new())
        };
        let face = match &wall.surface {
            WallSurface::Plane { normal, d } => Face::new(
                outer,
                inner,
                FaceSurface::Plane {
                    normal: *normal,
                    d: *d,
                },
            ),
            WallSurface::Curved { surface, reversed } => {
                if *reversed {
                    Face::new_reversed(outer, inner, surface.clone())
                } else {
                    Face::new(outer, inner, surface.clone())
                }
            }
        };
        faces.push(topo.add_face(face));
    }

    if walls.is_empty() {
        // A face with no boundary but seams is closed (a whole torus): the
        // two caps never meet, and the one further along the carrier normal
        // encloses the other as its cavity.
        let (bottom, top) = (faces[0], faces[1]);
        let (outer, inner) = if delta > 0.0 {
            (top, bottom)
        } else {
            (bottom, top)
        };
        let outer = topo.add_shell(Shell::new(vec![outer])?);
        let inner = topo.add_shell(Shell::new(vec![inner])?);
        return Ok(topo.add_solid(Solid::new(outer, vec![inner])));
    }
    let shell = topo.add_shell(Shell::new(faces)?);
    Ok(topo.add_solid(Solid::new(shell, Vec::new())))
}

/// A loop as a face with the given `reversed` flag stores it: the kernel
/// reads a reversed face's wires the other way round
/// (`is_forward() != reversed`), so its effective, counter-clockwise loop is
/// stored reversed.
fn stored(mut uses: Vec<OrientedEdge>, reversed: bool) -> Vec<OrientedEdge> {
    if reversed {
        uses.reverse();
        for oriented in &mut uses {
            *oriented = OrientedEdge::new(oriented.edge(), !oriented.is_forward());
        }
    }
    uses
}

const fn wall_reversed(surface: &WallSurface) -> bool {
    matches!(surface, WallSurface::Curved { reversed: true, .. })
}

fn unreachable_curve<T>() -> Result<T, OperationsError> {
    Err(unsupported(
        "boundary edge changed type while thickening".into(),
    ))
}

/// The carrier offset by `delta` along its own normal, keeping its frame so
/// the offset boundary keeps its parameters.
fn offset_carrier(surface: &FaceSurface, delta: f64) -> Result<FaceSurface, OperationsError> {
    let collapsed = |what: &str, value: f64| {
        unsupported(format!(
            "offsetting by {delta} collapses the {what} (to {value})"
        ))
    };
    Ok(match surface {
        FaceSurface::Cylinder(cylinder) => {
            let radius = cylinder.radius() + delta;
            if !radius.is_finite() || radius <= 0.0 {
                return Err(collapsed("cylinder radius", radius));
            }
            FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
                cylinder.origin(),
                cylinder.axis(),
                radius,
                cylinder.x_axis(),
            )?)
        }
        FaceSurface::Cone(cone) => {
            // A point at distance v along a generator moves by delta along
            // `radial·sin a − axis·cos a`; the same half-angle cone through
            // every moved point has its apex delta / cos a down the axis.
            let apex = cone.apex() - cone.axis() * (delta / cone.half_angle().cos());
            FaceSurface::Cone(ConicalSurface::with_ref_dir(
                apex,
                cone.axis(),
                cone.half_angle(),
                cone.x_axis(),
            )?)
        }
        FaceSurface::Sphere(sphere) => {
            let radius = sphere.radius() + delta;
            if !radius.is_finite() || radius <= 0.0 {
                return Err(collapsed("sphere radius", radius));
            }
            FaceSurface::Sphere(SphericalSurface::with_frame(
                sphere.center(),
                radius,
                sphere.z_axis(),
                sphere.x_axis(),
            )?)
        }
        FaceSurface::Torus(torus) => {
            let minor = torus.minor_radius() + delta;
            if !minor.is_finite() || minor <= 0.0 || minor >= torus.major_radius() {
                return Err(collapsed("torus tube", minor));
            }
            FaceSurface::Torus(ToroidalSurface::with_axis_and_ref_dir(
                torus.center(),
                torus.major_radius(),
                minor,
                torus.z_axis(),
                torus.x_axis(),
            )?)
        }
        FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => {
            return Err(unsupported("not a curved analytic face".into()));
        }
    })
}

/// The offset of a circular boundary arc, when it is again a circle in the
/// same frame (a parallel or meridian, or any circle on a sphere): fitted
/// through two offset points and certified along the whole arc.
fn offset_circle(
    circle: &Circle3D,
    (t0, t1): (f64, f64),
    offset_point: &dyn Fn(Point3) -> Result<Point3, OperationsError>,
    check_tol: f64,
) -> Result<Circle3D, OperationsError> {
    let radial = |t: f64| {
        let (sin, cos) = t.sin_cos();
        circle.u_axis() * cos + circle.v_axis() * sin
    };
    let (ta, tb) = (t0, f64::midpoint(t0, t1));
    let (qa, qb) = (
        offset_point(circle.evaluate(ta))?,
        offset_point(circle.evaluate(tb))?,
    );
    let chord = radial(ta) - radial(tb);
    let radius = (qa - qb).dot(chord) / chord.length_squared();
    if !radius.is_finite() || radius <= check_tol {
        return Err(unsupported(format!(
            "a circular boundary edge offsets to radius {radius}"
        )));
    }
    let center = qa - radial(ta) * radius;
    for i in 0..=16 {
        let t = t0 + (t1 - t0) * f64::from(i) / 16.0;
        let q = offset_point(circle.evaluate(t))?;
        if (q - (center + radial(t) * radius)).length() > check_tol {
            return Err(unsupported(
                "a circular boundary edge that is not a parallel or meridian has no circular \
                 offset"
                    .into(),
            ));
        }
    }
    Ok(Circle3D::with_axes(
        center,
        circle.normal(),
        radius,
        circle.u_axis(),
        circle.v_axis(),
    )?)
}

/// The planar wall between a ruling and its offset.
fn line_wall(
    start: Point3,
    end: Point3,
    offset_start: Point3,
    offset_end: Point3,
    out: Vec3,
    check_tol: f64,
) -> Result<WallSurface, OperationsError> {
    let mut normal = (end - start).cross(offset_start - start).normalize()?;
    if normal.dot(out) < 0.0 {
        normal = -normal;
    }
    let d = normal.dot(start - Point3::new(0.0, 0.0, 0.0));
    if (normal.dot(offset_end - Point3::new(0.0, 0.0, 0.0)) - d).abs() > check_tol {
        return Err(unsupported(
            "a line boundary edge and its offset are not coplanar".into(),
        ));
    }
    Ok(WallSurface::Plane { normal, d })
}

/// The wall between a circular arc and its coaxial offset: a plane between
/// coplanar circles, a cylinder between equal ones, a cone otherwise.
/// Returns whether a planar wall between two closed circles is an annulus.
fn circle_wall(
    circle: &Circle3D,
    offset: &Circle3D,
    domain: (f64, f64),
    closed: bool,
    out: Vec3,
    check_tol: f64,
) -> Result<(WallSurface, bool), OperationsError> {
    let axis = circle.normal();
    let shift = offset.center() - circle.center();
    let axial = shift.dot(axis);
    if (shift - axis * axial).length() > check_tol {
        return Err(unsupported(
            "a circular boundary edge and its offset are not coaxial".into(),
        ));
    }
    let mid = f64::midpoint(domain.0, domain.1);
    let point = circle.evaluate(mid);
    let oriented = |surface: FaceSurface| -> Result<WallSurface, OperationsError> {
        let (u, v) = surface
            .project_point(point)
            .ok_or_else(|| unsupported("wall has no surface parameter".into()))?;
        let reversed = surface.normal(u, v).dot(out) < 0.0;
        Ok(WallSurface::Curved { surface, reversed })
    };
    if axial.abs() <= check_tol {
        let normal = if axis.dot(out) < 0.0 { -axis } else { axis };
        let d = normal.dot(circle.center() - Point3::new(0.0, 0.0, 0.0));
        return Ok((WallSurface::Plane { normal, d }, closed));
    }
    let wall = if (offset.radius() - circle.radius()).abs() <= check_tol {
        FaceSurface::Cylinder(CylindricalSurface::with_ref_dir(
            circle.center(),
            axis,
            circle.radius(),
            circle.u_axis(),
        )?)
    } else {
        // The radius runs linearly along the shared axis and reaches zero at
        // the apex, beyond the smaller circle.
        let at_apex = circle.radius() / (circle.radius() - offset.radius());
        let apex = circle.center() + shift * at_apex;
        let to_circle = circle.center() - apex;
        let half_angle = to_circle.length().atan2(circle.radius());
        FaceSurface::Cone(ConicalSurface::with_ref_dir(
            apex,
            to_circle.normalize()?,
            half_angle,
            circle.u_axis(),
        )?)
    };
    for t in [domain.0, mid, domain.1] {
        for p in [circle.evaluate(t), offset.evaluate(t)] {
            let (u, v) = wall
                .project_point(p)
                .ok_or_else(|| unsupported("wall has no surface parameter".into()))?;
            if wall
                .evaluate(u, v)
                .is_none_or(|s| (s - p).length() > check_tol)
            {
                return Err(unsupported(
                    "a circular boundary edge and its offset do not bound one cone".into(),
                ));
            }
        }
    }
    Ok((oriented(wall)?, false))
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use remus_math::tolerance::Tolerance;
    use remus_topology::Topology;
    use remus_topology::test_utils::make_unit_square_face;

    use super::*;

    #[test]
    fn thicken_positive() {
        let mut topo = Topology::new();
        let face = make_unit_square_face(&mut topo);

        let solid = thicken(&mut topo, face, 2.0).unwrap();

        let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(vol, 2.0),
            "1×1 face thickened by 2 should have volume ~2.0, got {vol}"
        );
    }

    #[test]
    fn thicken_negative() {
        let mut topo = Topology::new();
        let face = make_unit_square_face(&mut topo);

        let solid = thicken(&mut topo, face, -1.5).unwrap();

        let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
        let tol = Tolerance::loose();
        assert!(
            tol.approx_eq(vol, 1.5),
            "1×1 face thickened by -1.5 should have volume ~1.5, got {vol}"
        );
    }

    #[test]
    fn thicken_zero_error() {
        let mut topo = Topology::new();
        let face = make_unit_square_face(&mut topo);
        assert!(thicken(&mut topo, face, 0.0).is_err());
    }

    #[test]
    fn thicken_nurbs_face() {
        use remus_math::nurbs::surface::NurbsSurface;
        use remus_math::vec::Point3;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::{Face, FaceSurface};
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};

        let mut topo = Topology::new();

        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
                vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .unwrap();

        let tol = 1e-7;
        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), tol));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), tol));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), tol));
        let v3 = topo.add_vertex(Vertex::new(Point3::new(0.0, 1.0, 0.0), tol));

        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v3, EdgeCurve::Line));
        let e3 = topo.add_edge(Edge::new(v3, v0, EdgeCurve::Line));

        let wire = Wire::new(
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, true),
                OrientedEdge::new(e2, true),
                OrientedEdge::new(e3, true),
            ],
            true,
        )
        .unwrap();
        let wid = topo.add_wire(wire);
        let face = topo.add_face(Face::new(wid, vec![], FaceSurface::Nurbs(surface)));

        let solid = thicken(&mut topo, face, 2.0).unwrap();

        let vol = crate::measure::solid_volume(&topo, solid, 0.1).unwrap();
        assert!(
            vol > 0.0,
            "thickened NURBS face should have positive volume, got {vol}"
        );
    }
}
