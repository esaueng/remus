//! Face filling: create a smooth NURBS surface from boundary curves.
//!
//! Fills an N-sided boundary with a surface patch. For 4-sided boundaries,
//! uses Coons patch interpolation.

use remus_math::nurbs::projection::{SurfaceSeedGrid, project_point_to_surface_with_grid};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;

/// Certify the bounded annular trim supported by non-planar section caps.
///
/// A generated bilinear cap owns the full rectangular parameter domain. Its
/// holes are exact only when every hole is a non-touching interior rectangle
/// made from four surface iso-lines. Anything broader needs a general trimmed
/// surface construction, so it is refused instead of attaching a wire that
/// the cap surface or tessellator cannot honor.
///
/// # Errors
///
/// Returns an error if a hole is not a closed four-line rectangle, leaves the
/// cap surface, touches its outer boundary, or overlaps another hole.
pub(crate) fn validate_annular_cap_holes(
    topo: &Topology,
    surface: &NurbsSurface,
    holes: &[WireId],
    scale: f64,
) -> Result<(), OperationsError> {
    if holes.is_empty() {
        return Ok(());
    }

    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let u_span = u1 - u0;
    let v_span = v1 - v0;
    if !u_span.is_finite() || u_span <= 0.0 || !v_span.is_finite() || v_span <= 0.0 {
        return Err(OperationsError::InvalidInput {
            reason: "non-planar cap has an invalid parameter domain".into(),
        });
    }

    let grid = SurfaceSeedGrid::for_surface(surface);
    let distance_tol = (scale * 1e-7).max(1e-8);
    let param_tol = 1e-6 * u_span.max(v_span);
    let boundary_margin = 10.0 * param_tol;
    let mut rectangles: Vec<(f64, f64, f64, f64)> = Vec::with_capacity(holes.len());

    for &wire_id in holes {
        let wire = topo.wire(wire_id)?;
        if !wire.is_closed() || wire.edges().len() != 4 {
            return Err(OperationsError::InvalidInput {
                reason: "non-planar cap holes require closed four-edge rectangular loops".into(),
            });
        }

        let mut corners = Vec::with_capacity(4);
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            if !matches!(edge.curve(), EdgeCurve::Line) {
                return Err(OperationsError::InvalidInput {
                    reason: "non-planar cap holes currently require line-edge loops".into(),
                });
            }
            let start = topo.vertex(oe.oriented_start(edge))?.point();
            let end = topo.vertex(oe.oriented_end(edge))?.point();
            let mut start_uv = None;
            let mut end_uv = None;
            for sample in 0..=8 {
                let fraction = f64::from(sample) / 8.0;
                let point = start + (end - start) * fraction;
                let projection =
                    project_point_to_surface_with_grid(surface, point, distance_tol, &grid)
                        .map_err(|_| OperationsError::InvalidInput {
                            reason:
                                "non-planar cap hole could not be projected onto its fill surface"
                                    .into(),
                        })?;
                if !projection.distance.is_finite() || projection.distance > distance_tol {
                    return Err(OperationsError::InvalidInput {
                        reason: format!(
                            "non-planar cap hole leaves its fill surface by {} (tolerance {})",
                            projection.distance, distance_tol
                        ),
                    });
                }
                if projection.u <= u0 + boundary_margin
                    || projection.u >= u1 - boundary_margin
                    || projection.v <= v0 + boundary_margin
                    || projection.v >= v1 - boundary_margin
                {
                    return Err(OperationsError::InvalidInput {
                        reason: "non-planar cap hole touches or leaves the outer cap boundary"
                            .into(),
                    });
                }
                if sample == 0 {
                    start_uv = Some(Point2::new(projection.u, projection.v));
                } else if sample == 8 {
                    end_uv = Some(Point2::new(projection.u, projection.v));
                }
            }

            let start_uv = start_uv.ok_or_else(|| OperationsError::InvalidInput {
                reason: "non-planar cap hole has no start parameter".into(),
            })?;
            let end_uv = end_uv.ok_or_else(|| OperationsError::InvalidInput {
                reason: "non-planar cap hole has no end parameter".into(),
            })?;
            let constant_u = (start_uv.x() - end_uv.x()).abs() <= param_tol;
            let constant_v = (start_uv.y() - end_uv.y()).abs() <= param_tol;
            if constant_u == constant_v {
                return Err(OperationsError::InvalidInput {
                    reason: "non-planar cap hole edges must follow one surface parameter at a time"
                        .into(),
                });
            }
            corners.push(start_uv);
        }

        for index in 0..4 {
            let current = corners[index];
            let next = corners[(index + 1) % 4];
            let following = corners[(index + 2) % 4];
            let current_u = (current.x() - next.x()).abs() <= param_tol;
            let next_u = (next.x() - following.x()).abs() <= param_tol;
            if current_u == next_u {
                return Err(OperationsError::InvalidInput {
                    reason: "non-planar cap hole edges do not alternate around a rectangle".into(),
                });
            }
        }

        let min_u = corners.iter().map(|p| p.x()).fold(f64::INFINITY, f64::min);
        let max_u = corners
            .iter()
            .map(|p| p.x())
            .fold(f64::NEG_INFINITY, f64::max);
        let min_v = corners.iter().map(|p| p.y()).fold(f64::INFINITY, f64::min);
        let max_v = corners
            .iter()
            .map(|p| p.y())
            .fold(f64::NEG_INFINITY, f64::max);
        if max_u - min_u <= boundary_margin || max_v - min_v <= boundary_margin {
            return Err(OperationsError::InvalidInput {
                reason: "non-planar cap hole has a degenerate parameter-space area".into(),
            });
        }

        if rectangles
            .iter()
            .any(|&(other_min_u, other_max_u, other_min_v, other_max_v)| {
                min_u < other_max_u + boundary_margin
                    && max_u > other_min_u - boundary_margin
                    && min_v < other_max_v + boundary_margin
                    && max_v > other_min_v - boundary_margin
            })
        {
            return Err(OperationsError::InvalidInput {
                reason: "non-planar cap holes overlap or touch in parameter space".into(),
            });
        }
        rectangles.push((min_u, max_u, min_v, max_v));
    }

    Ok(())
}

/// Fill a 4-sided boundary with a Coons patch.
///
/// Given 4 boundary curves as polylines (each connecting to the next),
/// creates a smooth bilinear NURBS surface that interpolates the boundaries.
///
/// The curves should be ordered: bottom, right, top (reversed), left (reversed).
///
/// # Errors
///
/// Returns an error if fewer than 4 boundary curves are provided or if
/// curves have mismatched lengths.
pub fn fill_coons_patch(
    topo: &mut Topology,
    curves: &[Vec<Point3>],
) -> Result<remus_topology::face::FaceId, OperationsError> {
    if curves.len() < 4 {
        return Err(OperationsError::InvalidInput {
            reason: format!(
                "Coons patch requires 4 boundary curves, got {}",
                curves.len()
            ),
        });
    }

    let bottom = &curves[0];
    let right = &curves[1];
    let top = &curves[2];
    let left = &curves[3];

    let n_u = bottom.len();
    let n_v = right.len();

    if top.len() != n_u || left.len() != n_v {
        return Err(OperationsError::InvalidInput {
            reason: "Coons patch boundary curves must have consistent point counts".into(),
        });
    }

    if n_u < 2 || n_v < 2 {
        return Err(OperationsError::InvalidInput {
            reason: "boundary curves must have at least 2 points each".into(),
        });
    }

    let surface = coons_surface(bottom, right, top, left)?;

    let p00 = bottom[0];
    let p10 = bottom[n_u - 1];
    let p01 = top[0];
    let p11 = top[n_u - 1];

    let corners = [p00, p10, p11, p01];
    let verts: Vec<_> = corners
        .iter()
        .map(|&p| topo.add_vertex(Vertex::new(p, 1e-7)))
        .collect();

    let n_corners = verts.len();
    let edges: Vec<_> = (0..n_corners)
        .map(|i| {
            let next = (i + 1) % n_corners;
            topo.add_edge(Edge::new(verts[i], verts[next], EdgeCurve::Line))
        })
        .collect();

    let oriented: Vec<_> = edges
        .iter()
        .map(|&eid| OrientedEdge::new(eid, true))
        .collect();
    let wire = Wire::new(oriented, true).map_err(OperationsError::Topology)?;
    let wire_id = topo.add_wire(wire);

    let face = Face::new(wire_id, vec![], FaceSurface::Nurbs(surface));
    Ok(topo.add_face(face))
}

/// The bilinearly-blended Coons NURBS surface of four boundary polylines.
///
/// Degree 1 in both directions with uniform clamped knots, so the surface
/// interpolates the boundary polylines exactly: the `v = 0` iso-curve is the
/// piecewise-linear `bottom`, `v = 1` is `top`, and likewise `left`/`right`
/// at `u = 0`/`u = 1`. Curves run: bottom and top left→right, left and right
/// bottom→top. `bottom`/`top` must share a point count, as must
/// `left`/`right`.
///
/// # Errors
///
/// Returns an error on mismatched point counts or an invalid NURBS net.
pub(crate) fn coons_surface(
    bottom: &[Point3],
    right: &[Point3],
    top: &[Point3],
    left: &[Point3],
) -> Result<NurbsSurface, OperationsError> {
    let n_u = bottom.len();
    let n_v = right.len();
    if top.len() != n_u || left.len() != n_v {
        return Err(OperationsError::InvalidInput {
            reason: "Coons patch boundary curves must have consistent point counts".into(),
        });
    }
    if n_u < 2 || n_v < 2 {
        return Err(OperationsError::InvalidInput {
            reason: "boundary curves must have at least 2 points each".into(),
        });
    }

    // Build the Coons patch: P(u,v) = Lc(u,v) + Ld(u,v) - B(u,v)
    // where:
    //   Lc = (1-v)*bottom(u) + v*top(u)      (linear blend of u-curves)
    //   Ld = (1-u)*left(v) + u*right(v)       (linear blend of v-curves)
    //   B  = bilinear interpolation of corners
    // `NurbsSurface` control nets are indexed `[u_row][v_col]`, so the outer
    // loop runs over u. (The original loop nested the other way round — a
    // transpose that square grids masked and a 3×2 net rejects.)
    let mut control_points = Vec::with_capacity(n_u);
    let mut weights = Vec::with_capacity(n_u);

    let p00 = bottom[0];
    let p10 = bottom[n_u - 1];
    let p01 = top[0];
    let p11 = top[n_u - 1];

    for i in 0..n_u {
        #[allow(clippy::cast_precision_loss)]
        let u = i as f64 / (n_u - 1) as f64;

        let mut row = Vec::with_capacity(n_v);
        let mut weight_row = Vec::with_capacity(n_v);

        for j in 0..n_v {
            #[allow(clippy::cast_precision_loss)]
            let v = j as f64 / (n_v - 1) as f64;

            // Lc: linear blend along v
            let lc = blend(bottom[i], top[i], v);

            // Ld: linear blend along u
            let ld = blend(left[j], right[j], u);

            // B: bilinear of corners
            let b = bilinear(p00, p10, p01, p11, u, v);

            // Coons: Lc + Ld - B
            let point = Point3::new(
                lc.x() + ld.x() - b.x(),
                lc.y() + ld.y() - b.y(),
                lc.z() + ld.z() - b.z(),
            );

            row.push(point);
            weight_row.push(1.0);
        }

        control_points.push(row);
        weights.push(weight_row);
    }

    // Build NURBS surface (degree 1 for bilinear Coons)
    let degree_u = 1.min(n_u - 1);
    let degree_v = 1.min(n_v - 1);

    let knots_u = build_clamped_knots(n_u, degree_u);
    let knots_v = build_clamped_knots(n_v, degree_v);

    Ok(NurbsSurface::new(
        degree_u,
        degree_v,
        knots_u,
        knots_v,
        control_points,
        weights,
    )?)
}

/// Linear blend: (1-t)*a + t*b
fn blend(a: Point3, b: Point3, t: f64) -> Point3 {
    Point3::new(
        a.x().mul_add(1.0 - t, b.x() * t),
        a.y().mul_add(1.0 - t, b.y() * t),
        a.z().mul_add(1.0 - t, b.z() * t),
    )
}

/// Bilinear interpolation of 4 corner points.
fn bilinear(p00: Point3, p10: Point3, p01: Point3, p11: Point3, u: f64, v: f64) -> Point3 {
    let bottom = blend(p00, p10, u);
    let top = blend(p01, p11, u);
    blend(bottom, top, v)
}

/// Build a clamped knot vector for n control points and given degree.
fn build_clamped_knots(n: usize, degree: usize) -> Vec<f64> {
    let mut knots = Vec::with_capacity(n + degree + 1);
    knots.extend(std::iter::repeat_n(0.0, degree + 1));
    let internal = n.saturating_sub(degree + 1);
    for i in 1..=internal {
        #[allow(clippy::cast_precision_loss)]
        knots.push(i as f64 / (internal + 1) as f64);
    }
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn coons_patch_flat_square() {
        let mut topo = Topology::new();

        // 4 boundary curves forming a unit square in z=0
        let bottom = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let right = vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)];
        let top = vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 0.0)];
        let left = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];

        let face_id = fill_coons_patch(&mut topo, &[bottom, right, top, left]).unwrap();

        let face = topo.face(face_id).unwrap();
        assert!(matches!(face.surface(), FaceSurface::Nurbs(_)));
    }

    #[test]
    fn coons_patch_saddle() {
        let mut topo = Topology::new();

        // Saddle-shaped boundary (corners at different heights)
        let bottom = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.5, 0.0, 0.1),
            Point3::new(1.0, 0.0, 0.0),
        ];
        let right = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.5, 0.1),
            Point3::new(1.0, 1.0, 0.0),
        ];
        let top = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.5, 1.0, -0.1),
            Point3::new(1.0, 1.0, 0.0),
        ];
        let left = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 0.5, -0.1),
            Point3::new(0.0, 1.0, 0.0),
        ];

        let face_id = fill_coons_patch(&mut topo, &[bottom, right, top, left]).unwrap();

        let face = topo.face(face_id).unwrap();
        if let FaceSurface::Nurbs(surf) = face.surface() {
            // Center of saddle should be near (0.5, 0.5, ~0)
            let center = surf.evaluate(0.5, 0.5);
            assert!((center.x() - 0.5).abs() < 0.2);
            assert!((center.y() - 0.5).abs() < 0.2);
        }
    }

    #[test]
    fn coons_patch_too_few_curves_error() {
        let mut topo = Topology::new();
        let curve = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        assert!(fill_coons_patch(&mut topo, &[curve]).is_err());
    }

    #[test]
    fn coons_patch_mismatched_lengths_error() {
        let mut topo = Topology::new();
        let bottom = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        let right = vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)];
        let top = vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ]; // 3 points vs bottom's 2
        let left = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];

        assert!(fill_coons_patch(&mut topo, &[bottom, right, top, left]).is_err());
    }
}
