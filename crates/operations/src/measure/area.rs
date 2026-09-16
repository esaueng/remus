//! Face and solid surface area computation.

use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::shell::ShellId;
use remus_topology::solid::SolidId;
use remus_topology::{BodyClass, BodyId};

use crate::tessellate;

use super::helpers::{collect_solid_face_ids, collect_wire_positions};

/// Compute the area of a single face.
///
/// Line-only planar faces use exact Newell integration. Planar faces whose
/// boundaries contain circles, ellipses, parabolas, hyperbolas, or NURBS arcs
/// recognized as one of those forms use the exact Green-theorem boundary
/// integrator, including inner wires. Other planar boundaries retain the
/// established 256-sample polygon path. NURBS faces tessellate and sum
/// triangle areas.
///
/// The result is deflection-independent on every path except the NURBS-face
/// tessellation and the sampled planar fallback; both are documented at the
/// functions that own them.
///
/// # Errors
///
/// Returns an error if the face is missing or tessellation fails.
pub fn face_area(
    topo: &Topology,
    face_id: FaceId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;

    match face.surface() {
        FaceSurface::Plane { .. } => {
            if planar_boundary_needs_curve_integral(topo, face_id)? {
                Ok(
                    remus_check::properties::face_integrator::integrate_face(topo, face_id, 5)?
                        .area
                        .abs(),
                )
            } else {
                planar_face_area(topo, face_id)
            }
        }
        // Analytic curved faces integrate the exact surface by Gauss
        // quadrature, so the area is deflection-independent. `deflection`
        // is accepted for API compatibility and ignored on this path.
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_) => Ok(remus_check::properties::face_integrator::integrate_face(
            topo, face_id, 8,
        )?
        .area
        .abs()),
        FaceSurface::Nurbs(_) => {
            let mesh = tessellate::tessellate(topo, face_id, deflection)?;
            Ok(triangle_mesh_area(&mesh))
        }
    }
}

/// Whether the boundary needs and supports the exact curved-edge planar
/// moment integrator in `remus-check`.
///
/// Keep this list deliberately identical to
/// `face_integrator::planar_wire_monomial_moments`: routing an unsupported
/// edge through `integrate_face` would silently replace this module's
/// established 256-sample fallback with the check crate's coarser generic
/// polygon path. A line-only boundary stays on the cheaper exact Newell path.
///
/// Native circle, ellipse, parabola, and hyperbola edges are exact. A NURBS
/// edge is exact only when curve recognition identifies it as one of those
/// forms at a scale-relative tolerance (see
/// `face_integrator::nurbs_boundary_is_recognized`); any other NURBS keeps
/// the whole face on the sampled fallback.
fn planar_boundary_needs_curve_integral(
    topo: &Topology,
    face_id: FaceId,
) -> Result<bool, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;

    let face = topo.face(face_id)?;
    let mut has_supported_curve = false;
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id)?;
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge())?;
            match edge.curve() {
                EdgeCurve::Line => {}
                EdgeCurve::Circle(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Parabola(_)
                | EdgeCurve::Hyperbola(_) => has_supported_curve = true,
                EdgeCurve::NurbsCurve(nc) => {
                    if !nurbs_boundary_is_recognized(topo, edge, nc)? {
                        return Ok(false);
                    }
                    has_supported_curve = true;
                }
            }
        }
    }
    Ok(has_supported_curve)
}

/// Whether a NURBS boundary edge is recognized as an analytic form at a
/// scale-relative tolerance.
///
/// Mirrors `face_integrator::nurbs_boundary_is_recognized` (which owns the
/// tolerance doctrine); duplicated rather than shared because `operations`
/// cannot call into `check`'s private helpers and the predicate itself is
/// three lines.
fn nurbs_boundary_is_recognized(
    topo: &Topology,
    edge: &remus_topology::edge::Edge,
    nc: &remus_math::nurbs::curve::NurbsCurve,
) -> Result<bool, crate::OperationsError> {
    use remus_geometry::convert::{RecognizedCurve, recognize_curve};

    let start = topo.vertex(edge.start())?.point();
    let mut extent: f64 = 0.0;
    for p in nc.control_points() {
        extent = extent.max((*p - start).length());
    }
    let extent = extent.max(1e-12);
    let tolerance = (extent * 1e-9).max(1e-12);
    Ok(!matches!(
        recognize_curve(nc, tolerance),
        RecognizedCurve::NotRecognized
    ))
}

/// Sampled fallback for a planar boundary the exact moment integrator does
/// not support. Computes the Newell area of a 256-sample polygon and
/// subtracts sampled inner-wire areas.
fn planar_face_area(topo: &Topology, face_id: FaceId) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let outer_wire = topo.wire(face.outer_wire())?;
    let outer_positions = collect_wire_positions(topo, outer_wire)?;

    let outer_area = newell_area(&outer_positions);

    // Subtract hole areas.
    let mut hole_area = 0.0;
    for &inner_wid in face.inner_wires() {
        let inner_wire = topo.wire(inner_wid)?;
        let inner_positions = collect_wire_positions(topo, inner_wire)?;
        hole_area += newell_area(&inner_positions);
    }

    Ok((outer_area - hole_area).abs())
}

/// Compute the area of a polygon using Newell's method.
fn newell_area(positions: &[Point3]) -> f64 {
    let n = positions.len();
    if n < 3 {
        return 0.0;
    }

    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut sz = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        let vi = positions[i];
        let vj = positions[j];
        sx = vi.z().mul_add(-vj.y(), vi.y().mul_add(vj.z(), sx));
        sy = vi.x().mul_add(-vj.z(), vi.z().mul_add(vj.x(), sy));
        sz = vi.y().mul_add(-vj.x(), vi.x().mul_add(vj.y(), sz));
    }

    0.5 * sz.mul_add(sz, sx.mul_add(sx, sy * sy)).sqrt()
}

/// Sum of triangle areas from a tessellated mesh.
fn triangle_mesh_area(mesh: &tessellate::TriangleMesh) -> f64 {
    let mut area = 0.0;
    let idx = &mesh.indices;
    let pos = &mesh.positions;
    let tri_count = idx.len() / 3;

    for t in 0..tri_count {
        let i0 = idx[t * 3] as usize;
        let i1 = idx[t * 3 + 1] as usize;
        let i2 = idx[t * 3 + 2] as usize;

        let a = pos[i1] - pos[i0];
        let b = pos[i2] - pos[i0];
        area += 0.5 * a.cross(b).length();
    }

    area
}

/// Compute the total surface area of a solid.
///
/// Sums `face_area()` over every face in every shell.
///
/// # Errors
///
/// Returns an error if a topology lookup or tessellation fails.
pub fn solid_surface_area(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    let mut total = 0.0;
    for fid in collect_solid_face_ids(topo, solid)? {
        total += face_area(topo, fid, deflection)?;
    }
    Ok(total)
}

/// Compute the total area of a first-class sheet body.
///
/// # Errors
///
/// Returns a typed body-class mismatch for a solid-owned shell, or an error
/// if a topology lookup or face-area computation fails.
pub fn sheet_surface_area(
    topo: &Topology,
    sheet: ShellId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    let actual = topo.body_class_of(BodyId::Shell(sheet))?;
    if actual != BodyClass::Sheet {
        return Err(crate::OperationsError::BodyClassMeasureMismatch {
            operation: "surface area",
            expected: BodyClass::Sheet.as_str(),
            actual: actual.as_str(),
        });
    }

    let mut total = 0.0;
    for &face in topo.shell(sheet)?.faces() {
        total += face_area(topo, face, deflection)?;
    }
    Ok(total)
}

/// Compute the area-weighted center of a first-class sheet body.
///
/// The exact face geometry is integrated directly with bounded Gauss
/// quadrature; this does not depend on a tessellation deflection.
///
/// # Errors
///
/// Returns a typed body-class mismatch for a shell that is not tagged as a
/// sheet, or an error if integration fails or the sheet has zero area.
pub fn sheet_center_of_area(
    topo: &Topology,
    sheet: ShellId,
) -> Result<Point3, crate::OperationsError> {
    let actual = topo.body_class_of(BodyId::Shell(sheet))?;
    if actual != BodyClass::Sheet {
        return Err(crate::OperationsError::BodyClassMeasureMismatch {
            operation: "center of area",
            expected: BodyClass::Sheet.as_str(),
            actual: actual.as_str(),
        });
    }

    let gauss_order = remus_check::properties::PropertiesOptions::default().gauss_order;
    let mut area = 0.0;
    let mut center_x = 0.0;
    let mut center_y = 0.0;
    let mut center_z = 0.0;
    for &face in topo.shell(sheet)?.faces() {
        let contribution =
            remus_check::properties::face_integrator::integrate_face(topo, face, gauss_order)?;
        area += contribution.area;
        center_x += contribution.centroid_x;
        center_y += contribution.centroid_y;
        center_z += contribution.centroid_z;
    }
    if !area.is_finite() || area <= 0.0 {
        return Err(crate::OperationsError::InvalidInput {
            reason: "sheet has zero or non-finite area".to_owned(),
        });
    }
    let center = Point3::new(center_x / area, center_y / area, center_z / area);
    if !center.0.iter().all(|value| value.is_finite()) {
        return Err(crate::OperationsError::InvalidInput {
            reason: "sheet center of area is non-finite".to_owned(),
        });
    }
    Ok(center)
}

/// Compute the area of any surface-bearing body.
///
/// # Errors
///
/// Returns a typed mismatch for a wire body or a shell that is not tagged as
/// a sheet, or propagates topology and face-area failures.
pub fn body_surface_area(
    topo: &Topology,
    body: BodyId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    match body {
        BodyId::Solid(solid) => solid_surface_area(topo, solid, deflection),
        BodyId::Shell(sheet) => sheet_surface_area(topo, sheet, deflection),
        BodyId::Wire(wire) => {
            let actual = topo.body_class_of(BodyId::Wire(wire))?;
            Err(crate::OperationsError::BodyClassMeasureMismatch {
                operation: "surface area",
                expected: "solid or sheet",
                actual: actual.as_str(),
            })
        }
    }
}
