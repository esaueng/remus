//! Edge and wire length computation.

use remus_topology::face::FaceId;
use remus_topology::{BodyClass, BodyId, Topology};

/// Compute the length of a single edge.
///
/// For line edges, returns the Euclidean distance between endpoints.
/// For NURBS curve edges, uses numerical integration (Simpson's rule).
///
/// # Errors
///
/// Returns an error if the edge lookup fails or a stored trim is invalid.
/// Legacy edges without stored trims retain endpoint-based reconstruction.
pub fn edge_length(
    topo: &Topology,
    edge_id: remus_topology::edge::EdgeId,
) -> Result<f64, crate::OperationsError> {
    let edge = topo.edge(edge_id)?;
    let (t0, t1) =
        if edge.trim().is_some() || matches!(edge.curve(), remus_topology::edge::EdgeCurve::Line) {
            crate::authoritative_edge_domain(edge, "edge length")?
        } else {
            // Compatibility adapter for legacy public-API edges. Stored authority
            // always takes precedence and invalid stored ranges never fall back.
            // The shared read-only adapter validates the reconstructed range
            // and endpoints without copying the topology arena.
            crate::reconstruct_legacy_edge_domain(topo, edge_id, "edge length")?
        };
    match edge.curve() {
        remus_topology::edge::EdgeCurve::Line => {
            let start = topo.vertex(edge.start())?.point();
            let end = topo.vertex(edge.end())?.point();
            Ok((end - start).length())
        }
        remus_topology::edge::EdgeCurve::NurbsCurve(curve) => {
            // Integrate only the authoritative edge span. Splits can share a
            // carrier, and descending trims trace the same length backwards.
            let intervals = 50;
            let lo = t0.min(t1);
            let dt = (t1 - t0).abs() / f64::from(intervals);
            let mut length = 0.0;
            for i in 0..intervals {
                let a = f64::from(i).mul_add(dt, lo);
                let b = a + dt;
                let mid = f64::midpoint(a, b);
                let va = curve.derivatives(a, 1)[1].length();
                let vm = curve.derivatives(mid, 1)[1].length();
                let vb = curve.derivatives(b, 1)[1].length();
                length += (dt / 6.0) * vm.mul_add(4.0, va + vb);
            }
            Ok(length)
        }
        remus_topology::edge::EdgeCurve::Circle(circle) => Ok((t1 - t0).abs() * circle.radius()),
        remus_topology::edge::EdgeCurve::Ellipse(ellipse) => {
            if edge.is_closed() {
                Ok(ellipse.approximate_circumference())
            } else {
                // Keep the existing chord approximation, on the stored span.
                let intervals = 50;
                let dt = (t1 - t0) / f64::from(intervals);
                let mut length = 0.0;
                let mut prev = ellipse.evaluate(t0);
                for i in 1..=intervals {
                    let curr = ellipse.evaluate(f64::from(i).mul_add(dt, t0));
                    length += (curr - prev).length();
                    prev = curr;
                }
                Ok(length)
            }
        }
        remus_topology::edge::EdgeCurve::Hyperbola(h) => Ok(h.arc_length(t0, t1)),
        remus_topology::edge::EdgeCurve::Parabola(p) => Ok(p.arc_length(t0, t1)),
    }
}

/// Compute the total length (perimeter) of a wire.
///
/// Sums the length of all edges in the wire.
///
/// # Errors
///
/// Returns an error if any edge lookup fails.
pub fn wire_length(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
) -> Result<f64, crate::OperationsError> {
    let wire = topo.wire(wire_id)?;
    let mut total = 0.0;
    for oe in wire.edges() {
        total += edge_length(topo, oe.edge())?;
    }
    Ok(total)
}

/// Compute length through the body-level dispatch contract.
///
/// # Errors
///
/// Wire bodies delegate to [`wire_length`]. Solid and sheet bodies return a
/// typed dimensional mismatch rather than an invented zero length.
pub fn body_length(topo: &Topology, body: BodyId) -> Result<f64, crate::OperationsError> {
    match body {
        BodyId::Wire(wire) => {
            let actual = topo.body_class_of(body)?;
            if actual != BodyClass::Wire {
                return Err(crate::OperationsError::BodyClassMeasureMismatch {
                    operation: "length",
                    expected: BodyClass::Wire.as_str(),
                    actual: actual.as_str(),
                });
            }
            wire_length(topo, wire)
        }
        BodyId::Solid(_) | BodyId::Shell(_) => {
            let actual = topo.body_class_of(body)?;
            Err(crate::OperationsError::BodyClassMeasureMismatch {
                operation: "length",
                expected: BodyClass::Wire.as_str(),
                actual: actual.as_str(),
            })
        }
    }
}

/// Compute the perimeter of a face (outer wire length).
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn face_perimeter(topo: &Topology, face_id: FaceId) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;
    wire_length(topo, face.outer_wire())
}
