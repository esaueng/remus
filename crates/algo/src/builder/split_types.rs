//! Transient data structures for the face-splitting pipeline.
//!
//! These types carry edge and face data through the splitting stages:
//! pcurve computation, wire building, and sub-face construction.

use brepkit_math::curves2d::Curve2D;
use brepkit_math::vec::{Point2, Point3};
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::face::{FaceId, FaceSurface};

use super::plane_frame::PlaneFrame;
use crate::ds::Rank;

// ---------------------------------------------------------------------------
// Edge-level types
// ---------------------------------------------------------------------------

/// A 2D-oriented edge on a face's parameter space.
///
/// Each edge carries both 3D geometry (for assembly) and a 2D pcurve
/// (for wire construction and classification in parameter space).
#[derive(Debug, Clone)]
pub struct OrientedPCurveEdge {
    /// 3D edge curve (Line, Circle, Ellipse, NurbsCurve).
    pub curve_3d: EdgeCurve,
    /// Exact parameter interval in `curve_3d`'s native direction.
    pub trim: Option<(f64, f64)>,
    /// 2D curve in this face's (u,v) parameter space.
    pub pcurve: Curve2D,
    /// Start point in (u,v) space.
    pub start_uv: Point2,
    /// End point in (u,v) space.
    pub end_uv: Point2,
    /// Start point in 3D.
    pub start_3d: Point3,
    /// End point in 3D.
    pub end_3d: Point3,
    /// Whether this edge is traversed in its natural direction.
    pub forward: bool,
    /// Index of the source edge in the face splitter's input edge list.
    /// Used by `build_topology_face` to share edge entities between
    /// adjacent sub-face loops from the SAME face split.
    /// `None` for edges not tracked (e.g., from boundary splitting).
    pub source_edge_idx: Option<usize>,
    /// Pave block ID from GFA arena. Used for cross-face edge sharing:
    /// section edges from the same FF intersection curve share this ID
    /// across different faces, enabling manifold edge topology.
    pub pave_block_id: Option<usize>,
    /// Store-space index of the topology edge this pcurve edge was built
    /// from, when known (Issue 12 construction lineage). Sub-segments
    /// inherit their parent's value; synthesized edges carry `None`.
    pub source_topo_edge: Option<usize>,
}

impl OrientedPCurveEdge {
    /// Endpoint-derived support domain in this wire use's traversal direction.
    ///
    /// Face partitioning uses this as a structural support-arc heuristic. The
    /// exact trim remains carried separately and is written to result edges.
    #[must_use]
    pub fn domain(&self) -> (f64, f64) {
        self.curve_3d
            .domain_with_endpoints(self.start_3d, self.end_3d)
    }

    /// Parameter domain in this wire use's traversal direction.
    ///
    /// This intentionally retains the endpoint-derived complement-arc
    /// convention used by the partitioner's structural tests.
    #[must_use]
    pub fn native_domain(&self) -> (f64, f64) {
        if self.forward {
            self.curve_3d
                .domain_with_endpoints(self.start_3d, self.end_3d)
        } else {
            self.curve_3d
                .domain_with_endpoints(self.end_3d, self.start_3d)
        }
    }

    /// Exact child trim for endpoints expressed in wire traversal order.
    #[must_use]
    pub fn sub_trim(&self, start: Point3, end: Point3) -> Option<(f64, f64)> {
        let parent = self.trim.unwrap_or_else(|| self.domain());
        let traversal_parent = if self.forward {
            Some(parent)
        } else {
            reverse_trim(Some(parent))
        }?;
        let trim = sub_trim(&self.curve_3d, traversal_parent, start, end);
        if self.forward {
            trim
        } else {
            reverse_trim(trim)
        }
    }

    /// Exact child trim for a normalized sub-span in wire traversal order.
    #[must_use]
    pub fn sub_trim_fraction(&self, start: f64, end: f64) -> Option<(f64, f64)> {
        self.trim.map(|(t0, t1)| {
            if self.forward {
                ((t1 - t0).mul_add(start, t0), (t1 - t0).mul_add(end, t0))
            } else {
                (
                    (t1 - t0).mul_add(1.0 - end, t0),
                    (t1 - t0).mul_add(1.0 - start, t0),
                )
            }
        })
    }
}

/// Reverse an interval so it traces the opposite endpoint order.
#[must_use]
pub const fn reverse_trim(trim: Option<(f64, f64)>) -> Option<(f64, f64)> {
    match trim {
        Some((t0, t1)) => Some((t1, t0)),
        None => None,
    }
}

/// Narrow a parent interval to exact child endpoints on the same carrier.
#[must_use]
pub fn sub_trim(
    curve: &EdgeCurve,
    parent: (f64, f64),
    start: Point3,
    end: Point3,
) -> Option<(f64, f64)> {
    let angular = |start_parameter: f64, end_parameter: f64| {
        angular_sub_trim(parent, start_parameter, end_parameter)
    };
    match curve {
        EdgeCurve::Line => None,
        EdgeCurve::Circle(circle) => angular(circle.project(start), circle.project(end)),
        EdgeCurve::Ellipse(ellipse) => angular(ellipse.project(start), ellipse.project(end)),
        EdgeCurve::Hyperbola(hyperbola) => Some((hyperbola.project(start), hyperbola.project(end))),
        EdgeCurve::Parabola(parabola) => Some((parabola.project(start), parabola.project(end))),
        EdgeCurve::NurbsCurve(nurbs) => {
            let project = |point| {
                brepkit_math::nurbs::projection::project_point_to_curve(nurbs, point, 1e-9)
                    .ok()
                    .map(|result| result.parameter)
            };
            Some((project(start)?, project(end)?))
        }
    }
}

fn angular_sub_trim(
    parent: (f64, f64),
    start_parameter: f64,
    end_parameter: f64,
) -> Option<(f64, f64)> {
    const EPS: f64 = 1e-12;
    const TAU: f64 = std::f64::consts::TAU;
    let (lo, hi) = (parent.0.min(parent.1), parent.0.max(parent.1));
    let lift = |parameter: f64, preferred: f64| {
        let mut lifted = TAU.mul_add(((preferred - parameter) / TAU).round(), parameter);
        if lifted < lo - EPS {
            lifted += TAU;
        }
        if lifted > hi + EPS {
            lifted -= TAU;
        }
        (lifted >= lo - EPS && lifted <= hi + EPS).then_some(lifted)
    };

    let start = lift(start_parameter, parent.0)?;
    let mut end = lift(end_parameter, start)?;
    if parent.1 >= parent.0 {
        if end < start - EPS {
            end += TAU;
        }
        (end <= hi + EPS).then_some((start, end))
    } else {
        if end > start + EPS {
            end -= TAU;
        }
        (end >= lo - EPS).then_some((start, end))
    }
}

/// An intersection curve between two faces, with pcurves on each.
///
/// Produced by face-face intersection. Consumed by the face splitter.
#[derive(Debug, Clone)]
pub struct SectionEdge {
    /// 3D intersection curve.
    pub curve_3d: EdgeCurve,
    /// Exact parameter interval tracing `start` to `end` on `curve_3d`.
    pub trim: Option<(f64, f64)>,
    /// pcurve on face A's surface.
    pub pcurve_a: Curve2D,
    /// pcurve on face B's surface.
    pub pcurve_b: Curve2D,
    /// 3D start point (trimmed to face boundaries).
    pub start: Point3,
    /// 3D end point (trimmed to face boundaries).
    pub end: Point3,
    /// Optional pre-computed UV endpoints on face A (avoids re-projection).
    /// When `Some`, `split_face_2d` uses these instead of projecting `start`/`end`.
    pub start_uv_a: Option<Point2>,
    /// Optional pre-computed UV endpoint on face A.
    pub end_uv_a: Option<Point2>,
    /// Optional pre-computed UV endpoints on face B.
    pub start_uv_b: Option<Point2>,
    /// Optional pre-computed UV endpoint on face B.
    pub end_uv_b: Option<Point2>,
    /// When set, this section edge only applies to this face during splitting.
    /// `None` means the edge applies to both faces in the pair (normal case).
    /// `Some(id)` means only distribute to that face (coplanar case -- each
    /// face gets boundary edges clipped to the other's interior).
    #[allow(dead_code)]
    pub target_face: Option<FaceId>,
    /// Pave block ID from the GFA arena. When two faces share the same
    /// FF section curve, their section edges have the same pave_block_id.
    /// Used by `build_topology_face` to share the topology edge entity
    /// across faces (shared edge identity for cross-face edges).
    pub pave_block_id: Option<usize>,
}

impl SectionEdge {
    /// Endpoint-derived structural support domain of this section edge.
    #[must_use]
    pub fn domain(&self) -> (f64, f64) {
        self.curve_3d.domain_with_endpoints(self.start, self.end)
    }
}

// ---------------------------------------------------------------------------
// Face-level types
// ---------------------------------------------------------------------------

/// All edges incident to a face, ready for 2D wire construction.
///
/// Boundary edges come from the face's original wire(s). Section edges
/// come from face-face intersections. Both must be expressed as pcurves
/// in the face's parameter space before feeding to the wire builder.
#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
pub struct FaceEdgeSet {
    /// Original boundary edges (possibly split at intersection vertices).
    pub boundary: Vec<OrientedPCurveEdge>,
    /// New edges from face-face intersections.
    pub section: Vec<OrientedPCurveEdge>,
}

/// A sub-face produced by the wire builder after face splitting.
///
/// Retains the parent face's surface geometry (never tessellated).
/// The wire loops are expressed in both 2D (for classification) and
/// 3D (for assembly).
#[derive(Debug, Clone)]
pub struct SplitSubFace {
    /// Surface from the parent face (preserved, never tessellated).
    pub surface: FaceSurface,
    /// Outer wire boundary in 2D + 3D.
    pub outer_wire: Vec<OrientedPCurveEdge>,
    /// Inner wire boundaries (holes).
    pub inner_wires: Vec<Vec<OrientedPCurveEdge>>,
    /// Whether the face normal is reversed relative to the surface.
    pub reversed: bool,
    /// The original face this sub-face was split from.
    #[allow(dead_code)]
    pub parent: FaceId,
    /// Which solid this face came from.
    #[allow(dead_code)]
    pub rank: Rank,
    /// Pre-computed interior point (3D) for classification.
    /// When set, `fill_images_faces` uses this instead of computing one
    /// from the UV polygon centroid.
    pub precomputed_interior: Option<Point3>,
}

// ---------------------------------------------------------------------------
// Surface info cache
// ---------------------------------------------------------------------------

/// Cached surface info per face for consistent UV operations across stages.
///
/// For plane faces, stores a [`PlaneFrame`] for 3D<->UV projection.
/// For analytic faces (cylinder, cone, sphere, torus), stores periodicity
/// flags -- UV projection uses the surface's native parameterization.
#[derive(Debug, Clone)]
pub enum SurfaceInfo {
    /// Plane face with a cached reference frame.
    #[allow(dead_code)]
    Plane(PlaneFrame),
    /// Parametric surface with native UV. Periodicity flags indicate whether
    /// the u or v parameter wraps (e.g. cylinder u in [0, 2pi)).
    Parametric {
        /// Whether the u parameter is periodic.
        u_periodic: bool,
        /// Whether the v parameter is periodic.
        v_periodic: bool,
    },
}

impl SurfaceInfo {
    /// Returns the `PlaneFrame` if this is a plane face, `None` otherwise.
    #[must_use]
    #[allow(dead_code)]
    pub fn as_plane_frame(&self) -> Option<&PlaneFrame> {
        match self {
            Self::Plane(f) => Some(f),
            Self::Parametric { .. } => None,
        }
    }

    /// Returns `(u_periodic, v_periodic)`.
    #[must_use]
    pub fn periodicity(&self) -> (bool, bool) {
        match self {
            Self::Plane(_) => (false, false),
            Self::Parametric {
                u_periodic,
                v_periodic,
            } => (*u_periodic, *v_periodic),
        }
    }
}

/// Construction lineage recorded while materializing and assembling result
/// edges (Issue 12). Store-space indices throughout.
#[derive(Debug, Default, Clone)]
pub struct EdgeLineageLog {
    /// Materialized wire edge index → the pave block it instantiates.
    pub to_pave_block: std::collections::BTreeMap<usize, usize>,
    /// Rebuilt edge index → the edge it was rebuilt from (weld remaps,
    /// canonicalization, arc chain splits).
    pub rewrites: std::collections::BTreeMap<usize, usize>,
}

#[cfg(test)]
mod trim_tests {
    use std::f64::consts::{PI, TAU};

    use super::angular_sub_trim;

    #[test]
    fn angular_sub_trim_keeps_full_circle_boundary_on_parent_winding() {
        assert_eq!(angular_sub_trim((0.0, TAU), 0.0, PI), Some((0.0, PI)));
        assert_eq!(angular_sub_trim((0.0, TAU), PI, 0.0), Some((PI, TAU)));
        assert_eq!(angular_sub_trim((TAU, 0.0), 0.0, PI), Some((TAU, PI)));
        assert_eq!(angular_sub_trim((TAU, 0.0), PI, 0.0), Some((PI, 0.0)));
    }
}
