#![allow(dead_code)]
//! Shared type definitions, constants, and the selection truth table for the
//! boolean pipeline.

use remus_math::surfaces::{CylindricalSurface, SphericalSurface};
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

/// Number of samples used when discretizing closed curves (circles, ellipses)
/// in the analytic boolean path. All code paths must use this constant so that
/// band fragments, cap face polygons, and holed-face inner wires share the
/// same vertices and edges at their boundaries.
pub(super) const CLOSED_CURVE_SAMPLES: usize = 32;

/// Minimum fragment count for parallel classification via rayon.
/// Below this threshold, sequential iteration is faster due to rayon's
/// thread-pool synchronization overhead (~5-20us).
#[cfg(not(target_arch = "wasm32"))]
pub(super) const PARALLEL_THRESHOLD: usize = 64;

/// Default tessellation deflection for non-planar faces in boolean operations.
///
/// A larger value produces fewer triangles (faster but coarser approximation).
/// Since the boolean result decomposes curved faces into individual planar
/// triangles, keeping this coarse avoids face-count explosion in sequential
/// boolean operations.
pub(super) const DEFAULT_BOOLEAN_DEFLECTION: f64 = 0.1;

/// Number of angular segments used to approximate cylinder faces in the
/// classification face data. 16 segments = 16 quads per cylinder band,
/// sufficient for correct ray-crossing parity.
pub(super) const CLASSIFIER_CYL_SEGMENTS: usize = 16;

/// Threshold: use CDT batch splitting for faces with this many or more chords.
///
/// Below this threshold, the iterative approach is fast enough and avoids the
/// CDT setup overhead. Above it, the iterative O(N*F) approach becomes a
/// bottleneck while CDT stays O(N log N).
pub(super) const CDT_CHORD_THRESHOLD: usize = 5;

/// Snap distance multiplier for CDT vertex matching.
///
/// Chord endpoints are computed by line-edge intersection, which accumulates
/// floating-point error on the order of ~10x `tol.linear`. Use 100x as the
/// snap threshold to reliably capture all on-chord/on-boundary vertices
/// without pulling in nearby-but-off-chord polygon vertices.
pub(super) const CDT_SNAP_FACTOR: f64 = 100.0;

/// Minimum face count for a valid solid.
///
/// A cylinder (2 caps + 1 barrel = 3 faces) is the minimal closed solid
/// produced by boolean operations between boxes and curved primitives.
pub(super) const MIN_SOLID_FACES: usize = 3;

/// The type of boolean operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    /// Union of two solids.
    Fuse,
    /// Subtraction: first minus second.
    Cut,
    /// Intersection: common volume.
    Intersect,
}

/// A face specification for mixed-surface solid assembly.
///
/// Used by `assemble_solid_mixed` to build solids with faces of any
/// surface type -- not just planar.
#[derive(Clone)]
pub enum FaceSpec {
    /// A planar face defined by vertex positions and plane equation.
    Planar {
        /// Vertex positions (at least 3).
        vertices: Vec<Point3>,
        /// Outward-facing normal.
        normal: Vec3,
        /// Plane equation signed distance (n * p = d).
        d: f64,
        /// Inner wire vertex loops (holes in the face).
        inner_wires: Vec<Vec<Point3>>,
    },
    /// A face with a pre-built surface and vertex positions for the boundary wire.
    Surface {
        /// Vertex positions for the outer wire (at least 3).
        vertices: Vec<Point3>,
        /// The surface geometry.
        surface: FaceSurface,
        /// Whether the face's surface normal should be reversed.
        reversed: bool,
        /// Inner wire vertex loops (holes in the face).
        inner_wires: Vec<Vec<Point3>>,
    },
    /// A cylindrical face with circle edges on angular boundaries.
    ///
    /// Unlike `Surface`, this variant creates `EdgeCurve::Circle` for edges
    /// that span an angular range on the cylinder (constant-v boundaries),
    /// preserving curve geometry for correct tessellation and volume computation.
    CylindricalFace {
        /// Vertex positions for the outer wire (at least 3).
        vertices: Vec<Point3>,
        /// The cylindrical surface geometry.
        cylinder: CylindricalSurface,
        /// Whether the face's surface normal should be reversed.
        reversed: bool,
        /// Inner wire vertex loops (holes in the face).
        inner_wires: Vec<Vec<Point3>>,
    },
    /// A spherical cap face (e.g. a fillet's corner ball patch) whose
    /// boundary edges are great-circle arcs of the sphere.
    ///
    /// Like `CylindricalFace`, this variant mints `EdgeCurve::Circle` edges —
    /// the short great-circle arc between each consecutive vertex pair — so
    /// adjacent faces share true arc geometry instead of straight chords:
    /// the shared edge pool then samples the real seam curve, edge display
    /// draws arcs, and STEP export keeps circles.
    SphereCapFace {
        /// Vertex positions for the outer wire (at least 3), on the sphere.
        vertices: Vec<Point3>,
        /// The spherical surface geometry.
        sphere: SphericalSurface,
        /// Whether the face's surface normal should be reversed.
        reversed: bool,
        /// Inner wire vertex loops (holes in the face).
        inner_wires: Vec<Vec<Point3>>,
    },
    /// A face whose wires are copied verbatim from an existing face.
    ///
    /// The other variants describe a wire as a list of vertex positions, so
    /// the assembler can only mint straight edges between consecutive
    /// positions. That vocabulary cannot express a loop whose whole boundary
    /// is one closed curve — a drilled hole's rim is a single circle edge
    /// with `start == end`, i.e. ONE position — and the mixed-spec assembler
    /// used to drop such loops (`< 3` positions) outright, which silently
    /// filled in the hole and left its bore wall with a free edge.
    ///
    /// This variant instead deep-copies the source face's wires, preserving
    /// every edge's exact curve, and registers the copies in the same
    /// vertex/edge dedup maps as the other specs so neighbouring rebuilt
    /// faces share them.
    Existing {
        /// The face whose surface, orientation, and wires are copied.
        face: FaceId,
        /// Replacement outer-wire vertex positions (a rebuilt/trimmed
        /// boundary), or `None` to copy the source's outer wire verbatim.
        /// Inner wires are always copied verbatim.
        outer: Option<Vec<Point3>>,
    },
}

impl FaceSpec {
    /// Returns a reference to this face's positional inner wires.
    ///
    /// [`Self::Existing`] carries its holes as topology rather than
    /// positions, so it reports none here; the assembler copies them.
    #[must_use]
    pub fn inner_wires(&self) -> &[Vec<Point3>] {
        match self {
            Self::Planar { inner_wires, .. }
            | Self::Surface { inner_wires, .. }
            | Self::CylindricalFace { inner_wires, .. }
            | Self::SphereCapFace { inner_wires, .. } => inner_wires,
            Self::Existing { .. } => &[],
        }
    }

    /// Returns a mutable slice of this face's positional inner wires.
    pub fn inner_wires_mut(&mut self) -> &mut [Vec<Point3>] {
        match self {
            Self::Planar { inner_wires, .. }
            | Self::Surface { inner_wires, .. }
            | Self::CylindricalFace { inner_wires, .. }
            | Self::SphereCapFace { inner_wires, .. } => inner_wires,
            Self::Existing { .. } => &mut [],
        }
    }

    /// Returns a mutable slice of this face's outer-wire vertex positions.
    ///
    /// Empty for an [`Self::Existing`] face that reuses its source wire.
    pub fn vertices_mut(&mut self) -> &mut [Point3] {
        match self {
            Self::Planar { vertices, .. }
            | Self::Surface { vertices, .. }
            | Self::CylindricalFace { vertices, .. }
            | Self::SphereCapFace { vertices, .. } => vertices,
            Self::Existing { outer, .. } => outer.as_deref_mut().unwrap_or(&mut []),
        }
    }

    /// Returns this face's outer-wire vertex positions, if it has any.
    #[must_use]
    pub fn vertices(&self) -> &[Point3] {
        match self {
            Self::Planar { vertices, .. }
            | Self::Surface { vertices, .. }
            | Self::CylindricalFace { vertices, .. }
            | Self::SphereCapFace { vertices, .. } => vertices,
            Self::Existing { outer, .. } => outer.as_deref().unwrap_or(&[]),
        }
    }
}

/// Options for boolean operations.
#[derive(Debug, Clone, Copy)]
pub struct BooleanOptions {
    /// Tessellation deflection for the mesh fallback, in model units.
    ///
    /// Lower values produce more triangles (more accurate but slower).
    /// Default: 0.1.
    pub deflection: f64,
    /// Tolerance for geometric comparisons.
    ///
    /// Controls fast-path decisions, GFA predicates, mesh welding,
    /// validation thresholds, and requested post-processing. Default:
    /// `Tolerance::new()`.
    pub tolerance: Tolerance,
    /// Merge co-surface face fragments after assembly.
    ///
    /// When `true`, the result is post-processed to merge adjacent faces that
    /// share the same underlying surface (same-domain
    /// analysis). This dramatically reduces face count -- e.g. sequential
    /// booleans on curved surfaces drop from 2871 to ~106 faces.
    ///
    /// Non-convex merged faces are handled correctly by the
    /// `polygon_clip_intervals` fallback in the analytic chord splitter,
    /// so this is safe for intermediate results fed into further booleans.
    ///
    /// Internal validity repairs may still merge faces even when this is
    /// `false`; this flag controls the optional result simplification pass.
    /// If same-domain merging would make an otherwise valid boolean result
    /// invalid, that simplification is rolled back and the valid unsimplified
    /// result is returned.
    /// Default: `true`.
    pub unify_faces: bool,
    /// Run full shape healing on every successful boolean result via
    /// [`crate::heal::heal_solid`]. A healing or post-heal validation failure
    /// fails and rolls back the whole operation.
    ///
    /// Use for final results only -- healing can corrupt intermediates fed into
    /// further booleans (non-convex merged faces confuse chord splitting).
    ///
    /// Default: `false`.
    pub heal_after_boolean: bool,
}

impl Default for BooleanOptions {
    fn default() -> Self {
        Self {
            deflection: DEFAULT_BOOLEAN_DEFLECTION,
            tolerance: Tolerance::new(),
            unify_faces: true,
            heal_after_boolean: false,
        }
    }
}

impl BooleanOptions {
    /// Convert these legacy options into the authoritative operation context.
    ///
    /// The option tolerance becomes the context tolerance and `deflection`
    /// becomes the allowed-approximation budget. Work budgets retain their
    /// context defaults because `BooleanOptions` has never exposed them.
    #[must_use]
    pub fn operation_context(self) -> remus_math::context::OperationContext {
        remus_math::context::OperationContext::new()
            .with_tolerance(self.tolerance)
            .with_fallback(remus_math::context::FallbackPolicy::AllowApproximate {
                budget: self.deflection,
            })
    }
}

/// Which operand a face fragment originated from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    A,
    B,
}

pub(super) use remus_algo::FaceClass;

/// Result of classifying an intersection curve against a face boundary.
pub(super) enum CurveClassification {
    /// The curve crosses the face boundary -- contains entry/exit points.
    Crossings(Vec<Point3>),
    /// The entire curve lies inside the face (no boundary crossings).
    FullyContained,
    /// The entire curve lies outside the face.
    FullyOutside,
}

/// Internal context carrying tolerance-derived thresholds through the boolean
/// pipeline. Computed once from `BooleanOptions` at the start of a boolean
/// operation to avoid repeated derivation and hardcoded epsilon values.
#[derive(Debug, Clone, Copy)]
pub(super) struct BooleanContext {
    /// Base tolerance (used when wiring ctx through the full pipeline).
    #[allow(dead_code)]
    pub(super) tol: Tolerance,
    /// Vertex merge distance: vertices closer than this are considered identical.
    pub(super) vertex_merge: f64,
    /// Point classification tolerance: distance threshold for on-surface tests.
    pub(super) classify_tol: f64,
    /// Degenerate polygon threshold: skip polygons with area below this.
    pub(super) degenerate_area: f64,
}

impl BooleanContext {
    pub(super) fn from_options(opts: &BooleanOptions) -> Self {
        let tol = opts.tolerance;
        Self {
            tol,
            // 1000x linear tolerance for vertex merging -- aggressive enough to
            // catch coincident vertices while preserving distinct features.
            vertex_merge: tol.linear * 1000.0,
            // Classification tolerance for point-in-solid tests.
            classify_tol: tol.linear * 100.0,
            // Degenerate area threshold (area < this -> skip polygon).
            degenerate_area: tol.linear * tol.linear,
        }
    }
}

/// An intersection segment between two faces.
#[derive(Debug)]
pub(super) struct IntersectionSegment {
    pub(super) face_a: FaceId,
    pub(super) face_b: FaceId,
    pub(super) p0: Point3,
    pub(super) p1: Point3,
}

/// A fragment of a face after splitting along intersection chords.
#[derive(Debug)]
pub(super) struct FaceFragment {
    pub(super) vertices: Vec<Point3>,
    pub(super) normal: Vec3,
    pub(super) d: f64,
    pub(super) source: Source,
}

/// Parameters for a single face in a face-pair intersection test.
pub(super) struct FacePairSide<'a> {
    pub(super) fid: FaceId,
    pub(super) verts: &'a [Point3],
    pub(super) normal: Vec3,
    pub(super) d: f64,
}

/// Snapshot of face data for analytic boolean processing.
pub(super) struct FaceSnapshot {
    pub(super) id: FaceId,
    pub(super) surface: FaceSurface,
    pub(super) vertices: Vec<Point3>,
    pub(super) normal: Vec3,
    pub(super) d: f64,
    /// Whether the original face was reversed (needed to preserve orientation
    /// when carrying unsplit faces through sequential booleans).
    pub(super) reversed: bool,
}

/// Analytic face fragment preserving the original surface type.
pub(super) struct AnalyticFragment {
    /// Polygon boundary in 3D (for classification and planar assembly fallback).
    pub(super) vertices: Vec<Point3>,
    /// The original surface type of the face.
    pub(super) surface: FaceSurface,
    /// Normal of the face (for planar) or of the polygon approximation.
    pub(super) normal: Vec3,
    /// Plane d coefficient (for planar faces).
    pub(super) d: f64,
    /// Which operand this fragment came from.
    pub(super) source: Source,
    /// Edge curve types for the boundary segments.
    /// `None` = straight line, `Some(curve)` = exact curve (circle, ellipse).
    pub(super) edge_curves: Vec<Option<EdgeCurve>>,
    /// Whether the source face was reversed (preserved for non-planar faces).
    pub(super) source_reversed: bool,
    /// The original input `FaceId` this fragment was created from.
    /// Used by `BooleanState` to track provenance (images/origins).
    pub(super) source_face_id: Option<FaceId>,
}

/// Extracted face data: `(FaceId, vertices, normal, d)`.
pub(super) type FaceData = Vec<(FaceId, Vec<Point3>, Vec3, f64)>;

/// Determine whether a fragment should be kept and whether to flip its normal.
///
/// Returns `Some(false)` to keep as-is, `Some(true)` to keep and flip, or
/// `None` to discard.
#[allow(clippy::match_same_arms)] // arms are semantically distinct (truth table rows)
pub(super) const fn select_fragment(
    source: Source,
    class: FaceClass,
    op: BooleanOp,
) -> Option<bool> {
    match (source, class, op) {
        // From A, Outside B
        (Source::A, FaceClass::Outside, BooleanOp::Fuse | BooleanOp::Cut) => Some(false),
        (Source::A, FaceClass::Outside, BooleanOp::Intersect) => None,
        // From A, Inside B
        (Source::A, FaceClass::Inside, BooleanOp::Fuse | BooleanOp::Cut) => None,
        (Source::A, FaceClass::Inside, BooleanOp::Intersect) => Some(false),
        // From B, Outside A
        (Source::B, FaceClass::Outside, BooleanOp::Fuse) => Some(false),
        (Source::B, FaceClass::Outside, BooleanOp::Cut | BooleanOp::Intersect) => None,
        // From B, Inside A
        (Source::B, FaceClass::Inside, BooleanOp::Fuse) => None,
        (Source::B, FaceClass::Inside, BooleanOp::Cut) => Some(true), // flip
        (Source::B, FaceClass::Inside, BooleanOp::Intersect) => Some(false),
        // Coplanar same -- keep only from A to avoid duplicates.
        (Source::A, FaceClass::CoplanarSame, BooleanOp::Fuse | BooleanOp::Intersect) => Some(false),
        (_, FaceClass::CoplanarSame, _) => None,
        // Coplanar opposite -- for Cut, A's face facing opposite B should be kept
        // (it forms the "skin" at the cut boundary). In all other cases, discard.
        (Source::A, FaceClass::CoplanarOpposite, BooleanOp::Cut) => Some(false),
        (_, FaceClass::CoplanarOpposite, _) => None,
        // On boundary -- treat like CoplanarSame: keep from A only.
        (Source::A, FaceClass::On, BooleanOp::Fuse | BooleanOp::Cut | BooleanOp::Intersect) => {
            Some(false)
        }
        (_, FaceClass::On, _) => None,
        // Unknown is only used by the algo crate's builder; never emitted by
        // the operations pipeline classifier.
        (_, FaceClass::Unknown, _) => {
            debug_assert!(
                false,
                "FaceClass::Unknown must never reach fragment selection"
            );
            None
        }
    }
}
