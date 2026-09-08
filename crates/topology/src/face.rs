//! Face — a bounded region of a surface.
//!
//! # Orientation semantics
//!
//! Each face has a `reversed` flag that relates the face's topological
//! orientation to the geometric surface normal. When `reversed` is `false`,
//! the face's outward normal coincides with the surface normal. When `true`,
//! the logical outward normal is opposite to the geometric surface normal.
//! This is used by boolean operations when a curved face must contribute
//! with flipped winding without altering the underlying surface definition.

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::traits::ParametricSurface;
use remus_math::vec::{Point3, Vec3};

use crate::arena;
use crate::face_loop::LoopId;
use crate::wire::WireId;

/// Typed handle for a [`Face`] stored in an [`Arena`](crate::Arena).
pub type FaceId = arena::Id<Face>;

/// The geometric surface associated with a face.
#[derive(Debug, Clone)]
pub enum FaceSurface {
    /// An infinite plane defined by a normal vector and signed distance from
    /// the origin.
    Plane {
        /// Outward-pointing normal of the plane.
        normal: Vec3,
        /// Signed distance from the origin along the normal.
        d: f64,
    },
    /// A NURBS surface.
    Nurbs(NurbsSurface),
    /// A cylindrical surface.
    Cylinder(CylindricalSurface),
    /// A conical surface.
    Cone(ConicalSurface),
    /// A spherical surface.
    Sphere(SphericalSurface),
    /// A toroidal surface.
    Torus(ToroidalSurface),
}

impl FaceSurface {
    /// Evaluate the surface at parameters `(u, v)`.
    ///
    /// Returns `None` for `Plane` since it has no true UV parameterization.
    /// For analytic and NURBS surfaces, dispatches to the
    /// [`ParametricSurface`] trait implementation.
    #[must_use]
    pub fn evaluate(&self, u: f64, v: f64) -> Option<Point3> {
        match self {
            Self::Plane { .. } => None,
            Self::Cylinder(c) => Some(ParametricSurface::evaluate(c, u, v)),
            Self::Cone(c) => Some(ParametricSurface::evaluate(c, u, v)),
            Self::Sphere(s) => Some(ParametricSurface::evaluate(s, u, v)),
            Self::Torus(t) => Some(ParametricSurface::evaluate(t, u, v)),
            Self::Nurbs(n) => Some(ParametricSurface::evaluate(n, u, v)),
        }
    }

    /// Surface normal at parameters `(u, v)`.
    ///
    /// For `Plane`, returns the stored normal directly (ignoring `u`, `v`).
    /// For analytic and NURBS surfaces, dispatches to the
    /// [`ParametricSurface`] trait implementation. NURBS surfaces fall back
    /// to `Vec3::Z` at degenerate points.
    #[must_use]
    pub fn normal(&self, u: f64, v: f64) -> Vec3 {
        match self {
            Self::Plane { normal, .. } => *normal,
            Self::Cylinder(c) => ParametricSurface::normal(c, u, v),
            Self::Cone(c) => ParametricSurface::normal(c, u, v),
            Self::Sphere(s) => ParametricSurface::normal(s, u, v),
            Self::Torus(t) => ParametricSurface::normal(t, u, v),
            Self::Nurbs(n) => ParametricSurface::normal(n, u, v),
        }
    }

    /// Project a 3D point onto the surface, returning `(u, v)` parameters.
    ///
    /// Returns `None` for `Plane` (no true UV parameterization).
    /// For analytic and NURBS surfaces, dispatches to the
    /// [`ParametricSurface`] trait implementation.
    #[must_use]
    pub fn project_point(&self, point: Point3) -> Option<(f64, f64)> {
        match self {
            Self::Plane { .. } => None,
            Self::Cylinder(c) => Some(ParametricSurface::project_point(c, point)),
            Self::Cone(c) => Some(ParametricSurface::project_point(c, point)),
            Self::Sphere(s) => Some(ParametricSurface::project_point(s, point)),
            Self::Torus(t) => Some(ParametricSurface::project_point(t, point)),
            Self::Nurbs(n) => Some(ParametricSurface::project_point(n, point)),
        }
    }

    /// Partial derivative ∂S/∂u at parameters `(u, v)`.
    ///
    /// Returns `None` for `Plane` since it has no true UV parameterization.
    /// For analytic and NURBS surfaces, dispatches to the
    /// [`ParametricSurface`] trait implementation.
    #[must_use]
    pub fn partial_u(&self, u: f64, v: f64) -> Option<Vec3> {
        match self {
            Self::Plane { .. } => None,
            Self::Cylinder(c) => Some(ParametricSurface::partial_u(c, u, v)),
            Self::Cone(c) => Some(ParametricSurface::partial_u(c, u, v)),
            Self::Sphere(s) => Some(ParametricSurface::partial_u(s, u, v)),
            Self::Torus(t) => Some(ParametricSurface::partial_u(t, u, v)),
            Self::Nurbs(n) => Some(ParametricSurface::partial_u(n, u, v)),
        }
    }

    /// Partial derivative ∂S/∂v at parameters `(u, v)`.
    ///
    /// Returns `None` for `Plane` since it has no true UV parameterization.
    /// For analytic and NURBS surfaces, dispatches to the
    /// [`ParametricSurface`] trait implementation.
    #[must_use]
    pub fn partial_v(&self, u: f64, v: f64) -> Option<Vec3> {
        match self {
            Self::Plane { .. } => None,
            Self::Cylinder(c) => Some(ParametricSurface::partial_v(c, u, v)),
            Self::Cone(c) => Some(ParametricSurface::partial_v(c, u, v)),
            Self::Sphere(s) => Some(ParametricSurface::partial_v(s, u, v)),
            Self::Torus(t) => Some(ParametricSurface::partial_v(t, u, v)),
            Self::Nurbs(n) => Some(ParametricSurface::partial_v(n, u, v)),
        }
    }

    /// Estimate a characteristic radius for tessellation density.
    ///
    /// Returns the radius for cylinder/sphere, a mid-generator radius for
    /// cones, and the major radius for tori. For NURBS, estimates from the
    /// control-point bounding-box diagonal. Returns `f64::INFINITY` for planes.
    #[must_use]
    pub fn estimate_radius(&self) -> f64 {
        match self {
            Self::Plane { .. } => f64::INFINITY,
            Self::Cylinder(c) => c.radius(),
            Self::Cone(c) => c.radius_at(1.0),
            Self::Sphere(s) => s.radius(),
            Self::Torus(t) => t.major_radius(),
            Self::Nurbs(n) => {
                // Estimate from control-point spread: half the bounding-box diagonal.
                let cps = n.control_points();
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for row in cps {
                    for p in row {
                        min[0] = min[0].min(p.x());
                        min[1] = min[1].min(p.y());
                        min[2] = min[2].min(p.z());
                        max[0] = max[0].max(p.x());
                        max[1] = max[1].max(p.y());
                        max[2] = max[2].max(p.z());
                    }
                }
                let dx = max[0] - min[0];
                let dy = max[1] - min[1];
                let dz = max[2] - min[2];
                dx.hypot(dy).hypot(dz) * 0.5
            }
        }
    }

    /// Type tag string for debugging and serialization.
    #[must_use]
    pub const fn type_tag(&self) -> &'static str {
        match self {
            Self::Plane { .. } => "plane",
            Self::Cylinder(_) => "cylinder",
            Self::Cone(_) => "cone",
            Self::Sphere(_) => "sphere",
            Self::Torus(_) => "torus",
            Self::Nurbs(_) => "nurbs",
        }
    }

    /// Whether this surface is planar.
    #[must_use]
    pub const fn is_planar(&self) -> bool {
        matches!(self, Self::Plane { .. })
    }

    /// Whether this surface is analytic (non-NURBS).
    #[must_use]
    pub const fn is_analytic(&self) -> bool {
        !matches!(self, Self::Nurbs(_))
    }

    /// Convert to an [`AnalyticSurface`](remus_math::analytic_intersection::AnalyticSurface)
    /// reference if applicable.
    ///
    /// Returns `None` for `Plane` and `Nurbs` variants.
    #[must_use]
    pub fn as_analytic(&self) -> Option<remus_math::analytic_intersection::AnalyticSurface<'_>> {
        use remus_math::analytic_intersection::AnalyticSurface;
        match self {
            Self::Cylinder(c) => Some(AnalyticSurface::Cylinder(c)),
            Self::Cone(c) => Some(AnalyticSurface::Cone(c)),
            Self::Sphere(s) => Some(AnalyticSurface::Sphere(s)),
            Self::Torus(t) => Some(AnalyticSurface::Torus(t)),
            Self::Plane { .. } | Self::Nurbs(_) => None,
        }
    }
}

/// A topological face: a bounded region of a surface.
///
/// A face has exactly one authoritative outer loop and zero or more inner
/// loops. Boundary wires are a compatibility facade kept synchronized by
/// topology-owned mutation APIs; strict validation refuses facade divergence.
///
/// # Periodic surfaces
///
/// Two boundary representations are supported for a full-revolution band on a
/// periodic surface (cylinder, cone, sphere, torus):
///
/// * **Doubled seam** (what the kernel's own builders emit): a single outer
///   wire walks both rims and the seam twice — e.g. `bottom rim, seam up, top
///   rim reversed, seam down`.
/// * **Two-ring**: the outer wire is one full-turn rim and a single inner
///   wire is the other rim, wound opposite in the surface's (u, v) parameter
///   space, with no seam edge anywhere. Tessellation recognizes this shape
///   for cylinder/cone (structured band) and sphere/torus (latitude band)
///   walls; other wire layouts on periodic faces fall back to chart-based
///   meshing, which cannot represent ring boundaries.
///
/// A two-ring band's inner wire still follows the usual convention: wound
/// opposite the outer wire in the surface's (u, v) parameter space.
#[derive(Debug, Clone)]
pub struct Face {
    /// The outer boundary wire of this face.
    outer_wire: WireId,
    /// Inner boundary wires representing holes in this face.
    inner_wires: Vec<WireId>,
    /// Authoritative boundary loops in outer-then-inner order. Empty only
    /// until a newly allocated face is promoted, or for invalid legacy input.
    boundary_loops: Vec<LoopId>,
    /// The geometric surface underlying this face.
    surface: FaceSurface,
    /// Whether the face orientation is reversed relative to the surface normal.
    ///
    /// When `true`, the face's topological orientation (outward normal for
    /// volume computation) is opposite to the geometric surface normal. This
    /// is used by boolean operations when a curved face must contribute with
    /// flipped winding without altering the underlying surface definition.
    reversed: bool,
}

impl Face {
    /// Creates a new face with the given outer wire, inner wires, and surface.
    #[must_use]
    pub const fn new(outer_wire: WireId, inner_wires: Vec<WireId>, surface: FaceSurface) -> Self {
        Self {
            outer_wire,
            inner_wires,
            boundary_loops: Vec::new(),
            surface,
            reversed: false,
        }
    }

    /// Creates a new face with reversed orientation relative to the surface normal.
    ///
    /// Used by boolean operations when a curved face must contribute with
    /// flipped winding (e.g., a cylinder face from the tool solid that becomes
    /// part of the result with opposite orientation).
    #[must_use]
    pub fn new_reversed(
        outer_wire: WireId,
        inner_wires: Vec<WireId>,
        surface: FaceSurface,
    ) -> Self {
        Self {
            outer_wire,
            inner_wires,
            boundary_loops: Vec::new(),
            surface,
            reversed: true,
        }
    }

    /// Returns the outer boundary wire of this face.
    #[must_use]
    pub const fn outer_wire(&self) -> WireId {
        self.outer_wire
    }

    /// Sets the outer boundary wire.
    #[deprecated(
        since = "0.1.0",
        note = "use Topology::set_face_boundary_wires so Loop/Coedge authority stays synchronized"
    )]
    pub fn set_outer_wire(&mut self, wire_id: WireId) {
        self.outer_wire = wire_id;
    }

    /// Returns the inner boundary wires (holes) of this face.
    #[must_use]
    pub fn inner_wires(&self) -> &[WireId] {
        &self.inner_wires
    }

    /// Returns the authoritative outer boundary loop, when installed.
    #[must_use]
    pub fn outer_loop(&self) -> Option<LoopId> {
        self.boundary_loops.first().copied()
    }

    /// Returns the authoritative inner boundary loops.
    #[must_use]
    pub fn inner_loops(&self) -> &[LoopId] {
        self.boundary_loops.get(1..).unwrap_or_default()
    }

    /// Returns every authoritative boundary loop in outer-then-inner order.
    #[must_use]
    pub fn boundary_loops(&self) -> &[LoopId] {
        &self.boundary_loops
    }

    /// Returns a mutable reference to the inner wires list.
    #[deprecated(
        since = "0.1.0",
        note = "use Topology::set_face_boundary_wires so Loop/Coedge authority stays synchronized"
    )]
    pub fn inner_wires_mut(&mut self) -> &mut Vec<WireId> {
        &mut self.inner_wires
    }

    /// Replaces every boundary-wire reference in one operation.
    ///
    /// Public callers use [`Topology::set_face_boundary_wires`](crate::Topology::set_face_boundary_wires),
    /// which validates the complete replacement and keeps derived loops and
    /// pcurve uses coherent before calling this commit-only helper.
    pub(crate) fn replace_boundary_wires(&mut self, outer_wire: WireId, inner_wires: Vec<WireId>) {
        self.outer_wire = outer_wire;
        self.inner_wires = inner_wires;
    }

    /// Replaces the authoritative boundary-loop handles after a validated
    /// topology-owned boundary commit.
    pub(crate) fn replace_boundary_loops(&mut self, loops: Vec<LoopId>) {
        self.boundary_loops = loops;
    }

    /// Returns a reference to the surface geometry of this face.
    #[must_use]
    pub const fn surface(&self) -> &FaceSurface {
        &self.surface
    }

    /// Sets the surface geometry of this face.
    pub fn set_surface(&mut self, surface: FaceSurface) {
        self.surface = surface;
    }

    /// Returns whether this face's orientation is reversed relative to its
    /// surface normal.
    #[must_use]
    pub const fn is_reversed(&self) -> bool {
        self.reversed
    }

    /// Sets whether this face's orientation is reversed relative to its
    /// surface normal.
    pub fn set_reversed(&mut self, reversed: bool) {
        self.reversed = reversed;
    }

    /// Returns the effective plane normal, accounting for the `reversed` flag.
    ///
    /// Returns `None` if the surface is not planar.
    #[must_use]
    pub fn effective_plane_normal(&self) -> Option<Vec3> {
        match &self.surface {
            FaceSurface::Plane { normal, .. } => {
                if self.reversed {
                    Some(-*normal)
                } else {
                    Some(*normal)
                }
            }
            _ => None,
        }
    }

    /// Composes orientation: toggles the `reversed` flag if `flip` is true.
    pub fn compose_orientation(&mut self, flip: bool) {
        if flip {
            self.reversed = !self.reversed;
        }
    }
}
