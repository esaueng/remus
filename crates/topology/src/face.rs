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

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use remus_math::analytic_intersection::AnalyticSurface;
    use remus_math::nurbs::surface::NurbsSurface;
    use remus_math::surfaces::{
        ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface,
    };
    use remus_math::vec::{Point3, Vec3};

    use super::{Face, FaceSurface};
    use crate::WireId;
    use crate::edge::{Edge, EdgeCurve};
    use crate::topology::Topology;
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    /// Tolerance for quantities derived from a closed-form parameterisation.
    const TOL: f64 = 1e-9;
    /// Looser tolerance for iterative projections.
    const PROJ_TOL: f64 = 1e-6;

    /// A deliberately oblique unit axis, so an axis-aligned mistake shows up.
    fn oblique_axis() -> Vec3 {
        Vec3::new(1.0, 2.0, 2.0)
    }

    fn unit_axis() -> Vec3 {
        oblique_axis().normalize().unwrap()
    }

    /// Off-origin cylinder, radius neither 0 nor 1.
    fn cylinder() -> CylindricalSurface {
        CylindricalSurface::new(Point3::new(4.0, -3.0, 2.5), oblique_axis(), 2.75).unwrap()
    }

    /// Off-origin cone, half-angle away from every degenerate value.
    fn cone() -> ConicalSurface {
        ConicalSurface::new(Point3::new(-2.0, 1.5, 3.0), oblique_axis(), 0.6).unwrap()
    }

    /// Off-origin sphere with an oblique polar axis, radius neither 0 nor 1.
    fn sphere() -> SphericalSurface {
        SphericalSurface::with_axis(Point3::new(1.5, -2.5, 4.0), 3.5, oblique_axis()).unwrap()
    }

    /// Off-origin torus with an oblique axis, both radii neither 0 nor 1.
    fn torus() -> ToroidalSurface {
        ToroidalSurface::with_axis(Point3::new(-1.0, 2.0, -3.5), 4.25, 1.25, oblique_axis())
            .unwrap()
    }

    /// A bilinear NURBS patch whose control-point box is
    /// x∈[3, 9], y∈[-4, 2], z∈[1, 5] — every extent distinct from its own
    /// min, so a corrupted `max - min` changes the estimated radius.
    fn nurbs() -> NurbsSurface {
        NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![Point3::new(3.0, -4.0, 1.0), Point3::new(9.0, -4.0, 5.0)],
                vec![Point3::new(3.0, 2.0, 5.0), Point3::new(9.0, 2.0, 1.0)],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .unwrap()
    }

    fn plane_surface() -> FaceSurface {
        FaceSurface::Plane {
            normal: unit_axis(),
            d: 1.75,
        }
    }

    /// The five surfaces that carry a real UV parameterisation.
    fn parametric_surfaces() -> Vec<(&'static str, FaceSurface)> {
        vec![
            ("cylinder", FaceSurface::Cylinder(cylinder())),
            ("cone", FaceSurface::Cone(cone())),
            ("sphere", FaceSurface::Sphere(sphere())),
            ("torus", FaceSurface::Torus(torus())),
            ("nurbs", FaceSurface::Nurbs(nurbs())),
        ]
    }

    /// A sample `(u, v)` inside every one of those surfaces' domains.
    const SAMPLE_UV: (f64, f64) = (0.7, 0.55);

    /// Component of `w` along `axis`, and the length of what is left.
    fn split_along(w: Vec3, axis: Vec3) -> (f64, f64) {
        let axial = w.dot(axis);
        (axial, (w - axis * axial).length())
    }

    fn tri_wire(topo: &mut Topology, ox: f64, oy: f64, size: f64) -> WireId {
        let v0 = topo.add_vertex(Vertex::new(Point3::new(ox, oy, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(ox + size, oy, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(ox, oy + size, 0.0), 1e-7));
        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));
        topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(e0, true),
                    OrientedEdge::new(e1, true),
                    OrientedEdge::new(e2, true),
                ],
                true,
            )
            .unwrap(),
        )
    }

    // -----------------------------------------------------------------
    // FaceSurface delegates
    // -----------------------------------------------------------------

    #[test]
    fn evaluate_matches_the_closed_form_of_every_variant() {
        let (u, v) = SAMPLE_UV;
        let axis = unit_axis();

        // Plane has no true UV parameterization.
        assert!(plane_surface().evaluate(u, v).is_none());

        let cyl = cylinder();
        let p = FaceSurface::Cylinder(cyl.clone()).evaluate(u, v).unwrap();
        let (axial, radial) = split_along(p - cyl.origin(), axis);
        assert!((axial - v).abs() < TOL, "cylinder axial {axial}");
        assert!((radial - 2.75).abs() < TOL, "cylinder radial {radial}");

        let con = cone();
        let p = FaceSurface::Cone(con.clone()).evaluate(u, v).unwrap();
        let (axial, radial) = split_along(p - con.apex(), axis);
        assert!(
            (axial - v * 0.6_f64.sin()).abs() < TOL,
            "cone axial {axial}"
        );
        assert!(
            (radial - con.radius_at(v)).abs() < TOL,
            "cone radial {radial}"
        );

        let sph = sphere();
        let p = FaceSurface::Sphere(sph.clone()).evaluate(u, v).unwrap();
        let (axial, _) = split_along(p - sph.center(), axis);
        assert!(((p - sph.center()).length() - 3.5).abs() < TOL);
        assert!((axial - 3.5 * v.sin()).abs() < TOL, "sphere axial {axial}");

        let tor = torus();
        let p = FaceSurface::Torus(tor.clone()).evaluate(u, v).unwrap();
        let (axial, radial) = split_along(p - tor.center(), axis);
        assert!((axial - 1.25 * v.sin()).abs() < TOL, "torus axial {axial}");
        assert!(
            (radial - (4.25 + 1.25 * v.cos())).abs() < TOL,
            "torus radial {radial}"
        );

        // A bilinear patch at the centre of its domain is the control
        // points' average, whichever way the grid is indexed.
        let p = FaceSurface::Nurbs(nurbs()).evaluate(0.5, 0.5).unwrap();
        assert!((p.x() - 6.0).abs() < TOL);
        assert!((p.y() + 1.0).abs() < TOL);
        assert!((p.z() - 3.0).abs() < TOL);
    }

    #[test]
    fn normal_is_unit_length_and_consistent_with_the_surface() {
        let (u, v) = SAMPLE_UV;

        // Plane hands back exactly the stored normal, ignoring (u, v).
        let stored = unit_axis();
        let n = plane_surface().normal(u, v);
        assert!((n - stored).length() < TOL);
        assert!((plane_surface().normal(-3.0, 8.0) - stored).length() < TOL);

        for (name, surface) in parametric_surfaces() {
            let n = surface.normal(u, v);
            assert!(
                (n.length() - 1.0).abs() < 1e-7,
                "{name} normal not unit: {}",
                n.length()
            );
        }

        // Cylinder and sphere normals point radially outward.
        let cyl = cylinder();
        let surface = FaceSurface::Cylinder(cyl.clone());
        let p = surface.evaluate(u, v).unwrap();
        let w = p - cyl.origin();
        let radial = w - unit_axis() * w.dot(unit_axis());
        assert!(surface.normal(u, v).dot(radial.normalize().unwrap()) > 0.999_999);

        let sph = sphere();
        let surface = FaceSurface::Sphere(sph.clone());
        let p = surface.evaluate(u, v).unwrap();
        let outward = (p - sph.center()).normalize().unwrap();
        assert!(surface.normal(u, v).dot(outward) > 0.999_999);
    }

    #[test]
    fn project_point_round_trips_on_every_parametric_variant() {
        let (u, v) = SAMPLE_UV;

        // A plane has no UV parameterization, so it must project to None.
        // This is also what rules out every constant `Some((a, b))`.
        assert!(
            plane_surface()
                .project_point(Point3::new(1.0, 2.0, 3.0))
                .is_none()
        );

        for (name, surface) in parametric_surfaces() {
            let p = surface.evaluate(u, v).unwrap();
            let (pu, pv) = surface
                .project_point(p)
                .unwrap_or_else(|| panic!("{name} must project"));
            let back = surface.evaluate(pu, pv).unwrap();
            assert!(
                (back - p).length() < PROJ_TOL,
                "{name} projection did not land on the surface: {:?} -> ({pu}, {pv})",
                p
            );
        }
    }

    #[test]
    fn partials_span_the_tangent_plane_and_planes_have_none() {
        let (u, v) = SAMPLE_UV;

        assert!(plane_surface().partial_u(u, v).is_none());
        assert!(plane_surface().partial_v(u, v).is_none());

        for (name, surface) in parametric_surfaces() {
            let du = surface
                .partial_u(u, v)
                .unwrap_or_else(|| panic!("{name} partial_u"));
            let dv = surface
                .partial_v(u, v)
                .unwrap_or_else(|| panic!("{name} partial_v"));
            let cross = du.cross(dv);
            assert!(cross.length() > 1e-6, "{name} partials are degenerate");
            let aligned = cross.normalize().unwrap().dot(surface.normal(u, v)).abs();
            assert!(
                (aligned - 1.0).abs() < 1e-6,
                "{name} partials do not span the tangent plane: {aligned}"
            );
        }

        // The cylinder's partials are pinned by its closed form:
        // |∂S/∂u| = radius and ∂S/∂v is the unit axis.
        let surface = FaceSurface::Cylinder(cylinder());
        let du = surface.partial_u(u, v).unwrap();
        let dv = surface.partial_v(u, v).unwrap();
        assert!((du.length() - 2.75).abs() < TOL);
        assert!(du.dot(unit_axis()).abs() < TOL);
        assert!((dv - unit_axis()).length() < TOL);
    }

    #[test]
    fn estimate_radius_reports_each_variant_defining_radius() {
        let infinite = plane_surface().estimate_radius();
        assert!(
            infinite.is_infinite() && infinite.is_sign_positive(),
            "plane radius should be +inf, got {infinite}"
        );

        assert!((FaceSurface::Cylinder(cylinder()).estimate_radius() - 2.75).abs() < TOL);
        assert!((FaceSurface::Sphere(sphere()).estimate_radius() - 3.5).abs() < TOL);
        assert!((FaceSurface::Torus(torus()).estimate_radius() - 4.25).abs() < TOL);
        // Mid-generator radius: the cone's radius one unit from the apex.
        assert!(
            (FaceSurface::Cone(cone()).estimate_radius() - 0.6_f64.cos()).abs() < TOL,
            "cone radius {}",
            FaceSurface::Cone(cone()).estimate_radius()
        );
    }

    #[test]
    fn estimate_radius_of_nurbs_is_half_the_control_box_diagonal() {
        // Control box extents: dx = 9 - 3 = 6, dy = 2 - (-4) = 6, dz = 5 - 1 = 4.
        let expected = 0.5 * 88.0_f64.sqrt();
        let got = FaceSurface::Nurbs(nurbs()).estimate_radius();
        assert!(
            (got - expected).abs() < TOL,
            "nurbs radius {got}, expected {expected}"
        );
    }

    #[test]
    fn type_tag_is_distinct_and_stable_across_all_six_variants() {
        assert_eq!(plane_surface().type_tag(), "plane");
        assert_eq!(FaceSurface::Cylinder(cylinder()).type_tag(), "cylinder");
        assert_eq!(FaceSurface::Cone(cone()).type_tag(), "cone");
        assert_eq!(FaceSurface::Sphere(sphere()).type_tag(), "sphere");
        assert_eq!(FaceSurface::Torus(torus()).type_tag(), "torus");
        assert_eq!(FaceSurface::Nurbs(nurbs()).type_tag(), "nurbs");

        let mut tags = vec![
            plane_surface().type_tag(),
            FaceSurface::Cylinder(cylinder()).type_tag(),
            FaceSurface::Cone(cone()).type_tag(),
            FaceSurface::Sphere(sphere()).type_tag(),
            FaceSurface::Torus(torus()).type_tag(),
            FaceSurface::Nurbs(nurbs()).type_tag(),
        ];
        tags.sort_unstable();
        tags.dedup();
        assert_eq!(tags.len(), 6, "type tags must be pairwise distinct");
    }

    #[test]
    fn is_planar_and_is_analytic_classify_every_variant() {
        // Only Plane is planar.
        assert!(plane_surface().is_planar());
        assert!(!FaceSurface::Cylinder(cylinder()).is_planar());
        assert!(!FaceSurface::Cone(cone()).is_planar());
        assert!(!FaceSurface::Sphere(sphere()).is_planar());
        assert!(!FaceSurface::Torus(torus()).is_planar());
        assert!(!FaceSurface::Nurbs(nurbs()).is_planar());

        // Analytic means "not NURBS" — the plane counts as analytic.
        assert!(plane_surface().is_analytic());
        assert!(FaceSurface::Cylinder(cylinder()).is_analytic());
        assert!(FaceSurface::Cone(cone()).is_analytic());
        assert!(FaceSurface::Sphere(sphere()).is_analytic());
        assert!(FaceSurface::Torus(torus()).is_analytic());
        assert!(!FaceSurface::Nurbs(nurbs()).is_analytic());
    }

    #[test]
    fn as_analytic_exposes_the_quadrics_and_only_the_quadrics() {
        assert!(matches!(
            FaceSurface::Cylinder(cylinder()).as_analytic(),
            Some(AnalyticSurface::Cylinder(_))
        ));
        assert!(matches!(
            FaceSurface::Cone(cone()).as_analytic(),
            Some(AnalyticSurface::Cone(_))
        ));
        assert!(matches!(
            FaceSurface::Sphere(sphere()).as_analytic(),
            Some(AnalyticSurface::Sphere(_))
        ));
        assert!(matches!(
            FaceSurface::Torus(torus()).as_analytic(),
            Some(AnalyticSurface::Torus(_))
        ));
        assert!(plane_surface().as_analytic().is_none());
        assert!(FaceSurface::Nurbs(nurbs()).as_analytic().is_none());
    }

    // -----------------------------------------------------------------
    // Face boundary accessors
    // -----------------------------------------------------------------

    #[test]
    fn adding_a_hole_leaves_the_outer_boundary_alone() {
        let mut topo = Topology::new();
        let outer = tri_wire(&mut topo, 0.0, 0.0, 4.0);
        let hole = tri_wire(&mut topo, 1.0, 1.0, 1.0);

        let solid_face = topo.add_face(Face::new(outer, vec![], plane_surface()));
        let holed_face = topo.add_face(Face::new(outer, vec![hole], plane_surface()));

        let solid = topo.face(solid_face).unwrap();
        assert_eq!(solid.outer_wire(), outer);
        assert!(solid.inner_wires().is_empty());
        assert!(solid.outer_loop().is_some());
        assert!(
            solid.inner_loops().is_empty(),
            "a face without holes has no inner loops"
        );
        assert_eq!(solid.boundary_loops().len(), 1);

        let holed = topo.face(holed_face).unwrap();
        // The hole did not disturb the outer boundary.
        assert_eq!(holed.outer_wire(), outer);
        assert_eq!(holed.inner_wires(), [hole]);

        // The accessors do not confuse outer with inner.
        let outer_loop = holed.outer_loop().expect("outer loop installed");
        assert_eq!(
            holed.inner_loops().len(),
            1,
            "the hole must surface as exactly one inner loop"
        );
        assert_eq!(holed.boundary_loops().len(), 2);
        assert_eq!(holed.boundary_loops()[0], outer_loop);
        assert_eq!(holed.inner_loops()[0], holed.boundary_loops()[1]);
        assert_ne!(holed.inner_loops()[0], outer_loop);
    }

    #[test]
    fn inner_wires_mut_edits_the_face_in_place() {
        let mut topo = Topology::new();
        let outer = tri_wire(&mut topo, 0.0, 0.0, 4.0);
        let hole = tri_wire(&mut topo, 1.0, 1.0, 1.0);

        let mut face = Face::new(outer, vec![], plane_surface());
        #[allow(deprecated)]
        face.inner_wires_mut().push(hole);

        assert_eq!(
            face.inner_wires(),
            [hole],
            "the pushed hole must be visible through the face itself"
        );
        assert_eq!(face.outer_wire(), outer);
    }

    // -----------------------------------------------------------------
    // Face orientation
    // -----------------------------------------------------------------

    #[test]
    fn reversed_flag_defaults_off_and_round_trips() {
        let mut topo = Topology::new();
        let outer = tri_wire(&mut topo, 0.0, 0.0, 2.0);

        let mut face = Face::new(outer, vec![], plane_surface());
        assert!(!face.is_reversed(), "Face::new starts unreversed");
        assert!(
            Face::new_reversed(outer, vec![], plane_surface()).is_reversed(),
            "Face::new_reversed starts reversed"
        );

        face.set_reversed(true);
        assert!(face.is_reversed(), "set_reversed(true) must take effect");
        face.set_reversed(false);
        assert!(!face.is_reversed(), "set_reversed(false) must take effect");
    }

    #[test]
    fn effective_plane_normal_follows_the_reversed_flag() {
        let mut topo = Topology::new();
        let outer = tri_wire(&mut topo, 0.0, 0.0, 2.0);
        let stored = unit_axis();

        let mut face = Face::new(outer, vec![], plane_surface());
        let forward = face
            .effective_plane_normal()
            .expect("a planar face has an effective normal");
        assert!((forward - stored).length() < TOL);

        face.set_reversed(true);
        let flipped = face
            .effective_plane_normal()
            .expect("a planar face has an effective normal");
        assert!(
            (flipped + stored).length() < TOL,
            "a reversed planar face reports the opposite normal"
        );
        assert!(
            (flipped - forward).length() > 1.0,
            "reversing must actually change the reported normal"
        );

        // Non-planar surfaces have no effective plane normal.
        let curved = Face::new(outer, vec![], FaceSurface::Cylinder(cylinder()));
        assert!(curved.effective_plane_normal().is_none());
    }

    #[test]
    fn compose_orientation_toggles_only_when_flipping() {
        let mut topo = Topology::new();
        let outer = tri_wire(&mut topo, 0.0, 0.0, 2.0);

        let mut face = Face::new(outer, vec![], plane_surface());
        face.compose_orientation(false);
        assert!(!face.is_reversed(), "flip=false must leave the flag alone");

        face.compose_orientation(true);
        assert!(face.is_reversed(), "flip=true must toggle the flag on");

        face.compose_orientation(true);
        assert!(!face.is_reversed(), "flip=true must toggle the flag back");

        face.compose_orientation(true);
        face.compose_orientation(false);
        assert!(face.is_reversed(), "flip=false must still leave it alone");
    }
}
