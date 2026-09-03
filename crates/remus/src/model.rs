use remus_heal::fix::{FixConfig, FixResult};
use remus_io::ImportLimits;
use remus_io::step::{StepReadResult, StepWriteOptions};
use remus_math::aabb::Aabb3;
use remus_math::context::OperationContext;
use remus_math::nurbs::NurbsCurve;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::BlendResult;
use remus_operations::boolean::{BooleanOp, BooleanOutcome};
use remus_operations::journal_ops::{JournaledBlend, JournaledBoolean};
use remus_operations::tessellate::TriangleMesh;
use remus_operations::validate::{ValidationOptions, ValidationReport};
use remus_topology::journal::Journal;
use remus_topology::naming::{PersistentRef, Resolution};
use remus_topology::{EdgeId, FaceId, SolidId, Topology};

use crate::{GProps, HealError, IoError, OperationsError};

/// An owned native modeling session.
///
/// `Model` keeps the B-Rep topology and caller-visible operation policy
/// together. Modeling methods take and return typed arena handles; query and
/// export methods read the same topology, and persistent-reference resolution
/// reads its evolution journal.
#[derive(Debug, Clone)]
pub struct Model {
    topology: Topology,
    context: OperationContext,
}

impl Model {
    /// Creates an empty model with the default operation policy.
    #[must_use]
    pub fn new() -> Self {
        Self::with_context(OperationContext::new())
    }

    /// Creates an empty model with explicit tolerance, work, and fallback policy.
    #[must_use]
    pub fn with_context(context: OperationContext) -> Self {
        Self {
            topology: Topology::new(),
            context,
        }
    }

    /// Wraps an existing topology with the default operation policy.
    #[must_use]
    pub fn from_topology(topology: Topology) -> Self {
        Self {
            topology,
            context: OperationContext::new(),
        }
    }

    /// Wraps an existing topology with explicit operation policy.
    #[must_use]
    pub const fn from_parts(topology: Topology, context: OperationContext) -> Self {
        Self { topology, context }
    }

    /// Returns the session's topology for read-only interrogation.
    #[must_use]
    pub const fn topology(&self) -> &Topology {
        &self.topology
    }

    /// Returns the session's topology for lower-level construction.
    ///
    /// Direct mutation is intentionally explicit. The topology journal detects
    /// unjournaled mutations and inserts a fail-closed barrier before the next
    /// journaled operation.
    pub const fn topology_mut(&mut self) -> &mut Topology {
        &mut self.topology
    }

    /// Returns the session's operation policy.
    #[must_use]
    pub const fn context(&self) -> &OperationContext {
        &self.context
    }

    /// Returns the operation policy for in-place caller configuration.
    pub const fn context_mut(&mut self) -> &mut OperationContext {
        &mut self.context
    }

    /// Replaces the operation policy and returns the session for chaining.
    #[must_use]
    pub fn with_operation_context(mut self, context: OperationContext) -> Self {
        self.context = context;
        self
    }

    /// Returns the append-only evolution journal.
    #[must_use]
    pub fn journal(&self) -> &Journal {
        self.topology.journal()
    }

    /// Splits the session back into its owned topology and operation policy.
    #[must_use]
    pub fn into_parts(self) -> (Topology, OperationContext) {
        (self.topology, self.context)
    }

    /// Creates a box anchored at the origin.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid dimensions.
    pub fn make_box(&mut self, dx: f64, dy: f64, dz: f64) -> Result<SolidId, OperationsError> {
        remus_operations::primitives::make_box(&mut self.topology, dx, dy, dz)
    }

    /// Creates a cylinder along +Z with its base at the origin.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for an invalid radius or height.
    pub fn make_cylinder(&mut self, radius: f64, height: f64) -> Result<SolidId, OperationsError> {
        remus_operations::primitives::make_cylinder(&mut self.topology, radius, height)
    }

    /// Creates a cone or frustum along +Z.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid radii or height.
    pub fn make_cone(
        &mut self,
        bottom_radius: f64,
        top_radius: f64,
        height: f64,
    ) -> Result<SolidId, OperationsError> {
        remus_operations::primitives::make_cone(
            &mut self.topology,
            bottom_radius,
            top_radius,
            height,
        )
    }

    /// Creates a sphere centered at the origin.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for an invalid radius or segment count.
    pub fn make_sphere(
        &mut self,
        radius: f64,
        segments: usize,
    ) -> Result<SolidId, OperationsError> {
        remus_operations::primitives::make_sphere(&mut self.topology, radius, segments)
    }

    /// Creates a torus centered at the origin in the XY plane.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid radii or segment count.
    pub fn make_torus(
        &mut self,
        major_radius: f64,
        minor_radius: f64,
        segments: usize,
    ) -> Result<SolidId, OperationsError> {
        remus_operations::primitives::make_torus(
            &mut self.topology,
            major_radius,
            minor_radius,
            segments,
        )
    }

    /// Runs a boolean under this model's fallback policy and returns its quality.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`], including the typed exact-only refusal when
    /// the selected policy forbids a required approximation.
    pub fn boolean(
        &mut self,
        operation: BooleanOp,
        a: SolidId,
        b: SolidId,
    ) -> Result<BooleanOutcome, OperationsError> {
        remus_operations::boolean::boolean_with_context(
            &mut self.topology,
            operation,
            a,
            b,
            &self.context,
        )
    }

    /// Fuses two solids with disclosed result quality.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the boolean fails or policy refuses it.
    pub fn fuse(&mut self, a: SolidId, b: SolidId) -> Result<BooleanOutcome, OperationsError> {
        self.boolean(BooleanOp::Fuse, a, b)
    }

    /// Subtracts `b` from `a` with disclosed result quality.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the boolean fails or policy refuses it.
    pub fn cut(&mut self, a: SolidId, b: SolidId) -> Result<BooleanOutcome, OperationsError> {
        self.boolean(BooleanOp::Cut, a, b)
    }

    /// Intersects two solids with disclosed result quality.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the boolean fails or policy refuses it.
    pub fn intersect(&mut self, a: SolidId, b: SolidId) -> Result<BooleanOutcome, OperationsError> {
        self.boolean(BooleanOp::Intersect, a, b)
    }

    /// Runs an exact boolean and records construction-derived evolution.
    ///
    /// This is the persistent-naming path. Unlike [`Self::boolean`], it never
    /// uses an approximate fallback because a mesh result has no exact
    /// construction history to record.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the exact boolean or its evolution
    /// record cannot be completed transactionally.
    pub fn boolean_journaled(
        &mut self,
        operation: BooleanOp,
        a: SolidId,
        b: SolidId,
    ) -> Result<JournaledBoolean, OperationsError> {
        remus_operations::journal_ops::boolean_journaled_with_operation(
            &mut self.topology,
            operation,
            a,
            b,
        )
    }

    /// Fillets selected edges with the v2 validated blend engine.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid or unsupported blends.
    pub fn fillet(
        &mut self,
        solid: SolidId,
        edges: &[EdgeId],
        radius: f64,
    ) -> Result<BlendResult, OperationsError> {
        remus_operations::blend_ops::fillet_v2(&mut self.topology, solid, edges, radius)
    }

    /// Chamfers selected edges with the v2 validated blend engine.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid or unsupported blends.
    pub fn chamfer(
        &mut self,
        solid: SolidId,
        edges: &[EdgeId],
        first_distance: f64,
        second_distance: f64,
    ) -> Result<BlendResult, OperationsError> {
        remus_operations::blend_ops::chamfer_v2(
            &mut self.topology,
            solid,
            edges,
            first_distance,
            second_distance,
        )
    }

    /// Fillets selected edges and records face evolution in one transaction.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the blend or journal record fails. A
    /// refusal leaves both topology and journal unchanged.
    pub fn fillet_journaled(
        &mut self,
        solid: SolidId,
        edges: &[EdgeId],
        radius: f64,
    ) -> Result<JournaledBlend, OperationsError> {
        remus_operations::journal_ops::fillet_journaled(&mut self.topology, solid, edges, radius)
    }

    /// Chamfers selected edges and records face evolution in one transaction.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the blend or journal record fails. A
    /// refusal leaves both topology and journal unchanged.
    pub fn chamfer_journaled(
        &mut self,
        solid: SolidId,
        edges: &[EdgeId],
        first_distance: f64,
        second_distance: f64,
    ) -> Result<JournaledBlend, OperationsError> {
        remus_operations::journal_ops::chamfer_journaled(
            &mut self.topology,
            solid,
            edges,
            first_distance,
            second_distance,
        )
    }

    /// Extrudes a profile face.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid profile or sweep geometry.
    pub fn extrude(
        &mut self,
        face: FaceId,
        direction: Vec3,
        distance: f64,
    ) -> Result<SolidId, OperationsError> {
        remus_operations::extrude::extrude(&mut self.topology, face, direction, distance)
    }

    /// Revolves a profile face around an axis.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid profile or revolution geometry.
    pub fn revolve(
        &mut self,
        face: FaceId,
        axis_origin: Point3,
        axis_direction: Vec3,
        angle_radians: f64,
    ) -> Result<SolidId, OperationsError> {
        remus_operations::revolve::revolve(
            &mut self.topology,
            face,
            axis_origin,
            axis_direction,
            angle_radians,
        )
    }

    /// Sweeps a profile face along a NURBS path.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid profile or path geometry.
    pub fn sweep(
        &mut self,
        profile: FaceId,
        path: &NurbsCurve,
    ) -> Result<SolidId, OperationsError> {
        remus_operations::sweep::sweep(&mut self.topology, profile, path)
    }

    /// Lofts through two or more profile faces.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid or incompatible profiles.
    pub fn loft(&mut self, profiles: &[FaceId]) -> Result<SolidId, OperationsError> {
        remus_operations::loft::loft(&mut self.topology, profiles)
    }

    /// Sweeps a profile along a path with an optional guide curve.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] for invalid profile, path, or guide geometry.
    pub fn pipe(
        &mut self,
        profile: FaceId,
        path: &NurbsCurve,
        guide: Option<&NurbsCurve>,
    ) -> Result<SolidId, OperationsError> {
        remus_operations::pipe::pipe(&mut self.topology, profile, path, guide)
    }

    /// Computes a solid's volume at the requested fallback deflection.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the solid cannot be measured.
    pub fn volume(&self, solid: SolidId, deflection: f64) -> Result<f64, OperationsError> {
        remus_operations::measure::solid_volume(&self.topology, solid, deflection)
    }

    /// Computes a solid's total surface area.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the solid cannot be measured.
    pub fn surface_area(&self, solid: SolidId, deflection: f64) -> Result<f64, OperationsError> {
        remus_operations::measure::solid_surface_area(&self.topology, solid, deflection)
    }

    /// Computes a solid's center of mass for unit density.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the solid cannot be measured.
    pub fn center_of_mass(
        &self,
        solid: SolidId,
        deflection: f64,
    ) -> Result<Point3, OperationsError> {
        remus_operations::measure::solid_center_of_mass(&self.topology, solid, deflection)
    }

    /// Computes exact-integrated mass properties for unit density.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the solid cannot be integrated.
    pub fn mass_properties(&self, solid: SolidId) -> Result<GProps, OperationsError> {
        remus_operations::measure::mass_properties(&self.topology, solid)
    }

    /// Computes a solid's axis-aligned bounding box.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if the solid cannot be measured.
    pub fn bounding_box(&self, solid: SolidId) -> Result<Aabb3, OperationsError> {
        remus_operations::measure::solid_bounding_box(&self.topology, solid)
    }

    /// Tessellates a solid into a shared-boundary triangle mesh.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if any face cannot be tessellated.
    pub fn tessellate(
        &self,
        solid: SolidId,
        deflection: f64,
    ) -> Result<TriangleMesh, OperationsError> {
        remus_operations::tessellate::tessellate_solid(&self.topology, solid, deflection)
    }

    /// Tessellates a solid with explicit linear and angular tolerances.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if any face cannot be tessellated.
    pub fn tessellate_with_tolerance(
        &self,
        solid: SolidId,
        deflection: f64,
        angular_tolerance: f64,
    ) -> Result<TriangleMesh, OperationsError> {
        remus_operations::tessellate::tessellate_solid_with_tolerance(
            &self.topology,
            solid,
            deflection,
            angular_tolerance,
        )
    }

    /// Validates a solid with the default strict options.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if validation cannot inspect the topology.
    pub fn validate(&self, solid: SolidId) -> Result<ValidationReport, OperationsError> {
        remus_operations::validate::validate_solid(&self.topology, solid)
    }

    /// Validates a solid with caller-supplied options.
    ///
    /// # Errors
    ///
    /// Returns [`OperationsError`] if validation cannot inspect the topology.
    pub fn validate_with_options(
        &self,
        solid: SolidId,
        options: &ValidationOptions,
    ) -> Result<ValidationReport, OperationsError> {
        remus_operations::validate::validate_solid_with_options(&self.topology, solid, options)
    }

    /// Imports every solid from STEP into this model.
    ///
    /// # Errors
    ///
    /// Returns [`IoError`] for malformed, unsupported, or over-budget input.
    pub fn read_step(&mut self, input: &str) -> Result<Vec<SolidId>, IoError> {
        remus_io::step::read_step(input, &mut self.topology)
    }

    /// Imports STEP and returns bounded healing diagnostics.
    ///
    /// # Errors
    ///
    /// Returns [`IoError`] for malformed, unsupported, or over-budget input.
    pub fn read_step_with_report(&mut self, input: &str) -> Result<StepReadResult, IoError> {
        remus_io::step::read_step_with_report(input, &mut self.topology)
    }

    /// Imports STEP under explicit hostile-input resource limits.
    ///
    /// # Errors
    ///
    /// Returns [`IoError`] for malformed, unsupported, or over-budget input.
    pub fn read_step_with_limits(
        &mut self,
        input: &str,
        limits: ImportLimits,
    ) -> Result<Vec<SolidId>, IoError> {
        remus_io::step::read_step_with_limits(input, &mut self.topology, limits)
    }

    /// Imports STEP with explicit limits and bounded healing diagnostics.
    ///
    /// # Errors
    ///
    /// Returns [`IoError`] for malformed, unsupported, or over-budget input.
    pub fn read_step_with_limits_and_report(
        &mut self,
        input: &str,
        limits: ImportLimits,
    ) -> Result<StepReadResult, IoError> {
        remus_io::step::read_step_with_limits_and_report(input, &mut self.topology, limits)
    }

    /// Exports one or more solids as STEP AP203.
    ///
    /// # Errors
    ///
    /// Returns [`IoError`] for missing or unsupported topology.
    pub fn write_step(&self, solids: &[SolidId]) -> Result<String, IoError> {
        remus_io::step::write_step(&self.topology, solids)
    }

    /// Exports one or more solids as STEP with explicit metadata.
    ///
    /// # Errors
    ///
    /// Returns [`IoError`] for missing or unsupported topology.
    pub fn write_step_with_options(
        &self,
        solids: &[SolidId],
        options: &StepWriteOptions,
    ) -> Result<String, IoError> {
        remus_io::step::write_step_with_options(&self.topology, solids, options)
    }

    /// Runs the configurable healing pipeline on a solid.
    ///
    /// # Errors
    ///
    /// Returns [`HealError`] when a requested repair cannot be applied.
    pub fn heal(
        &mut self,
        solid: SolidId,
        config: &FixConfig,
    ) -> Result<(SolidId, FixResult), HealError> {
        remus_heal::fix::fix_shape(&mut self.topology, solid, config)
    }

    /// Resolves a persistent reference against the model's journal and topology.
    #[must_use]
    pub fn resolve(&self, reference: &PersistentRef) -> Resolution {
        remus_topology::naming::resolve(&self.topology, reference)
    }
}

impl Default for Model {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::unwrap_used)]

    use remus_math::context::FallbackPolicy;

    use super::*;

    #[test]
    fn model_owns_context_topology_and_journal() {
        let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
        let model = Model::with_context(context);

        assert_eq!(model.context().fallback, FallbackPolicy::ExactOnly);
        assert_eq!(model.topology().num_solids(), 0);
        assert!(model.journal().is_empty());
    }

    #[test]
    fn native_front_door_builds_validates_measures_meshes_and_exports() {
        let mut model = Model::new();
        let solid = model.make_box(2.0, 3.0, 4.0).unwrap();

        assert!((model.volume(solid, 0.1).unwrap() - 24.0).abs() < 1.0e-12);
        assert!(model.validate(solid).unwrap().is_valid());
        let mesh = model.tessellate(solid, 0.1).unwrap();
        assert!(remus_operations::tessellate::is_watertight(&mesh));
        assert_eq!(
            remus_operations::tessellate::non_manifold_edge_count(&mesh),
            0
        );
        assert!(
            model
                .write_step(&[solid])
                .unwrap()
                .starts_with("ISO-10303-21;")
        );
    }

    #[test]
    fn modeling_refusal_stays_typed_and_transactional() {
        let mut model = Model::new();
        let before = model.topology().num_solids();
        let error = model.make_box(0.0, 2.0, 3.0).unwrap_err();

        assert!(matches!(error, OperationsError::InvalidInput { .. }));
        assert_eq!(model.topology().num_solids(), before);
    }

    #[test]
    fn import_refusal_stays_typed_and_transactional() {
        let mut model = Model::new();
        let existing = model.make_box(1.0, 1.0, 1.0).unwrap();
        let before = model.topology().num_solids();
        let error = model.read_step("not a STEP file").unwrap_err();

        assert!(matches!(error, IoError::ParseError { .. }));
        assert_eq!(model.topology().num_solids(), before);
        assert!(model.topology().solid(existing).is_ok());
    }

    #[test]
    fn journaled_boolean_resolves_a_persistent_output_reference() {
        let mut model = Model::new();
        let block = model.make_box(30.0, 20.0, 10.0).unwrap();
        let cutter = model.make_cylinder(5.0, 15.0).unwrap();
        let result = model
            .boolean_journaled(BooleanOp::Cut, block, cutter)
            .unwrap();

        assert_eq!(model.journal().len(), 1);
        let reference = PersistentRef::operation_output(result.op, crate::EntityKind::Face, 0);
        assert!(matches!(
            model.resolve(&reference),
            Resolution::Bound { .. }
        ));
    }

    #[test]
    fn journaled_blend_refusal_rolls_back_topology_and_journal() {
        let mut model = Model::new();
        let solid = model.make_box(2.0, 3.0, 4.0).unwrap();
        let edges = remus_topology::explorer::solid_edges(model.topology(), solid).unwrap();
        let before_slots = model.topology().allocated_slot_count();
        let error = model
            .fillet_journaled(solid, &edges[..1], 0.0)
            .err()
            .unwrap();

        assert!(matches!(error, OperationsError::InvalidInput { .. }));
        assert_eq!(model.topology().allocated_slot_count(), before_slots);
        assert!(model.journal().is_empty());
        assert!(model.topology().solid(solid).is_ok());
    }
}
