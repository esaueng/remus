//! Gear teeth on an existing cylindrical face of a solid.
//!
//! A boss (material inside the cylinder) gets external teeth, a bore
//! (material outside) internal teeth, over an axial span of the face. The
//! cut runs in two booleans instead of one cut per tooth space:
//!
//! 1. cut a coaxial tube from `0.1·m` inside the root circle out to the
//!    boss (external), or from the bore out to `0.1·m` past the root circle
//!    (internal);
//! 2. fuse a gear ring whose bore (external) or rim (internal) is exactly
//!    that tube's other wall, so the two meet on one coaxial cylinder.
//!
//! The obvious single cut of every tooth space would put the tool's tip
//! lands on the blank's own cylinder whenever the tip diameter equals it
//! (the keep-diameter fit), and the tool's NURBS flanks would meet that
//! cylinder only along marched, approximate section curves. The two-step
//! order never intersects a flank with a cylinder: the flanks meet only the
//! cap planes, which have an exact ruled × plane arm.
//!
//! The two steps equal the single cut exactly when the band they rebuild is
//! solid material in the input, so that is proven first: the input's
//! intersection with the band's tube must have the tube band's exact
//! volume. A hole, slot or other feature crossing the band refuses with
//! [`GearRefusal::FeaturesInToothRegion`] rather than being filled in.

use std::f64::consts::PI;

use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::builder::make_circle_edge_with_ref;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::wire::{OrientedEdge, Wire};

use super::{
    Frame, GearError, GearKind, GearOutcome, GearParams, GearPlacement, GearRefusal, GearWarning,
    circular, flank_bound, gear_body, gear_geometry, local_outline, twist,
};
use crate::OperationsError;
use crate::boolean::{BooleanOp, boolean, boolean_with_evolution};
use crate::classify::{PointClassification, classify_point};

/// Depth below the root circle (external) or past it (internal), in modules,
/// where the tube cut stops and the gear ring begins. The ring and the kept
/// material meet on that coaxial cylinder exactly: an overlapping ring would
/// put the kept body's circles inside the ring's caps (or the ring's tooth
/// outline inside the kept caps), a face split the exact boolean refuses for
/// a non-circular inner loop, while coincident coaxial cylinders merge.
const SEAM_DEPTH: f64 = 0.1;

/// Relative volume agreement the band precondition demands; the band's
/// faces are planes and coaxial cylinders, whose volume integrates exactly.
const BAND_VOLUME_REL_TOL: f64 = 1e-7;

/// How the teeth are sized against the picked cylinder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GearFit {
    /// Keep the cylinder's diameter as the tip diameter (external) or inner
    /// tip diameter (internal) and solve the module from the tooth count.
    KeepDiameter,
    /// Use the given module; an external gear's tip may be smaller than the
    /// boss (it is turned down), an internal gear's inner tip larger than the
    /// bore (it is opened up).
    UseModule,
}

/// Placement of teeth on a cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CylinderGearOptions {
    /// Sizing mode.
    pub fit: GearFit,
    /// Reference direction for `phase`; `None` uses the direction of the
    /// face's seam (the cylinder's own x axis when the face has none).
    pub x_direction: Option<Vec3>,
    /// Rotation of the first tooth from `x_direction`, radians.
    pub phase: f64,
}

impl Default for CylinderGearOptions {
    fn default() -> Self {
        Self {
            fit: GearFit::KeepDiameter,
            x_direction: None,
            phase: 0.0,
        }
    }
}

/// Cut gear teeth into `solid` along the whole length of its cylindrical
/// `face`.
///
/// `params.kind` must match the face: [`GearKind::External`] for a boss,
/// [`GearKind::Internal`] for a bore. `params.module` is ignored under
/// [`GearFit::KeepDiameter`]; the solved module is reported in the
/// outcome's geometry. The bore and rim come from the existing body, so
/// `params.bore_diameter` and `params.rim_diameter` must be `None`.
///
/// The input solid is left in place; the result is a new solid. Face roles
/// cover the gear faces whose boolean history is one-to-one; faces the
/// fuse merged with the input's (typically the end caps) carry none.
///
/// # Errors
///
/// [`GearError::Refused`] for an out-of-range parameter, a face that is not
/// a full-turn cylinder of `solid`, a cylinder too
/// small (or a bore too large) for the requested module, a face width other
/// than the face's length, a profile that
/// cannot be built, or features crossing the tooth band;
/// [`GearError::Operations`] when a boolean fails. Refusals and failures
/// leave the topology unchanged.
#[allow(clippy::too_many_lines)]
pub fn cut_gear_teeth(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    params: &GearParams,
    options: &CylinderGearOptions,
) -> Result<GearOutcome, GearError> {
    if !solid_faces(topo, solid)?.contains(&face) {
        return Err(GearError::invalid("the face does not belong to the solid"));
    }
    let FaceSurface::Cylinder(cylinder) = topo.face(face)?.surface() else {
        return Err(GearError::refused(
            GearRefusal::NotACylinder,
            "teeth can only be cut on a cylindrical face",
        ));
    };
    let cylinder = cylinder.clone();
    let axis = cylinder.axis();
    let radius = cylinder.radius();
    let (v0, v1) = remus_check::properties::axial_v_range(topo, face, cylinder.origin(), axis)
        .map_err(OperationsError::from)?;
    if params.bore_diameter.is_some() || params.rim_diameter.is_some() {
        return Err(GearError::invalid(
            "teeth on a cylinder take their bore or rim from the existing body",
        ));
    }
    // Teeth span the whole face. A partial span leaves collars of the
    // cylinder beside the teeth, and the coplanar contact between the gear
    // ring's tooth-outline caps and the collars' annular caps is one the
    // exact boolean resolves only for some tooth counts with flush tips, and
    // not at all with turned-down or internal teeth, so partial spans are
    // refused.
    let width = params.face_width;
    let length = v1 - v0;
    let start = v0;
    if !(width.is_finite() && width > 0.0) {
        return Err(GearError::invalid(format!(
            "face width must be positive and finite, got {width}"
        )));
    }
    if (width - length).abs() > 1e-9 * length.max(radius) {
        return Err(GearError::refused(
            GearRefusal::PartialSpan,
            format!(
                "the teeth must span the whole face ({length} long), got a face width of {width}"
            ),
        ));
    }

    // Boss or bore, from which side of the face is material.
    let mid = start + 0.5 * width;
    let probe = |offset: f64| -> Result<bool, GearError> {
        let point = cylinder.origin() + axis * mid + cylinder.x_axis() * (radius + offset);
        Ok(classify_point(topo, solid, point, 0.01, 1e-6)? == PointClassification::Inside)
    };
    let epsilon = 1e-3 * radius;
    let kind = match (probe(-epsilon)?, probe(epsilon)?) {
        (true, false) => GearKind::External,
        (false, true) => GearKind::Internal,
        _ => {
            return Err(GearError::refused(
                GearRefusal::NotABossOrBore,
                "material lies on neither or both sides of the cylinder",
            ));
        }
    };
    if params.kind != kind {
        return Err(GearError::invalid(match kind {
            GearKind::External => "the cylinder is a boss: it takes an external gear",
            _ => "the cylinder is a bore: it takes an internal gear",
        }));
    }

    let z = f64::from(params.teeth);
    let mut resolved = params.clone();
    if options.fit == GearFit::KeepDiameter {
        // Tip diameter per normal module: `z/cos β ± 2(h_a ± x)`.
        let z = z / params.helix_angle.cos();
        let per_module = match kind {
            GearKind::External => z + 2.0 * (params.addendum + params.profile_shift),
            _ => z - 2.0 * (params.addendum - params.profile_shift),
        };
        if !(per_module.is_finite() && per_module > 0.0) {
            return Err(GearError::invalid(
                "the tooth count, addendum and profile shift leave no positive tip diameter",
            ));
        }
        resolved.module = 2.0 * radius / per_module;
    }
    let m = resolved.module;
    if !(m.is_finite() && m > 0.0) {
        return Err(GearError::invalid(format!(
            "module must be positive and finite, got {m}"
        )));
    }
    let c = circular(&resolved);
    let fit_tol = 1e-9 * radius;
    let mut build_params = resolved.clone();
    let band = if kind == GearKind::External {
        if c.tip_radius > radius + fit_tol {
            let geometry = gear_geometry(&resolved)?;
            return Err(GearError::refused(
                GearRefusal::CylinderTooSmall,
                format!(
                    "the tip diameter {} exceeds the cylinder diameter {}",
                    2.0 * c.tip_radius,
                    2.0 * radius
                ),
            )
            .with_geometry(&geometry));
        }
        let seam = (SEAM_DEPTH * m).mul_add(-1.0, c.root_radius);
        if seam <= 0.0 {
            return Err(GearError::invalid(
                "the root circle is too close to the axis to rebuild the teeth",
            ));
        }
        build_params.bore_diameter = Some(2.0 * seam);
        // The tube's outer wall is the boss itself, so no wall stands
        // outside the body.
        (seam, radius)
    } else {
        if c.tip_radius < radius - fit_tol {
            let mut sized = resolved;
            sized.rim_diameter = Some(2.0 * (c.root_radius + SEAM_DEPTH * m));
            let geometry = gear_geometry(&sized)?;
            return Err(GearError::refused(
                GearRefusal::BoreTooLarge,
                format!(
                    "the bore diameter {} exceeds the inner tip diameter {}",
                    2.0 * radius,
                    2.0 * c.tip_radius
                ),
            )
            .with_geometry(&geometry));
        }
        let seam = (SEAM_DEPTH * m).mul_add(1.0, c.root_radius);
        build_params.rim_diameter = Some(2.0 * seam);
        (radius, seam)
    };
    let mut geometry = gear_geometry(&build_params)?;
    // The rim is the existing body's, not the seam ring's.
    geometry
        .warnings
        .retain(|warning| !matches!(warning, GearWarning::ThinRim { .. }));

    let reference = match options.x_direction {
        Some(direction) => direction,
        None => seam_direction(topo, face, cylinder.origin(), axis)?
            .unwrap_or_else(|| cylinder.x_axis()),
    };
    let frame = Frame::new(
        &GearPlacement {
            origin: cylinder.origin() + axis * start,
            axis,
            x_direction: reference,
            phase: options.phase,
        },
        kind,
    )?;

    remus_topology::transaction::run_transacted(topo, |topo| {
        // The rebuilt band must be solid material in the input.
        let expected = PI * (band.1 * band.1 - band.0 * band.0) * width;
        let check = tube(topo, &frame, reference, band.0, band.1, width)?;
        let held = match boolean(topo, BooleanOp::Intersect, solid, check) {
            Ok(common) => crate::measure::solid_volume(topo, common, 1e-3 * radius)?,
            Err(OperationsError::EmptyResult { .. }) => 0.0,
            // Something in the band (a cross hole, a slot) meets the tube in
            // a way the exact boolean cannot resolve; the band is not proven
            // solid, so it is not rebuilt.
            Err(OperationsError::ExactOnlyUnattainable) => {
                return Err(GearError::refused(
                    GearRefusal::FeaturesInToothRegion,
                    "the tooth band could not be proven free of holes, slots or other features",
                ));
            }
            Err(error) => return Err(error.into()),
        };
        if (held - expected).abs() > BAND_VOLUME_REL_TOL * expected {
            return Err(GearError::refused(
                GearRefusal::FeaturesInToothRegion,
                format!(
                    "the tooth band holds {held} of {expected} volume units of material; \
                     a hole, slot or other feature crosses it"
                ),
            ));
        }

        let cut = tube(topo, &frame, reference, band.0, band.1, width)?;
        let kept = boolean(topo, BooleanOp::Cut, solid, cut)?;
        let local = local_outline(&build_params, &geometry, options.phase)?;
        let (profile_bound, spans_per_flank) = flank_bound(&local.outline, &geometry);
        let (ring, ring_roles, twisted) = gear_body(
            topo,
            local,
            &frame,
            reference,
            width,
            twist(&build_params, &geometry).into(),
        )?;
        let flank_deviation_bound = profile_bound + twisted.bound;
        let (result, map) = boolean_with_evolution(topo, BooleanOp::Fuse, kept, ring)?;

        // A ring face keeps its role only when the fuse maps it to exactly
        // one result face that no other input face also produced.
        let mut face_roles = Vec::new();
        if map.origin.is_exact() {
            let result_faces = solid_faces(topo, result)?;
            let sources = |output: usize| {
                map.modified
                    .iter()
                    .chain(&map.generated)
                    .filter(|(_, outputs)| outputs.contains(&output))
                    .map(|(&input, _)| input)
                    .collect::<Vec<_>>()
            };
            for (ring_face, role) in ring_roles {
                let Some(outputs) = map.modified.get(&ring_face.index()) else {
                    continue;
                };
                if let [output] = outputs.as_slice()
                    && sources(*output) == [ring_face.index()]
                    && let Some(&face) = result_faces.iter().find(|f| f.index() == *output)
                {
                    face_roles.push((face, role));
                }
            }
        }
        Ok(GearOutcome {
            solid: result,
            geometry: geometry.clone(),
            flank_deviation_bound,
            spans_per_flank,
            spans_along_axis: twisted.spans,
            face_roles,
        })
    })
    .map_err(|error| error.with_geometry(&geometry))
}

/// Radial direction of the picked face's seam: the start of its first
/// closed circle or axis-parallel line edge. It is only the default
/// reference for `phase`; the circles this operation adds start wherever
/// the reference points, since coincident coaxial walls merge whatever
/// their seams.
fn seam_direction(
    topo: &Topology,
    face: FaceId,
    origin: remus_math::vec::Point3,
    axis: Vec3,
) -> Result<Option<Vec3>, GearError> {
    let data = topo.face(face)?;
    for wire in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            let seam_like = match edge.curve() {
                EdgeCurve::Circle(_) => edge.start() == edge.end(),
                EdgeCurve::Line => true,
                EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => false,
            };
            if !seam_like {
                continue;
            }
            let offset = topo.vertex(edge.start())?.point() - origin;
            if let Ok(direction) = (offset - axis * offset.dot(axis)).normalize() {
                return Ok(Some(direction));
            }
        }
    }
    Ok(None)
}

/// A coaxial tube (or a solid cylinder when `inner` is not positive) of
/// `width` along the frame axis from the frame origin.
fn tube(
    topo: &mut Topology,
    frame: &Frame,
    reference: Vec3,
    inner: f64,
    outer: f64,
    width: f64,
) -> Result<SolidId, GearError> {
    let tol = remus_math::tolerance::Tolerance::new().linear;
    let rim = make_circle_edge_with_ref(topo, frame.origin, frame.axis, outer, reference, tol)?;
    let outer_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true)?);
    let mut holes = Vec::new();
    if inner > 0.0 {
        let hole =
            make_circle_edge_with_ref(topo, frame.origin, -frame.axis, inner, reference, tol)?;
        holes.push(topo.add_wire(Wire::new(vec![OrientedEdge::new(hole, true)], true)?));
    }
    let d = frame.axis.dot(Vec3::new(
        frame.origin.x(),
        frame.origin.y(),
        frame.origin.z(),
    ));
    let profile = topo.add_face(Face::new(
        outer_wire,
        holes,
        FaceSurface::Plane {
            normal: frame.axis,
            d,
        },
    ));
    Ok(crate::extrude::extrude(topo, profile, frame.axis, width)?)
}

#[cfg(test)]
mod tests;
