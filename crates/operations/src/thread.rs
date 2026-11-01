//! Screw threads on an existing cylindrical face of a solid.
//!
//! A boss (material inside the cylinder) gets an external thread, a bore
//! (material outside) an internal one, over an axial span of the face. The
//! thread form is a trapezoid in the axial section (crest flat, two flanks of
//! independent angle, root flat) with any number of starts and either hand,
//! which covers ISO metric and unified forms as well as forms shaped for
//! additive manufacturing: steep flanks that print without support, wide
//! crests and roots that leave no knife edges.
//!
//! The thread is the screw sweep of its transverse outline (see
//! [`profile`]), built as twisted extrusions: a thread ring. The ring is
//! put on the body the way gear teeth are ([`crate::gear::cut_gear_teeth`]),
//! in two booleans that never intersect a swept flank with a cylinder:
//!
//! 1. cut a coaxial tube from just inside the root out to the boss
//!    (external), or from the bore out to just past the root (internal);
//! 2. fuse the ring, whose bore (external) or rim (internal) is exactly that
//!    tube's other wall, so the two meet on one coaxial cylinder and the cap
//!    planes.
//!
//! The two steps equal cutting the thread groove only when the band they
//! rebuild is solid material in the input, so that is proven first, by the
//! exact volume of the input's intersection with the band's tube. A hole or
//! slot crossing the band refuses with
//! [`ThreadRefusal::FeaturesInThreadRegion`] rather than being filled in.
//!
//! An end of the thread that stops short of the face's end meets a collar of
//! the original cylinder; an end at the face's end runs out on its end plane.
//!
//! The ring is a stack of segments of at most half a turn, glued at shared
//! planes like the halves of a herringbone gear: a swept face that winds
//! several turns about the axis defeats point classification (rays seed on
//! the wrong turn), which drops faces from the fuse. Under half a turn also
//! keeps crest and root lands exact cylinders.
//!
//! Not covered: chamfered or tapered thread ends. The exact boolean cannot
//! yet cut a cone through the swept flanks (it returns an open shell), so a
//! chamfer needs the flank–cone curves (conical spirals, closed form) built
//! into the ring directly.

pub(crate) mod profile;

use std::f64::consts::{PI, TAU};

use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::builder::make_circle_edge_with_ref;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::explorer::solid_faces;
use remus_topology::face::{Face, FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire, WireId};

use crate::OperationsError;
use crate::boolean::{BooleanOp, boolean};
use crate::classify::{PointClassification, classify_point};
use profile::Segment;

/// Default thread tolerance as a fraction of the pitch.
pub const DEFAULT_TOLERANCE_PER_PITCH: f64 = 1e-4;

/// Largest supported number of starts.
pub const MAX_THREAD_STARTS: u32 = 16;

/// Largest supported flank angle from the plane across the axis, degrees.
pub const MAX_FLANK_ANGLE_DEG: f64 = 75.0;

/// Uniform Hermite spans allowed along the axis of one ring segment.
const MAX_THREAD_SPANS: usize = 4096;

/// Largest turn of one ring segment, in turns. A swept face winding several
/// turns about the axis defeats point classification and trim projection
/// (both seed on the wrong turn), so the ring is a stack of short segments;
/// under half a turn also keeps crest and root lands exact cylinders.
const MAX_SEGMENT_TURN: f64 = 0.5;

/// Most ring segments.
const MAX_SEGMENTS: usize = 2000;

/// Depth past the root, as a fraction of the thread depth, where the tube
/// cut stops and the thread ring begins. The ring and the kept material meet
/// on that coaxial cylinder exactly (see the module docs).
const SEAM_DEPTH: f64 = 0.1;

/// Relative volume agreement the band precondition demands; the band's
/// faces are planes and coaxial cylinders, whose volume integrates exactly.
const BAND_VOLUME_REL_TOL: f64 = 1e-7;

/// Which side of the cylinder carries the thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadKind {
    /// A boss: the thread's material lies inside its surface.
    External,
    /// A bore: the thread's material lies outside its surface.
    Internal,
}

/// Thread hand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadHand {
    /// Advances along the axis when turned counter-clockwise about it.
    Right,
    /// Advances along the axis when turned clockwise about it.
    Left,
}

/// A trapezoidal thread form in the axial section. Lengths are model units,
/// angles radians.
///
/// Along the axis one pitch holds the crest flat (`crest_width`), the upper
/// flank (facing the thread's end, `depth·tan(upper_flank_angle)` long), the
/// root flat and the lower flank (facing its start). The root flat takes
/// what is left of the pitch, so it must not be negative. Crest and root
/// refer to the thread's material: an external thread's crest is its major
/// diameter, an internal thread's its minor diameter.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreadForm {
    /// Axial distance between adjacent threads.
    pub pitch: f64,
    /// Radial distance from crest to root.
    pub depth: f64,
    /// Axial width of the crest flat; zero for a sharp crest.
    pub crest_width: f64,
    /// Angle of the flank facing the end of the thread from the plane
    /// across the axis; zero for a square flank. ISO metric: 30°.
    pub upper_flank_angle: f64,
    /// Angle of the flank facing the start of the thread.
    pub lower_flank_angle: f64,
    /// Number of starts; the lead is `starts · pitch`.
    pub starts: u32,
    /// Hand.
    pub hand: ThreadHand,
}

impl ThreadForm {
    /// The ISO 68-1 basic profile for `pitch`: 60° included angle, depth
    /// `5H/8`, crest flat `P/8`, root flat `P/4` (for an external thread;
    /// an internal thread swaps them), one right-hand start.
    #[must_use]
    pub fn iso_metric(pitch: f64, kind: ThreadKind) -> Self {
        let h = 3.0_f64.sqrt() / 2.0 * pitch;
        let (crest, _root) = match kind {
            ThreadKind::External => (pitch / 8.0, pitch / 4.0),
            ThreadKind::Internal => (pitch / 4.0, pitch / 8.0),
        };
        Self {
            pitch,
            depth: 5.0 / 8.0 * h,
            crest_width: crest,
            upper_flank_angle: PI / 6.0,
            lower_flank_angle: PI / 6.0,
            starts: 1,
            hand: ThreadHand::Right,
        }
    }

    /// The thread's lead: axial advance per turn.
    #[must_use]
    pub fn lead(&self) -> f64 {
        f64::from(self.starts) * self.pitch
    }

    /// Axial width of the root flat: the pitch less the crest and flanks.
    #[must_use]
    pub fn root_width(&self) -> f64 {
        self.depth.mul_add(
            -(self.upper_flank_angle.tan() + self.lower_flank_angle.tan()),
            self.pitch - self.crest_width,
        )
    }
}

/// Placement and sizing of a thread on a cylinder.
#[derive(Debug, Clone, PartialEq)]
pub struct CylinderThreadOptions {
    /// Diameter of the thread's outer surface: an external thread's crest
    /// (major) diameter, an internal thread's root (major) diameter. `None`
    /// fits the thread to the cylinder: an external crest on the boss, an
    /// internal crest on the bore.
    pub major_diameter: Option<f64>,
    /// Axial distance from the start of the face (its lower end along the
    /// cylinder's axis) to the start of the thread.
    pub offset: f64,
    /// Axial length of the thread; `None` runs to the end of the face.
    pub length: Option<f64>,
    /// Reference direction for `phase`; `None` uses the direction of the
    /// face's seam (the cylinder's own x axis when the face has none).
    pub x_direction: Option<Vec3>,
    /// Rotation of the thread's start about the axis from `x_direction`,
    /// radians.
    pub phase: f64,
    /// Bound on the swept surfaces' distance from the exact thread; `None`
    /// uses [`DEFAULT_TOLERANCE_PER_PITCH`] of the pitch.
    pub tolerance: Option<f64>,
}

impl Default for CylinderThreadOptions {
    fn default() -> Self {
        Self {
            major_diameter: None,
            offset: 0.0,
            length: None,
            x_direction: None,
            phase: 0.0,
            tolerance: None,
        }
    }
}

/// Dimensions of a thread.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreadGeometry {
    /// Boss or bore.
    pub kind: ThreadKind,
    /// Diameter of the thread's outer surface.
    pub major_diameter: f64,
    /// Diameter of the thread's inner surface.
    pub minor_diameter: f64,
    /// Axial pitch.
    pub pitch: f64,
    /// Axial advance per turn.
    pub lead: f64,
    /// Axial width of the crest flat.
    pub crest_width: f64,
    /// Axial width of the root flat.
    pub root_width: f64,
    /// Lead angle at the mean diameter, radians.
    pub lead_angle: f64,
    /// Axial length of the thread.
    pub length: f64,
    /// Turns of each start along the length.
    pub turns: f64,
}

/// A built thread.
#[derive(Debug, Clone, PartialEq)]
pub struct ThreadOutcome {
    /// The threaded solid.
    pub solid: SolidId,
    /// The thread's dimensions.
    pub geometry: ThreadGeometry,
    /// Proven bound on the thread surfaces' distance from the exact thread
    /// (outline and screw fits together), with a floating-point allowance.
    pub deviation_bound: f64,
    /// Hermite spans along the axis, over all ring segments.
    pub spans_along_axis: usize,
    /// Most Hermite spans across the profile in one flank.
    pub spans_per_flank: usize,
    /// The ring was glued on directly rather than fused.
    pub(crate) glued: bool,
}

/// Why a thread was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadRefusal {
    /// A parameter is out of range.
    InvalidParameter,
    /// The crest and flanks take more than the pitch.
    RootClosed,
    /// The picked face is not a cylinder.
    NotACylinder,
    /// The picked cylinder has material on neither or both sides.
    NotABossOrBore,
    /// The thread would reach past an end of the picked face.
    SpanOutsideFace,
    /// An external thread's crest lies outside the boss.
    CylinderTooSmall,
    /// An internal thread's crest lies inside the bore.
    BoreTooLarge,
    /// The thread's root reaches the axis.
    TooDeep,
    /// The tolerance needs more spans than allowed.
    ToleranceTooTight,
    /// A hole, slot or other feature crosses the band the thread rebuilds.
    FeaturesInThreadRegion,
}

impl ThreadRefusal {
    /// Stable machine-readable code.
    #[must_use]
    pub const fn code(self) -> &'static str {
        match self {
            Self::InvalidParameter => "invalid-parameter",
            Self::RootClosed => "root-closed",
            Self::NotACylinder => "not-a-cylinder",
            Self::NotABossOrBore => "not-a-boss-or-bore",
            Self::SpanOutsideFace => "span-outside-face",
            Self::CylinderTooSmall => "cylinder-too-small",
            Self::BoreTooLarge => "bore-too-large",
            Self::TooDeep => "too-deep",
            Self::ToleranceTooTight => "tolerance-too-tight",
            Self::FeaturesInThreadRegion => "features-in-thread-region",
        }
    }
}

/// Error from [`cut_thread`].
#[derive(Debug, thiserror::Error)]
pub enum ThreadError {
    /// The parameters describe no buildable thread.
    #[error("thread:{}: {reason}", code.code())]
    Refused {
        /// Refusal kind.
        code: ThreadRefusal,
        /// Human-readable reason.
        reason: String,
    },
    /// The underlying construction failed.
    #[error(transparent)]
    Operations(#[from] OperationsError),
}

impl ThreadError {
    pub(crate) fn refused(code: ThreadRefusal, reason: impl Into<String>) -> Self {
        Self::Refused {
            code,
            reason: reason.into(),
        }
    }

    fn invalid(reason: impl Into<String>) -> Self {
        Self::refused(ThreadRefusal::InvalidParameter, reason)
    }
}

impl From<remus_math::MathError> for ThreadError {
    fn from(error: remus_math::MathError) -> Self {
        Self::Operations(error.into())
    }
}

impl From<remus_topology::TopologyError> for ThreadError {
    fn from(error: remus_topology::TopologyError) -> Self {
        Self::Operations(error.into())
    }
}

fn require(condition: bool, reason: impl FnOnce() -> String) -> Result<(), ThreadError> {
    if condition {
        Ok(())
    } else {
        Err(ThreadError::invalid(reason()))
    }
}

fn positive(value: f64, name: &str) -> Result<(), ThreadError> {
    require(value.is_finite() && value > 0.0, || {
        format!("{name} must be positive and finite, got {value}")
    })
}

fn validate_form(form: &ThreadForm) -> Result<(), ThreadError> {
    positive(form.pitch, "pitch")?;
    positive(form.depth, "depth")?;
    require(
        form.crest_width.is_finite() && form.crest_width >= 0.0,
        || {
            format!(
                "crest width must be non-negative and finite, got {}",
                form.crest_width
            )
        },
    )?;
    let max_angle = MAX_FLANK_ANGLE_DEG.to_radians();
    for (angle, name) in [
        (form.upper_flank_angle, "upper flank angle"),
        (form.lower_flank_angle, "lower flank angle"),
    ] {
        require(
            angle.is_finite() && (0.0..=max_angle).contains(&angle),
            || {
                format!(
                    "{name} must be in [0°, {MAX_FLANK_ANGLE_DEG}°], got {}°",
                    angle.to_degrees()
                )
            },
        )?;
    }
    require((1..=MAX_THREAD_STARTS).contains(&form.starts), || {
        format!(
            "starts must be in [1, {MAX_THREAD_STARTS}], got {}",
            form.starts
        )
    })?;
    let root = form.root_width();
    if root < -1e-12 * form.pitch {
        return Err(ThreadError::refused(
            ThreadRefusal::RootClosed,
            format!(
                "the crest and flanks take {} of the {} pitch, leaving no root",
                form.pitch - root,
                form.pitch
            ),
        ));
    }
    Ok(())
}

/// Cut a thread into `solid` along its cylindrical `face`.
///
/// The kind comes from the face: a boss takes an external thread, a bore an
/// internal one. The input solid is left in place; the result is a new
/// solid.
///
/// # Errors
///
/// [`ThreadError::Refused`] for an out-of-range parameter, a face that is
/// not a full-turn cylinder of `solid`, a thread larger than the boss (or
/// smaller than the bore), a span reaching past either end of the face, or
/// features crossing the thread band; [`ThreadError::Operations`] when a
/// boolean fails. Refusals and failures leave the topology unchanged.
#[allow(clippy::too_many_lines)]
pub fn cut_thread(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    form: &ThreadForm,
    options: &CylinderThreadOptions,
) -> Result<ThreadOutcome, ThreadError> {
    validate_form(form)?;
    require(options.phase.is_finite(), || {
        format!("phase must be finite, got {}", options.phase)
    })?;
    if let Some(tolerance) = options.tolerance {
        positive(tolerance, "tolerance")?;
    }
    if !solid_faces(topo, solid)?.contains(&face) {
        return Err(ThreadError::invalid(
            "the face does not belong to the solid",
        ));
    }
    let FaceSurface::Cylinder(cylinder) = topo.face(face)?.surface() else {
        return Err(ThreadError::refused(
            ThreadRefusal::NotACylinder,
            "a thread can only be cut on a cylindrical face",
        ));
    };
    let cylinder = cylinder.clone();
    let axis = cylinder.axis();
    let radius = cylinder.radius();
    let (v0, v1) = remus_check::properties::axial_v_range(topo, face, cylinder.origin(), axis)
        .map_err(OperationsError::from)?;
    let face_length = v1 - v0;
    let offset = options.offset;
    require(offset.is_finite(), || {
        format!("offset must be finite, got {offset}")
    })?;
    let length = options.length.unwrap_or(face_length - offset);
    positive(length, "length")?;
    // A span within rounding of an end of the face snaps to it, so a
    // whole-face request never leaves a sliver collar.
    let snap = 1e-9 * face_length.max(radius);
    let (offset, length) = {
        let lo = if offset.abs() <= snap { 0.0 } else { offset };
        let hi = if (offset + length - face_length).abs() <= snap {
            face_length
        } else {
            offset + length
        };
        (lo, hi - lo)
    };
    if offset < 0.0 || offset + length > face_length {
        return Err(ThreadError::refused(
            ThreadRefusal::SpanOutsideFace,
            format!(
                "the thread spans {offset}..{} along the face, which is {face_length} long",
                offset + length
            ),
        ));
    }
    let start = v0 + offset;

    // Boss or bore, from which side of the face is material.
    let mid = start + 0.5 * length;
    let probe = |shift: f64| -> Result<bool, ThreadError> {
        let point = cylinder.origin() + axis * mid + cylinder.x_axis() * (radius + shift);
        Ok(classify_point(topo, solid, point, 0.01, 1e-6)? == PointClassification::Inside)
    };
    let epsilon = 1e-3 * radius.min(form.depth);
    let kind = match (probe(-epsilon)?, probe(epsilon)?) {
        (true, false) => ThreadKind::External,
        (false, true) => ThreadKind::Internal,
        _ => {
            return Err(ThreadError::refused(
                ThreadRefusal::NotABossOrBore,
                "material lies on neither or both sides of the cylinder",
            ));
        }
    };

    let fit_tol = 1e-9 * radius;
    let major = match options.major_diameter {
        Some(diameter) => {
            positive(diameter, "major diameter")?;
            0.5 * diameter
        }
        None => match kind {
            ThreadKind::External => radius,
            ThreadKind::Internal => radius + form.depth,
        },
    };
    let minor = major - form.depth;
    if minor <= 0.0 {
        return Err(ThreadError::refused(
            ThreadRefusal::TooDeep,
            format!(
                "a depth of {} reaches the axis of a {} major diameter",
                form.depth,
                2.0 * major
            ),
        ));
    }
    let seam_gap = SEAM_DEPTH * form.depth;
    // (crest, root) radii and the band the tube rebuilds.
    let (crest, root, band) = match kind {
        ThreadKind::External => {
            if major > radius + fit_tol {
                return Err(ThreadError::refused(
                    ThreadRefusal::CylinderTooSmall,
                    format!(
                        "the thread's major diameter {} exceeds the boss diameter {}",
                        2.0 * major,
                        2.0 * radius
                    ),
                ));
            }
            let major = major.min(radius);
            let minor = major - form.depth;
            let seam = minor - seam_gap;
            if seam <= 0.0 {
                return Err(ThreadError::refused(
                    ThreadRefusal::TooDeep,
                    "the thread's root is too close to the axis to rebuild the thread",
                ));
            }
            (major, minor, (seam, radius))
        }
        ThreadKind::Internal => {
            if minor < radius - fit_tol {
                return Err(ThreadError::refused(
                    ThreadRefusal::BoreTooLarge,
                    format!(
                        "the bore diameter {} exceeds the thread's minor diameter {}",
                        2.0 * radius,
                        2.0 * minor
                    ),
                ));
            }
            let minor = minor.max(radius);
            let major = minor + form.depth;
            (minor, major, (radius, major + seam_gap))
        }
    };
    let (major_r, minor_r) = (crest.max(root), crest.min(root));
    let lead = form.lead();
    let mean = major_r + minor_r;
    let geometry = ThreadGeometry {
        kind,
        major_diameter: 2.0 * major_r,
        minor_diameter: 2.0 * minor_r,
        pitch: form.pitch,
        lead,
        crest_width: form.crest_width,
        root_width: form.root_width().max(0.0),
        lead_angle: (lead / (PI * mean)).atan(),
        length,
        turns: length / lead,
    };

    let reference = match options.x_direction {
        Some(direction) => direction,
        None => seam_direction(topo, face, cylinder.origin(), axis)?
            .unwrap_or_else(|| cylinder.x_axis()),
    };
    let frame = Frame::new(cylinder.origin() + axis * start, axis, reference)?;
    let tolerance = options
        .tolerance
        .unwrap_or(DEFAULT_TOLERANCE_PER_PITCH * form.pitch);

    remus_topology::transaction::run_transacted(topo, |topo| {
        // The rebuilt band must be solid material in the input.
        let expected = PI * band.1.mul_add(band.1, -(band.0 * band.0)) * length;
        let check = tube(topo, &frame, band.0, band.1, length)?;
        let held = match boolean(topo, BooleanOp::Intersect, solid, check) {
            Ok(common) => crate::measure::solid_volume(topo, common, 1e-3 * radius)?,
            Err(OperationsError::EmptyResult { .. }) => 0.0,
            Err(OperationsError::ExactOnlyUnattainable) => {
                return Err(ThreadError::refused(
                    ThreadRefusal::FeaturesInThreadRegion,
                    "the thread band could not be proven free of holes, slots or other features",
                ));
            }
            Err(error) => return Err(error.into()),
        };
        if (held - expected).abs() > BAND_VOLUME_REL_TOL * expected {
            return Err(ThreadError::refused(
                ThreadRefusal::FeaturesInThreadRegion,
                format!(
                    "the thread band holds {held} of {expected} volume units of material; \
                     a hole, slot or other feature crosses it"
                ),
            ));
        }

        let cut = tube(topo, &frame, band.0, band.1, length)?;
        let kept = boolean(topo, BooleanOp::Cut, solid, cut)?;
        let outline = profile::outline(form, crest, root, options.phase, 0.5 * tolerance)?;
        let ring = thread_ring(
            topo,
            &frame,
            &outline.segments,
            kind,
            if kind == ThreadKind::External {
                band.0
            } else {
                band.1
            },
            length,
            form,
            0.5 * tolerance,
        )?;
        let twist = ring.spans;
        let ring_bound = ring.deviation_bound;
        let ring = ring.solid;
        // Glue the ring on where the faces it meets are recognized; the fuse
        // is the general fallback.
        let (result, glued) = match glue(topo, kept, ring, &frame, kind, band, crest, length)? {
            Some(result) => (result, true),
            None => (boolean(topo, BooleanOp::Fuse, kept, ring)?, false),
        };
        let scale = band.1.max(length);
        Ok(ThreadOutcome {
            solid: result,
            geometry: geometry.clone(),
            deviation_bound: (64.0 * f64::EPSILON)
                .mul_add(scale, outline.remainder_bound + ring_bound),
            spans_along_axis: twist,
            spans_per_flank: outline.spans_per_flank,
            glued,
        })
    })
}

/// The swept thread ring: the outline twisted along the axis, with its
/// seam cylinder as a bore (external) or rim (internal).
#[allow(clippy::too_many_arguments)]
fn thread_ring(
    topo: &mut Topology,
    frame: &Frame,
    segments: &[Segment],
    kind: ThreadKind,
    seam: f64,
    length: f64,
    form: &ThreadForm,
    tolerance: f64,
) -> Result<Ring, ThreadError> {
    let vertex_tol = Tolerance::new().linear;
    let thread = build_wire(
        topo,
        segments,
        frame,
        vertex_tol,
        kind == ThreadKind::Internal,
    )?;
    let circle = make_circle_edge_with_ref(
        topo,
        frame.origin,
        if kind == ThreadKind::External {
            -frame.axis
        } else {
            frame.axis
        },
        seam,
        frame.x,
        vertex_tol,
    )?;
    let circle = topo.add_wire(Wire::new(vec![OrientedEdge::new(circle, true)], true)?);
    let (outer, inner) = match kind {
        ThreadKind::External => (thread, circle),
        ThreadKind::Internal => (circle, thread),
    };
    let profile = topo.add_face(Face::new(outer, vec![inner], frame.plane()));
    let sign = match form.hand {
        ThreadHand::Right => 1.0,
        ThreadHand::Left => -1.0,
    };
    let rate = sign * TAU / form.lead();
    // Stack segments of at most `MAX_SEGMENT_TURN` turns, each the twisted
    // extrusion of the previous one's top cap. Every segment shares its
    // bottom boundary with the one below, so dropping the coincident caps
    // leaves one closed shell with no boolean.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let segments = (length / (MAX_SEGMENT_TURN * form.lead())).ceil().max(1.0) as usize;
    if segments > MAX_SEGMENTS {
        return Err(ThreadError::invalid(format!(
            "a thread {length} long turns {} times; at most {} turns are supported",
            length / form.lead(),
            MAX_SEGMENTS as f64 * MAX_SEGMENT_TURN
        )));
    }
    #[allow(clippy::cast_precision_loss)]
    let height = length / segments as f64;
    let mut faces = Vec::new();
    let mut seam_faces = Vec::with_capacity(segments);
    let mut deviation_bound: f64 = 0.0;
    let mut spans = 0;
    let mut current = profile;
    for index in 0..segments {
        let twisted = crate::extrude::twist::twisted_extrude_impl(
            topo,
            current,
            frame.origin,
            frame.axis,
            height,
            rate,
            tolerance,
            MAX_THREAD_SPANS,
        )
        .map_err(|error| match error {
            OperationsError::Math(remus_math::MathError::ResourceLimitExceeded { .. }) => {
                ThreadError::refused(
                    ThreadRefusal::ToleranceTooTight,
                    format!(
                        "the thread needs more than {MAX_THREAD_SPANS} spans per segment for a \
                         tolerance of {tolerance}"
                    ),
                )
            }
            other => other.into(),
        })?;
        let construction = &twisted.construction;
        let seam_face = match kind {
            ThreadKind::External => construction.holes.first().and_then(|h| h.sides.first()),
            ThreadKind::Internal => construction.outer_sides.first(),
        }
        .copied()
        .ok_or_else(|| ThreadError::invalid("thread ring segment has no seam wall"))?;
        seam_faces.push(seam_face);
        for face in solid_faces(topo, twisted.solid)? {
            let dropped = (index > 0 && face == construction.bottom)
                || (index + 1 < segments && face == construction.top)
                || face == seam_face;
            if !dropped {
                faces.push(face);
            }
        }
        deviation_bound = deviation_bound.max(twisted.deviation_bound);
        spans += twisted.spans;
        current = construction.top;
    }
    faces.push(merge_seam_walls(topo, &seam_faces)?);
    let shell = topo.add_shell(remus_topology::shell::Shell::new(faces)?);
    let solid = topo.add_solid(remus_topology::solid::Solid::new(shell, vec![]));
    Ok(Ring {
        solid,
        deviation_bound,
        spans,
    })
}

/// A built thread ring and its fit report.
struct Ring {
    solid: SolidId,
    deviation_bound: f64,
    /// Hermite spans along the axis, over all segments.
    spans: usize,
}

/// Put the ring on the kept body without a boolean, or `None` when the
/// faces it meets are not the ones the tube cut leaves.
///
/// The ring and the kept body meet exactly on the seam cylinder and on the
/// two end planes, and their interiors are disjoint, so the union's boundary
/// is both boundaries less the shared seam walls, with the faces on each end
/// plane merged:
///
/// - where the kept face and the ring's cap face the same way (the thread
///   runs out at a free end), their regions join: the seam circle loop is
///   replaced by the thread outline;
/// - where they face opposite ways (a collar or shoulder beyond the band),
///   the ring's cap covers part of the kept face, which keeps only the rest:
///   its seam circle loop is replaced by the outline, wound the other way.
///
/// A crest flush with a collar pinches the merged face along the crest arcs;
/// [`flush_end`] splits it into crescents. Any unexpected face falls back to
/// the fuse.
#[allow(clippy::too_many_arguments)]
fn glue(
    topo: &mut Topology,
    kept: SolidId,
    ring: SolidId,
    frame: &Frame,
    kind: ThreadKind,
    band: (f64, f64),
    crest: f64,
    length: f64,
) -> Result<Option<SolidId>, ThreadError> {
    let tol = Tolerance::new().linear;
    let seam = match kind {
        ThreadKind::External => band.0,
        ThreadKind::Internal => band.1,
    };
    let scale = band.1.max(length);
    let on_axis = |p: Point3| {
        let offset = p - frame.origin;
        (offset - frame.axis * offset.dot(frame.axis)).length() <= 1e-9 * scale
    };
    let coaxial_circle =
        |topo: &Topology, edge: EdgeId, radius: f64| -> Result<bool, ThreadError> {
            let edge = topo.edge(edge)?;
            Ok(match edge.curve() {
                EdgeCurve::Circle(c) => {
                    edge.start() == edge.end()
                        && (c.radius() - radius).abs() <= 1e-9 * scale
                        && on_axis(c.center())
                        && c.normal().cross(frame.axis).length() <= 1e-9
                }
                EdgeCurve::Line
                | EdgeCurve::NurbsCurve(_)
                | EdgeCurve::Ellipse(_)
                | EdgeCurve::Hyperbola(_)
                | EdgeCurve::Parabola(_) => false,
            })
        };
    // A wire that is one closed coaxial circle of `radius`.
    let circle_wire = |topo: &Topology, wire: WireId, radius: f64| -> Result<bool, ThreadError> {
        let edges = topo.wire(wire)?.edges();
        Ok(edges.len() == 1 && coaxial_circle(topo, edges[0].edge(), radius)?)
    };
    let planes = [frame.origin, frame.origin + frame.axis * length];
    let on_plane = |topo: &Topology, face: FaceId, origin: Point3| -> Result<bool, ThreadError> {
        Ok(match topo.face(face)?.surface() {
            FaceSurface::Plane { normal, d } => {
                normal.cross(frame.axis).length() <= 1e-9
                    && (normal.dot(Vec3::new(origin.x(), origin.y(), origin.z())) - d).abs()
                        <= 1e-9 * scale
            }
            FaceSurface::Nurbs(_)
            | FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_) => false,
        })
    };
    let effective_normal = |topo: &Topology, face: FaceId| -> Result<f64, ThreadError> {
        let data = topo.face(face)?;
        let FaceSurface::Plane { normal, .. } = data.surface() else {
            return Ok(0.0);
        };
        let sign = if data.is_reversed() { -1.0 } else { 1.0 };
        Ok(sign * normal.dot(frame.axis))
    };

    let kept_solid = topo.solid(kept)?.clone();
    let kept_faces = topo.shell(kept_solid.outer_shell())?.faces().to_vec();
    let ring_faces = solid_faces(topo, ring)?;

    // The seam walls.
    let wall = |topo: &Topology, faces: &[FaceId]| -> Result<Option<FaceId>, ThreadError> {
        let mut found = None;
        for &face in faces {
            if let FaceSurface::Cylinder(c) = topo.face(face)?.surface()
                && (c.radius() - seam).abs() <= 1e-9 * scale
                && on_axis(c.origin())
            {
                if found.is_some() {
                    return Ok(None);
                }
                found = Some(face);
            }
        }
        Ok(found)
    };
    let (Some(kept_wall), Some(ring_wall)) = (wall(topo, &kept_faces)?, wall(topo, &ring_faces)?)
    else {
        return Ok(None);
    };

    let mut removed = vec![kept_wall, ring_wall];
    let mut merged = Vec::with_capacity(2);
    for origin in planes {
        // The kept face and the ring cap on this plane that hold a seam
        // circle loop.
        let find = |topo: &Topology, faces: &[FaceId]| -> Result<Option<FaceId>, ThreadError> {
            let mut found = None;
            for &face in faces {
                if !on_plane(topo, face, origin)? {
                    continue;
                }
                let data = topo.face(face)?;
                let loops =
                    std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied());
                let mut holds = false;
                for wire in loops {
                    holds |= circle_wire(topo, wire, seam)?;
                }
                if holds {
                    if found.is_some() {
                        return Ok(None);
                    }
                    found = Some(face);
                }
            }
            Ok(found)
        };
        let (Some(kept_face), Some(cap)) = (find(topo, &kept_faces)?, find(topo, &ring_faces)?)
        else {
            return Ok(None);
        };
        let kept_data = topo.face(kept_face)?.clone();
        let cap_data = topo.face(cap)?.clone();
        // The ring cap: the outline loop and the seam circle loop.
        let (outline, cap_seam) = match kind {
            ThreadKind::External => (
                cap_data.outer_wire(),
                cap_data.inner_wires().first().copied(),
            ),
            ThreadKind::Internal => (
                cap_data
                    .inner_wires()
                    .first()
                    .copied()
                    .unwrap_or_else(|| cap_data.outer_wire()),
                Some(cap_data.outer_wire()),
            ),
        };
        let Some(cap_seam) = cap_seam else {
            return Ok(None);
        };
        if cap_data.inner_wires().len() != 1 || !circle_wire(topo, cap_seam, seam)? {
            return Ok(None);
        }
        let same_way = {
            let (a, b) = (
                effective_normal(topo, kept_face)?,
                effective_normal(topo, cap)?,
            );
            if a.abs() < 0.5 || b.abs() < 0.5 {
                return Ok(None);
            }
            a * b > 0.0
        };
        let kept_outer_is_seam = circle_wire(topo, kept_data.outer_wire(), seam)?;
        let mut kept_inners = Vec::new();
        let mut seam_inner = false;
        for &wire in kept_data.inner_wires() {
            if !seam_inner && circle_wire(topo, wire, seam)? {
                seam_inner = true;
            } else {
                kept_inners.push(wire);
            }
        }
        if kept_outer_is_seam == seam_inner {
            return Ok(None);
        }
        // A loop of the kept face on the crest circle (a collar flush with
        // the crest) pinches the merged face along the crest arcs: it splits
        // into crescents instead.
        let mut flush = None;
        for wire in std::iter::once(kept_data.outer_wire()).chain(kept_inners.iter().copied()) {
            if circle_wire(topo, wire, crest)? {
                flush = Some(wire);
            } else {
                for oriented in topo.wire(wire)?.edges() {
                    if let EdgeCurve::Circle(c) = topo.edge(oriented.edge())?.curve()
                        && on_axis(c.center())
                        && (c.radius() - crest).abs() <= tol.max(1e-9 * scale)
                    {
                        return Ok(None);
                    }
                }
            }
        }
        if let Some(rim) = flush {
            // Only a step face that is exactly the annulus between the seam
            // and the collar, facing the other way from the cap.
            if same_way || !kept_inners.is_empty() && kept_inners != [rim] {
                return Ok(None);
            }
            let rim_edge = topo.wire(rim)?.edges()[0].edge();
            let Some(split) = flush_end(
                topo,
                &FlushEnd {
                    frame,
                    origin,
                    crest,
                    scale,
                    step: kept_face,
                    rim_edge,
                    outline,
                    kept_faces: &kept_faces,
                    ring_faces: &ring_faces,
                },
            )?
            else {
                return Ok(None);
            };
            removed.push(kept_face);
            removed.push(cap);
            removed.push(split.collar);
            merged.push(split.new_collar);
            merged.extend(split.crescents);
            continue;
        }
        // Expected pairings (see the doc comment): the seam loop is the kept
        // face's outer loop exactly where the regions join for an external
        // thread, and an inner loop where they join for an internal one.
        // Where the faces point opposite ways the outline bounds the merged
        // face from the other side, so it winds the other way.
        let outline = if same_way {
            outline
        } else {
            let reversed: Vec<OrientedEdge> = topo
                .wire(outline)?
                .edges()
                .iter()
                .rev()
                .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
                .collect();
            topo.add_wire(Wire::new(reversed, true)?)
        };
        let (outer, inners) = match (kind, kept_outer_is_seam, same_way) {
            (ThreadKind::External, true, true) => (outline, kept_inners),
            (ThreadKind::External, false, false) | (ThreadKind::Internal, false, true) => {
                let mut inners = kept_inners;
                inners.push(outline);
                (kept_data.outer_wire(), inners)
            }
            (ThreadKind::Internal, true, false) => (outline, kept_inners),
            _ => return Ok(None),
        };
        let surface = kept_data.surface().clone();
        let face = topo.add_face(if kept_data.is_reversed() {
            Face::new_reversed(outer, inners, surface)
        } else {
            Face::new(outer, inners, surface)
        });
        removed.push(kept_face);
        removed.push(cap);
        merged.push(face);
    }

    let faces: Vec<FaceId> = kept_faces
        .iter()
        .chain(&ring_faces)
        .copied()
        .filter(|face| !removed.contains(face))
        .chain(merged)
        .collect();
    // Every edge of a closed shell bounds exactly two faces.
    let mut uses: std::collections::BTreeMap<usize, usize> = std::collections::BTreeMap::new();
    for &face in &faces {
        let data = topo.face(face)?;
        for wire in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied()) {
            for oriented in topo.wire(wire)?.edges() {
                *uses.entry(oriented.edge().index()).or_default() += 1;
            }
        }
    }
    if uses.values().any(|&n| n != 2) {
        return Ok(None);
    }
    let shell = topo.add_shell(remus_topology::shell::Shell::new(faces)?);
    let solid = topo.add_solid(remus_topology::solid::Solid::new(
        shell,
        kept_solid.inner_shells().to_vec(),
    ));
    Ok(Some(solid))
}

/// The faces at a band end where a collar is flush with the crest.
struct FlushEnd<'a> {
    frame: &'a Frame,
    /// A point of the end plane on the axis.
    origin: Point3,
    crest: f64,
    scale: f64,
    /// The kept body's step face: the annulus between the seam circle and
    /// the collar's rim circle.
    step: FaceId,
    /// The rim circle, shared by the step face and the collar face.
    rim_edge: EdgeId,
    /// The ring cap's outline loop.
    outline: WireId,
    kept_faces: &'a [FaceId],
    ring_faces: &'a [FaceId],
}

/// What [`flush_end`] built.
struct FlushSplit {
    /// The collar face whose rim was split (removed).
    collar: FaceId,
    /// Its replacement, with the rim split at every point the crest touches.
    new_collar: FaceId,
    /// The step's remains between the outline and the rim.
    crescents: Vec<FaceId>,
}

/// Split a flush collar's rim where the crest touches it.
///
/// Along the rim circle the outline alternates between crest arcs on the
/// circle and runs inside it (an internal thread: outside it). Each run and
/// the rim arc across it bound one crescent of the step face; the crest arcs
/// are shared directly between the ring's crest faces and the collar face,
/// whose rim becomes the chain of crest and crescent arcs. The collar rim's
/// own vertex is kept: a ring vertex at the same point is replaced by it, and
/// an arc across it (a crest arc, in its crest face too) is split there.
/// Returns `None` for any configuration outside that pattern.
#[allow(clippy::too_many_lines)]
fn flush_end(topo: &mut Topology, end: &FlushEnd<'_>) -> Result<Option<FlushSplit>, ThreadError> {
    let frame = end.frame;
    let tol = Tolerance::new().linear;
    let near = (1e-9 * end.scale).max(tol);
    let angle_of = |p: Point3| {
        let offset = p - frame.origin;
        offset.dot(frame.y).atan2(offset.dot(frame.x))
    };
    let radius_of = |p: Point3| {
        let offset = p - frame.origin;
        (offset - frame.axis * offset.dot(frame.axis)).length()
    };
    // The collar face holding the rim edge.
    let mut collar = None;
    for &face in end.kept_faces {
        if face == end.step {
            continue;
        }
        let data = topo.face(face)?;
        for wire in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied()) {
            if topo
                .wire(wire)?
                .edges()
                .iter()
                .any(|oe| oe.edge() == end.rim_edge)
            {
                if collar.is_some() {
                    return Ok(None);
                }
                collar = Some(face);
            }
        }
    }
    let Some(collar) = collar else {
        return Ok(None);
    };

    // The outline counter-clockwise about the axis.
    let mut outline: Vec<OrientedEdge> = topo.wire(end.outline)?.edges().to_vec();
    let turn = |topo: &Topology, oe: &OrientedEdge| -> Result<Option<f64>, ThreadError> {
        Ok(match topo.edge(oe.edge())?.curve() {
            EdgeCurve::Circle(c) => {
                let sign = c.normal().dot(frame.axis).signum();
                Some(if oe.is_forward() { sign } else { -sign })
            }
            EdgeCurve::Line
            | EdgeCurve::NurbsCurve(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => None,
        })
    };
    let mut direction = 0.0;
    for oe in &outline {
        if let Some(t) = turn(topo, oe)? {
            if direction != 0.0 && t * direction < 0.0 {
                return Ok(None);
            }
            direction = t;
        }
    }
    if direction == 0.0 {
        return Ok(None);
    }
    if direction < 0.0 {
        outline = outline
            .iter()
            .rev()
            .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
            .collect();
    }
    let start_of = |topo: &Topology,
                    oe: &OrientedEdge|
     -> Result<remus_topology::vertex::VertexId, ThreadError> {
        let edge = topo.edge(oe.edge())?;
        Ok(if oe.is_forward() {
            edge.start()
        } else {
            edge.end()
        })
    };
    let on_rim =
        |topo: &Topology, v: remus_topology::vertex::VertexId| -> Result<bool, ThreadError> {
            Ok((radius_of(topo.vertex(v)?.point()) - end.crest).abs() <= near)
        };
    let is_crest_arc = |topo: &Topology, oe: &OrientedEdge| -> Result<bool, ThreadError> {
        let edge = topo.edge(oe.edge())?;
        Ok(
            matches!(edge.curve(), EdgeCurve::Circle(c) if (c.radius() - end.crest).abs() <= near)
                && on_rim(topo, edge.start())?
                && on_rim(topo, edge.end())?,
        )
    };

    // Rotate the outline to start at a rim vertex.
    let n = outline.len();
    let mut first = None;
    for i in 0..n {
        if on_rim(topo, start_of(topo, &outline[i])?)? {
            first = Some(i);
            break;
        }
    }
    let Some(first) = first else {
        return Ok(None);
    };
    outline.rotate_left(first);

    // The collar rim's vertex: reuse a ring vertex there, or split the
    // outline edge or rim gap it falls in.
    let rim = topo.edge(end.rim_edge)?.clone();
    let anchor = rim.start();
    let anchor_point = topo.vertex(anchor)?.point();
    let anchor_angle = angle_of(anchor_point);
    let ring_vertex = {
        let mut found = None;
        for oe in &outline {
            let v = start_of(topo, oe)?;
            if (topo.vertex(v)?.point() - anchor_point).length() <= near {
                found = Some(v);
            }
        }
        found
    };
    if let Some(v) = ring_vertex {
        // Every ring edge through `v` now runs through the collar's vertex.
        let mut edges = std::collections::BTreeSet::new();
        for &face in end.ring_faces {
            let data = topo.face(face)?;
            for wire in std::iter::once(data.outer_wire()).chain(data.inner_wires().iter().copied())
            {
                for oe in topo.wire(wire)?.edges() {
                    edges.insert(oe.edge().index());
                }
            }
        }
        for index in edges {
            let Some(edge) = topo.edge_id_from_index(index) else {
                continue;
            };
            let data = topo.edge_mut(edge)?;
            if data.start() == v {
                data.set_start(anchor);
            }
            if data.end() == v {
                data.set_end(anchor);
            }
        }
    }

    // Counter-clockwise span from angle `a` to angle `b`.
    let span = |a: f64, b: f64| (b - a).rem_euclid(TAU);
    let plane_center = end.origin;
    let arc = |topo: &mut Topology,
               from: remus_topology::vertex::VertexId,
               to: remus_topology::vertex::VertexId|
     -> Result<EdgeId, ThreadError> {
        let (p, q) = (topo.vertex(from)?.point(), topo.vertex(to)?.point());
        let (a, b) = (angle_of(p), angle_of(q));
        let radial = frame.x * a.cos() + frame.y * a.sin();
        let circle = Circle3D::new_with_ref(plane_center, frame.axis, end.crest, radial)?;
        let t0 = circle.project(plane_center + radial * end.crest);
        let mut edge = Edge::with_tolerance(from, to, EdgeCurve::Circle(circle), Some(tol));
        edge.set_trim(Some((t0, t0 + span(a, b))));
        Ok(topo.add_edge(edge))
    };

    // Walk the outline: crest arcs go straight into the rim chain; each run
    // between rim vertices closes into a crescent with a new rim arc.
    let mut chain: Vec<OrientedEdge> = Vec::new();
    let mut crescent_loops: Vec<Vec<OrientedEdge>> = Vec::new();
    let mut run: Vec<OrientedEdge> = Vec::new();
    let mut anchor_placed = ring_vertex.is_some();
    let mut i = 0;
    while i < n {
        let oe = outline[i];
        if run.is_empty() && is_crest_arc(topo, &oe)? {
            let (from, to) = {
                let e = topo.edge(oe.edge())?;
                if oe.is_forward() {
                    (e.start(), e.end())
                } else {
                    (e.end(), e.start())
                }
            };
            let (a, b) = (
                angle_of(topo.vertex(from)?.point()),
                angle_of(topo.vertex(to)?.point()),
            );
            if !anchor_placed && span(a, anchor_angle) > 0.0 && span(a, anchor_angle) < span(a, b) {
                // The collar's vertex lies on this crest arc: split it there,
                // in the ring's crest face too (its other face, the cap, is
                // dropped).
                anchor_placed = true;
                let pieces = [arc(topo, from, anchor)?, arc(topo, anchor, to)?];
                let mut replaced = false;
                for &face in end.ring_faces {
                    let data = topo.face(face)?.clone();
                    let wires = std::iter::once(data.outer_wire())
                        .chain(data.inner_wires().iter().copied());
                    for (k, wire) in wires.enumerate() {
                        let edges = topo.wire(wire)?.edges().to_vec();
                        let Some(at) = edges.iter().position(|e| e.edge() == oe.edge()) else {
                            continue;
                        };
                        if wire == end.outline || replaced {
                            continue;
                        }
                        let same = edges[at].is_forward() == oe.is_forward();
                        let split = if same {
                            [
                                OrientedEdge::new(pieces[0], true),
                                OrientedEdge::new(pieces[1], true),
                            ]
                        } else {
                            [
                                OrientedEdge::new(pieces[1], false),
                                OrientedEdge::new(pieces[0], false),
                            ]
                        };
                        let mut rebuilt = edges[..at].to_vec();
                        rebuilt.extend(split);
                        rebuilt.extend_from_slice(&edges[at + 1..]);
                        let new_wire = topo.add_wire(Wire::new(rebuilt, true)?);
                        let (mut outer, mut inners) =
                            (data.outer_wire(), data.inner_wires().to_vec());
                        if k == 0 {
                            outer = new_wire;
                        } else {
                            inners[k - 1] = new_wire;
                        }
                        topo.set_face_boundary_wires(face, outer, inners)?;
                        replaced = true;
                    }
                }
                if !replaced {
                    return Ok(None);
                }
                chain.extend(pieces.iter().map(|&e| OrientedEdge::new(e, true)));
                i += 1;
                continue;
            }
            chain.push(oe);
            i += 1;
            continue;
        }
        run.push(oe);
        i += 1;
        let end_vertex = {
            let e = topo.edge(oe.edge())?;
            if oe.is_forward() { e.end() } else { e.start() }
        };
        if on_rim(topo, end_vertex)? {
            let from = start_of(topo, &run[0])?;
            let (a, b) = (
                angle_of(topo.vertex(from)?.point()),
                angle_of(topo.vertex(end_vertex)?.point()),
            );
            // The rim across this run, split at the collar's vertex when it
            // falls inside.
            let pieces = if !anchor_placed
                && span(a, anchor_angle) > 0.0
                && span(a, anchor_angle) < span(a, b)
            {
                anchor_placed = true;
                vec![arc(topo, from, anchor)?, arc(topo, anchor, end_vertex)?]
            } else {
                vec![arc(topo, from, end_vertex)?]
            };
            chain.extend(pieces.iter().map(|&e| OrientedEdge::new(e, true)));
            let mut lp = std::mem::take(&mut run);
            lp.extend(pieces.iter().rev().map(|&e| OrientedEdge::new(e, false)));
            crescent_loops.push(lp);
        }
    }
    if !run.is_empty() || !anchor_placed || crescent_loops.is_empty() {
        return Ok(None);
    }

    // The chain, rotated to start at the collar's vertex.
    let Some(pos) = chain
        .iter()
        .position(|oe| start_of(topo, oe).is_ok_and(|v| v == anchor))
    else {
        return Ok(None);
    };
    chain.rotate_left(pos);

    // Winding sign of a loop about the axis, from sampled points.
    let winding = |topo: &Topology, edges: &[OrientedEdge]| -> Result<f64, ThreadError> {
        let mut points = Vec::new();
        for oe in edges {
            let e = topo.edge(oe.edge())?;
            let (sp, ep) = (
                topo.vertex(e.start())?.point(),
                topo.vertex(e.end())?.point(),
            );
            let (t0, t1) = e.strict_domain().map_err(|error| {
                ThreadError::invalid(format!("thread end edge has no domain: {error}"))
            })?;
            let samples: Vec<Point3> = (0..8)
                .map(|k| {
                    let f = f64::from(k) / 8.0;
                    let f = if oe.is_forward() { f } else { 1.0 - f };
                    e.curve()
                        .evaluate_with_endpoints(t0 + (t1 - t0) * f, sp, ep)
                })
                .collect();
            points.extend(samples);
        }
        let mut area = 0.0;
        for k in 0..points.len() {
            let (p, q) = (
                points[k] - frame.origin,
                points[(k + 1) % points.len()] - frame.origin,
            );
            area += p.dot(frame.x) * q.dot(frame.y) - p.dot(frame.y) * q.dot(frame.x);
        }
        Ok(area.signum())
    };
    // The kept step face fixes how this kernel winds a face's outer loop
    // about its effective normal.
    let step = topo.face(end.step)?.clone();
    let FaceSurface::Plane { normal, .. } = step.surface() else {
        return Ok(None);
    };
    let facing = normal.dot(frame.axis).signum() * if step.is_reversed() { -1.0 } else { 1.0 };
    let outer_edges = topo.wire(step.outer_wire())?.edges().to_vec();
    let convention = winding(topo, &outer_edges)? * facing;
    let mut crescents = Vec::with_capacity(crescent_loops.len());
    for lp in crescent_loops {
        let lp = if winding(topo, &lp)? * facing * convention > 0.0 {
            lp
        } else {
            lp.iter()
                .rev()
                .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
                .collect()
        };
        let wire = topo.add_wire(Wire::new(lp, true)?);
        let surface = step.surface().clone();
        crescents.push(topo.add_face(if step.is_reversed() {
            Face::new_reversed(wire, vec![], surface)
        } else {
            Face::new(wire, vec![], surface)
        }));
    }

    // The collar face with its rim replaced by the chain, run the way the
    // rim was.
    let collar_data = topo.face(collar)?.clone();
    let rewrite = |topo: &mut Topology, wire: WireId| -> Result<WireId, ThreadError> {
        let edges = topo.wire(wire)?.edges().to_vec();
        let Some(at) = edges.iter().position(|oe| oe.edge() == end.rim_edge) else {
            return Ok(wire);
        };
        let rim_turn = turn(topo, &edges[at])?.unwrap_or(1.0);
        let replacement: Vec<OrientedEdge> = if rim_turn > 0.0 {
            chain.clone()
        } else {
            chain
                .iter()
                .rev()
                .map(|oe| OrientedEdge::new(oe.edge(), !oe.is_forward()))
                .collect()
        };
        let mut rebuilt = edges[..at].to_vec();
        rebuilt.extend(replacement);
        rebuilt.extend_from_slice(&edges[at + 1..]);
        Ok(topo.add_wire(Wire::new(rebuilt, true)?))
    };
    let outer = rewrite(topo, collar_data.outer_wire())?;
    let mut inners = Vec::with_capacity(collar_data.inner_wires().len());
    for &wire in collar_data.inner_wires() {
        inners.push(rewrite(topo, wire)?);
    }
    let surface = collar_data.surface().clone();
    let new_collar = topo.add_face(if collar_data.is_reversed() {
        Face::new_reversed(outer, inners, surface)
    } else {
        Face::new(outer, inners, surface)
    });
    Ok(Some(FlushSplit {
        collar,
        new_collar,
        crescents,
    }))
}

/// One cylinder face for the seam walls of a stack of segments.
///
/// Each segment's seam wall is the straight extrusion of one closed circle:
/// two circles and one seam line used twice. The twist leaves a concentric
/// circle as built, so every segment's seam line lies on one generator and
/// the walls together are the cylinder between the first bottom circle and
/// the last top circle, split only at the segment planes. The kept body's
/// seam wall is one face, and the fuse pairs coincident faces one to one.
fn merge_seam_walls(topo: &mut Topology, walls: &[FaceId]) -> Result<FaceId, ThreadError> {
    let (Some(&first), Some(&last)) = (walls.first(), walls.last()) else {
        return Err(ThreadError::invalid("thread ring has no seam wall"));
    };
    if walls.len() == 1 {
        return Ok(first);
    }
    let is = |topo: &Topology, oe: &OrientedEdge, circle: bool| {
        topo.edge(oe.edge()).is_ok_and(|e| {
            if circle {
                matches!(e.curve(), EdgeCurve::Circle(_))
            } else {
                matches!(e.curve(), EdgeCurve::Line)
            }
        })
    };
    let first_face = topo.face(first)?.clone();
    let first_wire = topo.wire(first_face.outer_wire())?.edges().to_vec();
    let second_wire = topo
        .wire(topo.face(walls[1])?.outer_wire())?
        .edges()
        .to_vec();
    let last_wire = topo.wire(topo.face(last)?.outer_wire())?.edges().to_vec();
    let before_last = topo
        .wire(topo.face(walls[walls.len() - 2])?.outer_wire())?
        .edges()
        .to_vec();
    let malformed = || ThreadError::invalid("thread ring seam wall is not a plain cylinder band");
    let circles = |wire: &[OrientedEdge]| -> Vec<EdgeId> {
        wire.iter()
            .filter(|oe| is(topo, oe, true))
            .map(OrientedEdge::edge)
            .collect()
    };
    let shared = |a: &[EdgeId], b: &[EdgeId]| a.iter().copied().find(|e| b.contains(e));
    let (first_circles, last_circles) = (circles(&first_wire), circles(&last_wire));
    let mid = shared(&first_circles, &circles(&second_wire)).ok_or_else(malformed)?;
    let prev = shared(&last_circles, &circles(&before_last)).ok_or_else(malformed)?;
    let far = last_circles
        .iter()
        .copied()
        .find(|&e| e != prev)
        .ok_or_else(malformed)?;
    let line_of = |wire: &[OrientedEdge]| {
        wire.iter()
            .find(|oe| is(topo, oe, false))
            .map(OrientedEdge::edge)
    };
    let (line_first, line_last) = (
        line_of(&first_wire).ok_or_else(malformed)?,
        line_of(&last_wire).ok_or_else(malformed)?,
    );
    let middle = topo.edge(mid)?.start();
    let far_vertex = topo.edge(far)?.start();
    let lf = topo.edge(line_first)?.clone();
    let start_vertex = if lf.start() == middle {
        lf.end()
    } else if lf.end() == middle {
        lf.start()
    } else {
        return Err(malformed());
    };
    let ll = topo.edge(line_last)?.clone();
    if ll.start() != far_vertex && ll.end() != far_vertex {
        return Err(malformed());
    }
    let seam = topo.add_edge(Edge::new(start_vertex, far_vertex, EdgeCurve::Line));
    // `line_first` runs `start → middle` or back; the seam runs
    // `start → far`.
    let forward = |oe: &OrientedEdge| oe.is_forward() == (lf.start() == start_vertex);
    let merged: Vec<OrientedEdge> = first_wire
        .iter()
        .map(|oe| {
            if oe.edge() == mid {
                OrientedEdge::new(far, oe.is_forward())
            } else if oe.edge() == line_first {
                OrientedEdge::new(seam, forward(oe))
            } else {
                *oe
            }
        })
        .collect();
    let wire = topo.add_wire(Wire::new(merged, true)?);
    let surface = first_face.surface().clone();
    Ok(topo.add_face(if first_face.is_reversed() {
        Face::new_reversed(wire, vec![], surface)
    } else {
        Face::new(wire, vec![], surface)
    }))
}

/// Turn a closed counter-clockwise local outline into a wire with shared
/// vertices; `hole` winds it clockwise.
fn build_wire(
    topo: &mut Topology,
    segments: &[Segment],
    frame: &Frame,
    tolerance: f64,
    hole: bool,
) -> Result<WireId, ThreadError> {
    let vertices: Vec<_> = segments
        .iter()
        .map(|segment| topo.add_vertex(Vertex::new(frame.point(segment.start()), tolerance)))
        .collect();
    let mut edges: Vec<EdgeId> = Vec::with_capacity(segments.len());
    for (i, segment) in segments.iter().enumerate() {
        let start = vertices[i];
        let end = vertices[(i + 1) % vertices.len()];
        let edge = match segment {
            Segment::Line { .. } => Edge::new(start, end, EdgeCurve::Line),
            Segment::Arc { radius, from, to } => {
                let radial = frame.x * from.cos() + frame.y * from.sin();
                let circle = Circle3D::new_with_ref(frame.origin, frame.axis, *radius, radial)?;
                let t0 = circle.project(frame.origin + radial * *radius);
                let mut edge =
                    Edge::with_tolerance(start, end, EdgeCurve::Circle(circle), Some(tolerance));
                edge.set_trim(Some((t0, t0 + (to - from))));
                edge
            }
            Segment::Flank(curve) => {
                let curve = remus_math::nurbs::NurbsCurve::new(
                    curve.degree(),
                    curve.knots().to_vec(),
                    curve
                        .control_points()
                        .iter()
                        .map(|&p| frame.point(p))
                        .collect(),
                    curve.weights().to_vec(),
                )?;
                let domain = curve.domain();
                let mut edge =
                    Edge::with_tolerance(start, end, EdgeCurve::NurbsCurve(curve), Some(tolerance));
                edge.set_trim(Some(domain));
                edge
            }
        };
        edges.push(topo.add_edge(edge));
    }
    let oriented: Vec<OrientedEdge> = if hole {
        edges
            .iter()
            .rev()
            .map(|&edge| OrientedEdge::new(edge, false))
            .collect()
    } else {
        edges
            .iter()
            .map(|&edge| OrientedEdge::new(edge, true))
            .collect()
    };
    Ok(topo.add_wire(Wire::new(oriented, true)?))
}

/// Radial direction of the picked face's seam: the start of its first
/// closed circle or axis-parallel line edge.
fn seam_direction(
    topo: &Topology,
    face: FaceId,
    origin: Point3,
    axis: Vec3,
) -> Result<Option<Vec3>, ThreadError> {
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

/// Orthonormal placement frame: local `(x, y, 0)` maps to
/// `origin + x·X + y·Y`, with `Y = axis × X`.
struct Frame {
    origin: Point3,
    axis: Vec3,
    x: Vec3,
    y: Vec3,
}

impl Frame {
    fn new(origin: Point3, axis: Vec3, x_direction: Vec3) -> Result<Self, ThreadError> {
        require(x_direction.0.iter().all(|c| c.is_finite()), || {
            "x direction must be finite".to_owned()
        })?;
        let axis = axis
            .normalize()
            .map_err(|_| ThreadError::invalid("axis must be non-zero"))?;
        let projected = x_direction - axis * x_direction.dot(axis);
        let x = projected
            .normalize()
            .map_err(|_| ThreadError::invalid("x direction must not be parallel to the axis"))?;
        Ok(Self {
            origin,
            axis,
            x,
            y: axis.cross(x),
        })
    }

    fn point(&self, p: Point3) -> Point3 {
        self.origin + self.x * p.x() + self.y * p.y()
    }

    fn plane(&self) -> FaceSurface {
        FaceSurface::Plane {
            normal: self.axis,
            d: self
                .axis
                .dot(Vec3::new(self.origin.x(), self.origin.y(), self.origin.z())),
        }
    }
}

/// A coaxial tube of `length` along the frame axis from the frame origin.
fn tube(
    topo: &mut Topology,
    frame: &Frame,
    inner: f64,
    outer: f64,
    length: f64,
) -> Result<SolidId, ThreadError> {
    let tol = Tolerance::new().linear;
    let rim = make_circle_edge_with_ref(topo, frame.origin, frame.axis, outer, frame.x, tol)?;
    let outer_wire = topo.add_wire(Wire::new(vec![OrientedEdge::new(rim, true)], true)?);
    let hole = make_circle_edge_with_ref(topo, frame.origin, -frame.axis, inner, frame.x, tol)?;
    let hole = topo.add_wire(Wire::new(vec![OrientedEdge::new(hole, true)], true)?);
    let profile = topo.add_face(Face::new(outer_wire, vec![hole], frame.plane()));
    Ok(crate::extrude::extrude(topo, profile, frame.axis, length)?)
}

#[cfg(test)]
mod tests;
