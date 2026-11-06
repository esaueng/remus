//! Measured reads of circular-section carriers: the stations a walked blend
//! stored, or the sections of a lofted one, read back for a caller that
//! measures them against their supports itself.
//!
//! [`sampled_canal`](super::sampled_canal) certifies the stored-station
//! construction family exactly. These reads are for recognition, where the
//! carrier may come from elsewhere:
//!
//! - **Walked stations** (degree 2 x 1, one span per station) agree on their
//!   radius only to the walk's tolerance, and a band riding out to a tangent
//!   point closes on collapsed sections, which are left out.
//! - **Lofted arcs** (degree 2 in the section direction, interpolated
//!   across stations, as a rolling-ball stripe along a curved edge is built)
//!   have no stations in their control net. Their sections are read off the
//!   surface: the circle through each section's two ends and midpoint, at
//!   sample stations spread over every knot span.
//!
//! Nothing here certifies the surface; the reads name the sections and a
//! robust projection onto them (a carrier with a hundred spans defeats the
//! generic projection's coarse seed).

use remus_math::nurbs::surface::NurbsSurface;
use remus_math::vec::{Point3, Vec3};
use remus_topology::face::{Face, FaceSurface};

use super::{CanalNormal, SampledCanal, SampledSection, StationCheck, refusal};
use crate::BlendError;

/// Relative radius drift between the sections a measured read accepts.
pub(super) const MEASURED_RADIUS_DRIFT: f64 = 1e-6;
/// Relative radius drift a lofted carrier's sections may show between the
/// stations it interpolates (exact at the stations, a rational quadratic
/// only close to a circle between them).
const LOFT_RADIUS_DRIFT: f64 = 1e-3;
/// Sample stations per knot span of a lofted carrier.
const LOFT_SAMPLES_PER_SPAN: usize = 4;
/// Grid steps of the local projection seed search.
const SEED_STEPS: usize = 8;

/// A lofted circular-section carrier read off its surface.
struct Loft<'a> {
    surface: &'a NurbsSurface,
    radius: f64,
    convex: bool,
    sections: Vec<SampledSection>,
}

/// A carrier read for measurement.
enum Measured<'a> {
    Stations(CanalNormal<'a>),
    Loft(Loft<'a>),
}

impl<'a> Measured<'a> {
    fn read(face: &'a Face) -> Result<Self, BlendError> {
        match CanalNormal::read(face, StationCheck::Measured) {
            Ok(field) => Ok(Self::Stations(field)),
            Err(stations) => match Loft::read(face) {
                Ok(Some(loft)) => Ok(Self::Loft(loft)),
                Ok(None) => Err(stations),
                Err(error) => Err(error),
            },
        }
    }

    const fn surface(&self) -> &'a NurbsSurface {
        match self {
            Self::Stations(field) => field.surface,
            Self::Loft(loft) => loft.surface,
        }
    }

    fn centre(&self, v: f64) -> Result<Point3, BlendError> {
        match self {
            Self::Stations(field) => field.centre(v),
            Self::Loft(loft) => loft.centre(v),
        }
    }

    /// Seed stations: `(v, start, end)` of every resolved section.
    fn seeds(&self) -> Vec<(f64, Point3, Point3)> {
        match self {
            Self::Stations(field) => {
                let rows = field.surface.control_points();
                let knots = field.surface.knots_v();
                (0..rows[0].len())
                    .map(|index| (knots[index + 1], rows[0][index], rows[2][index]))
                    .collect()
            }
            Self::Loft(loft) => loft
                .sections
                .iter()
                .map(|section| (section.v, section.start, section.end))
                .collect(),
        }
    }

    fn canal(&self) -> Result<SampledCanal, BlendError> {
        match self {
            Self::Stations(field) => super::canal_of(field),
            Self::Loft(loft) => Ok(SampledCanal {
                radius: loft.radius,
                convex: loft.convex,
                sections: loft.sections.clone(),
            }),
        }
    }

    #[allow(clippy::cast_precision_loss)]
    fn project(&self, point: Point3) -> (f64, f64) {
        let surface = self.surface();
        let seeds = self.seeds();
        let mut nearest = (f64::INFINITY, 0_usize);
        for (index, &(v, start, end)) in seeds.iter().enumerate() {
            let distance = (point - start)
                .length()
                .min((point - end).length())
                .min((point - surface.evaluate(0.5, v)).length());
            if distance < nearest.0 {
                nearest = (distance, index);
            }
        }
        let low = seeds[nearest.1.saturating_sub(1)].0;
        let high = seeds[(nearest.1 + 1).min(seeds.len() - 1)].0;
        let mut seed = (f64::INFINITY, 0.5, seeds[nearest.1].0);
        for i in 0..=SEED_STEPS {
            let u = i as f64 / SEED_STEPS as f64;
            for j in 0..=SEED_STEPS {
                let v = (high - low).mul_add(j as f64 / SEED_STEPS as f64, low);
                let distance = (point - surface.evaluate(u, v)).length();
                if distance < seed.0 {
                    seed = (distance, u, v);
                }
            }
        }
        match remus_math::nurbs::projection::project_point_to_surface_seeded(
            surface,
            point,
            1e-10,
            (seed.1, seed.2),
        ) {
            Ok(found) if found.distance <= seed.0 + 1e-12 => (found.u, found.v),
            // Newton wandered off: the local grid node is the answer's
            // neighbourhood, and the best there is.
            Ok(_) | Err(_) => (seed.1, seed.2),
        }
    }
}

impl<'a> Loft<'a> {
    /// The loft's sections, or `None` when the face is not shaped like a
    /// lofted arc carrier at all.
    #[allow(clippy::cast_precision_loss)]
    fn read(face: &'a Face) -> Result<Option<Self>, BlendError> {
        let FaceSurface::Nurbs(surface) = face.surface() else {
            return Ok(None);
        };
        if surface.degree_u() != 2
            || surface.degree_v() < 2
            || surface.control_points().len() != 3
            || surface.domain_u() != (0.0, 1.0)
        {
            return Ok(None);
        }
        let (v0, v1) = surface.domain_v();
        let mut breaks: Vec<f64> = surface
            .knots_v()
            .iter()
            .copied()
            .filter(|&knot| knot >= v0 && knot <= v1)
            .collect();
        breaks.dedup();
        let mut sections = Vec::new();
        let mut radius = None;
        for pair in breaks.windows(2) {
            for step in 0..LOFT_SAMPLES_PER_SPAN {
                let v = (pair[1] - pair[0])
                    .mul_add(step as f64 / LOFT_SAMPLES_PER_SPAN as f64, pair[0]);
                sections.push(Self::section(surface, v)?);
            }
        }
        sections.push(Self::section(surface, v1)?);
        for section in &sections {
            let r = (section.start - section.centre).length();
            let first = *radius.get_or_insert(r);
            if !r.is_finite() || (r - first).abs() > first * LOFT_RADIUS_DRIFT + 1e-9 {
                return Err(refusal(
                    "lofted carrier does not retain one circular-section radius",
                ));
            }
        }
        let radius = radius.ok_or_else(|| refusal("lofted carrier has no sections"))?;
        let middle = &sections[sections.len() / 2];
        let probe = surface.evaluate(0.5, middle.v);
        let mut outward = surface.normal(0.5, middle.v)?;
        if face.is_reversed() {
            outward = -outward;
        }
        let alignment = outward.dot((probe - middle.centre).normalize()?);
        if alignment.abs() < 0.9 {
            return Err(refusal(
                "lofted carrier normal does not follow its sections",
            ));
        }
        Ok(Some(Self {
            surface,
            radius,
            convex: alignment > 0.0,
            sections,
        }))
    }

    fn section(surface: &NurbsSurface, v: f64) -> Result<SampledSection, BlendError> {
        let start = surface.evaluate(0.0, v);
        let end = surface.evaluate(1.0, v);
        let middle = surface.evaluate(0.5, v);
        Ok(SampledSection {
            v,
            centre: circumcentre(start, middle, end)
                .ok_or_else(|| refusal("lofted carrier section has collapsed"))?,
            start,
            end,
        })
    }

    fn centre(&self, v: f64) -> Result<Point3, BlendError> {
        Ok(Self::section(self.surface, v)?.centre)
    }
}

/// Centre of the circle through three points.
fn circumcentre(a: Point3, b: Point3, c: Point3) -> Option<Point3> {
    let (u, w) = (b - a, c - a);
    let normal = u.cross(w);
    let square = normal.dot(normal);
    let scale = u.dot(u).max(w.dot(w));
    if square <= 1e-24 * scale * scale {
        return None;
    }
    let offset: Vec3 = (w.cross(normal) * u.dot(u) + normal.cross(u) * w.dot(w)) * (0.5 / square);
    Some(a + offset)
}

/// [`sampled_canal`](super::sampled_canal) for measurement.
///
/// A walked carrier whose stations agree on one radius only to the walk's
/// tolerance, or whose end sections collapse as the band rides out to a
/// tangent point, and a lofted arc carrier, are read too. Collapsed stations
/// are left out of
/// [`SampledCanal::sections`]. The read certifies nothing: the caller
/// measures the sections and the surface against the supports.
///
/// # Errors
/// Refuses carriers outside both families, or whose sections disagree on
/// their radius by more than one part in a million.
pub fn sampled_canal_measured(face: &Face) -> Result<SampledCanal, BlendError> {
    Measured::read(face)?.canal()
}

/// [`sampled_section_centre`](super::sampled_section_centre) on a
/// [`sampled_canal_measured`] read; a point beside a collapsed end section
/// has no centre.
///
/// # Errors
/// Refuses an unqualified carrier or a point in a collapsed span.
pub fn sampled_section_centre_measured(face: &Face, point: Point3) -> Result<Point3, BlendError> {
    let measured = Measured::read(face)?;
    let (_, v) = measured.project(point);
    measured.centre(v)
}

/// Parameters of the point of a [`sampled_canal_measured`] carrier closest to
/// `point`, seeded from the nearest section and refined by Newton from a
/// local grid over the sections either side of it.
///
/// # Errors
/// Refuses an unqualified carrier.
pub fn sampled_project_measured(face: &Face, point: Point3) -> Result<(f64, f64), BlendError> {
    Ok(Measured::read(face)?.project(point))
}
