//! O4.7 typed direct-method results for the construction family.
//!
//! `extrudeDetailed` and `revolveDetailed` are additive twins of `extrude`
//! and `revolve`; the legacy methods keep their return values and
//! thrown-error behavior. Each runs the same engine path as its legacy
//! method on exact-supported profiles and returns the typed
//! [`SolidOperationDetailedResult`] the boolean twins return, never throwing
//! on a refusal:
//!
//! - **exact**: `status: "ok"`, `details.quality: "exact"`;
//! - **refused**: `status: "error"` with the kernel diagnostic `code` and
//!   `category`, and the topology rolled back to its pre-call state.
//!
//! The twins are exact-only. Extrusion keeps open profile curves on exact
//! swept carriers. Revolution refuses curved profiles whose fallback bands
//! or later rings use line chords; true circular torus profiles and the
//! native analytic full-turn path remain supported. There is no
//! `approximate` success or `exactOnly` flag: supported profiles commit with
//! `quality: "exact"`, while chorded profiles return `exact_only_unattainable`.
//!
//! WASM `revolve` accepts degrees in `(0, 360]`; the native operation
//! receives radians. Both the direct twin and the batch op preserve that
//! conversion exactly (`angle_degrees.to_radians()`), matching the legacy
//! direct and batch paths.
//!
//! Transactionality reuses the native coordination: `extrude` and `revolve`
//! both run inside `run_transacted`, so a refusal already restores the exact
//! pre-call topology. The twins take no additional full-session snapshot;
//! the batch layer's per-op rollback covers the dispatch boundary as for
//! every other mutating op.
//!
//! The same bodies back the `executeBatch`/`executeBatchV2` ops of the same
//! names, whose `ok` value is this envelope, so direct and batch results are
//! identical by construction.

#![allow(clippy::missing_errors_doc)]

use serde_json::{Map, Value};
use tsify::Tsify as _;
use wasm_bindgen::prelude::*;

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use crate::error::{StructuredWasmError, WasmError, validate_finite};
use crate::handles::solid_id_to_u32;
use crate::kernel::BrepKernel;
use crate::types::SolidOperationDetailedResult;

/// Quality label of every successful construction result.
const QUALITY_EXACT: &str = "exact";

fn chorded_profile_refusal(operation: &str) -> StructuredWasmError {
    StructuredWasmError::exact_only_unattainable(format!(
        "{operation} would replace a profile curve with line chords"
    ))
}

fn ensure_exact_extrude_profile(topo: &Topology, face: FaceId) -> Result<(), StructuredWasmError> {
    let profile = topo.face(face)?;
    for (wire_id, is_inner) in std::iter::once((profile.outer_wire(), false)).chain(
        profile
            .inner_wires()
            .iter()
            .copied()
            .map(|wire| (wire, true)),
    ) {
        let wire = topo.wire(wire_id)?;
        for oriented in wire.edges() {
            let edge = topo.edge(oriented.edge())?;
            if edge.start() != edge.end() {
                continue;
            }
            let stays_exact = match edge.curve() {
                EdgeCurve::Line | EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => true,
                EdgeCurve::Circle(_) => !is_inner || wire.edges().len() == 1,
                EdgeCurve::Ellipse(_) => !is_inner,
                EdgeCurve::NurbsCurve(nurbs) => {
                    use remus_geometry::convert::{RecognizedCurve, recognize_curve};
                    match recognize_curve(nurbs, Tolerance::new().linear * 100.0) {
                        RecognizedCurve::Circle { .. } => !is_inner || wire.edges().len() == 1,
                        RecognizedCurve::Ellipse { .. } => !is_inner,
                        _ => false,
                    }
                }
            };
            if !stays_exact {
                return Err(chorded_profile_refusal("extrude"));
            }
        }
    }
    Ok(())
}

fn ensure_exact_revolve_profile(
    topo: &Topology,
    face: FaceId,
    origin: Point3,
    direction: Vec3,
    angle_degrees: f64,
) -> Result<(), StructuredWasmError> {
    let profile = topo.face(face)?;
    if !matches!(profile.surface(), FaceSurface::Plane { .. })
        && angle_degrees.to_radians() < std::f64::consts::TAU - Tolerance::new().angular
    {
        let points: Vec<_> = topo
            .wire(profile.outer_wire())?
            .edges()
            .iter()
            .map(|oriented| {
                let edge = topo.edge(oriented.edge())?;
                let start = if oriented.is_forward() {
                    edge.start()
                } else {
                    edge.end()
                };
                Ok::<_, StructuredWasmError>(topo.vertex(start)?.point())
            })
            .collect::<Result<_, _>>()?;
        if points.len() < 3 {
            return Ok(());
        }
        let mut normal = Vec3::new(0.0, 0.0, 0.0);
        for (point, next) in points.iter().zip(points.iter().cycle().skip(1)) {
            normal += Vec3::new(
                (point.y() - next.y()) * (point.z() + next.z()),
                (point.z() - next.z()) * (point.x() + next.x()),
                (point.x() - next.x()) * (point.y() + next.y()),
            );
        }
        if normal.normalize().is_err() {
            // The native path reports its degenerate boundary normal first.
            return Ok(());
        }
    }
    let mut has_curved_edge = false;
    for wire_id in
        std::iter::once(profile.outer_wire()).chain(profile.inner_wires().iter().copied())
    {
        for oriented in topo.wire(wire_id)?.edges() {
            let edge = topo.edge(oriented.edge())?;
            has_curved_edge |= matches!(
                edge.curve(),
                EdgeCurve::Circle(_)
                    | EdgeCurve::Ellipse(_)
                    | EdgeCurve::NurbsCurve(_)
                    | EdgeCurve::Hyperbola(_)
                    | EdgeCurve::Parabola(_)
            );
        }
    }
    if !has_curved_edge {
        return Ok(());
    }

    let is_full = angle_degrees.to_radians() >= std::f64::consts::TAU - Tolerance::new().angular;
    if is_full
        && remus_operations::revolve::supports_analytic_full_revolution(
            topo, face, origin, direction,
        )
        .map_err(StructuredWasmError::from)?
    {
        return Ok(());
    }

    // The remaining exact path is the single true-circle torus.
    let wire = topo.wire(profile.outer_wire())?;
    let exact_torus = (|| -> Option<bool> {
        if !profile.inner_wires().is_empty() || wire.edges().len() != 1 {
            return None;
        }
        let edge = topo.edge(wire.edges()[0].edge()).ok()?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            return None;
        };
        if edge.start() != edge.end() {
            return None;
        }
        let FaceSurface::Plane { normal, d } = profile.surface() else {
            return None;
        };
        let axis = direction.normalize().ok()?;
        let axis_plane_offset = normal.x().mul_add(
            origin.x(),
            normal.y().mul_add(origin.y(), normal.z() * origin.z()),
        ) - d;
        if !normal.dot(axis).is_finite()
            || normal.dot(axis).abs() > 1e-9
            || !axis_plane_offset.is_finite()
            || axis_plane_offset.abs() > Tolerance::new().linear * 100.0
        {
            return None;
        }
        let radial = circle.center() - origin;
        let major = (radial - axis * radial.dot(axis)).length();
        let tol = Tolerance::new().linear;
        if major <= circle.radius() + tol {
            return None;
        }
        if angle_degrees < 360.0 {
            let seam = topo.vertex(edge.start()).ok()?.point() - origin;
            let seam_radius = (seam - axis * seam.dot(axis)).length();
            if seam_radius <= tol {
                return None;
            }
        }
        Some(true)
    })()
    .unwrap_or(false);
    if exact_torus {
        Ok(())
    } else {
        Err(chorded_profile_refusal("revolve"))
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Extrude a planar face along a direction vector, as typed data.
    ///
    /// Additive twin of [`extrude`](Self::extrude_face): the same validation
    /// order (finite `dir_x`, `dir_y`, `dir_z`, `distance`, then the face
    /// handle) and the same engine for exact-supported profiles. Closed
    /// profiles needing line chords refuse with `exact_only_unattainable`;
    /// the legacy method is unchanged. Success has `details.quality: "exact"`.
    #[wasm_bindgen(js_name = "extrudeDetailed")]
    pub fn extrude_detailed(
        &mut self,
        face: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        distance: f64,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .extrude_detailed_impl(face, dir_x, dir_y, dir_z, distance)
            .into_ts()?)
    }

    /// Revolve a planar face around an axis, as typed data.
    ///
    /// Additive twin of [`revolve`](Self::revolve_face): the same validation
    /// order (finite `ox`, `oy`, `oz`, `dx`, `dy`, `dz`, `angle_degrees`,
    /// then the `(0, 360]` range check, then the face handle), the same
    /// degrees-to-radians conversion, and the same engine for exact-supported
    /// profiles. Curved profiles needing line chords refuse with
    /// `exact_only_unattainable`; the legacy method is unchanged. Success has
    /// `details.quality: "exact"`.
    #[wasm_bindgen(js_name = "revolveDetailed")]
    #[allow(clippy::too_many_arguments)]
    pub fn revolve_detailed(
        &mut self,
        face: u32,
        ox: f64,
        oy: f64,
        oz: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        angle_degrees: f64,
    ) -> Result<tsify::Ts<SolidOperationDetailedResult>, JsError> {
        Ok(self
            .revolve_detailed_impl(face, ox, oy, oz, dx, dy, dz, angle_degrees)
            .into_ts()?)
    }
}

/// Natively-testable bodies shared by the direct twins and the batch ops.
impl BrepKernel {
    pub(crate) fn extrude_detailed_impl(
        &mut self,
        face: u32,
        dir_x: f64,
        dir_y: f64,
        dir_z: f64,
        distance: f64,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<u32, StructuredWasmError> {
            validate_finite(dir_x, "dir_x").map_err(StructuredWasmError::from)?;
            validate_finite(dir_y, "dir_y").map_err(StructuredWasmError::from)?;
            validate_finite(dir_z, "dir_z").map_err(StructuredWasmError::from)?;
            validate_finite(distance, "distance").map_err(StructuredWasmError::from)?;
            let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
            let direction = Vec3::new(dir_x, dir_y, dir_z);
            let tol = Tolerance::new();
            if !tol.approx_eq(direction.length_squared(), 0.0) && !tol.approx_eq(distance, 0.0) {
                ensure_exact_extrude_profile(self.topo(), face_id)?;
            }
            // `extrude` owns its native rollback transaction; no additional
            // full-session snapshot is taken here.
            let solid =
                remus_operations::extrude::extrude(self.topo_mut(), face_id, direction, distance)
                    .map_err(StructuredWasmError::from)?;
            Ok(solid_id_to_u32(solid))
        })();

        match result {
            Ok(value) => {
                let mut details = Map::new();
                details.insert("quality".into(), Value::from(QUALITY_EXACT));
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("extrude"))
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn revolve_detailed_impl(
        &mut self,
        face: u32,
        ox: f64,
        oy: f64,
        oz: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        angle_degrees: f64,
    ) -> SolidOperationDetailedResult {
        let result = (|| -> Result<u32, StructuredWasmError> {
            validate_finite(ox, "ox").map_err(StructuredWasmError::from)?;
            validate_finite(oy, "oy").map_err(StructuredWasmError::from)?;
            validate_finite(oz, "oz").map_err(StructuredWasmError::from)?;
            validate_finite(dx, "dx").map_err(StructuredWasmError::from)?;
            validate_finite(dy, "dy").map_err(StructuredWasmError::from)?;
            validate_finite(dz, "dz").map_err(StructuredWasmError::from)?;
            validate_finite(angle_degrees, "angle_degrees").map_err(StructuredWasmError::from)?;
            if angle_degrees <= 0.0 || angle_degrees > 360.0 {
                return Err(StructuredWasmError::from(WasmError::InvalidInput {
                    reason: format!("angle_degrees must be in (0, 360], got {angle_degrees}"),
                }));
            }
            let face_id = self.resolve_face(face).map_err(StructuredWasmError::from)?;
            let origin = Point3::new(ox, oy, oz);
            let direction = Vec3::new(dx, dy, dz);
            if !Tolerance::new().approx_eq(direction.length_squared(), 0.0) {
                ensure_exact_revolve_profile(
                    self.topo(),
                    face_id,
                    origin,
                    direction,
                    angle_degrees,
                )?;
            }
            let angle_radians = angle_degrees.to_radians();
            // `revolve` owns its native rollback transaction; no additional
            // full-session snapshot is taken here.
            let solid = remus_operations::revolve::revolve(
                self.topo_mut(),
                face_id,
                origin,
                direction,
                angle_radians,
            )
            .map_err(StructuredWasmError::from)?;
            Ok(solid_id_to_u32(solid))
        })();

        match result {
            Ok(value) => {
                let mut details = Map::new();
                details.insert("quality".into(), Value::from(QUALITY_EXACT));
                SolidOperationDetailedResult::success_with_details(value, details)
            }
            Err(error) => {
                SolidOperationDetailedResult::error(error.with_direct_operation("revolve"))
            }
        }
    }
}

#[cfg(test)]
mod tests;
