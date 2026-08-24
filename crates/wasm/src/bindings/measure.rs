//! Measurement, validation, and distance query bindings.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use remus_math::vec::Point3;
use remus_operations::measure;

use crate::error::{validate_finite, validate_positive};
use crate::kernel::BrepKernel;

fn detailed_validation_result(
    report: remus_operations::validate::ValidationReport,
) -> crate::types::ValidationReportResult {
    #[allow(clippy::cast_possible_truncation)]
    let error_count = report.error_count() as u32;
    #[allow(clippy::cast_possible_truncation)]
    let warning_count = report.warning_count() as u32;
    let issues = report
        .issues
        .into_iter()
        .map(|issue| crate::types::ValidationIssueResult {
            severity: match issue.severity {
                remus_operations::validate::Severity::Error => "error".into(),
                remus_operations::validate::Severity::Warning => "warning".into(),
            },
            description: issue.description,
        })
        .collect();

    crate::types::ValidationReportResult {
        error_count,
        warning_count,
        issues,
    }
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Measurement ───────────────────────────────────────────────

    /// Compute the axis-aligned bounding box of a solid.
    ///
    /// Returns `[min_x, min_y, min_z, max_x, max_y, max_z]`.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or has no vertices.
    #[wasm_bindgen(js_name = "boundingBox")]
    pub fn bounding_box(&self, solid: u32) -> Result<Vec<f64>, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let aabb = measure::solid_bounding_box(&self.topo, solid_id)?;
        Ok(vec![
            aabb.min.x(),
            aabb.min.y(),
            aabb.min.z(),
            aabb.max.x(),
            aabb.max.y(),
            aabb.max.z(),
        ])
    }

    /// Compute the volume of a solid.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or tessellation fails.
    #[wasm_bindgen(js_name = "volume")]
    pub fn volume(&self, solid: u32, deflection: f64) -> Result<f64, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        Ok(measure::solid_volume(&self.topo, solid_id, deflection)?)
    }

    /// Compute the total surface area of a solid.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or tessellation fails.
    #[wasm_bindgen(js_name = "surfaceArea")]
    pub fn surface_area(&self, solid: u32, deflection: f64) -> Result<f64, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        Ok(measure::solid_surface_area(
            &self.topo, solid_id, deflection,
        )?)
    }

    /// Compute the area of a single face.
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid or tessellation fails.
    #[wasm_bindgen(js_name = "faceArea")]
    pub fn face_area(&self, face: u32, deflection: f64) -> Result<f64, JsError> {
        validate_positive(deflection, "deflection")?;
        let face_id = self.resolve_face(face)?;
        Ok(measure::face_area(&self.topo, face_id, deflection)?)
    }

    /// Compute the center of mass of a solid (uniform density).
    ///
    /// Returns `[x, y, z]`.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid has zero volume or tessellation fails.
    #[wasm_bindgen(js_name = "centerOfMass")]
    pub fn center_of_mass(&self, solid: u32, deflection: f64) -> Result<Vec<f64>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let com = measure::solid_center_of_mass(&self.topo, solid_id, deflection)?;
        Ok(vec![com.x(), com.y(), com.z()])
    }

    /// Compute the uniform-density inertia tensor about the center of mass.
    ///
    /// Returns the symmetric 3x3 matrix in row-major order, expressed in the
    /// kernel's global axes. Density is `1`; with the canonical millimetre
    /// length unit, each component has units of `mm^5`.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid, integration fails, or
    /// the solid has effectively zero volume.
    #[wasm_bindgen(js_name = "inertiaTensor")]
    pub fn inertia_tensor(&self, solid: u32) -> Result<Vec<f64>, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let properties = remus_check::properties::solid_properties(
            &self.topo,
            solid_id,
            &remus_check::properties::PropertiesOptions::default(),
        )?;
        let matrix = properties.matrix_of_inertia();
        Ok(matrix.into_iter().flatten().collect())
    }

    /// Compute the full mass properties of a solid (unit density).
    ///
    /// Returns a JSON string containing
    /// `{ volume, centerOfMass, inertia, principalMoments, principalAxes }`
    /// (see the `MassPropertiesResult` TypeScript type). The inertia tensor
    /// `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` is taken about the center of mass in
    /// the global axis directions; `principalAxes` is row-major, one unit
    /// vector per ascending principal moment. For just the 3x3 matrix, see
    /// [`inertia_tensor`](Self::inertia_tensor).
    ///
    /// Integration runs on the exact face geometry (analytic and NURBS
    /// surfaces, no tessellation), so there is no deflection parameter.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid, integration fails,
    /// or the solid has zero volume.
    #[wasm_bindgen(js_name = "massProperties")]
    pub fn mass_properties(&self, solid: u32) -> Result<JsValue, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let props = measure::mass_properties(&self.topo, solid_id)?;
        let (moments, axes) = props.principal_inertia();
        let result = crate::types::MassPropertiesResult {
            volume: props.mass,
            center_of_mass: vec![props.center.x(), props.center.y(), props.center.z()],
            inertia: props.inertia.to_vec(),
            principal_moments: moments.to_vec(),
            principal_axes: axes.iter().flatten().copied().collect(),
        };
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }

    /// Tessellate a solid and report position-welded mesh quality metrics.
    ///
    /// Returns a JSON string containing
    /// `{ boundaryEdges, nonManifoldEdges, eulerCharacteristic, isWatertight }`
    /// (see the `MeshQualityResult` TypeScript type). Vertices are welded on a
    /// 1 µm grid before counting, so position-duplicate vertices cannot mask
    /// a leak. Use before export to verify the mesh is watertight.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or tessellation fails.
    #[wasm_bindgen(js_name = "meshQuality")]
    pub fn mesh_quality(&self, solid: u32, deflection: f64) -> Result<JsValue, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let mesh =
            remus_operations::tessellate::tessellate_solid(&self.topo, solid_id, deflection)?;
        let quality = remus_operations::tessellate::welded_mesh_quality(&mesh);
        #[allow(clippy::cast_possible_truncation)]
        let result = crate::types::MeshQualityResult {
            boundary_edges: quality.boundary_edges as u32,
            non_manifold_edges: quality.non_manifold_edges as u32,
            euler_characteristic: quality.euler_characteristic as i32,
            is_watertight: quality.is_watertight(),
        };
        Ok(serde_json::to_string(&result)
            .map_err(|e| JsError::new(&e.to_string()))?
            .into())
    }

    /// Classify a point relative to a solid: inside, outside, or on boundary.
    ///
    /// Returns `"inside"`, `"outside"`, or `"boundary"`.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid.
    #[wasm_bindgen(js_name = "classifyPoint")]
    pub fn classify_point(
        &self,
        solid: u32,
        x: f64,
        y: f64,
        z: f64,
        tolerance: f64,
    ) -> Result<String, JsError> {
        for (name, value) in [("x", x), ("y", y), ("z", z), ("tolerance", tolerance)] {
            validate_finite(value, name)?;
        }
        let solid_id = self.resolve_solid(solid)?;
        let point = remus_math::vec::Point3::new(x, y, z);
        let result = remus_operations::classify::classify_point(
            &self.topo, solid_id, point, 0.1, tolerance,
        )?;
        Ok(match result {
            remus_operations::classify::PointClassification::Inside => "inside".into(),
            remus_operations::classify::PointClassification::Outside => "outside".into(),
            remus_operations::classify::PointClassification::OnBoundary => "boundary".into(),
        })
    }

    /// Compute the length of an edge.
    ///
    /// # Errors
    ///
    /// Returns an error if the edge handle is invalid.
    #[wasm_bindgen(js_name = "edgeLength")]
    pub fn edge_length(&self, edge: u32) -> Result<f64, JsError> {
        let edge_id = self.resolve_edge(edge)?;
        Ok(measure::edge_length(&self.topo, edge_id)?)
    }

    /// Compute the perimeter of a face.
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid.
    #[wasm_bindgen(js_name = "facePerimeter")]
    pub fn face_perimeter(&self, face: u32) -> Result<f64, JsError> {
        let face_id = self.resolve_face(face)?;
        Ok(measure::face_perimeter(&self.topo, face_id)?)
    }

    /// Validate a solid, returning the number of errors found.
    ///
    /// Returns 0 if the solid is valid.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid.
    #[wasm_bindgen(js_name = "validateSolid")]
    pub fn validate_solid(&self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let report = remus_operations::validate::validate_solid(&self.topo, solid_id)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(report.error_count() as u32)
    }

    /// Validate a solid with relaxed checks suitable for assembled geometry.
    ///
    /// Operations like boolean, fillet, and shell produce geometrically
    /// correct shapes that may not have fully manifold topology (faces
    /// from different operations may not share edges). This validation
    /// skips Euler characteristic, boundary edge, non-manifold edge, and
    /// shell connectivity checks.
    ///
    /// Returns 0 if the solid passes all structural checks.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid.
    #[wasm_bindgen(js_name = "validateSolidRelaxed")]
    pub fn validate_solid_relaxed(&self, solid: u32) -> Result<u32, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let report = remus_operations::validate::validate_solid_relaxed(&self.topo, solid_id)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(report.error_count() as u32)
    }

    /// Validate a solid with configurable tolerance scaling.
    ///
    /// `tolerance_scale` multiplies geometric tolerances used for the
    /// face-normal and face-area checks. Use `10.0` to reduce false
    /// positives on NURBS faces from fillet/shell operations.
    ///
    /// Returns 0 if the solid is valid.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid.
    #[wasm_bindgen(js_name = "validateSolidWithOptions")]
    pub fn validate_solid_with_options(
        &self,
        solid: u32,
        tolerance_scale: f64,
    ) -> Result<u32, JsError> {
        validate_finite(tolerance_scale, "tolerance_scale")?;
        let solid_id = self.resolve_solid(solid)?;
        let options = remus_operations::validate::ValidationOptions {
            tolerance_scale,
            ..remus_operations::validate::ValidationOptions::default()
        };
        let report = remus_operations::validate::validate_solid_with_options(
            &self.topo, solid_id, &options,
        )?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(report.error_count() as u32)
    }

    /// Validate a solid and return every diagnostic.
    ///
    /// Returns a JSON string containing
    /// `{ errorCount, warningCount, issues: [{ severity, description }] }`
    /// (see the `ValidationReportResult` TypeScript type). Diagnostics come
    /// from the same operations validator used by [`validate_solid`](Self::validate_solid).
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or validation fails.
    #[wasm_bindgen(js_name = "validateSolidDetailed")]
    pub fn validate_solid_detailed(&self, solid: u32) -> Result<JsValue, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let report = remus_operations::validate::validate_solid(&self.topo, solid_id)?;
        let result = detailed_validation_result(report);
        Ok(serde_json::to_string(&result)
            .map_err(|error| JsError::new(&error.to_string()))?
            .into())
    }

    /// Validate a solid with configurable tolerance scaling and return every
    /// diagnostic.
    ///
    /// `tolerance_scale` has the same meaning as in
    /// [`validate_solid_with_options`](Self::validate_solid_with_options).
    /// Returns a JSON string containing
    /// `{ errorCount, warningCount, issues: [{ severity, description }] }`.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or validation fails.
    #[wasm_bindgen(js_name = "validateSolidDetailedWithOptions")]
    pub fn validate_solid_detailed_with_options(
        &self,
        solid: u32,
        tolerance_scale: f64,
    ) -> Result<JsValue, JsError> {
        validate_finite(tolerance_scale, "tolerance_scale")?;
        let solid_id = self.resolve_solid(solid)?;
        let options = remus_operations::validate::ValidationOptions {
            tolerance_scale,
            ..remus_operations::validate::ValidationOptions::default()
        };
        let report = remus_operations::validate::validate_solid_with_options(
            &self.topo, solid_id, &options,
        )?;
        let result = detailed_validation_result(report);
        Ok(serde_json::to_string(&result)
            .map_err(|error| JsError::new(&error.to_string()))?
            .into())
    }

    // ── Distance ──────────────────────────────────────────────────

    /// Compute minimum distance from a point to a solid.
    ///
    /// Returns `[distance, closest_x, closest_y, closest_z]`.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid.
    #[wasm_bindgen(js_name = "pointToSolidDistance")]
    pub fn point_to_solid_distance(
        &self,
        px: f64,
        py: f64,
        pz: f64,
        solid: u32,
    ) -> Result<Vec<f64>, JsError> {
        validate_finite(px, "px")?;
        validate_finite(py, "py")?;
        validate_finite(pz, "pz")?;
        let solid_id = self.resolve_solid(solid)?;
        let result = remus_operations::distance::point_to_solid_distance(
            &self.topo,
            Point3::new(px, py, pz),
            solid_id,
        )?;
        Ok(vec![
            result.distance,
            result.point_b.x(),
            result.point_b.y(),
            result.point_b.z(),
        ])
    }

    /// Compute minimum distance between two solids.
    ///
    /// Returns `[distance, point_a_x, point_a_y, point_a_z, point_b_x, point_b_y, point_b_z]`.
    ///
    /// # Errors
    ///
    /// Returns an error if either solid handle is invalid.
    #[wasm_bindgen(js_name = "solidToSolidDistance")]
    pub fn solid_to_solid_distance(&self, a: u32, b: u32) -> Result<Vec<f64>, JsError> {
        let a_id = self.resolve_solid(a)?;
        let b_id = self.resolve_solid(b)?;
        let result = remus_operations::distance::solid_to_solid_distance(&self.topo, a_id, b_id)?;
        Ok(vec![
            result.distance,
            result.point_a.x(),
            result.point_a.y(),
            result.point_a.z(),
            result.point_b.x(),
            result.point_b.y(),
            result.point_b.z(),
        ])
    }

    /// Compute minimum distance from a point to a face.
    ///
    /// Returns `[distance, closest_x, closest_y, closest_z]`.
    ///
    /// # Errors
    ///
    /// Returns an error if the face handle is invalid.
    #[wasm_bindgen(js_name = "pointToFaceDistance")]
    pub fn point_to_face_distance(
        &self,
        px: f64,
        py: f64,
        pz: f64,
        face: u32,
    ) -> Result<Vec<f64>, JsError> {
        validate_finite(px, "px")?;
        validate_finite(py, "py")?;
        validate_finite(pz, "pz")?;
        let face_id = self.resolve_face(face)?;
        let result = remus_operations::distance::point_to_face(
            &self.topo,
            Point3::new(px, py, pz),
            face_id,
        )?;
        Ok(vec![
            result.distance,
            result.point_b.x(),
            result.point_b.y(),
            result.point_b.z(),
        ])
    }

    /// Compute minimum distance from a point to an edge.
    ///
    /// Returns `[distance, closest_x, closest_y, closest_z]`.
    ///
    /// # Errors
    ///
    /// Returns an error if the edge handle is invalid.
    #[wasm_bindgen(js_name = "pointToEdgeDistance")]
    pub fn point_to_edge_distance(
        &self,
        px: f64,
        py: f64,
        pz: f64,
        edge: u32,
    ) -> Result<Vec<f64>, JsError> {
        validate_finite(px, "px")?;
        validate_finite(py, "py")?;
        validate_finite(pz, "pz")?;
        let edge_id = self.resolve_edge(edge)?;
        let result = remus_operations::distance::point_to_edge(
            &self.topo,
            Point3::new(px, py, pz),
            edge_id,
        )?;
        Ok(vec![
            result.distance,
            result.point_b.x(),
            result.point_b.y(),
            result.point_b.z(),
        ])
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use crate::kernel::BrepKernel;

    fn batch_has_error(result: &str, idx: usize) -> bool {
        let parsed: serde_json::Value = serde_json::from_str(result).unwrap();
        parsed[idx]["error"].is_string()
    }

    #[test]
    fn detailed_validation_serializes_counts_and_issue_prose() {
        let report = remus_operations::validate::ValidationReport {
            issues: vec![
                remus_operations::validate::ValidationIssue {
                    severity: remus_operations::validate::Severity::Error,
                    description: "open shell".into(),
                },
                remus_operations::validate::ValidationIssue {
                    severity: remus_operations::validate::Severity::Warning,
                    description: "small face".into(),
                },
            ],
        };

        let result = super::detailed_validation_result(report);
        let json = serde_json::to_value(result).unwrap();
        assert_eq!(json["errorCount"], 1);
        assert_eq!(json["warningCount"], 1);
        assert_eq!(json["issues"][0]["severity"], "error");
        assert_eq!(json["issues"][0]["description"], "open shell");
        assert_eq!(json["issues"][1]["severity"], "warning");
        assert_eq!(json["issues"][1]["description"], "small face");
    }

    #[test]
    fn detailed_validation_of_box_matches_numeric_validator_counts() {
        let mut topo = remus_topology::topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
        let expected_errors = report.error_count();
        let expected_warnings = report.warning_count();

        let result = super::detailed_validation_result(report);
        assert_eq!(result.error_count as usize, expected_errors);
        assert_eq!(result.warning_count as usize, expected_warnings);
        assert_eq!(
            result.issues.len(),
            expected_errors + expected_warnings,
            "detailed validation must not drop diagnostics"
        );
    }

    #[test]
    fn box_inertia_tensor_matches_analytic_result() {
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let inertia = kernel.inertia_tensor(solid).unwrap();
        let expected = [50.0, 0.0, 0.0, 0.0, 40.0, 0.0, 0.0, 0.0, 26.0];

        assert_eq!(inertia.len(), expected.len());
        for (actual, expected) in inertia.iter().zip(expected) {
            assert!(
                (actual - expected).abs() < 1e-9,
                "expected inertia {expected}, got {actual}"
            );
        }
    }

    // ── Volume ─────────────────────────────────────────────────────

    #[test]
    fn box_volume_matches_w_times_h_times_d() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 3, "depth": 4}},
                {"op": "volume", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let v = parsed[1]["ok"].as_f64().unwrap();
        assert!(
            (v - 24.0).abs() < 0.1,
            "2x3x4 box volume should be 24.0, got {v}"
        );
    }

    #[test]
    fn unit_box_volume_is_one() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "volume", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let v = parsed[1]["ok"].as_f64().unwrap();
        assert!(
            (v - 1.0).abs() < 1e-6,
            "unit box volume should be 1.0, got {v}"
        );
    }

    #[test]
    fn volume_invalid_handle_is_error() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(r#"[{"op": "volume", "args": {"solid": 9999}}]"#);
        assert!(batch_has_error(&r, 0));
    }

    // ── Surface area ───────────────────────────────────────────────

    #[test]
    fn box_surface_area_matches_formula() {
        let (w, h, d) = (2.0_f64, 3.0_f64, 4.0_f64);
        let expected = 2.0 * (w * h + w * d + h * d);
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 3, "depth": 4}},
                {"op": "surfaceArea", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let area = parsed[1]["ok"].as_f64().unwrap();
        assert!(
            (area - expected).abs() < 0.1,
            "surface area should be {expected}, got {area}"
        );
    }

    #[test]
    fn surface_area_invalid_handle_is_error() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(r#"[{"op": "surfaceArea", "args": {"solid": 9999}}]"#);
        assert!(batch_has_error(&r, 0));
    }

    // ── Bounding box ───────────────────────────────────────────────

    #[test]
    fn box_bounding_box_corner_at_origin() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 3, "depth": 5}},
                {"op": "boundingBox", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let bb = parsed[1]["ok"].as_array().unwrap();
        assert_eq!(bb.len(), 6, "bounding box must have 6 components");
        let min_x = bb[0].as_f64().unwrap();
        let min_y = bb[1].as_f64().unwrap();
        let min_z = bb[2].as_f64().unwrap();
        let max_x = bb[3].as_f64().unwrap();
        let max_y = bb[4].as_f64().unwrap();
        let max_z = bb[5].as_f64().unwrap();
        assert!(min_x.abs() < 1e-6, "min_x should be ~0, got {min_x}");
        assert!(min_y.abs() < 1e-6, "min_y should be ~0, got {min_y}");
        assert!(min_z.abs() < 1e-6, "min_z should be ~0, got {min_z}");
        assert!(
            (max_x - 2.0).abs() < 1e-6,
            "max_x should be ~2, got {max_x}"
        );
        assert!(
            (max_y - 3.0).abs() < 1e-6,
            "max_y should be ~3, got {max_y}"
        );
        assert!(
            (max_z - 5.0).abs() < 1e-6,
            "max_z should be ~5, got {max_z}"
        );
    }

    #[test]
    fn bounding_box_invalid_handle_is_error() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(r#"[{"op": "boundingBox", "args": {"solid": 9999}}]"#);
        assert!(batch_has_error(&r, 0));
    }

    // ── Center of mass ─────────────────────────────────────────────

    #[test]
    fn box_center_of_mass_is_geometric_center() {
        let (w, h, d) = (4.0_f64, 6.0_f64, 2.0_f64);
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 4, "height": 6, "depth": 2}},
                {"op": "centerOfMass", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let com = parsed[1]["ok"].as_array().unwrap();
        assert_eq!(com.len(), 3);
        let cx = com[0].as_f64().unwrap();
        let cy = com[1].as_f64().unwrap();
        let cz = com[2].as_f64().unwrap();
        assert!(
            (cx - w / 2.0).abs() < 0.1,
            "CoM x should be ~{}, got {cx}",
            w / 2.0
        );
        assert!(
            (cy - h / 2.0).abs() < 0.1,
            "CoM y should be ~{}, got {cy}",
            h / 2.0
        );
        assert!(
            (cz - d / 2.0).abs() < 0.1,
            "CoM z should be ~{}, got {cz}",
            d / 2.0
        );
    }

    // ── Mass properties ────────────────────────────────────────────

    #[test]
    fn box_mass_properties_match_closed_form() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 3, "depth": 4}},
                {"op": "massProperties", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let props = &parsed[1]["ok"];
        let volume = props["volume"].as_f64().unwrap();
        assert!((volume - 24.0).abs() < 1e-9, "volume = {volume}");

        let com = props["centerOfMass"].as_array().unwrap();
        assert!((com[0].as_f64().unwrap() - 1.0).abs() < 1e-9);
        assert!((com[1].as_f64().unwrap() - 1.5).abs() < 1e-9);
        assert!((com[2].as_f64().unwrap() - 2.0).abs() < 1e-9);

        // Box about CoM: Ixx = m/12 (h² + d²), etc.; products vanish.
        let inertia = props["inertia"].as_array().unwrap();
        let m = 24.0;
        let expected = [
            m / 12.0 * (9.0 + 16.0),
            m / 12.0 * (4.0 + 16.0),
            m / 12.0 * (4.0 + 9.0),
        ];
        for (k_idx, exp) in expected.iter().enumerate() {
            let got = inertia[k_idx].as_f64().unwrap();
            assert!(
                (got - exp).abs() < 1e-9,
                "inertia[{k_idx}] = {got}, expected {exp}"
            );
        }
        for k_idx in 3..6 {
            assert!(inertia[k_idx].as_f64().unwrap().abs() < 1e-9);
        }

        // Principal moments of an axis-aligned box are the sorted diagonal.
        let moments = props["principalMoments"].as_array().unwrap();
        let mut sorted = expected;
        sorted.sort_by(f64::total_cmp);
        for (k_idx, exp) in sorted.iter().enumerate() {
            let got = moments[k_idx].as_f64().unwrap();
            assert!((got - exp).abs() < 1e-9, "principal[{k_idx}] = {got}");
        }
        assert_eq!(props["principalAxes"].as_array().unwrap().len(), 9);
    }

    #[test]
    fn cylinder_mass_properties_match_closed_form() {
        // Exercises the curved-surface (patch-tiled trimmed quadrature) and
        // exact circular-cap (Green's theorem) paths end to end.
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeCylinder", "args": {"radius": 3, "height": 10}},
                {"op": "massProperties", "args": {"solid": 0}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let props = &parsed[1]["ok"];
        let m = std::f64::consts::PI * 90.0;
        let volume = props["volume"].as_f64().unwrap();
        assert!(
            ((volume - m) / m).abs() < 1e-8,
            "volume = {volume}, expected {m}"
        );
        let inertia = props["inertia"].as_array().unwrap();
        let izz = m * 9.0 / 2.0;
        let ixx = m * 9.0f64.mul_add(3.0, 100.0) / 12.0;
        assert!(
            ((inertia[0].as_f64().unwrap() - ixx) / ixx).abs() < 1e-8,
            "Ixx = {}, expected {ixx}",
            inertia[0]
        );
        assert!(
            ((inertia[2].as_f64().unwrap() - izz) / izz).abs() < 1e-8,
            "Izz = {}, expected {izz}",
            inertia[2]
        );
    }

    #[test]
    fn mass_properties_invalid_handle_is_error() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(r#"[{"op": "massProperties", "args": {"solid": 9999}}]"#);
        assert!(batch_has_error(&r, 0));
    }

    // ── Mesh quality ───────────────────────────────────────────────

    #[test]
    fn box_mesh_quality_is_watertight() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "meshQuality", "args": {"solid": 0, "deflection": 0.1}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let q = &parsed[1]["ok"];
        assert_eq!(q["boundaryEdges"].as_u64().unwrap(), 0);
        assert_eq!(q["nonManifoldEdges"].as_u64().unwrap(), 0);
        assert_eq!(q["eulerCharacteristic"].as_i64().unwrap(), 2);
        assert!(q["isWatertight"].as_bool().unwrap());
    }

    #[test]
    fn cylinder_mesh_quality_is_watertight_genus0() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeCylinder", "args": {"radius": 3, "height": 10}},
                {"op": "meshQuality", "args": {"solid": 0, "deflection": 0.05}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let q = &parsed[1]["ok"];
        assert!(q["isWatertight"].as_bool().unwrap(), "quality = {q}");
        assert_eq!(q["eulerCharacteristic"].as_i64().unwrap(), 2);
    }

    #[test]
    fn mesh_quality_invalid_handle_is_error() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(r#"[{"op": "meshQuality", "args": {"solid": 9999}}]"#);
        assert!(batch_has_error(&r, 0));
    }

    // ── Edge length ────────────────────────────────────────────────
    // edge_length happy path works because JsError is only constructed
    // on error; use internal operations for edge-level queries.

    #[test]
    fn box_edge_length_via_operations() {
        // Use operations layer directly to avoid JsError on error paths.
        let mut topo = remus_topology::topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 3.0, 3.0, 3.0).unwrap();
        let edges = remus_topology::explorer::solid_edges(&topo, solid).unwrap();
        assert_eq!(edges.len(), 12, "box must have 12 edges");
        for &e in &edges {
            let len = remus_operations::measure::edge_length(&topo, e).unwrap();
            assert!(
                (len - 3.0).abs() < 1e-6,
                "all edges of a 3x3x3 box should have length 3, got {len}"
            );
        }
    }

    #[test]
    fn edge_length_rectangular_box_has_three_distinct_lengths() {
        let mut topo = remus_topology::topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 2.0, 4.0).unwrap();
        let edges = remus_topology::explorer::solid_edges(&topo, solid).unwrap();
        let mut lengths: Vec<f64> = edges
            .iter()
            .map(|&e| remus_operations::measure::edge_length(&topo, e).unwrap())
            .collect();
        lengths.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((lengths[0] - 1.0).abs() < 1e-6);
        assert!((lengths[4] - 2.0).abs() < 1e-6);
        assert!((lengths[8] - 4.0).abs() < 1e-6);
    }

    // ── Classify point ─────────────────────────────────────────────

    #[test]
    fn classify_point_inside_box() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 4, "height": 4, "depth": 4}},
                {"op": "classifyPoint", "args": {"solid": 0, "x": 2, "y": 2, "z": 2}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(parsed[1]["ok"].as_str().unwrap(), "inside");
    }

    #[test]
    fn classify_point_outside_box() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 1, "height": 1, "depth": 1}},
                {"op": "classifyPoint", "args": {"solid": 0, "x": 5, "y": 5, "z": 5}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        assert_eq!(parsed[1]["ok"].as_str().unwrap(), "outside");
    }

    #[test]
    fn classify_point_returns_valid_string() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[
                {"op": "makeBox", "args": {"width": 2, "height": 2, "depth": 2}},
                {"op": "classifyPoint", "args": {"solid": 0, "x": 1, "y": 1, "z": 1}}
            ]"#,
        );
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let result = parsed[1]["ok"].as_str().unwrap();
        assert!(
            matches!(result, "inside" | "outside" | "boundary"),
            "classify_point must return inside/outside/boundary, got {result}"
        );
    }

    #[test]
    fn classify_point_invalid_handle_is_error() {
        let mut k = BrepKernel::new();
        let r = k.execute_batch(
            r#"[{"op": "classifyPoint", "args": {"solid": 9999, "x": 0, "y": 0, "z": 0}}]"#,
        );
        assert!(batch_has_error(&r, 0));
    }

    // ── Face area via operations layer ──────────────────────────────

    #[test]
    fn face_area_of_box_face_is_correct() {
        let mut topo = remus_topology::topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, solid).unwrap();
        assert_eq!(faces.len(), 6);
        let valid_areas = [6.0_f64, 8.0_f64, 12.0_f64];
        for &f in &faces {
            let area = remus_operations::measure::face_area(&topo, f, 0.01).unwrap();
            let ok = valid_areas.iter().any(|&a| (area - a).abs() < 0.05);
            assert!(ok, "face area {area} should be one of {valid_areas:?}");
        }
    }

    // ── Distance ───────────────────────────────────────────────────

    #[test]
    fn point_to_solid_distance_from_outside() {
        let mut topo = remus_topology::topology::Topology::new();
        let solid = remus_operations::primitives::make_box(&mut topo, 1.0, 1.0, 1.0).unwrap();
        let result = remus_operations::distance::point_to_solid_distance(
            &topo,
            remus_math::vec::Point3::new(3.0, 0.5, 0.5),
            solid,
        )
        .unwrap();
        assert!(
            (result.distance - 2.0).abs() < 1e-4,
            "point (3,0.5,0.5) should be 2.0 from box, got {}",
            result.distance
        );
    }
}
