//! File I/O (import/export) bindings.

#![cfg(feature = "io")]
#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use remus_math::diagnostic::{DetailValue, ToDiagnostic};

use crate::error::{WasmError, validate_positive};
use crate::handles::{shell_id_to_u32, solid_id_to_u32, wire_id_to_u32};
use crate::helpers::TOL;
use crate::kernel::BrepKernel;

/// Build [`remus_io::ImportLimits`] from optional JS overrides.
///
/// `max_input_bytes` bounds the encoded input size; `max_entities` bounds
/// format-specific model records (vertices, faces, triangles, STEP entities).
/// Absent values keep the production defaults (128 MiB / 2,000,000).
pub(super) fn import_limits_from(
    max_input_bytes: Option<f64>,
    max_entities: Option<f64>,
) -> Result<remus_io::ImportLimits, WasmError> {
    let mut limits = remus_io::ImportLimits::default();
    if let Some(b) = max_input_bytes {
        validate_positive(b, "maxInputBytes")?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            limits.max_input_bytes = b as usize;
        }
    }
    if let Some(n) = max_entities {
        validate_positive(n, "maxEntities")?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            limits.max_model_entities = n as usize;
        }
    }
    Ok(limits)
}

/// Parse optional JSON metadata for STEP export.
fn step_write_options_from_json(
    options: Option<&str>,
) -> Result<remus_io::step::StepWriteOptions, WasmError> {
    match options {
        None => Ok(remus_io::step::StepWriteOptions::default()),
        Some(json) => serde_json::from_str(json).map_err(|error| WasmError::InvalidInput {
            reason: format!("invalid STEP write options JSON: {error}"),
        }),
    }
}

#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct StepImportResult {
    solids: Vec<u32>,
    diagnostics: Vec<StepImportDiagnostic>,
}

#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct StepImportDiagnostic {
    code: &'static str,
    category: &'static str,
    message: String,
    details: serde_json::Map<String, serde_json::Value>,
}

impl From<&remus_io::step::StepImportDiagnostic> for StepImportDiagnostic {
    fn from(source: &remus_io::step::StepImportDiagnostic) -> Self {
        let diagnostic = source.diagnostic();
        let details = diagnostic
            .details()
            .iter()
            .map(|(key, value)| {
                let value = match value {
                    DetailValue::Int(value) => serde_json::Value::from(*value),
                    DetailValue::Float(value) => serde_json::Value::from(*value),
                    DetailValue::Text(value) => serde_json::Value::from(value.clone()),
                };
                ((*key).to_owned(), value)
            })
            .collect();
        Self {
            code: diagnostic.code(),
            category: diagnostic.category().as_str(),
            message: diagnostic.message().to_owned(),
            details,
        }
    }
}

/// Parse optional JSON bounds for the STEP validation-property checker.
fn step_validation_options_from_json(
    options: Option<&str>,
) -> Result<remus_io::step::StepValidationOptions, WasmError> {
    match options {
        None => Ok(remus_io::step::StepValidationOptions::default()),
        Some(json) => serde_json::from_str(json).map_err(|error| WasmError::InvalidInput {
            reason: format!("invalid STEP validation options JSON: {error}"),
        }),
    }
}

pub(super) fn step_validation_result_json(
    result: remus_io::step::StepReadResult,
) -> serde_json::Value {
    let diagnostics = result
        .diagnostics()
        .iter()
        .map(StepImportDiagnostic::from)
        .collect::<Vec<_>>();
    serde_json::json!({
        "solids": result.solids().iter().copied().map(solid_id_to_u32).collect::<Vec<_>>(),
        "diagnostics": diagnostics,
        "validation": result.validation(),
    })
}

pub(super) fn step_body_result_json(result: &remus_io::step::StepReadResult) -> serde_json::Value {
    serde_json::json!({
        "solids": result.solids().iter().copied().map(solid_id_to_u32).collect::<Vec<_>>(),
        "sheets": result.sheets().iter().copied().map(shell_id_to_u32).collect::<Vec<_>>(),
        "diagnostics": result
            .diagnostics()
            .iter()
            .map(StepImportDiagnostic::from)
            .collect::<Vec<_>>(),
    })
}

#[wasm_bindgen]
impl BrepKernel {
    // ── Export ─────────────────────────────────────────────────────

    /// Export a solid to 3MF format (ZIP archive as bytes).
    ///
    /// Returns a `Uint8Array` in JavaScript containing the `.3mf` file.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or export fails.
    #[wasm_bindgen(js_name = "export3mf")]
    pub fn export_3mf(&self, solid: u32, deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let bytes = remus_io::threemf::write_threemf(&self.topo, &[solid_id], deflection)?;
        Ok(bytes)
    }

    /// Export a solid to binary STL format.
    ///
    /// Returns a `Uint8Array` containing the `.stl` file.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or export fails.
    #[wasm_bindgen(js_name = "exportStl")]
    pub fn export_stl(&self, solid: u32, deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let bytes = remus_io::stl::writer::write_stl(
            &self.topo,
            &[solid_id],
            deflection,
            remus_io::stl::writer::StlFormat::Binary,
        )?;
        Ok(bytes)
    }

    /// Export a solid to STL ASCII format.
    ///
    /// Returns the ASCII STL as UTF-8 bytes.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or export fails.
    #[wasm_bindgen(js_name = "exportStlAscii")]
    pub fn export_stl_ascii(&self, solid: u32, deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let bytes = remus_io::stl::writer::write_stl(
            &self.topo,
            &[solid_id],
            deflection,
            remus_io::stl::writer::StlFormat::Ascii,
        )?;
        Ok(bytes)
    }

    /// Export several solids into one 3MF package.
    ///
    /// The writer already supports multiple objects per package; this is the
    /// multi-solid twin of [`export3mf`](Self::export_3mf), mirroring
    /// [`exportStepMulti`](Self::export_step_multi) so a multi-body model
    /// exports as one file instead of forcing the caller to fuse first.
    ///
    /// # Errors
    ///
    /// Returns an error if `solids` is empty, a handle is invalid, the
    /// deflection is non-positive, or export fails.
    #[wasm_bindgen(js_name = "export3mfMulti")]
    pub fn export_3mf_multi(&self, solids: &[u32], deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_ids = solids
            .iter()
            .map(|&handle| self.resolve_solid(handle))
            .collect::<Result<Vec<_>, _>>()?;
        let bytes = remus_io::threemf::write_threemf(&self.topo, &solid_ids, deflection)?;
        Ok(bytes)
    }

    /// Export several solids into one binary STL file.
    ///
    /// The multi-solid twin of [`exportStl`](Self::export_stl): meshes are
    /// merged into a single facet stream, which is what slicers expect from
    /// a one-part-per-file workflow with multiple bodies.
    ///
    /// # Errors
    ///
    /// Returns an error if `solids` is empty, a handle is invalid, the
    /// deflection is non-positive, or export fails.
    #[wasm_bindgen(js_name = "exportStlMulti")]
    pub fn export_stl_multi(&self, solids: &[u32], deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_ids = solids
            .iter()
            .map(|&handle| self.resolve_solid(handle))
            .collect::<Result<Vec<_>, _>>()?;
        let bytes = remus_io::stl::writer::write_stl(
            &self.topo,
            &solid_ids,
            deflection,
            remus_io::stl::writer::StlFormat::Binary,
        )?;
        Ok(bytes)
    }

    /// Export a solid to OBJ format (UTF-8 string as bytes).
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or tessellation fails.
    #[wasm_bindgen(js_name = "exportObj")]
    pub fn export_obj(&self, solid: u32, deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let obj_str = remus_io::obj::write_obj(&self.topo, &[solid_id], deflection)?;
        Ok(obj_str.into_bytes())
    }

    /// Export a solid to glTF binary (.glb) format.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or tessellation fails.
    #[wasm_bindgen(js_name = "exportGlb")]
    pub fn export_glb(&self, solid: u32, deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let glb = remus_io::gltf::write_glb(&self.topo, &[solid_id], deflection)?;
        Ok(glb)
    }

    /// Export several solids into one OBJ file.
    ///
    /// The multi-solid twin of [`exportObj`](Self::export_obj), completing
    /// the set [`export3mfMulti`](Self::export_3mf_multi) started: every
    /// solid's facets merge into one vertex stream, which is what OBJ
    /// consumers expect from a single-file export.
    ///
    /// # Errors
    ///
    /// Returns an error if `solids` is empty, a handle is invalid, the
    /// deflection is non-positive, or export fails.
    #[wasm_bindgen(js_name = "exportObjMulti")]
    pub fn export_obj_multi(&self, solids: &[u32], deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        if solids.is_empty() {
            return Err(WasmError::InvalidInput {
                reason: "exportObjMulti requires at least one solid".to_string(),
            }
            .into());
        }
        let solid_ids = solids
            .iter()
            .map(|&handle| self.resolve_solid(handle))
            .collect::<Result<Vec<_>, _>>()?;
        let obj_str = remus_io::obj::write_obj(&self.topo, &solid_ids, deflection)?;
        Ok(obj_str.into_bytes())
    }

    /// Export several solids into one glTF binary (.glb) file.
    ///
    /// The multi-solid twin of [`exportGlb`](Self::export_glb): every
    /// solid's facets merge into one mesh in a single GLB.
    ///
    /// # Errors
    ///
    /// Returns an error if `solids` is empty, a handle is invalid, the
    /// deflection is non-positive, or export fails.
    #[wasm_bindgen(js_name = "exportGlbMulti")]
    pub fn export_glb_multi(&self, solids: &[u32], deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        if solids.is_empty() {
            return Err(WasmError::InvalidInput {
                reason: "exportGlbMulti requires at least one solid".to_string(),
            }
            .into());
        }
        let solid_ids = solids
            .iter()
            .map(|&handle| self.resolve_solid(handle))
            .collect::<Result<Vec<_>, _>>()?;
        let glb = remus_io::gltf::write_glb(&self.topo, &solid_ids, deflection)?;
        Ok(glb)
    }

    /// Export a solid to PLY format (binary little-endian).
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or tessellation fails.
    #[wasm_bindgen(js_name = "exportPly")]
    pub fn export_ply(&self, solid: u32, deflection: f64) -> Result<Vec<u8>, JsError> {
        validate_positive(deflection, "deflection")?;
        let solid_id = self.resolve_solid(solid)?;
        let ply = remus_io::ply::write_ply(
            &self.topo,
            &[solid_id],
            deflection,
            remus_io::ply::writer::PlyFormat::BinaryLittleEndian,
        )?;
        Ok(ply)
    }

    // ── Import ──────────────────────────────────────────────────────

    /// Import an OBJ file and return a solid handle.
    ///
    /// `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
    /// resource budgets below the production defaults (128 MiB / 2,000,000
    /// model entities); exceeding a budget returns an error before large
    /// allocations.
    ///
    /// # Errors
    ///
    /// Returns an error if the file is malformed, exceeds a resource limit,
    /// or mesh import fails.
    #[wasm_bindgen(js_name = "importObj")]
    pub fn import_obj(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<u32, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = std::str::from_utf8(data).map_err(|e| WasmError::InvalidInput {
            reason: format!("OBJ must be valid UTF-8: {e}"),
        })?;
        let mesh = remus_io::obj::read_obj_with_limits(text, limits)?;
        let solid_id = remus_io::stl::import::import_mesh(self.topo_mut(), &mesh, 1e-7)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(solid_id.index() as u32)
    }

    /// Import a GLB (glTF binary) file and return a solid handle.
    ///
    /// `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
    /// resource budgets (see [`import_obj`](Self::import_obj)).
    ///
    /// # Errors
    ///
    /// Returns an error if the file is malformed, exceeds a resource limit,
    /// or mesh import fails.
    #[wasm_bindgen(js_name = "importGlb")]
    pub fn import_glb(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<u32, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let mesh = remus_io::gltf::read_glb_with_limits(data, limits)?;
        let solid_id = remus_io::stl::import::import_mesh(self.topo_mut(), &mesh, 1e-7)?;
        #[allow(clippy::cast_possible_truncation)]
        Ok(solid_id.index() as u32)
    }

    /// Import an STL file (binary or ASCII) and return a solid handle.
    ///
    /// The mesh triangles are converted to planar B-Rep faces with
    /// vertex merging. `maxInputBytes` / `maxEntities` optionally tighten
    /// the hostile-input resource budgets (see
    /// [`import_obj`](Self::import_obj)).
    ///
    /// # Errors
    ///
    /// Returns an error if the STL data is malformed, exceeds a resource
    /// limit, or is empty.
    #[wasm_bindgen(js_name = "importStl")]
    pub fn import_stl(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<u32, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let mesh = remus_io::stl::reader::read_stl_with_limits(data, limits)?;
        let solid_id = remus_io::stl::import::import_mesh(self.topo_mut(), &mesh, TOL)?;
        Ok(solid_id_to_u32(solid_id))
    }

    /// Import a PLY file (ASCII or binary little-endian) and return a solid handle.
    ///
    /// Polygon faces are triangulated by the PLY reader, then converted to
    /// planar B-Rep faces with vertex merging. `maxInputBytes` /
    /// `maxEntities` optionally tighten the hostile-input resource budgets
    /// (see [`import_obj`](Self::import_obj)).
    ///
    /// # Errors
    ///
    /// Returns an error if the PLY data is malformed, empty, exceeds a
    /// resource limit, or cannot form a mesh-backed solid.
    #[wasm_bindgen(js_name = "importPly")]
    pub fn import_ply(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<u32, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let mesh = remus_io::ply::read_ply_with_limits(data, limits)?;
        let solid_id = remus_io::stl::import::import_mesh(self.topo_mut(), &mesh, TOL)?;
        Ok(solid_id_to_u32(solid_id))
    }

    /// Import a 3MF file and return solid handles.
    ///
    /// Returns handles for each object found in the 3MF archive.
    /// `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
    /// resource budgets (see [`import_obj`](Self::import_obj)).
    ///
    /// # Errors
    ///
    /// Returns an error if the 3MF data is malformed or exceeds a resource
    /// limit.
    #[wasm_bindgen(js_name = "import3mf")]
    pub fn import_3mf(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u32>, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let meshes = remus_io::threemf::reader::read_threemf_with_limits(data, limits)?;
        let mut handles = Vec::new();
        for mesh in &meshes {
            let solid_id = remus_io::stl::import::import_mesh(self.topo_mut(), mesh, TOL)?;
            handles.push(solid_id_to_u32(solid_id));
        }
        Ok(handles)
    }

    /// Import a triangle mesh from flat vertex/index arrays.
    ///
    /// `positions` is a flat `[x0,y0,z0, x1,y1,z1, ...]` array.
    /// `indices` is a flat `[i0,i1,i2, i3,i4,i5, ...]` array of triangle
    /// vertex indices. Returns a solid handle.
    ///
    /// # Errors
    ///
    /// Returns an error if the arrays are malformed or mesh import fails.
    #[wasm_bindgen(js_name = "importIndexedMesh")]
    pub fn import_indexed_mesh(
        &mut self,
        positions: &[f64],
        indices: &[u32],
    ) -> Result<u32, JsError> {
        use remus_math::vec::Point3;

        if !positions.len().is_multiple_of(3) {
            return Err(WasmError::InvalidInput {
                reason: format!(
                    "positions length {} is not a multiple of 3",
                    positions.len()
                ),
            }
            .into());
        }
        if !indices.len().is_multiple_of(3) {
            return Err(WasmError::InvalidInput {
                reason: format!("indices length {} is not a multiple of 3", indices.len()),
            }
            .into());
        }

        let verts: Vec<Point3> = positions
            .chunks_exact(3)
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
        let normals = Vec::new();

        let mesh = remus_operations::tessellate::TriangleMesh {
            positions: verts,
            normals,
            indices: indices.to_vec(),
        };

        let solid_id = remus_io::stl::import::import_mesh(self.topo_mut(), &mesh, TOL)?;
        Ok(solid_id_to_u32(solid_id))
    }

    /// Export a solid to STEP AP203 format.
    ///
    /// Returns the STEP file as a UTF-8 encoded byte vector.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or export fails.
    #[wasm_bindgen(js_name = "exportStep")]
    pub fn export_step(&self, solid: u32) -> Result<Vec<u8>, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let step_str = remus_io::step::writer::write_step(&self.topo, &[solid_id])?;
        Ok(step_str.into_bytes())
    }

    /// Export a solid to STEP AP203 with optional header metadata.
    ///
    /// `options` is an optional JSON string with `productName`, `fileName`,
    /// `timestamp`, and `validationProperties` fields. Missing fields retain
    /// the defaults used by [`exportStep`](Self::export_step).
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid, the options JSON is
    /// malformed, or export fails.
    #[wasm_bindgen(js_name = "exportStepWithOptions")]
    pub fn export_step_with_options(
        &self,
        solid: u32,
        options: Option<String>,
    ) -> Result<Vec<u8>, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let options = step_write_options_from_json(options.as_deref())?;
        let step =
            remus_io::step::writer::write_step_with_options(&self.topo, &[solid_id], &options)?;
        Ok(step.into_bytes())
    }

    /// Export several solids into one STEP AP203 file.
    ///
    /// The solids stay distinct in the output — they become separate
    /// `MANIFOLD_SOLID_BREP` items of a single
    /// `ADVANCED_BREP_SHAPE_REPRESENTATION`, so a reader recovers exactly the
    /// bodies that went in. `solids` is a JS `Uint32Array` or array of solid
    /// handles.
    ///
    /// # Errors
    ///
    /// Returns an error if `solids` is empty, if any handle is invalid, or if
    /// export fails.
    #[wasm_bindgen(js_name = "exportStepMulti")]
    pub fn export_step_multi(&self, solids: &[u32]) -> Result<Vec<u8>, JsError> {
        let solid_ids = solids
            .iter()
            .map(|&handle| self.resolve_solid(handle))
            .collect::<Result<Vec<_>, _>>()?;
        let step_str = remus_io::step::writer::write_step(&self.topo, &solid_ids)?;
        Ok(step_str.into_bytes())
    }

    /// Export several solids into one STEP AP203 file with optional metadata.
    ///
    /// The JSON shape and defaults match
    /// [`exportStepWithOptions`](Self::export_step_with_options).
    ///
    /// # Errors
    ///
    /// Returns an error if `solids` is empty, a handle is invalid, the options
    /// JSON is malformed, or export fails.
    #[wasm_bindgen(js_name = "exportStepMultiWithOptions")]
    pub fn export_step_multi_with_options(
        &self,
        solids: &[u32],
        options: Option<String>,
    ) -> Result<Vec<u8>, JsError> {
        let solid_ids = solids
            .iter()
            .map(|&handle| self.resolve_solid(handle))
            .collect::<Result<Vec<_>, _>>()?;
        let options = step_write_options_from_json(options.as_deref())?;
        let step =
            remus_io::step::writer::write_step_with_options(&self.topo, &solid_ids, &options)?;
        Ok(step.into_bytes())
    }

    /// Export one first-class sheet body as a STEP shell-based surface model.
    ///
    /// `options` accepts the same optional metadata JSON as
    /// [`exportStepWithOptions`](Self::export_step_with_options). Validation
    /// properties are refused because their current contract is solid-only.
    ///
    /// # Errors
    ///
    /// Returns an error if the handle is invalid, does not name a sheet body,
    /// the options JSON is malformed, or export fails.
    #[wasm_bindgen(js_name = "exportStepSheet")]
    pub fn export_step_sheet(
        &self,
        sheet: u32,
        options: Option<String>,
    ) -> Result<Vec<u8>, JsError> {
        let sheet_id = self.resolve_shell(sheet)?;
        let options = step_write_options_from_json(options.as_deref())?;
        let step =
            remus_io::step::write_step_bodies_with_options(&self.topo, &[], &[sheet_id], &options)?;
        Ok(step.into_bytes())
    }

    /// Import a STEP file and return solid handles.
    ///
    /// Returns handles for each solid found in the STEP file.
    /// `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
    /// resource budgets (see [`import_obj`](Self::import_obj)).
    ///
    /// # Errors
    ///
    /// Returns an error if the STEP data is malformed or exceeds a resource
    /// limit.
    #[wasm_bindgen(js_name = "importStep")]
    pub fn import_step(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u32>, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = std::str::from_utf8(data)
            .map_err(|e| JsError::new(&format!("STEP data is not valid UTF-8: {e}")))?;
        let solid_ids =
            remus_io::step::reader::read_step_with_limits(text, self.topo_mut(), limits)?;
        Ok(solid_ids.iter().map(|id| solid_id_to_u32(*id)).collect())
    }

    /// Import every supported STEP body root.
    ///
    /// Returns JSON with distinct `solids`, `sheets`, and bounded-healing
    /// `diagnostics` arrays so overlapping arena indices cannot blur handle
    /// classes.
    ///
    /// # Errors
    ///
    /// Returns an error if the STEP data is malformed or exceeds a resource
    /// limit.
    #[wasm_bindgen(js_name = "importStepBodies")]
    pub fn import_step_bodies(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<String, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = std::str::from_utf8(data)
            .map_err(|error| JsError::new(&format!("STEP data is not valid UTF-8: {error}")))?;
        let result = remus_io::step::read_step_bodies_with_limits(text, self.topo_mut(), limits)?;
        Ok(serde_json::to_string(&step_body_result_json(&result))?)
    }

    /// Import a STEP file and return solid handles plus bounded-healing diagnostics.
    ///
    /// The returned JavaScript string is JSON with shape
    /// `{ solids, diagnostics }`. Existing callers that only need handles can
    /// continue to use [`importStep`](Self::import_step).
    ///
    /// # Errors
    ///
    /// Returns an error if the STEP data is malformed or exceeds a resource
    /// limit.
    #[wasm_bindgen(js_name = "importStepWithReport")]
    pub fn import_step_with_report(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<JsValue, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = std::str::from_utf8(data)
            .map_err(|error| JsError::new(&format!("STEP data is not valid UTF-8: {error}")))?;
        let result =
            remus_io::step::read_step_with_limits_and_report(text, self.topo_mut(), limits)?;
        let result = StepImportResult {
            solids: result
                .solids()
                .iter()
                .map(|solid| solid_id_to_u32(*solid))
                .collect(),
            diagnostics: result
                .diagnostics()
                .iter()
                .map(StepImportDiagnostic::from)
                .collect(),
        };
        Ok(serde_json::to_string(&result)
            .map_err(|error| JsError::new(&error.to_string()))?
            .into())
    }

    /// Import STEP and opt in to CAx-IF geometric validation-property checks.
    ///
    /// Returns JSON with `solids`, bounded-healing `diagnostics`, and one
    /// `validation` report per solid.
    /// Deviations are diagnostics and do not discard valid imported geometry;
    /// malformed declarations fail transactionally. `options` accepts the
    /// camelCase fields of [`remus_io::step::StepValidationOptions`].
    ///
    /// # Errors
    ///
    /// Returns an error for malformed UTF-8, STEP, option JSON, validation
    /// declarations, or hostile-input limit violations.
    #[wasm_bindgen(js_name = "importStepWithValidation")]
    pub fn import_step_with_validation(
        &mut self,
        data: &[u8],
        options: Option<String>,
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<String, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let options = step_validation_options_from_json(options.as_deref())?;
        let text = std::str::from_utf8(data)
            .map_err(|e| JsError::new(&format!("STEP data is not valid UTF-8: {e}")))?;
        let result =
            remus_io::step::read_step_with_validation(text, self.topo_mut(), limits, options)?;
        Ok(serde_json::to_string(&step_validation_result_json(result))?)
    }

    // ── IGES Import/Export ────────────────────────────────────────

    /// Export a solid to IGES format.
    ///
    /// Returns the IGES file as a UTF-8 encoded byte vector.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or export fails.
    #[wasm_bindgen(js_name = "exportIges")]
    pub fn export_iges(&self, solid: u32) -> Result<Vec<u8>, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let iges_str = remus_io::iges::writer::write_iges(&self.topo, &[solid_id])?;
        Ok(iges_str.into_bytes())
    }

    /// Import an IGES file and return solid handles.
    ///
    /// `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
    /// resource budgets (see [`import_obj`](Self::import_obj)).
    ///
    /// # Errors
    ///
    /// Returns an error if the IGES data is malformed or exceeds a resource
    /// limit.
    #[wasm_bindgen(js_name = "importIges")]
    pub fn import_iges(
        &mut self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u32>, JsError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = std::str::from_utf8(data)
            .map_err(|e| JsError::new(&format!("IGES data is not valid UTF-8: {e}")))?;
        let solid_ids =
            remus_io::iges::reader::read_iges_with_limits(text, self.topo_mut(), limits)?;
        Ok(solid_ids.iter().map(|id| solid_id_to_u32(*id)).collect())
    }

    // ── Arena debug serialization ─────────────────────────────────

    /// Serialize several solids into one version 3 arena document.
    ///
    /// Shared topology is encoded once with dense local indices. Input order
    /// and duplicate handles are preserved as document roots. This format
    /// intentionally excludes unrelated kernel session state.
    ///
    /// # Errors
    ///
    /// Returns an error if any solid handle is invalid or serialization fails.
    #[wasm_bindgen(js_name = "serializeSolids")]
    pub fn serialize_solids(&self, solids: &[u32]) -> Result<Vec<u8>, JsError> {
        let solid_ids = solids
            .iter()
            .map(|&handle| self.resolve_solid(handle))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(remus_io::arena_io::serialize_solids(
            &self.topo, &solid_ids,
        )?)
    }

    /// Reconstruct solid roots from a version 1 through 5 arena document.
    ///
    /// Every restored entity receives a fresh kernel handle. Documents with
    /// Sheet, wire, and compound roots must be loaded through their dedicated
    /// binding or the native Rust document API.
    ///
    /// # Errors
    ///
    /// Returns an error if the buffer is malformed, exceeds import limits,
    /// contains a non-solid root, or reconstruction fails.
    #[wasm_bindgen(js_name = "deserializeSolids")]
    pub fn deserialize_solids(&mut self, data: &[u8]) -> Result<Vec<u32>, JsError> {
        let solid_ids = remus_io::arena_io::deserialize_solids(data, self.topo_mut())?;
        Ok(solid_ids.into_iter().map(solid_id_to_u32).collect())
    }

    /// Serialize a solid's complete in-memory topology sub-arena to bytes.
    ///
    /// Captures every vertex, edge, wire, face, shell reachable from the
    /// solid with byte-exact f64 values (no geometry re-derivation or
    /// tolerance normalization). Unlike STEP/IGES export, this preserves the
    /// kernel's exact in-memory state — intended for capturing live operands
    /// and replaying them in a native Rust harness to reproduce
    /// sub-ULP-sensitive boolean behavior.
    ///
    /// This writer emits a single-root version 3 document. Returns a
    /// `Uint8Array` consumable by
    /// `remus_io::arena_io::deserialize_solid`.
    ///
    /// # Errors
    ///
    /// Returns an error if the solid handle is invalid or serialization fails.
    #[wasm_bindgen(js_name = "serializeSolid")]
    pub fn serialize_solid(&self, solid: u32) -> Result<Vec<u8>, JsError> {
        let solid_id = self.resolve_solid(solid)?;
        let bytes = remus_io::arena_io::serialize_solid(&self.topo, solid_id)?;
        Ok(bytes)
    }

    /// Reconstruct one solid from a version 1 through 5 single-root buffer.
    ///
    /// # Errors
    ///
    /// Returns an error if the buffer is malformed or reconstruction fails.
    #[wasm_bindgen(js_name = "deserializeSolid")]
    pub fn deserialize_solid(&mut self, data: &[u8]) -> Result<u32, JsError> {
        let solid_id = remus_io::arena_io::deserialize_solid(data, self.topo_mut())?;
        Ok(solid_id_to_u32(solid_id))
    }

    /// Serialize several first-class sheet bodies into a version 4 arena document.
    ///
    /// Shared topology is encoded once with dense local indices. Input order
    /// and duplicate handles are preserved as document roots.
    ///
    /// # Errors
    ///
    /// Returns an error if any shell handle is invalid, is not tagged as a
    /// sheet, or serialization fails.
    #[wasm_bindgen(js_name = "serializeSheets")]
    pub fn serialize_sheets(&self, sheets: &[u32]) -> Result<Vec<u8>, JsError> {
        let sheet_ids = sheets
            .iter()
            .map(|&handle| self.resolve_shell(handle))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(remus_io::arena_io::serialize_sheets(
            &self.topo, &sheet_ids,
        )?)
    }

    /// Reconstruct standalone sheet roots from a version 4 or 5 arena document.
    ///
    /// # Errors
    ///
    /// Returns an error if the buffer is malformed, contains another root
    /// class, or reconstruction fails.
    #[wasm_bindgen(js_name = "deserializeSheets")]
    pub fn deserialize_sheets(&mut self, data: &[u8]) -> Result<Vec<u32>, JsError> {
        let sheet_ids = remus_io::arena_io::deserialize_sheets(data, self.topo_mut())?;
        Ok(sheet_ids.into_iter().map(shell_id_to_u32).collect())
    }

    /// Serialize one first-class sheet body into a version 4 arena document.
    ///
    /// # Errors
    ///
    /// Returns an error if the shell handle is invalid, is not tagged as a
    /// sheet, or serialization fails.
    #[wasm_bindgen(js_name = "serializeSheet")]
    pub fn serialize_sheet(&self, sheet: u32) -> Result<Vec<u8>, JsError> {
        let sheet_id = self.resolve_shell(sheet)?;
        Ok(remus_io::arena_io::serialize_sheet(&self.topo, sheet_id)?)
    }

    /// Reconstruct one first-class sheet body from a version 4 or 5 arena document.
    ///
    /// # Errors
    ///
    /// Returns an error if the buffer is malformed, does not contain exactly
    /// one sheet root, or reconstruction fails.
    #[wasm_bindgen(js_name = "deserializeSheet")]
    pub fn deserialize_sheet(&mut self, data: &[u8]) -> Result<u32, JsError> {
        let sheet_id = remus_io::arena_io::deserialize_sheet(data, self.topo_mut())?;
        Ok(shell_id_to_u32(sheet_id))
    }

    /// Serialize first-class wire bodies into a version 5 arena document.
    ///
    /// Shared topology is encoded once. Root order and duplicate handles are
    /// preserved.
    ///
    /// # Errors
    ///
    /// Returns an error if any handle is invalid, is not tagged as a wire
    /// body, or serialization fails.
    #[wasm_bindgen(js_name = "serializeWires")]
    pub fn serialize_wires(&self, wires: &[u32]) -> Result<Vec<u8>, JsError> {
        let wire_ids = wires
            .iter()
            .map(|&handle| self.resolve_wire(handle))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(remus_io::arena_io::serialize_wires(&self.topo, &wire_ids)?)
    }

    /// Reconstruct standalone wire roots from a version 5 arena document.
    ///
    /// # Errors
    ///
    /// Returns an error if the buffer is malformed, contains non-wire roots,
    /// or reconstruction fails.
    #[wasm_bindgen(js_name = "deserializeWires")]
    pub fn deserialize_wires(&mut self, data: &[u8]) -> Result<Vec<u32>, JsError> {
        let wire_ids = remus_io::arena_io::deserialize_wires(data, self.topo_mut())?;
        Ok(wire_ids.into_iter().map(wire_id_to_u32).collect())
    }

    /// Serialize one first-class wire body into a version 5 arena document.
    ///
    /// # Errors
    ///
    /// Returns an error if the handle is invalid, is not tagged as a wire
    /// body, or serialization fails.
    #[wasm_bindgen(js_name = "serializeWire")]
    pub fn serialize_wire(&self, wire: u32) -> Result<Vec<u8>, JsError> {
        let wire_id = self.resolve_wire(wire)?;
        Ok(remus_io::arena_io::serialize_wire(&self.topo, wire_id)?)
    }

    /// Reconstruct one first-class wire body from a version 5 arena document.
    ///
    /// # Errors
    ///
    /// Returns an error if the buffer is malformed, does not contain exactly
    /// one wire root, or reconstruction fails.
    #[wasm_bindgen(js_name = "deserializeWire")]
    pub fn deserialize_wire(&mut self, data: &[u8]) -> Result<u32, JsError> {
        let wire_id = remus_io::arena_io::deserialize_wire(data, self.topo_mut())?;
        Ok(wire_id_to_u32(wire_id))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn step_options_json_is_optional_and_supports_partial_overrides() {
        assert_eq!(
            step_write_options_from_json(None).unwrap(),
            remus_io::step::StepWriteOptions::default()
        );
        let options = step_write_options_from_json(Some(
            r#"{"productName":"Custom part","fileName":"part.step"}"#,
        ))
        .unwrap();
        assert_eq!(options.product_name, "Custom part");
        assert_eq!(options.file_name, "part.step");
        assert_eq!(options.timestamp, "2024-01-01T00:00:00");
        assert!(!options.validation_properties);
    }

    #[test]
    fn step_validation_options_json_is_optional_and_supports_partial_overrides() {
        assert_eq!(
            step_validation_options_from_json(None).unwrap(),
            remus_io::step::StepValidationOptions::default()
        );
        let options = step_validation_options_from_json(Some(
            r#"{"volumeRelativeTolerance":0.001,"centroidAbsoluteTolerance":0.01}"#,
        ))
        .unwrap();
        assert!((options.volume_relative - 0.001).abs() < f64::EPSILON);
        assert!((options.surface_area_relative - 0.005).abs() < f64::EPSILON);
        assert!((options.centroid_absolute - 0.01).abs() < f64::EPSILON);
    }

    #[test]
    fn direct_step_validation_binding_returns_typed_report() {
        let mut source = BrepKernel::new();
        let solid = source.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let step = source
            .export_step_with_options(solid, Some(r#"{"validationProperties":true}"#.to_string()))
            .unwrap();
        assert!(String::from_utf8_lossy(&step).contains("geometric validation property"));

        let mut imported = BrepKernel::new();
        let json = imported
            .import_step_with_validation(&step, None, None, None)
            .unwrap();
        let result: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(result["solids"].as_array().unwrap().len(), 1);
        assert!(result["diagnostics"].as_array().unwrap().is_empty());
        assert!(
            (result["validation"][0]["declared"]["volume"]
                .as_f64()
                .unwrap()
                - 24.0)
                .abs()
                < 1e-9
        );
        assert!(
            (result["validation"][0]["recomputed"]["surfaceArea"]
                .as_f64()
                .unwrap()
                - 52.0)
                .abs()
                < 1e-9
        );
        assert!(
            result["validation"][0]["diagnostics"]
                .as_array()
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn step_options_binding_writes_custom_metadata() {
        let mut kernel = BrepKernel::new();
        let solid = kernel.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let bytes = kernel
            .export_step_with_options(
                solid,
                Some(
                    r#"{"productName":"Custom part","fileName":"part.step","timestamp":"2026-08-03T00:00:00"}"#
                        .to_string(),
                ),
            )
            .unwrap();
        let step = String::from_utf8(bytes).unwrap();
        assert!(step.contains("PRODUCT('Custom part', 'Custom part'"));
        assert!(step.contains("FILE_NAME('part.step', '2026-08-03T00:00:00'"));
    }

    #[test]
    fn multi_body_3mf_packages_every_solid() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let b = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let bytes = kernel.export_3mf_multi(&[a, b], 0.1).unwrap();
        // A 3MF is a zip package (PK magic), and the two-body export must be
        // strictly larger than either single-body export.
        assert_eq!(&bytes[0..2], b"PK");
        let single = kernel.export_3mf(a, 0.1).unwrap();
        assert!(bytes.len() > single.len());
    }

    #[test]
    fn multi_body_binary_stl_merges_facets() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let b = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let merged = kernel.export_stl_multi(&[a, b], 0.1).unwrap();
        let single = kernel.export_stl(a, 0.1).unwrap();
        // Binary STL: bytes 80..84 hold the little-endian facet count. Two
        // boxes carry exactly twice one box's facets.
        let count = |bytes: &[u8]| u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]);
        assert_eq!(count(&merged), 2 * count(&single));
    }

    #[test]
    fn multi_body_obj_merges_vertex_streams() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let b = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let merged = String::from_utf8(kernel.export_obj_multi(&[a, b], 0.1).unwrap()).unwrap();
        let single = String::from_utf8(kernel.export_obj(a, 0.1).unwrap()).unwrap();
        // Two boxes carry exactly twice one box's vertex lines.
        let vertex_lines = |obj: &str| obj.lines().filter(|line| line.starts_with("v ")).count();
        assert_eq!(vertex_lines(&merged), 2 * vertex_lines(&single));
    }

    #[test]
    fn multi_body_glb_is_one_valid_container() {
        let mut kernel = BrepKernel::new();
        let a = kernel.make_box_solid(1.0, 1.0, 1.0).unwrap();
        let b = kernel.make_box_solid(2.0, 2.0, 2.0).unwrap();
        let merged = kernel.export_glb_multi(&[a, b], 0.1).unwrap();
        // GLB header: magic "glTF", then version 2, then the total length.
        assert_eq!(&merged[0..4], b"glTF");
        assert_eq!(
            u32::from_le_bytes([merged[4], merged[5], merged[6], merged[7]]),
            2
        );
        assert_eq!(
            u32::from_le_bytes([merged[8], merged[9], merged[10], merged[11]]) as usize,
            merged.len()
        );
        let single = kernel.export_glb(a, 0.1).unwrap();
        assert!(merged.len() > single.len());
    }

    #[test]
    fn imports_ascii_ply_tetrahedron() {
        let ply = b"ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nelement face 4\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n3 0 2 1\n3 0 1 3\n3 1 2 3\n3 2 0 3\n";
        let mut kernel = BrepKernel::new();

        let solid = kernel.import_ply(ply, None, None).unwrap();
        let solid_id = kernel.resolve_solid(solid).unwrap();
        let faces = remus_topology::explorer::solid_faces(kernel.topo(), solid_id).unwrap();

        assert_eq!(faces.len(), 4);
    }

    #[test]
    fn ply_round_trip_preserves_mesh() {
        // Happy-path direct calls are safe under native `cargo test` because
        // `JsError` is only constructed on error paths.
        let mut k = BrepKernel::new();
        let solid = {
            let r = k.execute_batch(
                r#"[{"op": "makeBox", "args": {"width": 2, "height": 3, "depth": 4}}]"#,
            );
            let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
            u32::try_from(parsed[0]["ok"].as_u64().unwrap()).unwrap()
        };

        let ply = k.export_ply(solid, 0.1).unwrap();
        assert!(!ply.is_empty(), "PLY export produced no bytes");

        let imported = k.import_ply(&ply, None, None).unwrap();
        let r = k.execute_batch(&format!(
            r#"[{{"op": "volume", "args": {{"solid": {imported}}}}}]"#
        ));
        let parsed: serde_json::Value = serde_json::from_str(&r).unwrap();
        let vol = parsed[0]["ok"].as_f64().unwrap();
        assert!(
            (vol - 24.0).abs() < 0.1,
            "round-tripped box volume should be ~24, got {vol}"
        );
    }

    #[test]
    fn import_limit_violation_reports_limit_error() {
        // Exceeding maxInputBytes must fail before parsing. Route through the
        // io crate's typed error (constructing a JsError would panic on the
        // native test target).
        let limits = remus_io::ImportLimits {
            max_input_bytes: 4,
            ..Default::default()
        };
        let data = b"solid tiny\nendsolid tiny\n";
        let result = remus_io::stl::reader::read_stl_with_limits(data, limits);
        assert!(
            matches!(result, Err(remus_io::IoError::LimitExceeded { .. })),
            "expected LimitExceeded, got {result:?}"
        );
    }

    #[test]
    fn multi_solid_arena_roundtrip_preserves_order_and_fresh_handles() {
        let mut source = BrepKernel::new();
        let first = source.make_box_solid(1.0, 2.0, 3.0).unwrap();
        let second = source.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let bytes = source.serialize_solids(&[second, first, second]).unwrap();

        let mut destination = BrepKernel::new();
        let sentinel = destination.make_box_solid(0.5, 0.5, 0.5).unwrap();
        let restored = destination.deserialize_solids(&bytes).unwrap();

        assert_eq!(restored.len(), 3);
        assert_eq!(restored[0], restored[2]);
        assert_ne!(restored[0], restored[1]);
        assert!(restored[0] > sentinel);
        assert!((destination.volume(restored[0], 0.1).unwrap() - 24.0).abs() < 1e-9);
        assert!((destination.volume(restored[1], 0.1).unwrap() - 6.0).abs() < 1e-9);
    }

    #[test]
    fn sheet_arena_bindings_preserve_root_order_class_and_fresh_handles() {
        let mut source = BrepKernel::new();
        let face = remus_topology::builder::make_rectangle_face(source.topo_mut(), 1.5, 0.5, 1e-7)
            .unwrap();
        let sheet_id = remus_operations::sew::make_sheet_body(source.topo_mut(), &[face]).unwrap();
        let sheet = shell_id_to_u32(sheet_id);
        let bytes = source.serialize_sheets(&[sheet, sheet]).unwrap();

        let mut destination = BrepKernel::new();
        let sentinel = destination
            .topo_mut()
            .add_shell(remus_topology::shell::Shell::empty());
        let restored = destination.deserialize_sheets(&bytes).unwrap();

        assert_eq!(restored.len(), 2);
        assert_eq!(restored[0], restored[1]);
        assert!(restored[0] > shell_id_to_u32(sentinel));
        assert_eq!(
            destination
                .topo()
                .shell(destination.resolve_shell(restored[0]).unwrap())
                .unwrap()
                .body_class(),
            remus_topology::BodyClass::Sheet
        );

        let single = source.serialize_sheet(sheet).unwrap();
        let mut single_destination = BrepKernel::new();
        let restored_single = single_destination.deserialize_sheet(&single).unwrap();
        assert_eq!(
            single_destination
                .topo()
                .shell(single_destination.resolve_shell(restored_single).unwrap())
                .unwrap()
                .body_class(),
            remus_topology::BodyClass::Sheet
        );
    }

    #[test]
    fn wire_arena_bindings_preserve_root_order_class_and_fresh_handles() {
        let mut source = BrepKernel::new();
        let wire_id = remus_topology::builder::make_polygon_wire(
            source.topo_mut(),
            &[
                remus_math::vec::Point3::new(0.0, 0.0, 0.0),
                remus_math::vec::Point3::new(2.0, 0.0, 0.0),
                remus_math::vec::Point3::new(2.0, 1.0, 0.0),
                remus_math::vec::Point3::new(0.0, 1.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let wire = wire_id_to_u32(wire_id);
        let bytes = source.serialize_wires(&[wire, wire]).unwrap();

        let mut destination = BrepKernel::new();
        let sentinel = remus_topology::builder::make_regular_polygon_wire(
            destination.topo_mut(),
            0.25,
            3,
            1e-7,
        )
        .unwrap();
        let restored = destination.deserialize_wires(&bytes).unwrap();

        assert_eq!(restored.len(), 2);
        assert_eq!(restored[0], restored[1]);
        assert!(restored[0] > wire_id_to_u32(sentinel));
        assert_eq!(
            destination
                .topo()
                .wire(destination.resolve_wire(restored[0]).unwrap())
                .unwrap()
                .body_class(),
            remus_topology::BodyClass::Wire
        );
        assert!((destination.wire_length(restored[0]).unwrap() - 6.0).abs() < 1e-9);

        let single = source.serialize_wire(wire).unwrap();
        let mut single_destination = BrepKernel::new();
        let restored_single = single_destination.deserialize_wire(&single).unwrap();
        assert_eq!(
            single_destination
                .topo()
                .wire(single_destination.resolve_wire(restored_single).unwrap())
                .unwrap()
                .body_class(),
            remus_topology::BodyClass::Wire
        );
    }

    #[test]
    fn sheet_step_bindings_preserve_distinct_body_class_and_open_boundary() {
        let mut source = BrepKernel::new();
        let face = remus_topology::builder::make_rectangle_face(source.topo_mut(), 2.0, 1.0, 1e-7)
            .unwrap();
        let surface = remus_math::nurbs::NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                vec![
                    remus_math::vec::Point3::new(-1.0, -0.5, 0.0),
                    remus_math::vec::Point3::new(1.0, -0.5, 0.0),
                ],
                vec![
                    remus_math::vec::Point3::new(-1.0, 0.5, 0.0),
                    remus_math::vec::Point3::new(1.0, 0.5, 0.0),
                ],
            ],
            vec![vec![1.0, 1.0], vec![1.0, 1.0]],
        )
        .unwrap();
        source
            .topo_mut()
            .face_mut(face)
            .unwrap()
            .set_surface(remus_topology::face::FaceSurface::Nurbs(surface));
        let sheet_id = remus_operations::sew::make_sheet_body(source.topo_mut(), &[face]).unwrap();
        let sheet = shell_id_to_u32(sheet_id);

        let step = source.export_step_sheet(sheet, None).unwrap();
        let text = String::from_utf8(step.clone()).unwrap();
        assert!(text.contains("SHELL_BASED_SURFACE_MODEL("));
        assert!(text.contains("OPEN_SHELL("));
        assert!(text.contains("B_SPLINE_SURFACE_WITH_KNOTS("));

        let mut destination = BrepKernel::new();
        let json = destination.import_step_bodies(&step, None, None).unwrap();
        let result: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert!(result["solids"].as_array().unwrap().is_empty());
        let restored = result["sheets"][0].as_u64().unwrap() as u32;
        assert_eq!(
            destination
                .topo()
                .shell(destination.resolve_shell(restored).unwrap())
                .unwrap()
                .body_class(),
            remus_topology::BodyClass::Sheet
        );
        let restored_id = destination.resolve_shell(restored).unwrap();
        let report = remus_check::validate::validate_sheet_body(
            destination.topo(),
            restored_id,
            &remus_check::validate::ValidateOptions::default(),
        )
        .unwrap();
        assert_eq!(report.error_count(), 0);
        assert!(report.warning_count() > 0);
    }
}
