//! # remus-wasm-io
//!
//! File-format translators for remus as a WebAssembly module of their own.
//!
//! The kernel module (`remus-wasm`) owns the live topology; this module never
//! sees it. Bodies cross the boundary as exact arena documents: the kernel's
//! `serializeSolids` / `serializeSheets` produce the bytes an export method
//! here consumes, and every import method here returns bytes the kernel's
//! `deserializeSolids` / `deserializeSheets` restore. That codec is
//! byte-exact for `f64` geometry (no re-derivation, no tolerance
//! normalisation), so the extra hop adds no approximation.
//!
//! Consumers load this module only when a file is read or written, which
//! keeps the translators out of the kernel module's size budget.

#![allow(clippy::missing_errors_doc)]

pub mod error;

use remus_io::arena_io::{self, DeserializedDocument};
use remus_io::step::{StepReadResult, StepValidationOptions, StepWriteOptions};
use remus_io::{ImportLimits, IoError};
use remus_math::diagnostic::{DetailValue, ToDiagnostic};
use remus_topology::Topology;
use remus_topology::solid::SolidId;
use wasm_bindgen::prelude::*;

use crate::error::{IoWasmError, validate_positive};

/// Vertex-merge tolerance for mesh imports; matches the kernel's `TOL`.
const MESH_TOL: f64 = 1e-7;

/// The translator. Stateless: every call works in a fresh scratch topology.
#[wasm_bindgen]
#[derive(Default)]
pub struct RemusIo {}

/// Bodies restored from a STEP file, plus the reader's report.
#[wasm_bindgen]
pub struct StepImportResult {
    solids: Vec<u8>,
    sheets: Vec<u8>,
    report: String,
}

#[wasm_bindgen]
impl StepImportResult {
    /// Solid roots as an arena document for `BrepKernel.deserializeSolids`.
    /// Empty when the file held no solids.
    #[wasm_bindgen(getter)]
    #[must_use]
    pub fn solids(&self) -> Vec<u8> {
        self.solids.clone()
    }

    /// Sheet roots as an arena document for `BrepKernel.deserializeSheets`.
    /// Empty when the file held no sheets.
    #[wasm_bindgen(getter)]
    #[must_use]
    pub fn sheets(&self) -> Vec<u8> {
        self.sheets.clone()
    }

    /// JSON: `solidCount`, `sheetCount`, `diagnostics`, and `validation`
    /// when validation properties were requested.
    #[wasm_bindgen(getter)]
    #[must_use]
    pub fn report(&self) -> String {
        self.report.clone()
    }
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

#[derive(serde::Serialize)]
#[serde(rename_all = "camelCase")]
struct StepReport<V> {
    solid_count: usize,
    sheet_count: usize,
    diagnostics: Vec<StepImportDiagnostic>,
    #[serde(skip_serializing_if = "Option::is_none")]
    validation: Option<V>,
}

// ── Shared helpers ─────────────────────────────────────────────────

/// Build [`ImportLimits`] from optional JS overrides.
///
/// `max_input_bytes` bounds the encoded input size; `max_entities` bounds
/// format-specific model records. Absent values keep the production
/// defaults (256 MiB / 3,000,000).
fn import_limits_from(
    max_input_bytes: Option<f64>,
    max_entities: Option<f64>,
) -> Result<ImportLimits, IoWasmError> {
    let mut limits = ImportLimits::default();
    if let Some(bytes) = max_input_bytes {
        validate_positive(bytes, "maxInputBytes")?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            limits.max_input_bytes = bytes as usize;
        }
    }
    if let Some(entities) = max_entities {
        validate_positive(entities, "maxEntities")?;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            limits.max_model_entities = entities as usize;
        }
    }
    Ok(limits)
}

fn step_write_options_from_json(options: Option<&str>) -> Result<StepWriteOptions, IoWasmError> {
    match options {
        None => Ok(StepWriteOptions::default()),
        Some(json) => serde_json::from_str(json).map_err(|error| {
            IoWasmError::invalid(format!("invalid STEP write options JSON: {error}"))
        }),
    }
}

fn step_validation_options_from_json(
    options: Option<&str>,
) -> Result<StepValidationOptions, IoWasmError> {
    match options {
        None => Ok(StepValidationOptions::default()),
        Some(json) => serde_json::from_str(json).map_err(|error| {
            IoWasmError::invalid(format!("invalid STEP validation options JSON: {error}"))
        }),
    }
}

fn utf8<'a>(data: &'a [u8], format: &str) -> Result<&'a str, IoWasmError> {
    std::str::from_utf8(data)
        .map_err(|error| IoWasmError::invalid(format!("{format} data is not valid UTF-8: {error}")))
}

/// Restore an arena document into a scratch topology.
fn load_document(bytes: &[u8]) -> Result<(Topology, DeserializedDocument), IoWasmError> {
    if bytes.is_empty() {
        return Err(IoWasmError::invalid(
            "body document is empty (serialize bodies with BrepKernel first)",
        ));
    }
    let mut topo = Topology::new();
    let document = arena_io::deserialize_document(bytes, &mut topo)?;
    Ok((topo, document))
}

/// Restore a document and require solid roots, which every mesh writer needs.
fn load_solids(bytes: &[u8]) -> Result<(Topology, Vec<SolidId>), IoWasmError> {
    let (topo, document) = load_document(bytes)?;
    if document.solids.is_empty() {
        return Err(IoWasmError::invalid("body document holds no solids"));
    }
    Ok((topo, document.solids))
}

/// Solid roots as a document, or no bytes at all when there are none.
fn solids_document(topo: &Topology, solids: &[SolidId]) -> Result<Vec<u8>, IoError> {
    if solids.is_empty() {
        return Ok(Vec::new());
    }
    arena_io::serialize_solids(topo, solids)
}

fn sheets_document(
    topo: &Topology,
    sheets: &[remus_topology::shell::ShellId],
) -> Result<Vec<u8>, IoError> {
    if sheets.is_empty() {
        return Ok(Vec::new());
    }
    arena_io::serialize_sheets(topo, sheets)
}

fn step_result(
    topo: &Topology,
    result: &StepReadResult,
    with_validation: bool,
) -> Result<StepImportResult, IoWasmError> {
    let report = StepReport {
        solid_count: result.solids().len(),
        sheet_count: result.sheets().len(),
        diagnostics: result
            .diagnostics()
            .iter()
            .map(StepImportDiagnostic::from)
            .collect(),
        validation: with_validation.then(|| result.validation()),
    };
    Ok(StepImportResult {
        solids: solids_document(topo, result.solids())?,
        sheets: sheets_document(topo, result.sheets())?,
        report: serde_json::to_string(&report)?,
    })
}

fn mesh_solid_document(
    mesh: &remus_operations::tessellate::TriangleMesh,
) -> Result<Vec<u8>, IoWasmError> {
    let mut topo = Topology::new();
    let solid = remus_io::stl::import::import_mesh(&mut topo, mesh, MESH_TOL)?;
    Ok(arena_io::serialize_solid(&topo, solid)?)
}

// ── Native implementation (testable without a JS runtime) ─────────

impl RemusIo {
    fn export_step_impl(bodies: &[u8], options: Option<&str>) -> Result<Vec<u8>, IoWasmError> {
        let (topo, document) = load_document(bodies)?;
        let options = step_write_options_from_json(options)?;
        let step = remus_io::step::write_step_bodies_with_options(
            &topo,
            &document.solids,
            &document.sheets,
            &options,
        )?;
        Ok(step.into_bytes())
    }

    fn export_iges_impl(bodies: &[u8]) -> Result<Vec<u8>, IoWasmError> {
        let (topo, solids) = load_solids(bodies)?;
        Ok(remus_io::iges::writer::write_iges(&topo, &solids)?.into_bytes())
    }

    fn export_3mf_impl(bodies: &[u8], deflection: f64) -> Result<Vec<u8>, IoWasmError> {
        validate_positive(deflection, "deflection")?;
        let (topo, solids) = load_solids(bodies)?;
        Ok(remus_io::threemf::write_threemf(
            &topo, &solids, deflection,
        )?)
    }

    fn export_stl_impl(
        bodies: &[u8],
        deflection: f64,
        format: remus_io::stl::writer::StlFormat,
    ) -> Result<Vec<u8>, IoWasmError> {
        validate_positive(deflection, "deflection")?;
        let (topo, solids) = load_solids(bodies)?;
        Ok(remus_io::stl::writer::write_stl(
            &topo, &solids, deflection, format,
        )?)
    }

    fn export_obj_impl(bodies: &[u8], deflection: f64) -> Result<Vec<u8>, IoWasmError> {
        validate_positive(deflection, "deflection")?;
        let (topo, solids) = load_solids(bodies)?;
        Ok(remus_io::obj::write_obj(&topo, &solids, deflection)?.into_bytes())
    }

    fn export_glb_impl(bodies: &[u8], deflection: f64) -> Result<Vec<u8>, IoWasmError> {
        validate_positive(deflection, "deflection")?;
        let (topo, solids) = load_solids(bodies)?;
        Ok(remus_io::gltf::write_glb(&topo, &solids, deflection)?)
    }

    fn export_ply_impl(bodies: &[u8], deflection: f64) -> Result<Vec<u8>, IoWasmError> {
        validate_positive(deflection, "deflection")?;
        let (topo, solids) = load_solids(bodies)?;
        Ok(remus_io::ply::write_ply(
            &topo,
            &solids,
            deflection,
            remus_io::ply::writer::PlyFormat::BinaryLittleEndian,
        )?)
    }

    fn import_step_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = utf8(data, "STEP")?;
        let mut topo = Topology::new();
        let solids = remus_io::step::reader::read_step_with_limits(text, &mut topo, limits)?;
        Ok(solids_document(&topo, &solids)?)
    }

    fn import_step_bodies_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<StepImportResult, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = utf8(data, "STEP")?;
        let mut topo = Topology::new();
        let result = remus_io::step::read_step_bodies_with_limits(text, &mut topo, limits)?;
        step_result(&topo, &result, false)
    }

    fn import_step_with_report_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<StepImportResult, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = utf8(data, "STEP")?;
        let mut topo = Topology::new();
        let result = remus_io::step::read_step_with_limits_and_report(text, &mut topo, limits)?;
        step_result(&topo, &result, false)
    }

    fn import_step_with_validation_impl(
        data: &[u8],
        options: Option<&str>,
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<StepImportResult, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let options = step_validation_options_from_json(options)?;
        let text = utf8(data, "STEP")?;
        let mut topo = Topology::new();
        let result = remus_io::step::read_step_with_validation(text, &mut topo, limits, options)?;
        step_result(&topo, &result, true)
    }

    fn import_iges_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = utf8(data, "IGES")?;
        let mut topo = Topology::new();
        let solids = remus_io::iges::reader::read_iges_with_limits(text, &mut topo, limits)?;
        Ok(solids_document(&topo, &solids)?)
    }

    fn import_stl_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let mesh = remus_io::stl::reader::read_stl_with_limits(data, limits)?;
        mesh_solid_document(&mesh)
    }

    fn import_obj_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let text = utf8(data, "OBJ")?;
        let mesh = remus_io::obj::read_obj_with_limits(text, limits)?;
        mesh_solid_document(&mesh)
    }

    fn import_glb_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let mesh = remus_io::gltf::read_glb_with_limits(data, limits)?;
        mesh_solid_document(&mesh)
    }

    fn import_ply_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let mesh = remus_io::ply::read_ply_with_limits(data, limits)?;
        mesh_solid_document(&mesh)
    }

    fn import_3mf_impl(
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, IoWasmError> {
        let limits = import_limits_from(max_input_bytes, max_entities)?;
        let meshes = remus_io::threemf::reader::read_threemf_with_limits(data, limits)?;
        let mut topo = Topology::new();
        let mut solids = Vec::with_capacity(meshes.len());
        for mesh in &meshes {
            solids.push(remus_io::stl::import::import_mesh(
                &mut topo, mesh, MESH_TOL,
            )?);
        }
        Ok(solids_document(&topo, &solids)?)
    }

    fn import_indexed_mesh_impl(
        positions: &[f64],
        indices: &[u32],
    ) -> Result<Vec<u8>, IoWasmError> {
        if !positions.len().is_multiple_of(3) {
            return Err(IoWasmError::invalid(format!(
                "positions length {} is not a multiple of 3",
                positions.len()
            )));
        }
        if !indices.len().is_multiple_of(3) {
            return Err(IoWasmError::invalid(format!(
                "indices length {} is not a multiple of 3",
                indices.len()
            )));
        }
        let mesh = remus_operations::tessellate::TriangleMesh {
            positions: positions
                .chunks_exact(3)
                .map(|c| remus_math::vec::Point3::new(c[0], c[1], c[2]))
                .collect(),
            normals: Vec::new(),
            indices: indices.to_vec(),
        };
        mesh_solid_document(&mesh)
    }
}

// ── JavaScript surface ─────────────────────────────────────────────

// `&self` is what makes these JavaScript instance methods; the translator
// itself carries no state.
#[allow(clippy::unused_self)]
#[wasm_bindgen]
impl RemusIo {
    /// Create a translator.
    #[wasm_bindgen(constructor)]
    #[must_use]
    pub fn new() -> Self {
        Self {}
    }

    // ── Export: `bodies` is an arena document from the kernel ──────

    /// Write the bodies of an arena document as STEP AP203 (UTF-8 bytes).
    ///
    /// Accepts solid documents (`serializeSolids`) and sheet documents
    /// (`serializeSheets`); several roots become distinct bodies in one file.
    #[wasm_bindgen(js_name = "exportStep")]
    pub fn export_step(&self, bodies: &[u8]) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_step_impl(bodies, None)?)
    }

    /// [`exportStep`](Self::export_step) with optional header metadata.
    ///
    /// `options` is a JSON string with `productName`, `fileName`,
    /// `timestamp`, and `validationProperties` fields; missing fields keep
    /// their defaults.
    #[wasm_bindgen(js_name = "exportStepWithOptions")]
    pub fn export_step_with_options(
        &self,
        bodies: &[u8],
        options: Option<String>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_step_impl(bodies, options.as_deref())?)
    }

    /// Write the solids of an arena document as IGES (UTF-8 bytes).
    #[wasm_bindgen(js_name = "exportIges")]
    pub fn export_iges(&self, bodies: &[u8]) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_iges_impl(bodies)?)
    }

    /// Tessellate the solids of an arena document into one 3MF package.
    #[wasm_bindgen(js_name = "export3mf")]
    pub fn export_3mf(&self, bodies: &[u8], deflection: f64) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_3mf_impl(bodies, deflection)?)
    }

    /// Tessellate the solids of an arena document into one binary STL.
    #[wasm_bindgen(js_name = "exportStl")]
    pub fn export_stl(&self, bodies: &[u8], deflection: f64) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_stl_impl(
            bodies,
            deflection,
            remus_io::stl::writer::StlFormat::Binary,
        )?)
    }

    /// Tessellate the solids of an arena document into one ASCII STL.
    #[wasm_bindgen(js_name = "exportStlAscii")]
    pub fn export_stl_ascii(&self, bodies: &[u8], deflection: f64) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_stl_impl(
            bodies,
            deflection,
            remus_io::stl::writer::StlFormat::Ascii,
        )?)
    }

    /// Tessellate the solids of an arena document into one OBJ (UTF-8 bytes).
    #[wasm_bindgen(js_name = "exportObj")]
    pub fn export_obj(&self, bodies: &[u8], deflection: f64) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_obj_impl(bodies, deflection)?)
    }

    /// Tessellate the solids of an arena document into one glTF binary.
    #[wasm_bindgen(js_name = "exportGlb")]
    pub fn export_glb(&self, bodies: &[u8], deflection: f64) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_glb_impl(bodies, deflection)?)
    }

    /// Tessellate the solids of an arena document into one binary PLY.
    #[wasm_bindgen(js_name = "exportPly")]
    pub fn export_ply(&self, bodies: &[u8], deflection: f64) -> Result<Vec<u8>, JsError> {
        Ok(Self::export_ply_impl(bodies, deflection)?)
    }

    // ── Import: returns an arena document for the kernel ───────────

    /// Read a STEP file; returns a solid document for `deserializeSolids`,
    /// or no bytes when the file holds no solids.
    ///
    /// `maxInputBytes` / `maxEntities` optionally tighten the hostile-input
    /// resource budgets below the production defaults (256 MiB / 3,000,000).
    #[wasm_bindgen(js_name = "importStep")]
    pub fn import_step(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_step_impl(data, max_input_bytes, max_entities)?)
    }

    /// Read every supported STEP body root: solids and sheets, each as its
    /// own document, with bounded-healing diagnostics in the report.
    #[wasm_bindgen(js_name = "importStepBodies")]
    pub fn import_step_bodies(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<StepImportResult, JsError> {
        Ok(Self::import_step_bodies_impl(
            data,
            max_input_bytes,
            max_entities,
        )?)
    }

    /// Read a STEP file's solids with bounded-healing diagnostics.
    #[wasm_bindgen(js_name = "importStepWithReport")]
    pub fn import_step_with_report(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<StepImportResult, JsError> {
        Ok(Self::import_step_with_report_impl(
            data,
            max_input_bytes,
            max_entities,
        )?)
    }

    /// Read STEP and check CAx-IF geometric validation properties.
    ///
    /// The report carries one `validation` entry per solid. `options` accepts
    /// the camelCase fields of the STEP validation options.
    #[wasm_bindgen(js_name = "importStepWithValidation")]
    pub fn import_step_with_validation(
        &self,
        data: &[u8],
        options: Option<String>,
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<StepImportResult, JsError> {
        Ok(Self::import_step_with_validation_impl(
            data,
            options.as_deref(),
            max_input_bytes,
            max_entities,
        )?)
    }

    /// Read an IGES file; returns a solid document, or no bytes when empty.
    #[wasm_bindgen(js_name = "importIges")]
    pub fn import_iges(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_iges_impl(data, max_input_bytes, max_entities)?)
    }

    /// Read an STL file (binary or ASCII) into one mesh-backed solid document.
    #[wasm_bindgen(js_name = "importStl")]
    pub fn import_stl(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_stl_impl(data, max_input_bytes, max_entities)?)
    }

    /// Read an OBJ file into one mesh-backed solid document.
    #[wasm_bindgen(js_name = "importObj")]
    pub fn import_obj(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_obj_impl(data, max_input_bytes, max_entities)?)
    }

    /// Read a glTF binary into one mesh-backed solid document.
    #[wasm_bindgen(js_name = "importGlb")]
    pub fn import_glb(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_glb_impl(data, max_input_bytes, max_entities)?)
    }

    /// Read a PLY file into one mesh-backed solid document.
    #[wasm_bindgen(js_name = "importPly")]
    pub fn import_ply(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_ply_impl(data, max_input_bytes, max_entities)?)
    }

    /// Read a 3MF package; every object becomes a solid root of one document.
    #[wasm_bindgen(js_name = "import3mf")]
    pub fn import_3mf(
        &self,
        data: &[u8],
        max_input_bytes: Option<f64>,
        max_entities: Option<f64>,
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_3mf_impl(data, max_input_bytes, max_entities)?)
    }

    /// Build a mesh-backed solid document from flat `[x,y,z,...]` positions
    /// and `[i0,i1,i2,...]` triangle indices.
    #[wasm_bindgen(js_name = "importIndexedMesh")]
    pub fn import_indexed_mesh(
        &self,
        positions: &[f64],
        indices: &[u32],
    ) -> Result<Vec<u8>, JsError> {
        Ok(Self::import_indexed_mesh_impl(positions, indices)?)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_operations::measure::solid_volume;
    use remus_operations::primitives::make_box;

    fn box_document() -> Vec<u8> {
        let mut topo = Topology::new();
        let solid = make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        arena_io::serialize_solids(&topo, &[solid]).unwrap()
    }

    fn volumes(document: &[u8]) -> Vec<f64> {
        let mut topo = Topology::new();
        let solids = arena_io::deserialize_solids(document, &mut topo).unwrap();
        solids
            .into_iter()
            .map(|solid| solid_volume(&topo, solid, 0.1).unwrap())
            .collect()
    }

    #[test]
    fn step_round_trip_through_arena_documents_is_exact() {
        let step = RemusIo::export_step_impl(&box_document(), None).unwrap();
        assert!(String::from_utf8_lossy(&step).contains("MANIFOLD_SOLID_BREP"));

        let restored = RemusIo::import_step_impl(&step, None, None).unwrap();
        let volumes = volumes(&restored);
        assert_eq!(volumes.len(), 1);
        assert!((volumes[0] - 24.0).abs() < 1e-9, "volume {}", volumes[0]);
    }

    #[test]
    fn step_options_and_validation_flow_through_the_report() {
        let step = RemusIo::export_step_impl(
            &box_document(),
            Some(r#"{"productName":"Custom part","validationProperties":true}"#),
        )
        .unwrap();
        let text = String::from_utf8(step.clone()).unwrap();
        assert!(text.contains("PRODUCT('Custom part', 'Custom part'"));

        let result = RemusIo::import_step_with_validation_impl(&step, None, None, None).unwrap();
        let report: serde_json::Value = serde_json::from_str(&result.report()).unwrap();
        assert_eq!(report["solidCount"], 1);
        assert_eq!(report["sheetCount"], 0);
        assert!(report["diagnostics"].as_array().unwrap().is_empty());
        assert!(
            (report["validation"][0]["declared"]["volume"]
                .as_f64()
                .unwrap()
                - 24.0)
                .abs()
                < 1e-9
        );
        assert!(result.sheets().is_empty());
        assert_eq!(volumes(&result.solids()), vec![24.0]);

        let plain = RemusIo::import_step_with_report_impl(&step, None, None).unwrap();
        let report: serde_json::Value = serde_json::from_str(&plain.report()).unwrap();
        assert!(report.get("validation").is_none());
    }

    #[test]
    fn sheet_documents_export_and_come_back_as_sheets() {
        let mut topo = Topology::new();
        let face = remus_topology::builder::make_rectangle_face(&mut topo, 2.0, 1.0, 1e-7).unwrap();
        let sheet = remus_operations::sew::make_sheet_body(&mut topo, &[face]).unwrap();
        let document = arena_io::serialize_sheets(&topo, &[sheet]).unwrap();

        let step = RemusIo::export_step_impl(&document, None).unwrap();
        assert!(String::from_utf8_lossy(&step).contains("SHELL_BASED_SURFACE_MODEL("));

        let result = RemusIo::import_step_bodies_impl(&step, None, None).unwrap();
        assert!(result.solids().is_empty());
        let mut restored = Topology::new();
        let sheets = arena_io::deserialize_sheets(&result.sheets(), &mut restored).unwrap();
        assert_eq!(sheets.len(), 1);
        let report: serde_json::Value = serde_json::from_str(&result.report()).unwrap();
        assert_eq!(report["sheetCount"], 1);

        // Mesh writers need solids; a sheet-only document is a typed refusal.
        assert!(matches!(
            RemusIo::export_stl_impl(&document, 0.1, remus_io::stl::writer::StlFormat::Binary),
            Err(IoWasmError::InvalidInput { .. })
        ));
    }

    #[test]
    fn mesh_formats_round_trip_a_box() {
        let document = box_document();
        let stl =
            RemusIo::export_stl_impl(&document, 0.1, remus_io::stl::writer::StlFormat::Binary)
                .unwrap();
        assert_eq!(
            volumes(&RemusIo::import_stl_impl(&stl, None, None).unwrap()).len(),
            1
        );
        let ply = RemusIo::export_ply_impl(&document, 0.1).unwrap();
        let restored = volumes(&RemusIo::import_ply_impl(&ply, None, None).unwrap());
        assert!(
            (restored[0] - 24.0).abs() < 1e-6,
            "ply volume {}",
            restored[0]
        );
        let threemf = RemusIo::export_3mf_impl(&document, 0.1).unwrap();
        assert_eq!(&threemf[0..2], b"PK");
        assert_eq!(
            volumes(&RemusIo::import_3mf_impl(&threemf, None, None).unwrap()).len(),
            1
        );
        let glb = RemusIo::export_glb_impl(&document, 0.1).unwrap();
        assert_eq!(&glb[0..4], b"glTF");
        assert_eq!(
            volumes(&RemusIo::import_glb_impl(&glb, None, None).unwrap()).len(),
            1
        );
        let obj = RemusIo::export_obj_impl(&document, 0.1).unwrap();
        assert_eq!(
            volumes(&RemusIo::import_obj_impl(&obj, None, None).unwrap()).len(),
            1
        );
        let iges = RemusIo::export_iges_impl(&document).unwrap();
        assert!(!iges.is_empty());
    }

    #[test]
    fn invalid_arguments_are_typed_refusals() {
        assert!(matches!(
            RemusIo::export_stl_impl(&[], 0.1, remus_io::stl::writer::StlFormat::Binary),
            Err(IoWasmError::InvalidInput { .. })
        ));
        assert!(matches!(
            RemusIo::export_stl_impl(
                &box_document(),
                0.0,
                remus_io::stl::writer::StlFormat::Binary
            ),
            Err(IoWasmError::InvalidInput { .. })
        ));
        assert!(matches!(
            RemusIo::import_indexed_mesh_impl(&[0.0, 0.0], &[0, 1, 2]),
            Err(IoWasmError::InvalidInput { .. })
        ));
        assert!(matches!(
            RemusIo::import_step_impl(b"not step", Some(4.0), None),
            Err(IoWasmError::Io(IoError::LimitExceeded { .. }))
        ));
        assert!(matches!(
            RemusIo::export_step_impl(&box_document(), Some("{bad json")),
            Err(IoWasmError::InvalidInput { .. })
        ));
    }

    #[test]
    fn indexed_mesh_becomes_a_solid_document() {
        let positions = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let indices = [0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
        let document = RemusIo::import_indexed_mesh_impl(&positions, &indices).unwrap();
        let volumes = volumes(&document);
        assert!((volumes[0] - 1.0 / 6.0).abs() < 1e-9);
    }
}
