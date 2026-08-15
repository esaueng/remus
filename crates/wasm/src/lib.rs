//! # brepkit-wasm
//!
//! WebAssembly bindings for brepkit via `wasm-bindgen`.
//!
//! This is layer L3, the public API surface for JavaScript/TypeScript consumers.
//!
//! The primary entry point is [`kernel::BrepKernel`], which owns all modeling
//! state and exposes shape creation, operations, and tessellation to JS.

use wasm_bindgen::prelude::*;

mod bindings;
pub mod error;
mod handles;
mod helpers;
pub mod holed_face;
pub mod kernel;
mod logging;
pub mod panics;
pub mod shapes;
mod state;
mod types;

pub use types::FaceEvolutionPayloadV1;

/// Decode and validate a serialized version-1 face-evolution payload.
///
/// This is intended for persisted or transported payloads. It rejects unknown
/// fields, unsupported versions, incomplete source/result coverage, handles
/// outside the declared domains, duplicate pairs, and contradictory claims.
///
/// # Errors
///
/// Returns an error if `json` is malformed or violates the version-1 contract.
#[wasm_bindgen(js_name = "decodeEvolutionPayload")]
pub fn decode_evolution_payload(json: &str) -> Result<FaceEvolutionPayloadV1, JsError> {
    const MAX_EVOLUTION_PAYLOAD_BYTES: usize = 4 * 1024 * 1024;
    if json.len() > MAX_EVOLUTION_PAYLOAD_BYTES {
        return Err(JsError::new("evolution payload exceeds the 4 MiB limit"));
    }
    FaceEvolutionPayloadV1::decode(json).map_err(|error| JsError::new(&error))
}
