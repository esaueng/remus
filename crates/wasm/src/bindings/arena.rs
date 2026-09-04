//! Exact arena document bindings.
//!
//! These are the kernel's native interchange format: byte-exact copies of
//! reachable topology, used to move bodies between kernel instances and to
//! hand them to the separate `remus-wasm-io` translator module. They are
//! always compiled; the file-format translators behind the `io` feature are
//! not.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use crate::handles::{shell_id_to_u32, solid_id_to_u32, wire_id_to_u32};
use crate::kernel::BrepKernel;

#[wasm_bindgen]
impl BrepKernel {
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
}
