//! Exact arena document bindings.
//!
//! These are the kernel's native interchange format: byte-exact copies of
//! reachable topology, used to move bodies between kernel instances and to
//! hand them to the separate `remus-wasm-io` translator module. They are
//! always compiled; the file-format translators behind the `io` feature are
//! not.

#![allow(clippy::missing_errors_doc)]

use wasm_bindgen::prelude::*;

use crate::error::WasmError;
use crate::handles::{shell_id_to_u32, solid_id_to_u32, wire_id_to_u32};
use crate::kernel::BrepKernel;

/// A selected document copied into a fresh kernel, with ordered root remaps.
///
/// Read the roots and call `takeKernel()` once. Each root belongs exclusively
/// to that new owner. Freeing this object before taking the kernel drops the
/// copied document; freeing it afterwards leaves the taken kernel intact.
#[wasm_bindgen]
pub struct KernelDocumentCopy {
    kernel: Option<BrepKernel>,
    solids: Vec<u32>,
    sheets: Vec<u32>,
    wires: Vec<u32>,
}

#[wasm_bindgen]
impl KernelDocumentCopy {
    /// Solid roots corresponding to the requested solids, including aliases.
    #[wasm_bindgen(getter)]
    #[must_use]
    pub fn solids(&self) -> Vec<u32> {
        self.solids.clone()
    }

    /// Sheet roots corresponding to the requested sheets, including aliases.
    #[wasm_bindgen(getter)]
    #[must_use]
    pub fn sheets(&self) -> Vec<u32> {
        self.sheets.clone()
    }

    /// Wire roots corresponding to the requested wires, including aliases.
    #[wasm_bindgen(getter)]
    #[must_use]
    pub fn wires(&self) -> Vec<u32> {
        self.wires.clone()
    }

    /// Transfer the new kernel to the caller. A second call throws.
    #[wasm_bindgen(js_name = "takeKernel")]
    pub fn take_kernel(&mut self) -> Result<BrepKernel, JsError> {
        self.take_kernel_impl().map_err(Into::into)
    }
}

#[derive(Debug, thiserror::Error)]
enum DocumentCopyError {
    #[error(transparent)]
    Handle(#[from] WasmError),
    #[error(transparent)]
    Document(#[from] remus_io::IoError),
    #[error("document copy kernel has already been taken")]
    KernelTaken,
    #[error("a poisoned kernel cannot copy a document")]
    Poisoned,
    #[error("document copy entity limit {requested} exceeds the default maximum {maximum}")]
    EntityLimit { requested: usize, maximum: usize },
}

impl KernelDocumentCopy {
    fn take_kernel_impl(&mut self) -> Result<BrepKernel, DocumentCopyError> {
        self.kernel.take().ok_or(DocumentCopyError::KernelTaken)
    }
}

#[wasm_bindgen]
impl BrepKernel {
    /// Copy selected solid, sheet and wire roots into a fresh kernel.
    ///
    /// Retains exact document data and selected lineage without JSON transport,
    /// omitting unrelated live entities and retired slots. Root order and
    /// duplicate aliases are preserved. `maxEntities`, when supplied, can only
    /// lower the native default entity/reference budget; it is not a byte cap.
    ///
    /// The source and its checkpoints stay intact. The returned kernel has no
    /// checkpoints, sketches, assemblies or prepared moves. Rebuild selections
    /// in that owner from the returned roots. Numeric handles remain scoped to
    /// their kernel; equal numbers in two kernels do not imply equal identity.
    /// Release the old owner and its wrappers explicitly to reclaim its data.
    /// WASM linear memory need not shrink, and both documents coexist while
    /// copying. This does not validate geometric correctness.
    ///
    /// Direct-only: a JSON batch cannot return ownership of another kernel.
    #[wasm_bindgen(js_name = "copyDocument")]
    pub fn copy_document(
        &self,
        solids: &[u32],
        sheets: &[u32],
        wires: &[u32],
        max_entities: Option<u32>,
    ) -> Result<KernelDocumentCopy, JsError> {
        Ok(self.copy_document_impl(solids, sheets, wires, max_entities)?)
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

impl BrepKernel {
    fn copy_document_impl(
        &self,
        solids: &[u32],
        sheets: &[u32],
        wires: &[u32],
        max_entities: Option<u32>,
    ) -> Result<KernelDocumentCopy, DocumentCopyError> {
        use remus_io::arena_io::{CompactionRoots, compact_document_with_limit};
        if self.poisoned {
            return Err(DocumentCopyError::Poisoned);
        }
        let maximum = remus_io::ImportLimits::default().max_model_entities;
        let limit = max_entities.map_or(maximum, |limit| limit as usize);
        if limit > maximum {
            return Err(DocumentCopyError::EntityLimit {
                requested: limit,
                maximum,
            });
        }
        // Check before allocating resolved root lists. The native discovery
        // additionally accounts for every entity and reference it copies.
        if [solids.len(), sheets.len(), wires.len()]
            .into_iter()
            .any(|count| count > limit)
        {
            return Err(remus_io::IoError::LimitExceeded {
                resource: "document copy roots",
                limit,
                actual: solids.len().max(sheets.len()).max(wires.len()),
            }
            .into());
        }
        let solid_ids = solids
            .iter()
            .map(|&id| self.resolve_solid(id))
            .collect::<Result<Vec<_>, _>>()?;
        let sheet_ids = sheets
            .iter()
            .map(|&id| self.resolve_shell(id))
            .collect::<Result<Vec<_>, _>>()?;
        let wire_ids = wires
            .iter()
            .map(|&id| self.resolve_wire(id))
            .collect::<Result<Vec<_>, _>>()?;
        let copied = compact_document_with_limit(
            self.topo(),
            CompactionRoots {
                solids: &solid_ids,
                sheets: &sheet_ids,
                wires: &wire_ids,
                compounds: &[],
            },
            limit,
        )?;
        Ok(KernelDocumentCopy {
            kernel: Some(Self::from_document_topology(copied.topology)),
            solids: copied
                .roots
                .solids
                .into_iter()
                .map(solid_id_to_u32)
                .collect(),
            sheets: copied
                .roots
                .sheets
                .into_iter()
                .map(shell_id_to_u32)
                .collect(),
            wires: copied.roots.wires.into_iter().map(wire_id_to_u32).collect(),
        })
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used)]

    use super::*;

    #[test]
    fn document_copy_reclaims_tombstones_without_changing_source_or_checkpoints() {
        let mut source = BrepKernel::new();
        let solid = source.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let checkpoint = source.checkpoint_impl().unwrap();
        let mut stale = 0;
        for _ in 0..16 {
            stale = source.make_box_solid(1.0, 1.0, 1.0).unwrap();
            source.restore_checkpoint_impl(checkpoint).unwrap();
        }
        let bytes = source.serialize_solids(&[solid, solid]).unwrap();
        let identity = source.topo().cache_identity();
        let old_topology = std::rc::Rc::clone(&source.topo);
        let stats = source.checkpoint_memory_stats().unwrap();
        let mut copied = source
            .copy_document_impl(&[solid, solid], &[], &[], None)
            .unwrap();
        assert_eq!(copied.solids[0], copied.solids[1]);
        let mut destination = copied.take_kernel_impl().unwrap();
        assert!(matches!(
            copied.take_kernel_impl(),
            Err(DocumentCopyError::KernelTaken)
        ));
        assert_eq!(destination.serialize_solids(&copied.solids).unwrap(), bytes);
        assert_eq!(source.checkpoint_memory_stats().unwrap(), stats);
        assert_eq!(source.topo().cache_identity(), identity);
        assert!(std::rc::Rc::ptr_eq(&source.topo, &old_topology));
        assert!(!std::rc::Rc::ptr_eq(&source.topo, &destination.topo));
        let memory: serde_json::Value =
            serde_json::from_str(&destination.checkpoint_memory_stats().unwrap()).unwrap();
        assert_eq!(memory["allocatedSlots"], 64);
        assert_eq!(memory["retiredSlots"], 0);
        assert_eq!(memory["checkpoints"], 0);
        assert!((destination.volume(copied.solids[0], 0.02).unwrap() - 24.0).abs() < 1e-12);
        destination.make_box_solid(5.0, 5.0, 5.0).unwrap();
        assert!(source.resolve_solid(stale).is_err());
        source.restore_checkpoint_impl(checkpoint).unwrap();
        assert_eq!(source.serialize_solids(&[solid, solid]).unwrap(), bytes);
        let later = source.make_box_solid(1.0, 1.0, 1.0).unwrap();
        assert!(later > stale);
    }

    #[test]
    fn document_copy_preserves_mixed_roots_and_excludes_session_state() {
        let mut source = BrepKernel::new();
        let solid = source.make_box_solid(2.0, 3.0, 4.0).unwrap();
        let face = source
            .topo()
            .shell(
                source
                    .topo()
                    .solid(source.resolve_solid(solid).unwrap())
                    .unwrap()
                    .outer_shell(),
            )
            .unwrap()
            .faces()[0];
        let sheet_id = remus_operations::sew::make_sheet_body(source.topo_mut(), &[face]).unwrap();
        let sheet = shell_id_to_u32(sheet_id);
        let wire = source.make_regular_polygon_wire(1.0, 4).unwrap();
        source
            .sketches
            .push(crate::state::SketchState::default())
            .unwrap();
        source
            .gcs_sketches
            .push(crate::state::GcsSketchState::default())
            .unwrap();
        source
            .assemblies
            .push(crate::state::AssemblyState::new("source only"))
            .unwrap();
        source.checkpoint_impl().unwrap();
        let prepared = source
            .prepare_move_faces_impl(solid, &[crate::handles::face_id_to_u32(face)])
            .unwrap();
        let mut copied = source
            .copy_document_impl(&[solid, solid], &[sheet, sheet], &[wire, wire], None)
            .unwrap();
        let mut destination = copied.take_kernel_impl().unwrap();
        assert_eq!(copied.sheets[0], copied.sheets[1]);
        assert_eq!(copied.wires[0], copied.wires[1]);
        assert_eq!(
            source.serialize_solids(&[solid, solid]).unwrap(),
            destination.serialize_solids(&copied.solids).unwrap()
        );
        assert_eq!(
            source.serialize_sheets(&[sheet, sheet]).unwrap(),
            destination.serialize_sheets(&copied.sheets).unwrap()
        );
        assert_eq!(
            source.serialize_wires(&[wire, wire]).unwrap(),
            destination.serialize_wires(&copied.wires).unwrap()
        );
        assert_eq!(destination.sketches.iter().count(), 0);
        assert_eq!(destination.gcs_sketches.iter().count(), 0);
        assert_eq!(destination.assemblies.iter().count(), 0);
        assert_eq!(destination.checkpoints.active_len(), 0);
        assert!(!destination.release_prepared_move(prepared));
        assert!(source.release_prepared_move(prepared));
        let solid_face = destination
            .topo()
            .shell(
                destination
                    .topo()
                    .solid(destination.resolve_solid(copied.solids[0]).unwrap())
                    .unwrap()
                    .outer_shell(),
            )
            .unwrap()
            .faces()[0];
        assert_eq!(
            destination
                .topo()
                .shell(destination.resolve_shell(copied.sheets[0]).unwrap())
                .unwrap()
                .faces(),
            &[solid_face]
        );
    }

    #[test]
    fn document_copy_refusals_leave_source_exact_and_empty_copy_is_owned_once() {
        let mut source = BrepKernel::new();
        let solid = source.make_box_solid(1.0, 2.0, 3.0).unwrap();
        let bytes = source.serialize_solid(solid).unwrap();
        let stats = source.checkpoint_memory_stats().unwrap();
        assert!(matches!(
            source.copy_document_impl(&[u32::MAX], &[], &[], None),
            Err(DocumentCopyError::Handle(_))
        ));
        assert!(matches!(
            source.copy_document_impl(&[solid], &[], &[], Some(1)),
            Err(DocumentCopyError::Document(
                remus_io::IoError::LimitExceeded { .. }
            ))
        ));
        assert!(matches!(
            source.copy_document_impl(&[solid, solid], &[], &[], Some(1)),
            Err(DocumentCopyError::Document(
                remus_io::IoError::LimitExceeded { .. }
            ))
        ));
        assert!(matches!(
            source.copy_document_impl(&[], &[], &[], Some(u32::MAX)),
            Err(DocumentCopyError::EntityLimit { .. })
        ));
        let outer = source
            .topo()
            .solid(source.resolve_solid(solid).unwrap())
            .unwrap()
            .outer_shell();
        assert!(matches!(
            source.copy_document_impl(&[], &[shell_id_to_u32(outer)], &[], None),
            Err(DocumentCopyError::Document(remus_io::IoError::Topology(_)))
        ));
        source.poisoned = true;
        assert!(matches!(
            source.copy_document_impl(&[solid], &[], &[], None),
            Err(DocumentCopyError::Poisoned)
        ));
        source.poisoned = false;
        assert_eq!(source.serialize_solid(solid).unwrap(), bytes);
        assert_eq!(source.checkpoint_memory_stats().unwrap(), stats);
        let mut empty = source.copy_document_impl(&[], &[], &[], Some(0)).unwrap();
        let next = empty.take_kernel_impl().unwrap();
        assert_eq!(next.topo().num_solids(), 0);
        assert!(matches!(
            empty.take_kernel_impl(),
            Err(DocumentCopyError::KernelTaken)
        ));
    }

    #[test]
    fn document_copy_preserves_disabled_module_reuse_configuration() {
        let source = BrepKernel::new();
        source.set_mesh_reuse_enabled(false);
        source.set_measurement_reuse_enabled(false);
        let mesh = source.mesh_reuse_stats().unwrap();
        let measurement = source.measurement_reuse_stats().unwrap();
        let mut copied = source.copy_document_impl(&[], &[], &[], None).unwrap();
        let destination = copied.take_kernel_impl().unwrap();
        assert_eq!(destination.mesh_reuse_stats().unwrap(), mesh);
        assert_eq!(destination.measurement_reuse_stats().unwrap(), measurement);
        source.set_mesh_reuse_enabled(true);
        source.set_measurement_reuse_enabled(true);
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
}
