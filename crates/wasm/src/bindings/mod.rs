//! Domain-aligned binding modules.
//!
//! Each module adds `#[wasm_bindgen] impl RemusKernel { ... }` methods
//! for a specific domain. wasm-bindgen supports multiple impl blocks
//! across files.

pub mod arena;
pub mod assembly;
pub mod batch;
pub mod booleans;
pub mod checkpoint;
pub mod construction_detailed;
pub mod construction_imprint;
pub mod draft_analysis;
pub mod evolution;
pub mod gcs_sketch;
pub mod gear;
pub mod heal;
#[cfg(feature = "io")]
pub mod io;
pub mod lifecycle;
pub mod linked_sketch;
pub mod maximum_distance;
pub mod measure;
pub mod modifier_detailed;
pub mod naming;
pub mod nurbs;
pub mod nurbs_reuse;
pub mod operations;
pub mod performance;
pub mod polygon2d;
pub mod prepared_move;
pub mod primitives;
pub mod project_curve;
pub mod query;
pub mod shapes;
pub mod sketch;
pub mod tessellate;
pub mod thread;
pub mod transform_detailed;
pub mod transforms;

#[cfg(test)]
mod batch_optional_args_tests;
#[cfg(test)]
mod face_digest_tests;
#[cfg(test)]
mod face_move_preview_tests;
#[cfg(test)]
mod face_normal_contract_tests;
#[cfg(test)]
mod fillet_fail_closed_tests;
#[cfg(test)]
mod gridfinity_tests;
#[cfg(test)]
mod growing_holder_volume_tests;
#[cfg(test)]
mod holed_face_tests;
#[cfg(test)]
mod mixed_notch_tests;
#[cfg(test)]
mod qualify_convex_hull_tests;
#[cfg(test)]
mod qualify_edge_relations_tests;
#[cfg(test)]
mod qualify_ops_tests;
#[cfg(test)]
mod qualify_primitives_tests;
#[cfg(test)]
mod qualify_section_split_tests;
#[cfg(test)]
mod sequential_fillet_tests;
#[cfg(test)]
mod sew_faces_default_tests;
#[cfg(test)]
mod sketch_validation_tests;
#[cfg(test)]
mod store_local_switch_tests;
#[cfg(test)]
mod sweep_transaction_tests;

#[cfg(test)]
mod transaction_tests;
#[cfg(test)]
mod wire_traversal_contract_tests;
