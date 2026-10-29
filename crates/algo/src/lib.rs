//! # remus-algo
//!
//! General Fuse Algorithm (GFA) engine for boolean operations.
//!
//! This is layer L2, depending on `remus-math` and `remus-topology`.
//! The `remus-operations` crate delegates boolean, section, and split
//! operations to this crate.
//!
//! # Architecture
//!
//! The GFA follows a proven two-phase approach:
//!
//! 1. **PaveFiller** — intersects all shape pairs, builds pave blocks
//!    (edge segments at intersection points), and populates face info.
//! 2. **Builder** — splits faces using pave block data, classifies
//!    sub-faces relative to opposing solids, assembles result shells.
//! 3. **BOP** — selects faces based on boolean operation type
//!    (fuse/cut/intersect).

// Unit tests may use `sort_by`: test binaries never ship, so the per-call-site
// sort copies that `clippy.toml` disallows in production code do not matter there.
#![cfg_attr(test, allow(clippy::disallowed_methods))]

pub mod bop;
pub mod diagnostic;
pub mod error;
pub mod gfa;
mod memo;
pub mod pcurve_memo;
pub mod perf;
pub mod ray_geom_memo;
pub mod wire_area_memo;

pub use memo::MemoStats;

mod builder;
pub mod classifier;
mod conic_bounds;

pub use builder::FaceClass;
pub use builder::pcurve_compute::{
    compute_pcurve_on_surface, compute_pcurve_on_surface_in_domain, wall_chart, wall_chart_uv,
};
pub use builder::plane_frame::PlaneFrame;
pub use builder::split_types::sub_trim;
pub use pave_filler::curved_section_clip::discovery::{
    RuledFaceSection, RuledSurfaceTrace, discover_ruled_face_section,
};
pub use pave_filler::curved_section_clip::{
    BoundaryEvent as RuledSectionBoundaryEvent, ClipError as RuledSectionError,
    ClippedSection as RuledSectionClipping, RetainedInterval as RuledSectionInterval,
};
pub(crate) mod ds;
pub(crate) mod pave_filler;
