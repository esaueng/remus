//! IGES (Initial Graphics Exchange Specification) import and export.
//!
//! [`read_iges`] / [`write_iges`] are a declared lossy preview (planes and
//! polyline edges). [`brep`] is the qualified exact path for manifold solid
//! B-rep objects (type 186): [`read_iges_brep`] / [`write_iges_brep`].

pub mod brep;
pub mod reader;
pub mod writer;

pub use brep::{
    IgesBrepReadResult, IgesWriteOptions, iges_brep_as_step, read_iges_brep, write_iges_brep,
};
pub use reader::{read_iges, read_iges_with_limits};
pub use writer::write_iges;
