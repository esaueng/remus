//! Independently implemented ASCII BREP reader (FreeCAD saved shapes).
//!
//! The initial envelope is planar, straight-edged solid geometry. Unsupported
//! carriers, nonidentity locations and mixed-dimensional roots refuse atomically.
//! See `docs/kernel-maturity/freecad-brep-import.md` for the qualification boundary.

mod planar;
mod reader;

pub use reader::{read_brep, read_brep_with_limits};
