//! Independently implemented ASCII BREP reader (FreeCAD saved shapes).
//!
//! The envelope covers planar polygons, full circular caps and cylindrical bands
//! under proper rigid saved placements. Other carriers and mixed-dimensional
//! roots refuse atomically.
//! See `docs/kernel-maturity/freecad-brep-import.md` for the qualification boundary.

mod planar;
mod reader;

pub use reader::{read_brep, read_brep_with_limits};
