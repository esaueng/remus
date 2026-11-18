//! Cylinder and cone faces with fitted (NURBS) trims, read by an independent
//! quadrature and compared with the volume terms `solid_volume` sums (test
//! support).
//!
//! `solid_volume` reads such a face on its own boundary (the check crate's
//! `cylinder_green` and `cone_green`) and every other face as it did before,
//! so a pinned volume moves by exactly what these faces' terms move. This
//! reads each of them again with the stored-B-rep oracle of the wrap tests
//! (`crates/operations/src/wrap/volume_oracle.rs`: a polar-chart boundary
//! reduction in closed form, written apart from the face integrator), about
//! the same shell reference, and checks the two agree.

#![allow(dead_code, clippy::unwrap_used, clippy::expect_used, clippy::panic)]

#[allow(dead_code, clippy::all, clippy::pedantic, clippy::nursery)]
#[path = "../../src/wrap/volume_oracle.rs"]
mod volume_oracle;

use remus_check::properties::face_cache::integrate_face_volume_about_memoized;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

/// What [`fitted_faces`] read of one face.
#[derive(Debug, Clone, Copy)]
pub struct FittedFace {
    pub face: FaceId,
    /// The term `solid_volume` sums (order 5, about the shell reference).
    pub kernel: f64,
    /// The oracle's term about the same reference.
    pub oracle: f64,
    /// Whether the kernel read the face on its fitted boundary.
    pub on_boundary: bool,
}

fn has_fitted_trim(topo: &Topology, face: FaceId) -> bool {
    let face = topo.face(face).unwrap();
    matches!(
        face.surface(),
        FaceSurface::Cylinder(_) | FaceSurface::Cone(_)
    ) && std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .any(|wire| {
            topo.wire(wire).unwrap().edges().iter().any(|oriented| {
                matches!(
                    topo.edge(oriented.edge()).unwrap().curve(),
                    EdgeCurve::NurbsCurve(_)
                )
            })
        })
}

/// Every cylinder or cone face of `solid` with a fitted trim, read both ways
/// about its shell's reference.
pub fn fitted_faces(topo: &Topology, solid: SolidId) -> Vec<FittedFace> {
    let data = topo.solid(solid).unwrap();
    let mut out = Vec::new();
    for shell in std::iter::once(data.outer_shell()).chain(data.inner_shells().iter().copied()) {
        let faces = topo.shell(shell).unwrap().faces().to_vec();
        let reference = volume_oracle::vertex_box_centre(topo, &faces);
        for &face in &faces {
            if !has_fitted_trim(topo, face) {
                continue;
            }
            let term = integrate_face_volume_about_memoized(topo, face, 5, reference).unwrap();
            out.push(FittedFace {
                face,
                kernel: term.volume,
                oracle: volume_oracle::face_volume(topo, face, reference),
                on_boundary: term.fitted_boundary.is_some(),
            });
        }
    }
    out
}

/// Assert every fitted face of `solid` was read on its boundary and agrees
/// with the oracle within `tolerance` (absolute, in volume units); returns
/// how many there were.
pub fn assert_fitted_faces_match(topo: &Topology, solid: SolidId, tolerance: f64) -> usize {
    let faces = fitted_faces(topo, solid);
    for f in &faces {
        assert!(
            f.on_boundary,
            "face {} was not read on its boundary",
            f.face.index()
        );
        assert!(
            (f.kernel - f.oracle).abs() <= tolerance,
            "face {}: kernel {} vs oracle {} ({:.3e})",
            f.face.index(),
            f.kernel,
            f.oracle,
            f.kernel - f.oracle
        );
    }
    faces.len()
}
