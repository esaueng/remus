//! Regression (B130): every pcurve of an imported STEP body must trace its
//! edge, on import and after any similarity or affine transform.
//!
//! The OCCT-exported fillet plate carries LINE pcurves on four quarter-cylinder
//! bands and LINE or rational B-spline pcurves on every plane.
//!
//! - Import: a band whose LINE origin sat exactly half a turn from one end
//!   lifted that end onto the far periodic branch, so the pcurve swept three
//!   quarters of the cylinder (mid-span deviation 6.0 = the band's diameter).
//! - Transform: `transform_solid` and `copy_and_transform_solid` moved the 3D
//!   geometry but left every pcurve in the old chart. The plane chart is
//!   rebuilt from the moved plane and cylinder frames were rebuilt
//!   canonically, so a pure translation by (10, 0, 0) made `write_step`
//!   refuse a plane pcurve that missed its vertex by 10 (46.5 deviation
//!   under rotation).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_io::step::reader::read_step;
use remus_io::step::writer::write_step;
use remus_math::frame::Frame3;
use remus_math::mat::Mat4;
use remus_math::vec::{Point2, Point3};
use remus_operations::copy::copy_and_transform_solid;
use remus_operations::measure::solid_volume;
use remus_operations::transform::{linear_determinant, transform_solid};
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::{check_same_parameter, validate_boundary_authority};

const FILLETED_PLATE: &str = include_str!("data/openzcad_e_analytic_fillet_plate.step");
const TOLERANCE: f64 = 1e-7;

fn import_plate() -> (Topology, SolidId) {
    let mut topo = Topology::new();
    let solids = read_step(FILLETED_PLATE, &mut topo).expect("import STEP fixture");
    assert_eq!(solids.len(), 1);
    (topo, solids[0])
}

/// The largest sampled `SameParameter` deviation over every boundary use
/// that carries a pcurve on a surface with a UV evaluation, and the number
/// of uses measured.
fn max_same_parameter_deviation(topo: &Topology, solid: SolidId) -> (f64, usize) {
    let mut worst = 0.0_f64;
    let mut measured = 0;
    for face in solid_faces(topo, solid).unwrap() {
        for (edge, forward, _) in topo.pcurves_for_face(face) {
            if let Some(report) = check_same_parameter(topo, edge, face, forward, 64).unwrap() {
                worst = worst.max(report.max_deviation);
                measured += 1;
            }
        }
    }
    (worst, measured)
}

#[test]
fn analytic_fillet_plate_band_pcurves_trace_their_edges_on_import() {
    let (topo, solid) = import_plate();
    let (deviation, measured) = max_same_parameter_deviation(&topo, solid);
    assert_eq!(measured, 16, "four bands, four uses each");
    assert!(
        deviation <= TOLERANCE,
        "imported band pcurves leave their edges by {deviation}"
    );
}

/// A pcurve point lifted through its face's chart. Planes store no frame:
/// their pcurves live in the canonical chart that the `SameParameter`
/// proofs and the STEP reader and writer share.
fn on_surface(surface: &FaceSurface, uv: Point2) -> Point3 {
    match surface {
        FaceSurface::Plane { normal, d } => {
            let foot = *normal * (*d / normal.dot(*normal));
            let chart =
                Frame3::from_normal(Point3::new(foot.x(), foot.y(), foot.z()), *normal).unwrap();
            chart.origin + chart.x * uv.x() + chart.y * uv.y()
        }
        other => other.evaluate(uv.x(), uv.y()).unwrap(),
    }
}

/// Per boundary use in traversal order: the face's carrier family and, when
/// the use carries a pcurve, its surface image at evenly spaced normalized
/// parameters.
fn lifted_pcurves(
    topo: &Topology,
    solid: SolidId,
    undo_reversal: bool,
) -> Vec<(&'static str, Option<Vec<Point3>>)> {
    const SAMPLES: u32 = 32;
    let mut uses = Vec::new();
    for face_id in solid_faces(topo, solid).unwrap() {
        let face = topo.face(face_id).unwrap();
        for &loop_id in face.boundary_loops() {
            let boundary = topo.face_loop(loop_id).unwrap();
            let mut coedges = boundary.coedges().to_vec();
            // A reflection reverses every loop and each use's parameter range,
            // keeping the closed loop's first use as its anchor. Undo those
            // traversal changes only to compare each transformed trace with
            // the same source edge in the same parameter direction.
            if undo_reversal {
                coedges.reverse();
                if boundary.is_closed() {
                    coedges.rotate_right(1);
                }
            }
            for &coedge_id in &coedges {
                let samples = topo.coedge(coedge_id).unwrap().pcurve().map(|pcurve| {
                    (0..=SAMPLES)
                        .map(|k| {
                            let f = f64::from(k) / f64::from(SAMPLES);
                            let f = if undo_reversal { 1.0 - f } else { f };
                            let t = pcurve.t_start() + f * (pcurve.t_end() - pcurve.t_start());
                            on_surface(face.surface(), pcurve.evaluate(t))
                        })
                        .collect()
                });
                uses.push((face.surface().type_tag(), samples));
            }
        }
    }
    uses
}

fn matrices() -> [(&'static str, Mat4); 5] {
    let rotation =
        Mat4::translation(-4.0, 7.0, 2.5) * Mat4::rotation_z(0.7) * Mat4::rotation_x(0.3);
    [
        ("translation", Mat4::translation(10.0, 0.0, 0.0)),
        ("rotation", rotation),
        ("mirror", rotation * Mat4::scale(-1.0, 1.0, 1.0)),
        ("uniform scale", rotation * Mat4::scale(2.5, 2.5, 2.5)),
        ("anisotropic scale", Mat4::scale(2.0, 1.0, 1.0)),
    ]
}

/// Every surviving pcurve traces exactly the image of its original trace, the
/// band uses stay `SameParameter`, and the body exports and re-imports.
#[test]
fn transformed_fillet_plate_pcurves_stay_on_their_edges() {
    let (source, solid) = import_plate();
    let before = lifted_pcurves(&source, solid, false);
    let volume = solid_volume(&source, solid, 0.05).unwrap();
    for (what, matrix) in matrices() {
        for in_place in [true, false] {
            let label = format!("{what} ({})", if in_place { "in place" } else { "copy" });
            let mut topo = source.clone();
            let moved = if in_place {
                transform_solid(&mut topo, solid, &matrix).unwrap();
                solid
            } else {
                copy_and_transform_solid(&mut topo, solid, &matrix).unwrap()
            };
            validate_boundary_authority(&topo).unwrap();

            let after = lifted_pcurves(&topo, moved, linear_determinant(&matrix) < 0.0);
            assert_eq!(after.len(), before.len(), "{label}");
            for (index, ((_, old), (family, new))) in before.iter().zip(&after).enumerate() {
                let old = old.as_ref().unwrap();
                let Some(new) = new else {
                    // Only a carrier converted to NURBS drops its pcurves.
                    assert_eq!(*family, "nurbs", "{label}: use {index} lost its pcurve");
                    continue;
                };
                for (a, b) in old.iter().zip(new) {
                    let expected = matrix.mul_point(*a);
                    assert!(
                        (*b - expected).length() <= 1e-9,
                        "{label}: use {index} traces {b:?}, not {expected:?}"
                    );
                }
            }
            let (deviation, _) = max_same_parameter_deviation(&topo, moved);
            assert!(deviation <= TOLERANCE, "{label}: SameParameter {deviation}");

            let written = write_step(&topo, &[moved])
                .unwrap_or_else(|error| panic!("{label}: write_step refused: {error}"));
            let mut reread = Topology::new();
            let reread_solid = read_step(&written, &mut reread).unwrap()[0];
            let (deviation, _) = max_same_parameter_deviation(&reread, reread_solid);
            assert!(
                deviation <= TOLERANCE,
                "{label}: re-imported SameParameter {deviation}"
            );
            // NURBS-converted bands measure through a mesh, so the anisotropic
            // body is held to its own pre-export volume, not the closed form.
            let exported = solid_volume(&topo, moved, 0.05).unwrap();
            let expected = volume * linear_determinant(&matrix).abs();
            assert!(
                (exported - expected).abs() <= expected * 1e-5,
                "{label}: transformed volume {exported} != {expected}"
            );
            let actual = solid_volume(&reread, reread_solid, 0.05).unwrap();
            assert!(
                (actual - exported).abs() <= exported * 1e-9,
                "{label}: re-imported volume {actual} != {exported}"
            );
        }
    }
}
