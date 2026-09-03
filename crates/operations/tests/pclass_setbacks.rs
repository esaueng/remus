//! P-Class 5.4 exit gates for explicit variable-fillet setbacks.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::HashMap;

use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point3, Vec3};
use remus_operations::blend_ops::blend_failure_code;
use remus_operations::fillet::{FilletEdgeSetback, FilletRadiusLaw, fillet_variable_with_setbacks};
use remus_operations::measure::solid_volume;
use remus_operations::primitives::make_box;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::explorer::{solid_edges, solid_faces};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;
use remus_topology::validation::{validate_shell_closed, validate_shell_manifold};

fn origin_edges(topo: &Topology, solid: SolidId) -> Vec<(EdgeId, bool, Vec3)> {
    let origin = Point3::new(0.0, 0.0, 0.0);
    solid_edges(topo, solid)
        .unwrap()
        .into_iter()
        .filter_map(|edge_id| {
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            if (start - origin).length() < 1e-10 {
                Some((edge_id, true, (end - start).normalize().unwrap()))
            } else if (end - origin).length() < 1e-10 {
                Some((edge_id, false, (start - end).normalize().unwrap()))
            } else {
                None
            }
        })
        .collect()
}

fn assert_watertight_with_mesh_oracle(topo: &Topology, solid: SolidId) {
    let shell = topo
        .shell(topo.solid(solid).unwrap().outer_shell())
        .unwrap();
    validate_shell_closed(shell, topo).unwrap();
    validate_shell_manifold(shell, topo).unwrap();

    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.01, 0.1).unwrap();
    let mut canonical = HashMap::<(i64, i64, i64), u32>::new();
    let mut remap = vec![0_u32; mesh.positions.len()];
    for (index, point) in mesh.positions.iter().enumerate() {
        #[allow(clippy::cast_possible_truncation)]
        let key = (
            (point.x() * 1e7).round() as i64,
            (point.y() * 1e7).round() as i64,
            (point.z() * 1e7).round() as i64,
        );
        #[allow(clippy::cast_possible_truncation)]
        let next = canonical.len() as u32;
        remap[index] = *canonical.entry(key).or_insert(next);
    }

    let mut edge_uses = HashMap::<(u32, u32), usize>::new();
    let mut signed_six_volume = 0.0;
    for triangle in mesh.indices.chunks_exact(3) {
        let points = [
            mesh.positions[triangle[0] as usize],
            mesh.positions[triangle[1] as usize],
            mesh.positions[triangle[2] as usize],
        ];
        signed_six_volume += points[0].x()
            * (points[1].y() * points[2].z() - points[1].z() * points[2].y())
            - points[0].y() * (points[1].x() * points[2].z() - points[1].z() * points[2].x())
            + points[0].z() * (points[1].x() * points[2].y() - points[1].y() * points[2].x());
        let vertices = [
            remap[triangle[0] as usize],
            remap[triangle[1] as usize],
            remap[triangle[2] as usize],
        ];
        for (a, b) in [
            (vertices[0], vertices[1]),
            (vertices[1], vertices[2]),
            (vertices[2], vertices[0]),
        ] {
            let edge = if a < b { (a, b) } else { (b, a) };
            *edge_uses.entry(edge).or_default() += 1;
        }
    }
    assert_eq!(
        edge_uses.values().filter(|&&count| count != 2).count(),
        0,
        "the independently welded display mesh must be watertight"
    );
    let mesh_volume = (signed_six_volume / 6.0).abs();
    let brep_volume = solid_volume(topo, solid, 0.005).unwrap();
    let relative = (mesh_volume - brep_volume).abs() / brep_volume.abs().max(1.0);
    assert!(
        mesh_volume > 0.0 && brep_volume > 0.0 && relative < 0.03,
        "mesh volume {mesh_volume} vs B-Rep {brep_volume} ({:.2}%)",
        relative * 100.0
    );
}

#[test]
fn mixed_laws_with_declared_setbacks_build_one_exact_g1_corner_ball() {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let selected = origin_edges(&topo, input);
    assert_eq!(selected.len(), 3);

    let specs: Vec<_> = selected
        .iter()
        .zip([1.2_f64, 1.4, 1.6])
        .map(|(&(edge, origin_is_start, _), far_radius)| {
            // The S-curve is normalized over the setback stripe, so every
            // incident band reaches radius 1.0 with zero slope at the common
            // corner while carrying a different radius toward its far end.
            FilletEdgeSetback {
                edge,
                law: if origin_is_start {
                    FilletRadiusLaw::SCurve {
                        start: 1.0,
                        end: far_radius,
                    }
                } else {
                    FilletRadiusLaw::SCurve {
                        start: far_radius,
                        end: 1.0,
                    }
                },
                start_setback: if origin_is_start { 1.0 } else { 0.0 },
                end_setback: if origin_is_start { 0.0 } else { 1.0 },
            }
        })
        .collect();

    let result = fillet_variable_with_setbacks(&mut topo, input, &specs).unwrap();
    assert_watertight_with_mesh_oracle(&topo, result);

    let sphere_faces: Vec<_> = solid_faces(&topo, result)
        .unwrap()
        .into_iter()
        .filter(|&face| matches!(topo.face(face).unwrap().surface(), FaceSurface::Sphere(_)))
        .collect();
    assert_eq!(sphere_faces.len(), 1);
    let sphere_face = topo.face(sphere_faces[0]).unwrap();
    let FaceSurface::Sphere(sphere) = sphere_face.surface() else {
        unreachable!();
    };
    assert!((sphere.center() - Point3::new(1.0, 1.0, 1.0)).length() < 1e-10);
    assert!((sphere.radius() - 1.0).abs() < 1e-10);

    let sphere_wire = topo.wire(sphere_face.outer_wire()).unwrap();
    assert_eq!(sphere_wire.edges().len(), 3);
    let mut verified_spines = [false; 3];
    let adjacency = topo.build_adjacency(result).unwrap();
    for oriented in sphere_wire.edges() {
        let edge = topo.edge(oriented.edge()).unwrap();
        assert!(matches!(edge.curve(), EdgeCurve::Circle(_)));
        let start = topo.vertex(edge.start()).unwrap().point();
        let end = topo.vertex(edge.end()).unwrap().point();
        let mut matched = 0;
        let mut matched_spine = None;
        for (index, &(_, _, away)) in selected.iter().enumerate() {
            let start_station = (start - Point3::new(0.0, 0.0, 0.0)).dot(away);
            let end_station = (end - Point3::new(0.0, 0.0, 0.0)).dot(away);
            if (start_station - 1.0).abs() < 1e-10 && (end_station - 1.0).abs() < 1e-10 {
                verified_spines[index] = true;
                matched_spine = Some(index);
                matched += 1;
            }
        }
        assert_eq!(matched, 1, "each cap arc must terminate one declared spine");
        let matched_spine = matched_spine.unwrap();

        let faces = adjacency.faces_for_edge(oriented.edge());
        assert_eq!(faces.len(), 2);
        let tags = [
            topo.face(faces[0]).unwrap().surface().type_tag(),
            topo.face(faces[1]).unwrap().surface().type_tag(),
        ];
        assert!(tags.contains(&"sphere") && tags.contains(&"nurbs"));
        let nurbs_face_id = faces
            .iter()
            .copied()
            .find(|face| matches!(topo.face(*face).unwrap().surface(), FaceSurface::Nurbs(_)))
            .unwrap();
        let nurbs_face = topo.face(nurbs_face_id).unwrap();
        let FaceSurface::Nurbs(surface) = nurbs_face.surface() else {
            unreachable!();
        };
        let boundary_v = if selected[matched_spine].1 { 0.0 } else { 1.0 };
        for fraction in [0.2, 0.5, 0.8] {
            let point = surface.evaluate(fraction, boundary_v);
            let mut band_normal = surface.normal(fraction, boundary_v).unwrap();
            if nurbs_face.is_reversed() {
                band_normal = -band_normal;
            }
            let mut sphere_normal = (point - sphere.center()).normalize().unwrap();
            if sphere_face.is_reversed() {
                sphere_normal = -sphere_normal;
            }
            let angle = band_normal
                .cross(sphere_normal)
                .length()
                .atan2(band_normal.dot(sphere_normal));
            assert!(
                angle <= Tolerance::new().angular,
                "G1 seam angle {angle} at fraction {fraction}"
            );
        }
    }
    assert!(verified_spines.into_iter().all(|verified| verified));
}

#[test]
fn inconsistent_declared_setback_refuses_transactionally() {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let selected = origin_edges(&topo, input);
    assert_eq!(selected.len(), 3);
    let specs: Vec<_> = selected
        .iter()
        .enumerate()
        .map(|(index, &(edge, origin_is_start, _))| {
            let setback = if index == 0 { 0.8 } else { 1.0 };
            FilletEdgeSetback {
                edge,
                law: FilletRadiusLaw::Constant(1.0),
                start_setback: if origin_is_start { setback } else { 0.0 },
                end_setback: if origin_is_start { 0.0 } else { setback },
            }
        })
        .collect();
    let before = (
        topo.num_vertices(),
        topo.num_edges(),
        topo.num_wires(),
        topo.num_faces(),
        topo.num_shells(),
        topo.num_solids(),
    );
    let error = fillet_variable_with_setbacks(&mut topo, input, &specs).unwrap_err();
    assert_eq!(blend_failure_code(&error), "setback-mismatch");
    assert_eq!(
        before,
        (
            topo.num_vertices(),
            topo.num_edges(),
            topo.num_wires(),
            topo.num_faces(),
            topo.num_shells(),
            topo.num_solids(),
        )
    );
}

#[test]
fn nonstationary_corner_law_refuses_instead_of_building_a_g0_cap() {
    let mut topo = Topology::new();
    let input = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
    let selected = origin_edges(&topo, input);
    let specs: Vec<_> = selected
        .iter()
        .enumerate()
        .map(|(index, &(edge, origin_is_start, _))| FilletEdgeSetback {
            edge,
            law: if index == 0 {
                if origin_is_start {
                    FilletRadiusLaw::Linear {
                        start: 1.0,
                        end: 1.2,
                    }
                } else {
                    FilletRadiusLaw::Linear {
                        start: 1.2,
                        end: 1.0,
                    }
                }
            } else {
                FilletRadiusLaw::Constant(1.0)
            },
            start_setback: if origin_is_start { 1.0 } else { 0.0 },
            end_setback: if origin_is_start { 0.0 } else { 1.0 },
        })
        .collect();
    let before = topo.num_solids();
    let error = fillet_variable_with_setbacks(&mut topo, input, &specs).unwrap_err();
    assert_eq!(blend_failure_code(&error), "unsupported-setback-corner");
    assert_eq!(topo.num_solids(), before);
}
