//! Exact adapter for cylindrical blend bands terminated by affine NURBS caps.
//!
//! The ordinary blend remover intentionally accepts only analytic support and
//! termination surfaces.  This adapter recognizes one narrow freeform case:
//! a degree-one, equal-weight 2x2 NURBS cap whose complete control net proves
//! an affine bounded plane.  It substitutes a plane only on a private copy,
//! invokes the existing exact planar reconstruction, then restores the exact
//! original NURBS carrier and authoritative affine p-curves before publishing
//! the result.

use std::collections::{BTreeMap, BTreeSet};

use remus_geometry::convert::{CertifiedAffinePlane, certify_affine_nurbs_plane};
use remus_math::curves2d::{Curve2D, Line2D};
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::tolerance::Tolerance;
use remus_math::vec::{Point2, Vec2};
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::journal::{EntityKey, EntityKind};
use remus_topology::pcurve::PCurve;
use remus_topology::solid::SolidId;

use crate::OperationsError;
use crate::defeature::DefeatureOutcome;
use crate::resize_blend::ResizeBlendError;

#[derive(Clone)]
struct AffineCap {
    face: FaceId,
    surface: NurbsSurface,
    certificate: CertifiedAffinePlane,
}

struct AffineCapPlan {
    band: FaceId,
    caps: Vec<AffineCap>,
}

fn reconstruction(reason: impl Into<String>) -> OperationsError {
    ResizeBlendError::ReconstructionFailed {
        reason: reason.into(),
    }
    .into()
}

fn face_edges(topo: &Topology, face: FaceId) -> Result<Vec<EdgeId>, OperationsError> {
    let face = topo.face(face)?;
    let mut edges = Vec::new();
    for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        edges.extend(
            topo.wire(wire)?
                .edges()
                .iter()
                .map(remus_topology::wire::OrientedEdge::edge),
        );
    }
    edges.sort_unstable_by_key(|edge| edge.index());
    edges.dedup();
    Ok(edges)
}

fn other_face(
    adjacency: &remus_topology::adjacency::AdjacencyIndex,
    edge: EdgeId,
    band: FaceId,
) -> Option<FaceId> {
    let mut others: Vec<_> = adjacency
        .faces_for_edge(edge)
        .iter()
        .copied()
        .filter(|face| *face != band)
        .collect();
    others.sort_unstable_by_key(|face| face.index());
    others.dedup();
    let [other] = others.as_slice() else {
        return None;
    };
    Some(*other)
}

fn certify_source_line(
    topo: &Topology,
    edge_id: EdgeId,
    certificate: &CertifiedAffinePlane,
) -> Result<bool, OperationsError> {
    let edge = topo.edge(edge_id)?;
    if !matches!(edge.curve(), EdgeCurve::Line) {
        return Ok(false);
    }
    let start = topo.vertex(edge.start())?.point();
    let end = topo.vertex(edge.end())?.point();
    let tolerance = Tolerance::new().linear;
    let Some((u0, v0)) = certificate.parameters(start, tolerance) else {
        return Ok(false);
    };
    let Some((u1, v1)) = certificate.parameters(end, tolerance) else {
        return Ok(false);
    };
    // The original domain is a rectangle and affine parameters trace a line,
    // so endpoint membership proves the complete segment lies in the domain.
    let midpoint = start + (end - start) * 0.5;
    Ok(certificate.parameters(midpoint, tolerance).is_some()
        && (u1 - u0).hypot(v1 - v0) > 64.0 * f64::EPSILON)
}

fn certify_cross_trim(
    topo: &Topology,
    edge_id: EdgeId,
    certificate: &CertifiedAffinePlane,
) -> Result<bool, OperationsError> {
    let edge = topo.edge(edge_id)?;
    let EdgeCurve::Circle(circle) = edge.curve() else {
        return Ok(false);
    };
    edge.strict_domain().map_err(|error| {
        reconstruction(format!(
            "affine NURBS cap cross edge {} lacks an authoritative trim: {error}",
            edge_id.index()
        ))
    })?;
    let tolerance = Tolerance::new().linear;
    if 1.0 - circle.normal().dot(certificate.normal()).abs() > Tolerance::new().angular {
        return Ok(false);
    }
    let Some(center) = certificate.parameters(circle.center(), tolerance) else {
        return Ok(false);
    };
    let Some(axis_u) = certificate.parameters(circle.evaluate(0.0), tolerance) else {
        return Ok(false);
    };
    let Some(axis_v) =
        certificate.parameters(circle.evaluate(std::f64::consts::FRAC_PI_2), tolerance)
    else {
        return Ok(false);
    };
    let coefficients = [
        (axis_u.0 - center.0, axis_v.0 - center.0),
        (axis_u.1 - center.1, axis_v.1 - center.1),
    ];
    // Each affine UV coordinate of the 3D circle is A*cos(t)+B*sin(t).
    // Its only extrema are phase and phase+pi. Proving all four extrema are
    // in the rectangular NURBS domain proves the complete trim is in-domain;
    // checking the full carrier is deliberately stronger than arc sampling.
    Ok(coefficients.into_iter().all(|(cosine, sine)| {
        let phase = sine.atan2(cosine);
        [phase, phase + std::f64::consts::PI]
            .into_iter()
            .all(|parameter| {
                certificate
                    .parameters(circle.evaluate(parameter), tolerance)
                    .is_some()
            })
    }))
}

fn classify_affine_caps(
    topo: &Topology,
    solid: SolidId,
    band: FaceId,
    supports: [FaceId; 2],
) -> Result<Option<AffineCapPlan>, OperationsError> {
    if supports[0] == supports[1] || !matches!(topo.face(band)?.surface(), FaceSurface::Cylinder(_))
    {
        return Ok(None);
    }
    if !topo.face(band)?.inner_wires().is_empty()
        || supports
            .iter()
            .any(|support| !matches!(topo.face(*support), Ok(face) if matches!(face.surface(), FaceSurface::Plane { .. })))
    {
        return Ok(None);
    }
    let solid_faces: BTreeSet<_> = remus_topology::explorer::solid_faces(topo, solid)?
        .into_iter()
        .collect();
    if !solid_faces.contains(&band) || supports.iter().any(|face| !solid_faces.contains(face)) {
        return Ok(None);
    }

    let adjacency = topo.build_adjacency(solid)?;
    let mut support_contacts = BTreeMap::<FaceId, Vec<EdgeId>>::new();
    let mut cap_contacts = BTreeMap::<FaceId, Vec<EdgeId>>::new();
    for edge in face_edges(topo, band)? {
        let Some(other) = other_face(&adjacency, edge, band) else {
            return Ok(None);
        };
        if supports.contains(&other) {
            support_contacts.entry(other).or_default().push(edge);
        } else {
            cap_contacts.entry(other).or_default().push(edge);
        }
    }
    if supports.iter().any(|support| {
        support_contacts
            .get(support)
            .is_none_or(|edges| edges.len() != 1)
    }) || cap_contacts.len() != 2
        || cap_contacts.values().any(|edges| edges.len() != 1)
    {
        return Ok(None);
    }

    let mut cap_faces: Vec<_> = cap_contacts.keys().copied().collect();
    cap_faces.sort_unstable_by_key(|face| face.index());
    let mut caps = Vec::new();
    for cap in cap_faces {
        if !solid_faces.contains(&cap) || !topo.face(cap)?.inner_wires().is_empty() {
            return Ok(None);
        }
        let FaceSurface::Nurbs(surface) = topo.face(cap)?.surface() else {
            if !matches!(topo.face(cap)?.surface(), FaceSurface::Plane { .. }) {
                return Ok(None);
            }
            continue;
        };
        let Some(certificate) = certify_affine_nurbs_plane(surface, Tolerance::new().linear) else {
            return Ok(None);
        };
        let cross = cap_contacts[&cap][0];
        if !certify_cross_trim(topo, cross, &certificate)? {
            return Ok(None);
        }
        for edge in face_edges(topo, cap)? {
            if edge != cross && !certify_source_line(topo, edge, &certificate)? {
                return Ok(None);
            }
        }
        caps.push(AffineCap {
            face: cap,
            surface: surface.clone(),
            certificate,
        });
    }
    if caps.is_empty() {
        return Ok(None);
    }
    Ok(Some(AffineCapPlan { band, caps }))
}

fn set_affine_line_pcurves(
    topo: &mut Topology,
    face_id: FaceId,
    certificate: &CertifiedAffinePlane,
) -> Result<(), OperationsError> {
    let uses = std::iter::once(topo.face(face_id)?.outer_wire())
        .chain(topo.face(face_id)?.inner_wires().iter().copied())
        .map(|wire| topo.wire(wire).map(|wire| wire.edges().to_vec()))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();

    for oriented in uses {
        let edge_id = oriented.edge();
        let edge = topo.edge(edge_id)?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            return Err(reconstruction(format!(
                "restored affine NURBS cap {} retains curved boundary edge {}",
                face_id.index(),
                edge_id.index()
            )));
        }
        let start = topo.vertex(edge.start())?.point();
        let end = topo.vertex(edge.end())?.point();
        let tolerance = Tolerance::new().linear;
        let (u0, v0) = certificate.parameters(start, tolerance).ok_or_else(|| {
            reconstruction(format!(
                "edge {} start lies outside restored affine NURBS cap {}",
                edge_id.index(),
                face_id.index()
            ))
        })?;
        let (u1, v1) = certificate.parameters(end, tolerance).ok_or_else(|| {
            reconstruction(format!(
                "edge {} end lies outside restored affine NURBS cap {}",
                edge_id.index(),
                face_id.index()
            ))
        })?;
        let origin = Point2::new(u0, v0);
        let direction = Vec2::new(u1 - u0, v1 - v0);
        let length = direction.length();
        if !length.is_finite() || length <= 64.0 * f64::EPSILON {
            return Err(reconstruction(format!(
                "edge {} collapses in affine NURBS parameters",
                edge_id.index()
            )));
        }
        let midpoint = start + (end - start) * 0.5;
        if certificate.parameters(midpoint, tolerance).is_none() {
            return Err(reconstruction(format!(
                "edge {} trim leaves the bounded affine NURBS cap",
                edge_id.index()
            )));
        }
        let curve = Curve2D::Line(Line2D::new(origin, direction)?);
        let (t_start, t_end) = if oriented.is_forward() {
            (0.0, length)
        } else {
            (length, 0.0)
        };
        topo.set_pcurve_oriented(
            edge_id,
            face_id,
            oriented.is_forward(),
            PCurve::new(curve, t_start, t_end),
        )?;
        remus_topology::validation::validate_same_parameter(
            topo,
            edge_id,
            face_id,
            oriented.is_forward(),
            tolerance,
            65,
        )?;
    }
    Ok(())
}

fn take_face_pcurves(
    topo: &mut Topology,
    face_id: FaceId,
) -> Result<Vec<(EdgeId, bool, Option<PCurve>)>, OperationsError> {
    let uses = std::iter::once(topo.face(face_id)?.outer_wire())
        .chain(topo.face(face_id)?.inner_wires().iter().copied())
        .map(|wire| topo.wire(wire).map(|wire| wire.edges().to_vec()))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    let mut saved = Vec::with_capacity(uses.len());
    for oriented in uses {
        let pcurve =
            topo.remove_pcurve_oriented(oriented.edge(), face_id, oriented.is_forward())?;
        saved.push((oriented.edge(), oriented.is_forward(), pcurve));
    }
    Ok(saved)
}

fn restore_face_pcurves(
    topo: &mut Topology,
    face_id: FaceId,
    saved: Vec<(EdgeId, bool, Option<PCurve>)>,
) -> Result<(), OperationsError> {
    for (edge, forward, pcurve) in saved {
        if let Some(pcurve) = pcurve {
            topo.set_pcurve_oriented(edge, face_id, forward, pcurve)?;
        }
    }
    Ok(())
}

fn compose_boundary_history(
    copied_edges: &BTreeMap<usize, EdgeId>,
    copied_vertices: &BTreeMap<usize, remus_topology::vertex::VertexId>,
    history: Vec<(EntityKey, Option<EntityKey>)>,
) -> Result<Vec<(EntityKey, Option<EntityKey>)>, OperationsError> {
    let edge_sources: BTreeMap<_, _> = copied_edges
        .iter()
        .map(|(&source, copied)| (copied.index(), source))
        .collect();
    let vertex_sources: BTreeMap<_, _> = copied_vertices
        .iter()
        .map(|(&source, copied)| (copied.index(), source))
        .collect();
    let mut composed = Vec::with_capacity(history.len());
    for (source, target) in history {
        let original = match source.kind {
            EntityKind::Edge => edge_sources
                .get(&source.index)
                .copied()
                .map(EntityKey::edge),
            EntityKind::Vertex => vertex_sources
                .get(&source.index)
                .copied()
                .map(EntityKey::vertex),
            EntityKind::Face => None,
        }
        .ok_or_else(|| {
            reconstruction(format!(
                "proxy blend removal reported unmapped {:?} source {}",
                source.kind, source.index
            ))
        })?;
        composed.push((original, target));
    }
    composed.sort_unstable();
    composed.dedup();
    Ok(composed)
}

fn execute(
    topo: &mut Topology,
    solid: SolidId,
    supports: [FaceId; 2],
    plan: AffineCapPlan,
) -> Result<DefeatureOutcome, OperationsError> {
    let copied = crate::copy::copy_solid_with_entity_map(topo, solid)?;
    let copied_face = |source: FaceId| {
        copied
            .face_map
            .get(&source.index())
            .copied()
            .ok_or_else(|| reconstruction(format!("face {} was not copied", source.index())))
    };
    let copied_band = copied_face(plan.band)?;
    let _copied_supports = [copied_face(supports[0])?, copied_face(supports[1])?];
    let mut proxy_caps = Vec::with_capacity(plan.caps.len());
    for cap in &plan.caps {
        let copied_cap = copied_face(cap.face)?;
        let pcurves = take_face_pcurves(topo, copied_cap)?;
        topo.face_mut(copied_cap)?.set_surface(FaceSurface::Plane {
            normal: cap.certificate.normal(),
            d: cap.certificate.offset(),
        });
        proxy_caps.push((copied_cap, cap, pcurves));
    }

    let reconstructed =
        crate::resize_blend::remove_recognized_blend_faces(topo, copied.solid, &[copied_band])?;
    for (copied_cap, cap, _) in &proxy_caps {
        let result_cap = reconstructed
            .face_map
            .get(&copied_cap.index())
            .copied()
            .ok_or_else(|| {
                reconstruction(format!(
                    "affine NURBS cap {} has no reconstructed face",
                    cap.face.index()
                ))
            })?;
        topo.face_mut(result_cap)?
            .set_surface(FaceSurface::Nurbs(cap.surface.clone()));
        set_affine_line_pcurves(topo, result_cap, &cap.certificate)?;
        // The private intermediate is live in the arena until compaction.
        // Restore it too so no temporary proxy carrier can escape inspection.
        topo.face_mut(*copied_cap)?
            .set_surface(FaceSurface::Nurbs(cap.surface.clone()));
    }
    for (copied_cap, _, pcurves) in proxy_caps {
        restore_face_pcurves(topo, copied_cap, pcurves)?;
    }

    let mut ordered_face_map = BTreeMap::new();
    for (&source, copied_face) in &copied.face_map {
        if source == plan.band.index() {
            continue;
        }
        let target = reconstructed
            .face_map
            .get(&copied_face.index())
            .copied()
            .ok_or_else(|| {
                reconstruction(format!("copied face {source} lost construction history"))
            })?;
        ordered_face_map.insert(source, target);
    }
    let expected_sources: BTreeSet<_> = remus_topology::explorer::solid_faces(topo, solid)?
        .into_iter()
        .filter(|face| *face != plan.band)
        .map(remus_topology::arena::Id::index)
        .collect();
    let result_faces: BTreeSet<_> =
        remus_topology::explorer::solid_faces(topo, reconstructed.solid)?
            .into_iter()
            .collect();
    if ordered_face_map.keys().copied().collect::<BTreeSet<_>>() != expected_sources
        || ordered_face_map.values().copied().collect::<BTreeSet<_>>() != result_faces
    {
        return Err(reconstruction(
            "affine NURBS cap reconstruction face history is not a complete bijection",
        ));
    }
    let copied_edges = copied
        .edge_map
        .iter()
        .map(|(&source, &target)| (source, target))
        .collect();
    let copied_vertices = copied
        .vertex_map
        .iter()
        .map(|(&source, &target)| (source, target))
        .collect();
    let boundary_history = compose_boundary_history(
        &copied_edges,
        &copied_vertices,
        reconstructed.boundary_history,
    )?;
    let report = crate::validate::validate_solid(topo, reconstructed.solid)?;
    if !report.is_valid() {
        return Err(reconstruction(format!(
            "restored affine NURBS cap result failed validation with {} error(s)",
            report.error_count()
        )));
    }
    Ok(DefeatureOutcome {
        solid: reconstructed.solid,
        face_map: ordered_face_map.into_iter().collect(),
        boundary_history: Some(boundary_history),
    })
}

/// Heal one cylindrical blend strip between two true planar supports when at
/// least one end cap is a certified bounded affine NURBS plane.
///
/// Classification is read-only and returns `Ok(None)` outside this exact
/// family. Once recognized, all copying, proxy substitution, reconstruction,
/// and carrier restoration are atomic.
#[allow(clippy::redundant_pub_crate)]
pub(crate) fn heal_cylinder_plane_band_affine_cap(
    topo: &mut Topology,
    solid: SolidId,
    band_face: FaceId,
    supports: [FaceId; 2],
) -> Result<Option<DefeatureOutcome>, OperationsError> {
    let Some(plan) = classify_affine_caps(topo, solid, band_face, supports)? else {
        return Ok(None);
    };
    remus_topology::transaction::run_transacted(topo, |topo| {
        execute(topo, solid, supports, plan).map(Some)
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

    use remus_math::curves2d::Circle2D;
    use remus_math::vec::Vec3;
    use remus_topology::explorer::{solid_edges, solid_entity_counts, solid_faces, solid_vertices};
    use remus_topology::journal::EntityKind;

    use super::*;

    struct Fixture {
        topo: Topology,
        solid: SolidId,
        band: FaceId,
        supports: [FaceId; 2],
        cap: FaceId,
        cross: EdgeId,
    }

    fn fixture() -> Fixture {
        let mut topo = Topology::new();
        let sharp = crate::primitives::make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let seed = solid_edges(&topo, sharp).unwrap()[0];
        let solid = crate::blend_ops::fillet_v2(&mut topo, sharp, &[seed], 1.0)
            .unwrap()
            .solid;
        let band = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|face| {
                matches!(
                    topo.face(*face).unwrap().surface(),
                    FaceSurface::Cylinder(_)
                )
            })
            .expect("one cylinder band");
        let adjacency = topo.build_adjacency(solid).unwrap();
        let mut supports = Vec::new();
        let mut caps = Vec::new();
        for edge in face_edges(&topo, band).unwrap() {
            let other = other_face(&adjacency, edge, band).expect("manifold band edge");
            match topo.edge(edge).unwrap().curve() {
                EdgeCurve::Line => supports.push(other),
                EdgeCurve::Circle(_) => caps.push((other, edge)),
                curve => panic!("unexpected band boundary {}", curve.type_tag()),
            }
        }
        supports.sort_unstable_by_key(|face| face.index());
        supports.dedup();
        caps.sort_unstable_by_key(|(face, _)| face.index());
        caps.dedup_by_key(|(face, _)| face.index());
        assert_eq!(supports.len(), 2);
        assert_eq!(caps.len(), 2);
        Fixture {
            topo,
            solid,
            band,
            supports: [supports[0], supports[1]],
            cap: caps[0].0,
            cross: caps[0].1,
        }
    }

    fn affine_patch(topo: &Topology, cross: EdgeId, half_span: f64) -> NurbsSurface {
        let EdgeCurve::Circle(circle) = topo.edge(cross).unwrap().curve() else {
            panic!("cross edge is not circular");
        };
        let center = circle.center();
        let u = circle.u_axis() * (2.0 * half_span);
        let v = circle.v_axis() * (2.0 * half_span);
        let p00 = center - u * 0.5 - v * 0.5;
        let p10 = p00 + u;
        let p01 = p00 + v;
        // This fixture uses axis-aligned, binary-exact box/circle data. Build
        // the closure from the same affine additions the certificate proves.
        let p11 = p10 + v;
        NurbsSurface::new(
            1,
            1,
            vec![2.0, 2.0, 8.0, 8.0],
            vec![-3.0, -3.0, 3.0, 3.0],
            vec![vec![p00, p01], vec![p10, p11]],
            vec![vec![3.0; 2]; 2],
        )
        .unwrap()
    }

    fn install_source_pcurves(
        topo: &mut Topology,
        face: FaceId,
        certificate: &CertifiedAffinePlane,
    ) {
        let uses = std::iter::once(topo.face(face).unwrap().outer_wire())
            .chain(topo.face(face).unwrap().inner_wires().iter().copied())
            .flat_map(|wire| topo.wire(wire).unwrap().edges().to_vec())
            .collect::<Vec<_>>();
        for oriented in uses {
            let edge_id = oriented.edge();
            let edge = topo.edge(edge_id).unwrap();
            let start = topo.vertex(edge.start()).unwrap().point();
            let end = topo.vertex(edge.end()).unwrap().point();
            let (curve, natural) = match edge.curve() {
                EdgeCurve::Line => {
                    let uv0 = certificate
                        .parameters(start, Tolerance::new().linear)
                        .unwrap();
                    let uv1 = certificate
                        .parameters(end, Tolerance::new().linear)
                        .unwrap();
                    let delta = Vec2::new(uv1.0 - uv0.0, uv1.1 - uv0.1);
                    let length = delta.length();
                    (
                        Curve2D::Line(Line2D::new(Point2::new(uv0.0, uv0.1), delta).unwrap()),
                        (0.0, length),
                    )
                }
                EdgeCurve::Circle(circle) => {
                    let center = certificate
                        .parameters(circle.center(), Tolerance::new().linear)
                        .unwrap();
                    let axis = certificate
                        .parameters(circle.evaluate(0.0), Tolerance::new().linear)
                        .unwrap();
                    let quarter = certificate
                        .parameters(
                            circle.evaluate(std::f64::consts::FRAC_PI_2),
                            Tolerance::new().linear,
                        )
                        .unwrap();
                    let radius_u = (axis.0 - center.0).abs();
                    let radius_v = (quarter.1 - center.1).abs();
                    assert!((axis.1 - center.1).abs() <= 1e-12);
                    assert!((quarter.0 - center.0).abs() <= 1e-12);
                    assert!((radius_u - radius_v).abs() <= 1e-12);
                    (
                        Curve2D::Circle(
                            Circle2D::new(Point2::new(center.0, center.1), radius_u).unwrap(),
                        ),
                        edge.strict_domain().unwrap(),
                    )
                }
                curve => panic!("unsupported source cap edge {}", curve.type_tag()),
            };
            let range = if oriented.is_forward() {
                natural
            } else {
                (natural.1, natural.0)
            };
            topo.set_pcurve_oriented(
                edge_id,
                face,
                oriented.is_forward(),
                PCurve::new(curve, range.0, range.1),
            )
            .unwrap();
            remus_topology::validation::validate_same_parameter(
                topo,
                edge_id,
                face,
                oriented.is_forward(),
                Tolerance::new().linear,
                65,
            )
            .unwrap();
        }
    }

    fn install_affine_cap(fixture: &mut Fixture, surface: NurbsSurface) -> NurbsSurface {
        let original = surface.clone();
        let certificate = certify_affine_nurbs_plane(&surface, Tolerance::new().linear).unwrap();
        fixture
            .topo
            .face_mut(fixture.cap)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(surface));
        install_source_pcurves(&mut fixture.topo, fixture.cap, &certificate);
        original
    }

    fn mapped_face(outcome: &DefeatureOutcome, source: FaceId) -> FaceId {
        outcome.face_map[&source.index()]
    }

    #[test]
    fn public_removal_restores_exact_nurbs_carrier_and_sharp_box() {
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 16.0);
        let expected = install_affine_cap(&mut fixture, patch);
        let result = crate::resize_blend::resize_blend(
            &mut fixture.topo,
            fixture.solid,
            fixture.band,
            1.0,
            0.0,
        )
        .unwrap();
        let [cap_index] = result.evolution.modified[&fixture.cap.index()].as_slice() else {
            panic!("cap has one exact successor");
        };
        let result_cap = fixture.topo.face_id_from_index(*cap_index).unwrap();
        let FaceSurface::Nurbs(actual) = fixture.topo.face(result_cap).unwrap().surface() else {
            panic!("published cap lost its NURBS carrier");
        };
        assert_eq!(actual, &expected);
        assert_eq!(
            solid_entity_counts(&fixture.topo, result.solid).unwrap(),
            (6, 12, 8)
        );
        let volume = crate::measure::solid_volume(&fixture.topo, result.solid, 0.01).unwrap();
        assert!((volume - 1000.0).abs() <= 1e-7);
        for edge in face_edges(&fixture.topo, result_cap).unwrap() {
            assert!(matches!(
                fixture.topo.edge(edge).unwrap().curve(),
                EdgeCurve::Line
            ));
            let oriented = fixture
                .topo
                .wire(fixture.topo.face(result_cap).unwrap().outer_wire())
                .unwrap()
                .edges()
                .iter()
                .find(|use_| use_.edge() == edge)
                .unwrap();
            assert!(
                fixture
                    .topo
                    .pcurve_oriented(edge, result_cap, oriented.is_forward())
                    .is_some()
            );
        }
    }

    #[test]
    fn helper_composes_total_history_and_step_round_trips() {
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 16.0);
        let expected = install_affine_cap(&mut fixture, patch);
        let outcome = heal_cylinder_plane_band_affine_cap(
            &mut fixture.topo,
            fixture.solid,
            fixture.band,
            fixture.supports,
        )
        .unwrap()
        .expect("qualified affine cap");
        let cap = mapped_face(&outcome, fixture.cap);
        assert!(matches!(
            fixture.topo.face(cap).unwrap().surface(),
            FaceSurface::Nurbs(surface) if surface == &expected
        ));
        let history = outcome.boundary_history.as_ref().unwrap();
        let source: BTreeSet<_> = history.iter().map(|(source, _)| *source).collect();
        let expected_source: BTreeSet<_> = solid_edges(&fixture.topo, fixture.solid)
            .unwrap()
            .into_iter()
            .map(|edge| EntityKey::edge(edge.index()))
            .chain(
                solid_vertices(&fixture.topo, fixture.solid)
                    .unwrap()
                    .into_iter()
                    .map(|vertex| EntityKey::vertex(vertex.index())),
            )
            .collect();
        assert_eq!(source, expected_source);
        let targets: BTreeSet<_> = history.iter().filter_map(|(_, target)| *target).collect();
        let expected_targets: BTreeSet<_> = solid_edges(&fixture.topo, outcome.solid)
            .unwrap()
            .into_iter()
            .map(|edge| EntityKey::edge(edge.index()))
            .chain(
                solid_vertices(&fixture.topo, outcome.solid)
                    .unwrap()
                    .into_iter()
                    .map(|vertex| EntityKey::vertex(vertex.index())),
            )
            .collect();
        assert_eq!(targets, expected_targets);
        assert!(history.iter().all(|(source, target)| {
            target
                .is_none_or(|target| source.kind == target.kind && source.kind != EntityKind::Face)
        }));

        let step = remus_io::step::write_step(&fixture.topo, &[outcome.solid]).unwrap();
        let mut imported = Topology::new();
        let reread = remus_io::step::read_step(&step, &mut imported).unwrap()[0];
        let report = crate::validate::validate_solid(&imported, reread).unwrap();
        assert!(report.is_valid());
        assert_eq!(solid_entity_counts(&imported, reread).unwrap(), (6, 12, 8));
        assert!(
            solid_faces(&imported, reread)
                .unwrap()
                .into_iter()
                .any(|face| {
                    matches!(
                        imported.face(face).unwrap().surface(),
                        FaceSurface::Nurbs(surface) if surface == &expected
                    )
                })
        );
    }

    #[test]
    fn journaled_public_removal_records_one_atomic_exact_entry() {
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 16.0);
        let expected = install_affine_cap(&mut fixture, patch);
        let result = crate::journal_ops::resize_blend_journaled(
            &mut fixture.topo,
            fixture.solid,
            fixture.band,
            1.0,
            0.0,
        )
        .unwrap();
        assert!(result.map.origin.is_exact());
        assert!(result.map.is_complete());
        assert!(result.map.deleted.contains(&fixture.band.index()));
        assert_eq!(fixture.topo.journal().entries().len(), 1);
        let [cap_index] = result.map.modified[&fixture.cap.index()].as_slice() else {
            panic!("NURBS cap has one journaled successor");
        };
        let cap = fixture.topo.face_id_from_index(*cap_index).unwrap();
        assert!(matches!(
            fixture.topo.face(cap).unwrap().surface(),
            FaceSurface::Nurbs(surface) if surface == &expected
        ));
    }

    #[test]
    fn curved_and_too_narrow_caps_refuse_without_mutation() {
        let mut curved = fixture();
        let valid = affine_patch(&curved.topo, curved.cross, 16.0);
        let mut points = valid.control_points().to_vec();
        points[1][1] = points[1][1] + Vec3::new(0.0, 0.0, 1e-4);
        let twisted = NurbsSurface::new(
            valid.degree_u(),
            valid.degree_v(),
            valid.knots_u().to_vec(),
            valid.knots_v().to_vec(),
            points,
            valid.weights().to_vec(),
        )
        .unwrap();
        curved
            .topo
            .face_mut(curved.cap)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(twisted.clone()));
        let before = curved.topo.allocated_slot_count();
        assert!(
            heal_cylinder_plane_band_affine_cap(
                &mut curved.topo,
                curved.solid,
                curved.band,
                curved.supports,
            )
            .unwrap()
            .is_none()
        );
        assert_eq!(curved.topo.allocated_slot_count(), before);
        assert!(matches!(
            curved.topo.face(curved.cap).unwrap().surface(),
            FaceSurface::Nurbs(surface) if surface == &twisted
        ));

        let mut narrow = fixture();
        let patch = affine_patch(&narrow.topo, narrow.cross, 0.25);
        narrow
            .topo
            .face_mut(narrow.cap)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(patch.clone()));
        let before = narrow.topo.allocated_slot_count();
        assert!(
            heal_cylinder_plane_band_affine_cap(
                &mut narrow.topo,
                narrow.solid,
                narrow.band,
                narrow.supports,
            )
            .unwrap()
            .is_none()
        );
        assert_eq!(narrow.topo.allocated_slot_count(), before);
        assert!(matches!(
            narrow.topo.face(narrow.cap).unwrap().surface(),
            FaceSurface::Nurbs(surface) if surface == &patch
        ));
    }

    #[test]
    fn failed_public_fallback_rolls_back_affine_cap_input() {
        let mut fixture = fixture();
        let valid = affine_patch(&fixture.topo, fixture.cross, 16.0);
        let mut points = valid.control_points().to_vec();
        points[1][1] = points[1][1] + Vec3::new(0.0, 0.0, 1e-3);
        let twisted = NurbsSurface::new(
            1,
            1,
            valid.knots_u().to_vec(),
            valid.knots_v().to_vec(),
            points,
            valid.weights().to_vec(),
        )
        .unwrap();
        fixture
            .topo
            .face_mut(fixture.cap)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(twisted.clone()));
        let before_counts = solid_entity_counts(&fixture.topo, fixture.solid).unwrap();
        assert!(
            crate::resize_blend::resize_blend(
                &mut fixture.topo,
                fixture.solid,
                fixture.band,
                1.0,
                0.0,
            )
            .is_err()
        );
        assert_eq!(
            solid_entity_counts(&fixture.topo, fixture.solid).unwrap(),
            before_counts
        );
        assert!(matches!(
            fixture.topo.face(fixture.cap).unwrap().surface(),
            FaceSurface::Nurbs(surface) if surface == &twisted
        ));
    }

    #[test]
    fn translated_small_twist_never_qualifies() {
        let fixture = fixture();
        let valid = affine_patch(&fixture.topo, fixture.cross, 16.0);
        let translation = Vec3::new(1.0e12, -1.0e12, 1.0e12);
        let mut points = valid.control_points().to_vec();
        for point in points.iter_mut().flatten() {
            *point = *point + translation;
        }
        points[1][1] = points[1][1] + Vec3::new(0.0, 0.0, 1.0e-3);
        let translated_twist = NurbsSurface::new(
            1,
            1,
            valid.knots_u().to_vec(),
            valid.knots_v().to_vec(),
            points,
            valid.weights().to_vec(),
        )
        .unwrap();
        assert!(certify_affine_nurbs_plane(&translated_twist, Tolerance::new().linear).is_none());
    }

    // ---- B19 survivor tranche.  Each patch below is built from explicit
    // control points in the cap plane, so which boundary lies inside its
    // bounded domain is known by construction, not from the code under test.

    /// Degree-one 2x2 patch spanning `origin + [0,1]u + [0,1]v`, with the
    /// same shifted parameter domain as `affine_patch`.
    fn affine_patch_frame(origin: Point3, u: Vec3, v: Vec3) -> NurbsSurface {
        // Snap to a 2^-20 grid: the corner sums below are then exact, which
        // the certificate's error-free mixed-coefficient proof requires.  The
        // snap moves a side by < 1e-6, far inside every margin used here.
        let snap = |value: f64| (value * 1_048_576.0).round() / 1_048_576.0;
        let origin = Point3::new(snap(origin.x()), snap(origin.y()), snap(origin.z()));
        let u = Vec3::new(snap(u.x()), snap(u.y()), snap(u.z()));
        let v = Vec3::new(snap(v.x()), snap(v.y()), snap(v.z()));
        let p00 = origin;
        let p10 = origin + u;
        let p01 = origin + v;
        let p11 = p10 + v;
        NurbsSurface::new(
            1,
            1,
            vec![2.0, 2.0, 8.0, 8.0],
            vec![-3.0, -3.0, 3.0, 3.0],
            vec![vec![p00, p01], vec![p10, p11]],
            vec![vec![3.0; 2]; 2],
        )
        .unwrap()
    }

    use remus_math::vec::Point3;

    /// Cap-plane frame of the fixture: circle centre, its in-plane axes, and
    /// the extents of every cap vertex along them.
    struct CapFrame {
        center: Point3,
        u: Vec3,
        v: Vec3,
        lo: (f64, f64),
        hi: (f64, f64),
    }

    fn cap_frame(fixture: &Fixture) -> CapFrame {
        let EdgeCurve::Circle(circle) = fixture.topo.edge(fixture.cross).unwrap().curve().clone()
        else {
            panic!("cross edge is not circular");
        };
        let (u, v) = (circle.u_axis(), circle.v_axis());
        let mut lo = (f64::INFINITY, f64::INFINITY);
        let mut hi = (f64::NEG_INFINITY, f64::NEG_INFINITY);
        for edge in face_edges(&fixture.topo, fixture.cap).unwrap() {
            let data = fixture.topo.edge(edge).unwrap();
            for vertex in [data.start(), data.end()] {
                let offset = fixture.topo.vertex(vertex).unwrap().point() - circle.center();
                let (a, b) = (offset.dot(u), offset.dot(v));
                lo = (lo.0.min(a), lo.1.min(b));
                hi = (hi.0.max(a), hi.1.max(b));
            }
        }
        CapFrame {
            center: circle.center(),
            u,
            v,
            lo,
            hi,
        }
    }

    fn assert_declines_without_mutation(fixture: &mut Fixture, what: &str) {
        let slots = fixture.topo.allocated_slot_count();
        let result = heal_cylinder_plane_band_affine_cap(
            &mut fixture.topo,
            fixture.solid,
            fixture.band,
            fixture.supports,
        )
        .unwrap_or_else(|error| panic!("{what}: expected a scope decline, got {error}"));
        assert!(result.is_none(), "{what}: must decline");
        assert_eq!(
            fixture.topo.allocated_slot_count(),
            slots,
            "{what}: arena unchanged"
        );
    }

    #[test]
    fn patch_holding_the_cross_arc_but_not_the_cap_lines_declines() {
        // Half-span 1.5 around the unit fillet circle holds its whole carrier;
        // the 10 mm cap lines leave the bounded patch.
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 1.5);
        fixture
            .topo
            .face_mut(fixture.cap)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(patch));
        assert_declines_without_mutation(&mut fixture, "lines outside patch");
    }

    #[test]
    fn patch_holding_the_cap_lines_but_not_the_cross_arc_declines() {
        // A patch rotated 45 degrees whose near edge runs 1e-3 inside the arc
        // chord: every cap line (and its midpoint) is inside, but the arc
        // bulges past the chord toward the removed sharp corner.
        let mut fixture = fixture();
        let EdgeCurve::Circle(circle) = fixture.topo.edge(fixture.cross).unwrap().curve().clone()
        else {
            panic!("cross edge is not circular");
        };
        let data = fixture.topo.edge(fixture.cross).unwrap();
        let chord =
            [data.start(), data.end()].map(|vertex| fixture.topo.vertex(vertex).unwrap().point());
        let center = circle.center();
        // The removed sharp corner completes the square C, P, K, Q.
        let corner = chord[0] + (chord[1] - center);
        let along = (center - corner).normalize().unwrap();
        let across = circle.normal().cross(along).normalize().unwrap();
        let depth = (chord[0] - corner).dot(along);
        assert!(((chord[1] - corner).dot(along) - depth).abs() < 1e-12);
        let (mut far, mut lo, mut hi) = (depth, 0.0_f64, 0.0_f64);
        for edge in face_edges(&fixture.topo, fixture.cap).unwrap() {
            let data = fixture.topo.edge(edge).unwrap();
            for vertex in [data.start(), data.end()] {
                let offset = fixture.topo.vertex(vertex).unwrap().point() - corner;
                far = far.max(offset.dot(along));
                lo = lo.min(offset.dot(across));
                hi = hi.max(offset.dot(across));
            }
        }
        let origin = corner + along * (depth - 1e-3) + across * (lo - 1.0);
        let patch = affine_patch_frame(
            origin,
            along * (far - depth + 1.0),
            across * (hi - lo + 2.0),
        );
        let certificate = certify_affine_nurbs_plane(&patch, Tolerance::new().linear).unwrap();
        for edge in face_edges(&fixture.topo, fixture.cap).unwrap() {
            if edge != fixture.cross {
                assert!(certify_source_line(&fixture.topo, edge, &certificate).unwrap());
            }
        }
        assert!(!certify_cross_trim(&fixture.topo, fixture.cross, &certificate).unwrap());
        fixture
            .topo
            .face_mut(fixture.cap)
            .unwrap()
            .set_surface(FaceSurface::Nurbs(patch));
        assert_declines_without_mutation(&mut fixture, "arc outside patch");
    }

    #[test]
    fn tight_patch_around_the_sharp_cap_heals_exactly() {
        // The patch covers the cap square plus 0.25 mm: the restored sharp cap
        // and every rebuilt boundary lie inside, with little room to spare.
        let mut fixture = fixture();
        let frame = cap_frame(&fixture);
        let margin = 0.25;
        let origin =
            frame.center + frame.u * (frame.lo.0 - margin) + frame.v * (frame.lo.1 - margin);
        let patch = affine_patch_frame(
            origin,
            frame.u * (frame.hi.0 - frame.lo.0 + 2.0 * margin),
            frame.v * (frame.hi.1 - frame.lo.1 + 2.0 * margin),
        );
        let expected = install_affine_cap(&mut fixture, patch);
        let result = crate::resize_blend::resize_blend(
            &mut fixture.topo,
            fixture.solid,
            fixture.band,
            1.0,
            0.0,
        )
        .unwrap();
        assert_eq!(
            solid_entity_counts(&fixture.topo, result.solid).unwrap(),
            (6, 12, 8)
        );
        let volume = crate::measure::solid_volume(&fixture.topo, result.solid, 0.01).unwrap();
        assert!(
            (volume - 1000.0).abs() <= 1e-7,
            "sharp 10 mm cube, got {volume}"
        );
        let [cap_index] = result.evolution.modified[&fixture.cap.index()].as_slice() else {
            panic!("cap has one exact successor");
        };
        let cap = fixture.topo.face_id_from_index(*cap_index).unwrap();
        assert!(matches!(
            fixture.topo.face(cap).unwrap().surface(),
            FaceSurface::Nurbs(surface) if surface == &expected
        ));
    }

    #[test]
    fn no_live_face_keeps_the_nurbs_carrier_without_its_pcurves() {
        // The private proxy copy is live in the arena until compaction.  Every
        // face carrying the restored NURBS carrier, including that copy, must
        // keep p-curves on all of its edge uses.
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 16.0);
        let expected = install_affine_cap(&mut fixture, patch);
        heal_cylinder_plane_band_affine_cap(
            &mut fixture.topo,
            fixture.solid,
            fixture.band,
            fixture.supports,
        )
        .unwrap()
        .expect("qualified affine cap");
        let mut carriers = 0;
        for (face, data) in fixture.topo.faces().iter() {
            if !matches!(data.surface(), FaceSurface::Nurbs(surface) if surface == &expected) {
                continue;
            }
            carriers += 1;
            let uses = fixture.topo.face_oriented_edges(face).unwrap().len();
            assert_eq!(
                fixture.topo.pcurves_for_face(face).len(),
                uses,
                "NURBS face {} lost p-curve authority",
                face.index()
            );
        }
        assert!(
            carriers >= 3,
            "source, proxy copy and result carry the NURBS cap"
        );
    }

    fn circle_edge(
        topo: &mut Topology,
        circle: remus_math::curves::Circle3D,
        domain: (f64, f64),
    ) -> EdgeId {
        let start = topo.add_vertex(remus_topology::vertex::Vertex::new(
            circle.evaluate(domain.0),
            Tolerance::new().linear,
        ));
        let end = topo.add_vertex(remus_topology::vertex::Vertex::new(
            circle.evaluate(domain.1),
            Tolerance::new().linear,
        ));
        let mut edge = remus_topology::edge::Edge::new(start, end, EdgeCurve::Circle(circle));
        edge.set_trim(Some(domain));
        topo.add_edge(edge)
    }

    #[test]
    fn cross_trim_certificate_checks_the_true_carrier_extrema() {
        use remus_math::curves::Circle3D;
        // Unit circle about (0.3, -0.2, 5) in z = 5.  The patch rectangle is
        // rotated 0.37 rad against the circle axes, so every UV extremum sits
        // at a non-trivial phase.  Moving one side 1e-4 inside the circle
        // exposes exactly one extremal point.
        let rho: f64 = 0.37;
        let (sin, cos) = rho.sin_cos();
        let du = Vec3::new(cos, sin, 0.0);
        let dv = Vec3::new(-sin, cos, 0.0);
        let center = Point3::new(0.3, -0.2, 5.0);
        let circle = Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let mut topo = Topology::new();
        let edge = circle_edge(&mut topo, circle, (0.25, 1.75));
        let certificate = |lo: (f64, f64), hi: (f64, f64)| {
            let origin = center + du * lo.0 + dv * lo.1;
            let patch = affine_patch_frame(origin, du * (hi.0 - lo.0), dv * (hi.1 - lo.1));
            certify_affine_nurbs_plane(&patch, Tolerance::new().linear).unwrap()
        };
        // The far sides sit 0.75 beyond the circle so the centre's patch
        // parameters are off the domain midpoint (non-zero in v).
        let (inset, cut, far) = (1.0 + 1e-4, 1.0 - 1e-4, 1.75);
        assert!(
            certify_cross_trim(&topo, edge, &certificate((-inset, -inset), (far, far))).unwrap()
        );
        for (lo, hi) in [
            ((-cut, -inset), (far, far)),
            ((-inset, -cut), (far, far)),
            ((-inset, -inset), (cut, far)),
            ((-inset, -inset), (far, cut)),
        ] {
            assert!(
                !certify_cross_trim(&topo, edge, &certificate(lo, hi)).unwrap(),
                "patch {lo:?}..{hi:?} cuts the carrier"
            );
        }

        // A 0.01 mm circle tilted 5e-6 rad from the cap plane: every point is
        // within 5e-8 of the plane, but its normal disagrees by more than the
        // 1e-12 angular gate (1 - cos 5e-6 = 1.25e-11).
        let tilt: f64 = 5e-6;
        let small = Circle3D::new_with_ref(
            center,
            Vec3::new(tilt.sin(), 0.0, tilt.cos()),
            0.01,
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let tilted = circle_edge(&mut topo, small, (0.25, 1.75));
        let generous = certificate((-inset, -inset), (far, far));
        assert!(!certify_cross_trim(&topo, tilted, &generous).unwrap());
    }

    #[test]
    fn source_line_certificate_refuses_a_collapsed_line() {
        use remus_topology::vertex::Vertex;
        let patch = affine_patch_frame(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(4.0, 0.0, 0.0),
            Vec3::new(0.0, 4.0, 0.0),
        );
        let certificate = certify_affine_nurbs_plane(&patch, Tolerance::new().linear).unwrap();
        let mut topo = Topology::new();
        // Distinct vertices at one point inside the patch (u = 3.5, v = 1.5):
        // the edge has no affine direction.
        let a = topo.add_vertex(Vertex::new(
            Point3::new(2.0, 3.0, 0.0),
            Tolerance::new().linear,
        ));
        let b = topo.add_vertex(Vertex::new(
            Point3::new(2.0, 3.0, 0.0),
            Tolerance::new().linear,
        ));
        let c = topo.add_vertex(Vertex::new(
            Point3::new(3.0, 1.0, 0.0),
            Tolerance::new().linear,
        ));
        let collapsed = topo.add_edge(remus_topology::edge::Edge::new(a, b, EdgeCurve::Line));
        let proper = topo.add_edge(remus_topology::edge::Edge::new(a, c, EdgeCurve::Line));
        assert!(!certify_source_line(&topo, collapsed, &certificate).unwrap());
        assert!(certify_source_line(&topo, proper, &certificate).unwrap());
    }

    #[test]
    fn non_cylindrical_band_or_curved_support_declines() {
        // Same topology and certified cap; only the band carrier changes.
        let mut spherical = fixture();
        let patch = affine_patch(&spherical.topo, spherical.cross, 16.0);
        install_affine_cap(&mut spherical, patch);
        let sphere =
            remus_math::surfaces::SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 3.0).unwrap();
        spherical
            .topo
            .face_mut(spherical.band)
            .unwrap()
            .set_surface(FaceSurface::Sphere(sphere));
        assert_declines_without_mutation(&mut spherical, "spherical band");

        // Same topology; one support is a cylinder instead of a plane.
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 16.0);
        install_affine_cap(&mut fixture, patch);
        let cylinder = remus_math::surfaces::CylindricalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            50.0,
        )
        .unwrap();
        fixture
            .topo
            .face_mut(fixture.supports[1])
            .unwrap()
            .set_surface(FaceSurface::Cylinder(cylinder));
        assert_declines_without_mutation(&mut fixture, "cylindrical support");
    }

    /// Split `edge` at its midpoint in every wire that uses it.
    fn split_line_edge(topo: &mut Topology, solid: SolidId, edge: EdgeId) {
        use remus_topology::edge::Edge;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};
        let data = topo.edge(edge).unwrap();
        let (start, end) = (data.start(), data.end());
        let midpoint = topo.vertex(start).unwrap().point()
            + (topo.vertex(end).unwrap().point() - topo.vertex(start).unwrap().point()) * 0.5;
        let middle = topo.add_vertex(Vertex::new(midpoint, Tolerance::new().linear));
        let first = topo.add_edge(Edge::new(start, middle, EdgeCurve::Line));
        let second = topo.add_edge(Edge::new(middle, end, EdgeCurve::Line));
        for face in solid_faces(topo, solid).unwrap() {
            let outer = topo.face(face).unwrap().outer_wire();
            let uses = topo.wire(outer).unwrap().edges().to_vec();
            if !uses.iter().any(|use_| use_.edge() == edge) {
                continue;
            }
            let mut rebuilt = Vec::new();
            for use_ in uses {
                if use_.edge() == edge {
                    if use_.is_forward() {
                        rebuilt.push(OrientedEdge::new(first, true));
                        rebuilt.push(OrientedEdge::new(second, true));
                    } else {
                        rebuilt.push(OrientedEdge::new(second, false));
                        rebuilt.push(OrientedEdge::new(first, false));
                    }
                } else {
                    rebuilt.push(use_);
                }
            }
            let wire = topo.add_wire(Wire::new(rebuilt, true).unwrap());
            let inner = topo.face(face).unwrap().inner_wires().to_vec();
            topo.set_face_boundary_wires(face, wire, inner).unwrap();
        }
    }

    #[test]
    fn split_spring_contact_is_outside_the_affine_adapter() {
        // The adapter qualifies exactly one contact edge per support; a split
        // spring belongs to the general split-chain proof instead.
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 16.0);
        install_affine_cap(&mut fixture, patch);
        let adjacency = fixture.topo.build_adjacency(fixture.solid).unwrap();
        let spring = face_edges(&fixture.topo, fixture.band)
            .unwrap()
            .into_iter()
            .find(|edge| {
                adjacency
                    .faces_for_edge(*edge)
                    .contains(&fixture.supports[0])
            })
            .unwrap();
        split_line_edge(&mut fixture.topo, fixture.solid, spring);
        assert!(
            crate::validate::validate_solid(&fixture.topo, fixture.solid)
                .unwrap()
                .is_valid()
        );
        assert_declines_without_mutation(&mut fixture, "split spring");
    }

    #[test]
    fn affine_cap_with_an_inner_wire_declines() {
        use remus_topology::edge::Edge;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};
        let mut fixture = fixture();
        let patch = affine_patch(&fixture.topo, fixture.cross, 16.0);
        install_affine_cap(&mut fixture, patch);
        let frame = cap_frame(&fixture);
        let mid = (
            0.5 * (frame.lo.0 + frame.hi.0),
            0.5 * (frame.lo.1 + frame.hi.1),
        );
        let corners = [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)].map(|(a, b)| {
            let point = frame.center + frame.u * (mid.0 + a) + frame.v * (mid.1 + b);
            fixture
                .topo
                .add_vertex(Vertex::new(point, Tolerance::new().linear))
        });
        let hole: Vec<_> = (0..4)
            .map(|index| {
                let edge = fixture.topo.add_edge(Edge::new(
                    corners[index],
                    corners[(index + 1) % 4],
                    EdgeCurve::Line,
                ));
                OrientedEdge::new(edge, true)
            })
            .collect();
        let inner = fixture.topo.add_wire(Wire::new(hole, true).unwrap());
        let outer = fixture.topo.face(fixture.cap).unwrap().outer_wire();
        fixture
            .topo
            .set_face_boundary_wires(fixture.cap, outer, vec![inner])
            .unwrap();
        assert_declines_without_mutation(&mut fixture, "cap with a hole");
    }
}
