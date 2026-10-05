//! Link section PaveBlocks with existing boundary PaveBlocks.
//!
//! After ForceInterfEE has grouped boundary PBs into CommonBlocks,
//! this pass checks each FF section PB against boundary PBs. When a
//! section PB has the same resolved vertex endpoints as a boundary PB
//! (and compatible curve geometry), it is added to the boundary PB's
//! CommonBlock — or a new CB is created for the pair.
//!
//! This implements the reference implementation's `IsExistingPaveBlock`
//! pattern: section edges that coincide with face boundary edges are
//! linked so `MakeSplitEdges` creates one shared edge entity.

use remus_math::tolerance::Tolerance;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;

use crate::ds::{GfaArena, PaveBlockId};
use crate::error::AlgoError;

/// Quantized 3D position pair for endpoint matching.
type QPair = ((i64, i64, i64), (i64, i64, i64));

/// Quantized circle geometry (center, radius, axis up to sign) for
/// matching full closed blocks whose seam vertices differ.
type QCircle = ((i64, i64, i64), i64, (i64, i64, i64));

/// Quantization scale for unit axis directions.
const AXIS_SCALE: f64 = 1.0e7;

fn circle_key(
    c: &remus_math::curves::Circle3D,
    qpt: impl Fn(remus_math::vec::Point3) -> (i64, i64, i64),
    linear_scale: f64,
) -> QCircle {
    let n = c.normal();
    // Canonicalize axis sign so opposite-facing but coincident circles match.
    let flip = match (n.x().abs() > 0.5, n.y().abs() > 0.5) {
        (true, _) => n.x() < 0.0,
        (false, true) => n.y() < 0.0,
        (false, false) => n.z() < 0.0,
    };
    let n = if flip { -n } else { n };
    #[allow(clippy::cast_possible_truncation)]
    let qaxis = (
        (n.x() * AXIS_SCALE).round() as i64,
        (n.y() * AXIS_SCALE).round() as i64,
        (n.z() * AXIS_SCALE).round() as i64,
    );
    #[allow(clippy::cast_possible_truncation)]
    let qr = (c.radius() * linear_scale).round() as i64;
    (qpt(c.center()), qr, qaxis)
}

/// Link section PBs to coincident boundary PBs via CommonBlocks.
///
/// For each leaf section PB (from `arena.curves`), resolves its vertex
/// endpoints and searches for a boundary PB with matching resolved
/// endpoints and compatible curve geometry. If found, links them in a
/// CommonBlock so `MakeSplitEdges` creates a shared edge.
///
/// # Errors
///
/// Returns [`AlgoError`] if topology lookups fail.
#[allow(clippy::unnecessary_wraps)] // Signature matches other PaveFiller passes
pub fn perform(topo: &Topology, tol: Tolerance, arena: &mut GfaArena) -> Result<(), AlgoError> {
    // Collect resolved endpoints for all boundary leaf PBs.
    // Key: (min_pos, max_pos) quantized at tolerance, Value: list of PB IDs.
    let scale = 1.0 / tol.linear;
    let qpt = |p: remus_math::vec::Point3| -> (i64, i64, i64) {
        (
            (p.x() * scale).round() as i64,
            (p.y() * scale).round() as i64,
            (p.z() * scale).round() as i64,
        )
    };

    let mut boundary_index: std::collections::HashMap<QPair, Vec<PaveBlockId>> =
        std::collections::HashMap::new();

    // Secondary index for full closed circle blocks (start == end vertex):
    // their endpoint is an arbitrary seam vertex, so endpoint-pair keys
    // cannot match across differently-seamed but coincident circles.
    let mut closed_index: std::collections::HashMap<QCircle, Vec<PaveBlockId>> =
        std::collections::HashMap::new();

    let all_edge_pbs: Vec<Vec<PaveBlockId>> = arena
        .edge_pave_blocks
        .values()
        .map(|pbs| arena.collect_leaf_pave_blocks(pbs))
        .collect();

    for leaf_pbs in &all_edge_pbs {
        for &pb_id in leaf_pbs {
            let Some(pb) = arena.pave_blocks.get(pb_id) else {
                continue;
            };
            let sv = arena.resolve_vertex(pb.start.vertex);
            let ev = arena.resolve_vertex(pb.end.vertex);
            let Ok(sp) = topo.vertex(sv).map(remus_topology::vertex::Vertex::point) else {
                continue;
            };
            let Ok(ep) = topo.vertex(ev).map(remus_topology::vertex::Vertex::point) else {
                continue;
            };
            let qs = qpt(sp);
            let qe = qpt(ep);
            let key = if qs <= qe { (qs, qe) } else { (qe, qs) };
            boundary_index.entry(key).or_default().push(pb_id);

            if sv == ev
                && let Ok(edge) = topo.edge(pb.original_edge)
                && let EdgeCurve::Circle(c) = edge.curve()
            {
                closed_index
                    .entry(circle_key(c, qpt, scale))
                    .or_default()
                    .push(pb_id);
            }
        }
    }

    let mut linked = 0_usize;

    // Collect section PB IDs upfront to avoid borrowing arena.curves while mutating arena.
    let section_pb_ids: Vec<PaveBlockId> = arena
        .curves
        .iter()
        .flat_map(|c| c.pave_blocks.iter().copied())
        .collect();

    for root_pb_id in &section_pb_ids {
        let leaves = arena.collect_leaf_pave_blocks(&[*root_pb_id]);
        for section_pb_id in leaves {
            if arena.pb_to_cb.contains_key(&section_pb_id) {
                continue;
            }

            let Some(section_pb) = arena.pave_blocks.get(section_pb_id) else {
                continue;
            };
            let sv = arena.resolve_vertex(section_pb.start.vertex);
            let ev = arena.resolve_vertex(section_pb.end.vertex);
            let Ok(sp) = topo.vertex(sv).map(remus_topology::vertex::Vertex::point) else {
                continue;
            };
            let Ok(ep) = topo.vertex(ev).map(remus_topology::vertex::Vertex::point) else {
                continue;
            };
            let qs = qpt(sp);
            let qe = qpt(ep);
            let key = if qs <= qe { (qs, qe) } else { (qe, qs) };

            // Check curve compatibility with each candidate.
            // Use graceful skip (not `?`) for edge lookups — consistent with
            // vertex lookups above. A stale original_edge should skip the PB,
            // not abort the entire linking pass.
            let Ok(section_edge) = topo.edge(section_pb.original_edge) else {
                continue;
            };
            let section_curve = section_edge.curve().clone();

            let mut linked_this = false;
            if let Some(candidates) = boundary_index.get(&key) {
                for &boundary_pb_id in candidates {
                    if try_link(
                        topo,
                        tol,
                        arena,
                        section_pb_id,
                        &section_curve,
                        boundary_pb_id,
                    ) {
                        linked += 1;
                        linked_this = true;
                        break; // One match is sufficient
                    }
                }
            } else {
                log::trace!("link_existing: no boundary PB at position");
            }

            // Fallback for full closed circles: the endpoint-pair key uses
            // the seam vertex, which is arbitrary, so coincident circles
            // with different seams never share a key. Match on quantized
            // circle geometry instead.
            if !linked_this
                && sv == ev
                && let EdgeCurve::Circle(c) = &section_curve
                && let Some(candidates) = closed_index.get(&circle_key(c, qpt, scale))
            {
                for &boundary_pb_id in candidates {
                    if try_link(
                        topo,
                        tol,
                        arena,
                        section_pb_id,
                        &section_curve,
                        boundary_pb_id,
                    ) {
                        linked += 1;
                        break;
                    }
                }
            }
        }
    }

    if linked > 0 {
        log::debug!(
            "link_existing: linked {linked} section PBs with boundary PBs ({} section total, {} boundary groups)",
            section_pb_ids.len(),
            boundary_index.len()
        );
    }

    Ok(())
}

/// Attempt to link a section PB with a boundary PB into a CommonBlock.
///
/// Returns `true` if the pair was linked. Skips self-pairs, pairs already
/// sharing a CB, and geometrically incompatible curves.
#[allow(clippy::float_cmp)] // Exact spline span identity is required, not proximity.
fn try_link(
    topo: &Topology,
    tol: Tolerance,
    arena: &mut GfaArena,
    section_pb_id: PaveBlockId,
    section_curve: &EdgeCurve,
    boundary_pb_id: PaveBlockId,
) -> bool {
    // Self-match guard: coplanar FF section PBs can appear in both
    // arena.curves and arena.edge_pave_blocks. Skip self-pairing.
    if boundary_pb_id == section_pb_id {
        return false;
    }

    if arena.pb_to_cb.get(&boundary_pb_id) == arena.pb_to_cb.get(&section_pb_id)
        && arena.pb_to_cb.contains_key(&section_pb_id)
    {
        return false;
    }

    let Some(boundary_pb) = arena.pave_blocks.get(boundary_pb_id) else {
        return false;
    };

    let Ok(boundary_edge) = topo.edge(boundary_pb.original_edge) else {
        return false;
    };
    let boundary_curve = boundary_edge.curve();

    let identical_spline_span = match section_curve {
        EdgeCurve::NurbsCurve(a) => match boundary_curve {
            EdgeCurve::NurbsCurve(b) => {
                arena
                    .pave_blocks
                    .get(section_pb_id)
                    .is_some_and(|section_pb| {
                        super::helpers::identical_nurbs_span(
                            a,
                            (section_pb.start.parameter, section_pb.end.parameter),
                            b,
                            (boundary_pb.start.parameter, boundary_pb.end.parameter),
                        )
                    })
            }
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_) => false,
        },
        EdgeCurve::Line
        | EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => false,
    };
    // Equal coefficients AND equal parameter spans prove the whole trace.
    // Endpoints alone cannot identify a spline branch or a co-endpoint lens.
    if !identical_spline_span && !curves_compatible(section_curve, boundary_curve, tol) {
        return false;
    }

    if let Some(&cb_id) = arena.pb_to_cb.get(&boundary_pb_id) {
        if let Some(cb) = arena.common_blocks.get_mut(cb_id) {
            cb.pave_blocks.push(section_pb_id);
        }
        arena.pb_to_cb.insert(section_pb_id, cb_id);
    } else {
        arena.create_common_block(vec![boundary_pb_id, section_pb_id]);
    }

    log::debug!(
        "link_existing: linked section PB {section_pb_id:?} with boundary PB {boundary_pb_id:?}"
    );
    true
}

/// Check if two edge curves are geometrically compatible.
fn curves_compatible(a: &EdgeCurve, b: &EdgeCurve, tol: Tolerance) -> bool {
    match (a, b) {
        (EdgeCurve::Line, EdgeCurve::Line) => true,
        (EdgeCurve::Circle(ca), EdgeCurve::Circle(cb)) => {
            (ca.radius() - cb.radius()).abs() < tol.linear
                && (ca.center() - cb.center()).length() < tol.linear
                && ca.normal().dot(cb.normal()).abs() > 1.0 - tol.angular
        }
        (EdgeCurve::Ellipse(ea), EdgeCurve::Ellipse(eb)) => {
            (ea.semi_major() - eb.semi_major()).abs() < tol.linear
                && (ea.semi_minor() - eb.semi_minor()).abs() < tol.linear
                && (ea.center() - eb.center()).length() < tol.linear
                && ea.normal().dot(eb.normal()).abs() > 1.0 - tol.angular
        }
        // Same-type conic coincidence: identical placement implies
        // identical point sets, since both parameterizations are injective
        // over the whole real line.
        (EdgeCurve::Hyperbola(ha), EdgeCurve::Hyperbola(hb)) => {
            (ha.semi_major() - hb.semi_major()).abs() < tol.linear
                && (ha.semi_minor() - hb.semi_minor()).abs() < tol.linear
                && (ha.center() - hb.center()).length() < tol.linear
                && ha.normal().dot(hb.normal()).abs() > 1.0 - tol.angular
                // Sign matters here: `Hyperbola3D` models a single branch,
                // so an anti-parallel real axis is the OTHER branch.
                && ha.u_axis().dot(hb.u_axis()) > 1.0 - tol.angular
        }
        (EdgeCurve::Parabola(pa), EdgeCurve::Parabola(pb)) => {
            (pa.focal_length() - pb.focal_length()).abs() < tol.linear
                && (pa.vertex() - pb.vertex()).length() < tol.linear
                && pa.axis_dir().dot(pb.axis_dir()) > 1.0 - tol.angular
                // A parabola is symmetric about its axis, so only the PLANE
                // matters, not the sign of its normal.
                && pa.normal().dot(pb.normal()).abs() > 1.0 - tol.angular
        }
        (EdgeCurve::NurbsCurve(_), EdgeCurve::NurbsCurve(_)) => false,
        // Different curve types cannot be geometrically coincident. Every
        // left-hand variant is listed rather than using `_`, so adding an
        // `EdgeCurve` variant still makes the compiler flag this site.
        (
            EdgeCurve::Line
            | EdgeCurve::Circle(_)
            | EdgeCurve::Ellipse(_)
            | EdgeCurve::Hyperbola(_)
            | EdgeCurve::Parabola(_)
            | EdgeCurve::NurbsCurve(_),
            _,
        ) => false,
    }
}

#[cfg(test)]
mod spline_identity_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]
    use super::*;
    use crate::ds::{Pave, PaveBlock};
    use remus_math::nurbs::curve::NurbsCurve;
    use remus_math::vec::Point3;
    use remus_topology::edge::Edge;
    use remus_topology::vertex::Vertex;

    fn pair(mid_y: f64, span: (f64, f64)) -> bool {
        oriented_pair(mid_y, span, false)
    }

    fn oriented_pair(mid_y: f64, span: (f64, f64), reverse: bool) -> bool {
        let mut topo = Topology::new();
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let va = topo.add_vertex(Vertex::new(a, 1e-7));
        let vb = topo.add_vertex(Vertex::new(b, 1e-7));
        let curve = |y| {
            EdgeCurve::NurbsCurve(
                NurbsCurve::new(
                    2,
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![a, Point3::new(1.0, y, 0.0), b],
                    vec![1.0; 3],
                )
                .unwrap(),
            )
        };
        let section_curve = curve(-1.0);
        let section = topo.add_edge(Edge::new(va, vb, section_curve.clone()));
        let boundary_curve = curve(mid_y);
        let boundary_curve = if reverse {
            let EdgeCurve::NurbsCurve(c) = boundary_curve else {
                unreachable!()
            };
            EdgeCurve::NurbsCurve(c.reversed())
        } else {
            boundary_curve
        };
        let (boundary_start, boundary_end) = if reverse { (vb, va) } else { (va, vb) };
        let boundary = topo.add_edge(Edge::new(boundary_start, boundary_end, boundary_curve));
        let mut arena = GfaArena::new();
        let section_pb = arena.pave_blocks.alloc(PaveBlock::new(
            section,
            Pave::new(va, 0.0),
            Pave::new(vb, 1.0),
        ));
        let boundary_pb = arena.pave_blocks.alloc(PaveBlock::new(
            boundary,
            Pave::new(boundary_start, span.0),
            Pave::new(boundary_end, span.1),
        ));
        try_link(
            &topo,
            Tolerance::new(),
            &mut arena,
            section_pb,
            &section_curve,
            boundary_pb,
        )
    }

    #[test]
    fn identical_spline_coefficients_and_span_share_one_boundary() {
        assert!(pair(-1.0, (0.0, 1.0)));
    }

    #[test]
    fn reversed_spline_links_only_its_identical_whole_trace() {
        assert!(oriented_pair(-1.0, (0.0, 1.0), true));
        assert!(!oriented_pair(1.0, (0.0, 1.0), true));
        assert!(!oriented_pair(-1.0, (0.25, 0.75), true));
        let a = NurbsCurve::new(
            2,
            vec![2.0, 2.0, 2.0, 3.0, 5.0, 5.0, 5.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(2.0, -1.0, 0.0),
                Point3::new(4.0, 0.0, 0.0),
            ],
            vec![1.0, 2.0, 0.5, 1.0],
        )
        .unwrap();
        let b = a.reversed();
        assert!(super::super::helpers::identical_nurbs_span(
            &a,
            (2.25, 4.5),
            &b,
            (2.5, 4.75)
        ));
        assert!(super::super::helpers::identical_nurbs_span(
            &a,
            (4.5, 2.25),
            &b,
            (2.5, 4.75)
        ));
        assert!(!super::super::helpers::identical_nurbs_span(
            &a,
            (2.25, 4.5),
            &b,
            (2.25, 4.5)
        ));
    }

    #[test]
    fn coendpoint_spline_lenses_never_link() {
        assert!(!pair(1.0, (0.0, 1.0)));
        assert!(!pair(-1.0 - 1e-10, (0.0, 1.0)));
    }

    #[test]
    fn equal_spline_coefficients_do_not_link_different_parameter_spans() {
        assert!(!pair(-1.0, (0.25, 0.75)));
    }
}
