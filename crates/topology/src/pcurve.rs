//! `PCurve` — 2D parametric curves on surfaces, stored per edge **use**.
//!
//! A pcurve represents an edge's geometry projected into a face's surface
//! parameter space (u, v). `PCurves` are essential for exact boolean
//! operations, surface trimming, and proper I/O with STEP/IGES formats.
//!
//! # Per-use storage (RFC 0002, Stage 2 authority flip)
//!
//! The p-curve itself lives on [`Coedge`](crate::coedge::Coedge). This module
//! keeps only the compatibility index from `(edge, face, orientation)` to the
//! authoritative coedge. A periodic face's seam therefore retains two
//! independent branches without duplicating geometry in a registry.
//!
//! The registry itself is raw storage, internal to the crate. Public access
//! goes through [`Topology`](crate::Topology): the oriented methods
//! (`pcurve_oriented`, `set_pcurve_oriented`, …) address a use exactly, and
//! the `(edge, face)` convenience methods (`pcurve`, `set_pcurve`, …)
//! resolve the single use — failing closed with
//! [`TopologyError::SeamPcurveAmbiguous`](crate::TopologyError::SeamPcurveAmbiguous)
//! when both branches are present, instead of answering arbitrarily.

use std::collections::{HashMap, HashSet};

use remus_math::curves2d::Curve2D;

use crate::coedge::CoedgeId;
use crate::edge::EdgeId;
use crate::face::FaceId;

/// Key identifying one pcurve use: which edge, on which face, traversed in
/// which direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PCurveKey {
    /// The edge this pcurve belongs to.
    pub edge: EdgeId,
    /// The face whose surface parameter space the pcurve lives in.
    pub face: FaceId,
    /// The traversal orientation of this use relative to the edge's
    /// natural direction. Distinguishes the two branches of a seam edge.
    pub forward: bool,
}

impl PCurveKey {
    /// Creates a new pcurve use key.
    #[must_use]
    pub const fn new(edge: EdgeId, face: FaceId, forward: bool) -> Self {
        Self {
            edge,
            face,
            forward,
        }
    }
}

/// A 2D parametric curve on a surface, with parameter bounds.
///
/// The curve is defined in the face's surface (u, v) parameter space.
/// The parameter range `[t_start, t_end]` maps to the edge's 3D start
/// and end vertices (respecting orientation).
#[derive(Debug, Clone)]
pub struct PCurve {
    /// The 2D curve in surface parameter space.
    curve: Curve2D,
    /// Start parameter on the 2D curve.
    t_start: f64,
    /// End parameter on the 2D curve.
    t_end: f64,
}

impl PCurve {
    /// Creates a new pcurve.
    #[must_use]
    pub const fn new(curve: Curve2D, t_start: f64, t_end: f64) -> Self {
        Self {
            curve,
            t_start,
            t_end,
        }
    }

    /// Returns a reference to the 2D curve.
    #[must_use]
    pub const fn curve(&self) -> &Curve2D {
        &self.curve
    }

    /// Returns the start parameter.
    #[must_use]
    pub const fn t_start(&self) -> f64 {
        self.t_start
    }

    /// Returns the end parameter.
    #[must_use]
    pub const fn t_end(&self) -> f64 {
        self.t_end
    }

    /// Evaluates the pcurve at parameter `t`.
    #[must_use]
    pub fn evaluate(&self, t: f64) -> remus_math::vec::Point2 {
        self.curve.evaluate(t)
    }
}

/// Compatibility index from the historical per-use key to the authoritative
/// coedge. Access goes through [`Topology`](crate::Topology)'s pcurve methods.
#[derive(Default, Clone)]
pub struct PCurveRegistry {
    uses: HashMap<PCurveKey, CoedgeId>,
}

impl std::fmt::Debug for PCurveRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Rollback rebuilds this map; diagnostics must not depend on its hash seed.
        let Self { uses } = self;
        let mut uses: Vec<_> = uses.iter().collect();
        uses.sort_by_key(|(key, _)| (key.edge.index(), key.face.index(), key.forward));
        f.debug_struct("PCurveRegistry")
            .field("uses", &uses)
            .finish()
    }
}

impl PCurveRegistry {
    /// Creates a new, empty pcurve registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Indexes one authoritative coedge use.
    pub(crate) fn index_use(
        &mut self,
        edge: EdgeId,
        face: FaceId,
        forward: bool,
        coedge: CoedgeId,
    ) {
        self.uses
            .insert(PCurveKey::new(edge, face, forward), coedge);
    }

    /// Resolves one indexed use.
    pub(crate) fn get_use(&self, edge: EdgeId, face: FaceId, forward: bool) -> Option<CoedgeId> {
        self.uses.get(&PCurveKey::new(edge, face, forward)).copied()
    }

    /// The indexed uses of `edge` on `face`: `(forward, coedge)` per entry,
    /// forward branch first. At most two entries.
    pub(crate) fn uses_on_face(&self, edge: EdgeId, face: FaceId) -> Vec<(bool, CoedgeId)> {
        [true, false]
            .into_iter()
            .filter_map(|forward| {
                self.get_use(edge, face, forward)
                    .map(|use_id| (forward, use_id))
            })
            .collect()
    }

    /// Removes pcurves whose edge or face has been retired.
    pub(crate) fn remove_for_retired_entities(
        &mut self,
        retired_edges: &HashSet<EdgeId>,
        retired_faces: &HashSet<FaceId>,
    ) {
        self.uses.retain(|key, _| {
            !retired_edges.contains(&key.edge) && !retired_faces.contains(&key.face)
        });
    }

    /// Removes every index entry for one face before its loops are replaced.
    pub(crate) fn remove_face(&mut self, face: FaceId) {
        self.uses.retain(|key, _| key.face != face);
    }

    /// Returns the number of indexed coedge uses, including uses without a
    /// stored pcurve.
    #[must_use]
    pub fn len(&self) -> usize {
        self.uses.len()
    }

    /// Returns true if the compatibility use index is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.uses.is_empty()
    }

    /// All stored uses for a given face.
    pub(crate) fn uses_for_face(&self, face: FaceId) -> Vec<(EdgeId, bool, CoedgeId)> {
        let mut out: Vec<_> = self
            .uses
            .iter()
            .filter(|(k, _)| k.face == face)
            .map(|(k, v)| (k.edge, k.forward, *v))
            .collect();
        out.sort_by_key(|(e, forward, _)| (e.index(), !forward));
        out
    }

    /// All stored uses for a given edge.
    pub(crate) fn uses_for_edge(&self, edge: EdgeId) -> Vec<(FaceId, bool, CoedgeId)> {
        let mut out: Vec<_> = self
            .uses
            .iter()
            .filter(|(k, _)| k.edge == edge)
            .map(|(k, v)| (k.face, k.forward, *v))
            .collect();
        out.sort_by_key(|(f, forward, _)| (f.index(), !forward));
        out
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use remus_math::curves2d::{Curve2D, Line2D, NurbsCurve2D};
    use remus_math::vec::{Point2, Point3, Vec2};

    use crate::edge::{Edge, EdgeCurve, EdgeId};
    use crate::face::{Face, FaceId, FaceSurface};
    use crate::topology::Topology;
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    use super::*;

    #[test]
    fn registry_debug_is_independent_of_map_rebuild_order() {
        let (topo, edge, face) = make_simple_topology();
        let coedge = topo.coedges_of_edge(edge)[0];
        let mut first = PCurveRegistry::new();
        first.index_use(edge, face, true, coedge);
        first.index_use(edge, face, false, coedge);
        let mut second = PCurveRegistry::new();
        second.index_use(edge, face, false, coedge);
        second.index_use(edge, face, true, coedge);
        assert_eq!(format!("{first:?}"), format!("{second:?}"));
        second.uses.remove(&PCurveKey::new(edge, face, true));
        assert_ne!(format!("{first:?}"), format!("{second:?}"));
    }

    fn make_simple_topology() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();

        let v0 = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let v1 = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v2 = topo.add_vertex(Vertex::new(Point3::new(1.0, 1.0, 0.0), 1e-7));

        let e0 = topo.add_edge(Edge::new(v0, v1, EdgeCurve::Line));
        let e1 = topo.add_edge(Edge::new(v1, v2, EdgeCurve::Line));
        let e2 = topo.add_edge(Edge::new(v2, v0, EdgeCurve::Line));

        let wire = Wire::new(
            vec![
                OrientedEdge::new(e0, true),
                OrientedEdge::new(e1, true),
                OrientedEdge::new(e2, true),
            ],
            true,
        )
        .unwrap();
        let wire_id = topo.add_wire(wire);

        let face = Face::new(
            wire_id,
            vec![],
            FaceSurface::Plane {
                normal: remus_math::vec::Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        );
        let face_id = topo.add_face(face);

        (topo, e0, face_id)
    }

    fn line_pcurve(x0: f64, y0: f64, dx: f64, dy: f64) -> PCurve {
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(x0, y0), Vec2::new(dx, dy)).unwrap()),
            0.0,
            1.0,
        )
    }

    #[test]
    fn set_and_get_pcurve_resolves_the_single_use() {
        let (mut topo, edge_id, face_id) = make_simple_topology();

        topo.set_pcurve(edge_id, face_id, line_pcurve(0.0, 0.0, 1.0, 0.0))
            .unwrap();

        assert!(topo.has_pcurve(edge_id, face_id).unwrap());
        let pc = topo.pcurve(edge_id, face_id).unwrap().unwrap();
        assert!((pc.t_start() - 0.0).abs() < f64::EPSILON);
        assert!((pc.t_end() - 1.0).abs() < f64::EPSILON);
        // Stored under the wire's actual use orientation (forward here).
        assert!(topo.pcurve_oriented(edge_id, face_id, true).is_some());
        assert!(topo.pcurve_oriented(edge_id, face_id, false).is_none());
    }

    #[test]
    fn pcurve_evaluate() {
        let pcurve = line_pcurve(0.0, 0.0, 1.0, 0.0);
        let p = pcurve.evaluate(0.5);
        assert!((p.x() - 0.5).abs() < 1e-10);
        assert!((p.y() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn remove_pcurve_round_trip() {
        let (mut topo, edge_id, face_id) = make_simple_topology();
        topo.set_pcurve(edge_id, face_id, line_pcurve(0.0, 0.0, 1.0, 0.0))
            .unwrap();
        assert!(topo.has_pcurve(edge_id, face_id).unwrap());
        assert!(topo.remove_pcurve(edge_id, face_id).unwrap().is_some());
        assert!(!topo.has_pcurve(edge_id, face_id).unwrap());
    }

    #[test]
    fn pcurves_for_face_reports_uses() {
        let (mut topo, edge_id, face_id) = make_simple_topology();
        topo.set_pcurve(edge_id, face_id, line_pcurve(0.0, 0.0, 1.0, 0.0))
            .unwrap();
        let uses = topo.pcurves_for_face(face_id);
        assert_eq!(uses.len(), 1);
        assert_eq!(uses[0].0, edge_id);
        assert!(uses[0].1, "stored under the forward use");
    }

    #[test]
    fn nurbs_pcurve() {
        let curve = NurbsCurve2D::from_line(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)).unwrap();
        let pcurve = PCurve::new(Curve2D::Nurbs(curve), 0.0, 1.0);

        let p = pcurve.evaluate(0.5);
        assert!((p.x() - 0.5).abs() < 1e-10);
        assert!((p.y() - 0.5).abs() < 1e-10);
    }
}

/// The periodic-seam tests from RFC 0002 — **flipped** from characterizing
/// the defect (PR #10) to proving the fix, exactly as each test's comment
/// promised. One seam edge, one face, two independent parameter-space
/// branches.
#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod seam_characterization {
    use remus_math::curves::Circle3D;
    use remus_math::curves2d::{Curve2D, Line2D};
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::{Point2, Point3, Vec2, Vec3};

    use crate::TopologyError;
    use crate::edge::{Edge, EdgeCurve, EdgeId};
    use crate::face::{Face, FaceId, FaceSurface};
    use crate::pcurve::PCurve;
    use crate::topology::Topology;
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    /// A unit cylinder's side face whose boundary wire uses one vertical
    /// seam edge twice: forward on the u = 0 branch, reversed at u = 2π.
    fn make_cylinder_side_face_with_seam() -> (Topology, EdgeId, FaceId) {
        let mut topo = Topology::new();

        let v_bottom = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let v_top = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 1.0), 1e-7));

        let seam = topo.add_edge(Edge::new(v_bottom, v_top, EdgeCurve::Line));
        let bottom = topo.add_edge(Edge::new(
            v_bottom,
            v_bottom,
            EdgeCurve::Circle(
                Circle3D::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap(),
            ),
        ));
        let top = topo.add_edge(Edge::new(
            v_top,
            v_top,
            EdgeCurve::Circle(
                Circle3D::new(Point3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap(),
            ),
        ));

        let wire_id = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(seam, true),
                    OrientedEdge::new(top, true),
                    OrientedEdge::new(seam, false),
                    OrientedEdge::new(bottom, false),
                ],
                true,
            )
            .unwrap(),
        );

        let face_id = topo.add_face(Face::new(
            wire_id,
            vec![],
            FaceSurface::Cylinder(
                CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0)
                    .unwrap(),
            ),
        ));

        (topo, seam, face_id)
    }

    fn branch_pcurve(u: f64, upward: bool) -> PCurve {
        let (y0, dy) = if upward { (0.0, 1.0) } else { (1.0, -1.0) };
        PCurve::new(
            Curve2D::Line(Line2D::new(Point2::new(u, y0), Vec2::new(0.0, dy)).unwrap()),
            0.0,
            1.0,
        )
    }

    #[test]
    fn both_seam_branches_are_retained_independently() {
        // FLIPPED from "second_seam_pcurve_silently_replaces_the_first":
        // coedge-hosted storage retains both parameter-space branches.
        const TAU: f64 = std::f64::consts::TAU;
        let (mut topo, seam, face_id) = make_cylinder_side_face_with_seam();

        topo.set_pcurve_oriented(seam, face_id, true, branch_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face_id, false, branch_pcurve(TAU, false))
            .unwrap();

        assert_eq!(topo.num_pcurves(), 2, "both branches stored");
        let at_zero = topo.pcurve_oriented(seam, face_id, true).unwrap();
        let at_tau = topo.pcurve_oriented(seam, face_id, false).unwrap();
        assert!((at_zero.evaluate(0.0).x() - 0.0).abs() < 1e-12);
        assert!((at_tau.evaluate(0.0).x() - TAU).abs() < 1e-12);
    }

    #[test]
    fn edge_face_access_on_a_seam_fails_closed() {
        // FLIPPED: the (edge, face) pair no longer answers arbitrarily on a
        // seam — every unoriented accessor returns the typed ambiguity.
        const TAU: f64 = std::f64::consts::TAU;
        let (mut topo, seam, face_id) = make_cylinder_side_face_with_seam();

        // set_pcurve refuses before any entry exists: the WIRE uses the
        // edge twice, so the pair cannot identify a use.
        assert!(matches!(
            topo.set_pcurve(seam, face_id, branch_pcurve(0.0, true)),
            Err(TopologyError::SeamPcurveAmbiguous { .. })
        ));

        topo.set_pcurve_oriented(seam, face_id, true, branch_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face_id, false, branch_pcurve(TAU, false))
            .unwrap();

        assert!(matches!(
            topo.pcurve(seam, face_id),
            Err(TopologyError::SeamPcurveAmbiguous { .. })
        ));
        assert!(matches!(
            topo.has_pcurve(seam, face_id),
            Err(TopologyError::SeamPcurveAmbiguous { .. })
        ));
        assert!(matches!(
            topo.remove_pcurve(seam, face_id),
            Err(TopologyError::SeamPcurveAmbiguous { .. })
        ));
    }

    #[test]
    fn per_edge_query_reports_both_uses() {
        // FLIPPED from "pcurves_for_edge_reports_one_use_for_a_seam_edge":
        // consumers walking the edge's pcurves now see the whole seam.
        const TAU: f64 = std::f64::consts::TAU;
        let (mut topo, seam, face_id) = make_cylinder_side_face_with_seam();
        topo.set_pcurve_oriented(seam, face_id, true, branch_pcurve(0.0, true))
            .unwrap();
        topo.set_pcurve_oriented(seam, face_id, false, branch_pcurve(TAU, false))
            .unwrap();

        assert_eq!(topo.pcurves_for_edge(seam).len(), 2);
        assert_eq!(topo.pcurves_for_face(face_id).len(), 2);
    }

    #[test]
    fn coedges_resolve_their_own_branch() {
        // The authoritative use identity is also the pcurve storage identity;
        // tuple access below is only a compatibility adapter.
        const TAU: f64 = std::f64::consts::TAU;
        let (mut topo, seam, face_id) = make_cylinder_side_face_with_seam();
        let uses = topo.coedges_of_edge(seam);
        assert_eq!(uses.len(), 2);
        for &coedge_id in &uses {
            let forward = topo.coedge(coedge_id).unwrap().is_forward();
            let branch = if forward {
                branch_pcurve(0.0, true)
            } else {
                branch_pcurve(TAU, false)
            };
            assert!(topo.set_coedge_pcurve(coedge_id, branch).unwrap().is_none());
        }
        for coedge_id in uses {
            let forward = topo.coedge(coedge_id).unwrap().is_forward();
            let pc = topo.coedge_pcurve(coedge_id).unwrap().unwrap();
            let expected_u = if forward { 0.0 } else { TAU };
            assert!(
                (pc.evaluate(0.0).x() - expected_u).abs() < 1e-12,
                "each coedge use resolves its own parameter-space branch"
            );
            assert_eq!(
                topo.pcurve_oriented(seam, face_id, forward)
                    .unwrap()
                    .evaluate(0.0)
                    .x()
                    .to_bits(),
                expected_u.to_bits()
            );
        }
    }

    #[test]
    fn seam_ambiguity_diagnostic_is_pinned() {
        use remus_math::diagnostic::{FailureCategory, ToDiagnostic};
        let (topo, seam, face_id) = make_cylinder_side_face_with_seam();
        drop(topo);
        let d = TopologyError::SeamPcurveAmbiguous {
            edge: seam,
            face: face_id,
        }
        .diagnostic();
        assert_eq!(d.category(), FailureCategory::InvalidTopology);
        assert_eq!(d.code(), "seam_pcurve_ambiguous");
    }
}
