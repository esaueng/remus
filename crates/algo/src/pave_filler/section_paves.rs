//! Put paves on face–face section curves and drop the pieces that lie off a
//! trimmed curved face (B7).
//!
//! Phase FF gives every section curve ONE pave block from its two ends, and
//! [`super::make_blocks`] splits only operand-edge blocks, so a section never
//! learns where it crosses the boundary of a face it lies on. The plane-face
//! paths make up for that per face (line clips, the split registry), and the
//! analytic extent phase FF restricts curved sections with is a height band
//! plus an angular gap. That extent is the face only while the face is a
//! chart rectangle: it knows nothing of a HOLE in a cylinder wall, nor of a
//! BAY an earlier cut left in the wall's outer wire. A section entering
//! either reached both faces whole, each face decided alone what to keep, and
//! the two disagreed.
//!
//! This pass runs after the operand edges have been split and before
//! `link_existing`/`make_split_edges`:
//!
//! 1. For every section curve with a face that is a TRIMMED analytic curved
//!    face (it has hole wires, or its outer wire is not a chart rectangle),
//!    the pave vertices on that face's trimming edges that lie on the curve
//!    (within the weld band: marched geometry is good to ~1e-6) become extra
//!    paves of the curve's block.
//! 2. The block is split at them, exactly as an operand-edge block is.
//! 3. A leaf that lies off such a face (interior samples strictly in a hole
//!    or outside the outer wire, ends off the face or on its boundary) is on
//!    neither face's material and is dropped — for BOTH faces at once.
//!
//! A curve the pass changes is marked [`IntersectionCurveDS::paved`]; its
//! `pave_blocks` then lists the surviving leaves, which the builder feeds as
//! one section per leaf, each carrying its pave-block id. Every other curve
//! is left exactly as phase FF made it — in particular every curve between
//! two faces that are chart rectangles or planes, whose calibrated per-face
//! handling this pass does not touch.

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use remus_math::compact_sort::CompactSort;
use remus_math::det_hash::DetHashMap;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::vertex::VertexId;
use remus_topology::wire::WireId;

use crate::chart_trim::{ChartLoop, chart_loop, chart_v_periodic};
use crate::ds::{GfaArena, IntersectionCurveDS, Pave, PaveBlockId};
use crate::error::AlgoError;

/// Weld-band multiple of the linear tolerance: a pave vertex minted on a
/// marched edge sits up to ~1e-6 off the exact section it lies on.
const WELD_FACTOR: f64 = 100.0;

/// Interior fractions of a leaf probed against the face. A leaf is dropped
/// only when every probe is decisively off the face and neither end is
/// decisively on it.
const LEAF_PROBES: [f64; 3] = [0.25, 0.5, 0.75];

/// An analytic curved face whose trimmed region is not a chart rectangle,
/// with the pave vertices on its trimming edges.
struct TrimmedFace {
    surface: FaceSurface,
    v_periodic: bool,
    /// The outer wire, when it closes in the chart and is not a rectangle
    /// there. `None` leaves the outer boundary to phase FF's extent, which is
    /// exact for a rectangle.
    outer: Option<ChartLoop>,
    /// Inner wires that close in the chart: ordinary holes.
    holes: Vec<ChartLoop>,
    /// Resolved pave vertices on the trimming edges, ascending by id.
    vertices: Vec<(VertexId, Point3)>,
}

impl TrimmedFace {
    /// Whether a section piece lies off this face: every interior probe is
    /// off the trimmed region and clear of every boundary outline by more
    /// than `band` plus the outline's chord error, and neither end is
    /// decisively on the face.
    ///
    /// The end condition is what keeps an unpaved piece whole: a piece that
    /// starts on the face's material and dips off it has a crossing the pave
    /// search missed, and its probes cannot speak for its ends.
    fn excludes(&self, probes: &[Point3], ends: [Point3; 2], band: f64) -> bool {
        let off = |p: Point3| {
            let Some(uv) = self.surface.project_point(p) else {
                return false;
            };
            self.outer
                .as_ref()
                .is_some_and(|outer| !outer.contains(uv, self.v_periodic))
                || self
                    .holes
                    .iter()
                    .any(|hole| hole.contains(uv, self.v_periodic))
        };
        probes
            .iter()
            .all(|&p| off(p) && !self.near_boundary(p, band))
            && ends.iter().all(|&p| off(p) || self.near_boundary(p, band))
    }

    /// Whether `p` is within `band`, plus the outline's certified continuous
    /// chord-error bound, of a boundary. An unavailable bound is infinity,
    /// so it always retains the leaf rather than trusting sampled membership.
    fn near_boundary(&self, p: Point3, band: f64) -> bool {
        self.outer
            .iter()
            .chain(self.holes.iter())
            .any(|outline| outline.distance(p) <= band + 2.0 * outline.sagitta)
    }
}

/// Collect the resolved pave vertices on the edges of `wire_id` into `out`,
/// leaving out seam edges (an edge the wire runs both ways is no boundary of
/// the face's region).
fn trimming_vertices(
    topo: &Topology,
    arena: &GfaArena,
    wire_id: WireId,
    out: &mut BTreeMap<VertexId, Point3>,
) {
    let Ok(wire) = topo.wire(wire_id) else {
        return;
    };
    let mut uses: BTreeMap<EdgeId, usize> = BTreeMap::new();
    for oe in wire.edges() {
        *uses.entry(oe.edge()).or_default() += 1;
    }
    for (&edge_id, &count) in &uses {
        if count != 1 {
            continue;
        }
        let mut ends: Vec<VertexId> = Vec::new();
        if let Some(pb_ids) = arena.edge_pave_blocks.get(&edge_id) {
            for leaf in arena.collect_leaf_pave_blocks(pb_ids) {
                if let Some(pb) = arena.pave_blocks.get(leaf) {
                    ends.push(pb.start.vertex);
                    ends.push(pb.end.vertex);
                }
            }
        } else if let Ok(edge) = topo.edge(edge_id) {
            ends.push(edge.start());
            ends.push(edge.end());
        }
        for vid in ends {
            let vid = arena.resolve_vertex(vid);
            if let Ok(v) = topo.vertex(vid) {
                out.insert(vid, v.point());
            }
        }
    }
}

/// `face_id` as a trimmed curved face; `None` for a plane, a NURBS face, and
/// an analytic face that is a chart rectangle without holes (whose extent
/// phase FF already knows exactly).
fn trimmed_face(
    topo: &Topology,
    arena: &GfaArena,
    face_id: FaceId,
    tol: Tolerance,
) -> Option<TrimmedFace> {
    let face = topo.face(face_id).ok()?;
    let surface = face.surface();
    let v_periodic = chart_v_periodic(surface)?;
    let mut vertices: BTreeMap<VertexId, Point3> = BTreeMap::new();
    let outer = chart_loop(topo, surface, face.outer_wire(), tol)
        .filter(|outline| !outline.is_chart_rectangle());
    if outer.is_some() {
        trimming_vertices(topo, arena, face.outer_wire(), &mut vertices);
    }
    let mut holes = Vec::new();
    for &wire_id in face.inner_wires() {
        if let Some(hole) = chart_loop(topo, surface, wire_id, tol) {
            holes.push(hole);
            trimming_vertices(topo, arena, wire_id, &mut vertices);
        }
    }
    if outer.is_none() && holes.is_empty() {
        return None;
    }
    Some(TrimmedFace {
        surface: surface.clone(),
        v_periodic,
        outer,
        holes,
        vertices: vertices.into_iter().collect(),
    })
}

/// Parameter of the point of `curve` nearest `p` within `(t0, t1)`, and that
/// point. `None` for carriers the pass does not pave.
fn project_on_section(
    curve: &EdgeCurve,
    (t0, t1): (f64, f64),
    (p0, p1): (Point3, Point3),
    p: Point3,
) -> Option<(f64, Point3)> {
    match curve {
        EdgeCurve::Line => {
            let d = p1 - p0;
            let len_sq = d.length_squared();
            if len_sq <= 0.0 {
                return None;
            }
            let s = ((p - p0).dot(d) / len_sq).clamp(0.0, 1.0);
            Some(((t1 - t0).mul_add(s, t0), p0 + d * s))
        }
        EdgeCurve::Circle(c) => {
            let t = t0 + (c.project(p) - t0).rem_euclid(TAU);
            (t <= t1).then(|| (t, curve.evaluate_with_endpoints(t, p0, p1)))
        }
        EdgeCurve::Ellipse(e) => {
            let t = t0 + (e.project(p) - t0).rem_euclid(TAU);
            (t <= t1).then(|| (t, curve.evaluate_with_endpoints(t, p0, p1)))
        }
        EdgeCurve::NurbsCurve(n) => {
            let proj = remus_math::nurbs::projection::project_point_to_curve(n, p, 1e-12).ok()?;
            (proj.parameter >= t0 && proj.parameter <= t1).then_some((proj.parameter, proj.point))
        }
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_) => None,
    }
}

/// Put trimming-edge paves on the section curves of trimmed curved faces,
/// split the curves' pave blocks there, and drop the leaves that lie off
/// such a face.
///
/// # Errors
///
/// Returns [`AlgoError`] if a topology lookup fails.
pub fn perform(topo: &Topology, tol: Tolerance, arena: &mut GfaArena) -> Result<(), AlgoError> {
    let weld = tol.linear * WELD_FACTOR;
    let mut faces: DetHashMap<FaceId, Option<TrimmedFace>> = DetHashMap::default();

    for index in 0..arena.curves.len() {
        let (face_a, face_b) = (arena.curves[index].face_a, arena.curves[index].face_b);
        for face_id in [face_a, face_b] {
            faces
                .entry(face_id)
                .or_insert_with(|| trimmed_face(topo, arena, face_id, tol));
        }
        let trimmed: Vec<&TrimmedFace> = [face_a, face_b]
            .iter()
            .filter_map(|face_id| faces.get(face_id).and_then(Option::as_ref))
            .collect();
        if trimmed.is_empty() {
            continue;
        }
        let Some((root, paves)) = section_paves(topo, arena, &arena.curves[index], &trimmed, weld)?
        else {
            continue;
        };
        let split = !paves.is_empty();
        let leaves = if split {
            if let Some(pb) = arena.pave_blocks.get_mut(root) {
                pb.extra_paves = paves;
            }
            super::make_blocks::split_at_extra_paves(arena, root)
        } else {
            vec![root]
        };
        let mut kept = Vec::with_capacity(leaves.len());
        for &leaf in &leaves {
            if !leaf_off_face(
                topo,
                arena,
                &arena.curves[index].curve,
                leaf,
                &trimmed,
                weld,
            )? {
                kept.push(leaf);
            }
        }
        if !split && kept.len() == leaves.len() {
            continue;
        }
        log::debug!(
            "section_paves: curve#{index} has {} leaf block(s), {} kept",
            leaves.len(),
            kept.len()
        );
        let curve = &mut arena.curves[index];
        curve.pave_blocks = kept;
        curve.paved = true;
    }
    Ok(())
}

/// The root block of `curve_ds` and the trimming-edge pave vertices lying on
/// the curve strictly inside its span, ascending along it. `None` when the
/// curve is not one the pass handles.
fn section_paves(
    topo: &Topology,
    arena: &GfaArena,
    curve_ds: &IntersectionCurveDS,
    trimmed: &[&TrimmedFace],
    weld: f64,
) -> Result<Option<(PaveBlockId, Vec<Pave>)>, AlgoError> {
    // Only a curve still in the state phase FF left it in: one unsplit block.
    let [root] = curve_ds.pave_blocks.as_slice() else {
        return Ok(None);
    };
    let Some(pb) = arena.pave_blocks.get(*root) else {
        return Ok(None);
    };
    if !pb.children.is_empty() || !pb.extra_paves.is_empty() {
        return Ok(None);
    }
    if matches!(
        curve_ds.curve,
        EdgeCurve::Hyperbola(_) | EdgeCurve::Parabola(_)
    ) {
        return Ok(None);
    }
    let (t0, t1) = pb.parameter_range();
    if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
        return Ok(None);
    }
    let start_v = arena.resolve_vertex(pb.start.vertex);
    let end_v = arena.resolve_vertex(pb.end.vertex);
    let p0 = topo.vertex(start_v)?.point();
    let p1 = topo.vertex(end_v)?.point();

    let bbox = super::helpers::conservative_curve_aabb(&curve_ds.curve, p0, p1)
        .map(|bbox| bbox.expanded(weld + super::helpers::coordinate_slack(bbox)));

    let mut hits: Vec<(f64, VertexId, Point3)> = Vec::new();
    for face in trimmed {
        for &(vid, point) in &face.vertices {
            if vid == start_v || vid == end_v || hits.iter().any(|&(_, seen, _)| seen == vid) {
                continue;
            }
            if bbox.is_some_and(|bbox| !bbox.contains_point(point)) {
                continue;
            }
            if (point - p0).length() <= weld || (point - p1).length() <= weld {
                continue;
            }
            let Some((t, foot)) = project_on_section(&curve_ds.curve, (t0, t1), (p0, p1), point)
            else {
                continue;
            };
            if (foot - point).length() > weld || !pb.contains_parameter_interior(t, 1e-10) {
                continue;
            }
            hits.push((t, vid, point));
        }
    }
    // Deterministic order along the curve; ties by vertex id.
    hits.compact_sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.index().cmp(&b.1.index())));
    let mut paves: Vec<Pave> = Vec::new();
    let mut last_point: Option<Point3> = None;
    for (t, vid, point) in hits {
        if last_point.is_some_and(|q| (q - point).length() <= weld) {
            continue;
        }
        paves.push(Pave::new(vid, t));
        last_point = Some(point);
    }
    // A pave marks where the section CROSSES a trimming edge. A section that
    // rides the boundary through a vertex of it (a cap plane's circle lying
    // along a patch's own rim arcs, two of which meet there) crosses nothing
    // at that vertex, and the builder's re-trace handling is calibrated on
    // the whole section: the midpoints of both pieces beside such a vertex
    // lie on the boundary, and the vertex is not paved.
    let at = |t: f64| match &curve_ds.curve {
        EdgeCurve::Line => p0 + (p1 - p0) * ((t - t0) / (t1 - t0)),
        curve @ (EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_)) => curve.evaluate_with_endpoints(t, p0, p1),
    };
    let rides = |t: f64| {
        let p = at(t);
        trimmed.iter().any(|face| face.near_boundary(p, weld))
    };
    let bounds: Vec<f64> = std::iter::once(t0)
        .chain(paves.iter().map(|pave| pave.parameter))
        .chain(std::iter::once(t1))
        .collect();
    let mut crossing = Vec::with_capacity(paves.len());
    for (k, pave) in paves.into_iter().enumerate() {
        let before = f64::midpoint(bounds[k], bounds[k + 1]);
        let after = f64::midpoint(bounds[k + 1], bounds[k + 2]);
        if !(rides(before) && rides(after)) {
            crossing.push(pave);
        }
    }
    Ok(Some((*root, crossing)))
}

/// Whether the leaf block lies off one of the curve's trimmed faces (see
/// [`TrimmedFace::excludes`]).
fn leaf_off_face(
    topo: &Topology,
    arena: &GfaArena,
    curve: &EdgeCurve,
    leaf: PaveBlockId,
    trimmed: &[&TrimmedFace],
    weld: f64,
) -> Result<bool, AlgoError> {
    let Some(pb) = arena.pave_blocks.get(leaf) else {
        return Ok(false);
    };
    let (ta, tb) = pb.parameter_range();
    let start = topo.vertex(arena.resolve_vertex(pb.start.vertex))?.point();
    let end = topo.vertex(arena.resolve_vertex(pb.end.vertex))?.point();
    let probes = LEAF_PROBES.map(|f| match curve {
        EdgeCurve::Line => start + (end - start) * f,
        EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::NurbsCurve(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => {
            curve.evaluate_with_endpoints((tb - ta).mul_add(f, ta), start, end)
        }
    });
    Ok(trimmed
        .iter()
        .any(|face| face.excludes(&probes, [start, end], weld)))
}

#[cfg(test)]
mod tests {
    //! Closed-form oracles on a unit cylinder wall (axis z). The trimming is
    //! a chart rectangle `u ∈ [u0, u0 + 1]` between two heights — a hole in
    //! the wall, or a bay notched into its top rim — so every crossing of a
    //! section with it is known exactly and each expected pave parameter is
    //! written by hand.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::aabb::Aabb3;
    use remus_math::curves::Circle3D;
    use remus_math::surfaces::CylindricalSurface;
    use remus_math::vec::Vec3;
    use remus_topology::edge::Edge;
    use remus_topology::face::Face;
    use remus_topology::vertex::Vertex;
    use remus_topology::wire::{OrientedEdge, Wire};

    use super::*;

    const TOL: f64 = 1e-7;
    const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);

    struct Fixture {
        topo: Topology,
        arena: GfaArena,
        cyl: CylindricalSurface,
        wall: FaceId,
        other: FaceId,
        /// Trimming edges: lower arc, right generator, left generator (and,
        /// for a hole, the upper arc last).
        trim: Vec<EdgeId>,
        /// The wall's seam edge, when its outer wire is cut open.
        seam: Option<EdgeId>,
    }

    fn cylinder() -> CylindricalSurface {
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap()
    }

    /// The iso-`v` circle of `cyl` at height `z`, parameterised by the
    /// cylinder's own `u`.
    fn ring(cyl: &CylindricalSurface, z: f64) -> Circle3D {
        let center = Point3::new(0.0, 0.0, z);
        let circle = Circle3D::new_with_ref(
            center,
            Vec3::new(0.0, 0.0, 1.0),
            1.0,
            cyl.evaluate(0.0, z) - center,
        )
        .unwrap();
        for u in [0.3, 1.7, 4.0] {
            assert!((circle.evaluate(u) - cyl.evaluate(u, z)).length() < 1e-12);
        }
        circle
    }

    fn arc(
        topo: &mut Topology,
        arena: &mut GfaArena,
        cyl: &CylindricalSurface,
        z: f64,
        (from, a): (VertexId, f64),
        (to, b): (VertexId, f64),
    ) -> EdgeId {
        let mut edge = Edge::new(from, to, EdgeCurve::Circle(ring(cyl, z)));
        edge.set_trim(Some((a, b)));
        let edge = topo.add_edge(edge);
        arena.init_edge_pave_block(edge, from, a, to, b);
        edge
    }

    fn line(topo: &mut Topology, arena: &mut GfaArena, from: VertexId, to: VertexId) -> EdgeId {
        let edge = topo.add_edge(Edge::new(from, to, EdgeCurve::Line));
        arena.init_edge_pave_block(edge, from, 0.0, to, 1.0);
        edge
    }

    fn closed_ring_wire(topo: &mut Topology, cyl: &CylindricalSurface, z: f64) -> WireId {
        let seam = topo.add_vertex(Vertex::new(cyl.evaluate(0.0, z), TOL));
        let mut edge = Edge::new(seam, seam, EdgeCurve::Circle(ring(cyl, z)));
        edge.set_trim(Some((0.0, TAU)));
        let edge = topo.add_edge(edge);
        topo.add_wire(Wire::new(vec![OrientedEdge::new(edge, true)], true).unwrap())
    }

    fn other_face(topo: &mut Topology, cyl: &CylindricalSurface) -> FaceId {
        let outer = closed_ring_wire(topo, cyl, 9.0);
        topo.add_face(Face::new(
            outer,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 9.0,
            },
        ))
    }

    /// A wall with the rectangular hole `u ∈ [u0, u0 + 1] × v ∈ [1, 3]`.
    fn holed(u0: f64) -> Fixture {
        let mut topo = Topology::new();
        let mut arena = GfaArena::new();
        let cyl = cylinder();
        let (u1, v0, v1) = (u0 + 1.0, 1.0, 3.0);
        let a = topo.add_vertex(Vertex::new(cyl.evaluate(u0, v0), TOL));
        let b = topo.add_vertex(Vertex::new(cyl.evaluate(u1, v0), TOL));
        let c = topo.add_vertex(Vertex::new(cyl.evaluate(u1, v1), TOL));
        let d = topo.add_vertex(Vertex::new(cyl.evaluate(u0, v1), TOL));
        let bottom = arc(&mut topo, &mut arena, &cyl, v0, (a, u0), (b, u1));
        let right = line(&mut topo, &mut arena, b, c);
        let top = arc(&mut topo, &mut arena, &cyl, v1, (d, u0), (c, u1));
        let left = line(&mut topo, &mut arena, a, d);
        let hole_wire = topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(bottom, true),
                    OrientedEdge::new(right, true),
                    OrientedEdge::new(top, false),
                    OrientedEdge::new(left, false),
                ],
                true,
            )
            .unwrap(),
        );
        let outer = closed_ring_wire(&mut topo, &cyl, 0.0);
        let wall = topo.add_face(Face::new(
            outer,
            vec![hole_wire],
            FaceSurface::Cylinder(cyl.clone()),
        ));
        let other = other_face(&mut topo, &cyl);
        Fixture {
            topo,
            arena,
            cyl,
            wall,
            other,
            trim: vec![bottom, right, left, top],
            seam: None,
        }
    }

    /// A seam-cut wall `v ∈ [0, 4]`, optionally with the bay
    /// `u ∈ [1, 2] × v ∈ [3, 4]` notched into its top rim.
    #[allow(clippy::useless_let_if_seq)]
    fn seam_cut(bay: bool) -> Fixture {
        let mut topo = Topology::new();
        let mut arena = GfaArena::new();
        let cyl = cylinder();
        let s0 = topo.add_vertex(Vertex::new(cyl.evaluate(0.0, 0.0), TOL));
        let s1 = topo.add_vertex(Vertex::new(cyl.evaluate(0.0, 4.0), TOL));
        let bottom = arc(&mut topo, &mut arena, &cyl, 0.0, (s0, 0.0), (s0, TAU));
        let seam = line(&mut topo, &mut arena, s0, s1);
        let mut edges = vec![
            OrientedEdge::new(bottom, true),
            OrientedEdge::new(seam, true),
        ];
        let mut trim = Vec::new();
        if bay {
            let a = topo.add_vertex(Vertex::new(cyl.evaluate(1.0, 3.0), TOL));
            let b = topo.add_vertex(Vertex::new(cyl.evaluate(2.0, 3.0), TOL));
            let c = topo.add_vertex(Vertex::new(cyl.evaluate(2.0, 4.0), TOL));
            let d = topo.add_vertex(Vertex::new(cyl.evaluate(1.0, 4.0), TOL));
            // Top rim, traversed from the seam at u = 2π back to u = 0.
            let rim_far = arc(&mut topo, &mut arena, &cyl, 4.0, (c, 2.0), (s1, TAU));
            let right = line(&mut topo, &mut arena, b, c);
            let floor = arc(&mut topo, &mut arena, &cyl, 3.0, (a, 1.0), (b, 2.0));
            let left = line(&mut topo, &mut arena, a, d);
            let rim_near = arc(&mut topo, &mut arena, &cyl, 4.0, (s1, 0.0), (d, 1.0));
            edges.extend([
                OrientedEdge::new(rim_far, false),
                OrientedEdge::new(right, false),
                OrientedEdge::new(floor, false),
                OrientedEdge::new(left, true),
                OrientedEdge::new(rim_near, false),
            ]);
            trim = vec![floor, right, left];
        } else {
            let top = arc(&mut topo, &mut arena, &cyl, 4.0, (s1, 0.0), (s1, TAU));
            edges.push(OrientedEdge::new(top, false));
        }
        edges.push(OrientedEdge::new(seam, false));
        let outer = topo.add_wire(Wire::new(edges, true).unwrap());
        let wall = topo.add_face(Face::new(outer, vec![], FaceSurface::Cylinder(cyl.clone())));
        let other = other_face(&mut topo, &cyl);
        Fixture {
            topo,
            arena,
            cyl,
            wall,
            other,
            trim,
            seam: Some(seam),
        }
    }

    impl Fixture {
        /// Pave `edge` at `parameter`, as an edge-face crossing would, with
        /// the vertex `offset` off the exact point.
        fn cross(&mut self, edge: EdgeId, parameter: f64, point: Point3, offset: Vec3) -> VertexId {
            let vertex = self.topo.add_vertex(Vertex::new(point + offset, TOL));
            let block = self.arena.edge_pave_blocks[&edge][0];
            self.arena
                .pave_blocks
                .get_mut(block)
                .unwrap()
                .add_extra_pave(Pave::new(vertex, parameter));
            vertex
        }

        /// Register a section curve between the wall and the other face.
        fn section(
            &mut self,
            curve: EdgeCurve,
            range: (f64, f64),
            ends: (Point3, Point3),
        ) -> usize {
            let start = self.topo.add_vertex(Vertex::new(ends.0, TOL));
            let end = if (ends.0 - ends.1).length() < TOL {
                start
            } else {
                self.topo.add_vertex(Vertex::new(ends.1, TOL))
            };
            let mut edge = Edge::new(start, end, curve.clone());
            if !matches!(curve, EdgeCurve::Line) {
                edge.set_trim(Some(range));
            }
            let edge = self.topo.add_edge(edge);
            let block = self.arena.pave_blocks.alloc(crate::ds::PaveBlock::new(
                edge,
                Pave::new(start, range.0),
                Pave::new(end, range.1),
            ));
            self.arena.curves.push(IntersectionCurveDS {
                curve,
                face_a: self.wall,
                face_b: self.other,
                bbox: Aabb3::from_points([ends.0, ends.1]),
                pave_blocks: vec![block],
                t_range: range,
                paved: false,
            });
            self.arena.curves.len() - 1
        }

        /// The generator at chart angle `u` from height `z0` to `z1`, with
        /// the `(0, length)` range phase FF gives a line.
        fn generator(&mut self, u: f64, z0: f64, z1: f64) -> usize {
            let ends = (self.cyl.evaluate(u, z0), self.cyl.evaluate(u, z1));
            self.section(EdgeCurve::Line, (0.0, z1 - z0), ends)
        }

        /// The closed iso-`v` ring at height `z`, seamed at `u = 0`.
        fn ring_section(&mut self, z: f64) -> usize {
            let seam = self.cyl.evaluate(0.0, z);
            let circle = EdgeCurve::Circle(ring(&self.cyl, z));
            self.section(circle, (0.0, TAU), (seam, seam))
        }

        fn run(&mut self) {
            crate::pave_filler::make_blocks::perform(&mut self.arena).unwrap();
            perform(&self.topo, Tolerance::new(), &mut self.arena).unwrap();
        }

        fn ranges(&self, curve: usize) -> Vec<(f64, f64)> {
            self.arena.curves[curve]
                .pave_blocks
                .iter()
                .map(|&id| self.arena.pave_blocks.get(id).unwrap().parameter_range())
                .collect()
        }

        fn block(&self, curve: usize, leaf: usize) -> &crate::ds::PaveBlock {
            let id = self.arena.curves[curve].pave_blocks[leaf];
            self.arena.pave_blocks.get(id).unwrap()
        }

        fn untouched(&self, curve: usize) -> bool {
            let ds = &self.arena.curves[curve];
            let [root] = ds.pave_blocks.as_slice() else {
                return false;
            };
            let block = self.arena.pave_blocks.get(*root).unwrap();
            !ds.paved && block.children.is_empty() && block.extra_paves.is_empty()
        }
    }

    fn assert_ranges(actual: &[(f64, f64)], expected: &[(f64, f64)]) {
        assert_eq!(actual.len(), expected.len(), "{actual:?} vs {expected:?}");
        for (a, e) in actual.iter().zip(expected) {
            assert!(
                (a.0 - e.0).abs() < 1e-9 && (a.1 - e.1).abs() < 1e-9,
                "{actual:?} vs {expected:?}"
            );
        }
    }

    #[test]
    fn generator_through_a_hole_keeps_only_its_two_outer_pieces() {
        let mut f = holed(1.0);
        let (lower, upper) = (f.trim[0], f.trim[3]);
        let below = f.cross(lower, 1.5, f.cyl.evaluate(1.5, 1.0), ZERO);
        let above = f.cross(upper, 1.5, f.cyl.evaluate(1.5, 3.0), ZERO);
        let curve = f.generator(1.5, 0.5, 3.5);
        f.run();

        assert!(f.arena.curves[curve].paved);
        // z = 1 and z = 3 on a line parameterised by length from z = 0.5.
        assert_ranges(&f.ranges(curve), &[(0.0, 0.5), (2.5, 3.0)]);
        // The pieces END ON the hole wire's own pave vertices.
        assert_eq!(f.block(curve, 0).end.vertex, below);
        assert_eq!(f.block(curve, 1).start.vertex, above);
    }

    #[test]
    fn a_pave_vertex_inside_the_weld_band_still_splits_and_one_outside_does_not() {
        let radial = Vec3::new(1.5_f64.cos(), 1.5_f64.sin(), 0.0);
        // 5e-6 off the generator: marched-geometry noise, inside 100·tol.
        let mut near = holed(1.0);
        let (lower, upper) = (near.trim[0], near.trim[3]);
        near.cross(lower, 1.5, near.cyl.evaluate(1.5, 1.0), radial * 5e-6);
        near.cross(upper, 1.5, near.cyl.evaluate(1.5, 3.0), radial * 5e-6);
        let curve = near.generator(1.5, 0.5, 3.5);
        near.run();
        assert_ranges(&near.ranges(curve), &[(0.0, 0.5), (2.5, 3.0)]);

        // 5e-5 off: not this curve's crossing. With no pave the piece starts
        // and ends on material, so the hole may not swallow it.
        let mut far = holed(1.0);
        far.cross(lower, 1.5, far.cyl.evaluate(1.5, 1.0), radial * 5e-5);
        far.cross(upper, 1.5, far.cyl.evaluate(1.5, 3.0), radial * 5e-5);
        let curve = far.generator(1.5, 0.5, 3.5);
        far.run();
        assert!(far.untouched(curve));
    }

    #[test]
    fn closed_ring_through_a_hole_loses_the_arc_inside_it() {
        let mut f = holed(1.0);
        let (right, left) = (f.trim[1], f.trim[2]);
        // The ring at z = 2 crosses the hole's two generators at mid-height.
        let at_left = f.cross(left, 0.5, f.cyl.evaluate(1.0, 2.0), ZERO);
        let at_right = f.cross(right, 0.5, f.cyl.evaluate(2.0, 2.0), ZERO);
        let curve = f.ring_section(2.0);
        f.run();

        assert!(f.arena.curves[curve].paved);
        assert_ranges(&f.ranges(curve), &[(0.0, 1.0), (2.0, TAU)]);
        assert_eq!(f.block(curve, 0).end.vertex, at_left);
        assert_eq!(f.block(curve, 1).start.vertex, at_right);
        // Both surviving arcs meet at the ring's own seam vertex.
        assert_eq!(f.block(curve, 0).start.vertex, f.block(curve, 1).end.vertex);
    }

    #[test]
    fn section_wholly_inside_a_hole_is_dropped_for_both_faces() {
        let mut f = holed(1.0);
        let curve = f.generator(1.5, 1.5, 2.5);
        f.run();
        assert!(f.arena.curves[curve].paved);
        assert!(f.arena.curves[curve].pave_blocks.is_empty());
    }

    #[test]
    fn section_clear_of_the_hole_is_left_as_phase_ff_made_it() {
        let mut f = holed(1.0);
        let lower = f.trim[0];
        f.cross(lower, 1.5, f.cyl.evaluate(1.5, 1.0), ZERO);
        // Beside the hole (u = 3) and below it (u = 1.5, z < 1).
        let beside = f.generator(3.0, 0.5, 3.5);
        let below = f.generator(1.5, 0.2, 0.8);
        f.run();
        assert!(f.untouched(beside));
        assert!(f.untouched(below));
    }

    #[test]
    fn hole_straddling_the_chart_seam_is_read_through_the_period() {
        // The hole spans u ∈ [2π − 0.5, 2π + 0.5]; a generator at u = 0.2 and
        // one at u = 2π − 0.2 both run through it.
        for u in [0.2, TAU - 0.2] {
            let mut f = holed(TAU - 0.5);
            let (lower, upper) = (f.trim[0], f.trim[3]);
            let on_arc = if u < 1.0 { u + TAU } else { u };
            f.cross(lower, on_arc, f.cyl.evaluate(u, 1.0), ZERO);
            f.cross(upper, on_arc, f.cyl.evaluate(u, 3.0), ZERO);
            let curve = f.generator(u, 0.5, 3.5);
            f.run();
            assert_ranges(&f.ranges(curve), &[(0.0, 0.5), (2.5, 3.0)]);
        }
    }

    #[test]
    fn an_inner_wire_that_winds_the_period_is_a_band_rim_not_a_hole() {
        let mut topo = Topology::new();
        let arena = GfaArena::new();
        let cyl = cylinder();
        let outer = closed_ring_wire(&mut topo, &cyl, 0.0);
        let rim = closed_ring_wire(&mut topo, &cyl, 2.0);
        let wall = topo.add_face(Face::new(
            outer,
            vec![rim],
            FaceSurface::Cylinder(cyl.clone()),
        ));
        assert!(trimmed_face(&topo, &arena, wall, Tolerance::new()).is_none());
    }

    #[test]
    fn hole_outline_is_sampled_in_wire_order_with_its_chord_error() {
        let f = holed(1.0);
        let face = trimmed_face(&f.topo, &f.arena, f.wall, Tolerance::new()).unwrap();
        // The fixture's outer wire is a lone ring: it winds, so only the
        // hole trims the face.
        assert!(face.outer.is_none());
        assert_eq!(face.holes.len(), 1);
        let hole = &face.holes[0];
        assert_eq!(hole.uv.len(), 4 * crate::chart_trim::SAMPLES_PER_EDGE);
        assert!((hole.lo.x() - 1.0).abs() < 1e-9 && (hole.lo.y() - 1.0).abs() < 1e-9);
        assert!((hole.hi.x() - 2.0).abs() < 1e-9 && (hole.hi.y() - 3.0).abs() < 1e-9);
        // The arcs are chorded: sagitta of a unit-radius arc of 1/24 rad.
        let expected = 1.0 - (0.5_f64 / 24.0).cos();
        assert!((hole.sagitta - expected).abs() < 1e-12, "{}", hole.sagitta);
        // The four corners, no interior pave vertices yet.
        assert_eq!(face.vertices.len(), 4);
        // A hole that is itself a chart rectangle is still a hole.
        assert!(hole.is_chart_rectangle());
    }

    #[test]
    fn a_seam_cut_wall_without_trimming_is_not_this_pass_s_face() {
        let mut f = seam_cut(false);
        let face = trimmed_face(&f.topo, &f.arena, f.wall, Tolerance::new());
        assert!(face.is_none());
        // So a section on it, even one carrying a seam pave, stays whole.
        let seam = f.seam.unwrap();
        f.cross(seam, 0.5, f.cyl.evaluate(0.0, 2.0), ZERO);
        let curve = f.ring_section(2.0);
        f.run();
        assert!(f.untouched(curve));
    }

    #[test]
    fn generator_into_a_bay_of_the_outer_wire_loses_its_piece_in_the_bay() {
        let mut f = seam_cut(true);
        let floor = f.trim[0];
        let at_floor = f.cross(floor, 1.5, f.cyl.evaluate(1.5, 3.0), ZERO);
        // From the wall, across the bay's floor at z = 3, ending in the bay.
        let through = f.generator(1.5, 0.5, 3.8);
        // And one that never leaves the bay.
        let inside = f.generator(1.5, 3.2, 3.8);
        f.run();

        assert!(f.arena.curves[through].paved);
        assert_ranges(&f.ranges(through), &[(0.0, 2.5)]);
        assert_eq!(f.block(through, 0).end.vertex, at_floor);
        assert!(f.arena.curves[inside].paved);
        assert!(f.arena.curves[inside].pave_blocks.is_empty());
    }

    #[test]
    fn closed_ring_across_a_bay_keeps_the_arcs_on_the_wall() {
        let mut f = seam_cut(true);
        let (right, left) = (f.trim[1], f.trim[2]);
        // The ring at z = 3.5 crosses the bay's two generators (z 3 → 4).
        let at_left = f.cross(left, 0.5, f.cyl.evaluate(1.0, 3.5), ZERO);
        let at_right = f.cross(right, 0.5, f.cyl.evaluate(2.0, 3.5), ZERO);
        let curve = f.ring_section(3.5);
        f.run();

        assert_ranges(&f.ranges(curve), &[(0.0, 1.0), (2.0, TAU)]);
        assert_eq!(f.block(curve, 0).end.vertex, at_left);
        assert_eq!(f.block(curve, 1).start.vertex, at_right);
    }

    #[test]
    fn a_section_riding_the_boundary_is_not_paved_at_a_vertex_it_passes() {
        let mut f = seam_cut(true);
        let floor = f.trim[0];
        // An arc lying ALONG the bay's floor (z = 3, u 1.2 → 1.8), and a
        // vertex paved on that floor at u = 1.5: nothing is crossed there.
        f.cross(floor, 1.5, f.cyl.evaluate(1.5, 3.0), ZERO);
        let ends = (f.cyl.evaluate(1.2, 3.0), f.cyl.evaluate(1.8, 3.0));
        let riding = f.section(EdgeCurve::Circle(ring(&f.cyl, 3.0)), (1.2, 1.8), ends);
        // The ruling through the same vertex does cross the floor there.
        let crossing = f.generator(1.5, 2.5, 3.5);
        f.run();
        assert!(f.untouched(riding));
        assert_ranges(&f.ranges(crossing), &[(0.0, 0.5)]);
    }

    #[test]
    fn a_bayed_wall_offers_its_trimming_vertices_but_not_its_seam_s() {
        let mut f = seam_cut(true);
        let seam = f.seam.unwrap();
        let on_seam = f.cross(seam, 0.5, f.cyl.evaluate(0.0, 2.0), ZERO);
        crate::pave_filler::make_blocks::perform(&mut f.arena).unwrap();
        let face = trimmed_face(&f.topo, &f.arena, f.wall, Tolerance::new()).unwrap();
        let outer = face.outer.as_ref().unwrap();
        assert!(!outer.is_chart_rectangle());
        assert!((outer.lo.x() - 0.0).abs() < 1e-9 && (outer.hi.x() - TAU).abs() < 1e-9);
        assert!((outer.lo.y() - 0.0).abs() < 1e-9 && (outer.hi.y() - 4.0).abs() < 1e-9);
        // Both seam ends (the rims share them) and the bay's four corners;
        // the vertex paved on the seam is on an edge the wire runs both
        // ways, so it is left out.
        assert_eq!(face.vertices.len(), 6);
        assert!(face.vertices.iter().all(|&(vid, _)| vid != on_seam));
        // A section through that seam vertex therefore stays whole.
        let curve = f.ring_section(2.0);
        perform(&f.topo, Tolerance::new(), &mut f.arena).unwrap();
        assert!(f.untouched(curve));
    }

    #[test]
    fn chart_membership_takes_the_periodic_image_inside_the_outline() {
        let f = seam_cut(true);
        let face = trimmed_face(&f.topo, &f.arena, f.wall, Tolerance::new()).unwrap();
        let outer = face.outer.as_ref().unwrap();
        // On the wall, in the bay, and above the top rim.
        assert!(outer.contains((0.5, 3.5), false));
        assert!(!outer.contains((1.5, 3.5), false));
        assert!(!outer.contains((0.5, 4.5), false));
        // The same wall point named one period away.
        assert!(outer.contains((0.5 - TAU, 3.5), false));
        assert!(outer.contains((0.5 + TAU, 3.5), false));
        // Model distance to the outline from the bay's middle: the bay's
        // generators at u = 1 and u = 2 are a chord of half a radian away,
        // nearer than its floor 0.5 below.
        let in_bay = f.cyl.evaluate(1.5, 3.5);
        let chord = 2.0 * 0.25_f64.sin();
        assert!((outer.distance(in_bay) - chord).abs() < 1e-12);
    }

    // Review oracle: the exact rational quadratic cylinder arc has a height
    // peak z = 2.496243741134287 at u = 1.5. Only its knot spacing changes;
    // the whole generator z = 1.5..2.0 stays below the hole on material.
    #[allow(clippy::too_many_lines)]
    fn material_below_rational_hole_bump(k0: f64, k1: f64, reversed: bool) {
        let mut f = seam_cut(false);
        let mut cps = Vec::new();
        let mut weights = Vec::new();
        for (k, (a, b)) in [(1.0_f64, 1.4_f64), (1.4, 1.6), (1.6, 2.0)]
            .into_iter()
            .enumerate()
        {
            let mid = f64::midpoint(a, b);
            let weight = ((b - a) / 2.0).cos();
            if k == 0 {
                cps.push(f.cyl.evaluate(a, 1.0));
                weights.push(1.0);
            }
            let c = f.cyl.evaluate(mid, 0.0);
            cps.push(Point3::new(
                c.x() / weight,
                c.y() / weight,
                if k == 1 { 4.0 } else { 1.0 },
            ));
            weights.push(weight);
            cps.push(f.cyl.evaluate(b, 1.0));
            weights.push(1.0);
        }
        let curve = remus_math::nurbs::NurbsCurve::new(
            2,
            vec![0., 0., 0., k0, k0, k1, k1, 1., 1., 1.],
            cps,
            weights,
        )
        .unwrap();
        let peak = curve.evaluate(f64::midpoint(k0, k1));
        let (peak_u, peak_v) = f.cyl.project_point(peak);
        assert!((peak - f.cyl.evaluate(peak_u, peak_v)).length() < 1e-12);
        assert!((peak_u - 1.5).abs() < 1e-12 && peak_v > 2.49);
        let a = f.topo.add_vertex(Vertex::new(f.cyl.evaluate(1., 1.), TOL));
        let b = f.topo.add_vertex(Vertex::new(f.cyl.evaluate(2., 1.), TOL));
        let c = f.topo.add_vertex(Vertex::new(f.cyl.evaluate(2., 3.), TOL));
        let d = f.topo.add_vertex(Vertex::new(f.cyl.evaluate(1., 3.), TOL));
        let mut bottom = if reversed {
            Edge::new(b, a, EdgeCurve::NurbsCurve(curve))
        } else {
            Edge::new(a, b, EdgeCurve::NurbsCurve(curve))
        };
        bottom.set_trim(Some(if reversed { (1., 0.) } else { (0., 1.) }));
        let bottom = f.topo.add_edge(bottom);
        if reversed {
            f.arena.init_edge_pave_block(bottom, b, 1., a, 0.);
        } else {
            f.arena.init_edge_pave_block(bottom, a, 0., b, 1.);
        }
        let right = line(&mut f.topo, &mut f.arena, b, c);
        let top = arc(&mut f.topo, &mut f.arena, &f.cyl, 3., (d, 1.), (c, 2.));
        let left = line(&mut f.topo, &mut f.arena, a, d);
        let hole = f.topo.add_wire(
            Wire::new(
                vec![
                    OrientedEdge::new(left, true),
                    OrientedEdge::new(top, true),
                    OrientedEdge::new(right, false),
                    OrientedEdge::new(bottom, reversed),
                ],
                true,
            )
            .unwrap(),
        );
        let outer = f.topo.face(f.wall).unwrap().outer_wire();
        f.wall = f.topo.add_face(Face::new(
            outer,
            vec![hole],
            FaceSurface::Cylinder(f.cyl.clone()),
        ));
        let trimmed = trimmed_face(&f.topo, &f.arena, f.wall, Tolerance::new()).unwrap();
        let outline = &trimmed.holes[0];
        assert!(
            outline.sagitta.is_finite(),
            "the certificate must be available"
        );
        assert!(outline.distance(peak) <= outline.sagitta);
        // The second face is a radial plane rectangle, so this generator is an
        // actual carrier intersection, bounded at the rectangle's two heights.
        let radial = f.cyl.evaluate(1.5, 0.0) - Point3::new(0., 0., 0.);
        let positions = [
            Point3::new(-2. * radial.x(), -2. * radial.y(), 1.5),
            Point3::new(2. * radial.x(), 2. * radial.y(), 1.5),
            Point3::new(2. * radial.x(), 2. * radial.y(), 2.),
            Point3::new(-2. * radial.x(), -2. * radial.y(), 2.),
        ];
        let vertices = positions.map(|p| f.topo.add_vertex(Vertex::new(p, TOL)));
        let mut edges = Vec::new();
        for k in 0..4 {
            edges.push(OrientedEdge::new(
                line(
                    &mut f.topo,
                    &mut f.arena,
                    vertices[k],
                    vertices[(k + 1) % 4],
                ),
                true,
            ));
        }
        let wire = f.topo.add_wire(Wire::new(edges, true).unwrap());
        f.other = f.topo.add_face(Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(-radial.y(), radial.x(), 0.),
                d: 0.,
            },
        ));
        let section = f.generator(1.5, 1.5, 2.0);
        f.run();

        assert_eq!(
            f.ranges(section),
            vec![(0.0, 0.5)],
            "The whole generator is material below the exact hole boundary z>2.49"
        );
    }

    #[test]
    fn narrow_knot_hole_bump_keeps_the_material_generator() {
        for reversed in [false, true] {
            material_below_rational_hole_bump(0.001, 0.002, reversed);
        }
    }

    #[test]
    fn wide_knot_hole_bump_keeps_the_same_material_generator() {
        for reversed in [false, true] {
            material_below_rational_hole_bump(1.0 / 3.0, 2.0 / 3.0, reversed);
        }
    }
}
