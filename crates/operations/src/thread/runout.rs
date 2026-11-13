//! Cone runouts at the ends of a thread, built into the ring directly.
//!
//! A runout is the cone `r_c(z)` that sits on the thread's crest at the end
//! plane and falls to its root `depth / tan(angle)` into the thread. The
//! thread's material near that end is bounded by the larger of the thread
//! surface and the cone (the smaller, for an internal thread): the groove
//! fills up to the cone and fades out on the end plane, with no undercut
//! against the material beyond it.
//!
//! The zone the cone crosses is one twisted extrusion `E`, the mirror of a
//! chamfer's (`chamfer`): crest faces stay whole, root faces are gone, and
//! each flank keeps the triangle on the crest side of the cone. Where a flank
//! meets the cone is the same conical helix, from the flank's root corner on
//! the root plane to its crest corner on the end plane.
//!
//! The cone bounds the material over each groove: per start, the patch
//! between the root arc on the root plane, the two flank trims and an arc of
//! the crest circle on the end plane. Radius, height and angle are linear
//! along a trim, so in the cone's own `(angle, height)` chart the patch is a
//! trapezoid with straight sides. Its angular extent is the root arc plus the
//! zone's twist plus a flank, which passes a full turn on an ordinary
//! single-start thread, and a cone face cannot hold more than one turn of its
//! surface. So the patch is cut along generators of the cone (exact lines)
//! into pieces of at most [`MAX_PIECE_TURN`], the root arc excepted: that arc
//! is shared with the full-depth thread beyond the root plane and is never
//! split, so one piece spans at least it.
//!
//! On the end plane the ring's cap is the whole annulus out to the crest
//! circle, bounded by the crest arcs and the patches' arcs.

use std::f64::consts::TAU;

use remus_math::compact_sort::CompactSort;
use remus_math::curves::Circle3D;
use remus_math::tolerance::Tolerance;
use remus_math::vec::Point3;
use remus_topology::Topology;
use remus_topology::edge::{Edge, EdgeCurve, EdgeId};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::vertex::{Vertex, VertexId};
use remus_topology::wire::{OrientedEdge, Wire};

use super::chamfer::{
    Cone, RootSide, Zone, conical_helix, edge_run, effective_use, oriented_ends, retrim_flank,
    thread_wire,
};
use super::profile::PieceRole;
use super::{Frame, ThreadEndSide, ThreadError, ThreadFaceRole, ThreadKind};
use crate::extrude::twist::TwistedExtrusion;

/// Largest turn of one cone piece about the axis, in turns, where the root
/// arc does not force a longer one (see the module docs).
pub(super) const MAX_PIECE_TURN: f64 = 0.5;

/// Clearance every generator keeps from a patch corner it does not pass
/// through, as a fraction of the cut spacing: no piece gets a sliver edge.
const CUT_MARGIN: f64 = 0.05;

/// The generators that cut a patch, as angles in the patch's own chart.
///
/// The chart runs the root arc from `0` to `bottom` and the end-plane arc
/// from `top.0` to `top.1` (`top.1 − top.0 ≥ bottom ≥ 0`), all in radians the
/// way the end-plane arc turns. The cuts are sorted and lie strictly inside
/// the patch's extent. None splits the root arc: a cut is `0` or `bottom`
/// exactly (it passes through that corner), or stays [`CUT_MARGIN`] of the
/// spacing clear of the arc. Every cut stays that far from the top corners
/// too, so each meets the patch's boundary well inside an edge.
pub(super) fn cut_angles(bottom: f64, top: (f64, f64), max_width: f64) -> Vec<f64> {
    let (t1, t2) = top;
    let lo = t1.min(0.0);
    let hi = t2.max(bottom);
    let range = hi - lo;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let count = (range / max_width).ceil().max(1.0) as usize;
    if count < 2 {
        return Vec::new();
    }
    #[allow(clippy::cast_precision_loss)]
    let step = range / count as f64;
    let margin = CUT_MARGIN * step;
    // Where no cut may fall: across the root arc and around every corner.
    let mut forbidden = [
        (-margin, bottom + margin),
        (t1 - margin, t1 + margin),
        (t2 - margin, t2 + margin),
    ];
    forbidden.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64)> = Vec::with_capacity(3);
    for interval in forbidden {
        match merged.last_mut() {
            // Intervals closer than a margin merge, so two of their ends
            // never leave a sliver between them.
            Some(last) if interval.0 <= last.1 + margin => last.1 = last.1.max(interval.1),
            _ => merged.push(interval),
        }
    }
    let inside = |c: f64| c > lo + margin && c < hi - margin;
    // A cut through a root corner, where that corner is inside the extent
    // and clear of the top corners.
    let through = |corner: f64| {
        inside(corner) && (corner - t1).abs() >= margin && (corner - t2).abs() >= margin
    };
    let corners: Vec<f64> = if bottom > 0.0 {
        vec![0.0, bottom]
    } else {
        vec![0.0]
    }
    .into_iter()
    .filter(|&corner| through(corner))
    .collect();
    let mut cuts: Vec<f64> = Vec::new();
    for index in 1..count {
        #[allow(clippy::cast_precision_loss)]
        let grid = lo + step * index as f64;
        let Some(&(from, to)) = merged.iter().find(|&&(from, to)| grid > from && grid < to) else {
            cuts.push(grid);
            continue;
        };
        // The nearest allowed angle on either side of a forbidden one.
        let left = corners
            .iter()
            .copied()
            .filter(|&corner| corner <= grid && corner >= from)
            .fold(from, f64::max);
        let right = corners
            .iter()
            .copied()
            .filter(|&corner| corner >= grid && corner <= to)
            .fold(to, f64::min);
        cuts.extend([left, right].into_iter().filter(|&c| inside(c)));
    }
    cuts.compact_sort_by(f64::total_cmp);
    let mut kept: Vec<f64> = Vec::with_capacity(cuts.len());
    for cut in cuts {
        match kept.last() {
            Some(&last) if cut - last < 0.5 * margin => {}
            _ => kept.push(cut),
        }
    }
    kept
}

/// A point of a patch's boundary: a corner of the patch or the end of a
/// generator, at angle `phi` in the patch's chart.
#[derive(Debug, Clone, Copy)]
struct Node {
    phi: f64,
    vertex: VertexId,
}

/// A boundary edge between two consecutive nodes of a chain; `forward` when
/// it runs from the first of them to the second.
#[derive(Debug, Clone, Copy)]
struct Link {
    edge: EdgeId,
    forward: bool,
}

/// A chain of boundary nodes in increasing chart angle.
#[derive(Debug, Clone, Default)]
struct Chain {
    nodes: Vec<Node>,
    links: Vec<Link>,
}

impl Chain {
    fn start(node: Node) -> Self {
        Self {
            nodes: vec![node],
            links: Vec::new(),
        }
    }

    /// Append `other`, whose first node is this chain's last.
    fn extend(&mut self, other: &Self) {
        self.nodes.extend(other.nodes.iter().skip(1));
        self.links.extend(&other.links);
    }

    /// The chain run the other way.
    fn reversed(&self) -> Self {
        Self {
            nodes: self.nodes.iter().rev().copied().collect(),
            links: self
                .links
                .iter()
                .rev()
                .map(|link| Link {
                    edge: link.edge,
                    forward: !link.forward,
                })
                .collect(),
        }
    }

    fn index_at(&self, phi: f64) -> Option<usize> {
        self.nodes
            .iter()
            .position(|node| (node.phi - phi).abs() <= 1e-12)
    }
}

/// One groove's cone patch: the descending flank `first`, the root land
/// between (none for a sharp root) and the ascending flank `second`, as
/// positions of the thread wire.
#[derive(Debug, Clone, Copy)]
struct Patch {
    first: usize,
    root: Option<usize>,
    second: usize,
}

/// Fill the thread up to the runout cone in `e`, the twisted extrusion
/// across the zone between the cone's root plane and the end plane. `e`'s
/// root faces and its cap on the end plane are replaced; its cap on the root
/// plane is left to the caller (it is the next segment's profile, or the
/// previous segment's cap).
#[allow(clippy::too_many_lines, clippy::too_many_arguments)]
pub(super) fn runout_zone(
    topo: &mut Topology,
    frame: &Frame,
    e: &TwistedExtrusion,
    cone: &Cone,
    root_side: RootSide,
    rate: f64,
    tolerance: f64,
    piece_roles: &[PieceRole],
    side_of_thread: ThreadEndSide,
) -> Result<Zone, ThreadError> {
    let tol = Tolerance::new().linear;
    let invalid = |what: &str| ThreadError::invalid(format!("runout construction: {what}"));
    let c = &e.construction;
    let (wire, sides, seam_wall) = thread_wire(c, cone.kind)?;
    let n = sides.len();
    let (root_verts, full_verts, root_edges, full_edges, old_cap) = match root_side {
        RootSide::Bottom => (
            wire.verts(),
            wire.top_verts(),
            wire.edge_ids(),
            wire.top_edge_ids(),
            c.top,
        ),
        RootSide::Top => (
            wire.top_verts(),
            wire.verts(),
            wire.top_edge_ids(),
            wire.edge_ids(),
            c.bottom,
        ),
    };
    let vertical = wire.vertical_edge_ids();
    if root_verts.len() != n
        || full_verts.len() != n
        || vertical.len() != n
        || piece_roles.len() != n
    {
        return Err(invalid("mismatched construction record"));
    }
    // (radius, angle, height) in the thread's frame.
    let local = |p: Point3| {
        let offset = p - frame.origin;
        let (x, y) = (offset.dot(frame.x), offset.dot(frame.y));
        (x.hypot(y), y.atan2(x), offset.dot(frame.axis))
    };
    let place = |radius: f64, angle: f64, height: f64| {
        let (sin, cos) = angle.sin_cos();
        frame.origin + frame.x * (radius * cos) + frame.y * (radius * sin) + frame.axis * height
    };
    let is_flank = |i: usize| {
        matches!(
            piece_roles[i],
            PieceRole::UpperFlank | PieceRole::LowerFlank
        )
    };
    // The positions of a flank's root and crest corners.
    let corners = |topo: &Topology, i: usize| -> Result<(usize, usize), ThreadError> {
        let j = (i + 1) % n;
        let ri = local(topo.vertex(wire.verts()[i])?.point()).0;
        let rj = local(topo.vertex(wire.verts()[j])?.point()).0;
        // The root is the smaller radius outside, the larger inside.
        Ok(if (ri < rj) == (cone.kind == ThreadKind::External) {
            (i, j)
        } else {
            (j, i)
        })
    };
    let run_from = |topo: &Topology, edge: EdgeId, from: VertexId| -> Result<f64, ThreadError> {
        edge_run(topo, frame, edge, from)
    };

    let mut patches = Vec::new();
    for i in 0..n {
        if !is_flank(i) || corners(topo, i)?.1 != i {
            continue;
        }
        let next = (i + 1) % n;
        let (root, second) = if piece_roles[next] == PieceRole::Root {
            (Some(next), (next + 1) % n)
        } else {
            (None, next)
        };
        if !is_flank(second) || corners(topo, second)?.0 != second {
            return Err(invalid("a groove is not two flanks about a root"));
        }
        patches.push(Patch {
            first: i,
            root,
            second,
        });
    }
    if patches.is_empty() {
        return Err(invalid("the thread has no groove"));
    }

    let surface = FaceSurface::Cone(cone.surface(frame)?);
    let probe = topo.vertex(root_verts[patches[0].second])?.point();
    let (u, v) = surface
        .project_point(probe)
        .ok_or_else(|| invalid("cannot project onto the cone"))?;
    let reversed = surface.normal(u, v).dot(cone.outward(frame, probe)?) < 0.0;
    let end_center = frame.origin + frame.axis * cone.end_z;
    let edge_tol = |bound: f64| tolerance + bound + e.deviation_bound;

    let mut faces = Vec::new();
    let mut roles = Vec::new();
    let mut trim_bound: f64 = 0.0;
    // The end-plane arcs of each patch, in increasing chart angle, with the
    // direction the cone's pieces run them in.
    let mut top_chains: Vec<(Chain, bool)> = Vec::with_capacity(patches.len());
    for patch in &patches {
        let a1 = root_verts[(patch.first + 1) % n];
        let c1 = full_verts[patch.first];
        let a2 = root_verts[patch.second];
        let c2 = full_verts[(patch.second + 1) % n];
        // A flank's turn from its root corner on the root plane to its crest
        // corner on the end plane: its own run on one plane plus the twist
        // between the planes, as the multiple of a turn the corners fix.
        let turn_of = |topo: &Topology, position: usize| -> Result<f64, ThreadError> {
            let (root_at, crest_at) = corners(topo, position)?;
            let own = run_from(topo, wire.edge_ids()[position], wire.verts()[root_at])?;
            let (pa, pc) = (
                local(topo.vertex(root_verts[root_at])?.point()),
                local(topo.vertex(full_verts[crest_at])?.point()),
            );
            let expected = rate.mul_add(pc.2 - pa.2, own);
            let raw = pc.1 - pa.1;
            Ok(raw + TAU * ((expected - raw) / TAU).round())
        };
        let turn1 = turn_of(topo, patch.first)?;
        let turn2 = turn_of(topo, patch.second)?;
        let bottom_run = match patch.root {
            Some(root) => run_from(topo, root_edges[root], a1)?,
            None => 0.0,
        };
        let top_run = bottom_run + turn2 - turn1;
        let dir = top_run.signum();
        let (bottom, t1, t2) = (dir * bottom_run, dir * turn1, dir * (bottom_run + turn2));
        if !(top_run.abs() > 0.0 && bottom >= -1e-9 && t2 - t1 <= TAU * (1.0 + 1e-9)) {
            return Err(invalid("a groove does not open toward the end plane"));
        }
        let bottom = bottom.max(0.0);
        let cuts = cut_angles(bottom, (t1, t2), MAX_PIECE_TURN * TAU);
        let (pa1, pc1) = (
            local(topo.vertex(a1)?.point()),
            local(topo.vertex(c1)?.point()),
        );
        let (pa2, pc2) = (
            local(topo.vertex(a2)?.point()),
            local(topo.vertex(c2)?.point()),
        );
        let angle_at = |phi: f64| dir.mul_add(phi, pa1.1);

        // A flank's trim from its root corner at chart angle `from` to its
        // crest corner at `to`, split where the cuts cross it, and the flank
        // face rebuilt on the crest side of it.
        let mut trim_side = |topo: &mut Topology,
                             position: usize,
                             (root_corner, crest_corner): (VertexId, VertexId),
                             (pa, pc): ((f64, f64, f64), (f64, f64, f64)),
                             (from, to): (f64, f64)|
         -> Result<(Chain, bool), ThreadError> {
            let mut lambdas: Vec<(f64, f64)> = cuts
                .iter()
                .copied()
                .filter(|&cut| (cut - from) * (cut - to) < 0.0)
                .map(|cut| ((cut - from) / (to - from), cut))
                .collect();
            lambdas.compact_sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut nodes = vec![Node {
                phi: from,
                vertex: root_corner,
            }];
            for &(lambda, cut) in &lambdas {
                let point = place(
                    (pc.0 - pa.0).mul_add(lambda, pa.0),
                    angle_at(cut),
                    (pc.2 - pa.2).mul_add(lambda, pa.2),
                );
                nodes.push(Node {
                    phi: cut,
                    vertex: topo.add_vertex(Vertex::new(point, tol)),
                });
            }
            nodes.push(Node {
                phi: to,
                vertex: crest_corner,
            });
            let mut links = Vec::with_capacity(nodes.len() - 1);
            for pair in nodes.windows(2) {
                let (p, q) = (
                    topo.vertex(pair[0].vertex)?.point(),
                    topo.vertex(pair[1].vertex)?.point(),
                );
                let turn = dir * (pair[1].phi - pair[0].phi);
                let (fit, bound) = conical_helix(frame, p, q, turn, tolerance)?;
                trim_bound = trim_bound.max(bound);
                let mut edge = Edge::with_tolerance(
                    pair[0].vertex,
                    pair[1].vertex,
                    EdgeCurve::NurbsCurve(fit.clone()),
                    Some(edge_tol(bound)),
                );
                edge.set_trim(Some(fit.domain()));
                links.push(Link {
                    edge: topo.add_edge(edge),
                    forward: true,
                });
            }
            let (root_at, _) = corners(topo, position)?;
            let removed = [full_edges[position], vertical[root_at]];
            let chain: Vec<EdgeId> = links.iter().map(|link| link.edge).collect();
            let (face, used) = retrim_flank(
                topo,
                sides[position],
                removed,
                (root_corner, crest_corner),
                &chain,
            )?;
            faces.push(face);
            roles.push((face, piece_roles[position].into()));
            Ok((Chain { nodes, links }, used))
        };
        let (side1, used1) = trim_side(topo, patch.first, (a1, c1), (pa1, pc1), (0.0, t1))?;
        let (side2, used2) = trim_side(topo, patch.second, (a2, c2), (pa2, pc2), (bottom, t2))?;

        // The arc of the crest circle on the end plane, split at the cuts.
        // Every piece runs counter-clockwise about the axis.
        let mut top = Chain::start(Node {
            phi: t1,
            vertex: c1,
        });
        let mut inner: Vec<f64> = cuts
            .iter()
            .copied()
            .filter(|&cut| cut > t1 && cut < t2)
            .collect();
        inner.compact_sort_by(f64::total_cmp);
        for cut in inner {
            let point = place(pc1.0, angle_at(cut), pc1.2);
            top.nodes.push(Node {
                phi: cut,
                vertex: topo.add_vertex(Vertex::new(point, tol)),
            });
        }
        top.nodes.push(Node {
            phi: t2,
            vertex: c2,
        });
        for pair in 0..top.nodes.len() - 1 {
            let (from, to) = (top.nodes[pair], top.nodes[pair + 1]);
            let (start, end) = if dir > 0.0 { (from, to) } else { (to, from) };
            let angle = angle_at(start.phi);
            let radial = frame.x * angle.cos() + frame.y * angle.sin();
            let circle = Circle3D::new_with_ref(end_center, frame.axis, pc1.0, radial)?;
            let t0 = circle.project(end_center + radial * pc1.0);
            let mut edge = Edge::with_tolerance(
                start.vertex,
                end.vertex,
                EdgeCurve::Circle(circle),
                Some(tol),
            );
            edge.set_trim(Some((t0, t0 + (to.phi - from.phi))));
            top.links.push(Link {
                edge: topo.add_edge(edge),
                forward: dir > 0.0,
            });
        }

        // The patch's lower and upper boundary, each in increasing chart
        // angle, and the trims that close it where a side is a generator.
        let corner = |phi: f64, vertex: VertexId| Node { phi, vertex };
        let mut lower = if t1 < 0.0 {
            side1.reversed()
        } else {
            Chain::start(corner(0.0, a1))
        };
        if let Some(root) = patch.root {
            lower.nodes.push(corner(bottom, a2));
            lower.links.push(Link {
                edge: root_edges[root],
                forward: topo.edge(root_edges[root])?.start() == a1,
            });
        }
        if t2 > bottom {
            lower.extend(&side2);
        }
        let mut upper = if t1 > 0.0 {
            side1.clone()
        } else {
            Chain::start(corner(t1, c1))
        };
        upper.extend(&top);
        if t2 < bottom {
            upper.extend(&side2.reversed());
        }
        #[allow(clippy::float_cmp)]
        let left_closure = (t1 == 0.0).then(|| side1.links[0]);
        #[allow(clippy::float_cmp)]
        let right_closure = (t2 == bottom).then(|| side2.links[0]);

        // The generators.
        let mut ends = Vec::with_capacity(cuts.len());
        let mut chords = Vec::with_capacity(cuts.len());
        for &cut in &cuts {
            let (Some(low), Some(high)) = (lower.index_at(cut), upper.index_at(cut)) else {
                return Err(invalid("a generator misses the patch boundary"));
            };
            chords.push(topo.add_edge(Edge::new(
                lower.nodes[low].vertex,
                upper.nodes[high].vertex,
                EdgeCurve::Line,
            )));
            ends.push((low, high));
        }

        // The cone's pieces run the first trim against its flank face; in the
        // chart's counter-clockwise order that trim runs crest to root.
        let flip = !used1;
        if used2 != flip {
            return Err(invalid("the flanks of a groove disagree on its side"));
        }
        if let Some(root) = patch.root {
            let used = effective_use(topo, sides[root], root_edges[root])?
                .ok_or_else(|| invalid("root face misses its arc"))?;
            let forward = topo.edge(root_edges[root])?.start() == a1;
            // The cone takes the root face's place against the root arc.
            if (forward != flip) != used {
                return Err(invalid("the root arc runs against its groove"));
            }
        }
        for slab in 0..=cuts.len() {
            let (low0, high0) = if slab == 0 { (0, 0) } else { ends[slab - 1] };
            let (low1, high1) = if slab == cuts.len() {
                (lower.nodes.len() - 1, upper.nodes.len() - 1)
            } else {
                ends[slab]
            };
            let mut ring: Vec<Link> = lower.links[low0..low1].to_vec();
            if slab == cuts.len() {
                ring.extend(right_closure);
            } else {
                ring.push(Link {
                    edge: chords[slab],
                    forward: true,
                });
            }
            ring.extend(upper.links[high0..high1].iter().rev().map(|link| Link {
                edge: link.edge,
                forward: !link.forward,
            }));
            if slab == 0 {
                ring.extend(left_closure.map(|link| Link {
                    edge: link.edge,
                    forward: !link.forward,
                }));
            } else {
                ring.push(Link {
                    edge: chords[slab - 1],
                    forward: false,
                });
            }
            if ring.is_empty() {
                return Err(invalid("an empty cone piece"));
            }
            // `ring` is the piece's loop counter-clockwise in the chart; the
            // face stores it the way its surface's own normal sees it.
            let stored: Vec<OrientedEdge> = if flip == reversed {
                ring.iter()
                    .map(|link| OrientedEdge::new(link.edge, link.forward))
                    .collect()
            } else {
                ring.iter()
                    .rev()
                    .map(|link| OrientedEdge::new(link.edge, !link.forward))
                    .collect()
            };
            let piece_wire = topo.add_wire(Wire::new(stored, true)?);
            let face = topo.add_face(if reversed {
                Face::new_reversed(piece_wire, vec![], surface.clone())
            } else {
                Face::new(piece_wire, vec![], surface.clone())
            });
            faces.push(face);
            roles.push((face, ThreadFaceRole::Runout(side_of_thread)));
        }
        // The pieces run the end-plane arc down the chart when they keep the
        // chart's order, up it when they are flipped.
        top_chains.push((top, flip));
    }

    for (i, &side) in sides.iter().enumerate() {
        if piece_roles[i] == PieceRole::Crest {
            faces.push(side);
            roles.push((side, ThreadFaceRole::Crest));
        }
    }

    // The end cap: the old cap's thread loop with every groove's run of
    // flank and root edges replaced by that groove's arcs.
    let cap = topo.face(old_cap)?.clone();
    let loops: Vec<_> = std::iter::once(cap.outer_wire())
        .chain(cap.inner_wires().iter().copied())
        .collect();
    let mut thread_loop = None;
    for (k, &id) in loops.iter().enumerate() {
        if topo
            .wire(id)?
            .edges()
            .iter()
            .any(|oe| full_edges.contains(&oe.edge()))
        {
            thread_loop = Some(k);
        }
    }
    let thread_loop = thread_loop.ok_or_else(|| invalid("the end cap misses the thread"))?;
    let mut old = topo.wire(loops[thread_loop])?.edges().to_vec();
    let position_of = |edge: EdgeId| full_edges.iter().position(|&e| e == edge);
    // Start the walk on the first edge of a groove's run: a flank left from
    // its crest corner.
    let mut first = None;
    for (k, oe) in old.iter().enumerate() {
        let Some(position) = position_of(oe.edge()) else {
            return Err(invalid("the end cap holds a foreign edge"));
        };
        if is_flank(position) {
            let (_, crest_at) = corners(topo, position)?;
            if oriented_ends(topo, *oe)?.0 == full_verts[crest_at] {
                first = Some(k);
                break;
            }
        }
    }
    old.rotate_left(first.ok_or_else(|| invalid("the end cap has no groove"))?);
    let mut rebuilt: Vec<OrientedEdge> = Vec::with_capacity(old.len());
    let mut k = 0;
    while k < old.len() {
        let position = position_of(old[k].edge()).ok_or_else(|| invalid("a foreign edge"))?;
        if piece_roles[position] == PieceRole::Crest {
            rebuilt.push(old[k]);
            k += 1;
            continue;
        }
        let Some(index) = patches
            .iter()
            .position(|patch| patch.first == position || patch.second == position)
        else {
            return Err(invalid("a root outside every groove"));
        };
        let patch = patches[index];
        let (top, flip) = &top_chains[index];
        // The loop runs the arcs up the chart when it enters the groove by
        // its descending flank.
        let upward = patch.first == position;
        let cap_effective_upward = upward != cap.is_reversed();
        // The cone's pieces must run them the other way.
        if cap_effective_upward == *flip {
            return Err(invalid("the end cap runs with the cone"));
        }
        if upward {
            rebuilt.extend(
                top.links
                    .iter()
                    .map(|link| OrientedEdge::new(link.edge, link.forward)),
            );
        } else {
            rebuilt.extend(
                top.links
                    .iter()
                    .rev()
                    .map(|link| OrientedEdge::new(link.edge, !link.forward)),
            );
        }
        k += if patch.root.is_some() { 3 } else { 2 };
    }
    let new_loop = topo.add_wire(Wire::new(rebuilt, true)?);
    let mut wires = loops;
    wires[thread_loop] = new_loop;
    let (outer, inners) = (wires[0], wires[1..].to_vec());
    let cap_surface = cap.surface().clone();
    faces.push(topo.add_face(if cap.is_reversed() {
        Face::new_reversed(outer, inners, cap_surface)
    } else {
        Face::new(outer, inners, cap_surface)
    }));

    Ok(Zone {
        faces,
        walls: vec![seam_wall],
        trim_bound,
        roles,
    })
}
