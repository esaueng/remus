//! Simple sampled outlines for planar faces (B276, B277).
//!
//! A planar face is triangulated from its boundary chains as they were
//! sampled, and that sampled outline must be a simple polygon: the CDT
//! cannot recover two crossing boundary constraints without a Steiner vertex
//! that no neighbour shares, so the planar mesher gives up on such an outline
//! and fans it, and a fan over a self-crossing outline winds part of the face
//! the wrong way. The exact boundary never crosses itself, but its chords can:
//! where two boundary curves leave a vertex at a small angle (the crescent a
//! thread's flank cuts out of a collar shoulder, between the shoulder's rim
//! circle and the thread's crest), a coarse chord of one curve passes the
//! dense samples of the other before the two have separated by the chord's
//! sag.
//!
//! [`refine_planar_crossings`] bisects, on the curve itself, the longer
//! curved chord of every crossing pair until no two chords of any planar face
//! cross. The new samples go into the shared chains, so every face on the
//! edge picks them up, and every planar face on a split edge is read again in
//! the next round: a split can make a neighbour's outline cross that was
//! simple before. A crossing between two straight edges cannot come from
//! sampling and is left alone.
//!
//! Planar faces that share an edge form one group, and each group refines
//! on its own: its splits touch only its edges. When a group's crossings
//! cannot be cleared within the bounds below, its chains go back to exactly
//! what was sampled, so its faces keep their old outlines, neighbours never
//! disagree, and other groups keep their refinement.

use remus_math::compact_sort::CompactSort;
use remus_math::det_hash::DetHashMap;
use remus_math::predicates::orient2d;
use remus_math::vec::{Point2, Point3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::boundary_plan::{SampleChains, SampleParams};
use super::edge_sampling::AnchoredNurbsCurve;
use super::planar::project_by_normal;

/// Bisection rounds; each round halves every chord still crossing.
const MAX_ROUNDS: usize = 32;

/// Segment-pair tests one face may spend in one round's crossing search.
/// Only pairs that involve a curved chord and overlap in both axes count.
const MAX_PAIR_TESTS: usize = 4_000_000;

/// Active-list entries one face's crossing search may read in one round,
/// pruning scans and survivor scans together.
const MAX_VISITS: usize = 16 * MAX_PAIR_TESTS;

/// One chord of a planar face's sampled outline.
struct Chord {
    a: Point2,
    b: Point2,
    /// The edge whose chain holds the chord, and the chord's first sample.
    edge: usize,
    at: usize,
    /// Whether the chord can be bisected on its curve.
    curved: bool,
}

/// Bisect crossing chords of planar faces' sampled outlines, in place.
///
/// Returns the number of samples inserted.
///
/// # Errors
///
/// Returns an error if a topology lookup fails.
pub(super) fn refine_planar_crossings(
    topo: &Topology,
    all_faces: &[FaceId],
    edge_points: &mut SampleChains,
    edge_params: &mut SampleParams,
) -> Result<usize, crate::OperationsError> {
    // The planar faces, in face-set order, and which of them use each edge.
    let mut planar: Vec<FaceId> = Vec::new();
    let mut users: DetHashMap<usize, Vec<usize>> = DetHashMap::default();
    for &face_id in all_faces {
        let face = topo.face(face_id)?;
        if !matches!(face.surface(), FaceSurface::Plane { .. }) {
            continue;
        }
        let ordinal = planar.len();
        planar.push(face_id);
        for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
        {
            for oriented in topo.wire(wire_id)?.edges() {
                let faces = users.entry(oriented.edge().index()).or_default();
                if faces.last() != Some(&ordinal) {
                    faces.push(ordinal);
                }
            }
        }
    }
    // Groups: planar faces joined through shared edges (union-find rooted
    // at the smallest ordinal, so groups come out in face-set order).
    let mut parent: Vec<usize> = (0..planar.len()).collect();
    let find = |parent: &mut Vec<usize>, mut x: usize| {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    };
    let mut edges: Vec<usize> = users.keys().copied().collect();
    edges.compact_sort_unstable();
    for edge in &edges {
        let faces = &users[edge];
        for pair in faces.windows(2) {
            let (ra, rb) = (find(&mut parent, pair[0]), find(&mut parent, pair[1]));
            if ra != rb {
                parent[ra.max(rb)] = ra.min(rb);
            }
        }
    }
    let mut groups: DetHashMap<usize, Vec<usize>> = DetHashMap::default();
    let mut roots: Vec<usize> = Vec::new();
    for ordinal in 0..planar.len() {
        let root = find(&mut parent, ordinal);
        let members = groups.entry(root).or_default();
        if members.is_empty() {
            roots.push(root);
        }
        members.push(ordinal);
    }
    let mut inserted = 0;
    for root in roots {
        let group = &groups[&root];
        inserted += refine_group(topo, &planar, group, &users, edge_points, edge_params)?;
    }
    Ok(inserted)
}

/// [`refine_planar_crossings`] on one group of planar faces joined through
/// shared edges: every edge it splits is used by no planar face outside the
/// group. Restores the group's chains as sampled when its crossings cannot
/// be cleared, and returns the samples it kept.
fn refine_group(
    topo: &Topology,
    planar: &[FaceId],
    group: &[usize],
    users: &DetHashMap<usize, Vec<usize>>,
    edge_points: &mut SampleChains,
    edge_params: &mut SampleParams,
) -> Result<usize, crate::OperationsError> {
    // Each edge's chain as sampled, kept from its first split on, to put back
    // if the crossings cannot be cleared.
    let mut sampled: DetHashMap<usize, (Vec<Point3>, Vec<Option<f64>>)> = DetHashMap::default();
    let mut spent = vec![0_usize; planar.len()];
    let mut pending: Vec<usize> = group.to_vec();
    let mut inserted = 0;
    let mut cleared = false;
    'rounds: for _ in 0..MAX_ROUNDS {
        let mut requests: DetHashMap<usize, Vec<usize>> = DetHashMap::default();
        let mut touched = vec![false; planar.len()];
        for &ordinal in &pending {
            let chords = face_chords(topo, planar[ordinal], edge_points, edge_params)?;
            // Only a curved chord can be bisected: a face of straight edges
            // has nothing to refine.
            if !chords.iter().any(|chord| chord.curved) {
                continue;
            }
            let Some(split) = crossing_splits(&chords) else {
                break 'rounds;
            };
            if split.is_empty() {
                continue;
            }
            // Each round may add at most one sample per chord; a face whose
            // crossings need more than its own sample count again is not a
            // sampling artefact.
            spent[ordinal] += split.len();
            if spent[ordinal] > chords.len().max(64) {
                break 'rounds;
            }
            for &index in &split {
                let chord = &chords[index];
                requests.entry(chord.edge).or_default().push(chord.at);
            }
            touched[ordinal] = true;
        }
        if requests.is_empty() {
            cleared = true;
            break;
        }
        let mut edges: Vec<usize> = requests.keys().copied().collect();
        edges.compact_sort_unstable();
        let mut round_inserted = 0;
        for edge in edges {
            let mut at = requests.remove(&edge).unwrap_or_default();
            at.compact_sort_unstable();
            at.dedup();
            if !sampled.contains_key(&edge)
                && let (Some(points), Some(params)) =
                    (edge_points.get(&edge), edge_params.get(&edge))
            {
                sampled.insert(edge, (points.clone(), params.clone()));
            }
            let count = bisect_chords(topo, edge, &at, edge_points, edge_params)?;
            if count > 0 {
                for &ordinal in users.get(&edge).into_iter().flatten() {
                    touched[ordinal] = true;
                }
            }
            round_inserted += count;
        }
        // Nothing could be split: the crossings stay as they are.
        if round_inserted == 0 {
            break;
        }
        inserted += round_inserted;
        pending = group
            .iter()
            .copied()
            .filter(|&ordinal| touched[ordinal])
            .collect();
    }
    if !cleared {
        for (edge, (points, params)) in sampled {
            edge_points.insert(edge, points);
            edge_params.insert(edge, params);
        }
        return Ok(0);
    }
    Ok(inserted)
}

/// Every chord of a planar face's outer and inner wires, projected onto the
/// face's plane.
fn face_chords(
    topo: &Topology,
    face_id: FaceId,
    edge_points: &SampleChains,
    edge_params: &SampleParams,
) -> Result<Vec<Chord>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Plane { normal, .. } = face.surface() else {
        return Ok(Vec::new());
    };
    let normal = *normal;
    let mut chords = Vec::new();
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        for oriented in topo.wire(wire_id)?.edges() {
            let index = oriented.edge().index();
            let Some(points) = edge_points.get(&index) else {
                continue;
            };
            let edge = topo.edge(oriented.edge())?;
            let params = edge_params.get(&index);
            let refinable = |at: usize| {
                !matches!(edge.curve(), EdgeCurve::Line)
                    && params.is_some_and(|params| {
                        matches!(
                            (params.get(at), params.get(at + 1)),
                            (Some(Some(_)), Some(Some(_)))
                        )
                    })
            };
            for (at, pair) in points.windows(2).enumerate() {
                chords.push(Chord {
                    a: project_by_normal(pair[0], normal),
                    b: project_by_normal(pair[1], normal),
                    edge: index,
                    at,
                    curved: refinable(at),
                });
            }
        }
    }
    Ok(chords)
}

/// The chords to bisect so that crossing pairs separate: the longer curved
/// chord of each pair that properly crosses. A pair of straight chords is
/// never read (sampling cannot have made it cross, and many straight sides
/// overlapping in `x` must not spend the budget a crescent needs). `None`
/// when the search exceeds its work bounds.
fn crossing_splits(chords: &[Chord]) -> Option<Vec<usize>> {
    let lo = |c: &Chord| c.a.x().min(c.b.x());
    let hi = |c: &Chord| c.a.x().max(c.b.x());
    let mut order: Vec<usize> = (0..chords.len()).collect();
    order.compact_sort_by(|&i, &j| lo(&chords[i]).total_cmp(&lo(&chords[j])).then(i.cmp(&j)));
    // Chords whose x-extent still reaches the sweep line, curved and
    // straight apart. The straight list is only read, and so only pruned,
    // for a curved chord.
    let mut curved: Vec<usize> = Vec::new();
    let mut straight: Vec<usize> = Vec::new();
    let mut split: Vec<usize> = Vec::new();
    let (mut tests, mut visits) = (0_usize, 0_usize);
    for &i in &order {
        let chord = &chords[i];
        let x = lo(chord);
        // Both scans count: the one that prunes a list and the one that
        // reads its survivors.
        visits += curved.len();
        curved.retain(|&j| hi(&chords[j]) >= x);
        if chord.curved {
            visits += straight.len();
            straight.retain(|&j| hi(&chords[j]) >= x);
        }
        let (y_lo, y_hi) = (chord.a.y().min(chord.b.y()), chord.a.y().max(chord.b.y()));
        let straight_others: &[usize] = if chord.curved { &straight } else { &[] };
        visits += curved.len() + straight_others.len();
        if visits > MAX_VISITS {
            return None;
        }
        for &j in curved.iter().chain(straight_others) {
            let other = &chords[j];
            if other.a.y().max(other.b.y()) < y_lo || other.a.y().min(other.b.y()) > y_hi {
                continue;
            }
            tests += 1;
            if tests > MAX_PAIR_TESTS {
                return None;
            }
            if !properly_cross(chord, other) {
                continue;
            }
            let length = |c: &Chord| (c.b - c.a).length();
            let pick = match (chord.curved, other.curved) {
                (true, true) => Some(if length(chord) >= length(other) { i } else { j }),
                (true, false) => Some(i),
                (false, true) => Some(j),
                (false, false) => None,
            };
            split.extend(pick);
        }
        if chord.curved {
            curved.push(i);
        } else {
            straight.push(i);
        }
    }
    split.compact_sort_unstable();
    split.dedup();
    Some(split)
}

/// Whether two chords cross at a point inside both. Chords that share an end
/// sample (neighbours in a chain, or the chains' common vertex) never do.
fn properly_cross(first: &Chord, second: &Chord) -> bool {
    let ends = [first.a, first.b];
    if ends.contains(&second.a) || ends.contains(&second.b) {
        return false;
    }
    let side = |p: Point2, q: Point2, r: Point2| orient2d(p, q, r).signum();
    let (c, d) = (
        side(first.a, first.b, second.a),
        side(first.a, first.b, second.b),
    );
    let (e, f) = (
        side(second.a, second.b, first.a),
        side(second.a, second.b, first.b),
    );
    c * d < 0.0 && e * f < 0.0
}

/// Bisect the chords of `edge` that start at the samples `at` (ascending),
/// each at the middle of its parameter span. Returns the samples inserted.
fn bisect_chords(
    topo: &Topology,
    edge: usize,
    at: &[usize],
    edge_points: &mut SampleChains,
    edge_params: &mut SampleParams,
) -> Result<usize, crate::OperationsError> {
    let Some(edge_id) = topo.edge_id_from_index(edge) else {
        return Ok(0);
    };
    let curve = topo.edge(edge_id)?.curve();
    let anchored = match curve {
        EdgeCurve::NurbsCurve(nurbs) => AnchoredNurbsCurve::new(nurbs),
        EdgeCurve::Line
        | EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => None,
    };
    // The samplers' own expressions (see `sample_circle_uniform`), so an
    // inserted sample moves with a rigid translation exactly as its
    // neighbours do.
    let evaluate = |t: f64| -> Option<Point3> {
        match curve {
            EdgeCurve::Line => None,
            EdgeCurve::Circle(circle) => {
                let r = circle.radius();
                Some(
                    circle.center()
                        + (circle.u_axis() * (r * t.cos()) + circle.v_axis() * (r * t.sin())),
                )
            }
            EdgeCurve::Ellipse(ellipse) => Some(
                ellipse.center()
                    + (ellipse.u_axis() * (ellipse.semi_major() * t.cos())
                        + ellipse.v_axis() * (ellipse.semi_minor() * t.sin())),
            ),
            EdgeCurve::Hyperbola(hyperbola) => Some(hyperbola.evaluate(t)),
            EdgeCurve::Parabola(parabola) => Some(parabola.evaluate(t)),
            EdgeCurve::NurbsCurve(nurbs) => Some(
                anchored
                    .as_ref()
                    .map_or_else(|| nurbs.evaluate(t), |anchored| anchored.evaluate(t)),
            ),
        }
    };
    // A closed NURBS chain wraps its parameter at the knot seam; a chord
    // across the wrap has no parameter between its ends.
    let wrap = match curve {
        EdgeCurve::NurbsCurve(nurbs) => {
            let (u0, u1) = nurbs.domain();
            0.5 * (u1 - u0)
        }
        EdgeCurve::Line
        | EdgeCurve::Circle(_)
        | EdgeCurve::Ellipse(_)
        | EdgeCurve::Hyperbola(_)
        | EdgeCurve::Parabola(_) => f64::INFINITY,
    };
    let (Some(points), Some(params)) = (edge_points.get_mut(&edge), edge_params.get_mut(&edge))
    else {
        return Ok(0);
    };
    if points.len() != params.len() {
        return Ok(0);
    }
    let mut inserted = 0;
    for &i in at.iter().rev() {
        let (Some(&Some(t0)), Some(&Some(t1))) = (params.get(i), params.get(i + 1)) else {
            continue;
        };
        if (t1 - t0).abs() >= wrap {
            continue;
        }
        let t = f64::midpoint(t0, t1);
        if !(t0.min(t1) < t && t < t0.max(t1)) {
            continue;
        }
        let Some(point) = evaluate(t) else {
            continue;
        };
        if !(point.x().is_finite() && point.y().is_finite() && point.z().is_finite())
            || point == points[i]
            || point == points[i + 1]
        {
            continue;
        }
        points.insert(i + 1, point);
        params.insert(i + 1, Some(t));
        inserted += 1;
    }
    Ok(inserted)
}

#[cfg(test)]
mod tests;
