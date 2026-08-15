//! Comprehensive solid validation.
//!
//! Performs structural and geometric validation on solids.

use brepkit_math::tolerance::Tolerance;
use brepkit_topology::Topology;
use brepkit_topology::TopologyError;
use brepkit_topology::edge::EdgeCurve;
use brepkit_topology::explorer;
use brepkit_topology::face::{Face, FaceSurface};
use brepkit_topology::solid::SolidId;

/// A validation issue found in a solid.
#[derive(Debug, Clone)]
pub struct ValidationIssue {
    /// Severity of the issue.
    pub severity: Severity,
    /// Human-readable description.
    pub description: String,
}

/// Severity of a validation issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    /// The solid is invalid and may cause downstream failures.
    Error,
    /// The solid has a potential problem but may still be usable.
    Warning,
}

/// Result of validating a solid.
#[derive(Debug, Clone)]
pub struct ValidationReport {
    /// All issues found.
    pub issues: Vec<ValidationIssue>,
}

impl ValidationReport {
    /// Whether the solid passed all validation checks (no errors).
    #[must_use]
    pub fn is_valid(&self) -> bool {
        !self.issues.iter().any(|i| i.severity == Severity::Error)
    }

    /// Count of error-severity issues.
    #[must_use]
    pub fn error_count(&self) -> usize {
        self.issues
            .iter()
            .filter(|i| i.severity == Severity::Error)
            .count()
    }

    /// Count of warning-severity issues.
    #[must_use]
    pub fn warning_count(&self) -> usize {
        self.issues
            .iter()
            .filter(|i| i.severity == Severity::Warning)
            .count()
    }
}

/// How much quadrature the shell-orientation check may spend.
///
/// The check establishes a SIGN, and a sign needs far less accuracy than a
/// mass property: the verdict only has to clear a floor of `diag^3 * 1e-9`, so
/// a low order is enough wherever the cost matters. Integrating a trimmed
/// quadric face is expensive out of proportion to everything around it — on a
/// box with one cylindrical bore, one cylinder face costs 898 us at order 5
/// against under 3.6 us for each of the six planes — so the order is the knob
/// that decides whether a caller can afford the check at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OrientationCheck {
    /// Do not run it. For callers that do not act on the answer.
    Skip,
    /// Run it at this Gauss order.
    Order(usize),
}

/// Options for controlling validation tolerance.
///
/// Operations like fillet and shell produce NURBS faces where geometric
/// checks (normal length, face area) may trigger false positives at
/// default tolerance. Increasing `tolerance_scale` relaxes these
/// thresholds.
#[derive(Debug, Clone)]
pub struct ValidationOptions {
    /// Multiplier applied to geometric tolerances for the face normal
    /// length check and the degenerate face area check. Default is `1.0`.
    /// A value of `10.0` means tolerances are 10x more permissive.
    pub tolerance_scale: f64,
    /// What the shell-orientation check is allowed to cost. Defaults to the
    /// same Gauss order `mass_properties` uses, so the default validation is
    /// the strict one; a caller that does not act on the report can turn it
    /// down or off.
    pub orientation: OrientationCheck,
    /// Check shell orientation consistency: adjacent faces must traverse
    /// each shared edge in opposite effective senses (is_forward XOR
    /// is_reversed). Defaults to `true`: construction ops (revolve,
    /// extrude, sweep, loft, pipe), GFA boolean outputs, and blend bands
    /// all emit consistent shells (the orientation-emission campaign).
    pub check_orientation: bool,
}

impl Default for ValidationOptions {
    fn default() -> Self {
        Self {
            tolerance_scale: 1.0,
            orientation: OrientationCheck::Order(
                brepkit_check::properties::PropertiesOptions::default().gauss_order,
            ),
            check_orientation: true,
        }
    }
}

/// Compute the raw Euler characteristic (V - E + F) for a solid.
///
/// Returns the unmodified V - E + F value. For a genus-0 closed manifold
/// solid without inner wire loops this equals 2. Solids with through-holes
/// (genus > 0) or inner loops will have different values — use
/// [`validate_solid`] for a full topological check that accounts for these.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn euler_characteristic(
    topo: &Topology,
    solid: SolidId,
) -> Result<i64, crate::OperationsError> {
    let (f, e, v) = explorer::solid_entity_counts(topo, solid)?;
    #[allow(clippy::cast_possible_wrap)]
    let euler = (v as i64) - (e as i64) + (f as i64);
    Ok(euler)
}

/// Decompose a solid's faces into edge-connected components.
///
/// Two faces belong to the same component when they share an edge. The
/// first component contains the solid's first face; ordering beyond that is
/// discovery order.
fn face_connectivity_components<V: std::ops::Deref<Target = [brepkit_topology::face::FaceId]>>(
    faces: &[brepkit_topology::face::FaceId],
    edge_map: &std::collections::BTreeMap<usize, V>,
) -> Vec<Vec<brepkit_topology::face::FaceId>> {
    use std::collections::{HashMap, HashSet, VecDeque};

    // Face index -> neighbor face indices via shared edges. A shared edge only
    // needs a spanning star to connect all of its incident faces; materializing
    // the complete clique would make this quadratic for malformed,
    // high-fanout edges that validation is expected to reject.
    let mut adjacency: HashMap<usize, HashSet<usize>> = HashMap::new();
    for adj_faces in edge_map.values() {
        let adj_faces: &[brepkit_topology::face::FaceId] = adj_faces;
        let Some((first, rest)) = adj_faces.split_first() else {
            continue;
        };
        for face in rest {
            if first.index() != face.index() {
                adjacency
                    .entry(first.index())
                    .or_default()
                    .insert(face.index());
                adjacency
                    .entry(face.index())
                    .or_default()
                    .insert(first.index());
            }
        }
    }

    let by_index: HashMap<usize, brepkit_topology::face::FaceId> =
        faces.iter().map(|f| (f.index(), *f)).collect();
    let mut visited: HashSet<usize> = HashSet::new();
    let mut components = Vec::new();
    for &start in faces {
        if !visited.insert(start.index()) {
            continue;
        }
        let mut component = vec![start];
        let mut queue = VecDeque::from([start.index()]);
        while let Some(current) = queue.pop_front() {
            if let Some(neighbors) = adjacency.get(&current) {
                for &n in neighbors {
                    if by_index.contains_key(&n) && visited.insert(n) {
                        component.push(by_index[&n]);
                        queue.push_back(n);
                    }
                }
            }
        }
        components.push(component);
    }
    components
}

/// Whether any two OUTER-shell face components materially overlap in space.
///
/// The hazard is NESTING — a tool fragment left closed but floating inside the
/// stock (the equal-radius cross-drill's sealed bore lobes), which must not be
/// blessed as a disjoint union or the consuming editor's repair path stays
/// silent. Side-by-side pieces are exactly what multi-component acceptance is
/// for.
///
/// AABB *intersection* is the wrong predicate for nesting, because an AABB is
/// only tight on axis-aligned geometry. A cylinder standing off the box's
/// CORNER diagonal is a clear distance from it, yet the two boxes interpenetrate
/// over the whole corner region — so an intersection-thickness test called a
/// genuinely disjoint fuse "debris" and reported a bogus disconnected shell,
/// dependent on where the operand sat rather than on whether it touched.
/// Containment is tight in the direction that matters: nesting implies it, and a
/// diagonal offset does not manufacture it.
///
/// So the test mirrors the boolean gate's `components_are_disjoint_pieces`: AABB
/// containment (from sampled face polygons — vertex positions alone
/// under-represent curved faces, whose only stored vertices may sit on a seam)
/// is the cheap PRE-FILTER, and a suspect pair earns its verdict from a real
/// ray-parity test against the enclosing candidate's own surface. That second
/// stage matters because containment is necessary but not sufficient: a ring's
/// box contains the box of a separate piece sitting in its hole.
///
/// Two exemptions mirror how legitimate containment arises: components carrying
/// any inner-shell (cavity) face (containment is a cavity's defining property),
/// and pairs where either component has genus above zero (a full hollow revolve
/// stores its toroidal cavity wall as a second outer-shell component — the shape
/// the historical higher-genus connectivity skip existed for). Whenever the
/// answer cannot be established the result is `true`, failing toward the
/// historical "disconnected" report rather than blessing the unknown.
fn outer_components_materially_overlap(
    topo: &Topology,
    solid: SolidId,
    components: &[Vec<brepkit_topology::face::FaceId>],
    component_genus_2: &[i64],
) -> Result<bool, crate::OperationsError> {
    use std::collections::HashSet;

    // Keep strict validation safe for untrusted models. A spatial index alone
    // cannot bound the worst case because arbitrarily many component AABBs can
    // be mutually nested. Exceeding this budget fails closed, preserving the
    // historical disconnected-shell verdict instead of spending quadratic
    // time on a crafted solid.
    const MAX_COMPONENT_PAIR_CHECKS: usize = 4_096;

    let solid_data = topo.solid(solid)?;
    let outer_faces: HashSet<usize> = topo
        .shell(solid_data.outer_shell())?
        .faces()
        .iter()
        .map(|f| f.index())
        .collect();

    // (index into `components`, AABB, genus*2) for each outer-shell component.
    let mut boxes: Vec<(usize, brepkit_math::aabb::Aabb3, i64)> = Vec::new();
    for (ci, comp) in components.iter().enumerate() {
        if !comp.iter().all(|f| outer_faces.contains(&f.index())) {
            continue; // cavity component — containment is expected
        }
        let genus_2 = component_genus_2.get(ci).copied().unwrap_or(0);
        let mut pts: Vec<brepkit_math::vec::Point3> = Vec::new();
        for &fid in comp {
            if let Ok(poly) = crate::boolean::face_polygon(topo, fid) {
                pts.extend(poly);
            }
        }
        if pts.len() < 3 {
            // Cannot establish an extent — fail toward the historical
            // "disconnected" report rather than blessing the unknown.
            return Ok(true);
        }
        boxes.push((ci, brepkit_math::aabb::Aabb3::from_points(pts), genus_2));
    }

    let eps = crate::boolean::COMPONENT_OVERLAP_MARGIN_MM;
    let contains = |o: &brepkit_math::aabb::Aabb3, i: &brepkit_math::aabb::Aabb3| {
        o.min.x() - eps <= i.min.x()
            && o.min.y() - eps <= i.min.y()
            && o.min.z() - eps <= i.min.z()
            && o.max.x() + eps >= i.max.x()
            && o.max.y() + eps >= i.max.y()
            && o.max.z() + eps >= i.max.z()
    };
    let materially_intersects = |a: &brepkit_math::aabb::Aabb3, b: &brepkit_math::aabb::Aabb3| {
        a.max.x().min(b.max.x()) - a.min.x().max(b.min.x()) > eps
            && a.max.y().min(b.max.y()) - a.min.y().max(b.min.y()) > eps
            && a.max.z().min(b.max.z()) - a.min.z().max(b.min.z()) > eps
    };

    let mut pair_checks = 0usize;
    for (i, (ci_a, a, ga)) in boxes.iter().enumerate() {
        for (ci_b, b, gb) in boxes.iter().skip(i + 1) {
            pair_checks += 1;
            if pair_checks > MAX_COMPONENT_PAIR_CHECKS {
                return Ok(true);
            }
            if *ga > 0 || *gb > 0 {
                continue; // higher-genus containment is a cavity wall, not debris
            }
            // Containment is the nesting case. For partial AABB overlap, test
            // every boundary vertex in both directions: accepting the pair
            // solely because neither box contains the other would let ordinary
            // interpenetrating components bypass validation.
            let (outer_ci, outer_box, inner_ci) = if contains(a, b) {
                (*ci_a, a, *ci_b)
            } else if contains(b, a) {
                (*ci_b, b, *ci_a)
            } else {
                if !materially_intersects(a, b) {
                    continue;
                }
                let mut triangles = [Vec::new(), Vec::new()];
                for (slot, (ci, bbox)) in triangles.iter_mut().zip([(*ci_a, a), (*ci_b, b)]) {
                    let diag = (bbox.max - bbox.min).length();
                    let deflection = (diag / 200.0).max(1e-4);
                    for &fid in &components[ci] {
                        let mesh = crate::tessellate::tessellate_with_uvs(topo, fid, deflection)?;
                        for tri in mesh.mesh.indices.chunks_exact(3) {
                            slot.push([
                                mesh.mesh.positions[tri[0] as usize],
                                mesh.mesh.positions[tri[1] as usize],
                                mesh.mesh.positions[tri[2] as usize],
                            ]);
                        }
                    }
                }
                if triangles[0].iter().any(|&ta| {
                    triangles[1]
                        .iter()
                        .any(|&tb| crate::mesh_boolean::triangles_intersect(ta, tb, eps))
                }) {
                    return Ok(true);
                }
                for (probe_ci, surface_ci, surface_box) in [(*ci_a, *ci_b, b), (*ci_b, *ci_a, a)] {
                    let diag = (surface_box.max - surface_box.min).length();
                    let deflection = (diag / 200.0).max(1e-4);
                    let mut probes = Vec::new();
                    for &fid in &components[probe_ci] {
                        let face = topo.face(fid)?;
                        for wid in std::iter::once(face.outer_wire())
                            .chain(face.inner_wires().iter().copied())
                        {
                            for oe in topo.wire(wid)?.edges() {
                                probes.push(topo.vertex(topo.edge(oe.edge())?.start())?.point());
                            }
                        }
                    }
                    if probes.is_empty() {
                        return Ok(true);
                    }
                    match crate::boolean::component_encloses_any_point(
                        topo,
                        &components[surface_ci],
                        &probes,
                        deflection,
                    ) {
                        Some(true) => return Ok(true),
                        Some(false) => {}
                        None => return Ok(true),
                    }
                }
                continue;
            };
            // Confirm with ray parity against the enclosing candidate's own
            // surface, so a piece merely sitting in a ring's hole survives.
            let diag = (outer_box.max - outer_box.min).length();
            let deflection = (diag / 200.0).max(1e-4);
            let Some(probe) = crate::boolean::any_vertex_of(topo, &components[inner_ci]) else {
                return Ok(true);
            };
            match crate::boolean::component_encloses_point(
                topo,
                &components[outer_ci],
                probe,
                deflection,
            ) {
                Some(true) => return Ok(true),
                Some(false) => {}
                None => return Ok(true),
            }
        }
    }
    Ok(false)
}

/// Count vertices, edges, faces, and inner wire loops of one face component.
///
/// Entities are deduplicated by id within the component, mirroring
/// [`explorer::solid_entity_counts`] scoped to the component's faces.
fn component_counts(
    topo: &Topology,
    component: &[brepkit_topology::face::FaceId],
) -> Result<(i64, i64, i64, i64), crate::OperationsError> {
    use std::collections::HashSet;

    let mut edge_ids: HashSet<usize> = HashSet::new();
    let mut vertex_ids: HashSet<usize> = HashSet::new();
    let mut inner_loops: i64 = 0;
    for &fid in component {
        let face = topo.face(fid)?;
        #[allow(clippy::cast_possible_wrap)]
        {
            inner_loops += face.inner_wires().len() as i64;
        }
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();
        for wid in wire_ids {
            for oe in topo.wire(wid)?.edges() {
                let edge = topo.edge(oe.edge())?;
                edge_ids.insert(oe.edge().index());
                vertex_ids.insert(edge.start().index());
                vertex_ids.insert(edge.end().index());
            }
        }
    }
    #[allow(clippy::cast_possible_wrap)]
    Ok((
        vertex_ids.len() as i64,
        edge_ids.len() as i64,
        component.len() as i64,
        inner_loops,
    ))
}

/// Report a shell that is turned the wrong way round.
///
/// A shell can be closed, 2-manifold and consistently wound and still face
/// inward; every check above passes on such a body, and so does
/// `measure::solid_volume`, which returns the magnitude of its integral and so
/// reads an inside-out solid at its correct positive volume. brepkit#59 built
/// exactly that from a segmented revolve and nothing in the kernel could say
/// so. The winding sign is what an STL facet normal is derived from, so the
/// body exported inside out.
///
/// Two statements, one per shell role:
///
/// * the OUTER shell must enclose a positive signed volume;
/// * every INNER shell is a cavity and must enclose a negative one — a cavity
///   wound outward adds its void to the body instead of removing it.
///
/// Both are silent when the answer cannot be established (a face that will not
/// integrate, a body with no measurable extent) rather than guessing.
fn shell_orientation_issues(
    topo: &Topology,
    solid: SolidId,
    check: OrientationCheck,
) -> Result<Vec<ValidationIssue>, crate::OperationsError> {
    let OrientationCheck::Order(order) = check else {
        return Ok(Vec::new());
    };
    let Some(floor) = crate::measure::negligible_volume(topo, solid) else {
        return Ok(Vec::new());
    };
    let solid_data = topo.solid(solid)?;
    let mut issues = Vec::new();

    if let Some(signed) = crate::measure::shell_signed_volume(topo, solid_data.outer_shell(), order)
        && signed < -floor
    {
        issues.push(ValidationIssue {
            severity: Severity::Error,
            description: format!(
                "the outer shell is inside out: it encloses a signed volume of {signed}, \
                 so every face points into the body"
            ),
        });
    }

    for &inner in solid_data.inner_shells() {
        if let Some(signed) = crate::measure::shell_signed_volume(topo, inner, order)
            && signed > floor
        {
            issues.push(ValidationIssue {
                severity: Severity::Error,
                description: format!(
                    "cavity shell {} is wound outward: it encloses a signed volume of \
                     {signed}, so the void adds to the body instead of removing from it",
                    inner.index()
                ),
            });
        }
    }

    Ok(issues)
}

/// Validate a solid, returning a report of all issues found.
///
/// Checks performed:
/// Returns `true` if every edge in the face is a straight line.
fn face_all_edges_straight(
    topo: &Topology,
    face: &brepkit_topology::face::Face,
) -> Result<bool, TopologyError> {
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let wire = topo.wire(wire_id)?;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            if !matches!(edge.curve(), brepkit_topology::edge::EdgeCurve::Line) {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn same_oriented_circle(
    a: &brepkit_math::curves::Circle3D,
    b: &brepkit_math::curves::Circle3D,
    tol: Tolerance,
) -> bool {
    (a.center() - b.center()).length() <= tol.linear
        && tol.approx_eq(a.radius(), b.radius())
        && a.normal().dot(b.normal()) > 0.0
        && (1.0 - a.normal().dot(b.normal())) <= tol.angular.max(1e-12)
}

const MAX_AMBIGUOUS_CIRCLE_ARC_COMPARISONS: usize = 4_096;

fn ambiguous_circle_arc_warnings(
    topo: &Topology,
    face_id: brepkit_topology::face::FaceId,
    wire_id: brepkit_topology::wire::WireId,
    tol: Tolerance,
) -> Result<Vec<ValidationIssue>, crate::OperationsError> {
    // One diagnostic identifies the wire-level hazard. Bounding comparisons also
    // prevents a malformed wire with a huge endpoint bucket from making strict
    // validation quadratic when none of its circles match.
    let wire = topo.wire(wire_id)?;
    let mut by_endpoints = std::collections::HashMap::new();
    let mut comparisons = 0;
    for oriented in wire.edges() {
        let edge = topo.edge(oriented.edge())?;
        let EdgeCurve::Circle(circle) = edge.curve() else {
            continue;
        };
        if edge.start() == edge.end() {
            continue;
        }
        let candidates = by_endpoints
            .entry((edge.start(), edge.end()))
            .or_insert_with(Vec::new);
        for &candidate_id in candidates.iter() {
            if comparisons == MAX_AMBIGUOUS_CIRCLE_ARC_COMPARISONS {
                return Ok(vec![ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "wire {} on face {} has too many same-endpoint circle arcs to check \
                         individually",
                        wire_id.index(),
                        face_id.index()
                    ),
                }]);
            }
            comparisons += 1;
            if oriented.edge() == candidate_id {
                continue;
            }
            let candidate = topo.edge(candidate_id)?;
            let EdgeCurve::Circle(candidate_circle) = candidate.curve() else {
                continue;
            };
            if same_oriented_circle(candidate_circle, circle, tol) {
                return Ok(vec![ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "wire {} on face {} has ambiguous complementary circle arcs {} and {} \
                         with the same stored vertex order",
                        wire_id.index(),
                        face_id.index(),
                        candidate_id.index(),
                        oriented.edge().index()
                    ),
                }]);
            }
        }
        candidates.push(oriented.edge());
    }
    Ok(Vec::new())
}

fn periodic_outer_rims_are_full_turns(
    topo: &Topology,
    face: &Face,
) -> Result<bool, crate::OperationsError> {
    if !matches!(
        face.surface(),
        FaceSurface::Cylinder(_)
            | FaceSurface::Cone(_)
            | FaceSurface::Sphere(_)
            | FaceSurface::Torus(_)
    ) {
        return Ok(true);
    }
    let wire = topo.wire(face.outer_wire())?;
    let mut uses = std::collections::HashMap::new();
    for oe in wire.edges() {
        *uses.entry(oe.edge()).or_insert(0_usize) += 1;
    }
    let doubled: std::collections::HashSet<_> = uses
        .iter()
        .filter_map(|(&edge, &count)| (count == 2).then_some(edge))
        .collect();
    if doubled.is_empty() {
        return Ok(true);
    }
    if uses.values().any(|&count| count > 2) {
        return Ok(false);
    }

    let mut curved = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for oe in wire.edges() {
        if doubled.contains(&oe.edge()) || !seen.insert(oe.edge()) {
            continue;
        }
        let edge = topo.edge(oe.edge())?;
        if !matches!(edge.curve(), EdgeCurve::Line) {
            curved.push((oe.edge().index(), edge.start(), edge.end()));
        }
    }
    if curved.is_empty() {
        return Ok(true);
    }

    let accepts = |project: &dyn Fn(brepkit_math::vec::Point3) -> f64| {
        Ok::<bool, crate::OperationsError>(
            crate::tessellate::rim_chain::collect_full_turn_rim_cycles_any(topo, &curved, project)?
                .is_some(),
        )
    };
    let project_u = |point| face.surface().project_point(point).map_or(0.0, |uv| uv.0);
    if accepts(&project_u)? {
        return Ok(true);
    }
    if matches!(face.surface(), FaceSurface::Torus(_)) {
        let project_v = |point| face.surface().project_point(point).map_or(0.0, |uv| uv.1);
        return accepts(&project_v);
    }
    Ok(false)
}

/// 1. **Euler-Poincaré**: V - E + F = 2(1 - g) for genus-g closed solid
/// 2. **Manifold edges**: each edge shared by exactly 2 faces
/// 3. **Boundary edges**: no edge shared by only 1 face (open shell)
/// 4. **Degenerate faces**: each face has at least 3 vertices
/// 5. **Face normal consistency**: normals should be non-zero
/// 6. **Wire closure**: every wire forms a closed loop
/// 7. **Degenerate face area**: near-zero polygon area warning for planar faces
/// 8. **Zero-length edges**: edges with coincident start/end vertices
/// 9. **Empty wires**: wires with no edges
/// 10. **Shell connectivity**: multiple edge-connected components are valid
///     only when each is independently closed and Euler-consistent (tangent
///     or disjoint fuse results, cavity shells)
/// 11. **Redundant faces**: same face ID appearing twice in shell
/// 12. **Edge vertex consistency**: edge vertices belong to the solid
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn validate_solid(
    topo: &Topology,
    solid: SolidId,
) -> Result<ValidationReport, crate::OperationsError> {
    validate_solid_with_options(topo, solid, &ValidationOptions::default())
}

/// Validate a solid with configurable tolerance options.
///
/// Same checks as [`validate_solid`] but with tolerance scaling.
/// Use `ValidationOptions { tolerance_scale: 10.0, .. }` to relax
/// geometric checks for NURBS faces produced by fillet/shell operations.
///
/// # Errors
///
/// Returns an error if topology lookups fail.
#[allow(clippy::too_many_lines)]
pub fn validate_solid_with_options(
    topo: &Topology,
    solid: SolidId,
    options: &ValidationOptions,
) -> Result<ValidationReport, crate::OperationsError> {
    let mut issues = Vec::new();
    let tol = Tolerance::new();
    // Clamp to [0.1, 1000]: below 0.1 risks false positives on exact
    // geometry, above 1000 makes the check meaningless.
    let scale = options.tolerance_scale.clamp(0.1, 1000.0);

    let (f, e, v) = explorer::solid_entity_counts(topo, solid)?;

    // Euler-Poincaré formula for a cell complex with inner loops:
    //   V - E + F = 2(1 - g) + L
    // where g is the genus and L is the total number of inner wire loops
    // across all faces. For a genus-0 solid with no holes: V-E+F = 2.
    // With L inner wires: V-E+F = 2 + L.
    let mut total_inner_loops: i64 = 0;
    let faces = explorer::solid_faces(topo, solid)?;
    for fid in &faces {
        let face = topo.face(*fid)?;
        #[allow(clippy::cast_possible_wrap)]
        {
            total_inner_loops += face.inner_wires().len() as i64;
        }
    }

    let edge_map = explorer::edge_to_face_map(topo, solid)?;

    // Edge-connected face components. A fuse of solids that touch only on a
    // measure-zero set (tangent line/point) or not at all legitimately keeps
    // each operand as its own closed component, and a hollow solid's cavity
    // shell is a separate component by construction — so both the Euler and
    // connectivity checks below evaluate PER COMPONENT: the solid is valid
    // when every component independently forms a closed Euler-consistent
    // shell, and invalid when any component is itself defective.
    let components = face_connectivity_components(&faces, &edge_map);

    #[allow(clippy::cast_possible_wrap)]
    let euler = (v as i64) - (e as i64) + (f as i64);
    // Adjusted Euler: subtract inner loops to get the standard characteristic.
    let adjusted_euler = euler - total_inner_loops;
    let genus_times_2 = 2 - adjusted_euler;
    let mut components_euler_ok = true;
    let mut component_genus_2: Vec<i64> = Vec::new();
    if components.len() <= 1 {
        if genus_times_2 < 0 || genus_times_2 % 2 != 0 {
            components_euler_ok = false;
            issues.push(ValidationIssue {
                severity: Severity::Error,
                description: format!(
                    "Euler characteristic V-E+F = {euler} is invalid \
                     (expected V-E+F = 2+L with L={total_inner_loops} inner loops, \
                     got V={v}, E={e}, F={f})"
                ),
            });
        }
    } else {
        for (ci, comp) in components.iter().enumerate() {
            let (cv, ce, cf, cl) = component_counts(topo, comp)?;
            let comp_euler = cv - ce + cf;
            let comp_genus_2 = 2 - (comp_euler - cl);
            component_genus_2.push(comp_genus_2);
            if comp_genus_2 < 0 || comp_genus_2 % 2 != 0 {
                components_euler_ok = false;
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "Euler characteristic V-E+F = {comp_euler} is invalid for shell \
                         component {ci} (expected V-E+F = 2+L with L={cl} inner loops, \
                         got V={cv}, E={ce}, F={cf})"
                    ),
                });
            }
        }
    }
    let mut boundary_edges = 0;
    let mut non_manifold_edges = 0;

    for (&edge_idx, faces) in &edge_map {
        match faces.len() {
            0 => {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!("edge {edge_idx} is not referenced by any face"),
                });
            }
            1 => {
                boundary_edges += 1;
            }
            2 => {} // correct
            n => {
                non_manifold_edges += 1;
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "edge {edge_idx} is shared by {n} faces (non-manifold, expected 2)"
                    ),
                });
            }
        }
    }

    if boundary_edges > 0 {
        issues.push(ValidationIssue {
            severity: Severity::Error,
            description: format!("{boundary_edges} boundary edge(s) found (shell is not closed)"),
        });
    }

    if non_manifold_edges > 0 {
        issues.push(ValidationIssue {
            severity: Severity::Error,
            description: format!("{non_manifold_edges} non-manifold edge(s) found"),
        });
    }

    issues.extend(shell_orientation_issues(topo, solid, options.orientation)?);

    // Only faces on a planar surface bounded entirely by straight edges
    // require ≥3 unique vertices. Faces with curved edges (Circle,
    // Ellipse, NURBS) or non-planar surfaces (Cylinder, Sphere, Torus,
    // etc.) can validly have fewer vertices because the surface/edge
    // geometry defines the boundary shape.
    let faces = explorer::solid_faces(topo, solid)?;
    for fid in &faces {
        let face_data = topo.face(*fid)?;
        let is_planar = matches!(
            face_data.surface(),
            brepkit_topology::face::FaceSurface::Plane { .. }
        );

        if is_planar && face_all_edges_straight(topo, face_data)? {
            let face_verts = explorer::face_vertices(topo, *fid)?;
            if face_verts.len() < 3 {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "face {} has only {} vertices (need at least 3)",
                        fid.index(),
                        face_verts.len()
                    ),
                });
            }
        }
    }

    let scaled_tol = Tolerance {
        linear: tol.linear * scale,
        angular: tol.angular * scale,
        relative: tol.relative * scale,
    };
    for fid in &faces {
        let face = topo.face(*fid)?;
        if let brepkit_topology::face::FaceSurface::Plane { normal, .. } = face.surface() {
            let len = normal.length();
            if !scaled_tol.approx_eq(len, 1.0) {
                issues.push(ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "face {} has non-unit normal (length = {len})",
                        fid.index()
                    ),
                });
            }
        }
    }

    for fid in &faces {
        let face = topo.face(*fid)?;
        if !periodic_outer_rims_are_full_turns(topo, face)? {
            issues.push(ValidationIssue {
                severity: Severity::Warning,
                description: format!(
                    "periodic face {} has a doubled seam edge but its remaining curved boundary \
                     does not form closed or full-turn rim cycles",
                    fid.index()
                ),
            });
        }
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wire_id in wire_ids {
            let wire = topo.wire(wire_id)?;
            if let Err(_e) = brepkit_topology::validation::validate_wire_closed(wire, topo) {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "wire {} on face {} is not closed",
                        wire_id.index(),
                        fid.index()
                    ),
                });
            }
            issues.extend(ambiguous_circle_arc_warnings(
                topo, *fid, wire_id, scaled_tol,
            )?);
        }
    }

    // Only meaningful for faces bounded entirely by straight edges.
    // The polygon area formula uses vertex positions, which is
    // meaningless when edges are curved (e.g. a cylinder cap has
    // 1 vertex → zero polygon area despite being a valid disc).
    let area_tol_sq = scaled_tol.linear * scaled_tol.linear;
    for fid in &faces {
        let face = topo.face(*fid)?;

        // Skip non-planar faces and faces with curved edges — the polygon
        // area formula is only meaningful for planar faces with straight edges.
        if !matches!(
            face.surface(),
            brepkit_topology::face::FaceSurface::Plane { .. }
        ) {
            continue;
        }
        if !face_all_edges_straight(topo, face)? {
            continue;
        }

        let wire = topo.wire(face.outer_wire())?;

        let mut positions = Vec::new();
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let vid = oe.oriented_start(edge);
            positions.push(topo.vertex(vid)?.point());
        }

        if positions.len() >= 3 {
            let area = polygon_area_3d(&positions);
            if area < area_tol_sq {
                issues.push(ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "face {} has near-zero area ({area:.2e} < {area_tol_sq:.2e})",
                        fid.index()
                    ),
                });
            }
        }
    }

    // Skip intentionally closed edges (like circles) when checking for
    // coincident start/end vertices.
    let all_edges = explorer::solid_edges(topo, solid)?;
    for eid in &all_edges {
        let edge = topo.edge(*eid)?;
        if !edge.is_closed() {
            let p_start = topo.vertex(edge.start())?.point();
            let p_end = topo.vertex(edge.end())?.point();
            let dx = p_start.x() - p_end.x();
            let dy = p_start.y() - p_end.y();
            let dz = p_start.z() - p_end.z();
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist < tol.linear {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "edge {} has near-zero length ({dist:.2e} < {:.2e})",
                        eid.index(),
                        tol.linear
                    ),
                });
            }
        }
    }

    for fid in &faces {
        let face = topo.face(*fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wire_id in wire_ids {
            let wire = topo.wire(wire_id)?;
            if wire.edges().is_empty() {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "wire {} on face {} has no edges",
                        wire_id.index(),
                        fid.index()
                    ),
                });
            }
        }
    }

    // Shell connectivity. Multiple edge-connected components are valid ONLY
    // when every component independently forms a closed Euler-consistent
    // shell (tangent/disjoint fuse keeps each operand whole; a cavity shell
    // shares no edges with the outer shell by construction) AND no two
    // OUTER-shell components materially overlap in space. The overlap veto is
    // what separates a legitimate tangent/disjoint union (components touch on
    // at most a measure-zero set, so their AABB intersection is a
    // zero-thickness slab) from a broken boolean that left tool fragments
    // closed but floating INSIDE the stock (the equal-radius cross-drill:
    // bore lobes sit wholly within the shaft, and blessing them silences the
    // downstream heal that used to repair the body). Cavity (inner-shell)
    // components are exempt — containment is their job. A disconnection
    // paired with ANY closure or per-component-Euler defect is still the
    // classic assembly failure and is reported as before. The historical
    // genus>0 skip is preserved via `components_euler_ok`, which
    // per-component Euler evaluation already accepts for higher-genus
    // components.
    if components.len() > 1 {
        let overlapping =
            outer_components_materially_overlap(topo, solid, &components, &component_genus_2)?;
        if !(components_euler_ok && boundary_edges == 0 && !overlapping) {
            let unreachable = faces.len() - components.first().map_or(0, Vec::len);
            issues.push(ValidationIssue {
                severity: Severity::Error,
                description: format!(
                    "shell is disconnected: {unreachable} face(s) not reachable from first face"
                ),
            });
        }
    }

    {
        let mut face_counts = std::collections::HashMap::new();
        for fid in &faces {
            *face_counts.entry(fid.index()).or_insert(0usize) += 1;
        }
        for (&idx, &count) in &face_counts {
            if count > 1 {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!("face {idx} appears {count} times in shell (redundant)"),
                });
            }
        }
    }

    let vertex_set: std::collections::HashSet<usize> = {
        let verts = explorer::solid_vertices(topo, solid)?;
        verts.iter().map(|v| v.index()).collect()
    };
    for eid in &all_edges {
        let edge = topo.edge(*eid)?;
        if !vertex_set.contains(&edge.start().index()) {
            issues.push(ValidationIssue {
                severity: Severity::Error,
                description: format!(
                    "edge {} start vertex {} not found in solid",
                    eid.index(),
                    edge.start().index()
                ),
            });
        }
        if !vertex_set.contains(&edge.end().index()) {
            issues.push(ValidationIssue {
                severity: Severity::Error,
                description: format!(
                    "edge {} end vertex {} not found in solid",
                    eid.index(),
                    edge.end().index()
                ),
            });
        }
    }

    // Orientation consistency: adjacent faces must traverse a shared edge in
    // opposite effective senses (is_forward XOR is_reversed). Edge-use
    // counting alone cannot see this — the mixed-socket bin's body operand
    // passed every count while 20 shared edges carried same-sense uses,
    // which surfaced two subsystems later as winding-inverted mesh triangles.
    // Delegates to the check-crate shell validator per shell.
    if options.check_orientation {
        let solid_data = topo.solid(solid)?;
        let shells = std::iter::once(solid_data.outer_shell())
            .chain(solid_data.inner_shells().iter().copied())
            .collect::<Vec<_>>();
        for shell_id in shells {
            for issue in brepkit_check::validate::shell::check_shell_orientation(topo, shell_id)
                .map_err(|e| crate::OperationsError::InvalidInput {
                    reason: e.to_string(),
                })?
            {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: issue.description,
                });
            }
        }
    }

    Ok(ValidationReport { issues })
}

/// Validate a solid with relaxed checks suitable for assembled geometry.
///
/// Operations like boolean, fillet, and shell produce solids where faces
/// may not share edges (each face has its own wire/edge topology). These
/// shapes are geometrically correct (volumes, tessellation, I/O all work)
/// but fail strict manifold checks.
///
/// Relaxed mode checks:
/// - Wire closure (every wire forms a closed loop)
/// - Degenerate faces (planar faces with < 3 vertices)
/// - Empty wires
/// - Zero-length edges
/// - Redundant faces
/// - Edge vertex consistency
///
/// Skipped in relaxed mode:
/// - Euler-Poincaré characteristic (assembled shells may have multiple components)
/// - Boundary edges (faces from different operations may not share edges)
/// - Non-manifold edges (edge duplication is expected in assembled geometry)
/// - Shell connectivity (multiple disconnected face groups are valid)
/// - Shell orientation (a shell that is not closed encloses no signed volume to
///   take the sign of, so [`ValidationOptions::orientation`] is not read here)
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn validate_solid_relaxed(
    topo: &Topology,
    solid: SolidId,
) -> Result<ValidationReport, crate::OperationsError> {
    validate_solid_relaxed_with_options(topo, solid, &ValidationOptions::default())
}

/// Validate a solid with relaxed checks and configurable tolerance options.
///
/// Combines the relaxed check set of [`validate_solid_relaxed`] with the
/// tolerance scaling of [`validate_solid_with_options`].
///
/// # Errors
///
/// Returns an error if topology lookups fail.
#[allow(clippy::too_many_lines)]
pub fn validate_solid_relaxed_with_options(
    topo: &Topology,
    solid: SolidId,
    options: &ValidationOptions,
) -> Result<ValidationReport, crate::OperationsError> {
    let mut issues = Vec::new();
    let tol = Tolerance::new();
    // Clamp to [0.1, 1000]: below 0.1 risks false positives on exact
    // geometry, above 1000 makes the check meaningless.
    let scale = options.tolerance_scale.clamp(0.1, 1000.0);

    let faces = explorer::solid_faces(topo, solid)?;

    for fid in &faces {
        let face_data = topo.face(*fid)?;
        let is_planar = matches!(
            face_data.surface(),
            brepkit_topology::face::FaceSurface::Plane { .. }
        );

        if is_planar && face_all_edges_straight(topo, face_data)? {
            let face_verts = explorer::face_vertices(topo, *fid)?;
            if face_verts.len() < 3 {
                issues.push(ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "face {} has only {} vertices (need at least 3)",
                        fid.index(),
                        face_verts.len()
                    ),
                });
            }
        }
    }

    let scaled_tol = Tolerance {
        linear: tol.linear * scale,
        angular: tol.angular * scale,
        relative: tol.relative * scale,
    };
    for fid in &faces {
        let face = topo.face(*fid)?;
        if let brepkit_topology::face::FaceSurface::Plane { normal, .. } = face.surface() {
            let len = normal.length();
            if !scaled_tol.approx_eq(len, 1.0) {
                issues.push(ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "face {} has non-unit normal (length = {len})",
                        fid.index()
                    ),
                });
            }
        }
    }

    // Wire closure — demoted to Warning for relaxed validation.
    // Boolean assembly can produce faces with technically-open wires
    // when edge dedup or vertex merging creates tiny gaps. These are
    // usually below the linear tolerance and don't affect downstream use.
    for fid in &faces {
        let face = topo.face(*fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wire_id in wire_ids {
            let wire = topo.wire(wire_id)?;
            if let Err(_e) = brepkit_topology::validation::validate_wire_closed(wire, topo) {
                // Demoted to Warning: boolean operations can produce
                // micro-gaps in wires from edge splitting that don't affect
                // downstream tessellation or volume. Strict checking would
                // reject ~25% of currently valid boolean results.
                issues.push(ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "wire {} on face {} is not closed",
                        wire_id.index(),
                        fid.index()
                    ),
                });
            }
        }
    }

    let area_tol_sq = scaled_tol.linear * scaled_tol.linear;
    for fid in &faces {
        let face = topo.face(*fid)?;

        if !matches!(
            face.surface(),
            brepkit_topology::face::FaceSurface::Plane { .. }
        ) {
            continue;
        }
        if !face_all_edges_straight(topo, face)? {
            continue;
        }

        let wire = topo.wire(face.outer_wire())?;
        let mut positions = Vec::new();
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let vid = oe.oriented_start(edge);
            positions.push(topo.vertex(vid)?.point());
        }

        if positions.len() >= 3 {
            let area = polygon_area_3d(&positions);
            if area < area_tol_sq {
                issues.push(ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "face {} has near-zero area ({area:.2e} < {area_tol_sq:.2e})",
                        fid.index()
                    ),
                });
            }
        }
    }

    // Zero-length edges — demoted to Warning in relaxed validation.
    // Boolean edge splitting can create tiny edges below tolerance.
    let all_edges = explorer::solid_edges(topo, solid)?;
    for eid in &all_edges {
        let edge = topo.edge(*eid)?;
        if !edge.is_closed() {
            let p_start = topo.vertex(edge.start())?.point();
            let p_end = topo.vertex(edge.end())?.point();
            let dx = p_start.x() - p_end.x();
            let dy = p_start.y() - p_end.y();
            let dz = p_start.z() - p_end.z();
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist < tol.linear {
                issues.push(ValidationIssue {
                    severity: Severity::Warning,
                    description: format!(
                        "edge {} has near-zero length ({dist:.2e} < {:.2e})",
                        eid.index(),
                        tol.linear
                    ),
                });
            }
        }
    }

    for fid in &faces {
        let face = topo.face(*fid)?;
        let wire_ids: Vec<_> = std::iter::once(face.outer_wire())
            .chain(face.inner_wires().iter().copied())
            .collect();

        for wire_id in wire_ids {
            let wire = topo.wire(wire_id)?;
            if wire.edges().is_empty() {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!(
                        "wire {} on face {} has no edges",
                        wire_id.index(),
                        fid.index()
                    ),
                });
            }
        }
    }

    {
        let mut face_counts = std::collections::HashMap::new();
        for fid in &faces {
            *face_counts.entry(fid.index()).or_insert(0usize) += 1;
        }
        for (&idx, &count) in &face_counts {
            if count > 1 {
                issues.push(ValidationIssue {
                    severity: Severity::Error,
                    description: format!("face {idx} appears {count} times in shell (redundant)"),
                });
            }
        }
    }

    let vertex_set: std::collections::HashSet<usize> = {
        let verts = explorer::solid_vertices(topo, solid)?;
        verts.iter().map(|v| v.index()).collect()
    };
    for eid in &all_edges {
        let edge = topo.edge(*eid)?;
        if !vertex_set.contains(&edge.start().index()) {
            issues.push(ValidationIssue {
                severity: Severity::Error,
                description: format!(
                    "edge {} start vertex {} not found in solid",
                    eid.index(),
                    edge.start().index()
                ),
            });
        }
        if !vertex_set.contains(&edge.end().index()) {
            issues.push(ValidationIssue {
                severity: Severity::Error,
                description: format!(
                    "edge {} end vertex {} not found in solid",
                    eid.index(),
                    edge.end().index()
                ),
            });
        }
    }

    Ok(ValidationReport { issues })
}

/// Compute the area of a 3D polygon using the cross-product method.
///
/// For a planar polygon with vertices `p0, p1, ..., pN`, the area is
/// half the magnitude of the sum of cross products `(p[i] - p[0]) × (p[i+1] - p[0])`.
fn polygon_area_3d(positions: &[brepkit_math::vec::Point3]) -> f64 {
    use brepkit_math::vec::Vec3;

    if positions.len() < 3 {
        return 0.0;
    }

    let p0 = positions[0];
    let mut sum = Vec3::new(0.0, 0.0, 0.0);

    for i in 1..positions.len() - 1 {
        let a = positions[i] - p0;
        let b = positions[i + 1] - p0;
        sum += a.cross(b);
    }

    sum.length() * 0.5
}

#[cfg(test)]
mod tests;
