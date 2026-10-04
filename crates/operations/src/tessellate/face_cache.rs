//! Content-keyed per-face mesh reuse for solid tessellation (PERF-D01).
//!
//! A direct edit (`push_pull::move_faces`) returns a new solid with new face
//! handles, yet most of its faces are geometrically unchanged and the moved
//! ones are a rigid translation of their predecessors. This cache lets the
//! next tessellation of the edited body reuse the previous body's per-face
//! triangulation instead of re-meshing every face.
//!
//! # What a face's mesh depends on
//!
//! Stage D of `tessellate_faces_core` meshes one face at a time with
//! `tessellate_face_with_shared_edges`. Its output is a function of:
//!
//! * the face itself: surface carrier, orientation flag, wire structure,
//!   every edge curve, trim and vertex position, and the pcurve of every
//!   edge use on this face;
//! * the boundary plan: the final shared sample chain (global ids and
//!   positions) of every edge of the face, after circle sync, torus
//!   densification, seam splits, contact refinement and Steiner reconcile;
//! * the tolerances and policy: linear deflection, angular tolerance,
//!   `circle_floor`, and the per-face latitude-cap flag;
//! * the merge-grid lookups it performs on the shared vertex pool.
//!
//! The key (`FaceKey`) records all of the first three. Global vertex ids
//! never enter it directly: the chains are recorded as face-local ordinals
//! (first appearance), so the equality pattern the meshers rely on is kept
//! while the numbering of the new body is free to differ. The fourth input
//! is handled at capture and replay time (below).
//!
//! # Exact and translated reuse
//!
//! An **exact** hit has bit-identical content (the face did not move); its
//! replay reproduces the fresh mesh bit for bit, including the boundary
//! vertex normals, whose per-face surface-normal contributions are cached
//! too.
//!
//! A **translated** hit is a face whose positions (vertices, chain samples,
//! curve and surface control points, analytic origins) equal the stored
//! ones shifted by `delta = anchor_new - anchor_old` within
//! `TRANSLATION_ULPS` units of roundoff of the face's coordinate scale
//! (plane offsets compared as `d - n·anchor`), whose unit vectors agree
//! within `DIRECTION_ULPS` (a rebuilt frame carries signed zeros and
//! one-ulp renormalization), and whose every other value (radii, knots,
//! weights, trims, pcurves, tolerances, structure) is bit-identical. Its
//! cached interior vertices are shifted by `delta`; normals are reused as
//! stored.
//!
//! A translate of a fresh mesh is not always the fresh mesh of the
//! translated face: the meshers lay their interior grid out from the
//! boundary's chart bounds and sort band rims by chart angle, so cocircular
//! grid quads and the `rem_euclid(TAU)` seam are decided by roundoff, and a
//! one-ulp change in a chart coordinate flips a diagonal (measured on the
//! Hammer Holder: a moved quarter-cylinder blend re-meshed with flipped grid
//! diagonals). A translated hit is therefore accepted only when the
//! mesher's chart input is bit-identical (`TranslationRule`): every
//! boundary sample's `project_point` coordinates on an analytic surface
//! without pcurves, the `project_by_normal` coordinates on a plane. NURBS
//! faces chart by Newton projection, which cannot be checked without
//! re-projecting, so they are reused only exactly. Refused translates are
//! counted (`FaceMeshCacheStats::translation_refused`) and re-meshed.
//! Boundary-normal contributions of a translated hit are re-evaluated (the
//! NURBS normal projection is loose enough to move them by `1e-2`).
//!
//! # Capture and replay
//!
//! A miss meshes the face normally and then inspects what it appended to
//! the shared pool. The face is cacheable only when every triangle corner
//! is either one of its own boundary-chain vertices or a vertex it created,
//! it rewrote no boundary normal, and the merge map grew by exactly the
//! vertices it interned. Anything else (a weld onto a neighbour's interior
//! vertex, a pole cap that writes rim normals) is counted as uncacheable and
//! never stored.
//!
//! A hit replays exactly the pool operations of the capture: it interns
//! each cached vertex through the same merge grid and pushes it, then emits
//! the triangles with boundary ordinals mapped to the new body's chain ids.
//! If any interned vertex would land on an existing pool vertex (a pool
//! state the capture never saw), the replay rolls back and the face is
//! meshed fresh.
//!
//! # Bounds and scope
//!
//! The cache is per thread and opt-in ([`enable_face_mesh_cache`]); when
//! disabled the pipeline is untouched. It holds at most a configured number
//! of faces and estimated bytes, with deterministic FIFO eviction; an entry
//! larger than the whole byte budget is never retained. Because the key is
//! the content itself, in-place mutation (transforms, `*_mut` edits,
//! healing, checkpoint restore, deletion) cannot serve a stale mesh: changed
//! content simply misses. Holed planar faces (stage B CDT jobs) and the
//! whole-mesh passes (edge sampling, boundary reconcile, weld, dedupe, gap
//! fill) are not cached.

use std::cell::RefCell;
use std::collections::VecDeque;

use remus_math::det_hash::DetHashMap;
use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};

use super::{MERGE_GRID, TriangleMesh, point_merge_key};

/// Default bound on retained face meshes.
pub const DEFAULT_FACE_MESH_CACHE_ENTRIES: usize = 4096;

/// Default bound on estimated retained bytes (keys plus meshes).
pub const DEFAULT_FACE_MESH_CACHE_BYTES: usize = 32 * 1024 * 1024;

/// Allowed positional deviation, in units of roundoff of the face's
/// coordinate scale, for two faces to count as rigid translates.
const TRANSLATION_ULPS: f64 = 1024.0;

/// Allowed deviation of a unit-vector component, in units of roundoff.
const DIRECTION_ULPS: f64 = 64.0;

/// Key layout version; bump when the recorded content changes.
const KEY_VERSION: u64 = 1;

/// Corner flag marking a boundary ordinal (otherwise a local vertex index).
const BOUNDARY_CORNER: u32 = 1 << 31;

/// Grid for the bucket hash of anchor-relative positions. Coarse on
/// purpose: positions straddling a cell only cost a miss, and the full
/// comparison decides every hit.
const BUCKET_GRID: f64 = 1.0 / 4096.0;

/// Positions sampled into the bucket hash (all of them are compared).
const BUCKET_POINTS: usize = 64;

/// Cumulative statistics of this thread's face mesh cache.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FaceMeshCacheStats {
    /// Faces looked up.
    pub lookups: u64,
    /// Hits whose content and position matched bit for bit.
    pub exact_hits: u64,
    /// Hits replayed from a rigid translate whose chart input is
    /// bit-identical (see the module docs).
    pub translated_hits: u64,
    /// Lookups with no usable entry (including refused translates).
    pub misses: u64,
    /// Misses whose content was a rigid translate of a retained face but
    /// whose chart input was not bit-identical, so a replay could differ
    /// from a fresh mesh (see the module docs).
    pub translation_refused: u64,
    /// Hits whose replay met an unexpected pool vertex and re-meshed.
    pub replay_conflicts: u64,
    /// Missed faces whose output could not be captured safely.
    pub uncacheable: u64,
    /// Faces captured and retained.
    pub stored: u64,
    /// Entries evicted (FIFO).
    pub evictions: u64,
    /// Entries currently retained.
    pub entries: usize,
    /// Estimated retained bytes (not RSS).
    pub retained_bytes: usize,
    /// Bound on retained entries.
    pub max_entries: usize,
    /// Bound on estimated retained bytes.
    pub max_bytes: usize,
}

thread_local! {
    static FACE_MESH_CACHE: RefCell<Option<FaceMeshCache>> = const { RefCell::new(None) };
}

/// Enable this thread's face mesh cache with the default bounds.
///
/// Keeps the retained entries when the cache is already enabled.
pub fn enable_face_mesh_cache() {
    enable_face_mesh_cache_with_limits(
        DEFAULT_FACE_MESH_CACHE_ENTRIES,
        DEFAULT_FACE_MESH_CACHE_BYTES,
    );
}

/// Enable this thread's face mesh cache bounded by `max_entries` faces and
/// `max_bytes` estimated bytes.
///
/// When the cache is already enabled its bounds change and the oldest
/// entries are evicted to fit. `max_entries == 0` keeps the cache enabled
/// but retains nothing.
pub fn enable_face_mesh_cache_with_limits(max_entries: usize, max_bytes: usize) {
    FACE_MESH_CACHE.with(|cell| {
        let mut slot = cell.borrow_mut();
        let cache = slot.get_or_insert_with(FaceMeshCache::default);
        cache.max_entries = max_entries;
        cache.max_bytes = max_bytes;
        cache.evict_to_fit(0);
    });
}

/// Disable this thread's face mesh cache and drop its entries.
pub fn disable_face_mesh_cache() {
    FACE_MESH_CACHE.with(|cell| *cell.borrow_mut() = None);
}

/// Drop every retained entry, keeping the cache enabled and its counters.
pub fn clear_face_mesh_cache() {
    FACE_MESH_CACHE.with(|cell| {
        if let Some(cache) = cell.borrow_mut().as_mut() {
            cache.clear();
        }
    });
}

/// Statistics of this thread's face mesh cache, or `None` when disabled.
#[must_use]
pub fn face_mesh_cache_stats() -> Option<FaceMeshCacheStats> {
    FACE_MESH_CACHE.with(|cell| cell.borrow().as_ref().map(FaceMeshCache::stats))
}

/// A cache taken off its thread for a test's fresh reference tessellation.
#[cfg(test)]
pub(super) struct ParkedCache(FaceMeshCache);

/// Take this thread's cache off (disabling it) so a reference tessellation
/// runs fresh; [`unpark`] puts it back with its entries and counters.
#[cfg(test)]
pub(super) fn park() -> Option<ParkedCache> {
    FACE_MESH_CACHE.with(|cell| cell.borrow_mut().take().map(ParkedCache))
}

/// Restore a cache taken by [`park`].
#[cfg(test)]
pub(super) fn unpark(parked: Option<ParkedCache>) {
    if let Some(ParkedCache(cache)) = parked {
        FACE_MESH_CACHE.with(|cell| *cell.borrow_mut() = Some(cache));
    }
}

/// Mesher inputs that are not part of the face itself.
#[derive(Clone, Copy)]
pub(super) struct MeshRequest {
    pub(super) deflection: f64,
    pub(super) angular_tol: f64,
    pub(super) circle_floor: bool,
    pub(super) allow_latitude_cap: bool,
}

/// Content key of one face's stage-D mesh.
#[derive(Debug, Clone)]
pub(super) struct FaceKey {
    /// Structure, flags and face-local ordinals (exact).
    tags: Vec<u64>,
    /// Non-positional values as bits (exact).
    scalars: Vec<u64>,
    /// Unit-vector components (axes, normals), compared within
    /// [`DIRECTION_ULPS`] so a rebuilt frame (signed zeros, one-ulp
    /// renormalization) still matches.
    dirs: Vec<f64>,
    /// Positional values, compared up to a rigid translation.
    points: Vec<Point3>,
    /// Plane `(normal, d)` pairs; `d` is compared as `d - normal·anchor`.
    planes: Vec<(Vec3, f64)>,
    /// Whether (and how) a translated copy of this face may be replayed.
    translation: TranslationRule,
    /// Chart coordinates that must be bit-identical for a translated replay
    /// (see `translation_rule`). Not hashed, so a refused translate still
    /// lands in its bucket and is counted.
    chart: Vec<u64>,
    /// Bucket hash over the exact parts and coarse relative positions.
    hash: u64,
}

/// When a translated match may stand in for a fresh mesh.
///
/// The meshers make a few discrete choices that are decided by roundoff, so
/// a translate of a fresh mesh equals the fresh mesh of the translated face
/// only where those choices cannot flip:
///
/// * Analytic faces chart their boundary with `project_point`; their CDT
///   grid is laid out from the chart bounds and the band meshers sort rims
///   by chart angle, so the cocircular grid quads and the `rem_euclid(TAU)`
///   seam are decided by roundoff. A translate is replayed only when every
///   boundary sample's chart coordinates are bit-identical (recorded in the
///   exact tags, see `translation_rule`).
/// * Planar faces triangulate the boundary projected onto a coordinate
///   plane (`project_by_normal`): a translate is replayed only when that
///   projection is bit-identical.
/// * NURBS faces chart the boundary by Newton projection, which is not
///   bit-reproducible under translation, so they are replayed only when
///   their content is bit-identical.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TranslationRule {
    /// Analytic chart; its boundary chart coordinates are in the exact tags.
    Chart,
    /// Planar: the two coordinates kept by `project_by_normal` must match
    /// bit for bit (the dropped axis index is recorded).
    PlanarProjection(usize),
    /// Only bit-identical content may be replayed.
    Refused,
}

impl FaceKey {
    fn anchor(&self) -> Point3 {
        self.points
            .first()
            .copied()
            .unwrap_or_else(|| Point3::new(0.0, 0.0, 0.0))
    }

    fn scale(&self) -> f64 {
        let mut scale = 1.0_f64;
        for p in &self.points {
            scale = scale.max(p.x().abs()).max(p.y().abs()).max(p.z().abs());
        }
        for (_, d) in &self.planes {
            scale = scale.max(d.abs());
        }
        scale
    }

    const fn bytes(&self) -> usize {
        self.tags.len() * 8
            + self.scalars.len() * 8
            + self.dirs.len() * 8
            + self.points.len() * 24
            + self.planes.len() * 32
            + self.chart.len() * 8
    }
}

/// Word-wise FNV-style mixer: deterministic on every platform.
struct Mixer(u64);

impl Mixer {
    const fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    const fn word(&mut self, w: u64) {
        self.0 = (self.0 ^ w).wrapping_mul(0x0000_0100_0000_01b3);
        self.0 ^= self.0 >> 29;
    }
}

/// Accumulates a [`FaceKey`].
struct KeyBuilder {
    tags: Vec<u64>,
    scalars: Vec<u64>,
    dirs: Vec<f64>,
    points: Vec<Point3>,
    planes: Vec<(Vec3, f64)>,
}

impl KeyBuilder {
    fn tag(&mut self, value: u64) {
        self.tags.push(value);
    }

    fn flag(&mut self, value: bool) {
        self.tags.push(u64::from(value));
    }

    fn count(&mut self, value: usize) {
        self.tags.push(value as u64);
    }

    fn scalar(&mut self, value: f64) {
        self.scalars.push(value.to_bits());
    }

    fn scalars(&mut self, values: &[f64]) {
        self.count(values.len());
        self.scalars.extend(values.iter().map(|v| v.to_bits()));
    }

    fn direction(&mut self, v: Vec3) {
        self.dirs.extend([v.x(), v.y(), v.z()]);
    }

    fn point(&mut self, p: Point3) {
        self.points.push(p);
    }

    fn surface(&mut self, surface: &FaceSurface) {
        match surface {
            FaceSurface::Plane { normal, d } => {
                self.tag(1);
                self.planes.push((*normal, *d));
                self.direction(*normal);
            }
            FaceSurface::Nurbs(s) => {
                self.tag(2);
                self.count(s.degree_u());
                self.count(s.degree_v());
                self.scalars(s.knots_u());
                self.scalars(s.knots_v());
                self.count(s.control_points().len());
                for (row, weights) in s.control_points().iter().zip(s.weights()) {
                    self.count(row.len());
                    for &p in row {
                        self.point(p);
                    }
                    self.scalars(weights);
                }
            }
            FaceSurface::Cylinder(c) => {
                self.tag(3);
                self.point(c.origin());
                self.direction(c.axis());
                self.scalar(c.radius());
                self.direction(c.x_axis());
                self.direction(c.y_axis());
            }
            FaceSurface::Cone(c) => {
                self.tag(4);
                self.point(c.apex());
                self.direction(c.axis());
                self.scalar(c.half_angle());
                self.direction(c.x_axis());
                self.direction(c.y_axis());
            }
            FaceSurface::Sphere(s) => {
                self.tag(5);
                self.point(s.center());
                self.scalar(s.radius());
                self.direction(s.x_axis());
                self.direction(s.y_axis());
                self.direction(s.z_axis());
            }
            FaceSurface::Torus(t) => {
                self.tag(6);
                self.point(t.center());
                self.scalar(t.major_radius());
                self.scalar(t.minor_radius());
                self.direction(t.x_axis());
                self.direction(t.y_axis());
                self.direction(t.z_axis());
            }
        }
    }

    fn curve(&mut self, curve: &EdgeCurve) {
        match curve {
            EdgeCurve::Line => self.tag(1),
            EdgeCurve::NurbsCurve(c) => {
                self.tag(2);
                self.count(c.degree());
                self.scalars(c.knots());
                self.count(c.control_points().len());
                for &p in c.control_points() {
                    self.point(p);
                }
                self.scalars(c.weights());
            }
            EdgeCurve::Circle(c) => {
                self.tag(3);
                self.point(c.center());
                self.direction(c.normal());
                self.scalar(c.radius());
                self.direction(c.u_axis());
                self.direction(c.v_axis());
            }
            EdgeCurve::Ellipse(c) => {
                self.tag(4);
                self.point(c.center());
                self.direction(c.normal());
                self.scalar(c.semi_major());
                self.scalar(c.semi_minor());
                self.direction(c.u_axis());
                self.direction(c.v_axis());
            }
            EdgeCurve::Hyperbola(c) => {
                self.tag(5);
                self.point(c.center());
                self.direction(c.normal());
                self.scalar(c.semi_major());
                self.scalar(c.semi_minor());
                self.direction(c.u_axis());
                self.direction(c.v_axis());
            }
            EdgeCurve::Parabola(c) => {
                self.tag(6);
                self.point(c.vertex());
                self.direction(c.axis_dir());
                self.scalar(c.focal_length());
                self.direction(c.u_axis());
            }
        }
    }

    fn pcurve(&mut self, pcurve: Option<&remus_topology::pcurve::PCurve>) {
        use remus_math::curves2d::Curve2D;
        let Some(pcurve) = pcurve else {
            self.tag(0);
            return;
        };
        self.scalar(pcurve.t_start());
        self.scalar(pcurve.t_end());
        match pcurve.curve() {
            Curve2D::Line(c) => {
                self.tag(1);
                self.scalars(&[
                    c.origin().x(),
                    c.origin().y(),
                    c.direction().x(),
                    c.direction().y(),
                ]);
            }
            Curve2D::Circle(c) => {
                self.tag(2);
                self.scalars(&[c.center().x(), c.center().y(), c.radius()]);
            }
            Curve2D::Ellipse(c) => {
                self.tag(3);
                self.scalars(&[
                    c.center().x(),
                    c.center().y(),
                    c.semi_major(),
                    c.semi_minor(),
                    c.rotation(),
                ]);
            }
            Curve2D::Nurbs(c) => {
                self.tag(4);
                self.count(c.degree());
                self.scalars(c.knots());
                self.count(c.control_points().len());
                for p in c.control_points() {
                    self.scalar(p.x());
                    self.scalar(p.y());
                }
                self.scalars(c.weights());
            }
        }
    }

    fn finish(self, translation: TranslationRule, chart: Vec<u64>) -> FaceKey {
        let mut mixer = Mixer::new();
        for &t in &self.tags {
            mixer.word(t);
        }
        mixer.word(u64::MAX);
        for &s in &self.scalars {
            mixer.word(s);
        }
        mixer.word(u64::MAX);
        for &d in &self.dirs {
            // Coarse on purpose (see `BUCKET_GRID`); `-0.0` rounds to 0.
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            mixer.word((d * 4096.0).round() as i64 as u64);
        }
        mixer.word(self.points.len() as u64);
        mixer.word(self.planes.len() as u64);
        if let Some(&anchor) = self.points.first() {
            let step = (self.points.len() / BUCKET_POINTS).max(1);
            for p in self.points.iter().step_by(step) {
                let rel = *p - anchor;
                for c in [rel.x(), rel.y(), rel.z()] {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    mixer.word((c / BUCKET_GRID).round() as i64 as u64);
                }
            }
        }
        FaceKey {
            tags: self.tags,
            scalars: self.scalars,
            dirs: self.dirs,
            points: self.points,
            planes: self.planes,
            translation,
            chart,
            hash: mixer.0,
        }
    }
}

/// Face-local identity of the boundary-chain vertices: ordinal to global id.
pub(super) struct FaceBoundary {
    gids: Vec<u32>,
    /// `(gid, ordinal)` sorted by gid for lookup.
    by_gid: Vec<(u32, u32)>,
}

impl FaceBoundary {
    fn ordinal(&self, gid: u32) -> Option<u32> {
        self.by_gid
            .binary_search_by_key(&gid, |&(g, _)| g)
            .ok()
            .map(|i| self.by_gid[i].1)
    }
}

/// Build the content key of a face and the identity of its boundary
/// vertices in the current pool.
///
/// # Errors
///
/// Propagates topology lookup failures.
pub(super) fn face_key(
    topo: &Topology,
    face_id: FaceId,
    request: MeshRequest,
    edge_chains: &DetHashMap<usize, Vec<u32>>,
    positions: &[Point3],
) -> Result<(FaceKey, FaceBoundary), crate::OperationsError> {
    let face = topo.face(face_id)?;
    let mut b = KeyBuilder {
        tags: Vec::new(),
        scalars: Vec::new(),
        dirs: Vec::new(),
        points: Vec::new(),
        planes: Vec::new(),
    };
    b.tag(KEY_VERSION);
    b.scalar(request.deflection);
    b.scalar(request.angular_tol);
    b.flag(request.circle_floor);
    b.flag(request.allow_latitude_cap);
    b.flag(face.is_reversed());

    let mut edge_ordinals: DetHashMap<usize, u64> = DetHashMap::default();
    let mut vertex_ordinals: DetHashMap<usize, u64> = DetHashMap::default();
    let mut gid_ordinals: DetHashMap<u32, u32> = DetHashMap::default();
    let mut gids: Vec<u32> = Vec::new();
    // Vertices and chain samples: the points the meshers place in the chart.
    let mut boundary_points: Vec<Point3> = Vec::new();
    let mut has_pcurve = false;

    let wires: Vec<_> = std::iter::once(face.outer_wire())
        .chain(face.inner_wires().iter().copied())
        .collect();
    b.count(wires.len());
    for wire_id in wires {
        let wire = topo.wire(wire_id)?;
        b.count(wire.edges().len());
        b.flag(wire.is_closed());
        for oriented in wire.edges() {
            let edge_id = oriented.edge();
            let edge_index = edge_id.index();
            b.flag(oriented.is_forward());
            let pcurve = topo.pcurve_oriented(edge_id, face_id, oriented.is_forward());
            has_pcurve |= pcurve.is_some();
            b.pcurve(pcurve);
            let next = edge_ordinals.len() as u64;
            let ordinal = *edge_ordinals.entry(edge_index).or_insert(next);
            b.tag(ordinal);
            if ordinal != next {
                continue;
            }
            let edge = topo.edge(edge_id)?;
            for vertex_id in [edge.start(), edge.end()] {
                let next = vertex_ordinals.len() as u64;
                let ordinal = *vertex_ordinals.entry(vertex_id.index()).or_insert(next);
                b.tag(ordinal);
                if ordinal == next {
                    let point = topo.vertex(vertex_id)?.point();
                    b.point(point);
                    boundary_points.push(point);
                }
            }
            b.curve(edge.curve());
            match edge.trim() {
                Some((t0, t1)) => {
                    b.tag(1);
                    b.scalar(t0);
                    b.scalar(t1);
                }
                None => b.tag(0),
            }
            match edge_chains.get(&edge_index) {
                Some(chain) => {
                    b.count(chain.len() + 1);
                    for &gid in chain {
                        let next = u32::try_from(gids.len()).unwrap_or(u32::MAX);
                        let ordinal = *gid_ordinals.entry(gid).or_insert(next);
                        b.tag(u64::from(ordinal));
                        if ordinal == next {
                            gids.push(gid);
                            let point = positions
                                .get(gid as usize)
                                .copied()
                                .unwrap_or_else(|| Point3::new(f64::NAN, f64::NAN, f64::NAN));
                            b.point(point);
                            boundary_points.push(point);
                        }
                    }
                }
                None => b.tag(0),
            }
        }
    }
    let mut signature = Vec::new();
    let translation =
        translation_rule(face.surface(), &boundary_points, has_pcurve, &mut signature);
    b.surface(face.surface());

    let mut by_gid: Vec<(u32, u32)> = gid_ordinals.into_iter().collect();
    by_gid.sort_unstable();
    Ok((
        b.finish(translation, signature),
        FaceBoundary { gids, by_gid },
    ))
}

/// Decide whether a translate of this face may be replayed, and record the
/// chart input that must then be bit-identical.
///
/// A translated replay equals a fresh mesh of the moved face only when the
/// mesher sees the same chart coordinates: its interior grid is laid out
/// from the boundary's chart bounds, and the cocircular quads of that grid
/// (and the angle sorts of the band meshers) are decided by roundoff, so a
/// one-ulp change in a boundary chart coordinate can flip a diagonal.
///
/// * Analytic surfaces chart the boundary with `project_point`; the bits of
///   every boundary sample's `(u, v)` are appended to `signature` (compared
///   exactly). A face whose edge uses carry pcurves charts through them
///   instead and is refused.
/// * Planes are handled in [`match_keys`] (their projection is the kept pair
///   of world coordinates).
/// * NURBS faces chart the boundary by Newton projection, which is not
///   reproducible bit for bit under translation: refused.
fn translation_rule(
    surface: &FaceSurface,
    boundary: &[Point3],
    has_pcurve: bool,
    signature: &mut Vec<u64>,
) -> TranslationRule {
    let mut chart = |project: &dyn Fn(Point3) -> (f64, f64)| {
        signature.push(boundary.len() as u64);
        for &p in boundary {
            let (u, v) = project(p);
            signature.push(u.to_bits());
            signature.push(v.to_bits());
        }
        TranslationRule::Chart
    };
    match surface {
        FaceSurface::Plane { normal, .. } => {
            let (ax, ay, az) = (normal.x().abs(), normal.y().abs(), normal.z().abs());
            let dropped = if az >= ax && az >= ay {
                2
            } else if ay >= ax {
                1
            } else {
                0
            };
            TranslationRule::PlanarProjection(dropped)
        }
        FaceSurface::Nurbs(_) => TranslationRule::Refused,
        FaceSurface::Cylinder(_)
        | FaceSurface::Cone(_)
        | FaceSurface::Sphere(_)
        | FaceSurface::Torus(_)
            if has_pcurve =>
        {
            TranslationRule::Refused
        }
        FaceSurface::Cylinder(c) => chart(&|p| c.project_point(p)),
        FaceSurface::Cone(c) => chart(&|p| c.project_point(p)),
        FaceSurface::Sphere(s) => chart(&|p| s.project_point(p)),
        FaceSurface::Torus(t) => chart(&|p| t.project_point(p)),
    }
}

/// How a stored key relates to a probe key.
#[derive(Clone, Copy, PartialEq, Eq)]
enum KeyMatch {
    /// Bit-identical content at the same position.
    Exact,
    /// Rigid translate within roundoff whose chart input is bit-identical.
    Translated,
    /// Rigid translate within roundoff that the face's
    /// [`TranslationRule`] does not allow to stand in for a fresh mesh.
    TranslationRefused,
}

fn match_keys(stored: &FaceKey, probe: &FaceKey) -> Option<KeyMatch> {
    if stored.hash != probe.hash
        || stored.tags != probe.tags
        || stored.scalars != probe.scalars
        || stored.dirs.len() != probe.dirs.len()
        || stored.points.len() != probe.points.len()
        || stored.planes.len() != probe.planes.len()
    {
        return None;
    }
    let bitwise = stored
        .dirs
        .iter()
        .zip(&probe.dirs)
        .all(|(a, b)| a.to_bits() == b.to_bits())
        && stored
            .points
            .iter()
            .zip(&probe.points)
            .all(|(a, b)| same_point_bits(*a, *b))
        && stored
            .planes
            .iter()
            .zip(&probe.planes)
            .all(|(a, b)| a.1.to_bits() == b.1.to_bits());
    if bitwise {
        return Some(KeyMatch::Exact);
    }
    let mut allowed = probe.translation != TranslationRule::Refused
        && stored.translation == probe.translation
        && stored.chart == probe.chart;
    if let TranslationRule::PlanarProjection(dropped) = probe.translation
        && allowed
    {
        let kept = |p: &Point3| -> [u64; 2] {
            let c = [p.x(), p.y(), p.z()];
            let (a, b) = match dropped {
                0 => (c[1], c[2]),
                1 => (c[0], c[2]),
                _ => (c[0], c[1]),
            };
            [a.to_bits(), b.to_bits()]
        };
        allowed = stored
            .points
            .iter()
            .zip(&probe.points)
            .all(|(a, b)| kept(a) == kept(b));
    }
    let dir_tol = DIRECTION_ULPS * f64::EPSILON;
    if stored
        .dirs
        .iter()
        .zip(&probe.dirs)
        .any(|(a, b)| (a - b).abs() > dir_tol)
    {
        return None;
    }
    let tol = TRANSLATION_ULPS * f64::EPSILON * stored.scale().max(probe.scale());
    let (sa, pa) = (stored.anchor(), probe.anchor());
    let within = |a: f64, b: f64| (a - b).abs() <= tol;
    for (s, p) in stored.points.iter().zip(&probe.points) {
        let (rs, rp) = (*s - sa, *p - pa);
        if !(within(rs.x(), rp.x()) && within(rs.y(), rp.y()) && within(rs.z(), rp.z())) {
            return None;
        }
    }
    for ((sn, sd), (pn, pd)) in stored.planes.iter().zip(&probe.planes) {
        let rs = sd - sn.dot(Vec3::new(sa.x(), sa.y(), sa.z()));
        let rp = pd - pn.dot(Vec3::new(pa.x(), pa.y(), pa.z()));
        if !within(rs, rp) {
            return None;
        }
    }
    Some(if allowed {
        KeyMatch::Translated
    } else {
        KeyMatch::TranslationRefused
    })
}

fn same_point_bits(a: Point3, b: Point3) -> bool {
    a.x().to_bits() == b.x().to_bits()
        && a.y().to_bits() == b.y().to_bits()
        && a.z().to_bits() == b.z().to_bits()
}

fn same_vec_bits(a: Vec3, b: Vec3) -> bool {
    a.x().to_bits() == b.x().to_bits()
        && a.y().to_bits() == b.y().to_bits()
        && a.z().to_bits() == b.z().to_bits()
}

/// Surface-normal contribution of a face at one of its boundary vertices.
#[derive(Debug, Clone, Copy)]
pub(super) enum NormalSlot {
    /// Not evaluated when the entry was captured.
    Unknown,
    /// The oriented contribution (`None` where the surface has no normal).
    Known(Option<Vec3>),
}

/// One retained face mesh.
#[derive(Debug, Clone)]
struct CachedFace {
    key: FaceKey,
    positions: Vec<Point3>,
    normals: Vec<Vec3>,
    interned: Vec<bool>,
    corners: Vec<u32>,
    boundary_normals: Vec<NormalSlot>,
    bytes: usize,
}

impl CachedFace {
    const fn estimate_bytes(&mut self) {
        self.bytes = std::mem::size_of::<Self>()
            + self.key.bytes()
            + self.positions.len() * 24
            + self.normals.len() * 24
            + self.interned.len()
            + self.corners.len() * 4
            + self.boundary_normals.len() * std::mem::size_of::<NormalSlot>();
    }
}

#[derive(Default)]
struct FaceMeshCache {
    max_entries: usize,
    max_bytes: usize,
    next_id: u64,
    order: VecDeque<u64>,
    entries: DetHashMap<u64, CachedFace>,
    buckets: DetHashMap<u64, Vec<u64>>,
    retained_bytes: usize,
    stats: FaceMeshCacheStats,
}

impl FaceMeshCache {
    fn stats(&self) -> FaceMeshCacheStats {
        FaceMeshCacheStats {
            entries: self.entries.len(),
            retained_bytes: self.retained_bytes,
            max_entries: self.max_entries,
            max_bytes: self.max_bytes,
            ..self.stats
        }
    }

    fn clear(&mut self) {
        self.order.clear();
        self.entries.clear();
        self.buckets.clear();
        self.retained_bytes = 0;
    }

    /// Best usable entry for `key` (exact before translated), or the
    /// refusal when only a refused translate exists.
    fn find(&self, key: &FaceKey) -> Option<(u64, KeyMatch)> {
        let mut best: Option<(u64, KeyMatch)> = None;
        for &id in self.buckets.get(&key.hash)? {
            let Some(kind) = self.entries.get(&id).and_then(|e| match_keys(&e.key, key)) else {
                continue;
            };
            match kind {
                KeyMatch::Exact => return Some((id, kind)),
                KeyMatch::Translated => {
                    if best.is_none_or(|(_, k)| k == KeyMatch::TranslationRefused) {
                        best = Some((id, kind));
                    }
                }
                KeyMatch::TranslationRefused => {
                    if best.is_none() {
                        best = Some((id, kind));
                    }
                }
            }
        }
        best
    }

    /// Evict FIFO until `incoming` more bytes (and, when nonzero, one more
    /// entry) fit the bounds.
    fn evict_to_fit(&mut self, incoming: usize) {
        while let Some(&oldest) = self.order.front() {
            let over_count = self.entries.len() + usize::from(incoming > 0) > self.max_entries;
            let over_bytes = self.retained_bytes + incoming > self.max_bytes;
            if !over_count && !over_bytes {
                break;
            }
            self.order.pop_front();
            if let Some(entry) = self.entries.remove(&oldest) {
                self.retained_bytes -= entry.bytes;
                if let Some(bucket) = self.buckets.get_mut(&entry.key.hash) {
                    bucket.retain(|&id| id != oldest);
                    if bucket.is_empty() {
                        self.buckets.remove(&entry.key.hash);
                    }
                }
                self.stats.evictions += 1;
            }
        }
    }

    fn insert(&mut self, entry: CachedFace) {
        if self.max_entries == 0 || entry.bytes > self.max_bytes {
            return;
        }
        // Identical faces captured in one call (or re-meshed after a replay
        // conflict) keep the first entry.
        if self
            .find(&entry.key)
            .is_some_and(|(_, kind)| kind != KeyMatch::TranslationRefused)
        {
            return;
        }
        self.evict_to_fit(entry.bytes);
        let id = self.next_id;
        self.next_id += 1;
        self.retained_bytes += entry.bytes;
        self.buckets.entry(entry.key.hash).or_default().push(id);
        self.order.push_back(id);
        self.entries.insert(id, entry);
        self.stats.stored += 1;
    }
}

/// Per-face state held by a tessellation call.
struct SessionFace {
    boundary: FaceBoundary,
    normals: Vec<NormalSlot>,
    /// Captured mesh awaiting its normal contributions and storage.
    pending: Option<CachedFace>,
}

/// Pool state recorded before a face is meshed, for capture.
pub(super) struct CaptureStart {
    pos_start: usize,
    idx_start: usize,
    map_len: usize,
    boundary_normals: Vec<Vec3>,
}

/// Cache access for one `tessellate_faces_core` call.
///
/// Created only when this thread's cache is enabled. Captures are stored by
/// [`Session::commit`] after the whole tessellation succeeded, so a failed
/// tessellation never leaves entries behind.
pub(super) struct Session {
    faces: DetHashMap<FaceId, SessionFace>,
}

impl Session {
    /// A session when this thread's cache is enabled.
    pub(super) fn begin() -> Option<Self> {
        FACE_MESH_CACHE.with(|cell| {
            cell.borrow().as_ref().map(|_| Self {
                faces: DetHashMap::default(),
            })
        })
    }

    /// Try to emit the face from the cache. `Ok(())` when the face's
    /// triangles were appended to `merged`; otherwise the boundary is handed
    /// back for a fresh mesh and capture.
    pub(super) fn try_replay(
        &mut self,
        face_id: FaceId,
        key: &FaceKey,
        boundary: FaceBoundary,
        merged: &mut TriangleMesh,
        point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
    ) -> Result<(), FaceBoundary> {
        let outcome = FACE_MESH_CACHE.with(|cell| {
            let mut slot = cell.borrow_mut();
            let cache = slot.as_mut()?;
            cache.stats.lookups += 1;
            let found = cache.find(key);
            if let Some((_, KeyMatch::TranslationRefused)) = found {
                cache.stats.misses += 1;
                cache.stats.translation_refused += 1;
                return None;
            }
            let Some((id, kind)) = found else {
                cache.stats.misses += 1;
                return None;
            };
            let entry = cache.entries.get(&id)?;
            let delta = key.anchor() - entry.key.anchor();
            if replay(entry, &boundary, kind, delta, merged, point_to_global) {
                // Boundary-normal contributions come from loose surface
                // projections (NURBS) that are not translation exact: reuse
                // them only for bit-identical content.
                let normals = if kind == KeyMatch::Exact {
                    entry.boundary_normals.clone()
                } else {
                    vec![NormalSlot::Unknown; boundary.gids.len()]
                };
                if kind == KeyMatch::Exact {
                    cache.stats.exact_hits += 1;
                } else {
                    cache.stats.translated_hits += 1;
                }
                Some(normals)
            } else {
                cache.stats.replay_conflicts += 1;
                None
            }
        });
        match outcome {
            Some(normals) => {
                self.faces.insert(
                    face_id,
                    SessionFace {
                        boundary,
                        normals,
                        pending: None,
                    },
                );
                Ok(())
            }
            None => Err(boundary),
        }
    }

    /// Record the pool state before meshing a missed face.
    pub(super) fn capture_start(
        boundary: &FaceBoundary,
        merged: &TriangleMesh,
        point_to_global: &DetHashMap<(i64, i64, i64), u32>,
    ) -> CaptureStart {
        CaptureStart {
            pos_start: merged.positions.len(),
            idx_start: merged.indices.len(),
            map_len: point_to_global.len(),
            boundary_normals: boundary
                .gids
                .iter()
                .map(|&gid| {
                    merged
                        .normals
                        .get(gid as usize)
                        .copied()
                        .unwrap_or_else(|| Vec3::new(0.0, 0.0, 0.0))
                })
                .collect(),
        }
    }

    /// Capture what the face just appended, when it is safe to replay.
    pub(super) fn capture_finish(
        &mut self,
        face_id: FaceId,
        key: FaceKey,
        boundary: FaceBoundary,
        start: &CaptureStart,
        merged: &TriangleMesh,
        point_to_global: &DetHashMap<(i64, i64, i64), u32>,
    ) {
        let pending = capture(key, &boundary, start, merged, point_to_global);
        if pending.is_none() {
            FACE_MESH_CACHE.with(|cell| {
                if let Some(cache) = cell.borrow_mut().as_mut() {
                    cache.stats.uncacheable += 1;
                }
            });
        }
        let normals = vec![NormalSlot::Unknown; boundary.gids.len()];
        self.faces.insert(
            face_id,
            SessionFace {
                boundary,
                normals,
                pending,
            },
        );
    }

    /// Cached oriented surface-normal contribution of `face` at boundary
    /// vertex `gid` ([`NormalSlot::Unknown`] when it must be evaluated).
    pub(super) fn cached_normal(&self, face: FaceId, gid: u32) -> NormalSlot {
        self.faces
            .get(&face)
            .and_then(|state| {
                let ordinal = state.boundary.ordinal(gid)? as usize;
                state.normals.get(ordinal).copied()
            })
            .unwrap_or(NormalSlot::Unknown)
    }

    /// Record a freshly evaluated contribution for a face of this call.
    pub(super) fn record_normal(&mut self, face: FaceId, gid: u32, normal: Option<Vec3>) {
        let Some(state) = self.faces.get_mut(&face) else {
            return;
        };
        let Some(ordinal) = state.boundary.ordinal(gid) else {
            return;
        };
        if let Some(slot) = state.normals.get_mut(ordinal as usize) {
            *slot = NormalSlot::Known(normal);
        }
    }

    /// Store every capture of this (successful) tessellation.
    pub(super) fn commit(self) {
        FACE_MESH_CACHE.with(|cell| {
            let mut slot = cell.borrow_mut();
            let Some(cache) = slot.as_mut() else {
                return;
            };
            let mut faces: Vec<(FaceId, SessionFace)> = self.faces.into_iter().collect();
            faces.sort_unstable_by_key(|(face, _)| face.index());
            for (_, state) in faces {
                if let Some(mut entry) = state.pending {
                    entry.boundary_normals = state.normals;
                    entry.estimate_bytes();
                    cache.insert(entry);
                }
            }
        });
    }
}

fn capture(
    key: FaceKey,
    boundary: &FaceBoundary,
    start: &CaptureStart,
    merged: &TriangleMesh,
    point_to_global: &DetHashMap<(i64, i64, i64), u32>,
) -> Option<CachedFace> {
    // A rewritten boundary normal is a side effect on shared state.
    for (ordinal, &gid) in boundary.gids.iter().enumerate() {
        let now = merged.normals.get(gid as usize)?;
        let before = start.boundary_normals.get(ordinal)?;
        if !same_vec_bits(*now, *before) {
            return None;
        }
    }
    let pos_start = start.pos_start;
    let mut corners = Vec::with_capacity(merged.indices.len().saturating_sub(start.idx_start));
    for &gid in merged.indices.get(start.idx_start..)? {
        if gid as usize >= pos_start {
            let local = u32::try_from(gid as usize - pos_start).ok()?;
            if local & BOUNDARY_CORNER != 0 {
                return None;
            }
            corners.push(local);
        } else {
            // A corner on any other pool vertex couples this face to a
            // neighbour's output: not replayable from this face alone.
            corners.push(boundary.ordinal(gid)? | BOUNDARY_CORNER);
        }
    }
    let positions = merged.positions.get(pos_start..)?.to_vec();
    let normals = merged.normals.get(pos_start..)?.to_vec();
    if normals.len() != positions.len() {
        return None;
    }
    let interned: Vec<bool> = positions
        .iter()
        .enumerate()
        .map(|(k, &p)| {
            point_to_global
                .get(&point_merge_key(p, MERGE_GRID))
                .copied()
                == u32::try_from(pos_start + k).ok()
        })
        .collect();
    let interned_count = interned.iter().filter(|&&i| i).count();
    if point_to_global.len() != start.map_len + interned_count {
        return None;
    }
    Some(CachedFace {
        key,
        positions,
        normals,
        interned,
        corners,
        boundary_normals: Vec::new(),
        bytes: 0,
    })
}

/// Replay a cached face into the pool. On any merge-grid conflict the pool
/// is restored and `false` returned.
fn replay(
    entry: &CachedFace,
    boundary: &FaceBoundary,
    kind: KeyMatch,
    delta: Vec3,
    merged: &mut TriangleMesh,
    point_to_global: &mut DetHashMap<(i64, i64, i64), u32>,
) -> bool {
    use std::collections::hash_map::Entry;

    let pos_start = merged.positions.len();
    let shift = kind == KeyMatch::Translated;
    let mut inserted: Vec<(i64, i64, i64)> = Vec::new();
    let mut conflict = false;
    for (k, (&p, &n)) in entry.positions.iter().zip(&entry.normals).enumerate() {
        let p = if shift { p + delta } else { p };
        if entry.interned.get(k).copied().unwrap_or(false) {
            let Ok(gid) = u32::try_from(pos_start + k) else {
                conflict = true;
                break;
            };
            let key = point_merge_key(p, MERGE_GRID);
            match point_to_global.entry(key) {
                Entry::Occupied(_) => {
                    conflict = true;
                    break;
                }
                Entry::Vacant(slot) => {
                    slot.insert(gid);
                    inserted.push(key);
                }
            }
        }
        merged.positions.push(p);
        merged.normals.push(n);
    }
    if !conflict {
        let mut indices = Vec::with_capacity(entry.corners.len());
        for &corner in &entry.corners {
            let gid = if corner & BOUNDARY_CORNER == 0 {
                u32::try_from(pos_start + corner as usize).ok()
            } else {
                boundary
                    .gids
                    .get((corner & !BOUNDARY_CORNER) as usize)
                    .copied()
            };
            if let Some(gid) = gid {
                indices.push(gid);
            } else {
                conflict = true;
                break;
            }
        }
        if !conflict {
            merged.indices.extend_from_slice(&indices);
            return true;
        }
    }
    merged.positions.truncate(pos_start);
    merged.normals.truncate(pos_start);
    for key in inserted {
        point_to_global.remove(&key);
    }
    false
}
