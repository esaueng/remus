//! B51: the torus–cone placements a deterministic 60-draw sweep of the B26
//! slow lattice left failing after B39.
//!
//! Two of them are one defect. A torus tube that runs THROUGH a frustum wall
//! (in one side, out another, clear of both caps) leaves the wall with two
//! pierce holes, each bounded by one closed marched torus × cone curve. When a
//! hole straddles the wall's seam meridian, the developed-chart mesher cuts
//! the hole loop at the seam and fabricates a vertex there. Cylinder walls
//! had their hole polylines pre-split at that crossing so both faces share
//! the vertex; cone walls did not, so the torus band stitched the unsplit
//! polyline and the wall the split one — a three-edge T-junction per
//! crossing. The sub-deflection gap filler closed it at preview deflections
//! and could not at the B26 scale-derived one (6 boundary edges, two
//! crossings), on a B-Rep that was exact and valid throughout.
//!
//! - **oblique** (sweep draw 5): cone 3/1/2.5 × torus 2.5/0.75,
//!   `T(-1, 1.5, 1.5)·Rz(π/4)`.
//! - **axis-aligned** (sweep draw 19): cone 1/2.5/3 × torus 4/0.75,
//!   `T(3, 4, 2)`.
//!
//! The third is a different cell, closed before this fixture by the certified
//! disjoint-carrier work and pinned here so it cannot regress:
//!
//! - **disjoint** (sweep draw 25): torus 2/0.75 × cone 1/1.5/1 at 1e3 scale,
//!   `T(0, 0, 3)·Rz(π/2)`. The cone floats above the torus's hole; the fuse
//!   used to return the cone alone (4.97e9 against 2.22e10 + 4.97e9).
//!
//! Every oracle is independent of the kernel's own volume and point
//! classification: closed-form operand volumes, a semi-analytic torus ∩
//! frustum integral (inclusion–exclusion), the result mesh's own signed
//! volume, and ray parity against that mesh at grid points whose expected
//! side analytic membership decides.
#![allow(
    clippy::disallowed_methods,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic
)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};
use std::sync::Mutex;

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_cone, make_torus};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
    welded_mesh_quality,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

/// One sweep draw: the stock sits at the origin, the tool under `placement`
/// (rotation, then the offset), every length multiplied by `scale`.
#[derive(Clone, Copy, Debug)]
struct Cell {
    /// Frustum `(r0, r1, h)`: base radius at z = 0, top radius at z = h.
    cone: (f64, f64, f64),
    /// Torus `(major, minor)`.
    torus: (f64, f64),
    cone_is_stock: bool,
    offset: (f64, f64, f64),
    angle_z: f64,
    scale: f64,
}

const OBLIQUE: Cell = Cell {
    cone: (3.0, 1.0, 2.5),
    torus: (2.5, 0.75),
    cone_is_stock: true,
    offset: (-1.0, 1.5, 1.5),
    angle_z: FRAC_PI_4,
    scale: 1.0,
};

const AXIS_ALIGNED: Cell = Cell {
    cone: (1.0, 2.5, 3.0),
    torus: (4.0, 0.75),
    cone_is_stock: true,
    offset: (3.0, 4.0, 2.0),
    angle_z: 0.0,
    scale: 1.0,
};

const DISJOINT: Cell = Cell {
    cone: (1.0, 1.5, 1.0),
    torus: (2.0, 0.75),
    cone_is_stock: false,
    offset: (0.0, 0.0, 3.0),
    angle_z: FRAC_PI_2,
    scale: 1000.0,
};

impl Cell {
    fn cone_dims(self) -> (f64, f64, f64) {
        let (r0, r1, h) = self.cone;
        (r0 * self.scale, r1 * self.scale, h * self.scale)
    }

    fn torus_dims(self) -> (f64, f64) {
        (self.torus.0 * self.scale, self.torus.1 * self.scale)
    }

    fn placement(self) -> Mat4 {
        let (x, y, z) = self.offset;
        Mat4::translation(x * self.scale, y * self.scale, z * self.scale)
            * Mat4::rotation_z(self.angle_z)
    }

    /// World frame of the cone and of the torus.
    fn frames(self) -> (Mat4, Mat4) {
        if self.cone_is_stock {
            (Mat4::identity(), self.placement())
        } else {
            (self.placement(), Mat4::identity())
        }
    }

    /// `(topology, stock, tool)`.
    fn operands(self) -> (Topology, SolidId, SolidId) {
        let mut topo = Topology::new();
        let (r0, r1, h) = self.cone_dims();
        let (major, minor) = self.torus_dims();
        let (stock, tool) = if self.cone_is_stock {
            let cone = make_cone(&mut topo, r0, r1, h).unwrap();
            (cone, make_torus(&mut topo, major, minor, 16).unwrap())
        } else {
            let torus = make_torus(&mut topo, major, minor, 16).unwrap();
            (torus, make_cone(&mut topo, r0, r1, h).unwrap())
        };
        transform_solid(&mut topo, tool, &self.placement()).unwrap();
        (topo, stock, tool)
    }

    fn exact(self, op: BooleanOp) -> (Topology, SolidId) {
        let (mut topo, stock, tool) = self.operands();
        let outcome = boolean_with_context(
            &mut topo,
            op,
            stock,
            tool,
            &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
        )
        .unwrap_or_else(|e| panic!("{self:?} {op:?}: exact boolean refused: {e:?}"));
        assert!(
            matches!(outcome.quality, BooleanQuality::Exact),
            "{self:?} {op:?}: ExactOnly returned {:?}",
            outcome.quality
        );
        (topo, outcome.solid)
    }

    fn cone_volume(self) -> f64 {
        let (r0, r1, h) = self.cone_dims();
        PI * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0
    }

    fn torus_volume(self) -> f64 {
        let (major, minor) = self.torus_dims();
        2.0 * PI * PI * major * minor * minor
    }

    /// Signed distance into the tube: positive inside.
    fn tube_depth(self, torus_inverse: &Mat4, p: Point3) -> f64 {
        let (major, minor) = self.torus_dims();
        let q = torus_inverse.mul_point(p);
        minor - (q.x().hypot(q.y()) - major).hypot(q.z())
    }

    /// A lower bound of the depth into the frustum: positive inside.
    fn frustum_depth(self, cone_inverse: &Mat4, p: Point3) -> f64 {
        let (r0, r1, h) = self.cone_dims();
        let q = cone_inverse.mul_point(p);
        let r = r0 + (r1 - r0) * q.z() / h;
        // The wall leans, so the radial slack overstates the distance to it
        // by at most 1/cos(half-angle); the cosine restores a true bound.
        let lean = h / h.hypot(r1 - r0);
        q.z().min(h - q.z()).min((r - q.x().hypot(q.y())) * lean)
    }

    /// Torus ∩ frustum by the midpoint rule over the frustum's local
    /// `(z, θ)`, with EXACT radial intervals: along each ray from the axis
    /// the tube's boundary is the line–torus quartic, so `∫ρ dρ` over the
    /// inside intervals is closed form. The interval length has a square-root
    /// end where a ray grazes the tube, so the rule converges as `h^1.5`:
    /// `independent_integral_is_converged` bounds the resolution used here
    /// by its own halving. Computed once per cell.
    fn intersection_volume(self) -> f64 {
        static CACHE: Mutex<Vec<(u64, f64)>> = Mutex::new(Vec::new());
        // The cells differ in their torus major radius and scale.
        let key = (self.torus.0 * self.scale).to_bits();
        let mut cache = CACHE.lock().unwrap();
        if let Some(&(_, volume)) = cache.iter().find(|(k, _)| *k == key) {
            return volume;
        }
        let volume = self.intersection_volume_at(600, 1200);
        cache.push((key, volume));
        volume
    }

    fn intersection_volume_at(self, nz: u32, nt: u32) -> f64 {
        let (r0, r1, h) = self.cone_dims();
        let (major, minor) = self.torus_dims();
        let (cone_frame, torus_frame) = self.frames();
        let to_torus = torus_frame.inverse().unwrap() * cone_frame;
        let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), major, minor).unwrap();
        let inside = |p: Point3| (p.x().hypot(p.y()) - major).hypot(p.z()) < minor;
        let dz = h / f64::from(nz);
        let dt = TAU / f64::from(nt);
        let mut volume = 0.0;
        for iz in 0..nz {
            let z = (f64::from(iz) + 0.5) * dz;
            let r = r0 + (r1 - r0) * z / h;
            let origin_local = Point3::new(0.0, 0.0, z);
            let origin = to_torus.mul_point(origin_local);
            for it in 0..nt {
                let theta = (f64::from(it) + 0.5) * dt;
                let dir = to_torus
                    .mul_point(origin_local + Vec3::new(theta.cos(), theta.sin(), 0.0))
                    - origin;
                let mut ts: Vec<f64> =
                    remus_math::analytic_intersection::intersect_line_torus(&torus, origin, dir)
                        .into_iter()
                        .filter(|&t| t > 0.0 && t < r)
                        .collect();
                ts.extend([0.0, r]);
                ts.sort_by(f64::total_cmp);
                for w in ts.windows(2) {
                    if inside(origin + dir * f64::midpoint(w[0], w[1])) {
                        volume += 0.5 * (w[1] * w[1] - w[0] * w[0]) * dt * dz;
                    }
                }
            }
        }
        volume
    }

    /// The boolean's volume from closed forms and the independent integral.
    fn expected_volume(self, op: BooleanOp) -> f64 {
        let inter = self.intersection_volume();
        let (stock, tool) = if self.cone_is_stock {
            (self.cone_volume(), self.torus_volume())
        } else {
            (self.torus_volume(), self.cone_volume())
        };
        match op {
            BooleanOp::Fuse => stock + tool - inter,
            BooleanOp::Cut => stock - inter,
            BooleanOp::Intersect => inter,
        }
    }
}

fn rel_err(a: f64, b: f64) -> f64 {
    (a - b).abs() / a.abs().max(b.abs())
}

fn diagonal(topo: &Topology, solid: SolidId) -> f64 {
    let aabb = solid_bounding_box(topo, solid).unwrap();
    (aabb.max - aabb.min).length()
}

/// Exact analytic faces only (the carriers of the inputs), a handful of them.
fn assert_analytic_census(topo: &Topology, solid: SolidId, what: &str) {
    let faces = explorer::solid_faces(topo, solid).unwrap();
    assert!(
        (2..=8).contains(&faces.len()),
        "{what}: {} faces is not an analytic result",
        faces.len()
    );
    let (mut torus, mut cone) = (0, 0);
    for fid in faces {
        match topo.face(fid).unwrap().surface() {
            FaceSurface::Torus(_) => torus += 1,
            FaceSurface::Cone(_) => cone += 1,
            FaceSurface::Plane { .. } => {}
            other => panic!("{what}: unexpected {} face", other.type_tag()),
        }
    }
    assert!(torus >= 1 && cone >= 1, "{what}: torus={torus} cone={cone}");
}

/// Both validators, strictly: the operations validator, and every
/// error-severity finding of the check crate. A disjoint union is one shell
/// holding `regions` edge-disconnected closed pieces, which the check
/// crate's by-edge-id connectivity and whole-shell Euler checks reject by
/// construction; for it those two are replaced by [`assert_edge_manifold`],
/// which counts the pieces itself.
fn assert_valid(topo: &Topology, solid: SolidId, regions: usize, what: &str) {
    use remus_check::validate::{CheckId, Severity, ValidateOptions};
    let ops = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        ops.is_valid(),
        "{what}: ops validator: {:?}",
        ops.issues
            .iter()
            .map(|i| &i.description)
            .collect::<Vec<_>>()
    );
    let report =
        remus_check::validate::validate_solid(topo, solid, &ValidateOptions::default()).unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == Severity::Error)
        .filter(|i| {
            regions == 1
                || !matches!(
                    i.check,
                    CheckId::ShellConnected | CheckId::SolidEulerCharacteristic
                )
        })
        .map(|i| (i.check, i.description.clone()))
        .collect();
    assert!(errors.is_empty(), "{what}: check crate: {errors:?}");
    assert_edge_manifold(topo, solid, regions, what);
}

/// Union-find root with path halving.
fn root(piece: &mut [usize], mut i: usize) -> usize {
    while piece[i] != i {
        piece[i] = piece[piece[i]];
        i = piece[i];
    }
    i
}

/// Every edge is used exactly twice across the faces' wires, once in each
/// direction, and the faces fall into exactly `regions` edge-connected
/// pieces. Counted here from the wires, not asked of a validator.
fn assert_edge_manifold(topo: &Topology, solid: SolidId, regions: usize, what: &str) {
    use std::collections::BTreeMap;
    let faces = explorer::solid_faces(topo, solid).unwrap();
    let mut uses: BTreeMap<usize, Vec<(usize, bool)>> = BTreeMap::new();
    for (fi, &fid) in faces.iter().enumerate() {
        let face = topo.face(fid).unwrap();
        for wire in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oriented in topo.wire(wire).unwrap().edges() {
                uses.entry(oriented.edge().index())
                    .or_default()
                    .push((fi, oriented.is_forward() != face.is_reversed()));
            }
        }
    }
    let mut piece: Vec<usize> = (0..faces.len()).collect();
    for (edge, used) in &uses {
        assert_eq!(
            used.len(),
            2,
            "{what}: edge {edge} used {} times",
            used.len()
        );
        assert_ne!(
            used[0].1, used[1].1,
            "{what}: edge {edge} traversed the same way twice"
        );
        let (a, b) = (root(&mut piece, used[0].0), root(&mut piece, used[1].0));
        piece[a] = b;
    }
    let pieces = (0..faces.len())
        .filter(|&i| root(&mut piece, i) == i)
        .count();
    assert_eq!(pieces, regions, "{what}: edge-connected pieces");
}

/// The preview deflections, the one that first opened the pierce (1e-3 of a
/// unit model) and the B26 proptest's scale-derived deflection.
fn deflections(cell: Cell, topo: &Topology, solid: SolidId) -> [f64; 4] {
    let s = cell.scale;
    [0.1 * s, 0.01 * s, 1e-3 * s, diagonal(topo, solid) * 1e-5]
}

/// Signed volume of a closed mesh, summed about one of its own vertices so a
/// body far from the origin loses nothing to cancellation.
fn mesh_volume(mesh: &TriangleMesh) -> f64 {
    let anchor = mesh.positions[mesh.indices[0] as usize];
    mesh.indices
        .chunks(3)
        .map(|t| {
            let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize] - anchor);
            a.dot(b.cross(c)) / 6.0
        })
        .sum()
}

/// Closed by index (every half-edge has its twin), closed by position after
/// welding, and enclosing the independent volume: an inscribed mesh may
/// undercount by its chord slab, never by a missing or doubled sheet.
fn assert_closed_mesh(cell: Cell, op: BooleanOp, topo: &Topology, solid: SolidId, what: &str) {
    let expected = cell.expected_volume(op);
    for d in deflections(cell, topo, solid) {
        let mesh = tessellate_solid(topo, solid, d).unwrap();
        let (b, n) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
        assert!(
            b == 0 && n == 0,
            "{what}: mesh at deflection {d:e} has {b} boundary and {n} non-manifold edges"
        );
        let welded = welded_mesh_quality(&mesh);
        assert_eq!(
            (welded.boundary_edges, welded.non_manifold_edges),
            (0, 0),
            "{what}: position-welded mesh at deflection {d:e} is not closed"
        );
        let volume = mesh_volume(&mesh);
        // A chord of sag d over a tube of radius r loses about d/r of the
        // cross-section; 6 d / minor covers every curved wall here.
        let slab = 6.0 * d / cell.torus_dims().1;
        assert!(
            rel_err(volume, expected) <= slab.max(5e-4),
            "{what}: mesh volume {volume:.6e} at deflection {d:e} vs independent oracle \
             {expected:.6e}"
        );
    }
}

/// Ray-parity containment against a watertight mesh: majority of three rays.
fn mesh_contains(mesh: &TriangleMesh, p: Point3) -> bool {
    let rays = [
        Vec3::new(0.3713, 0.5471, 0.7503),
        Vec3::new(-0.6207, 0.2201, 0.7525),
        Vec3::new(0.1307, -0.8813, 0.4541),
    ];
    let odd = rays
        .iter()
        .filter(|&&dir| {
            mesh.indices
                .chunks(3)
                .filter(|t| {
                    remus_math::ray_triangle::watertight_ray_triangle_intersect(
                        p,
                        dir,
                        mesh.positions[t[0] as usize],
                        mesh.positions[t[1] as usize],
                        mesh.positions[t[2] as usize],
                    )
                    .is_some_and(|hit| hit.t > 0.0)
                })
                .count()
                % 2
                == 1
        })
        .count();
    odd >= 2
}

/// Material accounting: on a grid over both operands' box, points clear of
/// both operand surfaces (by more than the mesh deflection) must sit on the
/// side analytic membership predicts. Returns how many probes fell in each
/// membership class `[neither, torus only, cone only, both]`.
fn assert_material(cell: Cell, op: BooleanOp, topo: &Topology, solid: SolidId) -> [usize; 4] {
    let (cone_frame, torus_frame) = cell.frames();
    let cone_inverse = cone_frame.inverse().unwrap();
    let torus_inverse = torus_frame.inverse().unwrap();
    let s = cell.scale;
    let mesh = tessellate_solid(topo, solid, 0.01 * s).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    // Both operands' box, from their own analytic extents.
    let (topo_ops, stock, tool) = cell.operands();
    let (sb, tb) = (
        solid_bounding_box(&topo_ops, stock).unwrap(),
        solid_bounding_box(&topo_ops, tool).unwrap(),
    );
    let axis = |p: Point3, k: usize| [p.x(), p.y(), p.z()][k];
    let lo = [0, 1, 2].map(|k| axis(sb.min, k).min(axis(tb.min, k)) - 0.1 * s);
    let hi = [0, 1, 2].map(|k| axis(sb.max, k).max(axis(tb.max, k)) + 0.1 * s);
    let n = 12_u32;
    let mut classes = [0usize; 4];
    let mut wrong = Vec::new();
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let at = |axis: usize, step: u32| {
                    lo[axis] + (hi[axis] - lo[axis]) * (f64::from(step) + 0.5) / f64::from(n)
                };
                let p = Point3::new(at(0, i), at(1, j), at(2, k));
                let tube = cell.tube_depth(&torus_inverse, p);
                let frustum = cell.frustum_depth(&cone_inverse, p);
                if tube.abs() < 0.03 * s || frustum.abs() < 0.03 * s {
                    continue;
                }
                let (in_t, in_f) = (tube > 0.0, frustum > 0.0);
                let expect = match op {
                    BooleanOp::Fuse => in_t || in_f,
                    BooleanOp::Cut if cell.cone_is_stock => in_f && !in_t,
                    BooleanOp::Cut => in_t && !in_f,
                    BooleanOp::Intersect => in_t && in_f,
                };
                classes[usize::from(in_t) + 2 * usize::from(in_f)] += 1;
                if mesh_contains(&mesh, p) != expect {
                    wrong.push(p);
                }
            }
        }
    }
    assert!(
        wrong.is_empty(),
        "{cell:?} {op:?}: {} probes on the wrong side, e.g. {:?}",
        wrong.len(),
        &wrong[..wrong.len().min(3)]
    );
    classes
}

fn check_leg(cell: Cell, op: BooleanOp, regions: usize) -> [usize; 4] {
    let what = format!("{cell:?} {op:?}");
    let (topo, solid) = cell.exact(op);
    assert_analytic_census(&topo, solid, &what);
    assert_valid(&topo, solid, regions, &what);
    assert_closed_mesh(cell, op, &topo, solid, &what);

    // Volume against the independent oracle. `solid_volume` reads an
    // inscribed mesh (the clamp puts it near diag·5e-5), so it may undercount
    // by a few 1e-4 relative.
    let deflection = 0.1 * cell.scale;
    let measured = solid_volume(&topo, solid, deflection).unwrap();
    let expected = cell.expected_volume(op);
    assert!(
        rel_err(measured, expected) <= 5e-4,
        "{what}: volume {measured:.6e} vs independent oracle {expected:.6e}"
    );

    // Translation invariance (the doubled-boundary detector).
    let mut moved = topo.clone();
    let shift = Mat4::translation(13.0 * cell.scale, -7.0 * cell.scale, 5.0 * cell.scale);
    transform_solid(&mut moved, solid, &shift).unwrap();
    let shifted = solid_volume(&moved, solid, deflection).unwrap();
    assert!(
        rel_err(measured, shifted) <= 1e-5,
        "{what}: volume moved {measured:.9e} -> {shifted:.9e} under translation"
    );

    assert_material(cell, op, &topo, solid)
}

/// A pierce cell's probes must reach every membership class, or the material
/// oracle would pass without ever looking inside the overlap.
fn assert_all_classes_probed(classes: [usize; 4], what: &str) {
    assert!(
        classes.iter().all(|&count| count >= 3),
        "{what}: probe classes [neither, torus, cone, both] = {classes:?}"
    );
}

#[test]
fn oblique_fuse() {
    assert_all_classes_probed(check_leg(OBLIQUE, BooleanOp::Fuse, 1), "oblique fuse");
}

#[test]
fn oblique_cut() {
    assert_all_classes_probed(check_leg(OBLIQUE, BooleanOp::Cut, 1), "oblique cut");
}

#[test]
fn oblique_intersect() {
    assert_all_classes_probed(
        check_leg(OBLIQUE, BooleanOp::Intersect, 1),
        "oblique intersect",
    );
}

#[test]
fn axis_aligned_fuse() {
    assert_all_classes_probed(
        check_leg(AXIS_ALIGNED, BooleanOp::Fuse, 1),
        "axis-aligned fuse",
    );
}

#[test]
fn axis_aligned_cut() {
    assert_all_classes_probed(
        check_leg(AXIS_ALIGNED, BooleanOp::Cut, 1),
        "axis-aligned cut",
    );
}

#[test]
fn axis_aligned_intersect() {
    assert_all_classes_probed(
        check_leg(AXIS_ALIGNED, BooleanOp::Intersect, 1),
        "axis-aligned intersect",
    );
}

/// The pierce keeps the frustum wall whole with one hole per tube passage,
/// and the torus as one band between the two pierce loops — and each pierce
/// loop is ONE closed edge shared by exactly the wall and the band, which is
/// the polyline the seam pre-split has to hand to both.
#[test]
fn pierce_fuse_shares_two_closed_loops_between_wall_and_band() {
    for cell in [OBLIQUE, AXIS_ALIGNED] {
        let (topo, solid) = cell.exact(BooleanOp::Fuse);
        let faces = explorer::solid_faces(&topo, solid).unwrap();
        let of_kind = |torus: bool| -> Vec<_> {
            faces
                .iter()
                .copied()
                .filter(|&f| match topo.face(f).unwrap().surface() {
                    FaceSurface::Torus(_) => torus,
                    FaceSurface::Cone(_) => !torus,
                    _ => false,
                })
                .collect()
        };
        let (walls, bands) = (of_kind(false), of_kind(true));
        assert_eq!((walls.len(), bands.len()), (1, 1), "{cell:?}");
        let wall = topo.face(walls[0]).unwrap();
        assert_eq!(wall.inner_wires().len(), 2, "{cell:?}: two pierce holes");
        let band_edges = explorer::face_edges(&topo, bands[0]).unwrap();
        for &hole in wall.inner_wires() {
            let edges = topo.wire(hole).unwrap().edges();
            assert_eq!(edges.len(), 1, "{cell:?}: a pierce loop is one edge");
            let edge = edges[0].edge();
            assert!(topo.edge(edge).unwrap().is_closed(), "{cell:?}");
            assert!(
                band_edges.contains(&edge),
                "{cell:?}: loop shared with band"
            );
        }
    }
}

/// The disjoint draw: both bodies survive the fuse, whole.
#[test]
fn disjoint_fuse_keeps_the_torus() {
    let cell = DISJOINT;
    // The frustum floats 2.25e3 above the tube, over the torus's hole.
    assert!(cell.intersection_volume() == 0.0);
    let classes = check_leg(cell, BooleanOp::Fuse, 2);
    assert!(
        classes[1] >= 3 && classes[2] >= 3 && classes[3] == 0,
        "probe classes [neither, torus, cone, both] = {classes:?}"
    );
    let (topo, solid) = cell.exact(BooleanOp::Fuse);
    let fused = solid_volume(&topo, solid, 0.1 * cell.scale).unwrap();
    assert!(
        rel_err(fused, cell.torus_volume() + cell.cone_volume()) <= 5e-4,
        "fuse {fused:.6e} vs torus {:.6e} + cone {:.6e}",
        cell.torus_volume(),
        cell.cone_volume()
    );
    // The silent-wrong answer was the cone alone: 18 % of the true union.
    assert!(fused > 5.0 * cell.cone_volume());
}

/// A disjoint cut returns the torus and a disjoint intersect is empty.
#[test]
fn disjoint_cut_and_intersect() {
    let cell = DISJOINT;
    let (topo, solid) = cell.exact(BooleanOp::Cut);
    let cut = solid_volume(&topo, solid, 0.1 * cell.scale).unwrap();
    assert!(rel_err(cut, cell.torus_volume()) <= 5e-4, "cut {cut:.6e}");
    let (topo, solid) = cell.exact(BooleanOp::Intersect);
    assert!(explorer::solid_faces(&topo, solid).unwrap().is_empty());
}

/// The inclusion–exclusion oracle is only as good as its integral: halving
/// the step in both directions moves it by less than 2e-4 (measured 8.5e-5
/// oblique, 1.4e-4 axis-aligned), and the `h^1.5` tail beyond the resolution
/// used is `1 / (2^1.5 - 1)` of that step, about half: under 1e-4 of the
/// intersection, a fifth of what the volume assertions allow.
#[test]
fn independent_integral_is_converged() {
    for cell in [OBLIQUE, AXIS_ALIGNED] {
        let coarse = cell.intersection_volume_at(300, 600);
        let used = cell.intersection_volume();
        assert!(used > 1.0, "{cell:?}: the tube passes through the frustum");
        assert!(
            rel_err(coarse, used) <= 2e-4,
            "{cell:?}: integral {coarse:.9} -> {used:.9}"
        );
    }
}
