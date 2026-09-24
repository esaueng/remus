//! B39 (B26 finding 14): a torus tube pierced by a frustum through its WALL
//! and a CAP at once. The pierce boundary on the tube is then a COMPOSITE loop
//! chained from three per-pair sections — torus × frustum-wall arcs, which end
//! on the frustum's rim circles, and torus × cap-plane ovals, which the rims
//! cut — so every chain junction is one exact rim∩torus crossing, shared by a
//! wall arc, a cap arc and the rim edge between the two cap/wall faces.
//!
//! Two cells, both refused or misbuilt before the fix:
//!
//! - **unit** (`T(1.5, 2, 1)·Ry(3π/2)`): the tube bites the frustum from rim
//!   to rim. One non-wrapping composite loop (two wall arcs, two cap arcs)
//!   bounds a disc on the tube; the wall is cut into two sectors. GFA used to
//!   degenerate to a 1-face open shell (typed refusal).
//! - **oblique** (`T(2.5, 0, 0)·Rx(π/4)`): the tube runs through the frustum.
//!   A whole base-cap oval and a composite loop (one wall arc notching the top
//!   rim, one top-cap arc) wrap the tube and bound a band; the wall is notched
//!   at one rim. The loops used to concatenate into one malformed wire.
//!
//! What had to hold, layer by layer: the marcher ends its wall arcs ON the
//! wall's rim band edge (not a step short); the cap ovals are trimmed at the
//! exact rim∩torus crossings; a wall arc that notches one rim is split so it
//! shares no endpoint pair with the rim span; the periodic wall splitter
//! emits both the notch lens and sector regions the greedy walker could not
//! trace; the plane×torus sampler no longer folds an oval back over its seam;
//! the torus notch-band mesher rules its rows between the loops.
//!
//! Every oracle here is independent of the kernel's own volume and point
//! classification: closed-form operand volumes, a semi-analytic torus ∩
//! frustum integral (inclusion–exclusion), and ray parity against the result's
//! watertight mesh at sample points on a grid (analytic membership decides the
//! expected side). The check crate's ray-cast classifier is NOT used: it
//! misreads trimmed torus bands in general (the already-landed B45 band
//! fixture disagrees with it at ~14 % of sample points).
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use remus_math::context::{FallbackPolicy, OperationContext};
use remus_math::mat::Mat4;
use remus_math::surfaces::ToroidalSurface;
use remus_math::vec::{Point3, Vec3};
use remus_operations::boolean::{BooleanOp, BooleanQuality, boolean_with_context};
use remus_operations::measure::{solid_bounding_box, solid_volume};
use remus_operations::primitives::{make_cone, make_torus};
use remus_operations::tessellate::{
    TriangleMesh, boundary_edge_count, non_manifold_edge_count, tessellate_solid,
    tessellate_solid_grouped_with_tolerance,
};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const MAJOR: f64 = 3.0;
const MINOR: f64 = 0.5;
const R_BASE: f64 = 1.5;
const R_TOP: f64 = 1.0;
const HEIGHT: f64 = 1.0;

#[derive(Clone, Copy, Debug)]
enum Cell {
    Unit,
    Oblique,
}

impl Cell {
    fn placement(self) -> Mat4 {
        match self {
            Self::Unit => Mat4::translation(1.5, 2.0, 1.0) * Mat4::rotation_y(3.0 * FRAC_PI_2),
            Self::Oblique => Mat4::translation(2.5, 0.0, 0.0) * Mat4::rotation_x(FRAC_PI_4),
        }
    }
}

fn operands(cell: Cell) -> (Topology, SolidId, SolidId) {
    let mut topo = Topology::new();
    let torus = make_torus(&mut topo, MAJOR, MINOR, 16).unwrap();
    let cone = make_cone(&mut topo, R_BASE, R_TOP, HEIGHT).unwrap();
    transform_solid(&mut topo, cone, &cell.placement()).unwrap();
    (topo, torus, cone)
}

fn exact(cell: Cell, op: BooleanOp) -> (Topology, SolidId) {
    let (mut topo, torus, cone) = operands(cell);
    let outcome = boolean_with_context(
        &mut topo,
        op,
        torus,
        cone,
        &OperationContext::new().with_fallback(FallbackPolicy::ExactOnly),
    )
    .unwrap_or_else(|e| panic!("{cell:?} {op:?}: exact boolean refused: {e:?}"));
    assert!(
        matches!(outcome.quality, BooleanQuality::Exact),
        "{cell:?} {op:?}: ExactOnly returned {:?}",
        outcome.quality
    );
    (topo, outcome.solid)
}

fn torus_volume() -> f64 {
    2.0 * PI * PI * MAJOR * MINOR * MINOR
}

fn frustum_volume() -> f64 {
    PI * HEIGHT * (R_BASE * R_BASE + R_BASE * R_TOP + R_TOP * R_TOP) / 3.0
}

/// Signed torus implicit: negative inside the tube.
fn torus_implicit(p: Point3) -> f64 {
    let radial = p.x().hypot(p.y()) - MAJOR;
    radial.mul_add(radial, p.z() * p.z()) - MINOR * MINOR
}

/// Positive inside the frustum (its local frame: axis +z, base at z = 0).
fn frustum_depth(inverse: &Mat4, p: Point3) -> f64 {
    let q = inverse.mul_point(p);
    let r = R_BASE + (R_TOP - R_BASE) * q.z() / HEIGHT;
    q.z().min(HEIGHT - q.z()).min(r - q.x().hypot(q.y()))
}

/// Torus ∩ frustum by the midpoint rule over the frustum's local `(z, θ)`,
/// with EXACT radial intervals: along each ray from the axis the tube's
/// boundary is the line–torus quartic, so `∫ρ dρ` over the inside intervals
/// is closed form. Converges to ~1e-6 at this resolution (probed at
/// 200×400 / 400×800 / 800×1600).
fn intersection_volume(cell: Cell) -> f64 {
    let m = cell.placement();
    let torus = ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), MAJOR, MINOR).unwrap();
    let (nz, nt) = (400_u32, 800_u32);
    let dz = HEIGHT / f64::from(nz);
    let dt = TAU / f64::from(nt);
    let mut volume = 0.0;
    for iz in 0..nz {
        let z = (f64::from(iz) + 0.5) * dz;
        let r = R_BASE + (R_TOP - R_BASE) * z / HEIGHT;
        let origin_local = Point3::new(0.0, 0.0, z);
        let origin = m.mul_point(origin_local);
        for it in 0..nt {
            let theta = (f64::from(it) + 0.5) * dt;
            let dir = m.mul_point(origin_local + Vec3::new(theta.cos(), theta.sin(), 0.0)) - origin;
            let mut ts: Vec<f64> =
                remus_math::analytic_intersection::intersect_line_torus(&torus, origin, dir)
                    .into_iter()
                    .filter(|&t| t > 0.0 && t < r)
                    .collect();
            ts.extend([0.0, r]);
            ts.sort_by(f64::total_cmp);
            for w in ts.windows(2) {
                if torus_implicit(origin + dir * f64::midpoint(w[0], w[1])) < 0.0 {
                    volume += 0.5 * (w[1] * w[1] - w[0] * w[0]) * dt * dz;
                }
            }
        }
    }
    volume
}

fn expected_volume(cell: Cell, op: BooleanOp) -> f64 {
    let inter = intersection_volume(cell);
    match op {
        BooleanOp::Fuse => torus_volume() + frustum_volume() - inter,
        BooleanOp::Cut => torus_volume() - inter,
        BooleanOp::Intersect => inter,
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
/// error-severity finding of the check crate.
fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let ops = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        ops.is_valid(),
        "{what}: ops validator: {:?}",
        ops.issues
            .iter()
            .map(|i| &i.description)
            .collect::<Vec<_>>()
    );
    let report = remus_check::validate::validate_solid(
        topo,
        solid,
        &remus_check::validate::ValidateOptions::default(),
    )
    .unwrap();
    let errors: Vec<_> = report
        .issues
        .iter()
        .filter(|i| i.severity == remus_check::validate::Severity::Error)
        .map(|i| (i.check, i.description.clone()))
        .collect();
    assert!(errors.is_empty(), "{what}: check crate: {errors:?}");
}

/// Watertight at the preview deflections, the fuzz harness deflection and
/// the B26 proptest's scale-derived deflection.
fn assert_watertight(topo: &Topology, solid: SolidId, what: &str) {
    let diag = diagonal(topo, solid);
    for d in [0.1, 0.01, diag * 4e-5 * 4.0, diag * 1e-5] {
        let mesh = tessellate_solid(topo, solid, d).unwrap();
        let (b, n) = (boundary_edge_count(&mesh), non_manifold_edge_count(&mesh));
        assert!(
            b == 0 && n == 0,
            "{what}: mesh at deflection {d:e} has {b} boundary and {n} non-manifold edges"
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

/// Material accounting: on a grid over the frustum's box, points clear of
/// both operand surfaces (by more than the mesh deflection) must sit on the
/// side analytic membership predicts.
fn assert_material(cell: Cell, op: BooleanOp, topo: &Topology, solid: SolidId, cone: SolidId) {
    let inverse = cell.placement().inverse().unwrap();
    let mesh = tessellate_solid(topo, solid, 0.01).unwrap();
    assert_eq!(boundary_edge_count(&mesh), 0);
    let bb = solid_bounding_box(topo, cone).unwrap();
    let n = 10_u32;
    let (mut probed, mut wrong) = (0, Vec::new());
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let at =
                    |lo: f64, hi: f64, s: u32| lo + (hi - lo) * (f64::from(s) + 0.5) / f64::from(n);
                let p = Point3::new(
                    at(bb.min.x(), bb.max.x(), i),
                    at(bb.min.y(), bb.max.y(), j),
                    at(bb.min.z(), bb.max.z(), k),
                );
                // Implicit value ≈ 2·r·distance for the tube.
                let tube = -torus_implicit(p) / (2.0 * MINOR);
                let frustum = frustum_depth(&inverse, p);
                if tube.abs() < 0.03 || frustum.abs() < 0.03 {
                    continue;
                }
                let (in_t, in_f) = (tube > 0.0, frustum > 0.0);
                let expect = match op {
                    BooleanOp::Fuse => in_t || in_f,
                    BooleanOp::Cut => in_t && !in_f,
                    BooleanOp::Intersect => in_t && in_f,
                };
                probed += 1;
                if mesh_contains(&mesh, p) != expect {
                    wrong.push(p);
                }
            }
        }
    }
    assert!(probed > 300, "{cell:?} {op:?}: only {probed} probes");
    assert!(
        wrong.is_empty(),
        "{cell:?} {op:?}: {} of {probed} probes on the wrong side, e.g. {:?}",
        wrong.len(),
        &wrong[..wrong.len().min(3)]
    );
}

fn check_leg(cell: Cell, op: BooleanOp) {
    let what = format!("{cell:?} {op:?}");
    let (topo, solid) = exact(cell, op);
    assert_analytic_census(&topo, solid, &what);
    assert_valid(&topo, solid, &what);
    assert_watertight(&topo, solid, &what);

    // Volume against the independent oracle. `solid_volume` reads an
    // inscribed mesh (the clamp puts it at ~5e-4 here), so it may undercount
    // by a few 1e-4 relative — never by the ~1 % the unruled band mesh lost.
    let measured = solid_volume(&topo, solid, 0.1).unwrap();
    let expected = expected_volume(cell, op);
    assert!(
        rel_err(measured, expected) <= 5e-4,
        "{what}: volume {measured:.6} vs independent oracle {expected:.6}"
    );

    // Translation invariance (the doubled-boundary detector).
    let mut moved = topo.clone();
    transform_solid(&mut moved, solid, &Mat4::translation(13.0, -7.0, 5.0)).unwrap();
    let shifted = solid_volume(&moved, solid, 0.1).unwrap();
    assert!(
        rel_err(measured, shifted) <= 1e-5,
        "{what}: volume moved {measured:.9} -> {shifted:.9} under translation"
    );

    let (_, _, cone) = operands(cell);
    assert_material(cell, op, &topo, solid, cone);
}

#[test]
fn unit_cell_fuse() {
    check_leg(Cell::Unit, BooleanOp::Fuse);
}

#[test]
fn unit_cell_cut() {
    check_leg(Cell::Unit, BooleanOp::Cut);
}

#[test]
fn unit_cell_intersect() {
    check_leg(Cell::Unit, BooleanOp::Intersect);
}

#[test]
fn oblique_cell_fuse() {
    check_leg(Cell::Oblique, BooleanOp::Fuse);
}

#[test]
fn oblique_cell_cut() {
    check_leg(Cell::Oblique, BooleanOp::Cut);
}

#[test]
fn oblique_cell_intersect() {
    check_leg(Cell::Oblique, BooleanOp::Intersect);
}

/// The oblique pierce keeps the torus as ONE band between two tube-wrapping
/// loops: the whole base-cap oval, and the composite loop (wall arc, split at
/// its midpoint, plus the top-cap arc, split at its midpoint).
#[test]
fn oblique_fuse_keeps_one_band_between_the_pierce_loops() {
    let (topo, solid) = exact(Cell::Oblique, BooleanOp::Fuse);
    let tori: Vec<_> = explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .collect();
    assert_eq!(tori.len(), 1, "one torus band");
    let band = topo.face(tori[0]).unwrap();
    assert_eq!(band.inner_wires().len(), 1, "band: one inner loop");
    let outer = topo.wire(band.outer_wire()).unwrap().edges().len();
    let inner = topo.wire(band.inner_wires()[0]).unwrap().edges().len();
    let mut loops = [outer, inner];
    loops.sort_unstable();
    assert_eq!(
        loops,
        [1, 4],
        "base oval + composite loop (2 wall + 2 cap arcs)"
    );
}

/// The unit bite cuts ONE disc out of the full torus: its composite loop
/// chains two wall arcs and two cap arcs (each cap arc split at its midpoint)
/// through the four exact rim∩torus crossings.
#[test]
fn unit_fuse_cuts_one_composite_disc_from_the_torus() {
    let (topo, solid) = exact(Cell::Unit, BooleanOp::Fuse);
    let tori: Vec<_> = explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Torus(_)))
        .collect();
    assert_eq!(tori.len(), 1);
    let face = topo.face(tori[0]).unwrap();
    assert_eq!(face.inner_wires().len(), 1, "one pierce hole");
    let hole = topo.wire(face.inner_wires()[0]).unwrap();
    assert_eq!(hole.edges().len(), 6, "2 wall arcs + 2×2 cap half-arcs");
}

/// Torus band triangles honour the requested deflection. Rows placed at
/// constant ring angle, started past a wandering loop's u-spread, left
/// triangles ~3e-2 off the tube at a 5e-4 request.
#[test]
fn oblique_band_mesh_stays_on_the_torus() {
    let (topo, solid) = exact(Cell::Oblique, BooleanOp::Fuse);
    assert_valid(&topo, solid, "oblique fuse");
    let deflection = 5e-4;
    let (mesh, offsets) = tessellate_solid_grouped_with_tolerance(
        &topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .unwrap();
    let faces = explorer::solid_faces(&topo, solid).unwrap();
    for (i, &fid) in faces.iter().enumerate() {
        let FaceSurface::Torus(torus) = topo.face(fid).unwrap().surface() else {
            continue;
        };
        let (lo, hi) = (offsets[i] as usize, offsets[i + 1] as usize);
        let mut worst = 0.0_f64;
        for t in mesh.indices[lo..hi].chunks(3) {
            let [a, b, c] = [0, 1, 2].map(|k| mesh.positions[t[k] as usize]);
            let centroid = Point3::new(
                (a.x() + b.x() + c.x()) / 3.0,
                (a.y() + b.y() + c.y()) / 3.0,
                (a.z() + b.z() + c.z()) / 3.0,
            );
            let (u, v) = torus.project_point(centroid);
            worst = worst.max((torus.evaluate(u, v) - centroid).length());
        }
        assert!(
            worst <= 2.0 * deflection,
            "torus band triangle {worst:.2e} off the surface at deflection {deflection:e}"
        );
    }
}
