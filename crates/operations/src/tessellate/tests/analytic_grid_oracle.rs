//! Bit-identity oracle for the trig-table analytic grid.
//!
//! `tessellate_analytic_per_vertex` is `tessellate_analytic` as it stood
//! before the per-column/per-row trigonometry tables, kept verbatim: it calls
//! a point closure and a normal closure once per vertex. Each case meshes the
//! same surface patch both ways and compares positions, normals and UVs bit
//! for bit and indices exactly, for spheres (full, polar caps, latitude bands,
//! longitude wedges), tori, cylinders and cones (apex fans and frusta) over
//! several frames, ranges and resolutions, including the clamped minimums.
//!
//! The per-vertex closures are fed twice: once with the surfaces' public
//! `evaluate`/`normal` (the call shape the face mesher used) and once with
//! the pre-change formulas written out over the public getters, so the
//! oracle does not move if those methods' shared helpers change.

#![allow(clippy::expect_used, clippy::panic)]

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use remus_math::fma::FusedMulAdd;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point3, Vec3};

use super::super::planar::{AnalyticGridSurface, tessellate_analytic};
use super::super::{AnalyticKind, TriangleMesh, TriangleMeshUV};

/// `tessellate_analytic` before the trig tables, verbatim.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_lines
)]
fn tessellate_analytic_per_vertex(
    surface_eval: impl Fn(f64, f64) -> Point3,
    normal_fn: impl Fn(f64, f64) -> Vec3,
    u_range: (f64, f64),
    v_range: (f64, f64),
    nu: usize,
    nv: usize,
    kind: AnalyticKind,
) -> Result<TriangleMeshUV, crate::OperationsError> {
    let nu = nu.max(4);
    let nv = nv.max(1);

    // Every analytic face type funnels through here, and `nu`/`nv` arrive
    // straight from the chord-deviation helper with nothing between them and a
    // caller-supplied deflection — so a small enough deflection asks for an
    // arbitrarily large grid and the allocations below never return. The CDT
    // interior path has carried this budget for a while; the analytic grid it
    // sits beside had none, which is the larger hole of the two since it covers
    // cylinder, cone, sphere and torus alike.
    super::super::nonplanar::validate_interior_grid_size(nu + 1, nv + 1)?;

    let num_verts = (nu + 1) * (nv + 1);
    let num_indices = nu * nv * 6;
    let mut positions = Vec::with_capacity(num_verts);
    let mut normals = Vec::with_capacity(num_verts);
    let mut uvs = Vec::with_capacity(num_verts);
    let mut indices = Vec::with_capacity(num_indices);

    // Only wrap u when the range spans a full period (approx 2pi).
    // For partial arcs (e.g. quarter-cylinder), the last grid column is a
    // distinct point that must NOT wrap back to the first column.
    let u_periodic = (u_range.1 - u_range.0 - std::f64::consts::TAU).abs()
        < remus_math::tolerance::Tolerance::new().linear;

    let mut grid = vec![0u32; (nu + 1) * (nv + 1)];
    for iv in 0..=nv {
        let v = v_range.0 + (v_range.1 - v_range.0) * (iv as f64) / (nv as f64);
        for iu in 0..=nu {
            let u = u_range.0 + (u_range.1 - u_range.0) * (iu as f64) / (nu as f64);
            let idx = positions.len() as u32;
            positions.push(surface_eval(u, v));
            normals.push(normal_fn(u, v));
            uvs.push([u, v]);
            grid[iv * (nu + 1) + iu] = idx;
        }
    }

    // Get grid index, wrapping u only for periodic seams (full circles).
    let gi = |iu: usize, iv: usize| -> u32 {
        let iu_w = if u_periodic && iu >= nu { 0 } else { iu };
        grid[iv * (nu + 1) + iu_w]
    };

    let v_min_degenerate = matches!(kind, AnalyticKind::SpherePole | AnalyticKind::ConeApex);
    let v_max_degenerate = matches!(kind, AnalyticKind::SpherePole | AnalyticKind::VMaxPole);

    for iv in 0..nv {
        let is_bottom = iv == 0;
        let is_top = iv == nv - 1;

        for iu in 0..nu {
            let i00 = gi(iu, iv);
            let i10 = gi(iu + 1, iv);
            let i01 = gi(iu, iv + 1);
            let i11 = gi(iu + 1, iv + 1);

            if is_bottom && v_min_degenerate {
                // Bottom pole/apex: triangle fan from the degenerate row.
                indices.push(i00);
                indices.push(i11);
                indices.push(i01);
            } else if is_top && v_max_degenerate {
                // Top pole: triangle fan from the degenerate row.
                indices.push(i00);
                indices.push(i10);
                indices.push(i01);
            } else {
                // Standard two-triangle quad.
                indices.push(i00);
                indices.push(i10);
                indices.push(i11);

                indices.push(i00);
                indices.push(i11);
                indices.push(i01);
            }
        }
    }

    Ok(TriangleMeshUV {
        mesh: TriangleMesh {
            positions,
            normals,
            indices,
        },
        uvs,
    })
}

// ── Pre-change `evaluate`/`normal` bodies over the public getters ──────────

fn cylinder_point(s: &CylindricalSurface, u: f64, v: f64) -> Point3 {
    let (sin_u, cos_u) = u.sin_cos();
    s.origin()
        + s.x_axis() * (s.radius() * cos_u)
        + s.y_axis() * (s.radius() * sin_u)
        + s.axis() * v
}

fn cylinder_normal(s: &CylindricalSurface, u: f64, _v: f64) -> Vec3 {
    let (sin_u, cos_u) = u.sin_cos();
    s.x_axis() * cos_u + s.y_axis() * sin_u
}

fn cone_point(s: &ConicalSurface, u: f64, v: f64) -> Point3 {
    let (sin_u, cos_u) = u.sin_cos();
    let (sin_a, cos_a) = s.half_angle().sin_cos();
    let radial = s.x_axis() * cos_u + s.y_axis() * sin_u;
    s.apex() + (radial * cos_a + s.axis() * sin_a) * v
}

fn cone_normal(s: &ConicalSurface, u: f64, _v: f64) -> Vec3 {
    let (sin_u, cos_u) = u.sin_cos();
    let (sin_a, cos_a) = s.half_angle().sin_cos();
    let radial = s.x_axis() * cos_u + s.y_axis() * sin_u;
    radial * sin_a + s.axis() * (-cos_a)
}

fn sphere_point(s: &SphericalSurface, u: f64, v: f64) -> Point3 {
    let (sin_u, cos_u) = u.sin_cos();
    let (sin_v, cos_v) = v.sin_cos();
    s.center()
        + s.x_axis() * (s.radius() * cos_v * cos_u)
        + s.y_axis() * (s.radius() * cos_v * sin_u)
        + s.z_axis() * (s.radius() * sin_v)
}

fn sphere_normal(s: &SphericalSurface, u: f64, v: f64) -> Vec3 {
    let (sin_u, cos_u) = u.sin_cos();
    let (sin_v, cos_v) = v.sin_cos();
    s.x_axis() * (cos_v * cos_u) + s.y_axis() * (cos_v * sin_u) + s.z_axis() * sin_v
}

fn torus_point(s: &ToroidalSurface, u: f64, v: f64) -> Point3 {
    let (sin_u, cos_u) = u.sin_cos();
    let (sin_v, cos_v) = v.sin_cos();
    let tube_radius = s.minor_radius().fma(cos_v, s.major_radius());
    let height = s.minor_radius() * sin_v;
    s.center()
        + s.x_axis() * (tube_radius * cos_u)
        + s.y_axis() * (tube_radius * sin_u)
        + s.z_axis() * height
}

fn torus_normal(s: &ToroidalSurface, u: f64, v: f64) -> Vec3 {
    let (sin_u, cos_u) = u.sin_cos();
    let (sin_v, cos_v) = v.sin_cos();
    let radial = s.x_axis() * cos_u + s.y_axis() * sin_u;
    radial * cos_v + s.z_axis() * sin_v
}

// ── Comparison ─────────────────────────────────────────────────────────────

/// A parameter interval `(start, end)`.
type Span = (f64, f64);

/// A patch request: `u` span, `v` span and [`kind_of`] tag.
type Patch = (Span, Span, u8);

/// One grid request.
#[derive(Clone, Copy, Debug)]
struct Grid {
    u_range: (f64, f64),
    v_range: (f64, f64),
    nu: usize,
    nv: usize,
}

fn kind_of(tag: u8) -> AnalyticKind {
    match tag {
        0 => AnalyticKind::General,
        1 => AnalyticKind::SpherePole,
        2 => AnalyticKind::ConeApex,
        _ => AnalyticKind::VMaxPole,
    }
}

fn assert_bit_identical(got: &TriangleMeshUV, want: &TriangleMeshUV, what: &str) {
    let (g, w) = (&got.mesh, &want.mesh);
    assert_eq!(g.positions.len(), w.positions.len(), "{what}: vertex count");
    assert_eq!(g.normals.len(), w.normals.len(), "{what}: normal count");
    assert_eq!(got.uvs.len(), want.uvs.len(), "{what}: uv count");
    assert_eq!(g.indices, w.indices, "{what}: indices");
    for (i, (a, b)) in g.positions.iter().zip(&w.positions).enumerate() {
        assert!(
            a.x().to_bits() == b.x().to_bits()
                && a.y().to_bits() == b.y().to_bits()
                && a.z().to_bits() == b.z().to_bits(),
            "{what}: position {i}: {a:?} != {b:?}"
        );
    }
    for (i, (a, b)) in g.normals.iter().zip(&w.normals).enumerate() {
        assert!(
            a.x().to_bits() == b.x().to_bits()
                && a.y().to_bits() == b.y().to_bits()
                && a.z().to_bits() == b.z().to_bits(),
            "{what}: normal {i}: {a:?} != {b:?}"
        );
    }
    for (i, (a, b)) in got.uvs.iter().zip(&want.uvs).enumerate() {
        assert!(
            a[0].to_bits() == b[0].to_bits() && a[1].to_bits() == b[1].to_bits(),
            "{what}: uv {i}: {a:?} != {b:?}"
        );
    }
}

/// Mesh `grid` with the table path and with the verbatim per-vertex path fed
/// both the public methods and the written-out formulas; all three must
/// agree bit for bit. Returns the vertex count so callers can check the
/// cases are not vacuous.
#[allow(clippy::too_many_arguments)]
fn check<S: AnalyticGridSurface>(
    surface: &S,
    method_point: impl Fn(f64, f64) -> Point3,
    method_normal: impl Fn(f64, f64) -> Vec3,
    formula_point: impl Fn(f64, f64) -> Point3,
    formula_normal: impl Fn(f64, f64) -> Vec3,
    grid: Grid,
    kind: u8,
    what: &str,
) -> usize {
    let what = format!("{what} {grid:?} kind {kind}");
    let Grid {
        u_range,
        v_range,
        nu,
        nv,
    } = grid;
    let table =
        tessellate_analytic(surface, u_range, v_range, nu, nv, kind_of(kind)).expect("table grid");
    let by_method = tessellate_analytic_per_vertex(
        method_point,
        method_normal,
        u_range,
        v_range,
        nu,
        nv,
        kind_of(kind),
    )
    .expect("per-vertex grid (methods)");
    let by_formula = tessellate_analytic_per_vertex(
        formula_point,
        formula_normal,
        u_range,
        v_range,
        nu,
        nv,
        kind_of(kind),
    )
    .expect("per-vertex grid (formulas)");
    assert_bit_identical(
        &by_method,
        &by_formula,
        &format!("{what} [methods vs formulas]"),
    );
    assert_bit_identical(&table, &by_formula, &what);
    table.mesh.positions.len()
}

fn resolutions() -> [(usize, usize); 6] {
    // (0, 0) and (3, 1) exercise the `max(4)` / `max(1)` clamps.
    [(0, 0), (3, 1), (4, 1), (17, 9), (64, 32), (181, 97)]
}

#[test]
fn sphere_grid_matches_per_vertex_bit_for_bit() {
    let spheres = [
        SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0).expect("sphere"),
        SphericalSurface::with_axis(
            Point3::new(1.5, -2.25, 7.0),
            3.7,
            Vec3::new(0.3, -0.4, 0.866),
        )
        .expect("sphere"),
        SphericalSurface::with_frame(
            Point3::new(-1.0e4, 3.3, 0.125),
            0.0125,
            Vec3::new(-1.0, 2.0, -0.5),
            Vec3::new(0.7, 0.1, 0.0),
        )
        .expect("sphere"),
    ];
    // (u range, v range, kind): full sphere, polar caps, latitude bands,
    // longitude wedges, a seam-straddling full turn, a negative band.
    let patches: [Patch; 8] = [
        ((0.0, TAU), (-FRAC_PI_2, FRAC_PI_2), 1),
        ((0.0, TAU), (0.0, FRAC_PI_2), 3),
        ((0.0, TAU), (-0.3, 0.9), 0),
        ((1.0, 1.0 + TAU), (-FRAC_PI_2, 0.25), 0),
        ((-PI, PI), (-1.2, -0.1), 0),
        ((0.25, 1.95), (-0.6, 1.1), 0),
        ((0.0, FRAC_PI_2), (0.0, FRAC_PI_2), 3),
        ((4.0, 5.5), (-FRAC_PI_2, FRAC_PI_2), 1),
    ];
    let mut vertices = 0;
    for sphere in &spheres {
        for &(u_range, v_range, kind) in &patches {
            for (nu, nv) in resolutions() {
                vertices += check(
                    sphere,
                    |u, v| sphere.evaluate(u, v),
                    |u, v| sphere.normal(u, v),
                    |u, v| sphere_point(sphere, u, v),
                    |u, v| sphere_normal(sphere, u, v),
                    Grid {
                        u_range,
                        v_range,
                        nu,
                        nv,
                    },
                    kind,
                    "sphere",
                );
            }
        }
    }
    assert!(vertices > 100_000, "sphere cases cover {vertices} vertices");
}

#[test]
fn torus_grid_matches_per_vertex_bit_for_bit() {
    let tori = [
        ToroidalSurface::new(Point3::new(0.0, 0.0, 0.0), 10.0, 3.0).expect("torus"),
        ToroidalSurface::with_axis(
            Point3::new(1.5, -2.25, 7.0),
            4.0,
            1.25,
            Vec3::new(0.3, -0.4, 0.866),
        )
        .expect("torus"),
        ToroidalSurface::with_axis_and_ref_dir(
            Point3::new(-1.0e4, 3.3, 0.125),
            2.0e3,
            1.0e-2,
            Vec3::new(-1.0, 2.0, -0.5),
            Vec3::new(0.7, 0.1, 0.0),
        )
        .expect("torus"),
    ];
    let patches: [(Span, Span); 5] = [
        ((0.0, TAU), (0.0, TAU)),
        ((0.0, TAU), (-PI, PI)),
        ((0.5, 2.0), (0.0, TAU)),
        ((0.0, TAU), (1.0, 2.5)),
        ((3.0, 3.0 + TAU), (-0.4, 5.9)),
    ];
    let mut vertices = 0;
    for torus in &tori {
        for &(u_range, v_range) in &patches {
            for (nu, nv) in resolutions() {
                vertices += check(
                    torus,
                    |u, v| torus.evaluate(u, v),
                    |u, v| torus.normal(u, v),
                    |u, v| torus_point(torus, u, v),
                    |u, v| torus_normal(torus, u, v),
                    Grid {
                        u_range,
                        v_range,
                        nu,
                        nv,
                    },
                    0,
                    "torus",
                );
            }
        }
    }
    assert!(vertices > 50_000, "torus cases cover {vertices} vertices");
}

#[test]
fn cylinder_grid_matches_per_vertex_bit_for_bit() {
    let cylinders = [
        CylindricalSurface::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 5.0)
            .expect("cylinder"),
        CylindricalSurface::with_ref_dir(
            Point3::new(1.5, -2.25, 7.0),
            Vec3::new(0.3, -0.4, 0.866),
            0.75,
            Vec3::new(0.0, 1.0, 0.2),
        )
        .expect("cylinder"),
        CylindricalSurface::new(
            Point3::new(-1.0e4, 3.3, 0.125),
            Vec3::new(-1.0, 2.0, -0.5),
            1.0e3,
        )
        .expect("cylinder"),
    ];
    let patches: [(Span, Span); 5] = [
        ((0.0, TAU), (0.0, 20.0)),
        ((0.0, TAU), (-5.0, 3.0)),
        ((0.0, FRAC_PI_2), (0.0, 1.0)),
        ((2.0, 2.0 + TAU), (-1.0e3, 1.0e3)),
        ((-PI, 0.5), (7.5, 7.75)),
    ];
    let mut vertices = 0;
    for cylinder in &cylinders {
        for &(u_range, v_range) in &patches {
            for (nu, nv) in resolutions() {
                vertices += check(
                    cylinder,
                    |u, v| cylinder.evaluate(u, v),
                    |u, v| cylinder.normal(u, v),
                    |u, v| cylinder_point(cylinder, u, v),
                    |u, v| cylinder_normal(cylinder, u, v),
                    Grid {
                        u_range,
                        v_range,
                        nu,
                        nv,
                    },
                    0,
                    "cylinder",
                );
            }
        }
    }
    assert!(
        vertices > 50_000,
        "cylinder cases cover {vertices} vertices"
    );
}

#[test]
fn cone_grid_matches_per_vertex_bit_for_bit() {
    let cones = [
        ConicalSurface::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            0.25 * PI,
        )
        .expect("cone"),
        ConicalSurface::with_ref_dir(
            Point3::new(1.5, -2.25, 7.0),
            Vec3::new(0.3, -0.4, 0.866),
            1.3,
            Vec3::new(0.0, 1.0, 0.2),
        )
        .expect("cone"),
        ConicalSurface::new(
            Point3::new(-1.0e4, 3.3, 0.125),
            Vec3::new(-1.0, 2.0, -0.5),
            0.1,
        )
        .expect("cone"),
    ];
    // Apex fans start at v = 0; frusta do not.
    let patches: [Patch; 5] = [
        ((0.0, TAU), (0.0, 10.0), 2),
        ((0.0, TAU), (2.0, 7.0), 0),
        ((0.0, FRAC_PI_2), (0.0, 1.0), 2),
        ((2.0, 2.0 + TAU), (-3.0, -0.5), 0),
        ((-PI, 0.5), (1.0e-3, 4.0e3), 0),
    ];
    let mut vertices = 0;
    for cone in &cones {
        for &(u_range, v_range, kind) in &patches {
            for (nu, nv) in resolutions() {
                vertices += check(
                    cone,
                    |u, v| cone.evaluate(u, v),
                    |u, v| cone.normal(u, v),
                    |u, v| cone_point(cone, u, v),
                    |u, v| cone_normal(cone, u, v),
                    Grid {
                        u_range,
                        v_range,
                        nu,
                        nv,
                    },
                    kind,
                    "cone",
                );
            }
        }
    }
    assert!(vertices > 50_000, "cone cases cover {vertices} vertices");
}

#[test]
fn oversized_grid_is_refused_by_both_paths() {
    let sphere = SphericalSurface::new(Point3::new(0.0, 0.0, 0.0), 1.0).expect("sphere");
    let (nu, nv) = (1usize << 20, 1usize << 20);
    let table = tessellate_analytic(
        &sphere,
        (0.0, TAU),
        (-FRAC_PI_2, FRAC_PI_2),
        nu,
        nv,
        AnalyticKind::SpherePole,
    );
    let per_vertex = tessellate_analytic_per_vertex(
        |u, v| sphere.evaluate(u, v),
        |u, v| sphere.normal(u, v),
        (0.0, TAU),
        (-FRAC_PI_2, FRAC_PI_2),
        nu,
        nv,
        AnalyticKind::SpherePole,
    );
    match (table, per_vertex) {
        (Err(a), Err(b)) => assert_eq!(a.to_string(), b.to_string()),
        _ => panic!("an oversized grid must be refused by both paths"),
    }
}
