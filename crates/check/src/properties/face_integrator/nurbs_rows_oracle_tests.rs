//! Bit-identity oracle for the NURBS row and grid quadrature.
//!
//! `integrate_parametric_reference` is [`integrate_parametric`] as it stood
//! before NURBS faces read their bases from per-axis tables
//! ([`NurbsRows`], [`integrate_nurbs_grid`]), and `add_reference` is
//! `Accumulator::add` from then, both verbatim: every abscissa runs the
//! per-point scratch solve. Every case compares the two contributions and
//! vector areas bit for bit, for area-only and full reads, full and
//! volume-only moments, Gauss orders 1 to 8, over rational and
//! non-rational surfaces of degrees 1 to 5 plus one above the u memo bound,
//! untrimmed, rectangle-trimmed (repeated v windows), polygon-trimmed with
//! pockets (windows that change per row and evict each other) and
//! seam-straddling periodic trims, on domains reaching outside the knot
//! range.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::suboptimal_flops,
    clippy::too_many_lines
)]

use super::*;

impl Accumulator {
    /// `add` before the per-axis tables, verbatim.
    fn add_reference<S: ParametricSurface + ?Sized, const AREA_ONLY: bool>(
        &mut self,
        surface: &S,
        u: f64,
        v: f64,
        w: f64,
        reference: Point3,
        scratch: &mut DerivativeScratch,
    ) {
        // One solve for position and both partials (a NURBS surface would
        // otherwise run a full evaluation plus the derivative solve per
        // Gauss point). The span-hinted path verifies the previous spans
        // before trusting them, so analytic surfaces (whose default ignores
        // the hint) and NURBS faces alike get exactly the unhinted answer.
        let (p, du, dv, _, _) = surface.span_hinted_point_and_partials_with_scratch(u, v, scratch);
        // Normal = du x dv (unnormalized, includes Jacobian)
        let n = Vec3::new(
            du.y() * dv.z() - du.z() * dv.y(),
            du.z() * dv.x() - du.x() * dv.z(),
            du.x() * dv.y() - du.y() * dv.x(),
        );
        let n_len = n.length();

        self.area += w * n_len;

        if AREA_ONLY {
            return;
        }
        let p = p - reference;

        // Volume: (1/3) P dot N (unnormalized N includes Jacobian)
        self.vol += w * p.dot(n) / 3.0;
        self.nx += w * n.x();
        self.ny += w * n.y();
        self.nz += w * n.z();

        // A strict orientation probe needs no centroid or inertia moments.
        // Keep area, volume and flux arithmetic above identical to the full
        // fixed rule so cold readings and reference/translation reuse agree.
        if self.volume_only {
            return;
        }

        // Volume moments via divergence theorem:
        // CoM_x = (1/2V) surface_integral(x^2 * n_x dA)
        // n already includes Jacobian, so n.x() = N_x * |J|
        self.mx += w * 0.5 * p.x() * p.x() * n.x();
        self.my += w * 0.5 * p.y() * p.y() * n.y();
        self.mz += w * 0.5 * p.z() * p.z() * n.z();

        self.qxx += w * p.x().powi(3) * n.x() / 3.0;
        self.qyy += w * p.y().powi(3) * n.y() / 3.0;
        self.qzz += w * p.z().powi(3) * n.z() / 3.0;
        self.qxy += w * 0.5 * p.x().powi(2) * p.y() * n.x();
        self.qxz += w * 0.5 * p.x().powi(2) * p.z() * n.x();
        self.qyz += w * 0.5 * p.y().powi(2) * p.z() * n.y();

        self.cx += w * p.x() * n_len;
        self.cy += w * p.y() * n_len;
        self.cz += w * p.z() * n_len;
    }
}

/// `integrate_parametric` before the per-axis tables, verbatim (with its
/// accumulator calls pointed at `add_reference`).
#[allow(clippy::cast_precision_loss)]
fn integrate_parametric_reference<S: ParametricSurface, const AREA_ONLY: bool>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    rule: IntegrationRule<'_>,
    sign: f64,
    trim: &UvTrim<'_>,
    scale: PatchScale,
) -> Result<FaceContribution, CheckError> {
    // Trimmed domains and NURBS carriers refine as sliced domains (an
    // untrimmed NURBS patch is one constant span per knot-aligned window);
    // untrimmed analytic rectangles refine by quadrants.
    let sliced = if trim.splits_domain() {
        rule.requested_refinement()
    } else if rule.knots.is_some() {
        rule.adaptive
    } else {
        None
    };
    if let Some(options) = sliced {
        let mut breaks = if trim.splits_domain() {
            trim.u_breaks(u_range)
        } else {
            vec![u_range.0, u_range.1]
        };
        let (knots_u, knots_v) = rule.knots.unwrap_or((&[], &[]));
        breaks.extend(interior_knots(knots_u, u_range));
        breaks.compact_sort_by(f64::total_cmp);
        breaks.dedup();
        let inner_knots = interior_knots(knots_v, v_range);
        // With cells aligned to the knots, one Gauss rule per knot span is the
        // natural starting cell; refinement supplies any further resolution.
        // The fixed rule's sub-span tiling exists only because it cannot align.
        #[allow(clippy::cast_precision_loss)]
        let start = if rule.knots.is_some() {
            PatchScale {
                u: scale.u * PATCHES_PER_KNOT_SPAN as f64,
                v: scale.v * PATCHES_PER_KNOT_SPAN as f64,
            }
        } else {
            scale
        };
        let spans = |u: f64| {
            if trim.splits_domain() {
                trim.v_spans(u, v_range)
            } else {
                vec![v_range]
            }
        };
        return adaptive::integrate_sliced(
            surface,
            &adaptive::Sliced {
                breaks: &breaks,
                inner_knots: &inner_knots,
                outer_scale: start.u,
                inner_scale: start.v,
                spans: &spans,
                outer_is_v: false,
            },
            sign,
            options,
            rule.reference,
            rule.cone_apex,
        );
    }
    if !trim.splits_domain()
        && let Some(options) = rule.adaptive
    {
        return adaptive::integrate(
            surface,
            (u_range, v_range),
            sign,
            scale,
            options,
            rule.reference,
            rule.cone_apex,
        );
    }
    let gauss_pts = gauss_legendre_points(rule.order);
    let nu = patch_count(u_range.1 - u_range.0, scale.u);
    let du_patch = (u_range.1 - u_range.0) / nu as f64;
    let u_scale = du_patch / 2.0;
    let mut acc = Accumulator {
        volume_only: rule.volume_only,
        ..Accumulator::default()
    };
    // One scratch per face integration: every NURBS abscissa below reuses
    // these buffers instead of allocating a derivative table each time.
    // Integration is single-threaded within a face, so one scratch suffices.
    let mut scratch = DerivativeScratch::new();

    if trim.splits_domain() {
        let breaks = trim.u_breaks(u_range);
        let eps = (u_range.1 - u_range.0).abs() * 1e-12;
        // Span buffers shared by every abscissa below.
        let (mut cuts, mut spans) = (Vec::new(), Vec::new());
        for w in breaks.windows(2) {
            let (u0, u1) = (w[0], w[1]);
            if u1 - u0 <= eps {
                continue;
            }
            let nu = patch_count(u1 - u0, scale.u);
            let du_patch = (u1 - u0) / nu as f64;
            let u_scale = du_patch / 2.0;
            for iu in 0..nu {
                let u_mid = du_patch.fma(iu as f64, u0) + u_scale;
                for gpu in gauss_pts {
                    let u = u_scale.fma(gpu.x, u_mid);
                    trim.v_spans_into(u, v_range, &mut cuts, &mut spans);
                    for &(a, b) in &spans {
                        let nv = patch_count(b - a, scale.v);
                        let dv_patch = (b - a) / nv as f64;
                        let v_scale = dv_patch / 2.0;
                        for iv in 0..nv {
                            let v_mid = dv_patch.fma(iv as f64, a) + v_scale;
                            for gpv in gauss_pts {
                                let v = v_scale.fma(gpv.x, v_mid);
                                acc.add_reference::<_, AREA_ONLY>(
                                    surface,
                                    u,
                                    v,
                                    gpu.w * gpv.w * u_scale * v_scale,
                                    rule.reference,
                                    &mut scratch,
                                );
                            }
                        }
                    }
                }
            }
        }
        if !AREA_ONLY {
            rule.record_flux(acc.flux(sign));
        }
        return Ok(acc.finish(sign));
    }

    let nv = patch_count(v_range.1 - v_range.0, scale.v);
    let dv_patch = (v_range.1 - v_range.0) / nv as f64;
    let v_scale = dv_patch / 2.0;
    for iu in 0..nu {
        let u_mid = du_patch.fma(iu as f64, u_range.0) + u_scale;
        for iv in 0..nv {
            let v_mid = dv_patch.fma(iv as f64, v_range.0) + v_scale;
            for gpu in gauss_pts {
                let u = u_scale.fma(gpu.x, u_mid);
                for gpv in gauss_pts {
                    let v = v_scale.fma(gpv.x, v_mid);
                    if !trim.accepts(u, v) {
                        continue;
                    }
                    acc.add_reference::<_, AREA_ONLY>(
                        surface,
                        u,
                        v,
                        gpu.w * gpv.w * u_scale * v_scale,
                        rule.reference,
                        &mut scratch,
                    );
                }
            }
        }
    }
    if !AREA_ONLY {
        rule.record_flux(acc.flux(sign));
    }
    Ok(acc.finish(sign))
}

fn surface(
    degree_u: usize,
    degree_v: usize,
    interior_u: &[f64],
    interior_v: &[f64],
    rational: bool,
) -> NurbsSurface {
    let clamped = |degree: usize, interior: &[f64]| {
        [
            vec![0.0; degree + 1],
            interior.to_vec(),
            vec![1.0; degree + 1],
        ]
        .concat()
    };
    let rows = degree_u + 1 + interior_u.len();
    let cols = degree_v + 1 + interior_v.len();
    let cps = (0..rows)
        .map(|i| {
            (0..cols)
                .map(|j| {
                    let (x, y) = (i as f64, j as f64);
                    Point3::new(
                        x * 1.7 - 3.0,
                        y * 0.9 + 0.25 * x,
                        (0.6 * x + 1.1 * y).sin() + 2.0,
                    )
                })
                .collect()
        })
        .collect();
    let weights = (0..rows)
        .map(|i| {
            (0..cols)
                .map(|j| {
                    if rational {
                        1.0 + 0.3 * ((i * 7 + j * 3) % 5) as f64
                    } else {
                        1.0
                    }
                })
                .collect()
        })
        .collect();
    NurbsSurface::new(
        degree_u,
        degree_v,
        clamped(degree_u, interior_u),
        clamped(degree_v, interior_v),
        cps,
        weights,
    )
    .expect("valid fixture surface")
}

fn surfaces() -> Vec<(&'static str, NurbsSurface)> {
    let interior = [0.25, 0.5, 0.5, 0.8];
    vec![
        ("bicubic", surface(3, 3, &interior, &[0.3, 0.7], false)),
        ("rational bicubic", surface(3, 3, &interior, &[0.6], true)),
        ("ruled 3 x 1", surface(3, 1, &interior, &[], false)),
        ("ruled 1 x 3", surface(1, 3, &[], &[0.4], false)),
        ("bilinear", surface(1, 1, &[0.5], &[], false)),
        (
            "quintic x quadric",
            surface(5, 2, &[0.5], &[0.2, 0.9], true),
        ),
        ("quartic", surface(4, 4, &[], &[], false)),
        (
            "degree 9 (above the memo bound)",
            surface(9, 2, &[], &[0.5], true),
        ),
    ]
}

fn uv_loop(points: &[(f64, f64)], u_periodic: bool) -> UvLoop {
    UvLoop::new(
        points.iter().map(|&(u, v)| Point2::new(u, v)).collect(),
        u_periodic,
        false,
    )
}

fn circle(cu: f64, cv: f64, r: f64, n: usize) -> Vec<(f64, f64)> {
    (0..n)
        .map(|k| {
            let t = std::f64::consts::TAU * k as f64 / n as f64;
            (r.mul_add(t.cos(), cu), r.mul_add(t.sin(), cv))
        })
        .collect()
}

/// Face outlines in the unit domain: none, an inset rectangle (every row's
/// v windows repeat), a wavy polygon with two pockets (windows change per
/// row, one row can see more than the window cache holds), and a periodic
/// outline straddling `u = 1`.
fn trims() -> Vec<(&'static str, FaceUv)> {
    let wavy: Vec<(f64, f64)> = (0..=48)
        .map(|k| {
            let u = 0.05 + 0.9 * k as f64 / 48.0;
            (u, 0.08 + 0.05 * (9.0 * u).sin())
        })
        .chain((0..=48).rev().map(|k| {
            let u = 0.05 + 0.9 * k as f64 / 48.0;
            (u, 0.9 + 0.06 * (7.0 * u).cos())
        }))
        .collect();
    // A comb: each tooth is a separate v window on the rows it crosses.
    let mut comb = vec![(0.1, 0.1), (0.9, 0.1)];
    for k in 0..6 {
        let v0 = 0.2 + 0.12 * k as f64;
        comb.extend([(0.9, v0), (0.3, v0 + 0.03), (0.9, v0 + 0.06)]);
    }
    comb.extend([(0.9, 0.95), (0.1, 0.95)]);
    let uv = |boundary: Vec<(f64, f64)>, pockets: Vec<Vec<(f64, f64)>>, periodic: bool| FaceUv {
        boundary: uv_loop(&boundary, periodic),
        pockets: pockets.iter().map(|p| uv_loop(p, periodic)).collect(),
        bands: Vec::new(),
        u_periodic: periodic,
        hole_vs: Vec::new(),
    };
    vec![
        (
            "rectangle",
            uv(
                vec![(0.1, 0.15), (0.85, 0.15), (0.85, 0.8), (0.1, 0.8)],
                vec![],
                false,
            ),
        ),
        (
            "wavy with pockets",
            uv(
                wavy,
                vec![circle(0.3, 0.45, 0.1, 24), circle(0.7, 0.5, 0.15, 31)],
                false,
            ),
        ),
        ("comb", uv(comb, vec![], false)),
        (
            "seam",
            uv(
                vec![(0.8, 0.2), (1.3, 0.25), (1.25, 0.75), (0.85, 0.7)],
                vec![circle(1.0, 0.45, 0.08, 20)],
                true,
            ),
        ),
    ]
}

fn contribution_bits(c: &FaceContribution) -> [u64; 14] {
    [
        c.area,
        c.volume,
        c.volume_moment_x,
        c.volume_moment_y,
        c.volume_moment_z,
        c.volume_second_x,
        c.volume_second_y,
        c.volume_second_z,
        c.volume_product_xy,
        c.volume_product_xz,
        c.volume_product_yz,
        c.centroid_x,
        c.centroid_y,
        c.centroid_z,
    ]
    .map(f64::to_bits)
}

type Outcome = (Result<[u64; 14], String>, Option<[u64; 3]>);

/// A `(u, v)` integration domain.
type Ranges = ((f64, f64), (f64, f64));

/// Both integrators on one case, as comparable bits.
fn both<const AREA_ONLY: bool>(
    surface: &NurbsSurface,
    ranges: Ranges,
    order: usize,
    volume_only: bool,
    trim: &UvTrim<'_>,
    scale: PatchScale,
) -> (Outcome, Outcome) {
    let run = |reference_path: bool| -> Outcome {
        let flux = Cell::new(None);
        let rule = IntegrationRule {
            order,
            adaptive: None,
            knots: None,
            reference: Point3::new(0.4, -1.25, 0.75),
            volume_only,
            flux: Some(&flux),
            cone_apex: None,
            cylinder_rows: false,
        };
        let result = if reference_path {
            integrate_parametric_reference::<_, AREA_ONLY>(
                surface, ranges.0, ranges.1, rule, -1.0, trim, scale,
            )
        } else {
            integrate_parametric::<_, AREA_ONLY>(
                surface, ranges.0, ranges.1, rule, -1.0, trim, scale,
            )
        };
        (
            result
                .map(|c| contribution_bits(&c))
                .map_err(|e| format!("{e:?}")),
            flux.get()
                .map(|n| [n.x().to_bits(), n.y().to_bits(), n.z().to_bits()]),
        )
    };
    (run(false), run(true))
}

#[test]
fn nurbs_quadrature_matches_the_per_point_reference_bit_for_bit() {
    let trims = trims();
    let mut cases = 0usize;
    for (label, s) in surfaces() {
        let scale = PatchScale {
            u: knot_axis_patch_scale(s.knots_u(), s.domain_u()),
            v: knot_axis_patch_scale(s.knots_v(), s.domain_v()),
        };
        let untrimmed = UvTrim::new(None, &[], &[], false);
        let mut cases_for: Vec<(&str, UvTrim<'_>, Ranges)> = vec![
            ("untrimmed", untrimmed, ((0.0, 1.0), (0.0, 1.0))),
            (
                "untrimmed beyond the domain",
                UvTrim::new(None, &[], &[], false),
                ((-0.2, 1.1), (0.05, 1.3)),
            ),
        ];
        for (trim_label, uv) in &trims {
            let range = if uv.u_periodic {
                ((0.75, 1.35), (0.0, 1.0))
            } else {
                ((0.0, 1.0), (0.0, 1.0))
            };
            cases_for.push((trim_label, UvTrim::boundary_of(uv), range));
            cases_for.push((trim_label, UvTrim::holes_of(uv), range));
        }
        for (trim_label, trim, ranges) in &cases_for {
            for order in 1..=8 {
                for volume_only in [false, true] {
                    let (new, old) = both::<false>(&s, *ranges, order, volume_only, trim, scale);
                    assert_eq!(
                        new, old,
                        "{label}, {trim_label}, order {order}, volume_only {volume_only}"
                    );
                    cases += 1;
                }
                let (new, old) = both::<true>(&s, *ranges, order, false, trim, scale);
                assert_eq!(new, old, "{label}, {trim_label}, order {order}, area only");
            }
        }
    }
    assert!(cases >= 8 * 10 * 8 * 2, "only {cases} cases");
}

/// The rows path actually runs: the trimmed fixture rows take the window
/// cache, including eviction, and the tables refuse a foreign surface.
#[test]
fn nurbs_rows_reuse_windows_and_refuse_foreign_tables() {
    let s = surface(3, 3, &[0.5], &[0.5], false);
    if !s.has_first_order_tables() {
        return;
    }
    let gauss = gauss_legendre_points(3);
    let mut rows = NurbsRows::new(&s, gauss, 0.25);
    let first = rows.window(0.1, 0.8);
    assert_eq!(
        rows.window(0.1, 0.8),
        first,
        "a repeated window reuses its table"
    );
    for k in 0..NURBS_ROW_WINDOWS {
        rows.window(0.2 + 0.01 * k as f64, 0.9);
    }
    assert_eq!(rows.windows.len(), NURBS_ROW_WINDOWS);
    assert!(
        rows.windows
            .iter()
            .all(|(key, _)| *key != (0.1f64.to_bits(), 0.8f64.to_bits())),
        "the oldest window was evicted"
    );
    let other = surface(3, 3, &[0.5], &[0.5], false);
    let foreign = other.first_order_axis_v(&[0.3]);
    let own_u = s.first_order_axis_u(&[0.3]);
    let own_v = s.first_order_axis_v(&[0.3]);
    assert!(s.first_order_at(&own_u, 0, &own_v, 0).is_some());
    assert!(s.first_order_at(&own_u, 0, &foreign, 0).is_none());
    assert!(s.first_order_at(&own_u, 1, &own_v, 0).is_none());
    assert!(s.first_order_at(&own_v, 0, &own_u, 0).is_none());
}
