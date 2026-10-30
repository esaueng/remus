//! Bit-identity oracle for [`integrate_parametric`] with its sliced request
//! out of line ([`integrate_sliced_request`]) and its knot-aligned path
//! reached through `as_nurbs`.
//!
//! `integrate_parametric_before` is the function as it stood before, kept
//! verbatim. Every case compares the two contributions and vector areas bit
//! for bit, for each analytic carrier and two NURBS patches, area-only and
//! full reads, full and volume-only moments, several Gauss orders,
//! untrimmed, outlined and holed domains, and fixed, refined (sliced and
//! quadrant) and knot-aligned rules.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use super::*;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};

/// [`integrate_parametric`] before the change, verbatim.
#[allow(clippy::cast_precision_loss)]
fn integrate_parametric_before<S: ParametricSurface, const AREA_ONLY: bool>(
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
    if let Some(knots) = rule.knots
        && let Some(contribution) = integrate_knot_aligned::<_, AREA_ONLY>(
            surface, u_range, v_range, rule, sign, trim, scale, knots,
        )?
    {
        return Ok(contribution);
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
        let nurbs = surface
            .as_nurbs()
            .filter(|nurbs| nurbs.has_first_order_tables());
        let mut rows = nurbs.map(|nurbs| NurbsRows::new(nurbs, gauss_pts, scale.v));
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
                    // `nurbs` is a constant `None` in the analytic
                    // instantiations, which drops the row path from them;
                    // `rows` alone lives in memory across the loop and does
                    // not fold.
                    if nurbs.is_some()
                        && let Some(rows) = rows.as_mut()
                    {
                        rows.add_row::<AREA_ONLY>(
                            &mut acc,
                            (u, gpu.w, u_scale),
                            &spans,
                            rule.reference,
                            &mut scratch,
                        );
                        continue;
                    }
                    for &(a, b) in &spans {
                        let nv = patch_count(b - a, scale.v);
                        let dv_patch = (b - a) / nv as f64;
                        let v_scale = dv_patch / 2.0;
                        for iv in 0..nv {
                            let v_mid = dv_patch.fma(iv as f64, a) + v_scale;
                            for gpv in gauss_pts {
                                let v = v_scale.fma(gpv.x, v_mid);
                                acc.add::<_, AREA_ONLY>(
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
    if let Some(nurbs) = surface
        .as_nurbs()
        .filter(|nurbs| nurbs.has_first_order_tables())
    {
        integrate_nurbs_grid::<AREA_ONLY>(
            nurbs,
            &mut acc,
            NurbsGrid {
                u: (u_range.0, nu, du_patch, u_scale),
                v: (v_range.0, nv, dv_patch, v_scale),
                gauss_pts,
                trim,
                reference: rule.reference,
            },
            &mut scratch,
        );
        if !AREA_ONLY {
            rule.record_flux(acc.flux(sign));
        }
        return Ok(acc.finish(sign));
    }
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
                    acc.add::<_, AREA_ONLY>(
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

type Outcome = Result<([u64; 14], Option<[u64; 3]>), String>;

fn outcome(result: Result<FaceContribution, CheckError>, flux: &Cell<Option<Vec3>>) -> Outcome {
    result
        .map(|c| {
            (
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
                .map(f64::to_bits),
                flux.get().map(|n| [n.x(), n.y(), n.z()].map(f64::to_bits)),
            )
        })
        .map_err(|error| error.to_string())
}

/// A loop over `domain` from points given in the unit square.
fn mapped(domain: ((f64, f64), (f64, f64)), points: &[(f64, f64)]) -> UvLoop {
    let ((u0, u1), (v0, v1)) = domain;
    UvLoop::new(
        points
            .iter()
            .map(|&(s, t)| Point2::new(s.mul_add(u1 - u0, u0), t.mul_add(v1 - v0, v0)))
            .collect(),
        false,
        false,
    )
}

fn trims(domain: ((f64, f64), (f64, f64))) -> Vec<FaceUv> {
    let face = |boundary: Vec<(f64, f64)>, pockets: Vec<Vec<(f64, f64)>>| FaceUv {
        boundary: mapped(domain, &boundary),
        pockets: pockets.iter().map(|p| mapped(domain, p)).collect(),
        bands: Vec::new(),
        u_periodic: false,
        hole_vs: Vec::new(),
    };
    let notch = vec![
        (0.05, 0.1),
        (0.9, 0.05),
        (0.95, 0.85),
        (0.55, 0.6),
        (0.4, 0.95),
        (0.1, 0.8),
    ];
    let hole = vec![(0.3, 0.3), (0.6, 0.35), (0.5, 0.55), (0.35, 0.5)];
    vec![face(notch.clone(), Vec::new()), face(notch, vec![hole])]
}

fn nurbs_patch(rational: bool) -> NurbsSurface {
    let cps = (0..5)
        .map(|i| {
            (0..4)
                .map(|j| {
                    let (x, y) = (f64::from(i), f64::from(j));
                    Point3::new(x * 1.3 - 2.0, y * 1.1 + 0.2 * x, (0.5 * x + 0.9 * y).sin())
                })
                .collect()
        })
        .collect();
    let weights = (0..5)
        .map(|i| {
            (0..4)
                .map(|j| {
                    if rational {
                        1.0 + 0.25 * f64::from((i * 3 + j) % 4)
                    } else {
                        1.0
                    }
                })
                .collect()
        })
        .collect();
    NurbsSurface::new(
        3,
        2,
        vec![0.0, 0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 0.6, 1.0, 1.0, 1.0],
        cps,
        weights,
    )
    .expect("valid fixture surface")
}

/// Run one carrier through both copies over every rule, trim and read.
fn compare<S: ParametricSurface>(
    name: &str,
    surface: &S,
    domain: ((f64, f64), (f64, f64)),
    knots: Option<(&[f64], &[f64])>,
    scale: PatchScale,
) {
    let refined = PropertiesOptions {
        adaptive_eps: 1e-5,
        max_depth: 3,
        ..PropertiesOptions::default()
    };
    let uvs = trims(domain);
    let mut cases = 0;
    for order in [2, 5, 8] {
        for adaptive in [None, Some(&refined)] {
            for volume_only in [false, true] {
                for sign in [1.0, -1.0] {
                    for trim_index in 0..=uvs.len() {
                        let trim = match trim_index {
                            0 => UvTrim::new(None, &[], &[], false),
                            k => UvTrim::boundary_of(&uvs[k - 1]),
                        };
                        for area_only in [false, true] {
                            let run = |shared: bool| {
                                let flux = Cell::new(None);
                                let rule = IntegrationRule {
                                    order,
                                    adaptive,
                                    knots,
                                    reference: Point3::new(0.4, -0.7, 1.3),
                                    volume_only,
                                    flux: Some(&flux),
                                    cone_apex: None,
                                };
                                let (u, v) = domain;
                                let result = match (shared, area_only) {
                                    (true, true) => integrate_parametric::<_, true>(
                                        surface, u, v, rule, sign, &trim, scale,
                                    ),
                                    (true, false) => integrate_parametric::<_, false>(
                                        surface, u, v, rule, sign, &trim, scale,
                                    ),
                                    (false, true) => integrate_parametric_before::<_, true>(
                                        surface, u, v, rule, sign, &trim, scale,
                                    ),
                                    (false, false) => integrate_parametric_before::<_, false>(
                                        surface, u, v, rule, sign, &trim, scale,
                                    ),
                                };
                                outcome(result, &flux)
                            };
                            let shared = run(true);
                            assert_eq!(
                                shared,
                                run(false),
                                "{name}: order {order}, refined {}, volume_only \
                                 {volume_only}, sign {sign}, trim {trim_index}, \
                                 area_only {area_only}",
                                adaptive.is_some(),
                            );
                            cases += usize::from(shared.is_ok());
                        }
                    }
                }
            }
        }
    }
    // Most cases must integrate; a comparison of two errors proves little.
    assert!(cases >= 100, "{name}: only {cases} cases integrated");
}

#[test]
fn split_matches_before_on_analytic_carriers() {
    let axis = Vec3::new(0.3, -0.4, 0.8);
    let origin = Point3::new(2.0, -1.5, 0.75);
    let cylinder = CylindricalSurface::new(origin, axis, 2.5).unwrap();
    compare(
        "cylinder",
        &cylinder,
        ((0.3, 2.6), (-1.0, 3.0)),
        None,
        PatchScale::ANGULAR,
    );
    let cone = ConicalSurface::new(origin, axis, 0.4).unwrap();
    compare(
        "cone",
        &cone,
        ((0.3, 2.6), (0.5, 3.0)),
        None,
        PatchScale::ANGULAR,
    );
    let sphere = SphericalSurface::new(origin, 3.0).unwrap();
    compare(
        "sphere",
        &sphere,
        ((0.3, 2.6), (-1.1, 1.2)),
        None,
        PatchScale::ANGULAR,
    );
    let torus = ToroidalSurface::new(origin, 5.0, 1.5).unwrap();
    compare(
        "torus",
        &torus,
        ((0.3, 2.6), (0.2, 4.0)),
        None,
        PatchScale::ANGULAR,
    );
}

#[test]
fn split_matches_before_on_nurbs() {
    for rational in [false, true] {
        let nurbs = nurbs_patch(rational);
        let scale = PatchScale {
            u: knot_axis_patch_scale(nurbs.knots_u(), nurbs.domain_u()),
            v: knot_axis_patch_scale(nurbs.knots_v(), nurbs.domain_v()),
        };
        let domain = ((0.05, 0.95), (0.0, 1.0));
        let name = if rational { "rational nurbs" } else { "nurbs" };
        compare(name, &nurbs, domain, None, scale);
        compare(
            name,
            &nurbs,
            domain,
            Some((nurbs.knots_u(), nurbs.knots_v())),
            scale,
        );
    }
}
