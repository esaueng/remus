//! Verbatim pre-change knot-aligned loop, used as a bit-identity oracle.
#![allow(
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]
use super::*;

#[test]
fn ruled_planar_patch_has_closed_form_volume_and_flux() {
    let ys = [0.0, 0.2, 0.55, 0.8, 1.0];
    let points = [0.0, 0.5, 1.0]
        .into_iter()
        .map(|x| ys.into_iter().map(|y| Point3::new(x, y, 2.0)).collect())
        .collect();
    let weights = [1.0, 0.7, 1.0]
        .into_iter()
        .map(|u| {
            [0.7, 1.2, 0.8, 1.1, 1.0]
                .into_iter()
                .map(|v| u * v)
                .collect()
        })
        .collect();
    let surface = NurbsSurface::new(
        2,
        1,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.2, 0.55, 0.8, 1.0, 1.0],
        points,
        weights,
    )
    .unwrap();
    for sign in [-1.0, 1.0] {
        let flux = Cell::new(None);
        let result = integrate_knot_aligned::<_, false>(
            &surface,
            (0.0, 1.0),
            (0.0, 1.0),
            IntegrationRule {
                order: 8,
                adaptive: None,
                knots: None,
                reference: Point3::new(1000.0, -300.0, 1.0),
                volume_only: true,
                ruled_guard: true,
                flux: Some(&flux),
                cone_apex: None,
            },
            sign,
            &UvTrim::new(None, &[], &[], true),
            PatchScale { u: 0.125, v: 0.025 },
            (surface.knots_u(), surface.knots_v()),
        )
        .unwrap()
        .unwrap();
        assert!((result.volume - sign / 3.0).abs() < 2e-12);
        assert!((flux.get().unwrap() - Vec3::new(0.0, 0.0, sign)).length() < 2e-12);
        assert!((result.area - 1.0).abs() < 2e-12);
    }
}

#[test]
fn discontinuous_linear_spans_decline_the_generator_certificate() {
    let surface =
        super::nurbs_rows_oracle_tests::surface(2, 1, &[0.3], &[0.2, 0.55, 0.55, 0.8], true);
    let mut topo = Topology::new();
    let wire = remus_topology::builder::make_polygon_wire(
        &mut topo,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
        1e-7,
    )
    .unwrap();
    let face = topo.add_face(Face::new(wire, vec![], FaceSurface::Nurbs(surface)));
    assert!(
        integrate_ruled_guard_volume_about(&topo, face, 5, Point3::new(0.0, 0.0, 0.0))
            .unwrap()
            .is_none()
    );
}

#[test]
fn ruled_generators_agree_with_independent_tensor_quadrature() {
    use super::nurbs_rows_oracle_tests::{surface, trims};
    for rational in [false, true] {
        for u_knots in [&[][..], &[0.3, 0.7][..]] {
            let s = surface(2, 1, u_knots, &[0.2, 0.55, 0.8], rational);
            let scale = PatchScale {
                u: knot_axis_patch_scale(s.knots_u(), s.domain_u()),
                v: knot_axis_patch_scale(s.knots_v(), s.domain_v()),
            };
            for (label, uv) in trims().into_iter().filter(|(_, uv)| !uv.u_periodic) {
                for order in [3, 5, 8] {
                    let old_flux = Cell::new(None);
                    let new_flux = Cell::new(None);
                    let rule = IntegrationRule {
                        order,
                        adaptive: None,
                        knots: Some((s.knots_u(), s.knots_v())),
                        reference: Point3::new(0.13, -0.27, 0.41),
                        volume_only: true,
                        ruled_guard: false,
                        flux: Some(&old_flux),
                        cone_apex: None,
                    };
                    let trim = UvTrim::boundary_of(&uv);
                    let old = integrate_knot_aligned_reference::<_, false>(
                        &s,
                        (0.0, 1.0),
                        (0.0, 1.0),
                        rule,
                        -1.0,
                        &trim,
                        scale,
                        (s.knots_u(), s.knots_v()),
                        Some(12),
                    )
                    .unwrap()
                    .unwrap();
                    let new = integrate_knot_aligned::<_, false>(
                        &s,
                        (0.0, 1.0),
                        (0.0, 1.0),
                        IntegrationRule {
                            ruled_guard: true,
                            flux: Some(&new_flux),
                            ..rule
                        },
                        -1.0,
                        &trim,
                        scale,
                        (s.knots_u(), s.knots_v()),
                    )
                    .unwrap()
                    .unwrap();
                    let allowance = 2e-11 * old.area.max(1.0);
                    assert!(
                        (old.volume - new.volume).abs() < allowance,
                        "{label}: {} vs {}",
                        old.volume,
                        new.volume
                    );
                    assert!(
                        (old_flux.get().unwrap() - new_flux.get().unwrap()).length() < allowance,
                        "{label}"
                    );
                    assert!(
                        new.area <= old.area + allowance,
                        "closure denominator must not grow"
                    );
                }
            }
        }
    }
}

#[test]
fn knot_aligned_rows_match_every_original_sample_bit_for_bit() {
    use super::nurbs_rows_oracle_tests::{contribution_bits, surfaces, trims};
    let reference = Point3::new(0.13, -0.27, 0.41);
    let mut cases = 0;
    for (label, surface) in surfaces() {
        let scale = PatchScale {
            u: knot_axis_patch_scale(surface.knots_u(), surface.domain_u()),
            v: knot_axis_patch_scale(surface.knots_v(), surface.domain_v()),
        };
        for (trim_label, uv) in trims() {
            for order in [1, 3, 5, 8] {
                for volume_only in [false, true] {
                    let old_flux = Cell::new(None);
                    let new_flux = Cell::new(None);
                    let rule = IntegrationRule {
                        order,
                        adaptive: None,
                        knots: Some((surface.knots_u(), surface.knots_v())),
                        reference,
                        volume_only,
                        ruled_guard: false,
                        flux: Some(&old_flux),
                        cone_apex: None,
                    };
                    let trim = UvTrim::boundary_of(&uv);
                    let old = integrate_knot_aligned_reference::<_, false>(
                        &surface,
                        (0.0, 1.0),
                        (0.0, 1.0),
                        rule,
                        -1.0,
                        &trim,
                        scale,
                        (surface.knots_u(), surface.knots_v()),
                        None,
                    )
                    .unwrap();
                    let new = integrate_knot_aligned::<_, false>(
                        &surface,
                        (0.0, 1.0),
                        (0.0, 1.0),
                        IntegrationRule {
                            flux: Some(&new_flux),
                            ..rule
                        },
                        -1.0,
                        &trim,
                        scale,
                        (surface.knots_u(), surface.knots_v()),
                    )
                    .unwrap();
                    assert_eq!(
                        old.as_ref().map(contribution_bits),
                        new.as_ref().map(contribution_bits),
                        "{label}, {trim_label}, order {order}, volume_only {volume_only}"
                    );
                    assert_eq!(
                        old_flux.get().map(|v| v.0.map(f64::to_bits)),
                        new_flux.get().map(|v| v.0.map(f64::to_bits))
                    );
                    cases += usize::from(old.is_some());
                }
            }
        }
    }
    assert!(cases >= 100, "the knot-aligned path must run: {cases}");
}
#[allow(clippy::too_many_arguments, clippy::cast_precision_loss)]
fn integrate_knot_aligned_reference<S: ParametricSurface, const AREA_ONLY: bool>(
    surface: &S,
    u_range: (f64, f64),
    v_range: (f64, f64),
    rule: IntegrationRule<'_>,
    sign: f64,
    trim: &UvTrim<'_>,
    scale: PatchScale,
    knots: (&[f64], &[f64]),
    v_order: Option<usize>,
) -> Result<Option<FaceContribution>, CheckError> {
    let knots_u = interior_knots(knots.0, u_range);
    let knots_v = interior_knots(knots.1, v_range);
    if knots_u.is_empty() && knots_v.is_empty() {
        return Ok(None);
    }
    let splits = trim.splits_domain();
    let mut breaks = if splits {
        trim.u_breaks(u_range)
    } else {
        vec![u_range.0, u_range.1]
    };
    // Every `u` interval is at least one cell, so the breaks are held to the
    // cap before any is sorted or integrated.
    breaks.extend_from_slice(&knots_u);
    if splits {
        let room = MAX_KNOT_ALIGNED_CELLS.saturating_sub(breaks.len());
        let Some(crossings) = trim.knot_crossing_breaks(u_range, &knots_v, room) else {
            return knot_axis::retry::<_, AREA_ONLY>(
                surface,
                (u_range, v_range),
                rule,
                sign,
                trim,
                scale,
            );
        };
        breaks.extend(crossings);
    }
    if breaks.len() > MAX_KNOT_ALIGNED_CELLS {
        return knot_axis::retry::<_, AREA_ONLY>(
            surface,
            (u_range, v_range),
            rule,
            sign,
            trim,
            scale,
        );
    }
    breaks.compact_sort_by(f64::total_cmp);
    breaks.dedup();

    let cell_u = knot_aligned_cell(u_range, scale.u);
    let cell_v = knot_aligned_cell(v_range, scale.v);
    let gauss_pts = gauss_legendre_points(rule.order);
    let gauss_v = gauss_legendre_points(v_order.unwrap_or(rule.order));
    let budget = MAX_KNOT_ALIGNED_CELLS.saturating_mul(gauss_pts.len());
    let mut acc = Accumulator {
        volume_only: rule.volume_only,
        ..Accumulator::default()
    };
    let mut scratch = DerivativeScratch::new();
    let (mut cuts, mut spans) = (Vec::new(), Vec::new());
    // Gauss lines integrated so far, each one `v` sub-cell at one `u`
    // abscissa: `order` of them make one tensor cell.
    // Every interval is integrated, however narrow: no width floor is safe,
    // since the parameter speed is unbounded (a narrow knot span, a steep
    // rational weight), and a sliver costs only its own Gauss lines. The
    // breaks are distinct after `dedup`, so none is empty.
    // Count before evaluating: a refusal after a partial integral otherwise
    // spends the entire cap again on every validation of the same carrier.
    // Both passes use identical abscissas, trim spans and budget arithmetic;
    // the admitted pass retains its original sample/accumulation order.
    for preflight in [true, false] {
        let mut lines = 0usize;
        for w in breaks.windows(2) {
            let (u0, u1) = (w[0], w[1]);
            let nu = knot_aligned_cells(u1 - u0, cell_u);
            let du_patch = (u1 - u0) / nu as f64;
            let u_scale = du_patch / 2.0;
            for iu in 0..nu {
                let u_mid = du_patch.fma(iu as f64, u0) + u_scale;
                for gpu in gauss_pts {
                    let u = u_scale.fma(gpu.x, u_mid);
                    if splits {
                        trim.v_spans_into(u, v_range, &mut cuts, &mut spans);
                    } else {
                        spans.clear();
                        spans.push((v_range.0.min(v_range.1), v_range.0.max(v_range.1)));
                    }
                    for &(a, b) in &spans {
                        let first = knots_v.partition_point(|&k| k <= a);
                        let last = knots_v.partition_point(|&k| k < b);
                        let inner = knots_v.get(first..last).unwrap_or(&[]);
                        let lows = std::iter::once(a).chain(inner.iter().copied());
                        let highs = inner.iter().copied().chain(std::iter::once(b));
                        for (p0, p1) in lows.zip(highs) {
                            let nv = knot_aligned_cells(p1 - p0, cell_v);
                            lines = lines.saturating_add(nv);
                            if lines > budget {
                                return knot_axis::retry::<_, AREA_ONLY>(
                                    surface,
                                    (u_range, v_range),
                                    rule,
                                    sign,
                                    trim,
                                    scale,
                                );
                            }
                            if preflight {
                                continue;
                            }
                            let dv_patch = (p1 - p0) / nv as f64;
                            let v_scale = dv_patch / 2.0;
                            for iv in 0..nv {
                                let v_mid = dv_patch.fma(iv as f64, p0) + v_scale;
                                for gpv in gauss_v {
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
        }
    }
    if !AREA_ONLY {
        rule.record_flux(acc.flux(sign));
    }
    Ok(Some(acc.finish(sign)))
}
