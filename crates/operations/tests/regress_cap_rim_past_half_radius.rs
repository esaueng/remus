//! Regression: rounding the top rim of a bare cylinder past half its radius.
//!
//! `plane_cylinder_fillet` capped the inward (bounded disc cap) case at
//! `r_c/2`, so an r = 2 cylinder took f = 0.9999 and refused f = 1.0 — and the
//! refusal arrived as a bare `partial-result: 0 succeeded, 1 failed`, which a
//! caller cannot tell from an internal failure. The cap was there because the
//! carrier torus (`major = r_c − f`, `minor = f`) becomes a horn at exactly
//! `f = r_c/2` and a self-intersecting spindle past it.
//!
//! The torus does; the FACE cut from it does not. The band spans a quarter of
//! the tube — from the wall contact at `v = 0`, tube radial `r_c`, to the plate
//! contact at `v = ±π/2`, tube radial `r_c − f`. A spindle crosses its own axis
//! only where the tube radial goes negative, `major + minor·cos v < 0`, i.e.
//! `|v| > arccos(−major/minor) ≥ π/2` for every non-negative major. The
//! self-intersecting lobe is disjoint from the quarter used. The real bound is
//! the rolling ball fitting inside the cylinder: `f < r_c`.
//!
//! What proves the trim here is not that the shell closes — a band spanning too
//! much of the tube would still close — but that the removed volume and the
//! band's own area match closed forms derived independently, at every radius on
//! both sides of 0.5.
//!
//! The downstream cost was concrete: OpenZCAD's adapter approximates this blend
//! with 128 cone faces against the kernel's 5, at −0.0023% to −0.0078% volume
//! error and +44% triangles, because its guards admit `0 < f < r`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::collections::HashMap;

use remus_blend::BlendError;
use remus_math::vec::Point3;
use remus_operations::OperationsError;
use remus_operations::blend_ops::{blend_failure_code, chamfer_v2, fillet_v2};
use remus_operations::measure;
use remus_operations::primitives::make_cylinder;
use remus_operations::tessellate::tessellate_solid_with_tolerance;
use remus_topology::Topology;
use remus_topology::edge::{EdgeCurve, EdgeId};
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

const PI: f64 = std::f64::consts::PI;

/// The one closed circular edge whose points all sit at `z = h`.
fn top_rim(topo: &Topology, solid: SolidId, h: f64) -> EdgeId {
    let mut found = Vec::new();
    for eid in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        let e = topo.edge(eid).unwrap();
        if e.start() != e.end() || !matches!(e.curve(), EdgeCurve::Circle(_)) {
            continue;
        }
        let p = topo.vertex(e.start()).unwrap().point();
        if (p.z() - h).abs() < 1e-9 {
            found.push(eid);
        }
    }
    assert_eq!(
        found.len(),
        1,
        "expected exactly one top rim, got {found:?}"
    );
    found[0]
}

/// Material removed from a cylinder of radius `r` by rounding its top rim at
/// radius `f`, for any `0 < f ≤ r`.
///
/// The result's cross-section at height `t` above the plane of the ball-centre
/// circle is a disc of radius `R + √(f² − t²)` with `R = r − f`, so the removed
/// solid is `∫₀^f π[r² − (R + √(f² − t²))²] dt`. With `r = R + f` the
/// integrand is `π(2Rf + f² − 2R√(f² − t²) − (f² − t²))`, and
/// `∫₀^f √(f² − t²) dt = πf²/4`, giving
///
/// ```text
///   V = π · ( R·f²·(2 − π/2) + f³/3 )
/// ```
///
/// Equivalently by Pappus: the annulus `π·f·(r² − R²)` minus the quarter-disc
/// torus solid `2π·(R + 4f/3π)·(πf²/4)`. At `f = r` (`R = 0`) it collapses to
/// `πr³/3`, so the body becomes `πr²h − πr³/3` — the hemispherical end
/// `πr²(h − r) + (2/3)πr³`.
fn removed_volume(r: f64, f: f64) -> f64 {
    let big_r = r - f;
    PI * (big_r * f * f * (2.0 - PI / 2.0) + f * f * f / 3.0)
}

/// Area of the toroidal band alone, by Pappus on the quarter tube arc: arc
/// length `πf/2`, mean radial `R + 2f/π`, so `2π(R + 2f/π)(πf/2)`.
///
/// This is the quantity that pins the trim. A band spanning more than the
/// quarter — the failure mode a self-intersecting apple torus would produce —
/// measures larger while still closing the shell.
fn band_area(r: f64, f: f64) -> f64 {
    PI * PI * f * (r - f) + 2.0 * PI * f * f
}

/// Total surface area of the filleted cylinder: base disc, shortened lateral,
/// shrunken top disc, and the band.
fn total_area(r: f64, h: f64, f: f64) -> f64 {
    let big_r = r - f;
    PI * r * r + 2.0 * PI * r * (h - f) + PI * big_r * big_r + band_area(r, f)
}

/// `(free edges, non-manifold edges)` counted over the B-rep wires.
fn brep_edge_health(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mut usage: HashMap<usize, usize> = HashMap::new();
    for fid in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face = topo.face(fid).unwrap();
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid).unwrap().edges() {
                *usage.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }
    (
        usage.values().filter(|&&c| c == 1).count(),
        usage.values().filter(|&&c| c >= 3).count(),
    )
}

/// `(free edges, non-manifold edges)` counted over the welded triangle mesh.
fn mesh_edge_health(topo: &Topology, solid: SolidId) -> (usize, usize) {
    let mesh = tessellate_solid_with_tolerance(topo, solid, 0.002, 0.05).unwrap();
    let q = 1e6;
    let mut canon: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut remap = vec![0u32; mesh.positions.len()];
    for (i, p) in mesh.positions.iter().enumerate() {
        let key = (
            (p.x() * q).round() as i64,
            (p.y() * q).round() as i64,
            (p.z() * q).round() as i64,
        );
        let next = canon.len() as u32;
        remap[i] = *canon.entry(key).or_insert(next);
    }
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for tri in mesh.indices.chunks_exact(3) {
        let v = [
            remap[tri[0] as usize],
            remap[tri[1] as usize],
            remap[tri[2] as usize],
        ];
        for k in 0..3 {
            let (a, b) = (v[k], v[(k + 1) % 3]);
            if a == b {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            *edges.entry(key).or_insert(0) += 1;
        }
    }
    (
        edges.values().filter(|&&c| c == 1).count(),
        edges.values().filter(|&&c| c > 2).count(),
    )
}

fn surface_census(topo: &Topology, solid: SolidId) -> Vec<&'static str> {
    let mut tags: Vec<_> = remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .map(|fid| topo.face(fid).unwrap().surface().type_tag())
        .collect();
    tags.sort_unstable();
    tags
}

/// Round the top rim and assert everything that must hold about the answer.
/// Returns the measured volume.
fn round_top_rim_and_check(r: f64, h: f64, f: f64) -> f64 {
    let what = format!("r={r} h={h} f={f} (f/r={})", f / r);
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, r, h).unwrap();
    let rim = top_rim(&topo, cyl, h);
    let result = fillet_v2(&mut topo, cyl, &[rim], f)
        .unwrap_or_else(|e| panic!("{what}: fillet refused ({}): {e}", blend_failure_code(&e)));
    let solid = result.solid;

    assert!(!result.is_partial, "{what}: result is partial");

    // Five faces on the OpenZCAD side of the boundary; four here, since the
    // kernel's cylinder lateral is one seamed face rather than two halves.
    assert_eq!(
        surface_census(&topo, solid),
        vec!["cylinder", "plane", "plane", "torus"],
        "{what}: face census"
    );

    // The band's carrier torus is exactly the rolling ball's trace.
    let torus = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .find_map(|fid| match topo.face(fid).unwrap().surface() {
            FaceSurface::Torus(t) => Some(t.clone()),
            _ => None,
        })
        .unwrap_or_else(|| panic!("{what}: no torus band"));
    assert!(
        (torus.major_radius() - (r - f)).abs() < 1e-9 && (torus.minor_radius() - f).abs() < 1e-9,
        "{what}: band torus should be R={}, r={f}, got R={}, r={}",
        r - f,
        torus.major_radius(),
        torus.minor_radius()
    );

    // Watertight, by three independent readings.
    let report = remus_operations::validate::validate_solid(&topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{what}: validate_solid reported {} error(s)",
        report.error_count()
    );
    assert_eq!(
        brep_edge_health(&topo, solid),
        (0, 0),
        "{what}: B-rep free/non-manifold edges"
    );
    assert_eq!(
        mesh_edge_health(&topo, solid),
        (0, 0),
        "{what}: mesh free/non-manifold edges"
    );

    // Volume against the closed form. The body is all-analytic, so this is
    // exact quadrature over the trimmed faces, not an inscribed mesh.
    let volume = measure::solid_volume(&topo, solid, 1e-5).unwrap();
    let expected = PI * r * r * h - removed_volume(r, f);
    assert!(
        (volume - expected).abs() <= 1e-12 * expected,
        "{what}: volume {volume} vs closed form {expected}"
    );

    // Area against the closed form — this is what says the band is a quarter
    // of the tube and not more of it.
    let area = measure::solid_surface_area(&topo, solid, 1e-6).unwrap();
    let expected_area = total_area(r, h, f);
    assert!(
        (area - expected_area).abs() <= 1e-4 * expected_area,
        "{what}: area {area} vs closed form {expected_area}"
    );

    // Every point of the band lies between the two contact radii and between
    // the two contact heights: the quarter tube, nothing of the inner lobe.
    let centre = torus.center();
    for i in 0..=64 {
        for j in 0..=64 {
            let u = 2.0 * PI * f64::from(i) / 64.0;
            let v = 0.5 * PI * f64::from(j) / 64.0;
            let p = torus.evaluate(u, v);
            let radial = (p.x() * p.x() + p.y() * p.y()).sqrt();
            assert!(
                radial >= (r - f) - 1e-9 && radial <= r + 1e-9,
                "{what}: band point at radial {radial}, outside [{}, {r}]",
                r - f
            );
            assert!(
                p.z() >= centre.z() - 1e-9 && p.z() <= h + 1e-9,
                "{what}: band point at z {}, outside [{}, {h}]",
                p.z(),
                centre.z()
            );
        }
    }

    // `mass_properties` must agree with `solid_volume`, and the centroid must
    // land on the axis below the top.
    let gprops = measure::mass_properties(&topo, solid).unwrap();
    assert!(
        (gprops.mass - volume).abs() <= 1e-9 * volume,
        "{what}: mass_properties {} vs solid_volume {volume}",
        gprops.mass
    );
    assert!(
        gprops.center.x().abs() < 1e-9 && gprops.center.y().abs() < 1e-9,
        "{what}: centroid off axis at {:?}",
        gprops.center
    );
    assert!(
        gprops.center.z() > 0.0 && gprops.center.z() < h,
        "{what}: centroid at z {} outside the body",
        gprops.center.z()
    );

    volume
}

/// The whole `f/r` range rounds, and the volume curve is the same closed form
/// on both sides of 0.5 — the boundary the old cap sat on.
#[test]
fn cap_rim_rounds_at_every_radius_below_the_cylinder_radius() {
    let (r, h) = (2.0, 12.0);
    for step in 1..=99 {
        let f = r * f64::from(step) / 100.0;
        round_top_rim_and_check(r, h, f);
    }
}

/// Continuity across `f/r = 0.5` measured on the results themselves: the second
/// difference of the volume curve straddling the old boundary must be the
/// closed form's own, not a step. A discontinuity here would mean the newly
/// admitted radii disagree with the ones that always worked.
#[test]
fn the_volume_curve_does_not_step_at_half_the_radius() {
    let (r, h) = (2.0, 12.0);
    let d = 0.001;
    let mut measured = Vec::new();
    let mut closed = Vec::new();
    for k in -3i32..=3 {
        let f = r * (0.5 + f64::from(k) * d);
        measured.push(round_top_rim_and_check(r, h, f));
        closed.push(PI * r * r * h - removed_volume(r, f));
    }
    for w in 0..measured.len() - 2 {
        let second_measured = measured[w] - 2.0 * measured[w + 1] + measured[w + 2];
        let second_closed = closed[w] - 2.0 * closed[w + 1] + closed[w + 2];
        assert!(
            (second_measured - second_closed).abs() < 1e-12,
            "volume curve steps across f/r = 0.5: second difference {second_measured} \
             against the closed form's {second_closed}"
        );
    }
}

/// The bound is `f < r` regardless of scale or height — the old one was
/// `f < r/2`, equally scale-invariant, which is what made it look like a real
/// geometric limit.
#[test]
fn the_bound_is_the_cylinder_radius_at_every_scale_and_height() {
    for &r in &[1.0, 2.25, 3.0, 10.0] {
        for &h in &[2.5_f64, 12.0, 50.0] {
            for &frac in &[0.5, 0.75, 0.95] {
                let f = r * frac;
                if f >= h {
                    continue; // the wall has to survive its own shortening
                }
                round_top_rim_and_check(r, h, f);
            }
        }
    }
}

/// At `f → r` the body converges on a hemispherical end.
#[test]
fn the_limit_of_the_blend_is_a_hemispherical_end() {
    let (r, h) = (2.0, 12.0);
    let f = r * (1.0 - 1e-4);
    let volume = round_top_rim_and_check(r, h, f);
    let hemispherical = PI * r * r * (h - r) + (2.0 / 3.0) * PI * r * r * r;
    assert!(
        (volume - hemispherical).abs() < 1e-4 * hemispherical,
        "at f = r(1 − 1e−4) the body should be within 1e−4 of the hemispherical \
         end {hemispherical}, got {volume}"
    );
}

/// `f ≥ r` is genuinely impossible — the rolling ball does not fit inside the
/// cylinder — so it must be refused BY NAME, with the edge and the achievable
/// maximum, not as a bare partial result. That is the same typed refusal the
/// `2f < h` case already returns, and it is the difference between a caller
/// being able to say "try a smaller radius" and not.
#[test]
fn a_radius_past_the_cylinder_radius_is_refused_by_name() {
    let (r, h) = (2.0, 12.0);
    // The last sliver below `r` goes with them: the cap it would leave is
    // smaller across than a vertex tolerance, so the answer is a degenerate
    // body rather than the hemisphere.
    for &f in &[r * (1.0 - 1e-9), r, r * 1.001, r * 1.5, r * 4.0] {
        let mut topo = Topology::new();
        let cyl = make_cylinder(&mut topo, r, h).unwrap();
        let rim = top_rim(&topo, cyl, h);
        let before = measure::solid_volume(&topo, cyl, 1e-5).unwrap();

        let err = fillet_v2(&mut topo, cyl, &[rim], f)
            .err()
            .unwrap_or_else(|| panic!("f = {f} must not produce a blend"));
        assert_eq!(
            blend_failure_code(&err),
            "radius-too-large",
            "f = {f}: got {err}"
        );
        match &err {
            OperationsError::Blend(BlendError::RadiusTooLarge { edge, max_radius }) => {
                assert_eq!(*edge, rim, "f = {f}: refusal must name the rim edge");
                assert!(
                    (max_radius - r).abs() < 1e-9,
                    "f = {f}: achievable maximum should be r = {r}, got {max_radius}"
                );
            }
            other => panic!("f = {f}: expected RadiusTooLarge, got {other}"),
        }

        // A refusal leaves the input alone.
        assert!(
            (measure::solid_volume(&topo, cyl, 1e-5).unwrap() - before).abs() < 1e-9,
            "f = {f}: the refused attempt changed the input solid"
        );
    }
}

/// The height-driven limit is a different one and must keep reporting itself:
/// a wall cannot be shortened past its own extent.
#[test]
fn the_height_limit_still_reports_itself_separately() {
    let (r, h) = (10.0, 4.0);
    let mut topo = Topology::new();
    let cyl = make_cylinder(&mut topo, r, h).unwrap();
    let rim = top_rim(&topo, cyl, h);
    let err = fillet_v2(&mut topo, cyl, &[rim], h)
        .err()
        .expect("a fillet as deep as the cylinder is tall must be refused");
    assert_eq!(blend_failure_code(&err), "radius-too-large", "got {err}");
}

/// The rim rebuild swaps the wall's rim circle for the contact circle and
/// re-points the wall's seam edge at the new circle's seam vertex — keeping the
/// seam's own curve. Both contact circles must therefore be seamed on the ray
/// the rim's seam vertex already sits on. They were not: built without a
/// reference direction, a circle about `+z` seams a quarter turn away, so the
/// seam line became a straight chord through the inside of the cylinder.
///
/// Nothing topological catches that — the shell closes and the tessellation is
/// watertight, because both work from the surface. `integrate_face` reads the
/// wire as the face's real boundary, and reported an r = 2, h = 12 cylinder
/// filleted at f = 0.5 as mass 71.882 against volume 150.160, with its centroid
/// at z = 12.48: above a solid twelve units tall.
#[test]
fn every_edge_of_a_rebuilt_wall_lies_on_the_wall() {
    let (r, h, f) = (2.0, 12.0, 0.5);
    for (what, solid) in [("fillet", 0u8), ("chamfer", 1u8)] {
        let mut topo = Topology::new();
        let cyl = make_cylinder(&mut topo, r, h).unwrap();
        let rim = top_rim(&topo, cyl, h);
        let result = if solid == 0 {
            fillet_v2(&mut topo, cyl, &[rim], f).unwrap()
        } else {
            chamfer_v2(&mut topo, cyl, &[rim], f, f).unwrap()
        };

        for fid in remus_topology::explorer::solid_faces(&topo, result.solid).unwrap() {
            let face = topo.face(fid).unwrap();
            let FaceSurface::Cylinder(cylinder) = face.surface() else {
                continue;
            };
            for oe in topo.wire(face.outer_wire()).unwrap().edges() {
                let edge = topo.edge(oe.edge()).unwrap();
                let a = topo.vertex(edge.start()).unwrap().point();
                let b = topo.vertex(edge.end()).unwrap().point();
                // Sample the edge, including the interior where a chord leaves
                // the surface even though both endpoints sit on it.
                for k in 0..=16 {
                    let t = f64::from(k) / 16.0;
                    let p = Point3::new(
                        a.x() + (b.x() - a.x()) * t,
                        a.y() + (b.y() - a.y()) * t,
                        a.z() + (b.z() - a.z()) * t,
                    );
                    let p = if matches!(edge.curve(), EdgeCurve::Line) {
                        p
                    } else {
                        continue; // circles about the axis are on it by construction
                    };
                    let d = p - cylinder.origin();
                    let axis = cylinder.axis();
                    let radial = (d - axis * axis.dot(d)).length();
                    assert!(
                        (radial - cylinder.radius()).abs() < 1e-9,
                        "{what}: the wall's seam runs off the wall — a point at \
                         radial {radial} on a cylinder of radius {}",
                        cylinder.radius()
                    );
                }
            }
        }

        // …which is exactly what made the measured mass disagree.
        let volume = measure::solid_volume(&topo, result.solid, 1e-5).unwrap();
        let gprops = measure::mass_properties(&topo, result.solid).unwrap();
        assert!(
            (gprops.mass - volume).abs() <= 1e-9 * volume,
            "{what}: mass_properties {} vs solid_volume {volume}",
            gprops.mass
        );
        assert!(
            gprops.center.z() > 0.0 && gprops.center.z() < h,
            "{what}: centroid at z {} outside the body",
            gprops.center.z()
        );
    }
}
