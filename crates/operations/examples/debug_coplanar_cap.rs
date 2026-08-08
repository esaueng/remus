#![allow(
    clippy::unwrap_used,
    clippy::print_stdout,
    clippy::print_stderr,
    missing_docs
)]

//! Harness for `box fuse cylinder` where the cylinder protrudes past a
//! vertical CORNER of the box — the family that used to fall back to a
//! co-refined mesh (the OpenZCAD default layout, both primitives based at
//! z=0 with equal height).
//!
//! Cap coplanarity looked like the trigger but is not: the discriminator is
//! whether the box's corner edge falls inside the cylinder, which the `corner`
//! and `sweep` modes demonstrate across z-layouts.
//!
//! Modes:
//!   (none)  the minimal reported repro, with its face census and volume
//!   sweep   the cx x cy grid under three z-layouts, listing every fallback
//!   single  crossings of ONE side face only (these always worked)
//!   corner  corner-swallowing vs not, against cap coplanarity
//!   seam    the same solid with the cylinder rotated about its own axis
//!   raw     below the operations gate: `gfa::boolean` output, free and
//!           non-manifold edges, per-face area and plane
//!   verify  volume vs the closed form, closed-manifold shell, and ray-cast
//!           classification of a point in the protruding wall

use brepkit_math::mat::Mat4;
use brepkit_operations::boolean::{BooleanOp, boolean};
use brepkit_operations::primitives;
use brepkit_operations::transform::transform_solid;
use brepkit_topology::Topology;
use brepkit_topology::explorer::solid_faces;

fn census(
    topo: &Topology,
    sid: brepkit_topology::arena::Id<brepkit_topology::solid::Solid>,
) -> String {
    let mut counts: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    for fid in solid_faces(topo, sid).unwrap() {
        *counts
            .entry(topo.face(fid).unwrap().surface().type_tag())
            .or_default() += 1;
    }
    format!("{counts:?}")
}

fn run(cx: f64, cy: f64, cz: f64, h: f64, verbose: bool) -> (usize, bool) {
    let mut topo = Topology::new();
    let bx = primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
    let cyl = primitives::make_cylinder(&mut topo, 6.0, h).unwrap();
    transform_solid(&mut topo, cyl, &Mat4::translation(cx, cy, cz)).unwrap();

    let result = boolean(&mut topo, BooleanOp::Fuse, bx, cyl).unwrap();
    let faces = solid_faces(&topo, result).unwrap();
    let curved = faces
        .iter()
        .filter(|f| topo.face(**f).unwrap().surface().type_tag() != "plane")
        .count();
    let fallback = curved == 0;
    if verbose {
        eprintln!(
            "cx={cx} cy={cy} cz={cz} h={h}: {} faces {}",
            faces.len(),
            census(&topo, result)
        );
        let vol = brepkit_operations::measure::solid_volume(&topo, result, 0.05).unwrap();
        eprintln!("  volume = {vol:.4}");
    }
    (faces.len(), fallback)
}

/// Area of the disc of radius `r` centred at (`cx`, `cy`) that lies inside the
/// box footprint 0..30 x 0..18, by direct integration over x. The integrand
/// has vertical tangents at the disc's extremes, so use many samples rather
/// than a high-order rule.
fn overlap_area(cx: f64, cy: f64, r: f64) -> f64 {
    const N: usize = 4_000_000;
    let (x0, x1) = ((cx - r).max(0.0), (cx + r).min(30.0));
    if x1 <= x0 {
        return 0.0;
    }
    let dx = (x1 - x0) / N as f64;
    let mut acc = 0.0;
    for k in 0..N {
        let x = (k as f64 + 0.5).mul_add(dx, x0);
        let half = (r * r - (x - cx) * (x - cx)).max(0.0).sqrt();
        let lo = (cy - half).max(0.0);
        let hi = (cy + half).min(18.0);
        acc += (hi - lo).max(0.0);
    }
    acc * dx
}

fn verify_one(cx: f64, cy: f64, cz: f64, h: f64) -> Result<(), String> {
    let r = 6.0;
    let mut topo = Topology::new();
    let bx = primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
    let cyl = primitives::make_cylinder(&mut topo, r, h).unwrap();
    transform_solid(&mut topo, cyl, &Mat4::translation(cx, cy, cz)).unwrap();
    let result = boolean(&mut topo, BooleanOp::Fuse, bx, cyl).unwrap();

    let faces = solid_faces(&topo, result).unwrap();
    let curved = faces
        .iter()
        .filter(|f| topo.face(**f).unwrap().surface().type_tag() != "plane")
        .count();
    if curved == 0 {
        return Err(format!("mesh fallback ({} all-planar faces)", faces.len()));
    }

    // Closed manifold: every edge used exactly twice.
    let mut usage: std::collections::HashMap<brepkit_topology::edge::EdgeId, usize> =
        std::collections::HashMap::new();
    for &fid in &faces {
        let f = topo.face(fid).unwrap();
        for w in std::iter::once(f.outer_wire()).chain(f.inner_wires().iter().copied()) {
            for oe in topo.wire(w).unwrap().edges() {
                *usage.entry(oe.edge()).or_default() += 1;
            }
        }
    }
    let free = usage.values().filter(|n| **n == 1).count();
    let nonman = usage.values().filter(|n| **n >= 3).count();
    if free > 0 || nonman > 0 {
        return Err(format!(
            "shell not closed manifold: {free} free, {nonman} non-manifold"
        ));
    }

    // Volume against the closed form.
    let z_lo = cz.max(0.0);
    let z_hi = (cz + h).min(24.0);
    let overlap = overlap_area(cx, cy, r) * (z_hi - z_lo).max(0.0);
    let expect = 30.0 * 18.0 * 24.0 + std::f64::consts::PI * r * r * h - overlap;
    let got = brepkit_operations::measure::solid_volume(&topo, result, 0.02).unwrap();
    let rel = (got - expect).abs() / expect;
    if rel > 1e-3 {
        return Err(format!(
            "volume {got:.4} vs closed form {expect:.4} (rel {rel:.2e})"
        ));
    }

    // Ray-cast classify a point inside the protruding wall band: just inside
    // the cylinder wall, outside the box in x or y, at mid-height of the
    // overlap. Only meaningful where the cylinder actually protrudes.
    let opts = brepkit_check::classify::ClassifyOptions::default();
    let zm = 0.5 * (z_lo + z_hi);
    for (px, py) in [
        (cx - 0.5 * r, cy),
        (cx, cy - 0.5 * r),
        (cx + 0.5 * r, cy),
        (cx, cy + 0.5 * r),
    ] {
        // Only probe points that are OUTSIDE the box, where the union must
        // still report solid because the cylinder is there.
        if px >= 0.0 && py >= 0.0 && px <= 30.0 && py <= 18.0 {
            continue;
        }
        let p = brepkit_math::vec::Point3::new(px, py, zm);
        let c = brepkit_check::classify::classify_point(&topo, result, p, &opts)
            .map_err(|e| format!("classify failed: {e}"))?;
        if c != brepkit_check::classify::PointClassification::Inside {
            return Err(format!(
                "point ({px:.2},{py:.2},{zm:.2}) in the protruding wall classified {c:?}"
            ));
        }
    }
    Ok(())
}

fn main() {
    env_logger::init();
    let mode = std::env::args().nth(1).unwrap_or_default();

    if mode == "sweep" {
        // Same cx/cy grid under three z-layouts. If coplanarity were the
        // trigger, only the first column would fall back.
        for (cz, h, tag) in [
            (0.0, 24.0, "coplanar cz=0 h=24"),
            (0.0, 30.0, "h=30"),
            (-3.0, 24.0, "cz=-3"),
        ] {
            let mut fb = 0;
            let mut tot = 0;
            let mut corner_fb = 0;
            for cxi in -5..=5 {
                for cy in [-2.0, 0.0, 4.0] {
                    let cx = f64::from(cxi);
                    let (n, f) = run(cx, cy, cz, h, false);
                    tot += 1;
                    if f {
                        fb += 1;
                        if cx.hypot(cy) < 6.0 {
                            corner_fb += 1;
                        }
                        eprintln!("  FALLBACK cx={cx} cy={cy} cz={cz} h={h} faces={n}");
                    }
                }
            }
            eprintln!("{tag}: {fb}/{tot} fallbacks ({corner_fb} of them corner-swallowing)");
        }
        return;
    }

    if mode == "single" {
        // Cylinder crosses ONLY the x=0 face: cy in (6,12) keeps it clear of
        // both y walls, so no corner is involved.
        for cyi in [7.0, 8.0, 9.0, 10.0, 11.0] {
            for cxi in [-4.0, -2.0, 0.0, 2.0, 4.0] {
                let (n, f) = run(cxi, cyi, 0.0, 24.0, false);
                eprintln!(
                    "cx={cxi} cy={cyi}: faces={n} {}",
                    if f { "FALLBACK" } else { "ok" }
                );
            }
        }
        return;
    }

    if mode == "corner" {
        // Does "cylinder swallows the box's vertical corner edge at (0,0)"
        // predict failure, and does breaking cap coplanarity rescue it?
        let cases: [(f64, f64); 6] = [
            (-4.0, 4.0),
            (-5.0, 4.0),
            (0.0, 0.0),
            (5.0, 4.0),
            (-3.0, 3.0),
            (-4.5, 4.5),
        ];
        for (cx, cy) in cases {
            let d = (cx * cx + cy * cy).sqrt();
            for (cz, h, tag) in [
                (0.0, 24.0, "coplanar"),
                (0.0, 30.0, "h=30"),
                (-3.0, 24.0, "cz=-3"),
            ] {
                let (n, f) = run(cx, cy, cz, h, false);
                eprintln!(
                    "cx={cx} cy={cy} cornerDist={d:.3} (inside={}) {tag}: faces={n} {}",
                    d < 6.0,
                    if f { "FALLBACK" } else { "ok" }
                );
            }
        }
        return;
    }

    if mode == "verify" {
        // No-fallback proves nothing on its own. Check each placement against
        // the closed form, against ray-cast classification of a point in the
        // protruding wall, and for a closed manifold shell.
        let mut bad = 0;
        let mut n = 0;
        for cxi in -5..=5 {
            for cy in [-2.0, 0.0, 4.0] {
                for (cz, h) in [(0.0, 24.0), (0.0, 30.0)] {
                    n += 1;
                    if let Err(e) = verify_one(f64::from(cxi), cy, cz, h) {
                        bad += 1;
                        eprintln!("BAD cx={cxi} cy={cy} cz={cz} h={h}: {e}");
                    }
                }
            }
        }
        eprintln!("\nverify: {}/{n} placements fully correct", n - bad);
        return;
    }

    if mode == "seam" {
        // The cylinder's seam sits at angle 0 (+x from its axis), i.e. at
        // (cx+r, cy). Rotating the cylinder about its own axis moves the seam
        // without changing the geometry at all. If the failure follows the
        // seam rather than the shape, that is the trigger.
        for deg in [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0] {
            let mut topo = Topology::new();
            let bx = primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
            let cyl = primitives::make_cylinder(&mut topo, 6.0, 24.0).unwrap();
            let rot = Mat4::rotation_z(f64::to_radians(deg));
            transform_solid(&mut topo, cyl, &rot).unwrap();
            transform_solid(&mut topo, cyl, &Mat4::translation(-4.0, 4.0, 0.0)).unwrap();
            let result = boolean(&mut topo, BooleanOp::Fuse, bx, cyl).unwrap();
            let faces = solid_faces(&topo, result).unwrap();
            let curved = faces
                .iter()
                .filter(|f| topo.face(**f).unwrap().surface().type_tag() != "plane")
                .count();
            let seam = (
                6.0_f64.mul_add(f64::to_radians(deg).cos(), -4.0),
                6.0_f64.mul_add(f64::to_radians(deg).sin(), 4.0),
            );
            eprintln!(
                "seam rot={deg:5.1}deg -> seam at ({:.2},{:.2}) inBox={} : {} faces {}",
                seam.0,
                seam.1,
                seam.0 >= 0.0 && seam.1 >= 0.0,
                faces.len(),
                if curved == 0 { "FALLBACK" } else { "ok" }
            );
        }
        return;
    }

    if mode == "raw" {
        // Below the operations acceptance gate: what does GFA itself emit?
        let cx: f64 = std::env::args().nth(2).map_or(-4.0, |s| s.parse().unwrap());
        let cy: f64 = std::env::args().nth(3).map_or(4.0, |s| s.parse().unwrap());
        let cz: f64 = std::env::args().nth(4).map_or(0.0, |s| s.parse().unwrap());
        let h: f64 = std::env::args().nth(5).map_or(24.0, |s| s.parse().unwrap());
        let mut topo = Topology::new();
        let bx = primitives::make_box(&mut topo, 30.0, 18.0, 24.0).unwrap();
        let cyl = primitives::make_cylinder(&mut topo, 6.0, h).unwrap();
        transform_solid(&mut topo, cyl, &Mat4::translation(cx, cy, cz)).unwrap();

        let res =
            brepkit_algo::gfa::boolean(&mut topo, brepkit_algo::bop::BooleanOp::Fuse, bx, cyl);
        match res {
            Err(e) => eprintln!("RAW GFA ERROR: {e:?}"),
            Ok(sol) => {
                let solids = [sol];
                eprintln!("RAW GFA: ok");
                for &s in &solids {
                    let faces = solid_faces(&topo, s).unwrap();
                    eprintln!("  solid {s:?}: {} faces {}", faces.len(), census(&topo, s));
                    // edge usage counts across the solid
                    let mut usage: std::collections::HashMap<
                        brepkit_topology::edge::EdgeId,
                        usize,
                    > = std::collections::HashMap::new();
                    for &fid in &faces {
                        let f = topo.face(fid).unwrap();
                        let mut wires = vec![f.outer_wire()];
                        wires.extend(f.inner_wires().iter().copied());
                        for w in wires {
                            for oe in topo.wire(w).unwrap().edges() {
                                *usage.entry(oe.edge()).or_default() += 1;
                            }
                        }
                    }
                    let free: Vec<_> = usage.iter().filter(|(_, n)| **n == 1).collect();
                    let nonman: Vec<_> = usage.iter().filter(|(_, n)| **n >= 3).collect();
                    eprintln!(
                        "  free edges={} non-manifold edges={}",
                        free.len(),
                        nonman.len()
                    );
                    for (e, n) in &free {
                        let ed = topo.edge(**e).unwrap();
                        let (a, b) = (
                            topo.vertex(ed.start()).unwrap().point(),
                            topo.vertex(ed.end()).unwrap().point(),
                        );
                        eprintln!(
                            "    FREE {e:?} n={n} ({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3}) curve={}",
                            a.x(),
                            a.y(),
                            a.z(),
                            b.x(),
                            b.y(),
                            b.z(),
                            ed.curve().type_tag()
                        );
                    }
                    for (e, n) in &nonman {
                        let ed = topo.edge(**e).unwrap();
                        let (a, b) = (
                            topo.vertex(ed.start()).unwrap().point(),
                            topo.vertex(ed.end()).unwrap().point(),
                        );
                        eprintln!(
                            "    NONMANIFOLD {e:?} n={n} ({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3}) curve={}",
                            a.x(),
                            a.y(),
                            a.z(),
                            b.x(),
                            b.y(),
                            b.z(),
                            ed.curve().type_tag()
                        );
                    }
                    for &fid in &faces {
                        let f = topo.face(fid).unwrap();
                        let plane = match f.surface() {
                            brepkit_topology::face::FaceSurface::Plane { normal, d } => format!(
                                " n=({:.2},{:.2},{:.2}) d={:.3}",
                                normal.x(),
                                normal.y(),
                                normal.z(),
                                d
                            ),
                            _ => String::new(),
                        };
                        eprintln!(
                            "    face {fid:?} {} rev={} inner={} area={:.4}{plane}",
                            f.surface().type_tag(),
                            f.is_reversed(),
                            f.inner_wires().len(),
                            brepkit_operations::measure::face_area(&topo, fid, 0.05)
                                .unwrap_or(-1.0)
                        );
                    }
                }
            }
        }
        return;
    }

    // minimal repro
    run(-4.0, 4.0, 0.0, 24.0, true);
}
