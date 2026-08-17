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
//! Cap coplanarity alone is not the trigger; the family needs BOTH the box's
//! corner edge inside the cylinder AND at least one cap flush with a box face.
//! Both-flush is only one of three variants — top-flush (hanging below) and
//! bottom-flush (sticking above) fail identically, and the reported document is
//! top-flush. `matrix` shows the whole picture at a glance.
//!
//! What `matrix` still marks failing is a DIFFERENT, pre-existing defect: it
//! also reproduces with no flush cap at all. A box side-face plane is parallel
//! to the cylinder axis, so it meets the cylinder in two straight generators
//! rather than an ellipse, and `exact_plane_cylinder`
//! (`math/src/analytic_intersection.rs`) has no arm for that — it drops to a
//! sampled point chain. Within about r/600 of the axis lying ON the plane the
//! chain degrades and the fuse loses the protrusion outright. The acceptance
//! gate catches every one of those (`operands_are_represented`: the result's
//! bounding box no longer contains the cylinder's), so the output is faceted
//! but its volume is right.
//!
//! Modes:
//!   (none)  the minimal reported repro, with its face census and volume
//!   matrix  post-gate pass/fail table over six z-layouts x the cx/cy grid
//!   family  the three flush-cap variants, verified, plus the controls that
//!           must stay analytic and the reported document
//!   one     a single placement through the real operations gate, verbosely
//!   sweep   the cx x cy grid under three z-layouts, listing every fallback
//!   single  crossings of ONE side face only (these always worked)
//!   corner  corner-swallowing vs not, against cap coplanarity
//!   seam    the same solid with the cylinder rotated about its own axis
//!   raw     below the operations gate: `gfa::boolean` output, free and
//!           non-manifold edges, per-face area and plane, the result's bbox
//!           and volume, and the horizontal-area balance that names a missing
//!           cap
//!   verify  volume vs the closed form, closed-manifold shell, and ray-cast
//!           classification of a point in the protruding wall

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::primitives;
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::explorer::solid_faces;

fn census(topo: &Topology, sid: remus_topology::arena::Id<remus_topology::solid::Solid>) -> String {
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
        let vol = remus_operations::measure::solid_volume(&topo, result, 0.05).unwrap();
        eprintln!("  volume = {vol:.4}");
    }
    (faces.len(), fallback)
}

/// Area of the disc of radius `r` centred at (`cx`, `cy`) that lies inside the
/// box footprint 0..30 x 0..18, by direct integration over x. The integrand
/// has vertical tangents at the disc's extremes, so use many samples rather
/// than a high-order rule.
fn overlap_area(cx: f64, cy: f64, r: f64) -> f64 {
    const N: usize = 200_000;
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
    let mut usage: std::collections::HashMap<remus_topology::edge::EdgeId, usize> =
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
    let got = remus_operations::measure::solid_volume(&topo, result, 0.02).unwrap();
    let rel = (got - expect).abs() / expect;
    if rel > 1e-3 {
        return Err(format!(
            "volume {got:.4} vs closed form {expect:.4} (rel {rel:.2e})"
        ));
    }

    // Ray-cast classify a point inside the protruding wall band: just inside
    // the cylinder wall, outside the box in x or y, at mid-height of the
    // overlap. Only meaningful where the cylinder actually protrudes.
    let opts = remus_check::classify::ClassifyOptions::default();
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
        let p = remus_math::vec::Point3::new(px, py, zm);
        let c = remus_check::classify::classify_point(&topo, result, p, &opts)
            .map_err(|e| format!("classify failed: {e}"))?;
        if c != remus_check::classify::PointClassification::Inside {
            return Err(format!(
                "point ({px:.2},{py:.2},{zm:.2}) in the protruding wall classified {c:?}"
            ));
        }
    }
    Ok(())
}

/// The corrected failing family: the wall must cross the box's CORNER (two
/// adjacent side faces), and at LEAST ONE cap must be flush with a box face.
/// Both-flush is only one of three variants.
fn family_sweep() {
    let configs: [(f64, f64, &str); 3] = [
        (0.0, 24.0, "both-flush   (cz=0  h=24)"),
        (-6.0, 30.0, "top-flush    (cz=-6 h=30)"),
        (0.0, 30.0, "bottom-flush (cz=0  h=30)"),
    ];
    let mut facet = 0;
    let mut wrong = 0;
    let mut total = 0;
    for (cz, h, tag) in configs {
        let mut bad: Vec<String> = Vec::new();
        for cx in [-4.0, -2.0, 0.0, 2.0] {
            for cy in [4.0, 0.0, -2.0] {
                total += 1;
                let (n, fb) = run(cx, cy, cz, h, false);
                if fb {
                    facet += 1;
                    bad.push(format!("({cx},{cy}) FACET {n}f"));
                } else if let Err(e) = verify_one(cx, cy, cz, h) {
                    wrong += 1;
                    bad.push(format!("({cx},{cy}) {e}"));
                }
            }
        }
        if bad.is_empty() {
            eprintln!("{tag}: all 12 analytic and exact");
        } else {
            eprintln!("{tag}: {} bad -> {}", bad.len(), bad.join("; "));
        }
    }
    eprintln!(
        "\nfamily: {}/{total} correct ({facet} faceted, {wrong} wrong geometry)",
        total - facet - wrong
    );

    // Controls that must STAY analytic.
    eprintln!("\n-- controls --");
    let (n, fb) = run(-4.0, 9.0, 0.0, 24.0, false);
    eprintln!(
        "centred cy=9, both caps flush, ONE side face crossed: {n} faces {}",
        if fb { "FACET (control broken)" } else { "ok" }
    );
    for (cz, h) in [(-3.0, 24.0), (-3.0, 20.0)] {
        let mut bad = Vec::new();
        for cx in [-4.0, -2.0, 0.0, 2.0] {
            for cy in [4.0, 0.0, -2.0] {
                let (n, fb) = run(cx, cy, cz, h, false);
                if fb {
                    bad.push(format!("({cx},{cy}) {n}f"));
                }
            }
        }
        eprintln!(
            "no flush cap (cz={cz} h={h}), corner crossed: {}",
            if bad.is_empty() {
                "all 12 analytic".to_string()
            } else {
                format!("{} FACET -> {}", bad.len(), bad.join(" "))
            }
        );
    }

    // The user's live case.
    eprintln!("\n-- user's reported document --");
    let (n, fb) = run(-4.0, 4.0, -6.0, 30.0, false);
    eprintln!(
        "box(30,18,24) U cyl(r6,h30)@(-4,4,-6): {n} faces {}",
        if fb { "FACET" } else { "ok" }
    );
    match verify_one(-4.0, 4.0, -6.0, 30.0) {
        Ok(()) => eprintln!("  volume, shell and ray-cast all exact"),
        Err(e) => eprintln!("  WRONG: {e}"),
    }
}

/// One post-gate table over every z-layout, so the flush-cap family and the
/// no-flush controls are measured the same way. `.` analytic, `X` fallback.
fn matrix() {
    let configs: [(f64, f64, &str); 6] = [
        (0.0, 24.0, "both-flush      cz= 0 h=24"),
        (-6.0, 30.0, "top-flush       cz=-6 h=30"),
        (0.0, 30.0, "bottom-flush    cz= 0 h=30"),
        (-3.0, 30.0, "no-flush thru   cz=-3 h=30"),
        (-3.0, 24.0, "no-flush inside cz=-3 h=24"),
        (-3.0, 20.0, "no-flush inside cz=-3 h=20"),
    ];
    let cys = [4.0, 0.0, -2.0];
    let cxs = [-4.0, -2.0, 0.0, 2.0];
    let mut fb_total = 0;
    let mut total = 0;
    eprintln!("                              cy=4        cy=0        cy=-2");
    eprintln!("                            {}", "cx -4-2 0 2  ".repeat(3));
    for (cz, h, tag) in configs {
        let mut row = String::new();
        for cy in cys {
            for cx in cxs {
                let (_, fb) = run(cx, cy, cz, h, false);
                total += 1;
                if fb {
                    fb_total += 1;
                }
                row.push(if fb { 'X' } else { '.' });
                row.push(' ');
            }
            row.push_str("  ");
        }
        eprintln!("{tag}  {row}");
    }
    eprintln!("\ntotal {fb_total}/{total} fall back");
}

fn main() {
    env_logger::init();
    let mode = std::env::args().nth(1).unwrap_or_default();

    if mode == "matrix" {
        matrix();
        return;
    }

    // One placement through the real operations gate, verbosely.
    if mode == "one" {
        let a =
            |n: usize, d: f64| -> f64 { std::env::args().nth(n).map_or(d, |s| s.parse().unwrap()) };
        let (cx, cy, cz, h) = (a(2, -4.0), a(3, 0.0), a(4, -6.0), a(5, 30.0));
        run(cx, cy, cz, h, true);
        return;
    }

    if mode == "family" {
        family_sweep();
        return;
    }

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

        let res = remus_algo::gfa::boolean(&mut topo, remus_algo::bop::BooleanOp::Fuse, bx, cyl);
        match res {
            Err(e) => eprintln!("RAW GFA ERROR: {e:?}"),
            Ok(sol) => {
                let solids = [sol];
                eprintln!("RAW GFA: ok");
                {
                    let bb = remus_operations::measure::solid_bounding_box(&topo, sol).unwrap();
                    let vol = remus_operations::measure::solid_volume(&topo, sol, 0.02).unwrap();
                    let lens = overlap_area(cx, cy, 6.0);
                    let expect = 30.0 * 18.0 * 24.0 + std::f64::consts::PI * 36.0 * h
                        - lens * ((cz + h).min(24.0) - cz.max(0.0)).max(0.0);
                    eprintln!(
                        "  bbox min=({:.2},{:.2},{:.2}) max=({:.2},{:.2},{:.2})",
                        bb.min.x(),
                        bb.min.y(),
                        bb.min.z(),
                        bb.max.x(),
                        bb.max.y(),
                        bb.max.z()
                    );
                    eprintln!(
                        "  volume={vol:.3} closed-form={expect:.3} delta={:.3}",
                        vol - expect
                    );
                    // Divergence theorem in z: signed area must cancel on a
                    // closed shell. A non-zero sum names a missing horizontal cap.
                    let mut up = 0.0;
                    let mut down = 0.0;
                    for fid in solid_faces(&topo, sol).unwrap() {
                        let f = topo.face(fid).unwrap();
                        if let remus_topology::face::FaceSurface::Plane { normal, .. } = f.surface()
                        {
                            let nz = normal.z() * if f.is_reversed() { -1.0 } else { 1.0 };
                            let ar =
                                remus_operations::measure::face_area(&topo, fid, 0.02).unwrap();
                            if nz > 0.5 {
                                up += ar;
                            } else if nz < -0.5 {
                                down += ar;
                            }
                        }
                    }
                    eprintln!(
                        "  horizontal area up={up:.3} down={down:.3} imbalance={:.3} (pi*r^2={:.3})",
                        up - down,
                        std::f64::consts::PI * 36.0
                    );
                }
                for &s in &solids {
                    let faces = solid_faces(&topo, s).unwrap();
                    eprintln!("  solid {s:?}: {} faces {}", faces.len(), census(&topo, s));
                    // edge usage counts across the solid
                    let mut usage: std::collections::HashMap<remus_topology::edge::EdgeId, usize> =
                        std::collections::HashMap::new();
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
                            remus_topology::face::FaceSurface::Plane { normal, d } => format!(
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
                            remus_operations::measure::face_area(&topo, fid, 0.05).unwrap_or(-1.0)
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
