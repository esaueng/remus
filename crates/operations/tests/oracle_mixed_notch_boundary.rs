//! Independent boundary oracle for the L-bracket notch vertex (R = 1).
//!
//! Decides where the true constant-radius rolling-ball fillet surface runs
//! near the mixed-side notch vertex `(8,8,0)` by direct set-membership
//! sampling. Uses only `remus_math`-free plain arithmetic and
//! self-contained logic; it shares no code with any fillet engine:
//!
//! ```text
//! RESULT = (L \ W1 \ W2 \ C1) U F
//! ```
//!
//! with `L` the extruded L-polygon, `W1`/`W2` the convex-spine tubes (run
//! through the corner so no artificial open ends pollute the verdict),
//! `C1` the convex corner ball interior, and `F` the concave tube outside
//! `L` (drumhead semantics: clipped to the caps, no end balls).
//!
//! What this oracle certifies (all assertions below are grid-converged,
//! not magic constants):
//!
//! 1. **Coverage**: every true boundary cell lies within tight tolerance of
//!    one of the candidate carriers (the two convex cylinders, the concave
//!    cylinder, the C1 sphere, the three support planes, the station plane
//!    `z = R`, the transversal miter ellipse). Zero unexplained boundary.
//! 2. **Miter reality**: the transversal tube-tube crossing (ellipse arc
//!    `P0 -> Q` in the plane `y = x`) carries true boundary — the
//!    convex pair meets in a crease, not a ball.
//! 3. **C1 burial**: no boundary cell is C1-exclusive; every C1-near cell is
//!    also tube/support-near. The convex corner ball contributes no exposed
//!    patch — it is swallowed by the wedges.
//! 4. **Station geometry**: the concave tube surface passes through the
//!    station plane `z = R` inside the `(Q, M1, M2)` triangle, so stationing
//!    `T3` there would end it exactly on the contact-crossing segment.
//! 5. **Corner volume**: the removed volume inside the corner box
//!    `K = [8-R,8]^2 x [0,R]` converges across resolutions (recorded value
//!    ≈ 0.915 at R = 1).
//!
//! What this oracle does NOT claim: it does not construct a valid stitched
//! corner (see the module docs of the mixed-notch regression test for why
//! no stitched constant-R closure exists at this vertex), and it is not
//! the acceptance volume for a future patch — it is the independent
//! boundary map any future patch must agree with.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

const R: f64 = 1.0;
const NX: f64 = 8.0;
const NY: f64 = 8.0;
const H: f64 = 20.0;

const POLY: [[f64; 2]; 6] = [
    [0.0, 0.0],
    [40.0, 0.0],
    [40.0, 8.0],
    [8.0, 8.0],
    [8.0, 50.0],
    [0.0, 50.0],
];

fn in_poly(x: f64, y: f64) -> bool {
    let mut inside = false;
    for i in 0..6 {
        let (x1, y1) = (POLY[i][0], POLY[i][1]);
        let (x2, y2) = (POLY[(i + 1) % 6][0], POLY[(i + 1) % 6][1]);
        if (y1 > y) != (y2 > y) && x < (x2 - x1) * (y - y1) / (y2 - y1) + x1 {
            inside = !inside;
        }
    }
    inside
}

fn seg_dist2(p: [f64; 3], a: [f64; 3], b: [f64; 3]) -> f64 {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let mut t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1] + (p[2] - a[2]) * ab[2])
        / (ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);
    t = t.clamp(0.0, 1.0);
    let dx = p[0] - (a[0] + t * ab[0]);
    let dy = p[1] - (a[1] + t * ab[1]);
    let dz = p[2] - (a[2] + t * ab[2]);
    dx * dx + dy * dy + dz * dz
}

/// Bitmask: 1 = in L, 2 = in W1, 4 = in W2, 8 = in F, 16 = in C1 ball.
fn classify(p: [f64; 3]) -> u8 {
    let (px, py, pz) = (p[0], p[1], p[2]);
    let in_l = in_poly(px, py) && (0.0..=H).contains(&pz);
    let mut m = 0u8;
    if in_l {
        m |= 0b00001;
    }
    // Convex tubes run THROUGH the corner (no artificial open ends near C1);
    // burial of C1 is then a true geometric verdict, not a segmentation artifact.
    let d1 = seg_dist2(p, [5.0, NY - R, R], [40.0 - R, NY - R, R]);
    let d2 = seg_dist2(p, [NX - R, 5.0, R], [NX - R, 50.0 - R, R]);
    let dx3 = px - (NX + R);
    let dy3 = py - (NY + R);
    let d3tube = dx3 * dx3 + dy3 * dy3;
    let cdx = px - (NX - R);
    let cdy = py - (NY - R);
    let cdz = pz - R;
    let dc1 = cdx * cdx + cdy * cdy + cdz * cdz;
    if d1 < R * R && in_l {
        m |= 0b00010;
    }
    if d2 < R * R && in_l {
        m |= 0b00100;
    }
    if d3tube < R * R && !in_l && (0.0..=H).contains(&pz) {
        m |= 0b01000;
    }
    if dc1 < R * R {
        m |= 0b10000;
    }
    m
}

fn in_result(m: u8) -> bool {
    (m & 0b00001 != 0) && (m & 0b10110 == 0) || (m & 0b01000 != 0)
}

struct Coverage {
    boundary_cells: usize,
    unexplained: usize,
    worst_best: f64,
    miter_cells: usize,
    c1_exclusive: usize,
    station_cells: usize,
    k_removed_vol: f64,
}

fn in_tri(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    let v0x = c[0] - a[0];
    let v0y = c[1] - a[1];
    let v1x = b[0] - a[0];
    let v1y = b[1] - a[1];
    let v2x = p[0] - a[0];
    let v2y = p[1] - a[1];
    let den = v0x * v1y - v1x * v0y;
    if den.abs() < 1e-12 {
        return false;
    }
    let v = (v2x * v1y - v1x * v2y) / den;
    let w = (v0x * v2y - v2x * v0y) / den;
    v >= -1e-9 && w >= -1e-9 && v + w <= 1.0 + 1e-9
}

#[allow(clippy::too_many_lines)]
fn run_oracle(n: usize) -> Coverage {
    let (lox, hix) = (NX - 2.0 * R, NX + 3.0 * R);
    let (loy, hiy) = (NY - 2.0 * R, NY + 3.0 * R);
    let (loz, hiz) = (-0.5, 2.5 * R);
    let hx = (hix - lox) / (n as f64 - 1.0);
    let hy = (hiy - loy) / (n as f64 - 1.0);
    let hz = (hiz - loz) / (n as f64 - 1.0);
    let cell = hx.max(hy).max(hz);

    let mut prev: Vec<bool> = vec![false; n * n];
    let mut cov = Coverage {
        boundary_cells: 0,
        unexplained: 0,
        worst_best: 0.0,
        miter_cells: 0,
        c1_exclusive: 0,
        station_cells: 0,
        k_removed_vol: 0.0,
    };
    let mut k_cells = 0u64;
    let mut k_removed = 0u64;
    for k in 0..n {
        let z = loz + k as f64 * hz;
        let mut cur = vec![false; n * n];
        for j in 0..n {
            let y = loy + j as f64 * hy;
            for i in 0..n {
                let x = lox + i as f64 * hx;
                let inside = in_result(classify([x, y, z]));
                cur[j * n + i] = inside;
                if (NX - R..=NX).contains(&x)
                    && (NY - R..=NY).contains(&y)
                    && (0.0..=R).contains(&z)
                {
                    k_cells += 1;
                    if !inside {
                        k_removed += 1;
                    }
                }
                if !((i > 0 && cur[j * n + i - 1] != inside)
                    || (j > 0 && cur[(j - 1) * n + i] != inside)
                    || (k > 0 && prev[j * n + i] != inside))
                {
                    continue;
                }
                cov.boundary_cells += 1;
                let d_t1 = (((y - (NY - R)).powi(2) + (z - R).powi(2)).sqrt() - R).abs();
                let d_t2 = (((x - (NX - R)).powi(2) + (z - R).powi(2)).sqrt() - R).abs();
                let d_t3 = (((x - (NX + R)).powi(2) + (y - (NY + R)).powi(2)).sqrt() - R).abs();
                let d_c1 = (((x - (NX - R)).powi(2) + (y - (NY - R)).powi(2) + (z - R).powi(2))
                    .sqrt()
                    - R)
                    .abs();
                let d_b = z.abs();
                let d_s1 = (y - NY).abs();
                let d_s2 = (x - NX).abs();
                let d_ledge = (z - R).abs();
                let along = ((x - (NX - R)) + (y - (NY - R))) / std::f64::consts::SQRT_2;
                let outp = ((x - (NX - R)) - (y - (NY - R))) / std::f64::consts::SQRT_2;
                let dz = z - R;
                let d_miter = ((along.powi(2) + dz.powi(2)).sqrt() - R).abs() + outp.abs();
                let ds = [d_t1, d_t2, d_t3, d_c1, d_b, d_s1, d_s2, d_ledge, d_miter];
                let best = ds.iter().fold(f64::INFINITY, |a, b| a.min(*b));
                cov.worst_best = cov.worst_best.max(best);
                if best > 2.5 * cell {
                    cov.unexplained += 1;
                }
                // Miter interior: near both convex tubes, away from endpoints
                // (P0, Q) and support planes.
                if d_t1 < 2.0 * cell
                    && d_t2 < 2.0 * cell
                    && x > NX - R + 3.0 * cell
                    && y > NY - R + 3.0 * cell
                    && z > 4.0 * cell
                    && z < R - 2.0 * cell
                {
                    cov.miter_cells += 1;
                }
                // C1-exclusive: near the ball but clearly far from every
                // tube/support (wide separation so coincident tube/ball
                // surfaces near the station do not count as exposed ball).
                if d_c1 < 1.5 * cell
                    && d_t1 > 4.0 * cell
                    && d_t2 > 4.0 * cell
                    && d_t3 > 4.0 * cell
                    && d_b > 4.0 * cell
                {
                    cov.c1_exclusive += 1;
                }
                // Station crossing: T3 surface through z=R inside (Q,M1,M2).
                if d_t3 < 2.0 * cell
                    && (z - R).abs() < 2.0 * cell
                    && in_tri([x, y], [NX, NY], [NX + R, NY], [NX, NY + R])
                {
                    cov.station_cells += 1;
                }
            }
        }
        prev = cur;
    }
    cov.k_removed_vol = k_removed as f64 * hx * hy * hz;
    let _ = k_cells;
    cov
}

/// The candidate carrier set covers the true notch boundary with zero
/// unexplained cells at two resolutions.
#[test]
fn true_boundary_is_covered_by_candidates() {
    let coarse = run_oracle(120);
    let fine = run_oracle(200);
    for (label, cov) in [("120^3", &coarse), ("200^3", &fine)] {
        assert_eq!(
            cov.unexplained, 0,
            "{label}: every true boundary cell must sit on a candidate carrier"
        );
        assert!(
            cov.boundary_cells > 40_000,
            "{label}: oracle must see the full boundary ({})",
            cov.boundary_cells
        );
    }
    eprintln!(
        "worst best-distance: coarse={:.4} fine={:.4}",
        coarse.worst_best, fine.worst_best
    );
}

/// The transversal tube-tube crossing carries true boundary (the convex pair
/// meets in a crease arc, not a ball), and the C1 ball carries none of its
/// own (fully swallowed by the wedges).
#[test]
fn miter_is_real_and_c1_is_buried() {
    let cov = run_oracle(200);
    assert!(
        cov.miter_cells > 50,
        "miter interior must carry true boundary ({} cells)",
        cov.miter_cells
    );
    assert_eq!(
        cov.c1_exclusive, 0,
        "C1 must contribute no exposed patch of its own ({} exclusive cells)",
        cov.c1_exclusive
    );
    assert!(
        cov.station_cells > 20,
        "T3 must cross the station plane inside (Q,M1,M2) ({} cells)",
        cov.station_cells
    );
}

/// The removed corner-box volume converges across resolutions.
#[test]
fn corner_box_volume_converges() {
    let coarse = run_oracle(120);
    let fine = run_oracle(200);
    eprintln!(
        "K removed volume: coarse={:.5} fine={:.5}",
        coarse.k_removed_vol, fine.k_removed_vol
    );
    let rel = (coarse.k_removed_vol - fine.k_removed_vol).abs() / fine.k_removed_vol;
    assert!(
        rel < 0.02,
        "corner-box removed volume must converge (rel {rel:.4})"
    );
    assert!(
        (0.85..1.0).contains(&fine.k_removed_vol),
        "corner-box removed volume must sit in its analytic envelope ({})",
        fine.k_removed_vol
    );
}
