//! WASM contract test for `volume` across a growing-holder width change (B51).
//!
//! OpenZCAD's growing-holder recipe asserts that opening a countersunk
//! U-bracket from 44 to 10 mm changes `kernel.volume(solid, 0.001)` by
//! exactly the rebuilt floor section, 160 mm² × −34 = −5440 mm³. It read
//! −5411.3547: at width 10 the whole-solid mesh cracked (a planar CDT
//! Steiner point on an edge shared with another holed plane) and `volume`
//! fell through to the analytic rectangle over the NURBS-trimmed countersink
//! cones, 28.9 mm³ heavy, while width 44 measured on the closed mesh. This
//! drives the same construction through `executeBatch` (the JS surface) and
//! pins the width-to-width change and the closed-form volume.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use crate::kernel::BrepKernel;
use serde_json::{Value, json};

/// Run one batch operation and return its `ok` payload.
fn call(kernel: &mut BrepKernel, op: &str, args: Value) -> Value {
    let json = json!([{"op": op, "args": args}]).to_string();
    let out: Vec<Value> = serde_json::from_str(&kernel.execute_batch(&json)).unwrap();
    out[0]
        .get("ok")
        .cloned()
        .unwrap_or_else(|| panic!("{op} failed: {}", out[0]))
}

fn handle(v: &Value) -> u32 {
    u32::try_from(v.as_u64().unwrap_or_else(|| panic!("not a handle: {v}"))).unwrap()
}

/// A closed polygon face through `points` (line edges, closed wire,
/// planar face), as the adapter builds sketch profiles.
fn polygon_face(kernel: &mut BrepKernel, points: &[[f64; 3]]) -> u32 {
    let edges: Vec<u32> = (0..points.len())
        .map(|i| {
            let (a, b) = (points[i], points[(i + 1) % points.len()]);
            handle(&call(
                kernel,
                "makeLineEdge",
                json!({"x1": a[0], "y1": a[1], "z1": a[2], "x2": b[0], "y2": b[1], "z2": b[2]}),
            ))
        })
        .collect();
    let wire = handle(&call(
        kernel,
        "makeWire",
        json!({"edges": edges, "closed": true}),
    ));
    handle(&call(
        kernel,
        "makePlanarFaceFromWire",
        json!({"wire": wire}),
    ))
}

fn translation(x: f64, y: f64, z: f64) -> Value {
    json!([
        1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0
    ])
}

fn moved(kernel: &mut BrepKernel, solid: u32, matrix: Value) -> u32 {
    handle(&call(
        kernel,
        "copyAndTransformSolid",
        json!({"solid": solid, "matrix": matrix}),
    ))
}

/// The countersunk U-bracket of OpenZCAD's `syntheticHolderSolid`.
fn holder(kernel: &mut BrepKernel) -> u32 {
    let profile = [
        [0.0, 0.0, 0.0],
        [60.0, 0.0, 0.0],
        [60.0, 32.0, 0.0],
        [52.0, 32.0, 0.0],
        [52.0, 8.0, 0.0],
        [8.0, 8.0, 0.0],
        [8.0, 32.0, 0.0],
        [0.0, 32.0, 0.0],
    ];
    let face = polygon_face(kernel, &profile);
    let mut body = handle(&call(
        kernel,
        "extrude",
        json!({"face": face, "dx": 0.0, "dy": 0.0, "dz": 1.0, "distance": 20.0}),
    ));
    let half_tangent = (std::f64::consts::FRAC_PI_2 / 2.0).tan();
    let sink_depth = (4.5 - 2.5) / half_tangent;
    for x in [4.0, 56.0] {
        let section = [
            [0.0, 0.0, 0.0],
            [0.2_f64.mul_add(half_tangent, 4.5), 0.0, 0.0],
            [2.5, 0.0, 0.2 + sink_depth],
            [2.5, 0.0, 20.4],
            [0.0, 0.0, 20.4],
        ];
        let face = polygon_face(kernel, &section);
        let local = handle(&call(
            kernel,
            "revolve",
            json!({"face": face, "angle": 360.0}),
        ));
        let frame = json!([
            0.0, 1.0, 0.0, x, 1.0, 0.0, 0.0, 19.0, 0.0, 0.0, -1.0, 20.2, 0.0, 0.0, 0.0, 1.0
        ]);
        let tool = moved(kernel, local, frame);
        body = handle(&call(
            kernel,
            "cut",
            json!({"solidA": body, "solidB": tool}),
        ));
        call(kernel, "unifyFaces", json!({"solid": body}));
    }
    let boss = handle(&call(
        kernel,
        "makeBox",
        json!({"width": 0.4, "height": 6.0, "depth": 4.0}),
    ));
    let boss = moved(kernel, boss, translation(8.0, 14.0, 7.0));
    handle(&call(
        kernel,
        "fuse",
        json!({"solidA": body, "solidB": boss}),
    ))
}

/// The recipe along x at `width`: each end carved from a copy of the source
/// by its box mask and moved, the floor section rebuilt, one `fuseAll`, and
/// the unified union kept when it validates.
fn recipe(kernel: &mut BrepKernel, source: u32, width: f64) -> u32 {
    let mut piece = |negative: bool| {
        let copy = moved(kernel, source, translation(0.0, 0.0, 0.0));
        // The envelope (−0.5..60.5, 0..32, 0..20) grown by 1 + 5 % of 61.
        let (x0, x1) = if negative {
            (-4.55, 12.0)
        } else {
            (48.0, 64.55)
        };
        let mask = handle(&call(
            kernel,
            "makeBox",
            json!({"width": x1 - x0, "height": 40.1, "depth": 28.1}),
        ));
        let mask = moved(kernel, mask, translation(x0, -4.05, -4.05));
        let end = handle(&call(
            kernel,
            "intersect",
            json!({"solidA": copy, "solidB": mask}),
        ));
        call(kernel, "unifyFaces", json!({"solid": end}));
        let shift = if negative { 44.0 - width } else { width - 44.0 } / 2.0;
        moved(kernel, end, translation(shift, 0.0, 0.0))
    };
    let negative = piece(true);
    let positive = piece(false);
    let offset = 12.0 + (44.0 - width) / 2.0;
    let section = polygon_face(
        kernel,
        &[
            [offset, 0.0, 0.0],
            [offset, 8.0, 0.0],
            [offset, 8.0, 20.0],
            [offset, 0.0, 20.0],
        ],
    );
    let bridge = handle(&call(
        kernel,
        "extrude",
        json!({"face": section, "dx": 1.0, "dy": 0.0, "dz": 0.0, "distance": width - 8.0}),
    ));
    let raw = handle(&call(
        kernel,
        "fuseAll",
        json!({"solids": [negative, bridge, positive]}),
    ));
    let candidate = moved(kernel, raw, translation(0.0, 0.0, 0.0));
    call(kernel, "unifyFaces", json!({"solid": candidate}));
    if call(kernel, "validateSolid", json!({"solid": candidate})) == json!(0) {
        candidate
    } else {
        raw
    }
}

/// Closed form at `width`: U prism − 2 bores − 2 countersink frusta clipped
/// to the 8 mm arms + boss + floor growth (see the native regression
/// `operations/tests/regress_volume_open_mesh_route.rs`).
fn closed_form(width: f64) -> f64 {
    let a = 4.0_f64;
    let clipped = |r: f64| {
        let s = r.mul_add(r, -a * a).sqrt();
        let l = (r + s).ln();
        (2.0 * r.powi(3) / 3.0).mul_add(
            (a / r).asin(),
            (2.0 * a / 3.0) * (r / 2.0).mul_add(s, a * a / 2.0 * l),
        ) + a * r.mul_add(s, -a * a * l)
    };
    let half_tangent = (std::f64::consts::FRAC_PI_2 / 2.0).tan();
    let sink = (std::f64::consts::PI * (a.powi(3) - 2.5_f64.powi(3)) / 3.0 + clipped(4.5)
        - clipped(a))
        * half_tangent;
    let bore = std::f64::consts::PI * 2.5 * 2.5 * (20.0 - 2.0 / half_tangent);
    160.0_f64.mul_add(width - 44.0, 864.0 * 20.0 - 2.0 * (bore + sink) + 9.6)
}

fn volume(kernel: &mut BrepKernel, solid: u32, deflection: f64) -> f64 {
    call(
        kernel,
        "volume",
        json!({"solid": solid, "deflection": deflection}),
    )
    .as_f64()
    .unwrap()
}

#[test]
fn growing_holder_volume_change_is_the_floor_section() {
    let mut kernel = BrepKernel::new();
    let source = holder(&mut kernel);
    let base = recipe(&mut kernel, source, 44.0);
    let narrow = recipe(&mut kernel, source, 10.0);

    for (solid, width) in [(base, 44.0), (narrow, 10.0)] {
        assert_eq!(
            call(&mut kernel, "validateSolid", json!({"solid": solid})),
            json!(0),
            "width {width}: strict validation"
        );
        let id = kernel.resolve_solid(solid).unwrap();
        for deflection in [0.05, 0.01, 0.001] {
            let mesh =
                remus_operations::tessellate::tessellate_solid(kernel.topo(), id, deflection)
                    .unwrap();
            assert!(
                remus_operations::tessellate::is_watertight(&mesh),
                "width {width}: open mesh at {deflection}"
            );
        }
        // OpenZCAD's measurement deflection and the fine one both land within
        // the closed mesh's chord budget of the closed form; the rectangle
        // the open mesh used to fall through to was 2.6e-3 heavy.
        let exact = closed_form(width);
        for deflection in [0.08, 0.001] {
            let v = volume(&mut kernel, solid, deflection);
            assert!(
                (v - exact).abs() <= 1e-4 * exact,
                "width {width}: volume {v} at {deflection} vs closed form {exact}"
            );
        }
    }

    let change = volume(&mut kernel, narrow, 0.001) - volume(&mut kernel, base, 0.001);
    assert!(
        (change + 5440.0).abs() <= 1e-3,
        "width 44 → 10 changed the volume by {change}, not the floor section's −5440"
    );
}
