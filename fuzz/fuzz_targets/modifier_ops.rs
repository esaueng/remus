//! Structured fuzzing of the blend and modifier operations.
//!
//! Builds a bored or bossed body — a primitive with a boolean feature already
//! cut into it — then applies one modifier: fillet, chamfer, shell or draft.
//!
//! The bore is load-bearing. Five of the fourteen defects this harness exists
//! for were the same failure in five different operations: the modifier
//! silently dropped the body's inner wires, and the solid with a bore came
//! back with the bore filled. An unbored test body cannot express that
//! failure, so the generator produces one *with* a hole in seven cases out of
//! eight, and the hole-preservation oracle is the first thing checked after
//! every modifier.

#![no_main]

use libfuzzer_sys::fuzz_target;

mod invariants;
mod shapegen;

use arbitrary::Arbitrary;
use remus_operations::blend_ops::{chamfer_v2, fillet_v2};
use remus_operations::draft::draft;
use remus_operations::shell_op::shell;
use remus_topology::Topology;
use remus_topology::explorer;
use remus_topology::solid::SolidId;
use invariants as inv;
use shapegen::{BaseBody, Refusal};

/// Bodies past this size make a fuzz iteration a timeout report rather than a
/// correctness report.
const FACE_LIMIT: usize = 80;

/// Which modifier to apply, and with what.
#[derive(Debug, Arbitrary)]
enum Modifier {
    /// Constant-radius fillet on a chosen edge subset.
    Fillet { pick: u8, stride: u8, radius: u8 },
    /// Two-distance chamfer on a chosen edge subset.
    Chamfer {
        pick: u8,
        stride: u8,
        d1: u8,
        d2: u8,
    },
    /// Hollow the body, optionally opening some faces.
    Shell {
        thickness: u8,
        open: u8,
        stride: u8,
        open_count: u8,
    },
    /// Taper a chosen face subset about a neutral plane.
    Draft {
        pick: u8,
        stride: u8,
        pull: u8,
        angle: u8,
    },
}

#[derive(Debug, Arbitrary)]
struct Case {
    body: BaseBody,
    modifier: Modifier,
}

/// Blend magnitudes stay small relative to the half-unit feature lattice, so
/// the interesting refusals (radius too large, blend runs into a hole) are
/// reachable but do not dominate.
fn small(b: u8) -> f64 {
    0.05 + f64::from(b % 12) * 0.05
}

fuzz_target!(|case: Case| {
    let mut topo = Topology::new();
    let body = match case.body.build(&mut topo) {
        Ok(s) => s,
        Err(Refusal::Engine(_) | Refusal::Degenerate) => return,
    };

    let Ok(before) = inv::census(&topo, body.solid) else {
        return;
    };
    if before.faces > FACE_LIMIT {
        return;
    }
    // The generator's own output must be a solid before a modifier is blamed
    // for anything. A malformed base body is a boolean finding, and the
    // boolean_tree target is where it gets reported.
    if before.free_edges > 0 || before.non_manifold_edges > 0 || before.orphan_edges > 0 {
        return;
    }
    let Some(v_before) = inv::measure(&topo, body.solid) else {
        return;
    };

    // When the stock and the bore were interior-disjoint the base body's volume
    // is known by construction, so the body is checked against a closed form
    // before any modifier gets the blame for what comes out of it.
    if let Some(expected) = body.exact {
        inv::assert_exact_volume("base body", expected, v_before.volume);
    }

    apply(
        &mut topo,
        body.solid,
        &case.modifier,
        &before,
        v_before.volume,
    );
});

fn apply(
    topo: &mut Topology,
    body: SolidId,
    modifier: &Modifier,
    before: &inv::Census,
    v_before: f64,
) {
    match modifier {
        Modifier::Fillet {
            pick,
            stride,
            radius,
        } => {
            let Ok(edges) = explorer::solid_edges(topo, body) else {
                return;
            };
            let sel = shapegen::pick_subset(&edges, *pick, *stride, 4);
            if sel.is_empty() {
                return;
            }
            let r1 = small(*radius);
            let Ok(result) = fillet_v2(topo, body, &sel, r1) else {
                return; // RadiusTooLarge, UnsupportedVertexBlend, ... — all passes
            };
            // I8 — a blend that did only some of the edges must be an error,
            // not an Ok carrying a silent subset (#44).
            inv::assert_complete(
                "fillet",
                sel.len(),
                result.succeeded.len(),
                result.is_partial,
            );
            // I8b — the radius is an option, and an option must be honoured.
            // A second fillet at half the radius must not land on the same
            // solid (#52's corner mode accepted the request and ignored it).
            // Run on a clone so the comparison cannot disturb the result.
            let mut alt = topo.clone();
            if let Ok(other) = fillet_v2(&mut alt, body, &sel, r1 * 0.5)
                && !other.is_partial
                && let (Some(a), Some(b)) = (
                    inv::measure(topo, result.solid),
                    inv::measure(&alt, other.solid),
                )
            {
                inv::assert_option_honoured("fillet", "radius r", "radius r/2", a.volume, b.volume);
            }
            finish(topo, "fillet", result.solid, before, v_before, Growth::Any);
        }

        Modifier::Chamfer {
            pick,
            stride,
            d1,
            d2,
        } => {
            let Ok(edges) = explorer::solid_edges(topo, body) else {
                return;
            };
            let sel = shapegen::pick_subset(&edges, *pick, *stride, 4);
            if sel.is_empty() {
                return;
            }
            let Ok(result) = chamfer_v2(topo, body, &sel, small(*d1), small(*d2)) else {
                return;
            };
            inv::assert_complete(
                "chamfer",
                sel.len(),
                result.succeeded.len(),
                result.is_partial,
            );
            finish(topo, "chamfer", result.solid, before, v_before, Growth::Any);
        }

        Modifier::Shell {
            thickness,
            open,
            stride,
            open_count,
        } => {
            let Ok(faces) = explorer::solid_faces(topo, body) else {
                return;
            };
            // Half the cases hollow a closed body; half open one or two faces.
            let open_faces = if open_count % 2 == 0 {
                Vec::new()
            } else {
                shapegen::pick_subset(&faces, *open, *stride, 2)
            };
            let Ok(result) = shell(topo, body, small(*thickness), &open_faces) else {
                return; // Unsupported is the documented refusal here
            };
            // Hollowing must carry every inner wire onto BOTH skins, so the
            // count can only rise. #48 returned a body with the bores gone and
            // the shell open, and passed every check it was given.
            finish(topo, "shell", result, before, v_before, Growth::Shrinks);
        }

        Modifier::Draft {
            pick,
            stride,
            pull,
            angle,
        } => {
            let Ok(faces) = explorer::solid_faces(topo, body) else {
                return;
            };
            let sel = shapegen::pick_subset(&faces, *pick, *stride, 3);
            if sel.is_empty() {
                return;
            }
            let Ok(center) = shapegen::body_center(topo, body) else {
                return;
            };
            let dir = shapegen::axis_dir(*pull);
            // 1..=12 degrees: a real taper, well short of collapsing a wall.
            let radians = f64::from(1 + u32::from(angle % 12)).to_radians();
            let Ok(result) = draft(topo, body, &sel, dir, center, radians) else {
                return;
            };
            finish(topo, "draft", result, before, v_before, Growth::Any);
        }
    }
}

/// How the modifier is allowed to move the volume.
enum Growth {
    /// Any volume, as long as the two integrators agree about it.
    Any,
    /// The result must not be larger than the input — hollowing removes
    /// material, it never adds any.
    Shrinks,
}

/// The oracle battery every modifier result goes through.
fn finish(
    topo: &Topology,
    what: &str,
    result: SolidId,
    before: &inv::Census,
    v_before: f64,
    growth: Growth,
) {
    let Ok(after) = inv::census(topo, result) else {
        return;
    };

    // I1/I2 — draft (#41), chamfer (#43) and shell (#48) each returned an open
    // or Euler-inconsistent shell that every existing check accepted.
    inv::assert_closed_manifold(what, &after);

    // I3 — the defect class that hit five separate operations.
    inv::assert_holes_preserved(what, before, &after);

    if after.faces > FACE_LIMIT * 4 {
        return;
    }

    if let Ok(aabb) = remus_operations::measure::solid_bounding_box(topo, result) {
        let diag = (aabb.max - aabb.min).length();
        inv::assert_watertight_mesh(what, topo, result, inv::volume_deflection(diag) * 4.0);
    }

    // I5b — a modifier's own tolerances are the likeliest place for an absolute
    // distance to hide, because a blend radius and a shell thickness are both
    // lengths. #51's provenance budget reported surviving faces as deleted at
    // 1000x, which is this check's exact signature: the census moved with size.
    inv::assert_scale_invariant(what, topo, result, 1000.0);

    let Some(m) = inv::measure(topo, result) else {
        return;
    };

    // I4 — a modifier may not invent material out of nothing.
    if let Growth::Shrinks = growth {
        let slack = v_before.abs().mul_add(inv::VOL_SLACK, inv::VOL_FLOOR);
        assert!(
            m.volume <= v_before + slack,
            "{what}: hollowing produced {:.6} from a solid of {:.6} — \
             removing material cannot increase the volume",
            m.volume,
            v_before,
        );
    }

    // I5 — the two volume integrators must agree about whatever it built.
    inv::assert_measurements_agree(what, topo, result, m.volume);
}
