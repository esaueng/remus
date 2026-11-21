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

#![cfg_attr(not(test), no_main)]

#[cfg(not(test))]
use libfuzzer_sys::fuzz_target;

mod invariants;
mod shapegen;

use arbitrary::Arbitrary;
use invariants as inv;
use remus_operations::blend_ops::{chamfer_v2, fillet_v2};
use remus_operations::draft::draft;
use remus_operations::shell_op::shell;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::explorer;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;
use shapegen::{BaseBody, Refusal};

/// Bodies past this size make a fuzz iteration a timeout report rather than a
/// correctness report.
const FACE_LIMIT: usize = 80;

/// Triangles on the base body's trimmed sphere and torus faces, at the
/// battery's watertightness deflection, past which an iteration is a timeout
/// report rather than a correctness report.
///
/// [`FACE_LIMIT`] does not bound the work on its own. `solid_volume` measures
/// a body whose torus face carries a hole by meshing the whole body at a
/// deflection relative to its size, and a trimmed doubly-curved face costs
/// several times more per triangle than an untrimmed one. Every volume oracle
/// below and the volume guard inside `fillet_v2` and `chamfer_v2` take such a
/// reading, about a dozen per blend case, so one iteration costs in proportion
/// to that trimmed area, not to the face count. The six-face seed
/// `fillet-after-box-fused-with-tilted-torus-meridian-bite` carries 74,482
/// triangles here on its bitten torus; it ran 62 to 92 s per iteration in the
/// weekly runs against `-timeout=10` until Fuzz Smoke 2026-11-15 reported it
/// (run 43708174365), and the first local fuzz session after a whole-mesh
/// limit found `fillet-after-torus-fused-with-box-through-the-tube`, at 52,338,
/// in under two minutes (5.4 to 6.1 s natively). The frustum seed below the
/// limit carries 22,154 and runs in 0.64 s. Untrimmed spheres and tori, and
/// cylinders and cones, measure on exact routes and are not counted.
const TRIMMED_CURVED_LIMIT: usize = 30_000;

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

#[cfg(not(test))]
fuzz_target!(|case: Case| {
    run_case(&case);
});

/// How far a case got before it stopped.
#[derive(Debug, PartialEq, Eq)]
enum Outcome {
    /// The generator or a base-body measurement declined, or the base body
    /// was not a solid or had too many faces: nothing to judge.
    Declined,
    /// The base body is past [`TRIMMED_CURVED_LIMIT`].
    OverTrimmedCurvedLimit,
    /// The modifier was called; its result, if any, went through the oracles.
    Modified,
}

fn run_case(case: &Case) -> Outcome {
    let mut topo = Topology::new();
    let body = match case.body.build(&mut topo) {
        Ok(s) => s,
        Err(Refusal::Engine(_) | Refusal::Degenerate) => return Outcome::Declined,
    };

    let Ok(before) = inv::census(&topo, body.solid) else {
        return Outcome::Declined;
    };
    if before.faces > FACE_LIMIT {
        return Outcome::Declined;
    }
    // The generator's own output must be a solid before a modifier is blamed
    // for anything. A malformed base body is a boolean finding, and the
    // boolean_tree target is where it gets reported.
    if before.free_edges > 0 || before.non_manifold_edges > 0 || before.orphan_edges > 0 {
        return Outcome::Declined;
    }
    if trimmed_curved_triangles(&topo, body.solid).is_some_and(|n| n > TRIMMED_CURVED_LIMIT) {
        return Outcome::OverTrimmedCurvedLimit;
    }
    let Some(v_before) = inv::measure(&topo, body.solid) else {
        return Outcome::Declined;
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
    Outcome::Modified
}

/// Whether `face` is a sphere or torus face cut by a boolean: it carries a
/// hole, or one of its edges is a NURBS section curve.
fn is_trimmed_curved(topo: &Topology, face: FaceId) -> Option<bool> {
    let face = topo.face(face).ok()?;
    if !matches!(
        face.surface(),
        FaceSurface::Sphere(_) | FaceSurface::Torus(_)
    ) {
        return Some(false);
    }
    if !face.inner_wires().is_empty() {
        return Some(true);
    }
    let wire = topo.wire(face.outer_wire()).ok()?;
    for used in wire.edges() {
        if matches!(
            topo.edge(used.edge()).ok()?.curve(),
            EdgeCurve::NurbsCurve(_)
        ) {
            return Some(true);
        }
    }
    Some(false)
}

/// Triangles on `solid`'s trimmed sphere and torus faces at the deflection
/// the watertightness oracle uses. `None` when a lookup or the mesher
/// declines; the gate then stays open, as if it did not exist. A body with
/// no such face is not meshed at all.
fn trimmed_curved_triangles(topo: &Topology, solid: SolidId) -> Option<usize> {
    let faces = explorer::solid_faces(topo, solid).ok()?;
    let trimmed = faces
        .iter()
        .map(|&face| is_trimmed_curved(topo, face))
        .collect::<Option<Vec<bool>>>()?;
    if !trimmed.contains(&true) {
        return Some(0);
    }
    let aabb = remus_operations::measure::solid_bounding_box(topo, solid).ok()?;
    let diag = (aabb.max - aabb.min).length();
    let deflection = inv::volume_deflection(diag) * 4.0;
    // One group per face of `solid_faces`, in that order, plus a sentinel.
    let (_, offsets) = remus_operations::tessellate::tessellate_solid_grouped_with_tolerance(
        topo,
        solid,
        deflection,
        remus_math::chord::DEFAULT_ANGULAR_TOL,
    )
    .ok()?;
    let mut count = 0;
    for (i, &is_trimmed) in trimmed.iter().enumerate() {
        if is_trimmed {
            let (start, end) = (*offsets.get(i)?, *offsets.get(i + 1)?);
            count += (end - start) as usize / 3;
        }
    }
    Some(count)
}

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
                inv::assert_option_honoured(
                    "fillet",
                    "radius r",
                    "radius r/2",
                    v_before,
                    a.volume,
                    b.volume,
                );
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

#[cfg(test)]
mod tests {
    use super::*;

    fn decode(data: &[u8]) -> Case {
        Case::arbitrary_take_rest(arbitrary::Unstructured::new(data)).unwrap()
    }

    fn base_trimmed_curved_triangles(case: &Case) -> usize {
        let mut topo = Topology::new();
        let body = case.body.build(&mut topo).ok().unwrap();
        trimmed_curved_triangles(&topo, body.solid).unwrap()
    }

    /// Fuzz Smoke 2026-11-15 (run 43708174365): the first seed ran 62 to 92 s
    /// per iteration in the weekly runs before one reported it as a timeout;
    /// the second is the 5 s sibling a local fuzz session found within two
    /// minutes of a whole-mesh limit. Both base bodies are past the limit, so
    /// each case must stop before any measurement or modifier runs: a work
    /// bound, not a wall-clock one.
    #[test]
    fn bitten_torus_fillet_seeds_stop_at_the_trimmed_curved_limit() {
        let seeds: [(&str, &[u8]); 2] = [
            (
                "fillet-after-box-fused-with-tilted-torus-meridian-bite",
                include_bytes!(
                    "../corpus/modifier_ops/fillet-after-box-fused-with-tilted-torus-meridian-bite"
                ),
            ),
            (
                "fillet-after-torus-fused-with-box-through-the-tube",
                include_bytes!(
                    "../corpus/modifier_ops/fillet-after-torus-fused-with-box-through-the-tube"
                ),
            ),
        ];
        for (name, data) in seeds {
            let case = decode(data);
            assert!(matches!(case.modifier, Modifier::Fillet { .. }), "{name}");
            let triangles = base_trimmed_curved_triangles(&case);
            assert!(
                triangles > TRIMMED_CURVED_LIMIT,
                "{name}: {triangles} triangles on trimmed curved faces, inside the limit"
            );
            assert_eq!(run_case(&case), Outcome::OverTrimmedCurvedLimit, "{name}");
        }
    }

    /// The limit must not quietly retire the rest of the corpus: every other
    /// committed seed whose base body builds stays under it.
    #[test]
    fn other_committed_seeds_stay_under_the_trimmed_curved_limit() {
        let seeds: [(&str, &[u8]); 6] = [
            (
                "box-fused-with-rotated-disjoint-sphere",
                include_bytes!("../corpus/modifier_ops/box-fused-with-rotated-disjoint-sphere"),
            ),
            (
                "draft-folded-facet",
                include_bytes!("../corpus/modifier_ops/draft-folded-facet"),
            ),
            (
                "fillet-after-torus-pierced-through-frustum-cap-and-wall",
                include_bytes!(
                    "../corpus/modifier_ops/fillet-after-torus-pierced-through-frustum-cap-and-wall"
                ),
            ),
            (
                "shell-box-cut-by-overlapping-sphere-thin-wall-collapse",
                include_bytes!(
                    "../corpus/modifier_ops/shell-box-cut-by-overlapping-sphere-thin-wall-collapse"
                ),
            ),
            (
                "shell-box-fused-with-oblique-cylinder-boss-thin-wall-collapse",
                include_bytes!(
                    "../corpus/modifier_ops/shell-box-fused-with-oblique-cylinder-boss-thin-wall-collapse"
                ),
            ),
            (
                "shell-box-fused-with-rotated-disjoint-sphere",
                include_bytes!(
                    "../corpus/modifier_ops/shell-box-fused-with-rotated-disjoint-sphere"
                ),
            ),
        ];
        for (name, data) in seeds {
            let triangles = base_trimmed_curved_triangles(&decode(data));
            assert!(
                triangles <= TRIMMED_CURVED_LIMIT,
                "{name}: {triangles} triangles on trimmed curved faces, past the limit"
            );
        }
    }
}
