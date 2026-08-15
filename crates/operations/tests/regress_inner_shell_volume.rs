//! `solid_volume` must measure the body, not just its outer shell.
//!
//! `measure::solid_volume` tries a ladder of fast paths. The first,
//! `try_analytic_solid_volume`, recognises whole primitives — sphere, cylinder,
//! cone, torus plus planar caps — and returns their closed form. It read the
//! face list from `solid.outer_shell()` alone, so a body hollowed by a cavity
//! (an inner shell, which is what `Cut` builds for a fully contained tool) still
//! matched the recogniser on its intact outer wall and was reported at its
//! un-hollowed volume. An r10 h20 cylinder with an enclosed r4 h8 void read
//! 6283.185307179587 — exactly `pi*10^2*20` — against a true 5881.061447520093,
//! a silent 6.8% overstatement on a valid solid.
//!
//! Four more paths down the ladder summed per-face contributions over the outer
//! shell only, so they would have dropped the same cavity had routing reached
//! them; they enumerate `explorer::solid_faces` now, matching what
//! `remus_check::properties` has always done.
//!
//! Every expected value here is a hand closed form — outer primitive minus void
//! — composed from the dimension constants the model is built from. Nothing is
//! a recorded kernel measurement, and `mass_properties` is never used as the
//! reference for `solid_volume`: the two meet in `integrate_face`, so their
//! agreement proves nothing. (They are printed side by side only as a report.)
//!
//! Every case runs at 1x, 1000x and 0.001x. The tolerances are relative, so a
//! result that genuinely tracks the model is scale-invariant; each case's error
//! is in fact bit-identical across the three scales.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::f64::consts::PI;

use remus_math::mat::Mat4;
use remus_operations::boolean::{BooleanOp, boolean};
use remus_operations::measure::{mass_properties, solid_volume};
use remus_operations::primitives::{make_box, make_cylinder, make_sphere};
use remus_operations::transform::transform_solid;
use remus_topology::Topology;
use remus_topology::solid::SolidId;

/// One model: the solid, its hand closed form, how many shells it must have,
/// and the relative tolerance its route can hold.
struct Case {
    solid: SolidId,
    closed_form: f64,
    shells: usize,
    tol: f64,
}

/// Exact: the whole boundary integrates in closed form.
const EXACT: f64 = 1e-12;
/// A curved cavity wall reached through tessellation. Bounds the inscribed-mesh
/// undercount at the preview deflection used below; far tighter than the 6.8%
/// the defect produced, so it still pins the regression.
const TESSELLATED: f64 = 5e-4;

fn at(topo: &mut Topology, s: SolidId, x: f64, y: f64, z: f64) -> SolidId {
    transform_solid(topo, s, &Mat4::translation(x, y, z)).unwrap();
    s
}

/// Attach `tool`'s shell to `blank` as a cavity, reversing every tool face so
/// the cavity boundary faces into the void — exactly what the boolean's
/// `build_contained_cut_hollow` does for a fully contained tool. Building it
/// directly keeps a case on the measurement path when the boolean itself has
/// unrelated limits (concentric quadrics exceed its mesh work budget).
fn hollow_with(topo: &mut Topology, blank: SolidId, tool: SolidId) -> SolidId {
    let cavity = topo.solid(tool).unwrap().outer_shell();
    for fid in topo.shell(cavity).unwrap().faces().to_vec() {
        let f = topo.face_mut(fid).unwrap();
        let flipped = !f.is_reversed();
        f.set_reversed(flipped);
    }
    topo.solid_mut(blank).unwrap().add_inner_shell(cavity);
    blank
}

// ── The models ────────────────────────────────────────────────

/// The reported defect: r10 h20 cylinder, enclosed r4 h8 void at z in [6,14].
/// The outer shell alone is a textbook cylinder, so the primitive recogniser
/// matched it and returned `pi*r^2*h`.
fn cyl_in_cyl(topo: &mut Topology, k: f64) -> Case {
    let blank = make_cylinder(topo, 10.0 * k, 20.0 * k).unwrap();
    let tool = make_cylinder(topo, 4.0 * k, 8.0 * k).unwrap();
    let tool = at(topo, tool, 0.0, 0.0, 6.0 * k);
    Case {
        solid: boolean(topo, BooleanOp::Cut, blank, tool).unwrap(),
        closed_form: PI * (10.0 * k).powi(2) * (20.0 * k) - PI * (4.0 * k).powi(2) * (8.0 * k),
        shells: 2,
        tol: EXACT,
    }
}

/// The same body assembled directly rather than through `Cut`.
fn cyl_cavity_direct(topo: &mut Topology, k: f64) -> Case {
    let blank = make_cylinder(topo, 10.0 * k, 20.0 * k).unwrap();
    let tool = make_cylinder(topo, 4.0 * k, 8.0 * k).unwrap();
    let tool = at(topo, tool, 0.0, 0.0, 6.0 * k);
    Case {
        solid: hollow_with(topo, blank, tool),
        closed_form: PI * (10.0 * k).powi(2) * (20.0 * k) - PI * (4.0 * k).powi(2) * (8.0 * k),
        shells: 2,
        tol: EXACT,
    }
}

/// An OFF-AXIS void defeats the surface-of-revolution recogniser, so this takes
/// a different route than `cyl_in_cyl` and stops that case passing by accident.
fn cyl_offaxis_void(topo: &mut Topology, k: f64) -> Case {
    let blank = make_cylinder(topo, 10.0 * k, 20.0 * k).unwrap();
    let tool = make_cylinder(topo, 3.0 * k, 8.0 * k).unwrap();
    let tool = at(topo, tool, 4.0 * k, 0.0, 6.0 * k);
    Case {
        solid: boolean(topo, BooleanOp::Cut, blank, tool).unwrap(),
        closed_form: PI * (10.0 * k).powi(2) * (20.0 * k) - PI * (3.0 * k).powi(2) * (8.0 * k),
        shells: 2,
        tol: TESSELLATED,
    }
}

/// Concentric spheres: the recogniser's sphere branch, same defect class.
fn sphere_in_sphere(topo: &mut Topology, k: f64) -> Case {
    let blank = make_sphere(topo, 8.0 * k, 16).unwrap();
    let tool = make_sphere(topo, 5.0 * k, 16).unwrap();
    Case {
        solid: hollow_with(topo, blank, tool),
        closed_form: 4.0 / 3.0 * PI * ((8.0 * k).powi(3) - (5.0 * k).powi(3)),
        shells: 2,
        tol: TESSELLATED,
    }
}

/// Box 20^3 with an enclosed 6^3 void. A planar outer shell never matched the
/// primitive recogniser, so this was already right — it guards the paths below
/// it in the ladder, which summed over the outer shell only.
fn box_in_box(topo: &mut Topology, k: f64) -> Case {
    let blank = make_box(topo, 20.0 * k, 20.0 * k, 20.0 * k).unwrap();
    let tool = make_box(topo, 6.0 * k, 6.0 * k, 6.0 * k).unwrap();
    let tool = at(topo, tool, 7.0 * k, 7.0 * k, 7.0 * k);
    Case {
        solid: boolean(topo, BooleanOp::Cut, blank, tool).unwrap(),
        closed_form: (20.0 * k).powi(3) - (6.0 * k).powi(3),
        shells: 2,
        tol: EXACT,
    }
}

/// Box 20^3 with an enclosed r5 spherical void.
fn sphere_in_box(topo: &mut Topology, k: f64) -> Case {
    let blank = make_box(topo, 20.0 * k, 20.0 * k, 20.0 * k).unwrap();
    let tool = make_sphere(topo, 5.0 * k, 32).unwrap();
    let tool = at(topo, tool, 10.0 * k, 10.0 * k, 10.0 * k);
    Case {
        solid: boolean(topo, BooleanOp::Cut, blank, tool).unwrap(),
        closed_form: (20.0 * k).powi(3) - 4.0 / 3.0 * PI * (5.0 * k).powi(3),
        shells: 2,
        tol: TESSELLATED,
    }
}

/// TWO disjoint enclosed voids: the solid carries two inner shells, and both
/// must be subtracted.
fn two_voids(topo: &mut Topology, k: f64) -> Case {
    let blank = make_box(topo, 30.0 * k, 20.0 * k, 20.0 * k).unwrap();
    let t1 = make_box(topo, 4.0 * k, 4.0 * k, 4.0 * k).unwrap();
    let t1 = at(topo, t1, 3.0 * k, 8.0 * k, 8.0 * k);
    let s = boolean(topo, BooleanOp::Cut, blank, t1).unwrap();
    let t2 = make_box(topo, 5.0 * k, 5.0 * k, 5.0 * k).unwrap();
    let t2 = at(topo, t2, 20.0 * k, 8.0 * k, 8.0 * k);
    Case {
        solid: boolean(topo, BooleanOp::Cut, s, t2).unwrap(),
        closed_form: (30.0 * k) * (20.0 * k) * (20.0 * k) - (4.0 * k).powi(3) - (5.0 * k).powi(3),
        shells: 3,
        tol: EXACT,
    }
}

/// A void whose far face lies EXACTLY on the blank's outer wall. Touching the
/// wall does NOT make a cavity: this must stay a one-shell solid. It is the
/// counterexample to "any removed material is an inner shell".
fn void_touching_wall(topo: &mut Topology, k: f64) -> Case {
    let blank = make_box(topo, 20.0 * k, 20.0 * k, 20.0 * k).unwrap();
    let tool = make_box(topo, 6.0 * k, 6.0 * k, 6.0 * k).unwrap();
    let tool = at(topo, tool, 14.0 * k, 7.0 * k, 7.0 * k);
    Case {
        solid: boolean(topo, BooleanOp::Cut, blank, tool).unwrap(),
        closed_form: (20.0 * k).powi(3) - (6.0 * k).powi(3),
        shells: 1,
        tol: EXACT,
    }
}

/// A pocket that breaks the outer wall — one shell, and unchanged by the fix.
fn open_pocket(topo: &mut Topology, k: f64) -> Case {
    let blank = make_box(topo, 20.0 * k, 20.0 * k, 20.0 * k).unwrap();
    let tool = make_box(topo, 6.0 * k, 6.0 * k, 6.0 * k).unwrap();
    let tool = at(topo, tool, 7.0 * k, 7.0 * k, 17.0 * k);
    Case {
        solid: boolean(topo, BooleanOp::Cut, blank, tool).unwrap(),
        closed_form: (20.0 * k).powi(3) - (6.0 * k) * (6.0 * k) * (3.0 * k),
        shells: 1,
        tol: EXACT,
    }
}

/// CONTROLS: solid bodies. A hollow body must come out right AND a solid one
/// must not move — these still take the primitive recogniser's closed form.
fn plain_cylinder(topo: &mut Topology, k: f64) -> Case {
    Case {
        solid: make_cylinder(topo, 10.0 * k, 20.0 * k).unwrap(),
        closed_form: PI * (10.0 * k).powi(2) * (20.0 * k),
        shells: 1,
        tol: EXACT,
    }
}

fn plain_box(topo: &mut Topology, k: f64) -> Case {
    Case {
        solid: make_box(topo, 20.0 * k, 20.0 * k, 20.0 * k).unwrap(),
        closed_form: (20.0 * k).powi(3),
        shells: 1,
        tol: EXACT,
    }
}

fn plain_sphere(topo: &mut Topology, k: f64) -> Case {
    Case {
        solid: make_sphere(topo, 8.0 * k, 16).unwrap(),
        closed_form: 4.0 / 3.0 * PI * (8.0 * k).powi(3),
        shells: 1,
        tol: EXACT,
    }
}

#[test]
fn a_cavity_is_removed_from_the_measured_volume() {
    type Build = (&'static str, fn(&mut Topology, f64) -> Case);
    let cases: &[Build] = &[
        ("cyl-in-cyl", cyl_in_cyl),
        ("cyl-cavity-direct", cyl_cavity_direct),
        ("cyl-offaxis-void", cyl_offaxis_void),
        ("sphere-in-sphere", sphere_in_sphere),
        ("box-in-box", box_in_box),
        ("sphere-in-box", sphere_in_box),
        ("two-voids", two_voids),
        ("void-touching-wall", void_touching_wall),
        ("open-pocket", open_pocket),
        ("CONTROL plain-cylinder", plain_cylinder),
        ("CONTROL plain-box", plain_box),
        ("CONTROL plain-sphere", plain_sphere),
    ];

    for k in [1.0_f64, 1000.0, 0.001] {
        for (name, build) in cases {
            let mut topo = Topology::new();
            let case = build(&mut topo, k);

            let n_shells = 1 + topo.solid(case.solid).unwrap().inner_shells().len();
            assert_eq!(
                n_shells, case.shells,
                "{name} at {k}x: expected {} shell(s), got {n_shells} — the model this \
                 test measures is not the model it means to measure",
                case.shells,
            );

            // Deflection is a LENGTH: a scale sweep must scale it with the
            // model, or 1000x asks for a mesh no machine can hold.
            let volume = solid_volume(&topo, case.solid, 1e-2 * k).unwrap();
            let relative = (volume - case.closed_form).abs() / case.closed_form;
            let mp = mass_properties(&topo, case.solid).unwrap().mass;

            assert!(
                relative < case.tol,
                "{name} at {k}x read {volume} against a closed-form {}, relative {relative:e} \
                 (allowed {:e}). [mass_properties, for the report only, read {mp}]",
                case.closed_form,
                case.tol,
            );
        }
    }
}
