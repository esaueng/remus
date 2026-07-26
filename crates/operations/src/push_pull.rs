//! Direct push/pull editing of an existing solid's faces.
//!
//! These operations modify a solid in place (returning a new solid) by moving
//! one of its faces, as opposed to [`crate::offset_face`], which offsets a
//! standalone face and produces a new face.
//!
//! Both operations follow the same shape: derive an exact tool solid from the
//! selected face's own geometry, apply a boolean, merge the coplanar/coaxial
//! seams the boolean leaves behind, and refuse to return a result whose shell
//! is not closed.

use std::f64::consts::PI;

use brepkit_math::mat::Mat4;
use brepkit_math::surfaces::CylindricalSurface;
use brepkit_math::tolerance::Tolerance;
use brepkit_math::vec::{Point3, Vec3};
use brepkit_topology::Topology;
use brepkit_topology::explorer::solid_faces;
use brepkit_topology::face::{FaceId, FaceSurface};
use brepkit_topology::solid::SolidId;

use crate::boolean::{BooleanOp, boolean};
use crate::copy::copy_face;
use crate::extrude::extrude;
use crate::heal::unify_faces;
use crate::measure::solid_volume;
use crate::primitives::make_cylinder;
use crate::transform::transform_solid;

/// How a cylindrical face sits in its solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Concavity {
    /// A bore: material lies outside the cylinder, the solid's outward normal
    /// points at the axis.
    Hole,
    /// A boss: material lies inside the cylinder.
    Boss,
}

/// Move a planar face of `solid` along its outward normal.
///
/// A positive `distance` adds material (the face is pulled outward), a
/// negative one removes it (the face is pushed into the solid). The tool is
/// extruded from the face itself, so inner wires are carried through and a
/// face with holes keeps them as holes.
///
/// Coplanar seams left where the tool meets the original solid are merged, so
/// pulling a face twice by 1 gives the same topology as pulling it once by 2.
///
/// # Errors
///
/// Returns an error if `distance` is zero or non-finite, the face is not part
/// of `solid`, the face is not planar, or the result's shell is not closed.
pub fn push_pull_face(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    distance: f64,
) -> Result<SolidId, crate::OperationsError> {
    let tol = Tolerance::new();

    if !distance.is_finite() {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("push/pull distance must be finite, got {distance}"),
        });
    }
    if distance.abs() <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("push/pull distance must be non-zero, got {distance}"),
        });
    }

    ensure_face_in_solid(topo, solid, face)?;

    let face_data = topo.face(face)?;
    let normal =
        face_data
            .effective_plane_normal()
            .ok_or_else(|| crate::OperationsError::InvalidInput {
                reason: format!(
                    "push/pull requires a planar face, face {} is {}",
                    face.index(),
                    face_data.surface().type_tag()
                ),
            })?;

    // Extrude a COPY: `extrude` reuses the profile wire's edges for its bottom
    // cap, and a tool sharing edges with the operand it is cut from feeds the
    // boolean two solids that alias the same topology.
    let profile = copy_face(topo, face)?;

    // `extrude` walks the profile along `direction * distance`; give it the
    // outward normal for a pull and the inward one for a push, always with a
    // positive length, so the tool occupies the slab actually being added or
    // removed and stays flush with the face's own plane.
    let (direction, op) = if distance > 0.0 {
        (normal, BooleanOp::Fuse)
    } else {
        (-normal, BooleanOp::Cut)
    };
    let tool = extrude(topo, profile, direction, distance.abs())?;

    // A prismatic push/pull moves exactly `area * |distance|` of material, so
    // the result's volume is known before the boolean runs. Checking it is
    // what stops a silently-degraded result reaching the caller: a face whose
    // hole walls must merge with a coaxial wall already in the solid can come
    // back closed, correctly shaped at a glance, and short a bore.
    let area = crate::measure::face_area(topo, face, verify_deflection(topo, solid))?;
    let before = solid_volume(topo, solid, verify_deflection(topo, solid))?;
    let expected = distance.mul_add(area, before);

    let result = boolean(topo, op, solid, tool)?;
    unify_faces(topo, result)?;
    drop_stranded_inner_wires(topo, result)?;
    ensure_closed_shell(topo, result, "push/pull")?;
    ensure_volume(topo, result, expected, "push/pull")?;
    Ok(result)
}

/// Change the radius of a cylindrical face of `solid`.
///
/// Works for both a bore (material outside the cylinder) and a boss (material
/// inside it); the concavity is read from the face's own orientation. The
/// cylinder's axial extent is taken from the face, so the caps at either end
/// are preserved and only the wall moves.
///
/// # Errors
///
/// Returns an error if `new_radius` is not positive and finite, the face is
/// not part of `solid`, the face is not cylindrical, the new radius equals the
/// current one, or the result's shell is not closed.
pub fn resize_cylindrical_face(
    topo: &mut Topology,
    solid: SolidId,
    face: FaceId,
    new_radius: f64,
) -> Result<SolidId, crate::OperationsError> {
    let tol = Tolerance::new();

    if !new_radius.is_finite() || new_radius <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylinder radius must be positive, got {new_radius}"),
        });
    }

    ensure_face_in_solid(topo, solid, face)?;

    let face_data = topo.face(face)?;
    let FaceSurface::Cylinder(cyl) = face_data.surface() else {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!(
                "resize requires a cylindrical face, face {} is {}",
                face.index(),
                face_data.surface().type_tag()
            ),
        });
    };
    let cyl = cyl.clone();
    let old_radius = cyl.radius();
    let reversed = face_data.is_reversed();

    if (new_radius - old_radius).abs() <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylinder radius is already {old_radius}"),
        });
    }

    // A cylindrical surface's natural normal points away from the axis. When
    // the face is reversed the solid's outward normal points AT the axis, so
    // the material is outside the cylinder — a bore.
    let concavity = if reversed {
        Concavity::Hole
    } else {
        Concavity::Boss
    };

    let (base, height) = axial_extent(topo, face, &cyl)?;
    if height <= tol.linear {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylindrical face {} has no axial extent", face.index()),
        });
    }

    let axis = unit(cyl.axis())?;
    let grows = new_radius > old_radius;
    let before = solid_volume(topo, solid, verify_deflection(topo, solid))?;
    // Sweeping the wall outward adds material on a boss and removes it from a
    // bore; inward does the reverse. The magnitude is the annular sleeve
    // between the two radii over the face's own extent.
    let sleeve = PI * (new_radius * new_radius - old_radius * old_radius) * height;
    let expected = if concavity == Concavity::Boss {
        before + sleeve
    } else {
        before - sleeve
    };

    // Growing the wall sweeps it into open space, shrinking it sweeps back
    // through material already there. Either way the material that moves is the
    // annular sleeve between the two radii over the face's own extent — a plain
    // cylinder when growing (the sleeve's inner radius is the axis), a tube
    // when shrinking. Only whether it is added or removed changes.
    let (op, tool) = match (concavity, grows) {
        (Concavity::Boss, true) => (
            BooleanOp::Fuse,
            place_cylinder(topo, base, axis, new_radius, height)?,
        ),
        (Concavity::Hole, true) => (
            BooleanOp::Cut,
            place_cylinder(topo, base, axis, new_radius, height)?,
        ),
        (Concavity::Hole, false) => (
            BooleanOp::Fuse,
            make_tube(topo, base, axis, new_radius, old_radius, height)?,
        ),
        (Concavity::Boss, false) => (
            BooleanOp::Cut,
            make_tube(topo, base, axis, new_radius, old_radius, height)?,
        ),
    };

    let result = boolean(topo, op, solid, tool)?;
    unify_faces(topo, result)?;
    drop_stranded_inner_wires(topo, result)?;
    ensure_closed_shell(topo, result, "cylindrical resize")?;
    ensure_volume(topo, result, expected, "cylindrical resize")?;
    Ok(result)
}

/// The tube between `inner_r` and `outer_r` over the wall's axial span.
///
/// The bore is overshot at both ends so its caps never land on the outer
/// cylinder's: coincident caps would make the difference a coplanar-face
/// boolean for no benefit. The tube's own caps stay flush with the wall being
/// replaced, so the sleeve covers exactly the material that moves.
fn make_tube(
    topo: &mut Topology,
    base: Point3,
    axis: Vec3,
    inner_r: f64,
    outer_r: f64,
    height: f64,
) -> Result<SolidId, crate::OperationsError> {
    let outer = place_cylinder(topo, base, axis, outer_r, height)?;
    let overshoot = (height * 0.1).max(1e-3);
    let inner = place_cylinder(
        topo,
        base - unit(axis)? * overshoot,
        axis,
        inner_r,
        overshoot.mul_add(2.0, height),
    )?;
    boolean(topo, BooleanOp::Cut, outer, inner)
}

/// A deflection fine enough that the volume check resolves the sleeve.
fn verify_deflection(topo: &Topology, solid: SolidId) -> f64 {
    crate::measure::solid_bounding_box(topo, solid).map_or(0.01, |bb| {
        ((bb.max - bb.min).length() * 5e-4).clamp(1e-4, 0.05)
    })
}

/// Reject a result whose volume is not the one the edit must produce.
///
/// The construction above is geometric rather than exact, so this is the gate
/// that makes it trustworthy: a tool that reached material it should not have,
/// or a boolean that silently dropped it, moves the volume off the analytic
/// target and the attempt is rejected instead of returned.
fn ensure_volume(
    topo: &Topology,
    solid: SolidId,
    expected: f64,
    what: &str,
) -> Result<(), crate::OperationsError> {
    let actual = solid_volume(topo, solid, verify_deflection(topo, solid))?;
    // Volume is measured from a tessellation, so allow its discretisation
    // error — wide enough for a curved wall, far tighter than any real defect.
    let slack = expected.abs().mul_add(2e-3, 1e-6);
    if (actual - expected).abs() <= slack {
        return Ok(());
    }
    Err(crate::OperationsError::InvalidInput {
        reason: format!("{what} produced volume {actual}, expected {expected}"),
    })
}

/// Drop inner wires that bound nothing, returning how many were removed.
///
/// Replacing a coaxial cylindrical feature can leave the OLD rim behind as an
/// inner wire on the face that absorbed it — growing a boss from r=5 to r=8
/// leaves the r=5 circle as a hole in the new r=8 cap. Every edge of such a
/// wire is used by that one face alone, so it borders no second face and the
/// shell is open along it.
///
/// A wire in that state cannot be the boundary of a real cavity (a cavity
/// would have faces on the other side), so the hole is spurious and the face's
/// own surface already covers it. Removing the wire closes the shell without
/// moving any geometry — and the caller's volume gate confirms it.
fn drop_stranded_inner_wires(
    topo: &mut Topology,
    solid: SolidId,
) -> Result<usize, crate::OperationsError> {
    let mut uses: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
    for fid in solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
            for oe in topo.wire(wid)?.edges() {
                *uses.entry(oe.edge().index()).or_insert(0) += 1;
            }
        }
    }

    let mut stranded: Vec<(FaceId, Vec<usize>)> = Vec::new();
    for fid in solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        let mut drop_idx = Vec::new();
        for (i, &wid) in face.inner_wires().iter().enumerate() {
            let wire = topo.wire(wid)?;
            let all_free = wire
                .edges()
                .iter()
                .all(|oe| uses.get(&oe.edge().index()).copied().unwrap_or(0) == 1);
            if all_free && !wire.edges().is_empty() {
                drop_idx.push(i);
            }
        }
        if !drop_idx.is_empty() {
            stranded.push((fid, drop_idx));
        }
    }

    let mut removed = 0;
    for (fid, drop_idx) in stranded {
        let face = topo.face_mut(fid)?;
        // Remove from the back so earlier indices stay valid.
        for &i in drop_idx.iter().rev() {
            face.inner_wires_mut().remove(i);
            removed += 1;
        }
    }
    Ok(removed)
}

/// Reject a face that does not belong to `solid` (including its inner shells).
fn ensure_face_in_solid(
    topo: &Topology,
    solid: SolidId,
    face: FaceId,
) -> Result<(), crate::OperationsError> {
    if solid_faces(topo, solid)?.contains(&face) {
        return Ok(());
    }
    Err(crate::OperationsError::InvalidInput {
        reason: format!(
            "face {} is not part of solid {}",
            face.index(),
            solid.index()
        ),
    })
}

/// The closed-shell gate.
///
/// `validate_solid_relaxed` does not check shell closure, so a result can
/// measure the right volume and still be unexportable — a stale rim left on
/// one face is invisible to a volume check but leaves the shell open.
fn ensure_closed_shell(
    topo: &Topology,
    solid: SolidId,
    what: &str,
) -> Result<(), crate::OperationsError> {
    use brepkit_check::validate::checks::{CheckId, Severity};
    use brepkit_check::validate::{ValidateOptions, validate_solid};

    let report = validate_solid(topo, solid, &ValidateOptions::default())?;
    let open: Vec<&str> = report
        .issues
        .iter()
        .filter(|i| i.check == CheckId::ShellClosed && i.severity == Severity::Error)
        .map(|i| i.description.as_str())
        .collect();
    if open.is_empty() {
        return Ok(());
    }
    Err(crate::OperationsError::InvalidInput {
        reason: format!("{what} left an open shell: {}", open.join("; ")),
    })
}

/// The face's extent along its cylinder axis, as a base point and a height.
///
/// Taken from the face's own vertices rather than the surface (which is
/// unbounded), so the tool spans exactly the wall being moved.
fn axial_extent(
    topo: &Topology,
    face: FaceId,
    cyl: &CylindricalSurface,
) -> Result<(Point3, f64), crate::OperationsError> {
    let axis = unit(cyl.axis())?;
    let origin = cyl.origin();

    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    let face_data = topo.face(face)?;
    for wid in
        std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
    {
        let wire = topo.wire(wid)?;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            for vid in [edge.start(), edge.end()] {
                let t = (topo.vertex(vid)?.point() - origin).dot(axis);
                lo = lo.min(t);
                hi = hi.max(t);
            }
        }
    }

    if !lo.is_finite() || !hi.is_finite() {
        return Err(crate::OperationsError::InvalidInput {
            reason: format!("cylindrical face {} has no vertices", face.index()),
        });
    }
    Ok((origin + axis * lo, hi - lo))
}

/// Normalize a direction, mapping a degenerate one onto an operations error.
fn unit(v: Vec3) -> Result<Vec3, crate::OperationsError> {
    v.normalize().map_err(crate::OperationsError::Math)
}

/// Build the matrix taking the canonical +Z cylinder to `base`/`axis`.
fn frame_matrix(base: Point3, axis: Vec3) -> Result<Mat4, crate::OperationsError> {
    let z = unit(axis)?;
    // Any vector not parallel to the axis gives a usable reference direction;
    // the tube is rotationally symmetric so the choice is arbitrary.
    let seed = if z.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    // Gram-Schmidt, not a cross product: for the common +Z axis this yields the
    // identity frame, so the tool's seam sits at the same angle as the seam on
    // the wall it is replacing. A rotated frame puts the two coincident
    // cylindrical faces' seams at different angles, and the pair no longer
    // merges cleanly.
    let x = unit(seed - z * seed.dot(z))?;
    let y = z.cross(x);
    Ok(Mat4([
        [x.x(), y.x(), z.x(), base.x()],
        [x.y(), y.y(), z.y(), base.y()],
        [x.z(), y.z(), z.z(), base.z()],
        [0.0, 0.0, 0.0, 1.0],
    ]))
}

/// A cylinder of `radius`/`height` based at `base` and running along `axis`.
fn place_cylinder(
    topo: &mut Topology,
    base: Point3,
    axis: Vec3,
    radius: f64,
    height: f64,
) -> Result<SolidId, crate::OperationsError> {
    let solid = make_cylinder(topo, radius, height)?;
    transform_solid(topo, solid, &frame_matrix(base, axis)?)?;
    Ok(solid)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use std::collections::HashMap;
    use std::f64::consts::PI;

    use brepkit_math::mat::Mat4;

    use super::*;
    use crate::measure::solid_volume;
    use crate::primitives::make_box;

    const DEFLECTION: f64 = 0.01;

    fn cylinder_at(topo: &mut Topology, r: f64, h: f64, x: f64, y: f64, z: f64) -> SolidId {
        let c = make_cylinder(topo, r, h).unwrap();
        transform_solid(topo, c, &Mat4::translation(x, y, z)).unwrap();
        c
    }

    /// Volume within the tessellation's deflection error.
    fn assert_volume(topo: &Topology, solid: SolidId, expected: f64) {
        let v = solid_volume(topo, solid, DEFLECTION).unwrap();
        assert!(
            (v - expected).abs() < expected.abs().mul_add(1e-3, 1.0),
            "volume {v} != expected {expected}"
        );
    }

    /// Every edge must be used exactly twice across the solid's faces.
    ///
    /// This is the property the coaxial-bore bug broke while volume and
    /// relaxed validation both still passed.
    fn assert_watertight(topo: &Topology, solid: SolidId) {
        let mut counts: HashMap<usize, usize> = HashMap::new();
        for fid in solid_faces(topo, solid).unwrap() {
            let face = topo.face(fid).unwrap();
            for wid in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wid).unwrap().edges() {
                    *counts.entry(oe.edge().index()).or_insert(0) += 1;
                }
            }
        }
        let free: Vec<_> = counts.iter().filter(|&(_, &c)| c != 2).collect();
        assert!(
            free.is_empty(),
            "edges not shared by exactly 2 faces: {free:?}"
        );
    }

    fn face_count(topo: &Topology, solid: SolidId, tag: &str) -> usize {
        solid_faces(topo, solid)
            .unwrap()
            .iter()
            .filter(|&&f| topo.face(f).unwrap().surface().type_tag() == tag)
            .count()
    }

    /// The planar face whose outward normal is `dir` and which lies furthest
    /// along it — i.e. the visible face on that side.
    fn face_facing(topo: &Topology, solid: SolidId, dir: Vec3) -> FaceId {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&f| {
                topo.face(f)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| n.dot(dir) > 0.99)
            })
            .max_by(|&a, &b| {
                let along = |f: FaceId| {
                    let w = topo.face(f).unwrap().outer_wire();
                    let e = topo.wire(w).unwrap().edges()[0].edge();
                    (topo.vertex(topo.edge(e).unwrap().start()).unwrap().point()
                        - Point3::new(0.0, 0.0, 0.0))
                    .dot(dir)
                };
                along(a).partial_cmp(&along(b)).unwrap()
            })
            .expect("no face with the requested normal")
    }

    fn only_cylinder(topo: &Topology, solid: SolidId) -> FaceId {
        let cyls: Vec<_> = solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .filter(|&f| matches!(topo.face(f).unwrap().surface(), FaceSurface::Cylinder(_)))
            .collect();
        assert_eq!(cyls.len(), 1, "expected exactly one cylindrical face");
        cyls[0]
    }

    /// A 40x40x10 block with an r=3 through-bore at (20, 20).
    fn drilled_block(topo: &mut Topology) -> SolidId {
        let block = make_box(topo, 40.0, 40.0, 10.0).unwrap();
        let drill = cylinder_at(topo, 3.0, 10.0, 20.0, 20.0, 0.0);
        boolean(topo, BooleanOp::Cut, block, drill).unwrap()
    }

    /// A 40x40x10 block with an r=5 h=10 boss standing on its top face.
    fn bossed_block(topo: &mut Topology) -> SolidId {
        let block = make_box(topo, 40.0, 40.0, 10.0).unwrap();
        let boss = cylinder_at(topo, 5.0, 10.0, 20.0, 20.0, 10.0);
        boolean(topo, BooleanOp::Fuse, block, boss).unwrap()
    }

    // --- push_pull_face -------------------------------------------------

    #[test]
    fn pulling_a_box_face_adds_a_slab() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));

        let out = push_pull_face(&mut topo, block, top, 5.0).unwrap();

        assert_volume(&topo, out, 10.0 * 10.0 * 15.0);
        assert_watertight(&topo, out);
        // The seam where the tool met the block must be merged away.
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn pushing_a_box_face_removes_a_slab() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));

        let out = push_pull_face(&mut topo, block, top, -3.0).unwrap();

        assert_volume(&topo, out, 10.0 * 10.0 * 7.0);
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn pulling_twice_matches_pulling_once() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));
        let once = push_pull_face(&mut topo, block, top, 4.0).unwrap();

        let block2 = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top2 = face_facing(&topo, block2, Vec3::new(0.0, 0.0, 1.0));
        let step1 = push_pull_face(&mut topo, block2, top2, 2.0).unwrap();
        let top3 = face_facing(&topo, step1, Vec3::new(0.0, 0.0, 1.0));
        let twice = push_pull_face(&mut topo, step1, top3, 2.0).unwrap();

        assert_volume(&topo, twice, solid_volume(&topo, once, DEFLECTION).unwrap());
        assert_eq!(
            face_count(&topo, twice, "plane"),
            face_count(&topo, once, "plane")
        );
        assert_watertight(&topo, twice);
    }

    #[test]
    fn pulling_a_face_with_a_hole_keeps_the_hole() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let top = face_facing(&topo, drilled, Vec3::new(0.0, 0.0, 1.0));
        assert_eq!(
            topo.face(top).unwrap().inner_wires().len(),
            1,
            "the picked cap should carry the bore as an inner wire"
        );

        let out = push_pull_face(&mut topo, drilled, top, 5.0).unwrap();

        // The block grows to 15 tall and the bore grows with it.
        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 15.0, -(PI * 9.0 * 15.0)));
        assert_watertight(&topo, out);
        // The bore stays ONE cylindrical face, not two stacked bands.
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn pushing_a_face_with_a_hole_keeps_the_hole() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let top = face_facing(&topo, drilled, Vec3::new(0.0, 0.0, 1.0));

        let out = push_pull_face(&mut topo, drilled, top, -4.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 6.0, -(PI * 9.0 * 6.0)));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn push_pull_rejects_bad_input() {
        let mut topo = Topology::new();
        let block = make_box(&mut topo, 10.0, 10.0, 10.0).unwrap();
        let top = face_facing(&topo, block, Vec3::new(0.0, 0.0, 1.0));

        assert!(push_pull_face(&mut topo, block, top, 0.0).is_err());
        assert!(push_pull_face(&mut topo, block, top, f64::NAN).is_err());

        // A face belonging to a different solid.
        let other = make_box(&mut topo, 2.0, 2.0, 2.0).unwrap();
        let other_top = face_facing(&topo, other, Vec3::new(0.0, 0.0, 1.0));
        assert!(push_pull_face(&mut topo, block, other_top, 1.0).is_err());

        // A cylindrical face is not push/pull-able.
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);
        assert!(push_pull_face(&mut topo, drilled, bore, 1.0).is_err());
    }

    // --- resize_cylindrical_face ----------------------------------------

    #[test]
    fn widening_a_bore() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);

        let out = resize_cylindrical_face(&mut topo, drilled, bore, 5.0).unwrap();

        assert_volume(
            &topo,
            out,
            40.0f64.mul_add(40.0 * 10.0, -(PI * 25.0 * 10.0)),
        );
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    #[test]
    fn shrinking_a_bore() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);

        let out = resize_cylindrical_face(&mut topo, drilled, bore, 2.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, -(PI * 4.0 * 10.0)));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn growing_a_boss() {
        let mut topo = Topology::new();
        let bossed = bossed_block(&mut topo);
        let wall = only_cylinder(&topo, bossed);

        let out = resize_cylindrical_face(&mut topo, bossed, wall, 8.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, PI * 64.0 * 10.0));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn shrinking_a_boss() {
        let mut topo = Topology::new();
        let bossed = bossed_block(&mut topo);
        let wall = only_cylinder(&topo, bossed);

        let out = resize_cylindrical_face(&mut topo, bossed, wall, 3.0).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, PI * 9.0 * 10.0));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
    }

    #[test]
    fn resizing_a_bore_twice_stays_watertight() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);
        let wide = resize_cylindrical_face(&mut topo, drilled, bore, 5.0).unwrap();
        let bore2 = only_cylinder(&topo, wide);
        let narrow = resize_cylindrical_face(&mut topo, wide, bore2, 4.0).unwrap();

        assert_volume(
            &topo,
            narrow,
            40.0f64.mul_add(40.0 * 10.0, -(PI * 16.0 * 10.0)),
        );
        assert_watertight(&topo, narrow);
        assert_eq!(face_count(&topo, narrow, "cylinder"), 1);
    }

    /// Regression: an annular sleeve fused into a matching bore.
    ///
    /// Every contact is coincident — the sleeve's outer wall IS the bore wall,
    /// and its end caps sit in the caps' own planes inside their holes. The
    /// annuli used to classify inconsistently (one kept, one dropped), and the
    /// coplanar merge then carried the filled r=3 rim onto the merged cap,
    /// leaving free edges. Exercised directly here, below `resize`.
    #[test]
    fn sleeve_fused_into_a_matching_bore_closes_the_shell() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);

        let outer = cylinder_at(&mut topo, 3.0, 10.0, 20.0, 20.0, 0.0);
        let inner = cylinder_at(&mut topo, 2.0, 12.0, 20.0, 20.0, -1.0);
        let sleeve = boolean(&mut topo, BooleanOp::Cut, outer, inner).unwrap();

        let out = boolean(&mut topo, BooleanOp::Fuse, drilled, sleeve).unwrap();
        unify_faces(&mut topo, out).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 10.0, -(PI * 4.0 * 10.0)));
        assert_watertight(&topo, out);
        // The r=3 wall is gone and the r=2 one replaces it — one bore, not two.
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        assert_eq!(face_count(&topo, out, "plane"), 6);
    }

    /// Regression: two coaxial bore bands of equal radius must merge into one
    /// face. `unify_faces` used to treat each band's seam edge — which appears
    /// twice in the same wire — as a shared internal edge and delete it,
    /// leaving two disjoint rim circles that reassembled as an outer wire plus
    /// a bogus inner wire on a cylinder.
    #[test]
    fn stacked_coaxial_bore_bands_merge_into_one_wall() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);

        // A slab with a coaxial bore, stacked directly on top.
        let slab = make_box(&mut topo, 40.0, 40.0, 5.0).unwrap();
        transform_solid(&mut topo, slab, &Mat4::translation(0.0, 0.0, 10.0)).unwrap();
        let slab_bore = cylinder_at(&mut topo, 3.0, 5.0, 20.0, 20.0, 10.0);
        let holed_slab = boolean(&mut topo, BooleanOp::Cut, slab, slab_bore).unwrap();

        let out = boolean(&mut topo, BooleanOp::Fuse, drilled, holed_slab).unwrap();
        unify_faces(&mut topo, out).unwrap();

        assert_volume(&topo, out, 40.0f64.mul_add(40.0 * 15.0, -(PI * 9.0 * 15.0)));
        assert_watertight(&topo, out);
        assert_eq!(face_count(&topo, out, "cylinder"), 1);
        let bore = only_cylinder(&topo, out);
        assert!(
            topo.face(bore).unwrap().inner_wires().is_empty(),
            "a merged bore wall must not acquire an inner wire"
        );
    }

    #[test]
    fn resize_rejects_bad_input() {
        let mut topo = Topology::new();
        let drilled = drilled_block(&mut topo);
        let bore = only_cylinder(&topo, drilled);

        assert!(resize_cylindrical_face(&mut topo, drilled, bore, 0.0).is_err());
        assert!(resize_cylindrical_face(&mut topo, drilled, bore, -1.0).is_err());
        assert!(resize_cylindrical_face(&mut topo, drilled, bore, f64::INFINITY).is_err());
        // Already at this radius.
        assert!(resize_cylindrical_face(&mut topo, drilled, bore, 3.0).is_err());
        // A planar face is not resizable.
        let top = face_facing(&topo, drilled, Vec3::new(0.0, 0.0, 1.0));
        assert!(resize_cylindrical_face(&mut topo, drilled, top, 5.0).is_err());
    }
}
