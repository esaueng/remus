//! Volume, center of mass, and related computations for B-rep solids.

use remus_math::vec::{Point3, Vec3};
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

use crate::tessellate;

use super::helpers::{collect_solid_vertex_points, compute_angular_range};

/// Volume of a solid that contains a bored quadric — a sphere (or torus) face
/// carrying a full-revolution latitude-circle hole (a drilled tunnel rim) — via
/// exact per-face Gauss quadrature on the analytic surfaces.
///
/// The tessellation paths below cannot bound such an annular band: both of its
/// boundary loops are constant-v latitude circles, so the band's UV outline is
/// degenerate and the mesh fills the removed polar cap, over-counting. The
/// per-face analytic integrator (orientation-aware, hole-clipped) is exact.
///
/// Scope is deliberately narrow — only solids whose tessellated volume is known
/// to be wrong — so every other analytic solid keeps its existing
/// tessellation-based volume. Returns `None` (defer to tessellation) when no
/// bored quadric is present, when any face is NURBS, or when a face fails to
/// integrate.
/// Whether a sphere face's outer wire lies on a single constant-`v` latitude
/// (the simple bored-quadric band) rather than a scalloped, varying-`v` collar
/// floor. Projects the outer wire's vertices to `(u, v)` and tests the `v`
/// spread.
fn sphere_outer_wire_constant_v(
    topo: &Topology,
    face_id: FaceId,
    sphere: &remus_math::surfaces::SphericalSurface,
) -> Result<bool, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let wire = topo.wire(face.outer_wire())?;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge())?;
        let (sv, ev) = (topo.vertex(edge.start())?, topo.vertex(edge.end())?);
        let (sp, ep) = (sv.point(), ev.point());
        let (t0, t1) =
            crate::authoritative_edge_domain(edge, "sphere latitude-band classification")?;
        // Sample ALONG each edge, not just its start vertex: a great-circle arc
        // has both endpoints on the seam latitude yet bulges away from it, so
        // endpoint-only sampling would mis-read a scalloped collar floor as a
        // constant-v band and wrongly take the analytic fast path.
        for i in 0..=8 {
            let t = t0 + (t1 - t0) * (f64::from(i) / 8.0);
            let (_, v) = sphere.project_point(edge.curve().evaluate_with_endpoints(t, sp, ep));
            v_min = v_min.min(v);
            v_max = v_max.max(v);
        }
    }
    // Latitude-flatness threshold sized to the linear-tolerance magnitude: a
    // real band's v-spread is ~fp-noise; a collar's is a large fraction of a
    // radian.
    Ok((v_max - v_min) <= 1e-7)
}

/// Whether the solid has at least one sphere face that is a scalloped collar
/// (a bored quadric whose outer wire varies in `v`, e.g. a box ∩ sphere patch).
fn solid_has_scalloped_sphere_collar(
    topo: &Topology,
    solid: SolidId,
) -> Result<bool, crate::OperationsError> {
    for fid in remus_topology::explorer::solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        if let FaceSurface::Sphere(sphere) = face.surface()
            && !face.inner_wires().is_empty()
            && !sphere_outer_wire_constant_v(topo, fid, sphere)?
        {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Whether the solid has a torus notch band (torus − box: a kept toroidal patch
/// bounded by two tube-angle(`v`)-WRAPPING seam-arc loops, NOT constant-`v`
/// latitude circles). Its per-face analytic tessellation can't be sampled
/// watertight in isolation (the band wraps `v` and shares vertices with the
/// notch walls), and there is no closed-form volume — so the volume is taken off
/// the (watertight) whole-solid mesh instead, like the box ∩ sphere collar.
fn solid_has_torus_notch_band(topo: &Topology, solid: SolidId) -> bool {
    let Ok(faces) = remus_topology::explorer::solid_faces(topo, solid) else {
        return false;
    };
    faces.iter().any(|&fid| {
        topo.face(fid).is_ok_and(|f| match f.surface() {
            FaceSurface::Torus(t) => {
                f.inner_wires().len() == 1 && torus_wire_wraps_tube(topo, f.outer_wire(), t)
            }
            _ => false,
        })
    })
}

/// True when a torus face wire's vertices span the full tube angle `v` (a
/// `v`-wrapping seam loop), as opposed to a constant-`v` latitude circle. The
/// ordered edge samples must accumulate exactly one full `v` period.
fn torus_wire_wraps_tube(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
    torus: &remus_math::surfaces::ToroidalSurface,
) -> bool {
    use std::f64::consts::{PI, TAU};

    let Ok(wire) = topo.wire(wire_id) else {
        return false;
    };
    let mut vs: Vec<f64> = Vec::new();
    for oe in wire.edges() {
        let Ok(e) = topo.edge(oe.edge()) else {
            return false;
        };
        let (Ok(sv), Ok(ev)) = (topo.vertex(e.start()), topo.vertex(e.end())) else {
            return false;
        };
        let (sp, ep) = (sv.point(), ev.point());
        // Sample ALONG each edge (not just endpoints): a wrapping seam arc bows
        // far in v between its endpoints, so endpoint-only sampling can miss the
        // wrap and misclassify a notch band as a constant-v circle.
        for k in 0..=8 {
            let f = f64::from(k) / 8.0;
            let t = if oe.is_forward() { f } else { 1.0 - f };
            let p = e.curve().evaluate_with_endpoints(t, sp, ep);
            vs.push(torus.project_point(p).1);
        }
    }
    if vs.len() < 3 {
        return false;
    }
    let unwrap_delta = |from: f64, to: f64| {
        let delta = to - from;
        delta - TAU * ((delta + PI) / TAU).floor()
    };
    let winding = vs
        .windows(2)
        .fold(0.0, |acc, pair| acc + unwrap_delta(pair[0], pair[1]))
        + unwrap_delta(vs[vs.len() - 1], vs[0]);
    (winding.abs() - TAU).abs() <= 1.0e-6
}

/// Count mesh edges incident to a number of triangles other than 2 (boundary or
/// non-manifold edges). Zero means a closed 2-manifold.
fn mesh_boundary_edge_count(mesh: &tessellate::TriangleMesh) -> usize {
    use remus_math::det_hash::DetHashMap;
    let mut counts: DetHashMap<(u32, u32), usize> = DetHashMap::default();
    for tri in mesh.indices.chunks_exact(3) {
        for &(i, j) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if i < j { (i, j) } else { (j, i) };
            *counts.entry(key).or_insert(0) += 1;
        }
    }
    counts.values().filter(|&&c| c != 2).count()
}

/// Whether a cylinder/cone wall is a NOTCHED band — a trimmed region whose UV
/// outline is not the rectangle `[u_min, u_max] x [v_min, v_max]`.
///
/// Every measurement below the analytic path (this module's tessellation
/// routes, and `face_area`'s `sweep * r * height`) reads a quadric wall as that
/// rectangle. It is the right domain for a plain band or a partial sector, and
/// wrong for the wall a boss crossing a wall leaves behind: the buried arc is
/// removed only over the thickness it is buried in, so the survivor is the full
/// ring above the wall plus a tab hanging below it, and the rectangle credits
/// the whole cylinder — 3.05 % heavy on a 60x40x8 plate fused with an r=10 boss
/// overhanging its x=0 wall, against a per-face integral exact to the closed
/// form.
///
/// Signature: the outline visits three or more distinct axial levels. Two is
/// all a rectangle's corners can occupy, so a third is proof the rectangle
/// over-credits, and it is also what puts
/// [`remus_check::properties::face_integrator`] on its boundary-trimmed
/// branch, which integrates the real outline. Holed walls are routed by their
/// own trigger; NURBS never reaches here.
fn quadric_wall_is_notched_band(topo: &Topology, fid: FaceId) -> bool {
    let Ok(face) = topo.face(fid) else {
        return false;
    };
    let (axis, origin) = match face.surface() {
        FaceSurface::Cylinder(c) => (c.axis(), c.origin()),
        FaceSurface::Cone(c) => (c.axis(), c.apex()),
        _ => return false,
    };
    if !face.inner_wires().is_empty() {
        return false;
    }
    let Ok(pts) = crate::boolean::face_polygon(topo, fid) else {
        return false;
    };
    if pts.len() < 4 {
        return false;
    }
    let mut levels: Vec<f64> = pts.iter().map(|p| axis.dot(*p - origin)).collect();
    levels.sort_by(f64::total_cmp);
    let span = levels[levels.len() - 1] - levels[0];
    if span <= 0.0 {
        return false;
    }
    let eps = span * 1e-6;
    levels.dedup_by(|a, b| (*a - *b).abs() <= eps);
    levels.len() >= 3
}

/// Whether a quadric wall's outer boundary WINDS the surface's periodic angle
/// — it marches the whole way round the lateral instead of closing within it.
///
/// This is what separates the two kinds of notched band, and it is the
/// property that decides whether the per-face integrator can see the face at
/// all. [`remus_check::properties::face_integrator`] trims a quadric on its
/// projected outline only when that outline CLOSES; a boundary that winds the
/// period has no inside to test against, so the integrator falls back to
/// integrating the whole revolution over the boundary's `v` extent — the
/// analytic rectangle, which is exactly the over-count that must be deferred
/// to the structured tessellator.
///
/// * Winds: the rim a circle-outside cone/box fuse leaves — four corner
///   ring-arcs alternating with four wall arches, one closed chain around the
///   whole lateral. Deferred.
/// * Closes: the wall of a cross-drilled bore. Its rim is one closed NURBS
///   loop that spans part of the period and comes back, so the integrator
///   trims on the real outline and measures the face to its own chording.
///   Kept on the analytic path.
///
/// Measured on a r=3 h=30 shaft cross-drilled at r=3: the two bore walls sum
/// to -71.961 against a closed form of -72.000.
fn quadric_wall_boundary_winds_period(topo: &Topology, fid: FaceId) -> bool {
    let Ok(face) = topo.face(fid) else {
        return false;
    };
    let (axis, origin) = match face.surface() {
        FaceSurface::Cylinder(c) => (c.axis(), c.origin()),
        FaceSurface::Cone(c) => (c.axis(), c.apex()),
        _ => return false,
    };
    let Ok(axis) = axis.normalize() else {
        return false;
    };
    // Any frame perpendicular to the axis will do: a winding is the same
    // number in every one of them.
    let helper = if axis.x().abs() < 0.9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let Ok(fx) = axis.cross(helper).normalize() else {
        return false;
    };
    let fy = axis.cross(fx);

    let Ok(pts) = crate::boolean::face_polygon(topo, fid) else {
        return false;
    };
    if pts.len() < 3 {
        return false;
    }
    let angles: Vec<f64> = pts
        .iter()
        .map(|p| {
            let r = *p - origin;
            r.dot(fy).atan2(r.dot(fx))
        })
        .collect();

    // Sum of the SHORTEST step between consecutive samples, so the total is
    // independent of how finely the rim was sampled. A duplicated closing
    // point contributes zero.
    let tau = std::f64::consts::TAU;
    let winding: f64 = (0..angles.len())
        .map(|i| {
            let d = angles[(i + 1) % angles.len()] - angles[i];
            d - tau * ((d + std::f64::consts::PI) / tau).floor()
        })
        .sum();
    winding.abs() >= tau - 1e-3
}

fn analytic_faces_solid_volume(
    topo: &Topology,
    solid: SolidId,
) -> Result<Option<f64>, crate::OperationsError> {
    use remus_topology::explorer::solid_faces;

    let faces = solid_faces(topo, solid)?;
    if faces.is_empty() {
        return Ok(None);
    }

    // The Steinmetz lens fuse — two mutually-trimmed equal cylinders, whose
    // walls keep the lens ellipses as holes — has an EXACT closed-form volume
    // (computed directly below). The hole-unaware tessellation paths over-count
    // the lens, and a general holed-cylinder integrator was too broad to be
    // correct; the closed form is exact and needs no special integration.
    if solid_is_steinmetz_lens_fuse(topo, &faces) {
        return Ok(steinmetz_lens_fuse_volume(topo, &faces));
    }

    let mut has_bored_quadric = false;
    for &fid in &faces {
        let notched_quadric = quadric_wall_is_notched_band(topo, fid);
        if notched_quadric {
            has_bored_quadric = true;
        }
        let face = topo.face(fid)?;
        // A notched quadric with a marched NURBS rim that WINDS the period is
        // the wavy-band topology produced by circle-outside cone/box fuses.
        // Its analytic bounding rectangle over-counts the removed lobes; the
        // solid-level structured tessellator follows the actual rim, so defer
        // to it.
        //
        // The winding test is what keeps this off the cross-drilled shaft. A
        // bore wall is a quadric with no inner wires whose single closed NURBS
        // rim visits three or more axial levels, so it is "notched" by the
        // same level test and carries a NURBS rim by the same edge test — the
        // two conditions the wavy band was recognised by. It differs in the
        // only way that matters to the integrator: its rim CLOSES within the
        // period rather than marching round it, so the outline can be trimmed
        // on and the face measures exactly. Without the third condition every
        // cross-drilled shaft left the analytic path and fell through to
        // tessellation, which reads the UN-BORED stock — 848.040 against a
        // closed form of 704.230 at bore r=3, and the same 848.040 at r=2 and
        // r=1, three geometrically different holes.
        let has_nurbs_rim = notched_quadric
            && quadric_wall_boundary_winds_period(topo, fid)
            && topo.wire(face.outer_wire()).ok().is_some_and(|wire| {
                wire.edges().iter().any(|oe| {
                    topo.edge(oe.edge())
                        .is_ok_and(|edge| matches!(edge.curve(), EdgeCurve::NurbsCurve(_)))
                })
            });
        if has_nurbs_rim {
            return Ok(None);
        }
        match face.surface() {
            FaceSurface::Nurbs(_) => return Ok(None),
            // Sphere only: the per-face integrator's hole-clipping is wired up
            // for spheres. A bored torus would pass `hole_vs = []` and
            // over-integrate, so defer it to tessellation until torus
            // hole-clipping lands (with the torus−box analytic split).
            FaceSurface::Sphere(s) if !face.inner_wires().is_empty() => {
                // The integrator's hole-clipping models a band between two
                // constant-v latitudes. A collar whose OUTER wire varies in v
                // (great-circle/seam arcs, e.g. a box ∩ sphere patch) is not
                // that shape — its scalloped floor and lune bites would be
                // mis-integrated, so defer the whole solid to tessellation.
                if !sphere_outer_wire_constant_v(topo, fid, s)? {
                    return Ok(None);
                }
                has_bored_quadric = true;
            }
            // A holed cylinder/cone wall subtracts its holes in the same UV
            // domain its quadrature runs over, so it measures here rather than
            // being deferred: a bore's rim is removed from the wall it opens
            // in, and the bore wall itself — bounded by one closed edge, with
            // no analytic v extent to fall back on — takes its domain from
            // that edge instead of from the surface's unbounded one.
            FaceSurface::Cylinder(_) | FaceSurface::Cone(_) if !face.inner_wires().is_empty() => {
                has_bored_quadric = true;
            }
            // A torus's tube is periodic in BOTH parameters, so a hole that
            // wraps a period bounds no patch and has no "above" to count: the
            // integrator leaves it, and the solid is deferred to tessellation.
            FaceSurface::Torus(_) if !face.inner_wires().is_empty() => return Ok(None),
            _ => {}
        }
    }
    if !has_bored_quadric {
        return Ok(None);
    }

    let gauss_order = remus_check::properties::PropertiesOptions::default().gauss_order;
    let mut total = 0.0;
    for &fid in &faces {
        let Ok(properties) =
            remus_check::properties::face_integrator::integrate_face(topo, fid, gauss_order)
        else {
            return Ok(None);
        };
        total += properties.volume;
    }
    Ok(Some(total.abs()))
}

/// Exact volume of a fully-analytic SURFACE-OF-REVOLUTION solid (cone / cylinder
/// / torus walls + concentric circular planar disc caps) via the per-face
/// divergence-theorem integrators — no tessellation, so it is immune to the
/// inscribed-mesh undercount.
///
/// Returns `None` (defer to tessellation) unless EVERY face fits the revolution
/// signature, so this does NOT fire for boolean results that merely have an
/// arc-bounded planar face (a rounded-rect cap, an arc-frame lip):
///   * no NURBS face; inner wires only on PLANAR caps (an annulus cap
///     subtracts its holes; a bored-quadric solid is handled by
///     [`analytic_faces_solid_volume`] instead);
///   * at least one quadric wall (cylinder/cone/torus) — it is a revolution;
///   * every cylinder/cone/torus shares ONE axis line;
///   * every planar face is a circular disc/annulus/sector whose bounding
///     arc(s) are centred ON that shared axis.
fn analytic_revolution_solid_volume(topo: &Topology, solid: SolidId) -> Option<f64> {
    use remus_topology::explorer::solid_faces;

    let faces = solid_faces(topo, solid).ok()?;
    if faces.is_empty() {
        return None;
    }

    // Establish the shared revolution axis from the first quadric wall.
    let mut axis: Option<(Point3, Vec3)> = None;
    let mut has_wall = false;
    let axis_tol = 1e-7;

    let set_or_check_axis = |axis: &mut Option<(Point3, Vec3)>, o: Point3, d: Vec3| -> bool {
        let d = match d.normalize() {
            Ok(d) => d,
            Err(_) => return false,
        };
        match axis {
            None => {
                *axis = Some((o, d));
                true
            }
            Some((o0, d0)) => {
                // Same axis LINE: parallel directions and the origins' offset is
                // along the axis (no perpendicular component).
                if d0.cross(d).length() > 1e-6 {
                    return false;
                }
                let off = o - *o0;
                (off - *d0 * off.dot(*d0)).length() <= axis_tol * off.length().max(1.0)
            }
        }
    };

    for &fid in &faces {
        let face = topo.face(fid).ok()?;
        if !face.inner_wires().is_empty() {
            // Only a holed PLANAR cap is integrable here: an annulus cap (a
            // washer-style revolve, a coaxially bored tube) subtracts its holes
            // inside `planar_cap_signed_volume`, and the second pass still
            // requires every arc — inner wires included — to be centred on the
            // shared axis. A holed quadric wall has no hole-aware integrator.
            if !matches!(face.surface(), FaceSurface::Plane { .. }) {
                return None;
            }
        }
        match face.surface() {
            FaceSurface::Sphere(_) => return None,
            // NURBS faces are validated in the second pass (they must be the
            // degenerate on-axis band the revolve leaves when a profile touches
            // the axis); they need the axis, established here.
            FaceSurface::Nurbs(_) => {}
            FaceSurface::Cylinder(c) => {
                has_wall = true;
                if !set_or_check_axis(&mut axis, c.origin(), c.axis()) {
                    return None;
                }
            }
            FaceSurface::Cone(c) => {
                has_wall = true;
                if !set_or_check_axis(&mut axis, c.apex(), c.axis()) {
                    return None;
                }
            }
            FaceSurface::Torus(t) => {
                has_wall = true;
                if !set_or_check_axis(&mut axis, t.center(), t.z_axis()) {
                    return None;
                }
            }
            FaceSurface::Plane { .. } => {} // checked below, once the axis is known
        }
    }
    let (axis_o, axis_d) = axis?;
    if !has_wall {
        return None;
    }

    // Second pass (axis known): every planar face must be a circular
    // disc/annulus/sector centred on the shared axis, and every NURBS face must
    // be the degenerate on-axis band (zero radial extent). Cache each planar
    // cap's analytic volume here so the summation below reuses it rather than
    // re-traversing the wire and re-running arc recognition a second time.
    let mut cap_volumes: std::collections::HashMap<FaceId, f64> = std::collections::HashMap::new();
    for &fid in &faces {
        let face = topo.face(fid).ok()?;
        match face.surface() {
            FaceSurface::Plane { normal, .. } => {
                if normal.normalize().ok()?.cross(axis_d).length() > 1e-6 {
                    return None; // cap not perpendicular to the axis
                }
                if !planar_face_arcs_centered_on_axis(topo, fid, axis_o, axis_d) {
                    return None;
                }
                // Must be analytically integrable (a circular-arc-bounded cap).
                let v = planar_cap_signed_volume(topo, fid).ok()??;
                cap_volumes.insert(fid, v);
            }
            FaceSurface::Nurbs(_) if !nurbs_band_is_on_axis(topo, fid, axis_o, axis_d) => {
                return None;
            }
            _ => {}
        }
    }

    // All faces fit — sum the exact per-face divergence-theorem contributions
    // (quadric walls + analytic planar disc caps; the on-axis NURBS bands are
    // zero-area and contribute nothing). No tessellation occurs.
    let mut total = 0.0;
    for &fid in &faces {
        let face = topo.face(fid).ok()?;
        let c = match face.surface() {
            FaceSurface::Cylinder(_) => analytic_cylinder_signed_volume(topo, fid).ok()?,
            FaceSurface::Cone(_) => analytic_cone_signed_volume(topo, fid).ok()?,
            FaceSurface::Torus(_) => analytic_torus_signed_volume(topo, fid).ok()?,
            FaceSurface::Plane { .. } => *cap_volumes.get(&fid)?,
            FaceSurface::Nurbs(_) => 0.0, // degenerate on-axis band
            FaceSurface::Sphere(_) => return None,
        };
        total += c;
    }
    Some(total.abs())
}

/// Whether a NURBS face is a degenerate revolution band on the axis — all its
/// boundary vertices lie on the axis line (zero radial extent), so it bounds no
/// volume. This is the band a revolve leaves when the profile touches the axis.
fn nurbs_band_is_on_axis(topo: &Topology, face_id: FaceId, axis_o: Point3, axis_d: Vec3) -> bool {
    let Ok(face) = topo.face(face_id) else {
        return false;
    };
    let tol = 1e-7;
    let Ok(wire) = topo.wire(face.outer_wire()) else {
        return false;
    };
    for oe in wire.edges() {
        let Ok(edge) = topo.edge(oe.edge()) else {
            return false;
        };
        for vid in [edge.start(), edge.end()] {
            let Ok(v) = topo.vertex(vid) else {
                return false;
            };
            let off = v.point() - axis_o;
            let radial = off - axis_d * off.dot(axis_d);
            if radial.length() > tol {
                return false;
            }
        }
    }
    true
}

/// Whether every circular-arc edge of a planar face is centred on the given axis
/// line — the test that distinguishes a revolution disc/annulus cap (arcs about
/// the axis) from an arbitrary arc-bounded planar face (e.g. a rounded rectangle,
/// whose corner arcs are centred at the corners, off the axis).
fn planar_face_arcs_centered_on_axis(
    topo: &Topology,
    face_id: FaceId,
    axis_o: Point3,
    axis_d: Vec3,
) -> bool {
    let Ok(face) = topo.face(face_id) else {
        return false;
    };
    let tol = 1e-6;
    for wire_id in std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied()) {
        let Ok(wire) = topo.wire(wire_id) else {
            return false;
        };
        for oe in wire.edges() {
            let Ok(edge) = topo.edge(oe.edge()) else {
                return false;
            };
            let center = match edge.curve() {
                remus_topology::edge::EdgeCurve::Circle(c) => Some(c.center()),
                remus_topology::edge::EdgeCurve::NurbsCurve(nc) => {
                    let rtol = remus_math::tolerance::Tolerance::default().linear * 100.0;
                    match remus_geometry::convert::recognize_curve(nc, rtol) {
                        remus_geometry::convert::RecognizedCurve::Circle { center, .. } => {
                            Some(center)
                        }
                        _ => None,
                    }
                }
                _ => None,
            };
            if let Some(center) = center {
                // Distance from the arc centre to the axis line must be ~0.
                let off = center - axis_o;
                let perp = off - axis_d * off.dot(axis_d);
                if perp.length() > tol * off.length().max(1.0) {
                    return false;
                }
            }
        }
    }
    true
}

/// Exact volume of the STEINMETZ LENS FUSE — two equal-radius `r` cylinders with
/// perpendicular intersecting axes, fused.
///
/// `V = π·r²·(h₁ + h₂) − (16/3)·r³`: the two cylinder volumes (heights `h₁`,
/// `h₂` are each wall's cap-to-cap extent along its axis) minus their Steinmetz
/// intersection `16·r³/3`. Reads `r` and the two heights from the two holed
/// cylindrical walls (already verified to exist by
/// [`solid_is_steinmetz_lens_fuse`]). Returns `None` only on a topology lookup
/// failure or a malformed wall.
fn steinmetz_lens_fuse_volume(topo: &Topology, faces: &[FaceId]) -> Option<f64> {
    use std::f64::consts::PI;

    let mut r: Option<f64> = None;
    let mut heights: Vec<f64> = Vec::new();
    for &fid in faces {
        let face = topo.face(fid).ok()?;
        let FaceSurface::Cylinder(cyl) = face.surface() else {
            continue;
        };
        if face.inner_wires().is_empty() {
            continue; // Only the two holed walls.
        }
        // Equal radii: confirm the second wall matches the first.
        match r {
            None => r = Some(cyl.radius()),
            Some(r0) if (r0 - cyl.radius()).abs() > 1e-6 * r0.max(1.0) => return None,
            Some(_) => {}
        }
        // Cap-to-cap height = the axial (v) extent of the wall's outer wire.
        let wire = topo.wire(face.outer_wire()).ok()?;
        let mut v_min = f64::INFINITY;
        let mut v_max = f64::NEG_INFINITY;
        for oe in wire.edges() {
            let e = topo.edge(oe.edge()).ok()?;
            for vid in [e.start(), e.end()] {
                let p = topo.vertex(vid).ok()?.point();
                let (_, v) = cyl.project_point(p);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
        }
        if !v_min.is_finite() || !v_max.is_finite() || v_max <= v_min {
            return None;
        }
        heights.push(v_max - v_min);
    }
    let r = r?;
    if heights.len() != 2 {
        return None;
    }
    let v_cyls = PI * r * r * (heights[0] + heights[1]);
    let v_steinmetz = 16.0 / 3.0 * r * r * r;
    Some(v_cyls - v_steinmetz)
}

/// Whether the solid is the STEINMETZ LENS FUSE — two equal-radius cylinders
/// with PERPENDICULAR, INTERSECTING axes, fused — for which the volume has the
/// EXACT closed form [`steinmetz_lens_fuse_volume`].
///
/// Validated by both topology AND geometry, so a different equal-radius two-
/// cylinder fuse with the same topology (e.g. oblique or parallel-offset axes,
/// whose intersection is NOT `16r³/3`) is rejected and defers to tessellation:
///   * exactly two cylindrical faces, each carrying inner wires (the two seam
///     ellipses as holes); every other face planar (the four end caps);
///   * the two holed walls SHARE their inner-wire edges (the same seam ellipses
///     bound both);
///   * the two cylinders are EQUAL RADIUS, their axes PERPENDICULAR
///     (`|a₁·a₂| ≈ 0`) and INTERSECTING (closest-approach of the two axis lines
///     ≈ 0). The closed form holds only for that right-angle configuration.
///
/// An ordinary drilled cylinder has ONE holed wall (its bore rim is not shared
/// with a second cylindrical wall), so it returns `false`.
fn solid_is_steinmetz_lens_fuse(topo: &Topology, faces: &[FaceId]) -> bool {
    use std::collections::HashSet;

    let mut holed_cyl_walls: Vec<FaceId> = Vec::new();
    let mut planar_normals: Vec<Vec3> = Vec::new();
    for &fid in faces {
        let Ok(face) = topo.face(fid) else {
            return false;
        };
        match face.surface() {
            FaceSurface::Cylinder(_) if !face.inner_wires().is_empty() => holed_cyl_walls.push(fid),
            // An UNHOLED cylinder face means a third cylinder is attached (its
            // wall carries no lens hole); the lens fuse has EXACTLY two
            // cylindrical faces, both holed. Reject so its volume isn't dropped.
            FaceSurface::Cylinder(_) => return false,
            FaceSurface::Plane { normal, .. } => planar_normals.push(*normal),
            // Any sphere/cone/torus/NURBS face, or a holed non-cylinder, is not
            // the cyl∪cyl lens signature.
            _ => return false,
        }
    }
    if holed_cyl_walls.len() != 2 {
        return false;
    }
    // The two holed walls must SHARE their inner-wire edges (the seam ellipses).
    let inner_edges = |fid: FaceId| -> HashSet<usize> {
        let mut s = HashSet::new();
        if let Ok(face) = topo.face(fid) {
            for &wid in face.inner_wires() {
                if let Ok(wire) = topo.wire(wid) {
                    for oe in wire.edges() {
                        s.insert(oe.edge().index());
                    }
                }
            }
        }
        s
    };
    let a = inner_edges(holed_cyl_walls[0]);
    let b = inner_edges(holed_cyl_walls[1]);
    if a.is_empty() || a != b {
        return false;
    }

    // Geometry: equal radius, perpendicular + intersecting axes.
    let (Ok(f0), Ok(f1)) = (topo.face(holed_cyl_walls[0]), topo.face(holed_cyl_walls[1])) else {
        return false;
    };
    let (FaceSurface::Cylinder(c0), FaceSurface::Cylinder(c1)) = (f0.surface(), f1.surface())
    else {
        return false;
    };
    let Some(axis_isect) = cylinders_perpendicular_and_intersecting(c0, c1) else {
        return false;
    };

    // Account for EVERY face: the lens fuse has EXACTLY four planar caps — two
    // per cylinder, each perpendicular to its own axis (normal parallel to `a0`
    // or `a1`). Require that exact tally: a plane pointing any other way, OR an
    // extra axis-aligned plane (e.g. an attached box's face), means a foreign
    // body whose volume the two-cylinder closed form would silently drop. Reject
    // anything but exactly 2 caps per axis.
    let a0 = c0.axis();
    let a1 = c1.axis();
    // Parallelism via squared cosine (n·a)² ≥ (1−ε)²·|n|²·|a|², so a non-unit
    // (but parallel) plane normal or axis isn't spuriously rejected.
    let thr = (1.0 - 1e-6) * (1.0 - 1e-6);
    let mut caps_a0 = 0_usize;
    let mut caps_a1 = 0_usize;
    for n in &planar_normals {
        let nn = n.dot(*n);
        if nn < 1e-20 {
            return false;
        }
        let na0 = n.dot(a0);
        let na1 = n.dot(a1);
        if na0 * na0 >= thr * nn * a0.dot(a0) {
            caps_a0 += 1;
        } else if na1 * na1 >= thr * nn * a1.dot(a1) {
            caps_a1 += 1;
        } else {
            return false;
        }
    }
    if caps_a0 != 2 || caps_a1 != 2 {
        return false;
    }

    // Non-truncation: the closed form `−16r³/3` is the INFINITE-cylinder
    // Steinmetz solid, valid only when neither finite wall is cut shorter than
    // the lens. Each wall must extend ≥ r past the axis-intersection point on
    // both sides (project the intersection onto each axis; both caps ≥ r away).
    let r = c0.radius();
    wall_extends_past(topo, holed_cyl_walls[0], c0, axis_isect, r)
        && wall_extends_past(topo, holed_cyl_walls[1], c1, axis_isect, r)
}

/// Whether a cylinder wall's cap-to-cap extent reaches at least `r` past the
/// axis-intersection point on BOTH sides (the non-truncation precondition for
/// the right-angle Steinmetz closed form). Reads the wall's axial (v) extent
/// from its outer wire and compares against the intersection's axial coordinate.
fn wall_extends_past(
    topo: &Topology,
    wall: FaceId,
    cyl: &remus_math::surfaces::CylindricalSurface,
    axis_isect: Point3,
    r: f64,
) -> bool {
    let Ok(face) = topo.face(wall) else {
        return false;
    };
    let Ok(wire) = topo.wire(face.outer_wire()) else {
        return false;
    };
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for oe in wire.edges() {
        let Ok(e) = topo.edge(oe.edge()) else {
            return false;
        };
        for vid in [e.start(), e.end()] {
            let Ok(v) = topo.vertex(vid) else {
                return false;
            };
            let (_, vv) = cyl.project_point(v.point());
            v_min = v_min.min(vv);
            v_max = v_max.max(vv);
        }
    }
    if !v_min.is_finite() || !v_max.is_finite() {
        return false;
    }
    let (_, v_isect) = cyl.project_point(axis_isect);
    let tol = 1e-6 * r.max(1.0);
    v_isect - v_min >= r - tol && v_max - v_isect >= r - tol
}

/// If two cylinders are equal-radius with perpendicular, intersecting axes —
/// the geometric precondition for the right-angle Steinmetz closed form —
/// returns their axis-intersection point; otherwise `None`.
fn cylinders_perpendicular_and_intersecting(
    c0: &remus_math::surfaces::CylindricalSurface,
    c1: &remus_math::surfaces::CylindricalSurface,
) -> Option<Point3> {
    let r0 = c0.radius();
    if (r0 - c1.radius()).abs() > 1e-6 * r0.max(1.0) {
        return None; // Unequal radius.
    }
    let a0 = c0.axis();
    let a1 = c1.axis();
    if a0.dot(a1).abs() > 1e-6 {
        return None; // Not perpendicular.
    }
    // Closest approach of the two axis lines (perpendicular ⇒ the system
    // decouples): s* = −(w0·a0), t* = w0·a1, where w0 = o0 − o1.
    let o0 = c0.origin();
    let o1 = c1.origin();
    let w0 = Vec3::new(o0.x() - o1.x(), o0.y() - o1.y(), o0.z() - o1.z());
    let s = -w0.dot(a0);
    let t = w0.dot(a1);
    let p0 = Point3::new(
        o0.x() + a0.x() * s,
        o0.y() + a0.y() * s,
        o0.z() + a0.z() * s,
    );
    let p1 = Point3::new(
        o1.x() + a1.x() * t,
        o1.y() + a1.y() * t,
        o1.z() + a1.z() * t,
    );
    if (p0 - p1).length() <= 1e-6 * r0.max(1.0) {
        // Both closest points coincide ⇒ the axes meet; return the midpoint.
        Some(Point3::new(
            0.5 * (p0.x() + p1.x()),
            0.5 * (p0.y() + p1.y()),
            0.5 * (p0.z() + p1.z()),
        ))
    } else {
        None
    }
}

/// Try to compute the volume of a solid analytically by detecting known
/// primitive shapes (sphere, cylinder, cone/frustum, torus).
///
/// Returns `None` if the solid is not a recognized pure primitive, in which
/// case the caller should fall back to tessellation.
///
/// Detection rules (single pass over shell faces):
/// - Any `Nurbs` face -> `None` (fall back)
/// - All faces are `Sphere` -> sphere formula `(4/3)pi*r^3`
/// - Exactly 1 `Cylinder` + >=1 `Plane` caps, 0 other analytic -> `pi*r^2*h`
/// - Exactly 1 `Cone` + <=2 `Plane` caps, 0 other analytic -> cone/frustum formula
///   (cap radii are read from the `Circle3D` edges of the cap faces)
/// - Exactly 1 `Torus` + 0 planes, 0 other analytic -> `2*pi^2*R*r^2`
#[allow(clippy::too_many_lines)]
fn try_analytic_solid_volume(topo: &Topology, solid: SolidId) -> Option<f64> {
    use std::f64::consts::PI;

    let solid_data = topo.solid(solid).ok()?;
    // Every closed form below is the volume of a WHOLE primitive, derived from
    // the outer shell alone. An inner shell is a cavity: its faces are part of
    // the boundary and remove material, but nothing here can see them, so the
    // recogniser would happily match the outer wall and report the body as if
    // it had never been hollowed. Refuse, and let the shell-complete paths the
    // caller tries next (which enumerate `explorer::solid_faces`) measure it.
    if !solid_data.inner_shells().is_empty() {
        return None;
    }
    let shell = topo.shell(solid_data.outer_shell()).ok()?;

    let mut sphere_r: Option<f64> = None;
    let mut cyl: Option<(Point3, Vec3, f64)> = None; // (origin, axis, radius)
    let mut cone_params: Option<(Point3, Vec3)> = None; // (apex, axis)
    let mut torus_params: Option<(f64, f64)> = None; // (major_r, minor_r)
    let mut torus_face_id: Option<FaceId> = None;
    let mut planes: Vec<(Vec3, f64)> = Vec::new();
    let mut plane_face_ids: Vec<FaceId> = Vec::new();

    for &fid in shell.faces() {
        let face = topo.face(fid).ok()?;
        // A holed analytic face means the solid is bored/pocketed; the closed-form
        // primitive volumes below integrate the surface as if the hole were filled.
        // Defer the whole solid to the hole-aware tessellation path. (The validated
        // Steinmetz lens fuse is handled by `analytic_faces_solid_volume`, which the
        // caller tries after this returns `None`.)
        if !face.inner_wires().is_empty() {
            return None;
        }
        match face.surface() {
            FaceSurface::Nurbs(_) => return None,
            FaceSurface::Plane { normal, d } => {
                planes.push((*normal, *d));
                plane_face_ids.push(fid);
            }
            FaceSurface::Sphere(s) => {
                let r = s.radius();
                match sphere_r {
                    None => sphere_r = Some(r),
                    // Multiple sphere faces must all share the same radius.
                    Some(existing) if (r - existing).abs() > existing * 1e-6 => return None,
                    Some(_) => {}
                }
            }
            FaceSurface::Cylinder(c) => {
                if cyl.is_some() {
                    return None;
                }
                cyl = Some((c.origin(), c.axis(), c.radius()));
            }
            FaceSurface::Cone(c) => {
                if cone_params.is_some() {
                    return None;
                }
                cone_params = Some((c.apex(), c.axis()));
            }
            FaceSurface::Torus(t) => {
                if torus_params.is_some() {
                    return None;
                }
                torus_params = Some((t.major_radius(), t.minor_radius()));
                torus_face_id = Some(fid);
            }
        }
    }

    if let Some(r) = sphere_r
        && cyl.is_none()
        && cone_params.is_none()
        && torus_params.is_none()
        && planes.is_empty()
    {
        // A non-uniform scale transforms vertices but leaves the sphere
        // surface radius unchanged, making the analytic formula wrong.
        let sphere_faces: Vec<_> = shell.faces().to_vec();
        let center = if let Ok(f) = topo.face(sphere_faces[0]) {
            if let FaceSurface::Sphere(s) = f.surface() {
                s.center()
            } else {
                return None;
            }
        } else {
            return None;
        };
        let mut max_dist = 0.0_f64;
        let mut min_dist = f64::INFINITY;
        for &fid in &sphere_faces {
            if let Ok(face) = topo.face(fid)
                && let Ok(wire) = topo.wire(face.outer_wire())
            {
                for oe in wire.edges() {
                    if let Ok(e) = topo.edge(oe.edge())
                        && let Ok(v) = topo.vertex(e.start())
                    {
                        let d = (v.point() - center).length();
                        max_dist = max_dist.max(d);
                        min_dist = min_dist.min(d);
                    }
                }
            }
        }
        // If all vertices are equidistant (within 1%), use analytic formula
        if (max_dist - min_dist).abs() < r * 0.01 {
            return Some(4.0 / 3.0 * PI * r * r * r);
        }
        // Non-uniform scale detected -- fall through to tessellation
        return None;
    }

    // A pure cylinder has exactly 1 cylindrical face and 2 planar caps.
    // If there are more than 2 planes the solid is compound (e.g. a box
    // with a drilled hole has 1 cylindrical hole-wall + 6 box faces).
    // In the compound case the cylindrical face is a concave inner surface
    // and the formula pi*r^2*h would compute the cylinder volume, not the solid.
    if let Some((origin, axis, r)) = cyl
        && cone_params.is_none()
        && torus_params.is_none()
        && sphere_r.is_none()
        && planes.len() == 2
    {
        let origin_vec = Vec3::new(origin.x(), origin.y(), origin.z());
        let mut ts = cap_t_values(origin_vec, axis, &planes);
        if ts.len() >= 2 {
            ts.sort_by(f64::total_cmp);
            if let (Some(&t_min), Some(&t_max)) = (ts.first(), ts.last()) {
                return Some(PI * r * r * (t_max - t_min));
            }
        }
    }

    // Cap radii are read directly from the Circle3D edges of the cap faces,
    // bypassing the ConicalSurface parameterization entirely. Heights are
    // derived from the circle centers projected onto the cone axis.
    if let Some((apex, axis)) = cone_params
        && cyl.is_none()
        && torus_params.is_none()
        && sphere_r.is_none()
    {
        let apex_vec = Vec3::new(apex.x(), apex.y(), apex.z());

        // Collect (circle_center, radius) from each plane cap face.
        let mut cap_circles: Vec<(Point3, f64)> = Vec::new();
        for &fid in &plane_face_ids {
            if let Some(cap) = find_cap_circle(topo, fid) {
                cap_circles.push(cap);
            }
        }

        // If any cap face did not yield a circle, the cone is degenerate or
        // unsupported -- fall back to tessellation rather than silently wrong answer.
        if cap_circles.len() != plane_face_ids.len() {
            return None;
        }

        match cap_circles.as_slice() {
            [(c, r)] => {
                // Pointed cone: h = distance from apex to cap center along axis.
                let c_vec = Vec3::new(c.x(), c.y(), c.z());
                let h = (c_vec - apex_vec).dot(axis).abs();
                return Some(PI / 3.0 * r * r * h);
            }
            [(c1, r1), (c2, r2)] => {
                // Frustum: h = distance between cap centers projected onto axis.
                let c1_vec = Vec3::new(c1.x(), c1.y(), c1.z());
                let c2_vec = Vec3::new(c2.x(), c2.y(), c2.z());
                let h = (c2_vec - c1_vec).dot(axis).abs();
                return Some(PI * h / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2));
            }
            _ => {}
        }
    }

    if let Some((r_major, r_minor)) = torus_params
        && cyl.is_none()
        && cone_params.is_none()
        && sphere_r.is_none()
    {
        if planes.is_empty() {
            return Some(2.0 * PI * PI * r_major * r_minor * r_minor);
        }
        // A partial-revolve torus sector: one trimmed band + two planar disc
        // caps that both contain the torus axis.
        if planes.len() == 2
            && let Some(tid) = torus_face_id
            && let Some(v) =
                partial_torus_sector_volume(topo, tid, r_major, r_minor, &planes, &plane_face_ids)
        {
            return Some(v);
        }
        return None;
    }

    None
}

/// Exact volume of a partial-revolve torus sector: one `Torus` band trimmed to
/// a sweep `Δu` plus two planar disc caps whose planes contain the torus axis,
/// each bounded by a single closed tube circle (radius = minor, centre at
/// major distance from the axis). `V = (Δu / 2π) · 2π²·R·r² = π·R·r²·Δu`.
///
/// `Δu` is read from the band's axis-centred seam arc, whose CCW start→end
/// span is the swept angle by the codebase-wide arc convention, and
/// cross-checked against the dihedral between the cap planes. Returns `None`
/// (defer to tessellation) unless every guard pins the structure.
fn partial_torus_sector_volume(
    topo: &Topology,
    torus_face: FaceId,
    r_major: f64,
    r_minor: f64,
    planes: &[(Vec3, f64)],
    plane_face_ids: &[FaceId],
) -> Option<f64> {
    use std::f64::consts::{PI, TAU};

    let face = topo.face(torus_face).ok()?;
    let FaceSurface::Torus(t) = face.surface() else {
        return None;
    };
    let center = t.center();
    let center_vec = Vec3::new(center.x(), center.y(), center.z());
    let axis_d = t.z_axis().normalize().ok()?;
    let tol = 1e-7 * r_major.max(1.0);

    // Both cap planes must contain the torus axis line.
    for &(n, d) in planes {
        let n_unit = n.normalize().ok()?;
        if n_unit.dot(axis_d).abs() > 1e-9 {
            return None;
        }
        if (n.dot(center_vec) - d).abs() > tol {
            return None;
        }
    }

    // Each cap must be a single closed tube circle: radius = minor, centre at
    // major distance from the axis.
    for &fid in plane_face_ids {
        let cap = topo.face(fid).ok()?;
        if !cap.inner_wires().is_empty() {
            return None;
        }
        let wire = topo.wire(cap.outer_wire()).ok()?;
        let edges = wire.edges();
        if edges.len() != 1 {
            return None;
        }
        let edge = topo.edge(edges[0].edge()).ok()?;
        if edge.start() != edge.end() {
            return None;
        }
        let remus_topology::edge::EdgeCurve::Circle(c) = edge.curve() else {
            return None;
        };
        if (c.radius() - r_minor).abs() > tol {
            return None;
        }
        let off = c.center() - center;
        let perp = off - axis_d * off.dot(axis_d);
        if (perp.length() - r_major).abs() > tol {
            return None;
        }
    }

    // The band's seam: a non-closed Circle edge centred ON the axis. Its CCW
    // start→end span is the swept angle.
    let wire = topo.wire(face.outer_wire()).ok()?;
    let mut sweep: Option<f64> = None;
    for oe in wire.edges() {
        let edge = topo.edge(oe.edge()).ok()?;
        if edge.start() == edge.end() {
            continue;
        }
        let remus_topology::edge::EdgeCurve::Circle(c) = edge.curve() else {
            continue;
        };
        let off = c.center() - center;
        let perp = off - axis_d * off.dot(axis_d);
        // The seam must be centred on the axis AND wound about +axis — an
        // antiparallel normal would make the CCW span read the complement.
        if perp.length() > tol
            || c.normal().cross(axis_d).length() > 1e-9
            || c.normal().dot(axis_d) < 0.0
        {
            continue;
        }
        let sp = topo.vertex(edge.start()).ok()?.point();
        let ep = topo.vertex(edge.end()).ok()?.point();
        let delta = (c.project(ep) - c.project(sp)).rem_euclid(TAU);
        match sweep {
            None => sweep = Some(delta),
            Some(prev) if (prev - delta).abs() < 1e-9 => {}
            Some(_) => return None,
        }
    }
    let du = sweep?;
    if !(1e-12..=TAU - 1e-12).contains(&du) {
        return None;
    }

    // Cross-check: the outward cap normals of a Δu sector satisfy
    // n₀·n₁ = −cos(Δu) (n₀ = −t̂, n₁ = t̂ rotated by Δu about the axis).
    let n0 = planes[0].0.normalize().ok()?;
    let n1 = planes[1].0.normalize().ok()?;
    if (n0.dot(n1) + du.cos()).abs() > 1e-6 {
        return None;
    }

    Some(PI * r_major * r_minor * r_minor * du)
}

/// Minimum |n . axis| for a plane to be considered a perpendicular cap face
/// (i.e. the plane normal is within ~8 deg of the axis direction).
const AXIS_PARALLEL_MIN_DOT: f64 = 0.99;

/// Compute signed distances along `axis` from `ref_pt` to cap planes that are
/// roughly perpendicular to the axis (`|n . axis| > AXIS_PARALLEL_MIN_DOT`).
///
/// For a plane `n . P = d`, the intersection with the line `ref_pt + t * axis`
/// satisfies `t = (d - n . ref_pt) / (n . axis)`.
fn cap_t_values(ref_pt: Vec3, axis: Vec3, planes: &[(Vec3, f64)]) -> Vec<f64> {
    let mut ts = Vec::new();
    for &(n, d) in planes {
        let nd = n.dot(axis);
        if nd.abs() > AXIS_PARALLEL_MIN_DOT {
            ts.push((d - n.dot(ref_pt)) / nd);
        }
    }
    ts
}

/// Search a face's outer wire for a `Circle3D` edge and return its `(center, radius)`.
///
/// Used by the cone volume formula to read cap radii directly from the geometry
/// rather than inferring them from the `ConicalSurface` parameterization.
fn find_cap_circle(topo: &Topology, face_id: FaceId) -> Option<(Point3, f64)> {
    let face = topo.face(face_id).ok()?;
    let wire = topo.wire(face.outer_wire()).ok()?;
    for oe in wire.edges() {
        // Use let-else so a missing edge skips to the next iteration
        // rather than returning None for the whole face.
        let Ok(edge) = topo.edge(oe.edge()) else {
            continue;
        };
        if let remus_topology::edge::EdgeCurve::Circle(c) = edge.curve() {
            return Some((c.center(), c.radius()));
        }
    }
    None
}

/// Clamp the tessellation deflection used for volume so curved faces are
/// sampled finely enough for an accurate boundary integral.
///
/// A coarse preview deflection inscribes too few facets in a curved face and
/// under-counts its volume; volume is a precise query, so cap the deflection at
/// a small fraction of the solid's curvature-aware bounding-box diagonal.
/// Using only topology vertices is not sufficient: a closed circular edge can
/// have one coincident start/end vertex even when its radius is enormous. Such
/// an under-estimate would force an unnecessarily tiny deflection and allow a
/// compact model to exhaust resources during tessellation. Never coarsens a
/// finer request, and falls back to `requested` if the extent cannot be
/// determined.
///
/// A solid-extent scale (rather than per-curved-face curvature radius) is used
/// deliberately: it keeps the deflection consistent between a sub-solid and a
/// boolean result containing it, preserving the `volume(A ∪ B) == volume(A) +
/// volume(B)` invariant for coincident-contact fuses. A curvature-radius cap
/// would tessellate a shared face differently in each context and break it.
pub(super) fn volume_tessellation_deflection(
    topo: &Topology,
    solid: SolidId,
    requested: f64,
) -> f64 {
    let Ok(aabb) = super::solid_bounding_box(topo, solid) else {
        return requested;
    };
    let diag = (aabb.max - aabb.min).length();
    if !diag.is_finite() || diag <= 0.0 {
        return requested;
    }
    requested.min((diag * 5e-5).max(1e-9))
}

/// The signed volume a shell's faces enclose, `(1/3) integral P.n dA` summed
/// over the exact face geometry.
///
/// Positive when the shell is wound OUTWARD, negative when it faces inward.
/// `integrate_face` already applies each face's stored reversal, so a cavity
/// shell — every face reversed — comes back negative, which is what makes it
/// subtract in [`solid_volume`].
///
/// Returns `None` when any face fails to integrate, which is a "cannot say"
/// rather than a verdict: callers must not read that as "correctly wound".
pub fn shell_signed_volume(
    topo: &Topology,
    shell: remus_topology::shell::ShellId,
    gauss_order: usize,
) -> Option<f64> {
    let mut total = 0.0;
    for &fid in topo.shell(shell).ok()?.faces() {
        total += remus_check::properties::face_integrator::integrate_face(topo, fid, gauss_order)
            .ok()?
            .volume;
    }
    Some(total)
}

/// The smallest volume this model can distinguish from zero: the cube of its
/// own vertex-bounding-box diagonal, scaled by a dimensionless epsilon.
///
/// A volume is `L^3`, so the yardstick has to be too — an absolute `1e-9 mm^3`
/// would call a 0.001x model inverted and a 1000x one flat. Returns `None` when
/// the model has no measurable extent.
pub fn negligible_volume(topo: &Topology, solid: SolidId) -> Option<f64> {
    /// Dimensionless: the fraction of the model's own extent-cubed below which
    /// a signed volume says nothing about orientation.
    const RELATIVE_FLOOR: f64 = 1e-9;

    let pts = collect_solid_vertex_points(topo, solid).ok()?;
    let (&first, rest) = pts.split_first()?;
    let (mut lo, mut hi) = (first, first);
    for p in rest {
        lo = Point3::new(lo.x().min(p.x()), lo.y().min(p.y()), lo.z().min(p.z()));
        hi = Point3::new(hi.x().max(p.x()), hi.y().max(p.y()), hi.z().max(p.z()));
    }
    let diag = (hi - lo).length();
    (diag.is_finite() && diag > 0.0).then_some(diag * diag * diag * RELATIVE_FLOOR)
}

/// Whether the solid's outer shell is turned inside out.
///
/// A shell can be closed, 2-manifold and consistently wound and still face
/// inward — remus#59's segmented revolve built exactly that, and nothing in
/// the measurement layer could see it, because [`solid_volume`] reports the
/// magnitude of its integral and so reads an inverted body at its correct
/// positive volume. The sign is what an STL facet normal is derived from, so
/// such a body exports inside out.
///
/// Returns `false` when the answer cannot be established — a face that will not
/// integrate, or a body with no measurable extent. This is a detector, not a
/// proof of correctness: `false` means "not shown to be inverted".
///
/// # Errors
///
/// Returns an error if topology lookups fail.
pub fn solid_is_inverted(topo: &Topology, solid: SolidId) -> Result<bool, crate::OperationsError> {
    let outer = topo.solid(solid)?.outer_shell();
    let order = remus_check::properties::PropertiesOptions::default().gauss_order;
    let (Some(signed), Some(floor)) = (
        shell_signed_volume(topo, outer, order),
        negligible_volume(topo, solid),
    ) else {
        return Ok(false);
    };
    Ok(signed < -floor)
}

/// Compute the volume of a solid using the signed tetrahedra method
/// (divergence theorem on a surface tessellation).
///
/// For each triangle `(v0, v1, v2)`, the signed volume of the
/// tetrahedron it forms with the origin is `v0 . (v1 x v2) / 6`.
///
/// For pure-primitive solids (sphere, cylinder, cone, torus), uses exact
/// analytic formulas instead of tessellation.
///
/// # Orientation
///
/// The result is a MAGNITUDE: an inside-out solid reports the same positive
/// number its correctly-wound twin would. That is deliberate — a volume is a
/// positive quantity and every caller reads it as one — but it means this
/// function cannot be used to ask whether a body is inverted. Ask
/// [`solid_is_inverted`], or run [`crate::validate::validate_solid`], which
/// reports an inverted outer shell (and a cavity wound the wrong way) as an
/// error.
///
/// # Errors
///
/// Returns an error if tessellation or topology lookups fail.
pub fn solid_volume(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    // Fast path: exact analytic formula for known primitives.
    if let Some(v) = try_analytic_solid_volume(topo, solid) {
        if std::env::var("BK_VOL_TRACE").is_ok() {
            log::debug!("VOL_TRACE try_analytic -> {v}");
        }
        return Ok(v);
    }

    // Fast path: a solid whose faces are ALL analytic (planes + quadrics,
    // no NURBS) integrates exactly via per-face Gauss quadrature on the
    // analytic surfaces — orientation-aware and immune to the inscribed-mesh
    // undercount and the degenerate-UV annular-band over-count that the
    // tessellation paths below suffer on bored quadrics (e.g. a cylinder
    // drilled through a sphere).
    if let Some(v) = analytic_faces_solid_volume(topo, solid)? {
        if std::env::var("BK_VOL_TRACE").is_ok() {
            log::debug!("VOL_TRACE analytic_faces -> {v}");
        }
        return Ok(v);
    }

    // A surface-of-revolution solid that is fully analytic (cone / cylinder /
    // torus walls with concentric circular disc-cap planes, no NURBS) integrates
    // with NO tessellation — exact, immune to the inscribed-mesh undercount. The
    // recogniser is deliberately narrow (concentric caps about one axis) so it
    // does NOT catch boolean results that merely happen to have arc-bounded
    // planar faces (rounded-rect caps, arc-frame lips).
    if let Some(v) = analytic_revolution_solid_volume(topo, solid) {
        if std::env::var("BK_VOL_TRACE").is_ok() {
            log::debug!("VOL_TRACE revolution -> {v}");
        }
        return Ok(v);
    }

    // Volume integrates the boundary, so curved faces must be tessellated
    // finely or the inscribed mesh under-counts them (a swept cylinder or a
    // box with a cylindrical hole measures ~1-2% low at a coarse preview
    // deflection). Clamp the deflection to a small fraction of the solid's
    // extent — never coarsening a finer request — so the volume is accurate
    // regardless of the (preview-tuned) deflection the caller passes.
    let deflection = volume_tessellation_deflection(topo, solid, deflection);

    // A scalloped sphere collar (box ∩ sphere) cannot be per-face tessellated
    // watertight (its band path needs the solid's shared boundary vertices), and
    // its analytic integral is the hard u-dependent lune trim we defer. The
    // whole-solid mesh IS watertight, so take the divergence-theorem volume off
    // that closed mesh.
    if solid_has_scalloped_sphere_collar(topo, solid)? {
        let mesh = tessellate::tessellate_solid(topo, solid, deflection)?;
        if !mesh.indices.is_empty() && mesh_boundary_edge_count(&mesh) == 0 {
            return Ok(signed_volume_from_mesh(&mesh));
        }
        // Non-watertight mesh: fall through to the generic paths below rather
        // than return a leaky volume.
    }

    // A torus notch band (torus − box) likewise has no closed-form volume and
    // can't be per-face tessellated watertight (the band wraps the tube and
    // shares vertices with the notch walls). The whole-solid mesh IS watertight
    // (the structured notch-band tessellator), so take the divergence-theorem
    // volume off it. Per-face summation would under-count (the band's own per-
    // face mesh isn't closed).
    if solid_has_torus_notch_band(topo, solid) {
        let mesh = tessellate::tessellate_solid(topo, solid, deflection)?;
        if !mesh.indices.is_empty() && mesh_boundary_edge_count(&mesh) == 0 {
            return Ok(signed_volume_from_mesh(&mesh));
        }
        // Non-watertight mesh: fall through rather than return a leaky volume.
    }

    // Fast path: for solids made entirely of planar triangular faces
    // (e.g. mesh imports), compute volume directly from face geometry.
    // This avoids re-tessellation which has known WASM winding issues.
    if let Ok(v) = solid_volume_from_faces(topo, solid, deflection) {
        return Ok(v);
    }

    // Planar polygon volume (Newell area) is disabled: GFA boolean results
    // go through merge_duplicate_edges which can create crossed polygon
    // winding, making Newell area wrong. Always use tessellation-based
    // volume which handles all cases correctly.

    // For solids with faces that have inner wires (holes from boolean ops)
    // or reversed non-planar faces (inner walls from shell/boolean operations),
    // use direct per-face tessellation with signed-volume summation.
    // tessellate() handles face reversal (flips winding + normals), so raw
    // signed tets are correct even without a globally watertight mesh.
    //
    // This scan, and the two below it, deliberately look at the OUTER shell
    // only. They are not integrating — they are choosing between two paths that
    // are both shell-complete, and the fallthrough (the whole-solid mesh) is the
    // safer of the two. Widening them to every shell re-routes bodies that the
    // whole-solid mesh already measures correctly: a box with an enclosed
    // spherical void has a clean outer shell, but its cavity contributes a
    // reversed FULL sphere face, which would flip this to `true` and hand the
    // body to the direct path — where `analytic_sphere_signed_volume` reads the
    // patch window off the wire's vertices, finds a closed seam with no distinct
    // u extent, and credits the whole void with zero. Measured: the void's
    // 523.6 mm³ vanished and a 20 mm cube with a r5 void read a flat 8000.
    let needs_direct_tessellation = {
        let s = topo.solid(solid)?;
        let sh = topo.shell(s.outer_shell())?;
        sh.faces().iter().any(|&fid| {
            topo.face(fid).is_ok_and(|f| {
                !f.inner_wires().is_empty()
                    || (f.is_reversed() && !matches!(f.surface(), FaceSurface::Plane { .. }))
            })
        })
    };
    // Per-face summation is only exact when the faces' own meshes tile the
    // same closed surface the solid does. That holds for planes and for the
    // quadrics, which the direct path integrates analytically anyway — but not
    // for NURBS, whose per-face mesh need not reproduce the trimmed patch the
    // solid mesh carries. Summing those leaves the integral open: a filleted
    // L-blank whose blend wall came out NURBS measured 49.1 mm³ removed by the
    // per-face path against 34.7 from the solid's own (watertight) mesh.
    //
    // So when a NURBS face is present, prefer the closed whole-solid mesh —
    // the same reasoning the scalloped-sphere and torus-notch cases above
    // already apply. Solids without NURBS keep the existing routing, so the
    // common bored-solid path costs no extra tessellation.
    if needs_direct_tessellation {
        let has_nurbs = {
            let s = topo.solid(solid)?;
            let sh = topo.shell(s.outer_shell())?;
            sh.faces().iter().any(|&fid| {
                topo.face(fid)
                    .is_ok_and(|f| matches!(f.surface(), FaceSurface::Nurbs(_)))
            })
        };
        // The same reasoning covers a sphere patch that is not a latitude BAND.
        // `volume_from_direct_face_tessellation` integrates a sphere face
        // analytically over the [u] x [v] box its wire spans, which is the
        // patch itself only when the wire is constant-v. A vertex-blend corner
        // ball — a spherical triangle closed by three great-circle arcs — fills
        // barely a quarter of that box, so the direct path credits it with four
        // times its real surface and the solid measures much too large. A plain
        // filleted box never noticed: with no inner wire anywhere it takes the
        // whole-solid mesh above. Drill one hole and the same body routed here
        // instead, and its corner fillets read as removing half of what the
        // undrilled plate's removed.
        let has_non_band_sphere = {
            let s = topo.solid(solid)?;
            let sh = topo.shell(s.outer_shell())?;
            let mut found = false;
            for &fid in sh.faces() {
                let face = topo.face(fid)?;
                if let FaceSurface::Sphere(sphere) = face.surface()
                    && !sphere_outer_wire_constant_v(topo, fid, sphere)?
                {
                    found = true;
                    break;
                }
            }
            found
        };
        if has_nurbs || has_non_band_sphere {
            let mesh = tessellate::tessellate_solid(topo, solid, deflection)?;
            if !mesh.indices.is_empty() && mesh_boundary_edge_count(&mesh) == 0 {
                let vol = signed_volume_from_mesh(&mesh);
                if vol > 1e-12 {
                    return Ok(vol);
                }
            }
        }
        return volume_from_direct_face_tessellation(topo, solid, deflection);
    }

    // Try watertight tessellation -- gives correct volume via signed tetrahedra
    // since the mesh is closed.
    let mesh = tessellate::tessellate_solid(topo, solid, deflection)?;
    if !mesh.indices.is_empty() {
        let vol = signed_volume_from_mesh(&mesh);
        if vol > 1e-12 {
            return Ok(vol);
        }
    }

    // Fallback: per-face tessellation with centroid-based winding correction.
    volume_from_per_face_tessellation(topo, solid, deflection)
}

/// Divergence-theorem volume of a solid WITHOUT the absolute value, so the sign
/// reports shell orientation: positive for an outward-oriented (material-inside)
/// solid, negative for an inverted one.
///
/// [`solid_volume`] deliberately reports a magnitude, which makes it blind to a
/// globally inverted shell — the defect class that drops a boolean to the mesh
/// fallback with no volume error to show for it.
///
/// # Errors
///
/// Returns an error if the solid cannot be tessellated.
pub fn oriented_solid_volume(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    let mesh = tessellate::tessellate_solid(topo, solid, deflection)?;
    let idx = &mesh.indices;
    let pos = &mesh.positions;
    let mut total = 0.0;
    for t in 0..idx.len() / 3 {
        let v0 = pos[idx[t * 3] as usize];
        let v1 = pos[idx[t * 3 + 1] as usize];
        let v2 = pos[idx[t * 3 + 2] as usize];
        let a = Vec3::new(v0.x(), v0.y(), v0.z());
        let b = Vec3::new(v1.x(), v1.y(), v1.z());
        let c = Vec3::new(v2.x(), v2.y(), v2.z());
        total += a.dot(b.cross(c));
    }
    Ok(total / 6.0)
}

/// Compute signed volume from a watertight triangle mesh using
/// the divergence theorem (signed tetrahedra method).
fn signed_volume_from_mesh(mesh: &tessellate::TriangleMesh) -> f64 {
    let idx = &mesh.indices;
    let pos = &mesh.positions;
    let tri_count = idx.len() / 3;

    let mut total = 0.0;
    for t in 0..tri_count {
        let v0 = pos[idx[t * 3] as usize];
        let v1 = pos[idx[t * 3 + 1] as usize];
        let v2 = pos[idx[t * 3 + 2] as usize];

        let a = Vec3::new(v0.x(), v0.y(), v0.z());
        let b = Vec3::new(v1.x(), v1.y(), v1.z());
        let c = Vec3::new(v2.x(), v2.y(), v2.z());

        total += a.dot(b.cross(c));
    }

    (total / 6.0).abs()
}

/// Compute volume by tessellating each face independently and summing
/// signed tetrahedra contributions (divergence theorem).
///
/// `tessellate()` already handles face reversal (flipping triangle
/// winding for reversed faces), so the raw signed tetrahedra sum
/// produces the correct result without any winding heuristic.
fn volume_from_per_face_tessellation(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    // Outer shell plus every cavity shell. `tessellate()` applies the face's
    // stored reversal to the winding, so a cavity face's signed tetrahedra
    // subtract the void without any extra sign handling here.
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;

    let mut total: f64 = 0.0;
    for fid in faces {
        let mesh = tessellate::tessellate(topo, fid, deflection)?;
        let idx = &mesh.indices;
        if std::env::var("BK_VOL_TRACE").is_ok() {
            log::debug!("VOL_TRACE direct plane face {fid:?} tris={}", idx.len() / 3);
        }
        let pos = &mesh.positions;
        let tri_count = idx.len() / 3;

        for t in 0..tri_count {
            let v0 = pos[idx[t * 3] as usize];
            let v1 = pos[idx[t * 3 + 1] as usize];
            let v2 = pos[idx[t * 3 + 2] as usize];

            let a = Vec3::new(v0.x(), v0.y(), v0.z());
            let b = Vec3::new(v1.x(), v1.y(), v1.z());
            let c = Vec3::new(v2.x(), v2.y(), v2.z());

            total += a.dot(b.cross(c));
        }
    }

    let signed_volume = total / 6.0;
    if signed_volume < 0.0 {
        log::debug!(
            "volume_from_per_face_tessellation: raw signed volume is negative ({signed_volume:.6}), \
             possible face orientation issue"
        );
    }
    Ok(signed_volume.abs())
}

/// Exact signed volume contribution of a cylindrical face via the
/// divergence theorem: `V = (1/3) integral P.n dA`.
///
/// For a cylinder parameterised as
///   `P(u,v) = O + r*(cos u * ex + sin u * ey) + v * a`
/// the outward normal is `n = cos u * ex + sin u * ey`, dA = r du dv.
///
/// Integrating analytically over `u in [u1,u2], v in [v1,v2]`:
///   `V = (r/3) * h * [ ox*(sin u2 - sin u1) + oy*(-cos u2 + cos u1) + r*(u2 - u1) ]`
/// where `ox = O.ex`, `oy = O.ey`, `h = v2 - v1`.
///
/// For a reversed face the contribution is negated.
fn analytic_cylinder_signed_volume(
    topo: &Topology,
    face_id: FaceId,
) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let cyl = match face.surface() {
        FaceSurface::Cylinder(c) => c,
        _ => {
            return Err(crate::OperationsError::InvalidInput {
                reason: "analytic_cylinder_signed_volume requires a cylinder face".into(),
            });
        }
    };

    let wire = topo.wire(face.outer_wire())?;
    let mut u_vals = Vec::new();
    let mut v_vals = Vec::new();
    for oe in wire.edges() {
        if let Ok(edge) = topo.edge(oe.edge()) {
            for &vid in &[edge.start(), edge.end()] {
                if let Ok(vtx) = topo.vertex(vid) {
                    let (u, v) = cyl.project_point(vtx.point());
                    u_vals.push(u);
                    v_vals.push(v);
                }
            }
            // Sample circle-edge midpoints for angular coverage.
            if !edge.is_closed()
                && let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve()
                && let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end()))
            {
                let ts = circle.project(sv.point());
                let te = circle.project(ev.point());
                // Choose the shorter arc for the midpoint.
                let fwd = (te - ts).rem_euclid(std::f64::consts::TAU);
                let mid_t = if fwd <= std::f64::consts::PI {
                    ts + fwd * 0.5
                } else {
                    ts - (std::f64::consts::TAU - fwd) * 0.5
                };
                let mid = circle.evaluate(mid_t);
                let (u, _) = cyl.project_point(mid);
                u_vals.push(u);
            }
            // A revolution-band boundary is a rational NURBS arc, not an
            // `EdgeCurve::Circle`. Sample its domain midpoint too, or a partial
            // (sub-2π) band has only its two endpoint angles, `compute_angular_range`
            // falls back to the full 2π, and the band over-counts (gh #968).
            if !edge.is_closed()
                && let remus_topology::edge::EdgeCurve::NurbsCurve(nc) = edge.curve()
            {
                let (t0, t1) = nc.domain();
                let (u, _) = cyl.project_point(nc.evaluate(f64::midpoint(t0, t1)));
                u_vals.push(u);
            }
        }
    }

    let v_min = v_vals.iter().copied().fold(f64::INFINITY, f64::min);
    let v_max = v_vals.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let h = v_max - v_min;
    if h.abs() < 1e-15 {
        return Ok(0.0);
    }

    let u_range = compute_angular_range(&mut u_vals);

    let r = cyl.radius();
    let x_axis = cyl.x_axis();
    let y_axis = cyl.y_axis();

    let o_vec = Vec3::new(cyl.origin().x(), cyl.origin().y(), cyl.origin().z());
    let ox = o_vec.dot(x_axis);
    let oy = o_vec.dot(y_axis);

    let (u1, u2) = u_range;
    let (sin1, cos1) = u1.sin_cos();
    let (sin2, cos2) = u2.sin_cos();

    let vol = (r / 3.0) * h * (ox * (sin2 - sin1) + oy * (-cos2 + cos1) + r * (u2 - u1));

    Ok(if face.is_reversed() { -vol } else { vol })
}

/// Exact signed volume contribution of a PLANAR face whose boundary is made of
/// straight and circular-arc edges (a revolve cap: a disc, an annulus, or an
/// angular sector of either), via the divergence theorem.
///
/// For a planar face every point satisfies `P·n = d` (the plane offset), so the
/// volume integral `(1/3)∮∮ P·n dA` reduces to `(1/3)·d·A`, where `A` is the
/// signed area in the plane oriented by the face normal. `A` is computed
/// EXACTLY by Green's theorem — `A = (1/2)∮(x dy − y dx)` — summing each edge's
/// chord term plus the exact circular-segment bulge for arc edges (including a
/// full closed circle, whose 2π sweep gives the disc area πρ²), so a circular
/// boundary is not chorded (the reason the tessellation path under-counts it).
///
/// Returns `Ok(None)` when an edge is neither a line nor a circular arc (e.g. a
/// general spline boundary), so the caller falls back to tessellation.
fn planar_cap_signed_volume(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Option<f64>, crate::OperationsError> {
    // Only claim a circular CAP (disc / annulus / sector): a face with no arc
    // edge is an ordinary polygon, which keeps this recogniser scoped to
    // genuine revolve caps and out of arbitrary planar-faced solids.
    Ok(planar_face_signed_volume(topo, face_id)?
        .and_then(|exact| (exact.arc_edges > 0).then_some(exact.volume)))
}

/// The closed-form integral of one planar face, with the terms a caller needs to
/// decide whether to trust it.
struct PlanarFaceExact {
    /// Divergence-theorem contribution `(1/3)·(p·n̂_out)·A`.
    volume: f64,
    /// Geometric area (outer wire less its holes).
    area: f64,
    /// How many boundary edges are circular arcs.
    arc_edges: usize,
    /// Total arc length over those edges — the only part of the boundary a
    /// tessellation has to approximate, so the budget for comparing this
    /// against the face's own mesh is proportional to it.
    arc_length: f64,
}

/// Exact signed volume contribution of any line-and-arc-bounded PLANAR face.
///
/// See [`planar_cap_signed_volume`] for the derivation; this is the unrestricted
/// form, which an all-polygon face also satisfies (with `arc_edges == 0`).
fn planar_face_signed_volume(
    topo: &Topology,
    face_id: FaceId,
) -> Result<Option<PlanarFaceExact>, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let FaceSurface::Plane { normal, d } = face.surface() else {
        return Ok(None);
    };
    let normal = *normal;
    // `d` is the offset along the STORED normal (`P·n = d`); the divergence
    // term needs it along the UNIT normal, so scale by 1/|n| for the callers
    // that hand in an unnormalised plane.
    let n_len = normal.length();
    if n_len <= 0.0 || !n_len.is_finite() {
        return Ok(None);
    }
    let d = *d / n_len;

    // Right-handed in-plane frame: ex × ey = normal, so a boundary wound CCW as
    // seen from +normal yields a positive signed area.
    let frame = match remus_math::frame::Frame3::from_normal(Point3::new(0.0, 0.0, 0.0), normal) {
        Ok(f) => f,
        Err(_) => return Ok(None),
    };
    let (ex, ey) = (frame.x, frame.y);

    // Hole areas subtract by MAGNITUDE, not by trusting the stored hole-wire
    // winding: boolean results can emit an inner rim wound the SAME way as the
    // outer (the #1045 hole-winding class), which would ADD the hole's disc.
    // A hole is inside the outer by definition, so |outer| − Σ|hole| is the
    // geometric area either way.
    let Some(outer) = planar_wire_signed_area2(topo, face.outer_wire(), ex, ey)? else {
        return Ok(None);
    };
    let mut arc_edges = outer.arc_edges;
    let mut arc_length = outer.arc_length;
    let mut area_mag2 = outer.area2.abs();
    for &iw in face.inner_wires() {
        let Some(hole) = planar_wire_signed_area2(topo, iw, ex, ey)? else {
            return Ok(None);
        };
        arc_edges += hole.arc_edges;
        arc_length += hole.arc_length;
        area_mag2 -= hole.area2.abs();
    }
    if area_mag2 < 0.0 {
        return Ok(None); // holes exceed the outer boundary — not a sane cap
    }

    // `area_mag2/2` is the geometric area. The divergence-theorem contribution
    // is (1/3)·(p·n̂_out)·|A|, where the outward normal is `+normal` for a
    // forward face and `−normal` for a reversed one, so `p·n̂_out = ±d` (the
    // plane offset along that outward normal). The sign therefore comes only
    // from the outward offset, not from the wire winding.
    let area = area_mag2 / 2.0;
    let d_out = if face.is_reversed() { -d } else { d };
    Ok(Some(PlanarFaceExact {
        volume: d_out * area / 3.0,
        area,
        arc_edges,
        arc_length,
    }))
}

/// One planar wire's Green's-theorem terms.
struct PlanarWireArea {
    /// Signed DOUBLED area, `∮(x dy − y dx)`.
    area2: f64,
    /// Circular-arc edge count.
    arc_edges: usize,
    /// Total arc length over those edges.
    arc_length: f64,
}

/// Green's-theorem signed doubled area (`∮(x dy − y dx)`) of one planar wire in
/// the `(ex, ey)` frame, plus its circular-arc edge count and arc length.
/// `Ok(None)` when an edge is neither a line nor a circular arc, so the caller
/// falls back to tessellation.
fn planar_wire_signed_area2(
    topo: &Topology,
    wire_id: remus_topology::wire::WireId,
    ex: Vec3,
    ey: Vec3,
) -> Result<Option<PlanarWireArea>, crate::OperationsError> {
    let to_2d = |p: Point3| {
        let v = Vec3::new(p.x(), p.y(), p.z());
        (v.dot(ex), v.dot(ey))
    };
    let tol_lin = remus_math::tolerance::Tolerance::default().linear;
    let mut area2: f64 = 0.0; // accumulates 2·A (Green's ∮(x dy − y dx))
    let mut arc_edges = 0_usize;
    let mut arc_length = 0.0_f64;
    {
        let wire = topo.wire(wire_id)?;
        for oe in wire.edges() {
            let edge = topo.edge(oe.edge())?;
            let (sv, ev) = if oe.is_forward() {
                (edge.start(), edge.end())
            } else {
                (edge.end(), edge.start())
            };
            let pa = topo.vertex(sv)?.point();
            let pb = topo.vertex(ev)?.point();
            let (ax, ay) = to_2d(pa);
            let (bx, by) = to_2d(pb);
            // Chord term: triangle (origin, a, b) doubled.
            area2 += ax * by - bx * ay;

            // A degenerate edge collapsed to a point that is NOT a closed circle
            // (e.g. the inner "arc" at the axis where a disc cap reaches r = 0, or
            // a zero-length line) contributes no chord and no bulge — skip it (and
            // do NOT let curve recognition on a zero-length arc decline the whole
            // cap). A CLOSED `Circle` rim also has coincident endpoints but bounds
            // a full disc, so it falls through to the arc handler below.
            let is_closed_circle =
                matches!(edge.curve(), remus_topology::edge::EdgeCurve::Circle(_))
                    && edge.start() == edge.end();
            if (pa - pb).length() < tol_lin && !is_closed_circle {
                continue;
            }

            // Circular-arc bulge correction (segment between the arc and its
            // chord). A `Line` has no bulge. A `Circle`/arc-`NurbsCurve` adds
            // sign·ρ²·(|α| − sin|α|), α the signed sweep about the arc centre.
            let arc = match edge.curve() {
                remus_topology::edge::EdgeCurve::Line => None,
                // The exact bulge correction below is circular-arc only.
                // These types have no circular bulge, so the whole exact path
                // declines rather than applying a wrong correction.
                remus_topology::edge::EdgeCurve::Ellipse(_)
                | remus_topology::edge::EdgeCurve::Hyperbola(_)
                | remus_topology::edge::EdgeCurve::Parabola(_) => return Ok(None),
                remus_topology::edge::EdgeCurve::Circle(c) => Some((c.center(), c.radius())),
                remus_topology::edge::EdgeCurve::NurbsCurve(nc) => {
                    let tol = remus_math::tolerance::Tolerance::default().linear * 100.0;
                    match remus_geometry::convert::recognize_curve(nc, tol) {
                        remus_geometry::convert::RecognizedCurve::Circle {
                            center, radius, ..
                        } => Some((center, radius)),
                        remus_geometry::convert::RecognizedCurve::Line { .. } => None,
                        _ => return Ok(None),
                    }
                }
            };

            if let Some((center, radius)) = arc {
                arc_edges += 1;
                // The bulge correction (circular segment between the arc and its
                // chord) is `sign·ρ²·(|α| − sin|α|)`. Compute the sweep in the
                // curve's NATURAL direction (start→mid→end), then flip its sign for
                // a reversed `OrientedEdge`, so the bulge is consistent with the
                // chord term above (which uses the oriented endpoints). Without the
                // flip, a reversed inner rim of an annulus ADDS its segment instead
                // of subtracting it (inflated area).
                let nat_alpha = if is_closed_circle {
                    // A full circle sweeps 2π in its natural (CCW) direction → the
                    // bulge gives the disc area πρ². (The seam endpoint's antipode is
                    // NOT the domain midpoint, so the open-arc disambiguation below
                    // does not apply.)
                    std::f64::consts::TAU
                } else {
                    // Sample the arc at its DOMAIN midpoint (the domain need not be
                    // [0,1]) to disambiguate the signed sweep > π for a major arc.
                    let nat_start = topo.vertex(edge.start())?.point();
                    let nat_end = topo.vertex(edge.end())?.point();
                    let (t0, t1) = crate::authoritative_edge_domain(
                        edge,
                        "planar curved-boundary volume integration",
                    )?;
                    let mid_pt = edge.curve().evaluate_with_endpoints(
                        f64::midpoint(t0, t1),
                        nat_start,
                        nat_end,
                    );
                    let (cx, cy) = to_2d(center);
                    let (sx, sy) = to_2d(nat_start);
                    let (ex, ey) = to_2d(nat_end);
                    let (mx, my) = to_2d(mid_pt);
                    let va = (sx - cx, sy - cy);
                    let vm = (mx - cx, my - cy);
                    let vb = (ex - cx, ey - cy);
                    // Signed sweep start→mid→end (each leg in (−π, π]).
                    let ang = |u: (f64, f64), w: (f64, f64)| -> f64 {
                        (u.0 * w.1 - u.1 * w.0).atan2(u.0 * w.0 + u.1 * w.1)
                    };
                    ang(va, vm) + ang(vm, vb)
                };
                let alpha = if oe.is_forward() {
                    nat_alpha
                } else {
                    -nat_alpha
                };
                area2 += alpha.signum() * radius * radius * (alpha.abs() - alpha.abs().sin());
                arc_length += radius * alpha.abs();
            }
        }
    }
    Ok(Some(PlanarWireArea {
        area2,
        arc_edges,
        arc_length,
    }))
}

/// Exact signed volume contribution of a conical face via the divergence
/// theorem: `V = (1/3) integral P.n dA`.
///
/// For a cone parameterised as
///   `P(u,v) = apex + v*(cos_a*(cos u * ex + sin u * ey) + sin_a * axis)`
/// the outward normal is `n = sin_a*(cos u * ex + sin u * ey) - cos_a * axis`,
/// and `dA = v * cos_a * du dv`.
///
/// The integrand `P.n * dA` simplifies to closed form over `[u1,u2] x [v1,v2]`.
fn analytic_cone_signed_volume(
    topo: &Topology,
    face_id: FaceId,
) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let cone = match face.surface() {
        FaceSurface::Cone(c) => c,
        _ => {
            return Err(crate::OperationsError::InvalidInput {
                reason: "analytic_cone_signed_volume requires a cone face".into(),
            });
        }
    };

    let wire = topo.wire(face.outer_wire())?;
    let mut u_vals = Vec::new();
    let mut v_vals = Vec::new();
    for oe in wire.edges() {
        if let Ok(edge) = topo.edge(oe.edge()) {
            for &vid in &[edge.start(), edge.end()] {
                if let Ok(vtx) = topo.vertex(vid) {
                    let (u, v) = cone.project_point(vtx.point());
                    // The apex (v ≈ 0) lies on the axis where u is undefined;
                    // its arbitrary projected u corrupts the angular-range gap
                    // detection (a per-segment band touching the apex would read
                    // a 2× span). Keep its v for the v-range, but omit its u.
                    if v.abs() > 1e-9 {
                        u_vals.push(u);
                    }
                    v_vals.push(v);
                }
            }
            if !edge.is_closed()
                && let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve()
                && let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end()))
            {
                let ts = circle.project(sv.point());
                let te = circle.project(ev.point());
                let fwd = (te - ts).rem_euclid(std::f64::consts::TAU);
                let mid_t = if fwd <= std::f64::consts::PI {
                    ts + fwd * 0.5
                } else {
                    ts - (std::f64::consts::TAU - fwd) * 0.5
                };
                let mid = circle.evaluate(mid_t);
                let (u, _) = cone.project_point(mid);
                u_vals.push(u);
            }
            // Sample NURBS revolution-band arcs too (see the cylinder case, #968).
            // Skip an arc that degenerates to the apex (v ≈ 0), where u is
            // undefined.
            if !edge.is_closed()
                && let remus_topology::edge::EdgeCurve::NurbsCurve(nc) = edge.curve()
            {
                let (t0, t1) = nc.domain();
                let (u, v) = cone.project_point(nc.evaluate(f64::midpoint(t0, t1)));
                if v.abs() > 1e-9 {
                    u_vals.push(u);
                }
            }
        }
    }

    let v_min = v_vals.iter().copied().fold(f64::INFINITY, f64::min);
    let v_max = v_vals.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if (v_max - v_min).abs() < 1e-15 {
        return Ok(0.0);
    }

    let u_range = compute_angular_range(&mut u_vals);

    let (sin_a, cos_a) = cone.half_angle().sin_cos();
    let x_axis = cone.x_axis();
    let y_axis = cone.y_axis();
    let axis = cone.axis();
    let apex = cone.apex();
    let a_vec = Vec3::new(apex.x(), apex.y(), apex.z());

    // Compute the divergence-theorem integral analytically.
    //
    // P(u,v) = apex + v*(cos_a*radial(u) + sin_a*axis)
    // n(u) = sin_a*radial(u) - cos_a*axis   (outward normal direction)
    // dA = v * cos_a * du * dv
    //
    // P.n = apex.(sin_a*radial - cos_a*axis)
    //     + v*(cos_a*sin_a*(radial.radial) + sin_a^2*(axis.radial) - cos_a^2*(radial.axis) - cos_a*sin_a*(axis.axis))
    //     = apex.(sin_a*radial - cos_a*axis) + v*(cos_a*sin_a - cos_a*sin_a)
    //     = apex.(sin_a*radial(u) - cos_a*axis)
    //
    // The v-dependent terms cancel: cos_a*sin_a - cos_a*sin_a = 0, so P.n is v-independent.
    //
    // Full integrand = (1/3) * P.n * dA = (1/3) * [a_vec.(sin_a*radial(u) - cos_a*axis)] * v*cos_a * du * dv
    //
    // integral = (cos_a/3) * [(v^2/2)|v1..v2] * integral[sin_a*(ax*cos_u + ay*sin_u) - cos_a*az] du
    // where ax = a_vec.x_axis, ay = a_vec.y_axis, az = a_vec.axis
    let ax = a_vec.dot(x_axis);
    let ay = a_vec.dot(y_axis);
    let az = a_vec.dot(axis);

    let v2_half = (v_max * v_max - v_min * v_min) / 2.0;

    let (u1, u2) = u_range;
    let (sin1, cos1) = u1.sin_cos();
    let (sin2, cos2) = u2.sin_cos();

    let u_integral = sin_a * (ax * (sin2 - sin1) + ay * (-cos2 + cos1)) - cos_a * az * (u2 - u1);

    let vol = (cos_a / 3.0) * v2_half * u_integral;

    Ok(if face.is_reversed() { -vol } else { vol })
}

/// Exact signed volume contribution of a spherical face via the divergence
/// theorem: `V = (1/3) integral P.n dA`.
///
/// For a sphere parameterised as
///   `P(u,v) = C + r*(cos_v*cos_u*ex + cos_v*sin_u*ey + sin_v*ez)`
/// the outward normal equals the unit radial direction, and `dA = r^2*cos_v * du dv`.
///
/// `P.n = C.n + r`, so the integrand is `(1/3)*(C.n + r)*r^2*cos_v du dv`.
#[allow(clippy::too_many_lines)]
fn analytic_sphere_signed_volume(
    topo: &Topology,
    face_id: FaceId,
) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let sph = match face.surface() {
        FaceSurface::Sphere(s) => s,
        _ => {
            return Err(crate::OperationsError::InvalidInput {
                reason: "analytic_sphere_signed_volume requires a sphere face".into(),
            });
        }
    };

    let wire = topo.wire(face.outer_wire())?;
    let mut u_vals = Vec::new();
    let mut v_vals = Vec::new();
    for oe in wire.edges() {
        if let Ok(edge) = topo.edge(oe.edge()) {
            for &vid in &[edge.start(), edge.end()] {
                if let Ok(vtx) = topo.vertex(vid) {
                    let (u, v) = sph.project_point(vtx.point());
                    u_vals.push(u);
                    v_vals.push(v);
                }
            }
            if !edge.is_closed()
                && let remus_topology::edge::EdgeCurve::Circle(circle) = edge.curve()
                && let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end()))
            {
                let ts = circle.project(sv.point());
                let te = circle.project(ev.point());
                let fwd = (te - ts).rem_euclid(std::f64::consts::TAU);
                let mid_t = if fwd <= std::f64::consts::PI {
                    ts + fwd * 0.5
                } else {
                    ts - (std::f64::consts::TAU - fwd) * 0.5
                };
                let mid = circle.evaluate(mid_t);
                let (u, _) = sph.project_point(mid);
                u_vals.push(u);
            }
        }
    }

    let mut v_min = v_vals.iter().copied().fold(f64::INFINITY, f64::min);
    let mut v_max = v_vals.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    // For sphere caps (single circle boundary at one latitude), the boundary
    // vertices all share approximately the same v, so v_max ~ v_min.
    // Determine which pole the face covers by checking a face interior point.
    if (v_max - v_min).abs() < 0.01 {
        let v_boundary = f64::midpoint(v_min, v_max);
        let positions = crate::boolean::face_polygon(topo, face_id)?;
        if positions.is_empty() {
            return Ok(0.0);
        }
        let n = positions.len() as f64;
        let avg = Point3::new(
            positions.iter().map(|p| p.x()).sum::<f64>() / n,
            positions.iter().map(|p| p.y()).sum::<f64>() / n,
            positions.iter().map(|p| p.z()).sum::<f64>() / n,
        );
        let (_, v_interior) = sph.project_point(avg);
        if v_interior > v_boundary {
            v_min = v_boundary;
            v_max = std::f64::consts::FRAC_PI_2;
        } else {
            v_min = -std::f64::consts::FRAC_PI_2;
            v_max = v_boundary;
        }
    }

    let u_range = compute_angular_range(&mut u_vals);

    let r = sph.radius();
    let x_axis = sph.x_axis();
    let y_axis = sph.y_axis();
    let z_axis = sph.z_axis();
    let c = sph.center();
    let c_vec = Vec3::new(c.x(), c.y(), c.z());

    // P.n = C.(cos_v*cos_u*ex + cos_v*sin_u*ey + sin_v*ez) + r
    // dA = r^2 * cos_v * du * dv
    //
    // Integrand = (1/3) * (cx*cos_v*cos_u + cy*cos_v*sin_u + cz*sin_v + r) * r^2 * cos_v
    // where cx = C.ex, cy = C.ey, cz = C.ez
    let cx = c_vec.dot(x_axis);
    let cy = c_vec.dot(y_axis);
    let cz = c_vec.dot(z_axis);

    let (u1, u2) = u_range;
    let (sin_u1, cos_u1) = u1.sin_cos();
    let (sin_u2, cos_u2) = u2.sin_cos();
    let du = u2 - u1;

    // integral cos_v*cos_v dv = v/2 + sin(2v)/4
    let vv_integral = |v: f64| -> f64 { v / 2.0 + (2.0 * v).sin() / 4.0 };
    let cos2_v = vv_integral(v_max) - vv_integral(v_min);

    // integral cos_v dv = sin_v
    let cos_v_int = v_max.sin() - v_min.sin();

    // integral sin_v*cos_v dv = sin^2(v)/2
    let sincos_v = (v_max.sin().powi(2) - v_min.sin().powi(2)) / 2.0;

    // Full integral:
    // cx * cos2_v * (sin_u2 - sin_u1)
    // + cy * cos2_v * (-cos_u2 + cos_u1)
    // + cz * sincos_v * du
    // + r * cos_v_int * du
    let vol = (r * r / 3.0)
        * (cx * cos2_v * (sin_u2 - sin_u1)
            + cy * cos2_v * (-cos_u2 + cos_u1)
            + cz * sincos_v * du
            + r * cos_v_int * du);

    Ok(if face.is_reversed() { -vol } else { vol })
}

/// Exact signed volume contribution of a toroidal face via the divergence
/// theorem: `V = (1/3) integral P.n dA`.
///
/// For a torus parameterised as
///   `P(u,v) = C + (R + r*cos_v)*(cos_u*ex + sin_u*ey) + r*sin_v*ez`
/// the outward normal `n = cos_v*(cos_u*ex + sin_u*ey) + sin_v*ez`,
/// and `dA = r*(R + r*cos_v) du dv`.
#[allow(clippy::too_many_lines)]
fn analytic_torus_signed_volume(
    topo: &Topology,
    face_id: FaceId,
) -> Result<f64, crate::OperationsError> {
    let face = topo.face(face_id)?;
    let tor = match face.surface() {
        FaceSurface::Torus(t) => t,
        _ => {
            return Err(crate::OperationsError::InvalidInput {
                reason: "analytic_torus_signed_volume requires a torus face".into(),
            });
        }
    };

    let wire = topo.wire(face.outer_wire())?;
    let mut u_vals = Vec::new();
    let mut v_vals = Vec::new();
    for oe in wire.edges() {
        if let Ok(edge) = topo.edge(oe.edge()) {
            for &vid in &[edge.start(), edge.end()] {
                if let Ok(vtx) = topo.vertex(vid) {
                    let (u, v) = tor.project_point(vtx.point());
                    u_vals.push(u);
                    v_vals.push(v);
                }
            }
            // Sample each arc edge's midpoint to widen the angular ranges past
            // the two endpoint angles — without this a partial (sub-2π) band has
            // only its corner angles, `compute_angular_range` falls back to the
            // full 2π, and the band over-counts (gh #968). Both the major (u) and
            // minor (v) ranges are captured, since an arc may run in either
            // direction. A revolution-band boundary is a rational `NurbsCurve`
            // (the swept circle) or a `Circle` (the profile arc copy).
            if !edge.is_closed()
                && let (Ok(sv), Ok(ev)) = (topo.vertex(edge.start()), topo.vertex(edge.end()))
            {
                let mid = match edge.curve() {
                    remus_topology::edge::EdgeCurve::Circle(circle) => {
                        let ts = circle.project(sv.point());
                        let te = circle.project(ev.point());
                        let fwd = (te - ts).rem_euclid(std::f64::consts::TAU);
                        let mid_t = if fwd <= std::f64::consts::PI {
                            ts + fwd * 0.5
                        } else {
                            ts - (std::f64::consts::TAU - fwd) * 0.5
                        };
                        Some(circle.evaluate(mid_t))
                    }
                    remus_topology::edge::EdgeCurve::NurbsCurve(nc) => {
                        let (t0, t1) = nc.domain();
                        Some(nc.evaluate(f64::midpoint(t0, t1)))
                    }
                    _ => None,
                };
                if let Some(mid) = mid {
                    let (u, v) = tor.project_point(mid);
                    u_vals.push(u);
                    v_vals.push(v);
                }
            }
        }
    }

    let v_min = v_vals.iter().copied().fold(f64::INFINITY, f64::min);
    let v_max = v_vals.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if (v_max - v_min).abs() < 1e-15 {
        return Ok(0.0);
    }

    // The minor (v) angle is periodic, so the raw [min, max] span is wrong for
    // a band whose samples straddle the v = 0/2π seam: a rim at the outer
    // equator can project to v = 2π−ε from float noise, turning a [0, π] band
    // into a phantom [π/2, 2π] complement (a −27% "exact" volume). With enough
    // samples (≥3 distinct), pick the range gap-wise like `u` does — the band
    // is the complement of the largest angular gap. A band given by just two
    // distinct minor angles spanning MORE than π stays ambiguous either way
    // (e.g. a fillet's concave quarter rim, whose endpoints are 270° apart but
    // whose band is the 90° short side), so decline and fall back to
    // tessellation rather than integrate the wrong portion.
    let (v_min, v_max) = {
        let mut sorted = v_vals.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        sorted.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        if sorted.len() < 3 {
            if (v_max - v_min) > std::f64::consts::PI + 1e-9 {
                return Err(crate::OperationsError::InvalidInput {
                    reason:
                        "torus band minor range is ambiguous (seam-straddling, no interior sample)"
                            .into(),
                });
            }
            (v_min, v_max)
        } else {
            compute_angular_range(&mut v_vals)
        }
    };

    let u_range = compute_angular_range(&mut u_vals);

    let big_r = tor.major_radius();
    let small_r = tor.minor_radius();
    let x_axis = tor.x_axis();
    let y_axis = tor.y_axis();
    let z_axis = tor.z_axis();
    let c = tor.center();
    let c_vec = Vec3::new(c.x(), c.y(), c.z());

    // P.n = [C + (R+r*cos_v)*radial_u + r*sin_v*ez] . [cos_v*radial_u + sin_v*ez]
    //     = C.(cos_v*radial_u + sin_v*ez) + (R+r*cos_v)*cos_v + r*sin^2_v
    //     = cos_v*(cx*cos_u + cy*sin_u) + sin_v*cz + (R+r*cos_v)*cos_v + r*sin^2_v
    //     = cos_v*(cx*cos_u + cy*sin_u) + sin_v*cz + R*cos_v + r*cos^2_v + r*sin^2_v
    //     = cos_v*(cx*cos_u + cy*sin_u) + sin_v*cz + R*cos_v + r
    //
    // dA = r*(R + r*cos_v) du dv
    //
    // Full integrand = (1/3) * P.n * dA
    let cx = c_vec.dot(x_axis);
    let cy = c_vec.dot(y_axis);
    let cz = c_vec.dot(z_axis);

    let (u1, u2) = u_range;
    let (sin_u1, cos_u1) = u1.sin_cos();
    let (sin_u2, cos_u2) = u2.sin_cos();
    let du = u2 - u1;

    // We need to integrate over v:
    // integral [cos_v*(cx*cos_u + cy*sin_u) + cz*sin_v + R*cos_v + r] * r*(R + r*cos_v) dv
    //
    // Expand the product with (R + r*cos_v):
    // = r * integral [cos_v*(cx*cos_u+cy*sin_u)*(R+r*cos_v)
    //        + cz*sin_v*(R+r*cos_v)
    //        + R*cos_v*(R+r*cos_v)
    //        + r*(R+r*cos_v)] dv
    //
    // This is a sum of standard trigonometric integrals.
    // Let S = cx*cos_u + cy*sin_u (depends on u, integrated separately)

    // Standard integrals over [v1,v2]:
    let sv1 = v_min.sin();
    let sv2 = v_max.sin();
    let cv1 = v_min.cos();
    let cv2 = v_max.cos();
    let dv = v_max - v_min;

    // integral cos_v dv = sin_v
    let i_cos = sv2 - sv1;
    // integral cos^2_v dv = v/2 + sin(2v)/4
    let i_cos2 =
        (v_max / 2.0 + (2.0 * v_max).sin() / 4.0) - (v_min / 2.0 + (2.0 * v_min).sin() / 4.0);
    // integral sin_v dv = -cos_v
    let i_sin = -cv2 + cv1;
    // integral sin_v*cos_v dv = sin^2(v)/2
    let i_sincos = (sv2 * sv2 - sv1 * sv1) / 2.0;
    // Group terms by u-dependence:
    // Terms with S (= cx*cos_u + cy*sin_u):
    //   r*[R*i_cos + r*i_cos2] * integral S du
    let s_u_integral = cx * (sin_u2 - sin_u1) + cy * (-cos_u2 + cos_u1);
    let s_coeff = small_r * (big_r * i_cos + small_r * i_cos2);

    // Terms with cz*sin_v:
    //   r*cz*[R*i_sin + r*i_sincos] * du
    let cz_coeff = small_r * cz * (big_r * i_sin + small_r * i_sincos);

    // Terms with R*cos_v:
    //   r*R*[R*i_cos + r*i_cos2] * du
    let rcos_coeff = small_r * big_r * (big_r * i_cos + small_r * i_cos2);

    // Terms with r (constant in v):
    //   r*r*[R*dv + r*i_cos] * du
    let const_coeff = small_r * small_r * (big_r * dv + small_r * i_cos);

    let vol = (1.0 / 3.0) * (s_coeff * s_u_integral + (cz_coeff + rcos_coeff + const_coeff) * du);

    Ok(if face.is_reversed() { -vol } else { vol })
}

/// Exact per-face divergence sum for a solid whose WHOLE boundary integrates in
/// closed form — planes by Green's theorem, quadrics by their analytic
/// integrators — so the result carries no tessellation error at all.
///
/// This exists because a per-face divergence sum is only valid when every face
/// contributes an integral over the SAME boundary. Integrating the quadrics
/// exactly while chording the planes breaks that: the planes' chord polygons no
/// longer meet the quadrics' true arcs, and the sliver between them is charged
/// to nobody. The loss is one-sided, scales with each plane's offset from the
/// origin, and depends on how the modeller happened to split the body — so two
/// equivalent decompositions of one solid measure differently. Integrating the
/// planes exactly too removes the mismatch and the decomposition dependence.
///
/// Returns `None` — defer to the tessellated summation — unless
///   * every face is a plane or a quadric (no NURBS), and
///   * no quadric face carries an inner wire (the quadric integrators below are
///     hole-unaware, so a bored wall must keep whatever the caller did before),
///     and
///   * every planar face is bounded only by lines and circular arcs, so
///     [`planar_face_signed_volume`] applies, and
///   * every planar face's closed form AGREES with the area of its own mesh to
///     within the chord budget — see [`planar_face_area_is_consistent`].
fn exact_analytic_face_volume(topo: &Topology, solid: SolidId, deflection: f64) -> Option<f64> {
    // Cavity faces are boundary too, and their stored reversal makes each one's
    // signed contribution subtract. Enumerating only the outer shell would
    // integrate the un-hollowed body.
    let faces = remus_topology::explorer::solid_faces(topo, solid).ok()?;
    if faces.is_empty() {
        return None;
    }

    let mut total = 0.0;
    for fid in faces {
        let face = topo.face(fid).ok()?;
        let holed = !face.inner_wires().is_empty();
        total += match face.surface() {
            FaceSurface::Nurbs(_) => return None,
            FaceSurface::Plane { .. } => {
                let exact = planar_face_signed_volume(topo, fid).ok()??;
                if !planar_face_area_is_consistent(topo, fid, &exact, deflection) {
                    return None;
                }
                exact.volume
            }
            FaceSurface::Cylinder(_) if !holed => {
                analytic_cylinder_signed_volume(topo, fid).ok()?
            }
            FaceSurface::Cone(_) if !holed => analytic_cone_signed_volume(topo, fid).ok()?,
            FaceSurface::Sphere(_) if !holed => analytic_sphere_signed_volume(topo, fid).ok()?,
            FaceSurface::Torus(_) if !holed => analytic_torus_signed_volume(topo, fid).ok()?,
            _ => return None, // holed quadric wall: no hole-aware integrator here
        };
    }
    Some(total.abs())
}

/// Whether a planar face's closed-form area is a REFINEMENT of the area its own
/// tessellation reports, rather than a different answer.
///
/// Green's theorem integrates whatever loops the face stores; it cannot tell a
/// hole from a mis-traced loop. A shelled cup whose rim boundary sorted into
/// loops that jump across the solid, or a boolean result whose merged wire self-
/// crosses, would still yield a confident number. Its own mesh is built from the
/// same wires by an independent route (sample, project, constrained-triangulate,
/// drop the exterior), so the two agreeing means the wires really do bound the
/// region claimed.
///
/// The budget is what CHORDING can account for: replacing an arc of length `L`
/// by an inscribed polyline at deflection `δ` moves the enclosed area by at most
/// about `(2/3)·L·δ`, and nothing else should differ. `SLACK` covers samplers
/// that land coarser than the nominal deflection (an angular criterion, a
/// segment-count floor); it is a factor, not a fixed pad, so the check still
/// tightens to nothing as `δ → 0`.
fn planar_face_area_is_consistent(
    topo: &Topology,
    face_id: FaceId,
    exact: &PlanarFaceExact,
    deflection: f64,
) -> bool {
    /// Multiple of the ideal chord budget allowed before the closed form is
    /// treated as describing a different region than the mesh does.
    const SLACK: f64 = 16.0;

    // The probe needs a mesh whose error is BOUNDED, not a fine one: the budget
    // scales with the deflection the mesh was built at, so a coarse probe is
    // just as conclusive and costs a fraction as much. Floor it at a thousandth
    // of the face's own arc length, so a caller asking for 1e-6 does not pay for
    // a million-segment rim it never sees. An arc-free face tessellates to the
    // same polygon at every deflection, so it needs no floor — and its budget
    // then collapses to round-off, which is right: a polygon's mesh area IS its
    // Green's-theorem area.
    let probe = if exact.arc_length > 0.0 {
        deflection.max(exact.arc_length * 1e-3)
    } else {
        deflection
    };
    let Ok(mesh) = tessellate::tessellate(topo, face_id, probe) else {
        return false;
    };
    let mut mesh_area = 0.0;
    for tri in mesh.indices.chunks_exact(3) {
        let p = |k: usize| {
            let v = mesh.positions[tri[k] as usize];
            Vec3::new(v.x(), v.y(), v.z())
        };
        mesh_area += (p(1) - p(0)).cross(p(2) - p(0)).length() / 2.0;
    }
    let budget =
        SLACK * (2.0 / 3.0) * exact.arc_length * probe.max(0.0) + 1e-9 * exact.area.abs().max(1.0);
    (exact.area - mesh_area).abs() <= budget
}

/// Compute volume by tessellating each face and summing signed tetrahedra
/// WITHOUT winding correction. Relies on `tessellate()` already handling
/// face reversal (via `is_reversed` flag) to produce correctly oriented
/// triangles. For analytic surface faces (cylinder, cone, sphere, torus),
/// uses exact analytical integration via the divergence theorem instead
/// of tessellation.
///
/// # Accuracy
///
/// When the whole boundary is analytic (planes + quadrics, every planar face
/// bounded by lines and circular arcs) the result is EXACT — floating-point
/// round-off only, no dependence on `deflection` and none on how the body was
/// decomposed. Otherwise the NURBS and spline-bounded faces fall back to their
/// own tessellation, and the sum inherits that chord error: it is biased low by
/// roughly `(2/3)·Σ(Lᵢ·δ·|dᵢ|)/3` over those faces (arc length `Lᵢ`, plane
/// offset `dᵢ`, deflection `δ`), shrinking linearly with `deflection`.
pub fn volume_from_direct_face_tessellation(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<f64, crate::OperationsError> {
    // Exact whenever the whole boundary integrates in closed form.
    if let Some(v) = exact_analytic_face_volume(topo, solid, deflection) {
        return Ok(v);
    }

    // Outer shell plus every cavity shell: a reversed cavity face's signed
    // contribution subtracts the void.
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;

    let mut total: f64 = 0.0;
    for fid in faces {
        let face = topo.face(fid)?;

        // Use exact analytical volume for analytic surface faces.
        match face.surface() {
            FaceSurface::Cylinder(_) => {
                let v = analytic_cylinder_signed_volume(topo, fid)? * 6.0;
                if std::env::var("BK_VOL_TRACE").is_ok() {
                    log::debug!("VOL_TRACE direct cyl face {:?} -> {}", fid, v / 6.0);
                }
                total += v;
                continue;
            }
            FaceSurface::Cone(_) => {
                total += analytic_cone_signed_volume(topo, fid)? * 6.0;
                continue;
            }
            FaceSurface::Sphere(_) => {
                total += analytic_sphere_signed_volume(topo, fid)? * 6.0;
                continue;
            }
            FaceSurface::Torus(_) => {
                total += analytic_torus_signed_volume(topo, fid)? * 6.0;
                continue;
            }
            FaceSurface::Plane { .. } | FaceSurface::Nurbs(_) => {}
        }

        let mesh = tessellate::tessellate(topo, fid, deflection)?;
        let idx = &mesh.indices;
        let pos = &mesh.positions;
        let tri_count = idx.len() / 3;

        let mut face_total = 0.0;
        for t in 0..tri_count {
            let v0 = pos[idx[t * 3] as usize];
            let v1 = pos[idx[t * 3 + 1] as usize];
            let v2 = pos[idx[t * 3 + 2] as usize];

            let a = Vec3::new(v0.x(), v0.y(), v0.z());
            let b = Vec3::new(v1.x(), v1.y(), v1.z());
            let c = Vec3::new(v2.x(), v2.y(), v2.z());

            face_total += a.dot(b.cross(c));
        }

        total += face_total;
    }

    Ok((total / 6.0).abs())
}

/// Compute the volume of a solid directly from its face vertex
/// positions, bypassing tessellation. Only valid for solids composed
/// entirely of planar triangular faces (e.g. mesh imports).
///
/// Returns an error if the solid contains non-planar or
/// non-triangular faces.
///
/// # Errors
///
/// Returns [`crate::OperationsError`] if topology lookups fail or if the
/// solid contains non-planar/non-triangular faces.
pub fn solid_volume_from_faces(
    topo: &Topology,
    solid: SolidId,
    _deflection: f64,
) -> Result<f64, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;
    use remus_topology::face::FaceSurface;

    // Outer shell plus every cavity shell.
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;

    let mut total = 0.0;
    let mut all_planar_triangles = true;

    for fid in faces {
        let face = topo.face(fid)?;

        // Only use the fast path for planar faces with exactly 3 line edges.
        if !matches!(face.surface(), FaceSurface::Plane { .. }) {
            all_planar_triangles = false;
            break;
        }

        let wire = topo.wire(face.outer_wire())?;
        let edges = wire.edges();
        if edges.len() != 3 {
            all_planar_triangles = false;
            break;
        }

        let mut pts = Vec::with_capacity(3);
        for oe in edges {
            let edge = topo.edge(oe.edge())?;
            if !matches!(edge.curve(), EdgeCurve::Line) {
                all_planar_triangles = false;
                break;
            }
            let vid = if oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            pts.push(topo.vertex(vid)?.point());
        }
        if !all_planar_triangles {
            break;
        }

        let a = Vec3::new(pts[0].x(), pts[0].y(), pts[0].z());
        let b = Vec3::new(pts[1].x(), pts[1].y(), pts[1].z());
        let c = Vec3::new(pts[2].x(), pts[2].y(), pts[2].z());

        // Enumerating the cavity shells is not enough on its own: a cavity's
        // faces are stored REVERSED, and the wire winding alone does not say
        // so. Without this the void's tetrahedra add instead of subtract and a
        // hollow triangulated body reads as the outer body PLUS the void.
        let orientation = if face.is_reversed() { -1.0 } else { 1.0 };
        total += orientation * a.dot(b.cross(c));
    }

    if all_planar_triangles {
        Ok((total / 6.0).abs())
    } else {
        Err(crate::OperationsError::InvalidInput {
            reason: "solid contains non-planar or non-triangular faces".to_string(),
        })
    }
}

/// Compute the full mass properties of a solid, assuming uniform density.
///
/// Returns volume, center of mass, and the inertia tensor about the center
/// of mass, integrated on the exact face geometry (Gauss quadrature over
/// analytic and NURBS surfaces — no tessellation, so no deflection
/// parameter). Cavity shells contribute with reversed orientation.
///
/// # Errors
///
/// Returns an error if the solid handle is invalid, integration fails, or
/// the solid has zero volume.
pub fn mass_properties(
    topo: &Topology,
    solid: SolidId,
) -> Result<remus_check::properties::GProps, crate::OperationsError> {
    // The cubic second-moment integrands carry one polynomial degree more
    // than the volume/CoM terms; order 8 keeps curved-surface inertia at
    // ~1e-9 relative where the default order 5 leaves ~3e-8.
    let options = remus_check::properties::PropertiesOptions {
        gauss_order: 8,
        ..Default::default()
    };
    Ok(remus_check::properties::solid_properties(
        topo, solid, &options,
    )?)
}

/// Compute the center of mass of a solid, assuming uniform density.
///
/// Uses the same signed-tetrahedra decomposition as `solid_volume`,
/// accumulating the centroid contribution of each tetrahedron:
/// `centroid += signed_vol * (a + b + c)`, then divides by
/// `4 * total_volume`.
///
/// Cavity shells count. Their faces are stored reversed, so `tessellate`
/// flips their winding and their signed tetrahedra subtract both the void's
/// volume and its first moment — which is what makes the result the composite
/// centroid `(V_out*c_out - V_void*c_void) / (V_out - V_void)` rather than the
/// outer body's own.
///
/// # Errors
///
/// Returns an error if the solid has zero volume or tessellation fails.
pub fn solid_center_of_mass(
    topo: &Topology,
    solid: SolidId,
    deflection: f64,
) -> Result<Point3, crate::OperationsError> {
    // Fast path: for all-planar-triangle solids, compute directly
    // from face geometry (avoids re-tessellation winding issues).
    if let Ok(com) = center_of_mass_from_faces(topo, solid) {
        return Ok(com);
    }

    // tessellate() already handles face reversal (flips winding),
    // so signed tetrahedra sum is correct without winding heuristics.
    // Outer shell plus every cavity shell.
    let faces = remus_topology::explorer::solid_faces(topo, solid)?;

    let mut total_vol: f64 = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;

    for fid in faces {
        let mesh = tessellate::tessellate(topo, fid, deflection)?;
        let idx = &mesh.indices;
        let pos = &mesh.positions;
        let tri_count = idx.len() / 3;

        for t in 0..tri_count {
            let v0 = pos[idx[t * 3] as usize];
            let v1 = pos[idx[t * 3 + 1] as usize];
            let v2 = pos[idx[t * 3 + 2] as usize];

            let a = Vec3::new(v0.x(), v0.y(), v0.z());
            let b = Vec3::new(v1.x(), v1.y(), v1.z());
            let c = Vec3::new(v2.x(), v2.y(), v2.z());

            let signed_vol = a.dot(b.cross(c));
            total_vol += signed_vol;
            cx += signed_vol * (v0.x() + v1.x() + v2.x());
            cy += signed_vol * (v0.y() + v1.y() + v2.y());
            cz += signed_vol * (v0.z() + v1.z() + v2.z());
        }
    }

    if total_vol.abs() < 1e-15 {
        // Volume too small to compute weighted CoM -- fall back to vertex centroid.
        let vertex_points = collect_solid_vertex_points(topo, solid)?;
        let n = vertex_points.len().max(1) as f64;
        let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
        for p in &vertex_points {
            sx += p.x();
            sy += p.y();
            sz += p.z();
        }
        return Ok(Point3::new(sx / n, sy / n, sz / n));
    }

    let denom = 4.0 * total_vol;
    Ok(Point3::new(cx / denom, cy / denom, cz / denom))
}

/// Compute center of mass directly from face vertex positions for
/// solids composed entirely of planar triangular faces.
///
/// Enumerates outer shell plus every cavity shell, and applies each face's
/// stored reversal to the tetrahedron sign, so a void subtracts its volume AND
/// its first moment.
fn center_of_mass_from_faces(
    topo: &Topology,
    solid: SolidId,
) -> Result<Point3, crate::OperationsError> {
    use remus_topology::edge::EdgeCurve;
    use remus_topology::face::FaceSurface;

    let faces = remus_topology::explorer::solid_faces(topo, solid)?;

    let mut total_vol = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    let mut cz = 0.0;

    for fid in faces {
        let face = topo.face(fid)?;
        if !matches!(face.surface(), FaceSurface::Plane { .. }) {
            return Err(crate::OperationsError::InvalidInput {
                reason: "non-planar face".to_string(),
            });
        }
        let wire = topo.wire(face.outer_wire())?;
        let edges = wire.edges();
        if edges.len() != 3 {
            return Err(crate::OperationsError::InvalidInput {
                reason: "non-triangular face".to_string(),
            });
        }

        let mut pts = Vec::with_capacity(3);
        for oe in edges {
            let edge = topo.edge(oe.edge())?;
            if !matches!(edge.curve(), EdgeCurve::Line) {
                return Err(crate::OperationsError::InvalidInput {
                    reason: "non-line edge".to_string(),
                });
            }
            let vid = if oe.is_forward() {
                edge.start()
            } else {
                edge.end()
            };
            pts.push(topo.vertex(vid)?.point());
        }

        let a = Vec3::new(pts[0].x(), pts[0].y(), pts[0].z());
        let b = Vec3::new(pts[1].x(), pts[1].y(), pts[1].z());
        let c = Vec3::new(pts[2].x(), pts[2].y(), pts[2].z());

        // A face carried reversed points its wire the other way round, so its
        // tetrahedra count with the opposite sign. Cavity shells are stored
        // exactly that way.
        let orientation = if face.is_reversed() { -1.0 } else { 1.0 };
        let signed_vol = orientation * a.dot(b.cross(c));
        total_vol += signed_vol;
        cx += signed_vol * (pts[0].x() + pts[1].x() + pts[2].x());
        cy += signed_vol * (pts[0].y() + pts[1].y() + pts[2].y());
        cz += signed_vol * (pts[0].z() + pts[1].z() + pts[2].z());
    }

    if total_vol.abs() < 1e-15 {
        return Err(crate::OperationsError::InvalidInput {
            reason: "solid has zero volume, center of mass is undefined".into(),
        });
    }

    let denom = 4.0 * total_vol;
    Ok(Point3::new(cx / denom, cy / denom, cz / denom))
}

#[cfg(test)]
mod regression_tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;
    use remus_topology::builder::{make_face_from_wire, make_polygon_wire};
    use remus_topology::face::FaceSurface;

    fn unit_square_extrude_volume() -> (f64, bool) {
        let mut topo = Topology::new();
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let wire = make_polygon_wire(&mut topo, &pts, 1e-7).unwrap();
        let face = make_face_from_wire(&mut topo, wire).unwrap();
        let cap_is_plane = matches!(
            topo.face(face).unwrap().surface(),
            FaceSurface::Plane { .. }
        );
        let solid =
            crate::extrude::extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 1.0).unwrap();
        let vol = solid_volume(&topo, solid, 0.01).unwrap();
        (vol, cap_is_plane)
    }

    #[test]
    fn unit_square_extrude_volume_is_one() {
        let (vol, cap_is_plane) = unit_square_extrude_volume();
        assert!(
            cap_is_plane,
            "axis-aligned square cap must be a planar face"
        );
        assert!((vol - 1.0).abs() < 1e-6, "expected 1.0, got {vol}");
    }

    #[test]
    fn rectangle_extrude_volume_matches_box() {
        // A non-square, non-unit axis-aligned rectangle whose winding-derived
        // cap normal must still come out correct.
        let mut topo = Topology::new();
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(5.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ];
        let wire = make_polygon_wire(&mut topo, &pts, 1e-7).unwrap();
        let face = make_face_from_wire(&mut topo, wire).unwrap();
        let solid =
            crate::extrude::extrude(&mut topo, face, Vec3::new(0.0, 0.0, 1.0), 3.0).unwrap();
        let vol = solid_volume(&topo, solid, 0.01).unwrap();
        assert!((vol - 30.0).abs() < 1e-6, "expected 30.0, got {vol}");
    }

    /// Build the census Steinmetz fuse: two equal r=3, h=20 cylinders with
    /// perpendicular intersecting axes (one along z, one along x), fused.
    fn steinmetz_fuse_census() -> (Topology, SolidId) {
        use remus_math::mat::Mat4;
        let mut topo = Topology::new();
        let c1 = crate::primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
        crate::transform::transform_solid(&mut topo, c1, &Mat4::translation(0.0, 0.0, -10.0))
            .unwrap();
        let c2 = crate::primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            c2,
            &Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
        )
        .unwrap();
        crate::transform::transform_solid(&mut topo, c2, &Mat4::translation(-10.0, 0.0, 0.0))
            .unwrap();
        let res =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, c1, c2).unwrap();
        (topo, res)
    }

    #[test]
    fn steinmetz_lens_fuse_closed_form_volume() {
        let (topo, res) = steinmetz_fuse_census();
        // The gate fires, and the closed form gives the EXACT volume:
        // V = π·9·(20+20) − (16/3)·27 = 1130.97 − 144 = 986.97.
        let faces = remus_topology::explorer::solid_faces(&topo, res).unwrap();
        assert!(
            solid_is_steinmetz_lens_fuse(&topo, &faces),
            "the perpendicular cyl∪cyl fuse must be detected as the lens fuse"
        );
        let v = steinmetz_lens_fuse_volume(&topo, &faces).expect("closed form");
        let expect = std::f64::consts::PI * 9.0 * 40.0 - 16.0 / 3.0 * 27.0;
        assert!(
            (v - expect).abs() < 1e-9,
            "closed-form lens volume {v} should equal {expect} (986.97)"
        );
        // The public `solid_volume` returns the same exact value.
        let vol = solid_volume(&topo, res, 0.01).unwrap();
        assert!(
            (vol - expect).abs() < 1e-6,
            "solid_volume {vol} should match closed form {expect}"
        );
    }

    #[test]
    fn steinmetz_gate_does_not_fire_on_plain_or_coaxial_cylinders() {
        use remus_math::mat::Mat4;
        // A plain cylinder (one wall, no holes) is NOT the lens fuse.
        let mut topo = Topology::new();
        let cyl = crate::primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, cyl).unwrap();
        assert!(
            !solid_is_steinmetz_lens_fuse(&topo, &faces),
            "a plain cylinder is not the lens fuse"
        );

        // A coaxial cyl∩cyl (two collinear cylinders, no mutually-trimmed lens
        // walls) is NOT the lens fuse.
        let mut topo2 = Topology::new();
        let a = crate::primitives::make_cylinder(&mut topo2, 5.0, 20.0).unwrap();
        let b = crate::primitives::make_cylinder(&mut topo2, 5.0, 20.0).unwrap();
        crate::transform::transform_solid(&mut topo2, b, &Mat4::translation(0.0, 0.0, 10.0))
            .unwrap();
        let inter = crate::boolean::boolean(&mut topo2, crate::boolean::BooleanOp::Intersect, a, b)
            .unwrap();
        let f2 = remus_topology::explorer::solid_faces(&topo2, inter).unwrap();
        assert!(
            !solid_is_steinmetz_lens_fuse(&topo2, &f2),
            "coaxial cyl∩cyl is not the lens fuse"
        );
    }

    #[test]
    fn cyl_perp_intersecting_predicate() {
        use remus_math::surfaces::CylindricalSurface;
        let cyl = |o: Point3, a: Vec3, r: f64| CylindricalSurface::new(o, a, r).unwrap();
        let z = cyl(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 3.0);

        // The census config: z⊥x, axes meet at the origin, equal r → Some(origin).
        let x = cyl(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 3.0);
        let isect = cylinders_perpendicular_and_intersecting(&z, &x).expect("axes meet");
        assert!((isect - Point3::new(0.0, 0.0, 0.0)).length() < 1e-9);

        // Unequal radius → None.
        let x_big = cyl(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 4.0);
        assert!(cylinders_perpendicular_and_intersecting(&z, &x_big).is_none());

        // Non-perpendicular (45°), intersecting, equal r → None (its
        // intersection is NOT 16r³/3, so the closed form would be wrong).
        let diag = cyl(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 1.0), 3.0);
        assert!(cylinders_perpendicular_and_intersecting(&z, &diag).is_none());

        // Parallel-offset equal r (both along z) → None (not perpendicular).
        let z_off = cyl(Point3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0), 3.0);
        assert!(cylinders_perpendicular_and_intersecting(&z, &z_off).is_none());

        // Perpendicular but SKEW (x-axis shifted in y so its line never meets
        // the z-axis) → None (not intersecting).
        let x_skew = cyl(Point3::new(0.0, 5.0, 8.0), Vec3::new(1.0, 0.0, 0.0), 3.0);
        assert!(cylinders_perpendicular_and_intersecting(&z, &x_skew).is_none());

        // Perpendicular + intersecting but the meet point is OFF-origin
        // (x-axis through (0,0,4)): returns Some at that point.
        let x_high = cyl(Point3::new(0.0, 0.0, 4.0), Vec3::new(1.0, 0.0, 0.0), 3.0);
        let isect_high = cylinders_perpendicular_and_intersecting(&z, &x_high).expect("axes meet");
        assert!((isect_high - Point3::new(0.0, 0.0, 4.0)).length() < 1e-9);
    }

    #[test]
    fn drilled_cylinder_volume_subtracts_the_bore() {
        // A coaxial tube (cylinder r=5 h=20 with a coaxial r=2 bore cut through
        // it). Its holed analytic faces (annular planar caps with inner wires +
        // two cylinder walls) must NOT hit a hole-FILLING analytic fast-path —
        // they route to the hole-aware revolution integrator (which subtracts
        // cap holes by MAGNITUDE, immune to a boolean's same-wound inner rims),
        // giving the bore-subtracted V = π·(5²−2²)·20 = π·420, not the
        // solid-cylinder π·500.
        use std::f64::consts::PI;
        let mut topo = Topology::new();
        let outer = crate::primitives::make_cylinder(&mut topo, 5.0, 20.0).unwrap();
        let bore = crate::primitives::make_cylinder(&mut topo, 2.0, 20.0).unwrap();
        let tube = crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Cut, outer, bore)
            .unwrap();

        // Neither analytic fast-path may claim this holed solid.
        assert!(
            try_analytic_solid_volume(&topo, tube).is_none(),
            "the holed tube must not hit the whole-solid analytic primitive path"
        );
        assert!(
            analytic_faces_solid_volume(&topo, tube).unwrap().is_none(),
            "the holed tube must not hit the per-face analytic path (it ignores holes)"
        );

        let expect = PI * (25.0 - 4.0) * 20.0; // bore subtracted
        let vol = solid_volume(&topo, tube, 0.005).unwrap();
        let solid_cyl = PI * 25.0 * 20.0; // hole-FILLED (the wrong answer)
        assert!(
            (vol - expect).abs() < expect * 0.01,
            "drilled tube volume {vol} should be the bore-subtracted {expect}, \
             not the hole-filled {solid_cyl}"
        );
        assert!(
            (vol - solid_cyl).abs() > solid_cyl * 0.05,
            "drilled tube volume {vol} must be clearly LESS than the solid cylinder \
             {solid_cyl} (the bore is really removed)"
        );
    }

    #[test]
    fn plain_primitives_still_use_the_analytic_fast_path() {
        // The Finding-3 hole guard must not over-gate: a plain (hole-less)
        // cylinder, cone, sphere and torus must STILL hit the closed-form
        // analytic fast-path (no tessellation perf regression).
        use std::f64::consts::PI;
        let mut t = Topology::new();
        let cyl = crate::primitives::make_cylinder(&mut t, 3.0, 10.0).unwrap();
        let v = try_analytic_solid_volume(&t, cyl).expect("plain cylinder fast-path");
        assert!((v - PI * 9.0 * 10.0).abs() < 1e-9);

        let mut t = Topology::new();
        let cone = crate::primitives::make_cone(&mut t, 4.0, 0.0, 9.0).unwrap();
        let v = try_analytic_solid_volume(&t, cone).expect("plain cone fast-path");
        assert!((v - PI / 3.0 * 16.0 * 9.0).abs() < 1e-6);

        let mut t = Topology::new();
        let sph = crate::primitives::make_sphere(&mut t, 5.0, 32).unwrap();
        let v = try_analytic_solid_volume(&t, sph).expect("plain sphere fast-path");
        assert!((v - 4.0 / 3.0 * PI * 125.0).abs() < 1e-6);

        let mut t = Topology::new();
        let tor = crate::primitives::make_torus(&mut t, 6.0, 2.0, 32).unwrap();
        let v = try_analytic_solid_volume(&t, tor).expect("plain torus fast-path");
        assert!((v - 2.0 * PI * PI * 6.0 * 4.0).abs() < 1e-6);
    }

    #[test]
    fn truncated_perpendicular_fuse_gate_defers() {
        // A SHORT perpendicular equal-radius fuse: the second cylinder is only
        // h=2 (< r=3 past the axis intersection on each side), so the lens is
        // truncated and the infinite-cylinder term −16r³/3 would be wrong. The
        // gate must DECLINE so tessellation computes the true (truncated) volume.
        use remus_math::mat::Mat4;
        let mut topo = Topology::new();
        let c1 = crate::primitives::make_cylinder(&mut topo, 3.0, 20.0).unwrap();
        crate::transform::transform_solid(&mut topo, c1, &Mat4::translation(0.0, 0.0, -10.0))
            .unwrap();
        // Short cross cylinder: h=2, centred on the z-axis (caps at x=±1, only 1
        // past the intersection — less than r=3).
        let c2 = crate::primitives::make_cylinder(&mut topo, 3.0, 2.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            c2,
            &Mat4::rotation_y(std::f64::consts::FRAC_PI_2),
        )
        .unwrap();
        crate::transform::transform_solid(&mut topo, c2, &Mat4::translation(-1.0, 0.0, 0.0))
            .unwrap();
        let res =
            crate::boolean::boolean(&mut topo, crate::boolean::BooleanOp::Fuse, c1, c2).unwrap();
        let faces = remus_topology::explorer::solid_faces(&topo, res).unwrap();
        assert!(
            !solid_is_steinmetz_lens_fuse(&topo, &faces),
            "a truncated (short) perpendicular fuse must not use the infinite-cylinder closed form"
        );
    }

    #[test]
    fn gate_rejects_extra_face_beyond_the_lens_fuse() {
        // The two-cylinder closed form must fire ONLY when the solid is EXACTLY
        // the lens fuse. A solid carrying the lens pair PLUS an extra attached
        // cylinder still has two holed walls + their caps, but the extra
        // cylinder's volume would be dropped — so the gate must account for
        // every face and reject any foreign one.
        let (mut topo, res) = steinmetz_fuse_census();
        let census_faces = remus_topology::explorer::solid_faces(&topo, res).unwrap();
        // Sanity: the clean census (2 holed walls + 4 caps) passes.
        assert!(solid_is_steinmetz_lens_fuse(&topo, &census_faces));

        // Build a separate plain cylinder in the SAME arena and grab its
        // (UNHOLED) cylindrical wall face + one of its caps.
        let extra = crate::primitives::make_cylinder(&mut topo, 1.0, 4.0).unwrap();
        let extra_faces = remus_topology::explorer::solid_faces(&topo, extra).unwrap();
        let extra_cyl = extra_faces
            .iter()
            .copied()
            .find(|&f| {
                topo.face(f)
                    .is_ok_and(|fc| matches!(fc.surface(), FaceSurface::Cylinder(_)))
            })
            .expect("plain cylinder wall face");
        let extra_cap = extra_faces
            .iter()
            .copied()
            .find(|&f| {
                topo.face(f)
                    .is_ok_and(|fc| matches!(fc.surface(), FaceSurface::Plane { .. }))
            })
            .expect("plain cylinder cap face");

        // Lens pair + an extra UNHOLED cylinder wall → reject (would drop its
        // volume).
        let mut with_extra_cyl = census_faces.clone();
        with_extra_cyl.push(extra_cyl);
        assert!(
            !solid_is_steinmetz_lens_fuse(&topo, &with_extra_cyl),
            "an extra unholed cylinder face must make the gate decline"
        );

        // Lens pair + an extra planar cap whose normal is NOT aligned with
        // either lens axis (a tilted cylinder's cap) → reject.
        let tilted = crate::primitives::make_cylinder(&mut topo, 1.0, 4.0).unwrap();
        crate::transform::transform_solid(
            &mut topo,
            tilted,
            &remus_math::mat::Mat4::rotation_x(0.7),
        )
        .unwrap();
        let tilted_cap = remus_topology::explorer::solid_faces(&topo, tilted)
            .unwrap()
            .into_iter()
            .find(|&f| {
                topo.face(f)
                    .is_ok_and(|fc| matches!(fc.surface(), FaceSurface::Plane { .. }))
            })
            .expect("tilted cylinder cap");
        let mut with_foreign_cap = census_faces.clone();
        with_foreign_cap.push(tilted_cap);
        assert!(
            !solid_is_steinmetz_lens_fuse(&topo, &with_foreign_cap),
            "a planar cap not aligned with either lens axis must make the gate decline"
        );

        // An EXTRA axis-aligned plane is also rejected: the lens fuse has EXACTLY
        // two caps per axis, so a fifth cap aligned with `a0` (here the z-aligned
        // plain-cylinder cap) makes `caps_a0 == 3` — a foreign attached body whose
        // volume the closed form would silently drop.
        let mut with_aligned_cap = census_faces;
        with_aligned_cap.push(extra_cap);
        assert!(
            !solid_is_steinmetz_lens_fuse(&topo, &with_aligned_cap),
            "an extra axis-aligned cap beyond the exactly-four lens caps must make the gate decline"
        );
    }

    /// A full-disc cap bounded by a SINGLE closed circle (`v→v`) must integrate to
    /// the exact disc area `πρ²` — its 2π sweep, not a dropped zero. Exercised via
    /// a TWO-section revolution (two stacked cone frustums → two cone walls + two
    /// closed-circle disc caps), which the single-primitive volume path declines
    /// (two cones), forcing `analytic_revolution_solid_volume`. The volume must be
    /// exact AND deflection-independent (analytic, not the inscribed mesh).
    #[test]
    fn closed_circle_disc_cap_volume_is_exact() {
        use remus_topology::explorer::solid_faces;
        use std::f64::consts::{PI, TAU};
        let mut topo = Topology::new();
        // (6,0)→(4,6) cone, (4,6)→(2,12) cone, (2,12)→(0,12) top disc cap,
        // (0,12)→(0,0) on-axis (no face); bottom (0,0)→(6,0) disc cap.
        let wire = make_polygon_wire(
            &mut topo,
            &[
                Point3::new(6.0, 0.0, 0.0),
                Point3::new(4.0, 0.0, 6.0),
                Point3::new(2.0, 0.0, 12.0),
                Point3::new(0.0, 0.0, 12.0),
                Point3::new(0.0, 0.0, 0.0),
            ],
            1e-7,
        )
        .unwrap();
        let face = topo.add_face(remus_topology::face::Face::new(
            wire,
            vec![],
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 1.0, 0.0),
                d: 0.0,
            },
        ));
        let solid = crate::revolve::revolve(
            &mut topo,
            face,
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            TAU,
        )
        .unwrap();

        // The merged solid has 2 cone walls + 2 closed-circle disc caps; the top
        // cap's closed circle must contribute (1/3)·12·π·2² to the volume.
        let fr = |rb: f64, rt: f64, h: f64| PI * h / 3.0 * rb.mul_add(rb, rb.mul_add(rt, rt * rt));
        let expected = fr(6.0, 4.0, 6.0) + fr(4.0, 2.0, 6.0);

        let v_fine = solid_volume(&topo, solid, 0.0001).unwrap();
        let v_coarse = solid_volume(&topo, solid, 0.1).unwrap();
        assert!(
            (v_fine - expected).abs() / expected < 1e-9,
            "two-section revolve volume {expected}, got {v_fine}"
        );
        assert!(
            (v_fine - v_coarse).abs() < 1e-9,
            "volume must be analytic (deflection-independent): {v_fine} vs {v_coarse}"
        );

        // Direct check: the top disc cap (a single closed circle, radius 2 at
        // z=12) contributes exactly (1/3)·12·π·4 via `planar_cap_signed_volume`.
        let top_cap = solid_faces(&topo, solid)
            .unwrap()
            .into_iter()
            .find(|&fid| {
                let f = topo.face(fid).unwrap();
                matches!(f.surface(), FaceSurface::Plane { .. })
                    && topo.wire(f.outer_wire()).unwrap().edges().iter().all(|oe| {
                        topo.vertex(topo.edge(oe.edge()).unwrap().start())
                            .unwrap()
                            .point()
                            .z()
                            > 11.0
                    })
            })
            .expect("top disc cap");
        let cap_v = planar_cap_signed_volume(&topo, top_cap).unwrap().unwrap();
        assert!(
            (cap_v.abs() - 12.0 * PI * 4.0 / 3.0).abs() < 1e-9,
            "closed-circle disc cap contribution should be (1/3)·12·π·4, got {cap_v}"
        );
    }

    /// An ANNULAR planar cap (outer rim + a reversed inner-rim hole) must
    /// SUBTRACT the inner circular segment, giving area `π(R²−r²)`. The arc-bulge
    /// sweep must respect the oriented (reversed) inner rim — otherwise the inner
    /// segment is added (inflated area `π(R²+r²)`).
    #[test]
    fn annular_cap_volume_is_exact() {
        use remus_math::curves::Circle3D;
        use remus_topology::edge::{Edge, EdgeCurve};
        use remus_topology::face::Face;
        use remus_topology::vertex::Vertex;
        use remus_topology::wire::{OrientedEdge, Wire};
        use std::f64::consts::PI;

        let (r_out, r_in, h) = (7.0_f64, 5.0_f64, 4.0_f64);
        let axis = Vec3::new(0.0, 0.0, 1.0);
        let mut topo = Topology::new();

        // Outer rim: a closed CCW circle at z = h, radius r_out.
        let v_out = topo.add_vertex(Vertex::new(Point3::new(r_out, 0.0, h), 1e-7));
        let outer_c = Circle3D::new(Point3::new(0.0, 0.0, h), axis, r_out).unwrap();
        let e_out = topo.add_edge(Edge::new(v_out, v_out, EdgeCurve::Circle(outer_c)));
        let outer_wire =
            topo.add_wire(Wire::new(vec![OrientedEdge::new(e_out, true)], true).unwrap());

        // Inner rim (the hole): a closed circle at the same z, radius r_in, wound
        // OPPOSITE the outer wire — here via a reversed `OrientedEdge`.
        let v_in = topo.add_vertex(Vertex::new(Point3::new(r_in, 0.0, h), 1e-7));
        let inner_c = Circle3D::new(Point3::new(0.0, 0.0, h), axis, r_in).unwrap();
        let e_in = topo.add_edge(Edge::new(v_in, v_in, EdgeCurve::Circle(inner_c)));
        let inner_wire =
            topo.add_wire(Wire::new(vec![OrientedEdge::new(e_in, false)], true).unwrap());

        let cap = topo.add_face(Face::new(
            outer_wire,
            vec![inner_wire],
            FaceSurface::Plane { normal: axis, d: h },
        ));

        let cap_v = planar_cap_signed_volume(&topo, cap).unwrap().unwrap();
        // Exact annulus contribution: (1/3)·h·π·(R²−r²) (outward normal +axis).
        let expected = h * PI * (r_out * r_out - r_in * r_in) / 3.0;
        assert!(
            (cap_v.abs() - expected).abs() < 1e-9,
            "annular cap contribution should subtract the inner segment: \
             expected {expected}, got {cap_v} (inflated would be {})",
            h * PI * (r_out * r_out + r_in * r_in) / 3.0
        );
    }
}
