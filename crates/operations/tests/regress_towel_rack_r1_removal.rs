//! Exact removal of imported R1 beads on the towel-rack body.
//!
//! Fixture: `tests/data/towel_rack_r1_body.step`, sanitized from the supplied
//! `Towel Rack v2.step` (original sha256
//! `5b77f2fa67730a319136dc9516526dd8896b31e06eb319ed56cac625c5669f0f`,
//! verified byte-identical to the saved project's embedded STEP before
//! sanitizing). Sanitization touched only identity metadata: the multi-line
//! `FILE_NAME` statement (user path, author, timestamp) became one neutral
//! line, and one `DESCRIPTIVE_REPRESENTATION_ITEM` tmp path was neutralized.
//! Reimporting both files gives identical volume (1e-12), face census, all 15
//! cylinder frames (bit-identical), and all 23 planes (bit-identical), so the
//! fixture preserves geometry and topology exactly. The `.openzcad` project
//! itself is not committed: it carries owner/project identities.
//!
//! Body facts (re-derived here, never hard-coded as handles): one solid, 46
//! faces (23 planes, 15 cylinders, 4 spheres, 2 tori, 2 cones), volume
//! 136978.772725 mm³, valid. Four R1 (r = 1 mm) cylinders, axis +X, surface
//! origin x = 43: two with all-planar neighborhoods (STEP #1293/#1318, wire
//! 4-cycles, centroids z ≈ 65.4) and two ending on the R8 cylinder
//! (STEP #1281/#1306, 5-edge wires with a NURBS R8 contact, centroids
//! z ≈ 83.34). STEP entity numbers are investigation hints only; every
//! selection below is geometric (radius, axis, wire structure, centroid).
//!
//! Every success test asserts exactness (analytic volume oracle, sharp-edge
//! position, sibling placement, carrier census, edge-count deltas) or fails
//! loudly. Tessellation success is never used as proof.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::tolerance::Tolerance;
use remus_math::vec::Vec3;
use remus_topology::Topology;
use remus_topology::edge::EdgeCurve;
use remus_topology::face::{FaceId, FaceSurface};
use remus_topology::solid::SolidId;

fn load_fixture() -> (Topology, SolidId) {
    let step = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/data/towel_rack_r1_body.step"
    ))
    .unwrap();
    let mut topo = Topology::new();
    let solids = remus_io::step::read_step(&step, &mut topo).unwrap();
    assert_eq!(solids.len(), 1, "fixture holds one solid");
    (topo, solids[0])
}

fn centroid_of(topo: &Topology, face: FaceId) -> (f64, f64, f64) {
    let polygon = remus_operations::boolean::face_polygon(topo, face).unwrap();
    #[allow(clippy::cast_precision_loss)]
    let n = polygon.len() as f64;
    polygon.iter().fold((0.0, 0.0, 0.0), |(x, y, z), p| {
        (x + p.x() / n, y + p.y() / n, z + p.z() / n)
    })
}

fn dist2(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)
}

/// Select an R1 band geometrically: radius ≈ 1, axis parallel to X, surface
/// origin x ≈ 43, outer-wire edge count, and centroid hemisphere.
///
/// `wire_edges` 4 selects the all-planar pair (centroid z ≈ 65.4);
/// 5 selects the R8-ending pair (centroid z ≈ 83.34). `y_sign` picks the side.
fn select_r1(
    topo: &Topology,
    solid: SolidId,
    wire_edges: usize,
    y_sign: f64,
    what: &str,
) -> FaceId {
    let tol = Tolerance::new();
    let mut candidates: Vec<FaceId> = remus_topology::explorer::solid_faces(topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| {
            let FaceSurface::Cylinder(cylinder) = topo.face(*face).unwrap().surface() else {
                return false;
            };
            if !tol.approx_eq(cylinder.radius(), 1.0) {
                return false;
            }
            let axis = cylinder.axis().normalize().unwrap();
            if (axis.x().abs() - 1.0).abs() > 1e-9 {
                return false;
            }
            if (cylinder.origin().x() - 43.0).abs() > 1e-6 {
                return false;
            }
            if topo
                .wire(topo.face(*face).unwrap().outer_wire())
                .unwrap()
                .edges()
                .len()
                != wire_edges
            {
                return false;
            }
            if !topo.face(*face).unwrap().inner_wires().is_empty() {
                return false;
            }
            let centroid = centroid_of(topo, *face);
            (centroid.1 > 0.0) == (y_sign > 0.0)
        })
        .collect();
    assert_eq!(
        candidates.len(),
        1,
        "{what}: exactly one R1 band must match"
    );
    candidates.pop().unwrap()
}

/// Split a band's neighbors into tangent planar supports (line contacts) and
/// end faces (arc contacts). Requires exactly two of each, all planar.
fn supports_and_ends(topo: &Topology, solid: SolidId, band: FaceId) -> (Vec<FaceId>, Vec<FaceId>) {
    let adjacency = topo.build_adjacency(solid).unwrap();
    let band_data = topo.face(band).unwrap();
    let mut supports = Vec::new();
    let mut ends = Vec::new();
    for oriented in topo.wire(band_data.outer_wire()).unwrap().edges() {
        let edge = topo.edge(oriented.edge()).unwrap();
        let mut neighbors: Vec<FaceId> = adjacency
            .faces_for_edge(oriented.edge())
            .iter()
            .copied()
            .filter(|face| *face != band)
            .collect();
        neighbors.sort_unstable_by_key(|face| face.index());
        neighbors.dedup();
        assert_eq!(
            neighbors.len(),
            1,
            "band boundary must be manifold, edge {} has {:?}",
            oriented.edge().index(),
            neighbors.iter().map(|f| f.index()).collect::<Vec<_>>()
        );
        let neighbor = neighbors[0];
        assert!(
            topo.face(neighbor).unwrap().surface().is_planar(),
            "band neighbor {} must be planar",
            neighbor.index()
        );
        if matches!(edge.curve(), EdgeCurve::Line) {
            if !supports.contains(&neighbor) {
                supports.push(neighbor);
            }
        } else if !ends.contains(&neighbor) {
            ends.push(neighbor);
        }
    }
    assert_eq!(supports.len(), 2, "two tangent planar supports");
    assert_eq!(ends.len(), 2, "two end faces");
    (supports, ends)
}

fn unit_plane(topo: &Topology, face: FaceId) -> (Vec3, f64) {
    match topo.face(face).unwrap().surface() {
        FaceSurface::Plane { normal, d } => {
            let unit = normal.normalize().unwrap();
            let scale = unit.dot(*normal);
            assert!(scale.abs() > 1e-12, "degenerate plane normal");
            (unit, *d / scale)
        }
        other => panic!("expected a plane, got {}", other.type_tag()),
    }
}

fn triple(a: (Vec3, f64), b: (Vec3, f64), c: (Vec3, f64)) -> Option<(f64, f64, f64)> {
    let bc = b.0.cross(c.0);
    let det = a.0.dot(bc);
    if det.abs() < 1e-6 {
        return None;
    }
    let ca = c.0.cross(a.0);
    let ab = a.0.cross(b.0);
    let v = (bc * a.1 + ca * b.1 + ab * c.1) * (1.0 / det);
    Some((v.x(), v.y(), v.z()))
}

/// Analytic oracle for the bead volume removed with a z = 65.9-class strip.
///
/// The bead cross-section perpendicular to X is translation-invariant: the
/// unit footprint (square corner `(y0, z0)`, tangent circle center
/// `(yc, zc)`, radius `r`) clipped below by `x = x_lo` and above by the
/// oblique end plane `(nx, 0, nz) · p = d`. With `sy = sign(yc - y0)` and
/// `sz = sign(zc - z0)`, closed-form moments give `m0 = r²(1-pi/4)`,
/// `m1 = z0 * m0 + sz * r³(1-pi/4-1/6)`, and
/// `V = ((d - nx * x_lo) * m0 - nz * m1) / nx`. Every input is measured from
/// the imported topology; tangency and axis-alignment structure is asserted.
#[allow(clippy::too_many_arguments)]
fn bead_volume_oracle(
    y0: f64,
    z0: f64,
    yc: f64,
    zc: f64,
    r: f64,
    x_lo: f64,
    nx: f64,
    nz: f64,
    d: f64,
    what: &str,
) -> f64 {
    assert!(
        ((yc - y0).abs() - r).abs() < 1e-9 && ((zc - z0).abs() - r).abs() < 1e-9,
        "{what}: tangent bead structure"
    );
    let sz = (zc - z0).signum();
    assert!(
        (yc - y0).abs() > 1e-12 && sz.abs() > 0.0,
        "{what}: non-degenerate bead"
    );
    let m0 = r * r * (1.0 - std::f64::consts::PI / 4.0);
    let m1 = z0 * m0 + sz * r * r * r * (1.0 - std::f64::consts::PI / 4.0 - 1.0 / 6.0);
    let volume = ((d - nx * x_lo) * m0 - nz * m1) / nx;
    assert!(volume > 0.0, "{what}: positive bead volume");
    volume
}

fn surface_census(topo: &Topology, solid: SolidId) -> Vec<(String, usize)> {
    let mut map = std::collections::BTreeMap::new();
    for face in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        *map.entry(topo.face(face).unwrap().surface().type_tag().to_string())
            .or_insert(0) += 1;
    }
    map.into_iter().collect()
}

fn edge_census(topo: &Topology, solid: SolidId) -> Vec<(String, usize)> {
    let mut map = std::collections::BTreeMap::new();
    for edge in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        let tag = match topo.edge(edge).unwrap().curve() {
            EdgeCurve::Line => "Line",
            EdgeCurve::Circle(_) => "Circle",
            EdgeCurve::Ellipse(_) => "Ellipse",
            EdgeCurve::NurbsCurve(_) => "Nurbs",
            _ => "other",
        };
        *map.entry(tag.to_string()).or_insert(0) += 1;
    }
    map.into_iter().collect()
}

fn assert_valid(topo: &Topology, solid: SolidId, what: &str) {
    let report = remus_operations::validate::validate_solid(topo, solid).unwrap();
    assert!(
        report.is_valid(),
        "{what} must validate, got {:?}",
        report
            .issues
            .iter()
            .map(|issue| issue.description.clone())
            .collect::<Vec<_>>()
    );
}

/// Measured oracle bundle for one planar R1 band.
type BeadOracle = (f64, Vec<FaceId>, Vec<FaceId>, Vec<(f64, f64, f64)>);

/// Analytic bead-loss oracle plus the measured planes and sharp corners it
/// was derived from, for one planar R1 band. All inputs are measured from
/// the imported topology; tangency and axis-alignment structure is asserted.
fn measured_bead_oracle(topo: &Topology, solid: SolidId, band: FaceId, what: &str) -> BeadOracle {
    let (supports, ends) = supports_and_ends(topo, solid, band);
    let (n0, d0) = unit_plane(topo, supports[0]);
    let (n1, d1) = unit_plane(topo, supports[1]);
    let y0 = if n0.y().abs() > 0.5 {
        assert!(
            n0.x().abs() < 1e-12 && n0.z().abs() < 1e-12,
            "{what}: y support axis"
        );
        d0 / n0.y()
    } else {
        assert!(n1.y().abs() > 0.5, "{what}: one y support");
        assert!(
            n1.x().abs() < 1e-12 && n1.z().abs() < 1e-12,
            "{what}: y support axis"
        );
        d1 / n1.y()
    };
    let z0 = if n0.z().abs() > 0.5 {
        assert!(
            n0.x().abs() < 1e-12 && n0.y().abs() < 1e-12,
            "{what}: z support axis"
        );
        d0 / n0.z()
    } else {
        assert!(n1.z().abs() > 0.5, "{what}: one z support");
        d1 / n1.z()
    };
    let FaceSurface::Cylinder(band_cyl) = topo.face(band).unwrap().surface().clone() else {
        unreachable!("R1 selector guarantees a cylinder");
    };
    assert!(
        (band_cyl.axis().x().abs() - 1.0).abs() < 1e-12,
        "{what}: band axis"
    );
    let (yc, zc) = (band_cyl.origin().y(), band_cyl.origin().z());
    let (mut x_lo, mut oblique) = (None, None);
    for &end in &ends {
        let (n, d) = unit_plane(topo, end);
        if n.y().abs() < 1e-12 && n.z().abs() < 1e-12 {
            assert!(x_lo.is_none(), "{what}: one perpendicular cap");
            x_lo = Some(d / n.x());
        } else {
            assert!(n.y().abs() < 1e-12, "{what}: oblique cap contains Y");
            assert!(oblique.is_none(), "{what}: one oblique cap");
            oblique = Some((n, d));
        }
    }
    let (x_lo, (nhi, dhi)) = (
        x_lo.expect("perpendicular cap"),
        oblique.expect("oblique cap"),
    );
    let loss = bead_volume_oracle(y0, z0, yc, zc, 1.0, x_lo, nhi.x(), nhi.z(), dhi, what);
    let corners: Vec<(f64, f64, f64)> = ends
        .iter()
        .map(|end| {
            let pe = unit_plane(topo, *end);
            triple((n0, d0), (n1, d1), pe).expect("sharp corner exists")
        })
        .collect();
    (loss, supports, ends, corners)
}

/// Heal one planar R1 and prove every exactness claim about the result.
/// Returns the healed topology for sequential tests.
fn heal_planar_r1(y_sign: f64, what: &str) -> (Topology, SolidId, f64) {
    let tol = Tolerance::new();
    let (mut topo, solid) = load_fixture();
    let band = select_r1(&topo, solid, 4, y_sign, what);
    let (expected_loss, supports, _ends, expected_corners) =
        measured_bead_oracle(&topo, solid, band, what);

    let faces_before = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let edges_before = edge_census(&topo, solid);
    let volume_before = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();
    let sibling_r1s = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| *face != band)
        .filter(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder) if tol.approx_eq(cylinder.radius(), 1.0)
            )
        })
        .count();
    assert_eq!(sibling_r1s, 3, "{what}: three sibling R1s present");

    let result =
        remus_operations::resize_blend::resize_blend(&mut topo, solid, band, 1.0, 0.0).unwrap();
    assert_valid(&topo, result.solid, what);
    let faces = remus_topology::explorer::solid_faces(&topo, result.solid).unwrap();
    assert_eq!(faces.len(), faces_before - 1, "{what}: only the band goes");
    let volume_after = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    let loss = volume_before - volume_after;
    assert!(
        (loss - expected_loss).abs() < 1e-4,
        "{what}: bead loss {loss:.6} differs from analytic oracle {expected_loss:.6}"
    );

    // The sharp edge: a line shared by the healed supports with endpoints at
    // the measured triples.
    let adjacency = topo.build_adjacency(result.solid).unwrap();
    let healed_supports: Vec<FaceId> = result
        .evolution
        .modified
        .iter()
        .filter_map(|(&source, targets)| {
            let [target] = targets.as_slice() else {
                return None;
            };
            supports
                .iter()
                .any(|support| support.index() == source)
                .then(|| topo.face_id_from_index(*target))?
        })
        .collect();
    assert_eq!(
        healed_supports.len(),
        2,
        "{what}: supports evolve one-to-one"
    );
    let mut sharp_found = false;
    for edge in remus_topology::explorer::solid_edges(&topo, result.solid).unwrap() {
        let incident = adjacency.faces_for_edge(edge);
        if incident.len() != 2
            || !incident.iter().all(|face| healed_supports.contains(face))
            || !matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line)
        {
            continue;
        }
        let edge_data = topo.edge(edge).unwrap();
        let a = topo.vertex(edge_data.start()).unwrap().point();
        let b = topo.vertex(edge_data.end()).unwrap().point();
        let matches = |p: (f64, f64, f64), q: (f64, f64, f64)| {
            ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2) + (p.2 - q.2).powi(2)).sqrt() < 1e-6
        };
        let a3 = (a.x(), a.y(), a.z());
        let b3 = (b.x(), b.y(), b.z());
        if (matches(a3, expected_corners[0]) && matches(b3, expected_corners[1]))
            || (matches(a3, expected_corners[1]) && matches(b3, expected_corners[0]))
        {
            sharp_found = true;
        }
    }
    assert!(sharp_found, "{what}: sharp edge at the measured corners");

    // Siblings survive with exact radius and placement.
    let mut surviving: Vec<((f64, f64, f64), f64)> = faces
        .iter()
        .filter_map(|face| match topo.face(*face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder) if tol.approx_eq(cylinder.radius(), 1.0) => {
                Some((centroid_of(&topo, *face), cylinder.radius()))
            }
            _ => None,
        })
        .collect();
    assert_eq!(surviving.len(), 3, "{what}: three sibling R1s survive");
    surviving.sort_by(|a, b| a.0.2.total_cmp(&b.0.2).then(a.0.1.total_cmp(&b.0.1)));
    let mut before: Vec<(f64, f64, f64)> = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .into_iter()
        .filter(|face| *face != band)
        .filter_map(|face| match topo.face(face).unwrap().surface() {
            FaceSurface::Cylinder(cylinder) if tol.approx_eq(cylinder.radius(), 1.0) => {
                Some(centroid_of(&topo, face))
            }
            _ => None,
        })
        .collect();
    before.sort_by(|a, b| a.2.total_cmp(&b.2).then(a.1.total_cmp(&b.1)));
    assert_eq!(before.len(), 3, "{what}: three sibling baselines");
    for ((centroid, radius), baseline) in surviving.iter().zip(before.iter()) {
        assert!((radius - 1.0).abs() < 1e-9, "{what}: sibling radius");
        assert!(
            dist2(*centroid, *baseline) < 1e-12,
            "{what}: sibling placement"
        );
    }

    // Carriers: one cylinder fewer, everything else identical.
    assert_eq!(
        surface_census(&topo, result.solid),
        surface_census(&topo, solid)
            .into_iter()
            .map(|(tag, count)| {
                if tag == "cylinder" {
                    (tag, count - 1)
                } else {
                    (tag, count)
                }
            })
            .collect::<Vec<_>>(),
        "{what}: carrier census"
    );
    // Exact topology deltas: one cross circle, one cross ellipse, and the two
    // springs replaced by one sharp line; NURBS contacts untouched.
    let edges_after = edge_census(&topo, result.solid);
    let count = |census: &[(String, usize)], tag: &str| {
        census.iter().find(|(t, _)| t == tag).map_or(0, |(_, n)| *n)
    };
    assert_eq!(
        count(&edges_after, "Circle"),
        count(&edges_before, "Circle") - 1,
        "{what}: one cross circle removed"
    );
    assert_eq!(
        count(&edges_after, "Ellipse"),
        count(&edges_before, "Ellipse") - 1,
        "{what}: one cross ellipse removed"
    );
    assert_eq!(
        count(&edges_after, "Line"),
        count(&edges_before, "Line") - 1,
        "{what}: two springs become one sharp edge"
    );
    assert_eq!(
        count(&edges_after, "Nurbs"),
        count(&edges_before, "Nurbs"),
        "{what}: NURBS contacts untouched"
    );

    // Face evolution is total: band deleted, every survivor modified.
    assert!(
        result.evolution.deleted.contains(&band.index()),
        "{what}: band deleted"
    );
    assert_eq!(
        result.evolution.modified.len(),
        faces_before - 1,
        "{what}: total surviving face history"
    );

    // STEP round-trip preserves validation, census, and volume.
    let step = remus_io::step::write_step(&topo, &[result.solid]).unwrap();
    let mut reread = Topology::new();
    let solid2 = remus_io::step::read_step(&step, &mut reread).unwrap()[0];
    assert_valid(&reread, solid2, &format!("{what} round-trip"));
    assert_eq!(
        remus_topology::explorer::solid_faces(&reread, solid2)
            .unwrap()
            .len(),
        faces_before - 1,
        "{what} round-trip faces"
    );
    let volume_reread = remus_operations::measure::solid_volume(&reread, solid2, 0.02).unwrap();
    assert!(
        (volume_reread - volume_after).abs() < 1e-6 * volume_after.abs().max(1.0),
        "{what} round-trip volume"
    );

    // Failure rollback: a wrong radius witness refuses without mutation.
    let faces_still = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let error =
        remus_operations::resize_blend::resize_blend(&mut topo, solid, band, 2.0, 0.0).unwrap_err();
    assert!(
        format!("{error}").contains("expected radius"),
        "{what}: witness refusal names the radius"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .len(),
        faces_still,
        "{what}: refusal preserves the input"
    );
    assert!(
        (remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap() - volume_before)
            .abs()
            < 1e-12,
        "{what}: refusal preserves volume"
    );
    (topo, result.solid, volume_after)
}

#[test]
fn planar_r1_lower_minus_y_removes_exactly() {
    // STEP #1293 analogue: all-planar R1, centroid z ≈ 65.4, y < 0.
    heal_planar_r1(-1.0, "towel-R1-lower-minus-y");
}

#[test]
fn planar_r1_lower_plus_y_removes_exactly() {
    // STEP #1318 analogue: all-planar R1, centroid z ≈ 65.4, y > 0.
    heal_planar_r1(1.0, "towel-R1-lower-plus-y");
}

#[test]
fn curved_end_r1_pair_refuses_precisely() {
    // STEP #1281/#1306 analogues: 5-edge R1s ending on the R8 cylinder
    // (centroids z ≈ 83.34). Exact removal needs a curved-carrier sharp
    // termination, which is unavailable, so both calls must refuse with the
    // precise missing-construction reason and leave the body untouched.
    for y_sign in [-1.0, 1.0] {
        let what = format!("towel-R1-upper-{y_sign:+}");
        let (mut topo, solid) = load_fixture();
        let band = select_r1(&topo, solid, 5, y_sign, &what);
        let faces_before = remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .len();
        let volume_before = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();
        let error = remus_operations::resize_blend::resize_blend(&mut topo, solid, band, 1.0, 0.0)
            .unwrap_err();
        assert_eq!(
            remus_operations::resize_blend::resize_blend_failure_code(&error),
            "resize-blend-failed",
            "{what}: stable failure code"
        );
        assert!(
            format!("{error}").contains("curved-carrier sharp termination"),
            "{what}: precise missing construction, got {error}"
        );
        assert_eq!(
            remus_topology::explorer::solid_faces(&topo, solid)
                .unwrap()
                .len(),
            faces_before,
            "{what}: refusal preserves faces"
        );
        assert!(
            (remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap() - volume_before)
                .abs()
                < 1e-12,
            "{what}: refusal preserves volume"
        );
        // The public defeature path refuses with the same typed cause.
        let error2 = remus_operations::defeature::defeature(&mut topo, solid, &[band]).unwrap_err();
        assert!(
            matches!(
                error2,
                remus_operations::OperationsError::Unsupported { .. }
            ),
            "{what}: defeature refusal is typed, got {error2:?}"
        );
    }
}

#[test]
fn sequential_shared_support_pair_removes_exactly() {
    // The two planar R1s share support plane33. Removing the second from the
    // healed body proves sequential edits compose on extended supports.
    let (mut topo, healed_once, _) = heal_planar_r1(-1.0, "towel-R1-sequential-first");
    let band = select_r1(&topo, healed_once, 4, 1.0, "towel-R1-sequential-second");
    let volume_once = remus_operations::measure::solid_volume(&topo, healed_once, 0.02).unwrap();
    let (expected_second, _, _, _) =
        measured_bead_oracle(&topo, healed_once, band, "towel-R1-sequential-second");
    let result =
        remus_operations::resize_blend::resize_blend(&mut topo, healed_once, band, 1.0, 0.0)
            .unwrap();
    assert_valid(&topo, result.solid, "towel-R1-sequential-second");
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .len(),
        44,
        "sequential: two bands removed"
    );
    let volume_twice = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    // The second loss matches the analytic oracle recomputed on the healed
    // body (extended supports keep their planes, so the oracle is unchanged).
    let loss = volume_once - volume_twice;
    assert!(
        (loss - expected_second).abs() < 1e-4,
        "sequential: second loss {loss:.6} differs from oracle {expected_second:.6}"
    );
    let tol = Tolerance::new();
    let remaining = remus_topology::explorer::solid_faces(&topo, result.solid)
        .unwrap()
        .into_iter()
        .filter(|face| {
            matches!(
                topo.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder) if tol.approx_eq(cylinder.radius(), 1.0)
            )
        })
        .count();
    assert_eq!(remaining, 2, "sequential: only the R8 pair remains");
}

#[test]
fn journaled_planar_r1_removal_records_total_history() {
    // The journaled path demands total edge/vertex history; it passes only
    // because the surgical healer maps every boundary explicitly.
    let (mut topo, solid) = load_fixture();
    let band = select_r1(&topo, solid, 4, -1.0, "towel-R1-journaled");
    let result =
        remus_operations::journal_ops::resize_blend_journaled(&mut topo, solid, band, 1.0, 0.0)
            .unwrap();
    assert_valid(&topo, result.solid, "towel-R1-journaled");
    let volume = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    let (expected_loss, _, _, _) =
        measured_bead_oracle(&topo, solid, band, "towel-R1-journaled-oracle");
    let volume_before = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();
    assert!(
        ((volume_before - volume) - expected_loss).abs() < 1e-4,
        "journaled: exact healed loss {loss:.6}",
        loss = volume_before - volume
    );
    assert!(
        result.map.deleted.contains(&band.index()),
        "journaled: band deleted"
    );
}
