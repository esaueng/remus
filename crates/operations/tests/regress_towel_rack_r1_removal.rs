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
//! faces (23 planes, 15 cylinders, 4 spheres, 2 tori, 2 cones), valid. Four
//! R1 (r = 1 mm) cylinders, axis +X, surface origin x = 43: two with
//! all-planar neighborhoods (STEP #1293/#1318, wire 4-cycles, centroids
//! z ≈ 65.4) removed by sharp-triple restoration, and two ending on the R8
//! cylinder (STEP #1281/#1306, 5-edge wires with a NURBS R8 contact,
//! centroids z ≈ 83.34) removed by transverse-cylinder termination (sharp
//! edge to the R8 piercing P2 plus an analytic circle arc P2→Q* on the
//! support/R8 intersection). STEP entity numbers are investigation hints
//! only; every selection below is geometric (radius, axis, wire structure,
//! centroid).
//!
//! Every success test asserts exactness (analytic volume oracle, sharp-edge
//! position, sibling placement, carrier census, edge-count deltas) or fails
//! loudly. Tessellation success is never used as proof.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::mat::Mat4;
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

/// Split a band's neighbors into tangent planar supports (line contacts),
/// planar end faces (arc contacts), and curved end faces. Requires exactly
/// two supports; end composition varies by strip class.
fn supports_and_ends(
    topo: &Topology,
    solid: SolidId,
    band: FaceId,
) -> (Vec<FaceId>, Vec<FaceId>, Vec<FaceId>) {
    let adjacency = topo.build_adjacency(solid).unwrap();
    let band_data = topo.face(band).unwrap();
    let mut supports = Vec::new();
    let mut planar_ends = Vec::new();
    let mut curved_ends = Vec::new();
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
        if matches!(edge.curve(), EdgeCurve::Line) {
            assert!(
                topo.face(neighbor).unwrap().surface().is_planar(),
                "line-contact neighbor {} must be a planar support",
                neighbor.index()
            );
            if !supports.contains(&neighbor) {
                supports.push(neighbor);
            }
        } else if topo.face(neighbor).unwrap().surface().is_planar() {
            if !planar_ends.contains(&neighbor) {
                planar_ends.push(neighbor);
            }
        } else if !curved_ends.contains(&neighbor) {
            curved_ends.push(neighbor);
        }
    }
    assert_eq!(supports.len(), 2, "two tangent planar supports");
    (supports, planar_ends, curved_ends)
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
    let (supports, ends, curved) = supports_and_ends(topo, solid, band);
    assert!(
        curved.is_empty() && ends.len() == 2,
        "{what}: planar oracle needs two planar ends"
    );
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
    assert_manifold_exact(&topo, result.solid, what);
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
    // The input solid is untouched by heals, so its pre-heal snapshots apply.
    let census_still = surface_census(&topo, solid);
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
        faces_before,
        "{what}: refusal preserves faces"
    );
    assert_eq!(
        surface_census(&topo, solid),
        census_still,
        "{what}: refusal preserves carriers"
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

// ---------------------------------------------------------------------------
// R8 compound-end removal (STEP #1281/#1318... precisely: #1281/#1306).
// ---------------------------------------------------------------------------

/// Assert closed manifold topology with exact local structure: every edge
/// borders exactly two faces, every vertex sees as many faces as edges
/// (disk link), no face is degenerate, and every wire is a consistently
/// oriented closed walk.
fn assert_manifold_exact(topo: &Topology, solid: SolidId, what: &str) {
    let adjacency = topo.build_adjacency(solid).unwrap();
    for edge in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        assert_eq!(
            adjacency.faces_for_edge(edge).len(),
            2,
            "{what}: edge {} is not 2-manifold",
            edge.index()
        );
    }
    let mut faces_per_vertex: std::collections::HashMap<usize, std::collections::HashSet<usize>> =
        std::collections::HashMap::new();
    let mut edges_per_vertex: std::collections::HashMap<usize, std::collections::HashSet<usize>> =
        std::collections::HashMap::new();
    for face in remus_topology::explorer::solid_faces(topo, solid).unwrap() {
        let face_data = topo.face(face).unwrap();
        for wire_id in
            std::iter::once(face_data.outer_wire()).chain(face_data.inner_wires().iter().copied())
        {
            let wire = topo.wire(wire_id).unwrap();
            let sequence = wire.edges();
            assert!(!sequence.is_empty(), "{what}: empty wire");
            for (i, oriented) in sequence.iter().enumerate() {
                let edge_data = topo.edge(oriented.edge()).unwrap();
                let next = &sequence[(i + 1) % sequence.len()];
                let next_data = topo.edge(next.edge()).unwrap();
                assert_eq!(
                    oriented.oriented_end(edge_data),
                    next.oriented_start(next_data),
                    "{what}: wire {} is not a closed oriented walk",
                    wire_id.index()
                );
                for vertex in [edge_data.start(), edge_data.end()] {
                    faces_per_vertex
                        .entry(vertex.index())
                        .or_default()
                        .insert(face.index());
                    edges_per_vertex
                        .entry(vertex.index())
                        .or_default()
                        .insert(oriented.edge().index());
                }
            }
        }
        let area = remus_operations::measure::face_area(topo, face, 0.02).unwrap();
        assert!(
            area > 1e-9,
            "{what}: face {} has degenerate area {area:.3e}",
            face.index()
        );
    }
    for (vertex, faces) in &faces_per_vertex {
        let edges = edges_per_vertex.get(vertex).cloned().unwrap_or_default();
        assert!(
            !faces.is_empty() && faces.len() == edges.len(),
            "{what}: vertex {vertex} sees {} faces but {} edges (non-disk link)",
            faces.len(),
            edges.len()
        );
    }
}

/// Heal one R8-ending R1 and prove every exactness claim. Returns the healed
/// topology plus the measured loss for mirror-differential checks.
fn heal_r8_r1(y_sign: f64, what: &str) -> (Topology, SolidId, f64) {
    let tol = Tolerance::new();
    let (mut topo, solid) = load_fixture();
    let band = select_r1(&topo, solid, 5, y_sign, what);
    let (supports, planar_ends, curved_ends) = supports_and_ends(&topo, solid, band);
    assert_eq!(planar_ends.len(), 2, "{what}: west plus oblique caps");
    assert_eq!(curved_ends.len(), 1, "{what}: one R8 end face");
    let FaceSurface::Cylinder(r8c) = topo.face(curved_ends[0]).unwrap().surface().clone() else {
        unreachable!("curved end must be the R8 cylinder");
    };
    let (r8_radius, r8_origin, r8_axis) = (r8c.radius(), r8c.origin(), r8c.axis());
    assert!(
        tol.approx_eq(r8_radius, 8.0),
        "{what}: R8 radius {r8_radius}",
    );
    let corners = derive_r8_corners(
        &topo,
        solid,
        band,
        &supports,
        &planar_ends,
        curved_ends[0],
        what,
    );
    // Prism oracle from the west cap and oblique cap with the band frame;
    // the true loss sits below it by the compound end shape (~0.1 here).
    let (n0, d0) = unit_plane(&topo, supports[0]);
    let (n1, d1) = unit_plane(&topo, supports[1]);
    let (westface, west_triple) = {
        let west = planar_ends
            .iter()
            .copied()
            .find(|face| *face != corners.obl)
            .expect("west cap");
        let plane = unit_plane(&topo, west);
        (
            west,
            triple((n0, d0), (n1, d1), plane).expect("west triple"),
        )
    };
    let _ = westface;
    let FaceSurface::Cylinder(band_cyl) = topo.face(band).unwrap().surface().clone() else {
        unreachable!("R1 selector guarantees a cylinder");
    };
    let (obl_n, obl_d) = unit_plane(&topo, corners.obl);
    // Corner coordinates from the axis supports (both axis-aligned here).
    let y0 = if n0.y().abs() > 0.5 {
        d0 / n0.y()
    } else {
        d1 / n1.y()
    };
    let z0 = if n0.z().abs() > 0.5 {
        d0 / n0.z()
    } else {
        d1 / n1.z()
    };
    let prism = bead_volume_oracle(
        y0,
        z0,
        band_cyl.origin().y(),
        band_cyl.origin().z(),
        1.0,
        west_triple.0,
        obl_n.x(),
        obl_n.z(),
        obl_d,
        what,
    );
    let faces_before = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let volume_before = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();

    let result =
        remus_operations::resize_blend::resize_blend(&mut topo, solid, band, 1.0, 0.0).unwrap();
    assert_valid(&topo, result.solid, what);
    assert_manifold_exact(&topo, result.solid, what);
    let faces = remus_topology::explorer::solid_faces(&topo, result.solid).unwrap();
    assert_eq!(faces.len(), faces_before - 1, "{what}: only the band goes");
    let volume_after = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    let loss = volume_before - volume_after;
    // Scaled end-shape band: the native ~0.1 gap scales with volume.
    assert!(
        loss > prism - 1.0 && loss < prism + 1e-3,
        "synthetic: scaled loss {loss:.6} inside scaled prism {prism:.6}"
    );
    assert!(
        prism - loss < 0.5,
        "{what}: end-shape gap {gap:.6} bounded",
        gap = prism - loss
    );
    // Deflection invariance proves the delta is measured exactly, so any
    // residual is geometry, not tessellation noise.
    for deflection in [0.05, 0.02] {
        let a = remus_operations::measure::solid_volume(&topo, solid, deflection).unwrap();
        let b = remus_operations::measure::solid_volume(&topo, result.solid, deflection).unwrap();
        assert!(
            ((a - b) - loss).abs() < 1e-9,
            "{what}: loss deflection-invariant at {deflection}"
        );
    }

    // Sharp edge: line shared by the healed supports, endpoints at W and P2.
    let adjacency = topo.build_adjacency(result.solid).unwrap();
    let healed = |source: FaceId| {
        result.evolution.modified.iter().find_map(|(&from, to)| {
            (from == source.index()).then(|| {
                let [one] = to.as_slice() else {
                    panic!("{what}: one-to-one face evolution");
                };
                topo.face_id_from_index(*one).unwrap()
            })
        })
    };
    let healed_supports: Vec<FaceId> = supports
        .iter()
        .map(|face| healed(*face).expect("support evolves"))
        .collect();
    let healed_r8 = healed(curved_ends[0]).expect("R8 evolves");
    assert_eq!(
        corners.r8, curved_ends[0],
        "{what}: derived R8 is the curved end"
    );
    let healed_sy = healed(corners.sy).expect("Sy evolves");
    let healed_sz = healed(corners.sz).expect("Sz evolves");
    let healed_obl = healed(corners.obl).expect("oblique cap evolves");
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
        let at = |p: (f64, f64, f64), q: (f64, f64, f64)| {
            ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2) + (p.2 - q.2).powi(2)).sqrt() < 1e-6
        };
        let a3 = (a.x(), a.y(), a.z());
        let b3 = (b.x(), b.y(), b.z());
        if (at(a3, corners.west) && at(b3, corners.p2))
            || (at(a3, corners.p2) && at(b3, corners.west))
        {
            sharp_found = true;
        }
    }
    assert!(sharp_found, "{what}: sharp edge W to P2");

    // R8 boundary arc: circle shared by healed R8 and healed Sy with the
    // analytic section parameters, certified trim, and P2/Q* endpoints.
    let mut arcs = Vec::new();
    for edge in remus_topology::explorer::solid_edges(&topo, result.solid).unwrap() {
        let incident = adjacency.faces_for_edge(edge);
        if incident.len() != 2 || !incident.contains(&healed_r8) || !incident.contains(&healed_sy) {
            continue;
        }
        if let EdgeCurve::Circle(circle) = topo.edge(edge).unwrap().curve() {
            arcs.push((edge, circle.clone()));
        }
    }
    assert_eq!(arcs.len(), 1, "{what}: one R8 boundary arc");
    let (arc_edge, arc) = &arcs[0];
    assert!(
        (arc.radius() - r8_radius).abs() < 1e-12,
        "{what}: arc radius"
    );
    let radial_axis = {
        let axis = r8_axis;
        let offset = arc.center() - r8_origin;
        (offset - axis * offset.dot(axis)).length()
    };
    assert!(radial_axis < 1e-9, "{what}: arc center on R8 axis");
    let Some((trim_start, trim_end)) = topo.edge(*arc_edge).unwrap().trim() else {
        panic!("{what}: arc has certified trim");
    };
    assert!(
        (trim_end - trim_start).abs() < std::f64::consts::PI,
        "{what}: minor arc"
    );
    // Trim endpoints must be exactly {P2, Q*} as a set: each evaluated end
    // matches at least one corner, and both corners are covered.
    let mut matched = [false, false];
    for evaluated in [arc.evaluate(trim_start), arc.evaluate(trim_end)] {
        let mut hit = false;
        for (i, want) in [corners.p2, corners.qstar].iter().enumerate() {
            let gap = ((evaluated.x() - want.0).powi(2)
                + (evaluated.y() - want.1).powi(2)
                + (evaluated.z() - want.2).powi(2))
            .sqrt();
            if gap < 1e-9 {
                matched[i] = true;
                hit = true;
            }
        }
        assert!(hit, "{what}: arc trim endpoint on a corner");
    }
    assert!(
        matched[0] && matched[1],
        "{what}: arc trim covers P2 and Q*"
    );

    // Re-anchored generatrices terminate exactly at P2 and Q*.
    // E_z extension: line shared by healed R8 and healed Sz, one endpoint at
    // P2, the other at the pre-op far endpoint.
    let mut ez_found = false;
    let mut eo_found = false;
    for edge in remus_topology::explorer::solid_edges(&topo, result.solid).unwrap() {
        let incident = adjacency.faces_for_edge(edge);
        if incident.len() != 2 || !matches!(topo.edge(edge).unwrap().curve(), EdgeCurve::Line) {
            continue;
        }
        let edge_data = topo.edge(edge).unwrap();
        let a = topo.vertex(edge_data.start()).unwrap().point();
        let b = topo.vertex(edge_data.end()).unwrap().point();
        let near = |p: remus_math::vec::Point3, want: (f64, f64, f64)| {
            ((p.x() - want.0).powi(2) + (p.y() - want.1).powi(2) + (p.z() - want.2).powi(2)).sqrt()
                < 1e-6
        };
        if incident.contains(&healed_r8) && incident.contains(&healed_sz) {
            // One end at P2, the other at the pre-op far end of E_z.
            let far = corners.ez_far;
            if (near(a, corners.p2) && near(b, far)) || (near(b, corners.p2) && near(a, far)) {
                ez_found = true;
            }
        }
        if incident.contains(&healed_r8) && incident.contains(&healed_obl) {
            let far = corners.eo_far;
            if (near(a, corners.qstar) && near(b, far)) || (near(b, corners.qstar) && near(a, far))
            {
                eo_found = true;
            }
        }
    }
    assert!(ez_found, "{what}: E_z extended exactly to P2");
    assert!(eo_found, "{what}: E_o extended exactly to Q*");

    // Sibling R1s bit-exact; R8 carrier bit-exact; carrier census exact.
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
    for ((centroid, radius), baseline) in surviving.iter().zip(before.iter()) {
        assert!((radius - 1.0).abs() < 1e-9, "{what}: sibling radius");
        assert!(
            dist2(*centroid, *baseline) < 1e-12,
            "{what}: sibling placement"
        );
    }
    let healed_r8_surface = topo.face(healed_r8).unwrap().surface().clone();
    match &healed_r8_surface {
        FaceSurface::Cylinder(after) => {
            assert_eq!(
                after.radius().to_bits(),
                r8_radius.to_bits(),
                "{what}: R8 radius bit-exact"
            );
            assert_eq!(after.origin(), r8_origin, "{what}: R8 origin bit-exact");
            assert_eq!(after.axis(), r8_axis, "{what}: R8 axis bit-exact");
        }
        _ => panic!("{what}: R8 stays a cylinder"),
    }
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
    // Exact topology deltas: springs vanish into one sharp line, the west
    // circle is replaced by the R8 arc (net zero circles), the oblique arc
    // and NURBS contact disappear.
    let edges_after = edge_census(&topo, result.solid);
    let edges_before = edge_census(&topo, solid);
    let count = |census: &[(String, usize)], tag: &str| {
        census.iter().find(|(t, _)| t == tag).map_or(0, |(_, n)| *n)
    };
    assert_eq!(
        count(&edges_after, "Circle"),
        count(&edges_before, "Circle"),
        "{what}: west circle replaced by R8 arc"
    );
    assert_eq!(
        count(&edges_after, "Ellipse"),
        count(&edges_before, "Ellipse") - 1,
        "{what}: oblique arc removed"
    );
    assert_eq!(
        count(&edges_after, "Line"),
        count(&edges_before, "Line") - 1,
        "{what}: two springs become one sharp edge"
    );
    assert_eq!(
        count(&edges_after, "Nurbs"),
        count(&edges_before, "Nurbs") - 1,
        "{what}: NURBS contact removed"
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
    (topo, result.solid, loss)
}

/// Analytic R8-end corners derived from the imported topology: the west
/// triple W, the sharp/R8-generatrix piercing P2, and the
/// generatrix/support crossing Q*. All inputs measured; no coordinates.
struct R8Corners {
    west: (f64, f64, f64),
    p2: (f64, f64, f64),
    qstar: (f64, f64, f64),
    ez_far: (f64, f64, f64),
    eo_far: (f64, f64, f64),
    sy: FaceId,
    sz: FaceId,
    obl: FaceId,
    r8: FaceId,
}

fn derive_r8_corners(
    topo: &Topology,
    solid: SolidId,
    band: FaceId,
    supports: &[FaceId],
    planar_ends: &[FaceId],
    r8: FaceId,
    what: &str,
) -> R8Corners {
    assert_eq!(planar_ends.len(), 2, "{what}: west cap plus oblique cap");
    // West cap: axis-aligned perpendicular plane; oblique cap contains Y.
    let mut west = None;
    let mut obl = None;
    for &end in planar_ends {
        let (n, d) = unit_plane(topo, end);
        if n.y().abs() < 1e-12 && n.z().abs() < 1e-12 {
            assert!(west.is_none(), "{what}: one west cap");
            west = Some((end, d / n.x()));
        } else {
            assert!(n.y().abs() < 1e-12, "{what}: oblique cap contains Y");
            assert!(obl.is_none(), "{what}: one oblique cap");
            obl = Some(end);
        }
    }
    let (westface, _) = west.expect("west cap");
    let oblface = obl.expect("oblique cap");
    let (n0, d0) = unit_plane(topo, supports[0]);
    let (n1, d1) = unit_plane(topo, supports[1]);
    let west_triple = triple((n0, d0), (n1, d1), unit_plane(topo, westface)).expect("west triple");
    // E_z: the R8/support generatrix touching a band vertex; P2 where the
    // sharp line meets it, verified on R8.
    let band_vertices: std::collections::HashSet<usize> = {
        let mut set = std::collections::HashSet::new();
        let outer = topo.face(band).unwrap().outer_wire();
        for oriented in topo.wire(outer).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            set.insert(edge.start().index());
            set.insert(edge.end().index());
        }
        set
    };
    let adjacency = topo.build_adjacency(solid).unwrap();
    let mut ez_pick: Option<remus_topology::edge::EdgeId> = None;
    let mut sz_pick: Option<FaceId> = None;
    for edge in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        let data = topo.edge(edge).unwrap();
        if !matches!(data.curve(), EdgeCurve::Line) {
            continue;
        }
        let mut adjacent: Vec<FaceId> = adjacency.faces_for_edge(edge).to_vec();
        adjacent.sort_unstable_by_key(|face| face.index());
        let touches_band = [data.start().index(), data.end().index()]
            .iter()
            .any(|index| band_vertices.contains(index));
        if !touches_band {
            continue;
        }
        let has_r8 = adjacent.contains(&r8);
        let support = supports
            .iter()
            .copied()
            .find(|face| adjacent.contains(face));
        if has_r8 && support.is_some() {
            assert!(ez_pick.is_none(), "{what}: unique E_z generatrix");
            ez_pick = Some(edge);
            sz_pick = support;
        }
    }
    let (ez, sz) = (
        ez_pick.expect("E_z generatrix"),
        sz_pick.expect("E_z support"),
    );
    let ez_far = {
        let ez_data = topo.edge(ez).unwrap();
        let far = [ez_data.start(), ez_data.end()]
            .into_iter()
            .find(|vertex| !band_vertices.contains(&vertex.index()))
            .expect("E_z far endpoint");
        let point = topo.vertex(far).unwrap().point();
        (point.x(), point.y(), point.z())
    };
    let sy = supports
        .iter()
        .copied()
        .find(|face| *face != sz)
        .expect("distinct supports");
    // Sharp line from the two support planes through the west triple.
    let sharp_direction = {
        let (a_n, _) = unit_plane(topo, supports[0]);
        let (b_n, _) = unit_plane(topo, supports[1]);
        a_n.cross(b_n).normalize().unwrap()
    };
    let ez_data = topo.edge(ez).unwrap();
    let ez_a = topo.vertex(ez_data.start()).unwrap().point();
    let ez_b = topo.vertex(ez_data.end()).unwrap().point();
    let west_point = remus_math::vec::Point3::new(west_triple.0, west_triple.1, west_triple.2);
    // Line-line intersection inside the support plane (both lines lie in it).
    let ez_direction = (ez_b - ez_a).normalize().unwrap();
    let cross = sharp_direction.cross(ez_direction);
    assert!(
        cross.length() > 1e-9,
        "{what}: sharp meets E_z transversely"
    );
    let difference = ez_a - west_point;
    let t = difference.cross(ez_direction).dot(cross) / cross.dot(cross);
    let p2 = west_point + sharp_direction * t;
    // Verify P2 back on both lines and on R8.
    let on_ez = ((p2 - ez_a).cross(ez_direction)).length();
    assert!(on_ez < 1e-9, "{what}: P2 on E_z, gap {on_ez:.3e}");
    let FaceSurface::Cylinder(r8c) = topo.face(r8).unwrap().surface().clone() else {
        unreachable!("R8 selector guarantees a cylinder");
    };
    let r8_axis = r8c.axis().normalize().unwrap();
    let radial = (p2 - r8c.origin()) - r8_axis * (p2 - r8c.origin()).dot(r8_axis);
    assert!(
        (radial.length() - r8c.radius()).abs() < 1e-9,
        "{what}: P2 on R8"
    );
    // Q*: E_o (R8/OBL generatrix touching a band vertex) meets Sy.
    let mut eo_pick: Option<remus_topology::edge::EdgeId> = None;
    for edge in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        let data = topo.edge(edge).unwrap();
        if !matches!(data.curve(), EdgeCurve::Line) {
            continue;
        }
        let mut adjacent: Vec<FaceId> = adjacency.faces_for_edge(edge).to_vec();
        adjacent.sort_unstable_by_key(|face| face.index());
        if !adjacent.contains(&r8) || !adjacent.contains(&oblface) {
            continue;
        }
        if [data.start().index(), data.end().index()]
            .iter()
            .any(|index| band_vertices.contains(index))
        {
            assert!(eo_pick.is_none(), "{what}: unique E_o generatrix");
            eo_pick = Some(edge);
        }
    }
    let eo = eo_pick.expect("E_o generatrix");
    let eo_data = topo.edge(eo).unwrap();
    let eo_far = {
        let far = [eo_data.start(), eo_data.end()]
            .into_iter()
            .find(|vertex| !band_vertices.contains(&vertex.index()))
            .expect("E_o far endpoint");
        let point = topo.vertex(far).unwrap().point();
        (point.x(), point.y(), point.z())
    };
    let (sy_n, sy_d) = unit_plane(topo, sy);
    let eo_a = topo.vertex(eo_data.start()).unwrap().point();
    let eo_b = topo.vertex(eo_data.end()).unwrap().point();
    let eo_direction = (eo_b - eo_a).normalize().unwrap();
    assert!(
        eo_direction.dot(sy_n).abs() > 1e-6,
        "{what}: E_o transverse to Sy"
    );
    let eo_origin = Vec3::new(eo_a.x(), eo_a.y(), eo_a.z());
    let qstar = eo_a + eo_direction * ((sy_d - eo_origin.dot(sy_n)) / eo_direction.dot(sy_n));
    let qradial = (qstar - r8c.origin()) - r8_axis * (qstar - r8c.origin()).dot(r8_axis);
    assert!(
        (qradial.length() - r8c.radius()).abs() < 1e-7,
        "{what}: Q* on R8"
    );
    let qstar_vec = Vec3::new(qstar.x(), qstar.y(), qstar.z());
    let (obl_n, obl_d) = unit_plane(topo, oblface);
    assert!(
        (sy_n.dot(qstar_vec) - sy_d).abs() < 1e-9 && (obl_n.dot(qstar_vec) - obl_d).abs() < 1e-7,
        "{what}: Q* on Sy and oblique face"
    );
    assert!(
        ((qstar.x() - p2.x()).powi(2)
            + (qstar.y() - p2.y()).powi(2)
            + (qstar.z() - p2.z()).powi(2))
        .sqrt()
            > 1e-7,
        "{what}: P2 and Q* distinct"
    );
    R8Corners {
        west: west_triple,
        p2: (p2.x(), p2.y(), p2.z()),
        qstar: (qstar.x(), qstar.y(), qstar.z()),
        ez_far,
        eo_far,
        sy,
        sz,
        obl: oblface,
        r8,
    }
}

/// Sharp edge plus R8 arc existence with analytic parameters, resolved
/// through a caller-provided face evolution map. Used by the lighter
/// (defeature-direct, journaled) R8 tests so they pin the replacement
/// boundaries, not just counts and validity.
#[allow(clippy::too_many_arguments)]
fn assert_r8_replacement_boundaries(
    topo: &Topology,
    solid: SolidId,
    corners: &R8Corners,
    supports: &[FaceId],
    r8_source: FaceId,
    evolution: &remus_operations::evolution::EvolutionMap,
    r8_radius: f64,
    what: &str,
) {
    let healed = |source: FaceId| {
        evolution.modified.iter().find_map(|(&from, to)| {
            (from == source.index()).then(|| {
                let [one] = to.as_slice() else {
                    panic!("{what}: one-to-one face evolution");
                };
                topo.face_id_from_index(*one).unwrap()
            })
        })
    };
    let healed_supports: Vec<FaceId> = supports
        .iter()
        .map(|face| healed(*face).expect("support evolves"))
        .collect();
    let healed_r8 = healed(r8_source).expect("R8 evolves");
    let healed_sy = healed(corners.sy).expect("Sy evolves");
    let adjacency = topo.build_adjacency(solid).unwrap();
    let near = |p: remus_math::vec::Point3, want: (f64, f64, f64)| {
        ((p.x() - want.0).powi(2) + (p.y() - want.1).powi(2) + (p.z() - want.2).powi(2)).sqrt()
            < 1e-6
    };
    let mut sharp_found = false;
    let mut arcs = Vec::new();
    for edge in remus_topology::explorer::solid_edges(topo, solid).unwrap() {
        let incident = adjacency.faces_for_edge(edge);
        if incident.len() != 2 {
            continue;
        }
        let edge_data = topo.edge(edge).unwrap();
        let a = topo.vertex(edge_data.start()).unwrap().point();
        let b = topo.vertex(edge_data.end()).unwrap().point();
        if incident.iter().all(|face| healed_supports.contains(face))
            && matches!(edge_data.curve(), EdgeCurve::Line)
            && ((near(a, corners.west) && near(b, corners.p2))
                || (near(a, corners.p2) && near(b, corners.west)))
        {
            sharp_found = true;
        }
        if incident.contains(&healed_r8)
            && incident.contains(&healed_sy)
            && let EdgeCurve::Circle(circle) = edge_data.curve()
        {
            assert!(
                (circle.radius() - r8_radius).abs() < 1e-12,
                "{what}: arc radius"
            );
            arcs.push(edge);
        }
    }
    assert!(sharp_found, "{what}: sharp edge W to P2");
    assert_eq!(arcs.len(), 1, "{what}: one R8 boundary arc");
}

#[test]
fn r8_end_r1_minus_y_removes_exactly() {
    // STEP #1306 analogue: R8-ending R1, centroid z ≈ 83.34, y < 0.
    heal_r8_r1(-1.0, "towel-R1-upper-minus-y");
}

#[test]
fn r8_end_r1_plus_y_removes_exactly() {
    // STEP #1281 analogue: R8-ending R1, centroid z ≈ 83.34, y > 0.
    heal_r8_r1(1.0, "towel-R1-upper-plus-y");
}

#[test]
fn r8_pair_mirror_losses_agree() {
    // The two R8 strips mirror each other across y = 0; their independently
    // measured losses agree far tighter than either absolute oracle band,
    // proving the reconstruction is symmetric and not fixture-tuned.
    let (_, _, loss_minus) = heal_r8_r1(-1.0, "towel-R1-mirror-minus");
    let (_, _, loss_plus) = heal_r8_r1(1.0, "towel-R1-mirror-plus");
    assert!(
        (loss_minus - loss_plus).abs() < 1e-3,
        "mirror losses {loss_minus:.6} vs {loss_plus:.6} disagree"
    );
}

#[test]
fn sequential_planar_reverse_order_removes_exactly() {
    // Shared support plane33, opposite order from the existing test.
    let (mut topo, healed_once, _) = heal_planar_r1(1.0, "towel-R1-seq-rev-first");
    let band = select_r1(&topo, healed_once, 4, -1.0, "towel-R1-seq-rev-second");
    let volume_once = remus_operations::measure::solid_volume(&topo, healed_once, 0.02).unwrap();
    let (expected_second, _, _, _) =
        measured_bead_oracle(&topo, healed_once, band, "towel-R1-seq-rev-second");
    let result =
        remus_operations::resize_blend::resize_blend(&mut topo, healed_once, band, 1.0, 0.0)
            .unwrap();
    assert_valid(&topo, result.solid, "towel-R1-seq-rev-second");
    assert_manifold_exact(&topo, result.solid, "towel-R1-seq-rev-second");
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .len(),
        44,
        "sequential reverse: two bands removed"
    );
    let volume_twice = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    assert!(
        ((volume_once - volume_twice) - expected_second).abs() < 1e-4,
        "sequential reverse: second loss matches oracle"
    );
}

#[test]
fn sequential_r8_pair_first_order_removes_exactly() {
    // Shared support plane36: remove #1306, then #1281 from the healed body.
    let (mut topo, healed_once, loss_first) = heal_r8_r1(-1.0, "towel-R1-r8seq-first");
    let band = select_r1(&topo, healed_once, 5, 1.0, "towel-R1-r8seq-second");
    let result =
        remus_operations::resize_blend::resize_blend(&mut topo, healed_once, band, 1.0, 0.0)
            .unwrap();
    assert_valid(&topo, result.solid, "towel-R1-r8seq-second");
    assert_manifold_exact(&topo, result.solid, "towel-R1-r8seq-second");
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .len(),
        44,
        "sequential R8: two bands removed"
    );
    let volume_twice = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    let volume_once = remus_operations::measure::solid_volume(&topo, healed_once, 0.02).unwrap();
    let loss_second = volume_once - volume_twice;
    assert!(
        loss_second > 0.0,
        "sequential R8: second removal loses volume"
    );
    assert!(
        (loss_second - loss_first).abs() < 1e-3,
        "sequential R8: mirrored losses {loss_second:.6} vs {loss_first:.6}"
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
    assert_eq!(remaining, 2, "sequential R8: only the planar pair remains");
    // STEP round-trip of the twice-healed body.
    let step = remus_io::step::write_step(&topo, &[result.solid]).unwrap();
    let mut reread = Topology::new();
    let solid2 = remus_io::step::read_step(&step, &mut reread).unwrap()[0];
    assert_valid(&reread, solid2, "towel-R1-r8seq round-trip");
    assert_eq!(
        remus_topology::explorer::solid_faces(&reread, solid2)
            .unwrap()
            .len(),
        44,
        "sequential R8 round-trip faces"
    );
}

#[test]
fn sequential_r8_pair_second_order_removes_exactly() {
    // Opposite order: #1281 first, then #1306.
    let (mut topo, healed_once, loss_first) = heal_r8_r1(1.0, "towel-R1-r8seq2-first");
    let band = select_r1(&topo, healed_once, 5, -1.0, "towel-R1-r8seq2-second");
    let result =
        remus_operations::resize_blend::resize_blend(&mut topo, healed_once, band, 1.0, 0.0)
            .unwrap();
    assert_valid(&topo, result.solid, "towel-R1-r8seq2-second");
    assert_manifold_exact(&topo, result.solid, "towel-R1-r8seq2-second");
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .len(),
        44,
        "sequential R8 reverse: two bands removed"
    );
    let volume_twice = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    let volume_once = remus_operations::measure::solid_volume(&topo, healed_once, 0.02).unwrap();
    assert!(
        ((volume_once - volume_twice) - loss_first).abs() < 1e-3,
        "sequential R8 reverse: mirrored losses agree"
    );
}

#[test]
fn sequential_shared_end_pair_removes_exactly() {
    // Different supports but shared end faces (plane4/plane15): remove a
    // planar R1, then the neighboring R8 R1 from the healed body.
    let (mut topo, healed_once, _) = heal_planar_r1(-1.0, "towel-R1-sharedend-first");
    let band = select_r1(&topo, healed_once, 5, -1.0, "towel-R1-sharedend-second");
    let result =
        remus_operations::resize_blend::resize_blend(&mut topo, healed_once, band, 1.0, 0.0)
            .unwrap();
    assert_valid(&topo, result.solid, "towel-R1-sharedend-second");
    assert_manifold_exact(&topo, result.solid, "towel-R1-sharedend-second");
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .len(),
        44,
        "shared ends: two bands removed"
    );
}

#[test]
fn synthetic_scaled_translated_r8_removal() {
    // Same topology at different dimensions and placement (uniform 1.5x plus
    // translation): selectors, corners, and oracles are all re-derived, so
    // nothing about the construction is specific to the towel coordinates.
    // transform_solid preserves analytic carriers under uniform scale.
    let (mut topo, solid) = load_fixture();
    remus_operations::transform::transform_solid(&mut topo, solid, &Mat4::scale(1.5, 1.5, 1.5))
        .unwrap();
    remus_operations::transform::transform_solid(
        &mut topo,
        solid,
        &Mat4::translation(100.0, 0.0, 25.0),
    )
    .unwrap();
    let step = remus_io::step::write_step(&topo, &[solid]).unwrap();
    let mut topo2 = Topology::new();
    let body = remus_io::step::read_step(&step, &mut topo2).unwrap()[0];
    let tol = Tolerance::new();
    let band = remus_topology::explorer::solid_faces(&topo2, body)
        .unwrap()
        .into_iter()
        .filter(|face| {
            matches!(
                topo2.face(*face).unwrap().surface(),
                FaceSurface::Cylinder(cylinder) if tol.approx_eq(cylinder.radius(), 1.5)
            ) && topo2
                .wire(topo2.face(*face).unwrap().outer_wire())
                .unwrap()
                .edges()
                .len()
                == 5
                && centroid_of(&topo2, *face).1 < 0.0
        })
        .collect::<Vec<_>>();
    assert_eq!(band.len(), 1, "synthetic: one R8-ending R1 at 1.5x");
    let band = band[0];
    let (supports, planar_ends, curved_ends) = supports_and_ends(&topo2, body, band);
    assert_eq!(planar_ends.len(), 2, "synthetic: two planar ends");
    assert_eq!(curved_ends.len(), 1, "synthetic: one R8 end");
    let FaceSurface::Cylinder(r8c) = topo2.face(curved_ends[0]).unwrap().surface().clone() else {
        unreachable!("R8 end must be a cylinder");
    };
    assert!(
        (r8c.radius() - 12.0).abs() < 1e-9,
        "synthetic: scaled R8 radius, got {}",
        r8c.radius()
    );
    let volume_before = remus_operations::measure::solid_volume(&topo2, body, 0.02).unwrap();
    let result =
        remus_operations::resize_blend::resize_blend(&mut topo2, body, band, 1.5, 0.0).unwrap();
    assert_valid(&topo2, result.solid, "synthetic");
    assert_manifold_exact(&topo2, result.solid, "synthetic");
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo2, result.solid)
            .unwrap()
            .len(),
        45,
        "synthetic: only the band goes"
    );
    let volume_after = remus_operations::measure::solid_volume(&topo2, result.solid, 0.02).unwrap();
    let loss = volume_before - volume_after;
    // Cube-scaled prism oracle, re-derived on the transformed body: the
    // native end-shape gap (~0.1) scales with volume, so the band below is
    // the scaled equivalent. This proves exactness at the new scale rather
    // than mere covariance.
    let (n0, d0) = unit_plane(&topo2, supports[0]);
    let (n1, d1) = unit_plane(&topo2, supports[1]);
    let west = planar_ends
        .iter()
        .copied()
        .find(|face| {
            let (n, _) = unit_plane(&topo2, *face);
            n.y().abs() < 1e-12 && n.z().abs() < 1e-12
        })
        .expect("synthetic west cap");
    let west_triple = triple((n0, d0), (n1, d1), unit_plane(&topo2, west)).expect("west triple");
    let obl = planar_ends
        .iter()
        .copied()
        .find(|face| *face != west)
        .expect("synthetic oblique cap");
    let (obl_n, obl_d) = unit_plane(&topo2, obl);
    let FaceSurface::Cylinder(band_cyl) = topo2.face(band).unwrap().surface().clone() else {
        unreachable!("synthetic band is a cylinder");
    };
    let y0 = if n0.y().abs() > 0.5 {
        d0 / n0.y()
    } else {
        d1 / n1.y()
    };
    let z0 = if n0.z().abs() > 0.5 {
        d0 / n0.z()
    } else {
        d1 / n1.z()
    };
    let prism = bead_volume_oracle(
        y0,
        z0,
        band_cyl.origin().y(),
        band_cyl.origin().z(),
        1.5,
        west_triple.0,
        obl_n.x(),
        obl_n.z(),
        obl_d,
        "synthetic",
    );
    assert!(
        loss > prism - 2.0 && loss < prism + 1e-3,
        "synthetic: scaled loss {loss:.6} inside scaled prism {prism:.6}"
    );
    assert!(
        result.evolution.deleted.contains(&band.index()),
        "synthetic: band deleted"
    );
    assert_eq!(
        result.evolution.modified.len(),
        45,
        "synthetic: total surviving face history"
    );
    // STEP round-trip of the scaled heal.
    let step2 = remus_io::step::write_step(&topo2, &[result.solid]).unwrap();
    let mut topo3 = Topology::new();
    let solid3 = remus_io::step::read_step(&step2, &mut topo3).unwrap()[0];
    assert_valid(&topo3, solid3, "synthetic round-trip");
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo3, solid3)
            .unwrap()
            .len(),
        45,
        "synthetic round-trip faces"
    );
}

#[test]
fn r8_end_wrong_witness_rolls_back() {
    // Radius witness mismatch refuses without mutation: faces, census, and
    // volume all preserved, validated afterwards.
    let what = "towel-R1-r8-witness";
    let (mut topo, solid) = load_fixture();
    let band = select_r1(&topo, solid, 5, -1.0, what);
    let faces_before = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let census_before = surface_census(&topo, solid);
    let volume_before = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();
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
        faces_before,
        "{what}: refusal preserves faces"
    );
    assert_eq!(
        surface_census(&topo, solid),
        census_before,
        "{what}: refusal preserves carriers"
    );
    assert!(
        (remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap() - volume_before)
            .abs()
            < 1e-12,
        "{what}: refusal preserves volume"
    );
    assert_valid(&topo, solid, what);
}

#[test]
fn multi_band_selection_refuses() {
    // Selecting two complete bands at once is not one blend region.
    let what = "towel-R1-multiband";
    let (mut topo, solid) = load_fixture();
    let first = select_r1(&topo, solid, 4, -1.0, what);
    let second = select_r1(&topo, solid, 4, 1.0, what);
    let faces_before = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let error =
        remus_operations::defeature::defeature(&mut topo, solid, &[first, second]).unwrap_err();
    assert!(
        format!("{error}").contains("complete analytic blend band"),
        "{what}: multi-band refusal names the rule, got {error}"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .len(),
        faces_before,
        "{what}: refusal preserves faces"
    );
}

#[test]
fn defeature_direct_r8_heals() {
    // The public defeature path routes cylindrical bands through the same
    // surgical reconstruction; the replacement boundaries are checked, not
    // just counts and validity.
    let what = "towel-R1-defeature-direct";
    let (mut topo, solid) = load_fixture();
    let band = select_r1(&topo, solid, 5, -1.0, what);
    let (supports, planar_ends, curved_ends) = supports_and_ends(&topo, solid, band);
    assert_eq!(curved_ends.len(), 1, "{what}: one R8 end face");
    let corners = derive_r8_corners(
        &topo,
        solid,
        band,
        &supports,
        &planar_ends,
        curved_ends[0],
        what,
    );
    let (healed, evolution) =
        remus_operations::defeature::defeature_with_evolution(&mut topo, solid, &[band]).unwrap();
    assert_valid(&topo, healed, what);
    assert_manifold_exact(&topo, healed, what);
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, healed)
            .unwrap()
            .len(),
        45,
        "{what}: only the band goes"
    );
    assert_r8_replacement_boundaries(
        &topo,
        healed,
        &corners,
        &supports,
        curved_ends[0],
        &evolution,
        8.0,
        what,
    );
}

#[test]
fn journaled_r8_removal_records_total_history() {
    // The journaled path demands total edge/vertex history including the new
    // sharp edge and R8 arc; it passes only because the surgical healer maps
    // every boundary explicitly.
    let what = "towel-R1-r8-journaled";
    let (mut topo, solid) = load_fixture();
    let band = select_r1(&topo, solid, 5, -1.0, what);
    let (supports, planar_ends, curved_ends) = supports_and_ends(&topo, solid, band);
    assert_eq!(curved_ends.len(), 1, "{what}: one R8 end face");
    let corners = derive_r8_corners(
        &topo,
        solid,
        band,
        &supports,
        &planar_ends,
        curved_ends[0],
        what,
    );
    let volume_before = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();
    let result =
        remus_operations::journal_ops::resize_blend_journaled(&mut topo, solid, band, 1.0, 0.0)
            .unwrap();
    assert_valid(&topo, result.solid, what);
    assert_manifold_exact(&topo, result.solid, what);
    assert_r8_replacement_boundaries(
        &topo,
        result.solid,
        &corners,
        &supports,
        curved_ends[0],
        &result.map,
        8.0,
        what,
    );
    let volume = remus_operations::measure::solid_volume(&topo, result.solid, 0.02).unwrap();
    let loss = volume_before - volume;
    assert!(
        loss > 0.0,
        "{what}: journaled removal loses bead volume, got {loss:.6}"
    );
    assert!(
        result.map.deleted.contains(&band.index()),
        "journaled: band deleted"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, result.solid)
            .unwrap()
            .len(),
        45,
        "journaled: only the band goes"
    );
}

#[test]
fn import_carries_no_pcurves_premise() {
    // The towel STEP file carries no per-use pcurves (measured), so
    // per-use pcurve preservation is vacuous here: the surgical in-place
    // edits structurally preserve registry entries by reusing FaceIds and
    // EdgeIds (as E13-style copy mechanics prove for copies), and new edges
    // follow the same no-pcurve policy as every other heal path. This test
    // pins the premise so a future fixture with pcurves cannot silently pass
    // without them.
    let (topo, _) = load_fixture();
    assert_eq!(
        topo.num_pcurves(),
        0,
        "premise: towel import carries no pcurves"
    );
}

#[test]
fn tampered_generatrix_proof_refuses_with_rollback() {
    // Move E_z's far endpoint 1e-3 off the R8 carrier: the generatrix proof
    // must refuse (not extend off-carrier), with full rollback.
    let what = "towel-R1-tamper-generatrix";
    let (mut topo, solid) = load_fixture();
    let band = select_r1(&topo, solid, 5, -1.0, what);
    let (supports, _, curved_ends) = supports_and_ends(&topo, solid, band);
    assert_eq!(curved_ends.len(), 1, "{what}: one R8 end face");
    // E_z far end: the R8/support generatrix touching a band vertex, moved
    // at its non-band endpoint. This is exactly the edge whose generatrix
    // proof must fire.
    let adjacency = topo.build_adjacency(solid).unwrap();
    let band_vertices: std::collections::HashSet<usize> = {
        let mut set = std::collections::HashSet::new();
        let outer = topo.face(band).unwrap().outer_wire();
        for oriented in topo.wire(outer).unwrap().edges() {
            let edge = topo.edge(oriented.edge()).unwrap();
            set.insert(edge.start().index());
            set.insert(edge.end().index());
        }
        set
    };
    let mut moved = false;
    for edge in remus_topology::explorer::solid_edges(&topo, solid).unwrap() {
        let data = topo.edge(edge).unwrap();
        if !matches!(data.curve(), EdgeCurve::Line) {
            continue;
        }
        let adjacent: Vec<_> = adjacency.faces_for_edge(edge).to_vec();
        if !adjacent.contains(&curved_ends[0]) {
            continue;
        }
        if !adjacent.iter().any(|face| supports.contains(face)) {
            continue;
        }
        let band_endpoints = [data.start().index(), data.end().index()]
            .into_iter()
            .filter(|index| band_vertices.contains(index))
            .count();
        if band_endpoints != 1 {
            continue;
        }
        for vertex in [data.start(), data.end()] {
            if band_vertices.contains(&vertex.index()) {
                continue;
            }
            let point = topo.vertex(vertex).unwrap().point();
            // Push 1e-3 off the R8 carrier (radially outward).
            let shifted = remus_math::vec::Point3::new(point.x() + 1e-3, point.y(), point.z());
            topo.vertex_mut(vertex).unwrap().set_point(shifted);
            moved = true;
            break;
        }
        if moved {
            break;
        }
    }
    assert!(moved, "{what}: tampered the E_z far endpoint");
    let faces_before = remus_topology::explorer::solid_faces(&topo, solid)
        .unwrap()
        .len();
    let census_before = surface_census(&topo, solid);
    let volume_before = remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap();
    let error =
        remus_operations::resize_blend::resize_blend(&mut topo, solid, band, 1.0, 0.0).unwrap_err();
    assert!(
        format!("{error}").contains("R8"),
        "{what}: generatrix proof names R8, got {error}"
    );
    assert_eq!(
        remus_topology::explorer::solid_faces(&topo, solid)
            .unwrap()
            .len(),
        faces_before,
        "{what}: refusal preserves faces"
    );
    assert_eq!(
        surface_census(&topo, solid),
        census_before,
        "{what}: refusal preserves carriers"
    );
    assert!(
        (remus_operations::measure::solid_volume(&topo, solid, 0.02).unwrap() - volume_before)
            .abs()
            < 1e-9,
        "{what}: refusal preserves volume"
    );
}
