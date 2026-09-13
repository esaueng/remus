//! Property oracles for the structured targets.
//!
//! A crash fuzzer would have found none of the fourteen kernel defects that
//! motivated this harness. Every one of them produced confident, well-formed,
//! *wrong* output: a bore filled in, a shell left open, a face that measured
//! zero, an operation that quietly did four of the five things it was asked
//! to do. So the value here is entirely in the oracles, not in reaching
//! `unreachable!()`.
//!
//! Each function below states a property the engine must hold, and panics
//! with the numbers when it does not — libFuzzer turns that into a
//! reproducible artifact.
//!
//! **A typed refusal is a pass.** `Unsupported`, `RadiusTooLarge`,
//! `EmptyResult` and friends are the engine correctly declining to return a
//! wrong answer. Callers stop the case; nothing here is invoked.

#![allow(dead_code)]

use std::collections::BTreeMap;

use remus_math::aabb::Aabb3;
use remus_math::vec::Point3;
use remus_operations::measure::{mass_properties, solid_bounding_box, solid_volume};
use remus_operations::tessellate::{
    boundary_edge_count, non_manifold_edge_count, tessellate_solid,
};
use remus_topology::Topology;
use remus_topology::edge::EdgeId;
use remus_topology::explorer;
use remus_topology::solid::SolidId;

/// Relative slack for volume comparisons.
///
/// Deliberately loose. These are *gross-disagreement* detectors, not precision
/// checks: the defects they exist to catch were order-one — a bore counted as
/// material, a wall integrating to exactly zero, 1735 mm³ invented from
/// nothing. A tight bound here would only manufacture false positives out of
/// ordinary inscribed-mesh undercount.
pub const VOL_SLACK: f64 = 1e-2;

/// Absolute floor, so near-zero volumes do not divide the relative test.
pub const VOL_FLOOR: f64 = 1e-6;

/// Topological census of a solid, cheap enough to run at every tree node.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Census {
    pub faces: usize,
    pub edges: usize,
    pub vertices: usize,
    /// Total inner wires across all faces — the hole mouths.
    pub inner_wires: usize,
    /// Edges referenced by exactly one face use: the shell is open there.
    pub free_edges: usize,
    /// Edges referenced by three or more face uses.
    pub non_manifold_edges: usize,
    /// Edges referenced by no face at all.
    pub orphan_edges: usize,
    /// Per surface-type face counts, for the analytic-vs-mesh tell.
    pub surfaces: BTreeMap<&'static str, usize>,
    /// One entry per connected component of the face-adjacency graph.
    ///
    /// Euler's formula is a statement about a *single* closed surface, and a
    /// solid is free to be several: a fuse of two operands that do not touch
    /// is one solid with two shells, which is a correct result. Summing `V-E+F`
    /// across them gives `2n`, not 2, so the aggregate figure cannot be tested
    /// against a constant — and worse, a genus error in one shell can cancel
    /// against an error in another. The test belongs per component.
    pub shells: Vec<ShellCensus>,
}

/// `V`, `E`, `F` and the inner-wire count for one connected shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ShellCensus {
    pub faces: usize,
    pub edges: usize,
    pub vertices: usize,
    pub inner_wires: usize,
}

impl ShellCensus {
    /// `V - E + F`, unadjusted.
    #[must_use]
    #[allow(clippy::cast_possible_wrap)]
    pub fn euler(&self) -> i64 {
        self.vertices as i64 - self.edges as i64 + self.faces as i64
    }

    /// `2 - (V - E + F - L)`, which must be a non-negative even number:
    /// twice the genus of this closed orientable surface.
    #[must_use]
    #[allow(clippy::cast_possible_wrap)]
    pub fn twice_genus(&self) -> i64 {
        2 - (self.euler() - self.inner_wires as i64)
    }
}

impl Census {
    /// `V - E + F` summed over the whole solid.
    ///
    /// Only useful for reporting. Test [`ShellCensus::twice_genus`] instead.
    #[must_use]
    #[allow(clippy::cast_possible_wrap)]
    pub fn euler(&self) -> i64 {
        self.vertices as i64 - self.edges as i64 + self.faces as i64
    }
}

/// Take a census.
///
/// # Errors
///
/// Propagates topology lookup failures.
pub fn census(topo: &Topology, solid: SolidId) -> Result<Census, remus_topology::TopologyError> {
    let (faces, edges, vertices) = explorer::solid_entity_counts(topo, solid)?;

    let mut inner_wires = 0;
    let mut surfaces: BTreeMap<&'static str, usize> = BTreeMap::new();
    for fid in explorer::solid_faces(topo, solid)? {
        let face = topo.face(fid)?;
        inner_wires += face.inner_wires().len();
        *surfaces.entry(face.surface().type_tag()).or_default() += 1;
    }

    // `edge_to_face_map` counts *face uses*, so a seam edge that appears twice
    // in one face's wire counts twice — which is what manifoldness means here.
    let map = explorer::edge_to_face_map(topo, solid)?;
    let mut free_edges = 0;
    let mut non_manifold_edges = 0;
    let mut orphan_edges = 0;
    for uses in map.values() {
        match uses.len() {
            0 => orphan_edges += 1,
            1 => free_edges += 1,
            2 => {}
            _ => non_manifold_edges += 1,
        }
    }

    Ok(Census {
        faces,
        edges,
        vertices,
        inner_wires,
        free_edges,
        non_manifold_edges,
        orphan_edges,
        surfaces,
        shells: shell_census(topo, solid)?,
    })
}

/// Split a solid into connected components of the face-adjacency graph and
/// count `V`, `E`, `F` and `L` within each.
///
/// Components are derived from shared edges rather than read off the solid's
/// shell list, because the question Euler's formula asks is about connectivity,
/// not about how the arena chose to file the faces.
///
/// # Errors
///
/// Propagates topology lookup failures.
pub fn shell_census(
    topo: &Topology,
    solid: SolidId,
) -> Result<Vec<ShellCensus>, remus_topology::TopologyError> {
    use std::collections::{BTreeSet, HashMap};

    let faces = explorer::solid_faces(topo, solid)?;
    let index: HashMap<_, usize> = faces.iter().enumerate().map(|(i, f)| (*f, i)).collect();

    // Union-find over faces, joined wherever two faces share an edge.
    let mut parent: Vec<usize> = (0..faces.len()).collect();
    fn find(parent: &mut [usize], mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }

    let map = explorer::edge_to_face_map(topo, solid)?;
    for uses in map.values() {
        let mut it = uses.iter().filter_map(|f| index.get(f).copied());
        if let Some(first) = it.next() {
            for other in it {
                let (a, b) = (find(&mut parent, first), find(&mut parent, other));
                if a != b {
                    parent[a] = b;
                }
            }
        }
    }

    // Accumulate per component.
    let mut groups: BTreeMap<usize, (usize, BTreeSet<usize>, BTreeSet<usize>, usize)> =
        BTreeMap::new();
    for (i, fid) in faces.iter().enumerate() {
        let root = find(&mut parent, i);
        let slot = groups
            .entry(root)
            .or_insert_with(|| (0, BTreeSet::new(), BTreeSet::new(), 0));
        slot.0 += 1;
        slot.3 += topo.face(*fid)?.inner_wires().len();
        for eid in explorer::face_edges(topo, *fid)? {
            slot.1.insert(eid.index());
        }
        for vid in explorer::face_vertices(topo, *fid)? {
            slot.2.insert(vid.index());
        }
    }

    Ok(groups
        .into_values()
        .map(|(f, e, v, l)| ShellCensus {
            faces: f,
            edges: e.len(),
            vertices: v.len(),
            inner_wires: l,
        })
        .collect())
}

// ── I1/I2: the result is actually a solid ──────────────────────────────

/// **Closed 2-manifold shell, and a consistent Euler characteristic.**
///
/// Every edge is used by exactly two faces, and `V - E + F - L = 2 - 2g` for
/// a non-negative integer genus `g`.
///
/// Catches the defect class where an operation returns something that is not
/// a solid while every check it was given passed — `shell` returning
/// `V-E+F = 7` where 3 was required, with 72 open mesh edges (#48); the open
/// shells left by `draft` (#41) and `chamfer` (#43).
///
/// The Euler test is applied **per connected shell**. A solid may legitimately
/// be several disjoint shells — fusing two operands that do not touch is a
/// correct two-shell result — and the aggregate `V-E+F` is then `2n`, not 2.
/// Testing the sum against a constant reports every such fuse as a defect,
/// which it is not; and it would let a genus error in one shell cancel against
/// an error in another.
///
/// # Panics
///
/// Panics with the census when the shell is open, non-manifold, or when any
/// component has an impossible Euler characteristic.
pub fn assert_closed_manifold(what: &str, c: &Census) {
    assert!(
        c.free_edges == 0 && c.non_manifold_edges == 0 && c.orphan_edges == 0,
        "{what}: result is not a closed 2-manifold — {} free edge(s), {} non-manifold edge(s), \
         {} orphan edge(s); census {c:?}",
        c.free_edges,
        c.non_manifold_edges,
        c.orphan_edges,
    );

    for (i, s) in c.shells.iter().enumerate() {
        let tg = s.twice_genus();
        assert!(
            tg >= 0 && tg % 2 == 0,
            "{what}: shell {i} of {} has V-E+F = {} with L = {} inner loop(s), implying genus \
             {}, which is not a non-negative integer; shell {s:?}; whole solid {c:?}",
            c.shells.len(),
            s.euler(),
            s.inner_wires,
            f64::from(i32::try_from(tg).unwrap_or(i32::MAX)) / 2.0,
        );
    }
}

/// **Mesh-level watertightness. Necessary, and nowhere near sufficient.**
///
/// The B-Rep check above and this one are not the same statement: the
/// tessellator welds shared boundary vertices and can paper over a small
/// B-Rep gap, while a B-Rep that is closed can still tessellate to a leaky
/// mesh through a collapsed seam. Defect #48 was visible here as 72 open mesh
/// edges, so both rungs are checked.
///
/// **Watertightness is satisfied by bodies that are wrong.** #52's
/// cross-drilled shaft passed this check with the bore filled *and* the bore
/// walls contributing zero triangles — two errors that cancel into a closed
/// mesh. Treat it only as a co-signature: pair it with the hole census
/// ([`assert_holes_preserved`]) and with a volume known independently
/// ([`assert_exact_volume`]). On its own it proves the mesh is closed, not
/// that it is the right mesh.
///
/// The converse is equally true and is why this check is kept: a solid can
/// measure exactly right and still tessellate open. The pointed-cone finding
/// in this harness's first campaign reported the exact analytic `πr²h/3`
/// through both volume routes while the mesh had 418 open edges.
///
/// # Panics
///
/// Panics when the mesh has boundary or non-manifold edges.
pub fn assert_watertight_mesh(what: &str, topo: &Topology, solid: SolidId, deflection: f64) {
    let Ok(mesh) = tessellate_solid(topo, solid, deflection) else {
        // A tessellation refusal is the engine declining, not a wrong answer.
        return;
    };
    let b = boundary_edge_count(&mesh);
    let n = non_manifold_edge_count(&mesh);
    assert!(
        b == 0 && n == 0,
        "{what}: tessellation at deflection {deflection} is not watertight — \
         {b} boundary edge(s), {n} non-manifold edge(s)",
    );
}

// ── I3: hole preservation ──────────────────────────────────────────────

/// **A modifier must not silently reduce the hole count.**
///
/// This one invariant covers the single largest defect class in the batch
/// that motivated the harness: `defeature` (#39), `draft` (#41),
/// `chamfer` (#43), `split` (#45) and `shell` (#48) each returned a solid
/// with the bore filled in, and each passed every check it was given.
///
/// Applied only to operations whose contract says the holes survive. An
/// operation *asked* to remove a hole is exempt, and a refusal is exempt.
///
/// # Panics
///
/// Panics when inner wires were lost.
pub fn assert_holes_preserved(what: &str, before: &Census, after: &Census) {
    assert!(
        after.inner_wires >= before.inner_wires,
        "{what}: dropped inner wires without saying so — {} before, {} after. \
         An operation that cannot keep a hole must refuse with a typed error, \
         not return a filled body. before {before:?}; after {after:?}",
        before.inner_wires,
        after.inner_wires,
    );
}

// ── I4: volume bounds ──────────────────────────────────────────────────

/// A volume reading, plus the box it came from.
pub struct Measured {
    pub volume: f64,
    pub aabb: Aabb3,
}

/// Read a solid's volume and bounding box.
///
/// Returns `None` when either measurement declines — a refusal, not a finding.
#[must_use]
pub fn measure(topo: &Topology, solid: SolidId) -> Option<Measured> {
    let aabb = solid_bounding_box(topo, solid).ok()?;
    let diag = (aabb.max - aabb.min).length();
    let volume = checked_volume(solid_volume(topo, solid, volume_deflection(diag)))?;
    Some(Measured { volume, aabb })
}

/// A typed refusal may be skipped; successful non-finite output is a finding.
pub fn checked_volume<E>(result: Result<f64, E>) -> Option<f64> {
    let volume = result.ok()?;
    assert!(
        volume.is_finite(),
        "successful volume measurement returned {volume}"
    );
    Some(volume)
}

/// A deflection fine enough to beat `solid_volume`'s internal clamp
/// (`min(requested, bbox_diag * 5e-5)`), so two requests actually differ,
/// but coarse enough that a fuzz iteration stays sub-second.
#[must_use]
pub fn volume_deflection(diag: f64) -> f64 {
    if diag.is_finite() && diag > 0.0 {
        (diag * 4e-5).max(1e-7)
    } else {
        1e-3
    }
}

/// **A boolean may not invent material.**
///
/// * `cut(a, b)` ⊆ `a`
/// * `fuse(a, b)` ≤ `vol(a) + vol(b)` and ≥ `max(vol(a), vol(b))`
/// * `intersect(a, b)` ≤ `min(vol(a), vol(b))`
///
/// and in every case the result's bounding box lies inside the operands'.
///
/// Catches `split` inventing 1735 mm³ (#45) and any boolean that returns a
/// superset of what it was given.
///
/// # Panics
///
/// Panics when the result exceeds its operands.
pub fn assert_volume_bounds(what: &str, op: &str, a: &Measured, b: &Measured, r: &Measured) {
    let slack = |v: f64| v.abs().mul_add(VOL_SLACK, VOL_FLOOR);

    match op {
        "cut" => assert!(
            r.volume <= a.volume + slack(a.volume),
            "{what}: cut produced {:.6} from a target of {:.6} — a cut cannot add material",
            r.volume,
            a.volume,
        ),
        "fuse" => {
            let sum = a.volume + b.volume;
            assert!(
                r.volume <= sum + slack(sum),
                "{what}: fuse produced {:.6} from operands summing to {:.6} — \
                 a union cannot exceed the sum of its parts",
                r.volume,
                sum,
            );
            let biggest = a.volume.max(b.volume);
            assert!(
                r.volume >= biggest - slack(biggest),
                "{what}: fuse produced {:.6}, less than its larger operand {:.6} — \
                 a union contains each operand",
                r.volume,
                biggest,
            );
        }
        "intersect" => {
            let smallest = a.volume.min(b.volume);
            assert!(
                r.volume <= smallest + slack(smallest),
                "{what}: intersect produced {:.6}, more than its smaller operand {:.6}",
                r.volume,
                smallest,
            );
        }
        _ => {}
    }

    // Containment in the operands' combined box catches invented material that
    // happens to balance out in the volume total.
    let hull = a.aabb.union(b.aabb);
    let margin = ((hull.max - hull.min).length() * 1e-6).max(1e-6);
    assert!(
        hull.expanded(margin).contains_point(r.aabb.min)
            && hull.expanded(margin).contains_point(r.aabb.max),
        "{what}: result box [{:?} .. {:?}] escapes the operands' box [{:?} .. {:?}] — \
         the result occupies space neither operand did",
        r.aabb.min,
        r.aabb.max,
        hull.min,
        hull.max,
    );
}

// ── I5: measurement, against a volume known by construction ────────────

/// **A body whose volume is known by construction must measure that volume.**
///
/// This is the *primary* measurement oracle, and it is deliberately not a
/// comparison between two of the kernel's own routes. Every measurement defect
/// closed in this batch was found by a hand-derived closed form and by nothing
/// else — #49 by a Steinmetz solid, #50 by Pappus' band area, #53 by Steiner's
/// formula for a rounded box. A closed form is the only oracle in this file
/// that does not consult the code under test.
///
/// The generator is built to keep one available. Every primitive it emits has
/// an elementary volume, placements are rigid so that volume survives them,
/// and a boolean over interior-disjoint operands has an exact answer too
/// (see [`assert_disjoint_boolean_exact`]). Where the closed form is unknown
/// the case falls back to the inequality in [`assert_volume_bounds`].
///
/// # Panics
///
/// Panics when the measured volume misses the constructed one.
pub fn assert_exact_volume(what: &str, expected: f64, measured: f64) {
    if !expected.is_finite() || !measured.is_finite() {
        return;
    }
    let scale = expected.abs().max(measured.abs()).max(VOL_FLOOR);
    let rel = (expected - measured).abs() / scale;
    assert!(
        rel <= VOL_SLACK,
        "{what}: volume is {expected:.9} by construction but the kernel measured \
         {measured:.9} (relative error {rel:.3e}). This is a closed form derived \
         outside the kernel, so the kernel is wrong.",
    );
}

/// **A boolean over interior-disjoint operands has an exact answer.**
///
/// When two solids' bounding boxes do not overlap in their interiors, the
/// solids cannot overlap either, and the algebra is total: `fuse` is the sum,
/// `cut` is the target untouched, `intersect` is empty. No approximation is
/// involved and no tolerance is needed, so this converts the loose inequality
/// of [`assert_volume_bounds`] into an equality.
///
/// This is the shape of the "a boolean silently drops an operand" defect: a
/// tool exactly tangent to a target's planar face makes `fuseAll` return the
/// target alone and `cut` a no-op. Both results are perfectly formed, both
/// satisfy every bound, both satisfy watertightness, and both are caught here
/// because the answer is a number known in advance. The generator's quantized
/// lattice exists to make tangency common rather than unreachable.
///
/// # Panics
///
/// Panics when a disjoint boolean did not return its exact algebraic result.
pub fn assert_disjoint_boolean_exact(what: &str, op: &str, va: f64, vb: f64, vr: f64) {
    let expected = match op {
        "fuse" => va + vb,
        "cut" => va,
        "intersect" => 0.0,
        _ => return,
    };
    let scale = expected.abs().max(vr.abs()).max(VOL_FLOOR);
    let rel = (expected - vr).abs() / scale;
    assert!(
        rel <= VOL_SLACK,
        "{what}: the operands do not overlap, so {op} must measure {expected:.9} \
         (operands {va:.9} and {vb:.9}), but the result measured {vr:.9} \
         (relative error {rel:.3e}). A disjoint boolean has an exact answer; \
         a result that merely looks plausible is an operand that was dropped.",
    );
}

/// **Two internal volume paths agreeing. A weak, secondary signal.**
///
/// `mass_properties` and `solid_volume` are *not* independent: below the face
/// list they meet in the same `integrate_face`, and they are exact for planar
/// results. So they agree on the all-planar bodies this is usually tested
/// against because they share their code, not because either is right — #53's
/// open-arc chord error moved both by the same 2.0% and this check saw
/// nothing.
///
/// It is kept because it does catch a defect confined to *one* route — #46's
/// wire-orientation bug rejected the exact planar path while leaving
/// tessellation alone. That is a real class, and the check is cheap. It is not
/// a substitute for [`assert_exact_volume`], and nothing in this harness
/// should be described as proven by it.
///
/// # Panics
///
/// Panics when the two disagree beyond [`VOL_SLACK`].
pub fn assert_measurements_agree(what: &str, topo: &Topology, solid: SolidId, tessellated: f64) {
    let Ok(props) = mass_properties(topo, solid) else {
        return; // a refusal, not a finding
    };
    if !props.mass.is_finite() {
        return;
    }
    let scale = props.mass.abs().max(tessellated.abs()).max(VOL_FLOOR);
    let rel = (props.mass - tessellated).abs() / scale;
    assert!(
        rel <= VOL_SLACK,
        "{what}: mass_properties says {:.9} but solid_volume says {:.9} \
         (relative difference {rel:.3e}). These two routes share their face \
         integrator, so they normally agree even when both are wrong; a \
         disagreement means one route alone is misreading the geometry.",
        props.mass,
        tessellated,
    );
}

/// **Refining the tessellation must not inflate the volume.**
///
/// A well-formed inscribed mesh converges to the truth *from below*, so a
/// finer deflection may only move the reading up by a hair. A volume that
/// climbs under refinement is the recorded signature of a self-intersection,
/// a collapsed seam or a doubled face.
///
/// # Panics
///
/// Panics when the two readings differ beyond [`VOL_SLACK`].
pub fn assert_deflection_stable(what: &str, topo: &Topology, solid: SolidId, coarse: f64) {
    let Ok(aabb) = solid_bounding_box(topo, solid) else {
        return;
    };
    let diag = (aabb.max - aabb.min).length();
    let fine = volume_deflection(diag) * 0.4;
    let Ok(v_fine) = solid_volume(topo, solid, fine) else {
        return;
    };
    if !v_fine.is_finite() {
        return;
    }
    let scale = v_fine.abs().max(coarse.abs()).max(VOL_FLOOR);
    let rel = (v_fine - coarse).abs() / scale;
    assert!(
        rel <= VOL_SLACK,
        "{what}: volume moved from {coarse:.9} to {v_fine:.9} when the deflection was \
         refined (relative {rel:.3e}). A sound solid's inscribed mesh converges from \
         below; movement this large signals broken geometry.",
    );
}

// ── I5b: scale invariance ──────────────────────────────────────────────

/// **The same shape at a different size must give the same relative answers.**
///
/// Modelling is scale-free: a part drawn in metres and the same part drawn in
/// millimetres are the same part. So under a uniform scale by `s`, with every
/// deflection scaled to match, the topology census must be *identical*, the
/// mesh must be watertight in both or neither, and the volume must move by
/// exactly `s³`.
///
/// This is the one oracle here that no amount of single-scale fuzzing reaches,
/// and it is aimed at a pattern this kernel keeps showing: a tolerance written
/// as an absolute distance rather than as a fraction of the model. #51's
/// provenance budget reported surviving faces as deleted at 1000×.
///
/// Two honest limits, both measured rather than assumed:
///
/// * **A body whose defect is scale-free — most of them — passes at every
///   scale.** This isolates one class, and earns its cost only because that
///   class is otherwise invisible.
/// * **The small end now reaches about 1e-5, not below.** `transform_solid`
///   used to reject any matrix whose determinant fell under `Tolerance.linear`
///   (1e-7) — a volume ratio measured against a length — so for a uniform
///   scale the test reduced to `s³ <= 1e-7` and every `s <= 0.00464` was
///   called degenerate, a metres-to-millimetres conversion among them.
///   Measured then: 1×, 0.1×, 0.01×, 0.005× and 0.0047× transformed; 0.0046×
///   and 0.001× were refused. That guard is now a dimensionless test on the
///   matrix's *shape* and a uniform scale of any size passes, so the sweep
///   below runs 0.001×. The floor that remains is the tessellator's absolute
///   `MERGE_GRID` (1e-7 in `operations::tessellate`): at a model scale near
///   1e-6 that grid becomes comparable to the model's own feature size and
///   welds distinct vertices, and a 1.7-radius sphere's mesh picks up 485
///   free edges. It does so identically whether the sphere is **built** at
///   1e-6 or scaled down to it, so it is not the transform — but it is the
///   next thing this oracle would report, and it is why the sweep stops at
///   0.001 rather than going further.
///
/// Non-finite or degenerate scaled geometry, and any refusal from the
/// transform, are passes. A refusal makes the check silently inert, which is
/// exactly the failure mode this harness hunts elsewhere; it is tolerated only
/// because the alternative is a false positive on every case. With the
/// determinant band gone, a uniform scale never takes that exit.
///
/// # Panics
///
/// Panics when the census, the watertightness or the `s³` volume law breaks.
pub fn assert_scale_invariant(what: &str, topo: &Topology, solid: SolidId, s: f64) {
    use remus_math::mat::Mat4;
    use remus_operations::transform::transform_solid;

    let Ok(base) = census(topo, solid) else {
        return;
    };
    let Ok(aabb) = solid_bounding_box(topo, solid) else {
        return;
    };
    let diag = (aabb.max - aabb.min).length();
    if !(diag.is_finite() && diag > 0.0) {
        return;
    }
    // Relative deflections, so the two runs ask for the same mesh density.
    let defl = volume_deflection(diag) * 4.0;
    let Ok(v0) = solid_volume(topo, solid, volume_deflection(diag)) else {
        return;
    };
    let watertight0 = tessellate_solid(topo, solid, defl)
        .map(|m| boundary_edge_count(&m) == 0 && non_manifold_edge_count(&m) == 0);

    let mut scaled = topo.clone();
    if transform_solid(&mut scaled, solid, &Mat4::scale(s, s, s)).is_err() {
        return; // a refusal is a pass
    }
    let Ok(after) = census(&scaled, solid) else {
        return;
    };

    assert!(
        base.faces == after.faces
            && base.edges == after.edges
            && base.vertices == after.vertices
            && base.inner_wires == after.inner_wires
            && base.free_edges == after.free_edges
            && base.non_manifold_edges == after.non_manifold_edges,
        "{what}: scaling the body by {s}x changed its topology census from \
         {base:?} to {after:?}. Size is not shape; a count that moves with the \
         model's units is a tolerance written as an absolute distance.",
    );

    let Ok(scaled_aabb) = solid_bounding_box(&scaled, solid) else {
        return;
    };
    let scaled_diag = (scaled_aabb.max - scaled_aabb.min).length();
    if !(scaled_diag.is_finite() && scaled_diag > 0.0) {
        return;
    }

    if let Ok(w0) = watertight0
        && let Ok(mesh) = tessellate_solid(&scaled, solid, volume_deflection(scaled_diag) * 4.0)
    {
        let w1 = boundary_edge_count(&mesh) == 0 && non_manifold_edge_count(&mesh) == 0;
        assert!(
            w0 == w1,
            "{what}: at 1x the tessellation is {}, at {s}x the same shape at the \
             same relative deflection is {}. Whether a mesh closes must not depend \
             on the units the model is drawn in.",
            if w0 { "watertight" } else { "leaky" },
            if w1 { "watertight" } else { "leaky" },
        );
    }

    let Ok(v1) = solid_volume(&scaled, solid, volume_deflection(scaled_diag)) else {
        return;
    };
    if !v0.is_finite() || !v1.is_finite() {
        return;
    }
    let expected = v0 * s * s * s;
    let denom = expected.abs().max(v1.abs()).max(VOL_FLOOR);
    let rel = (expected - v1).abs() / denom;
    assert!(
        rel <= VOL_SLACK,
        "{what}: volume {v0:.9} scaled by {s}x should measure {expected:.9} \
         but measured {v1:.9} (relative error {rel:.3e}). Volume goes as the \
         cube of length; a different ratio is a length compared against a \
         constant somewhere.",
    );
}

// ── I6: determinism ────────────────────────────────────────────────────

/// A canonical, order-independent fingerprint of a solid's topology and
/// coarse geometry.
///
/// Entity *ids* are deliberately excluded: arena indices depend on allocation
/// order, which is an implementation detail. What must be reproducible is the
/// shape — counts, surface census, and the multiset of face normals and
/// centroids, quantized so that last-bit rounding does not register as
/// non-determinism.
#[must_use]
pub fn fingerprint(topo: &Topology, solid: SolidId) -> Option<Vec<u8>> {
    use std::hash::{DefaultHasher, Hash, Hasher};

    let c = census(topo, solid).ok()?;
    let mut rows: Vec<(i64, i64, i64, i64, i64, i64, &'static str)> = Vec::new();
    for fid in explorer::solid_faces(topo, solid).ok()? {
        let face = topo.face(fid).ok()?;
        let verts = remus_operations::boolean::face_polygon(topo, fid).ok()?;
        let n = verts.len().max(1) as f64;
        let cx = verts.iter().map(|p| p.x()).sum::<f64>() / n;
        let cy = verts.iter().map(|p| p.y()).sum::<f64>() / n;
        let cz = verts.iter().map(|p| p.z()).sum::<f64>() / n;
        let q = |v: f64| (v * 1e6).round() as i64;
        rows.push((
            q(cx),
            q(cy),
            q(cz),
            verts.len() as i64,
            face.inner_wires().len() as i64,
            i64::from(face.is_reversed()),
            face.surface().type_tag(),
        ));
    }
    rows.sort_unstable();

    let mut h = DefaultHasher::new();
    (c.faces, c.edges, c.vertices, c.inner_wires).hash(&mut h);
    c.surfaces.hash(&mut h);
    rows.hash(&mut h);
    Some(h.finish().to_le_bytes().to_vec())
}

/// **The same input must produce the same output.**
///
/// # Panics
///
/// Panics when two evaluations of the identical tree disagree.
pub fn assert_deterministic(what: &str, first: &[u8], second: &[u8]) {
    assert!(
        first == second,
        "{what}: two evaluations of the identical input produced different topology \
         fingerprints ({first:02x?} vs {second:02x?}). The engine is reading uninitialised \
         state, iterating a hash map, or depending on address order.",
    );
}

// ── I7: idempotence ────────────────────────────────────────────────────

/// **`fuse(a, a)` is `a`, and cutting twice is cutting once.**
///
/// # Panics
///
/// Panics when the repeat differs from the original.
pub fn assert_idempotent(what: &str, once: &Census, twice: &Census, v_once: f64, v_twice: f64) {
    let scale = v_once.abs().max(v_twice.abs()).max(VOL_FLOOR);
    assert!(
        (v_once - v_twice).abs() / scale <= VOL_SLACK,
        "{what}: repeating the operation changed the volume from {v_once:.9} to {v_twice:.9}",
    );
    assert!(
        once.inner_wires == twice.inner_wires,
        "{what}: repeating the operation changed the hole count from {} to {}",
        once.inner_wires,
        twice.inner_wires,
    );
}

// ── I8: completeness ───────────────────────────────────────────────────

/// **An operation given N items processes all N, or fails saying which it did not.**
///
/// Catches #44, where the binding returned a silent subset of the requested
/// blend — indistinguishable from success at the call site.
///
/// # Panics
///
/// Panics on a silent partial success.
pub fn assert_complete(what: &str, requested: usize, succeeded: usize, is_partial: bool) {
    assert!(
        !is_partial && succeeded >= requested,
        "{what}: asked for {requested} item(s), reported {succeeded} succeeded \
         (is_partial = {is_partial}) and still returned Ok. A partial result must be \
         a typed error naming what it skipped, never a success.",
    );
}

/// **An option the caller set must change what comes back.**
///
/// Completeness is not only about how many entities were touched — it is about
/// whether the request was honoured at all. `SweepCornerMode::Round` accepted
/// the request and returned the `Smooth` result (#52); the blend binding
/// accepted five edges and blended four (#44). Both are the same shape of
/// failure: `Ok`, well-formed, and not what was asked for.
///
/// The check is the crude one that actually works — run the operation twice
/// with two genuinely different settings and require the answers to differ. An
/// implementation that ignores the option produces the same solid twice and is
/// caught; one that honours it cannot fail this.
///
/// What is compared is the volume each setting *changed*, `before - after`,
/// not the resulting volumes themselves. A fillet of r = 0.05 on a unit box
/// edge removes ~5e-4; the same body fused with a disjoint torus of volume
/// ~800 does not make that fillet any less real, but it did push the
/// result-to-result difference under a body-relative floor and raised a false
/// alarm. Scaling by the change keeps the judgement local to the option.
///
/// Both operations *refusing* is a pass, and so is one refusing where the other
/// did not — that is the option changing the outcome.
///
/// # Panics
///
/// Panics when two different requests produced indistinguishable results:
/// neither setting changed the volume, or both changed it by the same amount.
pub fn assert_option_honoured(
    what: &str,
    setting_a: &str,
    setting_b: &str,
    before: f64,
    va: f64,
    vb: f64,
) {
    if !before.is_finite() || !va.is_finite() || !vb.is_finite() {
        return;
    }
    let da = before - va;
    let db = before - vb;
    let scale = da.abs().max(db.abs());
    assert!(
        scale > VOL_FLOOR,
        "{what}: {setting_a} and {setting_b} both left the volume at {before:.9} \
         (changes {da:.3e} and {db:.3e}). The option was accepted and then ignored.",
    );
    assert!(
        (da - db).abs() / scale > VOL_SLACK,
        "{what}: {setting_a} and {setting_b} are different requests but both \
         changed the volume by {da:.3e} (before {before:.9}, after {va:.9} and \
         {vb:.9}). The option was accepted and then ignored.",
    );
}

// ── B26/1: inclusion–exclusion ───────────────────────────────────────

/// `vol(A ∪ B) + vol(A ∩ B) = vol(A) + vol(B)`.
///
/// Callers must produce the union and intersection via
/// `boolean_with_context` under `ExactOnly` with disclosed `BooleanOutcome`
/// quality; any typed refusal (`Unsupported`, `ExactOnlyUnattainable`,
/// `EmptyResult`, …) on either call is a pass. A finite `Ok` volume that is
/// non-finite is a finding, never a skip: NaN/Inf out of a successful
/// measurement is malformed output, not a refusal.
///
/// # Panics
///
/// Panics when the two result volumes do not sum to the operand volumes
/// within [`VOL_SLACK`], or when any successful measurement is non-finite.
pub fn assert_inclusion_exclusion(what: &str, va: f64, vb: f64, v_union: f64, v_inter: f64) {
    for (label, v) in [
        ("vol(A)", va),
        ("vol(B)", vb),
        ("union", v_union),
        ("intersect", v_inter),
    ] {
        assert!(
            v.is_finite(),
            "{what}: inclusion–exclusion {label} measured {v} — a successful \
             measurement returning NaN/Inf is malformed output, not a refusal.",
        );
    }
    let lhs = v_union + v_inter;
    let rhs = va + vb;
    let scale = lhs.abs().max(rhs.abs()).max(VOL_FLOOR);
    let rel = (lhs - rhs).abs() / scale;
    assert!(
        rel <= VOL_SLACK,
        "{what}: inclusion–exclusion fails — vol(A ∪ B) = {v_union:.9} plus \
         vol(A ∩ B) = {v_inter:.9} sums to {lhs:.9}, but vol(A) + vol(B) = \
         {va:.9} + {vb:.9} = {rhs:.9} (relative error {rel:.3e}).",
    );
}

// ── B26/2: cut complement ────────────────────────────────────────────

/// `vol(A) = vol(A ∖ B) + vol(A ∩ B)`.
///
/// Same contract as [`assert_inclusion_exclusion`]: both results from the
/// exact path; typed `Err` is a pass, non-finite successful output is a
/// finding.
///
/// # Panics
///
/// Panics when the cut and the intersection do not sum to the target within
/// [`VOL_SLACK`], or when any successful measurement is non-finite.
pub fn assert_cut_complement(what: &str, va: f64, v_cut: f64, v_inter: f64) {
    for (label, v) in [("vol(A)", va), ("cut", v_cut), ("intersect", v_inter)] {
        assert!(
            v.is_finite(),
            "{what}: cut complement {label} measured {v} — a successful \
             measurement returning NaN/Inf is malformed output, not a refusal.",
        );
    }
    let scale = va.abs().max((v_cut + v_inter).abs()).max(VOL_FLOOR);
    let rel = (va - (v_cut + v_inter)).abs() / scale;
    assert!(
        rel <= VOL_SLACK,
        "{what}: cut complement fails — vol(A ∖ B) = {v_cut:.9} plus \
         vol(A ∩ B) = {v_inter:.9} sums to {:.9}, but vol(A) = {va:.9} \
         (relative error {rel:.3e}).",
        v_cut + v_inter,
    );
}

// ── B26/3: translation invariance ────────────────────────────────────

/// `solid_volume` must not move under rigid translation.
///
/// The recorded precedent is a doubled boundary integrating phantom
/// material whose contribution depends on absolute position. The comparison
/// uses the existing [`VOL_SLACK`]. A typed `Err` from the transform or the
/// second measurement is a pass; a successful but non-finite reading is a
/// finding.
///
/// # Panics
///
/// Panics when the translated body measures differently beyond [`VOL_SLACK`]
/// or returns non-finite output.
pub fn assert_translation_invariant(
    what: &str,
    topo: &Topology,
    solid: SolidId,
    v0: f64,
    dx: f64,
    dy: f64,
    dz: f64,
) {
    use remus_math::mat::Mat4;
    use remus_operations::transform::transform_solid;

    assert!(
        v0.is_finite(),
        "{what}: translation check input volume is {v0} — a successful \
         measurement returning NaN/Inf is malformed output, not a refusal.",
    );
    let mut moved = topo.clone();
    if transform_solid(&mut moved, solid, &Mat4::translation(dx, dy, dz)).is_err() {
        return; // a refusal is a pass
    }
    let Ok(aabb) = solid_bounding_box(&moved, solid) else {
        return;
    };
    let diag = (aabb.max - aabb.min).length();
    if !(diag.is_finite() && diag > 0.0) {
        return;
    }
    let Ok(v1) = solid_volume(&moved, solid, volume_deflection(diag)) else {
        return;
    };
    assert!(
        v1.is_finite(),
        "{what}: translated body measured {v1} — a successful measurement \
         returning NaN/Inf is malformed output, not a refusal.",
    );
    let scale = v0.abs().max(v1.abs()).max(VOL_FLOOR);
    let rel = (v0 - v1).abs() / scale;
    assert!(
        rel <= VOL_SLACK,
        "{what}: volume moved from {v0:.9} to {v1:.9} under translation by \
         ({dx:.3}, {dy:.3}, {dz:.3}) (relative {rel:.3e}).",
    );
}

// ── B26/4: exact-path stability under input nudges ───────────────────

/// One exact boolean attempt: success carries census + volume; refusal
/// carries the typed engine error. `OperationsError` has no `Clone`, so the
/// refusal keeps the error's rendering alongside its variant name — the
/// comparison in [`assert_nudge_stable`] is kind-level (refused vs refused
/// passes regardless of message), never string matching.
#[derive(Debug, Clone)]
pub enum NudgeOutcome {
    /// Exact boolean succeeded; census and volume were measured.
    Success {
        /// Topological census of the result.
        census: Census,
        /// Measured volume of the result.
        volume: f64,
    },
    /// The engine refused with a typed error.
    Refused {
        /// Variant name of the `OperationsError` (e.g. `Unsupported`).
        variant: &'static str,
        /// Full rendering of the error, for the panic message.
        reason: String,
    },
    /// The boolean succeeded but a follow-up census/volume measurement
    /// returned a typed `Err`. Compared by [`assert_nudge_stable`] as its
    /// own kind: a typed measurement refusal is not success data and must
    /// not be fabricated into any.
    Unmeasurable {
        /// Rendered measurement error.
        reason: String,
    },
}

/// Run one exact boolean and classify the outcome.
///
/// `boolean_with_context` under `ExactOnly` with disclosed quality:
/// `Exact` success is measured (census + volume); any `Err` becomes
/// [`NudgeOutcome::Refused`] with the typed error preserved (no strings
/// invented at this layer — the error value itself is carried). A
/// `BooleanQuality::Approximate` under `ExactOnly` violates the policy and
/// panics: it must never be relabelled a refusal.
pub fn exact_boolean_outcome(
    topo: &mut Topology,
    op: remus_operations::boolean::BooleanOp,
    a: SolidId,
    b: SolidId,
) -> NudgeOutcome {
    use remus_math::context::{FallbackPolicy, OperationContext};
    use remus_operations::boolean::{BooleanQuality, boolean_with_context};

    let context = OperationContext::new().with_fallback(FallbackPolicy::ExactOnly);
    match boolean_with_context(topo, op, a, b, &context) {
        Ok(outcome) => match outcome.quality {
            BooleanQuality::Exact => {
                let census = match census(topo, outcome.solid) {
                    Ok(c) => c,
                    Err(e) => {
                        return NudgeOutcome::Unmeasurable {
                            reason: e.to_string(),
                        };
                    }
                };
                let volume = match measure(topo, outcome.solid) {
                    Some(m) => m.volume,
                    None => {
                        return NudgeOutcome::Unmeasurable {
                            reason: "volume/bbox measurement refused".to_string(),
                        };
                    }
                };
                assert!(
                    volume.is_finite(),
                    "exact boolean result measured {volume} — a successful \
                     measurement returning NaN/Inf is malformed output, not \
                     a refusal.",
                );
                NudgeOutcome::Success { census, volume }
            }
            BooleanQuality::Approximate { deflection } => {
                panic!(
                    "ExactOnly policy returned Approximate quality at deflection \
                     {deflection} — the policy invariant is violated; this must \
                     never be relabelled a refusal.",
                );
            }
        },
        Err(e) => {
            let reason = e.to_string();
            let variant = match e {
                remus_operations::OperationsError::ExactOnlyUnattainable => "ExactOnlyUnattainable",
                remus_operations::OperationsError::InvalidInput { .. } => "InvalidInput",
                remus_operations::OperationsError::NonManifoldResult => "NonManifoldResult",
                remus_operations::OperationsError::EmptyResult { .. } => "EmptyResult",
                remus_operations::OperationsError::Unsupported { .. } => "Unsupported",
                remus_operations::OperationsError::BodyClassMeasureMismatch { .. } => {
                    "BodyClassMeasureMismatch"
                }
                remus_operations::OperationsError::BodyClassOperationUnsupported { .. } => {
                    "BodyClassOperationUnsupported"
                }
                remus_operations::OperationsError::BodyValidationFailed { .. } => {
                    "BodyValidationFailed"
                }
                remus_operations::OperationsError::HealingValidationFailed { .. } => {
                    "HealingValidationFailed"
                }
                remus_operations::OperationsError::HealingVerificationUnavailable { .. } => {
                    "HealingVerificationUnavailable"
                }
                remus_operations::OperationsError::HealingRepairRefused { .. } => {
                    "HealingRepairRefused"
                }
                remus_operations::OperationsError::ConfiguredHealingValidationFailed { .. } => {
                    "ConfiguredHealingValidationFailed"
                }
                remus_operations::OperationsError::ConfiguredHealingVerificationUnavailable {
                    ..
                } => "ConfiguredHealingVerificationUnavailable",
                remus_operations::OperationsError::PatternInstancesOverlap { .. } => {
                    "PatternInstancesOverlap"
                }
                remus_operations::OperationsError::Topology(_) => "Topology",
                remus_operations::OperationsError::Math(_) => "Math",
                remus_operations::OperationsError::Algo(_) => "Algo",
                remus_operations::OperationsError::Blend(_) => "Blend",
                remus_operations::OperationsError::ResizeBlend(_) => "ResizeBlend",
                remus_operations::OperationsError::Check(_) => "Check",
                remus_operations::OperationsError::Geometry(_) => "Geometry",
                remus_operations::OperationsError::Heal(_) => "Heal",
                remus_operations::OperationsError::Offset(_) => "Offset",
                remus_operations::OperationsError::PartialResult { .. } => "PartialResult",
            };
            NudgeOutcome::Refused { variant, reason }
        }
    }
}

/// Exact-path stability under a 1e-13 rigid input nudge.
///
/// Compares refusal-versus-success, never skipping kind changes:
/// * `Refused` vs `Refused` — pass.
/// * `Unmeasurable` vs `Unmeasurable` — pass.
/// * `Success` vs `Success` — full [`Census`] equality (every field,
///   including `shells` and `orphan_edges`) and volume within [`VOL_SLACK`].
/// * Any kind change — panic.
///
/// # Panics
///
/// Panics on outcome-kind flips, census moves, volume moves beyond
/// [`VOL_SLACK`], or non-finite success volumes.
pub fn assert_nudge_stable(what: &str, base: &NudgeOutcome, nudged: &NudgeOutcome) {
    match (base, nudged) {
        (NudgeOutcome::Refused { .. }, NudgeOutcome::Refused { .. })
        | (NudgeOutcome::Unmeasurable { .. }, NudgeOutcome::Unmeasurable { .. }) => {}
        (
            NudgeOutcome::Success {
                census: c0,
                volume: v0,
            },
            NudgeOutcome::Success {
                census: c1,
                volume: v1,
            },
        ) => {
            assert!(
                c0 == c1,
                "{what}: a 1e-13 input nudge changed the exact result's census \
                 from {c0:?} to {c1:?}.",
            );
            for (label, v) in [("base", *v0), ("nudged", *v1)] {
                assert!(
                    v.is_finite(),
                    "{what}: nudge-stability {label} volume is {v} — a \
                     successful measurement returning NaN/Inf is malformed \
                     output, not a refusal.",
                );
            }
            let scale = v0.abs().max(v1.abs()).max(VOL_FLOOR);
            let rel = (v0 - v1).abs() / scale;
            assert!(
                rel <= VOL_SLACK,
                "{what}: a 1e-13 input nudge moved the exact result's \
                 volume from {v0:.9} to {v1:.9} (relative {rel:.3e}).",
            );
        }
        (b, n) => {
            panic!(
                "{what}: a 1e-13 input nudge changed the exact outcome kind \
                 from {b:?} to {n:?}.",
            );
        }
    }
}

/// Nudge operand `b` by a rigid 1e-13 translation along +x, then run the
/// exact boolean on `(a, b)` in both arenas.
///
/// Uses [`transform_solid`](remus_operations::transform::transform_solid) so
/// vertices, edge curves, and face surfaces move together: the nudged input
/// is a consistent rigidly moved operand, not a vertex-edited inconsistent
/// B-Rep. Compare the pair with [`assert_nudge_stable`].
pub fn exact_boolean_nudged_pair(
    topo: &Topology,
    op: remus_operations::boolean::BooleanOp,
    a: SolidId,
    b: SolidId,
) -> Option<(NudgeOutcome, NudgeOutcome)> {
    use remus_math::mat::Mat4;
    use remus_operations::transform::transform_solid;

    let mut t0 = topo.clone();
    let mut t1 = topo.clone();
    if transform_solid(&mut t1, b, &Mat4::translation(1e-13, 0.0, 0.0)).is_err() {
        return None;
    }
    Some((
        exact_boolean_outcome(&mut t0, op, a, b),
        exact_boolean_outcome(&mut t1, op, a, b),
    ))
}

// ── B26/5: position-quantized free-edge check ─────────────────────────

/// Quantization grid for the position-based free-edge check.
///
/// Ten times the kernel linear tolerance (1e-7) and the tessellator weld
/// grid (1e-7): fine enough that the generator's half-unit lattice can never
/// merge distinct vertices into one key, coarse enough to absorb weld-scale
/// rounding. Quantization reduces — but cannot eliminate — split-key risk
/// from boundary rounding; a vertex sitting within half a quantum of a cell
/// boundary can still split keys across runs, so this oracle supplements the
/// edge-id census rather than replacing it.
pub const POS_KEY_GRID: f64 = 1e-6;

/// A position key for one vertex: coordinates quantized to [`POS_KEY_GRID`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PosKey(pub i64, pub i64, pub i64);

/// Quantize one vertex position to the [`POS_KEY_GRID`] lattice.
///
/// Returns `None` for non-finite coordinates: non-finite geometry is never
/// silently skipped or quantized into a key.
#[must_use]
pub fn pos_key(p: Point3) -> Option<PosKey> {
    for v in [p.x(), p.y(), p.z()] {
        if !v.is_finite() {
            return None;
        }
    }
    let q = |v: f64| (v / POS_KEY_GRID).round() as i64;
    Some(PosKey(q(p.x()), q(p.y()), q(p.z())))
}

/// Canonical endpoint and curve-midpoint keys. Closed curves retain their
/// geometric identity even when their endpoint vertices coincide.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EdgeGeomKey {
    /// Canonical (min, max) endpoint keys.
    pub ends: (PosKey, PosKey),
    /// Quantized curve midpoint.
    pub mid: PosKey,
    /// Distinguish truly point-like line edges without encoding IDs as geometry.
    pub point_edge: Option<usize>,
}

/// Failure mode of [`edge_geom_key`]. `Err` means the edge cannot be keyed
/// and the caller must decide: the closed check treats it as a finding (see
/// [`assert_position_closed`]), never a skip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeomKeyError {
    /// A topology lookup failed.
    Topology,
    /// A vertex position, trim bound, or midpoint evaluation is non-finite.
    NonFinite,
    /// A non-`Line` edge has no authoritative stored trim. Missing authority
    /// is never reconstructed: blessing it here would hide invalid output.
    MissingAuthority,
}

/// Read an edge's geometric key.
///
/// The midpoint is sampled through the edge curve at mid stored-trim
/// (`strict_domain`, which also validates finiteness), or at mid `[0, 1]`
/// for `Line`. Non-finite positions, bounds, or evaluations, failed lookups,
/// and missing non-`Line` authority are all `Err` — never silently skipped.
///
/// Only point-like straight lines carry an ID discriminant; coincident
/// endpoints on a curved edge still require valid trim authority.
pub fn edge_geom_key(topo: &Topology, eid: EdgeId) -> Result<EdgeGeomKey, GeomKeyError> {
    use remus_topology::edge::EdgeCurve;

    let edge = topo.edge(eid).map_err(|_| GeomKeyError::Topology)?;
    let a = topo
        .vertex(edge.start())
        .map_err(|_| GeomKeyError::Topology)?
        .point();
    let b = topo
        .vertex(edge.end())
        .map_err(|_| GeomKeyError::Topology)?
        .point();
    let (ka, kb) = (
        pos_key(a).ok_or(GeomKeyError::NonFinite)?,
        pos_key(b).ok_or(GeomKeyError::NonFinite)?,
    );
    let ends = if ka <= kb { (ka, kb) } else { (kb, ka) };
    let (t0, t1) = if matches!(edge.curve(), EdgeCurve::Line) {
        (0.0, 1.0)
    } else {
        edge.strict_domain().map_err(|e| match e {
            remus_topology::edge::EdgeDomainError::Missing { .. } => GeomKeyError::MissingAuthority,
            remus_topology::edge::EdgeDomainError::Invalid { .. } => GeomKeyError::NonFinite,
            _ => GeomKeyError::NonFinite,
        })?
    };
    if !t0.is_finite() || !t1.is_finite() {
        return Err(GeomKeyError::NonFinite);
    }
    let mid = edge.curve().evaluate_with_endpoints(0.5 * (t0 + t1), a, b);
    if !mid.x().is_finite() || !mid.y().is_finite() || !mid.z().is_finite() {
        return Err(GeomKeyError::NonFinite);
    }
    Ok(EdgeGeomKey {
        ends,
        mid: pos_key(mid).ok_or(GeomKeyError::NonFinite)?,
        point_edge: (matches!(edge.curve(), EdgeCurve::Line) && ka == kb).then_some(eid.index()),
    })
}

/// Per-key face-use counts over all shells (outer + inner), preserving
/// seam-use multiplicity (each wire occurrence counts once).
///
/// Multiplicity convention matches [`census`]'s `edge_to_face_map` count:
/// one wire occurrence = one face use. A seam edge traversed twice in one
/// face's wire contributes two uses — the B-Rep convention the manifold
/// check relies on. A single-face body whose boundary wire walks its seam
/// edges twice each (the torus primitive's genus-1 fundamental polygon:
/// one face, 2 edges, 4 occurrences) therefore keys each edge at 2 uses:
/// geometrically closed, matching the id census.
///
/// Returns the counts plus the number of wire occurrences that could not be
/// keyed. Unkeyable occurrences are reported, never dropped silently: see
/// [`assert_position_closed`].
pub fn position_edge_use_counts(
    topo: &Topology,
    solid: SolidId,
) -> Result<(BTreeMap<EdgeGeomKey, usize>, usize), remus_topology::TopologyError> {
    let mut counts: BTreeMap<EdgeGeomKey, usize> = BTreeMap::new();
    let mut unkeyed = 0usize;
    let data = topo.solid(solid)?;
    let shells = std::iter::once(data.outer_shell()).chain(data.inner_shells().iter().copied());
    for shell_id in shells {
        for fid in topo.shell(shell_id)?.faces().to_vec() {
            let face = topo.face(fid)?;
            for wire_id in
                std::iter::once(face.outer_wire()).chain(face.inner_wires().iter().copied())
            {
                for oe in topo.wire(wire_id)?.edges().to_vec() {
                    match edge_geom_key(topo, oe.edge()) {
                        Ok(key) => *counts.entry(key).or_default() += 1,
                        Err(_) => unkeyed += 1,
                    }
                }
            }
        }
    }
    Ok((counts, unkeyed))
}

/// Position-quantized closedness: every geometric edge key must have exactly
/// 2 uses; any unkeyable wire occurrence (non-finite geometry or missing
/// non-`Line` authority) is a finding.
///
/// Supplements the edge-id census, which is blind to position-duplicate
/// boundaries built from distinct ids.
///
/// # Panics
///
/// Panics on position-level free edges (1 use), non-manifold keys (3+ uses),
/// or unkeyable occurrences.
pub fn assert_position_closed(what: &str, topo: &Topology, solid: SolidId) {
    let Ok((counts, unkeyed)) = position_edge_use_counts(topo, solid) else {
        return;
    };
    let mut free: Vec<EdgeGeomKey> = Vec::new();
    let mut non_manifold: Vec<EdgeGeomKey> = Vec::new();
    for (key, uses) in &counts {
        match uses {
            1 => free.push(*key),
            2 => {}
            _ => non_manifold.push(*key),
        }
    }
    assert!(
        free.is_empty() && non_manifold.is_empty() && unkeyed == 0,
        "{what}: position-quantized edge check failed — {} key(s) with 1 use \
         (e.g. {free:.3?}), {} key(s) with 3+ uses (e.g. {non_manifold:.3?}), \
         {unkeyed} unkeyable wire occurrence(s) (non-finite geometry or \
         missing curve authority).",
        free.len(),
        non_manifold.len(),
    );
}
