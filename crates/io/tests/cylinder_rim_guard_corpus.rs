//! The cylinder rim guard over the fixture corpus.
//!
//! Shell assembly refuses a cylinder face whose boundary loops wind round
//! its axis an odd number of times in all: such a face has no end along the
//! axis (`remus_algo::cylinder_face_is_bounded`). The guard runs in every
//! boolean, so a wall it misreads is a refusal of a body that was exact.
//!
//! Every cylinder face of every STEP and arena fixture in `tests/data` is a
//! face of a real solid, bounded by construction: none may be counted as
//! endless. The faces are the corpus' own — partial turns for the most
//! part, full turns closed on a seam, rims of several edges, rims that are
//! splines — and they must actually be read (a guard that abstains
//! everywhere passes trivially). The makes the corpus does not hold (a wall
//! between two separate rim loops, a wall with a hole in it, a stepped
//! rim, an oblique one) are built by hand in the algo crate's
//! `builder_solid::helper_oracle_tests`.

// The tally is the point of running this with `--nocapture`.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::print_stderr
)]

use std::path::Path;

use remus_topology::Topology;
use remus_topology::explorer::solid_faces;
use remus_topology::face::FaceSurface;
use remus_topology::solid::SolidId;

fn data_dir() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

/// Every solid of every fixture with the extension `extension`, read by
/// `read`; a fixture that does not read (some are kept as refusals) is
/// skipped.
fn fixtures(
    extension: &str,
    read: impl Fn(&Path, &mut Topology) -> Option<Vec<SolidId>>,
) -> Vec<(String, Topology, Vec<SolidId>)> {
    let mut names: Vec<String> = std::fs::read_dir(data_dir())
        .unwrap()
        .map(|entry| entry.unwrap().file_name().into_string().unwrap())
        .filter(|name| {
            Path::new(name)
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case(extension))
        })
        .collect();
    names.sort();
    names
        .into_iter()
        .filter_map(|name| {
            let mut topo = Topology::new();
            let solids = read(&data_dir().join(&name), &mut topo)?;
            Some((name, topo, solids))
        })
        .collect()
}

#[test]
fn no_cylinder_face_of_the_fixture_corpus_is_counted_as_endless() {
    let mut bodies = fixtures("step", |path, topo| {
        let text = std::fs::read_to_string(path).ok()?;
        remus_io::step::reader::read_step(&text, topo).ok()
    });
    bodies.extend(fixtures("bin", |path, topo| {
        let bytes = std::fs::read(path).ok()?;
        remus_io::arena_io::deserialize_solid(&bytes, topo)
            .ok()
            .map(|solid| vec![solid])
    }));
    assert!(bodies.len() >= 100, "{} fixtures read", bodies.len());

    let (mut bounded, mut unread) = (0_usize, 0_usize);
    // The shapes the faces come in, to show the corpus covers them.
    let (mut seamed, mut two_loops, mut holed, mut spline_rimmed) = (0, 0, 0, 0);
    for (name, topo, solids) in &bodies {
        for &solid in solids {
            for face_id in solid_faces(topo, solid).unwrap() {
                let face = topo.face(face_id).unwrap();
                if !matches!(face.surface(), FaceSurface::Cylinder(_)) {
                    continue;
                }
                let verdict = remus_algo::cylinder_face_is_bounded(topo, face_id).unwrap();
                assert_ne!(
                    verdict,
                    Some(false),
                    "{name}: cylinder face {face_id:?} of {solid:?} is counted as having no end \
                     along its axis"
                );
                match verdict {
                    Some(_) => bounded += 1,
                    None => unread += 1,
                }
                let outer = topo.wire(face.outer_wire()).unwrap();
                let mut edges: Vec<_> = outer
                    .edges()
                    .iter()
                    .map(remus_topology::wire::OrientedEdge::edge)
                    .collect();
                let all = edges.len();
                edges.sort_by_key(|edge| edge.index());
                edges.dedup();
                seamed += usize::from(edges.len() < all);
                holed += usize::from(!face.inner_wires().is_empty());
                two_loops += usize::from(face.inner_wires().len() == 1 && all == 1);
                let splines = std::iter::once(face.outer_wire())
                    .chain(face.inner_wires().iter().copied())
                    .flat_map(|wire| topo.wire(wire).unwrap().edges().to_vec())
                    .any(|oe| {
                        matches!(
                            topo.edge(oe.edge()).unwrap().curve(),
                            remus_topology::edge::EdgeCurve::NurbsCurve(_)
                        )
                    });
                spline_rimmed += usize::from(splines);
            }
        }
    }
    eprintln!(
        "cylinder faces: {bounded} read and bounded, {unread} left unread; {seamed} with a \
         seam, {two_loops} between two single-edge loops, {holed} with inner loops, \
         {spline_rimmed} with a spline edge"
    );
    assert!(bounded >= 500, "only {bounded} cylinder faces were read");
    // Nine in ten are read: the guard does not pass by abstaining.
    assert!(unread * 9 <= bounded, "{unread} unread against {bounded}");
    assert!(
        seamed > 0 && spline_rimmed > 0,
        "the corpus lost its seamed or spline-rimmed walls"
    );
}
