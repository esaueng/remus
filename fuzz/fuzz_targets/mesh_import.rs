//! Triangle-mesh to B-Rep import (`stl::import::import_mesh`, IO-12).
//!
//! The mesh readers are fuzzed for parsing only; this target drives the
//! shared import every mesh format ends in. Corners are drawn from a small
//! pool of lattice points, either copied exactly or moved just inside or
//! just outside the weld tolerance, so coincident corners, duplicate and
//! degenerate triangles, over-shared edges and inconsistent winding are
//! common, with occasional out-of-range indices, non-finite coordinates and
//! extreme offsets.
//!
//! Oracles: no two welded vertices lie within the weld tolerance; an accepted
//! mesh yields one solid whose faces are planar triangles, no more than the
//! input triangle count, with every edge used by at most two face uses
//! (import refuses over-shared edges); a refusal that input validation can
//! decide must leave the topology empty, and an over-shared or all-degenerate
//! refusal must allocate no edge, wire, loop, coedge, face, shell or solid.

#![no_main]

use arbitrary::{Arbitrary, Unstructured};
use libfuzzer_sys::fuzz_target;
use remus_math::vec::{Point3, Vec3};
use remus_operations::tessellate::TriangleMesh;
use remus_topology::Topology;
use remus_topology::face::FaceSurface;

const MAX_VERTICES: usize = 48;
const MAX_TRIANGLES: usize = 96;
const MAX_POOL: usize = 12;

#[derive(Debug, Arbitrary)]
enum Offset {
    Origin,
    Far,
    VeryFar,
    NonFinite,
}

/// How a vertex derives from its shared pool point.
#[derive(Debug, Arbitrary)]
enum Placement {
    /// The pool point itself: an exact duplicate for the weld to merge.
    Copy,
    /// Moved half the weld tolerance along one axis: must still weld.
    Inside,
    /// Moved one and a half tolerances along one axis: must stay apart.
    Outside,
}

fn build(u: &mut Unstructured<'_>) -> arbitrary::Result<(TriangleMesh, f64, bool)> {
    let tolerance = [1e-7, 1e-3, 0.3, 0.0, -1.0][u.choose_index(5)?];
    let vertex_count = u.int_in_range(0..=MAX_VERTICES)?;
    let offset = match Offset::arbitrary(u)? {
        Offset::Origin => 0.0,
        Offset::Far => 1e9,
        Offset::VeryFar => 1e15,
        Offset::NonFinite => f64::NAN,
    };
    // Vertices draw from a small shared pool, so coincident and
    // near-tolerance corners are the common case rather than a 256^3 lottery.
    let pool_size = u.int_in_range(1..=MAX_POOL)?;
    let pool = (0..pool_size)
        .map(|_| {
            let [x, y, z]: [i8; 3] = u.arbitrary()?;
            Ok([
                offset + f64::from(x) * 0.5,
                f64::from(y) * 0.5,
                f64::from(z) * 0.5,
            ])
        })
        .collect::<arbitrary::Result<Vec<_>>>()?;
    let mut positions = Vec::with_capacity(vertex_count);
    for _ in 0..vertex_count {
        let mut point = pool[u.choose_index(pool.len())?];
        let step = match Placement::arbitrary(u)? {
            Placement::Copy => 0.0,
            Placement::Inside => 0.5 * tolerance,
            Placement::Outside => 1.5 * tolerance,
        };
        let sign = if u.arbitrary::<bool>()? { 1.0 } else { -1.0 };
        point[u.choose_index(3)?] += sign * step;
        positions.push(Point3::new(point[0], point[1], point[2]));
    }
    let with_normals = u.arbitrary::<bool>()?;
    let normals = if with_normals {
        (0..vertex_count)
            .map(|_| {
                let [x, y, z]: [i8; 3] = u.arbitrary()?;
                Ok(Vec3::new(f64::from(x), f64::from(y), f64::from(z)))
            })
            .collect::<arbitrary::Result<_>>()?
    } else {
        Vec::new()
    };
    let triangle_count = u.int_in_range(0..=MAX_TRIANGLES)?;
    let mut indices = Vec::with_capacity(triangle_count * 3);
    let mut in_range = true;
    for _ in 0..triangle_count * 3 {
        let raw: u8 = u.arbitrary()?;
        // Mostly valid references, with a sliver of out-of-range ones.
        let index = if raw == u8::MAX || vertex_count == 0 {
            in_range = false;
            u32::try_from(vertex_count).unwrap_or(u32::MAX)
        } else {
            u32::from(raw) % u32::try_from(vertex_count).unwrap_or(1)
        };
        indices.push(index);
    }
    let decidable_refusal = !in_range || tolerance <= 0.0 || offset.is_nan() || indices.len() < 3;
    Ok((
        TriangleMesh {
            positions,
            normals,
            indices,
        },
        tolerance,
        decidable_refusal,
    ))
}

/// The weld keeps the first vertex within tolerance of each corner and
/// creates one only when none exists, so no two vertices it created may lie
/// within tolerance of each other. Exact duplicates and corners moved half a
/// tolerance must therefore have merged.
fn assert_welded(topo: &Topology, tolerance: f64) {
    let tol_sq = tolerance * tolerance;
    let points: Vec<_> = topo.vertices().iter().map(|(_, v)| v.point()).collect();
    for (i, a) in points.iter().enumerate() {
        for b in &points[i + 1..] {
            // The weld's own distance expression, so rounding cannot differ.
            let (ex, ey, ez) = (a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
            assert!(
                ex.mul_add(ex, ey.mul_add(ey, ez * ez)) >= tol_sq,
                "vertices {a:?} and {b:?} lie within the weld tolerance {tolerance}"
            );
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let mut u = Unstructured::new(data);
    let Ok((mesh, tolerance, decidable_refusal)) = build(&mut u) else {
        return;
    };
    let mut topo = Topology::new();
    let result = remus_io::stl::import_mesh(&mut topo, &mesh, tolerance);
    assert_welded(&topo, tolerance);
    match result {
        Ok(solid) => {
            assert!(!decidable_refusal, "invalid mesh input was accepted");
            let faces = remus_topology::explorer::solid_faces(&topo, solid).expect("solid faces");
            assert!(!faces.is_empty());
            assert!(
                faces.len() <= mesh.indices.len() / 3,
                "more faces than triangles"
            );
            let mut uses = std::collections::BTreeMap::<usize, usize>::new();
            for face in faces {
                let face = topo.face(face).expect("face resolves");
                let FaceSurface::Plane { normal, d } = face.surface() else {
                    panic!("imported face is not planar");
                };
                assert!(normal.0.iter().chain([d]).all(|v| v.is_finite()));
                let wire = topo.wire(face.outer_wire()).expect("wire resolves");
                assert_eq!(wire.edges().len(), 3, "imported face is not a triangle");
                for oriented in wire.edges() {
                    *uses.entry(oriented.edge().index()).or_default() += 1;
                }
            }
            assert!(uses.values().all(|&count| count <= 2), "over-shared edge");
        }
        Err(error) => {
            if decidable_refusal {
                assert_eq!(
                    (
                        topo.num_vertices(),
                        topo.num_edges(),
                        topo.num_wires(),
                        topo.num_loops(),
                        topo.num_coedges(),
                        topo.num_faces(),
                        topo.num_shells(),
                        topo.num_solids(),
                    ),
                    (0, 0, 0, 0, 0, 0, 0, 0),
                    "input-validation refusal allocated topology"
                );
            }
            // An over-shared edge and an all-degenerate mesh are refused after
            // welding but before any edge, face, shell or solid exists (welded
            // vertices are allocated first, which import_mesh documents).
            let message = error.to_string();
            if message.contains("non-manifold mesh edge") || message.contains("no valid triangles")
            {
                assert_eq!(
                    (
                        topo.num_edges(),
                        topo.num_wires(),
                        topo.num_faces(),
                        topo.num_shells(),
                        topo.num_solids(),
                        topo.num_loops(),
                        topo.num_coedges(),
                    ),
                    (0, 0, 0, 0, 0, 0, 0),
                    "post-weld refusal allocated topology: {message}"
                );
            }
        }
    }
});
