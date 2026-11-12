#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use remus_math::curves::{Circle3D, Ellipse3D, Hyperbola3D, Parabola3D};
use remus_math::curves2d::{Circle2D, Curve2D, Ellipse2D, Line2D, NurbsCurve2D};
use remus_math::nurbs::curve::NurbsCurve;
use remus_math::nurbs::surface::NurbsSurface;
use remus_math::surfaces::{ConicalSurface, CylindricalSurface, SphericalSurface, ToroidalSurface};
use remus_math::vec::{Point2, Point3, Vec2, Vec3};
use remus_topology::Topology;
use remus_topology::attributes::{ColorRgb, EntityAttributes};
use remus_topology::edge::{Edge, EdgeCurve};
use remus_topology::face::{Face, FaceSurface};
use remus_topology::journal::{EntityKey, EventDraft, EvolutionDraft};
use remus_topology::pcurve::PCurve;
use remus_topology::shell::Shell;
use remus_topology::solid::Solid;
use remus_topology::vertex::Vertex;
use remus_topology::wire::{OrientedEdge, Wire};

use super::*;

const BUILD: &str = "remus-test/1";

fn curves() -> Vec<EdgeCurve> {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let o = Point3::new(0.1, 0.2, 0.3);
    vec![
        EdgeCurve::Line,
        EdgeCurve::NurbsCurve(
            NurbsCurve::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![
                    o,
                    Point3::new(1.0, 2.0, 0.5),
                    Point3::new(3.0, 0.0, 1.0 / 3.0),
                ],
                vec![1.0, 0.7, 1.0],
            )
            .unwrap(),
        ),
        EdgeCurve::Circle(Circle3D::new(o, z, 1.25).unwrap()),
        EdgeCurve::Ellipse(Ellipse3D::new(o, z, 2.0, 1.0 / 7.0).unwrap()),
        EdgeCurve::Hyperbola(Hyperbola3D::new(o, z, 1.5, 0.5).unwrap()),
        EdgeCurve::Parabola(Parabola3D::new(o, z, 0.3).unwrap()),
    ]
}

fn surfaces() -> Vec<FaceSurface> {
    let z = Vec3::new(0.0, 0.0, 1.0);
    let o = Point3::new(-1.0, 0.5, 1e-9);
    let row = |x: f64| vec![Point3::new(x, 0.0, 0.1 * x), Point3::new(x, 1.0, 0.3)];
    vec![
        FaceSurface::Plane {
            normal: z,
            d: -0.25,
        },
        FaceSurface::Nurbs(
            NurbsSurface::new(
                2,
                1,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![0.0, 0.0, 1.0, 1.0],
                vec![row(0.0), row(1.0), row(2.0)],
                vec![vec![1.0, 1.0], vec![0.8, 0.8], vec![1.0, 1.0]],
            )
            .unwrap(),
        ),
        FaceSurface::Cylinder(CylindricalSurface::new(o, z, 0.5).unwrap()),
        FaceSurface::Cone(ConicalSurface::new(o, z, 0.4).unwrap()),
        FaceSurface::Sphere(SphericalSurface::new(o, 3.0).unwrap()),
        FaceSurface::Torus(ToroidalSurface::new(o, 4.0, 1.0).unwrap()),
    ]
}

fn pcurves() -> Vec<Curve2D> {
    vec![
        Curve2D::Line(Line2D::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.5)).unwrap()),
        Curve2D::Circle(Circle2D::new(Point2::new(0.5, 0.5), 0.25).unwrap()),
        Curve2D::Ellipse(Ellipse2D::new(Point2::new(0.5, 0.5), 0.5, 0.25, 0.1).unwrap()),
        Curve2D::Nurbs(
            NurbsCurve2D::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)],
                vec![1.0, 1.0],
            )
            .unwrap(),
        ),
    ]
}

/// A session holding every carrier, attributes, journal history, retired
/// tombstones from a checkpoint restore and an unjournaled mutation.
fn session() -> Topology {
    let mut topo = Topology::new();
    let curves = curves();
    let pcurves = pcurves();
    let mut faces = Vec::new();
    for (index, surface) in surfaces().into_iter().enumerate() {
        let z = index as f64;
        let points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let vertices: Vec<_> = points
            .iter()
            .map(|&(x, y)| topo.add_vertex(Vertex::new(Point3::new(x, y, z), 1e-7)))
            .collect();
        let edges: Vec<_> = (0..4)
            .map(|i| {
                let curve = curves[(index + i) % curves.len()].clone();
                let mut edge =
                    Edge::with_tolerance(vertices[i], vertices[(i + 1) % 4], curve, Some(1e-6));
                if i % 2 == 0 {
                    edge.set_trim(Some((0.125, 0.875)));
                }
                topo.add_edge(edge)
            })
            .collect();
        let wire = topo.add_wire(
            Wire::new(
                edges.iter().map(|&e| OrientedEdge::new(e, true)).collect(),
                true,
            )
            .unwrap(),
        );
        let face = if index % 2 == 0 {
            Face::new(wire, vec![], surface)
        } else {
            Face::new_reversed(wire, vec![], surface)
        };
        let face = topo.add_face(face);
        for (i, &edge) in edges.iter().enumerate() {
            let curve = pcurves[(index + i) % pcurves.len()].clone();
            topo.set_pcurve(edge, face, PCurve::new(curve, 0.0, 1.0))
                .unwrap();
        }
        faces.push(face);
    }
    let shell = topo.add_shell(Shell::new(faces.clone()).unwrap());
    let solid = topo.add_solid(Solid::new(shell, vec![]));
    let pending = topo.journal_begin("build");
    let mut draft = EvolutionDraft::construction();
    for face in &faces {
        draft.push(
            EntityKey::face(face.index()),
            EventDraft::Generated { sources: vec![] },
        );
    }
    topo.journal_record_evolution(pending, draft).unwrap();
    topo.set_face_attributes(
        faces[1],
        EntityAttributes {
            name: Some("fläche ✓".into()),
            color: Some(ColorRgb::new(0.1, 0.2, 0.3).unwrap()),
        },
    )
    .unwrap();
    topo.set_solid_attributes(
        solid,
        EntityAttributes {
            name: None,
            color: Some(ColorRgb::new(1.0, 0.0, 0.5).unwrap()),
        },
    )
    .unwrap();
    let checkpoint = topo.clone();
    let stray = topo.add_vertex(Vertex::new(Point3::new(7.0, 7.0, 7.0), 1e-7));
    let _ = topo.add_edge(Edge::new(stray, stray, curves[2].clone()));
    topo.restore_preserving_handle_slots(&checkpoint);
    let pending = topo.journal_begin("split");
    let mut draft = EvolutionDraft::geometry();
    draft.push(
        EntityKey::face(faces[0].index()),
        EventDraft::Modified {
            from: EntityKey::face(faces[0].index()),
        },
    );
    draft.push(
        EntityKey::face(faces[2].index()),
        EventDraft::Merged {
            from: vec![
                EntityKey::face(faces[2].index()),
                EntityKey::face(faces[3].index()),
            ],
        },
    );
    topo.journal_record_evolution(pending, draft).unwrap();
    topo.add_vertex(Vertex::new(Point3::new(-1.0, -1.0, -1.0), 1e-7));
    topo
}

/// `Debug` text with the contents of lazily filled carrier caches
/// (`OnceLock`s, filled on first evaluation) dropped.
fn without_lazy_caches(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(at) = rest.find("OnceLock(") {
        out.push_str(&rest[..at + "OnceLock(".len()]);
        rest = &rest[at + "OnceLock(".len()..];
        let mut depth = 1;
        let end = rest
            .char_indices()
            .find(|&(_, c)| {
                depth += match c {
                    '(' => 1,
                    ')' => -1,
                    _ => 0,
                };
                depth == 0
            })
            .map_or(rest.len(), |(index, _)| index);
        rest = &rest[end..];
    }
    out.push_str(rest);
    out
}

fn write(topo: &Topology) -> Vec<u8> {
    write_snapshot(topo, BUILD, &[3, 0, u64::MAX], ImportLimits::default()).unwrap()
}

fn read(bytes: &[u8]) -> Result<SessionSnapshot, SnapshotError> {
    read_snapshot(bytes, BUILD, ImportLimits::default())
}

/// Re-seals edited bytes with a valid digest, as a hostile writer would.
fn reseal(bytes: &mut [u8]) {
    let content = bytes.len() - DIGEST_BYTES;
    let digest = digest_of(&bytes[..content]);
    bytes[content..content + 8].copy_from_slice(&digest.0[0].to_le_bytes());
    bytes[content + 8..].copy_from_slice(&digest.0[1].to_le_bytes());
}

fn payload_start(bytes: &[u8]) -> usize {
    13 + usize::from(bytes[12]) + 8
}

#[test]
fn round_trip_is_byte_exact_and_deterministic() {
    let topo = session();
    let bytes = write(&topo);
    assert_eq!(bytes, write(&topo), "writes are deterministic");
    // A clone starts a fresh cache lineage; the snapshot must not see it.
    #[allow(clippy::redundant_clone)]
    let copy = topo.clone();
    assert_eq!(bytes, write(&copy), "a clone writes the same bytes");

    let restored = read(&bytes).unwrap();
    assert_eq!(restored.session, vec![3, 0, u64::MAX]);
    assert_eq!(write(&restored.topology), bytes);
    assert_eq!(
        without_lazy_caches(&format!("{:?}", restored.topology.to_image())),
        without_lazy_caches(&format!("{:?}", topo.to_image()))
    );
    assert_eq!(snapshot_digest(&bytes).unwrap().to_hex().len(), 32);
    assert_eq!(snapshot_build(&bytes).unwrap(), BUILD);

    // Repeated NURBS carriers are written once and share storage again.
    let shared: Vec<_> = restored
        .topology
        .edges()
        .iter()
        .filter_map(|(_, edge)| match edge.curve() {
            EdgeCurve::NurbsCurve(curve) => Some(curve.knots().as_ptr()),
            _ => None,
        })
        .collect();
    assert!(shared.len() > 1);
    assert!(shared.iter().all(|&knots| knots == shared[0]));

    // Every geometry carrier evaluates bit for bit the same.
    for ((_, original), (_, copy)) in topo.edges().iter().zip(restored.topology.edges().iter()) {
        let (a, b) = (original.curve(), copy.curve());
        for t in [0.0, 0.3, 0.7] {
            let ends = (Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
            assert_eq!(
                a.evaluate_with_endpoints(t, ends.0, ends.1)
                    .0
                    .map(f64::to_bits),
                b.evaluate_with_endpoints(t, ends.0, ends.1)
                    .0
                    .map(f64::to_bits)
            );
        }
    }
}

#[test]
fn framing_refusals_are_typed() {
    let bytes = write(&session());

    assert_eq!(
        read(b"not a snapshot").unwrap_err(),
        SnapshotError::NotASnapshot
    );
    assert_eq!(read(&[]).unwrap_err(), SnapshotError::NotASnapshot);

    let mut other = bytes.clone();
    other[8] = 2;
    assert_eq!(
        read(&other).unwrap_err(),
        SnapshotError::UnsupportedFormat {
            found: 2,
            expected: SNAPSHOT_FORMAT_VERSION
        }
    );

    let error = read_snapshot(&bytes, "remus-test/2", ImportLimits::default()).unwrap_err();
    assert_eq!(error.code(), "build_mismatch");

    for len in [
        9,
        12,
        13,
        20,
        payload_start(&bytes),
        bytes.len() / 2,
        bytes.len() - 1,
    ] {
        let error = read(&bytes[..len]).unwrap_err();
        assert_eq!(error.code(), "corrupt", "truncated to {len}: {error}");
    }
    let mut extended = bytes.clone();
    extended.push(0);
    assert_eq!(read(&extended).unwrap_err().code(), "corrupt");

    for position in [payload_start(&bytes), bytes.len() / 2, bytes.len() - 1] {
        let mut flipped = bytes.clone();
        flipped[position] ^= 0x10;
        let error = read(&flipped).unwrap_err();
        assert_eq!(
            error,
            SnapshotError::corrupt("digest mismatch"),
            "at {position}"
        );
        assert!(snapshot_digest(&flipped).is_err());
    }
}

#[test]
fn limits_apply_to_both_sides() {
    let topo = session();
    let bytes = write(&topo);
    let small_bytes = ImportLimits {
        max_input_bytes: bytes.len() - 1,
        ..ImportLimits::default()
    };
    assert_eq!(
        read_snapshot(&bytes, BUILD, small_bytes)
            .unwrap_err()
            .code(),
        "limit_exceeded"
    );
    assert_eq!(
        write_snapshot(&topo, BUILD, &[3, 0, u64::MAX], small_bytes)
            .unwrap_err()
            .code(),
        "limit_exceeded"
    );
    let few_entities = ImportLimits {
        max_model_entities: 10,
        ..ImportLimits::default()
    };
    assert_eq!(
        read_snapshot(&bytes, BUILD, few_entities)
            .unwrap_err()
            .code(),
        "limit_exceeded"
    );
    assert_eq!(
        write_snapshot(&topo, BUILD, &[], few_entities)
            .unwrap_err()
            .code(),
        "limit_exceeded"
    );
    for build in ["", &"x".repeat(256)] {
        assert_eq!(
            write_snapshot(&topo, build, &[], ImportLimits::default())
                .unwrap_err()
                .code(),
            "invalid_request"
        );
    }
    assert_eq!(
        write_snapshot(&topo, BUILD, &[0; 65], ImportLimits::default())
            .unwrap_err()
            .code(),
        "invalid_request"
    );
}

/// A hostile writer can re-seal any edit with a valid digest. Every
/// single-byte edit of the payload, and every payload truncation, must
/// decode to a typed refusal or a valid topology, never a panic.
#[test]
fn resealed_edits_never_panic() {
    let bytes = write(&session());
    let start = payload_start(&bytes);
    let end = bytes.len() - DIGEST_BYTES;
    let (mut refused, mut accepted) = (0, 0);
    for position in start..end {
        for mask in [0x01, 0xff] {
            let mut edited = bytes.clone();
            edited[position] ^= mask;
            reseal(&mut edited);
            match read(&edited) {
                Ok(restored) => {
                    accepted += 1;
                    // Whatever was accepted is a usable topology.
                    drop(write(&restored.topology));
                }
                Err(error) => {
                    refused += 1;
                    assert!(
                        matches!(error.code(), "corrupt" | "invalid" | "limit_exceeded"),
                        "{error}"
                    );
                }
            }
        }
    }
    assert!(
        refused > 0 && accepted > 0,
        "{refused} refused, {accepted} accepted"
    );

    for cut in (start..end).step_by(7) {
        let mut truncated = bytes[..cut].to_vec();
        truncated.extend_from_slice(&[0; DIGEST_BYTES]);
        let len = (cut - start) as u64;
        truncated[start - 8..start].copy_from_slice(&len.to_le_bytes());
        reseal(&mut truncated);
        assert_eq!(read(&truncated).unwrap_err().code(), "corrupt");
    }
}
