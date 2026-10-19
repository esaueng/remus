//! Exact content digests of single faces.
//!
//! A consumer that re-reads a body after an edit, as a drag preview does on
//! every frame, needs to know which faces are unchanged so that it can keep
//! what it derived from them. Handles cannot tell it: an edit returns a new
//! body, so unchanged faces get new handles too.
//!
//! The digest hashes the face's complete content: its reversal, every field
//! of its support surface, and the whole boundary the NURBS-area memo keys
//! (every wire, its closed flag and oriented edges, each edge's curve, trim,
//! tolerance and vertex positions, and each edge use's pcurve), plus the
//! authoritative loops and coedges with their periodic windings, each wire's
//! body class and each vertex's tolerance: every field of the face and the
//! entities under it. Handles enter only as first-occurrence ordinals, plus
//! the relative arena order of the face's edges, which the face mesher reads.
//! Reals enter by their bits, so there is no tolerance: any change to the
//! face's content changes the digest. Repeating an edit from the same source repeats the digests of
//! the faces it leaves untouched. A copy can still change a face's digest
//! when it allocates the face's edges in a different relative order.
//!
//! Two independent 64-bit hashes of the content form a 128-bit digest. It is
//! a probabilistic identity for trusted kernel geometry, not a cryptographic
//! one: equal digests mean equal content except with negligible probability.

use std::hash::Hasher;

use remus_math::det_hash::DetHasher;
use remus_topology::Topology;
use remus_topology::face::FaceId;

use super::nurbs_area_memo::face_content_words;
use crate::OperationsError;

/// A face's 128-bit exact content digest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FaceDigest(pub [u64; 2]);

impl FaceDigest {
    /// Lower-case hex, 32 characters.
    #[must_use]
    pub fn to_hex(self) -> String {
        format!("{:016x}{:016x}", self.0[0], self.0[1])
    }
}

/// The exact content digest of `face`; see the module docs.
///
/// # Errors
///
/// Returns an error when the face or an entity of its boundary is missing.
pub fn face_content_digest(topo: &Topology, face: FaceId) -> Result<FaceDigest, OperationsError> {
    topo.face(face)?;
    let words = face_content_words(topo, face).ok_or_else(|| OperationsError::InvalidInput {
        reason: format!("face {} has missing boundary entities", face.index()),
    })?;
    let mut fnv = DetHasher::new();
    // SplitMix64 over whole words, seeded apart from FNV-1a's byte stream.
    let mut mixed = 0x243f_6a88_85a3_08d3_u64 ^ u64::try_from(words.len()).unwrap_or(u64::MAX);
    for &word in &words {
        fnv.write(&word.to_le_bytes());
        mixed = splitmix64(mixed ^ word);
    }
    Ok(FaceDigest([fnv.finish(), mixed]))
}

const fn splitmix64(state: u64) -> u64 {
    let mut z = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use remus_math::mat::Mat4;
    use remus_math::vec::Vec3;
    use remus_topology::Topology;
    use remus_topology::coedge::PeriodicWinding;
    use remus_topology::explorer::solid_faces;
    use remus_topology::face::FaceId;
    use remus_topology::solid::SolidId;

    use super::{FaceDigest, face_content_digest};
    use crate::{copy, primitives, push_pull, transform};

    fn digests(topo: &Topology, solid: SolidId) -> Vec<FaceDigest> {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .map(|face| face_content_digest(topo, face).unwrap())
            .collect()
    }

    fn face_with_normal(topo: &Topology, solid: SolidId, normal: Vec3) -> FaceId {
        solid_faces(topo, solid)
            .unwrap()
            .into_iter()
            .find(|&face| {
                topo.face(face)
                    .unwrap()
                    .effective_plane_normal()
                    .is_some_and(|n| n.dot(normal) > 0.999)
            })
            .unwrap()
    }

    fn moved_top(distance: f64) -> (Topology, SolidId, SolidId) {
        let mut topo = Topology::new();
        let body = primitives::make_box(&mut topo, 10.0, 8.0, 6.0).unwrap();
        let top = face_with_normal(&topo, body, Vec3::new(0.0, 0.0, 1.0));
        let moved = push_pull::move_faces(&mut topo, body, &[top], distance).unwrap();
        (topo, body, moved)
    }

    #[test]
    fn repeated_copies_agree_and_faces_differ() {
        let mut topo = Topology::new();
        let body = primitives::make_box(&mut topo, 10.0, 8.0, 6.0).unwrap();
        let first = copy::copy_solid(&mut topo, body).unwrap();
        let second = copy::copy_solid(&mut topo, body).unwrap();
        let digests_of_first = digests(&topo, first);
        assert_eq!(digests(&topo, second), digests_of_first);
        let mut distinct = digests(&topo, body);
        distinct.sort_unstable_by_key(|digest| digest.0);
        distinct.dedup();
        assert_eq!(distinct.len(), 6);
        assert_eq!(digests_of_first[0].to_hex().len(), 32);
    }

    /// The drag-preview case: each tick moves the same source face to a new
    /// distance, and the faces the move does not reshape keep their digests
    /// from tick to tick.
    #[test]
    fn a_face_move_changes_only_the_faces_it_reshapes() {
        let (near, near_body, near_moved) = moved_top(2.5);
        let (far, _, far_moved) = moved_top(3.0);
        let (again, _, again_moved) = moved_top(2.5);
        assert_eq!(digests(&again, again_moved), digests(&near, near_moved));
        let digest_at = |topo: &Topology, solid, normal| {
            face_content_digest(topo, face_with_normal(topo, solid, normal)).unwrap()
        };
        let (top, side, bottom) = (
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
        );
        assert_ne!(
            digest_at(&near, near_moved, top),
            digest_at(&far, far_moved, top)
        );
        assert_ne!(
            digest_at(&near, near_moved, side),
            digest_at(&far, far_moved, side)
        );
        assert_eq!(
            digest_at(&near, near_moved, bottom),
            digest_at(&far, far_moved, bottom),
            "the untouched bottom kept its digest"
        );
        assert_ne!(
            digest_at(&near, near_body, top),
            digest_at(&near, near_moved, top),
            "the moved face changed"
        );
    }

    #[test]
    fn every_surface_type_is_digested_and_placement_matters() {
        let mut topo = Topology::new();
        let bodies = [
            primitives::make_cylinder(&mut topo, 2.0, 5.0).unwrap(),
            primitives::make_cone(&mut topo, 2.5, 1.0, 4.0).unwrap(),
            primitives::make_sphere(&mut topo, 3.0, 16).unwrap(),
            primitives::make_torus(&mut topo, 4.0, 1.0, 16).unwrap(),
        ];
        // Distinct radii: two equal caps would rightly share a digest.
        let mut all = Vec::new();
        for &body in &bodies {
            all.extend(digests(&topo, body));
        }
        let shifted = copy::copy_solid(&mut topo, bodies[0]).unwrap();
        transform::transform_solid(&mut topo, shifted, &Mat4::translation(1e-9, 0.0, 0.0)).unwrap();
        let moved = digests(&topo, shifted);
        assert!(moved.iter().all(|digest| !all.contains(digest)));
        let count = all.len();
        all.sort_unstable_by_key(|digest| digest.0);
        all.dedup();
        assert_eq!(all.len(), count);
    }

    /// A vertex tolerance is face content even though the mesher ignores it.
    #[test]
    fn vertex_tolerance_is_digested() {
        let mut topo = Topology::new();
        let body = primitives::make_box(&mut topo, 10.0, 8.0, 6.0).unwrap();
        let top = face_with_normal(&topo, body, Vec3::new(0.0, 0.0, 1.0));
        let before = face_content_digest(&topo, top).unwrap();
        let wire = topo.face(top).unwrap().outer_wire();
        let edge = topo.wire(wire).unwrap().edges()[0].edge();
        let vertex = topo.edge(edge).unwrap().start();
        let original = topo.vertex(vertex).unwrap().tolerance();
        topo.vertex_mut(vertex)
            .unwrap()
            .set_tolerance(original * 2.0 + 1e-7)
            .unwrap();
        assert_ne!(face_content_digest(&topo, top).unwrap(), before);
    }

    /// Tripwire for the digest's completeness: every entity it reads is
    /// pinned at its current size, so adding a field fails here until
    /// `face_content_words` (and this pin) learn about it.
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn digest_reads_every_entity_field() {
        use std::mem::size_of;
        assert_eq!(
            [
                size_of::<remus_topology::vertex::Vertex>(),
                size_of::<remus_topology::edge::Edge>(),
                size_of::<remus_topology::wire::Wire>(),
                size_of::<remus_topology::face::Face>(),
                size_of::<remus_topology::Loop>(),
                size_of::<remus_topology::Coedge>(),
                size_of::<remus_math::surfaces::CylindricalSurface>(),
                size_of::<remus_math::surfaces::ConicalSurface>(),
                size_of::<remus_math::surfaces::SphericalSurface>(),
                size_of::<remus_math::surfaces::ToroidalSurface>(),
            ],
            [32, 176, 32, 184, 40, 128, 104, 104, 104, 112]
        );
    }

    /// A coedge's periodic winding is face content: changing only that
    /// branch changes the digest, and restoring it restores the digest.
    #[test]
    fn periodic_winding_is_digested() {
        let mut topo = Topology::new();
        let body = primitives::make_cylinder(&mut topo, 2.0, 5.0).unwrap();
        let lateral = solid_faces(&topo, body)
            .unwrap()
            .into_iter()
            .find(|&face| topo.face(face).unwrap().effective_plane_normal().is_none())
            .unwrap();
        let before = face_content_digest(&topo, lateral).unwrap();
        let loop_id = topo.face(lateral).unwrap().boundary_loops()[0];
        let coedge = topo.face_loop(loop_id).unwrap().coedges()[0];
        let current = topo.coedge(coedge).unwrap().periodic_winding();
        let shifted = PeriodicWinding::new(current.u() + 1, current.v());
        let original = topo.set_coedge_periodic_winding(coedge, shifted).unwrap();
        assert_ne!(face_content_digest(&topo, lateral).unwrap(), before);
        topo.set_coedge_periodic_winding(coedge, original).unwrap();
        assert_eq!(face_content_digest(&topo, lateral).unwrap(), before);
    }
}
