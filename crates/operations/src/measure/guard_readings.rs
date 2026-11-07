//! Prepared-handle ownership of the existing per-face volume integrals.
//!
//! Warm both independent Gauss orders on the fixed source and sharp parent
//! before a drag. During acceptance only changed faces need integrating.
//! Summation, boundary qualification, closure and order-agreement checks stay
//! in `volume`; this stores terms, never an inferred body-volume delta.

use std::cell::RefCell;
use std::sync::{Arc, Mutex, PoisonError};

use remus_check::properties::face_cache::FaceVolumeTerm;
use remus_math::vec::Point3;
use remus_topology::{FaceId, SolidId, Topology};

use super::volume_memo::{ContentKey, face_content_key};

const BYTE_BUDGET: usize = 4 * 1024 * 1024;

struct Entry {
    key: ContentKey,
    order: usize,
    reference: [u64; 3],
    term: FaceVolumeTerm,
}

#[derive(Default)]
struct Terms {
    entries: Vec<Entry>,
    bytes: usize,
}

pub struct GuardReadings(Mutex<Terms>);

thread_local! {
    static ACTIVE: RefCell<Option<Arc<GuardReadings>>> = const { RefCell::new(None) };
}

impl GuardReadings {
    pub fn new() -> Arc<Self> {
        Arc::new(Self(Mutex::new(Terms::default())))
    }

    pub fn enter<T>(self: &Arc<Self>, run: impl FnOnce() -> T) -> T {
        struct Restore(Option<Arc<GuardReadings>>);
        impl Drop for Restore {
            fn drop(&mut self) {
                ACTIVE.with(|slot| *slot.borrow_mut() = self.0.take());
            }
        }
        let _restore = Restore(ACTIVE.with(|slot| slot.replace(Some(self.clone()))));
        run()
    }

    /// Opportunistic: an unmeasurable source keeps the ordinary guard's error
    /// semantics. Failures are never cached or turned into a certificate.
    pub fn prime(self: &Arc<Self>, topo: &Topology, solid: SolidId) {
        let _span = crate::performance::span("blend_group.prepare_integrals");
        self.enter(|| super::volume::prime_guard_integrals(topo, solid));
    }
}

pub(super) fn integrate(
    topo: &Topology,
    face: FaceId,
    order: usize,
    reference: Point3,
) -> Result<FaceVolumeTerm, remus_check::CheckError> {
    let owner = ACTIVE.with(|slot| slot.borrow().clone());
    let reference_bits = reference.0.map(f64::to_bits);
    let key = owner
        .as_ref()
        .and_then(|_| face_content_key(topo, face, 0.0, BYTE_BUDGET));
    if let (Some(owner), Some(key)) = (&owner, &key) {
        let terms = owner.0.lock().unwrap_or_else(PoisonError::into_inner);
        if let Some(entry) = terms
            .entries
            .iter()
            .find(|e| e.order == order && e.reference == reference_bits && e.key == *key)
        {
            return Ok(entry.term);
        }
    }
    // Consult the existing integrator cache on a miss: in WASM it may already
    // hold the validator's order-5 reading of a newly built preview face.
    let term = match crate::performance::timed("volume.ruled-attempt", || {
        remus_check::properties::face_integrator::integrate_ruled_guard_volume_about(
            topo, face, order, reference,
        )
    }) {
        Ok(Some(term)) => {
            let _span = crate::performance::span("volume.ruled-certified");
            FaceVolumeTerm {
                area: term.closure_area_lower_bound,
                volume: term.volume,
                vector_area: term.vector_area,
            }
        }
        declined => {
            if declined.is_err() {
                let _span = crate::performance::span("volume.ruled-fallback");
            }
            remus_check::properties::face_cache::integrate_face_volume_about_memoized(
                topo, face, order, reference,
            )?
        }
    };
    if let (Some(owner), Some(key)) = (owner, key) {
        let mut terms = owner.0.lock().unwrap_or_else(PoisonError::into_inner);
        let bytes = key.bytes() + std::mem::size_of::<Entry>();
        if terms.bytes + bytes <= BYTE_BUDGET && terms.entries.len() < 4096 {
            terms.bytes += bytes;
            terms.entries.push(Entry {
                key,
                order,
                reference: reference_bits,
                term,
            });
        }
    }
    Ok(term)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn source_terms_survive_restore_but_changed_geometry_is_measured() {
        let mut topo = Topology::new();
        let solid = crate::primitives::make_box(&mut topo, 2.0, 3.0, 4.0).unwrap();
        let face = remus_topology::explorer::solid_faces(&topo, solid).unwrap()[0];
        let owner = GuardReadings::new();
        owner.prime(&topo, solid);
        let count = owner.0.lock().unwrap().entries.len();
        assert_eq!(count, 12);
        let reference = Point3::new(1.0, 1.5, 2.0);
        let original = owner
            .enter(|| integrate(&topo, face, 3, reference))
            .unwrap();
        let restored = topo.clone();
        assert_eq!(
            owner
                .enter(|| integrate(&restored, face, 3, reference))
                .unwrap(),
            original
        );
        assert_eq!(owner.0.lock().unwrap().entries.len(), count);

        let wire = topo.face(face).unwrap().outer_wire();
        let edge = topo.wire(wire).unwrap().edges()[0].edge();
        let vertex = topo.edge(edge).unwrap().start();
        let point = topo.vertex(vertex).unwrap().point();
        topo.vertex_mut(vertex)
            .unwrap()
            .set_point(point + remus_math::vec::Vec3::new(0.2, 0.3, 0.4));
        let measured = owner
            .enter(|| integrate(&topo, face, 3, reference))
            .unwrap();
        let fresh = integrate(&topo, face, 3, reference).unwrap();
        assert_eq!(measured, fresh);
        assert_eq!(owner.0.lock().unwrap().entries.len(), count + 1);
        assert_eq!(
            owner
                .enter(|| integrate(&restored, face, 3, reference))
                .unwrap(),
            original
        );
    }
}
