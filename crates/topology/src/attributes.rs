//! Topology attribute store — semantic names and display colors
//! (Issue 14; design in `docs/design/deferred-e3b-step-names-and-colors.md`).
//!
//! Attributes are public model data, not rendering hints: a face's color and
//! a solid's name survive modeling operations under the explicit propagation
//! rules implemented in `remus_operations::evolution` (driven by
//! construction-derived evolution events — attributes are never rebound by
//! geometric guessing).
//!
//! Storage is relational (the pcurve-registry pattern): entities are not
//! enlarged, and the store is keyed by typed handles. Scope v1: solids and
//! faces. An unset face color inherits the containing solid's color at
//! presentation time; the store itself records only explicit assignments.
//!
//! Colors are sRGB with channels in `[0, 1]` (the STEP `COLOUR_RGB` value
//! range); constructors refuse non-finite or out-of-range channels.

use std::collections::HashMap;

use crate::face::FaceId;
use crate::solid::SolidId;

/// An sRGB color with channels in `[0, 1]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ColorRgb {
    /// Red channel, `0..=1`.
    r: f64,
    /// Green channel, `0..=1`.
    g: f64,
    /// Blue channel, `0..=1`.
    b: f64,
}

impl ColorRgb {
    /// Creates a color, refusing non-finite or out-of-range channels.
    ///
    /// # Errors
    ///
    /// Returns [`TopologyError::InvalidColorChannel`](crate::TopologyError::InvalidColorChannel)
    /// naming the offending channel and value.
    pub fn new(r: f64, g: f64, b: f64) -> Result<Self, crate::TopologyError> {
        for (channel, value) in [("r", r), ("g", g), ("b", b)] {
            if !value.is_finite() || !(0.0..=1.0).contains(&value) {
                return Err(crate::TopologyError::InvalidColorChannel { channel, value });
            }
        }
        Ok(Self { r, g, b })
    }

    /// Red channel, in `0..=1`.
    #[must_use]
    pub fn r(self) -> f64 {
        self.r
    }

    /// Green channel, in `0..=1`.
    #[must_use]
    pub fn g(self) -> f64 {
        self.g
    }

    /// Blue channel, in `0..=1`.
    #[must_use]
    pub fn b(self) -> f64 {
        self.b
    }
}

/// The attributes one entity can carry (all optional; unset means absent,
/// never a default).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct EntityAttributes {
    /// Semantic name. Application vocabulary; the kernel never synthesizes,
    /// concatenates, or suffixes names.
    pub name: Option<String>,
    /// Display color (sRGB, `[0, 1]` channels).
    pub color: Option<ColorRgb>,
}

impl EntityAttributes {
    /// True when no attribute is set (such entries are not stored).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.name.is_none() && self.color.is_none()
    }
}

/// Relational attribute store, owned by [`Topology`](crate::Topology).
#[derive(Debug, Default, Clone)]
pub struct AttributeStore {
    solids: HashMap<SolidId, EntityAttributes>,
    faces: HashMap<FaceId, EntityAttributes>,
}

impl AttributeStore {
    /// Creates an empty store.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The attributes of a solid, if any are set.
    #[must_use]
    pub fn solid(&self, id: SolidId) -> Option<&EntityAttributes> {
        self.solids.get(&id)
    }

    /// The attributes of a face, if any are set.
    #[must_use]
    pub fn face(&self, id: FaceId) -> Option<&EntityAttributes> {
        self.faces.get(&id)
    }

    /// Sets (or clears, when empty) a solid's attributes.
    pub fn set_solid(&mut self, id: SolidId, attributes: EntityAttributes) {
        if attributes.is_empty() {
            self.solids.remove(&id);
        } else {
            self.solids.insert(id, attributes);
        }
    }

    /// Sets (or clears, when empty) a face's attributes.
    pub fn set_face(&mut self, id: FaceId, attributes: EntityAttributes) {
        if attributes.is_empty() {
            self.faces.remove(&id);
        } else {
            self.faces.insert(id, attributes);
        }
    }

    /// Removes a solid's attributes, returning them.
    pub fn remove_solid(&mut self, id: SolidId) -> Option<EntityAttributes> {
        self.solids.remove(&id)
    }

    /// Removes a face's attributes, returning them.
    pub fn remove_face(&mut self, id: FaceId) -> Option<EntityAttributes> {
        self.faces.remove(&id)
    }

    /// Number of entities carrying attributes.
    #[must_use]
    pub fn len(&self) -> usize {
        self.solids.len() + self.faces.len()
    }

    /// True when nothing carries attributes.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.solids.is_empty() && self.faces.is_empty()
    }

    /// All attributed faces, in deterministic (index) order.
    #[must_use]
    pub fn faces_with_attributes(&self) -> Vec<(FaceId, &EntityAttributes)> {
        let mut out: Vec<_> = self.faces.iter().map(|(&id, a)| (id, a)).collect();
        out.sort_by_key(|(id, _)| id.index());
        out
    }

    /// All attributed solids, in deterministic (index) order.
    #[must_use]
    pub fn solids_with_attributes(&self) -> Vec<(SolidId, &EntityAttributes)> {
        let mut out: Vec<_> = self.solids.iter().map(|(&id, a)| (id, a)).collect();
        out.sort_by_key(|(id, _)| id.index());
        out
    }

    /// Removes entries whose entity has been retired.
    pub(crate) fn remove_for_retired_entities(
        &mut self,
        retired_solids: &std::collections::HashSet<SolidId>,
        retired_faces: &std::collections::HashSet<FaceId>,
    ) {
        self.solids.retain(|id, _| !retired_solids.contains(id));
        self.faces.retain(|id, _| !retired_faces.contains(id));
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

    use std::collections::HashSet;

    use remus_math::vec::{Point3, Vec3};

    use super::{AttributeStore, ColorRgb, EntityAttributes};
    use crate::Topology;
    use crate::edge::{Edge, EdgeCurve};
    use crate::face::{Face, FaceId, FaceSurface};
    use crate::shell::Shell;
    use crate::solid::{Solid, SolidId};
    use crate::vertex::Vertex;
    use crate::wire::{OrientedEdge, Wire};

    /// A minimal live face, so the store is keyed by real arena handles
    /// (typed ids cannot be conjured from an index outside the arena).
    fn add_face(topo: &mut Topology) -> FaceId {
        let a = topo.add_vertex(Vertex::new(Point3::new(0.0, 0.0, 0.0), 1e-7));
        let b = topo.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), 1e-7));
        let e = topo.add_edge(Edge::new(a, b, EdgeCurve::Line));
        let w = topo.add_wire(Wire::new(vec![OrientedEdge::new(e, true)], false).unwrap());
        topo.add_face(Face::new(
            w,
            Vec::new(),
            FaceSurface::Plane {
                normal: Vec3::new(0.0, 0.0, 1.0),
                d: 0.0,
            },
        ))
    }

    fn add_solid(topo: &mut Topology) -> SolidId {
        let face = add_face(topo);
        let shell = topo.add_shell(Shell::new(vec![face]).unwrap());
        topo.add_solid(Solid::new(shell, Vec::new()))
    }

    fn named(name: &str) -> EntityAttributes {
        EntityAttributes {
            name: Some(name.to_owned()),
            ..Default::default()
        }
    }

    #[test]
    fn color_channels_are_stored_and_read_back_independently() {
        let color = ColorRgb::new(0.25, 0.5, 0.75).unwrap();
        assert!((color.r() - 0.25).abs() < f64::EPSILON, "r channel");
        assert!((color.g() - 0.5).abs() < f64::EPSILON, "g channel");
        assert!((color.b() - 0.75).abs() < f64::EPSILON, "b channel");
        // No channel may impersonate another.
        assert!(color.r() < color.g() && color.g() < color.b());

        // The documented `[0, 1]` range is enforced, non-finite included.
        assert!(ColorRgb::new(-0.1, 0.0, 0.0).is_err());
        assert!(ColorRgb::new(0.0, 1.5, 0.0).is_err());
        assert!(ColorRgb::new(0.0, 0.0, f64::NAN).is_err());
        // The endpoints themselves are inside the range.
        let black = ColorRgb::new(0.0, 0.0, 0.0).unwrap();
        let white = ColorRgb::new(1.0, 1.0, 1.0).unwrap();
        assert!(black.r() < white.r());
    }

    #[test]
    fn entity_attributes_are_empty_only_when_nothing_is_set() {
        assert!(EntityAttributes::default().is_empty());
        assert!(!named("hub").is_empty(), "a name is an attribute");
        assert!(
            !EntityAttributes {
                name: None,
                color: Some(ColorRgb::new(0.1, 0.2, 0.3).unwrap()),
            }
            .is_empty(),
            "a color alone is an attribute"
        );
    }

    #[test]
    fn set_then_get_round_trips_and_unset_entities_read_none() {
        let mut topo = Topology::new();
        let solid = add_solid(&mut topo);
        let other_solid = add_solid(&mut topo);
        let face = add_face(&mut topo);
        let other_face = add_face(&mut topo);

        let mut store = AttributeStore::new();
        assert!(store.solid(solid).is_none(), "nothing set yet");
        assert!(store.face(face).is_none(), "nothing set yet");

        store.set_solid(solid, named("bracket"));
        store.set_face(face, named("mounting face"));

        assert_eq!(
            store.solid(solid).unwrap().name.as_deref(),
            Some("bracket"),
            "a read must return exactly what was set"
        );
        assert_eq!(
            store.face(face).unwrap().name.as_deref(),
            Some("mounting face")
        );
        // An unqueried neighbour has nothing — the store never invents a
        // default entry for an entity that carries no attributes.
        assert!(store.solid(other_solid).is_none());
        assert!(store.face(other_face).is_none());

        // Overwriting replaces rather than accumulating.
        let color = ColorRgb::new(0.2, 0.4, 0.6).unwrap();
        store.set_solid(
            solid,
            EntityAttributes {
                name: None,
                color: Some(color),
            },
        );
        let stored = store.solid(solid).unwrap();
        assert_eq!(stored.name, None, "an overwrite replaces the whole record");
        assert_eq!(stored.color, Some(color));
        assert_eq!(store.len(), 2, "an overwrite is not a second entry");
    }

    #[test]
    fn setting_empty_attributes_clears_the_entry() {
        let mut topo = Topology::new();
        let solid = add_solid(&mut topo);
        let face = add_face(&mut topo);

        let mut store = AttributeStore::new();
        store.set_solid(solid, named("bracket"));
        store.set_face(face, named("web"));
        assert_eq!(store.len(), 2);

        store.set_solid(solid, EntityAttributes::default());
        store.set_face(face, EntityAttributes::default());
        assert!(store.solid(solid).is_none());
        assert!(store.face(face).is_none());
        assert!(store.is_empty(), "empty records are not stored");
    }

    #[test]
    fn remove_returns_the_stored_record_and_none_when_absent() {
        let mut topo = Topology::new();
        let solid = add_solid(&mut topo);
        let kept_solid = add_solid(&mut topo);
        let face = add_face(&mut topo);
        let kept_face = add_face(&mut topo);

        let mut store = AttributeStore::new();
        store.set_solid(solid, named("bracket"));
        store.set_solid(kept_solid, named("plate"));
        store.set_face(face, named("web"));
        store.set_face(kept_face, named("flange"));

        let removed = store.remove_solid(solid).expect("the record was set");
        assert_eq!(
            removed,
            named("bracket"),
            "remove hands back the record it removed, not a default"
        );
        let removed = store.remove_face(face).expect("the record was set");
        assert_eq!(removed, named("web"));

        // Removing one entity does not disturb another.
        assert_eq!(
            store.solid(kept_solid).unwrap().name.as_deref(),
            Some("plate")
        );
        assert_eq!(
            store.face(kept_face).unwrap().name.as_deref(),
            Some("flange")
        );

        // Removing what is not there is None, not an invented default.
        assert!(store.remove_solid(solid).is_none());
        assert!(store.remove_face(face).is_none());
    }

    #[test]
    fn len_and_is_empty_count_solids_and_faces_together() {
        let mut topo = Topology::new();
        let solid_a = add_solid(&mut topo);
        let solid_b = add_solid(&mut topo);
        let face = add_face(&mut topo);

        let mut store = AttributeStore::new();
        assert_eq!(store.len(), 0);
        assert!(store.is_empty());

        // Faces alone: the store is not empty even though no solid is
        // attributed (and vice versa).
        store.set_face(face, named("web"));
        assert_eq!(store.len(), 1);
        assert!(!store.is_empty(), "a face-only store is not empty");

        store.set_solid(solid_a, named("bracket"));
        assert!(!store.is_empty());
        store.set_solid(solid_b, named("plate"));
        assert_eq!(
            store.len(),
            3,
            "len is the sum of both maps (2 solids + 1 face)"
        );

        store.remove_face(face);
        assert_eq!(store.len(), 2);
        assert!(!store.is_empty(), "a solid-only store is not empty");
    }

    #[test]
    fn listings_yield_every_entry_sorted_by_index() {
        let mut topo = Topology::new();
        let face_a = add_face(&mut topo);
        let face_b = add_face(&mut topo);
        let solid_a = add_solid(&mut topo);
        let solid_b = add_solid(&mut topo);
        assert!(face_a.index() < face_b.index());
        assert!(solid_a.index() < solid_b.index());

        let mut store = AttributeStore::new();
        // Inserted in reverse index order: the listing must still be sorted.
        store.set_face(face_b, named("second face"));
        store.set_face(face_a, named("first face"));
        store.set_solid(solid_b, named("second solid"));
        store.set_solid(solid_a, named("first solid"));

        let faces = store.faces_with_attributes();
        assert_eq!(faces.len(), 2, "every attributed face is listed once");
        assert_eq!(faces[0].0, face_a);
        assert_eq!(faces[1].0, face_b);
        assert_eq!(faces[0].1.name.as_deref(), Some("first face"));
        assert_eq!(faces[1].1.name.as_deref(), Some("second face"));

        let solids = store.solids_with_attributes();
        assert_eq!(solids.len(), 2, "every attributed solid is listed once");
        assert_eq!(solids[0].0, solid_a);
        assert_eq!(solids[1].0, solid_b);
        assert_eq!(solids[0].1.name.as_deref(), Some("first solid"));
        assert_eq!(solids[1].1.name.as_deref(), Some("second solid"));
    }

    #[test]
    fn retiring_entities_removes_exactly_their_entries() {
        let mut topo = Topology::new();
        let retired_solid = add_solid(&mut topo);
        let live_solid = add_solid(&mut topo);
        let live_face = add_face(&mut topo);
        let retired_face = add_face(&mut topo);

        let mut store = AttributeStore::new();
        store.set_solid(retired_solid, named("gone solid"));
        store.set_solid(live_solid, named("kept solid"));
        store.set_face(live_face, named("kept face"));
        store.set_face(retired_face, named("gone face"));
        assert_eq!(store.len(), 4);

        let retired_solids: HashSet<SolidId> = std::iter::once(retired_solid).collect();
        let retired_faces: HashSet<FaceId> = std::iter::once(retired_face).collect();
        store.remove_for_retired_entities(&retired_solids, &retired_faces);

        assert!(
            store.solid(retired_solid).is_none(),
            "a retired solid's attributes must go"
        );
        assert!(
            store.face(retired_face).is_none(),
            "a retired face's attributes must go"
        );
        assert_eq!(
            store.solid(live_solid).unwrap().name.as_deref(),
            Some("kept solid"),
            "a live solid's attributes must survive"
        );
        assert_eq!(
            store.face(live_face).unwrap().name.as_deref(),
            Some("kept face"),
            "a live face's attributes must survive"
        );
        assert_eq!(store.len(), 2);

        // Retiring nothing removes nothing.
        store.remove_for_retired_entities(&HashSet::new(), &HashSet::new());
        assert_eq!(store.len(), 2);
    }
}
