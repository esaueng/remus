//! Topology attribute store — semantic names and display colors
//! (Issue 14; design in `docs/design/deferred-e3b-step-names-and-colors.md`).
//!
//! Attributes are public model data, not rendering hints: a face's color and
//! a solid's name survive modeling operations under the explicit propagation
//! rules implemented in `brepkit_operations::evolution` (driven by
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
    pub r: f64,
    /// Green channel, `0..=1`.
    pub g: f64,
    /// Blue channel, `0..=1`.
    pub b: f64,
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
    /// Application/user identifier, opaque to the kernel.
    pub app_id: Option<String>,
    /// Source import entity reference (e.g. a STEP entity number), opaque
    /// to the kernel.
    pub source_entity: Option<String>,
    /// Optional layer or group identifier, opaque to the kernel.
    pub layer: Option<String>,
}

impl EntityAttributes {
    /// True when no attribute is set (such entries are not stored).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.name.is_none()
            && self.color.is_none()
            && self.app_id.is_none()
            && self.source_entity.is_none()
            && self.layer.is_none()
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
