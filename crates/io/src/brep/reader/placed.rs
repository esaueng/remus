//! Bounded, iterative expansion by source identity and canonical location.

use std::collections::BTreeMap;

use super::locations::point;
use super::{
    CurveUse, Document, Geometry, ImportLimits, IoError, Locations, MAX_RECORDS, PcurveUse, Shape,
    Usage, ensure_limit, malformed,
};

type Key = (usize, usize);

struct Expansion<'a> {
    source: &'a Document,
    locations: Locations,
    output: Document,
    shapes: BTreeMap<Key, usize>,
    curves3: BTreeMap<Key, usize>,
    surfaces: BTreeMap<Key, usize>,
    used: usize,
    limit: usize,
}

impl Document {
    pub(super) fn place(mut self, limits: ImportLimits) -> Result<Self, IoError> {
        if self.locations.is_identity() {
            return Ok(self);
        }
        let locations = std::mem::take(&mut self.locations);
        let mut expansion = Expansion {
            source: &self,
            locations,
            output: Self {
                locations: Locations::default(),
                curves3: vec![],
                curves2: self.curves2.clone(),
                surfaces: vec![],
                shapes: vec![],
                root: self.root,
            },
            shapes: BTreeMap::new(),
            curves3: BTreeMap::new(),
            surfaces: BTreeMap::new(),
            used: self.curves2.len(),
            limit: limits.max_model_entities.min(MAX_RECORDS),
        };
        // Retain validation of unused source vertices/edges, as in the original
        // reader. Validate them in scratch storage, so an unused identity
        // occurrence does not leak orphan geometry into a placed import.
        for (i, shape) in self.shapes.iter().enumerate() {
            if matches!(
                shape.geometry,
                Geometry::Vertex { .. } | Geometry::Edge { .. }
            ) {
                expansion.expand((i, 0))?;
            }
        }
        let mut scratch = remus_topology::Topology::new();
        let vertices = expansion.output.vertices(&mut scratch);
        expansion.output.edges(&mut scratch, &vertices)?;
        expansion.shapes.clear();
        expansion.output.shapes.clear();
        expansion.expand((self.root.index, self.root.location))?;
        expansion.output.root = Usage {
            index: expansion.shapes[&(self.root.index, self.root.location)],
            location: 0,
            ..self.root
        };
        Ok(expansion.output)
    }
}

impl Expansion<'_> {
    fn charge(&mut self, count: usize) -> Result<(), IoError> {
        self.used = self.used.saturating_add(count);
        ensure_limit(
            "BREP materialized occurrences and references",
            self.used,
            self.limit,
        )
    }

    fn line(&mut self, index: usize, location: usize) -> Result<usize, IoError> {
        if let Some(id) = self.curves3.get(&(index, location)) {
            return Ok(*id);
        }
        self.charge(1)?;
        let matrix = self.locations.matrix(location);
        let id = self.output.curves3.len();
        self.output
            .curves3
            .push(self.source.curves3[index].moved(matrix)?);
        self.curves3.insert((index, location), id);
        Ok(id)
    }

    fn plane(&mut self, index: usize, location: usize) -> Result<usize, IoError> {
        if let Some(id) = self.surfaces.get(&(index, location)) {
            return Ok(*id);
        }
        self.charge(1)?;
        let matrix = self.locations.matrix(location);
        let id = self.output.surfaces.len();
        self.output
            .surfaces
            .push(self.source.surfaces[index].moved(matrix)?);
        self.surfaces.insert((index, location), id);
        Ok(id)
    }

    fn geometry(&mut self, geometry: Geometry, location: usize) -> Result<Geometry, IoError> {
        Ok(match geometry {
            Geometry::Vertex {
                point: p,
                tolerance,
            } => {
                let point = point(self.locations.matrix(location), p);
                if point.0.iter().any(|value| !value.is_finite()) {
                    return Err(malformed("non-finite located vertex"));
                }
                Geometry::Vertex { point, tolerance }
            }
            Geometry::Edge {
                tolerance,
                curve,
                pcurves,
            } => {
                self.charge(pcurves.len())?;
                let curve_location = self.locations.compose(location, curve.location)?;
                let curve = CurveUse {
                    curve: self.line(curve.curve, curve_location)?,
                    location: 0,
                    ..curve
                };
                let mut placed = Vec::with_capacity(pcurves.len());
                for p in pcurves {
                    let surface_location = self.locations.compose(location, p.location)?;
                    placed.push(PcurveUse {
                        surface: self.plane(p.surface, surface_location)?,
                        location: 0,
                        ..p
                    });
                }
                Geometry::Edge {
                    tolerance,
                    curve,
                    pcurves: placed,
                }
            }
            Geometry::Face {
                surface,
                tolerance,
                location: local,
            } => {
                let surface_location = self.locations.compose(location, local)?;
                Geometry::Face {
                    surface: self.plane(surface, surface_location)?,
                    tolerance,
                    location: 0,
                }
            }
            other => other,
        })
    }

    fn expand(&mut self, root: Key) -> Result<(), IoError> {
        let mut stack = vec![(root, false)];
        while let Some((key, finish)) = stack.pop() {
            if self.shapes.contains_key(&key) {
                continue;
            }
            let source = &self.source.shapes[key.0];
            if !finish {
                self.charge(1 + source.children.len())?;
                stack.push((key, true));
                for child in source.children.iter().rev() {
                    let world = self.locations.compose(key.1, child.location)?;
                    stack.push(((child.index, world), false));
                }
                continue;
            }
            let source = source.clone();
            let mut children = Vec::with_capacity(source.children.len());
            for child in source.children {
                let world = self.locations.compose(key.1, child.location)?;
                let index = *self
                    .shapes
                    .get(&(child.index, world))
                    .ok_or_else(|| malformed("unresolved placed child"))?;
                children.push(Usage {
                    index,
                    location: 0,
                    ..child
                });
            }
            let geometry = self.geometry(source.geometry, key.1)?;
            let id = self.output.shapes.len();
            self.output.shapes.push(Shape { geometry, children });
            self.shapes.insert(key, id);
        }
        Ok(())
    }
}
