//! STEP product structure: named products, placed occurrences and colours.
//!
//! The flat readers ([`read_step_bodies`](super::read_step_bodies) and
//! friends) build one solid per `MANIFOLD_SOLID_BREP` in the coordinates of
//! its own representation. A file that places a part through
//! `NEXT_ASSEMBLY_USAGE_OCCURRENCE` therefore comes back with every part at
//! its definition origin and a repeated part only once. This reader keeps the
//! product structure instead:
//!
//! - every product occurrence becomes a [`StepProductNode`] carrying its
//!   product's name, in depth-first order with parents before children;
//! - every occurrence of a solid becomes its own solid root, copied and moved
//!   by the occurrence's composed rigid placement, so repeated instances are
//!   distinct roots at their own positions;
//! - `STYLED_ITEM` / `OVER_RIDING_STYLED_ITEM` surface colours on a solid or on
//!   one of its faces become topology colour attributes, and every instance
//!   copy carries them.
//!
//! Supported product layouts are the ones exporters write in practice: a
//! definition's shape is the representation its `SHAPE_DEFINITION_REPRESENTATION`
//! names, plus every representation joined to it by an untransformed
//! `SHAPE_REPRESENTATION_RELATIONSHIP` (a placement-only `SHAPE_REPRESENTATION`
//! beside an `ADVANCED_BREP_SHAPE_REPRESENTATION`, for example). An occurrence's
//! placement comes from its `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` and the
//! `ITEM_DEFINED_TRANSFORMATION` of its transformed relationship, with the
//! item order proven by representation membership exactly as the flat reader
//! proves it. Geometry-free intermediate assemblies are ordinary nodes.
//!
//! Refusals are typed and atomic: a cyclic or unreachable occurrence graph, an
//! occurrence that places geometry without a placement, mapped-item
//! assemblies, mixed units and exhausted budgets leave the caller's topology
//! untouched. Presentation is never a reason to refuse: a style chain this
//! reader does not apply (a curve style, an occurrence-specific override, a
//! malformed or out-of-range colour) is counted in [`StepPresentationSummary`]
//! and the geometry imports unchanged.
//!
//! A solid that no product reaches is still imported, as a root outside every
//! node, so a file whose product data is partial loses no geometry.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use remus_math::mat::Mat4;
use remus_operations::copy::copy_and_transform_solid_detailed;
use remus_operations::transform::TransformPolicy;
use remus_topology::attributes::{ColorRgb, EntityAttributes};
use remus_topology::explorer::{solid_edges, solid_faces, solid_vertices};
use remus_topology::face::FaceId;
use remus_topology::solid::SolidId;
use remus_topology::transaction::run_append_only;

use super::{
    AttrSlot, ImportLimits, IoError, OccurrencePlacementOrder, StepBuilder, StepEntity,
    StepImportDiagnostic, Tolerance, Topology, diagnostic_entity_type, ensure_input_size,
    ensure_limit, exact_reference_list, find_exact_composite_component, is_solid_brep,
    merge_split_rim_arcs, occurrence_placement_order, parse_step_entities, resolve_unit_scale,
    split_attr_slots,
};

/// Deepest product nesting accepted, matching the assembly reader.
const MAX_PRODUCT_DEPTH: usize = 256;

/// One product occurrence: the root product, or a component placed in it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StepProductNode {
    /// The product's name (`PRODUCT.name`, or its `id` when the name is empty).
    pub name: String,
    /// Index of the enclosing node in [`StepStructureReadResult::nodes`];
    /// `None` for a root product.
    pub parent: Option<usize>,
}

/// What the presentation pass did, as counts.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StepPresentationSummary {
    /// Solid definitions given a colour.
    pub solid_colours: usize,
    /// Faces given a colour (counted once per definition face).
    pub face_colours: usize,
    /// Styled items this reader did not apply: targets other than a solid or
    /// face, occurrence-specific overrides, or chains with no surface colour.
    pub unapplied_styles: usize,
    /// Items whose styles named different colours; none of them was applied.
    pub conflicting_styles: usize,
    /// Colours refused for a non-finite or out-of-range channel.
    pub invalid_colours: usize,
}

/// A STEP file read with its product structure.
#[derive(Debug)]
pub struct StepStructureReadResult {
    /// Placed solid roots: each node's own solids in B-Rep entity order, then
    /// its components depth-first in occurrence entity order; solids no
    /// product reaches come last.
    pub solids: Vec<SolidId>,
    /// For each entry of [`Self::solids`], the node it belongs to; `None` for
    /// a solid no product reaches.
    pub solid_nodes: Vec<Option<usize>>,
    /// Product occurrences, parents before children.
    pub nodes: Vec<StepProductNode>,
    /// The geometry reader's import qualifications.
    pub diagnostics: Vec<StepImportDiagnostic>,
    /// The presentation pass's counts.
    pub presentation: StepPresentationSummary,
}

/// Read a STEP file's solids with their product structure, names and colours.
///
/// # Errors
/// Returns typed parse, unsupported-entity, limit or geometry errors. A refused
/// import restores existing topology.
pub fn read_step_structure(
    input: &str,
    topo: &mut Topology,
) -> Result<StepStructureReadResult, IoError> {
    read_step_structure_with_limits(input, topo, ImportLimits::default())
}

/// [`read_step_structure`] under explicit input, entity and expansion budgets.
///
/// Expanded occurrences consume `max_model_entities` independently of the
/// encoded entity count, and expanded names consume `max_input_bytes`.
///
/// # Errors
/// Same refusals and atomicity as [`read_step_structure`].
pub fn read_step_structure_with_limits(
    input: &str,
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<StepStructureReadResult, IoError> {
    ensure_input_size(input.len(), limits)?;
    let entities = parse_step_entities(input, limits)?;
    let has_geometry = entities.values().any(is_solid_brep);
    let Some(units) = resolve_unit_scale(&entities, has_geometry)? else {
        return Ok(StepStructureReadResult {
            solids: Vec::new(),
            solid_nodes: Vec::new(),
            nodes: Vec::new(),
            diagnostics: Vec::new(),
            presentation: StepPresentationSummary::default(),
        });
    };
    let graph = ProductGraph::read(&entities)?;
    run_append_only(topo, |topo| {
        let mut builder = StepBuilder::new(topo, &entities, units, limits)?;
        builder.face_origins = Some(HashMap::new());
        let transforms = graph.placements(&builder)?;
        let definitions = builder.build_all_solids()?;
        let face_origins = builder.face_origins.take().unwrap_or_default();
        let diagnostics = std::mem::take(&mut builder.diagnostics);
        drop(builder);
        for &(_, solid) in &definitions {
            merge_split_rim_arcs(topo, solid, Tolerance::new())?;
        }
        let definitions: BTreeMap<u64, SolidId> = definitions.into_iter().collect();
        let presentation = apply_presentation(topo, &entities, &definitions, &face_origins);
        let expansion = graph.expand(&definitions, &transforms, limits)?;
        let mut used = BTreeSet::new();
        // Every copy allocates a whole solid: charge its faces, edges and
        // vertices against the model budget before allocating, so repeated
        // large parts refuse with a typed limit instead of exhausting memory.
        let mut sizes: HashMap<u64, usize> = HashMap::new();
        let mut copied = 0usize;
        let mut solids = Vec::with_capacity(expansion.instances.len());
        let mut solid_nodes = Vec::with_capacity(expansion.instances.len());
        for instance in &expansion.instances {
            let definition = definitions[&instance.brep];
            // The first identity occurrence of a definition is the definition
            // itself; every other occurrence is a moved copy, so no two roots
            // share topology.
            let solid = if instance.world == Mat4::identity() && used.insert(instance.brep) {
                definition
            } else {
                let size = match sizes.get(&instance.brep) {
                    Some(&size) => size,
                    None => {
                        let size = solid_faces(topo, definition)?.len()
                            + solid_edges(topo, definition)?.len()
                            + solid_vertices(topo, definition)?.len();
                        sizes.insert(instance.brep, size);
                        size
                    }
                };
                copied = copied.saturating_add(size);
                ensure_limit(
                    "STEP occurrence copy entities",
                    copied,
                    limits.max_model_entities,
                )?;
                copy_and_transform_solid_detailed(
                    topo,
                    definition,
                    &instance.world,
                    TransformPolicy::ExactOnly,
                )?
                .0
            };
            solids.push(solid);
            solid_nodes.push(instance.node);
        }
        Ok(StepStructureReadResult {
            solids,
            solid_nodes,
            nodes: expansion.nodes,
            diagnostics,
            presentation,
        })
    })
    .map(|(result, _)| result)
}

fn invalid(reason: impl Into<String>) -> IoError {
    IoError::ParseError {
        reason: reason.into(),
    }
}

fn is_type(entity: &StepEntity<'_>, kind: &str) -> bool {
    entity.type_str().eq_ignore_ascii_case(kind)
}

fn typed<'a, 'input>(
    entities: &'a HashMap<u64, StepEntity<'input>>,
    id: u64,
    kind: &str,
) -> Result<&'a StepEntity<'input>, IoError> {
    entities
        .get(&id)
        .filter(|entity| is_type(entity, kind))
        .ok_or_else(|| invalid(format!("expected {kind} at #{id}")))
}

fn reference(slots: &[AttrSlot<'_>], index: usize, what: &str) -> Result<u64, IoError> {
    slots
        .get(index)
        .and_then(AttrSlot::as_ref_id)
        .ok_or_else(|| {
            invalid(format!(
                "{what}: attribute {index} must be an entity reference"
            ))
        })
}

fn references(slots: &[AttrSlot<'_>], index: usize, what: &str) -> Result<Vec<u64>, IoError> {
    match slots.get(index) {
        Some(AttrSlot::List(list)) => {
            exact_reference_list(list).map_err(|reason| invalid(format!("{what}: {reason}")))
        }
        _ => Err(invalid(format!("{what}: attribute {index} must be a list"))),
    }
}

/// The decoded text of a string attribute, or `None` for any other form.
fn text(slots: &[AttrSlot<'_>], index: usize) -> Option<String> {
    slots.get(index).and_then(AttrSlot::text_value)
}

// ── Product graph ───────────────────────────────────────────────────

/// One `NEXT_ASSEMBLY_USAGE_OCCURRENCE`.
struct Occurrence {
    id: u64,
    parent: u64,
    child: u64,
    /// The transformed relationship that places it, if any.
    relation: Option<u64>,
}

/// A product definition's shape: its representations and their solids.
#[derive(Default)]
struct Shape {
    representations: BTreeSet<u64>,
    breps: BTreeSet<u64>,
}

struct ProductGraph {
    /// Product definitions in entity order, with their product names.
    definitions: BTreeMap<u64, String>,
    shapes: BTreeMap<u64, Shape>,
    /// Occurrences by parent definition, in entity order.
    children: BTreeMap<u64, Vec<Occurrence>>,
    /// Definitions that are some occurrence's child.
    used_as_child: BTreeSet<u64>,
}

struct Instance {
    brep: u64,
    node: Option<usize>,
    world: Mat4,
}

struct Expansion {
    nodes: Vec<StepProductNode>,
    instances: Vec<Instance>,
}

impl ProductGraph {
    #[allow(clippy::too_many_lines)]
    fn read(entities: &HashMap<u64, StepEntity<'_>>) -> Result<Self, IoError> {
        let ordered: BTreeMap<u64, &StepEntity<'_>> =
            entities.iter().map(|(&id, entity)| (id, entity)).collect();
        if ordered
            .values()
            .any(|entity| is_type(entity, "MAPPED_ITEM") || is_type(entity, "REPRESENTATION_MAP"))
            && ordered
                .values()
                .any(|entity| is_type(entity, "NEXT_ASSEMBLY_USAGE_OCCURRENCE"))
        {
            return Err(IoError::UnsupportedEntity {
                entity: "MAPPED_ITEM".to_string(),
            });
        }

        // Product names, resolved once per product.
        let mut product_names: HashMap<u64, String> = HashMap::new();
        let mut definitions = BTreeMap::new();
        for (&id, entity) in &ordered {
            if !is_type(entity, "PRODUCT_DEFINITION")
                && !is_type(entity, "PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS")
            {
                continue;
            }
            let slots = split_attr_slots(&entity.attrs);
            let formation_id = reference(&slots, 2, "PRODUCT_DEFINITION")?;
            let formation = entities
                .get(&formation_id)
                .filter(|formation| {
                    is_type(formation, "PRODUCT_DEFINITION_FORMATION")
                        || is_type(
                            formation,
                            "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
                        )
                })
                .ok_or_else(|| invalid(format!("product definition #{id} has no formation")))?;
            let product_id = reference(
                &split_attr_slots(&formation.attrs),
                2,
                "PRODUCT_DEFINITION_FORMATION",
            )?;
            let name = if let Some(name) = product_names.get(&product_id) {
                name.clone()
            } else {
                let product = typed(entities, product_id, "PRODUCT")?;
                let slots = split_attr_slots(&product.attrs);
                let name = text(&slots, 1)
                    .filter(|name| !name.is_empty())
                    .or_else(|| text(&slots, 0))
                    .unwrap_or_default();
                product_names.insert(product_id, name.clone());
                name
            };
            definitions.insert(id, name);
        }

        // Untransformed representation links, both directions.
        let mut links: BTreeMap<u64, BTreeSet<u64>> = BTreeMap::new();
        for entity in ordered.values() {
            if !is_type(entity, "SHAPE_REPRESENTATION_RELATIONSHIP") {
                continue;
            }
            let slots = split_attr_slots(&entity.attrs);
            let first = reference(&slots, 2, "SHAPE_REPRESENTATION_RELATIONSHIP")?;
            let second = reference(&slots, 3, "SHAPE_REPRESENTATION_RELATIONSHIP")?;
            links.entry(first).or_default().insert(second);
            links.entry(second).or_default().insert(first);
        }

        let mut shapes: BTreeMap<u64, Shape> = BTreeMap::new();
        for (&id, entity) in &ordered {
            if !is_type(entity, "SHAPE_DEFINITION_REPRESENTATION") {
                continue;
            }
            let slots = split_attr_slots(&entity.attrs);
            let shape = entities
                .get(&reference(&slots, 0, "SHAPE_DEFINITION_REPRESENTATION")?)
                .ok_or_else(|| invalid(format!("shape definition #{id} has no shape")))?;
            if !is_type(shape, "PRODUCT_DEFINITION_SHAPE") {
                // A property-of-a-shape representation (validation
                // properties, for example) is not product geometry.
                continue;
            }
            let definition = reference(
                &split_attr_slots(&shape.attrs),
                2,
                "PRODUCT_DEFINITION_SHAPE",
            )?;
            if !definitions.contains_key(&definition) {
                // A shape aspect or an occurrence's own shape.
                continue;
            }
            let representation = reference(&slots, 1, "SHAPE_DEFINITION_REPRESENTATION")?;
            let entry = shapes.entry(definition).or_default();
            // Walk untransformed links: every representation reached shares
            // the definition's coordinate frame.
            let mut pending = vec![representation];
            while let Some(current) = pending.pop() {
                if !entry.representations.insert(current) {
                    continue;
                }
                ensure_limit(
                    "STEP product representations",
                    entry.representations.len(),
                    entities.len(),
                )?;
                pending.extend(links.get(&current).into_iter().flatten().copied());
            }
        }
        for shape in shapes.values_mut() {
            for &representation in &shape.representations {
                let Some(entity) = entities.get(&representation) else {
                    return Err(invalid(format!("missing representation #{representation}")));
                };
                let Some(items) = representation_items(entity) else {
                    continue;
                };
                for item in items? {
                    if entities.get(&item).is_some_and(is_solid_brep) {
                        shape.breps.insert(item);
                    }
                }
            }
        }

        // Each occurrence's transformed relationship, through its contextual
        // shape representation.
        let mut placements: BTreeMap<u64, u64> = BTreeMap::new();
        for (&id, entity) in &ordered {
            if !is_type(entity, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") {
                continue;
            }
            let slots = split_attr_slots(&entity.attrs);
            let relation = reference(&slots, 0, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")?;
            let shape = typed(
                entities,
                reference(&slots, 1, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")?,
                "PRODUCT_DEFINITION_SHAPE",
            )?;
            let occurrence = reference(
                &split_attr_slots(&shape.attrs),
                2,
                "PRODUCT_DEFINITION_SHAPE",
            )?;
            if placements.insert(occurrence, relation).is_some() {
                return Err(invalid(format!(
                    "occurrence #{occurrence} has more than one placement (#{id})"
                )));
            }
        }

        let mut children: BTreeMap<u64, Vec<Occurrence>> = BTreeMap::new();
        let mut used_as_child = BTreeSet::new();
        for (&id, entity) in &ordered {
            if !is_type(entity, "NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
                continue;
            }
            let slots = split_attr_slots(&entity.attrs);
            let parent = reference(&slots, 3, "NEXT_ASSEMBLY_USAGE_OCCURRENCE")?;
            let child = reference(&slots, 4, "NEXT_ASSEMBLY_USAGE_OCCURRENCE")?;
            for definition in [parent, child] {
                if !definitions.contains_key(&definition) {
                    return Err(invalid(format!(
                        "occurrence #{id} names #{definition}, which is not a product definition"
                    )));
                }
            }
            used_as_child.insert(child);
            children.entry(parent).or_default().push(Occurrence {
                id,
                parent,
                child,
                relation: placements.get(&id).copied(),
            });
        }
        Ok(Self {
            definitions,
            shapes,
            children,
            used_as_child,
        })
    }

    /// Every placed occurrence's transform from child into parent coordinates.
    fn placements(&self, builder: &StepBuilder<'_>) -> Result<HashMap<u64, Mat4>, IoError> {
        let mut transforms = HashMap::new();
        for occurrence in self.children.values().flatten() {
            let Some(relation) = occurrence.relation else {
                continue;
            };
            let transform = occurrence_transform(
                builder,
                occurrence,
                relation,
                self.shapes.get(&occurrence.child),
                self.shapes.get(&occurrence.parent),
            )?;
            transforms.insert(occurrence.id, transform);
        }
        Ok(transforms)
    }

    /// Every definition that holds a solid or has one somewhere below it,
    /// found once by walking occurrences upward from the definitions with
    /// solids: linear in the occurrence count, however often definitions are
    /// shared.
    fn with_geometry(&self) -> BTreeSet<u64> {
        let mut parents: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
        for occurrence in self.children.values().flatten() {
            parents
                .entry(occurrence.child)
                .or_default()
                .push(occurrence.parent);
        }
        let mut found: BTreeSet<u64> = self
            .shapes
            .iter()
            .filter(|(_, shape)| !shape.breps.is_empty())
            .map(|(&id, _)| id)
            .collect();
        let mut pending: Vec<u64> = found.iter().copied().collect();
        while let Some(id) = pending.pop() {
            for &parent in parents.get(&id).into_iter().flatten() {
                if found.insert(parent) {
                    pending.push(parent);
                }
            }
        }
        found
    }

    #[allow(clippy::too_many_lines)]
    fn expand(
        &self,
        definitions: &BTreeMap<u64, SolidId>,
        transforms: &HashMap<u64, Mat4>,
        limits: ImportLimits,
    ) -> Result<Expansion, IoError> {
        struct Frame {
            definition: u64,
            parent: Option<usize>,
            world: Mat4,
            depth: usize,
        }
        let with_geometry = self.with_geometry();
        let mut nodes = Vec::new();
        let mut instances = Vec::new();
        let mut reached = BTreeSet::new();
        let mut visited = BTreeSet::new();
        let mut name_bytes = 0usize;
        // Roots: definitions with a shape or with components, that no
        // occurrence uses. Pushed in reverse so the stack pops them in order.
        let mut stack: Vec<Frame> = self
            .definitions
            .keys()
            .filter(|id| {
                !self.used_as_child.contains(id)
                    && (self.shapes.contains_key(id) || self.children.contains_key(id))
            })
            .rev()
            .map(|&definition| Frame {
                definition,
                parent: None,
                world: Mat4::identity(),
                depth: 1,
            })
            .collect();
        while let Some(frame) = stack.pop() {
            ensure_limit("STEP assembly depth", frame.depth, MAX_PRODUCT_DEPTH)?;
            ensure_limit(
                "expanded STEP occurrences",
                nodes.len() + 1,
                limits.max_model_entities,
            )?;
            let name = self.definitions[&frame.definition].clone();
            name_bytes = name_bytes.saturating_add(name.len());
            ensure_limit(
                "expanded STEP name bytes",
                name_bytes,
                limits.max_input_bytes,
            )?;
            let node = nodes.len();
            nodes.push(StepProductNode {
                name,
                parent: frame.parent,
            });
            visited.insert(frame.definition);
            if let Some(shape) = self.shapes.get(&frame.definition) {
                for &brep in &shape.breps {
                    if !definitions.contains_key(&brep) {
                        return Err(invalid(format!("solid #{brep} was not built")));
                    }
                    ensure_limit(
                        "expanded STEP occurrences",
                        instances.len() + 1,
                        limits.max_model_entities,
                    )?;
                    reached.insert(brep);
                    instances.push(Instance {
                        brep,
                        node: Some(node),
                        world: frame.world,
                    });
                }
            }
            let occurrences = self.children.get(&frame.definition);
            for occurrence in occurrences.into_iter().flatten().rev() {
                let local = match transforms.get(&occurrence.id) {
                    Some(&transform) => transform,
                    None if with_geometry.contains(&occurrence.child) => {
                        return Err(invalid(format!(
                            "occurrence #{} places geometry but has no placement",
                            occurrence.id
                        )));
                    }
                    None => Mat4::identity(),
                };
                let world = frame.world * local;
                if world.0.iter().flatten().any(|value| !value.is_finite()) {
                    return Err(invalid("non-finite composed assembly placement"));
                }
                stack.push(Frame {
                    definition: occurrence.child,
                    parent: Some(node),
                    world,
                    depth: frame.depth + 1,
                });
            }
        }
        if let Some(stranded) = self
            .children
            .keys()
            .chain(self.used_as_child.iter())
            .find(|id| !visited.contains(id))
        {
            return Err(invalid(format!(
                "product definition #{stranded} is in an unreachable or cyclic assembly"
            )));
        }
        for &brep in definitions.keys() {
            if !reached.contains(&brep) {
                ensure_limit(
                    "expanded STEP occurrences",
                    instances.len() + 1,
                    limits.max_model_entities,
                )?;
                instances.push(Instance {
                    brep,
                    node: None,
                    world: Mat4::identity(),
                });
            }
        }
        Ok(Expansion { nodes, instances })
    }
}

/// The item list of a shape representation, or `None` for other entities.
fn representation_items(entity: &StepEntity<'_>) -> Option<Result<Vec<u64>, IoError>> {
    let kind = entity.type_str();
    if !kind.to_ascii_uppercase().ends_with("SHAPE_REPRESENTATION")
        || kind.eq_ignore_ascii_case("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")
    {
        return None;
    }
    let slots = split_attr_slots(&entity.attrs);
    Some(references(&slots, 1, kind))
}

/// One occurrence's rigid transform from child into parent coordinates.
fn occurrence_transform(
    builder: &StepBuilder<'_>,
    occurrence: &Occurrence,
    relation_id: u64,
    child: Option<&Shape>,
    parent: Option<&Shape>,
) -> Result<Mat4, IoError> {
    let entities = builder.entities;
    let relation = entities
        .get(&relation_id)
        .ok_or_else(|| invalid(format!("missing occurrence relationship #{relation_id}")))?;
    let (base, transformation) = if relation.is_complex() {
        (
            find_exact_composite_component(&relation.attrs, "REPRESENTATION_RELATIONSHIP"),
            find_exact_composite_component(
                &relation.attrs,
                "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
            ),
        )
    } else {
        (None, None)
    };
    let (Some(base), Some(transformation)) = (base, transformation) else {
        return Err(invalid(format!(
            "occurrence #{} needs a transformed shape relationship, got {}",
            occurrence.id,
            diagnostic_entity_type(relation)
        )));
    };
    let slots = split_attr_slots(base);
    let first_rep = reference(&slots, 2, "REPRESENTATION_RELATIONSHIP")?;
    let second_rep = reference(&slots, 3, "REPRESENTATION_RELATIONSHIP")?;
    let in_shape = |shape: Option<&Shape>, rep: u64| {
        shape.is_some_and(|shape| shape.representations.contains(&rep))
    };
    // ISO 10303-43 orders a relationship child then parent; some writers
    // reverse it. Representation ownership decides which, and must decide.
    let (child_rep, parent_rep) = match (
        in_shape(child, first_rep) && in_shape(parent, second_rep),
        in_shape(child, second_rep) && in_shape(parent, first_rep),
    ) {
        (true, false) => (first_rep, second_rep),
        (false, true) => (second_rep, first_rep),
        _ => {
            return Err(invalid(format!(
                "occurrence #{} relationship #{relation_id} does not join its child and parent shapes",
                occurrence.id
            )));
        }
    };
    let transformation = typed(
        entities,
        reference(&split_attr_slots(transformation), 0, "transformation")?,
        "ITEM_DEFINED_TRANSFORMATION",
    )?;
    let slots = split_attr_slots(&transformation.attrs);
    let first_item = reference(&slots, 2, "ITEM_DEFINED_TRANSFORMATION")?;
    let second_item = reference(&slots, 3, "ITEM_DEFINED_TRANSFORMATION")?;
    let items = |rep: u64| -> Result<BTreeSet<u64>, IoError> {
        let entity = entities
            .get(&rep)
            .ok_or_else(|| invalid(format!("missing representation #{rep}")))?;
        match representation_items(entity) {
            Some(items) => Ok(items?.into_iter().collect()),
            None => Err(IoError::UnsupportedEntity {
                entity: diagnostic_entity_type(entity),
            }),
        }
    };
    let child_items = items(child_rep)?;
    let parent_items = items(parent_rep)?;
    let order = occurrence_placement_order(
        [
            child_items.contains(&first_item),
            child_items.contains(&second_item),
        ],
        [
            parent_items.contains(&first_item),
            parent_items.contains(&second_item),
        ],
    )?;
    let (child_frame, parent_frame) = match order {
        Some(OccurrencePlacementOrder::Schema) => (first_item, second_item),
        Some(OccurrencePlacementOrder::Historical) => (second_item, first_item),
        None => {
            return Err(invalid(format!(
                "occurrence #{} placement items are not members of its representations",
                occurrence.id
            )));
        }
    };
    let child_frame = super::assembly::placement(builder, child_frame)?;
    let parent_frame = super::assembly::placement(builder, parent_frame)?;
    let inverse = child_frame
        .inverse()
        .map_err(|_| invalid("singular component placement"))?;
    let result = parent_frame * inverse;
    if result.0.iter().flatten().any(|value| !value.is_finite()) {
        return Err(invalid("non-finite occurrence transform"));
    }
    Ok(result)
}

// ── Presentation ────────────────────────────────────────────────────

/// Named colours of `DRAUGHTING_PRE_DEFINED_COLOUR` (ISO 10303-46).
fn predefined_colour(name: &str) -> Option<(f64, f64, f64)> {
    Some(match name.to_ascii_lowercase().as_str() {
        "red" => (1.0, 0.0, 0.0),
        "green" => (0.0, 1.0, 0.0),
        "blue" => (0.0, 0.0, 1.0),
        "yellow" => (1.0, 1.0, 0.0),
        "magenta" => (1.0, 0.0, 1.0),
        "cyan" => (0.0, 1.0, 1.0),
        "black" => (0.0, 0.0, 0.0),
        "white" => (1.0, 1.0, 1.0),
        _ => return None,
    })
}

#[derive(Clone, Copy)]
enum Resolved {
    Colour(ColorRgb),
    /// The chain names a colour this reader refuses (out of range).
    Invalid,
    /// The chain names two different colours.
    Conflict,
    /// No surface fill colour in the chain.
    None,
}

/// The surface fill colour a style list names, following
/// `PRESENTATION_STYLE_ASSIGNMENT` → `SURFACE_STYLE_USAGE` →
/// `SURFACE_SIDE_STYLE` → `SURFACE_STYLE_FILL_AREA` → `FILL_AREA_STYLE` →
/// `FILL_AREA_STYLE_COLOUR`. A chain that names two different colours is a
/// conflict and resolves to none.
/// Folds one more resolved colour into `found`: two different colours are a
/// conflict, and a conflict stays one.
fn combine(found: Resolved, next: Resolved) -> Resolved {
    match (found, next) {
        (Resolved::Conflict, _) | (_, Resolved::Conflict) => Resolved::Conflict,
        (Resolved::Colour(a), Resolved::Colour(b)) if a != b => Resolved::Conflict,
        (Resolved::Colour(a), _) => Resolved::Colour(a),
        (found, Resolved::None) => found,
        (_, next) => next,
    }
}

/// Resolves surface fill colours through the style chain, each entity once.
///
/// A chain shares entities freely (one assignment listed N times, one usage
/// referenced by N assignments), so every level is memoized by entity id and
/// every reference list is deduplicated: the work is bounded by the number
/// of distinct references in the file, not by the number of paths through it.
struct StyleResolver<'a, 'input> {
    entities: &'a HashMap<u64, StepEntity<'input>>,
    memo: HashMap<u64, Resolved>,
}

impl StyleResolver<'_, '_> {
    fn list(&self, id: u64, index: usize) -> BTreeSet<u64> {
        self.entities
            .get(&id)
            .and_then(|entity| match split_attr_slots(&entity.attrs).get(index) {
                Some(AttrSlot::List(list)) => exact_reference_list(list).ok(),
                _ => None,
            })
            .unwrap_or_default()
            .into_iter()
            .collect()
    }

    fn single(&self, id: u64, index: usize) -> Option<u64> {
        self.entities.get(&id).and_then(|entity| {
            split_attr_slots(&entity.attrs)
                .get(index)
                .and_then(AttrSlot::as_ref_id)
        })
    }

    fn is(&self, id: u64, kinds: &[&str]) -> bool {
        self.entities
            .get(&id)
            .is_some_and(|entity| kinds.iter().any(|kind| is_type(entity, kind)))
    }

    fn memoized(&mut self, id: u64, resolve: impl FnOnce(&mut Self) -> Resolved) -> Resolved {
        if let Some(&known) = self.memo.get(&id) {
            return known;
        }
        // Provisionally none, so a malformed self-reference terminates.
        self.memo.insert(id, Resolved::None);
        let resolved = resolve(self);
        self.memo.insert(id, resolved);
        resolved
    }

    /// `FILL_AREA_STYLE` → its `FILL_AREA_STYLE_COLOUR`s.
    fn area(&mut self, id: u64) -> Resolved {
        self.memoized(id, |this| {
            let mut found = Resolved::None;
            for colour in this.list(id, 1) {
                if !this.is(colour, &["FILL_AREA_STYLE_COLOUR"]) {
                    continue;
                }
                let Some(rgb) = this.single(colour, 1).and_then(|id| this.entities.get(&id)) else {
                    continue;
                };
                found = combine(found, colour_value(rgb));
            }
            found
        })
    }

    /// `SURFACE_SIDE_STYLE` → `SURFACE_STYLE_FILL_AREA` → `FILL_AREA_STYLE`.
    fn side(&mut self, id: u64) -> Resolved {
        self.memoized(id, |this| {
            let mut found = Resolved::None;
            for fill in this.list(id, 1) {
                if !this.is(fill, &["SURFACE_STYLE_FILL_AREA"]) {
                    continue;
                }
                let Some(area) = this.single(fill, 0) else {
                    continue;
                };
                if this.is(area, &["FILL_AREA_STYLE"]) {
                    found = combine(found, this.area(area));
                }
            }
            found
        })
    }

    /// `PRESENTATION_STYLE_ASSIGNMENT` → `SURFACE_STYLE_USAGE` (both or
    /// positive side) → `SURFACE_SIDE_STYLE`.
    fn assignment(&mut self, id: u64) -> Resolved {
        self.memoized(id, |this| {
            if !this.is(
                id,
                &[
                    "PRESENTATION_STYLE_ASSIGNMENT",
                    "PRESENTATION_STYLE_BY_CONTEXT",
                ],
            ) {
                return Resolved::None;
            }
            let mut found = Resolved::None;
            for usage in this.list(id, 0) {
                if !this.is(usage, &["SURFACE_STYLE_USAGE"]) {
                    continue;
                }
                let negative = this.entities.get(&usage).is_some_and(|entity| {
                    matches!(
                        split_attr_slots(&entity.attrs).first(),
                        Some(AttrSlot::Enum(side)) if side.eq_ignore_ascii_case(".NEGATIVE.")
                    )
                });
                if negative {
                    continue;
                }
                let Some(side) = this.single(usage, 1) else {
                    continue;
                };
                if this.is(side, &["SURFACE_SIDE_STYLE"]) {
                    found = combine(found, this.side(side));
                }
            }
            found
        })
    }

    /// The surface fill colour a styled item's style list names.
    fn styles(&mut self, styles: &[u64]) -> Resolved {
        let distinct: BTreeSet<u64> = styles.iter().copied().collect();
        distinct.into_iter().fold(Resolved::None, |found, id| {
            combine(found, self.assignment(id))
        })
    }
}

fn colour_value(entity: &StepEntity<'_>) -> Resolved {
    let slots = split_attr_slots(&entity.attrs);
    let channels = if is_type(entity, "COLOUR_RGB") {
        let real = |index: usize| match slots.get(index) {
            Some(AttrSlot::Other(raw)) => raw.trim().parse::<f64>().ok(),
            _ => None,
        };
        match (real(1), real(2), real(3)) {
            (Some(r), Some(g), Some(b)) => Some((r, g, b)),
            _ => return Resolved::Invalid,
        }
    } else if is_type(entity, "DRAUGHTING_PRE_DEFINED_COLOUR") {
        match text(&slots, 0).as_deref().and_then(predefined_colour) {
            Some(channels) => Some(channels),
            None => return Resolved::None,
        }
    } else {
        None
    };
    match channels {
        Some((r, g, b)) => ColorRgb::new(r, g, b).map_or(Resolved::Invalid, Resolved::Colour),
        None => Resolved::None,
    }
}

/// Apply every solid- and face-level surface colour, counting the rest.
fn apply_presentation(
    topo: &mut Topology,
    entities: &HashMap<u64, StepEntity<'_>>,
    solids: &BTreeMap<u64, SolidId>,
    faces: &HashMap<u64, Vec<FaceId>>,
) -> StepPresentationSummary {
    let mut summary = StepPresentationSummary::default();
    // Colours per target item, in styled-item entity order. An overriding
    // style sets its item's colour whatever plain styles say (the last
    // override wins); two plain styles that disagree leave the item
    // uncoloured and count once as a conflict.
    let mut ordered: Vec<(u64, &StepEntity<'_>)> =
        entities.iter().map(|(&id, entity)| (id, entity)).collect();
    ordered.sort_unstable_by_key(|(id, _)| *id);
    let mut resolver = StyleResolver {
        entities,
        memo: HashMap::new(),
    };
    let mut targets: BTreeMap<u64, Option<ColorRgb>> = BTreeMap::new();
    let mut overridden: BTreeSet<u64> = BTreeSet::new();
    for (_, entity) in ordered {
        let overriding = is_type(entity, "OVER_RIDING_STYLED_ITEM");
        if !overriding && !is_type(entity, "STYLED_ITEM") {
            if is_type(entity, "CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM") {
                summary.unapplied_styles += 1;
            }
            continue;
        }
        let slots = split_attr_slots(&entity.attrs);
        let Some(item) = slots.get(2).and_then(AttrSlot::as_ref_id) else {
            summary.unapplied_styles += 1;
            continue;
        };
        if !solids.contains_key(&item) && !faces.contains_key(&item) {
            summary.unapplied_styles += 1;
            continue;
        }
        let styles = match slots.get(1) {
            Some(AttrSlot::List(list)) => exact_reference_list(list).unwrap_or_default(),
            _ => Vec::new(),
        };
        let colour = match resolver.styles(&styles) {
            Resolved::Colour(colour) => colour,
            Resolved::Invalid => {
                summary.invalid_colours += 1;
                continue;
            }
            Resolved::Conflict => {
                summary.conflicting_styles += 1;
                continue;
            }
            Resolved::None => {
                summary.unapplied_styles += 1;
                continue;
            }
        };
        if overriding {
            overridden.insert(item);
            targets.insert(item, Some(colour));
            continue;
        }
        if overridden.contains(&item) {
            continue;
        }
        match targets.get(&item) {
            None => {
                targets.insert(item, Some(colour));
            }
            Some(Some(existing)) if *existing == colour => {}
            Some(Some(_)) => {
                summary.conflicting_styles += 1;
                targets.insert(item, None);
            }
            Some(None) => {}
        }
    }
    for (item, colour) in targets {
        let Some(colour) = colour else {
            continue;
        };
        if let Some(&solid) = solids.get(&item) {
            let mut attributes = topo.attributes().solid(solid).cloned().unwrap_or_default();
            attributes.color = Some(colour);
            if topo.set_solid_attributes(solid, attributes).is_ok() {
                summary.solid_colours += 1;
            }
        } else if let Some(face_ids) = faces.get(&item) {
            for &face in face_ids {
                let mut attributes: EntityAttributes =
                    topo.attributes().face(face).cloned().unwrap_or_default();
                attributes.color = Some(colour);
                if topo.set_face_attributes(face, attributes).is_ok() {
                    summary.face_colours += 1;
                }
            }
        }
    }
    summary
}
