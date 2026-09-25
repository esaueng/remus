//! Named, rigid STEP occurrences over shared solid definitions.

use std::collections::{BTreeMap, BTreeSet};

use remus_math::mat::Mat4;
use remus_operations::assembly::{Assembly, ComponentId};

use super::{
    AttrSlot, HashMap, ImportLimits, IoError, SolidId, StepBuilder, StepEntity,
    StepImportDiagnostic, Tolerance, Topology, ensure_input_size, ensure_limit,
    exact_reference_list, find_exact_composite_component, is_solid_brep, merge_split_rim_arcs,
    parse_step_entities, resolve_unit_scale, split_attr_slots,
};

/// An opt-in product structure; geometry remains in the caller's topology.
#[derive(Debug)]
pub struct StepAssemblyReadResult {
    /// Unique solid definitions, ordered by STEP B-Rep entity number.
    pub solids: Vec<SolidId>,
    /// Named occurrences with parent-relative transforms in millimetres.
    pub assembly: Assembly,
    /// The geometry reader's import qualifications.
    pub diagnostics: Vec<StepImportDiagnostic>,
    /// Declared length factor applied once to both geometry and placements.
    pub length_scale_to_mm: f64,
}

/// Read one root product and its solid-bearing, rigid component occurrences.
///
/// Supports `NEXT_ASSEMBLY_USAGE_OCCURRENCE` linked through product shape and
/// context-dependent shape representations, with schema-ordered
/// `ITEM_DEFINED_TRANSFORMATION` placements. Nested components own a solid;
/// geometry-free intermediate groups, mapped items and mixed units are refused.
/// Nesting is limited to 256 component levels; expanded occurrences also consume
/// `ImportLimits::max_model_entities`, independently of the encoded entity count.
///
/// # Errors
/// Returns typed parse, unsupported-entity, limit or geometry errors. A refused
/// import restores existing topology and permanently retires new handles.
pub fn read_step_assembly(
    input: &str,
    topo: &mut Topology,
) -> Result<StepAssemblyReadResult, IoError> {
    read_step_assembly_with_limits(input, topo, ImportLimits::default())
}

/// Read an assembly with explicit input, entity and expanded-occurrence limits.
///
/// # Errors
/// Same refusals and atomicity as [`read_step_assembly`].
pub fn read_step_assembly_with_limits(
    input: &str,
    topo: &mut Topology,
    limits: ImportLimits,
) -> Result<StepAssemblyReadResult, IoError> {
    ensure_input_size(input.len(), limits)?;
    let entities = parse_step_entities(input, limits)?;
    let units =
        resolve_unit_scale(&entities, true)?.ok_or_else(|| invalid("missing assembly units"))?;
    let snapshot = topo.clone();
    let result = (|| {
        let mut builder = StepBuilder::new(topo, &entities, units, limits)?;
        let graph = Graph::read(&builder)?;
        let definitions = builder.build_all_solids()?;
        let solid_map = definitions.iter().copied().collect();
        let assembly = graph.expand(&solid_map, limits)?;
        let diagnostics = std::mem::take(&mut builder.diagnostics);
        let solids: Vec<_> = definitions.into_iter().map(|(_, solid)| solid).collect();
        for &solid in &solids {
            merge_split_rim_arcs(builder.topo, solid, Tolerance::new())?;
        }
        Ok(StepAssemblyReadResult {
            solids,
            assembly,
            diagnostics,
            length_scale_to_mm: units.length,
        })
    })();
    if result.is_err() {
        topo.restore_preserving_handle_slots(&snapshot);
    }
    result
}

fn invalid(reason: impl Into<String>) -> IoError {
    IoError::ParseError {
        reason: reason.into(),
    }
}

fn entity<'a>(
    entities: &'a HashMap<u64, StepEntity>,
    id: u64,
    kind: &str,
) -> Result<&'a StepEntity, IoError> {
    entities
        .get(&id)
        .filter(|entity| entity.entity_type == kind)
        .ok_or_else(|| invalid(format!("expected {kind} at #{id}")))
}

fn reference(slots: &[AttrSlot<'_>], index: usize) -> Result<u64, IoError> {
    slots
        .get(index)
        .and_then(AttrSlot::as_ref_id)
        .ok_or_else(|| {
            invalid(format!(
                "assembly attribute {index} must be an entity reference"
            ))
        })
}

fn name(slots: &[AttrSlot<'_>], index: usize) -> Result<String, IoError> {
    match slots.get(index) {
        Some(AttrSlot::Text(value)) => Ok(value.clone()),
        _ => Err(invalid(format!(
            "assembly attribute {index} must be a string"
        ))),
    }
}

fn product_label(entities: &HashMap<u64, StepEntity>, id: u64) -> Result<String, IoError> {
    let definition = entity(entities, id, "PRODUCT_DEFINITION")?;
    let formation_id = reference(&split_attr_slots(&definition.attrs), 2)?;
    let formation = entities
        .get(&formation_id)
        .ok_or_else(|| invalid("missing product formation"))?;
    if !matches!(
        formation.entity_type.as_str(),
        "PRODUCT_DEFINITION_FORMATION" | "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE"
    ) {
        return Err(invalid("unsupported product formation"));
    }
    let product = entity(
        entities,
        reference(&split_attr_slots(&formation.attrs), 2)?,
        "PRODUCT",
    )?;
    name(&split_attr_slots(&product.attrs), 1)
}

struct Representation {
    id: u64,
    items: BTreeSet<u64>,
    solid: Option<u64>,
}

struct Occurrence {
    id: u64,
    child: u64,
    name: String,
    transform: Mat4,
}

struct PendingOccurrence<'a> {
    source: &'a Occurrence,
    parent: Option<ComponentId>,
    depth: usize,
    parent_world: Mat4,
}

struct Graph {
    root: u64,
    name: String,
    products: BTreeMap<u64, Representation>,
    children: BTreeMap<u64, Vec<Occurrence>>,
}

impl Graph {
    #[allow(clippy::too_many_lines)]
    fn read(builder: &StepBuilder<'_>) -> Result<Self, IoError> {
        let entities = builder.entities;
        let ordered: BTreeMap<_, _> = entities.iter().map(|(&id, entity)| (id, entity)).collect();
        let mut products = BTreeMap::new();
        for instance in ordered
            .values()
            .filter(|entity| entity.entity_type == "SHAPE_DEFINITION_REPRESENTATION")
        {
            let slots = split_attr_slots(&instance.attrs);
            let shape = entity(entities, reference(&slots, 0)?, "PRODUCT_DEFINITION_SHAPE")?;
            let definition = reference(&split_attr_slots(&shape.attrs), 2)?;
            product_label(entities, definition)?;
            let representation_id = reference(&slots, 1)?;
            let representation = entities
                .get(&representation_id)
                .ok_or_else(|| invalid("missing product representation"))?;
            if !matches!(
                representation.entity_type.as_str(),
                "SHAPE_REPRESENTATION" | "ADVANCED_BREP_SHAPE_REPRESENTATION"
            ) {
                return Err(IoError::UnsupportedEntity {
                    entity: representation.entity_type.clone(),
                });
            }
            let slots = split_attr_slots(&representation.attrs);
            let Some(AttrSlot::List(raw_items)) = slots.get(1) else {
                return Err(invalid("invalid representation items"));
            };
            let items: BTreeSet<_> = exact_reference_list(raw_items)
                .map_err(invalid)?
                .into_iter()
                .collect();
            if items.is_empty() {
                return Err(invalid("assembly representation has no items"));
            }
            let mut solid = None;
            for &id in &items {
                let item = entities
                    .get(&id)
                    .ok_or_else(|| invalid("missing representation item"))?;
                if is_solid_brep(item) {
                    if solid.replace(id).is_some() {
                        return Err(invalid("assembly component must contain exactly one solid"));
                    }
                } else if item.entity_type != "AXIS2_PLACEMENT_3D" {
                    return Err(IoError::UnsupportedEntity {
                        entity: item.entity_type.clone(),
                    });
                }
            }
            if products
                .insert(
                    definition,
                    Representation {
                        id: representation_id,
                        items,
                        solid,
                    },
                )
                .is_some()
            {
                return Err(invalid("ambiguous product shape representations"));
            }
        }
        let mut links = BTreeMap::new();
        let mut used_relations = BTreeSet::new();
        for instance in ordered
            .values()
            .filter(|entity| entity.entity_type == "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")
        {
            let slots = split_attr_slots(&instance.attrs);
            let relation = reference(&slots, 0)?;
            let shape = entity(entities, reference(&slots, 1)?, "PRODUCT_DEFINITION_SHAPE")?;
            let occurrence = reference(&split_attr_slots(&shape.attrs), 2)?;
            entity(entities, occurrence, "NEXT_ASSEMBLY_USAGE_OCCURRENCE")?;
            if links.insert(occurrence, relation).is_some() || !used_relations.insert(relation) {
                return Err(invalid("ambiguous occurrence shape relationship"));
            }
        }
        let mut parents = BTreeSet::new();
        let mut children_ids = BTreeSet::new();
        let mut children: BTreeMap<u64, Vec<Occurrence>> = BTreeMap::new();
        let mut occurrences = 0;
        for (&id, instance) in &ordered {
            if instance.entity_type != "NEXT_ASSEMBLY_USAGE_OCCURRENCE" {
                continue;
            }
            let slots = split_attr_slots(&instance.attrs);
            let parent = reference(&slots, 3)?;
            let child = reference(&slots, 4)?;
            let parent_rep = products
                .get(&parent)
                .ok_or_else(|| invalid("parent product has no shape representation"))?;
            let child_rep = products
                .get(&child)
                .ok_or_else(|| invalid("child product has no shape representation"))?;
            if child_rep.solid.is_none() {
                return Err(invalid(
                    "geometry-free intermediate assembly groups are unsupported",
                ));
            }
            let relation = links.get(&id).ok_or_else(|| {
                invalid("occurrence has no context-dependent shape representation")
            })?;
            let transform = occurrence_transform(builder, *relation, child_rep, parent_rep)?;
            children.entry(parent).or_default().push(Occurrence {
                id,
                child,
                name: name(&slots, 1)?,
                transform,
            });
            parents.insert(parent);
            children_ids.insert(child);
            occurrences += 1;
        }
        let roots: Vec<_> = parents.difference(&children_ids).copied().collect();
        if roots.len() != 1 || occurrences == 0 {
            return Err(invalid(
                "assembly requires exactly one root and an acyclic occurrence graph",
            ));
        }
        let root = roots[0];
        if products[&root].solid.is_some() {
            return Err(invalid("root assembly product must be geometry-free"));
        }
        if products
            .keys()
            .any(|id| *id != root && !children_ids.contains(id))
        {
            return Err(invalid("unconnected product definition"));
        }
        for (&id, instance) in &ordered {
            if instance.entity_type.is_empty()
                && find_exact_composite_component(
                    &instance.attrs,
                    "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
                )
                .is_some()
                && !used_relations.contains(&id)
            {
                return Err(invalid(
                    "unconnected transformed representation relationship",
                ));
            }
        }
        let assigned: BTreeSet<_> = products
            .values()
            .filter_map(|representation| representation.solid)
            .collect();
        if ordered
            .iter()
            .any(|(id, instance)| is_solid_brep(instance) && !assigned.contains(id))
        {
            return Err(invalid("unassigned solid definition in assembly"));
        }
        let name = product_label(entities, root)?;
        Ok(Self {
            root,
            name,
            products,
            children,
        })
    }

    fn expand(
        &self,
        solids: &BTreeMap<u64, SolidId>,
        limits: ImportLimits,
    ) -> Result<Assembly, IoError> {
        self.validate_acyclic()?;
        let mut assembly = Assembly::new(&self.name);
        // Ready occurrences follow file order, preserving the native writer's
        // component insertion order and therefore its BOM representative names.
        let mut pending: BTreeMap<_, PendingOccurrence<'_>> = self.children[&self.root]
            .iter()
            .enumerate()
            .map(|(index, source)| {
                (
                    (source.id, index),
                    PendingOccurrence {
                        source,
                        parent: None,
                        depth: 1,
                        parent_world: Mat4::identity(),
                    },
                )
            })
            .collect();
        let mut expanded_name_bytes = self.name.len();
        let mut scheduled = pending.len();
        ensure_limit(
            "expanded STEP occurrences",
            scheduled,
            limits.max_model_entities,
        )?;
        while let Some((_, current)) = pending.pop_first() {
            let occurrence = current.source;
            ensure_limit("STEP assembly depth", current.depth, 256)?;
            let world = current.parent_world * occurrence.transform;
            if world.0.iter().flatten().any(|value| !value.is_finite()) {
                return Err(invalid("non-finite composed assembly placement"));
            }
            let definition = self.products[&occurrence.child]
                .solid
                .ok_or_else(|| invalid("missing component solid"))?;
            let solid = *solids
                .get(&definition)
                .ok_or_else(|| invalid("missing built solid definition"))?;
            expanded_name_bytes = expanded_name_bytes.saturating_add(occurrence.name.len());
            ensure_limit(
                "expanded STEP name bytes",
                expanded_name_bytes,
                limits.max_input_bytes,
            )?;
            let id = if let Some(parent) = current.parent {
                assembly.add_child_component(
                    parent,
                    &occurrence.name,
                    solid,
                    occurrence.transform,
                )?
            } else {
                assembly.add_root_component(&occurrence.name, solid, occurrence.transform)
            };
            if let Some(children) = self.children.get(&occurrence.child) {
                ensure_limit(
                    "expanded STEP occurrences",
                    scheduled.saturating_add(children.len()),
                    limits.max_model_entities,
                )?;
                for child in children {
                    pending.insert(
                        (child.id, scheduled),
                        PendingOccurrence {
                            source: child,
                            parent: Some(id),
                            depth: current.depth + 1,
                            parent_world: world,
                        },
                    );
                    scheduled += 1;
                }
            }
        }
        Ok(assembly)
    }

    fn validate_acyclic(&self) -> Result<(), IoError> {
        let mut incoming = BTreeMap::<u64, usize>::new();
        for occurrence in self.children.values().flatten() {
            *incoming.entry(occurrence.child).or_default() += 1;
        }
        let mut ready = vec![self.root];
        let mut visited = 0;
        while let Some(product) = ready.pop() {
            visited += 1;
            for occurrence in self.children.get(&product).into_iter().flatten() {
                let count = incoming
                    .get_mut(&occurrence.child)
                    .ok_or_else(|| invalid("missing assembly indegree"))?;
                *count -= 1;
                if *count == 0 {
                    ready.push(occurrence.child);
                }
            }
        }
        if visited != self.products.len() {
            return Err(invalid("unreachable or cyclic assembly occurrences"));
        }
        Ok(())
    }
}

fn occurrence_transform(
    builder: &StepBuilder<'_>,
    relation_id: u64,
    child: &Representation,
    parent: &Representation,
) -> Result<Mat4, IoError> {
    let relation = builder
        .entities
        .get(&relation_id)
        .ok_or_else(|| invalid("missing occurrence relationship"))?;
    let base = find_exact_composite_component(&relation.attrs, "REPRESENTATION_RELATIONSHIP")
        .ok_or_else(|| invalid("occurrence requires a transformed shape relationship"))?;
    let slots = split_attr_slots(base);
    if reference(&slots, 2)? != child.id || reference(&slots, 3)? != parent.id {
        return Err(invalid(
            "occurrence relationship must order child then parent representations",
        ));
    }
    let transform = find_exact_composite_component(
        &relation.attrs,
        "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
    )
    .ok_or_else(|| invalid("missing occurrence transformation"))?;
    let transform = entity(
        builder.entities,
        reference(&split_attr_slots(transform), 0)?,
        "ITEM_DEFINED_TRANSFORMATION",
    )?;
    let slots = split_attr_slots(&transform.attrs);
    let child_item = reference(&slots, 2)?;
    let parent_item = reference(&slots, 3)?;
    if !child.items.contains(&child_item)
        || !parent.items.contains(&parent_item)
        || child.items.contains(&parent_item)
        || parent.items.contains(&child_item)
    {
        return Err(invalid(
            "assembly placements require unambiguous schema-ordered representation membership",
        ));
    }
    let child_frame = placement(builder, child_item)?;
    let parent_frame = placement(builder, parent_item)?;
    let inverse = child_frame
        .inverse()
        .map_err(|_| invalid("singular component placement"))?;
    let result = parent_frame * inverse;
    if result.0.iter().flatten().any(|value| !value.is_finite()) {
        return Err(invalid("non-finite composed occurrence transform"));
    }
    Ok(result)
}

fn placement(builder: &StepBuilder<'_>, id: u64) -> Result<Mat4, IoError> {
    let instance = entity(builder.entities, id, "AXIS2_PLACEMENT_3D")?;
    let slots = split_attr_slots(&instance.attrs);
    if slots.len() != 4 {
        return Err(invalid("invalid assembly placement attribute count"));
    }
    let (origin, axis, reference) = builder.axis2_placement_from_slots(id, &slots)?;
    let z = axis
        .normalize()
        .map_err(|_| invalid("degenerate assembly placement axis"))?;
    let x = (reference - z * reference.dot(z))
        .normalize()
        .map_err(|_| invalid("parallel assembly placement axes"))?;
    let y = z.cross(x);
    let matrix = Mat4([
        [x.x(), y.x(), z.x(), origin.x()],
        [x.y(), y.y(), z.y(), origin.y()],
        [x.z(), y.z(), z.z(), origin.z()],
        [0., 0., 0., 1.],
    ]);
    if matrix.0.iter().flatten().any(|value| !value.is_finite()) {
        return Err(invalid("non-finite assembly placement"));
    }
    Ok(matrix)
}
