//! Opt-in AP203 occurrence export; the legacy body writer stays separate.

use std::{collections::BTreeMap, fmt::Write as _};

use remus_math::{mat::Mat4, tolerance::Tolerance};
use remus_operations::assembly::{Assembly, Component, ComponentId};

use super::{
    IoError, Point3, StepWriteContext, StepWriteOptions, Topology, Vec3, export_uncertainty,
    step_string_literal,
};

/// Write a named assembly with rigid, orientation-preserving local placements.
///
/// Every component, including a parent component, owns a solid. Shared solid
/// handles emit one B-Rep definition. The assembly itself is a geometry-free
/// root product. Lengths are millimetres, as in the legacy STEP writer.
///
/// # Errors
/// Refuses empty assemblies, invalid topology and non-rigid/non-finite matrices.
#[allow(clippy::too_many_lines)]
pub fn write_step_assembly(topo: &Topology, assembly: &Assembly) -> Result<String, IoError> {
    if assembly.component_count() == 0 {
        return Err(IoError::InvalidTopology {
            reason: "empty STEP assembly".into(),
        });
    }
    let mut components = BTreeMap::new();
    let mut pending: Vec<_> = assembly
        .roots()
        .iter()
        .map(|&id| (id, Mat4::identity()))
        .collect();
    while let Some((id, parent_world)) = pending.pop() {
        let component = assembly
            .component(id)
            .ok_or_else(|| IoError::InvalidTopology {
                reason: "assembly contains a missing component".into(),
            })?;
        validate_rigid(component.transform)?;
        let world = parent_world * component.transform;
        if world.0.iter().flatten().any(|value| !value.is_finite()) {
            return Err(IoError::InvalidTopology {
                reason: "non-finite composed assembly placement".into(),
            });
        }
        components.insert(id, component);
        pending.extend(component.children.iter().map(|&child| (child, world)));
    }
    let mut solids: Vec<_> = components
        .values()
        .map(|component| component.solid)
        .collect();
    solids.sort_unstable();
    solids.dedup();
    let uncertainty = export_uncertainty(topo, &solids, &[])?;
    let mut context = StepWriteContext::new(StepWriteOptions {
        product_name: assembly.name().into(),
        ..StepWriteOptions::default()
    });
    context.prepare_boundary_authority(topo, &solids, &[])?;
    let mut breps = BTreeMap::new();
    for solid in solids {
        breps.insert(solid, context.write_solid(topo, solid)?);
    }
    let root = product(&mut context, assembly.name(), None, uncertainty)?;
    let mut products = BTreeMap::new();
    for (&id, component) in &components {
        products.insert(
            id,
            product(
                &mut context,
                &component.name,
                Some(breps[&component.solid]),
                uncertainty,
            )?,
        );
    }
    let mut items: BTreeMap<Option<ComponentId>, Vec<u64>> = products
        .iter()
        .map(|(&id, product)| (Some(id), product.items.clone()))
        .collect();
    items.insert(None, root.items.clone());
    for (&id, component) in &components {
        let child = &products[&id];
        let parent = component.parent.map_or(&root, |id| &products[&id]);
        let placement = write_placement(&mut context, component.transform);
        items.entry(component.parent).or_default().push(placement);
        write_occurrence(&mut context, id, component, parent, child, placement);
    }
    write_representation(&mut context, &root, &items[&None]);
    for (&id, product) in &products {
        write_representation(&mut context, product, &items[&Some(id)]);
    }
    Ok(context.finish())
}

fn validate_rigid(matrix: Mat4) -> Result<(), IoError> {
    let invalid = || IoError::InvalidTopology {
        reason: "STEP assembly placement must be a finite, proper rigid affine transform".into(),
    };
    let tolerance = Tolerance::new().angular;
    if matrix.0.iter().flatten().any(|value| !value.is_finite())
        || matrix.0[3]
            .iter()
            .zip([0., 0., 0., 1.])
            .any(|(actual, expected)| (actual - expected).abs() > tolerance)
    {
        return Err(invalid());
    }
    let axes: [Vec3; 3] = std::array::from_fn(|column| {
        Vec3::new(
            matrix.0[0][column],
            matrix.0[1][column],
            matrix.0[2][column],
        )
    });
    for (i, axis) in axes.iter().enumerate() {
        for (j, other) in axes.iter().enumerate() {
            let expected = if i == j { 1. } else { 0. };
            if (axis.dot(*other) - expected).abs() > tolerance {
                return Err(invalid());
            }
        }
    }
    if (axes[0].cross(axes[1]).dot(axes[2]) - 1.).abs() > tolerance {
        return Err(invalid());
    }
    Ok(())
}

struct Product {
    definition: u64,
    representation: u64,
    context: u64,
    origin: u64,
    items: Vec<u64>,
    has_solid: bool,
}

fn product(
    context: &mut StepWriteContext,
    name: &str,
    brep: Option<u64>,
    uncertainty: f64,
) -> Result<Product, IoError> {
    context.options.product_name = name.into();
    let definition = context.write_product_structure().definition;
    let representation = context.next_id();
    let geometric = context
        .write_geometric_context(uncertainty, false)?
        .representation;
    let origin = write_placement(context, Mat4::identity());
    let mut items = vec![origin];
    items.extend(brep);
    let shape = context.next_id();
    context.write_entity(
        shape,
        "PRODUCT_DEFINITION_SHAPE",
        &format!("'','',#{definition})"),
    );
    let link = context.next_id();
    context.write_entity(
        link,
        "SHAPE_DEFINITION_REPRESENTATION",
        &format!("#{shape},#{representation})"),
    );
    Ok(Product {
        definition,
        representation,
        context: geometric,
        origin,
        items,
        has_solid: brep.is_some(),
    })
}

fn write_placement(context: &mut StepWriteContext, matrix: Mat4) -> u64 {
    context.write_axis2_placement(
        Point3::new(matrix.0[0][3], matrix.0[1][3], matrix.0[2][3]),
        Vec3::new(matrix.0[0][2], matrix.0[1][2], matrix.0[2][2]),
        Vec3::new(matrix.0[0][0], matrix.0[1][0], matrix.0[2][0]),
    )
}

fn write_representation(context: &mut StepWriteContext, product: &Product, items: &[u64]) {
    let items = items
        .iter()
        .map(|id| format!("#{id}"))
        .collect::<Vec<_>>()
        .join(",");
    context.write_entity(
        product.representation,
        if product.has_solid {
            "ADVANCED_BREP_SHAPE_REPRESENTATION"
        } else {
            "SHAPE_REPRESENTATION"
        },
        &format!("'',({items}),#{})", product.context),
    );
}

fn write_occurrence(
    context: &mut StepWriteContext,
    id: ComponentId,
    component: &Component,
    parent: &Product,
    child: &Product,
    placement: u64,
) {
    let occurrence = context.next_id();
    context.write_entity(
        occurrence,
        "NEXT_ASSEMBLY_USAGE_OCCURRENCE",
        &format!(
            "'{id}',{},'',#{},#{},'{id}')",
            step_string_literal(&component.name),
            parent.definition,
            child.definition
        ),
    );
    let shape = context.next_id();
    context.write_entity(
        shape,
        "PRODUCT_DEFINITION_SHAPE",
        &format!("'','',#{occurrence})"),
    );
    let transform = context.next_id();
    context.write_entity(
        transform,
        "ITEM_DEFINED_TRANSFORMATION",
        &format!("'','',#{},#{placement})", child.origin),
    );
    let relation = context.next_id();
    let _ = writeln!(
        context.entities,
        "#{relation} = (REPRESENTATION_RELATIONSHIP('','',#{},#{}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{transform}) SHAPE_REPRESENTATION_RELATIONSHIP());",
        child.representation, parent.representation
    );
    let link = context.next_id();
    context.write_entity(
        link,
        "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",
        &format!("#{relation},#{shape})"),
    );
}
