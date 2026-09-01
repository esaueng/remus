# STEP conformance

Remus writes solid B-Rep geometry using the AP203
`CONFIG_CONTROL_DESIGN` schema and treats STEP entity support, not the header
schema label, as the reader boundary. This page records the implemented
interoperability contract and every known deviation from the current
[MBx-IF Recommended Practice for Geometric and Assembly Validation Properties](https://www.mbx-if.org/home/wp-content/uploads/2024/05/rec_prac_gvp_v46.pdf)
(version 4.6, 2023-04-21).

## Geometric validation properties

Validation properties are an explicit writer opt-in with
`validationProperties: true`; the default remains the historical STEP export
contract. When enabled, the writer:

- identifies Recommended Practice version 4.6 in `FILE_DESCRIPTION`;
- emits a product-level aggregate and a geometry-level declaration for every
  `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS`;
- assigns each solid through the AP203-compatible `SHAPE_ASPECT` →
  `PROPERTY_DEFINITION('','Shape for Validation Properties',...)` →
  `SHAPE_DEFINITION_REPRESENTATION` → `SHAPE_REPRESENTATION` chain;
- combines volume, surface area, and volume centroid in one empty-name
  `REPRESENTATION`, distinguished by the required representation-item names;
- uses the Recommended Practice's derived cubic-millimetre and
  square-millimetre unit entities, and millimetres for the centroid.

Surface area includes inner-shell surfaces. Volume and centroid use Gauss
quadrature over the exact analytic or NURBS face geometry. Analytic and
supported planar face areas are analytic;
unsupported trimmed NURBS area uses the established tessellation fallback at
`1e-4` mm deflection. Declarations are rejected instead of written if any
value is non-finite or non-positive.

`read_step_with_validation` opts into verification. Ordinary `read_step`
remains tolerant of property metadata it does not consume. The checked API
parses the geometry-level assignment, resolves explicit area and volume units,
recomputes properties after import/healing, and returns one report per solid.
A property deviation does not discard otherwise valid geometry; a malformed
declaration is a typed error and the complete import rolls back.

Default comparison bounds:

| Property | Bound |
| --- | --- |
| Volume | 0.5% relative |
| Surface area | 0.5% relative |
| Centroid | max(0.02 mm, 0.1% of solid AABB diagonal) |

All four bounds are caller-configurable through `StepValidationOptions` and
the direct/batch WASM contracts. Bounds must be finite and non-negative.

Stable report/refusal codes:

| Code | Category | Meaning |
| --- | --- | --- |
| `step_validation_properties_missing` | `unsupported` | No complete per-solid declaration was present. |
| `step_validation_volume_deviation` | `tolerance_violation` | Volume exceeds its relative bound. |
| `step_validation_surface_area_deviation` | `tolerance_violation` | Surface area exceeds its relative bound. |
| `step_validation_centroid_deviation` | `tolerance_violation` | Centroid distance exceeds its scale-aware bound. |
| `step_validation_invalid_options` | `invalid_input` | A comparison bound is negative or non-finite. |
| `step_validation_broken_assignment` | `invalid_input` | The geometry assignment chain is incomplete. |
| `step_validation_ambiguous_geometry` | `invalid_input` | One geometry assignment does not identify exactly one solid. |
| `step_validation_broken_property_chain` | `invalid_input` | A property-to-representation reference is missing or ambiguous. |
| `step_validation_duplicate_value` | `invalid_input` | A combined representation repeats one value. |
| `step_validation_incomplete_declaration` | `invalid_input` | Volume, area, or centroid is absent. |
| `step_validation_invalid_measure` | `invalid_input` | A measure is non-positive, non-finite, or has the wrong select type. |
| `step_validation_invalid_centroid` | `invalid_input` | The centroid is not a finite 3D point. |
| `step_validation_invalid_unit` | `invalid_input` | The declared unit is missing, dimensionally wrong, or cannot be resolved. |

## Known deviations and limits

- The writer remains AP203; AP242 schema output is O5.3. The geometry-level
  assignment therefore uses the Recommended Practice's Shape Representation
  method rather than `GEOMETRIC_ITEM_SPECIFIC_USAGE`.
- The public writer exports solids only. It emits no independent surfaces,
  curves, or points, so independent-curve length and the other class-specific
  properties are not applicable. B-Rep edge lengths are deliberately not
  mislabeled as independent-curve length.
- Product-level aggregate declarations are emitted but the checked reader
  compares geometry-level per-solid declarations only. Assembly-aware
  product/occurrence verification follows O5.1.
- Cloud-of-points sampling properties, tessellated validation properties,
  assembly validation properties, PMI validation properties, and QIF mappings
  are not implemented.
- Validation declarations are checked during import and returned to the
  caller; they are not persisted as topology attributes for a later re-export.

## Evidence

- `crates/io/tests/step_validation_properties.rs`: analytic box oracles,
  multi-solid product/per-solid round trip, both sides of the default volume
  boundary, missing-property reporting, unit/type refusal, and rollback.
- `crates/wasm/src/bindings/io.rs` and `batch.rs`: direct and
  `executeBatchV2` contract tests.
- `scripts/test-wasm-smoke.mjs`: shipped-package direct/batch success,
  deviation diagnostics, stable malformed-measure refusal, and rollback.
