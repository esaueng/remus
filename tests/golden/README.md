# Golden File Tests

The implemented harness is
`crates/operations/tests/golden_regression.rs`. It records deterministic,
six-decimal snapshots of measurements, bounding boxes, centers of mass, and
selected sorted mesh vertices in `tests/golden/data/`.

Run only the golden regression target with:

```bash
cargo test -p brepkit-operations --test golden_regression
```

When an intentional geometry change alters a snapshot, inspect the diff first,
then regenerate with:

```bash
UPDATE_GOLDEN=1 cargo test -p brepkit-operations --test golden_regression
git diff -- tests/golden/data
```

Do not update snapshots merely to make a test green. Confirm that volume,
bounds, center of mass, surface identity, and tessellation changes match the
intended operation. Keep output deterministic and compact; round values at the
serialization boundary rather than comparing raw floating-point text.

File names follow `{shape}_{operation}.golden`, for example
`boolean_box_minus_cylinder.golden`.
