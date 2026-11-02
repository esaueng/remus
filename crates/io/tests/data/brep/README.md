# Saved-shape fixtures

Only geometry payloads are retained; archive XML and application metadata are
excluded. Trailing whitespace is stripped; all numeric geometry and topology
records are retained. No reference library is linked or executed by the tests.

- `freecad-box-v1.brep`: the geometry records from the `PartShape.brp` entry of FreeCAD's public
  `src/Mod/Fem/femtest/data/calculix/box.FCStd` at tag `1.1.3`.
  Source: <https://github.com/FreeCAD/FreeCAD/blob/1.1.3/src/Mod/Fem/femtest/data/calculix/box.FCStd>.
  The source project is distributed under LGPL-2.1-or-later; this fixture is
  retained as third-party test data, outside the Rust implementation's license.
  It is a 10 mm cube with identity locations encoded both directly and as a
  composition. Expected volume: 1000 mm³; area: 600 mm².
- `box-v3.brep`: the `Body.brp` entry of OpenZCAD's synthetic
  `test/fixtures/freecad/two-solids.FCStd` at commit
  `a4543025652198bb226c1282b1af0c43e43e020e` (Apache-2.0).
  Source: <https://github.com/esaueng/OpenZCAD/blob/a4543025652198bb226c1282b1af0c43e43e020e/test/fixtures/freecad/two-solids.FCStd>.
  It is a 10 × 20 × 30 mm box. Expected volume: 6000 mm³; area: 2200 mm².

These are format fixtures, not a representative FreeCAD compatibility corpus.
Curved models, real placements, cavities and binary saves remain unqualified.

- `box-v2.brep`: a format control derived from the V3 box by writing the V2
  header and the four explicit UV endpoint coordinates for each linear pcurve.
  These redundant cache values exist in V2 only, not V1 or V3. Geometry and
  physical expectations are unchanged. This is synthetic format coverage.
