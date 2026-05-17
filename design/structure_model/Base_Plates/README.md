# Base Plates Scaffold

This folder now contains a three-stage scaffold. Live code lives under `Base_Plates\py`:

1. `base_plate_geometry.py`
- Reads the latest line model export, preferring `design/line_model/data/260516_v1_line_model.json`
- Extracts member dimensions, inclination, index, group
- Builds base plate records with two bottom-face orientation modes
- Normalizes geometry payloads to meters, including the inlined footing geometry

2. `base_plate_calculations.py`
- Validates base plate thickness and minimum corner clearance
- Iteratively adjusts bottom face along member axis to resolve corner collisions
- Includes full Part 1/2/3 engineering formulas from your GH script
- Adds an AISC-style steel-node detailing block for corner-radius minima and gusset/stiffener warnings
- Produces live engineering sizing recommendations for downstream geometry use
- Converts meter geometry to millimeters only for validation/math
- Produces GH-compatible keys: `a..n`, `pass_fail_summary`, `utilization_values`, `combined_report`

3. `ct_anchor_milling.py`
- Merges geometry + calculations results
- Builds CT milling records (holes + slot) from synchronized geometry
- Reconstructs timber column breps and applies milling cuts for inspection
- Exports JSON for CT environment
- Keeps CT/BTLx records in millimeters while rebuilding inspection Breps in the source geometry units for Rhino preview
- Adds explicit `timber_model_schema` block aligned to `design/timber_design/data/timber_model_0.json` shape
- Exports explicit milling tolerance parameters under `records[*].milling.tolerances`

## Typical Usage

```python
from pathlib import Path
from Base_Plates.py.base_plate_geometry import build_geometry_payload
from Base_Plates.py.base_plate_calculations import run_validation
from Base_Plates.py.ct_anchor_milling import export_ct_json

geometry = build_geometry_payload(
    bottom_face_mode="Perpendicular_to_grain",  # or "Parallel_to_ground"
)

calc = run_validation(
    geometry_payload=geometry,
    bottom_face_mode="Perpendicular_to_grain",
    min_allowable_clearance=80.0,
)

resolved_geometry = build_geometry_payload(
    bottom_face_mode="Perpendicular_to_grain",
    geometry_kind="footing",
    sizing_recommendations=calc["engineering"]["sizing_recommendations"],
)

out = export_ct_json(
    geometry_payload=resolved_geometry,
    calc_payload=calc,
    out_json_path=Path("design/structure_model/Base_Plates/ct_anchor_milling_export.json"),
)
```

## GH Py3 Wrappers

Paste-ready wrapper scripts live under `Base_Plates\py`:

- `gh_wrapper_geometry.py`
- `gh_wrapper_calculations.py`
- `gh_wrapper_ct_export.py`

Each wrapper maps GH inputs/outputs and delegates to the corresponding module.
Standard GH should use a two-pass handoff, `geometry -> calculations -> resolved geometry`, rather than a literal cyclic dependency.

## Bottom Face Modes

- `Perpendicular_to_grain`: bottom face normal follows member axis
- `Parallel_to_ground`: bottom face normal is global +Z

## CT Schema Fields

`ct_anchor_milling.export_ct_json` writes:

- `metadata`
- `records` (module-native CT anchor records)
- `timber_model_schema`

`timber_model_schema` mirrors the timber model structure used in `timber_design`:

- `data.elements.<guid>.data.edge`
- `data.elements.<guid>.data.features`
- `data.elements.<guid>.data.frame`
- `data.elements.<guid>.data.group`
- `data.elements.<guid>.data.height`
- `data.elements.<guid>.data.length`
- `data.elements.<guid>.data.level`
- `data.elements.<guid>.data.width`
- `data.elements.<guid>.dtype = compas_timber.elements/Beam`
- `data.elements.<guid>.guid`
- `data.elements.<guid>.name`

## Sizing And Tolerance Sync

- Geometry is meter-native; calculations publish `engineering.sizing_recommendations` in millimeters.
- On an optional second geometry pass, footing geometry can consume those recommendations so live code outputs drive the parametric footing.
- `engineering.sizing_recommendations` now also carries `corner_radius_min`, `corner_radius_preferred`, and the currently selected `corner_radius` for the future fillet geometry pass.
- Explicit GH overrides still win over recommended values.
- CT does not generate base-plate geometry. Base-plate geometry remains owned by geometry/calculation modules.
- Hole spacing/sizing comes from `calc_payload.engineering.sizing_recommendations` into `records[*].milling` and explicit `records[*].milling_geometry.holes` primitives.
- Slot dimensions come from `calc_payload.engineering.fabrication_parameters` into explicit `records[*].milling_geometry.slot` primitive.
- Fabrication tolerances come from `calc_payload.engineering.fabrication_parameters` into `records[*].milling.tolerances`.
- Recommended practice: keep tolerance values as independent process inputs (machine/tooling profile) and let sizing be design-driven.

## Units Contract

- Geometry payloads are emitted in meters.
- The legacy footing reference dimensions were authored in millimeters and are converted once to meters inside `base_plate_geometry.py`.
- Cluster footings now derive the four embedded-plate azimuth/altitude pairs from the four incident line-model members at that support node; the ordered member indices are exposed in `metadata.support_cluster_member_indices`.
- Embedded timber-plate holes default to the code baseline pattern (`2 x 2`, `13 mm` holes, `100 mm` pitch, `60 mm` gage), not the original Rhino reference-hole pattern.
- Validation and engineering formulas remain millimeter-native internally.
- `run_validation()` converts geometry to millimeters for analysis and emits `engineering.sizing_recommendations` in millimeters for the optional resolved-geometry pass.
- `ct_anchor_milling.py` exports `records` in millimeters for CT/BTLx, and rescales those same records back to the source geometry units only when building inspection Breps for Rhino.
- `timber_model_schema` remains meter-based so it stays aligned with the surrounding `compas_timber` model convention.

## Reused Concepts From `foundation_solver.py`

No edits were made to `design/line_model/foundation_solver.py`.
The scaffold reused its strategy ideas:

- Projected conflict logic:
  using pairwise geometry conflict checks between neighboring members
- Iterative resolution:
  applying incremental shifts (`shift_step`) until clearance passes or max attempts reached

This mirrors the solver pattern there: detect conflict -> iterate upward/along axis -> stop when non-intersecting.
