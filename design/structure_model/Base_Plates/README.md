# Base Plates Scaffold

This folder now contains a three-stage scaffold:

1. `base_plate_geometry.py`
- Reads latest line model export from `design/line_model/data/*shifted_lines*.json`
- Extracts member dimensions, inclination, index, group
- Builds base plate records with two bottom-face orientation modes

2. `base_plate_calculations.py`
- Validates base plate thickness and minimum corner clearance
- Iteratively adjusts bottom face along member axis to resolve corner collisions
- Includes full Part 1/2/3 engineering formulas from your GH script
- Synchronizes engineering sizing (plate + hole layout) back into geometry payload before CT export
- Produces GH-compatible keys: `a..n`, `pass_fail_summary`, `utilization_values`, `combined_report`

3. `ct_anchor_milling.py`
- Merges geometry + calculations results
- Builds CT milling records (holes + slot) from synchronized geometry
- Reconstructs timber column breps and applies milling cuts for inspection
- Exports JSON for CT environment
- Adds explicit `timber_model_schema` block aligned to `design/timber_design/data/timber_model_0.json` shape
- Exports explicit milling tolerance parameters under `records[*].milling.tolerances`

## Typical Usage

```python
from pathlib import Path
from Base_Plates.base_plate_geometry import build_geometry_payload
from Base_Plates.base_plate_calculations import run_validation
from Base_Plates.ct_anchor_milling import export_ct_json

geometry = build_geometry_payload(
    bottom_face_mode="Perpendicular_to_grain",  # or "Parallel_to_ground"
)

calc = run_validation(
    geometry_payload=geometry,
    bottom_face_mode="Perpendicular_to_grain",
    min_allowable_clearance=80.0,
  sync_plate_dimensions_from_engineering=True,
  sync_iterations=1,
)

out = export_ct_json(
  geometry_payload=calc["synced_geometry_payload"],
    calc_payload=calc,
    out_json_path=Path("design/structure_model/Base_Plates/ct_anchor_milling_export.json"),
)
```

## GH Py3 Wrappers

Paste-ready wrapper scripts are included in this folder:

- `gh_wrapper_geometry.py`
- `gh_wrapper_calculations.py`
- `gh_wrapper_ct_export.py`

Each wrapper maps GH inputs/outputs and delegates to the corresponding module.
`gh_wrapper_geometry.py` accepts optional `payload_override` so you can preview synced geometry directly.
`gh_wrapper_calculations.py` now outputs `synced_geometry_payload` and supports sync controls.
`gh_wrapper_ct_export.py` automatically prefers `calc_payload.synced_geometry_payload` when provided.

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

- Plate dimensions in CT geometry come from `calc_payload.adjusted_base_plates` after sync.
- CT does not generate base-plate geometry. Base-plate geometry remains owned by geometry/calculation modules.
- Hole spacing/sizing comes from `calc_payload.engineering.sizing_recommendations` into `records[*].milling` and explicit `records[*].milling_geometry.holes` primitives.
- Slot dimensions come from `calc_payload.engineering.fabrication_parameters` into explicit `records[*].milling_geometry.slot` primitive.
- Fabrication tolerances come from `calc_payload.engineering.fabrication_parameters` into `records[*].milling.tolerances`.
- Recommended practice: keep tolerance values as independent process inputs (machine/tooling profile) and let sizing be design-driven.

## Reused Concepts From `foundation_solver.py`

No edits were made to `design/line_model/foundation_solver.py`.
The scaffold reused its strategy ideas:

- Projected conflict logic:
  using pairwise geometry conflict checks between neighboring members
- Iterative resolution:
  applying incremental shifts (`shift_step`) until clearance passes or max attempts reached

This mirrors the solver pattern there: detect conflict -> iterate upward/along axis -> stop when non-intersecting.
