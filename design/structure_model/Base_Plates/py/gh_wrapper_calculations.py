"""
Grasshopper Py3 wrapper: base_plate_calculations

GH Inputs expected:
- geometry_payload
- bottom_face_mode
- min_allowable_clearance
- shift_step
- max_shift
- base_plate_min_thickness
- base_plate_max_thickness
- sync_plate_dimensions_from_engineering
- sync_iterations
- engineering_overrides
- bolt_dia
- bolt_hole_dia
- hole_clearance
- project_min_bolt_hole_dia
- rows
- holes_per_row
- pitch_parallel
- gage_perp
- end_distance
- edge_distance
- run_aisc_steel_node_checks
- run_stress_concentration_check
- corner_radius
- corner_radius_mode
- stress_concentration_factor
- use_gusset_plates
- use_web_stiffeners
- weld_utilization_limit_for_gusset_warning
- plate_slenderness_limit
- min_corner_radius
- project_min_corner_radius
- preferred_corner_radius_factor
- unsupported_plate_width
- eccentricity_mm
- concentrated_force_kN
- wall_thickness_mm
- concentrated_force_per_wall_thickness_limit
- enabled

GH Outputs:
- out
- payload
- report_text
- combined_report
- pass_fail_summary
- utilization_values
- sizing_recommendations
- steel_node_report
- steel_node_checks
- steel_node_recommendations
- corner_radius_recommendation
- synced_geometry_payload
- a
- b
- c
- d
- e
- f
- g
- h
- i
- j
- k
- l
- m
- n
"""

import importlib.util
import os
import sys
import traceback

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
HELPER_PATH = os.path.join(ROOT, "py", "base_plate_calculations.py")
RUN_TAG = "BPC_WRAPPER_SYNC_2026_05_17_PAYLOAD_NORMALIZE"

print("[GH Calc Wrapper] RUN_TAG:", RUN_TAG)
print("[GH Calc Wrapper] HELPER_PATH:", HELPER_PATH)
print("[GH Calc Wrapper] HELPER_EXISTS:", os.path.exists(HELPER_PATH))


def _load_helper():
    module_name = "base_plate_calculations"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
    # Dataclasses resolve annotations through sys.modules during execution.
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _to_python_data(value):
    if isinstance(value, dict):
        return {key: _to_python_data(item) for key, item in value.items()}
    try:
        keys = list(value.Keys)
    except Exception:
        keys = None
    if keys is not None:
        return {
            _to_python_data(key): _to_python_data(value[key])
            for key in keys
        }
    if isinstance(value, (list, tuple)):
        return [_to_python_data(item) for item in value]
    if not isinstance(value, (str, bytes)):
        try:
            value.Count
            return [_to_python_data(item) for item in value]
        except Exception:
            pass
    return value


def _unwrap_payload(value):
    """Normalize the common GH forms of a single dictionary payload."""
    normalized = value
    while True:
        wrapped_value = getattr(normalized, "Value", None)
        if wrapped_value is not None and wrapped_value is not normalized:
            normalized = wrapped_value
            continue
        branches = getattr(normalized, "Branches", None)
        if branches is not None:
            normalized = _to_python_data(branches)
            continue
        normalized = _to_python_data(normalized)
        if isinstance(normalized, dict):
            return normalized
        if isinstance(normalized, (list, tuple)) and len(normalized) == 1:
            normalized = normalized[0]
            continue
        return normalized


def _payload_summary(value):
    if isinstance(value, dict):
        return "dict keys={0}".format(sorted(value.keys()))
    if isinstance(value, (list, tuple)):
        return "{0} len={1}".format(type(value).__name__, len(value))
    return type(value).__name__


try:
    helper = _load_helper()

    if "enabled" in globals() and enabled is False:
        out = "Base plate calculations disabled."
        payload = {}
    else:
        geometry_payload_input = None
        geometry_payload_source = None
        for candidate_name in ("geometry_payload", "payload", "geometry"):
            if candidate_name in globals() and globals()[candidate_name]:
                geometry_payload_input = globals()[candidate_name]
                geometry_payload_source = candidate_name
                break
        if geometry_payload_input is None:
            raise ValueError("geometry_payload input is required")
        geometry_payload = _unwrap_payload(geometry_payload_input)
        if not isinstance(geometry_payload, dict):
            raise TypeError(
                "geometry payload from {0} did not normalize to a dict; got {1}".format(
                    geometry_payload_source,
                    _payload_summary(geometry_payload),
                )
            )
        print(
            "[GH Calc Wrapper] geometry payload source={0}; {1}".format(
                geometry_payload_source,
                _payload_summary(geometry_payload),
            )
        )

        geometry_metadata = geometry_payload.get("metadata", {}) if isinstance(geometry_payload, dict) else {}
        geometry_bottom_face_mode = (
            geometry_metadata.get("bottom_face_mode")
            if isinstance(geometry_metadata, dict)
            else None
        )
        kwargs = {
            "geometry_payload": geometry_payload,
            "bottom_face_mode": str(bottom_face_mode)
            if "bottom_face_mode" in globals() and bottom_face_mode
            else str(geometry_bottom_face_mode or "Perpendicular_to_grain"),
            "min_allowable_clearance": float(min_allowable_clearance) if "min_allowable_clearance" in globals() and min_allowable_clearance is not None else 80.0,
            "shift_step": float(shift_step) if "shift_step" in globals() and shift_step is not None else 10.0,
            "max_shift": float(max_shift) if "max_shift" in globals() and max_shift is not None else 400.0,
            "base_plate_min_thickness": float(base_plate_min_thickness) if "base_plate_min_thickness" in globals() and base_plate_min_thickness is not None else 10.0,
            "base_plate_max_thickness": float(base_plate_max_thickness) if "base_plate_max_thickness" in globals() and base_plate_max_thickness is not None else 60.0,
            "sync_plate_dimensions_from_engineering": bool(sync_plate_dimensions_from_engineering) if "sync_plate_dimensions_from_engineering" in globals() else True,
            "sync_iterations": int(sync_iterations) if "sync_iterations" in globals() and sync_iterations is not None else 1,
        }
        merged_engineering_overrides = {}
        if "engineering_overrides" in globals() and engineering_overrides:
            normalized_engineering_overrides = _to_python_data(engineering_overrides)
            if isinstance(normalized_engineering_overrides, dict):
                merged_engineering_overrides.update(normalized_engineering_overrides)
        for name in (
            "bolt_dia",
            "bolt_hole_dia",
            "hole_clearance",
            "project_min_bolt_hole_dia",
            "rows",
            "holes_per_row",
            "pitch_parallel",
            "gage_perp",
            "end_distance",
            "edge_distance",
            "run_aisc_steel_node_checks",
            "run_stress_concentration_check",
            "corner_radius",
            "corner_radius_mode",
            "stress_concentration_factor",
            "use_gusset_plates",
            "use_web_stiffeners",
            "weld_utilization_limit_for_gusset_warning",
            "plate_slenderness_limit",
            "min_corner_radius",
            "project_min_corner_radius",
            "preferred_corner_radius_factor",
            "unsupported_plate_width",
            "eccentricity_mm",
            "concentrated_force_kN",
            "wall_thickness_mm",
            "concentrated_force_per_wall_thickness_limit",
        ):
            if name in globals() and globals()[name] is not None:
                merged_engineering_overrides[name] = globals()[name]
        if merged_engineering_overrides:
            kwargs["engineering_overrides"] = merged_engineering_overrides

        payload = helper.run_validation(**kwargs)
        out = "Validated {0} members".format(len(payload.get("member_results", [])))

    report_text = payload.get("report_text", "") if payload else ""
    combined_report = payload.get("combined_report", "") if payload else ""
    pass_fail_summary = payload.get("pass_fail_summary", {}) if payload else {}
    utilization_values = payload.get("utilization_values", {}) if payload else {}
    sizing_recommendations = payload.get("engineering", {}).get("sizing_recommendations", {}) if payload else {}
    steel_node_report = payload.get("steel_node_report", "") if payload else ""
    steel_node_checks = payload.get("steel_node_checks", {}) if payload else {}
    steel_node_recommendations = payload.get("steel_node_recommendations", {}) if payload else {}
    corner_radius_recommendation = sizing_recommendations.get("corner_radius") if sizing_recommendations else None
    synced_geometry_payload = payload.get("synced_geometry_payload", {}) if payload else {}

    a = payload.get("a") if payload else None
    b = payload.get("b") if payload else None
    c = payload.get("c") if payload else None
    d = payload.get("d") if payload else None
    e = payload.get("e") if payload else None
    f = payload.get("f") if payload else None
    g = payload.get("g") if payload else None
    h = payload.get("h") if payload else None
    i = payload.get("i") if payload else None
    j = payload.get("j") if payload else None
    k = payload.get("k") if payload else None
    l = payload.get("l") if payload else None
    m = payload.get("m") if payload else None
    n = payload.get("n") if payload else None

except Exception:
    out = traceback.format_exc()
    print("[GH Calc Wrapper] Exception caught:")
    print(out)
    payload = {}
    report_text = ""
    combined_report = ""
    pass_fail_summary = {}
    utilization_values = {}
    sizing_recommendations = {}
    steel_node_report = ""
    steel_node_checks = {}
    steel_node_recommendations = {}
    corner_radius_recommendation = None
    synced_geometry_payload = {}
    a = b = c = d = e = f = g = h = i = j = k = l = m = n = None
