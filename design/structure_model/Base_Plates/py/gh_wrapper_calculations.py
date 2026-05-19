"""
Grasshopper Py3 wrapper: base_plate_calculations

GH Inputs expected:
- geometry_payload  (Item Access; receives one opaque geometry payload token)
- bottom_face_mode is read from geometry_payload
- min_allowable_clearance
- shift_step
- max_shift
- base_plate_min_thickness
- base_plate_max_thickness
- sync_plate_dimensions_from_engineering
- sync_iterations
- engineering_overrides
- Design/fabrication dimensions come from geometry_payload.
  Use engineering_overrides only for explicit analysis testing.
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
- payload  (opaque payload token; keep downstream inputs on Item Access)
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
- debug_status
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

try:
    import Rhino.Geometry as rg  # type: ignore
except Exception:
    rg = None

try:
    import scriptcontext as sc  # type: ignore
except Exception:
    sc = None

try:
    from Grasshopper.Kernel.Types import GH_ObjectWrapper  # type: ignore
except Exception:
    GH_ObjectWrapper = None

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
HELPER_PATH = os.path.join(ROOT, "py", "base_plate_calculations.py")
RUN_TAG = "BPC_WRAPPER_SYNC_2026_05_19_GEOMETRY_PAYLOAD_CHECKS"

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
        if isinstance(normalized, str) and sc is not None and normalized in sc.sticky:
            normalized = sc.sticky[normalized]
            continue
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


def _component_key():
    try:
        return str(ghenv.Component.InstanceGuid)  # type: ignore[name-defined]
    except Exception:
        return "standalone"


def _store_payload_reference(kind, value):
    if GH_ObjectWrapper is not None:
        print("[GH Calc Wrapper] payload output mode=gh_object_wrapper kind={0}".format(kind))
        return GH_ObjectWrapper(value)
    if sc is None:
        return value
    token = "BPG_PAYLOAD::{0}::{1}".format(kind, _component_key())
    sc.sticky[token] = value
    print("[GH Calc Wrapper] payload output mode=reference_token kind={0}".format(kind))
    return token


def _build_synced_plate_preview_breps(payload_dict):
    if not isinstance(payload_dict, dict):
        return []

    # Prefer high-fidelity preview geometry passed through from geometry wrapper.
    preview_block = payload_dict.get("preview")
    if isinstance(preview_block, dict):
        preview_breps = preview_block.get("breps_doc_units")
        if isinstance(preview_breps, list):
            return [item for item in preview_breps if item is not None]

    footing_breps = payload_dict.get("footing_breps")
    if isinstance(footing_breps, list) and footing_breps:
        return [item for item in footing_breps if item is not None]

    # Fallback: reconstruct simplified boxes from synced base plate records.
    if rg is None:
        return []
    plates = payload_dict.get("base_plates", [])
    if not isinstance(plates, list):
        return []

    breps = []
    for plate in plates:
        if not isinstance(plate, dict):
            continue
        try:
            center = plate.get("center")
            x_axis = plate.get("x_axis")
            y_axis = plate.get("y_axis")
            length = float(plate.get("length", 0.0) or 0.0)
            width = float(plate.get("width", 0.0) or 0.0)
            thickness = float(plate.get("thickness", 0.0) or 0.0)
            if not center or not x_axis or not y_axis:
                continue
            if length <= 0.0 or width <= 0.0 or thickness <= 0.0:
                continue

            origin = rg.Point3d(float(center[0]), float(center[1]), float(center[2]))
            plane = rg.Plane(
                origin,
                rg.Vector3d(float(x_axis[0]), float(x_axis[1]), float(x_axis[2])),
                rg.Vector3d(float(y_axis[0]), float(y_axis[1]), float(y_axis[2])),
            )
            box = rg.Box(
                plane,
                rg.Interval(-0.5 * length, 0.5 * length),
                rg.Interval(-0.5 * width, 0.5 * width),
                rg.Interval(0.0, thickness),
            )
            if not box.IsValid:
                continue
            brep = box.ToBrep()
            if brep is None:
                continue
            breps.append(brep)
        except Exception:
            continue
    return breps


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
        print("[GH Calc Wrapper DEBUG] geometry_payload_source=", geometry_payload_source)
        print("[GH Calc Wrapper DEBUG] geometry_payload type=", type(geometry_payload))
        if isinstance(geometry_payload, dict):
            print("[GH Calc Wrapper DEBUG] geometry_payload keys=", list(geometry_payload.keys()))
        else:
            print("[GH Calc Wrapper DEBUG] geometry_payload value=", geometry_payload)
        if not isinstance(geometry_payload, dict):
            raise TypeError(
                "geometry payload from {0} did not normalize to a dict; got {1}. "
                "Keep geometry_payload on Item Access; List/Tree Access can explode the payload.".format(
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
            "bottom_face_mode": str(geometry_bottom_face_mode or "Perpendicular_to_grain"),
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
        # Geometry and fabrication dimensions are designer-facing and are
        # extracted from geometry_payload. Keep scalar calc inputs limited to
        # analysis toggles/thresholds so stale GH wires cannot override geometry.
        for name in (
            "run_aisc_steel_node_checks",
            "run_stress_concentration_check",
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
        print("[GH Calc Wrapper DEBUG] payload type=", type(payload))
        if isinstance(payload, dict):
            print("[GH Calc Wrapper DEBUG] payload keys=", list(payload.keys()))
            print("[GH Calc Wrapper DEBUG] member_results count=", len(payload.get("member_results", [])))
        else:
            print("[GH Calc Wrapper DEBUG] payload value=", payload)
        out = "Validated {0} members".format(len(payload.get("member_results", [])))

    payload_data = payload
    report_text = payload_data.get("report_text", "") if payload_data else ""
    combined_report = payload_data.get("combined_report", "") if payload_data else ""
    pass_fail_summary = payload_data.get("pass_fail_summary", {}) if payload_data else {}
    utilization_values = payload_data.get("utilization_values", {}) if payload_data else {}
    sizing_recommendations = payload_data.get("engineering", {}).get("sizing_recommendations", {}) if payload_data else {}
    steel_node_report = payload_data.get("steel_node_report", "") if payload_data else ""
    steel_node_checks = payload_data.get("steel_node_checks", {}) if payload_data else {}
    steel_node_recommendations = payload_data.get("steel_node_recommendations", {}) if payload_data else {}
    corner_radius_recommendation = sizing_recommendations.get("corner_radius") if sizing_recommendations else None
    annotation_payload = payload_data.get("annotation_payload", {}) if payload_data else {}
    layout_payload = payload_data.get("layout_payload", {}) if payload_data else {}
    critical_dimensions = payload_data.get("critical_dimensions", {}) if payload_data else {}
    synced_geometry_payload = _store_payload_reference(
        "synced_geometry",
        payload_data.get("synced_geometry_payload", {}) if payload_data else {},
    )
    synced_preview_breps = _build_synced_plate_preview_breps(
        payload_data.get("synced_geometry_payload", {}) if payload_data else {}
    )
    synced_preview_payload = payload_data.get("synced_geometry_payload", {}) if payload_data else {}
    if isinstance(synced_preview_payload, dict) and isinstance(synced_preview_payload.get("preview"), dict) and isinstance((synced_preview_payload.get("preview") or {}).get("breps_doc_units"), list):
        synced_preview_source = "preview.breps_doc_units"
    elif isinstance(synced_preview_payload, dict) and isinstance(synced_preview_payload.get("footing_breps"), list) and synced_preview_payload.get("footing_breps"):
        synced_preview_source = "footing_breps"
    else:
        synced_preview_source = "base_plate_box_fallback"

    a = payload_data.get("a") if payload_data else None
    b = payload_data.get("b") if payload_data else None
    c = payload_data.get("c") if payload_data else None
    d = payload_data.get("d") if payload_data else None
    e = payload_data.get("e") if payload_data else None
    f = payload_data.get("f") if payload_data else None
    g = payload_data.get("g") if payload_data else None
    h = payload_data.get("h") if payload_data else None
    i = payload_data.get("i") if payload_data else None
    j = payload_data.get("j") if payload_data else None
    k = payload_data.get("k") if payload_data else None
    l = payload_data.get("l") if payload_data else None
    m = payload_data.get("m") if payload_data else None
    n = payload_data.get("n") if payload_data else None
    payload = _store_payload_reference("calculations", payload_data)
    debug_status = {
        "run_tag": RUN_TAG,
        "payload_type": type(payload_data).__name__,
        "member_result_count": len(payload_data.get("member_results", [])) if payload_data else 0,
        "adjusted_plate_count": len(payload_data.get("adjusted_base_plates", [])) if payload_data else 0,
        "synced_preview_brep_count": len(synced_preview_breps),
        "synced_preview_source": synced_preview_source,
        "has_report_text": bool(report_text),
        "has_combined_report": bool(combined_report),
        "annotation_dimension_count": len((annotation_payload or {}).get("dimensions", {})) if isinstance(annotation_payload, dict) else 0,
        "layout_dimension_count": len((layout_payload or {}).get("dimension_summary", {})) if isinstance(layout_payload, dict) else 0,
        "b_timber_brep_is_none": b is None,
        "payload_transport": "gh_object_wrapper" if GH_ObjectWrapper is not None else ("sticky_token" if sc is not None else "direct_value"),
        "payload_output": payload,
        "synced_geometry_payload_output": synced_geometry_payload,
    }
    # Legacy / generic-output aliases for GH components that have not yet had
    # their output names refreshed after wrapper updates.
    o = debug_status
    p = out
    q = report_text
    r = synced_preview_breps

except Exception:
    out = traceback.format_exc()
    print("[GH Calc Wrapper] Exception caught:")
    print(out)
    payload = {}
    report_text = out
    combined_report = out
    pass_fail_summary = {"wrapper_error": True}
    utilization_values = {}
    sizing_recommendations = {}
    steel_node_report = out
    steel_node_checks = {"wrapper_error": True}
    steel_node_recommendations = {}
    corner_radius_recommendation = None
    annotation_payload = {}
    layout_payload = {}
    critical_dimensions = {}
    synced_geometry_payload = {}
    synced_preview_breps = []
    debug_status = {
        "run_tag": RUN_TAG,
        "error": out,
    }
    a = report_text
    b = c = d = e = f = g = h = None
    i = utilization_values
    j = pass_fail_summary
    k = steel_node_report
    l = ""
    m = n = None
    o = debug_status
    p = out
    q = report_text
    r = synced_preview_breps
