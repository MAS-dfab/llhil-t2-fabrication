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
- enabled

GH Outputs:
- out
- payload
- report_text
- combined_report
- pass_fail_summary
- utilization_values
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
HELPER_PATH = os.path.join(ROOT, "base_plate_calculations.py")


def _load_helper():
    module_name = "base_plate_calculations"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    helper = _load_helper()

    if "enabled" in globals() and enabled is False:
        out = "Base plate calculations disabled."
        payload = {}
    else:
        if "geometry_payload" not in globals() or not geometry_payload:
            raise ValueError("geometry_payload input is required")

        kwargs = {
            "geometry_payload": geometry_payload,
            "bottom_face_mode": str(bottom_face_mode) if "bottom_face_mode" in globals() and bottom_face_mode else "Perpendicular_to_grain",
            "min_allowable_clearance": float(min_allowable_clearance) if "min_allowable_clearance" in globals() and min_allowable_clearance is not None else 80.0,
            "shift_step": float(shift_step) if "shift_step" in globals() and shift_step is not None else 10.0,
            "max_shift": float(max_shift) if "max_shift" in globals() and max_shift is not None else 400.0,
            "base_plate_min_thickness": float(base_plate_min_thickness) if "base_plate_min_thickness" in globals() and base_plate_min_thickness is not None else 10.0,
            "base_plate_max_thickness": float(base_plate_max_thickness) if "base_plate_max_thickness" in globals() and base_plate_max_thickness is not None else 60.0,
            "sync_plate_dimensions_from_engineering": bool(sync_plate_dimensions_from_engineering) if "sync_plate_dimensions_from_engineering" in globals() else True,
            "sync_iterations": int(sync_iterations) if "sync_iterations" in globals() and sync_iterations is not None else 1,
        }
        if "engineering_overrides" in globals() and engineering_overrides:
            kwargs["engineering_overrides"] = engineering_overrides

        payload = helper.run_validation(**kwargs)
        out = "Validated {0} members".format(len(payload.get("member_results", [])))

    report_text = payload.get("report_text", "") if payload else ""
    combined_report = payload.get("combined_report", "") if payload else ""
    pass_fail_summary = payload.get("pass_fail_summary", {}) if payload else {}
    utilization_values = payload.get("utilization_values", {}) if payload else {}
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
    payload = {}
    report_text = ""
    combined_report = ""
    pass_fail_summary = {}
    utilization_values = {}
    synced_geometry_payload = {}
    a = b = c = d = e = f = g = h = i = j = k = l = m = n = None
