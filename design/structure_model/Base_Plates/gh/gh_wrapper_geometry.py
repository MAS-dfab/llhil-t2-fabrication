"""
Grasshopper Py3 wrapper: base_plate_geometry

GH Inputs expected:
- payload_override
- line_model_path
- plate_length
- plate_width
- plate_thickness
- bottom_face_mode
- include_hierarchies
- enabled

GH Outputs:
- out
- payload
- members
- base_plates
- preview_breps
"""

import importlib.util
import os
import sys
import traceback

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
DEFAULT_PLATE_LENGTH = 0.8
DEFAULT_PLATE_WIDTH = 0.8
DEFAULT_PLATE_THICKNESS = 0.02

# Diagnostic: print resolved path
print("[GH Wrapper] ROOT:", ROOT)
HELPER_PATH = os.path.join(ROOT, "base_plate_geometry.py")
print("[GH Wrapper] HELPER_PATH:", HELPER_PATH)
print("[GH Wrapper] HELPER_EXISTS:", os.path.exists(HELPER_PATH))



def _load_helper():
    module_name = "base_plate_geometry"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
    # Required for dataclasses/type resolution during module execution in GH Py3.
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception as e:
        print("[GH Wrapper] Error importing base_plate_geometry.py:", e)
        import traceback
        print(traceback.format_exc())
        raise
    return module


try:
    helper = _load_helper()

    if "enabled" in globals() and enabled is False:
        out = "Base plate geometry disabled."
        payload = {}
        members = []
        base_plates = []
        preview_breps = []
    else:
        if "payload_override" in globals() and isinstance(payload_override, dict) and payload_override:
            payload = payload_override
        else:
            kwargs = {
                "plate_length": DEFAULT_PLATE_LENGTH,
                "plate_width": DEFAULT_PLATE_WIDTH,
                "plate_thickness": DEFAULT_PLATE_THICKNESS,
                "bottom_face_mode": "Perpendicular_to_grain",
            }
            if "line_model_path" in globals() and line_model_path:
                from pathlib import Path
                kwargs["line_model_path"] = Path(str(line_model_path))
            else:
                # Fallback for GH context: use absolute path to repo's line_model data
                from pathlib import Path
                fallback_path = Path(r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\line_model\data\meters_shifted_lines.json")
                if fallback_path.exists():
                    kwargs["line_model_path"] = fallback_path
                else:
                    # Try alternate filenames
                    for alt in ["0806_shifted_lines.json", "shifted_lines.json"]:
                        alt_path = fallback_path.parent / alt
                        if alt_path.exists():
                            kwargs["line_model_path"] = alt_path
                            break
            if "plate_length" in globals() and plate_length is not None:
                kwargs["plate_length"] = float(plate_length)
            if "plate_width" in globals() and plate_width is not None:
                kwargs["plate_width"] = float(plate_width)
            if "plate_thickness" in globals() and plate_thickness is not None:
                kwargs["plate_thickness"] = float(plate_thickness)
            if "bottom_face_mode" in globals() and bottom_face_mode:
                kwargs["bottom_face_mode"] = str(bottom_face_mode)
            if "include_hierarchies" in globals() and include_hierarchies:
                kwargs["include_hierarchies"] = include_hierarchies

            payload = helper.build_geometry_payload(**kwargs)
        members = payload.get("members", [])
        base_plates = payload.get("base_plates", [])
        print("[GH Wrapper] Extracted members: {0}".format(len(members)))
        print("[GH Wrapper] Extracted base_plates: {0}".format(len(base_plates)))
        
        preview_breps = []
        if hasattr(helper, "BasePlateRecord") and base_plates:
            try:
                bp_records = [helper.BasePlateRecord(**bp) for bp in base_plates]
                preview_breps = helper.build_preview_breps(bp_records)
                print("[GH Wrapper] Built {0} preview breps".format(len(preview_breps)))
            except Exception as bp_err:
                print("[GH Wrapper] Error building preview breps: {0}".format(bp_err))
                import traceback
                print(traceback.format_exc())
        
        unit_hint = ""
        if members and base_plates:
            try:
                mw = float((members[0] or {}).get("width") or 0.0)
                pl = float((base_plates[0] or {}).get("length") or 0.0)
                if mw > 0.0 and pl / mw > 20.0:
                    unit_hint = " | WARNING: plate/member scale looks too large; check mm vs m"
            except Exception:
                pass
        out = "Loaded {0} members and built {1} base plates{2}".format(len(members), len(base_plates), unit_hint)
        print("[GH Wrapper] " + out)

except Exception:
    out = traceback.format_exc()
    payload = {}
    members = []
    base_plates = []
    preview_breps = []
    print("[GH Wrapper] Exception caught:")
    print(out)
