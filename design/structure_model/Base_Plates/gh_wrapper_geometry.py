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
HELPER_PATH = os.path.join(ROOT, "base_plate_geometry.py")


def _load_helper():
    module_name = "base_plate_geometry"
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
        out = "Base plate geometry disabled."
        payload = {}
        members = []
        base_plates = []
        preview_breps = []
    else:
        if "payload_override" in globals() and isinstance(payload_override, dict) and payload_override:
            payload = payload_override
        else:
            kwargs = {}
            if "line_model_path" in globals() and line_model_path:
                from pathlib import Path
                kwargs["line_model_path"] = Path(str(line_model_path))
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
        preview_breps = helper.build_preview_breps([
            helper.BasePlateRecord(**bp) for bp in base_plates
        ]) if hasattr(helper, "BasePlateRecord") else []
        out = "Loaded {0} members and built {1} base plates".format(len(members), len(base_plates))

except Exception:
    out = traceback.format_exc()
    payload = {}
    members = []
    base_plates = []
    preview_breps = []
