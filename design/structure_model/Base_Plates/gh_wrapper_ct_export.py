"""
Grasshopper Py3 wrapper: ct_anchor_milling

GH Inputs expected:
- geometry_payload
- calc_payload
- out_json_path
- bottom_face_mode
- process_only_passed
- enabled

GH Outputs:
- out
- package
- records
- timber_model_schema
- inspection_breps (timber column breps with milling cuts)
- json_path
"""

import importlib.util
import os
import sys
import traceback
from pathlib import Path

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
HELPER_PATH = os.path.join(ROOT, "ct_anchor_milling.py")


def _load_helper():
    module_name = "ct_anchor_milling"
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
        out = "CT export disabled."
        package = {}
        records = []
        timber_model_schema = {}
        inspection_breps = []
        json_path = None
    else:
        if "geometry_payload" not in globals() or not geometry_payload:
            raise ValueError("geometry_payload input is required")
        if "calc_payload" not in globals() or not calc_payload:
            raise ValueError("calc_payload input is required")

        effective_geometry_payload = geometry_payload
        if isinstance(calc_payload, dict):
            maybe_synced = calc_payload.get("synced_geometry_payload")
            if isinstance(maybe_synced, dict) and maybe_synced:
                effective_geometry_payload = maybe_synced

        target_path = Path(str(out_json_path)) if "out_json_path" in globals() and out_json_path else (Path(ROOT) / "ct_anchor_milling_export.json")

        package = helper.export_ct_json(
            geometry_payload=effective_geometry_payload,
            calc_payload=calc_payload,
            out_json_path=target_path,
            bottom_face_mode=str(bottom_face_mode) if "bottom_face_mode" in globals() and bottom_face_mode else None,
            process_only_passed=bool(process_only_passed) if "process_only_passed" in globals() else False,
        )
        records = package.get("records", [])
        timber_model_schema = package.get("timber_model_schema", {})
        inspection_breps = helper.build_inspection_breps(records)
        json_path = str(target_path)
        out = "Exported {0} CT records".format(len(records))

except Exception:
    out = traceback.format_exc()
    package = {}
    records = []
    timber_model_schema = {}
    inspection_breps = []
    json_path = None
