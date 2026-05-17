"""
Grasshopper Py3 wrapper: ct_anchor_milling

GH Inputs expected:
- geometry_payload (optional when calc_payload.synced_geometry_payload is present)
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
HELPER_PATH = os.path.join(ROOT, "py", "ct_anchor_milling.py")
RUN_TAG = "CT_WRAPPER_SYNC_2026_05_17_PAYLOAD_NORMALIZE"

print("[GH CT Wrapper] RUN_TAG:", RUN_TAG)
print("[GH CT Wrapper] HELPER_PATH:", HELPER_PATH)
print("[GH CT Wrapper] HELPER_EXISTS:", os.path.exists(HELPER_PATH))


def _load_helper():
    module_name = "ct_anchor_milling"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
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
        out = "CT export disabled."
        package = {}
        records = []
        timber_model_schema = {}
        inspection_breps = []
        json_path = None
    else:
        calc_payload_input = None
        calc_payload_source = None
        for candidate_name in ("calc_payload", "calculations_payload", "calculation_payload", "payload"):
            if candidate_name in globals() and globals()[candidate_name]:
                calc_payload_input = globals()[candidate_name]
                calc_payload_source = candidate_name
                break
        if calc_payload_input is None:
            raise ValueError("calc_payload input is required")
        calc_payload = _unwrap_payload(calc_payload_input)
        if not isinstance(calc_payload, dict):
            raise TypeError(
                "calc payload from {0} did not normalize to a dict; got {1}".format(
                    calc_payload_source,
                    _payload_summary(calc_payload),
                )
            )
        print(
            "[GH CT Wrapper] calc payload source={0}; {1}".format(
                calc_payload_source,
                _payload_summary(calc_payload),
            )
        )

        geometry_payload_input = None
        geometry_payload_source = None
        for candidate_name in ("geometry_payload", "geometry"):
            if candidate_name in globals() and globals()[candidate_name]:
                geometry_payload_input = globals()[candidate_name]
                geometry_payload_source = candidate_name
                break
        geometry_payload = _unwrap_payload(geometry_payload_input) if geometry_payload_input is not None else {}
        maybe_synced = calc_payload.get("synced_geometry_payload")
        if geometry_payload and not isinstance(geometry_payload, dict):
            raise TypeError(
                "geometry payload from {0} did not normalize to a dict; got {1}".format(
                    geometry_payload_source,
                    _payload_summary(geometry_payload),
                )
            )
        if geometry_payload:
            print(
                "[GH CT Wrapper] geometry payload source={0}; {1}".format(
                    geometry_payload_source,
                    _payload_summary(geometry_payload),
                )
            )
        if not geometry_payload and not (isinstance(maybe_synced, dict) and maybe_synced):
            raise ValueError("geometry_payload input is required when calc_payload has no synced_geometry_payload")

        target_path = Path(str(out_json_path)) if "out_json_path" in globals() and out_json_path else (Path(ROOT) / "ct_anchor_milling_export.json")

        package = helper.export_ct_json(
            geometry_payload=geometry_payload,
            calc_payload=calc_payload,
            out_json_path=target_path,
            bottom_face_mode=str(bottom_face_mode) if "bottom_face_mode" in globals() and bottom_face_mode else None,
            process_only_passed=bool(process_only_passed) if "process_only_passed" in globals() else False,
        )
        records = package.get("records", [])
        timber_model_schema = package.get("timber_model_schema", {})
        inspection_breps = helper.build_inspection_breps(records)
        json_path = str(target_path)
        package_metadata = package.get("metadata", {}) if isinstance(package, dict) else {}
        if isinstance(package_metadata, dict) and package_metadata.get("json_write_succeeded") is False:
            out = "Built {0} CT records; JSON write skipped: {1}".format(
                len(records),
                package_metadata.get("json_write_error"),
            )
        else:
            out = "Exported {0} CT records".format(len(records))

except Exception:
    out = traceback.format_exc()
    print("[GH CT Wrapper] Exception caught:")
    print(out)
    package = {}
    records = []
    timber_model_schema = {}
    inspection_breps = []
    json_path = None
