"""
Grasshopper Py3 wrapper: ct_anchor_milling

GH Inputs expected:
- geometry_payload (optional when calc_payload.synced_geometry_payload is present; Item Access)
- calc_payload (Item Access)
- out_json_path
- bottom_face_mode
- process_only_passed
- enabled
- include_plate_fasteners (optional; default False)

GH Outputs:
- out
- package
- records
- timber_model_schema
- inspection_breps
- json_path
- status
- ct_fastener_interfaces
- ct_plate_fasteners (optional; empty by default)
- ct_plate_shapes (optional; empty by default)
"""

import importlib.util
import os
import sys
import traceback
from pathlib import Path

try:
    import scriptcontext as sc  # type: ignore
except Exception:
    sc = None

try:
    import Rhino  # type: ignore
except Exception:
    Rhino = None

try:
    from Grasshopper.Kernel.Types import GH_ObjectWrapper  # type: ignore
except Exception:
    GH_ObjectWrapper = None


ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
HELPER_PATH = os.path.join(ROOT, "py", "ct_anchor_milling.py")


def _load_helper():
    module_name = "ct_anchor_milling"
    sys.modules.pop(module_name, None)

    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))

    module = importlib.util.module_from_spec(spec)
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
        return {_to_python_data(key): _to_python_data(value[key]) for key in keys}

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
        return GH_ObjectWrapper(value)

    if sc is None:
        return value

    token = "BPG_PAYLOAD::{0}::{1}".format(kind, _component_key())
    sc.sticky[token] = value
    return token


def _coerce_geometry_list(items):
    """Return only Rhino geometry objects, converting to Brep when possible."""
    coerced = []

    for item in items or []:
        if item is None:
            continue

        geom = item

        if Rhino is not None:
            try:
                if isinstance(geom, Rhino.Geometry.GeometryBase):
                    coerced.append(geom)
                    continue
            except Exception:
                pass

        try:
            duplicate = getattr(item, "Duplicate", None)
            if callable(duplicate):
                geom = duplicate()
        except Exception:
            geom = item

        if Rhino is not None:
            try:
                if isinstance(geom, Rhino.Geometry.GeometryBase):
                    coerced.append(geom)
                    continue
            except Exception:
                pass

        try:
            to_brep = getattr(item, "ToBrep", None)
            if callable(to_brep):
                try:
                    brep = to_brep(False)
                except TypeError:
                    brep = to_brep()

                if brep is not None:
                    coerced.append(brep)
                    continue
        except Exception:
            pass

    return coerced


def _empty_outputs(message=""):
    return (
        message,
        {},
        [],
        {},
        [],
        None,
        {"ok": False, "message": message},
        [],
        [],
        [],
    )


# Initialize outputs so GH never receives nulls.
out, package, records, timber_model_schema, inspection_breps, json_path, status, ct_fastener_interfaces, ct_plate_fasteners, ct_plate_shapes = _empty_outputs("Initialized")


try:
    if "enabled" in globals() and enabled is False:
        out, package, records, timber_model_schema, inspection_breps, json_path, status, ct_fastener_interfaces, ct_plate_fasteners, ct_plate_shapes = _empty_outputs("CT export disabled.")

    else:
        helper = _load_helper()

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
                "calc payload from {0} did not normalize to a dict; got {1}. "
                "Keep calc_payload on Item Access; List/Tree Access can explode the payload.".format(
                    calc_payload_source,
                    _payload_summary(calc_payload),
                )
            )

        if "run_tag" in calc_payload and "error" in calc_payload and len(calc_payload.keys()) <= 3:
            raise ValueError(
                "calc_payload appears to be debug_status, not calculations payload. "
                "Wire the payload output of gh_wrapper_calculations to calc_payload input."
            )

        if not any(
            key in calc_payload
            for key in ("adjusted_base_plates", "member_results", "synced_geometry_payload")
        ):
            raise ValueError(
                "calc_payload missing required calculation keys. "
                "Expected one of: adjusted_base_plates, member_results, synced_geometry_payload."
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
                "geometry payload from {0} did not normalize to a dict; got {1}. "
                "Keep geometry_payload on Item Access; List/Tree Access can explode the payload.".format(
                    geometry_payload_source,
                    _payload_summary(geometry_payload),
                )
            )

        if not geometry_payload and not (isinstance(maybe_synced, dict) and maybe_synced):
            raise ValueError("geometry_payload input is required when calc_payload has no synced_geometry_payload")

        target_path = (
            Path(str(out_json_path))
            if "out_json_path" in globals() and out_json_path
            else Path(ROOT) / "ct_anchor_milling_export.json"
        )

        package_raw = helper.export_ct_json(
            geometry_payload=geometry_payload,
            calc_payload=calc_payload,
            out_json_path=target_path,
            bottom_face_mode=str(bottom_face_mode) if "bottom_face_mode" in globals() and bottom_face_mode else None,
            process_only_passed=bool(process_only_passed) if "process_only_passed" in globals() else False,
        )

        records = package_raw.get("records", [])
        timber_model_schema = package_raw.get("timber_model_schema", {})
        inspection_breps = _coerce_geometry_list(helper.build_inspection_breps(records))

        include_plate_fasteners_value = (
            bool(include_plate_fasteners)
            if "include_plate_fasteners" in globals()
            else False
        )

        ct_fastener_interfaces = []
        ct_plate_fasteners = []
        ct_plate_shapes = []

        if hasattr(helper, "build_ct_fastener_objects"):
            try:
                ct_fastener_package = helper.build_ct_fastener_objects(
                    records,
                    include_plate_fasteners=include_plate_fasteners_value,
                )
            except TypeError:
                ct_fastener_package = helper.build_ct_fastener_objects(records)

            if isinstance(ct_fastener_package, dict):
                ct_fastener_interfaces = ct_fastener_package.get("interfaces", []) or []

                if include_plate_fasteners_value:
                    ct_plate_fasteners = ct_fastener_package.get("plate_fasteners", []) or []
                    ct_plate_shapes = _coerce_geometry_list(ct_fastener_package.get("plate_shapes", []) or [])

        json_path = str(target_path)

        metadata = package_raw.get("metadata", {}) if isinstance(package_raw, dict) else {}
        json_write_succeeded = metadata.get("json_write_succeeded", True) if isinstance(metadata, dict) else True

        if not json_write_succeeded:
            out = "Built {0} CT records; JSON write skipped: {1}".format(
                len(records),
                metadata.get("json_write_error"),
            )
        else:
            out = "Exported {0} CT records".format(len(records))

        if len(records) == 0:
            out = (
                "Exported 0 CT records. "
                "Check calc_payload member_results/adjusted_base_plates and geometry_payload."
            )

        package = _store_payload_reference("ct_package", package_raw)

        status = {
            "ok": True,
            "record_count": len(records),
            "inspection_brep_count": len(inspection_breps),
            "ct_fastener_interface_count": len(ct_fastener_interfaces),
            "include_plate_fasteners": include_plate_fasteners_value,
            "ct_plate_fastener_count": len(ct_plate_fasteners),
            "ct_plate_shape_count": len(ct_plate_shapes),
            "json_path": json_path,
        }

except Exception:
    out = traceback.format_exc()
    package = {"wrapper_error": out}
    records = [{"wrapper_error": out}]
    timber_model_schema = {"wrapper_error": True}
    inspection_breps = []
    json_path = None
    status = {"ok": False, "error": out}
    ct_fastener_interfaces = []
    ct_plate_fasteners = []
    ct_plate_shapes = []


# GH outputs
a = out
b = package
c = records
d = timber_model_schema
e = inspection_breps
f = json_path
g = status
h = ct_fastener_interfaces
i = ct_plate_fasteners
j = ct_plate_shapes
