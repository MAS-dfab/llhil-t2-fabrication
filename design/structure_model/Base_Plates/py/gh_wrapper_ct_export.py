"""
Grasshopper Py3 wrapper: ct_anchor_milling

GH Inputs expected:
- geometry_payload (optional when calc_payload.synced_geometry_payload is present; Item Access)
- calc_payload (Item Access)
- out_json_path
- bottom_face_mode
- process_only_passed
- enabled

GH Outputs:
- out
- package
- records
- timber_model_schema
- inspection_breps (final timber column breps with milling cuts)
- json_path
- debug_status
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
RUN_TAG = "CT_WRAPPER_SYNC_2026_05_18_COUNTERBORE_DIAG"

print("[GH CT Wrapper] RUN_TAG:", RUN_TAG)
print("[GH CT Wrapper] HELPER_PATH:", HELPER_PATH)
print("[GH CT Wrapper] HELPER_EXISTS:", os.path.exists(HELPER_PATH))

# Early fallback assignments: if a GH component is running a truncated copy of
# this script, outputs still show a diagnostic instead of staying <null>.
out = "CT wrapper startup reached"
package = {"phase": "startup"}
records = []
timber_model_schema = {}
inspection_breps = []
json_path = None
debug_status = {
    "run_tag": RUN_TAG,
    "phase": "startup_prints_completed",
    "helper_path": HELPER_PATH,
    "helper_exists": bool(os.path.exists(HELPER_PATH)),
}
a = out
b = package
c = records
d = timber_model_schema
e = inspection_breps
f = json_path
g = debug_status


def _load_helper():
    module_name = "ct_anchor_milling"
    print("[GH CT Wrapper] HELPER_LOAD_START")
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
    # Dataclasses/type annotations can resolve through sys.modules during exec.
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    print("[GH CT Wrapper] HELPER_LOADED: True")
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
        print("[GH CT Wrapper] payload output mode=gh_object_wrapper kind={0}".format(kind))
        return GH_ObjectWrapper(value)
    if sc is None:
        return value
    token = "BPG_PAYLOAD::{0}::{1}".format(kind, _component_key())
    sc.sticky[token] = value
    print("[GH CT Wrapper] payload output mode=reference_token kind={0}".format(kind))
    return token


def _coerce_geometry_list(items):
    """Return only Rhino geometry objects, converting to Brep when possible."""
    coerced = []
    for item in (items or []):
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


print("[GH CT Wrapper] BODY_READY")


# Initialize outputs so GH never receives nulls from unassigned variables.
out = "CT wrapper initialized"
package = {}
records = []
timber_model_schema = {}
inspection_breps = []
json_path = None
debug_status = {
    "run_tag": RUN_TAG,
    "phase": "initialized",
}
a = out
b = package
c = records
d = timber_model_schema
e = inspection_breps
f = json_path
g = debug_status
print("[GH CT Wrapper] OUTPUTS_INITIALIZED")


try:
    helper = _load_helper()
    print("[GH CT Wrapper] HELPER_LOADED:", helper is not None)
    print(
        "[GH CT Wrapper] ENABLED_INPUT:",
        enabled if "enabled" in globals() else "<missing>",
    )

    if "enabled" in globals() and enabled is False:
        out = "CT export disabled."
        package = {}
        records = []
        timber_model_schema = {}
        inspection_breps = []
        json_path = None
        debug_status = {
            "run_tag": RUN_TAG,
            "phase": "disabled",
            "enabled_input": enabled if "enabled" in globals() else None,
        }
        a = out
        b = package
        c = records
        d = timber_model_schema
        e = inspection_breps
        f = json_path
        g = debug_status
    else:
        print("[GH CT Wrapper] ENTER_ACTIVE_BRANCH")
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
        print("[GH CT Wrapper DEBUG] calc_payload_source=", calc_payload_source)
        print("[GH CT Wrapper DEBUG] calc_payload_type=", type(calc_payload))
        if isinstance(calc_payload, dict):
            print("[GH CT Wrapper DEBUG] calc_payload_keys=", list(calc_payload.keys()))
        else:
            print("[GH CT Wrapper DEBUG] calc_payload_value=", calc_payload)
        if not isinstance(calc_payload, dict):
            raise TypeError(
                "calc payload from {0} did not normalize to a dict; got {1}. "
                "Keep calc_payload on Item Access; List/Tree Access can explode the payload.".format(
                    calc_payload_source,
                    _payload_summary(calc_payload),
                )
            )
        # Common GH wiring mistake: feeding debug_status instead of payload.
        if "run_tag" in calc_payload and "error" in calc_payload and len(calc_payload.keys()) <= 3:
            raise ValueError(
                "calc_payload appears to be debug_status, not calculations payload. "
                "Wire the 'payload' output of gh_wrapper_calculations to calc_payload input."
            )
        if not any(
            key in calc_payload
            for key in ("adjusted_base_plates", "member_results", "synced_geometry_payload")
        ):
            raise ValueError(
                "calc_payload missing required calculation keys. "
                "Expected one of: adjusted_base_plates, member_results, synced_geometry_payload. "
                "This usually means the wrong upstream output is connected."
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
                "geometry payload from {0} did not normalize to a dict; got {1}. "
                "Keep geometry_payload on Item Access; List/Tree Access can explode the payload.".format(
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
        inspection_breps_raw = helper.build_inspection_breps(records)
        inspection_breps = _coerce_geometry_list(inspection_breps_raw)
        print("[GH CT Wrapper] HELPER_HAS_RHINO_GEOMETRY:", bool(getattr(helper, "rg", None) is not None))
        print("[GH CT Wrapper] RECORD_COUNT:", len(records))
        print("[GH CT Wrapper] INSPECTION_BREP_COUNT:", len(inspection_breps))
        print("[GH CT Wrapper] INSPECTION_BREP_NULL_COUNT:", sum(1 for x in inspection_breps if x is None))
        if len(inspection_breps) != len(inspection_breps_raw):
            print(
                "[GH CT Wrapper] INSPECTION_BREP_COERCION: kept={0} dropped={1}".format(
                    len(inspection_breps),
                    len(inspection_breps_raw) - len(inspection_breps),
                )
            )
        inspection_cut_diagnostics = []
        try:
            helper.build_inspection_breps(
                records,
                diagnostics=inspection_cut_diagnostics,
            )
        except Exception as diagnostics_exc:
            inspection_cut_diagnostics = [
                {
                    "diagnostics_error": str(diagnostics_exc),
                }
            ]
        cut_diagnostic_lines = []
        for cut_diag in inspection_cut_diagnostics:
            if not isinstance(cut_diag, dict):
                continue
            cut_line = (
                "[GH CT Wrapper] CUT_DIAG member={0} attempts={1} successes={2} failures={3} "
                "solid_before_trim={4} plane_trimmed={5} capped={6} solid_after_trim={7} final_parts={8}".format(
                    cut_diag.get("member_id"),
                    cut_diag.get("cut_attempts"),
                    cut_diag.get("cut_successes"),
                    cut_diag.get("cut_failures"),
                    cut_diag.get("timber_solid_before_trim"),
                    cut_diag.get("plane_trimmed"),
                    cut_diag.get("plane_trim_cap_succeeded"),
                    cut_diag.get("timber_solid_after_trim"),
                    cut_diag.get("final_part_count"),
                )
            )
            cut_diagnostic_lines.append(cut_line)
            print(cut_line)
            slot_diag = cut_diag.get("slot_target_diagnostics")
            if isinstance(slot_diag, dict) and slot_diag:
                slot_line = (
                    "[GH CT Wrapper] SLOT_DIAG member={0} bbox_overlap={1} "
                    "bool_intersections={2} brep_brep_success={3} curves={4} "
                    "target_valid={5} target_solid={6} cutter_valid={7} cutter_solid={8} methods={9}"
                ).format(
                    cut_diag.get("member_id"),
                    slot_diag.get("bbox_overlap"),
                    slot_diag.get("boolean_intersection_count"),
                    slot_diag.get("brep_brep_success"),
                    slot_diag.get("brep_brep_curve_count"),
                    slot_diag.get("target_valid"),
                    slot_diag.get("target_solid"),
                    slot_diag.get("cutter_valid"),
                    slot_diag.get("cutter_solid"),
                    cut_diag.get("cut_methods"),
                )
                cut_diagnostic_lines.append(slot_line)
                print(slot_line)
        json_path = str(target_path)
        package_metadata = package.get("metadata", {}) if isinstance(package, dict) else {}
        reference_usage = package_metadata.get("reference_usage", {}) if isinstance(package_metadata, dict) else {}
        milling_feature_summary = package_metadata.get("milling_feature_summary", {}) if isinstance(package_metadata, dict) else {}
        if isinstance(milling_feature_summary, dict):
            print(
                "[GH CT Wrapper] COUNTERBORE_DIAG diameters={0} auto={1} factors={2} sources={3} matches_auto={4}".format(
                    milling_feature_summary.get("counterbore_diameters"),
                    milling_feature_summary.get("counterbore_auto_diameters"),
                    milling_feature_summary.get("counterbore_dia_factors"),
                    milling_feature_summary.get("counterbore_dia_sources"),
                    milling_feature_summary.get("counterbores_match_auto_proportion"),
                )
            )
        if isinstance(package_metadata, dict) and package_metadata.get("json_write_succeeded") is False:
            out = "Built {0} CT records; JSON write skipped: {1}".format(
                len(records),
                package_metadata.get("json_write_error"),
            )
        else:
            out = "Exported {0} CT records".format(len(records))
        if cut_diagnostic_lines:
            out = "{0}\n{1}".format(out, "\n".join(cut_diagnostic_lines))
        if len(records) == 0:
            out = (
                "Exported 0 CT records; "
                "calc_member_result_count={0}, calc_adjusted_plate_count={1}, "
                "has_synced_geometry_payload={2}, geometry_payload_provided={3}"
            ).format(
                len(calc_payload.get("member_results", [])) if isinstance(calc_payload.get("member_results"), list) else 0,
                len(calc_payload.get("adjusted_base_plates", [])) if isinstance(calc_payload.get("adjusted_base_plates"), list) else 0,
                bool(isinstance(calc_payload.get("synced_geometry_payload"), dict) and calc_payload.get("synced_geometry_payload")),
                bool(geometry_payload),
            )
        package = _store_payload_reference("ct_package", package)
        debug_status = {
            "run_tag": RUN_TAG,
            "record_count": len(records),
            "calc_member_result_count": len(calc_payload.get("member_results", [])) if isinstance(calc_payload.get("member_results"), list) else 0,
            "calc_adjusted_plate_count": len(calc_payload.get("adjusted_base_plates", [])) if isinstance(calc_payload.get("adjusted_base_plates"), list) else 0,
            "calc_has_synced_geometry_payload": bool(isinstance(calc_payload.get("synced_geometry_payload"), dict) and calc_payload.get("synced_geometry_payload")),
            "has_timber_model_schema": bool(timber_model_schema),
            "helper_has_rhino_geometry": bool(getattr(helper, "rg", None) is not None),
            "inspection_breps_python_type": type(inspection_breps).__name__,
            "inspection_brep_count": len(inspection_breps),
            "inspection_brep_null_count": sum(1 for x in inspection_breps if x is None),
            "inspection_cut_diagnostics": inspection_cut_diagnostics,
            "inspection_cut_diagnostic_lines": cut_diagnostic_lines,
            "reference_usage": reference_usage,
            "milling_feature_summary": milling_feature_summary,
            "cluster_expansion": {
                "expected_total": 16,
                "actual_total": len(records),
                "records_per_cluster": len(records) // max(len(calc_payload.get("adjusted_base_plates") or []), 1),
            },
            "payload_transport": "gh_object_wrapper" if GH_ObjectWrapper is not None else ("sticky_token" if sc is not None else "direct_value"),
            "package_output": package,
            "json_path": json_path,
        }
        # Legacy / generic-output aliases if this GH component has not yet had
        # its output names refreshed after wrapper updates.
        a = out
        b = package
        c = records
        d = timber_model_schema
        e = inspection_breps
        f = json_path
        g = debug_status

except Exception:
    try:
        out = traceback.format_exc()
    except NameError:
        import traceback as _traceback
        out = _traceback.format_exc()
    print("[GH CT Wrapper] Exception caught:")
    print(out)
    package = {"wrapper_error": out}
    records = [{"wrapper_error": out}]
    timber_model_schema = {"wrapper_error": True}
    inspection_breps = []
    json_path = None
    debug_status = {
        "run_tag": RUN_TAG,
        "error": out,
    }
    a = out
    b = package
    c = records
    d = timber_model_schema
    e = inspection_breps
    f = json_path
    g = debug_status