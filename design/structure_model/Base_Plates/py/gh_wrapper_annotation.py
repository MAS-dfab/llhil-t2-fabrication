"""
Grasshopper Py3 wrapper: base_footing_annotation_component

GH Inputs expected (all optional â€” component still runs with no connections):
- package          (Item Access; opaque payload from gh_wrapper_calculations or gh_wrapper_ct_export)
- payload          (Item Access; alias for package)
- timber_brep      (manual Brep list; used when package is absent or timber_source="manual")
- plate_brep       (manual Brep list)
- bolt_points      (manual Point3d list)
- washer_recess_breps (manual Brep list)
- slot_cut_brep    (manual Brep list)
- timber_source    (str: "auto" | "package" | "manual"; default "auto")
- plate_source     (str: "auto" | "package" | "manual"; default "auto")
- bolt_source      (str: "auto" | "package" | "manual"; default "auto")
- washer_source    (str: "auto" | "package" | "manual"; default "auto")
- slot_source      (str: "auto" | "package" | "manual"; default "auto")
- metadata         (dict override; leave unconnected to read from package)
- show_top_dimensions    (bool; default True)
- show_front_dimensions  (bool; default True)
- show_section_dimensions (bool; default True)
- show_code_labels       (bool; default True)
- dim_offset       (float mm; default 25)
- text_height      (float mm; default 8)
- arrow_size       (float mm; default 3)
- view_scale       (float; default 1.0)
- annotation_scale (float; default 1.0; larger value = larger annotations)
- enabled          (bool; default True)

GH Outputs:
- out              (status string)
- rh_geo_preview   (all annotation geometry for Rhino preview)
- dim_lines
- extension_lines
- arrowheads
- text_labels
- leader_lines
- code_check_labels
- reports          (dict with full annotation report)
- layout_package   (single packaged payload for layout wrapper)
- debug_status     (diagnostic dict)
"""

import importlib.util
import os
import sys
import traceback

try:
    import scriptcontext as sc  # type: ignore
except Exception:
    sc = None

try:
    from Grasshopper.Kernel.Types import GH_ObjectWrapper  # type: ignore
except Exception:
    GH_ObjectWrapper = None

try:
    import Rhino  # type: ignore
except Exception:
    Rhino = None

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
HELPER_PATH = os.path.join(ROOT, "py", "base_footing_annotation_component.py")
RUN_TAG = "ANNOTATION_WRAPPER_SYNC_2026_05_18"
_DEBUG_LOG = False

def _debug_print(*args):
    if _DEBUG_LOG:
        __import__("builtins").print(*args)


_debug_print("[GH Annotation Wrapper] RUN_TAG:", RUN_TAG)
_debug_print("[GH Annotation Wrapper] HELPER_PATH:", HELPER_PATH)
_debug_print("[GH Annotation Wrapper] HELPER_EXISTS:", os.path.exists(HELPER_PATH))


# ---------------------------------------------------------------------------
# Fallback output assignments â€” survive any early-exit or truncated script run
# ---------------------------------------------------------------------------
out = "Annotation wrapper startup reached"
rh_geo_preview = []
dim_lines = []
extension_lines = []
arrowheads = []
text_labels = []
leader_lines = []
code_check_labels = []
reports = {}
layout_package = {}
debug_status = {
    "run_tag": RUN_TAG,
    "phase": "startup",
    "helper_path": HELPER_PATH,
    "helper_exists": bool(os.path.exists(HELPER_PATH)),
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_helper():
    module_name = "base_footing_annotation_component"
    _debug_print("[GH Annotation Wrapper] HELPER_LOAD_START")
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    _debug_print("[GH Annotation Wrapper] HELPER_LOADED: True")
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
    return value


def _unwrap_payload(value):
    """Normalise the common GH forms of a single dictionary payload."""
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
        if isinstance(normalized, (list, tuple)) and len(normalized) == 1:
            normalized = normalized[0]
            continue
        return normalized


def _gh_input(name, default=None):
    """Read a GH input by name, returning default when absent or None."""
    value = globals().get(name, default)
    return default if value is None else value


def _coerce_float(value, default):
    try:
        return float(value)
    except Exception:
        return float(default)


def _mm_to_doc_scale():
    if Rhino is None:
        return 1.0
    try:
        doc = Rhino.RhinoDoc.ActiveDoc
        if doc is None:
            return 1.0
        return float(Rhino.RhinoMath.UnitScale(Rhino.UnitSystem.Millimeters, doc.ModelUnitSystem))
    except Exception:
        return 1.0


def _flatten_geo_list(value):
    """Accept a GH data-tree / list / single geometry item; return a flat list."""
    if value is None:
        return []
    branch_count = getattr(value, "BranchCount", None)
    branch_getter = getattr(value, "Branch", None)
    if isinstance(branch_count, int) and callable(branch_getter):
        items = []
        for i in range(branch_count):
            try:
                items.extend(list(branch_getter(i)))
            except Exception:
                pass
        return items
    try:
        return list(value)
    except TypeError:
        return [value]


def _is_geometry_like(value):
    if value is None:
        return False
    if isinstance(value, dict):
        return False
    return callable(getattr(value, "GetBoundingBox", None)) or callable(getattr(value, "ToBrep", None))


def _is_point_like(value):
    if value is None:
        return False
    if isinstance(value, dict):
        has_xyz = all(k in value for k in ("x", "y", "z")) or all(k in value for k in ("X", "Y", "Z"))
        return has_xyz
    if all(hasattr(value, key) for key in ("X", "Y", "Z")):
        return True
    try:
        items = list(value)
        return len(items) >= 3
    except Exception:
        return False


def _filter_values(values, validator):
    out = []
    for item in values or []:
        if validator(item):
            out.append(item)
    return out


def _first_connected_input(names):
    for name in names:
        if name in globals() and globals()[name] is not None:
            return name, globals()[name]
    return None, None


def _has_geometry_keys(data):
    if not isinstance(data, dict):
        return False
    geometry_keys = (
        "timber_brep",
        "timber_breps",
        "inspection_breps",
        "plate_brep",
        "plate_breps",
        "footing_breps",
        "base_plates",
        "preview_breps",
        "preview",
        "bolt_points",
        "hole_centers",
        "washer_recess_breps",
        "slot_cut_brep",
    )
    return any(key in data for key in geometry_keys)


def _iter_dicts_deep(value, _seen=None):
    if _seen is None:
        _seen = set()
    marker = id(value)
    if marker in _seen:
        return
    _seen.add(marker)
    if isinstance(value, dict):
        yield value
        for child in value.values():
            for nested in _iter_dicts_deep(child, _seen):
                yield nested
        return
    if isinstance(value, (list, tuple)):
        for item in value:
            for nested in _iter_dicts_deep(item, _seen):
                yield nested


def _pick_nested_list(payload_dict, candidate_keys, validator=None):
    if not isinstance(payload_dict, dict):
        return []
    for d in _iter_dicts_deep(payload_dict):
        for key in candidate_keys:
            raw = d.get(key)
            values = _flatten_geo_list(raw)
            values = [v for v in values if v is not None]
            if validator is not None:
                values = _filter_values(values, validator)
            if values:
                return values
    return []


def _metadata_from_payload(payload_dict):
    if not isinstance(payload_dict, dict):
        return None
    for d in _iter_dicts_deep(payload_dict):
        for key in ("annotation_metadata", "handoff", "metadata"):
            candidate = d.get(key)
            if isinstance(candidate, dict) and candidate:
                return candidate
    return None


def _payload_geometry_score(payload_dict):
    if not isinstance(payload_dict, dict):
        return 0
    score = 0
    key_sets = (
        (("timber_brep", "timber_breps", "inspection_breps", "timber"), _is_geometry_like),
        (("plate_brep", "plate_breps", "preview_breps", "breps_doc_units", "footing_breps", "base_plates"), _is_geometry_like),
        (("bolt_points", "hole_centers", "anchor_points", "bolt_hole_centers"), _is_point_like),
        (("washer_recess_brep", "washer_recess_breps", "washer_recesses"), _is_geometry_like),
        (("slot_cut_brep", "slot_cut_breps", "slot_brep", "slot_breps", "slot_cuts"), _is_geometry_like),
    )
    for keys, validator in key_sets:
        if _pick_nested_list(payload_dict, keys, validator=validator):
            score += 1
    if _metadata_from_payload(payload_dict):
        score += 1
    return score


def _resolve_best_package(candidate_names):
    best_name = None
    best_payload = None
    best_score = -1
    diagnostics = []
    for name in candidate_names:
        if name not in globals() or globals()[name] is None:
            diagnostics.append({"name": name, "state": "missing"})
            continue
        raw = globals()[name]
        unwrapped = _unwrap_payload(raw)
        payload = unwrapped
        used_nested = False
        if isinstance(payload, dict) and not _has_geometry_keys(payload):
            nested = payload.get("synced_geometry_payload")
            if isinstance(nested, dict):
                payload = nested
                used_nested = True
        score = _payload_geometry_score(payload)
        diagnostics.append(
            {
                "name": name,
                "state": "ok",
                "used_nested_synced": used_nested,
                "is_dict": isinstance(payload, dict),
                "score": score,
            }
        )
        if score > best_score:
            best_score = score
            best_name = name
            best_payload = payload
    return best_name, best_payload, diagnostics


# ---------------------------------------------------------------------------
# Initialized outputs (safe defaults shown in GH before the component runs)
# ---------------------------------------------------------------------------
out = "Annotation wrapper initialized"
rh_geo_preview = []
dim_lines = []
extension_lines = []
arrowheads = []
text_labels = []
leader_lines = []
code_check_labels = []
reports = {}
debug_status = {
    "run_tag": RUN_TAG,
    "phase": "initialized",
}

_debug_print("[GH Annotation Wrapper] OUTPUTS_INITIALIZED")


# ---------------------------------------------------------------------------
# Main execution
# ---------------------------------------------------------------------------
try:
    helper = _load_helper()

    if _gh_input("enabled", True) is False:
        out = "Annotation wrapper disabled."
        debug_status = {"run_tag": RUN_TAG, "phase": "disabled"}

    else:
        # --- resolve package input (accept common upstream names) ----------
        _package_source, _package, _package_diagnostics = _resolve_best_package(
            (
                "package",
                "payload",
                "synced_geometry_payload",
                "geometry_payload",
                "calc_payload",
                "calculation_payload",
                "calculations_payload",
            )
        )

        _timber_source_name, _timber_raw = _first_connected_input((
            "timber_brep",
            "inspection_breps",
            "timber_breps",
            "timber",
        ))
        _plate_source_name, _plate_raw = _first_connected_input((
            "plate_brep",
            "synced_preview_breps",
            "preview_breps",
            "footing_breps",
            "base_plates",
        ))
        _bolt_source_name, _bolt_raw = _first_connected_input(("bolt_points", "hole_centers"))
        _washer_source_name, _washer_raw = _first_connected_input(("washer_recess_breps", "washer_recess_brep"))
        _slot_source_name, _slot_raw = _first_connected_input(("slot_cut_brep", "slot_cut_breps", "slot_brep"))

        # If explicit GH inputs are not connected, recover geometry from nested
        # calculation/geometry payload dictionaries.
        if isinstance(_package, dict):
            if _timber_raw is None:
                _timber_raw = _pick_nested_list(
                    _package,
                    ("timber_brep", "timber_breps", "inspection_breps", "timber", "members"),
                    validator=_is_geometry_like,
                )
                if _timber_raw:
                    _timber_source_name = (_timber_source_name or "package") + ":nested"

            if _plate_raw is None:
                _plate_raw = _pick_nested_list(
                    _package,
                    (
                        "plate_brep",
                        "plate_breps",
                        "breps_doc_units",
                        "preview_breps",
                        "footing_breps",
                        "preview_breps",
                        "base_plates",
                    ),
                    validator=_is_geometry_like,
                )
                if _plate_raw:
                    _plate_source_name = (_plate_source_name or "package") + ":nested"

            if _bolt_raw is None:
                _bolt_raw = _pick_nested_list(
                    _package,
                    ("bolt_points", "hole_centers", "anchor_points", "bolt_hole_centers"),
                    validator=_is_point_like,
                )
                if _bolt_raw:
                    _bolt_source_name = (_bolt_source_name or "package") + ":nested"

            if _washer_raw is None:
                _washer_raw = _pick_nested_list(
                    _package,
                    ("washer_recess_brep", "washer_recess_breps", "washer_recesses"),
                    validator=_is_geometry_like,
                )
                if _washer_raw:
                    _washer_source_name = (_washer_source_name or "package") + ":nested"

            if _slot_raw is None:
                _slot_raw = _pick_nested_list(
                    _package,
                    ("slot_cut_brep", "slot_cut_breps", "slot_brep", "slot_breps", "slot_cuts"),
                    validator=_is_geometry_like,
                )
                if _slot_raw:
                    _slot_source_name = (_slot_source_name or "package") + ":nested"

        _timber_raw = _filter_values(_flatten_geo_list(_timber_raw), _is_geometry_like)
        _plate_raw = _filter_values(_flatten_geo_list(_plate_raw), _is_geometry_like)
        _bolt_raw = _filter_values(_flatten_geo_list(_bolt_raw), _is_point_like)
        _washer_raw = _filter_values(_flatten_geo_list(_washer_raw), _is_geometry_like)
        _slot_raw = _filter_values(_flatten_geo_list(_slot_raw), _is_geometry_like)

        _metadata_override = _gh_input("metadata")
        if _metadata_override is None and isinstance(_package, dict):
            _metadata_override = (
                _package.get("annotation_metadata")
                or _package.get("handoff")
                or _package.get("metadata")
            )
        if _metadata_override is None:
            _metadata_override = _metadata_from_payload(_package)

        _base_view_scale = max(_coerce_float(_gh_input("view_scale", 1.0), 1.0), 1e-9)
        _annotation_scale = max(_coerce_float(_gh_input("annotation_scale", 1.0), 1.0), 1e-9)
        # Helper interprets larger view_scale as smaller annotation graphics.
        # Keep slider semantics intuitive: larger annotation_scale => larger graphics.
        _effective_view_scale = _base_view_scale / _annotation_scale
        _mm_scale = _mm_to_doc_scale()
        _dim_offset = _coerce_float(_gh_input("dim_offset", 25.0), 25.0) * _mm_scale
        _text_height = _coerce_float(_gh_input("text_height", 8.0), 8.0) * _mm_scale
        _arrow_size = _coerce_float(_gh_input("arrow_size", 3.0), 3.0) * _mm_scale

        # --- call helper.run() with every optional input ------------------
        result = helper.run(
            package=_package,
            timber_brep=_flatten_geo_list(_timber_raw),
            plate_brep=_flatten_geo_list(_plate_raw),
            bolt_points=_flatten_geo_list(_bolt_raw),
            washer_recess_breps=_flatten_geo_list(_washer_raw),
            slot_cut_brep=_flatten_geo_list(_slot_raw),
            timber_source=_gh_input("timber_source", "auto"),
            plate_source=_gh_input("plate_source", "auto"),
            bolt_source=_gh_input("bolt_source", "auto"),
            washer_source=_gh_input("washer_source", "auto"),
            slot_source=_gh_input("slot_source", "auto"),
            metadata=_metadata_override,
            show_top_dimensions=_gh_input("show_top_dimensions", True),
            show_front_dimensions=_gh_input("show_front_dimensions", True),
            show_section_dimensions=_gh_input("show_section_dimensions", True),
            show_code_labels=_gh_input("show_code_labels", True),
            dim_offset=_dim_offset,
            text_height=_text_height,
            arrow_size=_arrow_size,
            view_scale=_effective_view_scale,
        )

        # Retry in explicit manual mode if the first pass yielded no geometry.
        if not (result.get("rh_geo_preview") or []):
            result = helper.run(
                package=None,
                timber_brep=_flatten_geo_list(_timber_raw),
                plate_brep=_flatten_geo_list(_plate_raw),
                bolt_points=_flatten_geo_list(_bolt_raw),
                washer_recess_breps=_flatten_geo_list(_washer_raw),
                slot_cut_brep=_flatten_geo_list(_slot_raw),
                timber_source="manual",
                plate_source="manual",
                bolt_source="manual",
                washer_source="manual",
                slot_source="manual",
                metadata=_metadata_override,
                show_top_dimensions=_gh_input("show_top_dimensions", True),
                show_front_dimensions=_gh_input("show_front_dimensions", True),
                show_section_dimensions=_gh_input("show_section_dimensions", True),
                show_code_labels=_gh_input("show_code_labels", True),
                dim_offset=_dim_offset,
                text_height=_text_height,
                arrow_size=_arrow_size,
                view_scale=_effective_view_scale,
            )

        # --- unpack outputs -----------------------------------------------
        rh_geo_preview = result.get("rh_geo_preview", [])
        dim_lines = result.get("dim_lines", [])
        extension_lines = result.get("extension_lines", [])
        arrowheads = result.get("arrowheads", [])
        text_labels = result.get("text_labels", [])
        leader_lines = result.get("leader_lines", [])
        code_check_labels = result.get("code_check_labels", [])
        reports = result.get("reports", {})

        _layout_payload = None
        if isinstance(_package, dict):
            _layout_payload = _package.get("layout_payload")
            if _layout_payload is None and isinstance(_package.get("reports"), dict):
                _layout_payload = _package["reports"].get("layout_payload")

        _combined_report = None
        if isinstance(_package, dict):
            _combined_report = _package.get("combined_report") or _package.get("report_text")
        if _combined_report is None and isinstance(reports, dict):
            _combined_report = reports.get("combined_report")

        layout_package = {
            "geometry_objects": list(rh_geo_preview or []),
            "annotation_objects": (
                list(dim_lines or [])
                + list(extension_lines or [])
                + list(arrowheads or [])
                + list(text_labels or [])
                + list(leader_lines or [])
                + list(code_check_labels or [])
            ),
            "metadata": _metadata_override if isinstance(_metadata_override, dict) else {},
            "combined_report": _combined_report,
            "layout_payload": _layout_payload if isinstance(_layout_payload, dict) else {},
            "reports": reports if isinstance(reports, dict) else {},
        }

        if isinstance(reports, dict):
            reports["layout_package"] = layout_package

        # Some component versions surface populated data under reports only.
        if isinstance(reports, dict):
            if not dim_lines:
                dim_lines = list(reports.get("dim_lines") or [])
            if not extension_lines:
                extension_lines = list(reports.get("extension_lines") or [])
            if not arrowheads:
                arrowheads = list(reports.get("arrowheads") or [])
            if not text_labels:
                text_labels = list(reports.get("text_labels") or [])
            if not leader_lines:
                leader_lines = list(reports.get("leader_lines") or [])

        _messages = result.get("messages", [])
        _source = result.get("source_report", {})
        out = "Annotation OK â€” geo: {0} items | {1}".format(
            len(rh_geo_preview),
            ", ".join("{0}={1}".format(k, v) for k, v in _source.items()
                      if k.endswith("_count") or k == "has_package"),
        )
        if _messages:
            out = "{0}\n{1}".format(out, "\n".join(str(m) for m in _messages))

        debug_status = {
            "run_tag": RUN_TAG,
            "phase": "completed",
            "package_source": _package_source,
            "package_candidates": _package_diagnostics,
            "timber_input": _timber_source_name,
            "plate_input": _plate_source_name,
            "bolt_input": _bolt_source_name,
            "washer_input": _washer_source_name,
            "slot_input": _slot_source_name,
            "timber_input_count": len(_flatten_geo_list(_timber_raw)),
            "plate_input_count": len(_flatten_geo_list(_plate_raw)),
            "bolt_input_count": len(_flatten_geo_list(_bolt_raw)),
            "washer_input_count": len(_flatten_geo_list(_washer_raw)),
            "slot_input_count": len(_flatten_geo_list(_slot_raw)),
            "mm_to_doc_scale": _mm_scale,
            "dim_offset_doc_units": _dim_offset,
            "text_height_doc_units": _text_height,
            "arrow_size_doc_units": _arrow_size,
            "base_view_scale": _base_view_scale,
            "annotation_scale": _annotation_scale,
            "effective_view_scale": _effective_view_scale,
            "geo_count": len(rh_geo_preview),
            "dim_line_count": len(dim_lines),
            "text_label_count": len(text_labels),
            "messages": _messages,
            "source_report": _source,
        }

        _debug_print("[GH Annotation Wrapper] DONE geo_count={0}".format(len(rh_geo_preview)))

except Exception:
    try:
        out = traceback.format_exc()
    except NameError:
        import traceback as _traceback
        out = _traceback.format_exc()
    _debug_print("[GH Annotation Wrapper] Exception caught:")
    _debug_print(out)
    rh_geo_preview = []
    dim_lines = []
    extension_lines = []
    arrowheads = []
    text_labels = []
    leader_lines = []
    code_check_labels = []
    reports = {"wrapper_error": out}
    layout_package = {}
    debug_status = {
        "run_tag": RUN_TAG,
        "phase": "error",
        "error": out,
    }


