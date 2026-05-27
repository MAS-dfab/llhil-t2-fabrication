"""
Grasshopper Python wrapper for the refactored base_footing_layout_component.

Single-source input contract:
- Connect ONE upstream payload to this wrapper via `package` or `payload`.
- Wrapper parses all fields internally (geometry, annotations, metadata, reports,
  and layout settings).

Recommended upstream source:
- `gh_wrapper_annotation` output `layout_package` (or `reports.layout_package`).
"""

import importlib.util
import logging
import os
import sys
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple, cast

try:
    import scriptcontext as sc  # type: ignore
except Exception:
    sc = None

if "package" not in globals():
    package = None
if "payload" not in globals():
    payload = None
if "geometry_objects" not in globals():
    geometry_objects = None
if "annotation_objects" not in globals():
    annotation_objects = None
if "metadata" not in globals():
    metadata = None
if "combined_report" not in globals():
    combined_report = None
if "layout_payload" not in globals():
    layout_payload = None
if "reports" not in globals():
    reports = None

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

ROOT = r"C:\Users\Juste\Documents\_GitHub\MAS-2526\10_llhil-t2-fabrication\design\structure_model\Base_Plates"
HELPER_PATH = os.path.join(ROOT, "py", "base_footing_layout_component.py")


def _load_helper() -> Any:
    module_name = "base_footing_layout_component"
    sys.modules.pop(module_name, None)
    spec = importlib.util.spec_from_file_location(module_name, HELPER_PATH)
    if spec is None or spec.loader is None:
        raise ImportError("Could not load {0}".format(HELPER_PATH))
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _pick_first(package: Dict[str, Any], keys: List[str], default: Any = None) -> Any:
    for key in keys:
        if key in package and package[key] is not None:
            return package[key]
    return default


def _to_python_data(value: Any) -> Any:
    if isinstance(value, dict):
        mapped = cast(Mapping[Any, Any], value)
        return {key: _to_python_data(mapped[key]) for key in mapped}

    keys = getattr(value, "Keys", None)
    if keys is not None:
        try:
            return {
                _to_python_data(key): _to_python_data(value[key])
                for key in list(keys)
            }
        except Exception:
            pass

    if isinstance(value, (list, tuple)):
        seq = cast(Iterable[Any], value)
        return [_to_python_data(item) for item in seq]

    if not isinstance(value, (str, bytes)):
        try:
            value.Count
            return [_to_python_data(item) for item in value]
        except Exception:
            pass

    return value


def _unwrap_payload(value: Any) -> Any:
    normalized: Any = value
    while True:
        obj = cast(object, normalized)
        wrapped_value = getattr(obj, "Value", None)
        if wrapped_value is not None and wrapped_value is not normalized:
            normalized = wrapped_value
            continue

        branches = getattr(obj, "Branches", None)
        if branches is not None:
            normalized = _to_python_data(branches)
            continue

        normalized = _to_python_data(normalized)

        sticky = getattr(sc, "sticky", None)
        if isinstance(normalized, str) and isinstance(sticky, dict) and normalized in sticky:
            sticky_map = cast(Mapping[str, Any], sticky)
            normalized = sticky_map[normalized]
            continue

        if isinstance(normalized, (list, tuple)):
            seq = cast(Sequence[Any], normalized)
            if len(seq) == 1:
                normalized = seq[0]
                continue

        return cast(Any, normalized)


def _resolve_source_payload(package_value: Any, payload_value: Any) -> Dict[str, Any]:
    candidates: List[Tuple[str, Any]] = [
        ("arg.package", package_value),
        ("arg.payload", payload_value),
        ("global.package", globals().get("package")),
        ("global.payload", globals().get("payload")),
    ]
    for name, value in candidates:
        if value is not None:
            return {
                "selected_source": name,
                "selected_value": value,
                "candidates": [
                    {"name": candidate_name, "has_value": candidate_value is not None}
                    for candidate_name, candidate_value in candidates
                ],
            }
    return {
        "selected_source": "none",
        "selected_value": None,
        "candidates": [
            {"name": candidate_name, "has_value": candidate_value is not None}
            for candidate_name, candidate_value in candidates
        ],
    }


def _as_list(value: Any) -> List[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return cast(List[Any], value)
    if isinstance(value, tuple):
        return list(cast(Iterable[Any], value))
    if isinstance(value, Iterable) and not isinstance(value, (str, bytes, dict)):
        return list(cast(Iterable[Any], value))
    return [value]


def _without_none(values: Any) -> List[Any]:
    return [item for item in _as_list(values) if item is not None]


def _as_dict(value: Any) -> Dict[str, Any]:
    if isinstance(value, dict):
        mapped = cast(Mapping[str, Any], value)
        return {str(key): mapped[key] for key in mapped}
    return {}


def _resolve_layout_inputs(package: Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(package.get("layout_package"), dict):
        package = _as_dict(package.get("layout_package"))

    layout_payload = _as_dict(package.get("layout_payload"))
    reports = _as_dict(package.get("reports"))

    if not layout_payload and isinstance(reports.get("layout_payload"), dict):
        layout_payload = _as_dict(reports.get("layout_payload"))

    geometry_objects = _without_none(
        _pick_first(
            package,
            ["geometry_objects", "rh_geo_preview", "footing_breps", "preview_breps"],
            [],
        )
    )

    annotation_objects = _without_none(package.get("annotation_objects"))
    if not annotation_objects:
        annotation_objects = (
            _without_none(package.get("dim_lines"))
            + _without_none(package.get("extension_lines"))
            + _without_none(package.get("arrowheads"))
            + _without_none(package.get("text_labels"))
            + _without_none(package.get("leader_lines"))
            + _without_none(package.get("code_check_labels"))
        )
    if not annotation_objects:
        annotation_objects = (
            _without_none(reports.get("dim_lines"))
            + _without_none(reports.get("extension_lines"))
            + _without_none(reports.get("arrowheads"))
            + _without_none(reports.get("text_labels"))
            + _without_none(reports.get("leader_lines"))
            + _without_none(reports.get("code_check_labels"))
        )

    metadata = _pick_first(package, ["annotation_metadata", "handoff", "metadata"], {})
    combined_report = _pick_first(package, ["combined_report", "report_text"], reports.get("combined_report"))

    return {
        "geometry_objects": geometry_objects,
        "annotation_objects": annotation_objects,
        "metadata": _as_dict(metadata),
        "combined_report": combined_report,
        "create_named_views": bool(_pick_first(layout_payload, ["create_named_views"], True)),
        "create_layout_guides": bool(_pick_first(layout_payload, ["create_layout_guides"], True)),
        "sheet_size": _pick_first(layout_payload, ["sheet_size"], package.get("sheet_size", "A3")),
        "drawing_scale": _pick_first(layout_payload, ["drawing_scale"], package.get("drawing_scale", 20.0)),
        "title_block_info": _pick_first(layout_payload, ["title_block_info"], package.get("title_block_info", {})),
        "project_name": _pick_first(layout_payload, ["project_name"], package.get("project_name", "")),
        "detail_name": _pick_first(layout_payload, ["detail_name"], package.get("detail_name", "")),
    }


def _build_resolved_inputs_from_args(
    geometry_objects: Any,
    annotation_objects: Any,
    metadata: Any,
    combined_report: Any,
    layout_payload: Any,
    reports: Any,
) -> Dict[str, Any]:
    """Build resolved_inputs directly from 6 named GH inputs."""
    lp = _as_dict(_unwrap_payload(layout_payload)) if layout_payload is not None else {}
    rp = _as_dict(_unwrap_payload(reports)) if reports is not None else {}

    geo = _without_none(_unwrap_payload(geometry_objects)) if geometry_objects is not None else []
    ann = _without_none(_unwrap_payload(annotation_objects)) if annotation_objects is not None else []
    if not ann:
        ann = (
            _without_none(rp.get("dim_lines"))
            + _without_none(rp.get("extension_lines"))
            + _without_none(rp.get("arrowheads"))
            + _without_none(rp.get("text_labels"))
            + _without_none(rp.get("leader_lines"))
            + _without_none(rp.get("code_check_labels"))
        )

    meta = _as_dict(_unwrap_payload(metadata)) if metadata is not None else {}
    rpt = _unwrap_payload(combined_report) if combined_report is not None else rp.get("combined_report")

    return {
        "geometry_objects": geo,
        "annotation_objects": ann,
        "metadata": meta,
        "combined_report": rpt,
        "create_named_views": bool(_pick_first(lp, ["create_named_views"], True)),
        "create_layout_guides": bool(_pick_first(lp, ["create_layout_guides"], True)),
        "sheet_size": _pick_first(lp, ["sheet_size"], "A3"),
        "drawing_scale": _pick_first(lp, ["drawing_scale"], 20.0),
        "title_block_info": _pick_first(lp, ["title_block_info"], {}),
        "project_name": _pick_first(lp, ["project_name"], ""),
        "detail_name": _pick_first(lp, ["detail_name"], ""),
    }


def script(
    package: Any = None,
    payload: Any = None,
    geometry_objects: Any = None,
    annotation_objects: Any = None,
    metadata: Any = None,
    combined_report: Any = None,
    layout_payload: Any = None,
    reports: Any = None,
) -> Dict[str, Any]:
    """
    Grasshopper Python entry point.

    Accepts either:
      - 6 named inputs: geometry_objects, annotation_objects, metadata,
        combined_report, layout_payload, reports  (direct wiring from annotation component)
      - OR a single dict payload via 'package' or 'payload'
    """
    # Detect which mode: 6-input direct wiring vs single-payload
    direct_mode = any(v is not None for v in [
        geometry_objects, annotation_objects, metadata,
        combined_report, layout_payload, reports,
    ])

    if not os.path.exists(HELPER_PATH):
        return {"error": "Layout helper not found at: {0}".format(HELPER_PATH)}

    try:
        layout = _load_helper()
    except Exception as exc:
        return {"error": "Failed to load layout helper: {0}".format(exc)}

    if direct_mode:
        source_label = "direct_6_inputs"
        nested_layout_package_used = False
        resolved_inputs = _build_resolved_inputs_from_args(
            geometry_objects, annotation_objects, metadata,
            combined_report, layout_payload, reports,
        )
    else:
        source_info = _resolve_source_payload(package, payload)
        source_value = source_info["selected_value"]
        source_label = source_info["selected_source"]
        if source_value is None:
            return {
                "error": "No input connected. Wire geometry_objects/annotation_objects/metadata/combined_report/layout_payload/reports, or connect a dict to package/payload.",
                "source_report": source_info,
            }
        unwrapped = _unwrap_payload(source_value)
        if not isinstance(unwrapped, dict):
            return {
                "error": "Single-payload input must be a dict. Got: {0}".format(type(unwrapped).__name__),
                "source_report": source_info,
            }
        payload_dict = cast(Dict[str, Any], unwrapped)
        nested_layout_package_used = False
        if isinstance(payload_dict.get("layout_package"), dict):
            payload_dict = _as_dict(payload_dict.get("layout_package"))
            nested_layout_package_used = True
        resolved_inputs = _resolve_layout_inputs(payload_dict)

    logging.debug("source=%s resolved_inputs keys=%s", source_label, list(resolved_inputs.keys()))

    try:
        result = layout.run(**resolved_inputs)
        # Collect pre-filter counts for diagnostics
        pre_filter_counts = {
            key: len(_as_list(result.get(key))) for key in
            ("viewport_guides", "title_block_curves", "title_block_text", "view_labels", "report_text_block")
        }
        for key in ("viewport_guides", "title_block_curves", "title_block_text", "view_labels", "report_text_block"):
            if key in result:
                result[key] = _without_none(result.get(key))
        # Probe: try creating one test rectangle to verify rg geometry works
        try:
            _test_rect = layout._rectangle(0.0, 0.0, 10.0, 10.0)
            _test_rect_type = type(_test_rect).__name__
            _test_rect_repr = str(_test_rect)[:120]
        except Exception as _te:
            _test_rect_type = "EXCEPTION"
            _test_rect_repr = str(_te)

        result["debug_info"] = {
            "rg_available": getattr(layout, "rg", "?") is not None,
            "create_layout_guides": resolved_inputs.get("create_layout_guides"),
            "sheet_size": resolved_inputs.get("sheet_size"),
            "drawing_scale": resolved_inputs.get("drawing_scale"),
            "source_mode": "direct_6_inputs" if direct_mode else "single_payload",
            "layout_payload_keys": list(_as_dict(
                _unwrap_payload(layout_payload) if direct_mode and layout_payload is not None else {}
            ).keys()),
            "test_rect_type": _test_rect_type,
            "test_rect_repr": _test_rect_repr,
            "pre_filter_counts": pre_filter_counts,
            "post_filter_counts": {
                key: len(_as_list(result.get(key))) for key in
                ("viewport_guides", "title_block_curves", "title_block_text", "view_labels", "report_text_block")
            },
            "first_viewport_guide": str(result.get("viewport_guides", [None])[0])[:120] if result.get("viewport_guides") else "EMPTY",
        }
        result["source_report"] = {
            "source_mode": "direct_6_inputs" if direct_mode else "single_payload",
            "selected_source": source_label,
            "nested_layout_package_used": False if direct_mode else nested_layout_package_used,
            "resolved_counts": {
                "geometry_objects": len(_as_list(resolved_inputs.get("geometry_objects"))),
                "annotation_objects": len(_as_list(resolved_inputs.get("annotation_objects"))),
            },
        }
        return result
    except Exception as e:
        logging.exception("Error while running layout component.")
        return {"error": str(e)}