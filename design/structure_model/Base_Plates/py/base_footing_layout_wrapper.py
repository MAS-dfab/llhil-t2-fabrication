"""
Grasshopper Python wrapper for the refactored base_footing_layout_component.

Single-source input contract:
- Connect ONE upstream payload to this wrapper via `package` or `payload`.
- Wrapper parses all fields internally (geometry, annotations, metadata, reports,
  and layout settings).

Recommended upstream source:
- `gh_wrapper_annotation` output `layout_package` (or `reports.layout_package`).
"""

import base_footing_layout_component as layout
import logging
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple, cast

try:
    import scriptcontext as sc  # type: ignore
except Exception:
    sc = None

if "package" not in globals():
    package = None
if "payload" not in globals():
    payload = None

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


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

    geometry_objects = _as_list(
        _pick_first(
            package,
            ["geometry_objects", "rh_geo_preview", "footing_breps", "preview_breps"],
            [],
        )
    )

    annotation_objects = _as_list(package.get("annotation_objects"))
    if not annotation_objects:
        annotation_objects = (
            _as_list(package.get("dim_lines"))
            + _as_list(package.get("extension_lines"))
            + _as_list(package.get("arrowheads"))
            + _as_list(package.get("text_labels"))
            + _as_list(package.get("leader_lines"))
            + _as_list(package.get("code_check_labels"))
        )
    if not annotation_objects:
        annotation_objects = (
            _as_list(reports.get("dim_lines"))
            + _as_list(reports.get("extension_lines"))
            + _as_list(reports.get("arrowheads"))
            + _as_list(reports.get("text_labels"))
            + _as_list(reports.get("leader_lines"))
            + _as_list(reports.get("code_check_labels"))
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


def script(package: Any = None, payload: Any = None) -> Dict[str, Any]:
    """
    Grasshopper Python entry point.

    Args:
        package: Single payload source (preferred input name).
        payload: Alias of package.

    Returns:
        dict: Outputs from the layout component.
    """
    source_info = _resolve_source_payload(package, payload)
    source_value = source_info["selected_value"]
    logging.debug("Received source payload: %s", source_value)

    unwrapped = _unwrap_payload(source_value)
    if not isinstance(unwrapped, dict):
        return {
            "error": "Input must be one dictionary-like payload connected to package/payload.",
            "source_report": {
                "selected_source": source_info["selected_source"],
                "nested_layout_package_used": False,
                "candidates": source_info["candidates"],
            },
        }

    payload_dict = cast(Dict[str, Any], unwrapped)
    nested_layout_package_used = False
    if isinstance(payload_dict.get("layout_package"), dict):
        payload_dict = _as_dict(payload_dict.get("layout_package"))
        nested_layout_package_used = True

    resolved_inputs = _resolve_layout_inputs(payload_dict)
    logging.debug("Resolved layout inputs: %s", resolved_inputs)

    try:
        result = layout.run(**resolved_inputs)
        logging.debug("Layout component output: %s", result)
        result["source_report"] = {
            "selected_source": source_info["selected_source"],
            "nested_layout_package_used": nested_layout_package_used,
            "candidates": source_info["candidates"],
            "resolved_counts": {
                "geometry_objects": len(_as_list(resolved_inputs.get("geometry_objects"))),
                "annotation_objects": len(_as_list(resolved_inputs.get("annotation_objects"))),
            },
        }
        result["input_sources"] = {
            "source_mode": "single_payload",
            "connected_input": "package or payload",
            "recommended_upstream": "gh_wrapper_annotation.layout_package",
        }
        return result
    except Exception as e:
        logging.exception("Error while running layout component.")
        return {"error": str(e)}