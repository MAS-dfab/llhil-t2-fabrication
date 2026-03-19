"""Export Grasshopper line inputs to importer-compatible JSON records.

Inputs (GH Python component):
- all_lines (or AllLines): line/curve/guid/compas items
- save (or Save): bool/button
- output_path (or OutputPath): optional target .json path
- allow_empty_overwrite (or AllowEmptyOverwrite): optional bool (default: False)

Outputs:
- ExportSummary: dict with counters and diagnostics
- ExportData: list of exported records
- out: status text
"""

from __future__ import annotations

import json
import os
import tempfile
import traceback
from typing import Any, Dict, List, Optional, Tuple

try:
    import Rhino.Geometry as rg  # type: ignore
    import scriptcontext as sc  # type: ignore
    import System  # type: ignore
    import Rhino  # type: ignore
except Exception:
    rg = None
    sc = None
    System = None
    Rhino = None


Point = Tuple[float, float, float]

DEFAULT_OUTPUT_PATH = (
    r"C:\Users\Juste\Documents\_GitHub\MAS-2526\llhil-t2-fabrication"
    r"\design\structure_model\data\Exchange Files\Files_In\_working\out_lines.json"
)


def _to_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "y", "on"):
            return True
        if text in ("0", "false", "no", "n", "off", ""):
            return False
    return default


def _as_point(value: Any) -> Optional[Point]:
    if value is None:
        return None

    if isinstance(value, dict):
        for keys in (("x", "y", "z"), ("X", "Y", "Z")):
            if all(key in value for key in keys):
                try:
                    return (float(value[keys[0]]), float(value[keys[1]]), float(value[keys[2]]))
                except Exception:
                    return None

        for key in ("point", "xyz", "coords", "position", "start", "end"):
            if key in value:
                pt = _as_point(value.get(key))
                if pt is not None:
                    return pt
        return None

    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            return (float(value[0]), float(value[1]), float(value[2]))
        except Exception:
            return None

    for keys in (("X", "Y", "Z"), ("x", "y", "z")):
        if all(hasattr(value, k) for k in keys):
            try:
                return (float(getattr(value, keys[0])), float(getattr(value, keys[1])), float(getattr(value, keys[2])))
            except Exception:
                return None

    return None


def _point_to_list(point: Point) -> List[float]:
    return [float(point[0]), float(point[1]), float(point[2])]


def _iter_sequence_candidate(value: Any) -> List[Any]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return list(value)
    try:
        return list(value)
    except Exception:
        return []


def _unwrap_once(value: Any) -> Any:
    if value is None:
        return None

    wrapped = getattr(value, "Value", None)
    if wrapped is not None and wrapped is not value:
        return wrapped

    script_var = getattr(value, "ScriptVariable", None)
    if callable(script_var):
        try:
            sv = script_var()
            if sv is not None and sv is not value:
                return sv
        except Exception:
            pass

    return value


def _extract_tree_items(tree_like: Any) -> List[Any]:
    items: List[Any] = []
    if tree_like is None:
        return items

    if hasattr(tree_like, "DataCount") and hasattr(tree_like, "BranchCount"):
        try:
            branch_count = int(getattr(tree_like, "BranchCount"))
        except Exception:
            branch_count = 0
        if branch_count > 0 and hasattr(tree_like, "Branch"):
            for i in range(branch_count):
                try:
                    items.extend(_iter_sequence_candidate(tree_like.Branch(i)))
                except Exception:
                    pass
        if items:
            return items

        if hasattr(tree_like, "AllData"):
            try:
                items.extend(_iter_sequence_candidate(tree_like.AllData()))
            except Exception:
                pass
        return items

    if hasattr(tree_like, "Branches"):
        try:
            branches = _iter_sequence_candidate(getattr(tree_like, "Branches"))
            for branch in branches:
                items.extend(_iter_sequence_candidate(branch))
        except Exception:
            pass
    return items


def _flatten_items(value: Any) -> List[Any]:
    current = value
    for _ in range(6):
        changed = False
        next_value = _unwrap_once(current)
        if next_value is not current:
            current = next_value
            changed = True

        tree_items = _extract_tree_items(current)
        if tree_items:
            current = tree_items
            changed = True

        if not changed:
            break

    if current is None:
        return []
    if isinstance(current, (list, tuple)):
        return list(current)
    return [current]


def _resolve_guid_text(value: Any) -> Optional[str]:
    if value in (None, ""):
        return None

    if System is not None:
        try:
            if isinstance(value, System.Guid):
                return str(value)
        except Exception:
            pass

    text = str(value).strip()
    if not text:
        return None

    if System is not None:
        try:
            return str(System.Guid(text))
        except Exception:
            return None
    return text


def _find_curve_from_guid(guid_text: str) -> Optional[Any]:
    if rg is None or System is None:
        return None

    try:
        guid = System.Guid(guid_text)
    except Exception:
        return None

    docs = []
    try:
        if sc is not None and getattr(sc, "doc", None) is not None:
            docs.append(sc.doc)
    except Exception:
        pass
    try:
        active_doc = Rhino.RhinoDoc.ActiveDoc if Rhino is not None else None
        if active_doc is not None and active_doc not in docs:
            docs.append(active_doc)
    except Exception:
        pass

    for doc in docs:
        try:
            rh_obj = doc.Objects.FindId(guid)
        except Exception:
            rh_obj = None
        if rh_obj is None:
            continue
        try:
            geo = rh_obj.Geometry
        except Exception:
            geo = None
        if isinstance(geo, rg.Curve):
            return geo
    return None


def _coerce_curve(value: Any) -> Tuple[Optional[Any], Optional[str]]:
    if rg is None or value is None:
        return None, None

    obj = _unwrap_once(value)

    if isinstance(obj, rg.Curve):
        return obj, None
    if isinstance(obj, rg.Line):
        return rg.LineCurve(obj), None

    if isinstance(obj, dict):
        for key in ("line", "curve", "geometry", "guid", "id"):
            if key in obj:
                curve, guid_text = _coerce_curve(obj.get(key))
                if curve is not None:
                    return curve, guid_text

    guid_text = _resolve_guid_text(obj)
    if guid_text is not None:
        curve = _find_curve_from_guid(guid_text)
        if curve is not None:
            return curve, guid_text

    return None, None


def _extract_compas_record(value: Any) -> Optional[Dict[str, Any]]:
    if value is None:
        return None

    obj = _unwrap_once(value)

    # Already importer-compatible.
    if isinstance(obj, dict):
        line_obj = obj.get("line")
        if isinstance(line_obj, dict):
            data = line_obj.get("data") if isinstance(line_obj.get("data"), dict) else {}
            start = _as_point(data.get("start"))
            end = _as_point(data.get("end"))
            if start is not None and end is not None:
                out = dict(obj)
                out_line = dict(line_obj)
                out_line["data"] = {
                    "start": _point_to_list(start),
                    "end": _point_to_list(end),
                }
                out["line"] = out_line
                if "type" not in out:
                    out["type"] = getattr(value, "name", None)
                return out

        # Simple dict with start/end.
        start = _as_point(obj.get("start") or obj.get("from") or obj.get("u"))
        end = _as_point(obj.get("end") or obj.get("to") or obj.get("v"))
        if start is not None and end is not None:
            return {
                "line": {
                    "dtype": "compas.geometry/Line",
                    "data": {
                        "start": _point_to_list(start),
                        "end": _point_to_list(end),
                    },
                    "guid": _resolve_guid_text(obj.get("guid") or obj.get("id")),
                    "name": obj.get("name"),
                },
                "type": obj.get("type") or obj.get("name"),
            }

    if hasattr(obj, "__jsondump__"):
        try:
            dumped = obj.__jsondump__()
            if isinstance(dumped, dict):
                data = dumped.get("data") if isinstance(dumped.get("data"), dict) else {}
                start = _as_point(data.get("start"))
                end = _as_point(data.get("end"))
                if start is not None and end is not None:
                    return {
                        "line": {
                            "dtype": dumped.get("dtype", "compas.geometry/Line"),
                            "data": {
                                "start": _point_to_list(start),
                                "end": _point_to_list(end),
                            },
                            "guid": dumped.get("guid"),
                            "name": dumped.get("name"),
                        },
                        "type": getattr(obj, "name", None),
                    }
        except Exception:
            pass

    if hasattr(obj, "to_data"):
        try:
            dumped = obj.to_data()
            if isinstance(dumped, dict):
                start = _as_point(dumped.get("start"))
                end = _as_point(dumped.get("end"))
                if start is not None and end is not None:
                    return {
                        "line": {
                            "dtype": "{}/{}".format(obj.__class__.__module__, obj.__class__.__name__),
                            "data": {
                                "start": _point_to_list(start),
                                "end": _point_to_list(end),
                            },
                            "guid": None,
                            "name": getattr(obj, "name", None),
                        },
                        "type": getattr(obj, "name", None),
                    }
        except Exception:
            pass

    return None


def _curve_to_record(curve: Any, guid_text: Optional[str], source_value: Any) -> Optional[Dict[str, Any]]:
    if rg is None or curve is None:
        return None
    try:
        a = curve.PointAtStart
        b = curve.PointAtEnd
    except Exception:
        return None

    return {
        "line": {
            "dtype": "compas.geometry/Line",
            "data": {
                "start": [float(a.X), float(a.Y), float(a.Z)],
                "end": [float(b.X), float(b.Y), float(b.Z)],
            },
            "guid": guid_text,
            "name": getattr(source_value, "name", None),
        },
        "type": getattr(source_value, "name", None),
    }


def _resolve_output_path(raw_path: Any) -> str:
    if raw_path in (None, ""):
        return DEFAULT_OUTPUT_PATH

    text = str(raw_path).replace("\r", "").replace("\n", "").strip().strip('"').strip("'")
    if not text:
        return DEFAULT_OUTPUT_PATH

    normalized = os.path.abspath(text)
    if normalized.lower().endswith(".json"):
        return normalized
    if os.path.isdir(normalized) or normalized.endswith(("\\", "/")):
        return os.path.join(normalized, "out_lines.json")
    return normalized + ".json"


def _atomic_write_json(path: str, payload: Any) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    fd, tmp_path = tempfile.mkstemp(prefix="out_lines_", suffix=".json", dir=parent or None)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2)
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.remove(tmp_path)
        except Exception:
            pass
        raise


def export_lines_payload(all_lines: Any) -> Dict[str, Any]:
    items = _flatten_items(all_lines)

    export_data: List[Dict[str, Any]] = []
    skipped = 0
    skipped_by_type: Dict[str, int] = {}
    resolved_from_compas = 0
    resolved_from_curve = 0

    for item in items:
        record = _extract_compas_record(item)
        if record is not None:
            export_data.append(record)
            resolved_from_compas += 1
            continue

        curve, guid_text = _coerce_curve(item)
        if curve is not None:
            record = _curve_to_record(curve, guid_text, item)
            if record is not None:
                export_data.append(record)
                resolved_from_curve += 1
                continue

        skipped += 1
        type_name = type(item).__name__
        skipped_by_type[type_name] = skipped_by_type.get(type_name, 0) + 1

    return {
        "records": export_data,
        "summary": {
            "input_count": len(items),
            "resolved_count": len(export_data),
            "resolved_from_compas": resolved_from_compas,
            "resolved_from_curve": resolved_from_curve,
            "skipped_count": skipped,
            "skipped_by_type": dict(sorted(skipped_by_type.items())),
        },
    }


_g = globals()

ExportSummary: Dict[str, Any] = {
    "input_count": 0,
    "resolved_count": 0,
    "skipped_count": 0,
}
ExportData: List[Dict[str, Any]] = []
out = "Waiting for all_lines input."


def _first_input(names: List[str]) -> Any:
    for name in names:
        if name in _g:
            return _g.get(name)
    return None


if "all_lines" in _g or "AllLines" in _g:
    try:
        _all_lines = _first_input(["all_lines", "AllLines"])
        _save = _to_bool(_first_input(["save", "Save"]), default=False)
        _allow_empty_overwrite = _to_bool(
            _first_input(["allow_empty_overwrite", "AllowEmptyOverwrite"]), default=False
        )
        _output_path = _resolve_output_path(_first_input(["output_path", "OutputPath", "json_path", "JsonPath"]))

        _result = export_lines_payload(_all_lines)
        ExportData = list(_result.get("records", []))
        ExportSummary = dict(_result.get("summary", {}))

        _resolved = int(ExportSummary.get("resolved_count", 0))
        _skipped = int(ExportSummary.get("skipped_count", 0))

        if _save:
            if _resolved > 0 or _allow_empty_overwrite:
                _atomic_write_json(_output_path, ExportData)
                out = "Saved {} records to {} (skipped: {}).".format(_resolved, _output_path, _skipped)
            else:
                out = "No records resolved; skipped file overwrite (skipped: {}).".format(_skipped)
        else:
            out = "Save is False. Resolved {} records (skipped: {}).".format(_resolved, _skipped)

    except Exception as ex:
        ExportSummary = {
            "error": str(ex),
            "traceback": traceback.format_exc(),
        }
        ExportData = []
        out = "Export failed: {}".format(ex)
