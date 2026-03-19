# Minimal Input Cheat-Sheet (Adapter)
# - CLI usage:
#     python compas_line_model_adapter.py --input in.json --output out.json
# - Converts COMPAS line-model JSON arrays like:
#     [{"line": {"data": {"start": [...], "end": [...]}, "guid": "...", "name": "..."}, "type": "..."}]
#   into importer-ready JSON objects like:
#     {"lines": [{"start": [...], "end": [...], "guid": "...", "id": "...", "attributes": {...}}]}
from __future__ import annotations

import argparse
import json
from typing import Any, Dict, List


def _normalize_compas_line_record(item: Dict[str, Any]) -> Dict[str, Any]:
    line = item.get("line") if isinstance(item.get("line"), dict) else {}
    data = line.get("data") if isinstance(line.get("data"), dict) else {}

    attributes: Dict[str, Any] = {}
    if item.get("type") is not None:
        attributes["type"] = item.get("type")
    if line.get("dtype") is not None:
        attributes["dtype"] = line.get("dtype")
    if line.get("name") is not None:
        attributes["source_name"] = line.get("name")

    normalized: Dict[str, Any] = {
        "start": data.get("start"),
        "end": data.get("end"),
        "attributes": attributes,
    }

    if line.get("guid") is not None:
        normalized["guid"] = line.get("guid")
    if line.get("name") is not None:
        normalized["id"] = str(line.get("name"))

    return normalized


def _build_common_attributes(item: Dict[str, Any], dtype: Any, source_name: Any) -> Dict[str, Any]:
    attributes: Dict[str, Any] = {}
    if item.get("type") is not None:
        attributes["type"] = item.get("type")
    if dtype is not None:
        attributes["dtype"] = dtype
    if source_name is not None:
        attributes["source_name"] = source_name
    return attributes


def _normalize_geometry_record(item: Dict[str, Any]) -> Dict[str, List[Dict[str, Any]]]:
    geometry = item.get("geometry") if isinstance(item.get("geometry"), dict) else {}
    data = geometry.get("data")
    dtype = str(geometry.get("dtype") or "")
    dtype_lower = dtype.lower()
    geometry_type = str(item.get("geometry_type") or "").lower()
    guid = geometry.get("guid")
    source_name = geometry.get("name")

    attributes = _build_common_attributes(item, dtype if dtype else None, source_name)

    out: Dict[str, List[Dict[str, Any]]] = {"lines": [], "points": [], "meshes": []}

    if "line" in dtype_lower or geometry_type == "line":
        if isinstance(data, dict) and data.get("start") is not None and data.get("end") is not None:
            line_record: Dict[str, Any] = {
                "start": data.get("start"),
                "end": data.get("end"),
                "attributes": attributes,
            }
            if guid is not None:
                line_record["guid"] = guid
            if source_name is not None:
                line_record["id"] = str(source_name)
            out["lines"].append(line_record)
        return out

    if "point" in dtype_lower:
        point_record: Dict[str, Any] = {
            "point": data,
            "attributes": attributes,
        }
        if guid is not None:
            point_record["guid"] = guid
        if source_name is not None:
            point_record["id"] = str(source_name)
        out["points"].append(point_record)
        return out

    if any(token in dtype_lower for token in ("mesh", "surface", "brep")) or geometry_type in (
        "mesh",
        "surface",
        "brep",
        "face",
        "area",
        "panel",
    ):
        mesh_record: Dict[str, Any] = {
            "geometry": data,
            "attributes": attributes,
        }
        if guid is not None:
            mesh_record["guid"] = guid
        if source_name is not None:
            mesh_record["id"] = str(source_name)
        out["meshes"].append(mesh_record)
        return out

    return out


def adapt_compas_line_model(payload: Any) -> Dict[str, Any]:
    if isinstance(payload, list):
        lines: List[Dict[str, Any]] = []
        points: List[Dict[str, Any]] = []
        meshes: List[Dict[str, Any]] = []
        for item in payload:
            if not isinstance(item, dict):
                continue
            if isinstance(item.get("line"), dict):
                lines.append(_normalize_compas_line_record(item))
            elif isinstance(item.get("geometry"), dict):
                normalized = _normalize_geometry_record(item)
                lines.extend(normalized["lines"])
                points.extend(normalized["points"])
                meshes.extend(normalized["meshes"])
            else:
                lines.append(item)
        out: Dict[str, Any] = {"lines": lines}
        if points:
            out["points"] = points
        if meshes:
            out["meshes"] = meshes
        return out

    if isinstance(payload, dict):
        if isinstance(payload.get("lines"), list):
            lines = []
            for item in payload["lines"]:
                if isinstance(item, dict) and isinstance(item.get("line"), dict):
                    lines.append(_normalize_compas_line_record(item))
                else:
                    lines.append(item)
            out = dict(payload)
            out["lines"] = lines
            return out
        return payload

    raise ValueError("Input JSON root must be an object or list.")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert COMPAS line-model JSON to importer-ready JSON.")
    parser.add_argument("--input", required=True, help="Path to source COMPAS JSON file.")
    parser.add_argument("--output", required=True, help="Path to normalized JSON output file.")
    return parser


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    with open(args.input, "r", encoding="utf-8") as stream:
        payload = json.load(stream)

    adapted = adapt_compas_line_model(payload)

    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(adapted, stream, indent=2)


if __name__ == "__main__":
    main()
