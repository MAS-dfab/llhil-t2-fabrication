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


def adapt_compas_line_model(payload: Any) -> Dict[str, Any]:
    if isinstance(payload, list):
        lines: List[Dict[str, Any]] = []
        for item in payload:
            if not isinstance(item, dict):
                continue
            if isinstance(item.get("line"), dict):
                lines.append(_normalize_compas_line_record(item))
            else:
                lines.append(item)
        return {"lines": lines}

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
