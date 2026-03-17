from __future__ import annotations

import argparse
import json
from typing import Any, Dict, List, Optional


def _normalize_dict_list(value: Any) -> List[Dict[str, Any]]:
    if value is None:
        return []
    if isinstance(value, list):
        return [item for item in value if isinstance(item, dict)]
    if isinstance(value, dict):
        return [value]
    return []


def _pick_list(payload: Dict[str, Any], *keys: str) -> List[Dict[str, Any]]:
    for key in keys:
        items = _normalize_dict_list(payload.get(key))
        if items:
            return items
    return []


def _resolve_node_id(node: Dict[str, Any], index: int, prefix: str = "N") -> str:
    for key in ("id", "node_id", "identifier"):
        value = node.get(key)
        if value not in (None, ""):
            return str(value)
    return f"{prefix}{index + 1}"


def _resolve_edge_id(edge: Dict[str, Any], index: int, prefix: str = "E") -> str:
    for key in ("id", "edge_id", "line_id", "member_id", "identifier"):
        value = edge.get(key)
        if value not in (None, ""):
            return str(value)
    return f"{prefix}{index + 1}"


def _to_node_ref(value: Any, node_ids: List[str]) -> Optional[str]:
    if value is None:
        return None

    text = str(value)
    if text in node_ids:
        return text

    try:
        index = int(value)
    except (TypeError, ValueError):
        return None

    if 0 <= index < len(node_ids):
        return node_ids[index]
    if 1 <= index <= len(node_ids):
        return node_ids[index - 1]
    return None


def _edge_node_ids(edge: Dict[str, Any], node_ids: List[str]) -> Optional[tuple[str, str]]:
    attrs = edge.get("attributes") if isinstance(edge.get("attributes"), dict) else {}

    start = None
    end = None

    for key in ("start_node", "u", "from", "a"):
        start = _to_node_ref(edge.get(key), node_ids)
        if start is not None:
            break
    for key in ("end_node", "v", "to", "b"):
        end = _to_node_ref(edge.get(key), node_ids)
        if end is not None:
            break

    if start is None:
        start = _to_node_ref(attrs.get("start_node"), node_ids)
    if end is None:
        end = _to_node_ref(attrs.get("end_node"), node_ids)

    if start is None or end is None or start == end:
        return None

    return start, end


def build_structure_export_json(
    karamba_payload: Dict[str, Any],
    *,
    schema: str = "structure_model_v1",
) -> Dict[str, Any]:
    """Build export JSON with nodes, edges and member breps from Karamba-like payload."""
    nodes_in = _pick_list(karamba_payload, "nodes", "vertices", "points")
    edges_in = _pick_list(karamba_payload, "edges", "lines", "members", "beams")
    member_elements_in = _pick_list(karamba_payload, "member_elements", "elements", "beam_elements")
    member_breps_in = _pick_list(karamba_payload, "member_breps", "breps")
    sections_in = _pick_list(karamba_payload, "cross_sections", "sections")

    node_records: List[Dict[str, Any]] = []
    node_ids: List[str] = []

    for index, node in enumerate(nodes_in):
        node_id = _resolve_node_id(node, index)
        node_ids.append(node_id)

        record = {
            "id": node_id,
            "x": float(node.get("x", 0.0)),
            "y": float(node.get("y", 0.0)),
            "z": float(node.get("z", 0.0)),
        }

        for key in ("support", "restraint", "attributes"):
            if key in node:
                record[key] = node[key]

        node_records.append(record)

    section_by_edge: Dict[str, Any] = {}
    for entry in sections_in:
        edge_key = entry.get("edge_id") or entry.get("line_id") or entry.get("id")
        if edge_key is None:
            continue
        section_by_edge[str(edge_key)] = entry.get("section") or entry.get("name") or entry

    elements_by_edge: Dict[str, List[Dict[str, Any]]] = {}
    for element in member_elements_in:
        edge_key = element.get("edge_id") or element.get("line_id") or element.get("member_id")
        if edge_key is None:
            continue
        edge_id = str(edge_key)
        elements_by_edge.setdefault(edge_id, []).append(element)

    breps_by_edge: Dict[str, List[Dict[str, Any]]] = {}
    for brep in member_breps_in:
        edge_key = brep.get("edge_id") or brep.get("line_id") or brep.get("member_id")
        if edge_key is None:
            continue
        edge_id = str(edge_key)
        breps_by_edge.setdefault(edge_id, []).append(brep)

    edge_records: List[Dict[str, Any]] = []
    export_breps: List[Dict[str, Any]] = []

    for index, edge in enumerate(edges_in):
        edge_id = _resolve_edge_id(edge, index)
        node_pair = _edge_node_ids(edge, node_ids)
        if node_pair is None:
            continue

        start_node, end_node = node_pair
        edge_record: Dict[str, Any] = {
            "id": edge_id,
            "start_node": start_node,
            "end_node": end_node,
        }

        if edge_id in section_by_edge:
            edge_record["cross_section"] = section_by_edge[edge_id]
        elif "cross_section" in edge:
            edge_record["cross_section"] = edge["cross_section"]
        elif "cross_sections" in edge:
            edge_record["cross_section"] = edge["cross_sections"]

        members = elements_by_edge.get(edge_id, [])
        if members:
            edge_record["member_elements"] = members

        edge_records.append(edge_record)

        for brep in breps_by_edge.get(edge_id, []):
            export_breps.append(
                {
                    "edge_id": edge_id,
                    "member_id": brep.get("member_id") or brep.get("id") or edge_id,
                    "brep": brep.get("brep") or brep.get("geometry") or brep,
                }
            )

    return {
        "schema": schema,
        "nodes": node_records,
        "edges": edge_records,
        "member_breps": export_breps,
        "metadata": {
            "source": "karamba",
            "input_nodes": len(nodes_in),
            "input_edges": len(edges_in),
            "output_nodes": len(node_records),
            "output_edges": len(edge_records),
            "output_member_breps": len(export_breps),
        },
    }


def _read_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("Input JSON root must be an object.")
    return data


def _write_json(path: str, payload: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export Karamba-like model payload to structure JSON.")
    parser.add_argument("--input", required=True, help="Path to Karamba model JSON source file.")
    parser.add_argument("--output", required=True, help="Path to output structure model JSON file.")
    parser.add_argument("--schema", default="structure_model_v1", help="Schema name written into output JSON.")
    return parser


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    source = _read_json(args.input)
    payload = build_structure_export_json(source, schema=args.schema)
    _write_json(args.output, payload)


if __name__ == "__main__":
    main()
