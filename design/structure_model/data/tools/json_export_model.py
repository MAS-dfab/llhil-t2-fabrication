# Minimal Input Cheat-Sheet (Exporter)
# - GH input names: model (or Model)
# - Function call: build_structure_export_json(model=...)
# - Minimal runnable input: {}
# - Minimal useful topology input:
#   {"nodes": [{"id": "N1", "x": 0, "y": 0, "z": 0}, {"id": "N2", "x": 1, "y": 0, "z": 0}],
#    "edges": [{"id": "E1", "start_node": "N1", "end_node": "N2"}]}
# - GH outputs: ExportJson, MemberLines, AreaLoadMeshes, LinearLoadLines, PointLoadPoints,
#   BoundaryPoints, JointNodes, out
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


def _edge_or_node_guid(item: Dict[str, Any]) -> Optional[str]:
    attrs = item.get("attributes") if isinstance(item.get("attributes"), dict) else {}
    for key in (
        "guid",
        "rhino_guid",
        "proxy_guid",
        "point_guid",
        "pt_guid",
        "curve_guid",
        "line_guid",
        "area_guid",
        "mesh_guid",
    ):
        value = item.get(key)
        if value not in (None, ""):
            return str(value)
        attr_value = attrs.get(key)
        if attr_value not in (None, ""):
            return str(attr_value)
    return None


def _coerce_proxy_map(value: Any) -> Dict[str, str]:
    result: Dict[str, str] = {}
    if isinstance(value, dict):
        for key, map_value in value.items():
            if key in (None, "") or map_value in (None, ""):
                continue
            result[str(key)] = str(map_value)
        return result

    if isinstance(value, list):
        for item in value:
            if not isinstance(item, dict):
                continue
            guid = item.get("guid") or item.get("proxy_guid")
            target_id = item.get("id") or item.get("target_id")
            if guid in (None, "") or target_id in (None, ""):
                continue
            result[str(guid)] = str(target_id)
    return result


def _extract_input_proxy_maps(payload: Dict[str, Any]) -> Dict[str, Dict[str, str]]:
    proxies = {
        "pt": {},
        "curve": {},
        "area": {},
    }

    nested = payload.get("guid_proxies") if isinstance(payload.get("guid_proxies"), dict) else {}

    point_sources = [
        payload.get("pt_guid_proxies"),
        payload.get("pt_guid_proxy"),
        payload.get("point_guid_proxies"),
        payload.get("point_guid_proxy"),
        nested.get("pt"),
        nested.get("points"),
    ]
    curve_sources = [
        payload.get("curve_guid_proxies"),
        payload.get("curve_guid_proxy"),
        payload.get("line_guid_proxies"),
        payload.get("line_guid_proxy"),
        nested.get("curve"),
        nested.get("curves"),
        nested.get("lines"),
    ]
    area_sources = [
        payload.get("area_guid_proxies"),
        payload.get("area_guid_proxy"),
        payload.get("mesh_guid_proxies"),
        payload.get("mesh_guid_proxy"),
        nested.get("area"),
        nested.get("areas"),
        nested.get("meshes"),
    ]

    for source in point_sources:
        proxies["pt"].update(_coerce_proxy_map(source))
    for source in curve_sources:
        proxies["curve"].update(_coerce_proxy_map(source))
    for source in area_sources:
        proxies["area"].update(_coerce_proxy_map(source))

    return proxies


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
    model: Dict[str, Any],
    *,
    schema: str = "structure_model_v1",
) -> Dict[str, Any]:
    """Build export JSON with nodes, edges, meshes and member breps from a Karamba model."""
    nodes_in = _pick_list(model, "nodes", "vertices", "points")
    edges_in = _pick_list(model, "edges", "lines", "members", "beams")
    meshes_in = _pick_list(model, "meshes", "areas", "shells", "faces")
    member_elements_in = _pick_list(model, "member_elements", "elements", "beam_elements")
    member_breps_in = _pick_list(model, "member_breps", "breps")
    sections_in = _pick_list(model, "cross_sections", "sections")
    input_proxies = _extract_input_proxy_maps(model)

    node_records: List[Dict[str, Any]] = []
    node_ids: List[str] = []
    point_guid_proxy: Dict[str, str] = dict(input_proxies["pt"])

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

        node_guid = _edge_or_node_guid(node)
        if node_guid is not None:
            point_guid_proxy[node_guid] = node_id

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
    curve_guid_proxy: Dict[str, str] = dict(input_proxies["curve"])

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

        edge_guid = _edge_or_node_guid(edge)
        if edge_guid is not None:
            curve_guid_proxy[edge_guid] = edge_id

        for brep in breps_by_edge.get(edge_id, []):
            export_breps.append(
                {
                    "edge_id": edge_id,
                    "member_id": brep.get("member_id") or brep.get("id") or edge_id,
                    "brep": brep.get("brep") or brep.get("geometry") or brep,
                }
            )

    mesh_records: List[Dict[str, Any]] = []
    area_guid_proxy: Dict[str, str] = dict(input_proxies["area"])
    for index, mesh in enumerate(meshes_in):
        mesh_id = str(mesh.get("id") or mesh.get("mesh_id") or f"M{index + 1}")
        attrs = mesh.get("attributes") if isinstance(mesh.get("attributes"), dict) else {}
        preserved_attrs = {k: v for k, v in attrs.items() if k not in ("guid", "rhino_guid", "area_guid", "mesh_guid")}

        mesh_record: Dict[str, Any] = {
            "id": mesh_id,
            "attributes": preserved_attrs,
        }
        for key in ("vertices", "faces", "mesh", "geometry", "brep"):
            if key in mesh:
                mesh_record[key] = mesh[key]

        mesh_records.append(mesh_record)

        mesh_guid = _edge_or_node_guid(mesh)
        if mesh_guid is not None:
            area_guid_proxy[mesh_guid] = mesh_id

    member_line_ids = [edge["id"] for edge in edge_records]
    area_mesh_ids = [mesh["id"] for mesh in mesh_records]
    point_ids = [node["id"] for node in node_records]

    node_by_id: Dict[str, Dict[str, Any]] = {node["id"]: node for node in node_records}
    connected_edges_by_node: Dict[str, List[str]] = {}
    for edge in edge_records:
        start_node = str(edge["start_node"])
        end_node = str(edge["end_node"])
        connected_edges_by_node.setdefault(start_node, []).append(str(edge["id"]))
        connected_edges_by_node.setdefault(end_node, []).append(str(edge["id"]))

    joint_records: List[Dict[str, Any]] = []
    for node_id, connected_edges in sorted(connected_edges_by_node.items()):
        # A joint is a node where at least two member lines meet.
        unique_edges = sorted(set(connected_edges))
        if len(unique_edges) < 2:
            continue

        node = node_by_id.get(node_id, {})
        joint_records.append(
            {
                "id": f"J{len(joint_records) + 1}",
                "node_id": node_id,
                "x": node.get("x"),
                "y": node.get("y"),
                "z": node.get("z"),
                "connected_edges": unique_edges,
                "degree": len(unique_edges),
            }
        )

    return {
        "schema": schema,
        "nodes": node_records,
        "edges": edge_records,
        "meshes": mesh_records,
        "member_breps": export_breps,
        "joints": joint_records,
        "output_lists": {
            "member_lines": member_line_ids,
            "area_load_meshes": area_mesh_ids,
            "linear_load_lines": list(member_line_ids),
            "point_load_points": point_ids,
            "boundary_points": list(point_ids),
            "joint_nodes": [joint["node_id"] for joint in joint_records],
        },
        "guid_proxies": {
            "pt": dict(sorted(point_guid_proxy.items())),
            "curve": dict(sorted(curve_guid_proxy.items())),
            "area": dict(sorted(area_guid_proxy.items())),
        },
        "metadata": {
            "source": "karamba",
            "input_nodes": len(nodes_in),
            "input_edges": len(edges_in),
            "input_meshes": len(meshes_in),
            "output_nodes": len(node_records),
            "output_edges": len(edge_records),
            "output_meshes": len(mesh_records),
            "output_member_breps": len(export_breps),
            "output_joints": len(joint_records),
        },
    }


def as_output_lists(model: Dict[str, Any]) -> Dict[str, List[str]]:
    """Return GH-friendly validation lists from the normalized export payload."""
    payload = build_structure_export_json(model)
    lists = payload.get("output_lists", {})
    return {
        "MemberLines": list(lists.get("member_lines", [])),
        "AreaLoadMeshes": list(lists.get("area_load_meshes", [])),
        "LinearLoadLines": list(lists.get("linear_load_lines", [])),
        "PointLoadPoints": list(lists.get("point_load_points", [])),
        "BoundaryPoints": list(lists.get("boundary_points", [])),
        "JointNodes": list(lists.get("joint_nodes", [])),
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
    parser = argparse.ArgumentParser(description="Export Karamba model payload to structure JSON.")
    parser.add_argument("--input", required=True, help="Path to Karamba model JSON source file.")
    parser.add_argument("--output", required=True, help="Path to output structure model JSON file.")
    parser.add_argument("--schema", default="structure_model_v1", help="Schema name written into output JSON.")
    return parser


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    model = _read_json(args.input)
    payload = build_structure_export_json(model=model, schema=args.schema)
    _write_json(args.output, payload)


# GH Py3 auto-run block: input `model` (or `Model`) -> JSON payload + validation lists.
_g = globals()
if "model" in _g or "Model" in _g:
    _model_input = _g.get("model", _g.get("Model"))
    if isinstance(_model_input, dict):
        ExportJson = build_structure_export_json(model=_model_input)
        _lists = as_output_lists(_model_input)
        MemberLines = _lists["MemberLines"]
        AreaLoadMeshes = _lists["AreaLoadMeshes"]
        LinearLoadLines = _lists["LinearLoadLines"]
        PointLoadPoints = _lists["PointLoadPoints"]
        BoundaryPoints = _lists["BoundaryPoints"]
        JointNodes = _lists["JointNodes"]
        out = (
            "Exported nodes: {}, edges: {}, meshes: {}, member_breps: {}, joints: {}"
        ).format(
            len(ExportJson.get("nodes", [])),
            len(ExportJson.get("edges", [])),
            len(ExportJson.get("meshes", [])),
            len(ExportJson.get("member_breps", [])),
            len(ExportJson.get("joints", [])),
        )


if __name__ == "__main__":
    main()
