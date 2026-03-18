# Minimal Input Cheat-Sheet (Importer)
# - GH input names: model (or Model)
#   (can be a parsed JSON object/list OR a file path string to JSON)
# - Optional GH proxy inputs:
#     pt_guid_proxies         (aliases: PtGuidProxies, PointGuidProxies)
#     curve_guid_proxies      (aliases: CurveGuidProxies, LineGuidProxies)
#     area_guid_proxies       (aliases: AreaGuidProxies, MeshGuidProxies)
#     point_load_guid_proxies (alias: PointLoadGuidProxies) -> filters PointLoadPoints
#     boundary_guid_proxies   (alias: BoundaryGuidProxies)  -> filters BoundaryPoints
# - Function call: import_line_model_json(payload=..., pt_guid_proxies=...,
#     curve_guid_proxies=..., area_guid_proxies=...,
#     point_load_guid_proxies=..., boundary_guid_proxies=...)
# - Minimal runnable input: {}
# - Minimal useful topology input:
#   {"lines": [{"start": [0, 0, 0], "end": [1, 0, 0]}]}
# - Also accepts COMPAS-style root arrays with entries like:
#   {"line": {"data": {"start": [...], "end": [...]}, "guid": "...", "name": "..."}, "type": "..."}
# - GH outputs: ImportJson, MemberLines, AreaLoadMeshes, LinearLoadLines, PointLoadPoints,
#   BoundaryPoints, JointNodes, out
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


Point = Tuple[float, float, float]


def _as_point(value: Any) -> Optional[Point]:
    """Convert common point formats into a 3D tuple."""
    if value is None:
        return None

    if isinstance(value, dict):
        if all(axis in value for axis in ("x", "y", "z")):
            try:
                return (float(value["x"]), float(value["y"]), float(value["z"]))
            except (TypeError, ValueError):
                return None

        for key in ("point", "xyz", "coords", "position"):
            if key in value:
                return _as_point(value[key])

        return None

    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            return (float(value[0]), float(value[1]), float(value[2]))
        except (TypeError, ValueError):
            return None

    return None


def _first_point(*candidates: Any) -> Optional[Point]:
    for candidate in candidates:
        point = _as_point(candidate)
        if point is not None:
            return point
    return None


def _line_start_end(line: Dict[str, Any]) -> Tuple[Optional[Point], Optional[Point]]:
    attrs = line.get("attributes") if isinstance(line.get("attributes"), dict) else {}

    start = _first_point(
        line.get("start"),
        line.get("from"),
        line.get("u"),
        line.get("a"),
        line.get("start_point"),
        line.get("startPoint"),
        line.get("start_node"),
        attrs.get("start"),
        attrs.get("start_point"),
        attrs.get("start_node"),
    )

    end = _first_point(
        line.get("end"),
        line.get("to"),
        line.get("v"),
        line.get("b"),
        line.get("end_point"),
        line.get("endPoint"),
        line.get("end_node"),
        attrs.get("end"),
        attrs.get("end_point"),
        attrs.get("end_node"),
    )

    return start, end


def _normalize_item_list(value: Any) -> List[Dict[str, Any]]:
    if value is None:
        return []
    if isinstance(value, list):
        return [item for item in value if isinstance(item, dict)]
    if isinstance(value, dict):
        return [value]
    return []


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


def _load_payload_from_path(payload: Any) -> Any:
    # GH may pass path-like values that are not strict Python str instances.
    if payload is None:
        return payload

    if not isinstance(payload, (dict, list, tuple)):
        candidate = str(payload).strip()
        if candidate:
            path = Path(candidate)
            if path.is_file():
                with open(path, "r", encoding="utf-8") as stream:
                    return json.load(stream)
    return payload


def _normalize_input_payload(payload: Any) -> Dict[str, Any]:
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

    if not isinstance(payload, dict):
        raise ValueError("Input JSON root must be an object or list.")

    normalized = dict(payload)
    for key in ("lines", "edges", "members", "segments"):
        items = normalized.get(key)
        if not isinstance(items, list):
            continue
        normalized[key] = [
            _normalize_compas_line_record(item) if isinstance(item, dict) and isinstance(item.get("line"), dict) else item
            for item in items
        ]
    return normalized


def _extract_vertices(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    for key in ("vertices", "nodes", "points"):
        if key in payload:
            return _normalize_item_list(payload.get(key))
    return []


def _extract_edges(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    for key in ("edges", "lines", "members", "segments"):
        if key in payload:
            return _normalize_item_list(payload.get(key))
    return []


def _extract_lines_only(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    for key in ("lines", "members", "edges", "segments"):
        items = _normalize_item_list(payload.get(key))
        if items:
            return items
    return []


def _extract_meshes(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    for key in ("meshes", "areas", "faces", "panels"):
        if key in payload:
            return _normalize_item_list(payload.get(key))
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


def _resolve_filtered_node_ids(
    value: Any, guid_map: Dict[str, str]
) -> Optional[List[str]]:
    """Resolve a GUID proxy input to a filtered list of node IDs.

    Returns None when no input is provided, so the caller can fall back to all
    node IDs. Supports:
    - list of GUID strings -> looked up in guid_map
    - dict {guid: node_id} -> node_id used directly (guid_map as fallback)
    - list of {guid, id} dicts -> id value used directly
    """
    if value is None:
        return None

    node_ids: List[str] = []

    if isinstance(value, dict):
        for guid, node_id in value.items():
            if guid in (None, ""):
                continue
            resolved = str(node_id) if node_id not in (None, "") else guid_map.get(str(guid))
            if resolved:
                node_ids.append(resolved)
        return node_ids if node_ids else None

    if isinstance(value, (list, tuple)):
        for item in value:
            if isinstance(item, str) and item:
                resolved = guid_map.get(item)
                if resolved:
                    node_ids.append(resolved)
            elif isinstance(item, dict):
                guid = item.get("guid") or item.get("proxy_guid")
                node_id = item.get("id") or item.get("target_id") or item.get("node_id")
                if node_id not in (None, ""):
                    node_ids.append(str(node_id))
                elif guid and str(guid) in guid_map:
                    node_ids.append(guid_map[str(guid)])
        return node_ids if node_ids else None

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


def import_line_model_json(
    payload: Any,
    *,
    decimals: int = 6,
    node_prefix: str = "N",
    edge_prefix: str = "E",
    mesh_prefix: str = "M",
    pt_guid_proxies: Any = None,
    curve_guid_proxies: Any = None,
    area_guid_proxies: Any = None,
    point_load_guid_proxies: Any = None,
    boundary_guid_proxies: Any = None,
) -> Dict[str, Any]:
    """Normalize line-model JSON into deduplicated vertices/edges and analysis target lists."""
    payload = _load_payload_from_path(payload)
    payload = _normalize_input_payload(payload)
    vertices = _extract_vertices(payload)
    edges = _extract_edges(payload)
    meshes = _extract_meshes(payload)
    input_proxies = _extract_input_proxy_maps(payload)
    # Explicit proxy inputs override/extend proxies embedded in the model payload.
    input_proxies["pt"].update(_coerce_proxy_map(pt_guid_proxies))
    input_proxies["curve"].update(_coerce_proxy_map(curve_guid_proxies))
    input_proxies["area"].update(_coerce_proxy_map(area_guid_proxies))

    # Fallback for models that only contain lines with embedded start/end node data.
    if not vertices and not edges:
        edges = _extract_lines_only(payload)

    node_by_key: Dict[Tuple[float, float, float], str] = {}
    node_records: List[Dict[str, Any]] = []
    point_guid_proxy: Dict[str, str] = dict(input_proxies["pt"])

    def ensure_node_id(point: Point) -> str:
        rounded = (round(point[0], decimals), round(point[1], decimals), round(point[2], decimals))
        existing = node_by_key.get(rounded)
        if existing is not None:
            return existing

        node_id = f"{node_prefix}{len(node_records) + 1}"
        node_by_key[rounded] = node_id
        node_records.append(
            {
                "id": node_id,
                "x": rounded[0],
                "y": rounded[1],
                "z": rounded[2],
            }
        )
        return node_id

    source_vertex_id_to_new: Dict[str, str] = {}
    for vertex in vertices:
        point = _first_point(vertex, vertex.get("point"), vertex.get("xyz"), vertex.get("coords"))
        if point is None:
            continue
        new_id = ensure_node_id(point)
        source_id = vertex.get("id")
        if source_id is not None:
            source_vertex_id_to_new[str(source_id)] = new_id
        vertex_guid = _edge_or_node_guid(vertex)
        if vertex_guid is not None:
            point_guid_proxy[vertex_guid] = new_id

    edge_records: List[Dict[str, Any]] = []
    seen_edge_pairs: Dict[Tuple[str, str], str] = {}
    curve_guid_proxy: Dict[str, str] = dict(input_proxies["curve"])

    for raw_edge in edges:
        attrs = raw_edge.get("attributes") if isinstance(raw_edge.get("attributes"), dict) else {}

        start_id = None
        end_id = None

        for key in ("start_node_id", "start_node", "u", "from", "a"):
            value = raw_edge.get(key)
            if value is not None and str(value) in source_vertex_id_to_new:
                start_id = source_vertex_id_to_new[str(value)]
                break
        for key in ("end_node_id", "end_node", "v", "to", "b"):
            value = raw_edge.get(key)
            if value is not None and str(value) in source_vertex_id_to_new:
                end_id = source_vertex_id_to_new[str(value)]
                break

        if start_id is None or end_id is None:
            start_point, end_point = _line_start_end(raw_edge)
            if start_id is None and start_point is not None:
                start_id = ensure_node_id(start_point)
            if end_id is None and end_point is not None:
                end_id = ensure_node_id(end_point)

        if start_id is None or end_id is None or start_id == end_id:
            continue

        pair = tuple(sorted((start_id, end_id)))
        if pair in seen_edge_pairs:
            continue

        edge_id = f"{edge_prefix}{len(edge_records) + 1}"
        seen_edge_pairs[pair] = edge_id

        preserved_attrs = {k: v for k, v in attrs.items() if k not in ("start", "end", "start_node", "end_node")}

        edge_record = {
            "id": edge_id,
            "start_node": start_id,
            "end_node": end_id,
            "attributes": preserved_attrs,
        }

        source_edge_id = raw_edge.get("id")
        if source_edge_id is not None:
            edge_record["source_id"] = source_edge_id

        if "cross_sections" in raw_edge:
            edge_record["cross_sections"] = raw_edge["cross_sections"]
        elif "cross_sections" in attrs:
            edge_record["cross_sections"] = attrs.get("cross_sections")

        edge_records.append(edge_record)

        edge_guid = _edge_or_node_guid(raw_edge)
        if edge_guid is not None:
            curve_guid_proxy[edge_guid] = edge_id

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

    mesh_records: List[Dict[str, Any]] = []
    area_guid_proxy: Dict[str, str] = dict(input_proxies["area"])
    for raw_mesh in meshes:
        mesh_id = f"{mesh_prefix}{len(mesh_records) + 1}"
        attrs = raw_mesh.get("attributes") if isinstance(raw_mesh.get("attributes"), dict) else {}
        preserved_attrs = {k: v for k, v in attrs.items() if k not in ("guid", "rhino_guid", "area_guid", "mesh_guid")}

        mesh_record = {
            "id": mesh_id,
            "attributes": preserved_attrs,
        }

        for key in ("vertices", "faces", "mesh", "geometry", "brep"):
            if key in raw_mesh:
                mesh_record[key] = raw_mesh[key]

        source_mesh_id = raw_mesh.get("id")
        if source_mesh_id is not None:
            mesh_record["source_id"] = source_mesh_id

        mesh_records.append(mesh_record)

        mesh_guid = _edge_or_node_guid(raw_mesh)
        if mesh_guid is not None:
            area_guid_proxy[mesh_guid] = mesh_id

    member_line_ids = [edge["id"] for edge in edge_records]
    area_mesh_ids = [mesh["id"] for mesh in mesh_records]
    point_ids = [node["id"] for node in node_records]

    # Resolve explicitly tagged GUID inputs to filtered node ID lists.
    # Falls back to all node IDs when the proxy input is not provided.
    _pl_ids = _resolve_filtered_node_ids(point_load_guid_proxies, point_guid_proxy)
    _bp_ids = _resolve_filtered_node_ids(boundary_guid_proxies, point_guid_proxy)

    return {
        "schema": "structure_model_v1",
        "nodes": node_records,
        "edges": edge_records,
        "meshes": mesh_records,
        "joints": joint_records,
        "output_lists": {
            "member_lines": member_line_ids,
            "area_load_meshes": area_mesh_ids,
            "linear_load_lines": list(member_line_ids),
            "point_load_points": _pl_ids if _pl_ids is not None else point_ids,
            "boundary_points": _bp_ids if _bp_ids is not None else point_ids,
            "joint_nodes": [joint["node_id"] for joint in joint_records],
        },
        "guid_proxies": {
            "pt": dict(sorted(point_guid_proxy.items())),
            "curve": dict(sorted(curve_guid_proxy.items())),
            "area": dict(sorted(area_guid_proxy.items())),
        },
        "metadata": {
            "input_nodes": len(vertices),
            "input_edges": len(edges),
            "input_meshes": len(meshes),
            "output_nodes": len(node_records),
            "output_edges": len(edge_records),
            "output_meshes": len(mesh_records),
            "output_joints": len(joint_records),
            "deduplicated_nodes": max(0, len(vertices) - len(node_records)),
            "deduplicated_edges": max(0, len(edges) - len(edge_records)),
        },
    }


def as_output_lists(model: Any) -> Dict[str, List[str]]:
    """Return GH-friendly validation lists from the normalized import payload."""
    payload = import_line_model_json(model)
    lists = payload.get("output_lists", {})
    return {
        "MemberLines": list(lists.get("member_lines", [])),
        "AreaLoadMeshes": list(lists.get("area_load_meshes", [])),
        "LinearLoadLines": list(lists.get("linear_load_lines", [])),
        "PointLoadPoints": list(lists.get("point_load_points", [])),
        "BoundaryPoints": list(lists.get("boundary_points", [])),
        "JointNodes": list(lists.get("joint_nodes", [])),
    }


def _read_json(path: str) -> Any:
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def _write_json(path: str, payload: Dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Normalize line model JSON into nodes/edges/meshes with IDs.")
    parser.add_argument("--input", required=True, help="Path to source JSON file.")
    parser.add_argument("--output", required=True, help="Path to normalized JSON output file.")
    parser.add_argument("--decimals", type=int, default=6, help="Rounding precision for vertex deduplication.")
    parser.add_argument("--node-prefix", default="N", help="Node ID prefix.")
    parser.add_argument("--edge-prefix", default="E", help="Edge ID prefix.")
    parser.add_argument("--mesh-prefix", default="M", help="Mesh ID prefix.")
    return parser


def main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    source = _read_json(args.input)
    normalized = import_line_model_json(
        source,
        decimals=args.decimals,
        node_prefix=args.node_prefix,
        edge_prefix=args.edge_prefix,
        mesh_prefix=args.mesh_prefix,
    )
    _write_json(args.output, normalized)


# GH Py3 auto-run block: input `model` (or `Model`) -> JSON payload + validation lists.
_g = globals()

# Always initialize outputs so GH does not show empty outputs without context.
ImportJson = ""
MemberLines = []
AreaLoadMeshes = []
LinearLoadLines = []
PointLoadPoints = []
BoundaryPoints = []
JointNodes = []
out = "Waiting for Model input."


def _get_first_input(globals_dict: Dict[str, Any], names: List[str]) -> Any:
    for name in names:
        if name in globals_dict:
            return globals_dict.get(name)
    return None


if "model" in _g or "Model" in _g:
    _model_input = _get_first_input(_g, ["model", "Model"])
    if _model_input in (None, ""):
        out = "Model input is empty. Provide a JSON path or parsed JSON object/list."
    else:
        try:
            _pt_proxies = _get_first_input(_g, ["pt_guid_proxies", "PtGuidProxies", "PointGuidProxies"])
            _curve_proxies = _get_first_input(_g, ["curve_guid_proxies", "CurveGuidProxies", "LineGuidProxies"])
            _area_proxies = _get_first_input(_g, ["area_guid_proxies", "AreaGuidProxies", "MeshGuidProxies"])
            _pl_proxies = _get_first_input(_g, ["point_load_guid_proxies", "PointLoadGuidProxies"])
            _bp_proxies = _get_first_input(_g, ["boundary_guid_proxies", "BoundaryGuidProxies"])

            _import_payload = import_line_model_json(
                _model_input,
                pt_guid_proxies=_pt_proxies,
                curve_guid_proxies=_curve_proxies,
                area_guid_proxies=_area_proxies,
                point_load_guid_proxies=_pl_proxies,
                boundary_guid_proxies=_bp_proxies,
            )
            ImportJson = json.dumps(_import_payload, indent=2)
            _lists = _import_payload.get("output_lists", {})
            MemberLines = list(_lists.get("member_lines", []))
            AreaLoadMeshes = list(_lists.get("area_load_meshes", []))
            LinearLoadLines = list(_lists.get("linear_load_lines", []))
            PointLoadPoints = list(_lists.get("point_load_points", []))
            BoundaryPoints = list(_lists.get("boundary_points", []))
            JointNodes = list(_lists.get("joint_nodes", []))
            out = (
                "Imported nodes: {}, edges: {}, meshes: {}, joints: {}"
            ).format(
                len(_import_payload.get("nodes", [])),
                len(_import_payload.get("edges", [])),
                len(_import_payload.get("meshes", [])),
                len(_import_payload.get("joints", [])),
            )
        except Exception as ex:
            out = "Import failed: {}".format(ex)


if __name__ == "__main__" and any(arg.startswith("--input") for arg in sys.argv[1:]):
    main()
