from __future__ import annotations

import argparse
import json
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
    payload: Dict[str, Any],
    *,
    decimals: int = 6,
    node_prefix: str = "N",
    edge_prefix: str = "E",
    mesh_prefix: str = "M",
) -> Dict[str, Any]:
    """Normalize line-model JSON into deduplicated vertices/edges and analysis target lists."""
    vertices = _extract_vertices(payload)
    edges = _extract_edges(payload)
    meshes = _extract_meshes(payload)
    input_proxies = _extract_input_proxy_maps(payload)

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

    return {
        "schema": "structure_model_v1",
        "nodes": node_records,
        "edges": edge_records,
        "meshes": mesh_records,
        "output_lists": {
            "member_lines": member_line_ids,
            "area_load_meshes": area_mesh_ids,
            "linear_load_lines": list(member_line_ids),
            "point_load_points": point_ids,
            "boundary_points": list(point_ids),
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
            "deduplicated_nodes": max(0, len(vertices) - len(node_records)),
            "deduplicated_edges": max(0, len(edges) - len(edge_records)),
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


if __name__ == "__main__":
    main()
