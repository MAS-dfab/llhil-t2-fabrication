# Minimal Input Cheat-Sheet (Exporter)
# - GH input names: model (or Model), save (or Save), output_path (or OutputPath)
#   - save: wire a GH Button → triggers JSON export to disk when True
#   - output_path: wire a GH Panel with the full file path (e.g. .../Files_Out/out_model.json)
# - Function call: build_structure_export_json(model=...)
# - Minimal runnable input: {}
# - Minimal useful topology input:
#   {"nodes": [{"id": "N1", "x": 0, "y": 0, "z": 0}, {"id": "N2", "x": 1, "y": 0, "z": 0}],
#    "edges": [{"id": "E1", "start_node": "N1", "end_node": "N2"}]}
# - GH outputs: ExportJson, MemberLines, AreaLoadMeshes, LinearLoadLines, PointLoadPoints,
#   BoundaryPoints, JointNodes, GHLineCurves, GHNodes, GHLoadPoints, GHSurfaces, out
from __future__ import annotations

import argparse
import json
import os
import traceback
from typing import Any, Dict, List, Optional, Tuple

try:
    import Rhino.Geometry as rg  # type: ignore
except Exception:  # pragma: no cover - Rhino is not available in CLI environments.
    rg = None


Point = Tuple[float, float, float]


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


def _as_point(value: Any) -> Optional[Point]:
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


def _node_point(node: Dict[str, Any]) -> Optional[Point]:
    attrs = node.get("attributes") if isinstance(node.get("attributes"), dict) else {}
    return _first_point(node, node.get("point"), attrs.get("point"), attrs.get("xyz"))


def _line_start_end(edge: Dict[str, Any], node_by_id: Dict[str, Dict[str, Any]]) -> Tuple[Optional[Point], Optional[Point]]:
    attrs = edge.get("attributes") if isinstance(edge.get("attributes"), dict) else {}
    start_id = edge.get("start_node") or edge.get("u") or edge.get("from") or attrs.get("start_node")
    end_id = edge.get("end_node") or edge.get("v") or edge.get("to") or attrs.get("end_node")

    start_node = node_by_id.get(str(start_id)) if start_id not in (None, "") else None
    end_node = node_by_id.get(str(end_id)) if end_id not in (None, "") else None
    return _node_point(start_node or {}), _node_point(end_node or {})


def _point_to_output(point: Point) -> Any:
    if rg is None:
        return point
    return rg.Point3d(point[0], point[1], point[2])


def _edge_to_output_curve(edge: Dict[str, Any], node_by_id: Dict[str, Dict[str, Any]]) -> Optional[Any]:
    start, end = _line_start_end(edge, node_by_id)
    if start is None or end is None:
        return None
    if rg is None:
        return {"start": start, "end": end}
    return rg.LineCurve(_point_to_output(start), _point_to_output(end))


def _mesh_geometry_candidates(mesh: Dict[str, Any]) -> List[Any]:
    return [
        mesh.get("mesh"),
        mesh.get("geometry"),
        mesh.get("brep"),
        mesh.get("surface"),
    ]


def _mesh_to_output_geometry(mesh: Dict[str, Any]) -> Optional[Any]:
    vertices = mesh.get("vertices") if isinstance(mesh.get("vertices"), list) else None
    faces = mesh.get("faces") if isinstance(mesh.get("faces"), list) else None

    if rg is not None:
        for candidate in _mesh_geometry_candidates(mesh):
            if candidate is None:
                continue
            if isinstance(candidate, (rg.Mesh, rg.Brep, rg.Surface)):
                return candidate
            if hasattr(candidate, "ToBrep"):
                try:
                    return candidate.ToBrep()
                except Exception:
                    pass

        if vertices and faces:
            rh_mesh = rg.Mesh()
            for vertex in vertices:
                point = _as_point(vertex)
                if point is None:
                    continue
                rh_mesh.Vertices.Add(point[0], point[1], point[2])

            for face in faces:
                if not isinstance(face, (list, tuple)):
                    continue
                indices = [int(value) for value in face[:4]]
                if len(indices) == 3:
                    rh_mesh.Faces.AddFace(indices[0], indices[1], indices[2])
                elif len(indices) == 4:
                    rh_mesh.Faces.AddFace(indices[0], indices[1], indices[2], indices[3])

            if rh_mesh.Vertices.Count > 0 and rh_mesh.Faces.Count > 0:
                rh_mesh.Normals.ComputeNormals()
                rh_mesh.Compact()
                return rh_mesh

    for candidate in _mesh_geometry_candidates(mesh):
        if candidate is not None:
            return candidate

    if vertices and faces:
        return {"vertices": vertices, "faces": faces}

    return None


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


def _coerce_point3_like(value: Any) -> Optional[Point]:
    if value is None:
        return None
    p = _as_point(value)
    if p is not None:
        return p
    for keys in (("X", "Y", "Z"), ("x", "y", "z")):
        if all(hasattr(value, k) for k in keys):
            try:
                return (float(getattr(value, keys[0])), float(getattr(value, keys[1])), float(getattr(value, keys[2])))
            except Exception:
                return None
    return None


def _unwrap_model_candidate(value: Any) -> Any:
    def _extract_tree_items(tree_like: Any) -> List[Any]:
        items: List[Any] = []
        if tree_like is None:
            return items

        # Grasshopper DataTree/GH_Structure patterns.
        if hasattr(tree_like, "DataCount") and hasattr(tree_like, "BranchCount"):
            try:
                branch_count = int(getattr(tree_like, "BranchCount"))
            except Exception:
                branch_count = 0
            if branch_count > 0 and hasattr(tree_like, "Branch"):
                for i in range(branch_count):
                    try:
                        branch = tree_like.Branch(i)
                        items.extend(_iter_sequence_candidate(branch))
                    except Exception:
                        pass
                if items:
                    return items

            if hasattr(tree_like, "AllData"):
                try:
                    data = tree_like.AllData()
                    items.extend(_iter_sequence_candidate(data))
                except Exception:
                    pass
                if items:
                    return items

        if hasattr(tree_like, "Branches"):
            try:
                for branch in _iter_sequence_candidate(getattr(tree_like, "Branches")):
                    items.extend(_iter_sequence_candidate(branch))
            except Exception:
                pass
        return items

    current = value
    for _ in range(6):
        changed = False

        tree_items = _extract_tree_items(current)
        if len(tree_items) == 1:
            current = tree_items[0]
            changed = True
        elif len(tree_items) > 1 and not isinstance(current, (list, tuple)):
            current = tree_items
            changed = True

        if isinstance(current, (list, tuple)) and len(current) == 1:
            current = current[0]
            changed = True

        # GH Goo wrappers often expose the payload as .Value
        if hasattr(current, "Value"):
            try:
                maybe = getattr(current, "Value")
                if maybe is not None and maybe is not current:
                    current = maybe
                    changed = True
            except Exception:
                pass

        # Some wrappers expose ScriptVariable() to provide the runtime value.
        if hasattr(current, "ScriptVariable"):
            try:
                maybe = current.ScriptVariable()
                if maybe is not None and maybe is not current:
                    current = maybe
                    changed = True
            except Exception:
                pass

        if not changed:
            break
    return current


def _iter_sequence_candidate(value: Any) -> List[Any]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return list(value)
    try:
        return list(value)
    except Exception:
        return []


def _coerce_model_to_payload(model_input: Any) -> Tuple[Optional[Dict[str, Any]], str]:
    raw = _unwrap_model_candidate(model_input)

    if isinstance(raw, dict):
        return raw, "dict"

    if isinstance(raw, (list, tuple)) and raw:
        # Common GH tree/list case: one payload object packed in a branch list.
        if len(raw) == 1:
            return _coerce_model_to_payload(raw[0])
        # If a dict payload exists in the list, prefer it.
        for item in raw:
            if isinstance(item, dict):
                return item, "dict-from-list"

    if isinstance(raw, str):
        text = raw.strip()
        if text.startswith("{") and text.endswith("}"):
            try:
                parsed = json.loads(text)
                if isinstance(parsed, dict):
                    return parsed, "json-string"
            except Exception:
                pass

    # Try generic Karamba-like object conversion: nodes + elements/shells.
    node_source = None
    for key in ("nodes", "Nodes"):
        if hasattr(raw, key):
            node_source = getattr(raw, key)
            break

    elem_source = None
    for key in ("elems", "Elems", "elements", "Elements"):
        if hasattr(raw, key):
            elem_source = getattr(raw, key)
            break

    nodes_seq = _iter_sequence_candidate(node_source)
    elems_seq = _iter_sequence_candidate(elem_source)
    if not nodes_seq:
        return None, "unsupported-model-type:{}".format(type(raw).__name__)

    nodes: List[Dict[str, Any]] = []
    node_ids: List[str] = []
    for i, node in enumerate(nodes_seq):
        point = _coerce_point3_like(node)
        if point is None:
            # try common node-position properties
            for pos_key in ("pos", "Pos", "point", "Point"):
                if hasattr(node, pos_key):
                    point = _coerce_point3_like(getattr(node, pos_key))
                    if point is not None:
                        break
        if point is None:
            continue
        node_id = "N{}".format(i + 1)
        node_ids.append(node_id)
        nodes.append({"id": node_id, "x": point[0], "y": point[1], "z": point[2]})

    if not nodes:
        return None, "unsupported-node-layout:{}".format(type(raw).__name__)

    edges: List[Dict[str, Any]] = []
    meshes: List[Dict[str, Any]] = []
    for e_i, elem in enumerate(elems_seq):
        node_inds = None
        for key in ("node_inds", "nodeInds", "NodeInds", "NodeInds", "ind", "Ind"):
            if hasattr(elem, key):
                try:
                    node_inds = list(getattr(elem, key))
                    break
                except Exception:
                    pass
        if not node_inds:
            continue

        try:
            idx = [int(v) for v in node_inds]
        except Exception:
            continue

        # Beam-like: use first and last node as line connectivity.
        if len(idx) >= 2:
            a = idx[0]
            b = idx[-1]
            if 0 <= a < len(node_ids) and 0 <= b < len(node_ids) and a != b:
                edges.append({"id": "E{}".format(len(edges) + 1), "start_node": node_ids[a], "end_node": node_ids[b]})

        # Shell-like: capture polygon face when at least 3 nodes are present.
        if len(idx) >= 3:
            face = [j for j in idx if 0 <= j < len(node_ids)]
            if len(face) >= 3:
                meshes.append(
                    {
                        "id": "M{}".format(len(meshes) + 1),
                        "vertices": [[nodes[j]["x"], nodes[j]["y"], nodes[j]["z"]] for j in face],
                        "faces": [list(range(len(face)))],
                    }
                )

    payload: Dict[str, Any] = {"nodes": nodes, "edges": edges}
    if meshes:
        payload["meshes"] = meshes
    return payload, "coerced-{}".format(type(raw).__name__)


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


def as_output_lists(model: Dict[str, Any]) -> Dict[str, List[Any]]:
    """Return GH-friendly validation and preview geometry lists from the export payload."""
    payload = build_structure_export_json(model)
    lists = payload.get("output_lists", {})
    nodes = _pick_list(payload, "nodes")
    edges = _pick_list(payload, "edges")
    meshes = _pick_list(payload, "meshes")
    node_by_id: Dict[str, Dict[str, Any]] = {str(node.get("id")): node for node in nodes if node.get("id") not in (None, "")}
    boundary_point_ids = [str(node_id) for node_id in list(lists.get("boundary_points", []))]
    point_load_ids = [str(node_id) for node_id in list(lists.get("point_load_points", []))]

    gh_nodes: List[Any] = []
    for node_id in boundary_point_ids:
        node = node_by_id.get(node_id)
        if not isinstance(node, dict):
            continue
        point = _node_point(node)
        if point is not None:
            gh_nodes.append(_point_to_output(point))

    gh_load_points: List[Any] = []
    for node_id in point_load_ids:
        node = node_by_id.get(node_id)
        if not isinstance(node, dict):
            continue
        point = _node_point(node)
        if point is not None:
            gh_load_points.append(_point_to_output(point))

    gh_line_curves: List[Any] = []
    for edge in edges:
        curve = _edge_to_output_curve(edge, node_by_id)
        if curve is not None:
            gh_line_curves.append(curve)

    gh_surfaces: List[Any] = []
    for mesh in meshes:
        geometry = _mesh_to_output_geometry(mesh)
        if geometry is not None:
            gh_surfaces.append(geometry)

    return {
        "MemberLines": list(lists.get("member_lines", [])),
        "AreaLoadMeshes": list(lists.get("area_load_meshes", [])),
        "LinearLoadLines": list(lists.get("linear_load_lines", [])),
        "PointLoadPoints": list(lists.get("point_load_points", [])),
        "BoundaryPoints": list(lists.get("boundary_points", [])),
        "JointNodes": list(lists.get("joint_nodes", [])),
        "GHLineCurves": gh_line_curves,
        "GHNodes": gh_nodes,
        "GHLoadPoints": gh_load_points,
        "GHSurfaces": gh_surfaces,
    }


def _read_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError("Input JSON root must be an object.")
    return data


def _write_json(path: str, payload: Dict[str, Any]) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2)


def _resolve_output_file_path(path: str) -> str:
    cleaned = str(path).replace("\r", "").replace("\n", "").strip().strip('"').strip("'")
    normalized = os.path.abspath(cleaned)
    if normalized.lower().endswith(".json"):
        return normalized
    if os.path.isdir(normalized) or normalized.endswith(("\\", "/")):
        return os.path.join(normalized, "out_model.json")
    return normalized + ".json"


def _coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("", "0", "false", "no", "off", "none", "null"):
            return False
        if text in ("1", "true", "yes", "on"):
            return True
    return bool(value)


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
try:
    if "model" in _g or "Model" in _g:
        _model_input = _g.get("model", _g.get("Model"))
        _save_flag = _coerce_bool(_g.get("save", _g.get("Save", False)))
        _out_path_raw = _g.get("output_path", _g.get("OutputPath", _g.get("file_path", _g.get("FilePath"))))
        _out_path = str(_out_path_raw).strip() if _out_path_raw not in (None, "") else ""
        _payload_input, _payload_source = _coerce_model_to_payload(_model_input)
        _input_type = type(_model_input).__name__

        if isinstance(_payload_input, dict):
            ExportJson = build_structure_export_json(model=_payload_input)
            _lists = as_output_lists(_payload_input)
            MemberLines = _lists["MemberLines"]
            AreaLoadMeshes = _lists["AreaLoadMeshes"]
            LinearLoadLines = _lists["LinearLoadLines"]
            PointLoadPoints = _lists["PointLoadPoints"]
            BoundaryPoints = _lists["BoundaryPoints"]
            JointNodes = _lists["JointNodes"]
            GHLineCurves = _lists["GHLineCurves"]
            GHNodes = _lists["GHNodes"]
            GHLoadPoints = _lists["GHLoadPoints"]
            GHSurfaces = _lists["GHSurfaces"]
            out = (
                "Exported nodes: {}, edges: {}, meshes: {}, member_breps: {}, joints: {}, gh_lines: {}, gh_nodes: {}, gh_load_points: {}, gh_surfaces: {}"
            ).format(
                len(ExportJson.get("nodes", [])),
                len(ExportJson.get("edges", [])),
                len(ExportJson.get("meshes", [])),
                len(ExportJson.get("member_breps", [])),
                len(ExportJson.get("joints", [])),
                len(GHLineCurves),
                len(GHNodes),
                len(GHLoadPoints),
                len(GHSurfaces),
            )
            out += "\nInput type: {}".format(_input_type)
            out += "\nPayload source: {}".format(_payload_source)
            if _save_flag:
                if _out_path:
                    try:
                        _resolved_path = _resolve_output_file_path(_out_path)
                        _write_json(_resolved_path, ExportJson)
                        out += "\nSaved -> {}".format(_resolved_path)
                    except Exception as _e:
                        out += "\nSave FAILED: {}".format(_e)
                else:
                    out += "\nSave triggered - no output_path wired."
            else:
                out += "\nSave not triggered (set save=True or click Button)."
        else:
            out = "Model input could not be coerced to payload (got {}).".format(type(_model_input).__name__)
            if _save_flag:
                out += " Save ignored because export payload was not built."
            if _out_path:
                out += " output_path='{}'".format(_out_path)
            out += " payload_source='{}'".format(_payload_source)
    else:
        out = "No model input detected. Wire Karamba model to 'model' (or 'Model')."
except Exception as _runtime_error:
    out = "Runtime error: {}\n{}".format(_runtime_error, traceback.format_exc())


if __name__ == "__main__" and "model" not in globals() and "Model" not in globals():
    main()
