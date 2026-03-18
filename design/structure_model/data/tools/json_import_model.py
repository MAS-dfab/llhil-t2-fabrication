# Minimal Input Cheat-Sheet (Importer)
# - GH input names: model (or Model)
#   (can be a parsed JSON object/list OR a file path string to JSON)
# - Optional GH preview toggle:
#     preview_geometry (aliases: PreviewGeometry, BuildPreviewGeometry) -> default false
# - Optional GH area auto-mesh toggle (preview path):
#     auto_mesh_areas (aliases: AutoMeshAreas, AutoMeshAreaLoads) -> default true
# - Optional GH proxy / geometry inputs:
#     curve_guid_proxies      (alias: LineGuidProxies)      -> linear load line filtering
#     area_geometry           (aliases: AreaGeometry, AreaMeshes, AreaSurfaces) -> area meshes
#     point_load_guid_proxies (alias: PointLoadGuidProxies) -> filters PointLoadPoints (empty when not connected)
#     boundary_guid_proxies   (alias: BoundaryGuidProxies)  -> filters BoundaryPoints (empty when not connected)
# - preview_kind values and their GH input / output correspondence:
#     members     -> (no extra input)              -> MemberLines
#     linear      -> LineGuidProxies               -> LinearLoadLines
#     point_loads -> point_load_guid_proxies       -> PointLoadPoints
#     boundary    -> boundary_guid_proxies         -> BoundaryPoints
#     joints      -> (no extra input)              -> JointNodes
#     areas       -> area_geometry                 -> AreaLoadMeshes
#     all         -> all of the above              -> PreviewGeometry
#               ("all" shows: members + joints + areas; select specific kinds for
#                point_loads/boundary/linear to avoid redundant all-node point clouds)
# - Function call: import_line_model_json(payload=...,
#     curve_guid_proxies=..., point_load_guid_proxies=..., boundary_guid_proxies=...)
# - Minimal runnable input: {}
# - Minimal useful topology input:
#   {"lines": [{"start": [0, 0, 0], "end": [1, 0, 0]}]}
# - Also accepts COMPAS-style root arrays with entries like:
#   {"line": {"data": {"start": [...], "end": [...]}, "guid": "...", "name": "..."}, "type": "..."}
# - GH preferred outputs (compressed):
#   ImportJson, MemberLines, AreaLoadMeshes, LinearLoadLines,
#   PointLoadPoints, BoundaryPoints, JointNodes, out
# - Optional preview bridge output:
#   PreviewGeometry (enable with preview_geometry=true, set preview_kind)
# - Geometry record outputs (for downstream ID→geometry mapping):
#   Vertices (nodes), Edges (edges), Meshes (mesh records)
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import Rhino.Geometry as rg  # type: ignore
except Exception:  # pragma: no cover - Rhino is not available in CLI environments.
    rg = None


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
        if (
            len(candidate) >= 2
            and ((candidate[0] == '"' and candidate[-1] == '"') or (candidate[0] == "'" and candidate[-1] == "'"))
        ):
            candidate = candidate[1:-1].strip()
        if candidate:
            # Allow direct JSON text passed from a panel.
            if candidate.startswith("{") or candidate.startswith("["):
                try:
                    return json.loads(candidate)
                except Exception:
                    pass

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


def _has_connected_filter_input(value: Any) -> bool:
    """Return True when a GH input is connected with a non-empty value."""
    if value is None:
        return False
    if isinstance(value, str):
        return value.strip() != ""
    if isinstance(value, (list, tuple)):
        return any(_has_connected_filter_input(item) for item in value)
    if isinstance(value, dict):
        return len(value) > 0
    return True


def _has_guid_like_filter_input(value: Any) -> bool:
    """Return True when input can be interpreted as GUID->ID filter data."""
    if value is None:
        return False
    if isinstance(value, str):
        return value.strip() != ""
    if isinstance(value, dict):
        return len(value) > 0
    if isinstance(value, (list, tuple)):
        for item in value:
            if isinstance(item, str) and item.strip() != "":
                return True
            if isinstance(item, dict) and len(item) > 0:
                return True
        return False
    return False


def _resolve_edge_ids_from_rhino_curves(
    value: Any,
    edge_records: List[Dict[str, Any]],
    nodes: List[Dict[str, Any]],
    *,
    tol: float = 1e-3,
) -> Optional[List[str]]:
    """Match Rhino Curve objects to edge IDs using distance-based endpoint matching.

    For each input curve, finds the nearest model node within *tol* distance for
    both endpoints, then looks up the edge connecting those two nodes.
    Returns None when value is None or contains no Rhino Curve instances.
    """
    if rg is None or value is None:
        return None

    items = list(value) if isinstance(value, (list, tuple)) else [value]
    curve_items: List[Any] = []
    for item in items:
        curve = _try_coerce_rhino_curve(item)
        if curve is not None:
            curve_items.append(curve)
    if not curve_items:
        return None

    # Flat list of (x, y, z, node_id) for nearest-neighbour search.
    node_coords: List[Tuple[float, float, float, str]] = []
    for node in nodes:
        xyz = _point_from_node_record(node)
        if xyz is not None:
            node_coords.append((xyz[0], xyz[1], xyz[2], str(node.get("id", ""))))

    if not node_coords:
        return None

    tol2 = tol * tol

    def _nearest_node_id(px: float, py: float, pz: float) -> Optional[str]:
        best_id: Optional[str] = None
        best_d2 = tol2
        for nx, ny, nz, nid in node_coords:
            d2 = (nx - px) ** 2 + (ny - py) ** 2 + (nz - pz) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_id = nid
        return best_id

    # Bidirectional lookup: (start_node_id, end_node_id) -> edge_id.
    edge_lookup: Dict[Tuple[str, str], str] = {}
    for edge in edge_records:
        sn = str(edge.get("start_node", ""))
        en = str(edge.get("end_node", ""))
        eid = str(edge.get("id", ""))
        if sn and en and eid:
            edge_lookup[(sn, en)] = eid
            edge_lookup[(en, sn)] = eid

    matched: List[str] = []
    seen: set = set()

    # First pass: exact node-to-node matching (fast path for simple line proxies).
    for crv in curve_items:
        ps = crv.PointAtStart
        pe = crv.PointAtEnd
        sn_id = _nearest_node_id(float(ps.X), float(ps.Y), float(ps.Z))
        en_id = _nearest_node_id(float(pe.X), float(pe.Y), float(pe.Z))
        if sn_id is None or en_id is None or sn_id == en_id:
            continue
        eid = edge_lookup.get((sn_id, en_id))
        if eid is not None and eid not in seen:
            matched.append(eid)
            seen.add(eid)

    # Second pass: perimeter/polycurve matching.
    # Include any edge whose start/end nodes lie on at least one input proxy curve.
    n_index = _node_index(nodes)
    for edge in edge_records:
        eid = str(edge.get("id", ""))
        if not eid or eid in seen:
            continue
        sn = n_index.get(str(edge.get("start_node", "")))
        en = n_index.get(str(edge.get("end_node", "")))
        if not sn or not en:
            continue
        a = _point_from_node_record(sn)
        b = _point_from_node_record(en)
        if a is None or b is None:
            continue

        pa = rg.Point3d(a[0], a[1], a[2])
        pb = rg.Point3d(b[0], b[1], b[2])
        for crv in curve_items:
            ok_a, ta = crv.ClosestPoint(pa)
            if not ok_a:
                continue
            qa = crv.PointAt(ta)
            if qa.DistanceTo(pa) > tol:
                continue

            ok_b, tb = crv.ClosestPoint(pb)
            if not ok_b:
                continue
            qb = crv.PointAt(tb)
            if qb.DistanceTo(pb) > tol:
                continue

            # Guard against both endpoints collapsing to the same curve parameter.
            if abs(float(tb) - float(ta)) <= 1e-9 and pa.DistanceTo(pb) > tol:
                continue

            matched.append(eid)
            seen.add(eid)
            break

    return matched if matched else None


def _resolve_node_ids_from_rhino_points(
    value: Any,
    nodes: List[Dict[str, Any]],
    *,
    tol: float = 1e-3,
) -> Optional[List[str]]:
    """Match Rhino Point3d objects to node IDs using distance-based nearest-neighbour search.

    Returns None when value is None or contains no Rhino Point3d instances.
    """
    if rg is None or value is None:
        return None

    items = list(value) if isinstance(value, (list, tuple)) else [value]
    point_items: List[Any] = []
    for item in items:
        if isinstance(item, rg.Point3d):
            point_items.append(item)
        elif isinstance(item, rg.Point):
            point_items.append(item.Location)
    if not point_items:
        return None

    node_coords: List[Tuple[float, float, float, str]] = []
    for node in nodes:
        xyz = _point_from_node_record(node)
        if xyz is not None:
            node_coords.append((xyz[0], xyz[1], xyz[2], str(node.get("id", ""))))

    if not node_coords:
        return None

    tol2 = tol * tol
    matched: List[str] = []
    seen: set = set()
    for pt in point_items:
        px, py, pz = float(pt.X), float(pt.Y), float(pt.Z)
        best_id: Optional[str] = None
        best_d2 = tol2
        for nx, ny, nz, nid in node_coords:
            d2 = (nx - px) ** 2 + (ny - py) ** 2 + (nz - pz) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_id = nid
        if best_id is not None and best_id not in seen:
            matched.append(best_id)
            seen.add(best_id)
    return matched if matched else None


def _build_linear_edges_from_curve_proxies(
    value: Any,
    nodes: List[Dict[str, Any]],
    edge_records: List[Dict[str, Any]],
    *,
    decimals: int = 6,
    tol: float = 1e-3,
) -> Optional[List[str]]:
    """Create/update linear edge records from proxy-curve segmentation.

    Returns None when no usable curve proxies are present.
    Returns [] when proxies are present but no segments can be resolved to nodes.
    """
    if rg is None or value is None:
        return None

    items = list(value) if isinstance(value, (list, tuple)) else [value]
    curve_items: List[Any] = []
    for item in items:
        curve = _try_coerce_rhino_curve(item)
        if curve is not None:
            curve_items.append(curve)
    if not curve_items:
        return None

    tol2 = tol * tol

    node_coords: List[Tuple[float, float, float, str]] = []
    for node in nodes:
        xyz = _point_from_node_record(node)
        nid = str(node.get("id", ""))
        if xyz is not None and nid:
            node_coords.append((xyz[0], xyz[1], xyz[2], nid))

    if not node_coords:
        return []

    def _nearest_node_id(px: float, py: float, pz: float) -> Optional[str]:
        best_id: Optional[str] = None
        best_d2 = tol2
        for nx, ny, nz, nid in node_coords:
            d2 = (nx - px) ** 2 + (ny - py) ** 2 + (nz - pz) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_id = nid
        return best_id

    pair_to_edge: Dict[Tuple[str, str], str] = {}
    existing_ids: set = set()
    for edge in edge_records:
        eid = str(edge.get("id", ""))
        sn = str(edge.get("start_node", ""))
        en = str(edge.get("end_node", ""))
        if not eid or not sn or not en:
            continue
        pair_to_edge[tuple(sorted((sn, en)))] = eid
        existing_ids.add(eid)

    def _new_edge_id() -> str:
        idx = 1
        while True:
            candidate = "LP{}".format(idx)
            if candidate not in existing_ids:
                existing_ids.add(candidate)
                return candidate
            idx += 1

    linear_ids: List[str] = []
    seen_linear: set = set()
    for crv in curve_items:
        for a, b in _segment_curve_by_neighbor_nodes(crv, nodes, decimals=decimals, snap_tol=tol):
            sn = _nearest_node_id(a[0], a[1], a[2])
            en = _nearest_node_id(b[0], b[1], b[2])
            if sn is None or en is None or sn == en:
                continue

            pair = tuple(sorted((sn, en)))
            eid = pair_to_edge.get(pair)
            if eid is None:
                eid = _new_edge_id()
                pair_to_edge[pair] = eid
                edge_records.append(
                    {
                        "id": eid,
                        "start_node": sn,
                        "end_node": en,
                        "attributes": {
                            "generated": "linear_proxy_segment",
                        },
                    }
                )

            if eid not in seen_linear:
                linear_ids.append(eid)
                seen_linear.add(eid)

    return linear_ids


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


def _point_from_node_record(node: Dict[str, Any]) -> Optional[Point]:
    try:
        return (float(node.get("x", 0.0)), float(node.get("y", 0.0)), float(node.get("z", 0.0)))
    except (TypeError, ValueError):
        return None


def _points_are_close(a: Point, b: Point, tol: float) -> bool:
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol and abs(a[2] - b[2]) <= tol


def _dedupe_sorted_scalars(values: List[float], tol: float) -> List[float]:
    if not values:
        return []

    ordered = sorted(float(value) for value in values)
    unique = [ordered[0]]
    for value in ordered[1:]:
        if abs(value - unique[-1]) > tol:
            unique.append(value)
    return unique


def _face_relation_is_exterior(relation: Any) -> bool:
    if relation is None:
        return True

    text = str(relation).lower()
    if "exterior" in text:
        return True
    if "interior" in text or "inside" in text or "boundary" in text:
        return False

    try:
        return int(relation) == 0
    except Exception:
        return False


def _closest_face_uv(face: Any, point: Any, *, tol: float) -> Optional[Tuple[float, float]]:
    try:
        closest = face.ClosestPoint(point)
    except Exception:
        return None

    if not isinstance(closest, tuple):
        return None

    ok = True
    if len(closest) >= 3 and isinstance(closest[0], bool):
        ok = bool(closest[0])
        if not ok:
            return None
        u = float(closest[1])
        v = float(closest[2])
    elif len(closest) >= 2:
        u = float(closest[0])
        v = float(closest[1])
    else:
        return None

    projected = face.PointAt(u, v)
    if projected.DistanceTo(point) > tol:
        return None
    if _face_relation_is_exterior(face.IsPointOnFace(u, v)):
        return None
    return (u, v)


def _collect_face_seed_parameters(face: Any, nodes: List[Dict[str, Any]], *, tol: float) -> List[Tuple[float, float]]:
    if rg is None:
        return []

    seeds: List[Tuple[float, float]] = []
    face_brep = None
    try:
        face_brep = face.DuplicateFace(False)
    except Exception:
        face_brep = None

    if face_brep is not None:
        for vertex in face_brep.Vertices:
            uv = _closest_face_uv(face, vertex.Location, tol=tol)
            if uv is not None:
                seeds.append(uv)

    for node in nodes:
        xyz = _point_from_node_record(node)
        if xyz is None:
            continue
        uv = _closest_face_uv(face, rg.Point3d(xyz[0], xyz[1], xyz[2]), tol=tol)
        if uv is not None:
            seeds.append(uv)

    return seeds


def _build_node_aware_mesh_for_face(face: Any, nodes: List[Dict[str, Any]], *, tol: float) -> Optional[Any]:
    if rg is None:
        return None

    seeds = _collect_face_seed_parameters(face, nodes, tol=tol)
    if not seeds:
        return None

    uv_tol = 1e-9
    u_values = _dedupe_sorted_scalars([u for u, _ in seeds], uv_tol)
    v_values = _dedupe_sorted_scalars([v for _, v in seeds], uv_tol)
    if len(u_values) < 2 or len(v_values) < 2:
        return None

    mesh = rg.Mesh()
    vertex_index_by_key: Dict[Tuple[float, float, float], int] = {}

    def _vertex_index(u: float, v: float) -> Optional[int]:
        if _face_relation_is_exterior(face.IsPointOnFace(u, v)):
            return None

        point = face.PointAt(u, v)
        key = (round(float(point.X), 9), round(float(point.Y), 9), round(float(point.Z), 9))
        existing = vertex_index_by_key.get(key)
        if existing is not None:
            return existing

        index = mesh.Vertices.Add(float(point.X), float(point.Y), float(point.Z))
        vertex_index_by_key[key] = index
        return index

    for ui in range(len(u_values) - 1):
        u0 = u_values[ui]
        u1 = u_values[ui + 1]
        if abs(u1 - u0) <= uv_tol:
            continue

        for vi in range(len(v_values) - 1):
            v0 = v_values[vi]
            v1 = v_values[vi + 1]
            if abs(v1 - v0) <= uv_tol:
                continue

            center_u = 0.5 * (u0 + u1)
            center_v = 0.5 * (v0 + v1)
            if _face_relation_is_exterior(face.IsPointOnFace(center_u, center_v)):
                continue

            indices: List[int] = []
            for corner_u, corner_v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1)):
                index = _vertex_index(corner_u, corner_v)
                if index is None:
                    indices = []
                    break
                indices.append(index)

            if not indices:
                continue

            ordered_unique: List[int] = []
            for index in indices:
                if index not in ordered_unique:
                    ordered_unique.append(index)

            if len(ordered_unique) == 3:
                mesh.Faces.AddFace(ordered_unique[0], ordered_unique[1], ordered_unique[2])
            elif len(ordered_unique) == 4:
                mesh.Faces.AddFace(ordered_unique[0], ordered_unique[1], ordered_unique[2], ordered_unique[3])

    if mesh.Vertices.Count == 0 or mesh.Faces.Count == 0:
        return None

    mesh.Normals.ComputeNormals()
    mesh.Compact()
    return mesh


def _build_node_aware_meshes_from_brep(
    brep: Any,
    nodes: Optional[List[Dict[str, Any]]],
    *,
    tol: float,
) -> List[Any]:
    if rg is None or brep is None or not nodes:
        return []

    meshes: List[Any] = []
    for face in brep.Faces:
        mesh = _build_node_aware_mesh_for_face(face, nodes, tol=tol)
        if mesh is not None:
            meshes.append(mesh)
    return meshes


def _try_coerce_rhino_geometry(value: Any) -> Optional[Any]:
    if rg is None or value is None:
        return None

    # Direct RhinoCommon geometry object (GH object input path).
    if isinstance(value, (rg.Mesh, rg.Brep, rg.Surface)):
        return value

    # Optional: decode simple JSON geometry when available.
    if isinstance(value, dict):
        encoded = value.get("rhino") or value.get("rhino_json") or value.get("rhinoJson")
        if isinstance(encoded, str) and encoded:
            try:
                return rg.GeometryBase.FromJSON(encoded)
            except Exception:
                return None

    return None


def _try_coerce_rhino_curve(value: Any) -> Optional[Any]:
    if rg is None or value is None:
        return None

    # GH wrapper objects often expose underlying Rhino geometry via Value/ScriptVariable.
    wrapped = getattr(value, "Value", None)
    if wrapped is not None and wrapped is not value:
        coerced = _try_coerce_rhino_curve(wrapped)
        if coerced is not None:
            return coerced

    script_var = getattr(value, "ScriptVariable", None)
    if callable(script_var):
        try:
            sv = script_var()
        except Exception:
            sv = None
        if sv is not None and sv is not value:
            coerced = _try_coerce_rhino_curve(sv)
            if coerced is not None:
                return coerced

    if isinstance(value, rg.Curve):
        return value
    if isinstance(value, rg.Line):
        return rg.LineCurve(value)

    geo = _try_coerce_rhino_geometry(value)
    if geo is None:
        return None
    if isinstance(geo, rg.Curve):
        return geo
    if isinstance(geo, rg.Line):
        return rg.LineCurve(geo)
    return None


def _extract_curve_from_edge(raw_edge: Dict[str, Any]) -> Optional[Any]:
    attrs = raw_edge.get("attributes") if isinstance(raw_edge.get("attributes"), dict) else {}
    candidates = [
        raw_edge.get("curve"),
        raw_edge.get("line"),
        raw_edge.get("geometry"),
        raw_edge.get("rhino_curve"),
        attrs.get("curve"),
        attrs.get("line"),
        attrs.get("geometry"),
        attrs.get("rhino_curve"),
    ]

    for candidate in candidates:
        curve = _try_coerce_rhino_curve(candidate)
        if curve is not None:
            return curve
    return None


def _segment_curve_by_neighbor_nodes(
    curve: Any,
    nodes: List[Dict[str, Any]],
    *,
    decimals: int,
    snap_tol: Optional[float] = None,
) -> List[Tuple[Point, Point]]:
    if rg is None:
        return []

    dedup_tol = max(10.0 ** (-decimals), 1e-6)
    _snap_tol = snap_tol if snap_tol is not None else dedup_tol
    t0 = float(curve.Domain.T0)
    t1 = float(curve.Domain.T1)
    start = curve.PointAtStart
    end = curve.PointAtEnd

    candidates: List[Tuple[float, Point]] = [
        (t0, (float(start.X), float(start.Y), float(start.Z))),
        (t1, (float(end.X), float(end.Y), float(end.Z))),
    ]

    for node in nodes:
        xyz = _point_from_node_record(node)
        if xyz is None:
            continue

        test_pt = rg.Point3d(xyz[0], xyz[1], xyz[2])
        ok, t = curve.ClosestPoint(test_pt)
        if not ok:
            continue

        on_curve = curve.PointAt(t)
        if on_curve.DistanceTo(test_pt) <= _snap_tol:
            candidates.append((float(t), xyz))

    # Sort by curve parameter and remove near-duplicates.
    candidates.sort(key=lambda item: item[0])
    ordered: List[Point] = []
    last_t: Optional[float] = None
    for t, xyz in candidates:
        if last_t is not None and abs(t - last_t) <= dedup_tol:
            continue
        if ordered:
            prev = ordered[-1]
            if abs(prev[0] - xyz[0]) <= dedup_tol and abs(prev[1] - xyz[1]) <= dedup_tol and abs(prev[2] - xyz[2]) <= dedup_tol:
                last_t = t
                continue
        ordered.append(xyz)
        last_t = t

    segments: List[Tuple[Point, Point]] = []
    for idx in range(len(ordered) - 1):
        a = ordered[idx]
        b = ordered[idx + 1]
        if abs(a[0] - b[0]) <= dedup_tol and abs(a[1] - b[1]) <= dedup_tol and abs(a[2] - b[2]) <= dedup_tol:
            continue
        segments.append((a, b))
    return segments


def _meshes_from_area_payload(
    raw_area: Dict[str, Any],
    *,
    nodes: Optional[List[Dict[str, Any]]] = None,
    tol: float = 1e-3,
) -> List[Any]:
    if rg is None:
        return []

    sources = [
        raw_area.get("mesh"),
        raw_area.get("geometry"),
        raw_area.get("brep"),
        raw_area.get("surface"),
    ]

    out_meshes: List[Any] = []
    for source in sources:
        geo = _try_coerce_rhino_geometry(source)
        if geo is None:
            continue

        if isinstance(geo, rg.Mesh):
            out_meshes.append(geo)
            continue

        brep = None
        if isinstance(geo, rg.Brep):
            brep = geo
        elif isinstance(geo, rg.Surface):
            try:
                brep = geo.ToBrep()
            except Exception:
                brep = None

        if brep is None:
            continue

        seeded_meshes = _build_node_aware_meshes_from_brep(brep, nodes, tol=tol)
        if seeded_meshes:
            out_meshes.extend(seeded_meshes)
            continue

        try:
            parts = rg.Mesh.CreateFromBrep(brep, rg.MeshingParameters.FastRenderMesh)
        except Exception:
            parts = None

        if not parts:
            continue

        for part in parts:
            if part is not None and part.Vertices.Count > 0 and part.Faces.Count > 0:
                out_meshes.append(part)

    return out_meshes


def _mesh_record_to_rhino_meshes(
    raw_mesh: Dict[str, Any],
    *,
    auto_mesh_areas: bool,
    nodes: Optional[List[Dict[str, Any]]] = None,
) -> List[Any]:
    if rg is None:
        return []

    vertices = raw_mesh.get("vertices") if isinstance(raw_mesh.get("vertices"), list) else None
    faces = raw_mesh.get("faces") if isinstance(raw_mesh.get("faces"), list) else None
    if vertices and faces:
        rh_mesh = rg.Mesh()
        valid = True
        for vertex in vertices:
            pt = _as_point(vertex)
            if pt is None:
                valid = False
                break
            rh_mesh.Vertices.Add(pt[0], pt[1], pt[2])
        if not valid:
            return []

        for face in faces:
            if not isinstance(face, (list, tuple)):
                continue
            try:
                idx = [int(i) for i in face]
            except (TypeError, ValueError):
                continue
            if len(idx) == 3:
                rh_mesh.Faces.AddFace(idx[0], idx[1], idx[2])
            elif len(idx) >= 4:
                rh_mesh.Faces.AddFace(idx[0], idx[1], idx[2], idx[3])

        if rh_mesh.Vertices.Count > 0 and rh_mesh.Faces.Count > 0:
            rh_mesh.Normals.ComputeNormals()
            rh_mesh.Compact()
            return [rh_mesh]
        return []

    if auto_mesh_areas:
        return _meshes_from_area_payload(raw_mesh, nodes=nodes)
    return []


def _build_area_mesh_geometry(
    meshes: List[Dict[str, Any]],
    *,
    auto_mesh_areas: bool = True,
    nodes: Optional[List[Dict[str, Any]]] = None,
) -> List[Any]:
    mesh_geometry: List[Any] = []
    for mesh in meshes:
        mesh_geometry.extend(_mesh_record_to_rhino_meshes(mesh, auto_mesh_areas=auto_mesh_areas, nodes=nodes))
    return mesh_geometry


def _build_area_mesh_geometry_from_input(
    value: Any,
    *,
    auto_mesh_areas: bool = True,
    nodes: Optional[List[Dict[str, Any]]] = None,
) -> List[Any]:
    """Convert direct GH area geometry input into Rhino meshes."""
    if value is None:
        return []

    if isinstance(value, (list, tuple)):
        items = list(value)
    else:
        items = [value]

    mesh_records: List[Dict[str, Any]] = []
    for item in items:
        if item is None:
            continue

        if isinstance(item, dict):
            record: Dict[str, Any] = {}
            for key in ("vertices", "faces", "mesh", "geometry", "brep", "surface"):
                if key in item and item.get(key) is not None:
                    record[key] = item.get(key)
            if record:
                mesh_records.append(record)
            continue

        geo = _try_coerce_rhino_geometry(item)
        if geo is None:
            continue

        if rg is not None and isinstance(geo, rg.Mesh):
            mesh_records.append({"mesh": geo})
        elif rg is not None and isinstance(geo, rg.Brep):
            mesh_records.append({"brep": geo})
        elif rg is not None and isinstance(geo, rg.Surface):
            mesh_records.append({"surface": geo})

    return _build_area_mesh_geometry(mesh_records, auto_mesh_areas=auto_mesh_areas, nodes=nodes)


def _node_index(nodes: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {str(node.get("id")): node for node in nodes if node.get("id") not in (None, "")}


def _edge_index(edges: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {str(edge.get("id")): edge for edge in edges if edge.get("id") not in (None, "")}


def _point_geometry_from_node_ids(node_ids: List[str], nodes: List[Dict[str, Any]]) -> List[Any]:
    if rg is None:
        return []

    index = _node_index(nodes)
    out: List[Any] = []
    for node_id in node_ids:
        node = index.get(str(node_id))
        if not node:
            continue
        xyz = _point_from_node_record(node)
        if xyz is None:
            continue
        out.append(rg.Point3d(xyz[0], xyz[1], xyz[2]))
    return out


def _dedupe_node_ids_by_position(
    node_ids: List[str],
    nodes: List[Dict[str, Any]],
    *,
    tol: float = 1e-6,
) -> List[str]:
    """Return node IDs deduplicated by spatial position.

    This avoids creating overlapping supports/loads when multiple node IDs share
    the same coordinates.
    """
    if not node_ids:
        return []

    index = _node_index(nodes)
    unique_ids: List[str] = []
    unique_xyz: List[Point] = []

    for node_id in node_ids:
        node = index.get(str(node_id))
        if not node:
            continue
        xyz = _point_from_node_record(node)
        if xyz is None:
            continue

        duplicate = False
        for ux, uy, uz in unique_xyz:
            if abs(xyz[0] - ux) <= tol and abs(xyz[1] - uy) <= tol and abs(xyz[2] - uz) <= tol:
                duplicate = True
                break
        if duplicate:
            continue

        unique_ids.append(str(node_id))
        unique_xyz.append(xyz)

    return unique_ids


def _line_geometry_from_edge_ids(edge_ids: List[str], edges: List[Dict[str, Any]], nodes: List[Dict[str, Any]]) -> List[Any]:
    if rg is None:
        return []

    n_index = _node_index(nodes)
    e_index = _edge_index(edges)
    out: List[Any] = []

    for edge_id in edge_ids:
        edge = e_index.get(str(edge_id))
        if not edge:
            continue

        start_node = n_index.get(str(edge.get("start_node", "")))
        end_node = n_index.get(str(edge.get("end_node", "")))
        if not start_node or not end_node:
            continue

        a = _point_from_node_record(start_node)
        b = _point_from_node_record(end_node)
        if a is None or b is None:
            continue
        out.append(rg.LineCurve(rg.Point3d(a[0], a[1], a[2]), rg.Point3d(b[0], b[1], b[2])))

    return out


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
    node_merge_tol = max(10.0 ** (-decimals), 1e-5)

    def ensure_node_id(point: Point) -> str:
        # First pass: tolerant weld to collapse near-coincident endpoints to a shared node ID.
        for node in node_records:
            xyz = _point_from_node_record(node)
            if xyz is not None and _points_are_close(point, xyz, node_merge_tol):
                return str(node["id"])

        # Second pass: exact rounded-key lookup for deterministic IDs.
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
    segmented_edge_ids: List[str] = []
    seen_edge_pairs: Dict[Tuple[str, str], str] = {}
    curve_guid_proxy: Dict[str, str] = dict(input_proxies["curve"])

    for raw_edge in edges:
        attrs = raw_edge.get("attributes") if isinstance(raw_edge.get("attributes"), dict) else {}
        from_curve_segmentation = False

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

        segment_node_pairs: List[Tuple[str, str]] = []

        if start_id is not None and end_id is not None and start_id != end_id:
            segment_node_pairs.append((start_id, end_id))
        else:
            start_point, end_point = _line_start_end(raw_edge)

            if (start_point is None or end_point is None) and rg is not None:
                curve = _extract_curve_from_edge(raw_edge)
                if curve is not None:
                    curve_segments = _segment_curve_by_neighbor_nodes(curve, node_records, decimals=decimals)
                    if len(curve_segments) > 1:
                        from_curve_segmentation = True
                    for seg_start, seg_end in curve_segments:
                        seg_start_id = ensure_node_id(seg_start)
                        seg_end_id = ensure_node_id(seg_end)
                        if seg_start_id != seg_end_id:
                            segment_node_pairs.append((seg_start_id, seg_end_id))

            if not segment_node_pairs:
                if start_id is None and start_point is not None:
                    start_id = ensure_node_id(start_point)
                if end_id is None and end_point is not None:
                    end_id = ensure_node_id(end_point)
                if start_id is not None and end_id is not None and start_id != end_id:
                    segment_node_pairs.append((start_id, end_id))

        if not segment_node_pairs:
            continue

        preserved_attrs = {k: v for k, v in attrs.items() if k not in ("start", "end", "start_node", "end_node")}

        for seg_start_id, seg_end_id in segment_node_pairs:
            pair = tuple(sorted((seg_start_id, seg_end_id)))
            if pair in seen_edge_pairs:
                continue

            edge_id = f"{edge_prefix}{len(edge_records) + 1}"
            seen_edge_pairs[pair] = edge_id

            edge_record = {
                "id": edge_id,
                "start_node": seg_start_id,
                "end_node": seg_end_id,
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
            if from_curve_segmentation:
                segmented_edge_ids.append(edge_id)

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

        for key in ("vertices", "faces", "mesh", "geometry", "brep", "surface"):
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

    _curve_filter_connected = _has_guid_like_filter_input(curve_guid_proxies)
    _pl_filter_connected = _has_guid_like_filter_input(point_load_guid_proxies)
    _bp_filter_connected = _has_guid_like_filter_input(boundary_guid_proxies)

    # Resolve explicitly tagged GUID inputs to filtered node ID lists.
    # Point-load/boundary lists are opt-in and remain empty when no proxy input is provided.
    _pl_ids = _resolve_filtered_node_ids(point_load_guid_proxies, point_guid_proxy)
    _bp_ids = _resolve_filtered_node_ids(boundary_guid_proxies, point_guid_proxy)
    # Filter linear load lines by curve GUID proxies when provided; falls back to all member edges.
    _ll_ids = _resolve_filtered_node_ids(curve_guid_proxies, curve_guid_proxy)

    _linear_output_ids = _ll_ids if _curve_filter_connected else list(member_line_ids)
    if _curve_filter_connected and _linear_output_ids is None:
        _linear_output_ids = []

    _point_output_ids = _pl_ids if _pl_filter_connected else []
    if _pl_filter_connected and _point_output_ids is None:
        _point_output_ids = []
    _point_output_ids = _dedupe_node_ids_by_position(
        list(_point_output_ids), node_records, tol=node_merge_tol
    )

    _boundary_output_ids = _bp_ids if _bp_filter_connected else []
    if _bp_filter_connected and _boundary_output_ids is None:
        _boundary_output_ids = []

    # Boundary supports should be unique by position, not only by node ID.
    _boundary_output_ids = _dedupe_node_ids_by_position(
        list(_boundary_output_ids), node_records, tol=node_merge_tol
    )

    _joint_output_ids = _dedupe_node_ids_by_position(
        [joint["node_id"] for joint in joint_records], node_records, tol=node_merge_tol
    )

    return {
        "schema": "structure_model_v1",
        "nodes": node_records,
        "edges": edge_records,
        "meshes": mesh_records,
        "joints": joint_records,
        "output_lists": {
            "member_lines": member_line_ids,
            "area_load_meshes": area_mesh_ids,
            "linear_load_lines": list(_linear_output_ids),
            "segmented_linear_load_lines": list(segmented_edge_ids),
            "point_load_points": list(_point_output_ids),
            "boundary_points": list(_boundary_output_ids),
            "joint_nodes": list(_joint_output_ids),
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
            "segmented_edge_count": len(segmented_edge_ids),
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
        "SegmentedLinearLoadLines": list(lists.get("segmented_linear_load_lines", [])),
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

# Preferred compact list outputs.
ImportJson = "{}"
MemberLines = []
AreaLoadMeshes = []
LinearLoadLines = []
PointLoadPoints = []
BoundaryPoints = []
JointNodes = []
out = "Waiting for Model input."

# Geometry record outputs (for downstream ID→geometry mapping).
Vertices = []
Edges = []
Meshes = []
PreviewGeometry = []


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
            _curve_proxies = _get_first_input(_g, ["curve_guid_proxies", "CurveGuidProxies", "LineGuidProxies"])
            _pl_proxies = _get_first_input(_g, ["point_load_guid_proxies", "PointLoadGuidProxies"])
            _bp_proxies = _get_first_input(_g, ["boundary_guid_proxies", "BoundaryGuidProxies"])
            _preview_enabled = _to_bool(
                _get_first_input(_g, ["preview_geometry", "PreviewGeometry", "BuildPreviewGeometry"]),
                default=False,
            )
            _preview_kind_raw = _get_first_input(
                _g,
                ["preview_kind", "PreviewKind", "PreviewTarget", "PreviewSource"],
            )
            _preview_kind = str(_preview_kind_raw).strip().lower() if _preview_kind_raw not in (None, "") else "members"
            _auto_mesh_areas = _to_bool(
                _get_first_input(_g, ["auto_mesh_areas", "AutoMeshAreas", "AutoMeshAreaLoads"]),
                default=True,
            )
            _area_geometry_input = _get_first_input(_g, ["area_geometry", "AreaGeometry", "AreaMeshes", "AreaSurfaces"])

            _import_payload = import_line_model_json(
                _model_input,
                curve_guid_proxies=_curve_proxies,
                point_load_guid_proxies=_pl_proxies,
                boundary_guid_proxies=_bp_proxies,
            )
            _summary_payload = {
                "schema": _import_payload.get("schema", "structure_model_v1"),
                "metadata": dict(_import_payload.get("metadata", {})),
                "output_list_counts": {
                    "member_lines": len(_import_payload.get("output_lists", {}).get("member_lines", [])),
                    "area_load_meshes": len(_import_payload.get("output_lists", {}).get("area_load_meshes", [])),
                    "linear_load_lines": len(_import_payload.get("output_lists", {}).get("linear_load_lines", [])),
                    "point_load_points": len(_import_payload.get("output_lists", {}).get("point_load_points", [])),
                    "boundary_points": len(_import_payload.get("output_lists", {}).get("boundary_points", [])),
                    "joint_nodes": len(_import_payload.get("output_lists", {}).get("joint_nodes", [])),
                },
            }
            ImportJson = json.dumps(_summary_payload, separators=(",", ":"))

            # Compressed list-first outputs.
            _lists = _import_payload.get("output_lists", {})
            _member_line_ids = list(_lists.get("member_lines", []))
            AreaLoadMeshes = list(_lists.get("area_load_meshes", []))
            _segmented_linear_ids = list(_lists.get("segmented_linear_load_lines", []))
            _linear_line_ids = _segmented_linear_ids if _segmented_linear_ids else list(_lists.get("linear_load_lines", []))
            _point_node_ids = list(_lists.get("point_load_points", []))
            _boundary_node_ids = list(_lists.get("boundary_points", []))
            _joint_node_ids = list(_lists.get("joint_nodes", []))

            # Default outputs (CLI/no-Rhino fallback): ID lists.
            MemberLines = list(_member_line_ids)
            LinearLoadLines = list(_linear_line_ids)
            PointLoadPoints = list(_point_node_ids)
            BoundaryPoints = list(_boundary_node_ids)
            JointNodes = list(_joint_node_ids)

            # Raw geometry records for downstream ID→geometry mapping.
            _nodes = list(_import_payload.get("nodes", []))
            _edges = list(_import_payload.get("edges", []))
            _meshes = list(_import_payload.get("meshes", []))
            Vertices = _nodes
            Edges = _edges
            Meshes = _meshes
            PreviewGeometry = []

            # Ensure solver-facing point lists are unique per spatial location.
            _runtime_dedupe_tol = 1e-5
            _point_node_ids = _dedupe_node_ids_by_position(_point_node_ids, _nodes, tol=_runtime_dedupe_tol)
            _boundary_node_ids = _dedupe_node_ids_by_position(_boundary_node_ids, _nodes, tol=_runtime_dedupe_tol)
            _joint_node_ids = _dedupe_node_ids_by_position(_joint_node_ids, _nodes, tol=_runtime_dedupe_tol)

            # Keep fallback ID outputs synchronized with deduplicated IDs.
            PointLoadPoints = list(_point_node_ids)
            BoundaryPoints = list(_boundary_node_ids)
            JointNodes = list(_joint_node_ids)

            # Geometry-based proxy resolution: when GH curve proxies are connected,
            # generate perimeter linear edges by segmenting those curves against model nodes.
            if rg is not None:
                _generated_linear_ids = _build_linear_edges_from_curve_proxies(
                    _curve_proxies, _nodes, _edges, decimals=6
                )
                if _generated_linear_ids is not None:
                    _linear_line_ids = _generated_linear_ids
                elif _has_guid_like_filter_input(_curve_proxies):
                    # Connected GUID-like filter with no matches should not fall back to all members.
                    _linear_line_ids = []

                _rhino_pl = _resolve_node_ids_from_rhino_points(_pl_proxies, _nodes)
                if _rhino_pl is not None:
                    _point_node_ids = _rhino_pl
                _rhino_bp = _resolve_node_ids_from_rhino_points(_bp_proxies, _nodes)
                if _rhino_bp is not None:
                    _boundary_node_ids = _rhino_bp

                # Re-apply positional deduplication after Rhino-proxy overrides.
                _point_node_ids = _dedupe_node_ids_by_position(_point_node_ids, _nodes, tol=_runtime_dedupe_tol)
                _boundary_node_ids = _dedupe_node_ids_by_position(_boundary_node_ids, _nodes, tol=_runtime_dedupe_tol)
                _joint_node_ids = _dedupe_node_ids_by_position(_joint_node_ids, _nodes, tol=_runtime_dedupe_tol)

                # GH-facing outputs should be geometry for direct Karamba compatibility.
                MemberLines = _line_geometry_from_edge_ids(_member_line_ids, _edges, _nodes)
                LinearLoadLines = _line_geometry_from_edge_ids(_linear_line_ids, _edges, _nodes)
                PointLoadPoints = _point_geometry_from_node_ids(_point_node_ids, _nodes)
                BoundaryPoints = _point_geometry_from_node_ids(_boundary_node_ids, _nodes)
                JointNodes = _point_geometry_from_node_ids(_joint_node_ids, _nodes)

            # Publish potentially augmented edges (includes generated linear proxy segments).
            Edges = _edges

            # AreaLoadMeshes: actual Rhino Mesh objects built from JSON mesh records and/or
            # area_geometry input (surfaces/breps). Runs unconditionally so the output is usable
            # without enabling preview. Falls back to string IDs when rg is unavailable (CLI).
            if rg is not None:
                _area_geo_built = _build_area_mesh_geometry(
                    _meshes, auto_mesh_areas=_auto_mesh_areas, nodes=_nodes
                )
                _area_geo_built += _build_area_mesh_geometry_from_input(
                    _area_geometry_input, auto_mesh_areas=_auto_mesh_areas, nodes=_nodes
                )
                AreaLoadMeshes = _area_geo_built
            else:
                AreaLoadMeshes = list(_lists.get("area_load_meshes", []))

            # Optional single-stream preview geometry for Custom Preview.
            if _preview_enabled and rg is not None:
                _preview_items: List[Any] = []

                _kind = _preview_kind
                if _kind in ("member", "member_line", "member_lines"):
                    _kind = "members"
                elif _kind in ("area", "area_load", "area_loads"):
                    _kind = "areas"
                elif _kind in ("linear", "linear_load", "linear_loads", "linear_load_lines", "lines", "load_lines"):
                    _kind = "linear"
                elif _kind in ("point", "point_load", "point_loads", "point_load_points"):
                    _kind = "point_loads"
                elif _kind in ("boundary", "boundary_point", "boundary_points"):
                    _kind = "boundary"
                elif _kind in ("joint", "joint_node", "joint_nodes"):
                    _kind = "joints"
                elif _kind not in ("members", "areas", "linear", "point_loads", "boundary", "joints", "all"):
                    _kind = "members"

                if _kind in ("members", "all"):
                    _preview_items.extend(_line_geometry_from_edge_ids(_member_line_ids, _edges, _nodes))

                # "linear" is a filtered subset of members — shown on its own, not duplicated in "all".
                if _kind == "linear":
                    _preview_items.extend(_line_geometry_from_edge_ids(_linear_line_ids, _edges, _nodes))

                # point_loads / boundary are opt-in and remain empty unless their proxy input is connected.
                if _kind == "point_loads":
                    _preview_items.extend(_point_geometry_from_node_ids(_point_node_ids, _nodes))

                if _kind == "boundary":
                    _preview_items.extend(_point_geometry_from_node_ids(_boundary_node_ids, _nodes))

                if _kind in ("joints", "all"):
                    _preview_items.extend(_point_geometry_from_node_ids(_joint_node_ids, _nodes))

                if _kind in ("areas", "all"):
                    _preview_items.extend(AreaLoadMeshes)

                PreviewGeometry = _preview_items

            if _preview_enabled:
                out = (
                    "Imported -> members: {}, areas: {}, linear: {}, load pts: {}, boundary: {}, joints: {} | preview({}): {}"
                ).format(
                    len(MemberLines),
                    len(AreaLoadMeshes),
                    len(LinearLoadLines),
                    len(PointLoadPoints),
                    len(BoundaryPoints),
                    len(JointNodes),
                    _preview_kind,
                    len(PreviewGeometry),
                )
            else:
                out = (
                    "Imported -> members: {}, areas: {}, linear: {}, load pts: {}, boundary: {}, joints: {}"
                ).format(
                    len(MemberLines),
                    len(AreaLoadMeshes),
                    len(LinearLoadLines),
                    len(PointLoadPoints),
                    len(BoundaryPoints),
                    len(JointNodes),
                )
        except Exception as ex:
            ImportJson = json.dumps(
                {
                    "schema": "structure_model_v1",
                    "error": str(ex),
                    "model_input_type": type(_model_input).__name__ if "_model_input" in locals() else "unknown",
                },
                separators=(",", ":"),
            )
            AreaLoadMeshes = []
            LinearLoadLines = []
            PointLoadPoints = []
            BoundaryPoints = []
            JointNodes = []
            out = "Import failed: {}".format(ex)


if __name__ == "__main__" and any(arg.startswith("--input") for arg in sys.argv[1:]):
    main()
