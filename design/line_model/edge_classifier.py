"""Edge classification utilities for line-model graphs."""

from compas.geometry import Vector, Line, Point, centroid_points
from compas.geometry import distance_point_point_xy
from compas.datastructures import Graph
from config import (
    DEFAULT_PARALLEL_TOL, DEFAULT_NEAR_THRESHOLD, 
    DEFAULT_OVERLAP, DEFAULT_SEG_X, DEFAULT_SEG_Y, DEFAULT_ANGLE_TOL
)
import math
from nodegraph import NodeGraph


def _get_support_points(graph, support_points=None):
    """Resolve supports from explicit points or graph support nodes."""
    if support_points:
        return [pt for pt in support_points if pt is not None]

    pts = []
    for n in graph.get_support_nodes():
        pt = graph.node_attribute(n, "point")
        if pt is not None:
            pts.append(pt)
    return pts


def _edge_key(edge):
    """Return orientation-independent edge key for undirected graph edges."""
    u, v = edge
    return frozenset((u, v))


def edges_from_keys(graph, edge_keys):
    """Convert edge keys back to actual Line objects for visualization."""
    lines = []
    for edge in graph.edges():
        if _edge_key(edge) in edge_keys:
            u, v = edge
            pu = graph.node_attribute(u, "point")
            pv = graph.node_attribute(v, "point")
            if pu is not None and pv is not None:
                lines.append(Line(pu, pv))
    return lines


def _edge_xy_direction(graph, edge):
    """Unitized XY direction for an edge, normalized to a single half-plane."""
    u, v = edge
    pu = graph.node_attribute(u, "point")
    pv = graph.node_attribute(v, "point")
    if pu is None or pv is None:
        return None

    d = Vector(pv.x - pu.x, pv.y - pu.y, 0.0)
    if d.length < 1e-9:
        return None
    d.unitize()
    return _normalize_vector_direction(d)


def _edge_xy_direction_from_node(graph, edge, node):
    """Unitized XY direction for edge, oriented from `node` to the opposite endpoint."""
    u, v = edge
    if node != u and node != v:
        return None
    other = v if node == u else u
    pn = graph.node_attribute(node, "point")
    po = graph.node_attribute(other, "point")
    if pn is None or po is None:
        return None
    d = Vector(po.x - pn.x, po.y - pn.y, 0.0)
    if d.length < 1e-9:
        return None
    d.unitize()
    return d


def _edge_on_seed_plane(graph, edge, seed_origin, seed_dir_xy, plane_tol):
    """
    Check whether edge endpoints lie on the vertical plane through seed_origin
    and aligned with seed_dir_xy.
    """
    plane_normal = Vector(seed_dir_xy.y, -seed_dir_xy.x, 0.0)
    if plane_normal.length < 1e-9:
        return False
    plane_normal.unitize()

    u, v = edge
    pu = graph.node_attribute(u, "point")
    pv = graph.node_attribute(v, "point")
    if pu is None or pv is None:
        return False

    for pt in (pu, pv):
        vec = Vector.from_start_end(seed_origin, pt)
        dist = abs(vec.dot(plane_normal))
        if dist > plane_tol:
            return False
    return True


def _grow_node_for(edge, node):
    u, v = edge
    return v if u == node else u


def _find_continuation_edges(
    graph,
    grow_node,
    secondary_by_node,
    secondary_keys,
    chosen_dir,
    seed_origin,
    parallel_tol,
    plane_tol,
):
    """
    Find up to 2 continuation edges from grow_node on the same seed plane.
    Direction is not used as a filter; plane membership controls selection.
    """
    in_plane = []

    for cand in secondary_by_node.get(grow_node, []):
        if _edge_key(cand["edge"]) in secondary_keys:
            continue
        same_plane = _edge_on_seed_plane(graph, cand["edge"], seed_origin, chosen_dir, plane_tol)
        if not same_plane:
            continue
        in_plane.append(cand)

    if in_plane:
        in_plane.sort(key=lambda it: str(it["edge"]))
        return in_plane[:2], "plane"

    parallel_candidates = []
    for cand in secondary_by_node.get(grow_node, []):
        if _edge_key(cand["edge"]) in secondary_keys:
            continue
        dxy = _edge_xy_direction_from_node(graph, cand["edge"], grow_node)
        if dxy is None:
            continue
        dot = dxy.dot(chosen_dir)
        if dot >= parallel_tol:
            parallel_candidates.append((dot, cand))

    if parallel_candidates:
        parallel_candidates.sort(key=lambda it: (-it[0], str(it[1]["edge"])))
        return [it[1] for it in parallel_candidates[:2]], "parallel_fallback"

    return [], "none"


def _group_center_xy(group_records, support_pts=None):
    """Calculate center of group, biased toward nearest support point."""
    pts = [Point(r["point"].x, r["point"].y, 0.0) for r in group_records]
    c = centroid_points(pts)
    cx, cy = c[0], c[1]
    
    if not support_pts:
        return cx, cy
    
    best = None
    best_d2 = float("inf")
    for sp in support_pts:
        dx = sp.x - cx
        dy = sp.y - cy
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best = sp
    
    return (best.x, best.y) if best else (cx, cy)


def _angle_delta(a, b):
    """Normalize angle difference to [-π, π]."""
    d = a - b
    while d <= -math.pi:
        d += 2.0 * math.pi
    while d > math.pi:
        d -= 2.0 * math.pi
    return d


def _candidate_entries_for_node(graph, node, candidates, used_edge_keys):
    """Get candidate edges for node with their directions, excluding used edges."""
    entries = []
    for item in candidates:
        if _edge_key(item["edge"]) in used_edge_keys:
            continue
        dxy = _edge_xy_direction_from_node(graph, item["edge"], node)
        if dxy is not None:
            entries.append((item, dxy))
    return entries


def _pick_best_tangent_entry(entries, rec):
    """Pick entry whose direction is closest to the tangent angle."""
    tangent = rec.get("tangent_ccw")
    if tangent is None or tangent.length < 1e-9:
        return None, None

    tangent_angle = math.atan2(tangent.y, tangent.x)
    best = None, None, float("inf")
    for item, dxy in entries:
        direction_angle = math.atan2(dxy.y, dxy.x)
        angle_gap = abs(_angle_delta(direction_angle, tangent_angle))
        if angle_gap < best[2]:
            best = item, dxy, angle_gap
    return best[0], best[1]


def _split_primary_by_direction(graph, primary_edges, parallel_tol):
    """Split primary edges into main_primary (dominant direction) and primary (others)."""
    if not primary_edges:
        return set(), set()
    
    # Collect directions and group by parallel alignment
    direction_groups = []
    for edge in primary_edges:
        dxy = _edge_xy_direction(graph, edge)
        if dxy is None:
            continue
        
        # Find group with matching parallel direction
        matched = None
        for grp in direction_groups:
            if abs(dxy.dot(grp["ref_dir"])) >= parallel_tol:
                matched = grp
                break
        
        if matched:
            matched["edges"].append(edge)
        else:
            direction_groups.append({"ref_dir": dxy, "edges": [edge]})
    
    # Main direction is the largest group
    if not direction_groups:
        return set(), set(_edge_key(e) for e in primary_edges)
    
    main_group = max(direction_groups, key=lambda g: len(g["edges"]))
    main_primary_keys = set(_edge_key(e) for e in main_group["edges"])
    
    primary_keys = set(_edge_key(e) for e in primary_edges if _edge_key(e) not in main_primary_keys)
    
    return main_primary_keys, primary_keys


def _add_edge_and_grow(item, node, direction, graph, items_list, keys_set, secondary_by_node, parallel_tol, plane_tol):
    """Add an edge to the list and grow it with continuations."""
    items_list.append(item)
    keys_set.add(_edge_key(item["edge"]))
    
    grow_node = _grow_node_for(item["edge"], node)
    grow_pt = graph.node_attribute(grow_node, "point")
    if grow_pt is not None:
        next_items, _ = _find_continuation_edges(
            graph=graph,
            grow_node=grow_node,
            secondary_by_node=secondary_by_node,
            secondary_keys=keys_set,
            chosen_dir=direction,
            seed_origin=grow_pt,
            parallel_tol=parallel_tol,
            plane_tol=plane_tol,
        )
        for nxt in next_items:
            items_list.append(nxt)
            keys_set.add(_edge_key(nxt["edge"]))


def _split_secondary_by_level_one_seed(
    graph,
    parallel_tol,
    plane_tol=1e-3,
):
    """Split leftover edges into secondary and tertiary via spatial/directional grouping."""
    
    # Identify level-1 nodes connected to primary edges
    primary_keys = set(_edge_key(e) for e in graph.edges() if graph.edge_attribute(e, "hierarchy") == "primary")
    level1_nodes = []
    
    for n in graph.nodes():
        try:
            level = int(float(graph.node_attribute(n, "level")))
        except (TypeError, ValueError):
            continue
        if level == 1 and any(_edge_key((n, nbr)) in primary_keys for nbr in graph.neighbors(n)):
            level1_nodes.append(n)
    
    # Gather all secondary items indexed by node
    secondary_items = [{"edge": e} for e in graph.edges() if graph.edge_attribute(e, "hierarchy") != "primary"]
    secondary_by_node = {}
    for item in secondary_items:
        u, v = item["edge"]
        secondary_by_node.setdefault(u, []).append(item)
        secondary_by_node.setdefault(v, []).append(item)
    
    # Build node records from level-1 nodes with candidates
    node_records = []
    for node in sorted(level1_nodes, key=str):
        pt = graph.node_attribute(node, "point")
        candidates = secondary_by_node.get(node, [])
        
        if pt is None or not candidates:
            continue
        
        # Find primary direction at this node
        primary_dir = None
        for nbr in graph.neighbors(node):
            if _edge_key((node, nbr)) in primary_keys:
                primary_dir = _edge_xy_direction(graph, (node, nbr))
                if primary_dir is not None:
                    break
        
        if primary_dir is not None:
            node_records.append({"node": node, "point": pt, "primary_dir": primary_dir, "candidates": candidates})
    
    # Early exit if no viable level-1 nodes
    if not node_records:
        return [], secondary_items
    
    # Spatially partition node records into 4 tiles
    support_pts = _get_support_points(graph)
    xs = [r["point"].x for r in node_records]
    ys = [r["point"].y for r in node_records]
    mid_x = 0.5 * (min(xs) + max(xs))
    mid_y = 0.5 * (min(ys) + max(ys))
    
    tiles = {(0, 0): [], (1, 0): [], (0, 1): [], (1, 1): []}
    for rec in node_records:
        ix = 0 if rec["point"].x <= mid_x else 1
        iy = 0 if rec["point"].y <= mid_y else 1
        tiles[(ix, iy)].append(rec)
    
    # Process each tile's direction groups
    secondary_group = []
    secondary_keys = set()
    
    for bidx, tile_key in enumerate([(0, 0), (1, 0), (0, 1), (1, 1)]):
        block_records = tiles[tile_key]
        if not block_records:
            continue
        
        # Group by parallel primary direction
        direction_groups = []
        for rec in block_records:
            matched = None
            for grp in direction_groups:
                if abs(rec["primary_dir"].dot(grp["ref_dir"])) >= parallel_tol:
                    matched = grp
                    break
            
            if matched:
                matched["nodes"].append(rec)
            else:
                direction_groups.append({"ref_dir": rec["primary_dir"], "nodes": [rec]})
        
        # Process each direction group
        for lidx, grp in enumerate(direction_groups):
            cx, cy = _group_center_xy(grp["nodes"], support_pts)
            group_nodes = sorted(grp["nodes"], key=lambda r: math.atan2(r["point"].y - cy, r["point"].x - cx))
            
            # Calculate tangents and angles
            for rec in group_nodes:
                px, py = rec["point"].x, rec["point"].y
                rec["angle"] = math.atan2(py - cy, px - cx)
                rv = Vector(px - cx, py - cy, 0.0)
                if rv.length > 1e-9:
                    rv.unitize()
                    tangent = Vector(-rv.y, rv.x, 0.0)
                else:
                    tangent = Vector(-rec["primary_dir"].y, rec["primary_dir"].x, 0.0)
                rec["tangent_ccw"] = tangent if tangent.length < 1e-9 else (tangent.unitize() or tangent)
            
            # Match and chain nodes
            used_nodes = set()
            out_group_id = bidx * 10 + lidx
            
            for rec in group_nodes:
                if rec["node"] in used_nodes:
                    continue
                
                entries_a = _candidate_entries_for_node(graph, rec["node"], rec["candidates"], secondary_keys)
                if not entries_a:
                    used_nodes.add(rec["node"])
                    continue
                
                # Find best partner at opposite angle
                partner = None
                best_gap = None
                for other in group_nodes:
                    if other["node"] in used_nodes or other["node"] == rec["node"]:
                        continue
                    gap = abs(abs(_angle_delta(rec["angle"], other["angle"])) - math.pi)
                    if partner is None or gap < best_gap:
                        partner, best_gap = other, gap
                
                # Single or paired chain
                if partner is None:
                    item, direction = _pick_best_tangent_entry(entries_a, rec)
                    if item:
                        _add_edge_and_grow(item, rec["node"], direction, graph, secondary_group, secondary_keys, secondary_by_node, parallel_tol, plane_tol)
                    used_nodes.add(rec["node"])
                else:
                    entries_b = _candidate_entries_for_node(graph, partner["node"], partner["candidates"], secondary_keys)
                    if not entries_b:
                        item, direction = _pick_best_tangent_entry(entries_a, rec)
                        if item:
                            _add_edge_and_grow(item, rec["node"], direction, graph, secondary_group, secondary_keys, secondary_by_node, parallel_tol, plane_tol)
                        used_nodes.add(rec["node"])
                        continue
                    
                    item_a, dir_a = _pick_best_tangent_entry(entries_a, rec)
                    item_b, dir_b = _pick_best_tangent_entry(entries_b, partner)
                    
                    if item_a:
                        _add_edge_and_grow(item_a, rec["node"], dir_a, graph, secondary_group, secondary_keys, secondary_by_node, parallel_tol, plane_tol)
                    if item_b:
                        _add_edge_and_grow(item_b, partner["node"], dir_b, graph, secondary_group, secondary_keys, secondary_by_node, parallel_tol, plane_tol)
                    
                    used_nodes.add(rec["node"])
                    used_nodes.add(partner["node"])
    
    # All unselected secondary items become tertiary
    tertiary_group = [item for item in secondary_items if _edge_key(item["edge"]) not in secondary_keys]
    
    return secondary_group, tertiary_group


def classify_edges_by_support_direction(
    graph,
    parallel_tol=None,
    debug=False,
    seed_plane_tol=1e-3,
):
    """Classify edges into primary/secondary and optionally tertiary."""
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL

    sup_edges = graph.get_support_edges()
    
    # Mark support edges as primary and collect their plane information
    primary_keys = set()
    support_planes = []  # List of (origin_point, direction_vector) tuples
    
    for edge in sup_edges:
        primary_keys.add(_edge_key(edge))
        graph.edge_attribute(edge, "hierarchy", "primary")
        
        # Extract plane from support edge
        u, v = edge
        pu = graph.node_attribute(u, "point")
        pv = graph.node_attribute(v, "point")
        if pu is not None and pv is not None:
            # Direction of support edge
            dir_xy = _edge_xy_direction(graph, edge)
            if dir_xy is not None:
                support_planes.append((pu, dir_xy))
    
    # Find all other edges on the same plane as support edges
    secondary_items = []
    for edge in graph.edges():
        if _edge_key(edge) in primary_keys:
            continue
        
        # Check if this edge lies on any support plane
        on_support_plane = False
        for origin, plane_dir in support_planes:
            if _edge_on_seed_plane(graph, edge, origin, plane_dir, seed_plane_tol):
                on_support_plane = True
                break
        
        if on_support_plane:
            # Add to primary
            primary_keys.add(_edge_key(edge))
            graph.edge_attribute(edge, "hierarchy", "primary")
        else:
            # Add to secondary processing list
            secondary_items.append({"edge": edge})
    
    # Split remaining edges into secondary and tertiary
    split_result = _split_secondary_by_level_one_seed(
        graph,
        parallel_tol=parallel_tol,
        plane_tol=seed_plane_tol,
    )

    secondary_group, tertiary_group = split_result

    secondary_keys = set(_edge_key(item["edge"]) for item in secondary_group)
    tertiary_keys = set(_edge_key(item["edge"]) for item in tertiary_group)

    # Separate primary edges into main_primary (dominant direction) and primary (others)
    primary_edges = [edge for edge in graph.edges() if _edge_key(edge) in primary_keys]
    main_primary_keys, primary_keys = _split_primary_by_direction(graph, primary_edges, parallel_tol)

    # Update hierarchy attributes
    for edge in graph.edges():
        ek = _edge_key(edge)
        if ek in main_primary_keys:
            graph.edge_attribute(edge, "hierarchy", "main_primary")
        elif ek in primary_keys:
            graph.edge_attribute(edge, "hierarchy", "primary")
        elif ek in secondary_keys:
            graph.edge_attribute(edge, "hierarchy", "secondary")
        elif ek in tertiary_keys:
            graph.edge_attribute(edge, "hierarchy", "tertiary")

    return main_primary_keys, primary_keys, secondary_keys, tertiary_keys

def build_category_index(graph, category_mode=2):
    """Build reusable node/edge category metadata."""
    idx = {"nodes": {}, "edges": {}}

    for n in graph.nodes():
        deg = graph.degree(n)
        group = graph.node_attribute(n, "group")
        level = graph.node_attribute(n, "level")
        reached = bool(graph.node_attribute(n, "reached"))

        degree_cat = "leaf" if deg == 1 else ("junction" if deg >= 3 else "chain")
        if level is None:
            level_cat = "L?"
        else:
            level_cat = "L{}".format(level)

        if category_mode == 0:
            category = (degree_cat,)
        elif category_mode == 1:
            category = (level_cat,)
        else:
            category = (degree_cat, level_cat)

        idx["nodes"][n] = {
            "degree": deg,
            "degree_cat": degree_cat,
            "group": group,
            "level": level,
            "level_cat": level_cat,
            "is_support": reached,
            "category": category,
        }

    for u, v in graph.edges():
        gu = idx["nodes"][u]["group"]
        gv = idx["nodes"][v]["group"]
        idx["edges"][(u, v)] = {
            "hierarchy": graph.edge_attribute((u, v), "hierarchy"),
            "inter_module": (gu is not None and gv is not None and gu != gv),
        }

    return idx

def create_subgraphs(graph, seg_x=None, seg_y=None, overlap=None, debug=False):
    """Divide graph into spatial subgraphs using a regular XY grid."""
    if seg_x is None:
        seg_x = DEFAULT_SEG_X
    if seg_y is None:
        seg_y = DEFAULT_SEG_Y
    if overlap is None:
        overlap = DEFAULT_OVERLAP
    
    node_pts = {}
    x_coords = []
    y_coords = []
    extras = []

    for n in graph.nodes():
        pt = graph.node_attribute(n, "point")
        if pt:
            node_pts[n] = pt
            x_coords.append(pt.x)
            y_coords.append(pt.y)

        extra = graph.node_attribute(n, "original_point")
        if extra:
            x_coords.append(extra.x)
            y_coords.append(extra.y)
            extras.append(extra)

    if not node_pts:
        return [], []

    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    x_range = x_max - x_min
    y_range = y_max - y_min
    cell_w = x_range / seg_x
    cell_h = y_range / seg_y

    node_attrs = ["x", "y", "z", "point", "group", "level", "is_support", "reached", "ntype"]
    edge_attrs = ["hierarchy", "etype", "group", "parallel_score", "nearest_support"]

    def _attrs(getter, key, names):
        return {name: val for name in names for val in [getter(key, name)] if val is not None}

    subgraphs = []
    for sj in range(seg_y):
        for si in range(seg_x):
            win_x_min = x_min + si * cell_w - cell_w * overlap
            win_x_max = x_min + (si + 1) * cell_w + cell_w * overlap
            win_y_min = y_min + sj * cell_h - cell_h * overlap
            win_y_max = y_min + (sj + 1) * cell_h + cell_h * overlap

            nodes_in_win = {
                n for n, pt in node_pts.items()
                if win_x_min <= pt.x <= win_x_max and win_y_min <= pt.y <= win_y_max
            }
            edges_in_win = [(u, v) for u, v in graph.edges() if u in nodes_in_win and v in nodes_in_win]

            sg = NodeGraph()
            for n in nodes_in_win:
                sg.add_node(n, **_attrs(graph.node_attribute, n, node_attrs))

            for u, v in edges_in_win:
                sg.add_edge(u, v, **_attrs(graph.edge_attribute, (u, v), edge_attrs))

            subgraphs.append({
                "si": si,
                "sj": sj,
                "index": sj * seg_x + si,
                "graph": sg,
                "edges": edges_in_win,
                "bounds": (win_x_min, win_x_max, win_y_min, win_y_max)
            })

    return subgraphs, extras


def _normalize_vector_direction(v):
    """Normalize vector to positive half-plane."""
    if v.x < -1e-9 or (abs(v.x) < 1e-9 and v.y < 0):
        return Vector(-v.x, -v.y, 0.0)
    return Vector(v.x, v.y, 0.0)


def find_dominant_direction(vecs, angle_tol=None):
    """Find dominant XY direction from vectors by angle binning."""
    if not vecs:
        return None
    
    if angle_tol is None:
        angle_tol = DEFAULT_ANGLE_TOL

    normalized = [_normalize_vector_direction(v) for v in vecs]
    angles = [math.atan2(v.y, v.x) for v in normalized]

    bins = {}
    for i, ang in enumerate(angles):
        bin_key = round(ang / angle_tol)
        if bin_key not in bins:
            bins[bin_key] = []
        bins[bin_key].append(normalized[i])

    largest_bin = max(bins.values(), key=len)

    pts = [Point(v.x, v.y, 0.0) for v in largest_bin]
    c = centroid_points(pts)
    dom = Vector(c[0], c[1], 0.0)

    if dom.length > 1e-9:
        dom.unitize()
        return dom
    return None





