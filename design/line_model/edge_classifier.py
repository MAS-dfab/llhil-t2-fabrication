"""Edge classification utilities for line-model graphs."""

from compas.geometry import Vector, Line, Point, centroid_points
from compas.geometry import distance_point_point_xy
from compas.datastructures import Graph
from config import (
    DEFAULT_PARALLEL_TOL, DEFAULT_NEAR_THRESHOLD, 
    DEFAULT_OVERLAP, DEFAULT_SEG_X, DEFAULT_SEG_Y, DEFAULT_ANGLE_TOL
)
import math


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


def _empty_classification_return(return_tertiary, return_node_edge_report=False):
    """Return correctly-shaped empty outputs for classifier API."""
    if return_tertiary and return_node_edge_report:
        return [], [], [], [], []
    if return_tertiary:
        return [], [], [], []
    return [], [], []


def _edge_key(edge):
    """Return orientation-independent edge key for undirected graph edges."""
    u, v = edge
    return frozenset((u, v))


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


def _split_secondary_by_level_one_seed(
    graph,
    primary_edges,
    secondary_items,
    parallel_tol,
    plane_tol=1e-3,
    return_node_edge_report=False,
):
    """Split leftover edges into secondary and tertiary."""
    if not secondary_items:
        return ([], [], []) if return_node_edge_report else ([], [])

    primary_keys = set(_edge_key(e) for e in primary_edges)
    level1_nodes = []
    for n in graph.nodes():
        try:
            level = int(float(graph.node_attribute(n, "level")))
        except (TypeError, ValueError):
            continue
        if level != 1:
            continue
        if any(_edge_key((n, nbr)) in primary_keys for nbr in graph.neighbors(n)):
            level1_nodes.append(n)

    if not level1_nodes:
        return ([], list(secondary_items), []) if return_node_edge_report else ([], list(secondary_items))

    secondary_by_node = {}
    for item in secondary_items:
        u, v = item["edge"]
        secondary_by_node.setdefault(u, []).append(item)
        secondary_by_node.setdefault(v, []).append(item)

    node_records = []
    for node in sorted(level1_nodes, key=lambda x: str(x)):
        pt = graph.node_attribute(node, "point")
        candidates = secondary_by_node.get(node, [])
        if pt is None or not candidates:
            continue

        primary_dir = None
        for nbr in graph.neighbors(node):
            if _edge_key((node, nbr)) in primary_keys:
                primary_dir = _edge_xy_direction(graph, (node, nbr))
                if primary_dir is not None:
                    break
        if primary_dir is None:
            continue

        node_records.append({"node": node, "point": pt, "primary_dir": primary_dir, "candidates": candidates})

    if not node_records:
        return ([], list(secondary_items), []) if return_node_edge_report else ([], list(secondary_items))

    support_pts = _get_support_points(graph)

    def _group_center_xy(group_nodes):
        pts = [Point(r["point"].x, r["point"].y, 0.0) for r in group_nodes]
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

        if best is None:
            return cx, cy
        return best.x, best.y

    xs = [r["point"].x for r in node_records]
    ys = [r["point"].y for r in node_records]
    mid_x = 0.5 * (min(xs) + max(xs))
    mid_y = 0.5 * (min(ys) + max(ys))

    tiles = {(0, 0): [], (1, 0): [], (0, 1): [], (1, 1): []}
    for rec in node_records:
        ix = 0 if rec["point"].x <= mid_x else 1
        iy = 0 if rec["point"].y <= mid_y else 1
        tiles[(ix, iy)].append(rec)

    def _candidate_entries_for_node(rec, used_edge_keys):
        out = []
        n = rec["node"]
        for item in rec["candidates"]:
            if _edge_key(item["edge"]) in used_edge_keys:
                continue
            dxy = _edge_xy_direction_from_node(graph, item["edge"], n)
            if dxy is None:
                continue
            out.append((item, dxy))
        return out

    def _xy_intersection_penalty(edge_a, edge_b):
        ua, va = edge_a
        ub, vb = edge_b
        pa = graph.node_attribute(ua, "point")
        qa = graph.node_attribute(va, "point")
        pb = graph.node_attribute(ub, "point")
        qb = graph.node_attribute(vb, "point")
        if not pa or not qa or not pb or not qb:
            return 0.0

        def orient(p, q, r):
            return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)

        o1 = orient(pa, qa, pb)
        o2 = orient(pa, qa, qb)
        o3 = orient(pb, qb, pa)
        o4 = orient(pb, qb, qa)
        if (o1 * o2 < 0.0) and (o3 * o4 < 0.0):
            return 1.0
        return 0.0

    def _angle_delta(a, b):
        d = a - b
        while d <= -math.pi:
            d += 2.0 * math.pi
        while d > math.pi:
            d -= 2.0 * math.pi
        return d

    secondary_group = []
    secondary_keys = set()
    node_edge_report = []

    def _pick_best_tangent_entry(entries, rec):
        best_item, best_dir, best_score = None, None, -1e9
        for item, dxy in entries:
            score = dxy.dot(rec["tangent_ccw"])
            if score > best_score:
                best_item, best_dir, best_score = item, dxy, score
        return best_item, best_dir

    def _add_chain(grp_idx, rec, seed_item, seed_dir):
        node = rec["node"]
        seed_origin = graph.node_attribute(node, "point")
        if seed_origin is None:
            return
        grow_node = _grow_node_for(seed_item["edge"], node)

        secondary_group.append(seed_item)
        secondary_keys.add(_edge_key(seed_item["edge"]))

        next_items, mode = _find_continuation_edges(
            graph=graph,
            grow_node=grow_node,
            secondary_by_node=secondary_by_node,
            secondary_keys=secondary_keys,
            chosen_dir=seed_dir,
            seed_origin=seed_origin,
            parallel_tol=parallel_tol,
            plane_tol=plane_tol,
        )
        for nxt in next_items:
            secondary_group.append(nxt)
            secondary_keys.add(_edge_key(nxt["edge"]))

        if return_node_edge_report:
            node_edge_report.append({
                "group": grp_idx,
                "node": node,
                "candidate_edges": [it["edge"] for it in rec["candidates"]],
                "seed_edge": seed_item["edge"],
                "grow_node": grow_node,
                "next_edges": [n["edge"] for n in next_items],
                "next_edge": next_items[0]["edge"] if next_items else None,
                "continuation_mode": mode,
            })

    def _add_best_single(grp_idx, rec, entries):
        best_item, best_dir = _pick_best_tangent_entry(entries, rec)
        if best_item is not None:
            _add_chain(grp_idx, rec, best_item, best_dir)

    block_order = [(0, 0), (1, 0), (0, 1), (1, 1)]
    for bidx, tile_key in enumerate(block_order):
        block_records = tiles[tile_key]
        if not block_records:
            continue

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

        for lidx, grp in enumerate(direction_groups):
            cx, cy = _group_center_xy(grp["nodes"])
            for rec in grp["nodes"]:
                px, py = rec["point"].x, rec["point"].y
                rec["angle"] = math.atan2(py - cy, px - cx)
                rv = Vector(px - cx, py - cy, 0.0)
                if rv.length > 1e-9:
                    rv.unitize()
                    tangent = Vector(-rv.y, rv.x, 0.0)
                else:
                    tangent = Vector(-rec["primary_dir"].y, rec["primary_dir"].x, 0.0)
                if tangent.length > 1e-9:
                    tangent.unitize()
                rec["tangent_ccw"] = tangent

            group_nodes = sorted(grp["nodes"], key=lambda r: r["angle"])
            used_nodes = set()
            out_group_id = bidx * 10 + lidx

            for rec in group_nodes:
                node = rec["node"]
                if node in used_nodes:
                    continue

                partner = None
                best_gap = None
                for other in group_nodes:
                    onode = other["node"]
                    if onode == node or onode in used_nodes:
                        continue
                    gap = abs(abs(_angle_delta(rec["angle"], other["angle"])) - math.pi)
                    if partner is None or gap < best_gap:
                        partner = other
                        best_gap = gap

                entries_a = _candidate_entries_for_node(rec, secondary_keys)
                if not entries_a:
                    used_nodes.add(node)
                    continue

                if partner is None:
                    _add_best_single(out_group_id, rec, entries_a)
                    used_nodes.add(node)
                    continue

                entries_b = _candidate_entries_for_node(partner, secondary_keys)
                if not entries_b:
                    _add_best_single(out_group_id, rec, entries_a)
                    used_nodes.add(node)
                    continue

                best_combo = None
                best_score = -1e9
                for item_a, dir_a in entries_a:
                    grow_a = _grow_node_for(item_a["edge"], rec["node"])
                    tan_a = dir_a.dot(rec["tangent_ccw"])
                    for item_b, dir_b in entries_b:
                        grow_b = _grow_node_for(item_b["edge"], partner["node"])
                        tan_b = dir_b.dot(partner["tangent_ccw"])
                        parallel_score = abs(dir_a.dot(dir_b))
                        crossing_penalty = _xy_intersection_penalty(item_a["edge"], item_b["edge"])
                        same_grow_penalty = 1.0 if grow_a == grow_b else 0.0

                        score = 3.0 * parallel_score + tan_a + tan_b - 4.0 * crossing_penalty - 2.0 * same_grow_penalty
                        if score > best_score:
                            best_score = score
                            best_combo = (item_a, dir_a, item_b, dir_b)

                if best_combo:
                    item_a, dir_a, item_b, dir_b = best_combo
                    _add_chain(out_group_id, rec, item_a, dir_a)
                    _add_chain(out_group_id, partner, item_b, dir_b)
                used_nodes.add(node)
                used_nodes.add(partner["node"])

    tertiary_group = [item for item in secondary_items if _edge_key(item["edge"]) not in secondary_keys]
    if return_node_edge_report:
        return secondary_group, tertiary_group, node_edge_report
    return secondary_group, tertiary_group


def classify_edges_by_support_direction(
    graph,
    parallel_tol=None,
    debug=False,
    support_points=None,
    return_tertiary=False,
    seed_plane_tol=1e-3,
    return_node_edge_report=False,
):
    """Classify edges into primary/secondary and optionally tertiary."""
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL

    sup_pts = _get_support_points(graph, support_points=support_points)
    if not sup_pts:
        if debug:
            print("WARNING: No support points available!")
        return _empty_classification_return(return_tertiary, return_node_edge_report)

    primary_lines, secondary_lines, data = [], [], []
    primary_edges, secondary_items = [], []

    def _set_hierarchy(edge, etype):
        # Keep explicit edge hierarchy in sync with classifier labels so
        # downstream filters can target hierarchies directly.
        if etype in ("primary", "secondary", "tertiary", "double"):
            graph.edge_attribute(edge, "hierarchy", etype)

    def _store(edge, line, etype, support_idx, dot):
        data.append({"edge": edge, "support_idx": support_idx, "dot": dot, "type": etype})
        graph.edge_attribute(edge, "main_secondary", etype)
        _set_hierarchy(edge, etype)
        graph.edge_attribute(edge, "parallel_score", dot)
        graph.edge_attribute(edge, "nearest_support", support_idx)
        if etype == "primary":
            primary_lines.append(line)
            primary_edges.append(edge)
        elif etype == "secondary":
            secondary_lines.append(line)
            secondary_items.append({"edge": edge, "line": line})

    for edge in graph.edges():
        if graph.edge_attribute(edge, "main_secondary") == "double":
            _set_hierarchy(edge, "double")
            line = graph.edge_line(edge)
            if line is not None:
                primary_lines.append(line)
                primary_edges.append(edge)
                data.append({"edge": edge, "support_idx": None, "dot": None, "type": "double"})
            continue

        line = graph.edge_line(edge)
        if line is None:
            continue

        mid = graph.edge_midpoint(edge)
        if mid is None:
            continue

        nearest_idx, nearest_pt = None, None
        best_dist = float("inf")
        for i, sp in enumerate(sup_pts):
            d = distance_point_point_xy(mid, sp)
            if d < best_dist:
                best_dist = d
                nearest_idx = i
                nearest_pt = Point(sp.x, sp.y, 0.0)
        if nearest_pt is None:
            continue

        edge_vec_xy = Vector(line.end.x - line.start.x, line.end.y - line.start.y, 0.0)
        if edge_vec_xy.length < 1e-9:
            _store(edge, line, "secondary", nearest_idx, 0.0)
            continue
        edge_vec_xy.unitize()

        sup_vec = Vector.from_start_end(Point(mid.x, mid.y, 0.0), nearest_pt)
        if sup_vec.length < 1e-9:
            _store(edge, line, "primary", nearest_idx, 1.0)
            continue
        sup_vec.unitize()

        dot = abs(edge_vec_xy.dot(sup_vec))
        _store(edge, line, "primary" if dot >= parallel_tol else "secondary", nearest_idx, dot)

    if return_tertiary:
        split_result = _split_secondary_by_level_one_seed(
            graph,
            primary_edges=primary_edges,
            secondary_items=secondary_items,
            parallel_tol=parallel_tol,
            plane_tol=seed_plane_tol,
            return_node_edge_report=return_node_edge_report,
        )

        if return_node_edge_report:
            secondary_group, tertiary_group, node_edge_report = split_result
        else:
            secondary_group, tertiary_group = split_result

        secondary_lines = [item["line"] for item in secondary_group]
        tertiary_lines = [item["line"] for item in tertiary_group]

        secondary_keys = set(_edge_key(item["edge"]) for item in secondary_group)
        tertiary_keys = set(_edge_key(item["edge"]) for item in tertiary_group)

        for d in data:
            k = _edge_key(d["edge"])
            if k in secondary_keys:
                d["type"] = "secondary"
                graph.edge_attribute(d["edge"], "main_secondary", "secondary")
                _set_hierarchy(d["edge"], "secondary")
            elif k in tertiary_keys:
                d["type"] = "tertiary"
                graph.edge_attribute(d["edge"], "main_secondary", "tertiary")
                _set_hierarchy(d["edge"], "tertiary")

        if return_node_edge_report:
            return primary_lines, secondary_lines, tertiary_lines, data, node_edge_report
        return primary_lines, secondary_lines, tertiary_lines, data

    return primary_lines, secondary_lines, data

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
            "main_secondary": graph.edge_attribute((u, v), "main_secondary"),
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
    edge_attrs = ["main_secondary", "etype", "group", "parallel_score", "nearest_support"]

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

            sg = Graph()
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


def is_segment_near_support(subgraph, sup_pts, threshold):
    """Return True if any subgraph node is near a support."""
    sg = subgraph["graph"]
    for n in sg.nodes():
        pt = sg.node_attribute(n, "point")
        if pt:
            for sp in sup_pts:
                dist = ((pt.x - sp.x)**2 + (pt.y - sp.y)**2)**0.5
                if dist < threshold:
                    return True
    return False


def classify_subgraph_edges(subgraph, sup_pts, near_threshold=None, parallel_tol=None):
    """Classify subgraph edges; near supports keep labels, far uses dominant dir."""
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL
    
    sg = subgraph["graph"]
    near_sup = is_segment_near_support(subgraph, sup_pts, near_threshold)

    all_edges = []
    primary_vecs = []

    for u, v in sg.edges():
        pu = sg.node_attribute(u, "point")
        pv = sg.node_attribute(v, "point")
        if not pu or not pv:
            continue

        evec = Vector(pv.x - pu.x, pv.y - pu.y, 0.0)
        if evec.length < 1e-9:
            continue
        evec.unitize()

        etype = sg.edge_attribute((u, v), "main_secondary") or "secondary"
        line = Line(pu, pv)

        all_edges.append({"vec": evec, "etype": etype, "line": line, "edge": (u, v)})

        if etype == "primary":
            primary_vecs.append(evec)

    primary_lines = []
    secondary_lines = []
    double_lines = []

    if near_sup:
        for ed in all_edges:
            if ed["etype"] == "double":
                double_lines.append(ed["line"])
            elif ed["etype"] == "primary":
                primary_lines.append(ed["line"])
            else:
                secondary_lines.append(ed["line"])
    else:
        dom_dir = find_dominant_direction(primary_vecs)

        for ed in all_edges:
            if ed["etype"] == "double":
                double_lines.append(ed["line"])
                continue
            
            new_etype = ed["etype"]
            
            if dom_dir:
                dot = abs(ed["vec"].dot(dom_dir))
                if dot >= parallel_tol:
                    new_etype = "primary"
                else:
                    new_etype = "secondary"
            
            if new_etype == "primary":
                primary_lines.append(ed["line"])
            else:
                secondary_lines.append(ed["line"])
            
            sg.edge_attribute(ed["edge"], "main_secondary", new_etype)

    return primary_lines, secondary_lines, double_lines, near_sup, sg


def classify_single_segment(graph, segment_index, seg_x=None, seg_y=None, 
                            parallel_tol=None, near_threshold=None):
    """Classify edges for one spatial segment."""
    if seg_x is None:
        seg_x = DEFAULT_SEG_X
    if seg_y is None:
        seg_y = DEFAULT_SEG_Y
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD
    
    classify_edges_by_support_direction(graph, parallel_tol)
    
    subgraphs, window = create_subgraphs(graph, seg_x, seg_y, debug=True)
    
    sup_pts = _get_support_points(graph)
    
    idx = int(segment_index) % len(subgraphs)
    sg_data = subgraphs[idx]
    
    primary, secondary, double, near_sup, sg = classify_subgraph_edges(
        sg_data, sup_pts, near_threshold, parallel_tol
    )
    
    return {
        "primary_lines": primary,
        "secondary_lines": secondary,
        "double_lines": double,
        "near_support": near_sup,
        "segment_index": idx,
        "subgraph": sg,
        "window": window
    }
