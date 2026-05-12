"""
Minimal line-constraint helpers for the current workflow.
"""

import math

from compas.geometry import Line, Point, Vector, Plane

# from design.line_model.edge_classifier import _edge_support_direction


def _resolve_edge(graph, edge):
    u, v = edge
    if graph.has_edge((u, v)):
        return (u, v)
    if graph.has_edge((v, u)):
        return (v, u)
    return None

def _edge_node_orientation(graph, edge, node):
    u, v = edge
    n = node 
    if n == u:
        return (u, v)
    else:
        return (v, u)

def _edge_key(edge):
    u, v = edge
    return (u, v) if u <= v else (v, u)


def _get_point(graph, node):
    pt = graph.node_attribute(node, "point")
    return Point(pt.x, pt.y, pt.z)


def _get_line(graph, edge):
    oriented = _resolve_edge(graph, edge)
    if oriented is None:
        return None

    shifted = graph.edge_attribute(oriented, "shifted_line")
    if shifted is not None:
        return shifted

    line_attr = graph.edge_attribute(oriented, "line")
    if isinstance(line_attr, Line):
        return line_attr
    if line_attr:
        return Line(Point(*line_attr[0]), Point(*line_attr[1]))

    u, v = oriented
    return Line(_get_point(graph, u), _get_point(graph, v))


def _line_param_and_on_segment(line, point, tol=1e-6):
    closest = line.closest_point(point)
    direction = line.direction
    length_sq = direction.length ** 2
    if length_sq < 1e-12:
        return 0.0, False
    vec = closest - line.start
    t = vec.dot(direction) / length_sq
    return t, (-tol <= t <= 1.0 + tol)


def _point_shifted_in_z(point, choose, z_offset):
    amt = abs(float(z_offset))
    if amt < 1e-12:
        return point
    dz = -amt if str(choose).lower() == "down" else amt
    return Point(point.x, point.y, point.z + dz)


def _dist(p, q):
    return ((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2) ** 0.5


def _set_node_point(graph, node, point):
    """
    Set node point and keep NodeGraph internal point index in sync when available.
    """
    old_pt = graph.node_attribute(node, "point")

    # Keep NodeGraph spatial index consistent if present.
    if hasattr(graph, "point_key") and hasattr(graph, "_point_index"):
        try:
            if old_pt is not None:
                old_key = graph.point_key(old_pt)
                if graph._point_index.get(old_key) == node:
                    del graph._point_index[old_key]
            graph._point_index[graph.point_key(point)] = node
        except Exception:
            pass

    graph.node_attribute(node, "point", point)
    graph.node_attribute(node, "x", point.x)
    graph.node_attribute(node, "y", point.y)
    graph.node_attribute(node, "z", point.z)


def _sync_node_points_from_edges(graph, edges):
    """
    Sync node 'point' from current edge geometry (shifted/line) for given edges.
    Uses endpoint-node best matching, then averages per node.
    """
    acc = {}  # node -> [Point, ...]

    for e in edges:
        oriented = _resolve_edge(graph, e)
        if oriented is None:
            continue
        u, v = oriented

        ln = _get_line(graph, oriented)
        if ln is None:
            continue

        pu = graph.node_attribute(u, "point")
        pv = graph.node_attribute(v, "point")
        if pu is None or pv is None:
            continue

        s, t = ln.start, ln.end
        score_uv = _dist(s, pu) + _dist(t, pv)
        score_vu = _dist(t, pu) + _dist(s, pv)
        if score_uv <= score_vu:
            mu, mv = s, t
        else:
            mu, mv = t, s

        acc.setdefault(u, []).append(mu)
        acc.setdefault(v, []).append(mv)

    for node, pts in acc.items():
        if not pts:
            continue
        x = sum(p.x for p in pts) / len(pts)
        y = sum(p.y for p in pts) / len(pts)
        z = sum(p.z for p in pts) / len(pts)
        _set_node_point(graph, node, Point(x, y, z))

    return list(acc.keys())



def snap_to_host(
    graph,
    target_hierarchy,
    move_node_level,
    host_hierarchy,
    target_categories=None,
    host_categories=None,
    target_level_pair=None,
    host_level_pair=None,
    z_offset=0.0,
    write_attribute=True,
    snap_8_degree_edges=True,
    debug=False,
):
    """
    Reposition the endpoint of each target edge at `move_node_level` onto the
    nearest connected host edge via XY projection, then shift by `z_offset` (signed).
    """
    req_target_pair = tuple(sorted((int(target_level_pair[0]), int(target_level_pair[1])))) if target_level_pair else None
    req_host_pair   = tuple(sorted((int(host_level_pair[0]),   int(host_level_pair[1]))))   if host_level_pair   else None

    host_hierarchies = {host_hierarchy} if isinstance(host_hierarchy, str) else set(host_hierarchy)

    def _level_pair(edge):
        u, v = edge
        return tuple(sorted((graph.node_attribute(u, "level"), graph.node_attribute(v, "level"))))

    def _h_match(edge_h, wanted_set):
        for wanted in wanted_set:
            if edge_h == wanted:
                return True
            if wanted == "primary" and (edge_h == "primary_orthogonal" or (isinstance(edge_h, str) and edge_h.startswith("primary_diagonal_"))):
                return True
        return False

    def _is_target(e):
        if not _h_match(graph.edge_attribute(e, "hierarchy"), {target_hierarchy}):
            return False
        if target_categories is not None and graph.edge_attribute(e, "e_category") not in target_categories:
            return False
        if req_target_pair is not None and _level_pair(e) != req_target_pair:
            return False
        return True

    def _is_host(e):
        if not _h_match(graph.edge_attribute(e, "hierarchy"), host_hierarchies):
            return False
        if host_categories is not None and graph.edge_attribute(e, "e_category") not in host_categories:
            return False
        if req_host_pair is not None and _level_pair(e) != req_host_pair:
            return False
        return True

    def _snap_xy(moved_pt, host_line):
        """Project moved_pt XY onto host_line; return the 3D point on the line."""
        h0, h1 = host_line.start, host_line.end
        dx, dy = h1.x - h0.x, h1.y - h0.y
        denom = dx * dx + dy * dy
        if denom < 1e-12:
            return None
        t = ((moved_pt.x - h0.x) * dx + (moved_pt.y - h0.y) * dy) / denom
        return Point(h0.x + t * dx, h0.y + t * dy, h0.z + t * (h1.z - h0.z))

    shifted_lines = []
    processed_edges = []
    untouched_lines = []
    debug_info = []

    for e in graph.edges():
        if not _is_target(e):
            continue

        current_line = _get_line(graph, e)
        if current_line is None:
            continue

        u, v = e
        lu = graph.node_attribute(u, "level")
        lv = graph.node_attribute(v, "level")
        
        if lu == move_node_level:
            moved, fixed = u, v
        elif lv == move_node_level:
            moved, fixed = v, u
        else:
            untouched_lines.append(current_line)
            continue

        moved_pt = current_line.start if moved == u else current_line.end
        fixed_pt = current_line.end if moved == u else current_line.start

        # Edge Z direction: positive = moved end is upper, negative = lower
        prefer_up = moved_pt.z <= fixed_pt.z

        host_candidates = []

        for he in graph.node_edges(moved):
            oriented = _resolve_edge(graph, he)
            if oriented is None or _edge_key(oriented) == _edge_key(e):
                continue
            if not _is_host(oriented):
                continue
            host_line = _get_line(graph, oriented)
            if host_line is None:
                continue

            snap_pt = _snap_xy(moved_pt, host_line)
            if snap_pt is None:
                continue

            dist = _dist(snap_pt, moved_pt)
            host_candidates.append((snap_pt, dist))

        if not host_candidates:
            untouched_lines.append(current_line)
            if debug:
                debug_info.append({"edge": e, "moved": moved, "reason": "no_host_snap"})
            continue

        # Prefer hosts on the matching Z side; fall back to closest overall
        if prefer_up:
            preferred = [(pt, d) for pt, d in host_candidates if pt.z >= moved_pt.z - 1e-6]
        else:
            preferred = [(pt, d) for pt, d in host_candidates if pt.z <= moved_pt.z + 1e-6]

        best_pt = min(preferred or host_candidates, key=lambda x: x[1])[0]

        if snap_8_degree_edges == True:
            if graph.degree(moved) < 8:
                new_pt = Point(best_pt.x, best_pt.y, best_pt.z + (float(z_offset) if prefer_up else -float(z_offset)))
                new_line = Line(new_pt, fixed_pt) if moved == u else Line(fixed_pt, new_pt)
            else:
                new_line = Line(moved_pt, fixed_pt) if moved == u else Line(fixed_pt, moved_pt)
        else:
            new_pt = Point(best_pt.x, best_pt.y, best_pt.z + (float(z_offset) if prefer_up else -float(z_offset)))
            new_line = Line(new_pt, fixed_pt) if moved == u else Line(fixed_pt, new_pt)
            
        oriented = _resolve_edge(graph, e)
        if write_attribute and oriented is not None:
            graph.edge_attribute(oriented, "line", new_line)
            graph.edge_attribute(oriented, "shifted_line", new_line)

        shifted_lines.append(new_line)
        processed_edges.append(oriented if oriented is not None else e)

        if debug:
            debug_info.append({"edge": e, "moved": moved, "prefer_up": prefer_up, "snap_pt": best_pt, "new_pt": new_pt, "host_count": len(host_candidates), "preferred_count": len(preferred)})

    synced_nodes = []
    if write_attribute and processed_edges:
        synced_nodes = _sync_node_points_from_edges(graph, processed_edges)

    return {
        "graph": graph,
        "shifted_lines": shifted_lines,
        "target_edges": processed_edges,
        "untouched_lines": untouched_lines,
        "info": {
            "target_hierarchy": target_hierarchy,
            "host_hierarchy": host_hierarchy,
            "move_node_level": move_node_level,
            "z_offset": float(z_offset),
            "shifted_count": len(processed_edges),
            "untouched_count": len(untouched_lines),
            "synced_node_count": len(synced_nodes),
        },
        "debug": debug_info if debug else None,
    }

def middle_node_solver(graph, t_value=.2, tolerance=1e-4, debug=False):
    """ Target middle node of the module and shift lines along the edge that is coplanar with leaf edge."""
    
    target_nodes = []
    shifted_lines = []
    
    for node in graph.nodes():
        if graph.node_valency(node) >= 8:
            target_nodes.append(node)
    # Step 2: For each target node, find the leaf edge and the coplanar edge, then shift the middle node along the coplanar edge.
    for node in target_nodes:
        leaf_edges = []
        coplanar_edges = []
        for edge in graph.node_edges(node):
            oriented = _resolve_edge(graph, edge)
            if oriented is None:
                continue
            if graph.edge_attribute(oriented, "level") == 0:
                edge_from_node = _edge_node_orientation(graph, oriented, node)
                leaf_edges.append(edge_from_node)
            else:
                # NOTE: orient shifted line acording to the node not edge becaus ewe are using shifted line
                edge_from_node = _edge_node_orientation(graph, oriented, node)
                coplanar_edges.append(edge_from_node)
            # NOTE: Move the primary and main primary lines category to original position...question is where????
        if not leaf_edges or not coplanar_edges:
            continue
        # Find the direction of the coplanar edge  
        
        for leaf in leaf_edges:
            leaf_vec = Vector.from_start_end(_get_point(graph, leaf[0]), _get_point(graph, leaf[1]))
            leaf_vec_xy = Vector(leaf_vec.x, leaf_vec.y, 0.0).unitized()

            
            for coplanar in coplanar_edges:
                cop_vec = Vector.from_start_end(_get_point(graph, coplanar[0]), _get_point(graph, coplanar[1]))
                cop_vec_xy = Vector(cop_vec.x, cop_vec.y, 0.0).unitized()
                dot = round(leaf_vec_xy.dot(cop_vec_xy), 4)
                print(dot)
                if dot == 1.00:  # Parallel and same direction
                    if debug:
                        print(f"DEBUG: Leaf edge {leaf} and Coplanar edge {coplanar} are parallel and same direction (Dot: {dot:.4f})")
                    coplanar_line = _get_line(graph, coplanar)
                    pt = coplanar_line.point_at(t_value)  # Get a point along the coplanar edge at t_value (0.2 means 20% along the edge)
                    u, v = leaf
                    shifted_line = Line(_get_point(graph, v), pt)
                    # graph.edge_attribute(leaf, "line", shifted_line)
                    graph.edge_attribute(leaf, "shifted_line", shifted_line)
                    shifted_lines.append(shifted_line)

    return shifted_lines
            
            # if debug:
                # for coplanar in coplanar_edges:
                #     cop_vec = Vector.from_start_end(_get_point(graph, coplanar[0]), _get_point(graph, coplanar[1]))
                #     cop_vec_xy = Vector(cop_vec.x, cop_vec.y, 0.0).unitized()
                #     dot = leaf_vec_xy.dot(cop_vec_xy)
                #     print(f"DEBUG: Leaf edge {leaf} and Coplanar edge {coplanar} - Dot: {dot:.4f}")
            