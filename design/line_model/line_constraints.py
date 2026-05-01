"""
Minimal line-constraint helpers for the current workflow.
"""

from compas.geometry import Line, Point, Vector, Plane


def _resolve_edge(graph, edge):
    u, v = edge
    if graph.has_edge((u, v)):
        return (u, v)
    if graph.has_edge((v, u)):
        return (v, u)
    return None


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

    shifted = graph.edge_attribute(oriented, "shifted_lines")
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


def _point_shifted_along_line(point, line, distance):
    if abs(float(distance)) < 1e-12:
        return point
    direction = Vector.from_start_end(line.start, line.end)
    if direction.length < 1e-12:
        return point
    direction.unitize()
    return point + direction * float(distance)


def commit_generated_lines(graph, hierarchy_filter=None, category_filter=None, clear_generated=False):
    committed = []
    if isinstance(category_filter, str):
        category_filter = {category_filter}
    elif category_filter is not None:
        category_filter = set(category_filter)

    def _hierarchy_match(edge_h):
        if hierarchy_filter is None:
            return True
        if edge_h == hierarchy_filter:
            return True
        # v4 compatibility alias: "primary" matches all primary_* buckets
        if hierarchy_filter == "primary":
            return edge_h == "primary_orthogonal" or (isinstance(edge_h, str) and edge_h.startswith("primary_diagonal_"))
        return False

    for e in graph.edges():
        gl = graph.edge_attribute(e, "generated_line")
        if gl is None:
            continue
        if not _hierarchy_match(graph.edge_attribute(e, "hierarchy")):
            continue
        if category_filter is not None and graph.edge_attribute(e, "e_category") not in category_filter:
            continue

        graph.edge_attribute(e, "line", gl)
        graph.edge_attribute(e, "shifted_lines", gl)
        committed.append(e)

        if clear_generated:
            graph.edge_attribute(e, "generated_line", None)
            graph.edge_attribute(e, "generated_shared_node", None)
            graph.edge_attribute(e, "generated_other_node", None)

    return {
        "graph": graph,
        "committed_edges": committed,
        "info": {
            "committed_count": len(committed),
            "hierarchy_filter": hierarchy_filter,
            "category_filter": list(category_filter) if category_filter is not None else None,
            "structure_preserved": True,
        },
    }


def regenerate_edges(
    graph,
    target_hierarchy,
    target_categories=None,
    target_level_pair=None,
    target_nodes=None,
    move_node_level=0,
    host_hierarchy="primary",
    host_categories=None,
    host_level_pair=None,
    choose="up",
    shift_along_host=0.0,
    prefer_on_segment=True,
    write_attribute=True,
    debug=False,
):
    """
    Move one endpoint of each target edge by vertical-plane/host intersections.
    """
    if target_categories is not None:
        if isinstance(target_categories, str):
            target_categories = {target_categories}
        else:
            target_categories = set(target_categories)
    if target_nodes is not None:
        target_nodes = set(target_nodes)
    if host_categories is not None:
        if isinstance(host_categories, str):
            host_categories = {host_categories}
        else:
            host_categories = set(host_categories)

    req_target_pair = tuple(sorted((int(target_level_pair[0]), int(target_level_pair[1])))) if target_level_pair else None
    req_host_pair = tuple(sorted((int(host_level_pair[0]), int(host_level_pair[1])))) if host_level_pair else None

    def edge_level_pair(edge):
        u, v = edge
        return tuple(sorted((graph.node_attribute(u, "level"), graph.node_attribute(v, "level"))))

    def _hierarchy_match(edge_h, wanted_h):
        if edge_h == wanted_h:
            return True
        # v4 compatibility alias
        if wanted_h == "primary":
            return edge_h == "primary_orthogonal" or (isinstance(edge_h, str) and edge_h.startswith("primary_diagonal_"))
        return False

    def is_target(edge):
        h = graph.edge_attribute(edge, "hierarchy")
        if _hierarchy_match(h, target_hierarchy):
            cat = graph.edge_attribute(edge, "e_category")
            if target_categories is None or cat in target_categories:
                if req_target_pair is None or edge_level_pair(edge) == req_target_pair:
                    if target_nodes is None:
                        return True
                    u, v = edge
                    if u in target_nodes or v in target_nodes:
                        return True
        return False

    def is_host(edge):
        h = graph.edge_attribute(edge, "hierarchy")
        if _hierarchy_match(h, host_hierarchy):
            cat = graph.edge_attribute(edge, "e_category")
            if host_categories is None or cat in host_categories:
                if req_host_pair is None or edge_level_pair(edge) == req_host_pair:
                    return True
        return False

    target_edges = []
    generated_lines = []
    untouched_lines = []
    processed_edges = []
    host_debug = []
    probe_lines = []
    selected_points = []
    all_intersection_points = []
    selected_host_lines = []
    intersection_markers = []

    for e in graph.edges():
        if not is_target(e):
            continue
        target_edges.append(e)

        current_line = _get_line(graph, e)
        if current_line is None:
            continue

        u, v = e
        lu = graph.node_attribute(u, "level")
        lv = graph.node_attribute(v, "level")

        if target_nodes is not None:
            if u in target_nodes and lu == move_node_level:
                moved, fixed = u, v
            elif v in target_nodes and lv == move_node_level:
                moved, fixed = v, u
            else:
                untouched_lines.append(current_line)
                if debug:
                    host_debug.append({"edge": e, "reason": "target_nodes_level_mismatch"})
                continue
        else:
            if lu == move_node_level:
                moved, fixed = u, v
            elif lv == move_node_level:
                moved, fixed = v, u
            else:
                untouched_lines.append(current_line)
                if debug:
                    host_debug.append({"edge": e, "reason": "move_node_level_not_found", "levels": (lu, lv)})
                continue

        if moved == u:
            moved_pt = current_line.start
            fixed_pt = current_line.end
        else:
            moved_pt = current_line.end
            fixed_pt = current_line.start

        axis = Vector.from_start_end(moved_pt, fixed_pt)
        if axis.length < 1e-9:
            axis = Vector(1, 0, 0)

        plane = Plane.from_point_and_two_vectors(moved_pt, axis, Vector(0, 0, 1))
        probe_line = Line(moved_pt, Point(moved_pt.x, moved_pt.y, moved_pt.z + 1.0))

        intersections = []
        host_candidates = []
        for he in graph.node_edges(moved):
            oriented = _resolve_edge(graph, he)
            if oriented is None:
                continue
            if _edge_key(oriented) == _edge_key(e):
                continue
            if not is_host(oriented):
                continue

            host_candidates.append(oriented)
            host_line = _get_line(graph, oriented)
            if host_line is None:
                continue

            ipt = plane.intersection_with_line(host_line, tol=1e-6)
            if ipt is None:
                continue

            t_host, on_seg = _line_param_and_on_segment(host_line, ipt)
            intersections.append((ipt, oriented, t_host, on_seg))

        for p, _host, _t, _on_seg in intersections:
            all_intersection_points.append(p)
            intersection_markers.append(Line(moved_pt, p))

        if not intersections:
            untouched_lines.append(current_line)
            if debug:
                host_debug.append({
                    "edge": e,
                    "moved_node": moved,
                    "fixed_node": fixed,
                    "host_candidates": host_candidates,
                    "selected_host": None,
                    "intersection_count": 0,
                })
            continue

        candidates = intersections
        if prefer_on_segment:
            in_segment = [it for it in intersections if it[3]]
            if in_segment:
                candidates = in_segment

        if str(choose).lower() == "down":
            ipt, selected_host, _sel_t, _sel_on_seg = min(candidates, key=lambda p: p[0].z)
        else:
            ipt, selected_host, _sel_t, _sel_on_seg = max(candidates, key=lambda p: p[0].z)

        selected_host_line = _get_line(graph, selected_host)
        if selected_host_line is not None:
            ipt = _point_shifted_along_line(ipt, selected_host_line, shift_along_host)

        new_line = Line(ipt, fixed_pt) if moved == u else Line(fixed_pt, ipt)

        if write_attribute:
            oriented = _resolve_edge(graph, e)
            if oriented is not None:
                graph.edge_attribute(oriented, "generated_line", new_line)
                graph.edge_attribute(oriented, "generated_shared_node", int(moved))
                graph.edge_attribute(oriented, "generated_other_node", int(fixed))

        generated_lines.append(new_line)
        processed_edges.append(e)
        probe_lines.append(probe_line)
        selected_points.append(ipt)
        sh = selected_host_line
        if sh is not None:
            selected_host_lines.append(sh)
            host_t, host_on_seg = _line_param_and_on_segment(sh, ipt)
        else:
            host_t, host_on_seg = None, None

        if debug:
            host_debug.append({
                "edge": e,
                "moved_node": moved,
                "fixed_node": fixed,
                "host_candidates": host_candidates,
                "selected_host": selected_host,
                "intersection_count": len(intersections),
                "selected_point": ipt,
                "moved_node_point": moved_pt,
                "intersection_points": [p[0] for p in intersections],
                "intersection_host_t": [p[2] for p in intersections],
                "intersection_on_segment": [p[3] for p in intersections],
                "selected_host_t": host_t,
                "selected_host_on_segment": host_on_seg,
            })

    return {
        "graph": graph,
        "target_edges": target_edges,
        "generated_lines": generated_lines,
        
        "info": {
            "target_hierarchy": target_hierarchy,
            "target_categories": list(target_categories) if target_categories is not None else None,
            "target_level_pair": req_target_pair,
            "target_nodes": list(target_nodes) if target_nodes is not None else None,
            "move_node_level": int(move_node_level),
            "host_hierarchy": host_hierarchy,
            "host_categories": list(host_categories) if host_categories is not None else None,
            "host_level_pair": req_host_pair,
            "choose": str(choose).lower(),
            "shift_along_host": float(shift_along_host),
            "prefer_on_segment": bool(prefer_on_segment),
            "target_count": len(target_edges),
            "generated_count": len(processed_edges),
            "untouched_count": len(untouched_lines),
            "structure_preserved": True,
        },
        "host_debug": host_debug if debug else None,
    }

