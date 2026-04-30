"""
Line-to-line constraint system (standalone prototype).

Purpose
-------
Store endpoint intent as constraints (fixed anchors or relative-on-line),
then reevaluate geometry after host lines move.

This file does NOT modify existing reciprocal or shifting solvers.
"""

from compas.geometry import Line, Point, Vector


# -----------------------------------------------------------------------------
# Edge / line helpers
# -----------------------------------------------------------------------------

def _resolve_edge(graph, edge):
    """Return oriented edge tuple existing in graph, else None."""
    u, v = edge
    if graph.has_edge((u, v)):
        return (u, v)
    if graph.has_edge((v, u)):
        return (v, u)
    return None


def _edge_key(edge):
    """Canonical key for undirected edge lookup."""
    u, v = edge
    return (u, v) if u <= v else (v, u)


def _get_point(graph, node):
    pt = graph.node_attribute(node, "point")
    return Point(pt.x, pt.y, pt.z)


def _set_point(graph, node, pt):
    graph.node_attribute(node, "point", pt)
    graph.node_attribute(node, "x", pt.x)
    graph.node_attribute(node, "y", pt.y)
    graph.node_attribute(node, "z", pt.z)


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


def _set_line(graph, edge, line):
    oriented = _resolve_edge(graph, edge)
    if oriented is None:
        return
    graph.edge_attribute(oriented, "shifted_lines", line)
    graph.edge_attribute(oriented, "line", line)


def _line_frame(line):
    """
    Build a local frame from line:
    t-axis along line, n-axis horizontal normal, b-axis binormal.
    """
    t = Vector.from_start_end(line.start, line.end)
    if t.length < 1e-9:
        return None
    t.unitize()

    z = Vector(0, 0, 1)
    n = t.cross(z)
    if n.length < 1e-9:
        n = t.cross(Vector(1, 0, 0))
    if n.length < 1e-9:
        return None
    n.unitize()

    b = n.cross(t)
    if b.length < 1e-9:
        return None
    b.unitize()
    return t, n, b


# -----------------------------------------------------------------------------
# Constraint schema
# -----------------------------------------------------------------------------

"""
Per edge endpoint attribute keys:

edge_constraint_start = {
    "type": "fixed" | "on_line",
    "fixed_point": [x, y, z],                 # for fixed
    "host_edge": [u, v],                      # for on_line
    "host_t": 0.0..1.0,                       # for on_line
    "offset_t": float,                        # local line tangent offset
    "offset_n": float,                        # local line normal offset
    "offset_b": float,                        # local line binormal offset
}

edge_constraint_end = { ... same schema ... }
"""


def make_fixed_constraint(pt):
    """Create fixed endpoint constraint."""
    return {
        "type": "fixed",
        "fixed_point": [pt.x, pt.y, pt.z],
        "host_edge": None,
        "host_t": None,
        "offset_t": 0.0,
        "offset_n": 0.0,
        "offset_b": 0.0,
    }


def make_on_line_constraint(host_edge, host_t, offset_t=0.0, offset_n=0.0, offset_b=0.0):
    """Create dependent endpoint constraint on another line."""
    return {
        "type": "on_line",
        "fixed_point": None,
        "host_edge": [host_edge[0], host_edge[1]],
        "host_t": float(host_t),
        "offset_t": float(offset_t),
        "offset_n": float(offset_n),
        "offset_b": float(offset_b),
    }


def set_edge_endpoint_constraint(graph, edge, which_end, constraint):
    """
    Assign endpoint constraint to edge.

    which_end: "start" or "end"
    """
    oriented = _resolve_edge(graph, edge)
    if oriented is None:
        raise KeyError("Edge not found: {}".format(edge))

    attr_name = "edge_constraint_start" if which_end == "start" else "edge_constraint_end"
    graph.edge_attribute(oriented, attr_name, constraint)


def get_edge_endpoint_constraint(graph, edge, which_end):
    oriented = _resolve_edge(graph, edge)
    if oriented is None:
        return None
    attr_name = "edge_constraint_start" if which_end == "start" else "edge_constraint_end"
    return graph.edge_attribute(oriented, attr_name)


# -----------------------------------------------------------------------------
# Constraint evaluation
# -----------------------------------------------------------------------------

def _eval_constraint_point(graph, constraint):
    """Compute point from one constraint."""
    if not constraint:
        return None

    ctype = constraint.get("type")
    if ctype == "fixed":
        fp = constraint.get("fixed_point")
        if not fp:
            return None
        return Point(fp[0], fp[1], fp[2])

    if ctype == "on_line":
        host = constraint.get("host_edge")
        if not host:
            return None
        host_edge = (host[0], host[1])
        host_line = _get_line(graph, host_edge)
        if host_line is None:
            return None

        t = float(constraint.get("host_t", 0.0))
        t = max(0.0, min(1.0, t))
        base = host_line.point_at(t)

        frame = _line_frame(host_line)
        if frame is None:
            return base
        tangent, normal, binormal = frame

        dt = float(constraint.get("offset_t", 0.0))
        dn = float(constraint.get("offset_n", 0.0))
        db = float(constraint.get("offset_b", 0.0))

        move = tangent * dt + normal * dn + binormal * db
        return Point(base.x + move.x, base.y + move.y, base.z + move.z)

    return None


def _edge_has_constraints(graph, edge):
    c0 = get_edge_endpoint_constraint(graph, edge, "start")
    c1 = get_edge_endpoint_constraint(graph, edge, "end")
    return c0 is not None or c1 is not None


def evaluate_constraints_once(graph):
    """
    Single pass over constrained edges:
    updates line geometry and node points from endpoint constraints.
    """
    moved = 0
    for edge in list(graph.edges()):
        if not _edge_has_constraints(graph, edge):
            continue

        oriented = _resolve_edge(graph, edge)
        if oriented is None:
            continue
        u, v = oriented

        current = _get_line(graph, oriented)
        if current is None:
            continue

        c_start = get_edge_endpoint_constraint(graph, oriented, "start")
        c_end = get_edge_endpoint_constraint(graph, oriented, "end")

        p0 = _eval_constraint_point(graph, c_start) if c_start else current.start
        p1 = _eval_constraint_point(graph, c_end) if c_end else current.end

        if p0 is None:
            p0 = current.start
        if p1 is None:
            p1 = current.end

        new_line = Line(p0, p1)
        _set_line(graph, oriented, new_line)

        # Push geometry to nodes too (keeps graph coherent for downstream tools)
        _set_point(graph, u, new_line.start)
        _set_point(graph, v, new_line.end)
        moved += 1

    return moved


def evaluate_constraints_iterative(graph, max_iter=20, tol=1e-4):
    """
    Iterative solve for dependency chains/cycles.

    Returns
    -------
    dict: {graph, iterations, converged, moved_edges}
    """
    prev_positions = {}
    converged = False
    total_moved = 0

    for i in range(max_iter):
        moved_now = evaluate_constraints_once(graph)
        total_moved += moved_now

        max_delta = 0.0
        for n in graph.nodes():
            pt = graph.node_attribute(n, "point")
            if pt is None:
                continue
            cur = (pt.x, pt.y, pt.z)
            old = prev_positions.get(n)
            if old is not None:
                dx = cur[0] - old[0]
                dy = cur[1] - old[1]
                dz = cur[2] - old[2]
                d = (dx * dx + dy * dy + dz * dz) ** 0.5
                if d > max_delta:
                    max_delta = d
            prev_positions[n] = cur

        if max_delta < tol:
            converged = True
            return {
                "graph": graph,
                "iterations": i + 1,
                "converged": converged,
                "moved_edges": total_moved,
            }

    return {
        "graph": graph,
        "iterations": max_iter,
        "converged": converged,
        "moved_edges": total_moved,
    }


# -----------------------------------------------------------------------------
#  builder 
# -----------------------------------------------------------------------------

def bind_level0_secondary_to_primary(graph, default_t=0.8, offset_b=0.0):
    """
    Auto-assign constraints:
    - For level-0 secondary edges:
      one end remains fixed at its current point (anchor),
      the other end depends on a connected primary line at parameter default_t.

    Heuristic:
    - anchor node is the endpoint that has primary neighbors.
    - dependent endpoint is the opposite end.

    Returns
    -------
    dict with counts and preview lines.
    """
    bound_edges = []

    for edge in list(graph.edges()):
        if graph.edge_attribute(edge, "hierarchy") != "secondary":
            continue
        u, v = edge
        lu = graph.node_attribute(u, "level")
        lv = graph.node_attribute(v, "level")
        if lu != 0 or lv != 0:
            continue

        u_primary = [e for e in graph.node_edges(u) if graph.edge_attribute(e, "hierarchy") == "primary"]
        v_primary = [e for e in graph.node_edges(v) if graph.edge_attribute(e, "hierarchy") == "primary"]
        if not u_primary and not v_primary:
            continue

        if u_primary and not v_primary:
            anchor, dep = u, v
            host = u_primary[0]
            start_is_anchor = True
        elif v_primary and not u_primary:
            anchor, dep = v, u
            host = v_primary[0]
            start_is_anchor = False
        else:
            # both sides have primary: choose side with more options
            if len(u_primary) >= len(v_primary):
                anchor, dep = u, v
                host = u_primary[0]
                start_is_anchor = True
            else:
                anchor, dep = v, u
                host = v_primary[0]
                start_is_anchor = False

        c_anchor = make_fixed_constraint(_get_point(graph, anchor))
        c_dep = make_on_line_constraint(host_edge=host, host_t=default_t, offset_b=offset_b)

        if start_is_anchor:
            set_edge_endpoint_constraint(graph, edge, "start", c_anchor)
            set_edge_endpoint_constraint(graph, edge, "end", c_dep)
        else:
            set_edge_endpoint_constraint(graph, edge, "start", c_dep)
            set_edge_endpoint_constraint(graph, edge, "end", c_anchor)

        bound_edges.append(edge)

    preview = []
    for e in bound_edges:
        ln = _get_line(graph, e)
        if ln is not None:
            preview.append(ln)

    return {
        "graph": graph,
        "bound_secondary_edges": bound_edges,
        "bound_count": len(bound_edges),
        "preview_lines": preview,
    }


def _line_point_at_from_shared_node(host_line, host_edge, shared_node, t):
    """
    Evaluate host line at t measured AWAY from the shared node.

    If shared node is host start -> point_at(t)
    If shared node is host end   -> point_at(1-t)
    """
    u, v = host_edge
    tt = max(0.0, min(1.0, float(t)))
    if shared_node == u:
        return host_line.point_at(tt)
    if shared_node == v:
        return host_line.point_at(1.0 - tt)
    return host_line.point_at(tt)


def regenerate_level0_secondary_from_primary_t(
    graph,
    t=0.8,
    write_attribute=True,
):
    """
    Rebuild level-0 secondary lines from neighboring level-0 primary lines.

    Important:
    - Does NOT move nodes.
    - Secondary endpoint at shared node is replaced by point_at(t) on host primary.
    - Other endpoint stays at the original secondary other node point.
    - Recompute anytime after primary lines move.

    Returns
    -------
    dict
        {
            "graph": graph,
            "generated_secondary_lines": list[Line],   # <-- use as output b
            "untouched_secondary_lines": list[Line],
            "host_primary_lines": list[Line],
            "info": {...}
        }
    """
    generated_secondary_lines = []
    untouched_secondary_lines = []
    host_primary_lines = []
    processed = 0
    generated_edge_keys = set()
    level0_secondary_edges = []

    # Cache primary line geometry for visualization
    for pe in graph.edges():
        if graph.edge_attribute(pe, "hierarchy") == "primary":
            pl = _get_line(graph, pe)
            if pl is not None:
                host_primary_lines.append(pl)

    for e in graph.edges():
        if graph.edge_attribute(e, "hierarchy") != "secondary":
            continue
        u, v = e
        if graph.node_attribute(u, "level") != 0 or graph.node_attribute(v, "level") != 0:
            continue
        level0_secondary_edges.append(e)

        # Candidate host primaries from same node, also level 0
        u_hosts = []
        for he in graph.node_edges(u):
            if graph.edge_attribute(he, "hierarchy") != "primary":
                continue
            hu, hv = he
            if graph.node_attribute(hu, "level") == 0 and graph.node_attribute(hv, "level") == 0:
                u_hosts.append(he)

        v_hosts = []
        for he in graph.node_edges(v):
            if graph.edge_attribute(he, "hierarchy") != "primary":
                continue
            hu, hv = he
            if graph.node_attribute(hu, "level") == 0 and graph.node_attribute(hv, "level") == 0:
                v_hosts.append(he)

        # Choose shared node side that has host primary
        if u_hosts and not v_hosts:
            shared = u
            other = v
            host_edge = u_hosts[0]
        elif v_hosts and not u_hosts:
            shared = v
            other = u
            host_edge = v_hosts[0]
        elif u_hosts and v_hosts:
            # deterministic choice: node with more host candidates
            if len(u_hosts) >= len(v_hosts):
                shared = u
                other = v
                host_edge = u_hosts[0]
            else:
                shared = v
                other = u
                host_edge = v_hosts[0]
        else:
            untouched_secondary_lines.append(_get_line(graph, e))
            continue

        host_line = _get_line(graph, host_edge)
        if host_line is None:
            untouched_secondary_lines.append(_get_line(graph, e))
            continue

        p_shared_new = _line_point_at_from_shared_node(host_line, host_edge, shared, t)
        p_other = _get_point(graph, other)
        new_line = Line(p_shared_new, p_other)

        if write_attribute:
            oriented = _resolve_edge(graph, e)
            if oriented is not None:
                graph.edge_attribute(oriented, "generated_line", new_line)

        generated_secondary_lines.append(new_line)
        generated_edge_keys.add(_edge_key(e))
        processed += 1

    # Build untouched list by edge membership (robust), not line-geometry equality
    for e in level0_secondary_edges:
        if _edge_key(e) in generated_edge_keys:
            continue
        ln = _get_line(graph, e)
        if ln is not None:
            untouched_secondary_lines.append(ln)

    return {
        "graph": graph,
        "generated_secondary_lines": generated_secondary_lines,
        "untouched_secondary_lines": untouched_secondary_lines,
        "host_primary_lines": host_primary_lines,
        "level0_secondary_lines": [_get_line(graph, e) for e in level0_secondary_edges if _get_line(graph, e) is not None],
        "info": {
            "level0_secondary_count": len(level0_secondary_edges),
            "secondary_generated_count": processed,
            "secondary_untouched_count": len(untouched_secondary_lines),
            "t_value": float(t),
            "structure_preserved": True,
        },
    }


def get_secondary_lines_by_level(graph, level=1, use_generated=True):
    """
    Collect all secondary lines whose two endpoint nodes are in the given level.

    Parameters
    ----------
    graph : NodeGraph
    level : int
        Node level filter (both endpoints must match).
    use_generated : bool
        If True, prefer edge attr 'generated_line' when available.

    Returns
    -------
    dict
        {
            "edges": list[(u,v)],
            "lines": list[Line],
            "info": {...}
        }
    """
    edges = []
    lines = []

    for e in graph.edges():
        if graph.edge_attribute(e, "hierarchy") != "secondary":
            continue
        u, v = e
        if graph.node_attribute(u, "level") != level or graph.node_attribute(v, "level") != level:
            continue

        ln = None
        if use_generated:
            ln = graph.edge_attribute(e, "generated_line")
        if ln is None:
            ln = _get_line(graph, e)
        if ln is None:
            continue

        edges.append(e)
        lines.append(ln)

    return {
        "edges": edges,
        "lines": lines,
        "info": {
            "level": int(level),
            "count": len(lines),
        },
    }


def commit_generated_lines(
    graph,
    hierarchy_filter=None,
    category_filter=None,
    clear_generated=False,
):
    """
    Promote edge attr 'generated_line' into active geometry attrs:
    - line
    - shifted_lines

    This updates geometry only. Graph structure and classification logic remain intact:
    - same nodes/edges
    - same hierarchy / e_category attributes

    Parameters
    ----------
    graph : NodeGraph
    hierarchy_filter : str or None
        Optional filter, e.g. "secondary" or "primary".
    category_filter : str or tuple/list/set or None
        Optional e_category filter, e.g. "default_diagonal" or ("default_diagonal", "moved_diagonal").
    clear_generated : bool
        If True, remove generated_line after commit.

    Returns
    -------
    dict
        {"graph": graph, "committed_edges": list[(u,v)], "info": {...}}
    """
    committed = []

    if isinstance(category_filter, str):
        category_filter = {category_filter}
    elif category_filter is not None:
        category_filter = set(category_filter)

    for e in graph.edges():
        gl = graph.edge_attribute(e, "generated_line")
        if gl is None:
            continue

        if hierarchy_filter is not None:
            if graph.edge_attribute(e, "hierarchy") != hierarchy_filter:
                continue

        if category_filter is not None:
            if graph.edge_attribute(e, "e_category") not in category_filter:
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


def regenerate_tertiary_01_from_neighbor_hosts(
    graph,
    t_level0=0.2,
    t_level1=0.2,
    host_hierarchy="primary",
    host_categories=None,
    write_attribute=True,
):
    """
    Regenerate tertiary edges connecting levels (0,1) from host lines at BOTH ends.

    For each tertiary edge with endpoint levels {0,1}:
    - at the level-0 node, pick one neighboring host edge and evaluate point_at(t_level0)
    - at the level-1 node, pick one neighboring host edge and evaluate point_at(t_level1)
    - new tertiary line = Line(p_from_level0_host, p_from_level1_host)

    This keeps graph structure/categories intact and only writes generated geometry.

    Parameters
    ----------
    graph : NodeGraph
    t_level0 : float
        t value for endpoint attached to level-0 node.
    t_level1 : float
        t value for endpoint attached to level-1 node.
    host_hierarchy : str
        Host edge hierarchy filter (default "primary").
    host_categories : iterable[str] or None
        Optional host e_category filter. Example:
        ("orthogonal", "default_diagonal", "moved_diagonal")
    write_attribute : bool
        If True writes edge attr "generated_line" and endpoint metadata.

    Returns
    -------
    dict
        {
            "graph": graph,
            "generated_lines": list[Line],
            "untouched_lines": list[Line],
            "processed_edges": list[(u,v)],
            "info": {...}
        }
    """
    if host_categories is not None:
        host_categories = set(host_categories)

    def _is_host(edge):
        if graph.edge_attribute(edge, "hierarchy") != host_hierarchy:
            return False
        if host_categories is None:
            return True
        return graph.edge_attribute(edge, "e_category") in host_categories

    def _pick_host_at_node(node, exclude_edge):
        hosts = []
        for he in graph.node_edges(node):
            oriented = _resolve_edge(graph, he)
            if oriented is None:
                continue
            if _edge_key(oriented) == _edge_key(exclude_edge):
                continue
            if _is_host(oriented):
                hosts.append(oriented)
        return hosts[0] if hosts else None

    generated_lines = []
    untouched_lines = []
    processed_edges = []

    for e in graph.edges():
        if graph.edge_attribute(e, "hierarchy") != "tertiary":
            continue

        u, v = e
        lu = graph.node_attribute(u, "level")
        lv = graph.node_attribute(v, "level")
        if set((lu, lv)) != {0, 1}:
            continue

        node_l0 = u if lu == 0 else v
        node_l1 = u if lu == 1 else v

        host0 = _pick_host_at_node(node_l0, e)
        host1 = _pick_host_at_node(node_l1, e)
        if host0 is None or host1 is None:
            ln = _get_line(graph, e)
            if ln is not None:
                untouched_lines.append(ln)
            continue

        host0_line = _get_line(graph, host0)
        host1_line = _get_line(graph, host1)
        if host0_line is None or host1_line is None:
            ln = _get_line(graph, e)
            if ln is not None:
                untouched_lines.append(ln)
            continue

        p0 = _line_point_at_from_shared_node(host0_line, host0, node_l0, t_level0)
        p1 = _line_point_at_from_shared_node(host1_line, host1, node_l1, t_level1)
        new_line = Line(p0, p1)

        if write_attribute:
            oriented = _resolve_edge(graph, e)
            if oriented is not None:
                graph.edge_attribute(oriented, "generated_line", new_line)
                graph.edge_attribute(oriented, "generated_shared_node", int(node_l0))
                graph.edge_attribute(oriented, "generated_other_node", int(node_l1))

        generated_lines.append(new_line)
        processed_edges.append(e)

    return {
        "graph": graph,
        "generated_lines": generated_lines,
        "untouched_lines": untouched_lines,
        "processed_edges": processed_edges,
        "info": {
            "tertiary_01_count": len(processed_edges) + len(untouched_lines),
            "generated_count": len(processed_edges),
            "untouched_count": len(untouched_lines),
            "t_level0": float(t_level0),
            "t_level1": float(t_level1),
            "host_hierarchy": host_hierarchy,
            "host_categories": list(host_categories) if host_categories is not None else None,
            "structure_preserved": True,
        },
    }


def regenerate_primary_diagonals_from_orthogonal_nodes(
    graph,
    t=0.8,
    level=None,
    write_attribute=True,
    target_node=None,
    apply_all_in_level=True,
    required_diagonal_count=None,
    required_orthogonal_count=None,
):
    """
    Regenerate primary diagonal lines from neighboring primary orthogonal lines.

    Rule:
    - At nodes where primary orthogonal and primary diagonal meet (same node),
      use the orthogonal line as host.
    - For each connected primary diagonal line, replace the shared-node endpoint by
      host_orthogonal.point_at(t) measured away from that shared node.
    - Keep the opposite diagonal endpoint unchanged.

    Typical use order:
    1) reciprocal on orthogonals
    2) call this function (diagonals become dependent on orthogonals)
    3) reciprocal on diagonals

    Parameters
    ----------
    graph : NodeGraph
    t : float
        Parameter on host orthogonal line (0..1).
    level : int or None
        If provided, only process nodes/edges where both endpoints match this level.
        If None, auto-detect all nodes that have both primary orthogonal and primary diagonal.
    target_node : int or None
        If provided, only this node is processed (must satisfy mixed primary condition).
    apply_all_in_level : bool
        If True and target_node is None, apply to all matching nodes in the selected level.
    required_diagonal_count : int or None
        If set, keep only nodes with exactly this number of candidate primary diagonal edges.
    required_orthogonal_count : int or None
        If set, keep only nodes with exactly this number of candidate primary orthogonal edges.
    write_attribute : bool
        If True writes generated diagonal geometry into edge attr "generated_line".

    Returns
    -------
    dict
        {
            "graph": graph,
            "generated_diagonal_lines": list[Line],
            "untouched_diagonal_lines": list[Line],
            "host_orthogonal_lines": list[Line],
            "processed_nodes": list[int],
            "info": {...}
        }
    """
    generated_diagonal_lines = []
    untouched_diagonal_lines = []
    host_orthogonal_lines = []
    processed_nodes = []

    def is_primary_orthogonal(edge):
        return (
            graph.edge_attribute(edge, "hierarchy") == "primary"
            and graph.edge_attribute(edge, "e_category") == "orthogonal"
        )

    def is_primary_diagonal(edge):
        return (
            graph.edge_attribute(edge, "hierarchy") == "primary"
            and graph.edge_attribute(edge, "e_category") in ("default_diagonal", "moved_diagonal")
        )

    candidate_nodes = []

    for node in graph.nodes():
        if level is not None and graph.node_attribute(node, "level") != level:
            continue

        node_edges = list(graph.node_edges(node))
        ortho_edges = []
        diag_edges = []

        for e in node_edges:
            oriented = _resolve_edge(graph, e)
            if oriented is None:
                continue
            u, v = oriented
            if is_primary_orthogonal(oriented):
                ortho_edges.append(oriented)
            elif is_primary_diagonal(oriented):
                diag_edges.append(oriented)

        if not ortho_edges or not diag_edges:
            continue

        candidate_nodes.append((node, ortho_edges, diag_edges))

    # Optional strict filtering by local topology counts
    if required_diagonal_count is not None:
        candidate_nodes = [c for c in candidate_nodes if len(c[2]) == int(required_diagonal_count)]
    if required_orthogonal_count is not None:
        candidate_nodes = [c for c in candidate_nodes if len(c[1]) == int(required_orthogonal_count)]

    # Mode 1: explicit node target
    if target_node is not None:
        candidate_nodes = [c for c in candidate_nodes if c[0] == target_node]
    # Mode 2: apply to all in level (default)
    elif not apply_all_in_level:
        candidate_nodes = []

    for node, ortho_edges, diag_edges in candidate_nodes:
        # Use one orthogonal host per node (deterministic first)
        host_edge = ortho_edges[0]
        host_line = _get_line(graph, host_edge)
        if host_line is None:
            continue
        host_orthogonal_lines.append(host_line)
        processed_nodes.append(node)

        for de in diag_edges:
            du, dv = de
            other = dv if du == node else du
            if other == node:
                untouched_diagonal_lines.append(_get_line(graph, de))
                continue

            p_shared_new = _line_point_at_from_shared_node(host_line, host_edge, node, t)
            p_other = _get_point(graph, other)
            new_line = Line(p_shared_new, p_other)

            if write_attribute:
                graph.edge_attribute(de, "generated_line", new_line)
                graph.edge_attribute(de, "generated_shared_node", int(node))
                graph.edge_attribute(de, "generated_other_node", int(other))

            generated_diagonal_lines.append(new_line)

    # collect diagonals that were not regenerated
    if write_attribute:
        for e in graph.edges():
            if not is_primary_diagonal(e):
                continue
            u, v = e
            if graph.edge_attribute(e, "generated_line") is None:
                untouched_diagonal_lines.append(_get_line(graph, e))

    return {
        "graph": graph,
        "generated_diagonal_lines": generated_diagonal_lines,
        "untouched_diagonal_lines": untouched_diagonal_lines,
        "host_orthogonal_lines": host_orthogonal_lines,
        "processed_nodes": processed_nodes,
        "info": {
            "level": level,
            "t_value": float(t),
            "target_node": target_node,
            "apply_all_in_level": bool(apply_all_in_level),
            "required_diagonal_count": required_diagonal_count,
            "required_orthogonal_count": required_orthogonal_count,
            "processed_node_count": len(processed_nodes),
            "generated_diagonal_count": len(generated_diagonal_lines),
            "untouched_diagonal_count": len(untouched_diagonal_lines),
            "structure_preserved": True,
        },
    }
