"""
Staged reciprocal helpers used by the current GH workflow.

Public API:
- reciprocal_staged_primary_from_subgraph
- reciprocal_diagonal_primary_from_subgraph
"""

import copy

from compas.geometry import Line, Point
from reciprocal import reciprocal_width_from_subgraph


# -----------------------------------------------------------------------------
# Core helpers
# -----------------------------------------------------------------------------

def _clone(graph):
    return copy.deepcopy(graph)


def _edge_key(edge):
    u, v = edge
    return (u, v) if u <= v else (v, u)


def _line_from_edge(graph, edge):
    u, v = edge
    shifted = graph.edge_attribute((u, v), "shifted_lines")
    if shifted is not None:
        return shifted

    line_attr = graph.edge_attribute((u, v), "line")
    if isinstance(line_attr, Line):
        return line_attr
    if line_attr:
        return Line(Point(*line_attr[0]), Point(*line_attr[1]))

    pu = graph.node_attribute(u, "point")
    pv = graph.node_attribute(v, "point")
    return Line(Point(pu.x, pu.y, pu.z), Point(pv.x, pv.y, pv.z))


def _set_node_point(graph, node, pt):
    graph.node_attribute(node, "point", pt)
    graph.node_attribute(node, "x", pt.x)
    graph.node_attribute(node, "y", pt.y)
    graph.node_attribute(node, "z", pt.z)


def _is_primary_orthogonal(edge_hierarchy, edge_category):
    # v4 naming
    if edge_hierarchy == "primary_orthogonal":
        return True
    # v3 naming fallback
    return edge_hierarchy == "primary" and edge_category == "orthogonal"


def _is_primary_diagonal(edge_hierarchy, edge_category):
    # v4 naming
    if isinstance(edge_hierarchy, str) and edge_hierarchy.startswith("primary_diagonal_"):
        return True
    # v3 naming fallback
    return edge_hierarchy == "primary" and edge_category in ("default_diagonal", "moved_diagonal")


def _is_primary_any(edge_hierarchy, edge_category):
    return _is_primary_orthogonal(edge_hierarchy, edge_category) or _is_primary_diagonal(edge_hierarchy, edge_category)


def _select_active_edges(graph, category_set):
    active = set()
    for e in graph.edges():
        h = graph.edge_attribute(e, "hierarchy")
        c = graph.edge_attribute(e, "e_category")

        if "orthogonal" in category_set:
            if _is_primary_orthogonal(h, c):
                active.add(_edge_key(e))
        else:
            if _is_primary_diagonal(h, c):
                active.add(_edge_key(e))
    return active


def _mask_hierarchy(graph, active_keys, demote_to="tertiary"):
    backup = {}
    for e in graph.edges():
        key = _edge_key(e)
        backup[key] = graph.edge_attribute(e, "hierarchy")
        graph.edge_attribute(e, "hierarchy", "primary" if key in active_keys else demote_to)
    return backup


def _restore_hierarchy(graph, backup):
    for e in graph.edges():
        key = _edge_key(e)
        if key in backup:
            graph.edge_attribute(e, "hierarchy", backup[key])


def _run_reciprocal_masked(
    graph,
    active_keys,
    engage_len=0.11,
    tol=0.1,
    rotation_sign=+1,
    min_degree=3,
    iterations=5,
    debug=False,
):
    backup = _mask_hierarchy(graph, active_keys, demote_to="tertiary")
    reciprocal_width_from_subgraph(
        graph,
        engage_len=engage_len,
        tol=tol,
        rotation_sign=rotation_sign,
        min_degree=min_degree,
        iterations=iterations,
        debug=debug,
    )
    _restore_hierarchy(graph, backup)


def _copy_shifted_lines(dst_graph, src_graph):
    for e in dst_graph.edges():
        line = src_graph.edge_attribute(e, "shifted_lines")
        if line is not None:
            dst_graph.edge_attribute(e, "shifted_lines", line)
            dst_graph.edge_attribute(e, "line", line)


def _sync_generated_diagonals_to_nodes(graph, active_keys):
    """
    Make generated diagonal geometry visible to reciprocal solver by syncing nodes.
    """
    for e in graph.edges():
        if _edge_key(e) not in active_keys:
            continue
        gl = graph.edge_attribute(e, "generated_line")
        if gl is None:
            continue

        shared = graph.edge_attribute(e, "generated_shared_node")
        other = graph.edge_attribute(e, "generated_other_node")
        u, v = e

        if shared is not None and graph.has_node(shared):
            _set_node_point(graph, shared, gl.start)
        elif graph.has_node(u):
            _set_node_point(graph, u, gl.start)

        if other is not None and graph.has_node(other):
            _set_node_point(graph, other, gl.end)
        elif graph.has_node(v):
            _set_node_point(graph, v, gl.end)

        graph.edge_attribute(e, "line", gl)
        graph.edge_attribute(e, "shifted_lines", gl)


# -----------------------------------------------------------------------------
# Public API
# -----------------------------------------------------------------------------

def reciprocal_staged_primary_from_subgraph(
    subgraph,
    engage_len=0.11,
    tol=0.1,
    rotation_sign=+1,
    min_degree=3,
    iterations=5,
    debug=False,
):
    """
    Two-pass reciprocal on primaries:
    1) orthogonal
    2) default_diagonal
    """
    working = _clone(subgraph)

    # Stage 1: orthogonal
    s1_graph = _clone(working)
    s1_active = _select_active_edges(s1_graph, {"orthogonal"})
    _run_reciprocal_masked(
        s1_graph, s1_active, engage_len, tol, rotation_sign, min_degree, iterations, debug
    )
    _copy_shifted_lines(working, s1_graph)

    # Stage 2: default diagonal
    s2_graph = _clone(working)
    s2_active = _select_active_edges(s2_graph, {"default_diagonal"})
    _run_reciprocal_masked(
        s2_graph, s2_active, engage_len, tol, rotation_sign, min_degree, iterations, debug
    )
    _copy_shifted_lines(working, s2_graph)

    stage_1_shifted = [_line_from_edge(s1_graph, e) for e in s1_graph.edges() if _edge_key(e) in s1_active]
    stage_2_shifted = [_line_from_edge(s2_graph, e) for e in s2_graph.edges() if _edge_key(e) in s2_active]
    primary_shifted = []
    non_primary = []
    for e in working.edges():
        h = working.edge_attribute(e, "hierarchy")
        c = working.edge_attribute(e, "e_category")
        ln = _line_from_edge(working, e)
        if _is_primary_any(h, c):
            primary_shifted.append(ln)
        else:
            non_primary.append(ln)

    return {
        "graph": working,
        "stage_1": {"graph": s1_graph, "active_edges": sorted(list(s1_active))},
        "stage_2": {"graph": s2_graph, "active_edges": sorted(list(s2_active))},
        "viz": {
            "stage_1_shifted_lines": [ln for ln in stage_1_shifted if ln is not None],
            "stage_2_shifted_lines": [ln for ln in stage_2_shifted if ln is not None],
            "primary_shifted_lines": [ln for ln in primary_shifted if ln is not None],
            "non_primary_lines": [ln for ln in non_primary if ln is not None],
        },
        "info": {
            "structure_preserved": True,
            "stage_1_category": "orthogonal",
            "stage_2_category": "default_diagonal",
            "stage_1_active_edge_count": len(s1_active),
            "stage_2_active_edge_count": len(s2_active),
        },
    }


def reciprocal_diagonal_primary_from_subgraph(
    subgraph,
    engage_len=0.11,
    tol=0.1,
    rotation_sign=+1,
    min_degree=3,
    iterations=5,
    debug=False,
):
    """
    Reciprocal only on primary diagonals.
    Supports both:
    - v3 naming: hierarchy='primary' + diagonal categories
    - v4 naming: hierarchy='primary_diagonal_*'
    """
    working = _clone(subgraph)
    active = _select_active_edges(working, {"default_diagonal", "moved_diagonal"})

    _sync_generated_diagonals_to_nodes(working, active)
    _run_reciprocal_masked(
        working, active, engage_len, tol, rotation_sign, min_degree, iterations, debug
    )

    shifted_diagonal = [_line_from_edge(working, e) for e in working.edges() if _edge_key(e) in active]
    untouched = [_line_from_edge(working, e) for e in working.edges() if _edge_key(e) not in active]

    return {
        "graph": working,
        "shifted_diagonal_lines": [ln for ln in shifted_diagonal if ln is not None],
        "untouched_lines": [ln for ln in untouched if ln is not None],
        "info": {
            "active_diagonal_count": len(active),
            "structure_preserved": True,
        },
    }
