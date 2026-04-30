"""
Staged reciprocal shifting utilities.

This module keeps the original graph topology untouched while applying
reciprocal shifts in two passes:
1) primary + orthogonal
2) primary + default_diagonal (original grid diagonals)

It does NOT modify reciprocal.py.
"""

import copy

from compas.geometry import Line, Point
from reciprocal import reciprocal_width_from_subgraph


# -----------------------------------------------------------------------------
# Graph helpers (structure-preserving)
# -----------------------------------------------------------------------------

def clone_graph_structure(graph):
    """Deep-copy a graph so node/edge structure and attributes are preserved."""
    return copy.deepcopy(graph)


def _edge_key(u, v):
    """Canonical undirected key for edge matching."""
    return (u, v) if u <= v else (v, u)


def _resolve_edge(graph, edge):
    """Return an oriented edge tuple that exists in the graph, or None."""
    u, v = edge
    if graph.has_edge((u, v)):
        return (u, v)
    if graph.has_edge((v, u)):
        return (v, u)
    return None


def _build_active_edge_keys(graph, target_category):
    """
    Build set of edges that are both primary and in the requested category.

    Parameters
    ----------
    graph : NodeGraph
    target_category : str
        Usually "orthogonal" or "default_diagonal".
    """
    active = set()
    for u, v in graph.edges():
        hierarchy = graph.edge_attribute((u, v), "hierarchy")
        category = graph.edge_attribute((u, v), "e_category")
        if hierarchy == "primary" and category == target_category:
            active.add(_edge_key(u, v))
    return active


def _mask_hierarchy_for_stage(graph, active_edge_keys, demote_to="tertiary"):
    """
    Keep hierarchy='primary' only on active edges, demote all others for this stage.

    Returns
    -------
    dict
        Backup of original hierarchy by canonical edge key.
    """
    backup = {}
    for u, v in graph.edges():
        key = _edge_key(u, v)
        original = graph.edge_attribute((u, v), "hierarchy")
        backup[key] = original

        if key not in active_edge_keys:
            graph.edge_attribute((u, v), "hierarchy", demote_to)
        else:
            graph.edge_attribute((u, v), "hierarchy", "primary")

    return backup


def _restore_hierarchy(graph, hierarchy_backup):
    """Restore hierarchy attributes from backup."""
    for u, v in graph.edges():
        key = _edge_key(u, v)
        if key in hierarchy_backup:
            graph.edge_attribute((u, v), "hierarchy", hierarchy_backup[key])


def _apply_stage_shifted_lines(target_graph, stage_graph):
    """
    Copy shifted lines from stage graph to target graph edge attributes.

    Writes both:
    - shifted_lines
    - line
    """
    for u, v in target_graph.edges():
        line = stage_graph.edge_attribute((u, v), "shifted_lines")
        if line is None:
            continue
        target_graph.edge_attribute((u, v), "shifted_lines", line)
        target_graph.edge_attribute((u, v), "line", line)


def _line_from_edge(graph, edge):
    """Resolve best available line geometry for an edge."""
    oriented = _resolve_edge(graph, edge)
    if oriented is None:
        return None

    u, v = oriented
    shifted = graph.edge_attribute(oriented, "shifted_lines")
    if shifted is not None:
        return shifted

    line = graph.edge_attribute(oriented, "line")
    if isinstance(line, Line):
        return line
    if line:
        return Line(Point(*line[0]), Point(*line[1]))

    pu = graph.node_attribute(u, "point")
    pv = graph.node_attribute(v, "point")
    return Line(Point(pu.x, pu.y, pu.z), Point(pv.x, pv.y, pv.z))


def _set_node_point(graph, node, pt):
    """Update node coordinates consistently."""
    graph.node_attribute(node, "point", pt)
    graph.node_attribute(node, "x", pt.x)
    graph.node_attribute(node, "y", pt.y)
    graph.node_attribute(node, "z", pt.z)


# -----------------------------------------------------------------------------
# Stage runner
# -----------------------------------------------------------------------------

def _run_single_stage(
    base_graph,
    target_category,
    engage_len=0.11,
    tol=0.1,
    rotation_sign=+1,
    min_degree=3,
    iterations=5,
    debug=False,
):
    """
    Run one reciprocal stage on a cloned graph while preserving topology.

    Returns
    -------
    dict
        {
            "graph": stage_graph,
            "active_edges": list[(u,v)],
            "result": reciprocal_result,
            "target_category": str,
        }
    """
    stage_graph = clone_graph_structure(base_graph)

    active_keys = _build_active_edge_keys(stage_graph, target_category)
    backup = _mask_hierarchy_for_stage(stage_graph, active_keys, demote_to="tertiary")

    result = reciprocal_width_from_subgraph(
        stage_graph,
        engage_len=engage_len,
        tol=tol,
        rotation_sign=rotation_sign,
        min_degree=min_degree,
        iterations=iterations,
        debug=debug,
    )

    # Keep shifted_lines that were written by reciprocal solver,
    # but restore original hierarchy labels for consistency.
    _restore_hierarchy(stage_graph, backup)

    return {
        "graph": stage_graph,
        "active_edges": sorted(list(active_keys)),
        "result": result,
        "target_category": target_category,
    }


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
    Two-step staged reciprocal for primary edges while keeping graph structure.

    Stage 1:
        primary + orthogonal
    Stage 2:
        primary + default_diagonal

    Parameters
    ----------
    subgraph : NodeGraph
        Input graph. Not modified in-place.

    Returns
    -------
    dict with keys:
        - graph: final graph copy with combined shifted_lines/line attrs
        - stage_1: stage metadata/result for orthogonal pass
        - stage_2: stage metadata/result for diagonal pass
        - info: summary
    """
    # Working graph keeps structure and accumulates shifted edge geometry.
    working = clone_graph_structure(subgraph)

    # Stage 1: orthogonal primary
    s1 = _run_single_stage(
        working,
        target_category="orthogonal",
        engage_len=engage_len,
        tol=tol,
        rotation_sign=rotation_sign,
        min_degree=min_degree,
        iterations=iterations,
        debug=debug,
    )
    _apply_stage_shifted_lines(working, s1["graph"])

    # Stage 2: default_diagonal primary (original grid diagonal)
    s2 = _run_single_stage(
        working,
        target_category="default_diagonal",
        engage_len=engage_len,
        tol=tol,
        rotation_sign=rotation_sign,
        min_degree=min_degree,
        iterations=iterations,
        debug=debug,
    )
    _apply_stage_shifted_lines(working, s2["graph"])

    stage_1_shifted_lines = [
        _line_from_edge(s1["graph"], (u, v))
        for (u, v) in s1["active_edges"]
    ]
    stage_1_shifted_lines = [ln for ln in stage_1_shifted_lines if ln is not None]
    stage_2_shifted_lines = [
        _line_from_edge(s2["graph"], (u, v))
        for (u, v) in s2["active_edges"]
    ]
    stage_2_shifted_lines = [ln for ln in stage_2_shifted_lines if ln is not None]
    primary_shifted_lines = []
    non_primary_lines = []
    for u, v in working.edges():
        hierarchy = working.edge_attribute((u, v), "hierarchy")
        line = _line_from_edge(working, (u, v))
        if hierarchy == "primary":
            primary_shifted_lines.append(line)
        else:
            non_primary_lines.append(line)

    return {
        "graph": working,
        "stage_1": s1,
        "stage_2": s2,
        "viz": {
            "stage_1_shifted_lines": stage_1_shifted_lines,
            "stage_2_shifted_lines": stage_2_shifted_lines,
            "primary_shifted_lines": primary_shifted_lines,
            "non_primary_lines": non_primary_lines,
        },
        "info": {
            "structure_preserved": True,
            "stage_1_category": "orthogonal",
            "stage_2_category": "default_diagonal",
            "stage_1_active_edge_count": len(s1["active_edges"]),
            "stage_2_active_edge_count": len(s2["active_edges"]),
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
    Run reciprocal only on primary diagonal edges.

    Diagonal set:
    - hierarchy == "primary"
    - e_category in ("default_diagonal", "moved_diagonal")

    Returns
    -------
    dict with keys:
        - graph
        - shifted_diagonal_lines
        - untouched_lines
        - info
    """
    working = clone_graph_structure(subgraph)

    active_keys = set()
    for u, v in working.edges():
        hierarchy = working.edge_attribute((u, v), "hierarchy")
        category = working.edge_attribute((u, v), "e_category")
        if hierarchy == "primary" and category in ("default_diagonal", "moved_diagonal"):
            active_keys.add(_edge_key(u, v))

    # IMPORTANT:
    # If line constraints generated diagonal geometry, sync those endpoints to nodes
    # so reciprocal_width_from_subgraph (which reads node points) uses updated positions.
    for u, v in working.edges():
        if _edge_key(u, v) not in active_keys:
            continue
        gl = working.edge_attribute((u, v), "generated_line")
        if gl is None:
            continue

        # Metadata written by line_constraints.regenerate_primary_diagonals_from_orthogonal_nodes
        shared = working.edge_attribute((u, v), "generated_shared_node")
        other = working.edge_attribute((u, v), "generated_other_node")

        if shared is not None and other is not None:
            if working.has_node(shared):
                _set_node_point(working, shared, gl.start)
            if working.has_node(other):
                _set_node_point(working, other, gl.end)
        else:
            # Fallback to edge orientation if metadata not available
            _set_node_point(working, u, gl.start)
            _set_node_point(working, v, gl.end)

        # Keep edge geometry coherent too
        working.edge_attribute((u, v), "line", gl)
        working.edge_attribute((u, v), "shifted_lines", gl)

    backup = _mask_hierarchy_for_stage(working, active_keys, demote_to="tertiary")

    _ = reciprocal_width_from_subgraph(
        working,
        engage_len=engage_len,
        tol=tol,
        rotation_sign=rotation_sign,
        min_degree=min_degree,
        iterations=iterations,
        debug=debug,
    )

    _restore_hierarchy(working, backup)

    shifted_diagonal_lines = []
    untouched_lines = []
    for u, v in working.edges():
        line = _line_from_edge(working, (u, v))
        if line is None:
            continue
        if _edge_key(u, v) in active_keys:
            shifted_diagonal_lines.append(line)
        else:
            untouched_lines.append(line)

    return {
        "graph": working,
        "shifted_diagonal_lines": shifted_diagonal_lines,
        "untouched_lines": untouched_lines,
        "info": {
            "active_diagonal_count": len(active_keys),
            "structure_preserved": True,
        },
    }
