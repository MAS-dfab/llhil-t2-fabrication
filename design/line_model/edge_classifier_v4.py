"""
Edge classifier v4.

Differences from v3:
- Keep v3 primary detection, then split primaries into:
  - `primary_orthogonal`
  - `primary_diagonal_<L0>_<L1>` (level-pair buckets)
- Reclassify every non-primary edge by node-level logic:
  - `secondary`: only (0,0) leaf edges
  - `tertiary`: (0,1)
  - `quaternary`: (1,2)
  - `ignored`: everything else
"""

import edge_classifier_v3 as v3
from config import DEFAULT_NEAR_THRESHOLD

# Re-export helpers that are still valid from v3
from edge_classifier_v3 import (  # noqa: F401
    categorize_edge_types,
    classify_edges_by_support_direction,
    combine_graphs,
    create_subgraphs,
    find_dominant_direction,
    get_default_directions,
    is_segment_near_support,
)


def _level_pair(subgraph, edge):
    u, v = edge
    lu = subgraph.node_attribute(u, "level")
    lv = subgraph.node_attribute(v, "level")
    return tuple(sorted((lu, lv)))


def _reclassify_non_primary_by_level(subgraph):
    """Apply v4 hierarchy mapping for non-primary edges."""
    for edge in subgraph.edges():
        hie = subgraph.edge_attribute(edge, "hierarchy")
        if hie == "primary":
            continue

        pair = _level_pair(subgraph, edge)
        u, v = edge
        lu = subgraph.node_attribute(u, "level")
        lv = subgraph.node_attribute(v, "level")

        # Secondary = ONLY (0,0) edges that are leaf edges.
        if pair == (0, 0) and subgraph.is_leaf_edge(edge):
            new_h = "secondary"
        elif pair == (0, 1):
            new_h = "tertiary"
        elif pair == (1, 2):
            new_h = "quaternary"
        else:
            new_h = "ignored"

        subgraph.edge_attribute(edge, "hierarchy", new_h)


def _reclassify_primary_by_category_and_level(subgraph):
    """Split primary into orthogonal vs diagonal(level-pair) buckets."""
    for edge in subgraph.edges():
        hie = subgraph.edge_attribute(edge, "hierarchy")
        if hie != "primary":
            continue

        cat = subgraph.edge_attribute(edge, "e_category")
        pair = _level_pair(subgraph, edge)

        if cat == "orthogonal":
            new_h = "primary_orthogonal"
        else:
            # default_diagonal + moved_diagonal are grouped as diagonal by level
            new_h = "primary_diagonal_{}_{}".format(pair[0], pair[1])

        subgraph.edge_attribute(edge, "hierarchy", new_h)


def classify_edges_in_subgraph(subgraph, sup_pts, parallel_tol=None, near_threshold=None):
    """
    v4 subgraph classification.

    Step 1: run v3 classifier (for base detection).
    Step 2: override non-primary hierarchies by level logic.
    Step 3: split primary into orthogonal/diagonal(level).
    """
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD

    v3.classify_edges_in_subgraph(subgraph, sup_pts, parallel_tol=parallel_tol, near_threshold=near_threshold)

    # Preserve raw v3 hierarchy for optional debugging.
    for edge in subgraph.edges():
        subgraph.edge_attribute(edge, "hierarchy_base", subgraph.edge_attribute(edge, "hierarchy"))

    _reclassify_non_primary_by_level(subgraph)
    _reclassify_primary_by_category_and_level(subgraph)


def classify_edges(graph, seg_x=None, seg_y=None, parallel_tol=None):
    """
    v4 graph classification entrypoint.

    Returns
    -------
    list[NodeGraph]
        Subgraphs with edge hierarchy in:
        primary_orthogonal / primary_diagonal_<L0>_<L1> /
        secondary / tertiary / quaternary / ignored
    """
    # Keep edge-category setup from v3.
    _, _ = categorize_edge_types(graph)
    _, subgraphs = create_subgraphs(graph, seg_x, seg_y)
    sup_pts = graph.get_support_points()

    for sg in subgraphs:
        classify_edges_in_subgraph(sg, sup_pts, parallel_tol=parallel_tol)

    return subgraphs
