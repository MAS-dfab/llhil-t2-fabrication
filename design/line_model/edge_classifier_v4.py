"""
Edge classifier v4.

Differences from v3:
- Keep PRIMARY logic from v3.
- Reclassify every NON-PRIMARY edge by node-level logic:
  - `secondary`: non-primary leaf edges touching level 0
  - `tertiary`: edges connecting levels (0, 1)
  - `quaternary`: edges connecting levels (1, 2)
  - `ignored`: everything else non-primary

This intentionally ignores the old `special` bucket for now.
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


def classify_edges_in_subgraph(subgraph, sup_pts, parallel_tol=None, near_threshold=None):
    """
    v4 subgraph classification.

    Step 1: run v3 classifier (for primary detection and base attrs).
    Step 2: override all non-primary hierarchies by level logic.
    """
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD

    v3.classify_edges_in_subgraph(subgraph, sup_pts, parallel_tol=parallel_tol, near_threshold=near_threshold)
    _reclassify_non_primary_by_level(subgraph)


def classify_edges(graph, seg_x=None, seg_y=None, parallel_tol=None):
    """
    v4 graph classification entrypoint.

    Returns
    -------
    list[NodeGraph]
        Subgraphs with edge hierarchy in:
        primary / secondary / tertiary / quaternary / ignored
    """
    # Keep edge-category setup from v3.
    _, _ = categorize_edge_types(graph)
    _, subgraphs = create_subgraphs(graph, seg_x, seg_y)
    sup_pts = graph.get_support_points()

    for sg in subgraphs:
        classify_edges_in_subgraph(sg, sup_pts, parallel_tol=parallel_tol)

    return subgraphs

