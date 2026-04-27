"""
Shift solver to apply shifting logic to secondary and terciary,spetial category of edges.

Usage in Grasshopper:
    Use the `shift_solver` component to apply shifting logic to secondary,terciary and spetial category of edges.
    
"""


from compas.geometry import Vector, Line, Point, centroid_points, Frame, Box
from compas.geometry import distance_point_point_xy
from compas.datastructures import Graph
from compas_rhino.conversions import line_to_rhino, point_to_rhino
from nodegraph import NodeGraph
from config import (
    DEFAULT_PARALLEL_TOL, DEFAULT_NEAR_THRESHOLD, 
    DEFAULT_OVERLAP, DEFAULT_SEG_X, DEFAULT_SEG_Y, DEFAULT_ANGLE_TOL
)
import math
import Rhino.Geometry as rg # type: ignore


# ==============================================================================
# Helpers
# ==============================================================================

def _get_pt(subgraph, node):
    """Get point from node."""
    pt = subgraph.node_attribute(node, 'point')
    if pt is None:
        return Point(subgraph.node_attribute(node, 'x'),
                    subgraph.node_attribute(node, 'y'),
                    subgraph.node_attribute(node, 'z') or 0.0)
    return Point(pt.x, pt.y, pt.z) if hasattr(pt, 'x') else Point(*pt)


def _get_line(subgraph, edge):
    """Get line from edge."""
    u, v = edge
    line = subgraph.edge_attribute((u, v), 'line')
    if line and isinstance(line, Line):
        return line
    if line:
        return Line(Point(*line[0]), Point(*line[1]))
    return Line(_get_pt(subgraph, u), _get_pt(subgraph, v))


# ==============================================================================
# Testing
# ==============================================================================

def shift_solver(graph):
    """
    Applies shifting logic to the specified category of edges in the graph.

    Parameters:
        graph (Graph): The graph containing the nodes and edges to be processed.
        category (str): The category of edges to apply shifting logic to. 
                        It can be "secondary", "terciary" or "spetial".
    Returns:
        None: The function modifies the graph in place by applying shifting logic to the specified category of edges.
    """
    ng = NodeGraph()
    
    # Find all the edges in the node 
    
    for key in graph.nodes():
        edges = graph.node_edges(key)


# ==============================================================================
# Main API
# ==============================================================================

def shift_from_subgraph(subgraph, debug=True):
    """
    Applies shifting logic to the specified category of edges in the subgraph.

    Parameters:
        subgraph (Graph): The subgraph containing the nodes and edges to be shifted.
    Returns:
        Returns dict with: shifted_lines, secondary_lines, nexus_lines, coplanar_groups, info
    """
    ng = NodeGraph()
    
    # Collect edges by type
    secondary_edges = []
    tertiary = []
    special = []
    
    nodes = []
    all_lines = []

    # Step 1: Find all the nodes for joint resolving task
    for key in subgraph.nodes():
        # remove leaf pts
        if key in subgraph.leaf_nodes() or subgraph.node_attribute(key, "reached") is True:
            continue
        point = subgraph.node_attribute(key, "point")
        nodes.append(key)
    
    # Step 2: Find all the edges that belong to the node
    for key in nodes:
        point = subgraph.node_attribute(key, "point")
        edges = subgraph.node_edges(key)
        lines = []
        for u, v in edges:
            line = _get_line(subgraph, (u, v))
            lines.append(line)
        all_lines.append(lines)
    
    return nodes, all_lines
    # _______ solver for secondary edges _______
    # Find all the primary edges in the same window and level and group them together.
    # Get planes of secondary_edges
    # Intersect planes with primary edges to find nexus points. 
    # Sort nexus points by z height
    # create new lines from starting point to nexus point
    
    # ________ solver for tertiary edges _______
    # First find all the nodes for resolving tertiary edges.
    # Loop troough all the nodes
    ### Elimination aproach for tertiary edges: CANDIDATES LIST 
    # Sort the secondary edges by z height 
    # conditions for a good candidate for tertiary edge:
    # if not proccessed in the solver for secondary edges?
    # if it belongs to the same window and level as the tertiary edge.
    # if the edge is neighbor - do the contra clock wise check to find the correct candidate and remove the edge from the candidate list _______!
    # chose correct primary edge to attach ???? and remove edge from the candidate list. QUESTION: what does correct candidate mean?
    
    # ________ solver for spetial edges _______
    # write conditions for good candidate


    
    # for u, v in subgraph.edges():
    #     etype = subgraph.edge_attribute((u, v), 'hierarchy')
    #     if etype in ('secondary'):
    #         reciprocal_edges.append((u, v))
    #     else:
    #         secondary_lines.append(_get_line(subgraph, (u, v)))
    
    # if debug:
    #     print(f"Edges: {len(reciprocal_edges)} reciprocal, {len(secondary_lines)} secondary")