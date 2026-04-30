"""
Shift solver to apply shifting logic to secondary and terciary,spetial category of edges.

Usage in Grasshopper:
    Use the `shift_solver` component to apply shifting logic to secondary,terciary and spetial category of edges.
    
"""


from compas.geometry import Vector, Line, Point, centroid_points, Frame, Box, Plane
from compas.geometry import distance_point_point_xy, intersection_segment_plane
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


def _node_lines(subgraph, edges, node):
    """Grt a nested list of all the lines in nodes."""
    node_lines = [] 
    for u, v in edges:
        if subgraph.edge_attribute((u, v), "shifted_lines"):
            line = subgraph.edge_attribute((u, v), "shifted_lines")
            node_lines.append(line)
        else:
            # make sure the line is always in direction - from the node
            u, v = (node, u if node == v else u)
            line = _get_line(subgraph, (u, v))
            node_lines.append(line)
    
    return node_lines

def _node_line(subgraph, edge, node):
    """Edge line that has direction from the given node."""
    if subgraph.edge_attribute(edge, "shifted_lines"):
        return subgraph.edge_attribute(edge, "shifted_lines")
    else:
        # make sure the line is always in direction - from the node
        u, v = edge
        u, v = (node, u if node == v else v)
        line = Line(_get_pt(subgraph, u), _get_pt(subgraph, v))
        return line

        
# ==============================================================================
# Shifting solvers
# ==============================================================================

def secondary_edges_solver(subgraph, node_edges, secondary_line):
    """
    Applies shifting logic to the secondary category of edges in the cluster of lines in one node.
    
    Parameters:
        subgraph (Graph): The subgraph containing the nodes and edges to be shifted.
        node_lines: Cluster of all the lines in one node
    Returns:
        Returns: shifted secondary lines
    
    """
    
    candidates = []
    
    # Step 1: Iterate trough each dge in the cluster
    for u, v in node_edges:
    # Step 1: Find best candidates
        # condition 1: the edge is not the same edge, they have to be primary
        etype = subgraph.edge_attribute((u, v), 'hierarchy')
        if etype in ("primary"):
            if subgraph.edge_attribute((u, v), "shifted_lines"):
                candidates.append(subgraph.edge_attribute((u, v), "shifted_lines"))
            else:
                candidates.append(_get_line(subgraph, (u, v)))
        else:
            continue
    # Step 2: Get intersection plane of the edge
    point = secondary_line.start
    v1 = Vector.from_start_end(point, secondary_line.end)
    intersection_plane = Plane.from_point_and_two_vectors(point, v1, [0,0,1])
    # Step 2: Intersect with the best candidates
    intersection_pts = []
    for cand in candidates:
        b = intersection_segment_plane(cand, intersection_plane)
        if b:
            intersection_pts.append(Point(*b))
    # Step 3: Find heighest intersection point
    highest_pt = max(intersection_pts, key=lambda pt: pt.z)
    # Step 4: Return a new line 
    shifted_line = Line(secondary_line.end, highest_pt)
    
    return shifted_line, intersection_pts

        


# ==============================================================================
# Main API
# ==============================================================================

def shift_from_subgraph(subgraph, node, debug=True):
    """
    Applies shifting logic to the specified category of edges in the subgraph.

    Parameters:
        subgraph (Graph): The subgraph containing the nodes and edges to be shifted.
    Returns:
        Returns dict with: shifted_lines, secondary_lines, nexus_lines, coplanar_groups, info
    """
    ng = NodeGraph()
    
    # Collect edges by type
    nodes_edges = {}    
    global_beams = {}
    
    secondary_lines = []
    secondary_edges = []
    reciprocal_edges = []
    tertiary_edges = []
    special_edges = []
    
    all_shifted_lines = []
    all_pts = []
    
    # NOTE: Steps for reciprocal solver: 
    # Step 1: collect data
    # Step 2: Find coplanar groups
    # Step 3: Relaxanitaion 
    # Step 4: Post relaxatin
    # Step 5: Final output
    
    
    # Step 1: Find all the nodes for joint resolving task and within the node find all node edges
    for key in subgraph.nodes():
        # remove leaf pts
        if key in subgraph.leaf_nodes() or subgraph.node_attribute(key, "reached") is True:
            continue
        nodes_edges[key] = subgraph.node_edges(key)
    
    
    ### Secondary beams solver
    # Step 1: Collect edges and establish global beam tracking
    for u, v in subgraph.edges():
        etype = subgraph.edge_attribute((u, v), 'hierarchy')
        if etype in ("secondary"):
            secondary_edges.append((u, v))
            
            # Try to get 'width' from graph. Fallback to engage_len if None.
            w = subgraph.edge_attribute((u, v), 'width')
            # shift_dist = (w / 2.0) if w else engage_len
            
            p0, p1 = _get_pt(subgraph, u), _get_pt(subgraph, v)
            
            global_beams[(u, v)] = {
                'p0': [p0.x, p0.y, p0.z], 
                'p1': [p1.x, p1.y, p1.z], 
                #'shift': shift_dist
            }
        else:
            continue
    
    if debug:
        print(f"Edges: {len(secondary_edges)} secondary")

    
    # Iterate trough each edge and solve first secondary lines 
    for key, edges in nodes_edges.items():
        # node_lines = _node_lines(subgraph, edges, key)
        global_intended_moves = {}
        
        for u, v in edges:
            if debug:
                print(u, v)
            etype = subgraph.edge_attribute((u, v), 'hierarchy')
            if etype in ("secondary"):
                line = _node_line(subgraph, (u, v), key)
                shifted_secondary, pts = secondary_edges_solver(subgraph, edges, line)
                global_beams[(u, v)]['p0'] = shifted_secondary.start
                global_beams[(u, v)]['p0'] = shifted_secondary.end
                secondary_lines.append(shifted_secondary)
                all_pts.append(pts)
                if debug:
                    print("secondary")
                    print(shifted_secondary)
                
    return secondary_lines, all_pts


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