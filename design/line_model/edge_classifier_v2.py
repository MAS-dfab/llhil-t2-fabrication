"""Edge classification utilities for line-model graphs."""

from compas.geometry import Vector, Line, Point, centroid_points
from compas.geometry import distance_point_point_xy
from compas.datastructures import Graph
from config import (
    DEFAULT_PARALLEL_TOL, DEFAULT_NEAR_THRESHOLD, 
    DEFAULT_OVERLAP, DEFAULT_SEG_X, DEFAULT_SEG_Y, DEFAULT_ANGLE_TOL
)
import math
from nodegraph import NodeGraph

# ----------------------------------
# Helpers
# ----------------------------------
def _normalize_vector_direction(v):
    """Normalize vector to positive half-plane."""
    if v.x < -1e-9 or (abs(v.x) < 1e-9 and v.y < 0):
        return Vector(-v.x, -v.y, 0.0)
    return Vector(v.x, v.y, 0.0)

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
    return d

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

# ----------------------------------
# Edge Classifier Helpers
# ----------------------------------

def _edge_support_direction(graph, edge, support_points=None, debug=False):
    """Get direction of the edge towards its closest support point."""
        
    # 1. Handle Support Points
    sup_pts = support_points if support_points else graph.get_support_points()
    if not sup_pts:
        return None # Return None consistently so the caller can skip it
    
    # 2. Get Geometry
    u, v = edge
    pu = graph.node_attribute(u, "point")
    pv = graph.node_attribute(v, "point")
    if pu is None or pv is None:
        return None
    
    # 3. Calculate Midpoint and Direction
    # We use a Line midpoint for simplicity
    mid = Line(pu, pv).midpoint
    edge_vec_xy = Vector(pv.x - pu.x, pv.y - pu.y, 0.0)
    edge_vec_xy.unitize()

    # 4. Find Closest Support Point (XY only)
    best_support_pt = min(
        (sp for sp in sup_pts if sp is not None), 
        key=lambda sp: distance_point_point_xy(mid, sp)
    )
    
    # 5. Create Support Vector (from mid to support)
    sup_vec = Vector.from_start_end(Point(mid.x, mid.y, 0.0), 
                                    Point(best_support_pt.x, best_support_pt.y, 0.0))
    sup_vec.unitize()

    # 6. Align Vector
    # If dot product is negative, the vectors are pointing in opposite directions (>90 deg)
    dot = edge_vec_xy.dot(sup_vec)
    
    if debug:
        print(f"DEBUG: Edge {edge} - Dot: {dot:.4f}")

    # If the edge points away from the support, flip it
    if dot < 0:
        return -edge_vec_xy
    
    return edge_vec_xy

def sort_edges_by_angle(graph, target_hierarchy=["primary", "main_primary"], clockwise=False):
    edge_vectors_xy = []
    sorted_by_angle = {}
    abs_tol = 1e-5 # degrees

    for edge in graph.edges():
        if graph.edge_attribute(edge, "hierarchy") in target_hierarchy:
            # NOTE: _edge_support_direction returns (vec, start_pt)
            edge_vec_xy = _edge_support_direction(graph, edge, debug=False)
            if edge_vec_xy is not None:
                edge_vectors_xy.append((_get_line(graph, edge), edge_vec_xy))
    
    for line, vec in edge_vectors_xy:
            # 1. Get angle in degrees (Standard CCW: 0 to 180, 0 to -180)
            angle = math.degrees(math.atan2(vec.y, vec.x))
            
            # 2. Normalize to [0, 360)
            if angle < 0:
                angle += 360.0
            
            # 3. Flip for Clockwise if needed
            # (360 - angle) % 360 converts CCW degrees to CW degrees
            if clockwise:
                angle = (360.0 - angle) % 360.0

            # 4. Apply the tolerance "bucket"
            # This groups angles within your abs_tol into the exact same key
            bucket_angle = round(angle / abs_tol) * abs_tol
            
            # Snap 360 back to 0 to avoid duplicate keys for the same direction
            if math.isclose(bucket_angle, 360.0, abs_tol=abs_tol):
                bucket_angle = 0.0

            sorted_by_angle.setdefault(bucket_angle, []).append(line)

    # 5. Return dictionary sorted by the keys (the angles)
    return dict(sorted(sorted_by_angle.items()))

def group_points_by_angle(graph, angles=[90, 180, 270]):
    """
    Groups points based on a list of degree boundaries.
    Default cuts at 90, 180, 270 creates the 4 quadrants.
    """
    abs_tol = 1e-5 # degrees
    groups = {i: [] for i in range(len(angles) + 1)}
    
    for edge in graph.edges():
        if graph.edge_attribute(edge, "group") == 0:
            # pt = graph.node_attribute(node, "point")
            edge_vec = _edge_support_direction(graph, edge)
            angle = math.degrees(math.atan2(edge_vec.y, edge_vec.x))
            # 2. Normalize to [0, 360)
            if angle < 0:
                angle += 360.0
            # 4. Apply the tolerance "bucket"
            # This groups angles within your abs_tol into the exact same key
            bucket_angle = round(angle / abs_tol) * abs_tol
            # Snap 360 back to 0 to avoid duplicate keys for the same direction
            if math.isclose(bucket_angle, 360.0, abs_tol=abs_tol):
                bucket_angle = 0.0
            
            # Find which "bin" the angle falls into
            bin_idx = 0
            for b in angles:
                if bucket_angle > b:
                    bin_idx += 1
                else:
                    break
            
            groups[bin_idx].append(_get_line(graph, edge))
        
    return groups

# --------------------------------------------------
# Edge categorization
# --------------------------------------------------

def segmentation(graph, debug=False):
    """Segment edges in the graph based on their position."""

    lines_by_group = {}
    
    # Group lines by their group attribute
    for edge in graph.edges():
        group = graph.edge_attribute(edge, "group")
        edge_line = graph.edge_lines_by_group(group)
        lines_by_group.setdefault(f"{group}", []).append(edge)
    
    # Sort primary lines clock wise direction or contra-clockwise direction
    sorted_by_angle = sort_edges_by_angle(graph, target_hierarchy=["primary", "main_primary"], clockwise=False)

    # Apply segmentation to groups individualy 
    bins = group_points_by_angle(graph, sorted_by_angle.keys())
    
    # Shift the segments 

    return bins, sorted_by_angle