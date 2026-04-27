"""Anchor functions for creating steel foundation.

Usage in Grasshopper:
    Use the `anchor` component to apply anchor steel foundation to anchor points.

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



def create_anchor_joint(graph, anchor_length=0.05):
    """
    Creates an anchor joint at the specified node key in the graph.
    The anchor joint is represented as a line segment extending from the node's point.

    Parameters:
        key (int): The node key where the anchor joint will be created.
        graph (Graph): The graph containing the node and its attributes.
        anchor_length (float): The length of the anchor joint line segment.
    Returns:
        Line: A line segment representing the anchor joint. 
    """
    ng = NodeGraph()
    
    # Get support node keys
    support_keys = [key for key in graph.nodes() if graph.node_attribute(key, "reached") is True]
    # support_key = ng.get_support_nodes()
    
    # Get connected edges and their vectors
    data = {}
    boxes = []
    anchor_ponts = []
    
    for key in support_keys:
        point = graph.node_attribute(key, "point")
        edges = graph.node_edges(key)
        # Get support id to identify the floor level 
        support_id = graph.node_attribute(key, "support_id")
        vectors = []
        for u, v in edges:
            other_key = v if u == key else u
            other_point = graph.node_attribute(other_key, "point")
            vector = Vector.from_start_end(point, other_point)
            # For each edge create steel box element and orient it in the direction of the vector from mid point to the support point.
            edge_line_midpoint = Line(point, other_point).midpoint
            orientation_vector = Vector.from_start_end(edge_line_midpoint, point)
            orientation_vector.unitize()
            projected_vector = Vector(orientation_vector.x, orientation_vector.y, 0)
            frame = Frame(point=point, xaxis=vector, yaxis=Vector(orientation_vector.x, orientation_vector.y, 0))
            box = Box(xsize=anchor_length*5, ysize=anchor_length*.5, zsize=anchor_length*1, frame=frame)
            
            vectors.append(vectors)
            boxes.append(box)
            
        data[support_id] = vectors
        anchor_ponts.append(point)
    

    
    return boxes, anchor_ponts
        