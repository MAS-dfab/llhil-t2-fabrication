"""Transformer functions for manipulating graph structures.

Usage in Grasshopper:
    Use the `transformer` component to apply transformations to graph data.

"""

from compas.geometry import Vector, Line, Point, centroid_points
from compas.geometry import distance_point_point_xy
from compas.datastructures import Graph
from compas_rhino.conversions import line_to_rhino
from config import (
    DEFAULT_PARALLEL_TOL, DEFAULT_NEAR_THRESHOLD, 
    DEFAULT_OVERLAP, DEFAULT_SEG_X, DEFAULT_SEG_Y, DEFAULT_ANGLE_TOL
)
import math
import Rhino.Geometry as rg

tol = 0.0001

# TODO: Implement the following transformations:
# - `edge_in_brep`: Find points that are inside a brep.
# - `move_z_direction`: Move points in z direction.
# - `move_y_direction`: Move points in y direction if they belong to a specific category.
# - `merge_all`: Merge all points that are within a certain distance of each other.

def edge_in_brep(graph, brep):
    """Find points in the graph that are inside the given brep."""
    all_edges = []
    intersection = []
    
    for edge in graph.edge_lines():
        rg_line = line_to_rhino(edge).ToNurbsCurve()
        pt_list = rg.Intersect.Intersection.CurveBrep(rg_line, brep, tol)[2]
        print (list(pt_list))
        if pt_list:
            intersection.append(pt_list)
            all_edges.append(edge)
    return all_edges, intersection
