"""Solver is function for moving the LineGraph edge in the correct position within the valancy

Usage in Grasshopper:
    Use the `solver` component to apply transformations to linegraph data.

"""

from compas.geometry import Vector, Line, Point, centroid_points
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


# --------------------------------------------------
# Joint Conditions
# --------------------------------------------------

def reciprocal_condition():
    """
    Checks if the joint is reciprocal, meaning that the mobility vector is in the opposite direction of the line vector.
    """
    pass

def move_in_direction_condition():
    """
    Checks if the joint should move in the direction of the mobility vector, meaning that the mobility vector is in the same direction as the line vector.
    """
    pass

def attachment_condition():
    """
    Checks if the joint should be attached, meaning that the mobility vector is perpendicular to the line vector.
    """
    pass


# --------------------------------------------------
# Joint Solvers
# --------------------------------------------------

def reciprocal_solver():
    """
    A simple solver that moves nodes in the opposite direction of the mobility vector, scaled by the mobility value.
    """
    # NOTE: Do we set conditions for this solver here? Like if its more than 3 lines and if it primary
    pass

def move_in_direction_solver():
    """
    A simple solver that moves nodes in the direction of the mobility vector, scaled by the mobility value.
    """
    pass

def attachment_solver():
    """
    A solver that tries to move nodes to the pt at line. 
    """
    pass


# --------------------------------------------------
# Main Solver
# --------------------------------------------------

def run_solver(graph, mobility_data, **kwargs):
    """
    Main solver function that takes in the graph, mobility data, and other parameters, and applies the appropriate solver based on the joint conditions.
    
    Input:
    - graph: The LineGraph data structure containing nodes and edges.
    - mobility_data: Should be valancy. 
    """
    
    # Step 1: Isolate primary first - get lines by hearchy 
    # Step 2: Get the valancy 
    # Step 1: Check planarity of the lines (are all connected edges in the same plane?)
    # Step 2: Make a tree of lines that are in the same plane
    # Step 3: Apply the solver to the tree based on the conditions of the joint. 
    
    pass

