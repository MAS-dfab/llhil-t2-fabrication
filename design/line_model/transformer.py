"""Transformer functions for manipulating graph structures.

Usage in Grasshopper:
    Use the `transformer` component to apply transformations to graph data.

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

tol = 0.0001
amplitude = 0.05

def run_simulation(pt_mobility, intersecting_edges, graph, brep, iterations, tol, debug=False):
    """
    Handles the iterative movement and intersection checking.
    Returns True if all edges are clear, False if intersections remain.
    """
    for n in range(iterations):
        # 1. Bulk Update Node Positions
        for key, mobility, m_vec in pt_mobility:
            current_pos = graph.node_attribute(key, "point")
            # Movement logic based on constraints
            if mobility == "z_free":
                graph.node_attribute(key, "point", current_pos + Vector(0, 0, m_vec[2]))
            elif mobility == "yz_free":
                graph.node_attribute(key, "point", current_pos + m_vec)

        # 2. Check Intersections (Only check the known problematic edges)
        still_intersecting = False
        for u, v in intersecting_edges:
            pu = point_to_rhino(graph.node_attribute(u, "point"))
            pv = point_to_rhino(graph.node_attribute(v, "point"))
            
            # Using a simple line curve for collision detection
            line_geo = rg.LineCurve(pu, pv)
            # CurveBrep [2] returns the intersection points
            if rg.Intersect.Intersection.CurveBrep(line_geo, brep, tol)[2]:
                still_intersecting = True
                break # Exit early: if even one edge hits, the iteration is failed

        if debug:
            status = "CLEAR" if not still_intersecting else "HIT"
            print(f"DEBUG: Sub-Iteration {n+1} | Result: {status}")

        if not still_intersecting:
            return True # Success: No more intersections

    return False # Exhausted iterations, still hitting

def edge_in_brep(graph, brep, max_loops=5, sub_iterations=5, debug=False):
    """Analyzes graph-brep collisions and moves nodes away from the surface."""
    ng = NodeGraph()
    tol = 0.0001
    amplitude = 0.05
    
    # --- 1. IDENTIFY INTERSECTIONS ---
    intersecting_edges = []
    for u, v in graph.edges():
        pu, pv = point_to_rhino(graph.node_attribute(u, "point")), point_to_rhino(graph.node_attribute(v, "point"))
        if rg.Intersect.Intersection.CurveBrep(rg.LineCurve(pu, pv), brep, tol)[2]:
            intersecting_edges.append((u, v))

    if not intersecting_edges:
        if debug: print("Clean: No intersections detected.")
        return [], [], []

    # --- 2. VECTOR CALCULATION (Run once) ---
    target_nodes = list(set([n for edge in intersecting_edges for n in edge]))
    pt_mobility = []
    
    for key in target_nodes:
        mobility = graph.node_attribute(key, "mobility")
        m_vec = ng.get_mobility_vector(mobility, amplitude=amplitude)
        
        # Orient vector away from Brep normal
        rg_pt = point_to_rhino(graph.node_attribute(key, "point"))
        # ClosestPoint returns (bool, point, u, v, normal)
        _, _, _, _, _, normal = brep.ClosestPoint(rg_pt, 0.0)
        normal.Unitize()
        
        # If vector points 'into' brep (dot < 0), flip it
        if rg.Vector3d(m_vec[0], m_vec[1], m_vec[2]) * normal < 0:
            m_vec = Vector(m_vec[0], -m_vec[1], m_vec[2]) if mobility == "yz_free" else -m_vec
            
        pt_mobility.append((key, mobility, m_vec))

    # --- 3. EXECUTE SIMULATION ---
    is_clear = False
    for loop in range(max_loops):
        is_clear = run_simulation(pt_mobility, intersecting_edges, graph, brep, sub_iterations, tol, debug)
        if is_clear:
            if debug: print(f"DEBUG: Graph cleared Brep at Loop {loop + 1}")
            break
            
    # Final data collection
    moved_points = [graph.node_attribute(k, "point") for k, _, _ in pt_mobility]
    return intersecting_edges, is_clear, moved_points