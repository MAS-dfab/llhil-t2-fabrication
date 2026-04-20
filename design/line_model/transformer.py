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
tol = 0.0001
threshold = 1.2 # 1-meter limit for mobility influence 

def run_simulation(pt_mobility, intersecting_edges, graph, brep, iterations, tol, threshold, debug=False):
    """
    Moves only the nodes belonging to edges that are CURRENTLY intersecting.
    """
    for n in range(iterations):
        # 1. Identify which edges are STILL intersecting right now
        currently_stuck_edges = []
        for u, v in intersecting_edges:
            pu = point_to_rhino(graph.node_attribute(u, "point"))
            pv = point_to_rhino(graph.node_attribute(v, "point"))
            
            line_geo = rg.LineCurve(pu, pv)
            if rg.Intersect.Intersection.CurveBrep(line_geo, brep, tol)[2]:
                currently_stuck_edges.append((u, v))

        # 2. Exit early if NO edges are stuck anymore
        if not currently_stuck_edges:
            if debug: print(f"DEBUG: All clear at sub-iteration {n+1}")
            return True

        # 3. Create a set of nodes that belong to these specific stuck edges
        active_node_ids = set()
        for u, v in currently_stuck_edges:
            active_node_ids.add(u)
            active_node_ids.add(v)

        # 4. Only move nodes that are BOTH in the active set AND within the distance threshold
        for dist, key, mobility, m_vec in pt_mobility:
            if key in active_node_ids and dist < threshold:
                current_pos = graph.node_attribute(key, "point")
                
                # Apply movement
                if mobility == "z_free":
                    graph.node_attribute(key, "point", current_pos + Vector(0, 0, m_vec[2]))
                elif mobility == "yz_free":
                    graph.node_attribute(key, "point", current_pos + m_vec)

    return False

def get_y_direction_modifier(point, brep, current_y_vec, tol=0.001):
    """
    Returns 1.0 if the direction is correct, -1.0 if it needs to be flipped.
    Uses a horizontal line probe to ignore Z-surface interference.
    """
    # Create a long horizontal line through the point
    line_start = point + rg.Vector3d(0, -5000, 0)
    line_end = point + rg.Vector3d(0, 5000, 0)
    probe = rg.LineCurve(line_start, line_end)
    
    # Find Y-axis hits
    success, _, intersect_pts = rg.Intersect.Intersection.CurveBrep(probe, brep, tol)
    
    # Check if point is inside brep
    is_inside = brep.IsPointInside(point, tol, True)
    
    if intersect_pts:
        # Sort hits by distance to find the nearest vertical wall
        nearest_wall = sorted(intersect_pts, key=lambda p: p.DistanceTo(point))[0]
        
        # Vector from wall to point (the 'escape' direction)
        y_escape_val = point.Y - nearest_wall.Y
        
        # If the point is inside, we want to move it towards the nearest wall (negative escape direction)
        if is_inside:
            y_escape_val = -y_escape_val
        # If the movement (current_y_vec) is opposite to the escape direction, flip it
        # (Using sign comparison: if signs are different, product is negative)
        if (current_y_vec * y_escape_val) < 0:
            return -1.0
            
    return 1.0

def edge_in_brep(graph, brep, max_loops=5, sub_iterations=5, debug=False):
    """Analyzes graph-brep collisions and moves nodes away from the surface."""
    
    ng = NodeGraph()
    
    # --- 1. IDENTIFY INTERSECTIONS ---
    intersecting_edges = []
    for u, v in graph.edges():
        pu, pv = point_to_rhino(graph.node_attribute(u, "point")), point_to_rhino(graph.node_attribute(v, "point"))
        if rg.Intersect.Intersection.CurveBrep(rg.LineCurve(pu, pv), brep, tol)[2]:
            intersecting_edges.append((u, v))

    if not intersecting_edges:
        if debug: print("Clean: No intersections detected.")
        return [], [], []

    # --- 2. VECTOR CALCULATION (Run once) & DISTANCE TRACKING ---
    target_nodes = list(set([n for edge in intersecting_edges for n in edge]))
    target_points = [graph.node_attribute(n, "point") for n in target_nodes]
    pt_mobility = []
    
    for key in target_nodes:
        mobility = graph.node_attribute(key, "mobility")
        m_vec = ng.get_mobility_vector(mobility, amplitude=amplitude)
        
        # Orient vector away from Brep normal
        rg_pt = point_to_rhino(graph.node_attribute(key, "point"))
        # ClosestPoint returns (bool, point, u, v, normal)
        success, cp, u, v, face_idx, normal = brep.ClosestPoint(rg_pt, 0.0)
        dist = rg_pt.DistanceTo(cp)
        normal.Unitize()
        dot = m_vec.dot(normal)
              
        # If vector points 'into' brep (dot < 0), flip it
        if dot < 0:
            m_vec = Vector(m_vec[0], -m_vec[1], m_vec[2]) if mobility == "yz_free" else -m_vec
        
        if mobility == "yz_free":
            y_modifier = get_y_direction_modifier(rg_pt, brep, m_vec[1], tol)
            m_vec = Vector(m_vec[0], m_vec[1] * y_modifier, m_vec[2])  
            
        pt_mobility.append((dist, key, mobility, m_vec))

    # --- 3. EXECUTE SIMULATION ---
    is_clear = False
    for loop in range(max_loops):
        # We only pass points that are within the 1.0m threshold
        active_points = [p for p in pt_mobility if p[0] < threshold]
        if not active_points and debug:
            print("DEBUG: No points remain within the threshold, but intersections still exist.")
            break
        # Pass the formatted list to run_simulation (stripping the distance for compatibility)
        sim_list = [(p[0], p[1], p[2], p[3]) for p in active_points]
        
        is_clear = run_simulation(sim_list, intersecting_edges, graph, brep, sub_iterations, tol, threshold, debug)
        if is_clear:
            if debug: print(f"DEBUG: Graph cleared Brep at Loop {loop + 1}")
            break
            
    # Final data collection
    moved_points = [graph.node_attribute(k, "point") for _, k, _, _ in pt_mobility]
    return intersecting_edges, is_clear, target_points