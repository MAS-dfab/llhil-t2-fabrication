"""
Leaf beams snapping and shoes creation.

Usage in grasshopper:
    from leaf_solver import compute_leaf_and_shoe
    from copy import deepcopy

    ng = deepcopy(graph)
    result = create_shoes_and_shift_leaves(ng, brep, min_length=0.30, min_angle=20)

    leaf_points = result["leaf_points"]
    shoe_lines = result["shoe_lines"]
"""

from compas.geometry import Point, Line, Vector
from compas.geometry import intersection_line_line
import Rhino.Geometry as rg  # type: ignore
import math


# ----------------------------------
# Leaf Shifting Helpers
# ----------------------------------
def orient_line_down(line, tol=1e-3):
    """Orient the line from higher Z to lower Z"""
    if line.start.z < line.end.z - tol:
        return True, Line(line.end, line.start)
    return False, line

def project_to_brep(pt, brep):
    """Project a point vertically (Z) onto a Rhino brep."""
    if brep is None:
        return pt
    origin = rg.Point3d(pt.x, pt.y, pt.z)
    ray = rg.Ray3d(origin, rg.Vector3d.ZAxis)
    hits = rg.Intersect.Intersection.RayShoot(ray, [brep], 1)
    if hits:
        return Point(hits[0].X, hits[0].Y, hits[0].Z)
    # Try downward if upward missed
    ray = rg.Ray3d(origin, -rg.Vector3d.ZAxis)
    hits = rg.Intersect.Intersection.RayShoot(ray, [brep], 1)
    if hits:
        return Point(hits[0].X, hits[0].Y, hits[0].Z)
    return pt


# ----------------------------------
# Leaf Shifting Algorithm
# ----------------------------------
def check_notch_length(p1, p2, min_length=180):
    """
    Check if the notch length (from the tooth of the notch to the side of the chord) is sufficient.
    """
    return p1.distance_to_point(p2) >= min_length

def shift_leaf_point_by_angle(oriented, projected, angle):
    """Rotate point around a fixed anchor."""
    axis = -oriented.direction.cross(projected.direction)
    origin = oriented.end

    rot_line = oriented.rotated(angle, axis, origin)
    meet_coords = intersection_line_line(rot_line, projected)
    if not meet_coords:
        raise ValueError("Lines do not intersect after rotation.")
    return Point(*meet_coords[0])

def shift_leaf_point_by_distance(leaf_point, projected, distance):
    vec = projected.direction * distance
    return leaf_point.translated(vec)


# ------------------------------
# Main API
# ------------------------------
def create_shoes_and_shift_leaves(graph, brep, min_length=0.18, min_angle=20):

    leaf_edges = graph.leaf_edges()

    # 1. Create X-shoe lines (and their Z-vector for compas_timber beam creation: not yet:( )
    shoe_lines = []
    adjacency = {}
    for u, v in leaf_edges:
        adjacency.setdefault(u, []).append((u, v))
        adjacency.setdefault(v, []).append((u, v))

    parent_children = {node : edges for node, edges in adjacency.items() if len(edges) > 1}
    # print (f"pc: {parent_children}")
        
    partitioned = {}
    temp_pts = []
    for parent_node, children_edges in parent_children.items():
        pairs_dict = {}
        for edge in children_edges:
            hie = graph.edge_attribute(edge, 'hierarchy')
            pairs_dict.setdefault(hie, []).append(edge)

        for hie, lst in pairs_dict.items():
            node_pair = [node for edge in lst for node in edge if graph.is_leaf(node)]
            
            # Save non x-shoe pairs for later shoe creation
            if len(children_edges) <= 2:
                temp_pts.append(node_pair)
                continue

            p1 = graph.node_point(node_pair[0])
            p2 = graph.node_point(node_pair[1])
            shoe_line = Line(p1, p2)
            shoe_lines.append(shoe_line)

        partitioned[parent_node] = pairs_dict


    # 2. Move leaf points and assign back to graph
    min_angle_rad = math.radians(min_angle)
    new_leaf_points = []

    for edge in leaf_edges:
        line = graph.edge_line(edge)
        is_flip, oriented = orient_line_down(line)

        proj_st = project_to_brep(oriented.start, brep)
        proj_nd = project_to_brep(oriented.end, brep)
        projected = Line(proj_st, proj_nd)

        leaf_pt = oriented.start.copy()

        # If angle is smaller than the angle constraint, shift(rotate) by angle
        angle_rad = oriented.direction.angle(projected.direction)
        if angle_rad < min_angle_rad:
            r_angle = min_angle_rad - angle_rad
            leaf_pt = shift_leaf_point_by_angle(oriented, projected, r_angle)
        
        # Check notch distance
        if not check_notch_length(leaf_pt, proj_st, min_length):
            leaf_pt = shift_leaf_point_by_distance(leaf_pt, projected, min_length)

        new_leaf_points.append(leaf_pt)

        # Update node coordinates in graph
        for node in edge:
            if graph.is_leaf(node):
                graph.node_attribute(node, "point", leaf_pt)
                graph.node_attributes(node, ["x", "y", "z"], [leaf_pt.x, leaf_pt.y, leaf_pt.z])

    # Create non x-shoe
    for pair in temp_pts:
        c1 = graph.node_point(pair[0])
        c2 = graph.node_point(pair[1])
        vec = Vector.from_start_end(c1, c2).unitized() * min_length
        shoe_line1 = Line(c1.translated(-vec), c1.translated(vec))
        shoe_line2 = Line(c2.translated(vec), c2.translated(-vec))
        shoe_lines.append(shoe_line1)
        shoe_lines.append(shoe_line2)

    return {
        "shoe_lines": shoe_lines,
        "group_dict": partitioned,
        "leaf_points": new_leaf_points
    }