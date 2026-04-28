"""
Leaf beams solver and shoe creation.

Usage in grasshopper:
    from leaf_solver import compute_leaf_and_shoe
    result = compute_leaf_and_shoe(graph, min_angle=25)
"""

from compas.geometry import Point, Line
from compas.geometry import intersection_line_line
import Rhino.Geometry as rg  # type: ignore
import math


# ----------------------------------
# Leaf Shifting Helpers
# ----------------------------------
def orient_line_down(line, tol=1e-3):
    """Orient the line from lower Z to higher Z"""
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
def shift_leaf_line(line, rail, angle):
    """Shift line along rail with a fixed anchor."""



# ------------------------------
# Main API
# ------------------------------
def compute_leaf_and_shoe(graph, brep, min_angle=20):

    # A1. Get leaf lines to evaluate angle between CLT plate

    # # A2. Get nodes with valency not 1
    # nodes = list(set([x for edge in leaf_edges for x in edge]))
    # parent_nodes = [n for n in nodes if graph.degree(n) != 1]

    # # A3. Create a dict of leaf clusters
    # leaves_group = {}
    # for pn in parent_nodes:
    #     for le in leaf_edges:
    #         if pn in le:
    #             leaves_group.setdefault(pn, []).append(le)

    # # A4. Partition edges by their colinearity, for now using hierarchy to find pairs
    # shoe_pairs = {}
    # for pn, edges in leaves_group.items():
    #     pair = {}
    #     for edge in edges:
    #         hie = graph.edge_attribute(edge, "hierarchy")
    #         if hie == "primary":
    #             pair.setdefault("primary", []).append(edge)
    #         if hie == "secondary":
    #             pair.setdefault("secondary", []).append(edge)

    #     for two in pair.values():
    #         shoe_pairs.setdefault(pn, []).append(two)
    # print (shoe_pairs)

    # 2. Create X-shoe lines and their Z-vector for compas_timber beam creation

    # 3. Move leaf points and assign back to graph

    min_angle_rad = math.radians(min_angle)
    leaf_edges = graph.leaf_edges()

    meet_pts = []
    for edge in leaf_edges:
        line = graph.edge_line(edge)
        is_flip, oriented = orient_line_down(line)

        proj_st = project_to_brep(oriented.start, brep)
        proj_nd = project_to_brep(oriented.end, brep)
        projected = Line(proj_st, proj_nd)

        angle_rad = oriented.direction.angle(projected.direction)

        # If angle is larger than the angle constraint, don't shift
        if angle_rad > min_angle_rad:
            continue
        
        r_angle = min_angle_rad - angle_rad
        r_axis = -oriented.direction.cross(projected.direction)
        origin = oriented.end

        rot_line = oriented.rotated(r_angle, r_axis, origin)
        meet_coords = intersection_line_line(rot_line, projected)
        if not meet_coords:
            raise ValueError  ##
        meet_pt = Point(*meet_coords[0])
        meet_pts.append(meet_pt)
        
        for node in edge:
            if graph.is_leaf(node):
                graph.node_attribute(node, "point", meet_pt)
                graph.node_attributes(node, ["x", "y", "z"], [meet_pt.x, meet_pt.y, meet_pt.z])
                pass
        
    # 4. Add new edge of shoe, with an attribute "shoe": line geometry
    return {
        "meet_pts": meet_pts
    }