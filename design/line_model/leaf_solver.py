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

from compas.geometry import Point, Line, Vector, Frame, angle_vectors_signed
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
def create_shoes_from_graph(graph, extension_length=0.35):
    """
    Create paired X-shoe lines at inset nodes grouped by shared apex.

    Inset nodes are sorted cyclically around each apex and connected
    across (0↔2, 1↔3 for four nodes; 0↔-1 for two) to form X shapes.

    Returns
    -------
    dict
        shoe_lines  : list of Line
        shoe_frames : list of Frame -- surface plane at each inset point
    """
    shoe_lines = []
    shoe_frames = []

    apex_to_insets = {}
    for node in graph.nodes_where({"ntype": "inset"}):
        neighbors = list(graph.neighbors(node))
        if neighbors:
            apex_to_insets.setdefault(neighbors[0], []).append(node)

    for apex, nodes in apex_to_insets.items():
        if len(nodes) < 2:
            continue

        apex_pt = graph.node_attribute(apex, "point")
        pts = [graph.node_attribute(n, "point") for n in nodes]

        def _angle(idx):
            p = pts[idx]
            vec = Vector(p.x - apex_pt.x, p.y - apex_pt.y, 0.0)
            return angle_vectors_signed(Vector(1, 0, 0), vec, Vector(0, 0, 1))

        order = sorted(range(len(nodes)), key=_angle)
        sorted_pts = [pts[i] for i in order]
        sorted_nodes = [nodes[i] for i in order]

        if len(nodes) >= 4:
            p0, p1, p2, p3 = sorted_pts[0], sorted_pts[1], sorted_pts[2], sorted_pts[3]
            if p0 and p2:
                p0_dir = Vector.from_start_end(p2, p0).unitized()
                p2_dir = Vector.from_start_end(p0, p2).unitized()
                p0_ext = p0 + p0_dir * extension_length
                p2_ext = p2 + p2_dir * extension_length

                ln = Line(p0_ext, p2_ext)
                attrs = {'shifted_line': ln, 'hierarchy': 'shoe', 'level': 0}
                graph.add_graph_edge(
                    sorted_nodes[0],
                    sorted_nodes[2],
                    **attrs
                )
                shoe_lines.append(ln)

            if p1 and p3:
                p1_dir = Vector.from_start_end(p3, p1).unitized()
                p3_dir = Vector.from_start_end(p1, p3).unitized()
                p1_ext = p1 + p1_dir * extension_length
                p3_ext = p3 + p3_dir * extension_length

                ln = Line(p1_ext, p3_ext)
                attrs = {'shifted_line': ln, 'hierarchy': 'shoe', 'level': 0}
                graph.add_graph_edge(
                    sorted_nodes[1],
                    sorted_nodes[3],
                    **attrs
                )
                shoe_lines.append(ln)
        else:
            if sorted_pts[0] and sorted_pts[-1]:
                ln = Line(sorted_pts[0], sorted_pts[-1])
                attrs = {'shifted_line': ln, 'hierarchy': 'shoe', 'level': 0}
                graph.add_graph_edge(
                    sorted_nodes[0],
                    sorted_nodes[-1],
                    **attrs
                )
                shoe_lines.append(ln)

        for n in sorted_nodes:
            f = graph.node_attribute(n, "brep_frame")
            if f is not None:
                shoe_frames.append(f)

    return {"shoe_lines": shoe_lines, "shoe_frames": shoe_frames}


def create_individual_shoes(graph, length_factor=0.5):
    """
    Create individual cross-shoe lines at every inset node.

    Two perpendicular lines are drawn at each inset point in the surface
    plane: one along the beam direction toward the apex, one transverse.

    Returns
    -------
    dict
        shoe_lines  : list of Line
        shoe_frames : list of Frame -- surface plane at each inset point
    """
    shoe_lines = []
    shoe_frames = []

    for node in graph.nodes_where({"ntype": "inset"}):
        pt = graph.node_attribute(node, "point")
        if pt is None:
            continue
        frame = graph.node_attribute(node, "brep_frame")
        if frame is None:
            frame = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))

        surface_normal = frame.xaxis.cross(frame.yaxis)
        neighbors = list(graph.neighbors(node))
        arm = 0.05
        primary = frame.xaxis
        if neighbors:
            apex_pt = graph.node_attribute(neighbors[0], "point")
            if apex_pt is not None:
                arm = pt.distance_to_point(apex_pt) * length_factor
                raw = Vector.from_start_end(apex_pt, pt)
                projected = raw - surface_normal * raw.dot(surface_normal)
                if projected.length > 1e-9:
                    projected.unitize()
                    primary = projected

        secondary = surface_normal.cross(primary)
        if secondary.length > 1e-9:
            secondary.unitize()
        else:
            secondary = frame.yaxis

        shoe_lines.append(Line(pt.translated(-primary * arm), pt.translated(primary * arm)))
        shoe_lines.append(Line(pt.translated(-secondary * arm), pt.translated(secondary * arm)))
        shoe_frames.append(frame)

    return {"shoe_lines": shoe_lines, "shoe_frames": shoe_frames}


def create_shoes_and_shift_leaves(graph, brep, min_length=0.18, min_angle=20):

    leaf_edges = graph.leaf_edges()

    # 2. Create X-shoe lines (and their Z-vector for compas_timber beam creation: not yet:( )
    shoe_lines = []
    adjacency = {}
    for u, v in leaf_edges:
        adjacency.setdefault(u, []).append((u, v))
        adjacency.setdefault(v, []).append((u, v))

    parent_children = {node : edges for node, edges in adjacency.items() if len(edges) > 1}
    print (f"pc: {parent_children}")
        
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