"""
Find non-intersecting beams which are reached to the anchors.

Usage in grasshopper:
    from foundation_solver import trim_anchored_beams_and_find_cut_planes

    result = trim_anchored_beams_and_find_cut_planes(graph, min_gap=0.02, min_radius=0.60, min_height=0.26)
    cross_sections = result["cross_sections"]
    cut_planes = result["cut_planes"]
"""

from compas.geometry import Plane, Vector, Point
import math


# -----------------------------
# Geometry helpers
#------------------------------
def get_line_directions_xy(line):
    """Get the line directions in XY plane."""
    dir_xy = Vector(line.end.x - line.start.x, line.end.y - line.start.y, 0).unitized()  # height direction
    perp_xy = dir_xy.cross(Vector(0, 0, 1)).unitized()  # width direction
    return dir_xy, perp_xy

def offset_points_along_vector_by_z(points, vector, z_offset):
    """Offset a list of points along a vector by a certain z offset."""
    if z_offset == 0:
        return points
    if vector.z == 0:
        raise ValueError("The vector is parallel to the XY plane, cannot offset by z.")
    
    dir = vector.unitized()
    angle = dir.angle(Vector(0, 0, 1))
    d = z_offset / math.cos(angle)

    return [pt + (dir * d) for pt in points]

def offset_points_xy(origin, dir1, dir2, height, width):
    """Offset a point in XY plane by two directions and height/width."""
    vec1 = dir1.unitized() * (height / 2)
    vec2 = dir2.unitized() * (width / 2)
    return [
        origin + vec1 + vec2,
        origin - vec1 + vec2,
        origin - vec1 - vec2,
        origin + vec1 - vec2,
    ]

def average_point(points):
    """Calculate the average point from a list of points."""
    x = sum(p.x for p in points) / len(points)
    y = sum(p.y for p in points) / len(points)
    z = sum(p.z for p in points) / len(points)
    return Point(x, y, z)

# ------------------------------------------------------------------------
# Separating Axis Theorem (SAT) for 2D cross sections (convex shapes)
# ------------------------------------------------------------------------
def _project(points, vector):
    return [Vector(*p).dot(vector) for p in points]

def _is_intervals_overlap(a, b):
    return not (max(a) < min(b) or max(b) < min(a))

def _is_intersect(crosec1, crosec2):
    axes = [
        crosec1[1] - crosec1[0],
        crosec1[2] - crosec1[1],
        crosec2[1] - crosec2[0],
        crosec2[2] - crosec2[1]
    ]

    for axis in axes:
        proj1 = _project(crosec1, axis)
        proj2 = _project(crosec2, axis)
        if not _is_intervals_overlap(proj1, proj2):
            return False
    return True


# ----------------------------------------
# Class Beam and projected cross section
# ----------------------------------------
class Beam:
    def __init__(self, line, width, height, **attrs):
        """
        Parameters
        ----------
        line : Line
            Line with cross section to be projected.
        width : float
            Width of the cross section
        height : float
            Height of the cross section

        """
        self.line = line
        self.width = width
        self.height = height
        self.attributes = attrs

        self.start = line.start
        self.end = line.end

        self.dir_xy, self.perp_xy = get_line_directions_xy(self.line)
        
        self.angle = self.dir_xy.angle(self.line.direction)
        self.height_xy = self.height / math.sin(self.angle)  # projected height in XY plane
        
        self.cross_section_xy = self.project_cross_section_xy()

    def project_cross_section_xy(self, z_offset=0.0):
        """
        Get the projected cross section of the beam as a list of 4 points in XY plane.

        Returns
        -------
        list of Point
            4 Points in XY plane, ordered by the line direction.
        """
        pts = offset_points_xy(self.start, self.dir_xy, self.perp_xy, self.height_xy, self.width)

        if z_offset != 0:
            pts = offset_points_along_vector_by_z(pts, self.line.direction, z_offset)
        return [Point(*p) for p in pts]

    def is_intersect(self, other, gap=0.0):
        if not isinstance(other, Beam):
            raise ValueError
        
        crosec1 = self.cross_section_xy
        crosec2 = other.cross_section_xy

        if gap != 0:
            cen1 = average_point(crosec1)
            cen2 = average_point(crosec2)
            crosec1 = offset_points_xy(cen1, self.dir_xy, self.perp_xy, self.height_xy + gap, self.width + gap)
            crosec2 = offset_points_xy(cen2, other.dir_xy, other.perp_xy, other.height_xy + gap, other.width + gap)

        return _is_intersect(crosec1, crosec2)


# --------------------------------------
# Accumulated Foundation Solver
# --------------------------------------
class FoundationSolver:
    def __init__(self, beams: list[Beam], min_radius, min_height, min_gap, step=0.01, max_attempts=100):
        self.beams = beams
        self.min_radius = min_radius
        self.min_height = min_height
        self.min_gap = min_gap
        self.step = step
        self._max_attempts = max_attempts

    def are_multiple_beams_intersect(self):
        for i in range(len(self.beams)):
            for j in range(i + 1, len(self.beams)):
                if self.beams[i].is_intersect(self.beams[j], gap=self.min_gap):
                    return True
        return False
    
    def solve(self):
        # 1. Check if cross sections are under the min_height or not.
        # 2. Check if cross sections are intersecting or not.
        # 3. Check if cross sections are in the bounds of the given min_radius.
        for i in range(self._max_attempts):
            curr_height = self.min_height + (i * self.step)
            if self.are_multiple_beams_intersect():
                for b in self.beams:
                    b.cross_section_xy = b.project_cross_section_xy(curr_height)
            else:
                break
            
        if i == self._max_attempts - 1:
            print (Warning("Failed to find non-intersecting beams within the max attempts."))

        return [b.cross_section_xy for b in self.beams], curr_height - self.step


# -------------------------------
# Main API
# -------------------------------
def trim_anchored_beams_and_find_cut_planes(graph, min_radius=0.60, min_height=0.26, min_gap=0.02, _step = 0.01):
    """
    xxx

    Parameters
    ----------
    graph : NodeGraph
        As attributes' library, typically not used for getting geometry.
    min_radius : float, optional
        Minimum radius of the foundations.
    min_height : float, optional
        Minimum height of the foundations.
    min_gap : float, optional
        Minimum gap between beams.

    Returns
    -------
    dict of dicts
        {
        "cross_sections": {support_node: [cross_section1, cross_section2, ...], ...},
        "trimmed_lines": {(u, v): Line, ...},
        "cut_planes": {support_node: Plane, ...},
        "foundation_heights": {support_node: float, ...}
        }
    """
    # 1. Group beams by support nodes
    beams_map = {}
    for sup_edge in graph.get_support_edges():
        # Use shifted line if exists, otherwise use the original line from the graph
        if "shifted_line" in graph.edge_attributes(sup_edge):
            ln = graph.edge_attribute(sup_edge, "shifted_line")
        else:
            ln = graph.edge_line(sup_edge)

        w = graph.edge_attribute(sup_edge, "width") if "width" in graph.edge_attributes(sup_edge) else 0.12
        h = graph.edge_attribute(sup_edge, "height") if "height" in graph.edge_attributes(sup_edge) else 0.14

        beam = Beam(ln, w, h)

        u, v = sup_edge
        sup_node = u if u in graph.get_support_nodes() else v
        beams_map.setdefault(sup_node, []).append(beam)
    
    # 2. Solve for each support node
    cro_secs, trims, cuts, steps = {}, {}, {}, {}
    for sup_node, beams in beams_map.items():
        solver = FoundationSolver(
            beams=beams,
            min_radius=min_radius,
            min_height=min_height,
            min_gap=min_gap,
            step=_step,
            max_attempts=100
        )
        crosecs, step = solver.solve()

        cro_secs[sup_node] = crosecs
        steps[sup_node] = step

        # 3. Assign cut planes to nodes' attrubutes in graph
        sup_pt = graph.node_point(sup_node)
        cut_pln = Plane(sup_pt + Vector(0, 0, step), Vector(0, 0, 1))
        cuts[sup_node] = cut_pln
        
        graph.node_attribute(sup_node, "cut_plane", cut_pln)
        
    return {
        "cross_sections": cro_secs,
        "trimmed_lines": {},  # temp.
        "cut_planes": cuts,
        "foundation_heights": steps
    }