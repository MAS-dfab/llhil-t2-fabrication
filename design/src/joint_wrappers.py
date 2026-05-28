"""Timber joint wrappers."""

from compas.tolerance import TOL
from compas.geometry import (
    Vector, Plane, Polyline, Point, Frame, NurbsCurve, Brep,
    angle_vectors, cross_vectors, dot_vectors,
    intersection_plane_plane_plane,
)
import math

# -----------------------------------
# Geometry Utilities
# -----------------------------------
def project_point_to_frame_along(point, direction, frame, tol=1e-6):
    point = Point(*point)
    dire = Vector(*direction).unitized()

    P = frame.point
    N = frame.normal.unitized()
    denom = dot_vectors(dire, N)
    
    if abs(denom) < tol:
        raise ValueError("Direction is parallel to the plane.")

    if denom < 0:
        dire = -dire
        denom = -denom
    
    num = dot_vectors(P - point, N)
    t = num / denom
    return point + dire * t


# -----------------------------------
# Joint Wrappers
# -----------------------------------
# TStepJoint, TMultiStepJoint, TBirdsmouthJoint, KBirdsmouthJoint and XLapJoint.
class TSJ(object):
    """A wrapper class for TStepJoint."""
    def __init__(self, joint):
        self._raw_joint = joint

    def __getattr__(self, name):
        return getattr(self._raw_joint, name)
    
class TMSJ(object):
    """A wrapper class for TMultiStepJoint."""
    def __init__(self, joint):
        self._raw_joint = joint

        self.main_ref_frame = self.main_beam.ref_sides[self.main_beam_ref_side_index]
        self.cross_ref_frame = self.cross_beam.ref_sides[self.cross_beam_ref_side_index]

        self.acute_angle = self._calculate_acute_angle()
        self.strut_length = self._get_strut_length()
        self.strut_direction = self._get_strut_direction()
        self.strut_vector = self._get_strut_vector()

        self.screws = None

    def __getattr__(self, name):
        return getattr(self._raw_joint, name)

    def _calculate_acute_angle(self):
        """Calculate the acute angle in degrees between two beams."""
        if len(self.elements) != 2:
            raise ValueError("Joint must have exactly two elements.")
        
        ea, eb = self.elements
        angle = angle_vectors(ea.centerline.direction, eb.centerline.direction, deg=True)
        if angle > 90:
            angle = 180 - angle
        return angle

    def _get_strut_length(self):
        strut_height = self.main_beam.height
        return strut_height / math.sin(math.radians(self.acute_angle))
    
    def _get_strut_direction(self):
        strut_direction = Vector(*cross_vectors(self.main_ref_frame.yaxis, self.cross_ref_frame.zaxis)).unitized()
        if TOL.is_positive(dot_vectors(self.main_ref_frame.normal, strut_direction)):
            strut_direction = -strut_direction
        return strut_direction

    def _get_strut_vector(self):
        return self.strut_direction * self.strut_length
    
    def calculate_screw_direction(self, orientation="perp_tread", flip=False):
        """
        Find the screw direction vector for a TMultiStepJoint based on the specified orientation.

        Parameters
        ----------
        orientation : str
            "perp_main", "perp_tread", "perp_cross" and "bisector" for screwing from the main beam to the cross beam,
            "cross_section" for screwing from the cross beam towards the cross section of the main beam.
        """
        # Option 1: screw perpendicular to the centerline of the main beam
        if orientation == "perp_main":
            main_ref_frame = self.main_beam.ref_sides[self.main_beam_ref_side_index]
            vec = -main_ref_frame.normal

        # Option 2: screw perpendicular to the jagged tread plane. But why does it called riser in CT?
        elif orientation == "perp_tread":
            tread_0, riser_0 = self._compute_base_planes()
            vec = -riser_0.normal
        
        # Option 3: screw perpendicular to the centerline of the cross beam
        elif orientation == "perp_cross":
            cross_ref_frame = self.cross_beam.ref_sides[self.cross_beam_ref_side_index]
            vec = -cross_ref_frame.normal

        elif orientation == "bisector":
            vec = -(self.main_ref_frame.normal + self.cross_ref_frame.normal).unitized()

        elif orientation == "cross_section":
            vec = -self.point_centerline_towards_joint(self.main_beam)
 
        else:
            raise ValueError("Invalid orientation type.")
        return vec.unitized()
    
    def get_strut_boundary(self, data_type="points"):
        """Get the rectangular boundary of the strut."""
        p0 = intersection_plane_plane_plane(
            Plane.from_frame(self.main_ref_frame),
            Plane.from_frame(self.cross_ref_frame),
            Plane.from_frame(self.main_beam.front_side(self.main_beam_ref_side_index)),
        )
        p0 = Point(*p0)
        p1 = p0 + self.strut_vector

        vW = self.main_ref_frame.yaxis * self.main_beam.width
        p2 = p1 + vW
        p3 = p0 + vW

        if data_type == "polyline":
            return Polyline([p0, p1, p2, p3, p0])
        elif data_type == "points":
            return (p0, p1, p2, p3)
        return
    
    def get_strut_frame(self):
        corners = self.get_strut_boundary(data_type="points")
        return Frame.from_points(corners[0], corners[1], corners[3])

    def find_screw_boundaries(self, orientation="perp_tread", flip=False, data_type="points"):
        """
        Find the entry and exit retangular boundaries.

        Parameters
        ----------
        orientation : str
            "perp_main", "perp_tread", "perp_cross", "cross_section" and "bisector".
        flip : bool
            whether to flip the screw direction.
        data_type : str
            "points", "brep", "polylines".
        """
        # 1. Get projection vector
        dire = self.calculate_screw_direction(orientation=orientation, flip=flip)

        coords = intersection_plane_plane_plane(
            Plane.from_frame(self.main_ref_frame),
            Plane.from_frame(self.cross_ref_frame),
            Plane.from_frame(self.main_beam.front_side(self.main_beam_ref_side_index)),
        )
        strut_start = Point(*coords)
        cross_side_opp = self.cross_beam.opp_side(self.cross_beam_ref_side_index)
        _strut_end = strut_start + self.strut_vector

        # Width vector
        vW = self.main_ref_frame.yaxis * self.main_beam.width
        
        if orientation in ("perp_main", "perp_tread", "perp_cross", "bisector"):
            # 2. Get projection sides (both main and cross beam)
            proj_start= project_point_to_frame_along(strut_start, dire, cross_side_opp)

            proj_end_to_cross = project_point_to_frame_along(_strut_end, dire, cross_side_opp)
            proj_end_to_main = project_point_to_frame_along(_strut_end, dire, self.main_ref_frame)
            
            pts_entry = [strut_start, proj_end_to_main, proj_end_to_main + vW, strut_start + vW]
            pts_exit = [proj_start, proj_end_to_cross, proj_end_to_cross + vW, proj_start + vW]

        elif orientation == "cross_section":
            # p0: projection to cross beam
            p0 = project_point_to_frame_along(strut_start, -dire, cross_side_opp)
            p1 = project_point_to_frame_along(_strut_end, -dire, cross_side_opp)        
            pts_entry = [p0, p1, p1 + vW, p0 + vW]

            # Find the farthest cross section side of the main beam to the strut start
            crosec_sides = self.main_beam.ref_sides[4:]
            farthest = max(crosec_sides, key=lambda s: s.point.distance_to_point(strut_start))
            pts_exit = [project_point_to_frame_along(p, dire, farthest) for p in pts_entry]

        else:
            raise ValueError("Invalid orientation type.")
        
        # Output data
        if data_type == "points":
            return (pts_entry, pts_exit)
        
        elif data_type == "brep":
            pts_entry += [pts_entry[0]]
            pts_exit += [pts_exit[0]]
            crv1 = NurbsCurve.from_points(pts_entry, degree=1)
            crv2 = NurbsCurve.from_points(pts_exit, degree=1)
            brep = Brep.from_loft([crv1, crv2])
            brep.cap_planar_holes()
            return brep
        
        elif data_type == "polylines":
            pts_entry += [pts_entry[0]]
            pts_exit += [pts_exit[0]]
            return [Polyline(pts_entry), Polyline(pts_exit)]
        else:
            raise ValueError("Invalid data type.")
    
class TBMJ(object):
    """A wrapper class for TBirdsmouthJoint."""
    def __init__(self, joint):
        self._raw_joint = joint

    def __getattr__(self, name):
        return getattr(self._raw_joint, name)

class KBMJ(object):
    """A wrapper class for KBirdsmouthJoint."""
    def __init__(self, joint):
        self._raw_joint = joint

    def __getattr__(self, name):
        return getattr(self._raw_joint, name)
    
class XLJ(object):
    """A wrapper class for XLapJoint."""
    def __init__(self, joint):
        self._raw_joint = joint

    def __getattr__(self, name):
        return getattr(self._raw_joint, name)