"""Timber footing planning."""

from compas.tolerance import TOL
from compas.geometry import (
    Vector, Plane, Polyline, Point, NurbsCurve, Brep,
    angle_vectors, cross_vectors, dot_vectors,
    intersection_plane_plane_plane
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
        joint : Joint
            joint to apply screws to.
        orientation : str
            "perp_main", "perp_tread" and "perp_cross" for screwing from the main beam to the cross beam,
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
        
        elif orientation == "cross_section":
            vec = -self.point_centerline_towards_joint(self.main_beam)

        else:
            raise ValueError("Invalid orientation type.")
        return vec.unitized()
    
    def calculate_screwing_area(self, data_switch=False):
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

        if data_switch:
            return Polyline([p0, p1, p2, p3, p0])
        return (p0, p1, p2, p3)

    def calculate_screw_volume(self, orientation="perp_tread", flip=False, data_type="points"):
        """
        
        Parameters
        ----------

        data_type : str
            "points", "brep", "polylines"
        """
        # 1. Get projection vector
        dire = self.calculate_screw_direction(orientation=orientation, flip=flip)

        if orientation == "perp_tread":
            # 2. Get projection sides (both main and cross beam)
            coords = intersection_plane_plane_plane(
                Plane.from_frame(self.main_ref_frame),
                Plane.from_frame(self.cross_ref_frame),
                Plane.from_frame(self.main_beam.front_side(self.main_beam_ref_side_index)),
            )
            strut_start = Point(*coords)

            cross_side = self.cross_beam.opp_side(self.cross_beam_ref_side_index)
            proj_start= project_point_to_frame_along(strut_start, dire, cross_side)

            _strut_end = strut_start + self.strut_vector
            proj_end_to_cross = project_point_to_frame_along(_strut_end, dire, cross_side)
            proj_end_to_main = project_point_to_frame_along(_strut_end, dire, self.main_ref_frame)
            
            pts = [strut_start, proj_start, proj_end_to_cross, proj_end_to_main]
            vW = self.main_ref_frame.yaxis * self.main_beam.width
            
            pts_main = [strut_start, proj_end_to_main, proj_end_to_main + vW, strut_start + vW]
            pts_cross = [proj_start, proj_end_to_cross, proj_end_to_cross + vW, proj_start + vW]
            
            if data_type == "points":
                return pts_main + pts_cross
            
            elif data_type == "brep":
                pts_main += [pts_main[0]]
                pts_cross += [pts_cross[0]]
                crv1 = NurbsCurve.from_points(pts_main, degree=1)
                crv2 = NurbsCurve.from_points(pts_cross, degree=1)
                brep = Brep.from_loft([crv1, crv2])
                brep.cap_planar_holes()
                return brep
            
            elif data_type == "polylines":
                pts_main += [pts_main[0]]
                pts_cross += [pts_cross[0]]
                return [Polyline(pts_main), Polyline(pts_cross)]


    def populate_screws(self, orientation="perp_tread", flip=False):
        dire = self.calculate_screw_direction(orientation=orientation, flip=flip)
        pass

# -----------------------------------
# Main API
# -----------------------------------
class ScrewSolver:
    def __init__(self, model, min_screw_spacing=0.021):
        self.model = model
        self.joints = model.joints
        self.min_screw_spacing = min_screw_spacing

    def is_collided(self):
        pass

    def populate_screws(self):
        pass


def apply_screws(model, step_angle_threshold=None, step_orientation="perp_tread"):

    JOINT_MAP = {
        TMultiStepJoint : TMSJ,

    }
    vecs = []
    for joint in model.joints:
        if joint.name != "TMultiStepJoint":
            continue  # temporary for testing only TMultiStepJoint
        
        # Wrap the joint with the corresponding class in JOINT_MAP if it exists
        joint_class = joint.__class__
        if joint_class in JOINT_MAP:
            joint = JOINT_MAP[joint_class](joint)

        # Screw from the side of the cross beam towards the cross section of the main beam
        if joint.acute_angle > step_angle_threshold:
            vec = joint.calculate_screw_direction(orientation="cross_section")
        # Screw from the reference side of the main beam towards cross beam
        else:
            vec = joint.calculate_screw_direction(orientation=step_orientation)

        vecs.append(vec)
    return vecs