"""Timber joint wrappers."""
from screw_spec_2 import ScrewSpecification  # NOTE: temp.
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
# Base joint wrapper (Parent)
# -----------------------------------
class BaseWrapper(object):
    """A base wrapper class for compas timber joints."""
    def __init__(self, joint):
        self._raw_joint = joint

        self.main_ref_frame = self.main_beam.ref_sides[self.main_beam_ref_side_index]  # NOTE: move to base step wrapper?
        self.cross_ref_frame = self.cross_beam.ref_sides[self.cross_beam_ref_side_index]

        self.entry_type = self.determine_entry_type()
        
    def __getattr__(self, name):
        return getattr(self._raw_joint, name)

    @property
    def is_planar(self):
        """Check if the  joint is planar."""
        tol = 1e-3
        def _is_coplanar(points):
            # Remove duplicates
            pts = []
            for p in points:
                if not pts or (p - pts[-1]).length > tol:
                    pts.append(p)
            if len(pts) < 4:
                return True

            # Find first non-colinear triple
            p0 = pts[0]
            n = None
            for i in range(1, len(pts) - 1):
                v1 = pts[i] - p0
                for j in range(i + 1, len(pts)):
                    v2 = pts[j] - p0
                    n = v1.cross(v2)
                    if n.length > tol:
                        break
                if n and n.length > tol:
                    break

            # All points colinear -> treat as planar
            if not n or n.length <= tol:
                return True

            # Check remaining points
            for p in pts:
                v = p - p0
                if abs(n.dot(v)) > tol:
                    return False
            return True

        if hasattr(self, "main_beam") and hasattr(self, "cross_beam"):
            ca, cb = self.main_beam, self.cross_beam
        else:
            ca, cb = self.element_a, self.element_b
        pts = [*ca.centerline, *cb.centerline]

        # 1. Check coplanarity
        if not _is_coplanar(pts):
            return False

        # 2. Check cross-section alignment (width direction)
        ya = ca.frame.yaxis.unitized()
        yb = cb.frame.yaxis.unitized()
        return abs(ya.dot(yb)) >= 1 - tol

    @property
    def acute_angle(self):
        """Calculate the acute angle in degrees between two beams."""
        angle = angle_vectors(
            self.main_beam.centerline.direction, self.cross_beam.centerline.direction, deg=True
        )
        if angle > 90:
            angle = 180 - angle
        return angle
    
    def _get_perpendicular_riser_normal(self):
        return None

    def _get_bisector_normal(self):
        return None
    
    def determine_entry_type(self):
        return None
    
    def calculate_screw_direction(self, orientation="perp_riser", flip=False):
        """
        Find the screw direction vector for a TMultiStepJoint based on the specified orientation.

        Parameters
        ----------
        orientation : str
            "perp_main", "perp_riser", "perp_cross" and "bisector" for screwing from the main beam to the cross beam,
            "along_cross" for screwing from the cross beam towards the cross section of the main beam.
        """
        # Option 1: screw perpendicular to the centerline of the main beam
        if orientation == "perp_main":
            vec = self._get_perpendicular_main_normal()

        # Option 2: screw perpendicular to the centerline of the cross beam
        elif orientation == "perp_cross":
            vec = self._get_perpendicular_cross_normal()

        # Option 3: screw perpendicular to the jagged riser plane
        elif orientation == "perp_riser":
            vec = self._get_perpendicular_riser_normal()
            if vec is None:
                raise ValueError(f"This joint type ({self.__class__}) does not physically have a riser face.")
            
        elif orientation == "bisector":
            vec = self._get_bisector_normal()
            if vec is None:
                raise ValueError(f"This joint type ({self.__class__}) does not physically have a bisector direction.")

        elif orientation == "along_cross":
            vec = -self.point_centerline_towards_joint(self.main_beam)
 
        else:
            raise ValueError("Invalid orientation type.")
        return vec.unitized() if not flip else -vec.unitized()

    def find_screw_boundaries(self, orientation="perp_riser", flip=False, data_type="points"):
        raise NotImplementedError(f"Subclass {self.__class__.__name__} must implement this method.")
    

# -----------------------------------
# Step joint family
# -----------------------------------
class BaseStepWrapper(BaseWrapper):
    """A base wrapper class for step joint family."""
    def __init__(self, joint):
        super().__init__(joint)

        self.strut_length = self._get_strut_length()
        self.strut_direction = self._get_strut_direction()
        self.strut_vector = self._get_strut_vector()

    def determine_entry_type(self):
        """Determine the entry type of the screw based on the angle."""
        if self.acute_angle > ScrewSpecification.ANGLE_THRESHOLD:
            return "crossed"
        return "aligned"

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

    def find_screw_boundaries(self, orientation="perp_riser", flip=False, data_type="points"):
        """
        Find the entry and exit retangular boundaries.

        Parameters
        ----------
        orientation : str
            "perp_main", "perp_riser", "perp_cross", "along_cross" and "bisector".
        flip : bool
            whether to flip the screw direction.
        data_type : str
            "points", "brep", "polylines".
        """
        if self.entry_type == "crossed":
            pass  # NOTE: find the entry and exit from get_strut_boundary()
        # NOTE: for aligned, entry corners + exit frame, for crossed, entry corners + two exit frames

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
        
        if orientation in ("perp_main", "perp_riser", "perp_cross", "bisector"):
            # 2. Get projection sides (both main and cross beam)
            entry_frame = self._get_entry_frame(flip=flip)
            proj_start_to_main = project_point_to_frame_along(strut_start, dire, entry_frame)
            proj_end_to_main = project_point_to_frame_along(_strut_end, dire, entry_frame)
            # proj_start_to_main = project_point_to_frame_along(strut_start, -dire, self.main_beam.opp_side(self.main_beam_ref_side_index))
            # proj_end_to_main = project_point_to_frame_along(_strut_end, -dire, self.main_beam.opp_side(self.main_beam_ref_side_index))

            proj_start_to_cross= project_point_to_frame_along(strut_start, dire, cross_side_opp)
            proj_end_to_cross = project_point_to_frame_along(_strut_end, dire, cross_side_opp)
            
            pts_entry = [proj_start_to_main, proj_end_to_main, proj_end_to_main + vW, proj_start_to_main + vW]
            pts_exit = [proj_start_to_cross, proj_end_to_cross, proj_end_to_cross + vW, proj_start_to_cross + vW]

        elif orientation == "along_cross":
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

class TSJ(BaseStepWrapper):
    """A wrapper class for TStepJoint."""
    def __init__(self, joint):
        super().__init__(joint)

    def _get_perpendicular_main_normal(self):
        main_ref_frame = self.main_beam.ref_sides[self.main_beam_ref_side_index]
        return main_ref_frame.normal
    
    def _get_perpendicular_cross_normal(self):
        cross_ref_frame = self.cross_beam.ref_sides[self.cross_beam_ref_side_index]
        return -cross_ref_frame.normal

    def _get_perpendicular_riser_normal(self):
        pass
        # def _get_butt_plane(self):
        #         cross_ref_side = self.cross_beam.ref_sides[self.cross_beam_ref_side_index]
        #         butt_plane = Plane(cross_ref_side.point, -cross_ref_side.normal)
        #         return butt_plane.translated(butt_plane.normal * self.step_depth).normal
        # return _get_butt_plane(self)
    
    def _get_bisector_normal(self):
        return (self.main_ref_frame.normal - self.cross_ref_frame.normal).unitized()
    
    def _get_entry_frame(self, flip=False):
        if flip:
            return self.cross_ref_frame
        return self.main_beam.opp_side(self.main_beam_ref_side_index)
    
class TMSJ(BaseStepWrapper):
    """A wrapper class for TMultiStepJoint."""
    def __init__(self, joint):
        super().__init__(joint)

    def _get_perpendicular_main_normal(self):
        main_ref_frame = self.main_beam.ref_sides[self.main_beam_ref_side_index]
        return -main_ref_frame.normal
    
    def _get_perpendicular_cross_normal(self):
        cross_ref_frame = self.cross_beam.ref_sides[self.cross_beam_ref_side_index]
        return -cross_ref_frame.normal
    
    def _get_perpendicular_riser_normal(self):
        tread_0, riser_0 = self._compute_base_planes()
        return -riser_0.normal
    
    def _get_bisector_normal(self):
        return -(self.main_ref_frame.normal + self.cross_ref_frame.normal).unitized()
    
    def _get_entry_frame(self, flip=False):
        if flip:
            return self.cross_ref_frame
        return self.main_ref_frame

# -----------------------------------
# TButtJoint wrapper
# -----------------------------------
class TBJ(BaseWrapper):
    """A wrapper class for TButtJoint."""
    def __init__(self, joint):
        super().__init__(joint)

    def determine_entry_type(self):
        # NOTE: maybe determine by is planar or not?
        return "crossed" if self.is_planar else "aligned"

    def find_screw_boundaries(self, orientation="perp_riser", flip=False, data_type="points"):
        pass


# -----------------------------------
# Birdsmouth joint family
# -----------------------------------
class BaseBirdsmouthWrapper(BaseWrapper):
    """A base wrapper class for birdsmouth joints."""
    def __init__(self, joint):
        super().__init__(joint)

    def determine_entry_type(self):
        return "crossed"  # Must be crossed entry


class TBMJ(BaseBirdsmouthWrapper):
    """A wrapper class for TBirdsmouthJoint."""
    def __init__(self, joint):
        super().__init__(joint)


class KBMJ(BaseBirdsmouthWrapper):
    """A sub-wrapper class splitting a KBirdsmouthJoint into two joint classes."""
    def __init__(self, joint, main_id):
        if main_id not in (0, 1):
            raise ValueError("main_id should be either 0 or 1.")
        self.main_id = main_id
        self.main_beam = joint.elements[main_id + 1]
        self.cross_beam = joint.elements[0]
        
        if main_id == 0:
            self.main_beam_ref_side_index = joint.main_ref_side_index
        else:
            self.main_beam_ref_side_index = joint.main_ref_side_index#(joint.main_ref_side_index + 2) % 4
        self.cross_beam_ref_side_index = joint.cross_ref_side_indices[main_id][0]

        super().__init__(joint)
    
    def find_screw_boundaries(self, orientation="perp_riser", flip=False, data_type="points"):
        pass
