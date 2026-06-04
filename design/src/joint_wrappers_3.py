"""Timber joint wrappers."""
from screw_spec_2 import ScrewSpecification  # NOTE: temp.
from compas.tolerance import TOL
from compas.geometry import (
    Vector, Plane, Polyline, Point, Frame, NurbsCurve, Brep,
    angle_vectors, cross_vectors, dot_vectors,
    intersection_plane_plane_plane, intersection_polyline_plane
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
        dire = -dire  # TODO: don't flip the direction, but flip the plane instead
        denom = -denom
    
    num = dot_vectors(P - point, N)
    t = num / denom
    return point + dire * t

def is_same_xy_sign(direction):
    """Return True if direction.x and direction.y have the same sign (or zero)."""
    x, y = direction.x, direction.y
    return (x >= 0 and y >= 0) or (x <= 0 and y <= 0)

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
        """Check if the joint is planar."""
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
    
    def determine_entry_type(self):
        return None
    
    def calculate_screw_direction(self, angle=None):
        """Calculate the screw direction based on the entry type and the given angle."""
        if angle is not None:
            if angle <= 0:
                raise ValueError("Angle should be larger than 0.")
            
        main_dire = self.point_centerline_towards_joint(self.main_beam)
        if self.entry_type == "aligned":
            rotation_axis = self._get_rotation_axis()
            return main_dire.rotated(math.radians(angle), rotation_axis).unitized()
        
        elif self.entry_type == "crossed":
            return -main_dire.unitized()
        
        elif self.entry_type == "krossed":
            return # Something
        
        else:
            raise ValueError("Invalid entry type.")

    def find_screw_boundaries(self, angle=None, flip=False, data_type="points"):
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
    
    def get_interface_boundary(self, data_type="points"):
        """Get the rectangular boundary of the interface between main and cross beam."""
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

    @property
    def default_exit_frame(self):
        beam_features = self.cross_beam.features
        longitudinal = next((f for f in beam_features if type(f).__name__ == "LongitudinalCut"), None)
        if longitudinal is not None:
            return longitudinal.plane_from_params_and_beam(self.cross_beam)
        return self.cross_beam.opp_side(self.cross_beam_ref_side_index)

    def find_screw_boundaries(self, angle=None, flip=False, data_type="points"):
        """
        Find the entry and exit retangular boundaries.

        Parameters
        ----------
        angle : float
            The angle to rotate the screw direction for aligned entry type. Ignored for crossed entry type.
        flip : bool
            Whether to flip the screw direction.
        data_type : str
            "points", "brep", "polylines".
        """
        if angle is not None:
            if angle <= 0:
                raise ValueError("Angle should be larger than 0.")
            
        # 1. Get projection vector
        dire = self.calculate_screw_direction(angle)

        coords = intersection_plane_plane_plane(
            Plane.from_frame(self.main_ref_frame),
            Plane.from_frame(self.cross_ref_frame),
            Plane.from_frame(self.main_beam.front_side(self.main_beam_ref_side_index)),
        )
        strut_start = Point(*coords)
        _strut_end = strut_start + self.strut_vector

        # Width vector
        vW = self.main_ref_frame.yaxis * self.main_beam.width
        
        exit_frame = self.default_exit_frame

        if self.entry_type == "aligned":
            # 2. Get projection sides (both main and cross beam)
            entry_frame = self._get_entry_frame(flip=flip)
            proj_start_to_main = project_point_to_frame_along(strut_start, dire, entry_frame)
            proj_end_to_main = project_point_to_frame_along(_strut_end, dire, entry_frame)
            
            pts_entry = [proj_start_to_main, proj_end_to_main, proj_end_to_main + vW, proj_start_to_main + vW]
            if self.__class__.__name__ == "TSJ":
                pts_entry = pts_entry[2:] + pts_entry[:2]  # flip the entry rectangle to be in the order of [p2, p3, p0, p1]
            pts_exit = [project_point_to_frame_along(p, dire, exit_frame) for p in pts_entry]

        elif self.entry_type == "crossed":
            # p0: projection to cross beam
            p0 = project_point_to_frame_along(strut_start, -dire, exit_frame)
            p1 = project_point_to_frame_along(_strut_end, -dire, exit_frame)        
            pts_entry = [p0, p1, p1 + vW, p0 + vW]
            if self.__class__.__name__ == "TSJ":
                pts_entry = pts_entry[2:] + pts_entry[:2]

            # Find the farthest cross section side of the main beam to the strut start
            crosec_sides = self.main_beam.ref_sides[4:]
            farthest = max(crosec_sides, key=lambda s: s.point.distance_to_point(strut_start))
            pts_exit = [project_point_to_frame_along(p, dire, farthest) for p in pts_entry]

        else:
            raise ValueError("Invalid entry type.")
        
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

    def _get_rotation_axis(self):
        return Vector(*cross_vectors(self.main_ref_frame.normal, self.cross_ref_frame.normal)).unitized()
    
    def _get_entry_frame(self, flip=False):
        if flip:
            return self.cross_ref_frame
        return self.main_beam.opp_side(self.main_beam_ref_side_index)
    
class TMSJ(BaseStepWrapper):
    """A wrapper class for TMultiStepJoint."""

    def __init__(self, joint):
        super().__init__(joint)

    def _get_rotation_axis(self):
        return -Vector(*cross_vectors(self.main_ref_frame.normal, self.cross_ref_frame.normal)).unitized()
    
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
        if self.is_planar:
            if self.acute_angle > ScrewSpecification.ANGLE_THRESHOLD:
                return "crossed"
            return "aligned"
        return "crossed"

    @property
    def interface_area(self):
        pass

    def find_screw_boundaries(self, angle=None, flip=False, data_type="points"):
        pass


# -----------------------------------
# Birdsmouth joint family
# -----------------------------------
class BaseBirdsmouthWrapper(BaseWrapper):
    """A base wrapper class for birdsmouth joints."""

    def __init__(self, joint):
        super().__init__(joint)

    @property
    def _is_acute(self):
        """check if beam pair is acute angled or obtuse angled."""
        angle = angle_vectors(
            self.main_beam.centerline.direction, self.cross_beam.centerline.direction, deg=True
        )
        return angle <= 90


class TBMJ(BaseBirdsmouthWrapper):
    """A wrapper class for TBirdsmouthJoint."""

    def __init__(self, joint):
        super().__init__(joint)


class KBMJ(BaseBirdsmouthWrapper):
    """A sub-wrapper class splitting a KBirdsmouthJoint into two joint classes."""

    def __init__(self, joint, main_id):     # main_id to specify which of the two main beams to compute
        if main_id not in (0, 1):
            raise ValueError("main_id should be either 0 or 1.")
        self.main_id = main_id
        self.main_beam = joint.elements[main_id + 1]
        self.cross_beam = joint.elements[0]
        self.cutting_planes = joint._get_cutting_planes()[main_id]
        self.miter_plane = joint._get_miter_planes()[main_id]
        
        if is_same_xy_sign(self.main_beam.centerline.direction):
            self.main_beam_ref_side_index = joint.main_ref_side_index + 3           # NOTE: main_ref_side pointing 'inwards' where its 'normal' is orientated towards the cross_beam centerline
            self._adjacency_direction = -1
        else:
            self.main_beam_ref_side_index = joint.main_ref_side_index + 1           # NOTE: main_ref_side pointing 'inwards' where its 'normal' is orientated towards the cross_beam centerline
            self._adjacency_direction = 1

        self.cross_beam_ref_side_index = joint.cross_ref_side_indices[main_id][1]   # NOTE: cross_beam_ref_side pointing 'inwards' where its 'normal' is orientated towards the main_ref_side centerline
        super().__init__(joint)

    def determine_entry_type(self):
        return "krossed"
    

    def determine_entry_type(self):
        return "krossed"  # kool entry type exclusive for KBMJ
    
    def _get_double_cut_planes(self):
        beam_features = self.main_beam.features
        double = next((f for f in beam_features if type(f).__name__ == "DoubleCut"), None)
        if double is not None:
            return double.planes_from_params_and_beam(self.main_beam) # returns a list of two planes
        return None                                                                 # NOTE: we dont need this since we have this our class as self

    def _get_bisector_of_centerlines(self):
        if self._is_acute:
            return (self.main_beam.centerline.direction + self.cross_beam.centerline.direction).unitized()
        else:
            flipped_main_centerline = (-self.cross_beam.centerline.direction.x, -self.cross_beam.centerline.direction.y, self.cross_beam.centerline.direction.z)
            return (self.main_beam.centerline.direction + Vector(*flipped_main_centerline)).unitized()
        
    def get_interface_boundary_vertical(self, data_type="polyline"):
        """
        Find the vertical interface boundaries for main beam and cross beam.
        
        Returns a polyline.

        Parameters
        ----------
        data_type : str
            "points", "polylines".       
        """
        def _sort_points(points, plane):
            # Find the center mass of the 4 points
            cx = sum(p.x for p in points) / 4.0
            cy = sum(p.y for p in points) / 4.0
            cz = sum(p.z for p in points) / 4.0
            
            normal = plane.normal
            nx, ny, nz = abs(normal[0]), abs(normal[1]), abs(normal[2])
            
            # Project points to the local 2D space of the cut_plane to calculate angles accurately
            def get_angle_fallback(point):
                # Project onto the flat plane that matches the highest normal direction
                if nz >= nx and nz >= ny:
                    return math.atan2(point.y - cy, point.x - cx)
                elif ny >= nx and ny >= nz:
                    return math.atan2(point.z - cz, point.x - cx)
                else:
                    return math.atan2(point.z - cz, point.y - cy)

            # Sort the points chronologically around the center (clockwise/counter-clockwise)
            sorted_points = sorted(points, key=get_angle_fallback)
            return sorted_points

        # Extract all cutting planes
        cut_plane_side = Plane.from_frame(self.cutting_planes[1])
        cut_plane_top = Plane.from_frame(self.cutting_planes[0])

        bottom_side = Plane.from_frame(self.main_beam.ref_sides[0])
        top_side = Plane.from_frame(self.main_beam.ref_sides[2])
        ref_side = Plane.from_frame(self.main_beam.ref_frame)
        opp_side = Plane.from_frame(self.main_beam.opp_side(self.main_ref_side_index))

        p0 = intersection_plane_plane_plane(ref_side, cut_plane_side, top_side)
        p1 = intersection_plane_plane_plane(opp_side, cut_plane_side, top_side)
        p2 = intersection_plane_plane_plane(bottom_side, ref_side, cut_plane_side)
        p3 = intersection_plane_plane_plane(bottom_side, opp_side, cut_plane_side)
        
        raw_points = [p0, p1, p2, p3]
        points = [Point(*p) for p in raw_points]
        sorted_points = _sort_points(points, cut_plane_side)
        poly = Polyline(sorted_points + [sorted_points[0]])
        
        # Intersect cross section with cuting side plane 
        int_pts = intersection_polyline_plane(poly, cut_plane_top)

        # Get a list of all the points 
        all_pts = sorted_points
        for pt in int_pts:
            all_pts.append(Point(*pt))
        
        # Get interface points
        plane_pt = cut_plane_top.point
        normal = Vector(*cut_plane_top.normal)
        normal_u = normal.unitized()
        interface_pts = []
        
        for pt in all_pts:
            v = Vector(*[pt[0] - plane_pt[0], pt[1] - plane_pt[1], pt[2] - plane_pt[2]])
            u = v.unitized()
            dot_product = u.dot(normal_u)
            if dot_product <= 1e-2:
                interface_pts.append(pt)
                
        sorted_interface_pts = _sort_points(interface_pts, cut_plane_side)
        
        # Return sorted data
        if data_type == "polyline":
            # Append the first point to the end to cleanly close the loop
            return Polyline(sorted_interface_pts + [sorted_interface_pts[0]])
        
        elif data_type == "points":
            return tuple(sorted_interface_pts)
        return 
    
    def get_interface_boundary_horizontal(self, data_type="polyline"):
        """
        Find the horizontal interface boundaries for main beam and cross beam.
        
        Returns a polyline.

        Parameters
        ----------
        data_type : str
            "points", "polylines".       
        """
        def _sort_points(points, plane):
            # Find the center mass of the 4 points
            cx = sum(p.x for p in points) / len(points)
            cy = sum(p.y for p in points) / len(points)
            cz = sum(p.z for p in points) / len(points)
            
            normal = plane.normal
            nx, ny, nz = abs(normal[0]), abs(normal[1]), abs(normal[2])
            
            # Project points to the local 2D space of the cut_plane to calculate angles accurately
            def get_angle_fallback(point):
                # Project onto the flat plane that matches the highest normal direction
                if nz >= nx and nz >= ny:
                    return math.atan2(point.y - cy, point.x - cx)
                elif ny >= nx and ny >= nz:
                    return math.atan2(point.z - cz, point.x - cx)
                else:
                    return math.atan2(point.z - cz, point.y - cy)

            # Sort the points chronologically around the center (clockwise/counter-clockwise)
            sorted_points = sorted(points, key=get_angle_fallback)
            return sorted_points

        # Extract all cutting planes
        cut_plane_side = Plane.from_frame(self.cutting_planes[1])
        cut_plane_top = Plane.from_frame(self.cutting_planes[0])

        bottom_side = Plane.from_frame(self.main_beam.ref_sides[0])
        top_side = Plane.from_frame(self.main_beam.ref_sides[2])
        ref_side = Plane.from_frame(self.main_beam.ref_frame)
        opp_side = Plane.from_frame(self.main_beam.opp_side(self.main_ref_side_index))

        p0 = intersection_plane_plane_plane(ref_side, cut_plane_top, top_side)
        p1 = intersection_plane_plane_plane(opp_side, cut_plane_top, top_side)
        p2 = intersection_plane_plane_plane(bottom_side, ref_side, cut_plane_top)
        p3 = intersection_plane_plane_plane(bottom_side, opp_side, cut_plane_top)
        
        raw_points = [p0, p1, p2, p3]
        points = [Point(*p) for p in raw_points]
        sorted_points = _sort_points(points, cut_plane_top)
        poly = Polyline(sorted_points + [sorted_points[0]])
        
        # Intersect cross section with cuting side plane 
        int_pts = [*intersection_polyline_plane(poly, cut_plane_side), *intersection_polyline_plane(poly, self.miter_plane)]
        # Get a list of all the points 
        all_pts = sorted_points
        for pt in int_pts:
            all_pts.append(Point(*pt))
        
        # Get interface points
        plane_pt = cut_plane_side.point
        miter_plane_pt = self.miter_plane.point
        normal = Vector(*cut_plane_side.normal)
        normal_miter = Vector(*self.miter_plane.normal)
        normal_u = normal.unitized()
        normal_miter_u = normal_miter.unitized()
        interface_pts = []
        
        for pt in all_pts:
            v = Vector(*[pt[0] - plane_pt[0], pt[1] - plane_pt[1], pt[2] - plane_pt[2]])
            u = v.unitized()
            
            v_miter = Vector(*[pt[0] - miter_plane_pt[0], pt[1] - miter_plane_pt[1], pt[2] - miter_plane_pt[2]])
            u_miter = v_miter.unitized()
            
            dot_product = u.dot(normal_u)
            dot_product_miter = u_miter.dot(normal_miter_u)
            if dot_product <= 1e-2 and dot_product_miter <= 1e-2:
                interface_pts.append(pt)
        
        sorted_interface_pts = _sort_points(interface_pts, cut_plane_top)
        
        # Return sorted data
        if data_type == "polyline":
            # Append the first point to the end to cleanly close the loop
            return Polyline(sorted_interface_pts + [sorted_interface_pts[0]])
        
        elif data_type == "points":
            return tuple(sorted_interface_pts)
        return 
    
    def calculate_screw_direction(self):
        """Calculate the screw direction based on centerline directions."""

        if self.entry_type == "krossed":
            return self._get_bisector_of_centerlines()
        
        else:
            raise ValueError("Invalid entry type. Its not kool")

    def find_screw_boundaries(self, angle=None, flip=False, data_type="points"):
        raise NotImplementedError(f"Subclass {self.__class__.__name__} must implement this method.")

    def _calculate_screw_directions(self):
        """Create a dictionary of candidate screw direction vectors.

        Keys:
            - 'bisector' : bisector direction
            - 'tilted_vertical' : vertical in z direction
        """
        # NOTE: screw directions towards joint 'location'
        screw_directions = {}
        
        # 1) Bisector direction
        try:
            bis = self._get_bisector_of_centerlines()
        except Exception:
            bis = None
        if bis is not None:
            screw_directions["bisector"] = bis.unitized()

        # 3) Tilted Vertical
        screw_directions["tilted_vertical"] = -Vector(0, 0, 1) # NOTE: to be tilted

        return screw_directions

    def _get_perpendicular_main_normal(self):
        """Find normal to main_ref_side, used for calculating entry frame for perp_vertical_double"""
        return None

    def _calculate_entry_exit_frames(self):
        """Calculate entry and exit frames for screw projection based on screw direction candidates."""
        screw_direction_a = self.calculate_screw_direction()
        pts_to_project = self.get_interface_boundary_vertical(data_type="points")
        
        projected_pts = []
        for p in pts_to_project:
            p_point = project_point_to_frame_along(p, screw_direction_a, self.main_beam.front_side(self.main_beam_ref_side_index), tol=1e-6)
            projected_pts.append(p_point)
        return Polyline(projected_pts + [projected_pts[0]])

    def find_screw_boundaries(self, flip=False, data_type="points"):
        """
        Find the entry and exit boundaries for all candidate screw directions.
        
        Returns a dict mapping a direction-key (sting) to a tuple (pts_entry, pts_exit).
        Each pts_* is a list of minimum 3 corner Points (entry polygon and exit polygon).

        Parameters
        ----------
        flip : bool
            Whether to flip the screw direction.
        data_type : str
            "points", "brep", "polylines".       
        """
        # NOTE: only works with 'crossed' entry type. all potential orientation types are calculated

        screw_directions = self._calculate_screw_directions()
        entry_frame, exit_frame = self._calculate_entry_exit_frames()

        boundaries = {}
        for key, dire in screw_directions.items():
            # 1. Find projection of a reference point to entry and exit frames along the screw direction
            # 2. Construct boundary Polygons based on the 'projected' points and the beam widths
            # 3. Store the boundaries in the dictionary with the same keys as screw_directions
            pass

        if data_type == "points":
            return boundaries

        # Placeholder: other data_type handling can be implemented later.
        raise NotImplementedError("find_screw_boundaries currently only supports data_type='points'.")