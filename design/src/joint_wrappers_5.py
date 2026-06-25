"""Timber joint wrappers."""
from screw_spec_2 import ScrewSpecification  # NOTE: temp.
from compas.tolerance import TOL
from compas.geometry import (
    Vector, Plane, Polyline, Point, Frame, NurbsCurve, Brep, Rotation, Transformation, 
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

def project_point_to_frame_signed(point, direction, frame, tol=1e-6):
    point = Point(*point)
    dire = Vector(*direction).unitized()

    P = frame.point
    N = frame.normal.unitized()
    denom = dot_vectors(dire, N)

    if abs(denom) < tol:
        raise ValueError("Direction is parallel to the plane.")

    num = dot_vectors(P - point, N)
    t = num / denom
    return point + dire * t

def is_same_xy_sign(direction):
    """Return True if direction.x and direction.y have the same sign (or zero)."""
    x, y = direction.x, direction.y
    return (x >= 0 and y >= 0) or (x <= 0 and y <= 0)

def find_average_point(points):
    cx = sum(p.x for p in points) / len(points)
    cy = sum(p.y for p in points) / len(points)
    cz = sum(p.z for p in points) / len(points)
    return Point(cx, cy, cz)

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

        elif self.entry_type == "butt_krossed":
            return -main_dire.unitized()
        
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
        elif data_type == "frame":
            return Frame.from_points(p0, p1, p3)
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
            "points", "frame", "brep", "polylines".
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
        
        elif data_type == "frame":
            frame_entry = Frame.from_points(pts_entry[0], pts_entry[1], pts_entry[3])
            frame_exit = Frame.from_points(pts_exit[0], pts_exit[1], pts_exit[3])
            return frame_entry, frame_exit
        
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
        return "butt_krossed"

    def _interface_plane(self):
        return Plane.from_frame(self.cross_ref_frame)

    def _project_vector_to_interface(self, vector):
        normal = Vector(*self._interface_plane().normal).unitized()
        projected = Vector(*vector) - normal * dot_vectors(vector, normal)
        if projected.length <= 1e-6:
            raise ValueError("Vector is perpendicular to the TButt interface plane.")
        return projected.unitized()

    def _interface_axes(self):
        normal = Vector(*self._interface_plane().normal).unitized()
        vertical = Vector(0,0,1)
        width = Vector(*cross_vectors(vertical, normal)).unitized()
        return vertical, width, normal

    def _sort_interface_points(self, points):
        center = find_average_point(points)
        vertical, width, _ = self._interface_axes()

        def vcoord(point):
            return dot_vectors(point - center, vertical)

        def wcoord(point):
            return dot_vectors(point - center, width)

        bottom = sorted(points, key=vcoord)[:2]
        top = sorted(points, key=vcoord)[2:]
        bottom = sorted(bottom, key=wcoord)
        top = sorted(top, key=wcoord)
        return (bottom[0], bottom[1], top[1], top[0])

    def _find_interface_points(self):
        interface_plane = self._interface_plane()
        side_planes = [Plane.from_frame(side) for side in self.main_beam.ref_sides[:4]]
        points = []

        for i, side_a in enumerate(side_planes):
            for side_b in side_planes[i + 1:]:
                coords = intersection_plane_plane_plane(interface_plane, side_a, side_b)
                if coords is None:
                    continue
                point = Point(*coords)
                if any(point.distance_to_point(other) <= 1e-6 for other in points):
                    continue
                points.append(point)

        return self._sort_interface_points(points)

    def _butt_side_frames(self):
        _, width, _ = self._interface_axes()
        side_frames = list(self.main_beam.ref_sides[:4])

        def side_score(frame):
            normal = Vector(*frame.normal).unitized()
            return dot_vectors(normal, width)

        ordered = sorted(side_frames, key=side_score)
        return ordered[0], ordered[-1]

    def calculate_butt_krossed_screw_directions(self, angle=None):

        away_from_joint = self.calculate_screw_direction()
        towards_joint = -away_from_joint
        vertical, _, _ = self._interface_axes()

        candidates = [
            towards_joint.rotated(math.radians(angle), vertical).unitized(),
            towards_joint.rotated(math.radians(-angle), vertical).unitized(),
        ]

        directions = []
        used = set()
        for frame in self._butt_side_frames():
            normal = Vector(*frame.normal).unitized()
            candidate_index = min(
                range(len(candidates)),
                key=lambda index: dot_vectors(candidates[index], normal),
            )
            if candidate_index in used and len(candidates) == 2:
                candidate_index = 1 - candidate_index
            used.add(candidate_index)
            directions.append(candidates[candidate_index])
        return tuple(directions)

    def project_butt_krossed_entry_points(self, target_point, angle=None):
        side_frames = self._butt_side_frames()
        directions = self.calculate_butt_krossed_screw_directions(angle=angle)
        return tuple(
            project_point_to_frame_signed(target_point, -direction, side_frame)
            for side_frame, direction in zip(side_frames, directions)
        )

    def get_interface_boundary(self, data_type="points"):
        pts_entry = list(self._find_interface_points())
        if data_type == "points":
            return pts_entry
        if data_type == "frame":
            return Frame.from_points(pts_entry[0], pts_entry[1], pts_entry[3])
        if data_type == "polyline":
            return Polyline(pts_entry + [pts_entry[0]])
        return None

    def find_screw_boundaries(self, angle=None, flip=False, data_type="points"):
        if angle is not None and angle <= 0:
            raise ValueError("Angle should be larger than 0.")

        interface_pts = self.get_interface_boundary(data_type="points")
        side_frames = self._butt_side_frames()
        directions = self.calculate_butt_krossed_screw_directions(angle=angle)
        exit_frame = self.cross_beam.opp_side(self.cross_beam_ref_side_index)

        pts_entry = []
        pts_exit = []
        for side_frame, direction in zip(side_frames, directions):
            pts_entry.append([
                project_point_to_frame_signed(point, -direction, side_frame)
                for point in interface_pts
            ])
            pts_exit.append([
                project_point_to_frame_signed(point, direction, exit_frame)
                for point in interface_pts
            ])

        if data_type == "points":
            return tuple(pts_entry), tuple(pts_exit)

        if data_type == "frame":
            entry_frames = tuple(
                Frame.from_points(points[0], points[1], points[3])
                for points in pts_entry
            )
            exit_frames = tuple(
                Frame.from_points(points[0], points[1], points[3])
                for points in pts_exit
            )
            return entry_frames, exit_frames

        if data_type == "polylines":
            entry_polylines = [
                Polyline(points + [points[0]])
                for points in pts_entry
            ]
            exit_polylines = [
                Polyline(points + [points[0]])
                for points in pts_exit
            ]
            return entry_polylines, exit_polylines

        raise ValueError("Unsupported data type.")


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
        self.is_same_xy_sign = is_same_xy_sign(self.main_beam.centerline.direction)
        
        if self.is_same_xy_sign:
            self.main_beam_ref_side_index = joint.main_ref_side_index + 3           # NOTE: main_ref_side pointing 'inwards' where its 'normal' is orientated towards the cross_beam centerline
            self._adjacency_direction = -1
        else:
            self.main_beam_ref_side_index = joint.main_ref_side_index + 1           # NOTE: main_ref_side pointing 'inwards' where its 'normal' is orientated towards the cross_beam centerline
            self._adjacency_direction = 1

        self.cross_beam_ref_side_index = joint.cross_ref_side_indices[main_id][1]   # NOTE: cross_beam_ref_side pointing 'inwards' where its 'normal' is orientated towards the main_ref_side centerline
        super().__init__(joint)

    def _get_bisector_of_centerlines(self):
        if self._is_acute:
            return (self.main_beam.centerline.direction + self.cross_beam.centerline.direction).unitized()
        else:
            flipped_main_centerline = (-self.cross_beam.centerline.direction.x, -self.cross_beam.centerline.direction.y, self.cross_beam.centerline.direction.z)
            return (self.main_beam.centerline.direction + Vector(*flipped_main_centerline)).unitized()
        
    def determine_entry_type(self):
        return "krossed"  # kool entry type exclusive for KBMJ
        
    def _get_vertical_pivot(self):
        x = self.main_beam.centerline.direction
        y = cross_vectors(x, Vector(0,0,1))
        pivot = Vector(*cross_vectors(x,y))
        return pivot
    
    def _get_rotation_axis(self):
        return Vector(*cross_vectors(self.main_beam.centerline.direction, Vector(0,0,1))).unitized()
        
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

    def _calculate_screw_directions(self, angle=None):
        """Create a dictionary of candidate screw direction vectors.

        Keys:
            - 'sides' : sides direction
        """
        # NOTE: screw directions away from joint 'location'. used for projecting entry and exit
            
        screw_directions = {}
        
        # 1) Side Direction: 
        if self.entry_type == "krossed":    # Used for sides, specific to KBMJ
            centerline_dir = self.main_beam.centerline.direction
            pivot = self._get_vertical_pivot()

            # flip based on vector direction
            if self.is_same_xy_sign:
                pivot = -pivot

            rotation_a = Rotation.from_axis_and_angle(pivot, math.radians(angle))
            rotation_b = Rotation.from_axis_and_angle(pivot, math.radians(-angle))
            direction_a = centerline_dir.transformed(rotation_a).unitized()
            direction_b = centerline_dir.transformed(rotation_b).unitized()
            
            axis = self._get_rotation_axis()
            rotation_axis = Rotation.from_axis_and_angle(axis, math.radians(-angle))
            direction_axis = centerline_dir.transformed(rotation_axis).unitized()

        if direction_a and direction_b:
            screw_directions["sides"] = direction_a, direction_b
            screw_directions["bottom"] = direction_axis

        if self.entry_type == "crossed":
            pass

        return screw_directions

    def _calculate_entry_exit_frames(self, data_type="polyline", angle=None):
        """Return a dict mapping each candidate direction name to (entry_polyline, exit_polyline).
        """
        # get candidate screw directions as {name: Vector, ...}

        directions = self._calculate_screw_directions(angle=angle)
        pts_to_project_vertical = self.get_interface_boundary_vertical(data_type="points")
        pts_to_project_horizontal = self.get_interface_boundary_horizontal(data_type="points")
        pts_to_project = pts_to_project_vertical

        if not pts_to_project:
            return {}
        entry_frame_a = self.main_beam.front_side(self.main_beam_ref_side_index)
        entry_frame_b = self.main_beam.front_side(self.main_beam_ref_side_index + 2)  # opposite sides

        exit_frame = self.cross_beam.opp_side(self.cross_beam_ref_side_index)
        
        bottom_frame = self.main_beam.ref_sides[0]
        top_frame = Plane.from_frame(self.main_beam.ref_sides[2])
        
        result = {}
        if data_type == "polyline":
            for name, dire in directions.items():
                if name == "sides":
                    entry_pts_a = [project_point_to_frame_along(p, dire[0], entry_frame_a, tol=1e-6) for p in pts_to_project]
                    exit_pts_a = [project_point_to_frame_along(p, -dire[0], exit_frame, tol=1e-6) for p in pts_to_project]

                    entry_pts_b = [project_point_to_frame_along(p, dire[1], entry_frame_b, tol=1e-6) for p in pts_to_project]
                    exit_pts_b = [project_point_to_frame_along(p, -dire[1], exit_frame, tol=1e-6) for p in pts_to_project]

                    # close polylines
                    entry_poly_a = Polyline(entry_pts_a + [entry_pts_a[0]])
                    exit_poly_a = Polyline(exit_pts_a + [exit_pts_a[0]])

                    entry_poly_b = Polyline(entry_pts_b + [entry_pts_b[0]])
                    exit_poly_b = Polyline(exit_pts_b + [exit_pts_b[0]])

                    result[name] = [entry_poly_a, exit_poly_a],[entry_poly_b, exit_poly_b]

                elif name == "bottom":
                    entry_pts = [project_point_to_frame_along(p, dire, bottom_frame, tol=1e-6) for p in pts_to_project]
                    exit_pts = [project_point_to_frame_along(p, -dire, top_frame, tol=1e-6) for p in pts_to_project]
                    
                    # close polylines
                    entry_poly = Polyline(entry_pts + [entry_pts[0]])
                    exit_poly = Polyline(exit_pts + [exit_pts[0]])
                    
                    result[name] = (entry_poly, exit_poly)
                elif name == "top":
                    pass

        elif data_type == "points":
            for name, dire in directions.items():
                if name == "sides":
                    entry_pts_a = [project_point_to_frame_along(p, dire[0], entry_frame_a, tol=1e-6) for p in pts_to_project]
                    exit_pts_a = [project_point_to_frame_along(p, -dire[0], exit_frame, tol=1e-6) for p in pts_to_project]

                    entry_pts_b = [project_point_to_frame_along(p, dire[1], entry_frame_b, tol=1e-6) for p in pts_to_project]
                    exit_pts_b = [project_point_to_frame_along(p, -dire[1], exit_frame, tol=1e-6) for p in pts_to_project]

                    result[name] = [entry_pts_a, exit_pts_a],[entry_pts_b, exit_pts_b]
                    
                elif name == "bottom":
                    entry_pts = [project_point_to_frame_along(p, dire, bottom_frame, tol=1e-6) for p in pts_to_project]
                    exit_pts = [project_point_to_frame_along(p, -dire, top_frame, tol=1e-6) for p in pts_to_project]
                    
                    result[name] = (entry_pts, exit_pts)

        return result
    
    @property
    def vertical_exit_frame(self):
        return self.cross_beam.opp_site(self.cross_beam_ref_side_index)

    def find_screw_boundaries(self, angle=None, flip=False, data_type="points"):
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
        # NOTE: only works with 'krossed' entry type. Other types are not kool.

        if angle is not None:
            if angle <= 0:
                raise ValueError("Angle should be larger than 0")

        if self.entry_type == "krossed":
            result = {}
            pts_side_a, pts_side_b = self._calculate_entry_exit_frames(angle=angle, data_type="points")["sides"] # NOTE: should return 2 lists of lists. 1 for 'front side' and one for 'opposite side'
            pts_entry_bottom, pts_exit_bottom = self._calculate_entry_exit_frames(angle=angle, data_type="points")["bottom"]
            
            result["sides"] = [(pts_side_a[0], pts_side_b[0]), (pts_side_a[1], pts_side_b[1])]
            result["bottom"] = (pts_entry_bottom, pts_exit_bottom)
            
        else:
            raise ValueError("Invalid entry type")
        
        if data_type == "points":
            return result
        elif data_type == "polylines":
            result = {}
            pts_side_a, pts_side_b = self._calculate_entry_exit_frames(angle=angle, data_type="polyline")["sides"] # NOTE: should return 2 lists of lists. 1 for 'front side' and one for 'opposite side'
            pts_entry_bottom, pts_exit_bottom = self._calculate_entry_exit_frames(angle=angle, data_type="polyline")["bottom"]
            
            result["sides"] = [(pts_side_a[0], pts_side_b[0]), (pts_side_a[1], pts_side_b[1])]
            result["bottom"] = (pts_entry_bottom, pts_exit_bottom)
            return result
        # Placeholder: other data_type handling can be implemented later.
        raise NotImplementedError("find_screw_boundaries currently only supports data_type='points'.")


class LMJ(BaseWrapper):
    """Wrapper for middle-node LMiterJoint screw planning."""

    SCREW_ANGLE = 20
    SCREW_SPACING = 0.030
    CROSS_LINE_SEPARATION = 0.5
    # Each value is one whole cross offset from the interface middle.
    # Examples: (0,) center only, (0, 1) center + one side, (-1, 1) two sides.
    CROSS_OFFSETS = (0, 1)

    def __init__(self, joint):
        self.main_beam = joint.beam_a
        self.cross_beam = joint.beam_b
        
        self.cutting_planes = tuple(self._to_plane(p) for p in joint._get_cutting_planes())
        self.main_beam_ref_side_index = int(self.main_beam.attributes.get("ref_side_index", 0)) % 4
        self.cross_beam_ref_side_index = int(self.cross_beam.attributes.get("ref_side_index", 0)) % 4
        super().__init__(joint)

    def determine_entry_type(self):
        return "middle"

    # ── Geometry helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _to_plane(value):
        if hasattr(value, "point") and hasattr(value, "normal"):
            return Plane(value.point, value.normal)
        return Plane.from_frame(value)

    @staticmethod
    def _same_guid(a, b):
        return str(a.guid) == str(b.guid)

    @staticmethod
    def _planes_equal(a, b, tol=1e-6):
        na, nb = Vector(*a.normal).unitized(), Vector(*b.normal).unitized()
        return (
            abs(abs(dot_vectors(na, nb)) - 1.0) <= tol
            and abs(dot_vectors(b.point - a.point, na)) <= tol
        )

    @staticmethod
    def _dedupe_points(points, tol=1e-6):
        unique = []
        for p in points:
            if not any(p.distance_to_point(q) <= tol for q in unique):
                unique.append(p)
        return unique

    @staticmethod
    def _sort_on_plane(points, plane):
        center = find_average_point(points)
        normal = Vector(*plane.normal).unitized()
        u = Vector(*cross_vectors(Vector(0, 0, 1), normal))
        u = Vector(1, 0, 0) if u.length <= 1e-6 else u.unitized()
        v = Vector(*cross_vectors(normal, u)).unitized()
        return sorted(points, key=lambda p: math.atan2(dot_vectors(p - center, v), dot_vectors(p - center, u)))

    @staticmethod
    def _is_inside(point, planes, tol=1e-6):
        return all(dot_vectors(point - pl.point, pl.normal) <= tol for pl in planes)

    @staticmethod
    def _area_on_plane(points, plane):
        if len(points) < 3:
            return 0.0
        frame = Frame.from_plane(plane)
        xy = [(frame.to_local_coordinates(p).x, frame.to_local_coordinates(p).y) for p in points]
        return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(xy, xy[1:] + xy[:1]))) * 0.5

    # ── Clipping ──────────────────────────────────────────────────────────────

    def _clip_by_plane(self, points, plane, tol=1e-6):
        if not points:
            return []
        pts = list(points)
        dists = [dot_vectors(p - plane.point, plane.normal) for p in pts]
        clipped = []
        for i, (cur, cd) in enumerate(zip(pts, dists)):
            prev, pd = pts[i - 1], dists[i - 1]
            if cd <= tol:
                if pd > tol:
                    t = pd / (pd - cd)
                    clipped.append(prev + (cur - prev) * t)
                clipped.append(cur)
            elif pd <= tol:
                t = pd / (pd - cd)
                clipped.append(prev + (cur - prev) * t)
        return self._dedupe_points(clipped, tol=tol)

    def _clip_by_planes(self, points, planes):
        for plane in planes:
            points = self._clip_by_plane(points, plane)
            if len(points) < 3:
                break
        return points

    # ── Beam / joint queries ──────────────────────────────────────────────────

    def _cutting_plane_for_beam(self, joint, beam):
        if type(joint).__name__ != "LMiterJoint":
            return None
        planes = tuple(self._to_plane(p) for p in joint._get_cutting_planes())
        if self._same_guid(joint.beam_a, beam):
            return planes[0]
        if self._same_guid(joint.beam_b, beam):
            return planes[1]
        return None

    def _cutting_planes_for_beam(self, beam):
        model = getattr(beam, "model", None)
        joints = model.get_joints_for_element(beam) if model else (self._raw_joint,)
        planes = []
        for joint in joints:
            plane = self._cutting_plane_for_beam(joint, beam)
            if plane and not any(self._planes_equal(plane, q) for q in planes):
                planes.append(plane)
        return planes

    def _top_side_frame(self, beam):
        return beam.ref_sides[max(range(4), key=lambda i: Vector(*beam.ref_sides[i].normal).unitized().z)]

    def _middle_point(self, interface_center):
        location = getattr(self._raw_joint, "location", None)
        if location is not None:
            return Point(*location)

        endpoints = []
        for beam in (self.main_beam, self.cross_beam):
            endpoints.append(min(beam.centerline, key=lambda p: p.distance_to_point(interface_center)))
        return find_average_point(endpoints)

    def _orient_spread_axis_outward(self, spread_axis, interface_center):
        middle = self._middle_point(interface_center)
        outward = Vector.from_start_end(middle, interface_center)
        if outward.length <= 1e-6:
            return spread_axis
        if dot_vectors(outward, spread_axis) < 0:
            return -spread_axis
        return spread_axis

    # ── Section geometry ──────────────────────────────────────────────────────

    def _intersect_plane_with_beam(self, beam, plane):
        side_planes = [Plane.from_frame(s) for s in beam.ref_sides]
        cutting_planes = [p for p in self._cutting_planes_for_beam(beam) if not self._planes_equal(p, plane)]

        raw = []
        for i, sa in enumerate(side_planes):
            for sb in side_planes[i + 1:]:
                try:
                    coords = intersection_plane_plane_plane(plane, sa, sb)
                except TypeError:
                    continue
                if coords:
                    raw.append(Point(*coords))

        raw = self._dedupe_points(raw)
        inside = [p for p in raw if self._is_inside(p, side_planes)]
        points = self._sort_on_plane(inside if len(inside) >= 3 else raw, plane)

        if len(points) < 3:
            raise ValueError("Could not find an LMiter interface section for beam {}.".format(beam.name))

        if not cutting_planes:
            return points

        clipped = self._clip_by_planes(points, cutting_planes)
        if len(clipped) >= 3:
            return self._sort_on_plane(clipped, plane)

        candidates = [
            (self._area_on_plane(c, plane), self._sort_on_plane(c, plane))
            for cp in cutting_planes
            if len(c := self._clip_by_plane(points, cp)) >= 3
        ]
        if candidates:
            return max(candidates)[1]

        raise ValueError("Could not find an LMiter interface section for beam {}.".format(beam.name))

    def _find_interface_points(self):
        from compas.geometry import Polygon, boolean_intersection_polygon_polygon

        plane = self.cutting_planes[0]
        frame = Frame.from_plane(plane)

        def to_polygon(beam):
            return Polygon([frame.to_local_coordinates(p) for p in self._intersect_plane_with_beam(beam, plane)])

        xy = boolean_intersection_polygon_polygon(to_polygon(self.main_beam), to_polygon(self.cross_beam))
        if len(xy) < 3:
            raise ValueError("Could not find the LMiter interface overlap between the two beams.")

        points = self._dedupe_points([frame.to_world_coordinates(Point(x, y, 0.0)) for x, y in xy])
        if len(points) < 3:
            raise ValueError("Could not find the LMiter interface overlap between the two beams.")

        return self._sort_on_plane(points, plane)

    # ── Entry points ──────────────────────────────────────────────────────────

    def _entry_on_cross_plane(self, beam, cross_plane, target):
        section = self._intersect_plane_with_beam(beam, cross_plane)
        top_plane = Plane.from_frame(self._top_side_frame(beam))
        top_pts = [p for p in section if abs(dot_vectors(p - top_plane.point, top_plane.normal)) <= 1e-5]

        if len(top_pts) >= 2:
            start, end = max(
                ((a, b) for i, a in enumerate(top_pts) for b in top_pts[i + 1:]),
                key=lambda pair: pair[0].distance_to_point(pair[1]),
            )
            axis = Vector.from_start_end(start, end)
            proj = project_point_to_frame_signed(target, top_plane.normal, top_plane)
            t = max(0.0, min(1.0, dot_vectors(proj - start, axis) / dot_vectors(axis, axis)))
            return start + axis * t

        return max(section, key=lambda p: dot_vectors(p - top_plane.point, top_plane.normal))

    def _copied_cross_plane_targets(self, center, spread_axis, interface_planes):
        for offset_count in self.CROSS_OFFSETS:
            target = Point(*(center + spread_axis * (float(offset_count) * self.SCREW_SPACING)))
            if not self._is_inside(target, interface_planes, tol=1e-3):
                continue
            yield target

    def _entry_from_target_on_top(self, beam, target, direction):
        top_frame = self._top_side_frame(beam)
        beam_planes = [Plane.from_frame(s) for s in beam.ref_sides]

        try:
            entry = project_point_to_frame_signed(target, -direction, top_frame)
        except ValueError:
            return None

        if not self._is_inside(entry, beam_planes, tol=1e-6):
            return None
        return Point(*entry)

    def populate_entry_points(self, angle=None):
        if angle is not None and angle <= 0:
            raise ValueError("Angle must be greater than 0.")

        screw_angle = 40.0
        interface = self._find_interface_points()
        center = find_average_point(interface)
        cutting_normal = Vector(*self.cutting_planes[0].normal).unitized()
        spread_axis = Vector(*cross_vectors(Vector(0, 0, 1), cutting_normal))
        if spread_axis.length <= 1e-6:
            spread_axis = self.main_beam.centerline.direction
        spread_axis = spread_axis.unitized()
        rotation_axis = spread_axis
        copy_axis = self._orient_spread_axis_outward(spread_axis, center)

        # Bound the spread to the interface width along the spread axis
        projections = [dot_vectors(p - center, copy_axis) for p in interface]
        interface_planes = [
            Plane(center + copy_axis * max(projections),  copy_axis),
            Plane(center + copy_axis * min(projections), -copy_axis),
        ]

        # Each screw is cutting_normal rotated +/-SCREW_ANGLE around the spread axis.
        # Main beam: ray goes down into the joint at +angle, cross beam at -angle.
        # From the top view the lines look perpendicular; in 3D they are angled inward.
        direction_specs = {
            "main": (cutting_normal, 1.0),
            "cross": (-cutting_normal, -1.0),
        }

        beam_specs = []
        for beam_index, (beam, kind) in enumerate(
            ((self.main_beam, "main"), (self.cross_beam, "cross"))
        ):
            base_normal, angle_sign = direction_specs[kind]
            screw_dir = base_normal.rotated(math.radians(angle_sign * screw_angle), rotation_axis).unitized()
            beam_specs.append({"beam": beam, "beam_index": beam_index, "kind": kind, "direction": screw_dir, "entries": []})

        line_separation = copy_axis * (self.SCREW_SPACING * self.CROSS_LINE_SEPARATION * 0.5)
        line_signs = {"main": -1.0, "cross": 1.0}
        separation_scales = (1.0, 0.5, 0.25, 0.0)

        for target in self._copied_cross_plane_targets(center, copy_axis, interface_planes):
            for scale in separation_scales:
                entries = []
                for spec in beam_specs:
                    line_target = Point(*(target + line_separation * line_signs[spec["kind"]] * scale))
                    entry = self._entry_from_target_on_top(spec["beam"], line_target, spec["direction"])
                    if entry is None:
                        try:
                            entry = self._entry_on_cross_plane(
                                spec["beam"],
                                Plane(line_target, copy_axis),
                                line_target,
                            )
                        except ValueError:
                            entry = None
                    if entry is None:
                        break
                    entries.append((spec, entry))
                else:
                    for spec, entry in entries:
                        spec["entries"].append((entry, spec["direction"]))
                    break

        for spec in beam_specs:
            if not spec["entries"]:
                continue

            points, directions = zip(*spec["entries"])

            yield {
                "beam": spec["beam"],
                "beam_index": spec["beam_index"],
                "points": tuple(points),
                "directions": tuple(directions),
            }

    def find_screw_boundaries(self, angle=None, flip=False, data_type="points"):
        interface = self._find_interface_points()
        entries = list(self.populate_entry_points(angle=angle))
        if data_type == "points":
            return tuple(item["points"] for item in entries), tuple(interface)
        if data_type == "polyline":
            return (
                tuple(Polyline(list(item["points"]) + [item["points"][0]]) for item in entries),
                Polyline(interface + [interface[0]]),
            )
        raise ValueError("Unsupported data_type: {}".format(data_type))
