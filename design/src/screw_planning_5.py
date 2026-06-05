"""Timber screw planning."""

from joint_wrappers_3 import TMSJ, TSJ, TBJ, TBMJ, KBMJ, project_point_to_frame_along, find_average_point # NOTE: temp.
from screw_spec_2 import ScrewSpecification
from screw import Screw, RejectReason
from compas.geometry import (
    Frame, Line, Cylinder, Polyline,
    cross_vectors, angle_vectors,
    KDTree, Point, Vector
)
from compas_timber.connections import (
    TMultiStepJoint, TStepJoint, TButtJoint, TBirdsmouthJoint, KBirdsmouthJoint
)
from compas_timber.fabrication import Drilling
import math


# -----------------------------------
# Screw Solver
# -----------------------------------
class ScrewSolver:
    JOINT_MAP = {
        TMultiStepJoint : TMSJ,
        TStepJoint : TSJ,
        TButtJoint : TBJ,
        TBirdsmouthJoint : TBMJ,
        KBirdsmouthJoint : KBMJ,
    }

    def __init__(self, model, spec_model="WT-plus-6.5"):
        self.model = model
        self.joints = model.joints
        self.spec_model = spec_model

        self._spec_cache = {
            "aligned": ScrewSpecification("aligned", spec_model=spec_model),
            "crossed": ScrewSpecification("crossed", spec_model=spec_model),
            "krossed": ScrewSpecification("crossed", spec_model=spec_model),  # temp. use the same spec for "krossed" entry type
        }

        self.capacity_warnings = []

    def cluster_joints(self, distance=0.20):
        kdtree = KDTree(self.joints)
        pass

    def are_screws_collided(self, cluster):
        pass
    
    def get_specification(self, joint):
        """Get the screw specification based on the joint's entry type."""
        return self._spec_cache[joint.entry_type]

    @staticmethod
    def convert_to_acute_angle(angle, deg=False):
        """Convert the angle to an acute angle."""
        if deg:
            angle = math.radians(angle)
        if angle < 0 or angle > math.pi:
            raise ValueError("Angle should be between 0 and 180 degrees.")
        if angle > math.pi / 2:
            return math.pi - angle
        return angle

    def shrink_aligned_entry_corners(self, joint, angle):
        """Shrink the entry face corners for aligned entry type with the constraint of screw penetration."""
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        dire = joint.calculate_screw_direction(angle=angle)

        # 1. Offset the corners where the screws will start to be placed.
        angle_dire_cross = angle_vectors(dire, joint.cross_beam.centerline.direction)
        angle_dire_cross = self.convert_to_acute_angle(angle_dire_cross)
        dist_perp_to_cross = math.sin(angle_dire_cross) * spec.penetration
        amp_start = dist_perp_to_cross / math.sin(math.radians(joint.acute_angle))

        # 2. Offset the corners where the screws will end to be placed.
        angle_dire_main = angle_vectors(dire, joint.main_beam.centerline.direction)
        angle_dire_main = self.convert_to_acute_angle(angle_dire_main)
        amp_end = spec.SCREW_DIAMETER / math.sin(angle_dire_main)  # The screw will attach the edge if divided by 2.0
        
        entry_corners, _ = joint.find_screw_boundaries(angle=angle, data_type="points")
        offset_dire = (entry_corners[1] - entry_corners[0]).unitized()

        for i, corner in enumerate(entry_corners):
            if i in (0, 3):
                entry_corners[i] = corner + offset_dire * amp_start
            else:
                entry_corners[i] = corner - offset_dire * amp_end
        return entry_corners

    def calculate_screw_capacity(self, joint, angle=None):
        """Calculate the maximum amount of screws in width and length direction."""        
        if joint.entry_type == "aligned":
            spec = self._spec_cache["aligned"]
            entry_corners = self.shrink_aligned_entry_corners(joint, angle=angle)
            len_l = entry_corners[0].distance_to_point(entry_corners[1])

            max_w_num = 3  # Spacings (25, 25, 25, 25)

            dire = joint.calculate_screw_direction(angle=angle)
            main_dire = joint.main_beam.centerline.direction
            acute_angle = angle_vectors(dire, main_dire)
            acute_angle = self.convert_to_acute_angle(acute_angle)

            l_spacing = spec.a1 / math.sin(acute_angle)
            max_l_num = len_l // l_spacing + 1

        elif joint.entry_type == "crossed":
            spec = self._spec_cache["crossed"]
            entry_corners, _ = joint.find_screw_boundaries(data_type="points")
            len_l = entry_corners[0].distance_to_point(entry_corners[1])

            max_w_num = 2  # Left and right each along width
            table = sorted(spec.spec_table["min_widths"].items(), key=lambda x: x[0], reverse=True)
            max_l_num = next((pair_count for pair_count, min_width in table if len_l >= min_width), 0)

        elif joint.entry_type == "krossed":
            spec = self._spec_cache["krossed"]
            entry_corners_a, _ = joint._calculate_entry_exit_frames(angle=angle, data_type="points")["sides"][0]
            entry_corners_b, _ = joint._calculate_entry_exit_frames(angle=angle, data_type="points")["sides"][1]
            entry_bottom, _ = joint._calculate_entry_exit_frames(angle=angle, data_type="points")["bottom"]
            
            len_l_a = entry_corners_a[0].distance_to_point(entry_corners_a[1])
            len_l_b = entry_corners_b[0].distance_to_point(entry_corners_b[1])
            len_l = len_l_a if len_l_a < len_l_b else len_l_b       # Take smaller value of the two sides. or average?

            max_w_num = 2
            table = sorted(spec.spec_table["min_widths"].items(), key=lambda x: x[0], reverse=True)
            # max_l_num = next((pair_count for pair_count, min_width in table if len_l >= min_width), 0)
            max_l_num = 1

        else:
            raise ValueError("Unknown entry type.")
        return int(max_w_num), int(max_l_num)
        # TODO: implement the a1_cg (65mm) check with "aligned" entry type

    @staticmethod
    def calculate_screw_distributions(amount, max_width_number=3):
        """Calculate the screw distributions in width and length direction based on the requested amount of screws."""
        if amount < 0:
            raise ValueError("Amount should be non-negative.")
        elif amount == 0:
            return []
        
        distributions = []
        for w_num in range(max_width_number, 0, -1):
            l_num = (amount + w_num - 1) // w_num  # ceiling division
            distributions.append((w_num, l_num))
        return distributions

    def populate_aligned_entry_points(self, joint, angle, amount: int = None):
        """
        Populate screw points on the entry face.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        angle : float
            The angle to determine the screw direction for aligned entry type.
        amount : int or None
            The requested amount of screws. If None, use the maximum capacity.
        
        Returns
        -------
        bool, list of list of Point
            Success flag and the entry points to create screw lines.
        """
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        if amount == 0:
            return False, None
        
        # 1. Calculate the maximum screw number in width and length
        max_w_num, max_l_num = self.calculate_screw_capacity(joint, angle=angle)
        if max_w_num > 3:
            raise NotImplementedError("Only support up to 3 screws in one row for now.")

        # 2. If requested amount is not provided or is more than the maximum capacity, use the maximum capacity.
        #    Otherwise, calculate the screw distributions by prioritizing the length direction to get more rows.
        if amount is None:
            pass
        elif amount <= max_w_num * max_l_num:
            distributions = self.calculate_screw_distributions(amount, max_w_num)
            # Length direction is the priority to fill
            distributions.sort(key=lambda x: x[1], reverse=True)
            best = next(((w_num, l_num) for w_num, l_num in distributions if l_num <= max_l_num))
            max_w_num, max_l_num = best
        else:
            self.capacity_warnings.append({
                "joint_guid": joint.guid,
                "joint": joint,
                "requested": amount,
                "capacity": (max_w_num, max_l_num),
                "screw_angle": angle,
                })
            success = False
            return success, None
        
        # 3. Create the step distances in width
        w_steps_map = {
            1: (spec.a2_cg * 2,) * 2,
            2: (spec.a2_cg, spec.a2, spec.a2_cg),
            3: (spec.a2_cg,) * 4,
        }
        w_steps = w_steps_map[max_w_num]

        # 4. Populate entry points
        pts_entry = self.shrink_aligned_entry_corners(joint, angle=angle)
        l = pts_entry[0].distance_to_point(pts_entry[1])
        l_step = l / (max_l_num - 1)

        vec_w = (pts_entry[3] - pts_entry[0]).unitized()
        vec_l = (pts_entry[1] - pts_entry[0]).unitized()

        # Start at the first offset point
        start = pts_entry[0] + (vec_w * w_steps[0])

        point_grid = [[None for _ in range(max_w_num)] for _ in range(max_l_num)]
        for i in range(max_l_num):
            curr_w = 0.0
            for j in range(max_w_num):
                point_grid[i][j] = start + (vec_w * curr_w) + (vec_l * i * l_step)
                curr_w += w_steps[j + 1]
        success = True
        return success, point_grid

    def regenerate_aligned_entry_points(self, joint):
        """Regenerate entry points when the requested screw amount is larger than the maximum capacity."""
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        if not self.capacity_warnings:
            return
        warning = next((w for w in self.capacity_warnings if w["joint_guid"] == joint.guid), None)

        requested = warning["requested"]
        w_num, _ = warning["capacity"]
        new_l_num = math.ceil(requested / w_num)
        
        # 3. Create the step distances in width
        w_steps_map = {
            1: (spec.a2_cg * 2,) * 2,
            2: (spec.a2_cg, spec.a2, spec.a2_cg),
            3: (spec.a2_cg,) * 4,
        }
        w_steps = w_steps_map[w_num]

        # 4. Populate entry points
        screw_angle = warning["screw_angle"]

        dire = joint.calculate_screw_direction(angle=screw_angle)
        main_dire = joint.main_beam.centerline.direction
        angle_main_screw = angle_vectors(dire, main_dire)
        angle_main_screw = self.convert_to_acute_angle(angle_main_screw)
        l_step = spec.a1 / math.sin(angle_main_screw)  # Fix the step

        pts_entry = self.shrink_aligned_entry_corners(joint, angle=screw_angle)
        vec_w = (pts_entry[3] - pts_entry[0]).unitized()
        vec_l = (pts_entry[1] - pts_entry[0]).unitized()

        # Start at the first offset point
        start = pts_entry[0] + (vec_w * w_steps[0])

        point_grid = [[None for _ in range(w_num)] for _ in range(new_l_num)]
        for i in range(new_l_num):
            curr_w = 0.0
            for j in range(w_num):
                point_grid[i][j] = start + (vec_w * curr_w) + (vec_l * i * l_step)
                curr_w += w_steps[j + 1]
        return point_grid
    
    def populate_crossed_entry_points(self, joint, amount: int = None):
        """
        Populate screw points on the entry face from the height sides.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        amount : int or None
            The requested amount of screws. If None, use the maximum capacity.
        orientation : str
            "perp_main", "perp_riser", "perp_cross", "along_main", and "bisector".

        Returns
        -------
        list of list of Point
            Inner list is along the width direction.
        """
        if joint.entry_type != "crossed":
            raise ValueError("Wrong entry type. Expected 'crossed'.")
        spec = self._spec_cache["crossed"]

        # Find offset entry points
        corners = joint.get_interface_boundary(data_type="points")
        offset_dire = joint.calculate_screw_direction()
        amp = math.cos(math.radians(spec.side_angle)) * spec.penetration
        offset_vec = offset_dire * amp

        pts_entry = [p + offset_vec for p in corners]

        # Calculate screw amount and check with capacity
        w_num, l_num = self.calculate_screw_capacity(joint)
        if amount is None:
            pass
        elif amount <= w_num * l_num:
            l_num = (amount // w_num) + (1 if amount % w_num > 0 else 0)
        else:
            self.capacity_warnings.append(
                {"joint_guid": joint.guid, "joint": joint, "requested": amount, "capacity": (w_num, l_num)}
            )
            pass
        if l_num > 3:
            raise NotImplementedError("Only support up to 3 pairs for now.")
        
        l = pts_entry[0].distance_to_point(pts_entry[1])
        # TODO: maybe fix a2_cg rather than fixing a1
        a2_cg = (l - spec.a2_red - spec.a1 * (l_num - 1)) / 2  # screw center to edge

        vec_l = (pts_entry[1] - pts_entry[0]).unitized()
        point_grid = []
        for i in range(l_num):
            pt_left = pts_entry[0] + (vec_l * a2_cg) + (vec_l * spec.a1) * i
            pt_right = pts_entry[3] + (vec_l * a2_cg) + (vec_l * spec.a1) * i + (vec_l * spec.a2_red)
            point_grid.append([pt_left, pt_right])
        return point_grid

    def populate_krossed_entry_points(self, joint, angle, amount: int = None):
        """
        Populate screw points on the entry face from the height sides.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        amount : int or None
            The requested amount of screws. If None, use the maximum capacity.
        orientation : str
            "sides"

        Returns
        -------
        lists of list of Point
            Inner list is along the width direction.
            Returns two list, acute, then obtuse sides.
        """
        if joint.entry_type != "krossed":
            raise ValueError("Wrong entry type. Not kool...")
        spec = self._spec_cache["krossed"]

        entry_points = {}
        
        # Find offset entry points for side A (acute)
        corners_a, _ = joint._calculate_entry_exit_frames(angle=angle, data_type="points")["sides"][0]
        offset_dire_a = joint._calculate_screw_directions(angle=angle)["sides"][0]
        entry_points["side_a"] = find_average_point(corners_a)
        
        # Find offset entry points for side B (obtuse)
        corners_b, _ = joint._calculate_entry_exit_frames(angle=angle, data_type="points")["sides"][1]
        offset_dire_a = joint._calculate_screw_directions(angle=angle)["sides"][1]
        entry_points["side_b"] = find_average_point(corners_b)

        # Find offset entry points for bottom
        entry_bottom, _ = joint._calculate_entry_exit_frames(angle=angle, data_type="points")["bottom"]
        
        # Test: get center point of the entry 
        cx = sum(p.x for p in entry_bottom) / len(entry_bottom)
        cy = sum(p.y for p in entry_bottom) / len(entry_bottom)
        cz = sum(p.z for p in entry_bottom) / len(entry_bottom)
        
        entry_points["bottom"] = Point(cx, cy, cz)
        
        return entry_points


    @staticmethod
    def generate_screw(entry_point, direction, interface, exit_frame, spec, screw_lengths):
        """
        Generate a single screw instance considering the penetration and screw length constraints.

        Parameters
        ----------
        entry_point : Point
            The entry point of the screw.
        direction : Vector
            The direction of the screw.
        interface : Frame
            The interface frame for the screw.
        exit_frame : Frame
            The exit frame for the screw.
        spec : ScrewSpecification
            The screw specification.
        screw_lengths : list of float
            The available screw lengths, from LONGEST to shortest.

        Returns
        -------
        Screw
            A screw instance with properties set based on the constraints.
        """
        dist_entry = project_point_to_frame_along(entry_point, direction, interface).distance_to_point(entry_point)
        pt_exit = project_point_to_frame_along(entry_point, direction, exit_frame)
        depth = entry_point.distance_to_point(pt_exit)

        # 0. Create an unplanned screw instance
        screw = Screw(entry_point, direction, spec.SCREW_DIAMETER)
        screw.dist_in_entry = dist_entry
        screw.available_depth = depth
        
        # 1. Check if sufficient penetration in exit beam is achieved
        dist_exit = depth - dist_entry
        if dist_exit < spec.penetration:
            screw.is_valid = False
            screw.reject_reason = RejectReason.EXIT_MATERIAL_TOO_THIN
            screw.dist_in_exit = dist_exit
            return screw
        
        # 2.
        min_required_length = dist_entry + spec.penetration
        valid_screws = [l for l in screw_lengths if l >= min_required_length]
        if not valid_screws:
            screw.is_valid = False
            screw.length = screw_lengths[0]  # The longest screw in the catalog
            screw.dist_in_exit = screw.length - dist_entry

            if (spec.penetration * 2) > screw.length:
                screw.reject_reason = RejectReason.SPEC_MAX_INSUFFICIENT
            else:
                screw.reject_reason = RejectReason.ENTRY_MATERIAL_TOO_THICK  # Do counterbore
            return screw
        
        # 3. 
        max_allowed_length = depth - spec.BACK_THRESHOLD
        suitable_length = next((l for l in valid_screws if l <= max_allowed_length), None)
        if suitable_length is None:
            screw.is_valid = False
            screw.length = valid_screws[-1]
            screw.reject_reason = RejectReason.EXIT_PROTRUSION  # Protuding out of the exit face
            screw.dist_in_exit = screw.length - dist_entry
            return screw
        
        # 4.
        screw.is_valid = True
        screw.length = suitable_length
        screw.dist_in_exit = suitable_length - dist_entry
        return screw
    
    def populate_aligned_screws(self, joint, angle, point_grid):
        """
        Populate screws from the entry face to the exit face.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        angle : float
            The angle to determine the screw direction for aligned entry type.
        point_grid : list of list of Point
            The entry points to create screw lines.

        Returns
        -------
        list of Screw
            A list of Screw instances with properties for future evaluation.
        """
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]
        screw_lengths = sorted(spec.SCREW_LENGTHS, reverse=True)

        interface = joint.get_interface_boundary(data_type="frame")
        
        # 2. Create exit frame and screw direction
        dire = joint.calculate_screw_direction(angle=angle)
        _, exit_frame = joint.find_screw_boundaries(angle=angle, data_type="frame")

        # 3. get the length of each screw line by projecting the point to the exit face
        screw_list = []
        for i, row in enumerate(point_grid):
            for j, pt_entry in enumerate(row):
                screw = self.generate_screw(pt_entry, dire, interface, exit_frame, spec, screw_lengths)
                screw.joint_guid = joint.guid
                screw.joint_type = joint.__class__.__name__
                screw.position = (i, j)

                screw_list.append(screw)
        return screw_list
    
    def populate_crossed_screws(self, joint, point_grid):
        """
        Populate screws from the height sides diagonally.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        point_grid : list of list of Point
            The entry points to create screw lines.
        Returns
        -------
        list of Screw
            A list of Screw instances with properties for future evaluation.
        """
        if joint.entry_type != "crossed":
            raise ValueError("Wrong entry type. Expected 'crossed'.")
        spec = self._spec_cache["crossed"]
        screw_lengths = sorted(spec.SCREW_LENGTHS, reverse=True)

        interface = joint.get_interface_boundary(data_type="frame")
        exit_frames = [joint.cross_beam.ref_sides[(joint.cross_beam_ref_side_index + i) % 4] for i in range(1, 4)]

        # Find rotate axis
        dire = -joint.calculate_screw_direction()
        normal = joint.main_beam.front_side(joint.main_beam_ref_side_index).normal
        rotate_axis = cross_vectors(dire, normal)
    
        screw_list = []
        for i, row in enumerate(point_grid):
            for j, pt_entry in enumerate(row):
                sign = -1 if j % 2 == 0 else +1
                side_vec = dire.rotated(sign * math.radians(spec.side_angle), rotate_axis)

                exit_frame = None
                min_dist = float("inf")
                for ef in exit_frames:
                    sample_exit = project_point_to_frame_along(pt_entry, side_vec, ef)
                    dist = pt_entry.distance_to_point(sample_exit)
                    if math.isclose(dist, 0, rel_tol=1e-3, abs_tol=1e-3):
                        continue
                    if dist < min_dist:
                        min_dist = dist
                        exit_frame = ef
    
                screw = self.generate_screw(pt_entry, side_vec, interface, exit_frame, spec, screw_lengths)
                screw.joint_guid = joint.guid
                screw.joint_type = joint.__class__.__name__
                screw.position = (i, j)

                screw_list.append(screw)
        return screw_list

    def populate_krossed_screws(self, joint, angle, point_grid):
        """
        Populate screw lines from the entry face to the exit face.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        angle : float
            The angle to determine the screw direction for aligned entry type.
        point_grid : list of list of Point
            The entry points to create screw lines.
        Returns
        -------
        list of Screw
            A flat list of Screw instances created for the provided point_grid.
        """

        if joint.entry_type != "krossed":
            raise ValueError("Wrong entry type. Expected 'krossed'. Not kool...")
        spec = self._spec_cache["krossed"]

        screw_lengths = sorted(spec.SCREW_LENGTHS, reverse=True)

        # interface frame from vertical boundary
        corners = joint.get_interface_boundary_vertical(data_type="points")
        interface = Frame.from_points(corners[0], corners[1], corners[2])

        # get entry/exit frames and screw directions for both sides and bottom (if present)
        frames_data = joint._calculate_entry_exit_frames(angle=angle, data_type="points")
        sides_info = frames_data.get("sides", [])
        bottom_info = frames_data.get("bottom", (None, None))

        screw_dirs = joint._calculate_screw_directions(angle=angle)
        screw_dirs_sides = screw_dirs.get("sides", [])
        screw_dir_bottom = screw_dirs.get("bottom", None)

        # precompute exit frames and corresponding base directions in a stable order:
        # [side_a, side_b, bottom]

        exit_frames = []
        base_dirs = []

        for si, side in enumerate(sides_info):
            _, pts_exit = side
            if pts_exit and len(pts_exit) >= 3:
                exit_frames.append(Frame.from_points(pts_exit[0], pts_exit[1], pts_exit[2]))
            else:
                exit_frames.append(None)
            base_dirs.append(screw_dirs_sides[si] if si < len(screw_dirs_sides) else None)

        # bottom
        _, pts_exit_bottom = bottom_info if bottom_info else (None, None)
        if pts_exit_bottom and len(pts_exit_bottom) >= 3:
            exit_frames.append(Frame.from_points(pts_exit_bottom[0], pts_exit_bottom[1], pts_exit_bottom[2]))
        else:
            exit_frames.append(None)
        base_dirs.append(screw_dir_bottom)

        screw_list = []
        for i, row in enumerate(point_grid):
            # map row index to one of the prepared sides/bottom
            map_idx = i if i < len(exit_frames) else (i % len(exit_frames))
            exit_frame = exit_frames[map_idx]
            base_dir = base_dirs[map_idx]

            for j, pt_entry in enumerate(row):
                # If we don't have a direction or exit frame, produce an invalid screw placeholder
                if base_dir is None or exit_frame is None:
                    screw = Screw(pt_entry, base_dir or Vector(0, 0, 1), spec.SCREW_DIAMETER)
                    screw.is_valid = False
                    screw.reject_reason = RejectReason.EXIT_MATERIAL_TOO_THIN
                    screw.joint_guid = joint.guid
                    screw.joint_type = joint.__class__.__name__
                    screw.position = (i, j)
                    screw_list.append(screw)
                    continue

                # project to exit frame along the nominal direction, then recompute the actual direction
                pt_exit = project_point_to_frame_along(pt_entry, base_dir, exit_frame)
                dist = pt_entry.distance_to_point(pt_exit)
                if math.isclose(dist, 0.0, rel_tol=1e-6, abs_tol=1e-6):
                    screw = Screw(pt_entry, base_dir, spec.SCREW_DIAMETER)
                    screw.is_valid = False
                    screw.reject_reason = RejectReason.EXIT_MATERIAL_TOO_THIN
                    screw.joint_guid = joint.guid
                    screw.joint_type = joint.__class__.__name__
                    screw.position = (i, j)
                    screw_list.append(screw)
                    continue

                actual_dir = Vector.from_start_end(pt_entry, pt_exit).unitized()

                screw = self.generate_screw(pt_entry, actual_dir, interface, exit_frame, spec, screw_lengths)
                screw.joint_guid = joint.guid
                screw.joint_type = joint.__class__.__name__
                screw.position = (i, j)

                screw_list.append(screw)

        return screw_list
    
    def create_screw_cylinders(self, screw_list):
        cylinders = []
        for screw in screw_list:
            if screw.is_valid:
                cylinders.append(screw.cylinder)
        return cylinders
    
    def add_drilling_features(self, joint, screws, target="main", depth_limited=True, tol=1e-3):
        """
        Add drilling features created at the joint to the beam(s).

        Parameters
        ----------
        joint : JointWrapper
            The joint to add drilling features on.
        screws : list of Screw
            The screw lines to create drilling features.
        target : str
            "main", "cross" or "both" to specify which beam(s) to add drilling features on.
        depth_limited : bool
            If True, restrict the drilling depth to the predefined screw lengths.
        tol : float
            A small tolerance to move the start point away from the entry face to avoid non-intersection issue.
        """
        if target == "main":
            beams = (joint.main_beam,)
        elif target == "cross":
            beams = (joint.cross_beam,)
        elif target == "both":
            beams = (joint.main_beam, joint.cross_beam)

        for screw in screws:
            # Move the start point slightly away from the entry face to avoid non-intersection issue
            line = screw.line
            line = Line(line[0] - line.direction * tol, line[1])

            for beam in beams:
                drilling = Drilling.from_line_and_element(
                    line, beam, ScrewSpecification.DRILLING_DIAMETER
                )
                drilling._is_joinery = False
                drilling.depth_limited = False
                beam.add_feature(drilling)
        return
    
# -----------------------------------
# Main API
# -----------------------------------
def apply_screws(
        model,
        spec_model="WT-plus-6.5",
        screw_map=None,
        screw_angle=40,
        skrew_angle=20,
        add_features=True,
        drill_target="main",
        depth_limited=True,
        with_data=False,
        debug=False
    ):
    """
    Populate screws on the joints and add drilling features to the beams.
    
    Parameters
    ----------
    model : TimberModel
        The timber model containing the joints and beams.
    spec_model : str
        The specification model to use for screws.
    screw_map : dict, optional
        A mapping of joint GUIDs to screw amounts.
    screw_angle : float, optional
        The angle to determine the screw direction for aligned entry type.
    add_features : bool, optional
        Whether to add drilling features to the beams.
    drill_target : str, optional
        The target beam(s) for drilling features. "main", "cross" or "both".
    depth_limited : bool, optional
        Whether to limit the drilling depth.
    with_data : bool, optional
        Whether to return detailed joint data.
    debug : bool, optional
        Whether to enable debug mode.
    
    Returns
    -------
    dict
        A dictionary containing joint data and capacity warnings if debug is True.
    """
    if add_features:
        model.process_joinery()
    solver = ScrewSolver(model, spec_model=spec_model)
    joint_data = {}

    joints_to_process = []
    # 1. Wrap the joint with the corresponding class in JOINT_MAP if it exists
    for joint in model.joints:
        if joint.name not in ("TMultiStepJoint", "TStepJoint", "KBirdsmouthJoint",):
            continue  # temporary for testing only TMultiStepJoint, TStepJoint first
        
        joint_class = joint.__class__
        if joint_class == KBirdsmouthJoint:
            kbmj_wrapper = ScrewSolver.JOINT_MAP[KBirdsmouthJoint]
            sub_joint_0 = kbmj_wrapper(joint, main_id=0)
            sub_joint_1 = kbmj_wrapper(joint, main_id=1)
            joints_to_process.extend([sub_joint_0, sub_joint_1])

        elif joint_class in ScrewSolver.JOINT_MAP:
            joints_to_process.append(ScrewSolver.JOINT_MAP[joint_class](joint))

    # 2. Populate screws and add drilling features
    for joint in joints_to_process:

        # 3. Get requested screw amount if provided, otherwise use the maximum capacity
        if screw_map is None:
            screw_amount = None
        else:
            screw_amount = int(screw_map.get(str(joint.guid), -1))
            if screw_amount == -1:
                screw_amount = 6
                # raise ValueError(f"Screw amount for joint {joint.guid} not specified in screw_map.")
            if screw_amount == 2:
                screw_amount = 3 # minimun screw amount 
        
        # if isinstance(joint, TSJ) and joint.entry_type == "aligned":
            
        if joint.entry_type == "aligned":

            success, entry_point_grid = solver.populate_aligned_entry_points(joint, angle=screw_angle, amount=screw_amount)
            if not success:
                entry_point_grid = solver.regenerate_aligned_entry_points(joint)
                # continue
            screws = solver.populate_aligned_screws(joint, angle=screw_angle, point_grid=entry_point_grid)                

        elif joint.entry_type == "crossed":
            entry_point_grid = solver.populate_crossed_entry_points(joint, amount=screw_amount)
            screws = solver.populate_crossed_screws(joint, entry_point_grid)


        elif joint.entry_type == "krossed":
            entry_points = solver.populate_krossed_entry_points(joint, angle=skrew_angle, amount=screw_amount)
            # entry_point_grid = [[entry_points["side_a"]], [entry_points["side_b"]], [entry_points["bottom"]]]
            entry_point_grid = [[entry_points["side_a"]], [entry_points["side_b"]]]
            screws = solver.populate_krossed_screws(joint, angle=skrew_angle, point_grid=entry_point_grid)

        else:
            raise ValueError("Unknown entry type.")

        # Allow to protrude through the exit beam into clt plate
        for screw in screws:
            if screw.is_valid:
                continue
            if screw.reject_reason == RejectReason.EXIT_PROTRUSION:
                if joint.__class__ == TSJ and joint.cross_beam.attributes["level"] == 0:
                    screw.protrude()
            elif screw.reject_reason == RejectReason.ENTRY_MATERIAL_TOO_THICK:
                    counterbored_lines = screw.counterbore(penetration=solver.get_specification(joint).penetration)        

        if add_features:
            solver.add_drilling_features(
                joint,
                screws=screws,
                target=drill_target,
                depth_limited=depth_limited
            )
            #if counterbored_lines:
                # Add counterbored drilling features here
                #pass
            
        if with_data:
            if joint.name == "KBirdsmouthJoint":
                interface = joint.get_interface_boundary_horizontal(data_type="polyline")
                entry_face, exit_face = joint.find_screw_boundaries(angle=skrew_angle, data_type="polylines")["sides"]
            else:
                interface = joint.get_interface_boundary(data_type="polyline")
                entry_face, exit_face = joint.find_screw_boundaries(angle=screw_angle, data_type="polylines")

            if joint.entry_type == "aligned":
                corners = solver.shrink_aligned_entry_corners(joint, angle=screw_angle)
                entry_face = Polyline(corners + [corners[0]])

            joint_data[joint] = {
                "interface": interface,
                "entry_face": entry_face,
                "exit_face": exit_face,
                "screws": screws,
            }

    final_output = {
        "joint_data": joint_data,
        "capacity_warnings": solver.capacity_warnings if debug else [],
    }
    return final_output

