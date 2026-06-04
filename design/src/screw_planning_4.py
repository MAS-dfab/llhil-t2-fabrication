"""Timber screw planning."""

from joint_wrappers_3 import TMSJ, TSJ, TBJ, TBMJ, KBMJ, project_point_to_frame_along # NOTE: temp.
from screw_spec_2 import ScrewSpecification
from compas.geometry import (
    Frame, Line, Cylinder, Polyline,
    cross_vectors, angle_vectors,
    KDTree
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
            "crossed": ScrewSpecification("crossed", spec_model=spec_model)
        }

        self.capacity_warnings = []
        self.rejected_logs = []

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
        list of list of Point
            Inner list is along the width direction.
        """
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        if amount == 0:
            return [[]]
        
        # 1. Calculate the maximum screw number in width and length
        max_w_num, max_l_num = self.calculate_screw_capacity(joint, angle=angle)
        if max_w_num > 3:
            raise NotImplementedError("Only support up to 3 screws in one row for now.")

        # 2. If requested amount is not provided, use the maximum capacity.
        #    For large requests (>10), prefer 3 screws in width

        forced_extend = True
        if amount is None:
            pass
        else:
            distributions = self.calculate_screw_distributions(amount, max_w_num)
            # Handle large requests by preferring 3-wide when possible
            if amount > 10:
                preferred_w = min(3, max_w_num)
                # If the preferred width fits within capacity, use it
                best = next(((w_num, l_num) for w_num, l_num in distributions if w_num == preferred_w and l_num <= max_l_num), None)
                if best is not None:
                    max_w_num, max_l_num = best
                
                if amount >= max_w_num * max_l_num:
                    # Amount exceeds capacity: force preferred width and add length rows
                    max_w_num = preferred_w
                    max_l_num = int(math.ceil(amount / float(max_w_num)))
                    forced_extend = True
                    # Log capacity warning but proceed to populate (points may later be rejected)
                    self.capacity_warnings.append({"joint_guid": joint.guid, "joint": joint, "requested": amount, "capacity": (max_w_num, max_l_num)})
        
            else:
                # Length direction is the priority to fill for small/normal requests
                if amount <= max_w_num * max_l_num:
                    distributions.sort(key=lambda x: x[1], reverse=True)
                    best = next(((w_num, l_num) for w_num, l_num in distributions if l_num <= max_l_num), None)
                    if best is not None:
                        max_w_num, max_l_num = best
                else:
                    self.capacity_warnings.append({"joint_guid": joint.guid, "joint": joint, "requested": amount, "capacity": (max_w_num, max_l_num)})
        
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

        # Compute a sensible length step. 
        dire = joint.calculate_screw_direction(angle=angle)
        main_dire = joint.main_beam.centerline.direction
        acute_angle = angle_vectors(dire, main_dire)
        acute_angle = self.convert_to_acute_angle(acute_angle)
        if forced_extend and acute_angle > 1e-6:
            l_step = spec.a1 / math.sin(acute_angle)
        else:
            l_step = (l / (max_l_num - 1)) if max_l_num > 1 else 0.0

        vec_w = (pts_entry[3] - pts_entry[0]).unitized()
        vec_l = (pts_entry[1] - pts_entry[0]).unitized()

        # Boundary limits
        width_limit = pts_entry[0].distance_to_point(pts_entry[3])
        length_limit = pts_entry[0].distance_to_point(pts_entry[1])

        # Start at the first offset point
        start = pts_entry[0] + (vec_w * w_steps[0])

        point_grid = [[None for _ in range(max_w_num)] for _ in range(max_l_num)]
        for i in range(max_l_num):
            curr_w = 0.0
            for j in range(max_w_num):
                pt = start + (vec_w * curr_w) + (vec_l * i * l_step)
                point_grid[i][j] = pt
                curr_w += w_steps[j + 1]
        
        # Remove the last row if it is outside the boundary
        if max_l_num > 1:
            last_row = point_grid[-1]
            last_row_valid = False
            for pt in last_row:
                if pt is not None:
                    vec_from_origin = pt - pts_entry[0]
                    proj_w_dist = vec_from_origin.dot(vec_w)
                    proj_l_dist = vec_from_origin.dot(vec_l)
                    if 0 <= proj_w_dist <= width_limit and 0 <= proj_l_dist <= length_limit:
                        last_row_valid = True
                        break
            if not last_row_valid:
                point_grid.pop()
        
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

    def populate_aligned_screw_lines(self, joint, angle, point_grid, restrict=True):
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
        restrict : bool
            If True, restrict the screw length to the predefined lengths.
        
        Returns
        -------
        list of list of Lines
            Inner list is along the width direction.
        """
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]
        screw_lengths = sorted(spec.SCREW_LENGTHS, reverse=True)

        corners = joint.get_interface_boundary(data_type="points")
        interface = Frame.from_points(corners[0], corners[1], corners[3])
        
        # 2. Create exit frame and screw direction
        _, pts_exit = joint.find_screw_boundaries(angle=angle, data_type="points")
        exit_frame = Frame.from_points(pts_exit[0], pts_exit[1], pts_exit[3])
        dire = joint.calculate_screw_direction(angle=angle)

        # 3. get the length of each screw line by projecting the point to the exit face
        line_grid = []
        for row in point_grid:
            line_row = []
            for pt_entry in row:
                # Skip None points (out of boundary)
                if pt_entry is None:
                    continue
                    
                pt_exit = project_point_to_frame_along(pt_entry, dire, exit_frame)

                if restrict:
                    # 1. Check if sufficient penetration in exit beam is achieved
                    dist = pt_entry.distance_to_point(pt_exit)
                    dist_entry = project_point_to_frame_along(pt_entry, dire, interface).distance_to_point(pt_entry)
                    dist_exit = dist - dist_entry
                    if dist_exit < spec.penetration:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "reject_type": "A",  # temp.
                            "entry_point": pt_entry,
                            "exit_point": pt_exit,
                            "penetrations": (dist_entry, dist_exit),
                            "required_penetration": spec.penetration,
                            "reason": "Insufficient penetration in exit beam."
                        })
                        continue
                    
                    # 2. Check if the screw length is within the predefined screw lengths
                    max_allowed_length = dist - spec.BACK_THRESHOLD
                    suitable_length = next((l for l in screw_lengths if l <= max_allowed_length), None)
                    if suitable_length is None:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "reject_type": "B",  # temp.
                            "entry_point": pt_entry,
                            "exit_point": pt_exit,
                            "distance_entry_to_exit": dist,
                            "max_allowed_length": max_allowed_length,
                            "reason": "Screw lengths are too long."
                        })
                        continue
                    pt_exit = pt_entry + dire * suitable_length

                    # 3. Check if sufficient penetration from interface to the end of screw is achieved
                    screw_in_exit = suitable_length - dist_entry
                    if screw_in_exit < spec.penetration:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "reject_type": "C",  # temp.
                            "entry_point": pt_entry,
                            "exit_point": pt_exit,
                            "penetrations": (dist_entry, screw_in_exit),
                            "required_penetration": spec.penetration,
                            "reason": "Screw lengths are too short in exit beam, but penetration is sufficient."
                        })
                        # amp = line_grid[0][0].distance_to_point(line_grid[-1][0]) * (spec.penetration - screw_in_exit) / (suitable_length - screw_in_exit)

                        continue

                line_row.append(Line(pt_entry, pt_exit))
            line_grid.append(line_row)
        return line_grid

    def populate_crossed_screw_lines(self, joint, point_grid, restrict=True):
        """
        Populate screw lines from the height sides diagonally.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        point_grid : list of list of Point
            The entry points to create screw lines.
        restrict : bool
            If True, restrict the screw length to the predefined lengths.
        
        Returns
        -------
        list of list of Lines
            Inner list is along the width direction.
        """
        if joint.entry_type != "crossed":
            raise ValueError("Wrong entry type. Expected 'crossed'.")
        spec = self._spec_cache["crossed"]
        screw_lengths = sorted(spec.SCREW_LENGTHS, reverse=True)

        exit_frames = [joint.cross_beam.ref_sides[(joint.cross_beam_ref_side_index + i) % 4] for i in range(1, 4)]

        # Find rotate axis
        dire = -joint.calculate_screw_direction()
        normal = joint.main_beam.front_side(joint.main_beam_ref_side_index).normal
        rotate_axis = cross_vectors(dire, normal)
    
        line_grid = []
        for row in point_grid:
            line_row = []
            for i, pt in enumerate(row):
                # Skip None points (out of boundary)
                if pt is None:
                    continue
                    
                sign = -1 if i % 2 == 0 else +1
                side_vec = dire.rotated(sign * math.radians(spec.side_angle), rotate_axis)

                exit_frame = None
                min_dist = float("inf")
                for ef in exit_frames:
                    sample_exit = project_point_to_frame_along(pt, side_vec, ef)
                    dist = pt.distance_to_point(sample_exit)
                    if math.isclose(dist, 0, rel_tol=1e-3, abs_tol=1e-3):
                        continue
                    if dist < min_dist:
                        min_dist = dist
                        exit_frame = ef
                
                pt_exit = project_point_to_frame_along(pt, side_vec, exit_frame)
                if restrict:
                    # 1. Check if sufficient penetration in exit beam is achieved
                    corners = joint.get_interface_boundary(data_type="points")
                    interface = Frame.from_points(corners[0], corners[1], corners[3])

                    dist = pt.distance_to_point(pt_exit)
                    dist_entry = project_point_to_frame_along(pt, dire, interface).distance_to_point(pt)
                    dist_exit = dist - dist_entry
                    if dist_exit < spec.penetration:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "reject_type": "A",  # temp.
                            "entry_point": pt,
                            "exit_point": pt_exit,
                            "penetrations": (dist_entry, dist_exit),
                            "required_penetration": spec.penetration,
                            "reason": "Insufficient penetration in exit beam."
                        })
                        continue

                    # 2. Check if the screw length is within the predefined screw lengths
                    max_allowed_length = dist - spec.BACK_THRESHOLD
                    suitable_length = next((l for l in screw_lengths if l <= max_allowed_length), None)
                    if suitable_length is None:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "reject_type": "B",  # temp.
                            "entry_point": pt,
                            "distance": dist,
                            "max_allowed_length": max_allowed_length,
                            "reason": "Screw lengths are too long."
                        })
                        continue
                    pt_exit = pt + side_vec * suitable_length

                    # 3. Check if sufficient penetration from interface to the end of screw is achieved
                    penetration_in_exit = suitable_length - dist_entry
                    if penetration_in_exit < spec.penetration:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "reject_type": "C",  # temp.
                            "entry_point": pt,
                            "exit_point": pt_exit,
                            "penetrations": (dist_entry, penetration_in_exit),
                            "required_penetration": spec.penetration,
                            "reason": "Screw lengths are too short in exit beam, but penetration is sufficient."
                        })
                        continue

                line_row.append(Line(pt, pt_exit))
            line_grid.append(line_row)
        return line_grid

    def has_rejections(self, joint):
        """Check if there are any rejections for the given joint."""
        if not self.rejected_logs:
            return False
        for rej in self.rejected_logs:
            if rej["joint_guid"] == joint.guid:
                return True
            return False
        
    def evaluate_screw_lengths(self, joint):
        if not self.rejected_logs or not self.has_rejections(joint):
            return # Something  # No rejections, all good
        rejects = []
        for rej in self.rejected_logs:
            if rej["joint_guid"] == joint.guid:
                rejects.append(rej)
        
        spec = self.get_specification(joint)
        for rej in rejects:
            _, pene_exit = rej["penetrations"]
            needed_length = (spec.BACK_THRESHOLD + spec.penetration) - pene_exit
        

    def create_screw_cylinders(self, line_grid):
        radius = ScrewSpecification.SCREW_DIAMETER / 2.0
        return [Cylinder.from_line_and_radius(line, radius) for row in line_grid for line in row]

    def add_drilling_features(self, joint, line_grid, target="main", depth_limited=True, tol=1e-3):
        """
        Add drilling features created at the joint to the beam(s).

        Parameters
        ----------
        joint : JointWrapper
            The joint to add drilling features on.
        line_grid : list of list of Line
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

        line_list = [line for line_row in line_grid for line in line_row]
        for line in line_list:
            # Move the start point slightly away from the entry face to avoid non-intersection issue
            line = Line(line[0] - line.direction * tol, line[1])

            for beam in beams:
                drilling = Drilling(depth_limited=depth_limited).from_line_and_element(
                    line, beam, ScrewSpecification.DRILLING_DIAMETER
                )
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
        model.process_joinerys()
    solver = ScrewSolver(model, spec_model=spec_model)
    joint_data = {}

    joints_to_process = []
    # 1. Wrap the joint with the corresponding class in JOINT_MAP if it exists
    for joint in model.joints:
        if joint.name not in ("TMultiStepJoint", "TStepJoint",):
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
        if not joint.is_planar:
            print(f"Warning: {joint.name} is not planar. Skipping screw placement.")
            continue  # temp.
        
        # 3. Get requested screw amount if provided, otherwise use the maximum capacity
        if screw_map is None:
            screw_amount = None
        else:
            screw_amount = int(screw_map.get(str(joint.guid), -1))
            if screw_amount == -1:
                raise ValueError(f"Screw amount for joint {joint.guid} not specified in screw_map.")
        
        # if isinstance(joint, TSJ) and joint.entry_type == "aligned":
            
        if joint.entry_type == "aligned":
            restrict = False if joint.__class__ == TSJ else True
            # restrict = True  # temp. restrict for testing
            entry_point_grid = solver.populate_aligned_entry_points(joint, angle=screw_angle, amount=screw_amount)
            screw_line_grid = solver.populate_aligned_screw_lines(joint, angle=screw_angle, point_grid=entry_point_grid, restrict=restrict)
            
        elif joint.entry_type == "crossed":
            entry_point_grid = solver.populate_crossed_entry_points(joint, amount=screw_amount)
            screw_line_grid = solver.populate_crossed_screw_lines(joint, entry_point_grid, restrict=True)

        else:
            raise ValueError("Unknown entry type.")
        
        if add_features:
            solver.add_drilling_features(
                joint,
                line_grid=screw_line_grid,
                target=drill_target,
                depth_limited=depth_limited
            )
        if with_data:
            interface = joint.get_interface_boundary(data_type="polyline")
            entry_face, exit_face = joint.find_screw_boundaries(angle=screw_angle, data_type="polylines")
            if joint.entry_type == "aligned":
                corners = solver.shrink_aligned_entry_corners(joint, angle=screw_angle)
                entry_face = Polyline(corners + [corners[0]])

            joint_data[joint.guid] = {
                "interface": interface,
                "entry_face": entry_face,
                "exit_face": exit_face,
                "entry_points": [pt for row in entry_point_grid for pt in row],
                "screw_lines": [ln for row in screw_line_grid for ln in row],
                "screw_lengths": [round(ln.length, 3) for row in screw_line_grid for ln in row],
            }
    final_output = {
        "joint_data": joint_data,
        "capacity_warnings": solver.capacity_warnings if debug else [],
        "rejected_logs": solver.rejected_logs if debug else [],
    }
    return final_output