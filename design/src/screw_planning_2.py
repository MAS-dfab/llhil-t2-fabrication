"""Timber screw planning."""

from joint_wrappers_2 import TMSJ, TSJ, TBJ, TBMJ, KBMJ, project_point_to_frame_along # NOTE: temp.
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

    def shrink_aligned_entry_corners(self, joint, orientation):
        """Shrink the entry face corners for aligned entry type with the constraint of screw penetration."""
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        dire = joint.calculate_screw_direction(orientation=orientation)
        angle_dire_cross = angle_vectors(dire, joint.cross_beam.centerline.direction)
        if angle_dire_cross > math.pi / 2:
            angle_dire_cross = math.pi - angle_dire_cross
        dist_perp_to_cross = math.sin(angle_dire_cross) * spec.penetration
        amp = dist_perp_to_cross / math.sin(math.radians(joint.acute_angle))

        entry_corners, _ = joint.find_screw_boundaries(orientation=orientation, data_type="points")
        dire = (entry_corners[1] - entry_corners[0]).unitized()
        offset_vec = dire * amp

        for i, corner in enumerate(entry_corners):
            if i in (0, 3):
                entry_corners[i] = corner + offset_vec
        return entry_corners

    def calculate_screw_capacity(self, joint, orientation):
        """Calculate the maximum amount of screws in width and length direction."""        
        if joint.entry_type == "aligned":
            spec = self._spec_cache["aligned"]
            entry_corners = self.shrink_aligned_entry_corners(joint, orientation)
            # entry_corners, _ = joint.find_screw_boundaries(orientation=orientation, data_type="points")
            len_l = entry_corners[0].distance_to_point(entry_corners[1])

            max_w_num = 2  # In our project it's always two screws (row) along width (25, 50, 25)

            dire = joint.calculate_screw_direction(orientation=orientation)
            main_dire = joint.main_beam.centerline.direction
            angle = angle_vectors(dire, main_dire)
            if angle > math.pi / 2:
                angle = math.pi - angle

            l_spacing = spec.a1 / math.sin(angle)
            max_l_num = len_l // l_spacing

        elif joint.entry_type == "crossed":
            spec = self._spec_cache["crossed"]
            entry_corners, _ = joint.find_screw_boundaries(orientation=orientation, data_type="points")
            len_l = entry_corners[0].distance_to_point(entry_corners[1])

            max_w_num = 2  # Left and right each along width
            table = sorted(spec.spec_table["min_widths"].items(), key=lambda x: x[0], reverse=True)
            max_l_num = next((pair_count for pair_count, min_width in table if len_l >= min_width), 0)

        else:
            raise ValueError("Unknown entry type.")
        return int(max_w_num), int(max_l_num)
        # TODO: implement the a1_cg (65mm) check with "aligned" entry type, 60/60 mm penetration for both entry types.

    def populate_aligned_entry_points(self, joint, amount: int = None, orientation="bisector"):
        """
        Populate screw points on the entry face.

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
        if joint.entry_type != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        pts_entry = self.shrink_aligned_entry_corners(joint, orientation)
        # pts_entry, _ = joint.find_screw_boundaries(orientation=orientation, data_type="points")

        w_num, l_num = self.calculate_screw_capacity(joint, orientation)
        if amount is None:
            pass
        elif amount <= w_num * l_num:
            l_num = amount // w_num + (1 if amount % w_num > 0 else 0)
        else:
            self.capacity_warnings.append(
                {"joint_guid": joint.guid, "joint": joint, "requested": amount, "capacity": (w_num, l_num)}
            )
            pass
        
        if w_num != 2:
            raise NotImplementedError("Only support 2 pairs for now.")
        w_steps = (spec.a2_cg, spec.a2, spec.a2_cg)

        l = pts_entry[0].distance_to_point(pts_entry[1])
        l_step = l / (l_num)

        vec_w = (pts_entry[3] - pts_entry[0]).unitized()
        vec_l = (pts_entry[1] - pts_entry[0]).unitized()

        # 2. Start at the first offset point
        start = pts_entry[0] + (vec_w * w_steps[0])# + (vec_l * l_step)

        # 3. Create point grid
        point_grid = [[None for _ in range(w_num)] for _ in range(l_num)]
        for i in range(l_num):
            curr_w = 0.0
            for j in range(w_num):
                point_grid[i][j] = start + (vec_w * curr_w) + (vec_l * i * l_step)
                curr_w += w_steps[j + 1]
        return point_grid
    
    def populate_crossed_entry_points(self, joint, amount: int = None, orientation="along_main"):
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
        offset_dire = joint.calculate_screw_direction(orientation=orientation)
        amp = math.cos(math.radians(spec.side_angle)) * spec.penetration
        offset_vec = offset_dire * amp

        pts_entry = [p + offset_vec for p in corners]

        # Calculate screw amount and check with capacity
        w_num, l_num = self.calculate_screw_capacity(joint, orientation)
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

    def populate_aligned_screw_lines(self, joint, point_grid, orientation="bisector", restrict=True):
        """
        Populate screw lines from the entry face to the exit face.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        point_grid : list of list of Point
            The entry points to create screw lines.
        orientation : str
            "perp_main", "perp_riser", "perp_cross", "bisector" and "along_main".
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

        # 2. Create exit frame and screw direction
        _, pts_exit = joint.find_screw_boundaries(orientation=orientation, data_type="points")
        exit_frame = Frame.from_points(pts_exit[0], pts_exit[1], pts_exit[3])
        dire = joint.calculate_screw_direction(orientation=orientation)

        # 3. get the length of each screw line by projecting the point to the exit face
        line_grid = []
        for row in point_grid:
            line_row = []
            for pt_entry in row:
                pt_exit = project_point_to_frame_along(pt_entry, dire, exit_frame)

                if restrict:
                    dist = pt_entry.distance_to_point(pt_exit)
                    max_allowed_length = dist - spec.BACK_THRESHOLD
                    
                    suitable_length = next((l for l in screw_lengths if l <= max_allowed_length), None)
                    if suitable_length is None:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "entry_point": pt_entry,
                            "distance_entry_to_exit": dist,
                            "max_allowed_length": max_allowed_length,
                            "reason": "No suitable screw length."
                        })
                        continue
                    pt_exit = pt_entry + dire * suitable_length

                    # Check if sufficient penetration from interface to exit face is achieved
                    corners = joint.get_interface_boundary(data_type="points")
                    interface = Frame.from_points(corners[0], corners[1], corners[3])
                    dist_in_entry = project_point_to_frame_along(pt_entry, dire, interface).distance_to_point(pt_entry)
                    penetration_in_exit = suitable_length - dist_in_entry
                    if penetration_in_exit < spec.penetration:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "entry_point": pt_entry,
                            "exit_point": pt_exit,
                            "penetrations": (dist_in_entry, penetration_in_exit),
                            "required_penetration": spec.penetration,
                            "reason": "Insufficient penetration to exit face."
                        })
                        continue

                line_row.append(Line(pt_entry, pt_exit))
            line_grid.append(line_row)
        # TODO: check teh penetration from interface to exit face
        return line_grid

    def populate_crossed_screw_lines(self, joint, point_grid, orientation="along_main", restrict=True):
        """
        Populate screw lines from the height sides diagonally.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        point_grid : list of list of Point
            The entry points to create screw lines.
        orientation : str
            "perp_main", "perp_riser", "perp_cross", "bisector" and "along_main".
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
        dire = -joint.calculate_screw_direction(orientation=orientation)
        normal = joint.main_beam.front_side(joint.main_beam_ref_side_index).normal
        rotate_axis = cross_vectors(dire, normal)
    
        line_grid = []
        for row in point_grid:
            line_row = []
            for i, pt in enumerate(row):
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
                dist = pt.distance_to_point(pt_exit)
                max_allowed_length = dist - spec.BACK_THRESHOLD
                if restrict:
                    suitable_length = next((l for l in screw_lengths if l <= max_allowed_length), None)
                    if suitable_length is None:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "entry_point": pt,
                            "distance": dist,
                            "max_allowed_length": max_allowed_length,
                            "reason": "No suitable screw length."
                        })
                        continue
                    pt_exit = pt + side_vec * suitable_length

                    # Check if sufficient penetration from interface to exit face is achieved
                    corners = joint.get_interface_boundary(data_type="points")
                    interface = Frame.from_points(corners[0], corners[1], corners[3])
                    dist_in_entry = project_point_to_frame_along(pt, side_vec, interface).distance_to_point(pt)
                    penetration_in_exit = suitable_length - dist_in_entry
                    if penetration_in_exit < spec.penetration:
                        self.rejected_logs.append({
                            "joint_guid": joint.guid,
                            "joint": joint,
                            "entry_point": pt,
                            "exit_point": pt_exit,
                            "penetrations": (dist_in_entry, penetration_in_exit),
                            "required_penetration": spec.penetration,
                            "reason": "Insufficient penetration to exit face."
                        })
                        continue

                line_row.append(Line(pt, pt_exit))
            line_grid.append(line_row)
        # TODO: check teh penetration from interface to exit face
        return line_grid

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
        orientation_tstep="perp_cross",
        orientation_tmulti="bisector",
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
    orientation_tstep : str, optional
        The orientation for TStepJoint aligned entry type.
    orientation_tmulti : str, optional
        The orientation for TMultiStepJoint aligned entry type.
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
        
        if isinstance(joint, TSJ) and joint.entry_type == "aligned":
            orientation = orientation_tstep
        elif isinstance(joint, TMSJ) and joint.entry_type == "aligned":
            orientation = orientation_tmulti

        if joint.entry_type == "aligned":
            entry_point_grid = solver.populate_aligned_entry_points(joint, amount=screw_amount, orientation=orientation)
            screw_line_grid = solver.populate_aligned_screw_lines(joint, entry_point_grid, orientation, restrict=True)
            
        elif joint.entry_type == "crossed":
            orientation = "along_main"  # Fix the orientation for crossed entry type
            entry_point_grid = solver.populate_crossed_entry_points(joint, amount=screw_amount, orientation=orientation)
            screw_line_grid = solver.populate_crossed_screw_lines(joint, entry_point_grid, orientation, restrict=True)

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
            entry_face, exit_face = joint.find_screw_boundaries(orientation=orientation, data_type="polylines")
            if joint.entry_type == "aligned":
                corners = solver.shrink_aligned_entry_corners(joint, orientation=orientation)
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