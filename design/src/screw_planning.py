"""Timber screw planning."""

from joint_wrappers import TMSJ, project_point_to_frame_along
from screw_spec import ScrewSpecification
from compas.geometry import (
    Frame, Line, Cylinder,
    cross_vectors, angle_vectors,
    KDTree
)
from compas_timber.connections import (
    TMultiStepJoint, TStepJoint, TBirdsmouthJoint, KBirdsmouthJoint, LLapJoint, XLapJoint
)
from compas_timber.fabrication import Drilling
import math

# -----------------------------------
# Screw Solver
# -----------------------------------
class ScrewSolver:
    
    def __init__(self, model, spec_model="WT-plus-6.5"):
        self.model = model
        self.joints = model.joints
        self.spec_model = spec_model

        self._spec_cache = {
            "aligned": ScrewSpecification("aligned", spec_model=spec_model),
            "crossed": ScrewSpecification("crossed", spec_model=spec_model)
        }

    def cluster_joints(self, distance=0.20):
        kdtree = KDTree(self.joints)
        pass

    def is_collided(self, joint):
        pass
    
    def determine_entry_type(self, joint):
        """Determine the entry type of the screw based on the angle."""
        if joint.acute_angle > ScrewSpecification().ANGLE_THRESHOLD:
            return "crossed"
        return "aligned"
    
    def get_specification(self, joint):
        entry_type = self.determine_entry_type(joint)
        return self._spec_cache[entry_type]
    
    def determine_pair_count(self, joint, orientation):
        pts_entry, _ = joint.find_screw_boundaries(orientation=orientation, data_type="points")
        entry_type = self.determine_entry_type(joint)

        if entry_type == "aligned":
            dist = pts_entry[0].distance_to_point(pts_entry[3])  # width of the entry face
        elif entry_type == "crossed":
            dist = pts_entry[0].distance_to_point(pts_entry[1])  # length of the entry face

        spec = self.get_specification(joint)
        table = sorted(spec.spec_table["min_widths"].items(), key=lambda x: x[0], reverse=True)
        return next((pair_count for pair_count, min_width in table if dist >= min_width), 0)
        
    def populate_aligned_entry_points(self, joint, orientation="perp_tread", nested=False):
        """
        Populate screw points on the entry face.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        orientation : str
            "perp_main", "perp_tread", "perp_cross", "cross_section", and "bisector".
        nested : bool
            If True, create a nested grid with the width list in the inner loop.
        
        Returns
        -------
        list of list of Point if nested, otherwise a flat list of Point.
        """
        if self.determine_entry_type(joint) != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        pts_entry, _ = joint.find_screw_boundaries(orientation=orientation, data_type="points")

        w_num = self.determine_pair_count(joint, orientation) + 1
        if w_num != 3:
            raise NotImplementedError("Only support 2 pairs for now.")
        w_steps = (spec.a2_cg, spec.a2, spec.a2_cg)

        dire = joint.calculate_screw_direction(orientation=orientation)
        main_dire = -joint.point_centerline_towards_joint(joint.main_beam)
        angle = angle_vectors(dire, main_dire)
        l_spacing = spec.a1 / math.sin(angle)
        l = pts_entry[0].distance_to_point(pts_entry[1])
        l_num = int(l // l_spacing)
        l_step = l / l_num

        vec_w = (pts_entry[3] - pts_entry[0]).unitized()
        vec_l = (pts_entry[1] - pts_entry[0]).unitized()

        # 2. Shrink the entry face
        start = pts_entry[0] + (vec_w * w_steps[0]) + (vec_l * l_step)

        # 3. Create point grid
        point_grid = [[None for _ in range(w_num - 1)] for _ in range(l_num - 1)]
        for i in range(l_num - 1):
            curr_w = 0.0
            for j in range(w_num - 1):
                point_grid[i][j] = start + (vec_w * curr_w) + (vec_l * i * l_step)
                curr_w += w_steps[j + 1]

        if not nested:
            point_grid = [pt for row in point_grid for pt in row]
        return point_grid
    
    def populate_aligned_screw_lines(self, joint, orientation="perp_tread", nested=False, restrict=True):
        """
        Populate screw lines from the entry face to the exit face.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        orientation : str
            "perp_main", "perp_tread", "perp_cross", "bisector" and "cross_section".
        nested : bool
            If True, create a nested grid with the width list in the inner loop.
        restrict : bool
            If True, restrict the screw length to the predefined lengths.
        
        Returns
        -------
        list of list of Lines if nested, otherwise a flat list of Lines.
        """
        if self.determine_entry_type(joint) != "aligned":
            raise ValueError("Wrong entry type. Expected 'aligned'.")
        spec = self._spec_cache["aligned"]

        # 1. Get the entry point grid
        entry_grid = self.populate_aligned_entry_points(joint, orientation=orientation, nested=True)

        # 2. Create exit frame and screw direction
        _, pts_exit = joint.find_screw_boundaries(orientation=orientation, data_type="points")
        exit_frame = Frame.from_points(pts_exit[0], pts_exit[1], pts_exit[3])
        dire = joint.calculate_screw_direction(orientation=orientation)

        # 3. get the length of each screw line by projecting the point to the exit face
        screw_lengths = sorted(spec.SCREW_LENGTHS, reverse=True)
        line_list = []
        for row in entry_grid:
            line_row = []

            if restrict:
                sample = row[0]
                sample_exit = project_point_to_frame_along(sample, dire, exit_frame)
                sample_dist = sample.distance_to_point(sample_exit)
                max_allowed_length = sample_dist - spec.BACK_THRESHOLD

                suitable_length = next((l for l in screw_lengths if l <= max_allowed_length), None)
                if suitable_length is None:
                    continue
                
                for pt_entry in row:
                    pt_exit = pt_entry + dire * suitable_length
                    line_row.append(Line(pt_entry, pt_exit))
            else:
                for pt_entry in row:
                    pt_exit = project_point_to_frame_along(pt_entry, dire, exit_frame)
                    line_row.append(Line(pt_entry, pt_exit))
            line_list.append(line_row)

        if not nested:
            line_list = [line for row in line_list for line in row]
        return line_list

    def populate_crossed_entry_points(self, joint, orientation="cross_section", nested=False):
        """
        Populate screw points on the entry face from the height sides.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        orientation : str
            "perp_main", "perp_tread", "perp_cross", "cross_section", and "bisector".
        nested : bool
            If True, create a nested grid with the width list in the inner loop.
        
        Returns
        -------
        list of list of Point if nested, otherwise a flat list of Point.
        """
        if self.determine_entry_type(joint) != "crossed":
            raise ValueError("Wrong entry type. Expected 'crossed'.")
        spec = self._spec_cache["crossed"]

        # Find offset entry points
        strut_corners = joint.get_strut_boundary(data_type="points")
        offset_dire = joint.calculate_screw_direction(orientation=orientation)
        amp = spec.side_offset / math.sin(math.radians(joint.acute_angle))
        offset_vec = offset_dire * amp

        pts_entry = [p + offset_vec for p in strut_corners]

        l_num = self.determine_pair_count(joint, orientation)
        if l_num > 3:
            raise NotImplementedError("Only support up to 3 pairs for now.")
        
        l = pts_entry[0].distance_to_point(pts_entry[1])
        a2_cg = (l - spec.a2_red - spec.a1 * (l_num - 1)) / 2  # screw center to edge

        vec_l = (pts_entry[1] - pts_entry[0]).unitized()
        point_grid = []
        for i in range(l_num):
            pt_left = pts_entry[0] + (vec_l * a2_cg) + (vec_l * spec.a1) * i
            pt_right = pts_entry[3] + (vec_l * a2_cg) + (vec_l * spec.a1) * i + (vec_l * spec.a2_red)
            point_grid.append([pt_left, pt_right])

        if not nested:
            return [pt for row in point_grid for pt in row]
        return point_grid

    def populate_crossed_screw_lines(self, joint, orientation="cross_section", nested=False, restrict=True):
        """
        Populate screw lines from the height sides diagonally.

        Parameters
        ----------
        joint : JointWrapper
            The joint to populate screws on.
        orientation : str
            "perp_main", "perp_tread", "perp_cross", "bisector" and "cross_section".
        nested : bool
            If True, create a nested grid with the width list in the inner loop.
        restrict : bool
            If True, restrict the screw length to the predefined lengths.
        
        Returns
        -------
        list of list of Lines if nested, otherwise a flat list of Lines.
        """
        if self.determine_entry_type(joint) != "crossed":
            raise ValueError("Wrong entry type. Expected 'crossed'.")
        spec = self._spec_cache["crossed"]
        screw_lengths = sorted(spec.SCREW_LENGTHS, reverse=True)

        entry_grid = self.populate_crossed_entry_points(joint, orientation=orientation, nested=True)
        exit_frames = [joint.cross_beam.ref_sides[(joint.cross_beam_ref_side_index + i) % 4] for i in range(1, 4)]

        # Find rotate axis
        dire = -joint.calculate_screw_direction(orientation=orientation)
        normal = joint.main_beam.front_side(joint.main_beam_ref_side_index).normal
        rotate_axis = cross_vectors(dire, normal)
    
        line_list = []
        for row in entry_grid:
            line_row = []
            for i, pt in enumerate(row):
                sign = -1 if i == 0 else +1
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
                        continue
                    pt_exit = pt + side_vec * suitable_length
                line_row.append(Line(pt, pt_exit))
            line_list.append(line_row)

        if not nested:
            line_list = [line for row in line_list for line in row]
        return line_list

    def create_screw_cylinders(self, joint, orientation, nested=False, restrict=True):
        if self.determine_entry_type(joint) == "aligned":
            line_list = self.populate_aligned_screw_lines(joint, orientation=orientation, nested=True, restrict=restrict)
            spec = self._spec_cache["aligned"]
        elif self.determine_entry_type(joint) == "crossed":
            line_list = self.populate_crossed_screw_lines(joint, orientation=orientation, nested=True, restrict=restrict)
            spec = self._spec_cache["crossed"]

        if nested:
            return [[Cylinder.from_line_and_radius(line, spec.SCREW_DIAMETER / 2) for line in row] for row in line_list]
        return [Cylinder.from_line_and_radius(line, spec.SCREW_DIAMETER / 2) for row in line_list for line in row]
    
    def add_drilling_features(self, joint, orientation, target="main", depth_limited=True, tol=1e-3, with_data=False):
        """
        Add drilling features created at the joint to the beam(s).

        Parameters
        ----------
        joint : JointWrapper
            The joint to add drilling features on.
        orientation : str
            "perp_main", "perp_tread", "perp_cross", "bisector" and "cross_section".
        target : str
            "main", "cross" or "both" to specify which beam(s) to add drilling features on.
        depth_limited : bool
            If True, restrict the drilling depth to the predefined screw lengths.
        tol : float
            A small tolerance to move the start point away from the entry face to avoid non-intersection issue.
        with_data : bool
            If True, return a dictionary of "entry_points" and "line_list" for debugging and visualization.
        """
        if self.determine_entry_type(joint) == "aligned":
            line_list = self.populate_aligned_screw_lines(joint, orientation=orientation, nested=False, restrict=True)
        elif self.determine_entry_type(joint) == "crossed":
            line_list = self.populate_crossed_screw_lines(joint, orientation=orientation, nested=False, restrict=True)

        if target == "main":
            beams = (joint.main_beam,)
        elif target == "cross":
            beams = (joint.cross_beam,)
        elif target == "both":
            beams = (joint.main_beam, joint.cross_beam)

        for line in line_list:
            # Move the start point slightly away from the entry face to avoid non-intersection issue
            line = Line(line[0] - line.direction * tol, line[1])

            for beam in beams:
                drilling = Drilling(depth_limited=depth_limited).from_line_and_element(
                    line, beam, ScrewSpecification().DRILLING_DIAMETER
                )
                beam.add_feature(drilling)
        
        if with_data:
            return {"entry_points": [ln[0] for ln in line_list], "screw_lines": line_list}
        return {}
    
# -----------------------------------
# Main API
# -----------------------------------
def apply_screws(
        model,
        spec_model="WT-plus-6.5",
        orientation_aligned="perp_tread",
        orientation_crossed="cross_section",
        drill_target="main",
        depth_limited=True,
        with_data=False
    ):
    JOINT_MAP = {
        TMultiStepJoint : TMSJ,

    }

    solver = ScrewSolver(model, spec_model=spec_model)

    results = {}
    for joint in model.joints:
        if joint.name != "TMultiStepJoint":
            continue  # temporary for testing only TMultiStepJoint
        
        # Wrap the joint with the corresponding class in JOINT_MAP if it exists
        joint_class = joint.__class__
        if joint_class in JOINT_MAP:
            joint = JOINT_MAP[joint_class](joint)
        
        if solver.determine_entry_type(joint) == "aligned":
            orientation = orientation_aligned
        elif solver.determine_entry_type(joint) == "crossed":
            orientation = orientation_crossed
        else:
            raise ValueError("Unknown entry type.")
        
        result = solver.add_drilling_features(
            joint,
            orientation=orientation,
            target=drill_target,
            depth_limited=depth_limited,
            with_data=with_data
        )
        if with_data:
            names = (joint.main_beam.name, joint.cross_beam.name)
            results[names] = result
    if with_data:
        return results
    return {}