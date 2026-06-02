import math
from compas.geometry import Plane, Transformation, Frame, Vector

# Assuming these are imported from your config file
from assembly_config import (
    ASSEMBLY_MODEL_TOL, COLORS, HIERARCHY_RANK, DIRECTION_RANK
)

import math
from compas.geometry import Plane, Transformation, Frame, Vector

class AssemblySolver:
    """
    Manages the lifecycle of orienting, processing, sorting, and calculating 
    fabrication attributes for a timber assembly model.
    """
    def __init__(self, model, hierarchy_rank=None, direction_rank=None, colors_map=None):
        self.model = model
        
        # Configuration Fallbacks
        self.hierarchy_rank = hierarchy_rank or HIERARCHY_RANK
        self.direction_rank = direction_rank or DIRECTION_RANK
        self.colors_map = colors_map or COLORS
        self.tolerance_limit = math.cos(math.radians(15.0))

    def run_pipeline(self):
        """Executes the fabrication pipeline in the strict required logical order."""
        
        # Step 1: Classify and apply physical metadata
        self._compute_assembly_metadata()
        
        # Step 2: Sort beams sequentially based on oriented positions
        # sorted_beams = self._sort_beams_by_attributes()
        sorted_beams = self._sort_beams_by_joints()
        
        # Step 3: Calculate fabrication vectors & tool frames
        self._compute_insertion_planes()
        grasp_frames = self._compute_grasp_frames()
        
        return sorted_beams, grasp_frames

    # -------------------------------------------------------------------------
    # Spatial Alignment (Station Orientation)
    # -------------------------------------------------------------------------

    def orient_to_station(self, target_frame):
        """
        Orients the entire model from its custom local plate frame 
        to the target fabrication station frame.
        """
        # 1. Safely extract tracking attributes from the model
        plate = self.model.attributes.get('clt_plate')
        cut_plane = self.model.attributes.get('cut_plane')
        
        if not plate or not cut_plane:
            raise ValueError("Model is missing 'clt_plate' or 'cut_plane' attributes required for orientation.")
            
        from_frame = plate.attributes.get('negative_frame')
        if not from_frame:
            raise ValueError("The CLT plate is missing its 'negative_frame' attribute.")

        # 2. Compute the exact coordinate matrix shift
        transformation = Transformation.from_frame_to_frame(from_frame, target_frame)
        
        # 3. Apply transformation across all linked elements uniformly
        cut_plane.transform(transformation)
        
        for p in self.model.plates:
            p.geometry.transform(transformation)
            p.transform(transformation)
            p.frame.transform(transformation)
            
        for beam in self.model.beams:
            beam.geometry.transform(transformation)
            beam.transform(transformation)
            beam.frame.transform(transformation)
            
        for joint in self.model.joints:
            joint.location.transform(transformation)
        
        # 4. Save references back to the model state
        self.model.attributes['cut_plane'] = cut_plane
        self.model.attributes['clt_plate'] = plate

    # -------------------------------------------------------------------------
    # Geometry Helpers
    # -------------------------------------------------------------------------
    def _get_beam_joints(self, beams):
        """ Build adjacency data mapping beam -> joints """
        beam_to_joints = {beam: [] for beam in beams}
        for joint in self.model.joints:
            for element in joint.elements:
                if element in beam_to_joints:
                    beam_to_joints[element].append(joint)
        
        return beam_to_joints
        
    # -------------------------------------------------------------------------
    # Internal Classification & Metadata
    # -------------------------------------------------------------------------

    def _determine_assembly_method(self, level: int, hierarchy: str, direction_id: str) -> str:
        """Business logic for matching assembly methods."""
        if level == 0:
            if hierarchy == "shoe" and direction_id in ["A", "B"]:
                return "robot" if direction_id == "A" else "human"
            return "human"
        if level == 1:
            return "human" if hierarchy == "tertiary" else "robot"
        return "robot"

    def _compute_assembly_metadata(self):
        """Enriches beams and plates with physical and operational attributes."""
        # Process plates
        for plate in self.model.plates:
            plate.attributes.update({
                "color": self.colors_map["plate"],
                "assembly_method": "plate"
            })

        # Process beams
        for beam in self.model.beams:
            attrs = beam.attributes
            method = self._determine_assembly_method(
                level=attrs.get("level"), 
                hierarchy=attrs.get("hierarchy"), 
                direction_id=attrs.get("direction_id")
            )
            
            volume = beam.geometry.volume
            beam.attributes.update({
                "color": self.colors_map.get(method),
                "assembly_method": method,
                "volume": volume,
                "weight": volume * 470,
                "crosssection": (beam.width, "x", beam.height),
                "blanklength": beam.blank_length
            })

    # -------------------------------------------------------------------------
    # Sorting Logic
    # -------------------------------------------------------------------------

    def _get_joint_position(self):
        """Computes spatial joint rankings for beam prioritizing."""
        joints_list = list(self.model.joints)
        sorted_z = sorted(joints_list, key=lambda j: j.location.z)
        sorted_xy = sorted(joints_list, key=lambda j: (j.location.x, j.location.y))
        
        # Build maps: beam -> joint index
        rank_z = {beam: idx for idx, joint in enumerate(sorted_z) for beam in joint.elements}
        rank_xy = {beam: idx for idx, joint in enumerate(sorted_xy) for beam in joint.elements}
        
        return rank_z, rank_xy

    def _sort_beams_by_attributes(self):
        """Sorts beams based on structural hierarchy and positional priorities."""
        rank_z, rank_xy = self._get_joint_position()
        beams_list = list(self.model.beams)
        attributes_rank = {}

        for beam in beams_list:
            attrs = beam.attributes
            level = attrs.get("level", 0)
            hierarchy = attrs.get("hierarchy", "")
            edge_a = attrs.get("edge", [0])[0]
            
            shoe_priority = 0 if hierarchy == "shoe" else 1
            middle_joint_priority = 999 if attrs.get("has_middle_joint") and level != 0 else 0
            hierarchy_idx = self.hierarchy_rank.get(hierarchy, 99)
            
            # Spatial priorities safely inferred from joint mapping
            joint_position_z = rank_z.get(beam, 999)
            joint_position_xy = rank_xy.get(beam, 999)

            attributes_rank[beam] = (
                shoe_priority, 
                level, 
                middle_joint_priority, 
                edge_a, 
                joint_position_z, 
                joint_position_xy, 
                hierarchy_idx
            )

        sorted_beams = sorted(beams_list, key=lambda b: attributes_rank[b])
        
        # Write sequence index back to attributes
        for idx, beam in enumerate(sorted_beams):
            beam.attributes["seq_idx"] = idx
            
        return sorted_beams

    def _sort_beams_by_joints(self):
        """Sorts beams based on joints hierarchy and positional priorities."""
        rank_z, rank_xy = self._get_joint_position()
        beams_list = list(self.model.beams)
        attributes_rank = {}
        count = -1

        beams_joints = self._get_beam_joints(beams_list)
        target_joint_types = ["TButtJoint", "TBirdsmouthJoint"]

        # --- STAGE 1: Initial Geometric Sorting ---
        for beam, joints in zip(beams_list, beams_joints.values()):
            matching_joints = [j for j in joints if beam in j.elements if type(j).__name__ in target_joint_types]
            attrs = beam.attributes
            level = attrs.get("level", 0)
            hierarchy = attrs.get("hierarchy", "")
            edge_a = attrs.get("edge", [0])[0]
            
            shoe_priority = 0 if hierarchy == "shoe" else 1
            middle_joint_priority = 999 if attrs.get("has_middle_joint") and level != 0 else 0
            hierarchy_idx = self.hierarchy_rank.get(hierarchy, 99)
            
            joint_position_z = rank_z.get(beam, 999)
            joint_position_xy = rank_xy.get(beam, 999)
            
            joint_priority = 99
            el_joint_priority = 99
            
            if matching_joints:
                for t_joint in matching_joints:
                    joint_elements = [element for element in t_joint.elements if element != beam]
                    
                    # Look back only at elements processed up to the current step
                    for beam_element in joint_elements:
                        if beam_element not in attributes_rank.keys():
                            el_attrs = beam_element.attributes
                            el_level = el_attrs.get("level", 0)
                            el_hierarchy = el_attrs.get("hierarchy", "")
                            el_edge_a = el_attrs.get("edge", [0])[0]
                            
                            el_shoe_priority = 0 if el_hierarchy == "shoe" else 1
                            el_middle_joint_priority = 999 if el_attrs.get("has_middle_joint") and el_level != 0 else 0
                            el_hierarchy_idx = self.hierarchy_rank.get(el_hierarchy, 99)
                            
                            el_joint_position_z = rank_z.get(beam_element, 999)
                            eL_joint_position_xy = rank_xy.get(beam_element, 999)
                            
                            if el_hierarchy_idx < hierarchy_idx:
                                el_joint_priority = count + 1
                                joint_priority = count + 2
                            else:
                                el_joint_priority = count + 2
                                joint_priority = count + 1
                            count += 2
                            # beam_element.attributes.update({"assembly_method": "robot"})
                            attributes_rank[beam_element] = (
                                el_shoe_priority, 
                                el_level, 
                                el_middle_joint_priority, 
                                el_joint_priority,
                                el_hierarchy_idx,
                                el_edge_a, 
                                el_joint_position_z, 
                                eL_joint_position_xy,
                            )

            if beam not in attributes_rank.keys(): 
                attributes_rank[beam] = (
                    shoe_priority, 
                    level, 
                    middle_joint_priority, 
                    joint_priority,
                    hierarchy_idx,
                    edge_a, 
                    joint_position_z, 
                    joint_position_xy,
                )

        # First pass sort based on geometric rules
        sorted_beams_geo = sorted(beams_list, key=lambda b: attributes_rank[b])

        # Write sequence index back to attributes
        for idx, beam in enumerate(sorted_beams_geo):
            beam.attributes["seq_idx"] = idx
            
        return sorted_beams_geo
        

    # -------------------------------------------------------------------------
    # Fabrication Logic (Planes & Vectors)
    # -------------------------------------------------------------------------

    @staticmethod
    def _find_average_plane_by_normal(planes):
        """Averages plane orientations safely via unit vectors."""
        if not planes:
            return None
        sum_nx = sum(p.normal.unitized().x for p in planes)
        sum_ny = sum(p.normal.unitized().y for p in planes)
        sum_nz = sum(p.normal.unitized().z for p in planes)
        
        avg_normal = Vector(sum_nx, sum_ny, sum_nz).unitized()
        return Plane(planes[0].point, avg_normal)

    def _compute_insertion_planes(self):
        """Calculates structural insertion directions for robotically handled beams."""
        beams_by_robot = [b for b in self.model.beams if b.attributes.get("assembly_method") == "robot"]
        
        beam_insertion_by_vector = []
        beam_insertion_by_z_axis = []

        for beam in beams_by_robot:
            if beam.attributes.get("has_middle_joint") or beam.attributes.get("hierarchy") == "shoe":
                beam_insertion_by_z_axis.append(beam)
            else:
                beam_insertion_by_vector.append(beam)

        # Build adjacency data mapping beam -> joints
        beam_to_joints = self._get_beam_joints(beam_insertion_by_vector)

        # Calculate insertion vectors based on joint boundaries
        for beam, joints in beam_to_joints.items():
            sorted_joints = sorted(joints, key=lambda j: j.location.z)[:2]  # Keep 2 lowest joints
            my_joint_planes = []

            for joint in sorted_joints:
                planes = [f.planes_from_params_and_beam(beam) for f in joint.features if type(f).__name__ in ["DoubleCut", "BirdsMouth"]]
                joint_planes = [p for sub_planes in planes for p in sub_planes]

                # Standardize orientation pointing downwards
                for i, plane in enumerate(joint_planes):
                    if plane.normal.unitized().dot([0, 0, -1]) < 0:
                        joint_planes[i] = Plane(plane.point, plane.normal * -1)

                if not joint_planes:
                    continue

                # 1. Sort planes by their z value (lowest first)
                sorted_planes_by_z = sorted(joint_planes, key=lambda p: p.point.z)
                candidate_planes = sorted_planes_by_z[:2] if len(sorted_planes_by_z) > 1 else sorted_planes_by_z
                
                # 2. Sort by alignment with global downward direction
                sorted_planes = sorted(
                    candidate_planes,
                    key=lambda plane: plane.normal.unitized().dot([0, 0, -1]),
                    reverse=True
                )
                
                # 3. CRITICAL: Sort by perpendicularity to the connected beam's axis
                other_beams = [b for b in joint.elements if b != beam]
                if other_beams:
                    other_beam = other_beams[0]
                    other_axis = other_beam.frame.xaxis.unitized()
                    sorted_planes = sorted(
                        sorted_planes,
                        key=lambda plane: abs(plane.normal.unitized().dot(other_axis)),
                        reverse=False  # Closer to 0 means more perpendicular
                    )
                
                # 4. Apply tolerance filter matching global down direction
                matching_planes = [p for p in sorted_planes if abs(p.normal.unitized().dot([0, 0, 1])) >= self.tolerance_limit]
                chosen_plane = matching_planes[0] if matching_planes else sorted_planes[0]
                
                my_joint_planes.append(chosen_plane)

            insertion_plane = self._find_average_plane_by_normal(my_joint_planes)
            beam.attributes["insertion_plane"] = Plane(beam.centerline.midpoint, insertion_plane.normal)

        for beam in beam_insertion_by_z_axis:
            beam.attributes["insertion_plane"] = Plane(beam.centerline.midpoint, Vector(0, 0, -1))

    def _compute_grasp_frames(self):
        """Generates tool pickup frames centered on the top faces of robotic beams."""
        grasp_frames = []
        for beam in self.model.beams:
            if beam.attributes.get("assembly_method") == "robot":
                beam_face = beam.side_as_surface(0)
                pick_up_point = beam_face.point_at(0.5, 0.5)
                
                beam_frame = beam.frame.copy()
                beam_frame.point = pick_up_point
                
                beam.attributes["grasp_frame"] = beam_frame
                grasp_frames.append(beam_frame)
        return grasp_frames