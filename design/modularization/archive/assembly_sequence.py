"""
Sorting beams for fabrication.

Usage in grasshopper:
    1. Set this component's inputs: 
       - final_models (Item Access, Type Hint: No Type Hint / custom COMPAS object)
    2. Set this component's outputs:
       - sorted_beams
       - sorted_points
"""


from doctest import debug
import math
from xml.parsers.expat import model
from compas_timber.fabrication import Lap
from compas.geometry import Plane, Transformation, Frame, Vector

from assembly_config import (
    ASSEMBLY_MODEL_TOL, COLORS, HIERARCHY_RANK, DIRECTION_RANK
)

# --------------------------------------
# Geometry helpers
# --------------------------------------

def _get_reordered_elements(model):
    """Reorder joints and beams for sorting."""
    joints_list = list(model.joints)
    sorted_joints = sorted(joints_list, key=lambda j: (j.location.x, j.location.y, j.location.z))
    joint_rank = {joint: idx for idx, joint in enumerate(sorted_joints)}
    unvisited = {(i, beam): beam for i, beam in enumerate(model.beams)}
    
    return sorted_joints, joint_rank, unvisited

def determine_assembly_method(level: int, hierarchy: str, direction_id: str) -> str:
    """Encapsulates the business logic for matching assembly methods."""
    if level == 0:
        if hierarchy == "shoe" and direction_id == "B":
            return "human"
        if hierarchy == "shoe" and direction_id == "A":
            return "robot"
        return "human"
        
    if level == 1:
        if hierarchy == "tertiary":
            return "human"
        return "robot"
        
    return "robot"  # Fallback default

def find_average_plane_by_normal(planes):
    """Find the average plane orientation using only the normals."""
    if not planes:
        return None
    
    # 1. Sum the X, Y, and Z components of the unitized normals safely
    sum_nx = sum(plane.normal.unitized().x for plane in planes)
    sum_ny = sum(plane.normal.unitized().y for plane in planes)
    sum_nz = sum(plane.normal.unitized().z for plane in planes)
    
    # 2. Combine them into a single vector and unitize it to get the clean average direction
    avg_normal = Vector(sum_nx, sum_ny, sum_nz).unitized()
    
    # 3. Return the new plane using the first plane's point and the average normal
    return Plane(planes[0].point, avg_normal)

# --------------------------------------
# Visualize beams
# --------------------------------------

def visualize_sorted_beams(model, colors_map=None, step=None):
    """Visualize sorted beams with colors."""
    colors_map = colors_map or COLORS
    
    geometry, colors = [], []
    
    # Add plates to geometry for visualization
    for plate in model.plates:
        plate.attributes["color"] = COLORS["plate"]
        plate.attributes["assembly_method"] = "plate"
        
        geometry.append(plate.geometry)
        colors.append(COLORS["plate"])


    # Add beams to geometry for visualization
    for beam in model.beams:
        attrs = beam.attributes
        
        # Step 1: Determine assembly method & look up color
        method = determine_assembly_method(
            level=attrs.get("level"), 
            hierarchy=attrs.get("hierarchy"), 
            direction_id=attrs.get("direction_id")
        )
        color = colors_map.get(method)
        
        # Step 2: Calculate physical metrics
        volume = beam.geometry.volume
        weight = volume * 470
        beam_width = beam.width
        beam_height = beam.height
        
        # Step 3: Add attributes to the beams
        beam.attributes["color"] = color
        beam.attributes["assembly_method"] = method
        beam.attributes["volume"] = volume
        beam.attributes["weight"] = weight
        beam.attributes["crosssection"] = beam_width, "x", beam_height
        beam.attributes["blanklength"] = beam.blank_length
        
        geometry.append(beam.geometry)
        colors.append(color)

    if step == None:
        step = len(geometry)
    
    return geometry[:step], colors[:step]


# --------------------------------------
# Orient beams to station frame
# --------------------------------------

def orient_to_station(model, frame):
    """Orient the model to a given station frame."""
        
    # Get model attributes for transformation
    plate = model.attributes['clt_plate']
    cut_plane =  model.attributes['cut_plane']
    from_frame = plate.attributes['negative_frame']

    # Compute transformation from model's current frame to the station frame
    O = Transformation.from_frame_to_frame(from_frame, frame)
    
    # Apply transformation to all elements in the model
    cut_plane.transform(O)
    for plate in model.plates:
        geo = plate.geometry
        geo.transform(O)
        plate.transform(O)
        plate.frame.transform(O)
    for beam in model.beams:
        geo = beam.geometry
        beam.transform(O)
        beam.frame.transform(O)
        geo.transform(O)
    for joint in model.joints:
        joint.location.transform(O)
    
    # Update model attributes if necessary
    model.attributes['cut_plane'] = cut_plane
    model.attributes['clt_plate'] = plate
        
    return model

# --------------------------------------
# Main API 
# --------------------------------------

def sort_beams_by_joint_location(model):
    """Sort beams for fabrication based on joint locations."""
    joints_list = list(model.joints)
    sorted_joints_by_z = sorted(joints_list, key=lambda j: (j.location.z))
    sorted_joints_by_xy = sorted(joints_list, key=lambda j: (j.location.x, j.location.y))
    sorted_beams = []
    sorted_joints_points = [joint.location for joint in sorted_joints_by_z]
    
    return sorted_joints_by_z, sorted_joints_by_xy, sorted_joints_points

def sort_beams_by_attributes(model, hierarchy_rank= None, direction_rank=None):
    """Sort beams for fabrication based on beam attributes."""
    beams_list = list(model.beams)
    # Step 1: Set the rank of specific attributes
    if hierarchy_rank == None and direction_rank == None:
        hierarchy_rank = HIERARCHY_RANK
        direction_rank = DIRECTION_RANK
    
    # NOTE: Add joint location as a priority?
    sorted_joints_by_z, sorted_joints_by_xy, _ = sort_beams_by_joint_location(model)
    joint_rank_by_z = {beam: idx for idx, joint in enumerate(sorted_joints_by_z) for beam in joint.elements}
    joint_rank_by_xy = {beam: idx for idx, joint in enumerate(sorted_joints_by_xy) for beam in joint.elements}

    # Step 2: Set the priorities of each attribute
    attributes_rank = {}
    for i, beam in enumerate(beams_list):
        level = beam.attributes["level"]
        edge = beam.attributes["edge"]
        edge_a = edge[0]
        hierarchy_idx = hierarchy_rank[beam.attributes["hierarchy"]]
        direction_id = direction_rank[beam.attributes["direction_id"]]
        joint_position_z = joint_rank_by_z[beam]
        joint_position_xy = joint_rank_by_xy[beam]
        
        shoe_priority = 0 if beam.attributes["hierarchy"] == "shoe" else 1
        middle_joint_priority = 999 if beam.attributes.get("has_middle_joint") and not level == 0 else 0

        
        attributes_rank[beam] = (shoe_priority, level, middle_joint_priority, edge_a, joint_position_z, joint_position_xy, hierarchy_idx)
        # beam.attributes["sort_key"] = (level, hierarchy_idx, direction_id, edge_a)
    
    # Step 3: Sort beams based on the defined priorities
    sorted_beams = sorted(beams_list, key=lambda b: attributes_rank[b])
    
    # NOTE: We have to add beam beams[i].attributes["seq_idx"] = x
    for i, beam in enumerate(sorted_beams):
        beam.attributes["seq_idx"] = i
        
    return sorted_beams

def beam_insertion_plane(model, debug=False):
    """Assign robot insertion plane for all the beams with assembly method robot""" 
    
    tolerance_degrees = 15.0
    tolerance_limit = math.cos(math.radians(tolerance_degrees))
    data = {}
    beams_by_robot = [beam for beam in model.beams if beam.attributes["assembly_method"] == "robot"]
    
    beam_insertion_by_vecotr = []
    beam_insertion_by_z_axis = []
    
    for beam in beams_by_robot:
        if beam.attributes.get("has_middle_joint") or beam.attributes["hierarchy"] == "shoe":
            beam_insertion_by_z_axis.append(beam)
        else:
            beam_insertion_by_vecotr.append(beam)
    
    for joint in model.joints:
        elements = joint.elements
        for element in elements:
            if element in beam_insertion_by_vecotr:
                data.setdefault(element, []).append(joint)
    
    # Sort data joints by Z value and choose two lowest ones
    data_sorted = {beam: sorted(joints, key=lambda joint: joint.location.z) for beam, joints in data.items()}
    # Chose always two lowest points as reference
    data_reference = {beam: joints[:2] for beam, joints in data_sorted.items()}

    main_data = {}
    insertion_plane = {}
    for beam, joints in data_reference.items():
        my_joint_planes = []
        for joint in joints:  
            features = joint.features
            joint_location = joint.location
            
            planes = [feature.planes_from_params_and_beam(beam) for feature in features if type(feature).__name__ in ["DoubleCut", "BirdsMouth"]]
            joint_planes = [p for f_planes in planes for p in f_planes]
            
            # Frame always pointing downwards
            for i, plane in enumerate(joint_planes):
                if plane.normal.unitized().dot([0, 0, -1]) < 0:
                    flipped_normal = plane.normal * -1
                    joint_planes[i] = Plane(plane.point, flipped_normal)
            
            # Sort planes by their z value
            sorted_planes_by_z = sorted(
                joint_planes,
                key=lambda plane: plane.point.z,
                reverse=False
            )
            
            # Sort them by their normal's alignment with the global downward direction and choose the best one
            my_planes = [sorted_planes_by_z[0], sorted_planes_by_z[1]]
            sorted_planes = sorted(
                my_planes,
                key=lambda plane: plane.normal.unitized().dot([0,0,-1]),
                reverse=True
            )
            
            # Check the between the other beam in the joint, chose the one that is more perpendicular to the other beam's axis
            other_beam = [b for b in joint.elements if b != beam][0]
            other_axis = other_beam.frame.xaxis.unitized()
            sorted_planes = sorted(
                sorted_planes,
                key=lambda plane: abs(plane.normal.unitized().dot(other_axis)),
                reverse=False
            )
            
            # Choose the one that is more aligned with the global downward direction within a certain tolerance, otherwise just choose the best one
            matching_planes = []
            for plane in sorted_planes:
                u_normal = plane.normal.unitized()
                dot_product = abs(u_normal.dot([0,0,1]))
                
                # If the dot product is close to 1.0, it's aligned within our tolerance
                if dot_product >= tolerance_limit:
                    matching_planes.append(plane)
                
            if matching_planes:
                my_plane = matching_planes[0]
            else:
                my_plane = sorted_planes[0]

            my_joint_planes.append(my_plane)
                
            main_data.setdefault(beam, []).append([joint_location, joint.name, features, my_plane])
        
        insertion_vector = find_average_plane_by_normal(my_joint_planes)
        
        # Assign beam attribute for insertion plane
        beam.attributes["insertion_vector"] = insertion_vector
        for beam in beam_insertion_by_z_axis:
            beam.attributes["insertion_vector"] = Vector(0,0,-1)
        
        insertion_plane[beam] = insertion_vector
        
    if debug:
        return beam_insertion_by_vecotr, main_data, insertion_plane, model
        
    return model
                
def beam_grasp_frame(model):
    """Assign robot grasp frame for all the beams with assembly method robot""" 
    # For now we are just aligning the grasp frame with the insertion vector and using a fixed approach direction, but this could be further optimized based on the joint planes and other factors
    grasp_frames = []
    for beam in model.beams:
        if beam.attributes["assembly_method"] == "robot":
            beam_face = beam.side_as_surface(0)
            pick_up_point = beam_face.point_at(0.5, 0.5)
            beam_frame = beam.frame.copy()
            beam_frame.point = pick_up_point

            beam.attributes["grasp_frame"] = beam_frame
            grasp_frames.append(beam_frame)
    
    return model, grasp_frames
    
    



    