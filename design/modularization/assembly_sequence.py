"""
Sorting beams for fabrication.

Usage in grasshopper:
    1. Set this component's inputs: 
       - final_models (Item Access, Type Hint: No Type Hint / custom COMPAS object)
    2. Set this component's outputs:
       - sorted_beams
       - sorted_points
"""


from xml.parsers.expat import model
from compas_timber.fabrication import Lap

hierarchy_rank = {"shoe": 0, "tertiary": 1, "secondary": 2, "primary": 3, "main_primary": 4}
# Priority should be maybe chosen, it could be hierarchy, level, or something else.

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


# --------------------------------------
# Main API - Sorting beams for fabrication
# --------------------------------------

def sort_beams_for_fabrication(model):
    """Sort beams for fabrication based on joint locations and beam attributes."""
    
    sorting_data = []
    sorted_joints, joint_rank, unvisited = _get_reordered_elements(model)

    for joint in sorted_joints:
        for (i, beam) in list(unvisited):
            if beam not in joint.elements:
                continue

            level = beam.attributes.get("level")
            edge = beam.attributes.get("edge")
            edge_a = edge[0]

            if level == 0:
                hierarchy_rank = {"shoe": 0, "tertiary": 2, "secondary": 4, "primary": 4, "main_primary": 4}
                start_priority = beam.centerline.start.z

            elif level == 1:
                hierarchy_rank = {"shoe": 0, "tertiary": 4, "secondary": 2, "primary": 3, "main_primary": 3}
                start_priority = 9999
            else: 
                hierarchy_rank = {"shoe": 0, "tertiary": 2, "secondary": 2, "primary": 4, "main_primary": 4}
                start_priority = 9999

            hierarchy = beam.attributes.get("hierarchy", "")
            shoe_priority = 0 if hierarchy == "shoe" else 1
            if beam.attributes.get("has_middle_joint") and level == 1:
                mid_priority = 1 
            else: 
                mid_priority = 0

            print(start_priority)
            
            joint_idx = joint_rank[joint]
            hierarchy_idx = hierarchy_rank.get(hierarchy)
            mid_point = beam.centerline.midpoint
    

            sorting_data.append((
                                shoe_priority,
                                level,
                                mid_priority,
                                hierarchy_idx,
                                edge_a,
                                start_priority,
                                mid_point.z,
                                joint_idx,
                                i,
                                beam,
                                joint,
                            ))
        

# for i,data in enumerate(sorting_data):
#     print(data[0], data[1],data[2], data[3])
# sorting_data.sort()
# for i,data in enumerate(sorting_data):
#     print("sorted", data[0], data[1],data[2],data[3])



# beams = final_models.beams
# sorted_beams = []
# for x, (*_, i, beam, joint) in enumerate(sorting_data):
#     if (i, beam) in unvisited:
#         beams[i].attributes["seq_idx"] = x
#         sorted_beams.append(beam.geometry)
#         #print(i, joint.name, joint.location)
#         del unvisited[(i, beam)]


    
# final_model = final_models