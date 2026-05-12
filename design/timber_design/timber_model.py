"""
Construct compas timber models based on the input graph.

Usage in grasshopper:
    from timber_model import graph_to_timber_models, apply_joints 
    models = graph_to_timber_models(graph, align_shoe=False)
    models = [apply_joints(model) for model in models]
"""
from compas.geometry import Vector, angle_vectors
from compas_timber.model import TimberModel
from compas_timber.elements import Beam, Plate
from compas.tolerance import Tolerance

from compas_timber.utils import get_polyline_normal_vector

from compas_timber.analyzers import TripletAnalyzer
from compas_timber.connections import JointTopology, TMultiStepJoint, TStepJoint, TBirdsmouthJoint, XLapJoint, KBirdsmouthJoint, TButtJoint, TLapJoint
from compas_timber.connections import LMiterJoint

from collections import Counter

# ---------------------------------------
# Graph helpers
# ---------------------------------------
def get_node_attribute(graph, key, name, else_value=None):
    """Get node attribute with a fallback value if attribute is missing or None."""
    attrs = graph.node_attributes(key)
    if name not in attrs:
        return else_value
    
    value = graph.node_attribute(key, name)
    if value is not None:
        return value
    return else_value

def get_edge_attribute(graph, key, name, else_value=None):
    """Get edge attribute with a fallback value if attribute is missing or None."""
    attrs = graph.edge_attributes(key)
    if name not in attrs:
        return else_value
    
    value = graph.edge_attribute(key, name)
    if value is not None:
        return value
    return else_value


# --------------------------------------
# Element Creations
# --------------------------------------
def create_plate(polyline, group_id, thickness):
    """Create a plate pointing upwards."""
    vec = get_polyline_normal_vector(polyline)
    if vec.z < 0:
        vec = -vec
    plate = Plate.from_outline_thickness(polyline, thickness, vec)
    plate.attributes = {
        'normal': vec,
        'group': group_id
    }
    return plate

def create_beam(graph, edge, group_id, plate_vec=None):
    """
    Create a beam aligned with global Z.
    If plate_vec is provided, align the SHOE beam with the plate normal.
    """
    ln = get_edge_attribute(graph, edge, 'shifted_line', else_value=graph.edge_line(edge))
    w = get_edge_attribute(graph, edge, 'width', else_value=0.10)
    h = get_edge_attribute(graph, edge, 'height', else_value=0.14)
    hie = get_edge_attribute(graph, edge, 'hierarchy')
    lvl = get_edge_attribute(graph, edge, 'level')

    if hie == 'shoe' and plate_vec is not None:
        beam = Beam.from_centerline(ln, w, h, plate_vec)
    else:
        beam = Beam.from_centerline(ln, w, h)
    beam.name = hie  # temp.
    
    beam.attributes = {
        'hierarchy': hie,
        'edge': edge,
        'group': group_id,
        'level': lvl
    }
    return beam


# --------------------------------------
# Graph to TimberModels conversion
# --------------------------------------
def graph_to_timber_models(graph, align_shoe=True):
    """Convert graph to timber models based on group index."""
    groups = {}
    for edge in graph.edges():
        g = get_edge_attribute(graph, edge, 'group')
        groups.setdefault(g, {'edges': [], 'clt_plate': None, 'cut_plane': None})['edges'].append(edge)

    for node in graph.nodes():
        for name in ('clt_plate', 'cut_plane'):
            val = get_node_attribute(graph, node, name)
            if val:
                g = get_node_attribute(graph, node, 'group')
                groups[g][name] = val
    

    models = []
    for g, data in groups.items():
        model = TimberModel(Tolerance(unit="M", absolute=0.001))
        model.attributes['group'] = g

        plate_vec = None
        if data['clt_plate']:
            plate = create_plate(data['clt_plate'], g, thickness=0.10)
            plate_vec = plate.attributes['normal']
            model.add_element(plate)
        
        if data['cut_plane']:
            model.attributes['cut_plane'] = data['cut_plane']

        for edge in data['edges']:
            beam = create_beam(graph, edge, g, plate_vec=plate_vec if align_shoe else None)
            model.add_element(beam)

        models.append(model)
    return models


# --------------------------------------
# Joinery helpers
# --------------------------------------
def is_planar(element_a, element_b):
    """Check if two beams are planar.

    Parameters
    ----------
    element_a : :class:`compas_timber.elements.Beam`
        The first beam.
    element_b : :class:`compas_timber.elements.Beam`
        The second beam.

    Returns
    -------
    bool
        ``True`` if the beams are planar, ``False`` otherwise.

    """
    # Get frame from each beam
    frame_a = element_a.frame
    frame_b = element_b.frame

    # Match orientation of the frames
    frame_a.xaxis = Vector(0, 0, 1)
    frame_b.xaxis = Vector(0, 0, 1)

    # Calculate the angle between the two vectors
    angle = angle_vectors(frame_a.normal, frame_b.normal, deg=True)

    # If the angle is close to 0 or 180 degrees, the beams are planar
    return abs(angle) < 1e-6 or abs(angle - 180) < 1e-6

# --------------------------------------
# Joinery planning
# --------------------------------------
def condition():
    pass



def apply_joints(model, max_distance=0.055):

    # reset beams and features
    for beam in model.beams:
        beam.reset_computed_properties()

    # compute line network
    model.connect_adjacent_beams(max_distance)

    max_offset = max(candidate.distance for candidate in model.joint_candidates)

    # handle non-pair joints (in this case a 3-way connection using TripletAnalyzer)
    analyzer = TripletAnalyzer(model, max_offset/2)
    clusters = analyzer.find()

    for cluster in clusters:
        if cluster.topology == JointTopology.TOPO_K:
            # keep track of the amount of elements per type
            counts = Counter(e.attributes["hierarchy"] for e in cluster.elements)
            if counts["secondary"] == 2 and counts["tertiary"] == 1: #skip this joint
                continue

            # define the order of the elements to be passed to the joint
            order = {"main_primary": 0, "primary": 1, "secondary": 2}
            if counts["main_primary"] == 2 and counts["primary"] == 1:
                order = {"primary": 0, "main_primary": 1, "secondary": 2} #override the order just for this condition
            reordered_elements =  sorted(cluster.elements, key=lambda e: order.get(e.attributes["hierarchy"], 99))

            if len(reordered_elements) != 3:
                raise ValueError(f"Something went wrong with the analyzer. There should be always 3 elements, got: {len(reordered_elements)}")

            # promote cluster
            kbj = KBirdsmouthJoint.promote_cluster(model, cluster, reordered_elements=reordered_elements, mill_depth=0.01)

    #handle all pair joints
    for candidate in model.joint_candidates:
        beam_a, beam_b = candidate.elements             #parallel check with beam.attributes
        if candidate.is_promoted == False:              #all joints that are not k-topology

            """CLT shoe to Top Beam"""
            if candidate.topology == JointTopology.TOPO_T and is_planar(candidate.element_a, candidate.element_b) == True:
                if beam_b.name == 'shoe':
                    t_step_joint = TStepJoint.create(model, candidate.element_a, candidate.element_b, 
                    step_shape= "double"
                    )

                    """T - Multi Step Joint"""
                # elif beam_a.attributes["middle_joint"] == True:
                #     t_lap_joint = TLapJoint.create(model, candidate.element_a, candidate.element_b)

                else:
                    if angle_vectors(candidate.element_a.centerline.direction, candidate.element_b.centerline.direction, 
                    deg=True) < 50:
                        step_shape = "heel"
                    else:
                        step_shape = "step"
                    t_step_joint = TMultiStepJoint.create(model, candidate.element_a, candidate.element_b, 
                    step_shape = step_shape,
                    step_depth = .020,
                    riser_angle = 90
                    )

                """T - Non-Planar Step Joint"""
            elif candidate.topology == JointTopology.TOPO_T and is_planar(candidate.element_a, candidate.element_b) == False:
                t_butt_joint = TButtJoint.create(model, candidate.element_a, candidate.element_b)
                # t_np_step_joint = TMultiStepJoint.create(model, candidate.element_a, candidate.element_b,
                # do_refine_cut=True,
                # step_depth = .020,
                # riser_angle = 90
                # )

                """L Mitre Joint"""
            elif candidate.topology==JointTopology.TOPO_L:
                l_miter_joint = LMiterJoint.create(model, candidate.element_a, candidate.element_b, 
                cutoff = False
                )

                """X Lap Joint"""
            elif candidate.topology == JointTopology.TOPO_X:
                x_lap_joint = XLapJoint.create(model, candidate.element_a, candidate.element_b
                )

            else:
                continue
    return model