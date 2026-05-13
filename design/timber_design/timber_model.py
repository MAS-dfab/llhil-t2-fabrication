"""
Construct compas timber models based on the input graph.

Usage in grasshopper:
    from timber_model import graph_to_timber_models, apply_joints 
    models = graph_to_timber_models(graph, align_shoe=False)
    models = [apply_joints(model) for model in models]
"""
from compas.geometry import Vector, angle_vectors, Translation, intersection_segment_plane, Line, Point, intersection_line_line_xy
from compas_timber.model import TimberModel
from compas_timber.elements import Beam, Plate

from compas_timber.utils import get_polyline_normal_vector

from compas_timber.analyzers import TripletAnalyzer

from compas_timber.connections import (
    JointTopology, TMultiStepJoint, TStepJoint, LMiterJoint,
    XLapJoint, KBirdsmouthJoint, TButtJoint, TLapJoint
)

from compas_timber.fabrication import JackRafterCut, LongitudinalCut
from collections import Counter

from timber_config import (
    TIMBER_MODEL_TOL, PLATE_THICKNESS, PLATE_Z_OFFSET, MAX_JOINT_DIST,
    TMULTI_HEEL_THRESHOLD, TMULTI_STEP_DEPTH, TMULTI_RISER_ANGLE, KBIRD_MILL_DEPTH
)

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
def create_plate(polyline, group_id, thickness, z_offset):
    """Create a plate pointing upwards."""
    if z_offset:
        polyline = polyline.transformed(Translation.from_vector(Vector(0, 0, z_offset)))

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
    is_mid = get_edge_attribute(graph, edge, 'middle_joint', else_value=False)

    if hie == 'shoe' and plate_vec is not None:
        beam = Beam.from_centerline(ln, w, h, plate_vec)
    else:
        beam = Beam.from_centerline(ln, w, h)
    beam.name = hie  # temp.
    
    beam.attributes = {
        'hierarchy': hie,
        'edge': edge,
        'group': group_id,
        'level': lvl,
        'is_middle_joint': is_mid
    }
    return beam


# --------------------------------------
# Graph to TimberModels conversion
# --------------------------------------
def graph_to_timber_models(graph, model_tol=None, plate_thickness=None, plate_z_offset=None, align_shoe=True):
    """Convert graph to timber models based on group index."""
    if model_tol is None:
        model_tol = TIMBER_MODEL_TOL
    if plate_thickness is None:
        plate_thickness = PLATE_THICKNESS
    if plate_z_offset is None:
        plate_z_offset = PLATE_Z_OFFSET

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
        model = TimberModel(model_tol)
        model.attributes['group'] = g

        plate_vec = None
        if data['clt_plate']:
            plate = create_plate(data['clt_plate'], g, thickness=plate_thickness, z_offset=plate_z_offset)
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
def _is_planar(element_a, element_b):
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

def _determine_lap_flip(candidate_a, candidate_b, lap_flip):
    """Align the lap side by finding"""
    start = candidate_b.centerline.start
    end = candidate_b.centerline.end
    mid = candidate_b.centerline.midpoint  # Cross beam midpoint

    # Pointing downwards
    aligned = start - end if start.z > end.z else end - start
    ori = start if start.z > end.z else end
    
    ln = Line(mid, mid + candidate_b.centerline.direction.cross(candidate_a.centerline.direction))
    meet = intersection_line_line_xy(ln, Line(ori, ori + Vector(0, 1, 0)))

    if meet:
        pt = Point(*meet)
        vec = pt - mid
        cross = vec.cross(aligned)
        return lap_flip ^ (cross.z <= 0)
    return


def orientate_plane(plane):
    """Orientate plane to have normal pointing downwards."""
    if plane.normal.z > 0:
        plane.normal = -plane.normal
    return plane


# --------------------------------------
# Joinery planning
# --------------------------------------
def _k_birdsmouth(model, mill_depth):
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
            KBirdsmouthJoint.promote_cluster(model, cluster, reordered_elements=reordered_elements, mill_depth=mill_depth)
    return

def _which_pair_joint(model):
    pass

# --------------------------------------
# Main API - Create Joints
# --------------------------------------
def apply_joints(
        model,
        max_distance=None,
        heel_threshold=None,
        step_depth=None,
        riser_angle=None,
        mill_depth=None,
        mid_lap_flip=False
    ):

    # Default config
    if max_distance is None:
        max_distance = MAX_JOINT_DIST
    if heel_threshold is None:
        heel_threshold = TMULTI_HEEL_THRESHOLD
    if mill_depth is None:
        mill_depth = KBIRD_MILL_DEPTH
    if step_depth is None:
        step_depth = TMULTI_STEP_DEPTH
    if riser_angle is None:
        riser_angle = TMULTI_RISER_ANGLE

    for beam in model.beams:
        beam.reset_computed_properties()

    model.connect_adjacent_beams(max_distance)

    # 1. Handle K joints with three beams first
    _k_birdsmouth(model, mill_depth=mill_depth)


    # 2. Handle all pair joints, T, L, X
    for candidate in model.joint_candidates:
        if candidate.is_promoted:  # all joints that are not k-topology
            continue

        topo = candidate.topology
        ca, cb = candidate.elements

        is_planar = _is_planar(ca, cb)

        # Planar T joints
        if topo == JointTopology.TOPO_T and is_planar:
            # CLT shoe to Top Beam
            if cb.name == 'shoe':
                TStepJoint.create(model, ca, cb, step_shape="double")

            # Middle T Lap Joint
            elif ca.attributes["is_middle_joint"]:
                # TLapJoint.create(model, ca, cb, flip_lap_side=_determine_lap_flip(ca, cb, mid_lap_flip))
                TLapJoint.create(model, ca, cb)

            else:
                if angle_vectors(ca.centerline.direction, cb.centerline.direction, deg=True) < heel_threshold:
                    step_shape = "heel"
                else:
                    step_shape = "step"

                TMultiStepJoint.create(
                    model, ca, cb,
                    step_shape=step_shape,
                    step_depth=step_depth,
                    riser_angle=riser_angle
                )

        # Non-planar T joints
        elif topo == JointTopology.TOPO_T and not is_planar:
            TButtJoint.create(model, ca, cb)

        # L Miter Joint
        elif topo == JointTopology.TOPO_L:
            LMiterJoint.create(model, ca, cb, cutoff=False)

        # X Lap Joint
        elif topo == JointTopology.TOPO_X:
            XLapJoint.create(model, ca, cb)

        else:
            continue
    return model

def apply_processings(model):
    for beam in model.beams:
        beam.reset_computed_properties()

        "JackRafterCut"
        if beam.attributes["hierarchy"] == "primary" or beam.attributes["hierarchy"] == "main_primary" and beam.attributes["level"] == max(b.attributes["level"] for b in model.beams):
            cutting_plane = model.attributes.get("cut_plane")
            orientated_cutting_plane = orientate_plane(cutting_plane)
            intersection = intersection_segment_plane(beam.centerline, orientated_cutting_plane)
            if intersection:
                jrc = JackRafterCut.from_plane_and_beam(orientated_cutting_plane, beam)
                beam.add_feature(jrc)
        else:
            continue

        # "LongitudinalCut"
        # if beam.attributes["hierarchy"] == "shoe":

    return model