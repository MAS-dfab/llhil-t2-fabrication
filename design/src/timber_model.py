"""
Construct compas timber models based on the input graph.

Usage in grasshopper:
    from timber_model import graph_to_timber_models, apply_joints

    models = graph_to_timber_models(graph, align_shoe=False)
    
    for model in models:
        apply_joints(model, k_miter_type="VERTICAL", k_miter_flag=True, debug=True)

        if process:
            apply_processings(model)
"""
from compas.geometry import (
    Vector, Frame, Plane, Line, Point, Translation,
    angle_vectors, intersection_line_plane, intersection_line_line_xy
)
from compas_timber.model import TimberModel
from compas_timber.elements import Beam, Plate

from compas_timber.utils import get_polyline_normal_vector

from compas_timber.analyzers import TripletAnalyzer

from compas_timber.connections import (
    JointTopology, TMultiStepJoint, TStepJoint, LMiterJoint,
    XLapJoint, KBirdsmouthJoint, TBirdsmouthJoint, TLapJoint
)

from compas_timber.fabrication import JackRafterCut, LongitudinalCut
from collections import Counter

from timber_config import (
    TIMBER_MODEL_TOL, PLATE_THICKNESS, PLATE_Z_OFFSET, MAX_JOINT_DIST,
    TMULTI_HEEL_THRESHOLD, TMULTI_STEP_DEPTH, TMULTI_RISER_ANGLE,
    KBIRD_MILL_DEPTH, KBIRD_MITER_TYPE
)

# --------------------------------------
# Geometry helpers
# --------------------------------------
def _average_points(points):
    x = sum(p.x for p in points) / len(points)
    y = sum(p.y for p in points) / len(points)
    z = sum(p.z for p in points) / len(points)
    return Point(x, y, z)

def _polyline_aligned_frame(polyline, thickness):
    """
    Create a compas Frame at the centroid of the polyline,
    aligned x with longest edge,
    and its normal pointing DOWNWARDS.
    """
    longest = max(polyline.lines, key=lambda ln: ln.length)

    normal = get_polyline_normal_vector(polyline)
    if normal.z < 0:
        normal = -normal

    center = _average_points(polyline.points[:-1])
    center += normal.unitized() * thickness

    cross = longest.direction.cross(normal)
    return Frame(center, longest.direction, cross)

# --------------------------------------
# Element Creations
# --------------------------------------
def create_plate(polyline, group_id, thickness, z_offset):
    """Create a plate pointing UPWARDS."""
    if z_offset:
        polyline = polyline.transformed(Translation.from_vector(Vector(0, 0, z_offset)))

    # Create a frame for orienting elements to the fabrication station
    neg_frame = _polyline_aligned_frame(polyline, thickness)
    pos_normal = -neg_frame.normal

    plate = Plate.from_outline_thickness(polyline, thickness, pos_normal)

    plate.attributes = {
        'positive_normal': pos_normal,
        'negative_frame': neg_frame,
        'group': group_id
    }
    return plate

def create_beam(graph, edge, group_id, plate_vec=None):
    """
    Create a beam aligned with global Z.
    If plate_vec is provided, align the SHOE beam with the plate normal.
    """
    ln = graph.get_edge_attribute(edge, 'shifted_line', else_value=graph.edge_line(edge))
    w = graph.get_edge_attribute(edge, 'width', else_value=0.10)
    h = graph.get_edge_attribute(edge, 'height', else_value=0.14)
    hie = graph.get_edge_attribute(edge, 'hierarchy')
    lvl = graph.get_edge_attribute(edge, 'level')
    reached = graph.get_edge_attribute(edge, 'reached', else_value=False)
    has_mid = graph.get_edge_attribute(edge, 'has_middle_joint', else_value=False)
    dir_id = graph.get_edge_attribute(edge, 'direction_id')

    if hie == 'shoe' and plate_vec is not None:
        beam = Beam.from_centerline(ln, w, h, plate_vec)
    else:
        beam = Beam.from_centerline(ln, w, h)
    
    beam.attributes = {
        'hierarchy': hie,
        'edge': edge,
        'group': group_id,
        'level': lvl,
        'reached': reached,
        'has_middle_joint': has_mid,
        'direction_id': dir_id
    }
    return beam


# --------------------------------------
# Graph to TimberModels conversion
# --------------------------------------
def graph_to_timber_models(graph, model_tol=None, plate_thickness=None, plate_z_offset=None, align_shoe=False):
    """Convert graph to timber models based on group index."""
    if model_tol is None:
        model_tol = TIMBER_MODEL_TOL
    if plate_thickness is None:
        plate_thickness = PLATE_THICKNESS
    if plate_z_offset is None:
        plate_z_offset = PLATE_Z_OFFSET

    groups = {}
    for edge in graph.edges():
        g = graph.get_edge_attribute(edge, 'group')
        groups.setdefault(g, {'edges': [], 'clt_plate': None, 'cut_plane': None})['edges'].append(edge)

    for node in graph.nodes():
        for name in ('clt_plate', 'cut_plane'):
            val = graph.get_node_attribute(node, name)
            if val:
                g = graph.get_node_attribute(node, 'group')
                groups[g][name] = val
    

    models = []
    for g, data in groups.items():
        model = TimberModel(model_tol)
        model.attributes['group'] = g

        plate_vec = None
        if data['clt_plate']:
            plate = create_plate(data['clt_plate'], g, thickness=plate_thickness, z_offset=plate_z_offset)
            plate_vec = plate.attributes['positive_normal']
            model.add_element(plate)

            model.attributes['clt_plate'] = plate
        
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
def is_planar_t_joint(candidate, tol=1e-3):
    """
    Check if a T joint candidate is planar.
    """
    if candidate.topology != JointTopology.TOPO_T:
        return False

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

    ca, cb = candidate.elements
    pts = [*ca.centerline, *cb.centerline]

    # 1. Check coplanarity
    if not _is_coplanar(pts):
        return False

    # 2. Check cross-section alignment (width direction)
    ya = ca.frame.yaxis.unitized()
    yb = cb.frame.yaxis.unitized()

    return abs(ya.dot(yb)) >= 1 - tol

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

    # NOTE: don't change axis in place
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

def _get_vertical_miter_plane(reordered_elements, flip=False):
    _, a, b = reordered_elements
    ori = a.centerline.start  # temp.

    cross = a.centerline.direction.cross(b.centerline.direction)
    if flip:
        return Plane.from_frame(Frame(ori, cross, -Vector(0, 0, 1)))
    return Plane.from_frame(Frame(ori, cross, Vector(0, 0, 1)))

def _get_average_miter_plane(reordered_elements, flip=False):
    _, a, b = reordered_elements
    ori = a.centerline.start
    dir_a = a.centerline.direction
    dir_b = b.centerline.direction

    cross = dir_a.cross(dir_b)

    y = dir_a + dir_b
    if flip:
        return Plane.from_frame(Frame(ori, cross, -y))
    return Plane.from_frame(Frame(ori, cross, y))

# --------------------------------------
# Joinery planning
# --------------------------------------
def _k_birdsmouth_solver(model, mill_depth, miter_type=None, miter_flag=False):
    """
    Handle K birdsmouth joints with three beams.

    Parameters
    ----------
    model : TimberModel
        The timber model after calling connect.adjacent_beams. 
    mill_depth : float
        The depth of the mill cut for the birdsmouth joint.
    miter_type : str, optional
        "VERTICAL" for a global Z aligned miter plane.
        "AVERAGE" for a miter plane based on the average of the two beam directions.
        If None, miter plane will generate within compas_timber definition, see k_birdsmouth.py.
    miter_flag : bool, optional
        Flip miter plane direction if generating weird cuts.
    
    Returns
    -------
    None
    """
    max_offset = max(candidate.distance for candidate in model.joint_candidates)

    # handle non-pair joints (in this case a 3-way connection using TripletAnalyzer)
    analyzer = TripletAnalyzer(model, max_offset/2)  # NOTE: don't hardcode the threshold
    clusters = analyzer.find()

    for cluster in clusters:
        if cluster.topology == JointTopology.TOPO_K:
            # keep track of the amount of elements per type
            counts = Counter(e.attributes["hierarchy"] for e in cluster.elements)
            if counts["secondary"] == 2 and counts["tertiary"] == 1: #skip this joint
                continue

            # define the order of the elements to be passed to the joint
            order = {"main_primary": 0, "primary": 1, "secondary": 2, "tertiary": 3}
            if counts["main_primary"] == 2 and counts["primary"] == 1:
                order = {"primary": 0, "main_primary": 1, "secondary": 2, "tertiary": 3} #override the order just for this condition
            reordered_elements = sorted(cluster.elements, key=lambda e: order.get(e.attributes["hierarchy"], float('inf')))

            if len(reordered_elements) != 3:
                raise ValueError(f"Something went wrong with the analyzer. There should be always 3 elements, got: {len(reordered_elements)}")

            # promote cluster
            if miter_type == 'VERTICAL':
                miter_pln = _get_vertical_miter_plane(reordered_elements, flip=miter_flag)
            elif miter_type == 'AVERAGE':
                miter_pln = _get_average_miter_plane(reordered_elements, flip=miter_flag)
            else:
                miter_pln = None

            kwargs = {"mill_depth": mill_depth, "miter_plane": miter_pln}
            # kwargs = {"mill_depth": mill_depth}
            KBirdsmouthJoint.promote_cluster(model, cluster, reordered_elements=reordered_elements, **kwargs)
    return


# --------------------------------------
# Main API - Create Joints
# --------------------------------------
def apply_joints(
        model,
        max_distance=None,
        k_mill_depth=None,
        k_miter_type=None,
        k_miter_flag=False,
        heel_threshold=None,
        step_depth=None,
        riser_angle=None,
        mid_lap_flip=False,
        debug=False
    ):

    # Default config
    if max_distance is None:
        max_distance = MAX_JOINT_DIST
    if k_mill_depth is None:
        k_mill_depth = KBIRD_MILL_DEPTH
    if k_miter_type is None:
        k_miter_type = KBIRD_MITER_TYPE
    if heel_threshold is None:
        heel_threshold = TMULTI_HEEL_THRESHOLD
    if step_depth is None:
        step_depth = TMULTI_STEP_DEPTH
    if riser_angle is None:
        riser_angle = TMULTI_RISER_ANGLE

    for beam in model.beams:
        beam.reset_computed_properties()

    model.connect_adjacent_beams(max_distance)

    # 1. Handle K joints with three beams first
    _k_birdsmouth_solver(
        model,
        mill_depth=k_mill_depth,
        miter_type=k_miter_type,
        miter_flag=k_miter_flag
    )

    # 2. Handle all pair joints, T, L, X
    for candidate in model.joint_candidates:
        if candidate.is_promoted:  # all joints that are not k-topology
            continue
        
        topo = candidate.topology
        ca, cb = candidate.elements

        if topo == JointTopology.TOPO_T:
            # Planar T joints
            if is_planar_t_joint(candidate):
                if cb.attributes["hierarchy"] == 'shoe' and ca.attributes["level"] == 0:
                    TStepJoint.create(model, ca, cb, step_shape="double")

                # Middle T Joint
                elif ca.attributes["has_middle_joint"] and cb.attributes["has_middle_joint"]:
                    # TLapJoint.create(model, ca, cb, flip_lap_side=_determine_lap_flip(ca, cb, mid_lap_flip))
                    # TLapJoint.create(model, ca, cb)
                    TStepJoint.create(model, ca, cb)

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
            else:
                # Non-planar T joints
                TBirdsmouthJoint.create(model, ca, cb)
        
        ### L Miter Joint at the middle of the structure
        elif topo == JointTopology.TOPO_L and ca.attributes["has_middle_joint"] and cb.attributes["has_middle_joint"]:
            LMiterJoint.create(model, ca, cb, cutoff=False)

        ### X Lap Joint
        elif topo == JointTopology.TOPO_X:
        #     if ca.attributes['direction_id'] == 'A':
        #         flip = False
        #     else:
        #         flip = True
            XLapJoint.create(model, ca, cb)

        else:
            if debug:
                print(f"Unhandled joint candidate with topology {topo}. edges: {ca.attributes['edge']}, {cb.attributes['edge']}")
            continue
    return

def apply_processings_middle_prototype(model):
    """Process joinery and finalize cuts which need to be done after."""
    model.process_joinery()
    
    clt_plate = model.attributes.get("clt_plate")
    for beam in model.beams:
        beam.reset_computed_properties()

        # Middle joint cut
        if beam.attributes["has_middle_joint"]:
            if beam.attributes['reached']:
                cutting_plane = model.attributes.get("cut_plane")
                if intersection_line_plane(beam.centerline, cutting_plane):
                    jrc = JackRafterCut.from_plane_and_beam(cutting_plane, beam)
                    beam.add_feature(jrc)
            
            else:
                if clt_plate:
                    if intersection_line_plane(beam.centerline, Plane.from_frame(clt_plate.frame)):
                        cutting_frame = clt_plate.frame
                        jrc = JackRafterCut.from_plane_and_beam(cutting_frame, beam)
                        beam.add_feature(jrc)

            # LongitudinalCut
        elif beam.attributes["hierarchy"] == "shoe":
            if clt_plate:
                cutting_frame = clt_plate.frame
                lc = LongitudinalCut.from_plane_and_beam(cutting_frame, beam)
                beam.add_feature(lc)

        else:
            continue
    return model

def apply_processings(model):
    """Process joinery and finalize cuts which need to be done after."""
    model.process_joinery()
    
    clt_plate = model.attributes.get("clt_plate")
    for beam in model.beams:
        beam.reset_computed_properties()

        # Middle joint cut
        if beam.attributes["has_middle_joint"]:
            if clt_plate:
                if intersection_line_plane(beam.centerline, Plane.from_frame(clt_plate.frame)):
                    cutting_frame = clt_plate.frame
                    jrc = JackRafterCut.from_plane_and_beam(cutting_frame, beam)
                    beam.add_feature(jrc)

            # LongitudinalCut
        elif beam.attributes["hierarchy"] == "shoe":
            if clt_plate:
                cutting_frame = clt_plate.frame
                lc = LongitudinalCut.from_plane_and_beam(cutting_frame, beam)
                beam.add_feature(lc)
        
            # End cut for reached beams
        elif beam.attributes['reached']:
                cutting_plane = model.attributes.get("cut_plane")
                if intersection_line_plane(beam.centerline, cutting_plane):
                    jrc = JackRafterCut.from_plane_and_beam(cutting_plane, beam)
                    beam.add_feature(jrc)

        else:
            continue
    return model