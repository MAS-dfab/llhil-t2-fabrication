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
import math

from compas.geometry import (
    Vector, Frame, Plane, Line, Point, Translation, Rotation,
    angle_vectors, intersection_line_plane, intersection_line_line_xy,
    cross_vectors, normalize_vector
)
from compas_timber.model import TimberModel
from compas_timber.elements import Beam, Plate

from compas_timber.utils import get_polyline_normal_vector

from compas_timber.analyzers import TripletAnalyzer

from compas_timber.connections import (
    JointTopology, TMultiStepJoint, TStepJoint, LMiterJoint,
    XLapJoint, KBirdsmouthJoint, TBirdsmouthJoint, TLapJoint, TButtJoint
)

from compas_timber.fabrication import JackRafterCut, LongitudinalCut
from collections import Counter

from timber_config import (
    TIMBER_MODEL_TOL, PLATE_THICKNESS, PLATE_Z_OFFSET, MAX_JOINT_DIST,
    TMULTI_HEEL_THRESHOLD, TMULTI_STEP_DEPTH, TMULTI_RISER_ANGLE,
    KBIRD_MILL_DEPTH, KBIRD_MITER_TYPE, TBUTT_ANGLE_THRESHOLD
)

# --------------------------------------
# Geometry helpers
# --------------------------------------
def _average_points(points):
    x = sum(p.x for p in points) / len(points)
    y = sum(p.y for p in points) / len(points)
    z = sum(p.z for p in points) / len(points)
    return Point(x, y, z)

def _angled_end_plane(beam, at_start, angle_deg, tilt_axis="width", offset=0.0):
    """Cutting plane through one end of a beam, tilted `angle_deg` from a square cut.

    Parameters
    ----------
    beam : :class:`compas_timber.elements.Beam`
    at_start : bool
        True for the start end, False for the end end.
    angle_deg : float
        Tilt of the cut measured from a square (perpendicular) cut.
    tilt_axis : str
        "height" tilts the cut in the beam's height plane (like a rafter),
        "width" tilts it sideways.
    offset : float
        Shift the plane outward along the beam axis by this distance. Use this
        to push the cut onto a blank that has been extended by `add_blank_extension`.

    Returns
    -------
    :class:`compas.geometry.Plane`
    """
    cl = beam.centerline
    axial = beam.frame.xaxis.unitized()          # x = along the centerline
    point = cl.start if at_start else cl.end

    # shift outward by `offset` to reach an extended (blank) tip
    if offset:
        point = point - axial * offset if at_start else point + axial * offset

    # square-cut normal points OUT of the beam at that end
    normal = -axial if at_start else axial

    rot_axis = beam.frame.zaxis if tilt_axis == "height" else beam.frame.yaxis
    R = Rotation.from_axis_and_angle(rot_axis, math.radians(angle_deg))
    return Plane(point, normal.transformed(R))


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

def create_beam(graph, edge, group_id, idx, plate_vec=None):
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
    c_sign = graph.get_edge_attribute(edge, 'cyclic_sign')

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
        'direction_id': dir_id,
        'cyclic_sign': c_sign,
        "idx": idx
    }
    
    # Assign a name to a beam
    beam.name = f"M{group_id}L{lvl}_{idx}"
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

    _fix_x_lap_side_for_shoes(graph) # NOTE: this is a temporary fix. Find more information in the function.

    groups = {}
    for edge in graph.edges():
        g = graph.get_edge_attribute(edge, 'group')
        groups.setdefault(g, {'edges': [], 'clt_plate': None, 'cut_plane': None, 'reached': None})['edges'].append(edge)

    for node in graph.nodes():
        for name in ('clt_plate', 'cut_plane'):
            val = graph.get_node_attribute(node, name)
            if val:
                g = graph.get_node_attribute(node, 'group')
                groups[g][name] = val

        # Add reached points to the group. 
        if graph.get_node_attribute(node, "reached"):
            g = graph.get_node_attribute(node, 'group')
            point = graph.get_node_attribute(node, "point")
            groups[g]["reached"] = point

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
            model.attributes['clt_plate_polyline'] = data['clt_plate']
        
        if data['cut_plane']:
            model.attributes['cut_plane'] = data['cut_plane']
        
        if data['reached']:
            model.attributes['reached'] = data['reached']

        for idx, edge in enumerate(data['edges']):
            beam = create_beam(graph, edge, g, idx, plate_vec=plate_vec if align_shoe else None)
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
    return angle < 1e-6 or angle - 180 < 1e-6

def _get_vertical_miter_plane_k(reordered_elements, flip=False):
    _, a, b = reordered_elements
    ori = a.centerline.start  # temp.

    cross = a.centerline.direction.cross(b.centerline.direction)
    if flip:
        return Plane.from_frame(Frame(ori, cross, -Vector(0, 0, 1)))
    return Plane.from_frame(Frame(ori, cross, Vector(0, 0, 1)))

def _get_average_miter_plane_k(reordered_elements, flip=False):
    _, a, b = reordered_elements
    ori = a.centerline.start
    dir_a = a.centerline.direction
    dir_b = b.centerline.direction

    cross = dir_a.cross(dir_b)

    y = dir_a + dir_b
    if flip:
        return Plane.from_frame(Frame(ori, cross, -y))
    return Plane.from_frame(Frame(ori, cross, y))

def _fix_x_lap_side_for_shoes(graph, offset_tol=1e-6):
    """
    Move one of the pair shoes slightly up in Z to avoid finding incorrect ref_side for the X Lap Joint.
    This is a temporary fix and should be handled better in the future.
    See: compas_timber -> l_lap_joint.py -> _get_beam_ref_side_index -> offset_vector, line 101.
    """
    # TODO: this is a temporary fix and should be handled better in the future. See: compas_timber -> l_lap_joint.py -> _get_beam_ref_side_index -> offset_vector, line 101.
    off_vec = Vector(0, 0, offset_tol)
    for edge in graph.edges_where({"hierarchy": "shoe", "direction_id": "A"}):
        ln = graph.get_edge_attribute(edge, 'shifted_line', else_value=graph.edge_line(edge))
        new_ln = Line(ln.start + off_vec, ln.end + off_vec)

        graph.edge_attribute(edge, 'shifted_line', new_ln)
    return
    
# --------------------------------------
# Joinery planning
# --------------------------------------
def _k_birdsmouth_solver(model, mill_depth, max_distance=None, miter_type=None, miter_flag=False):
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
    analyzer = TripletAnalyzer(model, max_distance=max_distance)
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
                miter_pln = _get_vertical_miter_plane_k(reordered_elements, flip=miter_flag)
            elif miter_type == 'AVERAGE':
                miter_pln = _get_average_miter_plane_k(reordered_elements, flip=miter_flag)
            else:
                miter_pln = None

            if reordered_elements[-1].attributes["level"] == 0:
                kwargs = {"mill_depth": mill_depth, "miter_plane": miter_pln}
            else:
                kwargs = {"mill_depth": mill_depth/2, "miter_plane": miter_pln}
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
        x_lap_flip=False,
        debug=False,
        tbutt_angle_threshold=None
    ):

    # Default config
    if max_distance is None:
        max_distance = MAX_JOINT_DIST
    if k_mill_depth is None:
        k_mill_depth = KBIRD_MILL_DEPTH
    # if k_miter_type is None:
    #     k_miter_type = KBIRD_MITER_TYPE
    if heel_threshold is None:
        heel_threshold = TMULTI_HEEL_THRESHOLD
    if step_depth is None:
        step_depth = TMULTI_STEP_DEPTH
    if riser_angle is None:
        riser_angle = TMULTI_RISER_ANGLE
    if tbutt_angle_threshold is None:
        tbutt_angle_threshold = TBUTT_ANGLE_THRESHOLD

    for beam in model.beams:
        beam.reset_computed_properties()

    
    model.connect_adjacent_beams(max_distance)

    # 1. Handle K joints with three beams first
    _k_birdsmouth_solver(
        model,
        max_distance=max_distance,
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

        if ca.attributes["level"] >= 1 and cb.attributes["hierarchy"] == 'shoe':
            continue # avoid creating joint between shoe and lower beam

        if topo == JointTopology.TOPO_T:
            # Planar T joints
            if is_planar_t_joint(candidate):
                if cb.attributes["hierarchy"] == 'shoe' and ca.attributes["level"] == 0:
                    TStepJoint.create(model, ca, cb, step_shape="double", step_depth=step_depth)

                # Middle T Joint
                elif ca.attributes["has_middle_joint"] and cb.attributes["has_middle_joint"]:
                    TMultiStepJoint.create(model, ca, cb, step_depth=step_depth) # NOTE: step_shape?

                else:
                    if angle_vectors(ca.centerline.direction, cb.centerline.direction, deg=True) < heel_threshold:
                        step_shape = "heel"
                    else:
                        step_shape = "step"

                    TMultiStepJoint.create(
                        model,
                        ca,
                        cb,
                        step_shape=step_shape,
                        step_depth=step_depth,
                        riser_angle=riser_angle
                    )
            else:
                if angle_vectors(ca.centerline.direction, cb.centerline.direction, deg=True) < tbutt_angle_threshold:
                    TButtJoint.create(model, ca, cb)
                else:
                    # Non-planar T joints
                    TBirdsmouthJoint.create(model, ca, cb)

        # ### L Miter Joint at the middle of the structure
        # elif topo == JointTopology.TOPO_L and ca.attributes["has_middle_joint"] and cb.attributes["has_middle_joint"]:
        #     LMiterJoint.create(model, ca, cb, cutoff=False)

        ### X Lap Joint
        elif topo == JointTopology.TOPO_X:
            # Fix the order of main and cross beams based on direction_id
            if ca.attributes['direction_id'] == 'A':
                XLapJoint.create(model, ca, cb, flip_lap_side=x_lap_flip)
            elif ca.attributes['direction_id'] == 'B':
                XLapJoint.create(model, cb, ca, flip_lap_side=x_lap_flip)
            else:
                raise ValueError("Unclassified direction_id found. Please check edge classification.")

        else:
            if debug:
                print(f"Unhandled joint candidate with topology {topo}. edges: {ca.attributes['edge']}, {cb.attributes['edge']}")
            continue
    return model

def apply_processings(model):
    """Process joinery and finalize cuts which need to be done after."""
    # Shoe bevel knob: set the extension you want per end, in METERS. The bevel
    # angle is derived from it later (angle = atan(2*X / height)).
    SHOE_EXTENSION = 0.03  # per end, in meters (0.03 = 3 cm)
    # False -> trapezoid: bevel + a flat tip face, length preserved.
    # True  -> full cut: the plane slices the whole beam, leaving one face.
    SHOE_FULL_CUT = True

    # Extend shoe blanks on BOTH ends BEFORE joinery, so the joint cuts are
    # computed against the final (extended) blank and don't shift afterwards.
    # The full cut needs 2*X of stock so the plane exits through the far face;
    # the trapezoid needs only X. The plane position (offset=X) is the same either way.

    for beam in model.beams:
        if beam.attributes.get("hierarchy") == "shoe":
            ext = 2.0 * SHOE_EXTENSION if SHOE_FULL_CUT else SHOE_EXTENSION
            beam.attributes["shoe_ext"] = SHOE_EXTENSION
            beam.add_blank_extension(ext, ext)

    model.process_joinery()

    clt_plate = model.attributes.get("clt_plate")
    anchor_point = model.attributes.get("reached")
    

    # Prepare vertical cut planes for middle joints and reached beams.
    if anchor_point:
        vertical_cut_plane_x = Plane(anchor_point, Vector(0, 1, 0))
        vertical_cut_plane_y = Plane(anchor_point, Vector(1, 0, 0))
    else:
        raise ValueError("Anchor point is required for vertical cut planes. Please check if it's provided in the graph node attributes.")
    


    for beam in model.beams:
        beam.reset_computed_properties()

        """JackRafterCut representing the middle joint condition, similar to LMiterJoint."""
        if beam.attributes["has_middle_joint"] and beam.attributes['level'] >= 1 and anchor_point:
            # Copy cut planes
            cut_plane_x = vertical_cut_plane_x.copy()
            cut_plane_y = vertical_cut_plane_y.copy()
            
            mid_point = beam.centerline.midpoint
            # extend the beams to ensure the cut planes intersect with the full cross-section.
            beam.add_blank_extension(-0.1, 0.1)  # extend by 10 cm on the end side (relative to beam length)

            # Orient the cut planes so their normal points towards the beam centerline midpoint, to ensure the correct side of the plane is used for cutting.
            if mid_point.y > cut_plane_x.point.y:
                cut_plane_x.normal = cut_plane_x.normal * -1
            if mid_point.x > cut_plane_y.point.x:
                cut_plane_y.normal = cut_plane_y.normal * -1

            # Check intersection and add cuts
            if intersection_line_plane(beam.centerline, cut_plane_x):
                jrc_x = JackRafterCut.from_plane_and_beam(cut_plane_x, beam, is_joinery=False)
                beam.add_feature(jrc_x)
            if intersection_line_plane(beam.centerline, cut_plane_y):
                jrc_y = JackRafterCut.from_plane_and_beam(cut_plane_y, beam, is_joinery=False)
                beam.add_feature(jrc_y)
        
            """LongitudinalCut for shoes"""
        elif beam.attributes["hierarchy"] == "shoe":
            # Blank already extended by X on both ends BEFORE joinery (see top of
            # this function), so existing joints don't shift. Reuse the same X and
            # derive the bevel angle from it.
            X = beam.attributes.get("shoe_ext", 0.0)
            theta = math.degrees(math.atan(2.0 * X / beam.height))   # angle derived from X
            SHOE_END_ANGLE = -theta                                  # negative keeps bevel on the Bottom
            if clt_plate:
                cutting_frame = clt_plate.frame
                lc = LongitudinalCut.from_plane_and_beam(cutting_frame, beam, is_joinery=False)
                beam.add_feature(lc)

            for at_start in (True, False):
                sign = 1.0 if at_start else -1.0   # +/- gives a symmetric trapezoid; flip both signs to swap top/bottom
                # offset=X moves the cut plane out to the newly extended blank tip
                plane = _angled_end_plane(beam, at_start, sign * SHOE_END_ANGLE, offset=X)
                jrc = JackRafterCut.from_plane_and_beam(plane, beam, is_joinery=False)
                beam.add_feature(jrc)

            """End cuts for reached beams"""
        elif beam.attributes['reached']:
                # Get the horizontal cutting plane for the beam
                cutting_plane = model.attributes.get("cut_plane")

                print(anchor_point, beam.name)
                cut_plane = vertical_cut_plane_y.copy()

                # Orient the cut plane normal to point towards the beam centerline midpoint
                mid_point = beam.centerline.midpoint
                if mid_point.x > cut_plane.point.x:
                    cut_plane.normal = cut_plane.normal * -1

                # Check intersection and add cuts
                if intersection_line_plane(beam.centerline, cutting_plane):
                    jrc = JackRafterCut.from_plane_and_beam(cutting_plane, beam, is_joinery=False)
                    beam.add_feature(jrc)
                    vjrc_y = JackRafterCut.from_plane_and_beam(cut_plane, beam, is_joinery=False)
                    beam.add_feature(vjrc_y)

        else:
            continue
    return model

def apply_joints_middle_prototype(
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
    miter_beams = []
    planes = []

    # 2. Handle all pair joints, T, L, X
    for candidate in model.joint_candidates:
        if candidate.is_promoted:  # all joints that are not k-topology
            continue

        topo = candidate.topology
        ca, cb = candidate.elements

        is_planar = _is_planar(ca, cb)

        ### Planar T joints
        if topo == JointTopology.TOPO_T and is_planar:
            # CLT shoe to middle beam
            # if cb.attributes["hierarchy"] == 'shoe' and ca.attributes["has_middle_joint"]:
            #     # TLapJoint.create(model, ca, cb, flip_lap_side=_determine_lap_flip(ca, cb, mid_lap_flip))

            # CLT shoe to Top Beam
            if cb.attributes["hierarchy"] == 'shoe':
                TStepJoint.create(model, ca, cb, step_shape="double")
            
            # # Middle T Lap Joint
            # elif ca.attributes["has_middle_joint"] and cb.attributes["has_middle_joint"]:
            #     # TLapJoint.create(model, ca, cb, flip_lap_side=_determine_lap_flip(ca, cb, mid_lap_flip))
            #     TLapJoint.create(model, ca, cb)

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

        ### Non-planar T joints
        elif topo == JointTopology.TOPO_T and not is_planar:
            TBirdsmouthJoint.create(model, ca, cb)

        ### L Miter Joint at the middle of the structure
        elif topo == JointTopology.TOPO_L and ca.attributes["has_middle_joint"] and cb.attributes["has_middle_joint"]:
            # miter_plane = _get_vertical_miter_plane_l(ca, cb, location=candidate.location, flip=False)
            # a = LMiterJoint.create(model, ca, cb, cutoff=False, miter_plane=miter_plane)
            a = LMiterJoint.create(model, ca, cb, cutoff=False)
            planes.extend(a.cutting_planes)
            miter_beams.extend([ca, cb])

        ### X Lap Joint
        elif topo == JointTopology.TOPO_X:
            XLapJoint.create(model, ca, cb)

        else:
            if debug:
                print(f"Unhandled joint candidate with topology {topo}. edges: {ca.attributes['edge']}, {cb.attributes['edge']}")
            continue

    return model

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
        # elif beam.attributes["hierarchy"] == "shoe":
        #     if clt_plate:
        #         cutting_frame = clt_plate.frame
        #         lc = LongitudinalCut.from_plane_and_beam(cutting_frame, beam)
        #         beam.add_feature(lc)

        else:
            continue
    return model