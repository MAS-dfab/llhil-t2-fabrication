"""
Edge classification for tree graph structures.

Usage in Grasshopper:
    from edge_classifier import classify_edges_by_Michael
    
    result = classify_edges_by_Michael(
        graph,
        segment_index=0,
        seg_x=8,
        seg_y=2
    )
    
    primary = result["primary_lines"]
    secondary = result["secondary_lines"]
    tertiary = result["tertiary_lines"]
    special = result[:special_lines"]
"""

from compas.geometry import Vector, Line, Point, centroid_points
from compas.geometry import distance_point_point_xy
from compas.datastructures import Graph
from config import (
    DEFAULT_PARALLEL_TOL, DEFAULT_NEAR_THRESHOLD, 
    DEFAULT_OVERLAP, DEFAULT_SEG_X, DEFAULT_SEG_Y, DEFAULT_ANGLE_TOL
)
import math
from nodegraph import NodeGraph


# --------------------------------------------------
# Edge categorization
# --------------------------------------------------
def get_default_directions_xy(graph):
    """Find two diagonal directions and one orthogonal direction of original grid, from un-inset points."""
    ori_pts = []
    for node in graph.nodes():
        ori_pt = graph.node_attribute(node, "original_point")
        if ori_pt:
            ori_pts.append(ori_pt)

    p0, p1, p2, p3 = ori_pts[:4]
    vec1 = Vector.from_start_end(p0, p2)
    vec2 = Vector.from_start_end(p1, p3)
    vec3 = Vector.from_start_end(p0, p1)
    vec_xy_1 = Vector(vec1.x, vec1.y, 0.0)
    vec_xy_2 = Vector(vec2.x, vec2.y, 0.0)
    vec_xy_3 = Vector(vec3.x, vec3.y, 0.0)
    return vec_xy_1.unitized(), vec_xy_2.unitized(), vec_xy_3.unitized()

def categorize_edge_types(graph, tol=1e-3):
    """Categorize beams into "orthogonal", "default_diagonal", 'moved_diagonal'"""
    # 1. Find two original grid diagonal directions from un-inset points
    default_dir1, default_dir2, ortho_dir = get_default_directions_xy(graph)

    for edge in graph.edges():
        evec = graph.edge_vector(edge)
        edir_xy = Vector(evec.x, evec.y, 0.0)
        edir_xy.unitize()

        # 2. Find default diagonal edges
        dot0 = abs(edir_xy.dot(default_dir1))
        dot1 = abs(edir_xy.dot(default_dir2))
        if (1 - tol) <= dot0 <= (1 + tol) or (1 - tol) <= dot1 <= (1 + tol):
            graph.edge_attribute(edge, "e_category", "default_diagonal")
            continue

        # 3. Find orthogonal edges
        dot = abs(edir_xy.dot(ortho_dir))
        if (1 - tol) <= dot <= (1 + tol) or (0 - tol) <= dot <= (0 + tol):
            graph.edge_attribute(edge, "e_category", "orthogonal")
            continue

        # 4. The rest are moved diagonal edges
        graph.edge_attribute(edge, "e_category", "moved_diagonal")
    return default_dir1, default_dir2, ortho_dir

# --------------------------------------------------
# Initial classification
# --------------------------------------------------
def classify_edges_by_support_direction(graph, edges, parallel_tol=None, debug=False, support_points=None):
    """
    Classify subgraph edge by comparing edge direction in XY
    to the direction from edge midpoint to nearest support.

    Parameters
    ----------
    subgraph : NodeGraph
        Graph with nodes having "point" and "reached" attributes.
    edge: edge
        Edge that needs to be category. 
    parallel_tol : float
        Threshold for dot product. >= parallel_tol is primary.
    debug : bool
        Print debug information.
    support_points : list of Point, optional
        Explicit support points. If None, auto-detect from graph.

    Returns
    -------
    tuple
        (primary_lines, secondary_lines, classification_data)
    """
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL
    
    # Get support points - either from parameter or use graph's method
    if support_points:
        sup_pts = support_points
        if debug:
            print("DEBUG: Using {} provided support points".format(len(sup_pts)))
    else:
        # Use NodeGraph's get_support_points method
        sup_pts = graph.get_support_points()
        if debug:
            print("DEBUG: Found {} support points from graph".format(len(sup_pts)))
    
    if not sup_pts:
        print("WARNING: No support points available!")
        return [], [], []

    primary_lines = []
    tertiary_lines = []
    data = []
    
    dot_values = []  # Track for debug

    for u, v in edges:
        # Preserve "double" edges - don't reclassify
        existing = graph.edge_attribute((u, v), "main_secondary")
        if existing == "double":
            pu = graph.node_attribute(u, "point")
            pv = graph.node_attribute(v, "point")
            if pu and pv:
                primary_lines.append(Line(pu, pv))  # doubles count as primary for output
                data.append({"edge": (u, v), "support_idx": None, "dot": None, "type": "double"})
            continue

        pu = graph.node_attribute(u, "point")
        pv = graph.node_attribute(v, "point")

        if pu is None or pv is None:
            continue

        # Edge vector in XY
        edge_vec = Vector.from_start_end(pu, pv)
        if edge_vec.length < 1e-9:
            continue
        edge_vec.unitize()

        # Midpoint
        mid = Line(pu, pv).midpoint

        # Find nearest support point
        best_support_idx = None
        best_d2 = float('inf')
        best_support_pt = None

        for i, sp in enumerate(sup_pts):
            if sp is None:
                continue
            d2 = distance_point_point_xy(mid, sp)
            if d2 < best_d2:
                best_d2 = d2
                best_support_idx = i
                best_support_pt = Point(sp.x, sp.y, 0.0)

        if best_support_pt is None:
            continue

        # Compare directions in XY
        mid_xy = Point(mid.x, mid.y, 0.0)
        sup_vec = Vector.from_start_end(mid_xy, best_support_pt)
        
        # Handle zero-length vector (midpoint at support)
        if sup_vec.length < 1e-9:
            # Default to primary if at support
            line = Line(pu, pv)
            primary_lines.append(line)
            etype = "primary"
            data.append({"edge": (u, v), "support_idx": best_support_idx, "dot": 1.0, "type": etype})
            graph.edge_attribute((u, v), "hierarchy", etype)
            graph.edge_attribute((u, v), "parallel_score", 1.0)
            graph.edge_attribute((u, v), "nearest_support", best_support_idx)
            continue
            
        sup_vec.unitize()
        
        # Edge vector projected to XY
        edge_vec_xy = Vector(edge_vec.x, edge_vec.y, 0.0)
        if edge_vec_xy.length < 1e-9:
            # Vertical edge - classify as tertiary
            line = Line(pu, pv)
            tertiary_lines.append(line)
            etype = 'tertiary'
            data.append({"edge": (u, v), "support_idx": best_support_idx, "dot": 0.0, "type": etype})
            graph.edge_attribute((u, v), "hierarchy", etype)
            graph.edge_attribute((u, v), "parallel_score", 0.0)
            graph.edge_attribute((u, v), "nearest_support", best_support_idx)
            continue
            
        edge_vec_xy.unitize()

        dot = abs(edge_vec_xy.dot(sup_vec))
        dot_values.append(dot)  # Track for debug
        line = Line(pu, pv)

        if dot >= parallel_tol:
            primary_lines.append(line)
            etype = "primary"
        else:
            tertiary_lines.append(line)
            etype = 'tertiary'

        data.append({
            "edge": (u, v),
            "support_idx": best_support_idx,
            "dot": dot,
            "type": etype
        })

        # Store on graph edge
        graph.edge_attribute((u, v), "hierarchy", etype)
        graph.edge_attribute((u, v), "parallel_score", dot)
        graph.edge_attribute((u, v), "nearest_support", best_support_idx)

    if debug and dot_values:
        print("DEBUG: Dot product statistics:")
        print("  Min: {:.4f}, Max: {:.4f}".format(min(dot_values), max(dot_values)))
        print("  Mean: {:.4f}".format(sum(dot_values) / len(dot_values)))
        print("  parallel_tol: {}".format(parallel_tol))
        above_tol = sum(1 for d in dot_values if d >= parallel_tol)
        print("  Edges >= tol: {}, Edges < tol: {}".format(above_tol, len(dot_values) - above_tol))
        print("  Sample dots (first 10): {}".format([round(d, 3) for d in dot_values[:10]]))
    
    if debug:
        print("DEBUG: Initial classification result:")
        print("  Primary: {}, Tertiary: {}".format(len(primary_lines), len(tertiary_lines)))

    return primary_lines, tertiary_lines, data

# --------------------------------------------------
# Dominant direction analysis
# --------------------------------------------------
def _normalize_vector_direction(v):
    """Normalize vector to positive half-plane for consistent comparison."""
    if v.x < -1e-9 or (abs(v.x) < 1e-9 and v.y < 0):
        return Vector(-v.x, -v.y, 0.0)
    return Vector(v.x, v.y, 0.0)

def find_dominant_direction(vecs, angle_tol=None):
    """
    Find the direction shared by most vectors.

    Parameters
    ----------
    vecs : list of Vector
        Input vectors.
    angle_tol : float
        Angle bin tolerance in radians (default from config).

    Returns
    -------
    Vector or None
        Dominant direction (unitized) or None if no vectors.
    """
    if not vecs:
        return None
    
    if angle_tol is None:
        angle_tol = DEFAULT_ANGLE_TOL

    # Normalize to positive half-plane
    normalized = [_normalize_vector_direction(v) for v in vecs]
    angles = [math.atan2(v.y, v.x) for v in normalized]

    # Bin by angle
    bins = {}
    for i, ang in enumerate(angles):
        bin_key = round(ang / angle_tol)
        if bin_key not in bins:
            bins[bin_key] = []
        bins[bin_key].append(normalized[i])

    largest_bin = max(bins.values(), key=len)

    # Use COMPAS centroid for averaging
    pts = [Point(v.x, v.y, 0.0) for v in largest_bin]
    c = centroid_points(pts)
    dom = Vector(c[0], c[1], 0.0)

    if dom.length > 1e-9:
        dom.unitize()
        return dom
    return None

# --------------------------------------------------
# Subgraph classification
# --------------------------------------------------
def is_segment_near_support(subgraph, sup_pts, threshold):
    """Check if any node in subgraph is within threshold of a support."""
    for n in subgraph.nodes():
        pt = subgraph.node_attribute(n, "point")
        if pt:
            for sp in sup_pts:
                dist = ((pt.x - sp.x)**2 + (pt.y - sp.y)**2)**0.5
                if dist < threshold:
                    return True
    return False

# --------------------------------------------------
# Subgraph creation
# --------------------------------------------------
def create_subgraphs(graph, seg_x=None, seg_y=None, overlap=None):
    """
    Divide graph into spatial subgraphs using a grid of windows.

    Parameters
    ----------
    graph : NodeGraph
        Source graph.
    seg_x : int
        Number of segments in X direction (default from config).
    seg_y : int
        Number of segments in Y direction (default from config).
    overlap : float
        Overlap factor for window boundaries (default from config).

    Returns
    -------
    list of dict
        Subgraph data with keys: si, sj, index, graph, edges
    """
    if seg_x is None:
        seg_x = DEFAULT_SEG_X
    if seg_y is None:
        seg_y = DEFAULT_SEG_Y
    if overlap is None:
        overlap = DEFAULT_OVERLAP
    
    # Collect node positions
    node_pts = {}
    x_coords = []
    y_coords = []

    for n in graph.nodes():
        pt = graph.node_attribute(n, "point")
        if pt:
            node_pts[n] = pt
            x_coords.append(pt.x)
            y_coords.append(pt.y)

        if "original_point" in graph.node_attributes(n):
            extra = graph.node_attribute(n, "original_point")
            if extra:
                x_coords.append(extra.x)
                y_coords.append(extra.y)

    if not node_pts:
        return []

    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    x_range = x_max - x_min
    y_range = y_max - y_min
    cell_w = x_range / seg_x
    cell_h = y_range / seg_y

    subgraphs_list = []
    subgraphs = []
    for sj in range(seg_y):
        for si in range(seg_x):
            # Window bounds with overlap
            win_x_min = x_min + si * cell_w - cell_w * overlap
            win_x_max = x_min + (si + 1) * cell_w + cell_w * overlap
            win_y_min = y_min + sj * cell_h - cell_h * overlap
            win_y_max = y_min + (sj + 1) * cell_h + cell_h * overlap

            # Find nodes in window
            nodes_in_win = set()
            for n, pt in node_pts.items():
                if win_x_min <= pt.x <= win_x_max and win_y_min <= pt.y <= win_y_max:
                    nodes_in_win.add(n)

            # Find edges with both endpoints in window
            edges_in_win = []
            for u, v in graph.edges():
                if u in nodes_in_win and v in nodes_in_win:
                    edges_in_win.append((u, v))

            # Build subgraph
            sg = NodeGraph()
            node_attrs = ["x", "y", "z", "point", "group", "level", "is_support", "reached", "ntype", "support_id", "original_point"]
            edge_attrs = ["e_category", "etype", "group", "parallel_score", "nearest_support"]

            for n in nodes_in_win:
                attrs = {}
                for key in node_attrs:
                    val = graph.node_attribute(n, key)
                    if val is not None:
                        attrs[key] = val
                sg.add_node(n, **attrs)

            for u, v in edges_in_win:
                attrs = {}
                for key in edge_attrs:
                    val = graph.edge_attribute((u, v), key)
                    if val is not None:
                        attrs[key] = val
                sg.add_edge(u, v, **attrs)
                sg.edge_attribute((u, v), "window_id", sj * seg_x + si)
            
            subgraphs.append(sg)
            subgraphs_list.append({
                "si": si,
                "sj": sj,
                "index": sj * seg_x + si,
                "graph": sg,
                "edges": edges_in_win,
                "bounds": (win_x_min, win_x_max, win_y_min, win_y_max)
            })

    return subgraphs_list, subgraphs

# --------------------------------------------------
# Classify edges in subgraphs
# --------------------------------------------------
def classify_edges_in_subgraph(subgraph, sup_pts, parallel_tol=None, near_threshold=None):

    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL

    near_sup = is_segment_near_support(subgraph, sup_pts, near_threshold)

    to_identify_dirs = []
    to_identify_edges = []
    for edge in subgraph.edges():
        if "e_category" not in subgraph.edge_attributes(edge):
            continue
        
        if subgraph.edge_attribute(edge, "e_category") == "orthogonal":
            subgraph.edge_attribute(edge, "hierarchy", "primary")
        elif subgraph.edge_attribute(edge, "e_category") == "moved_diagonal":
            subgraph.edge_attribute(edge, "hierarchy", "special")
        elif subgraph.edge_attribute(edge, "e_category") == "default_diagonal":
            evec = subgraph.edge_vector(edge)
            edir_xy = Vector(evec.x, evec.y, 0.0)
            edir_xy.unitize()
            to_identify_dirs.append(edir_xy)
            to_identify_edges.append(edge)
    dom_dir = find_dominant_direction(to_identify_dirs)
    dom_dir = None

    if dom_dir:
        for edge, edir_xy in zip(to_identify_edges, to_identify_dirs):
            dot = abs(edir_xy.dot(dom_dir))
            if dot >= parallel_tol:
                new_etype = "primary"
            else:
                if subgraph.is_leaf_edge(edge):
                    new_etype = "secondary"
                else:
                    new_etype = "tertiary"
            subgraph.edge_attribute(edge, "hierarchy", new_etype)
    
    # Classify by support direction 
    else:
        # Check if edges are near support and change the tolerance
        if near_sup:
            classify_edges_by_support_direction(subgraph, to_identify_edges, parallel_tol= .9, support_points=sup_pts)
        else:
            classify_edges_by_support_direction(subgraph, to_identify_edges, parallel_tol= .7, support_points=sup_pts)
    
    # Leaf edge should be category hierarchy of tertiary
    for edge, edir_xy in zip(to_identify_edges, to_identify_dirs):
        if subgraph.edge_attribute(edge, "hierarchy") == "tertiary":
            if subgraph.is_leaf_edge(edge):
                subgraph.edge_attribute(edge, "hierarchy", "secondary")
                
            
def combine_graphs(graphs):
    """Combine a list of subgraphs into one graph."""
    new_ng = NodeGraph()

    for sg in graphs:
        for node in sg.nodes():
            new_ng.add_node(node, **sg.node_attributes(node))
        for u, v in sg.edges():
            new_ng.add_edge(u, v, **sg.edge_attributes((u, v)))
    return new_ng

# --------------------------------------------------
# Main API
# --------------------------------------------------
def classify_edges_by_Michael(graph, seg_x=None, seg_y=None, parallel_tol=None):
    """
    Classify edges from Michael's study.
    Hierarchy: 'primary', 'secondary', 'tertiary', 'special'
    """
    # 1. Categorize beams into "orthogonal", "default_diagonal", "moved_diagonal"
    categorize_edge_types(graph)
    # 2. Subdivide structure by "windows".
    _, subgraphs = create_subgraphs(graph, seg_x, seg_y)
    
    sup_pts = graph.get_support_points()

    # 3. Set orthogonal beams to "primary" and moved_diagonal beams to "special".
    # 4. Find domainant and subdomainant (both must be "default_diagonal") direction of each "windows".
    # 5. Filter beams without orthogonals by domainant direction, and set them to "primary"
    # 6. Set beams which are not "primary", "orthogonal" and are "leaf_edges" to "secondary"
    # 7. Set rest of the beams which are not "primary", "secondary", "special" to "tertiary"
    for sg in subgraphs:
        classify_edges_in_subgraph(sg, sup_pts, parallel_tol)

    return subgraphs