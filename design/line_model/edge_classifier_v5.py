"""
Edge classifier v5.

Differences from v3:
- Keep v3 primary detection, then split primaries into:
  - `primary_orthogonal`
  - `primary_diagonal_<L0>_<L1>` (level-pair buckets)
- Reclassify every non-primary edge by node-level logic:
  - `secondary`: only (0,0) leaf edges
  - `tertiary`: (0,1)
  - `quaternary`: (1,2)
  - `ignored`: everything else
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
def get_default_directions(graph):
    """Find two original grid diagonal directions from un-inset points."""
    ori_pts = []
    for node in graph.nodes():
        ori_pt = graph.node_attribute(node, "original_point")
        if ori_pt:
            ori_pts.append(ori_pt)

    p0, p1, p2, p3 = ori_pts[:4]
    default_dir1 = Vector.from_start_end(p0, p2).unitized()
    default_dir2 = Vector.from_start_end(p1, p3).unitized()
    return default_dir1, default_dir2

def categorize_edge_types(graph, tol=1e-3):
    """Categorize beams into "orthogonal", "default_diagonal", 'moved_diagonal'"""
    # 1. Find two original grid diagonal directions from un-inset points
    default_dir1, default_dir2 = get_default_directions(graph)

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
        dot = abs(edir_xy.dot(Vector(0, 1, 0)))
        if (1 - tol) <= dot <= (1 + tol) or (0 - tol) <= dot <= (0 + tol):
            graph.edge_attribute(edge, "e_category", "orthogonal")
            continue

        # 4. The rest are moved diagonal edges
        graph.edge_attribute(edge, "e_category", "moved_diagonal")
    return default_dir1, default_dir2


def _level_pair(subgraph, edge):
    u, v = edge
    lu = subgraph.node_attribute(u, "level")
    lv = subgraph.node_attribute(v, "level")
    return tuple(sorted((lu, lv)))


def _reclassify_non_primary_by_level(subgraph):
    """Apply v4 hierarchy mapping for non-primary edges."""
    for edge in subgraph.edges():
        hie = subgraph.edge_attribute(edge, "hierarchy")
        if hie == "primary":
            continue

        pair = _level_pair(subgraph, edge)
        u, v = edge
        lu = subgraph.node_attribute(u, "level")
        lv = subgraph.node_attribute(v, "level")

        # Secondary = ONLY (0,0) edges that are leaf edges.
        if pair == (0, 0) and subgraph.is_leaf_edge(edge):
            new_h = "secondary"
        elif pair == (0, 1):
            new_h = "tertiary"
        elif pair == (1, 2):
            new_h = "quaternary"
        else:
            new_h = "ignored"

        subgraph.edge_attribute(edge, "hierarchy", new_h)


def _reclassify_primary_by_category_and_level(subgraph):
    """Split primary into orthogonal vs diagonal(level-pair) buckets."""
    for edge in subgraph.edges():
        hie = subgraph.edge_attribute(edge, "hierarchy")
        if hie != "primary":
            continue

        cat = subgraph.edge_attribute(edge, "e_category")
        pair = _level_pair(subgraph, edge)

        if cat == "orthogonal":
            new_h = "primary_orthogonal"
        else:
            # default_diagonal + moved_diagonal are grouped as diagonal by level
            new_h = "primary_diagonal_{}_{}".format(pair[0], pair[1])

        subgraph.edge_attribute(edge, "hierarchy", new_h)


def classify_edges_by_support_direction(graph, edges=None, parallel_tol=None, debug=False, support_points=None):
    """
    Classify subgraph edges by comparing edge direction in XY
    to the direction from each edge midpoint to the nearest support.
    """
    if edges is None:
        edges = graph.edges()

    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL

    if support_points:
        sup_pts = support_points
        if debug:
            print("DEBUG: Using {} provided support points".format(len(sup_pts)))
    else:
        sup_pts = graph.get_support_points()
        if debug:
            print("DEBUG: Found {} support points from graph".format(len(sup_pts)))

    if not sup_pts:
        if debug:
            print("WARNING: No support points available!")
        return [], [], []

    primary_lines = []
    tertiary_lines = []
    data = []
    dot_values = []

    for u, v in edges:
        existing = graph.edge_attribute((u, v), "main_secondary")
        if existing == "double":
            pu = graph.node_attribute(u, "point")
            pv = graph.node_attribute(v, "point")
            if pu and pv:
                primary_lines.append(Line(pu, pv))
                data.append({"edge": (u, v), "support_idx": None, "dot": None, "type": "double"})
            continue

        pu = graph.node_attribute(u, "point")
        pv = graph.node_attribute(v, "point")
        if pu is None or pv is None:
            continue

        edge_vec = Vector.from_start_end(pu, pv)
        if edge_vec.length < 1e-9:
            continue
        edge_vec.unitize()

        mid = Line(pu, pv).midpoint

        best_support_idx = None
        best_d2 = float("inf")
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

        mid_xy = Point(mid.x, mid.y, 0.0)
        sup_vec = Vector.from_start_end(mid_xy, best_support_pt)
        if sup_vec.length < 1e-9:
            line = Line(pu, pv)
            primary_lines.append(line)
            etype = "primary"
            data.append({"edge": (u, v), "support_idx": best_support_idx, "dot": 1.0, "type": etype})
            graph.edge_attribute((u, v), "hierarchy", etype)
            graph.edge_attribute((u, v), "parallel_score", 1.0)
            graph.edge_attribute((u, v), "nearest_support", best_support_idx)
            continue

        sup_vec.unitize()
        edge_vec_xy = Vector(edge_vec.x, edge_vec.y, 0.0)
        if edge_vec_xy.length < 1e-9:
            line = Line(pu, pv)
            tertiary_lines.append(line)
            etype = "tertiary"
            data.append({"edge": (u, v), "support_idx": best_support_idx, "dot": 0.0, "type": etype})
            graph.edge_attribute((u, v), "hierarchy", etype)
            graph.edge_attribute((u, v), "parallel_score", 0.0)
            graph.edge_attribute((u, v), "nearest_support", best_support_idx)
            continue

        edge_vec_xy.unitize()
        dot = abs(edge_vec_xy.dot(sup_vec))
        dot_values.append(dot)
        line = Line(pu, pv)
        if dot >= parallel_tol:
            primary_lines.append(line)
            etype = "primary"
        else:
            tertiary_lines.append(line)
            etype = "tertiary"

        data.append({"edge": (u, v), "support_idx": best_support_idx, "dot": dot, "type": etype})
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


def _normalize_vector_direction(v):
    if v.x < -1e-9 or (abs(v.x) < 1e-9 and v.y < 0):
        return Vector(-v.x, -v.y, 0.0)
    return Vector(v.x, v.y, 0.0)


def find_dominant_direction(vecs, angle_tol=None):
    if not vecs:
        return None
    if angle_tol is None:
        angle_tol = DEFAULT_ANGLE_TOL

    normalized = [_normalize_vector_direction(v) for v in vecs]
    angles = [math.atan2(v.y, v.x) for v in normalized]

    bins = {}
    for i, ang in enumerate(angles):
        bin_key = round(ang / angle_tol)
        bins.setdefault(bin_key, []).append(normalized[i])

    largest_bin = max(bins.values(), key=len)
    pts = [Point(v.x, v.y, 0.0) for v in largest_bin]
    c = centroid_points(pts)
    dom = Vector(c[0], c[1], 0.0)
    if dom.length > 1e-9:
        dom.unitize()
        return dom
    return None


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
            
     
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD

    for edge in subgraph.edges():
        subgraph.edge_attribute(edge, "hierarchy_base", subgraph.edge_attribute(edge, "hierarchy"))

    _reclassify_non_primary_by_level(subgraph)
    _reclassify_primary_by_category_and_level(subgraph)


def classify_edges(graph, seg_x=None, seg_y=None, parallel_tol=None):
    """
    v4 graph classification entrypoint.

    Returns
    -------
    list[NodeGraph]
        Subgraphs with edge hierarchy in:
        primary_orthogonal / primary_diagonal_<L0>_<L1> /
        secondary / tertiary / quaternary / ignored
    """
    # Keep edge-category setup from v3.
    _, _ = categorize_edge_types(graph)
    _, subgraphs = create_subgraphs(graph, seg_x, seg_y)
    sup_pts = graph.get_support_points()

    for sg in subgraphs:
        classify_edges_in_subgraph(sg, sup_pts, parallel_tol=parallel_tol)

    return subgraphs

# --------------------------------------------------
# Subgraph classification
# --------------------------------------------------
def is_segment_near_support(subgraph, sup_pts, threshold):
    """Check if any node in subgraph is within threshold of a support."""
    if isinstance(subgraph, dict):
        sg = subgraph["graph"]
    else:
        sg = subgraph

    for n in sg.nodes():
        pt = sg.node_attribute(n, "point")
        if pt:
            for sp in sup_pts:
                dist = ((pt.x - sp.x)**2 + (pt.y - sp.y)**2)**0.5
                if dist < threshold:
                    return True
    return False


def classify_subgraph_edges(subgraph, sup_pts, near_threshold=None, parallel_tol=None):
    """
    Classify edges within a subgraph.
    
    Near support: keeps original classification.
    Far from support: uses dominant direction of primary edges.

    Parameters
    ----------
    subgraph : dict
        Subgraph data from create_subgraphs.
    sup_pts : list of Point
        Support point locations.
    near_threshold : float
        Distance threshold for "near support" (default from config).
    parallel_tol : float
        Dot product threshold (default from config).

    Returns
    -------
    tuple
        (primary_lines, secondary_lines, double_lines, near_support, subgraph)
    """
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL
    
    sg = subgraph["graph"]
    near_sup = is_segment_near_support(subgraph, sup_pts, near_threshold)

    # Collect edge data
    all_edges = []
    primary_vecs = []

    for u, v in sg.edges():
        pu = sg.node_attribute(u, "point")
        pv = sg.node_attribute(v, "point")
        if not pu or not pv:
            continue

        evec = Vector(pv.x - pu.x, pv.y - pu.y, 0.0)
        if evec.length < 1e-9:
            continue
        evec.unitize()

        etype = sg.edge_attribute((u, v), "main_secondary")
        if etype is None:
            etype = "secondary"  # Default unclassified to secondary
        line = Line(pu, pv)

        all_edges.append({"vec": evec, "etype": etype, "line": line, "edge": (u, v)})

        if etype == "primary":
            primary_vecs.append(evec)

    primary_lines = []
    secondary_lines = []
    double_lines = []

    if near_sup:
        # Near support: keep original classification
        for ed in all_edges:
            if ed["etype"] == "double":
                double_lines.append(ed["line"])
            elif ed["etype"] == "primary":
                primary_lines.append(ed["line"])
            else:
                secondary_lines.append(ed["line"])
    else:
        # Far from support: use dominant direction
        dom_dir = find_dominant_direction(primary_vecs)

        for ed in all_edges:
            # Preserve "double" edges - don't reclassify
            if ed["etype"] == "double":
                double_lines.append(ed["line"])
                continue
            
            new_etype = ed["etype"]  # default to original
            
            if dom_dir:
                dot = abs(ed["vec"].dot(dom_dir))
                if dot >= parallel_tol:
                    new_etype = "primary"
                else:
                    new_etype = "secondary"
            
            if new_etype == "primary":
                primary_lines.append(ed["line"])
            else:
                secondary_lines.append(ed["line"])
            
            # Update the subgraph edge attribute (for reference)
            sg.edge_attribute(ed["edge"], "main_secondary", new_etype)

    return primary_lines, secondary_lines, double_lines, near_sup, sg

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
# Subgraph creation
# --------------------------------------------------
def create_subgraphs(graph, seg_x=None, seg_y=None, overlap=None, debug=False):
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
    debug : bool
        Print debug information.

    Returns
    -------
    tuple
        (subgraph metadata list, list of subgraph graphs)
    """
    if debug:
        print("DEBUG: create_subgraphs seg_x={}, seg_y={}, overlap={}".format(seg_x, seg_y, overlap))
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
        return [], []

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
            node_attrs = ["x", "y", "z", "point", "group", "level", "is_support", "reached", "ntype", "support_id"]
            edge_attrs = ["e_category", "main_secondary", "etype", "group", "parallel_score", "nearest_support"]

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

def classify_single_segment(graph, segment_index, seg_x=None, seg_y=None, 
                            parallel_tol=None, near_threshold=None):
    """
    Classify edges for a single segment.

    Parameters
    ----------
    graph : NodeGraph
        Input graph with nodes having "point" and "reached" attributes.
    segment_index : int
        Index of the segment to classify.
    seg_x : int
        Number of X segments for subgraph division (default from config).
    seg_y : int
        Number of Y segments for subgraph division (default from config).
    parallel_tol : float
        Dot product threshold (default from config).
    near_threshold : float
        Distance threshold for "near support" (default from config).

    Returns
    -------
    dict
        {
            "primary_lines": list,
            "secondary_lines": list,
            "double_lines": list,
            "near_support": bool,
            "segment_index": int,
            "subgraph": Graph
        }
    """
    # Apply config defaults
    if seg_x is None:
        seg_x = DEFAULT_SEG_X
    if seg_y is None:
        seg_y = DEFAULT_SEG_Y
    if parallel_tol is None:
        parallel_tol = DEFAULT_PARALLEL_TOL
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD
    
    # Initial classification
    classify_edges_by_support_direction(graph, parallel_tol=parallel_tol)
    
    # Create subgraphs
    subgraphs, _ = create_subgraphs(graph, seg_x, seg_y, debug=True)
    
    if not subgraphs:
        return {
            "primary_lines": [],
            "secondary_lines": [],
            "double_lines": [],
            "near_support": False,
            "segment_index": int(segment_index),
            "subgraph": None
        }

    # Get support points from graph
    sup_pts = graph.get_support_points()
    
    # Get specific segment
    idx = int(segment_index) % len(subgraphs)
    sg_data = subgraphs[idx]
    
    primary, secondary, double, near_sup = classify_subgraph_edges(
        sg_data, sup_pts, near_threshold, parallel_tol
    )
    
    return {
        "primary_lines": primary,
        "secondary_lines": secondary,
        "double_lines": double,
        "near_support": near_sup,
        "segment_index": idx,
        "subgraph": sg_data["graph"]
    }
