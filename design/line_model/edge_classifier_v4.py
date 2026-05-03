"""
Edge classifier v4.

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

from config import DEFAULT_NEAR_THRESHOLD, DEFAULT_PARALLEL_TOL, DEFAULT_OVERLAP, DEFAULT_SEG_X, DEFAULT_SEG_Y, DEFAULT_ANGLE_TOL
from compas.geometry import Vector, Line, Point, centroid_points
from compas.geometry import distance_point_point_xy
from compas.datastructures import Graph
import math
import edge_classifier_v3 as v3  # for reuse of base classification logic

def categorize_edge_types(graph):
    """
    Placeholder for categorize_edge_types.
    """
    # Assume it sets e_category or something
    for u, v in graph.edges():
        # Simple categorization based on existing
        main_sec = graph.edge_attribute((u, v), "main_secondary")
        if main_sec == "primary":
            graph.edge_attribute((u, v), "e_category", "orthogonal")  # or diagonal, but placeholder
        else:
            graph.edge_attribute((u, v), "e_category", "other")
    return [], []  # placeholder


def build_category_index(graph, category_mode=2):
    """
    Build reusable node/edge category metadata for a graph.

    Parameters
    ----------
    graph : Graph or NodeGraph
        Input graph. Works with both plain COMPAS Graph and NodeGraph.
    category_mode : int
        0 = valency only, 1 = level only, 2 = valency + level.
    Returns
    -------
    dict
        {
            "nodes": {node: {...}},
            "edges": {(u, v): {...}}
        }
    """
    idx = {"nodes": {}, "edges": {}}

    for n in graph.nodes():
        deg = graph.degree(n)
        group = graph.node_attribute(n, "group")
        level = graph.node_attribute(n, "level")
        reached = bool(graph.node_attribute(n, "reached"))
        pt = graph.node_attribute(n, "point")

        degree_cat = "leaf" if deg == 1 else ("junction" if deg >= 3 else "chain")
        if level is None:
            level_cat = "L?"
        else:
            level_cat = "L{}".format(level)

        if category_mode == 0:
            category = (degree_cat,)
        elif category_mode == 1:
            category = (level_cat,)
        else:
            category = (degree_cat, level_cat)

        idx["nodes"][n] = {
            "degree": deg,
            "degree_cat": degree_cat,
            "group": group,
            "level": level,
            "level_cat": level_cat,
            "is_support": reached,
            "category": category,
        }

    for u, v in graph.edges():
        gu = idx["nodes"][u]["group"]
        gv = idx["nodes"][v]["group"]
        idx["edges"][(u, v)] = {
            "main_secondary": graph.edge_attribute((u, v), "main_secondary"),
            "inter_module": (gu is not None and gv is not None and gu != gv),
        }

    return idx


def is_segment_near_support(subgraph, sup_pts, threshold):
    """Check if any node in subgraph is within threshold of a support."""
    sg = subgraph["graph"]
    for n in sg.nodes():
        pt = sg.node_attribute(n, "point")
        if pt:
            for sp in sup_pts:
                dist = ((pt.x - sp.x)**2 + (pt.y - sp.y)**2)**0.5
                if dist < threshold:
                    return True
    return False


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
    extras = []

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
                extras.append(extra)

    if not node_pts:
        return []

    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    x_range = x_max - x_min
    y_range = y_max - y_min
    cell_w = x_range / seg_x
    cell_h = y_range / seg_y

    # NOTE: Place where we should categorize edges based on their position in height layers 
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
            sg = Graph()
            node_attrs = ["x", "y", "z", "point", "group", "level", "is_support", "reached", "ntype"]
            edge_attrs = ["main_secondary", "etype", "group", "parallel_score", "nearest_support"]

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

            subgraphs.append({
                "si": si,
                "sj": sj,
                "index": sj * seg_x + si,
                "graph": sg,
                "edges": edges_in_win,
                "bounds": (win_x_min, win_x_max, win_y_min, win_y_max)
            })

    return subgraphs, extras


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


def classify_edges_in_subgraph(subgraph, sup_pts, parallel_tol=None, near_threshold=None):
    """
    v4 subgraph classification.

    Step 1: run v3 classifier (for base detection).
    Step 2: override non-primary hierarchies by level logic.
    Step 3: split primary into orthogonal/diagonal(level).
    """
    if near_threshold is None:
        near_threshold = DEFAULT_NEAR_THRESHOLD

    v3.classify_edges_in_subgraph(subgraph, sup_pts, parallel_tol=parallel_tol, near_threshold=near_threshold)

    # Preserve raw v3 hierarchy for optional debugging.
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
    classify_edges_by_support_direction(graph, parallel_tol)
    
    # Create subgraphs
    subgraphs, window = create_subgraphs(graph, seg_x, seg_y, debug=True)
    
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
        "subgraph": sg_data["graph"],
        "window": window
    }
