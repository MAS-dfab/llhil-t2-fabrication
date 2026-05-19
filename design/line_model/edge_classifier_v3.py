"""
Edge classification utilities for line-model graphs.

Usage in grasshopper:
    from edge_classifier_v3 import classify_edges, assign_edges_dimensions

    results = classify_edges(graph, parallel_tol=1e-3, digits=3, flip_sign=False, with_data=True)

    A_lines = results["A_lines"]
    primary_lines = results["primary"]

    assign_edges_dimensions(graph, scale="m")
"""

from compas.geometry import Vector

#---------------------------------------
# Graph utilities
#---------------------------------------
def get_default_directions_xy(graph):
    """
    Get default diagonal directions in XY plane by the first three nodes in each group.
    
    Parameters:
    -----------
    graph: NodeGraph
        Compas Graph extension.
    
    Returns:
    --------
    dict: {group_id: (dir1_xy, dir2_xy), ...}
    """
    # 1. Get apex, inset1, inset2 for each group
    groups = {}
    for node in graph.nodes():
        g = graph.node_attribute(node, "group")
        groups.setdefault(g, [])

        if len(groups[g]) < 3:
            groups[g].append(node)

    # 2. Get default diagonal directions in XY plane for each group
    directions = {}
    for g, (apex, inset1, inset2) in groups.items():
        vec1 = graph.node_point(inset1) - graph.node_point(apex)
        vec2 = graph.node_point(inset2) - graph.node_point(apex)

        dir1_xy = Vector(vec1.x, vec1.y, 0).unitized()
        dir2_xy = Vector(vec2.x, vec2.y, 0).unitized()

        directions[g] = (dir1_xy, dir2_xy)
    return directions

def get_cyclic_signs(graph, flip=False):
    """Get cyclic signs for each group by the order of the default diagonal directions."""
    directions = get_default_directions_xy(graph)

    signs = {}
    for g, (dir1, dir2) in directions.items():
        cross = dir1.cross(dir2)

        sign = +1 if cross.z > 0 else -1

        if flip:
            sign = -sign
        signs[g] = sign
    return signs

def group_edges(graph):
    """Group edges by their group id."""
    groups = {}
    for edge in graph.edges():
        # default to -1 if no group attribute found
        g = graph.get_edge_attribute(edge, "group", else_value=-1)
        groups.setdefault(g, []).append(edge)
    return groups


# --------------------------------
# 2D Collinearity check
# --------------------------------
def _line_signature_xy(line, digits=3):
    """Get a signature for a line based on its direction in XY plane."""
    # TODO: Handle vertical lines (infinite slope) if needed
    a, b = line
    m = (b.y - a.y) / (b.x - a.x)

    # y = mx + c => c = y - mx
    c = a.y - m * a.x  # Or c = b.y - m * b.x

    return (round(m, digits), round(c, digits))

def group_by_collinearity_xy(graph, edges=None, digits=3):
    """Group edges by collinearity in XY plane based on their line signatures."""
    if edges is None:
        edges = graph.edges()

    cg = {}
    for edge in edges:
        ln = graph.edge_line(edge)

        sig = _line_signature_xy(ln, digits=digits)
        cg.setdefault(sig, []).append(edge)
    return cg

def are_edges_collinear_xy(graph, edges, digits=3):
    """Check if a list of edges are collinear in XY plane."""
    sigs = set()
    for edge in edges:
        ln = graph.edge_line(edge)
        sig = _line_signature_xy(ln, digits=digits)
        sigs.add(sig)
    
    return len(sigs) == 1


# ---------------------------------------
# Edge classification utilities
# ---------------------------------------
def classify_directions(graph, edge_groups, parallel_tol=1e-3, with_data=False):
    """
    Classify edge directions based on default diagonal directions in XY plane.
    
    Parameters:
    -----------
    graph: NodeGraph
        Compas Graph extension.
    edge_groups: dict
        Dictionary of edge groups {group_id: [edges], ...}.
    parallel_tol: float
        Tolerance for checking parallelism to default directions.
    with_data: bool
        Whether to return classified line geometries.
    
    Returns:
    --------
    A_lines, B_lines, other_lines: list of Line
        Classified line geometries if with_data is True.
    """
    directions = get_default_directions_xy(graph)
    
    for g, edges in edge_groups.items():
        for edge in edges:
            dire = graph.edge_direction(edge)
            dire_xy = Vector(dire.x, dire.y, 0).unitized()
            dir1, dir2 = directions[g]

            dot1 = dire_xy.dot(dir1)
            dot2 = dire_xy.dot(dir2)

            if abs(dot1) >= 1 - parallel_tol:
                graph.edge_attribute(edge, "direction_id", "A")
            elif abs(dot2) >= 1 - parallel_tol:
                graph.edge_attribute(edge, "direction_id", "B")
            else:
                graph.edge_attribute(edge, "direction_id", "UNCLASSIFIED")
    
    if with_data:
        A_lines, B_lines, other_lines = [], [], []
        for e in graph.edges():
            ln = graph.get_edge_attribute(e, "shifted_line", else_value=graph.edge_line(e))

            if graph.edge_attribute(e, "direction_id") == "A":
                A_lines.append(ln)
            elif graph.edge_attribute(e, "direction_id") == "B":
                B_lines.append(ln)
            else:
                graph.edge_attribute(e, "direction_id", "UNCLASSIFIED")
                other_lines.append(ln)

        return A_lines, B_lines, other_lines
    return None, None, None

def dispatch_edges_by_lowest_node(graph, group, edges, digits=3, flip_sign=False):
    """
    Dispatch collinear edges into two groups based on a cyclic order of each group.
    
    Parameters:
    -----------
    graph: NodeGraph
        Compas graph extension.
    edges: list of (u, v)
        Edges to dispatch, should be collinear in XY plane.
    digits: int
        Number of digits to round for collinearity check.
    flip_sign: bool
        Whether to flip the cyclic sign for dispatching.

    Returns:
    --------
    dict: {-1: [edges], 1: [edges]}
        Dispatched edge groups based on cyclic sign.
    """
    if not are_edges_collinear_xy(graph, edges, digits=digits):
        raise ValueError("Edges are not collinear in XY plane, cannot dispatch by lowest node.")
    
    # 1. Find lowest node among all edges
    nodes = list(set(end for edge in edges for end in edge))
    pts = [graph.node_point(n) for n in nodes]
    
    lowest_node, lowest_pt = min(zip(nodes, pts), key=lambda x: x[1].z)

    # 2. Get support-to-lowest and edge midpoint-to-lowest vectors to compute cross product
    c_sign = get_cyclic_signs(graph, flip=flip_sign)[group]

    sup_node = next(graph.nodes_where({"support_id": group, "reached": True}), None)

    if sup_node is None:
        raise ValueError(f"No support node found for group {group}")

    vec = lowest_pt - graph.node_point(sup_node)
    vec_xy = Vector(vec.x, vec.y, 0).unitized()

    sign_groups = {-1: [], +1: []}
    for edge in edges:
        mid_pt = graph.edge_line(edge).midpoint

        edge_vec = mid_pt - lowest_pt
        edge_vec_xy = Vector(edge_vec.x, edge_vec.y, 0).unitized()

        cross = vec_xy.cross(edge_vec_xy)
        # 3. Get sign based on cross product direction and cyclic sign
        if cross.z > 0:
            sign_groups[c_sign].append(edge)
        else:
            sign_groups[-c_sign].append(edge)
    return sign_groups

def edges_by_hierarchy(graph, hierarchy):
    """Get edges by hierarchy."""
    return list(graph.edges_where({"hierarchy": hierarchy}))

def lines_by_hierarchy(graph, hierarchy):
    """Get lines by hierarchy."""
    edges = edges_by_hierarchy(graph, hierarchy)
    lines = [graph.get_edge_attribute(e, "shifted_line", else_value=graph.edge_line(e)) for e in edges]
    return lines


# ---------------------------------------
# Main API
# ---------------------------------------
def classify_edges(graph, parallel_tol=1e-3, digits=3, flip_sign=False, with_data=False):
    """
    Classify edges into hierarchy with the cyclic sign of each group.
    
    Parameters:
    -----------
    graph: NodeGraph
        Compas Graph extension.
    parallel_tol: float
        Tolerance for checking parallelism to dominant directions.
    digits: int
        Number of digits to round for collinearity check.
    flip_sign: bool
        Whether to flip the cyclic sign for dispatching.
    with_data: bool
        Whether to return classified line geometries.
    
    Returns:
    --------
    dict with keys(
        "A_lines", "B_lines", "other_lines", 
        "main_primary_lines", "primary_lines", "secondary_lines", "tertiary_lines", "shoe_lines"
    ), optional line geometries.
    """
    e_groups = group_edges(graph)
    A_lines, B_lines, other_lines = classify_directions(graph, e_groups, parallel_tol, with_data)

    for g, edges in e_groups.items():
        # 1. Exclude shoe edges
        ex_edges = [e for e in edges if graph.edge_attribute(e, "hierarchy") != "shoe"]
        
        # 2. Get collinear groups
        cg = group_by_collinearity_xy(graph, edges=ex_edges, digits=digits)
        
        len_set = list(set(len(v) for v in cg.values()))
        len_set.sort(reverse=True)
    
        for edges in cg.values():
            if len(edges) == len_set[0]:  # Biggest groups
                if graph.edge_attribute(edges[0], "direction_id") == "A":
                    for e in edges:
                        graph.edge_attribute(e, "hierarchy", "main_primary")

                elif graph.edge_attribute(edges[0], "direction_id") == "B":
                    for e in edges:
                        graph.edge_attribute(e, "hierarchy", "primary")
                else:
                    raise ValueError
            
            elif len(edges) == len_set[1]:  # Second biggest groups
                sign_group = dispatch_edges_by_lowest_node(
                    graph,
                    g,
                    edges,
                    digits=digits,
                    flip_sign=flip_sign
                )
                
                for sign, edges in sign_group.items():
                    if sign == -1:
                        for e in edges:
                            graph.edge_attribute(e, "hierarchy", "secondary")
                    elif sign == +1:
                        for e in edges:
                            graph.edge_attribute(e, "hierarchy", "tertiary")
                    else:
                        raise ValueError
                    
            else:  # Small groups
                for e in edges:
                    graph.edge_attribute(e, "hierarchy", "tertiary")

    if with_data:
        return {
            "A_lines": A_lines,
            "B_lines": B_lines,
            "other_lines": other_lines,
            "main_primary_lines": lines_by_hierarchy(graph, "main_primary"),
            "primary_lines": lines_by_hierarchy(graph, "primary"),
            "secondary_lines": lines_by_hierarchy(graph, "secondary"),
            "tertiary_lines": lines_by_hierarchy(graph, "tertiary"),
            "shoe_lines": lines_by_hierarchy(graph, "shoe"),
        }
    return {}


# -------------------------------------
# Edge dimension assignment
# -------------------------------------
def assign_edges_dimensions(graph, scale="m", exceptions=None):
    """
    Assign width and height attributes to edges based on their hierarchy and level.
    Widths are defined by hierarchies, and height are defined by levels.

    Parameters:
    -----------
    graph : NodeGraph
        Compas Graph extension.
    scale : str, optional
        The scale to use for dimensions ("m", "cm", "mm"). Default is "m".
    exceptions : dict, optional
        {(width, height): [edge1, edge2, ...], ...} for specific edges.
    """
    widths = [0.10, 0.10, 0.10]  # hierarchy
    heights = [0.10, 0.12, 0.14]  # level

    if scale == "mm":
        factor = 1000
    elif scale == "cm":
        factor = 100
    elif scale == "m":
        factor = 1
    else:
        raise ValueError("Unsupported scale, use 'm', 'cm' or 'mm'.")
    
    widths = [w * factor for w in widths]
    heights = [h * factor for h in heights]

    for edge in graph.edges():
        hie = graph.edge_attribute(edge, 'hierarchy')
        if hie in ('main_primary', 'primary'):
            hie_key = 2
        elif hie == 'secondary':
            hie_key = 1
        elif hie == 'tertiary':
            hie_key = 0
        else:
            hie_key = 0  # temp.
            
        lvl = graph.edge_attribute(edge, 'level')

        graph.edge_attribute(edge, 'width', widths[hie_key])
        graph.edge_attribute(edge, 'height', heights[lvl])
    
    if exceptions:
        for (w, h), edges in exceptions.items():
            for edge in edges:
                graph.edge_attribute(edge, 'width', w)
                graph.edge_attribute(edge, 'height', h)
    return