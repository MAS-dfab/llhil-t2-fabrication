"""
Tree structure builder for hierarchical grid-based branching systems.

Usage in Grasshopper:
    from tree_builder import build_tree_graph
    
    ng = build_tree_graph(
        boundary=boundary,
        supports=supports,
        roof_brep=roof_brep
    )
    
    a = ng.edge_lines_by_group(0)
    b = ng.edge_lines_by_group(1)
    c = ng
    d = ng.node_points()
"""

from nodegraph import NodeGraph
from config import (
    DEFAULT_DIV_X, DEFAULT_DIV_Y, DEFAULT_NUM_LEVELS,
    DEFAULT_INSET_EDGE, DEFAULT_INSET_INTERIOR, DEFAULT_REACH_TOL
)
from compas_rhino.conversions import curve_to_compas_polyline, point_to_compas
from compas.geometry import Point, Line, Vector, Frame
import Rhino.Geometry as rg  # type: ignore


# --------------------------------------------------
# Geometry helpers
# --------------------------------------------------
def get_plane_and_size(boundary):
    """Extract frame and dimensions from a rectangular boundary curve."""
    polyline = curve_to_compas_polyline(boundary)
    pts = polyline.points

    p0, p1, p3 = pts[0], pts[1], pts[3]

    xaxis = Vector.from_start_end(p0, p1)
    yaxis = Vector.from_start_end(p0, p3)

    lx = xaxis.length
    ly = yaxis.length

    xaxis.unitize()
    yaxis.unitize()

    return Frame(p0, xaxis, yaxis), lx, ly


def point_at(frame, u, v, w=0.0):
    """Compute a point on a frame given local UVW coordinates."""
    return Point(*frame.to_world_coordinates([u, v, w]))


def project_to_brep(pt, brep):
    """Project a point vertically (Z) onto a Rhino brep."""
    if brep is None:
        return pt

    def _first_valid_hit(hits):
        if not hits:
            return None
        for hit in hits:
            if hit is not None:
                return hit
        return None

    origin = rg.Point3d(pt.x, pt.y, pt.z)
    ray = rg.Ray3d(origin, rg.Vector3d.ZAxis)
    hits = rg.Intersect.Intersection.RayShoot(ray, [brep], 1)
    hit = _first_valid_hit(hits)
    if hit is not None:
        return Point(hit.X, hit.Y, hit.Z)
    # Try downward if upward missed
    ray = rg.Ray3d(origin, -rg.Vector3d.ZAxis)
    hits = rg.Intersect.Intersection.RayShoot(ray, [brep], 1)
    hit = _first_valid_hit(hits)
    if hit is not None:
        return Point(hit.X, hit.Y, hit.Z)
    return pt


def inset_corner(corner, center, inset_dist):
    """Move a corner point toward the center by inset distance in XY plane."""
    diag = Vector.from_start_end(corner, center)
    diag_xy = Vector(diag.x, diag.y, 0.0)
    diag_xy.unitize()
    return corner.translated(diag_xy * inset_dist)


def is_boundary_vertex(vi, vj, div_x, div_y):
    """Check if vertex indices are on the grid boundary."""
    return vi == 0 or vi == div_x or vj == 0 or vj == div_y


# --------------------------------------------------
# Grid generation
# --------------------------------------------------
def create_vertex_grid(plane, div_x, div_y, lx, ly):
    """Create a 2D grid of points on a plane."""
    dx = lx / float(div_x)
    dy = ly / float(div_y)
    
    grid = [[None for _ in range(div_x + 1)] for _ in range(div_y + 1)]
    for j in range(div_y + 1):
        for i in range(div_x + 1):
            grid[j][i] = point_at(plane, i * dx, j * dy, 0.0)
    return grid


def determine_cell_group(i, j, div_x, div_y):
    """Determine group_id and support_id for a cell based on its position."""
    mid_i = div_x // 2
    mid_j = div_y // 2
    
    if i < mid_i:
        group_id = 0
        sup_id = 0 if j < mid_j else 1
    else:
        group_id = 1
        sup_id = 3 if j < mid_j else 2
    
    return group_id, sup_id


# --------------------------------------------------
# Level builders
# --------------------------------------------------
def build_level_zero(vertex_grid, sup_pts, z_steps, roof_brep, config):
    """Build the base level cells with inset corners and apex points."""
    div_x = config["div_x"]
    div_y = config["div_y"]
    inset_edge = config["inset_edge"]
    inset_interior = config["inset_interior"]
    reach_tol = config["reach_tol"]
    
    records = []
    relations = []
    cell_grid = [[None for _ in range(div_x)] for _ in range(div_y)]
    
    for j in range(div_y):
        for i in range(div_x):
            corners = [
                vertex_grid[j][i],
                vertex_grid[j][i + 1],
                vertex_grid[j + 1][i + 1],
                vertex_grid[j + 1][i]
            ]
            corner_indices = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            center = (corners[0] + corners[1] + corners[2] + corners[3]) / 4

            # Compute inset corners with projection
            inset_corners = []
            for k, corner in enumerate(corners):
                vi, vj = corner_indices[k]
                on_boundary = is_boundary_vertex(vi, vj, div_x, div_y)
                inset_dist = inset_edge if on_boundary else inset_interior
                inset_pt = inset_corner(corner, center, inset_dist)
                inset_pt = project_to_brep(inset_pt, roof_brep)
                inset_corners.append(inset_pt)

            group_id, sup_id = determine_cell_group(i, j, div_x, div_y)
            sup = sup_pts[sup_id]

            # Compute apex with group-based height factor
            t = z_steps[0][1] if group_id == 0 else z_steps[1][1]# * 1.5
            apex_z = center.z + t * (sup.z - center.z)
            apex = Point(center.x, center.y, apex_z)

            reached = apex.distance_to_point(sup) <= reach_tol
            if reached:
                apex = sup

            rec = {
                "apex": apex,
                "group": group_id,
                "support": sup,
                "support_id": sup_id,
                "reached": reached,
                "original_corners": corners,
                "inset_corners": inset_corners,
                "children": [],
                "level": 0
            }

            cell_grid[j][i] = rec
            records.append(rec)

            edge_level = 0
            for ic in inset_corners:
                relations.append((apex, ic, group_id, "apex_inset", edge_level))
    
    return cell_grid, records, relations


def build_higher_levels(cell_grid, sup_pts, z_steps, num_levels, config):
    """Build hierarchical parent levels by merging 2x2 child cells."""
    records = []
    relations = []
    reach_tol = config["reach_tol"]
    
    for level in range(1, num_levels):
        rows = len(cell_grid)
        cols = len(cell_grid[0])
        new_grid = [[None for _ in range(cols - 1)] for _ in range(rows - 1)]

        for j in range(rows - 1):
            for i in range(cols - 1):
                children = [
                    cell_grid[j][i],
                    cell_grid[j][i + 1],
                    cell_grid[j + 1][i + 1],
                    cell_grid[j + 1][i]
                ]

                # Skip if any child is missing or groups don't match
                if not all(children):
                    continue
                if len(set(ch["group"] for ch in children)) != 1:
                    continue

                child_pts = [ch["apex"] for ch in children]
                px = sum(p.x for p in child_pts) / 4.0
                py = sum(p.y for p in child_pts) / 4.0
                avg_z = sum(p.z for p in child_pts) / 4.0

                A = children[0]
                sup = A["support"]
                group_id = A["group"]
                t_base = z_steps[0] if group_id == 0 else z_steps[1]
                t = t_base[level + 1]
                parent_z = avg_z + t * (sup.z - avg_z)
                parent = Point(px, py, parent_z)

                reached = parent.distance_to_point(sup) <= reach_tol
                if reached:
                    parent = sup

                rec = {
                    "apex": parent,
                    "group": A["group"],
                    "support": sup,
                    "support_id": A["support_id"],
                    "reached": reached,
                    "original_corners": [],
                    "inset_corners": [],
                    "children": child_pts,
                    "level": level
                }

                new_grid[j][i] = rec
                records.append(rec)

                for ch in child_pts:
                    edge_level = level + 1
                    relations.append((parent, ch, A["group"], "parent_child", edge_level))

        cell_grid = new_grid
    
    return records, relations


def finalize_top_level(records, sup_pts, num_levels, reach_tol=0.01, debug=False):
    """Snap top-level apexes to nearby support points. Uses NEAREST support, not inherited."""
    snapped_count = 0
    top_level_count = 0
    
    for rec in records:
        if rec["level"] == num_levels - 1:
            top_level_count += 1
            apex = rec["apex"]
            
            # Find NEAREST support (ignore inherited support_id)
            min_dist = float('inf')
            nearest_idx = -1
            nearest_sp = None
            
            for i, sp in enumerate(sup_pts):
                dist = apex.distance_to_point(sp)
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
                    nearest_sp = sp
            
            # Snap if within tolerance
            if min_dist <= reach_tol:
                rec["reached"] = True
                rec["support"] = nearest_sp
                rec["support_id"] = nearest_idx
                rec["apex"] = nearest_sp
                snapped_count += 1
                if debug:
                    print("Snapped apex to support {}: dist={:.4f}".format(nearest_idx, min_dist))
            elif debug:
                print("Top-level apex NOT snapped: dist to nearest support {} = {:.4f} (tol={})".format(
                    nearest_idx, min_dist, reach_tol))
    
    if debug:
        print("finalize_top_level: {} top-level records, {} snapped to supports".format(
            top_level_count, snapped_count))


# --------------------------------------------------
# Graph construction
# --------------------------------------------------
def build_graph_from_records(records, relations):
    """Construct a NodeGraph from cell records and edge relations."""
    ng = NodeGraph()
    
    for rec in records:
        # Supports are fixed, other apex nodes are z_free
        apex_mobility = "fixed" if rec["reached"] else "z_free"
        apex_node = ng.get_or_add_point_node(
            rec["apex"],
            ntype="apex",
            level=rec["level"],
            group=rec["group"],
            support_id=rec["support_id"],
            reached=rec["reached"],
            children=rec["children"],
            mobility=apex_mobility
        )

        # Add inset corner nodes (top level insets are fixed)
        inset_nodes = []
        for ic, oc in zip(rec["inset_corners"], rec["original_corners"]):
            ic_node = ng.get_or_add_point_node(ic, ntype="inset", level=0, mobility="fixed", original_point=oc)
            inset_nodes.append(ic_node)

        # Track child nodes
        child_nodes = []
        for ch in rec["children"]:
            ch_node = ng.get_or_add_point_node(ch)
            child_nodes.append(ch_node)
        ng.node_attribute(apex_node, "children", child_nodes)

    # Add edges from relations
    for p1, p2, group_id, etype, edge_level in relations:
        u = ng.get_or_add_point_node(p1)
        v = ng.get_or_add_point_node(p2)
        ng.add_graph_edge(u, v, group=group_id, etype=etype, level=edge_level)
    
    return ng


# --------------------------------------------------
# Main API
# --------------------------------------------------
def build_tree_graph(
    boundary,
    supports,
    roof_brep=None,
    div_x=None,
    div_y=None,
    num_levels=None,
    z_steps=None,
    inset_edge=None,
    inset_interior=None,
    reach_tol=None,
    debug=False
):
    """
    Build a hierarchical tree graph from a boundary, supports, and optional roof brep.
    
    Parameters
    ----------
    boundary : Rhino.Geometry.Curve
        Rectangular boundary curve defining the grid extent.
    supports : list of Rhino.Geometry.Point3d
        Four support points (corners of the tree structure).
    roof_brep : Rhino.Geometry.Brep, optional
        Surface to project base inset corners onto.
    div_x : int
        Number of divisions in X direction (default from config).
    div_y : int
        Number of divisions in Y direction (default from config).
    num_levels : int
        Number of hierarchical levels (default from config).
    z_steps : list of float, optional
        Height interpolation factors per level. Auto-generated if None.
    inset_edge : float
        Inset distance for boundary corners (default from config).
    inset_interior : float
        Inset distance for interior corners (default from config).
    reach_tol : float
        Tolerance for snapping to support points (default from config).
    
    Returns
    -------
    NodeGraph
        The constructed graph with all nodes and edges.
    """
    # Apply defaults from config
    if div_x is None:
        div_x = DEFAULT_DIV_X
    if div_y is None:
        div_y = DEFAULT_DIV_Y
    if num_levels is None:
        num_levels = DEFAULT_NUM_LEVELS
    if inset_edge is None:
        inset_edge = DEFAULT_INSET_EDGE
    if inset_interior is None:
        inset_interior = DEFAULT_INSET_INTERIOR
    if reach_tol is None:
        reach_tol = DEFAULT_REACH_TOL
    
    # Build config
    config = {
        "div_x": div_x,
        "div_y": div_y,
        "num_levels": num_levels,
        "inset_edge": inset_edge,
        "inset_interior": inset_interior,
        "reach_tol": reach_tol,
    }
    
    # Default z_steps
    if debug:
        print(z_steps)
    # if not z_steps or len(z_steps) < num_levels + 1:
    #     z_steps = [i / float(num_levels) for i in range(num_levels + 1)]
    
    # Setup
    plane, lx, ly = get_plane_and_size(boundary)
    sup_pts = [point_to_compas(s) for s in supports]
    vertex_grid = create_vertex_grid(plane, div_x, div_y, lx, ly)
    if debug:
        print(z_steps)
    # Build levels
    cell_grid, records_l0, relations_l0 = build_level_zero(
        vertex_grid, sup_pts, z_steps, roof_brep, config
    )

    records_upper, relations_upper = build_higher_levels(
        cell_grid, sup_pts, z_steps, num_levels, config
    )
    
    # Combine records and relations
    all_records = records_l0 + records_upper
    all_relations = relations_l0 + relations_upper
    
    if debug:
        print("DEBUG: Total records: {}, Level 0: {}, Upper: {}".format(
            len(all_records), len(records_l0), len(records_upper)))
        print("DEBUG: Support points: {}".format(len(sup_pts)))
        for i, sp in enumerate(sup_pts):
            print("  Support {}: ({:.2f}, {:.2f}, {:.2f})".format(i, sp.x, sp.y, sp.z))
    
    # Finalize top level
    finalize_top_level(all_records, sup_pts, num_levels, reach_tol, debug=debug)
    
    # Build graph
    ng = build_graph_from_records(all_records, all_relations)
    
    if debug:
        support_nodes = ng.get_support_nodes()
        print("DEBUG: Graph built with {} support nodes: {}".format(len(support_nodes), support_nodes))
    
    return ng


def get_group_lines(ng, group_id):
    """Get all edge lines for a specific group."""
    return ng.edge_lines_by_group(group_id)


def get_all_points(ng):
    """Get all node points from the graph."""
    return ng.node_points()
