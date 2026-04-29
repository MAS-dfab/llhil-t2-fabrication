"""
Tree structure builder for hierarchical grid-based branching systems.
Supports asymmetric configurations with 2 support points.

Usage in Grasshopper:
    from tree_builder_single import build_tree_graph
    
    ng = build_tree_graph(
        boundary=boundary,
        supports=supports,
        roof_brep=roof_brep
    )
    
    all_edges = ng.edge_lines()
    all_nodes = ng.node_points()
"""

from nodegraph_WN import NodeGraph
from config import (
    DEFAULT_DIV_X, DEFAULT_DIV_Y, DEFAULT_NUM_LEVELS,
    DEFAULT_INSET_EDGE, DEFAULT_INSET_INTERIOR, DEFAULT_REACH_TOL
)
from compas_rhino.conversions import curve_to_compas_polyline, point_to_compas
from compas.geometry import Point, Vector, Frame
import Rhino.Geometry as rg


# ==================================================
# Geometry helpers
# ==================================================

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
    """Compute a world point from local UVW coordinates on a frame."""
    return Point(*frame.to_world_coordinates([u, v, w]))


def project_to_brep(pt, brep):
    """Project a point vertically (along Z) onto a Rhino brep.
    
    Uses a vertical line + CurveBrep intersection to preserve
    the point's XY position. Only the Z coordinate is updated.
    Falls back to ClosestPoint if the vertical line misses.
    """
    if brep is None:
        return pt

    line_start = rg.Point3d(pt.x, pt.y, pt.z - 1000)
    line_end = rg.Point3d(pt.x, pt.y, pt.z + 1000)
    line_curve = rg.LineCurve(line_start, line_end)

    ok, overlap_curves, hit_points = rg.Intersect.Intersection.CurveBrep(
        line_curve, brep, 0.001
    )

    if ok and hit_points and len(hit_points) > 0:
        best_hit = None
        best_dist = float('inf')
        for hp in hit_points:
            dz = abs(hp.Z - pt.z)
            if dz < best_dist:
                best_dist = dz
                best_hit = hp
        if best_hit is not None:
            return Point(best_hit.X, best_hit.Y, best_hit.Z)

    # Fallback
    origin = rg.Point3d(pt.x, pt.y, pt.z)
    rc = brep.ClosestPoint(origin)
    if rc:
        return Point(rc.X, rc.Y, rc.Z)
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


def nearest_support(pt, sup_pts):
    """Find the nearest support point by XY distance.
    
    Returns (group_id, support_id, support_point).
    group_id == support_id (one group per support).
    """
    min_dist_sq = float('inf')
    best_idx = 0
    for i, sp in enumerate(sup_pts):
        dx = pt.x - sp.x
        dy = pt.y - sp.y
        d = dx * dx + dy * dy
        if d < min_dist_sq:
            min_dist_sq = d
            best_idx = i
    return best_idx, best_idx, sup_pts[best_idx]


# ==================================================
# Grid generation
# ==================================================

def compute_split_x(frame, sup_pts):
    """Compute the grid split position from support points.
    
    Projects all support points into the boundary's local X coordinate
    and returns the average. This positions the grid split line at
    the column axis, regardless of which side is extended.
    """
    local_xs = []
    for sp in sup_pts:
        # Project support point into frame's local coordinates
        vec = Vector.from_start_end(frame.point, sp)
        local_x = vec.dot(frame.xaxis)
        local_xs.append(local_x)
    return sum(local_xs) / len(local_xs)


def create_vertex_grid(frame, div_x, div_y, lx, ly, split_x):
    """Create a 2D grid of points on a frame.
    
    The grid is split at split_x in local X coordinates:
    - Left half: columns 0..div_x//2, spanning [0, split_x]
    - Right half: columns div_x//2..div_x, spanning [split_x, lx]
    Each half is uniformly subdivided.
    """
    dy = ly / float(div_y)
    half = div_x // 2

    # Left half
    x_left = [i * split_x / float(half) for i in range(half + 1)]

    # Right half (vertex at split_x is shared)
    right_count = div_x - half
    dx_right = (lx - split_x) / float(right_count)
    x_right = [split_x + i * dx_right for i in range(1, right_count + 1)]

    x_positions = x_left + x_right

    grid = [[None for _ in range(div_x + 1)] for _ in range(div_y + 1)]
    for j in range(div_y + 1):
        for i in range(div_x + 1):
            grid[j][i] = point_at(frame, x_positions[i], j * dy, 0.0)
    return grid


# ==================================================
# Level builders
# ==================================================

def build_level_zero(vertex_grid, sup_pts, z_steps, roof_brep, config):
    """Build level-0 cells: inset corners on the roof + apex nodes."""
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

            # Inset corners projected to roof brep
            inset_corners = []
            for k, corner in enumerate(corners):
                vi, vj = corner_indices[k]
                on_bnd = is_boundary_vertex(vi, vj, div_x, div_y)
                dist = inset_edge if on_bnd else inset_interior
                ic = inset_corner(corner, center, dist)
                ic = project_to_brep(ic, roof_brep)
                inset_corners.append(ic)

            # Find nearest support
            group_id, sup_id, sup = nearest_support(center, sup_pts)

            # Apex: XY at cell center, Z interpolated toward support
            t = z_steps[1]
            apex = Point(
                center.x,
                center.y,
                center.z + t * (sup.z - center.z)
            )

            reached = apex.distance_to_point(sup) <= reach_tol
            if reached:
                apex = sup

            rec = {
                "apex": apex,
                "group": group_id,
                "support": sup,
                "support_id": sup_id,
                "reached": reached,
                "inset_corners": inset_corners,
                "children": [],
                "level": 0
            }
            cell_grid[j][i] = rec
            records.append(rec)

            for ic in inset_corners:
                relations.append((apex, ic, group_id, "apex_inset"))

    # Post-process: align Y values of top/bottom boundary inset corners.
    # Left-side cells are wider, so their diagonal inset shifts Y more than
    # right-side cells. We use the right-side (narrower) cells' Y as reference.
    half = div_x // 2  # columns 0..half-1 are left, half..div_x-1 are right
    
    for j in [0, div_y - 1]:  # bottom row and top row only
        # Find reference Y from the rightmost cell's inset corners on the boundary edge
        ref_cell = cell_grid[j][div_x - 1]  # rightmost cell in this row
        if ref_cell is None:
            continue
        
        if j == 0:
            # Bottom edge: corners 0 and 1 (vj==0)
            ref_y_0 = ref_cell["inset_corners"][0].y  # corner (i, 0)
            ref_y_1 = ref_cell["inset_corners"][1].y  # corner (i+1, 0)
        else:
            # Top edge: corners 2 and 3 (vj==div_y)
            ref_y_2 = ref_cell["inset_corners"][2].y  # corner (i+1, div_y)
            ref_y_3 = ref_cell["inset_corners"][3].y  # corner (i, div_y)
        
        # Fix left-side cells' boundary inset corners
        for i in range(half):
            cell = cell_grid[j][i]
            if cell is None:
                continue
            
            if j == 0:
                # Bottom edge corners: index 0 and 1
                for k in [0, 1]:
                    old = cell["inset_corners"][k]
                    ref_y = ref_y_0 if k == 0 else ref_y_1
                    new_pt = Point(old.x, ref_y, old.z)
                    # Update in relations too
                    for idx, (p1, p2, gid, etype) in enumerate(relations):
                        if p2.distance_to_point(old) < 0.001:
                            relations[idx] = (p1, new_pt, gid, etype)
                    cell["inset_corners"][k] = new_pt
            else:
                # Top edge corners: index 2 and 3
                for k in [2, 3]:
                    old = cell["inset_corners"][k]
                    ref_y = ref_y_2 if k == 2 else ref_y_3
                    new_pt = Point(old.x, ref_y, old.z)
                    for idx, (p1, p2, gid, etype) in enumerate(relations):
                        if p2.distance_to_point(old) < 0.001:
                            relations[idx] = (p1, new_pt, gid, etype)
                    cell["inset_corners"][k] = new_pt

    return cell_grid, records, relations


def build_higher_levels(cell_grid, sup_pts, z_steps, num_levels, config,
                        frame=None, split_x=None):
    """Build hierarchical levels by merging 2x2 child cells.
    
    Cross-group merging is allowed: when children span two groups,
    the majority group is used (ties broken by nearest support).
    
    If frame and split_x are provided, apexes whose children straddle
    the column axis will have their X snapped to the support X.
    """
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
                if not all(children):
                    continue

                # Resolve group for cross-group merges
                child_groups = [ch["group"] for ch in children]
                unique = set(child_groups)

                if len(unique) == 1:
                    group_id = child_groups[0]
                    sup = children[0]["support"]
                    sup_id = children[0]["support_id"]
                else:
                    from collections import Counter
                    gc = Counter(child_groups)
                    top2 = gc.most_common(2)
                    if top2[0][1] > top2[1][1]:
                        group_id = top2[0][0]
                    else:
                        # Tie: use geometric center
                        child_pts_tmp = [ch["apex"] for ch in children]
                        cx = sum(p.x for p in child_pts_tmp) / 4.0
                        cy = sum(p.y for p in child_pts_tmp) / 4.0
                        group_id, _, _ = nearest_support(
                            Point(cx, cy, 0), sup_pts
                        )
                    sup = sup_pts[group_id]
                    sup_id = group_id

                # Parent apex: XY = children average, Z interpolated
                child_pts = [ch["apex"] for ch in children]
                px = sum(p.x for p in child_pts) / 4.0
                py = sum(p.y for p in child_pts) / 4.0
                avg_z = sum(p.z for p in child_pts) / 4.0

                t = z_steps[level + 1]
                parent_z = avg_z + t * (sup.z - avg_z)

                # Snap X to support if children straddle the column axis.
                # This means some children are on the left of split_x and
                # some on the right — the parent should be on the axis.
                if frame is not None and split_x is not None:
                    child_local_xs = []
                    for cp in child_pts:
                        v = Vector.from_start_end(frame.point, cp)
                        child_local_xs.append(v.dot(frame.xaxis))
                    has_left = any(lx < split_x - 0.01 for lx in child_local_xs)
                    has_right = any(lx > split_x + 0.01 for lx in child_local_xs)
                    if has_left and has_right:
                        px = sup.x

                parent = Point(px, py, parent_z)

                reached = parent.distance_to_point(sup) <= reach_tol
                if reached:
                    parent = sup

                rec = {
                    "apex": parent,
                    "group": group_id,
                    "support": sup,
                    "support_id": sup_id,
                    "reached": reached,
                    "inset_corners": [],
                    "children": child_pts,
                    "level": level
                }
                new_grid[j][i] = rec
                records.append(rec)

                for ch in child_pts:
                    relations.append((parent, ch, group_id, "parent_child"))

        cell_grid = new_grid

    return records, relations



def connect_top_to_supports(records, sup_pts, debug=False):
    """Connect the highest-level apexes to their nearest support.
    
    Creates an edge from each top-level apex to its nearest support
    and adds support nodes to the graph.
    """
    max_level = max(r["level"] for r in records)
    top_apexes = [r for r in records if r["level"] == max_level]

    new_records = []
    new_relations = []
    used = {}

    for rec in top_apexes:
        apex = rec["apex"]
        group_id, nearest_idx, nearest_sp = nearest_support(apex, sup_pts)

        new_relations.append((nearest_sp, apex, rec["group"], "support_top"))

        if nearest_idx not in used:
            used[nearest_idx] = {"sp": nearest_sp, "group": group_id, "children": []}
        used[nearest_idx]["children"].append(apex)

    for idx, info in used.items():
        new_records.append({
            "apex": info["sp"],
            "group": info["group"],
            "support": info["sp"],
            "support_id": idx,
            "reached": True,
            "inset_corners": [],
            "children": info["children"],
            "level": max_level + 1
        })

    if debug:
        print("connect_top_to_supports: {} top apexes (level {}) -> {} supports".format(
            len(top_apexes), max_level, len(used)))
        for idx, info in used.items():
            print("  Support {}: {} apexes connected".format(idx, len(info["children"])))

    return new_records, new_relations


# ==================================================
# Graph construction
# ==================================================

def build_graph(records, relations):
    """Construct a NodeGraph from cell records and edge relations."""
    ng = NodeGraph()

    for rec in records:
        mobility = "fixed" if rec["reached"] else "z_free"
        apex_node = ng.get_or_add_point_node(
            rec["apex"],
            ntype="apex",
            level=rec["level"],
            group=rec["group"],
            support_id=rec["support_id"],
            reached=rec["reached"],
            children=rec["children"],
            mobility=mobility
        )

        inset_nodes = []
        for ic in rec["inset_corners"]:
            inset_nodes.append(
                ng.get_or_add_point_node(ic, ntype="inset", level=0, mobility="fixed")
            )

        child_nodes = []
        for ch in rec["children"]:
            child_nodes.append(ng.get_or_add_point_node(ch))
        ng.node_attribute(apex_node, "children", child_nodes)

    for p1, p2, group_id, etype in relations:
        u = ng.get_or_add_point_node(p1)
        v = ng.get_or_add_point_node(p2)
        ng.add_graph_edge(u, v, group=group_id, etype=etype)

    return ng


# ==================================================
# Main API
# ==================================================

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
    Build a hierarchical tree graph from a boundary and support points.
    
    The grid automatically splits at the support column axis in X,
    creating wider cells on the cantilevered side and narrower cells
    on the support side. Works for both left and right extensions.
    
    Parameters
    ----------
    boundary : Rhino.Geometry.Curve
        Rectangular boundary curve.
    supports : list of Rhino.Geometry.Point3d
        2 support points (column positions).
    roof_brep : Rhino.Geometry.Brep, optional
        V-shaped roof surface for projecting inset corners.
    div_x : int
        Grid divisions in X (default from config).
    div_y : int
        Grid divisions in Y (default from config).
    num_levels : int
        Number of hierarchical levels (default from config).
    z_steps : list of float, optional
        Z interpolation factors per level, length = num_levels + 1.
        Default: uniform [0, 1/n, 2/n, ..., 1].
    inset_edge : float
        Inset distance for boundary corners.
    inset_interior : float
        Inset distance for interior corners.
    reach_tol : float
        Snap tolerance for support points.
    
    Returns
    -------
    NodeGraph
        The constructed graph with all nodes and edges.
    """
    # Defaults
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
    if not z_steps or len(z_steps) < num_levels + 1:
        z_steps = [i / float(num_levels) for i in range(num_levels + 1)]

    config = {
        "div_x": div_x,
        "div_y": div_y,
        "num_levels": num_levels,
        "inset_edge": inset_edge,
        "inset_interior": inset_interior,
        "reach_tol": reach_tol,
    }

    # Setup geometry
    frame, lx, ly = get_plane_and_size(boundary)
    sup_pts = [point_to_compas(s) for s in supports]

    # Auto-compute grid split from support positions
    split_x = compute_split_x(frame, sup_pts)

    if debug:
        print("DEBUG: boundary lx={:.2f}, ly={:.2f}".format(lx, ly))
        print("DEBUG: auto split_x={:.2f} (left={:.2f}m, right={:.2f}m)".format(
            split_x, split_x, lx - split_x))

    vertex_grid = create_vertex_grid(frame, div_x, div_y, lx, ly, split_x)

    if debug:
        x_coords = [vertex_grid[0][i].x for i in range(div_x + 1)]
        print("DEBUG: Grid X positions: {}".format(
            ["({:.2f})".format(x) for x in x_coords]))
        widths = [x_coords[i + 1] - x_coords[i] for i in range(div_x)]
        print("DEBUG: Column widths: {}".format(
            ["{:.2f}".format(w) for w in widths]))
        print("DEBUG: z_steps={}".format(z_steps))

    # Build all levels
    cell_grid, records_l0, relations_l0 = build_level_zero(
        vertex_grid, sup_pts, z_steps, roof_brep, config
    )
    records_upper, relations_upper = build_higher_levels(
        cell_grid, sup_pts, z_steps, num_levels, config,
        frame=frame, split_x=split_x
    )

    all_records = records_l0 + records_upper
    all_relations = relations_l0 + relations_upper

    if debug:
        print("DEBUG: Total records={}, L0={}, Upper={}".format(
            len(all_records), len(records_l0), len(records_upper)))
        print("DEBUG: {} supports:".format(len(sup_pts)))
        for i, sp in enumerate(sup_pts):
            print("  [{}] ({:.2f}, {:.2f}, {:.2f})".format(i, sp.x, sp.y, sp.z))
        from collections import Counter
        lc = Counter(r["level"] for r in all_records)
        print("DEBUG: Records per level: {}".format(dict(sorted(lc.items()))))

    # Connect top level to supports
    sup_records, sup_relations = connect_top_to_supports(
        all_records, sup_pts, debug=debug
    )
    all_records.extend(sup_records)
    all_relations.extend(sup_relations)

    # Build graph
    ng = build_graph(all_records, all_relations)

    if debug:
        support_nodes = ng.get_support_nodes()
        print("DEBUG: Graph has {} support nodes: {}".format(
            len(support_nodes), support_nodes))

    return ng


# ==================================================
# Convenience accessors
# ==================================================

def get_group_lines(ng, group_id):
    """Get all edge lines for a specific group."""
    return ng.edge_lines_by_group(group_id)


def get_all_points(ng):
    """Get all node points from the graph."""
    return ng.node_points()