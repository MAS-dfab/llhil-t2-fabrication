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
from compas.geometry import Point, Vector, Frame
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
    origin = rg.Point3d(pt.x, pt.y, pt.z)
    ray = rg.Ray3d(origin, rg.Vector3d.ZAxis)
    hits = rg.Intersect.Intersection.RayShoot(ray, [brep], 1)
    if hits:
        return Point(hits[0].X, hits[0].Y, hits[0].Z)
    # Try downward if upward missed
    ray = rg.Ray3d(origin, -rg.Vector3d.ZAxis)
    hits = rg.Intersect.Intersection.RayShoot(ray, [brep], 1)
    if hits:
        return Point(hits[0].X, hits[0].Y, hits[0].Z)
    return pt


def _brep_frame_at_point(pt, brep):
    """Return COMPAS Frame tangent to brep surface at pt; None if brep is None."""
    if brep is None:
        return None
    rg_pt = rg.Point3d(pt.x, pt.y, pt.z)
    min_dist = float('inf')
    result_frame = None
    for face in brep.Faces:
        ok, u, v = face.ClosestPoint(rg_pt)
        if not ok:
            continue
        d = face.PointAt(u, v).DistanceTo(rg_pt)
        if d < min_dist:
            min_dist = d
            ok2, plane = face.FrameAt(u, v)
            if ok2:
                result_frame = Frame(
                    Point(plane.Origin.X, plane.Origin.Y, plane.Origin.Z),
                    Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z),
                    Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z),
                )
    return result_frame


def inset_corner(corner, center, inset_dist):
    """Move a corner point toward the center by inset distance in XY plane."""
    diag = Vector.from_start_end(corner, center)
    diag_xy = Vector(diag.x, diag.y, 0.0)
    diag_xy.unitize()
    return corner.translated(diag_xy * inset_dist)


def is_boundary_vertex(vi, vj, div_x, div_y):
    """Check if vertex indices are on the grid boundary."""
    return vi == 0 or vi == div_x or vj == 0 or vj == div_y


def nearest_support_index(pt, support_points):
    """Return index of support point closest to pt."""
    best_i = 0
    best_d = float("inf")
    for i, sp in enumerate(support_points):
        d = pt.distance_to_point(sp)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def boundary_from_brep_projection(input_brep, tolerance=0.01):
    """Derive a boundary curve from the projected footprint of a single Brep."""
    if input_brep is None:
        raise ValueError("input_brep is required")
    return boundary_from_breps_projection([input_brep], tolerance)


def brep_center_point(brep):
    """Get the centroid of a Brep bounding box as a COMPAS Point."""
    bbox = brep.GetBoundingBox(rg.Plane.WorldXY)
    cx = (bbox.Min.X + bbox.Max.X) / 2.0
    cy = (bbox.Min.Y + bbox.Max.Y) / 2.0
    cz = (bbox.Min.Z + bbox.Max.Z) / 2.0
    return Point(cx, cy, cz)


def boundary_from_breps_projection(input_breps, tolerance=0.01):
    """
    Create a boundary curve by projecting multiple Breps to World XY.

    Uses all provided Brep edges, returns largest closed loop, and falls back
    to a bounding rectangle across all Breps.
    """
    if not input_breps:
        raise ValueError("input_breps is required")

    breps = [b for b in input_breps if b is not None]
    if not breps:
        raise ValueError("input_breps contains no valid Breps")

    world_xy = rg.Plane.WorldXY
    projected = []

    for brep in breps:
        for edge in brep.Edges:
            edge_curve = edge.ToNurbsCurve()
            if edge_curve is None:
                continue
            p = rg.Curve.ProjectToPlane(edge_curve, world_xy)
            if p is not None:
                projected.append(p)

    if projected:
        joined = rg.Curve.JoinCurves(projected, tolerance)
        closed = [c for c in joined if c is not None and c.IsClosed]
        if closed:
            closed.sort(key=lambda c: abs(rg.AreaMassProperties.Compute(c).Area), reverse=True)
            return closed[0]

    # Fallback: combined bounding box rectangle.
    bbox = breps[0].GetBoundingBox(world_xy)
    for brep in breps[1:]:
        bbox.Union(brep.GetBoundingBox(world_xy))

    x0 = bbox.Min.X
    y0 = bbox.Min.Y
    x1 = bbox.Max.X
    y1 = bbox.Max.Y
    pts = [
        rg.Point3d(x0, y0, 0.0),
        rg.Point3d(x1, y0, 0.0),
        rg.Point3d(x1, y1, 0.0),
        rg.Point3d(x0, y1, 0.0),
        rg.Point3d(x0, y0, 0.0),
    ]
    poly = rg.Polyline(pts)
    return poly.ToNurbsCurve()


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
def build_level_zero(vertex_grid, sup_pts, z_steps, roof_brep, config, support_breps=None):
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

            # Assign support: nearest if multi-brep mode, else quadrant-based
            group_id, default_sup_id = determine_cell_group(i, j, div_x, div_y)
            if config.get("use_nearest_support", False):
                sup_id = nearest_support_index(center, sup_pts)
                group_id = sup_id  # group == support index for viz
            else:
                sup_id = default_sup_id
            sup = sup_pts[sup_id]

            # Compute inset corners with projection
            inset_corners = []
            inset_frames = []
            for k, corner in enumerate(corners):
                vi, vj = corner_indices[k]
                on_boundary = is_boundary_vertex(vi, vj, div_x, div_y)
                inset_dist = inset_edge if on_boundary else inset_interior
                inset_pt = inset_corner(corner, center, inset_dist)

                # Project to support-specific Brep if provided
                target_brep = roof_brep
                if support_breps:
                    sp_idx = nearest_support_index(inset_pt, sup_pts)
                    if sp_idx < len(support_breps) and support_breps[sp_idx] is not None:
                        target_brep = support_breps[sp_idx]

                inset_pt = project_to_brep(inset_pt, target_brep)
                inset_corners.append(inset_pt)
                inset_frames.append(_brep_frame_at_point(inset_pt, target_brep))

            # z_steps: accept 2D [[g0_steps], [g1_steps]] or 1D [steps]
            if z_steps and isinstance(z_steps[0], (list, tuple)):
                flat_steps = z_steps[group_id % len(z_steps)]
            else:
                flat_steps = z_steps
            t = flat_steps[0]
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
                "inset_frames": inset_frames,
                "children": [],
                "level": 0
            }

            cell_grid[j][i] = rec
            records.append(rec)

            for ic in inset_corners:
                relations.append((apex, ic, group_id, "apex_inset"))
    
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

                # Skip if any child is missing
                if not all(children):
                    continue
                # Only skip group check in nearest-support mode
                if not config.get("use_nearest_support", False):
                    if len(set(ch["group"] for ch in children)) != 1:
                        continue

                child_pts = [ch["apex"] for ch in children]
                px = sum(p.x for p in child_pts) / 4.0
                py = sum(p.y for p in child_pts) / 4.0
                avg_z = sum(p.z for p in child_pts) / 4.0
                parent_center = Point(px, py, avg_z)

                # In nearest-support mode, parent finds its own nearest support
                if config.get("use_nearest_support", False):
                    sup_id = nearest_support_index(parent_center, sup_pts)
                    sup = sup_pts[sup_id]
                    group_id = sup_id
                else:
                    A = children[0]
                    sup = A["support"]
                    sup_id = A["support_id"]
                    group_id = A["group"]

                # Resolve z_steps
                if z_steps and isinstance(z_steps[0], (list, tuple)):
                    flat_steps = z_steps[group_id % len(z_steps)]
                else:
                    flat_steps = z_steps
                t = flat_steps[level + 1] if flat_steps and len(flat_steps) > level + 1 else 1.0
                parent_z = avg_z + t * (sup.z - avg_z)
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
                    "original_corners": [],
                    "inset_corners": [],
                    "inset_frames": [],
                    "children": child_pts,
                    "level": level
                }

                new_grid[j][i] = rec
                records.append(rec)

                for ch in child_pts:
                    relations.append((parent, ch, group_id, "parent_child"))

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
            
            nearest_idx = nearest_support_index(apex, sup_pts)
            nearest_sp = sup_pts[nearest_idx]
            min_dist = apex.distance_to_point(nearest_sp)
            
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

        frames = rec.get("inset_frames") or [None] * len(rec["inset_corners"])
        for ic, oc, frame in zip(rec["inset_corners"], rec["original_corners"], frames):
            ng.get_or_add_point_node(ic, ntype="inset", level=0, mobility="fixed", original_point=oc, brep_frame=frame)

        # Track child nodes
        child_nodes = []
        for ch in rec["children"]:
            ch_node = ng.get_or_add_point_node(ch)
            child_nodes.append(ch_node)
        ng.node_attribute(apex_node, "children", child_nodes)

    # Add edges from relations
    for p1, p2, group_id, etype in relations:
        u = ng.get_or_add_point_node(p1)
        v = ng.get_or_add_point_node(p2)
        ng.add_graph_edge(u, v, group=group_id, etype=etype)
    
    return ng


# --------------------------------------------------
# Main API
# --------------------------------------------------
def build_tree_graph(
    boundary,
    supports,
    roof_brep=None,
    support_breps=None,
    use_nearest_support=False,
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
    support_breps : list of Rhino.Geometry.Brep, optional
        Optional support-specific Breps (typically 4). If provided, inset
        points project to the Brep of the nearest support.
    use_nearest_support : bool
        If True, each cell grows toward its nearest support point.
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
        "use_nearest_support": use_nearest_support,
    }

    # Setup
    plane, lx, ly = get_plane_and_size(boundary)
    sup_pts = [point_to_compas(s) for s in supports]
    vertex_grid = create_vertex_grid(plane, div_x, div_y, lx, ly)

    # Build levels
    cell_grid, records_l0, relations_l0 = build_level_zero(
        vertex_grid, sup_pts, z_steps, roof_brep, config, support_breps=support_breps
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


def _build_single_brep_tree(brep, sup_pt, sup_idx, div_x, div_y, num_levels, flat_steps, config):
    """
    Build records and relations for one brep growing toward one support.
    No groups, no cross-brep logic.
    """
    inset_edge = config["inset_edge"]
    inset_interior = config["inset_interior"]
    reach_tol = config["reach_tol"]

    boundary = boundary_from_brep_projection(brep)
    plane, lx, ly = get_plane_and_size(boundary)
    vertex_grid = create_vertex_grid(plane, div_x, div_y, lx, ly)

    records = []
    relations = []
    cell_grid = [[None for _ in range(div_x)] for _ in range(div_y)]

    # Level 0 - insets on brep surface, apex toward support
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

            # Project center onto the brep to get its actual Z height
            center_on_brep = project_to_brep(center, brep)

            inset_corners = []
            inset_frames = []
            for k, corner in enumerate(corners):
                vi, vj = corner_indices[k]
                on_boundary = is_boundary_vertex(vi, vj, div_x, div_y)
                inset_dist = inset_edge if on_boundary else inset_interior
                inset_pt = inset_corner(corner, center, inset_dist)
                inset_pt = project_to_brep(inset_pt, brep)
                inset_corners.append(inset_pt)
                inset_frames.append(_brep_frame_at_point(inset_pt, brep))

            t = flat_steps[1] if len(flat_steps) > 1 else 0.5
            apex_z = center_on_brep.z + t * (sup_pt.z - center_on_brep.z)
            apex = Point(center.x, center.y, apex_z)

            reached = apex.distance_to_point(sup_pt) <= reach_tol
            if reached:
                apex = sup_pt

            rec = {
                "apex": apex,
                "group": sup_idx,
                "support": sup_pt,
                "support_id": sup_idx,
                "reached": reached,
                "original_corners": corners,
                "inset_corners": inset_corners,
                "inset_frames": inset_frames,
                "children": [],
                "level": 0
            }
            cell_grid[j][i] = rec
            records.append(rec)

            for ic in inset_corners:
                relations.append((apex, ic, sup_idx, "apex_inset"))

    # Higher levels via shared builder.
    # use_nearest_support=False so group/support are inherited from level-0 cells,
    # which all already point to sup_pt/sup_idx.
    higher_config = dict(config, use_nearest_support=False)
    records_upper, relations_upper = build_higher_levels(
        cell_grid, [sup_pt], flat_steps, num_levels, higher_config
    )
    records.extend(records_upper)
    relations.extend(relations_upper)

    return records, relations


def build_tree_graph_from_breps(
    input_breps,
    supports,
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
    Build one tree per Brep, each growing independently toward its nearest support.

    Each Brep gets its own grid. Inset corners are projected to that Brep surface.
    The tree grows from the Brep surface down to the nearest support in num_levels steps.
    All trees are combined into a single NodeGraph.
    """
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

    config = {
        "inset_edge": inset_edge,
        "inset_interior": inset_interior,
        "reach_tol": reach_tol,
    }

    # Resolve z_steps:
    #   - list of lists  -> per-brep steps, z_steps[i] for brep i
    #   - flat list      -> same steps for every brep
    #   - None           -> auto-generate
    default_steps = [i / float(num_levels) for i in range(num_levels + 1)]
    if z_steps and isinstance(z_steps[0], (list, tuple)):
        per_brep_steps = [list(s) for s in z_steps]
        is_per_brep = True
    else:
        per_brep_steps = list(z_steps) if z_steps else default_steps
        is_per_brep = False

    sup_pts = [point_to_compas(s) for s in supports]

    all_records = []
    all_relations = []

    for brep_idx, brep in enumerate(input_breps):
        if brep is None:
            continue
        ctr = brep_center_point(brep)
        # Use XY-only distance: breps sit at roof height, supports may be at
        # different elevations, so 3D distance picks the wrong support.
        ctr_xy = Point(ctr.x, ctr.y, 0.0)
        sup_pts_xy = [Point(s.x, s.y, 0.0) for s in sup_pts]
        sup_idx = nearest_support_index(ctr_xy, sup_pts_xy)
        sup_pt = sup_pts[sup_idx]

        if is_per_brep:
            flat_steps = per_brep_steps[brep_idx] if brep_idx < len(per_brep_steps) else default_steps
        else:
            flat_steps = per_brep_steps

        if debug:
            print("Brep {} center ({:.2f},{:.2f},{:.2f}) -> Support {}: ({:.2f},{:.2f},{:.2f})  steps={}".format(
                brep_idx, ctr.x, ctr.y, ctr.z, sup_idx, sup_pt.x, sup_pt.y, sup_pt.z, flat_steps))

        recs, rels = _build_single_brep_tree(
            brep, sup_pt, sup_idx, div_x, div_y, num_levels, flat_steps, config
        )
        all_records.extend(recs)
        all_relations.extend(rels)

    finalize_top_level(all_records, sup_pts, num_levels, reach_tol, debug=debug)

    ng = build_graph_from_records(all_records, all_relations)

    if debug:
        support_nodes = ng.get_support_nodes()
        print("DEBUG: Graph built with {} support nodes: {}".format(len(support_nodes), support_nodes))

    return ng
